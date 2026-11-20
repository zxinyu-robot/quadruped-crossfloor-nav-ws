"""通信中间件抽象层（无 ROS）：Cyclone DDS + SHM + DDSI-RTPS/UDP 组合传输。

端侧各功能组件（多源传感器 → 空间状态 → 模型推理 → 决策 → 运动控制）
通过命名 Topic 解耦交互；传输后端可插拔：

- InProcessTransport：进程内 pub/sub，离线闭环与 CI 测试用；
- CycloneDDSTransport：Cyclone DDS 真机后端（需 cyclonedds Python 绑定；
  SHM 用于同机进程间零拷贝，DDSI-RTPS/UDP 用于跨进程/跨机传输）。

QoS 按通道分级：高频传感状态用 best_effort，意图/反馈用 reliable。
"""
from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol


class QoS(str, Enum):
    BEST_EFFORT = "best_effort"   # 高频状态类（RGB、位姿），丢帧可接受
    RELIABLE = "reliable"         # 意图/反馈/控制指令，必达


@dataclass(frozen=True)
class Channel:
    """数据链路通道定义：多源传感器 / 空间状态 / 模型推理 / 决策 / 运动控制。"""

    name: str
    qos: QoS
    schema: str
    desc: str


CHANNELS: dict[str, Channel] = {
    # 传感器上行
    "rgb/frame": Channel("rgb/frame", QoS.BEST_EFFORT, "sensor.rgb_frame.v1",
                         "RGB 视觉流（相机/数据集上行给感知）"),
    "sensor/lidar_scan": Channel("sensor/lidar_scan", QoS.BEST_EFFORT, "sensor.scan.v1",
                                 "LiDAR 点云/扫描"),
    "sensor/imu": Channel("sensor/imu", QoS.BEST_EFFORT, "sensor.imu.v1", "IMU 数据"),
    # 空间状态
    "spatial/state": Channel("spatial/state", QoS.BEST_EFFORT, "spatial.state.v1",
                             "位姿·置信度·地图版本（StateGate 输入）"),
    # 模型推理上行
    "perception/semantic_obs": Channel(
        "perception/semantic_obs", QoS.RELIABLE, "perception.semantic_observation.v1",
        "VLM 语义观测（scene/landmarks/hazards/target_visible）"),
    # 决策下行
    "task/intent": Channel("task/intent", QoS.RELIABLE, "motionslam.task_intent.v1",
                           "TaskIntent 技能级决策"),
    # 执行与反馈
    "motion/action_step": Channel("motion/action_step", QoS.RELIABLE,
                                  "motion.action_step.v1", "高层运动原语（不含 cmd_vel）"),
    "exec/feedback": Channel("exec/feedback", QoS.RELIABLE,
                             "motionslam.execution_feedback.v1", "执行反馈回流"),
}

# 禁控字段不得成为任何通道 payload 字段
FORBIDDEN_PAYLOAD_KEYS = frozenset(
    {"cmd_vel", "/cmd_vel", "twist", "Twist", "initial_path", "/motion/command"}
)


def _scan_forbidden(obj: Any) -> str | None:
    if isinstance(obj, dict):
        for k, v in obj.items():
            if str(k) in FORBIDDEN_PAYLOAD_KEYS:
                return f"REJECT_FORBIDDEN_CONTROL:{k}"
            found = _scan_forbidden(v)
            if found:
                return found
    elif isinstance(obj, list):
        for item in obj:
            found = _scan_forbidden(item)
            if found:
                return found
    return None


@dataclass
class Message:
    channel: str
    payload: Any
    timestamp_ns: int = field(default_factory=time.time_ns)


Subscriber = Callable[[Message], None]


class Transport(Protocol):
    """传输后端协议：publish/subscribe。"""

    def publish(self, channel: str, payload: Any) -> str | None:
        """发布；返回 None 或拒绝码（如 REJECT_FORBIDDEN_CONTROL）。"""
        ...

    def subscribe(self, channel: str, cb: Subscriber) -> None:
        ...

    def close(self) -> None:
        ...


class InProcessTransport:
    """进程内后端：同步分发，离线闭环与单元测试用（等级：合成数据·单元测试）。"""

    def __init__(self) -> None:
        self._subs: dict[str, list[Subscriber]] = {}
        self.log: list[Message] = []   # 留痕，可审计

    def publish(self, channel: str, payload: Any) -> str | None:
        if channel not in CHANNELS:
            return f"REJECT_UNKNOWN_CHANNEL:{channel}"
        bad = _scan_forbidden(payload)
        if bad:
            return bad
        msg = Message(channel=channel, payload=payload)
        self.log.append(msg)
        for cb in self._subs.get(channel, []):
            cb(msg)
        return None

    def subscribe(self, channel: str, cb: Subscriber) -> None:
        self._subs.setdefault(channel, []).append(cb)

    def close(self) -> None:
        pass


class CycloneDDSTransport:
    """Cyclone DDS 后端（真机部署侧；懒加载 cyclonedds 绑定）。

    - 同机进程间：Cyclone DDS + 共享内存（SHM）零拷贝；
    - 跨进程/跨机：DDSI-RTPS over UDP；
    - 每个 Channel 映射为一个 DDS Topic，QoS 映射 best_effort/reliable。

    CI 不加载：import 失败时构造抛错并指向 InProcessTransport。
    """

    def __init__(self, domain_id: int = 0) -> None:
        try:
            from cyclonedds.domain import DomainParticipant  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "cyclonedds Python 绑定未安装（真机/部署侧依赖）；"
                "离线链路请用 InProcessTransport"
            ) from e
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter  # noqa: F401
        from cyclonedds.sub import DataReader  # noqa: F401

        self.domain_id = domain_id
        self._participant = DomainParticipant(domain_id)
        self._writers: dict[str, Any] = {}
        self._readers: list[Any] = []

    def publish(self, channel: str, payload: Any) -> str | None:
        if channel not in CHANNELS:
            return f"REJECT_UNKNOWN_CHANNEL:{channel}"
        bad = _scan_forbidden(payload)
        if bad:
            return bad
        writer = self._writers.get(channel)
        if writer is None:
            return f"REJECT_NOT_REGISTERED:{channel}"
        writer.write(payload)
        return None

    def subscribe(self, channel: str, cb: Subscriber) -> None:
        # Topic/Reader 注册由部署侧按 IdlType 完成；此处仅签名占位。
        raise NotImplementedError("Cyclone DDS 订阅注册在部署侧完成（IdlType per channel）")

    def close(self) -> None:
        self._participant.close()
