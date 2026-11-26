#!/usr/bin/env python3
"""RuntimeBridge：符号平面与度量平面的唯一接缝（主链）。

设计约束（与 `docs/架构/架构_度量符号统一运行时.md` 对应）：
  - 上游只能提交 TaskIntent（JSON），**不能**下发速度/关节（C6）；本节点是唯一入口。
  - 五级校验（schema/边界/状态有效性/可行性/安全）由 runtime_core.TaskGate 裁决。
  - 状态带置信度 + 有效期（C1a）：odom 新鲜度 + PGO/漂移给出 localization_confidence；
    过期或低置信度 → DEGRADED(stop_and_hold)，BT 收到 task_rejected 而不是带着错状态跑（C13）。
  - 异步：本节点不阻塞控制环，只在事件驱动下裁决（C7）；消费最新帧（C11）。
  - 影子模式：`gate_mode=shadow` 只观测直接到达 BT 的 task_accepted，不改变行为，便于灰度。
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt64

# 允许在不安装 runtime_core 的镜像里显式指定源码路径（参数 runtime_core_src）
def _bootstrap_runtime_core(src: str) -> None:
    if src and os.path.isdir(src) and src not in sys.path:
        sys.path.insert(0, src)
    try:
        import legged_nav  # noqa: F401
    except Exception as exc:  # pragma: no cover
        raise SystemExit(
            "runtime_core 未安装：pip install -e runtime_core 或用 -p runtime_core_src:=<path>"
        ) from exc


SKILL_TO_STAGE = {
    "ExploreFrontier": "SEARCH_SEMANTIC_TARGET",
    "Navigate": "NAV_TO_DESTINATION",
    "ReturnHome": "RETURN_HOME",
    "Stand": "STOP",
    "Stop": "STOP",
}


class RuntimeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("runtime_bridge")
        self.declare_parameter("gate_mode", "shadow")  # shadow | enforce
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("ttl_s", 0.5)
        self.declare_parameter("max_goal_dist_m", 12.0)
        self.declare_parameter("bounds", [-1.0, -1.0, 11.0, 11.0])
        self.declare_parameter("local_platform", "")  # 生产环境建议钉死，如 unitree_go2
        self.declare_parameter("local_robot_id", "")
        self.declare_parameter("confidence_tau_s", 0.6)
        self.declare_parameter("drift_penalty_m", 0.25)
        self.declare_parameter("runtime_core_src", "")
        self.declare_parameter("map_id", "")
        self.declare_parameter("state_revision", 0)  # 兜底；优先用 context_generation
        self.declare_parameter("context_generation_topic", "/lio/backend/context_generation")
        self.declare_parameter("intent_topic", "/runtime/task_request")
        self.declare_parameter("gate_topic", "/runtime/gate")
        self.declare_parameter("mission_event_topic", "/demo/mission/event")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("pgo_state_topic", "/lio/backend/pgo_state")
        self.declare_parameter("drift_topic", "/lio/backend/drift_estimate")
        self.declare_parameter("frontier_topic", "/demo/mission/frontier_snapshot")
        self.declare_parameter("semantic_valid_topic", "/semantic/valid")
        self.declare_parameter("goal_topic", "/demo/mission/semantic_goal")
        self.declare_parameter("body_height", 0.35)

        _bootstrap_runtime_core(str(self.get_parameter("runtime_core_src").value))
        from legged_nav.spatial_state import SpatialState
        from legged_nav.task_gate import Decision, GateLimits, TaskGate, degrade_policy
        from legged_nav.task_intent import validate_task_intent

        self._SpatialState = SpatialState
        self._Decision = Decision
        self._degrade_policy = degrade_policy
        self._validate = validate_task_intent
        bounds = list(self.get_parameter("bounds").value)
        self._gate = TaskGate(
            limits=GateLimits(
                min_confidence=float(self.get_parameter("min_confidence").value),
                ttl_ns=int(float(self.get_parameter("ttl_s").value) * 1e9),
                max_goal_dist_m=float(self.get_parameter("max_goal_dist_m").value),
                bounds=(bounds[0], bounds[1], bounds[2], bounds[3]),
                local_platform=str(self.get_parameter("local_platform").value),
                local_robot_id=str(self.get_parameter("local_robot_id").value),
            )
        )
        self._mode = str(self.get_parameter("gate_mode").value).lower()
        self._tau_s = max(0.05, float(self.get_parameter("confidence_tau_s").value))
        self._drift_penalty = float(self.get_parameter("drift_penalty_m").value)

        self._odom: Optional[Odometry] = None
        self._odom_at = 0.0
        self._pgo_state = "UNKNOWN"
        self._drift_m = 0.0
        self._candidates = 0
        self._candidates_at = 0.0
        self._context_generation = int(self.get_parameter("state_revision").value)
        self._active = None

        self.ev_pub = self.create_publisher(String, str(self.get_parameter("mission_event_topic").value), 20)
        self.gate_pub = self.create_publisher(String, str(self.get_parameter("gate_topic").value), 20)
        self.valid_pub = self.create_publisher(Bool, str(self.get_parameter("semantic_valid_topic").value), 10)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)

        self.create_subscription(String, str(self.get_parameter("intent_topic").value), self._on_request, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 20)
        self.create_subscription(String, str(self.get_parameter("pgo_state_topic").value), self._on_pgo, 10)
        self.create_subscription(String, str(self.get_parameter("drift_topic").value), self._on_drift, 10)
        self.create_subscription(String, str(self.get_parameter("frontier_topic").value), self._on_frontiers, 5)
        self.create_subscription(
            UInt64,
            str(self.get_parameter("context_generation_topic").value),
            self._on_context_generation,
            10,
        )
        self.create_subscription(
            String, str(self.get_parameter("mission_event_topic").value), self._on_mission_event, 20
        )
        self.get_logger().info(
            f"RuntimeBridge mode={self._mode} intent_topic={self.get_parameter('intent_topic').value} "
            f"min_conf={self._gate.limits.min_confidence} ttl={self._gate.limits.ttl_ns/1e9:.2f}s"
        )

    # ---------------- 输入 ----------------
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_at = time.time()

    def _on_pgo(self, msg: String) -> None:
        self._pgo_state = msg.data.strip().upper() or "UNKNOWN"

    def _on_drift(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
            self._drift_m = float(body.get("trans_m", body.get("drift_m", 0.0)))
        except (json.JSONDecodeError, TypeError, ValueError):
            self._drift_m = 0.0

    def _on_context_generation(self, msg: UInt64) -> None:
        """地图/上下文代数：意图 revision 必须与本地观测代数同一时间切面（C1a）。"""
        self._context_generation = int(msg.data)

    def _on_frontiers(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
            self._candidates = len(body.get("frontiers") or [])
            self._candidates_at = time.time()
        except json.JSONDecodeError:
            return

    def _on_mission_event(self, msg: String) -> None:
        """影子模式：观测直接进入 BT 的 task_accepted；同时清理活跃任务。"""
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(body.get("event", ""))
        if event in ("mission_succeeded", "mission_failed", "mission_canceled"):
            self._active = None
            return
        if event == "task_accepted" and self._mode == "shadow":
            intent, err = self._validate(
                {
                    "schema": "motionslam.task_intent.v1",
                    "task_id": body.get("command_id", ""),
                    "robot": {
                        "platform": str(self.get_parameter("local_platform").value),
                        "sn": str(self.get_parameter("local_robot_id").value) or "shadow",
                    },
                    "world_ref": {"map_id": "shadow", "revision": 0},
                    "skills": [{"name": "ExploreFrontier"}],
                },
                local_platform=str(self.get_parameter("local_platform").value),
                local_robot_id=str(self.get_parameter("local_robot_id").value),
            )
            if intent is None:
                self._publish_gate("SHADOW_REJECT", 1, err or "REJECT_INTENT", "", body.get("command_id", ""))
                return
            res = self._gate.evaluate(
                intent, state=self._spatial_state(), now_ns=time.time_ns(),
                candidate_count=self._fresh_candidates(), active=self._active,
            )
            self._publish_gate(res.decision, res.level, res.code, res.degraded_mode, intent.task_id)

    # ---------------- 裁决 ----------------
    def _spatial_state(self):
        map_id = str(self.get_parameter("map_id").value)
        revision = int(self._context_generation)
        if self._odom is None:
            return None
        age = time.time() - self._odom_at
        conf = math.exp(-age / self._tau_s)
        if self._pgo_state in ("DEGRADED", "LOST", "INVALID"):
            conf *= 0.5
        if self._drift_m > self._drift_penalty:
            conf *= 0.6
        p = self._odom.pose.pose.position
        return self._SpatialState(
            x=float(p.x), y=float(p.y), z=float(p.z),
            map_id=map_id, revision=revision,
            localization_confidence=max(0.0, min(1.0, conf)),
            valid=age < 1.0,
            timestamp_ns=time.time_ns() - int(age * 1e9),
        )

    def _fresh_candidates(self) -> int:
        if time.time() - self._candidates_at > 3.0:
            return 0
        return self._candidates

    def _on_request(self, msg: String) -> None:
        try:
            raw: Any = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_gate("REJECT", 1, "REJECT_INTENT_TYPE", "", "")
            return
        intent, err = self._validate(
            raw,
            local_platform=str(self.get_parameter("local_platform").value),
            local_robot_id=str(self.get_parameter("local_robot_id").value),
        )
        if intent is None:
            self._publish_gate("REJECT", 1, err or "REJECT_INTENT", "", str(raw.get("task_id", "")))
            self._emit_rejected(str(raw.get("task_id", "")), err or "REJECT_INTENT")
            return

        res = self._gate.evaluate(
            intent,
            state=self._spatial_state(),
            now_ns=time.time_ns(),
            candidate_count=self._fresh_candidates(),
            active=self._active,
        )
        self._publish_gate(res.decision, res.level, res.code, res.degraded_mode, intent.task_id)
        policy = self._degrade_policy(res)
        if res.decision == self._Decision.ACCEPT:
            self._active = intent
            self._emit_accepted(intent)
            self._publish_goal_if_any(intent)
            self._publish_valid(True)
        else:
            self._emit_rejected(intent.task_id, res.code)
            self._publish_valid(False)
            if policy == "hold_in_place":
                self._emit("runtime_degraded", task_id=intent.task_id,
                           mode=res.degraded_mode, code=res.code)

    # ---------------- 输出 ----------------
    def _publish_gate(self, decision: str, level: int, code: str, mode: str, task_id: str) -> None:
        body = {
            "ts": time.time(),
            "decision": decision,
            "level": int(level),
            "code": code,
            "degraded_mode": mode,
            "task_id": task_id,
            "candidates": self._candidates,
            "pgo_state": self._pgo_state,
            "drift_m": self._drift_m,
            "odom_age_ms": round((time.time() - self._odom_at) * 1000.0, 1) if self._odom_at else -1.0,
        }
        self.gate_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))

    def _publish_valid(self, ok: bool) -> None:
        self.valid_pub.publish(Bool(data=bool(ok)))

    def _emit_accepted(self, intent) -> None:
        stage_ids = [SKILL_TO_STAGE[s.name] for s in intent.skills if s.name in SKILL_TO_STAGE]
        goal = next((s.goal for s in intent.skills if s.goal is not None), None)
        body = {
            "event": "task_accepted",
            "command_id": intent.task_id,
            "language_query": "",
            "stage_ids": stage_ids,
            "gate": "runtime",
        }
        if goal is not None:
            body.update({"x": goal.x, "y": goal.y, "z": goal.z, "frame_id": goal.frame_id})
        self._emit(**body)

    def _publish_goal_if_any(self, intent) -> None:
        """Navigate 的几何目标走既有下游通道；闸门只放行，不替代度量平面。"""
        goal = next((s.goal for s in intent.skills if s.goal is not None), None)
        if goal is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = goal.frame_id or "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal.x)
        msg.pose.position.y = float(goal.y)
        msg.pose.position.z = float(self.get_parameter("body_height").value)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _emit_rejected(self, task_id: str, reason: str) -> None:
        self._emit("task_rejected", command_id=task_id, reason=reason)

    def _emit(self, event: str, **extra: Any) -> None:
        body = {"event": event, "mission_id": "", "status": "runtime", "phase": "runtime", **extra}
        self.ev_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))


def main() -> None:
    rclpy.init()
    node = RuntimeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
