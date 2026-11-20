"""执行层：Forwarder 协议 + 仿真执行器。

Forwarder 是端侧与平台控制 API（Unitree Sport / 其他四足 SDK）之间的
适配点。本仓只提供 SimForwarder（记录指令流，不产生真实运动指令）；
真机适配层属于部署侧代码，证据等级单独标注。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Protocol

from .skills import ActionStep
from .spatial_state import SpatialState


@dataclass
class StepResult:
    ok: bool
    detail: str = ""
    final_state: SpatialState | None = None


class Forwarder(Protocol):
    """平台执行适配接口。实现方必须保证：只接受 ActionStep 高层原语，
    不接受来自意图层的原始速度/轨迹字段。"""

    def execute(self, step: ActionStep, deadline_ns: int) -> StepResult:
        ...

    def abort(self, reason: str) -> None:
        ...

    def state(self) -> SpatialState:
        ...


@dataclass
class SimForwarder:
    """仿真执行器：按 ActionStep 更新内部位姿并记录指令流。

    仅用于离线闭环验证（证据等级：合成数据·单元测试），不代表真机能力。
    """

    initial_state: SpatialState = field(default_factory=SpatialState)
    fail_kinds: set[str] = field(default_factory=set)  # 注入失败的动作类型
    steps: list[dict[str, Any]] = field(default_factory=list)
    _state: SpatialState = field(default_factory=lambda: SpatialState())

    def __post_init__(self) -> None:
        self._state = SpatialState(
            x=self.initial_state.x, y=self.initial_state.y, z=self.initial_state.z,
            yaw_deg=self.initial_state.yaw_deg, map_id=self.initial_state.map_id,
            revision=self.initial_state.revision,
            localization_confidence=self.initial_state.localization_confidence or 1.0,
            valid=True, timestamp_ns=time.time_ns(),
        )

    def execute(self, step: ActionStep, deadline_ns: int) -> StepResult:
        self.steps.append({"kind": step.kind, "goal": step.goal})
        if step.kind in self.fail_kinds:
            return StepResult(ok=False, detail=f"sim_fail:{step.kind}")
        if step.kind == "navigate_to" and step.goal:
            self._state.x = float(step.goal.get("x", self._state.x))
            self._state.y = float(step.goal.get("y", self._state.y))
            self._state.z = float(step.goal.get("z", self._state.z))
        elif step.kind == "return_home":
            self._state.x = self._state.y = 0.0
        self._state.timestamp_ns = time.time_ns()
        return StepResult(ok=True, final_state=self._state)

    def abort(self, reason: str) -> None:
        self.steps.append({"kind": "abort", "reason": reason})

    def state(self) -> SpatialState:
        return self._state
