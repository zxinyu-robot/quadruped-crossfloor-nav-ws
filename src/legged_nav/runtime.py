"""端侧任务运行时：TaskIntent → 准入 → 状态门控 → 技能编排 → 执行 → 反馈。

闭环链路（对应简历的端侧自主闭环）：

    TaskIntent(JSON) ──validate──▶ 契约准入（schema/平台/禁区/禁控字段）
                        │
                        ▼
                   Admission（幂等/抢占/忙拒）
                        │
                        ▼
                   StateGate（定位有效性/有效期/地图版本一致）
                        │  失效 → HOLD + SAFETY_INTERLOCK
                        ▼
                   逐技能执行（Skill → ActionStep → Forwarder）
                        │
                        ▼
                   ExecutionFeedback（终态 + 进度 + 位姿回流）

模型/上层决策只允许到达技能层；速度、轨迹、电机指令不属于意图字段，
在契约校验阶段直接拒绝（REJECT_FORBIDDEN_CONTROL）。
"""
from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .admission import AdmitDecision, TaskAdmission
from .executor import Forwarder
from .feedback import build_feedback
from .skills import SkillError, compile_skill
from .spatial_state import SpatialState, StateGate
from .task_intent import TaskIntent, check_forbidden_areas, validate_task_intent


class TaskStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    HOLDING = "holding"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    REJECTED = "rejected"


TERMINAL = {
    TaskStatus.SUCCEEDED,
    TaskStatus.FAILED,
    TaskStatus.CANCELED,
    TaskStatus.REJECTED,
}


@dataclass
class TaskHandle:
    intent: TaskIntent
    status: TaskStatus
    skill_index: int = 0
    failure_reason: str = ""
    accepted_ns: int = 0
    finished_ns: int = 0


class TaskRuntime:
    """端侧任务编排。所有决策留痕为 ExecutionFeedback 事件。"""

    def __init__(
        self,
        forwarder: Forwarder,
        *,
        local_platform: str = "",
        local_robot_id: str = "",
        forbidden_areas: dict | None = None,
        state_gate: StateGate | None = None,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self.forwarder = forwarder
        self.local_platform = local_platform
        self.local_robot_id = local_robot_id
        self.forbidden_areas = forbidden_areas or {}
        self.state_gate = state_gate or StateGate()
        self.clock_ns = clock_ns
        self.admission = TaskAdmission()
        self.active: TaskHandle | None = None
        self.events: list[dict[str, Any]] = []

    def _emit(self, handle: TaskHandle, event: str, **kw: Any) -> None:
        self.events.append(
            build_feedback(
                task_id=handle.intent.task_id,
                state=handle.status.value,
                event=event,
                timestamp_ns=self.clock_ns(),
                **kw,
            )
        )

    def submit(self, raw: Any, state: SpatialState | None = None) -> dict[str, Any]:
        """接收 TaskIntent（dict/JSON）；返回 ExecutionFeedback 终态或受理事件。"""
        now = self.clock_ns()
        intent, err = validate_task_intent(
            raw,
            local_platform=self.local_platform,
            local_robot_id=self.local_robot_id,
        )
        if err:
            fb = build_feedback(
                task_id=_safe_task_id(raw), state=TaskStatus.REJECTED.value,
                event="rejected", detail=err, timestamp_ns=now,
            )
            self.events.append(fb)
            return fb

        assert intent is not None
        handle = TaskHandle(intent=intent, status=TaskStatus.ACCEPTED, accepted_ns=now)

        area_err = check_forbidden_areas(intent, self.forbidden_areas)
        if area_err:
            handle.status = TaskStatus.REJECTED
            self._emit(handle, "rejected", detail=area_err)
            return self.events[-1]

        decision = self.admission.decide(intent, self.active.intent if self.active else None)
        if decision == AdmitDecision.IDEMPOTENT:
            self._emit(handle, "idempotent")
            return self.events[-1]
        if decision == AdmitDecision.PREEMPT:
            assert self.active is not None
            self.active.status = TaskStatus.CANCELED
            self.forwarder.abort(f"preempted_by:{intent.task_id}")
            self._emit(self.active, "preempted")
            self.active = None
        elif decision != AdmitDecision.ACCEPT:
            handle.status = TaskStatus.REJECTED
            self._emit(handle, "rejected", detail=decision)
            return self.events[-1]

        self.admission.record(intent.task_id)
        self._emit(handle, "accepted")

        if state is not None:
            lock = self.state_gate.check(state, intent, now)
            if lock:
                handle.status = TaskStatus.HOLDING
                self._emit(handle, "holding", detail=lock)
                return self.events[-1]

        self.active = handle
        handle.status = TaskStatus.RUNNING
        self._emit(handle, "started")

        deadline_ns = (
            now + int(intent.deadline_s * 1e9) if intent.deadline_s > 0 else 2**62
        )

        for i, skill in enumerate(intent.skills):
            handle.skill_index = i
            try:
                steps = compile_skill(skill)
            except SkillError as e:
                self._fail(handle, f"skill_compile:{e}")
                return self.events[-1]
            for step in steps:
                if self.clock_ns() > deadline_ns:
                    self._fail(handle, "deadline_exceeded")
                    self.forwarder.abort("deadline")
                    return self.events[-1]
                result = self.forwarder.execute(step, deadline_ns)
                if not result.ok:
                    self._fail(handle, f"step_failed:{step.kind}:{result.detail}")
                    self.forwarder.abort(result.detail)
                    return self.events[-1]
                self._emit(
                    handle,
                    "progress",
                    progress={"skill_index": i, "skill": skill.name, "step": step.kind},
                )

        handle.status = TaskStatus.SUCCEEDED
        handle.finished_ns = self.clock_ns()
        self.active = None
        self._emit(handle, "succeeded")
        return self.events[-1]

    def _fail(self, handle: TaskHandle, reason: str) -> None:
        handle.status = TaskStatus.FAILED
        handle.failure_reason = reason
        handle.finished_ns = self.clock_ns()
        self.active = None
        self._emit(handle, "failed", detail=reason)

    def cancel(self, task_id: str, reason: str = "user_cancel") -> dict[str, Any]:
        """取消活动任务；幂等，取消后进入 canceled 终态。"""
        if self.active is None or self.active.intent.task_id != task_id:
            return build_feedback(
                task_id=task_id, state="idle", event="cancel_noop",
                detail="no_active_task", timestamp_ns=self.clock_ns(),
            )
        self.active.status = TaskStatus.CANCELED
        self.active.finished_ns = self.clock_ns()
        self.forwarder.abort(reason)
        self._emit(self.active, "canceled", detail=reason)
        self.active = None
        return self.events[-1]


def _safe_task_id(raw: Any) -> str:
    if isinstance(raw, dict):
        return str(raw.get("task_id") or "")
    return ""
