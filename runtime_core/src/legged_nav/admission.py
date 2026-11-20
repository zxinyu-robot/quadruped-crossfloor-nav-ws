"""任务准入：task_id 幂等 + 抢占决策（无 ROS）。"""
from __future__ import annotations

from .task_intent import TaskIntent


class AdmitDecision:
    ACCEPT = "accept"
    IDEMPOTENT = "idempotent"
    PREEMPT = "preempt"


class TaskAdmission:
    """task_id 幂等 + 抢占准入。

    - 与活动任务同 task_id → IDEMPOTENT（不重派发）；
    - 已受理过的 task_id → REJECT_DUPLICATE_TASK_ID（有界历史）；
    - 有活动任务：仅当活动任务 preemptible 且新任务 priority 更高 → PREEMPT，否则 REJECT_BUSY。
    """

    def __init__(self, history: int = 256) -> None:
        self._history = history
        self._seen: dict[str, None] = {}

    def decide(self, intent: TaskIntent, active: TaskIntent | None) -> str:
        if active is not None and intent.task_id == active.task_id:
            return AdmitDecision.IDEMPOTENT
        if intent.task_id in self._seen:
            return "REJECT_DUPLICATE_TASK_ID"
        if active is not None:
            if active.preemptible and intent.priority > active.priority:
                return AdmitDecision.PREEMPT
            return "REJECT_BUSY"
        return AdmitDecision.ACCEPT

    def record(self, task_id: str) -> None:
        self._seen[task_id] = None
        while len(self._seen) > self._history:
            self._seen.pop(next(iter(self._seen)))
