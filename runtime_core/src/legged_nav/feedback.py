"""执行反馈（上行）payload 构造。"""
from __future__ import annotations

import time
from typing import Any

from .contract_gen import FEEDBACK_SCHEMA

SCHEMA = FEEDBACK_SCHEMA


def build_feedback(
    *,
    task_id: str,
    state: str,
    event: str,
    progress: dict[str, Any] | None = None,
    detail: str = "",
    timestamp_ns: int = 0,
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "task_id": task_id,
        "state": state,
        "event": event,
        "progress": progress or {},
        "detail": detail,
        "timestamp_ns": timestamp_ns or time.time_ns(),
    }
