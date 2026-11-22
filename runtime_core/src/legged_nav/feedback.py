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
    """构建 schema=motionslam.execution_feedback.v1 的反馈对象。

    progress 展平为 ``progress.<key>``，与 C++ 权威实现
    (src/feedback.cpp) 的 string→string 平铺结构同构，
    保证同一 schema 下 C++/Python 双实现 payload 结构一致。
    """
    fb: dict[str, Any] = {
        "schema": SCHEMA,
        "task_id": task_id,
        "state": state,
        "event": event,
        "detail": detail,
        "timestamp_ns": timestamp_ns or time.time_ns(),
    }
    for key, value in (progress or {}).items():
        fb[f"progress.{key}"] = value
    return fb
