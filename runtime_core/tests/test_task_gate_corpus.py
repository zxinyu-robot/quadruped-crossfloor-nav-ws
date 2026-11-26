"""双实现对照：同一份语料分别喂 C++ `TaskGate`（tests/test_all.cpp 内含 task_gate_cases.inc）
与 Python `TaskGate`，断言 (decision, level, code, degraded_mode) 完全一致。"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from legged_nav.spatial_state import SpatialState
from legged_nav.task_gate import GateLimits, TaskGate
from legged_nav.task_intent import validate_task_intent

CASES = Path(__file__).with_name("task_gate_cases.json")


def _load():
    with open(CASES) as f:
        return json.load(f)


def _limits(doc: dict, override: dict | None) -> GateLimits:
    src = dict(doc["limits"])
    if override:
        src.update(override)
    return GateLimits(
        min_confidence=src["min_confidence"],
        ttl_ns=int(src["ttl_ns"]),
        max_goal_dist_m=src["max_goal_dist_m"],
        bounds=(src["min_x"], src["min_y"], src["max_x"], src["max_y"]),
        local_platform=src.get("local_platform", ""),
        local_robot_id=src.get("local_robot_id", ""),
        priority_range=(src.get("priority_min", -100), src.get("priority_max", 100)),
    )


def _state(doc: dict, override: dict | None, now_ns: int) -> SpatialState:
    src = dict(doc["state"])
    if override:
        src.update(override)
    return SpatialState(
        x=src["x"],
        y=src["y"],
        z=src["z"],
        yaw_deg=src["yaw_deg"],
        map_id=src["map_id"],
        revision=src["revision"],
        localization_confidence=src["localization_confidence"],
        valid=src["valid"],
        timestamp_ns=now_ns - int(src.get("age_ns", 10_000_000)),
    )


def _intent(raw: dict, limits: GateLimits):
    intent, err = validate_task_intent(
        raw, local_platform=limits.local_platform, local_robot_id=limits.local_robot_id
    )
    assert intent is not None, f"corpus intent invalid: {err}"
    return intent


CASE_LIST = _load()["cases"]


@pytest.mark.parametrize("case", CASE_LIST, ids=[c["name"] for c in CASE_LIST])
def test_corpus_parity(case):
    doc = _load()
    now_ns = int(doc["now_ns"])
    limits = _limits(doc, case.get("limits"))
    state = _state(doc, case.get("state"), now_ns)
    intent = _intent(case["intent"], limits)
    active = _intent(case["active_intent"], limits) if case.get("active_intent") else None

    gate = TaskGate(limits=limits)
    for task_id in case.get("seed_records", []):
        gate.admission.record(task_id)

    res = gate.evaluate(
        intent,
        state=state,
        now_ns=now_ns,
        candidate_count=int(case.get("candidate_count", 0)),
        active=active,
        forbidden_areas=case.get("forbidden_areas") or None,
    )
    expect = case["expect"]
    assert res.decision == expect["decision"]
    assert res.level == expect["level"]
    assert res.code == expect["code"]
    assert res.degraded_mode == expect.get("degraded_mode", "")


def test_corpus_has_python_and_cpp_runners():
    """语料必须两端都跑：本文件（Python）与 tests/task_gate_cases.inc（C++）。"""
    inc = Path(__file__).with_name("task_gate_cases.inc")
    assert inc.exists()
    assert "run_task_gate_corpus" in inc.read_text()
    assert len(CASE_LIST) >= 10
