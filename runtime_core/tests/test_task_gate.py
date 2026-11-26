"""TaskGate 五级准入：顺序、降级模式与仲裁。"""
from __future__ import annotations

from legged_nav.spatial_state import SpatialState
from legged_nav.task_gate import (
    Decision,
    DegradeMode,
    GateLimits,
    Level,
    TaskGate,
    degrade_policy,
)
from legged_nav.task_intent import IntentGoal, IntentSkill, TaskIntent

NOW = 1_000_000_000_000


def make_intent(skill: str = "Navigate", x: float = 1.0, y: float = 1.0, **kw) -> TaskIntent:
    intent = TaskIntent(
        task_id=kw.pop("task_id", "t-1"),
        platform=kw.pop("platform", "go2"),
        robot_id=kw.pop("robot_id", "go2-01"),
        map_id=kw.pop("map_id", "m1"),
        revision=kw.pop("revision", 3),
        skills=[IntentSkill(name=skill, goal=IntentGoal(x=x, y=y, z=0.0))],
        **kw,
    )
    return intent


def valid_state() -> SpatialState:
    return SpatialState(
        x=0.0, y=0.0, map_id="m1", revision=3,
        localization_confidence=0.9, valid=True, timestamp_ns=NOW - 10,
    )


def test_accept_happy_path():
    gate = TaskGate()
    res = gate.evaluate(make_intent(), state=valid_state(), now_ns=NOW)
    assert res.decision == Decision.ACCEPT
    assert res.level == Level.ARBITRATION
    assert degrade_policy(res) == "run"


def test_l1_unknown_skill_rejects_before_l2():
    gate = TaskGate()
    intent = make_intent(skill="open_door", x=999.0)
    res = gate.evaluate(intent, state=valid_state(), now_ns=NOW)
    assert res.decision == Decision.REJECT
    assert res.level == Level.SCHEMA
    assert res.code == "REJECT_UNKNOWN_SKILL"


def test_l2_goal_out_of_bounds():
    gate = TaskGate(limits=GateLimits(bounds=(0.0, 0.0, 10.0, 10.0)))
    res = gate.evaluate(make_intent(x=42.0), state=valid_state(), now_ns=NOW)
    assert res.decision == Decision.REJECT
    assert res.level == Level.BOUNDS
    assert res.code == "REJECT_GOAL_OUT_OF_BOUNDS"


def test_l3_expired_state_degrades_to_hold_not_reject():
    gate = TaskGate(limits=GateLimits(ttl_ns=100))
    state = valid_state()
    state.timestamp_ns = NOW - 10_000
    res = gate.evaluate(make_intent(), state=state, now_ns=NOW)
    assert res.decision == Decision.DEGRADED
    assert res.level == Level.VALIDITY
    assert res.code == "SAFETY_INTERLOCK:state_expired"
    assert res.degraded_mode == DegradeMode.STOP_AND_HOLD
    assert degrade_policy(res) == "hold_in_place"


def test_l3_low_confidence_degrades():
    gate = TaskGate(limits=GateLimits(min_confidence=0.8))
    state = valid_state()
    state.localization_confidence = 0.4
    res = gate.evaluate(make_intent(), state=state, now_ns=NOW)
    assert res.decision == Decision.DEGRADED
    assert res.code == "SAFETY_INTERLOCK:low_confidence"


def test_l3_wins_over_l4_and_l5():
    """状态无效时不允许用"无候选"或禁区结论短路降级判定。"""
    gate = TaskGate(limits=GateLimits(ttl_ns=1))
    state = valid_state()
    state.timestamp_ns = NOW - 500
    res = gate.evaluate(
        make_intent(skill="ExploreFrontier"),
        state=state,
        now_ns=NOW,
        candidate_count=0,
        forbidden_areas={"keepout": [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]},
    )
    assert res.level == Level.VALIDITY


def test_l4_search_without_candidates_degrades_to_no_candidate():
    gate = TaskGate()
    res = gate.evaluate(
        make_intent(skill="ExploreFrontier"), state=valid_state(), now_ns=NOW, candidate_count=0
    )
    assert res.decision == Decision.DEGRADED
    assert res.level == Level.FEASIBILITY
    assert res.degraded_mode == DegradeMode.NO_CANDIDATE


def test_l4_navigate_does_not_require_candidates():
    gate = TaskGate()
    res = gate.evaluate(
        make_intent(skill="Navigate"), state=valid_state(), now_ns=NOW, candidate_count=0
    )
    assert res.decision == Decision.ACCEPT


def test_l5_forbidden_area_fails_closed():
    gate = TaskGate()
    res = gate.evaluate(
        make_intent(x=1.0, y=1.0),
        state=valid_state(),
        now_ns=NOW,
        forbidden_areas={"keepout": [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]},
    )
    assert res.decision == Decision.REJECT
    assert res.level == Level.SAFETY


def test_arbitration_duplicate_and_idempotent():
    gate = TaskGate()
    first = gate.evaluate(make_intent(), state=valid_state(), now_ns=NOW)
    assert first.decision == Decision.ACCEPT
    again = gate.evaluate(
        make_intent(), state=valid_state(), now_ns=NOW, active=make_intent()
    )
    assert again.decision == Decision.ACCEPT
    assert again.code == "idempotent"

    busy = gate.evaluate(
        make_intent(task_id="t-2"), state=valid_state(), now_ns=NOW,
        active=make_intent(task_id="t-1"),
    )
    assert busy.decision == Decision.REJECT
    assert busy.code == "REJECT_BUSY"


def test_platform_mismatch_rejected():
    gate = TaskGate(limits=GateLimits(local_platform="go2"))
    res = gate.evaluate(make_intent(platform="b1"), state=valid_state(), now_ns=NOW)
    assert res.decision == Decision.REJECT
    assert res.code == "REJECT_PLATFORM_MISMATCH"
