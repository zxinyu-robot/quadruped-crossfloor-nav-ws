"""TaskIntent 契约校验的失效场景用例。"""

import pytest

from legged_nav.task_intent import check_forbidden_areas, validate_task_intent

VALID = {
    "schema": "motionslam.task_intent.v1",
    "task_id": "task-001",
    "robot": {"platform": "unitree_go2_edu", "sn": "go2_001"},
    "world_ref": {"map_id": "map_01", "floor_id": "floor_01", "frame_id": "world", "revision": 3},
    "skills": [{"name": "Navigate", "goal": {"x": 2.0, "y": 0.5, "z": 0.35}}],
    "policy": {"deadline_s": 120.0},
    "execution": {"priority": 0, "preemptible": False},
}


def mutate(**over):
    import copy

    raw = copy.deepcopy(VALID)
    raw.update(over)
    return raw


def test_valid_intent_accepted():
    intent, err = validate_task_intent(VALID)
    assert err is None
    assert intent.task_id == "task-001"
    assert intent.skills[0].goal.x == 2.0
    assert intent.revision == 3


@pytest.mark.parametrize(
    ("over", "code_prefix"),
    [
        ({"schema": "other.v9"}, "REJECT_SCHEMA"),
        ({"task_id": ""}, "REJECT_MISSING_TASK_ID"),
        ({"robot": {"platform": "a1", "sn": "x"}}, "REJECT_ROBOT_PLATFORM"),
        ({"robot": {"platform": "unitree_go2_edu"}}, "REJECT_MISSING_ROBOT_ID"),
        ({"world_ref": {"revision": -1}}, "REJECT_MISSING_MAP_ID"),
        ({"skills": []}, "REJECT_MISSING_SKILLS"),
        ({"skills": [{"name": "Fly"}]}, "REJECT_SKILL_UNSUPPORTED"),
        ({"skills": [{"name": "Navigate"}]}, "REJECT_SKILL_0_MISSING_GOAL"),
        ({"skills": [{"name": "Navigate", "goal": {"x": "a", "y": 0, "z": 0}}]},
         "REJECT_SKILL_0_BAD_GOAL"),
        ({"policy": {"deadline_s": -1}}, "REJECT_BAD_DEADLINE"),
        ({"execution": {"priority": "high"}}, "REJECT_BAD_PRIORITY"),
        ({"navigation": {"forbidden_areas": [1]}}, "REJECT_BAD_FORBIDDEN_AREAS"),
    ],
)
def test_reject_codes(over, code_prefix):
    intent, err = validate_task_intent(mutate(**over))
    assert intent is None
    assert err.startswith(code_prefix)


def test_forbidden_control_rejected_anywhere():
    raw = mutate(skills=[{"name": "Navigate", "goal": {"x": 1, "y": 0, "z": 0, "cmd_vel": 0.5}}])
    intent, err = validate_task_intent(raw)
    assert err == "REJECT_FORBIDDEN_CONTROL:cmd_vel"

    raw2 = mutate(extra={"twist": {"linear": 1.0}})
    _, err2 = validate_task_intent(raw2)
    assert err2 == "REJECT_FORBIDDEN_CONTROL:twist"


def test_robot_mismatch():
    _, err = validate_task_intent(VALID, local_robot_id="go2_002")
    assert err == "REJECT_ROBOT_MISMATCH"


def test_forbidden_area_goal_rejected_and_unknown_area_fails_closed():
    intent, err = validate_task_intent(
        mutate(navigation={"forbidden_areas": ["zone_A"]})
    )
    assert err is None
    areas = {"zone_A": [(1.0, 0.0), (3.0, 0.0), (3.0, 1.0), (1.0, 1.0)]}
    assert check_forbidden_areas(intent, areas) == "REJECT_GOAL_IN_FORBIDDEN_AREA:zone_A@skill0"
    assert check_forbidden_areas(intent, {}).startswith("REJECT_UNKNOWN_AREA:zone_A")

    intent2, _ = validate_task_intent(
        mutate(skills=[{"name": "Navigate", "goal": {"x": 10.0, "y": 5.0, "z": 0.0}}],
               navigation={"forbidden_areas": ["zone_A"]})
    )
    assert check_forbidden_areas(intent2, areas) is None
