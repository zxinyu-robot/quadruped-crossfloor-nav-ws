"""runtime_intent：ActionGroup / ThingEnvelope → TaskIntent 的降维规则。"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from runtime_intent import (  # noqa: E402
    INTENT_SCHEMA,
    build_intent,
    skill_goal,
    skills_from_envelope,
    skills_from_stages,
)


def test_build_intent_is_contract_shaped():
    body = build_intent(
        task_id="t-1",
        skills=[{"name": "ExploreFrontier"}],
        platform="unitree_go2",
        robot_id="go2-01",
        map_id="m1",
        revision=7,
        frame_id="world",
        priority=3,
        deadline_s=1.5,
    )
    assert body["schema"] == INTENT_SCHEMA
    assert body["robot"] == {"platform": "unitree_go2", "sn": "go2-01"}
    assert body["world_ref"] == {"map_id": "m1", "revision": 7, "frame_id": "world"}
    assert body["skills"] == [{"name": "ExploreFrontier"}]
    assert body["execution"] == {"priority": 3, "preemptible": False}
    assert body["policy"] == {"deadline_s": 1.5}


def test_build_intent_omits_nonpositive_deadline():
    """契约里 deadline_s 出现就必须 > 0，否则校验直接拒。"""
    body = build_intent(
        task_id="t-2",
        skills=[{"name": "Stand"}],
        platform="unitree_go2",
        robot_id="go2-01",
        map_id="m1",
        revision=0,
        deadline_s=0.0,
    )
    assert "policy" not in body


def test_skill_goal_normalises_and_rejects_garbage():
    assert skill_goal({"x": 1, "y": 2, "z": 0.35, "frame_id": "map"}) == {
        "x": 1.0,
        "y": 2.0,
        "z": 0.35,
        "frame_id": "map",
    }
    assert skill_goal({"x": 1}) is None
    assert skill_goal("nope") is None


def test_search_stage_has_no_goal():
    skills = skills_from_stages(["SEARCH_SEMANTIC_TARGET"], {"stages": []})
    assert skills == [{"name": "ExploreFrontier"}]


def test_nav_stage_requires_resolvable_waypoint():
    ag = {
        "stages": [
            {
                "stage_type": "NAVIGATE",
                "waypoints": [{"x": 2.0, "y": 3.0, "z": 0.0, "frame_id": "world"}],
            }
        ]
    }
    assert skills_from_stages(["NAV_TO_DESTINATION"], ag) == [
        {"name": "Navigate", "goal": {"x": 2.0, "y": 3.0, "z": 0.0, "frame_id": "world"}}
    ]
    # 有导航阶段但没有 waypoint：fail closed
    assert skills_from_stages(["NAV_TO_DESTINATION"], {"stages": []}) is None


def test_unknown_stage_falls_back_to_stand():
    assert skills_from_stages(["REPORT"], {"stages": []}) == [{"name": "Stand"}]


def test_envelope_maps_actions_to_skills():
    env = {
        "data": {
            "services": {
                "actions": [
                    {"skill": "Search"},
                    {"skill": "Navigate", "goal": {"x": 4.0, "y": 5.0, "z": 0.0}},
                ]
            }
        }
    }
    skills = skills_from_envelope(["Search", "Navigate"], env)
    assert [s["name"] for s in skills] == ["ExploreFrontier", "Navigate"]
    assert skills[1]["goal"]["x"] == 4.0


def test_envelope_navigate_without_goal_fails_closed():
    env = {"data": {"services": {"actions": [{"skill": "Navigate"}]}}}
    assert skills_from_envelope(["Navigate"], env) is None


@pytest.mark.parametrize("skills", [[], ["Fly"], ["Unknown"]])
def test_envelope_empty_or_unknown_returns_none(skills):
    assert skills_from_envelope(skills, {"data": {}}) is None
