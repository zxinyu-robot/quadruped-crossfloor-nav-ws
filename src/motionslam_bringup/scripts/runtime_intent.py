#!/usr/bin/env python3
"""上游意图 → TaskIntent 的降维（纯函数，便于单测；不依赖 ROS）。

主链约定：生产者（command_executor / 边端 adapter）只提交 TaskIntent，
五级校验由 `runtime_bridge_node` + `runtime_core.TaskGate` 完成。
"""
from __future__ import annotations

from typing import Any, Optional

INTENT_SCHEMA = "motionslam.task_intent.v1"

SKILL_WHITELIST = ("Navigate", "ExploreFrontier", "ReturnHome", "Stand", "Stop")

_STAGE_TO_SKILL = (
    ("SEARCH", "ExploreFrontier"),
    ("NAV", "Navigate"),
)


def skill_goal(goal: Any, default_frame: str = "world") -> Optional[dict[str, Any]]:
    """把任意 {x,y,z,frame_id} 归一成契约 goal；非法返回 None（fail closed）。"""
    if not isinstance(goal, dict):
        return None
    try:
        return {
            "x": float(goal.get("x")),
            "y": float(goal.get("y")),
            "z": float(goal.get("z", 0.0)),
            "frame_id": str(goal.get("frame_id") or default_frame or "world"),
        }
    except (TypeError, ValueError):
        return None


def skills_from_stages(
    stage_ids: list, action_group: dict[str, Any], default_frame: str = "world"
) -> Optional[list[dict[str, Any]]]:
    """ActionGroup stages → 技能列表。搜索阶段无导航 → ExploreFrontier；导航阶段必须带 goal。"""
    text = " ".join(str(s).upper() for s in stage_ids)
    has_search = "SEARCH" in text
    has_nav = "NAV" in text
    if has_search and not has_nav:
        return [{"name": "ExploreFrontier"}]
    if has_nav:
        from action_group import extract_nav_waypoint

        goal = skill_goal(extract_nav_waypoint(action_group), default_frame)
        if goal is None:
            return None
        return [{"name": "Navigate", "goal": goal}]
    return [{"name": "Stand"}]


def skills_from_envelope(
    skills: list, envelope: dict[str, Any], default_frame: str = "world"
) -> Optional[list[dict[str, Any]]]:
    """ThingEnvelope actions → 技能列表。Navigate 必须能解析出 goal，否则整体拒绝。"""
    out: list[dict[str, Any]] = []
    seen: list[str] = []
    for raw in skills:
        u = str(raw).upper()
        name = None
        if u in ("NAVIGATE", "NAV"):
            name = "Navigate"
        elif "SEARCH" in u or "EXPLORE" in u:
            name = "ExploreFrontier"
        elif u in ("STAND", "STOP"):
            name = "Stand"
        elif u in ("RETURNHOME", "RETURN_HOME"):
            name = "ReturnHome"
        if name and name not in seen:
            seen.append(name)
            out.append({"name": name})
    if not out:
        return None
    for item in out:
        if item["name"] != "Navigate":
            continue
        from behavior_mode import extract_nav_goal

        goal = skill_goal(extract_nav_goal(envelope), default_frame)
        if goal is None:
            return None
        item["goal"] = goal
    return out


def build_intent(
    *,
    task_id: str,
    skills: list[dict[str, Any]],
    platform: str,
    robot_id: str,
    map_id: str,
    revision: int,
    frame_id: str = "world",
    priority: int = 0,
    deadline_s: float = 0.0,
    preemptible: bool = False,
) -> dict[str, Any]:
    """组装 schema 合法的 TaskIntent。

    契约细节（见 runtime_core/src/legged_nav/task_intent.py）：
      - `policy.deadline_s` 若出现必须 > 0，否则 REJECT_BAD_DEADLINE → 这里 <=0 就不写；
      - `execution.priority` / `execution.preemptible` 才是优先级与抢占位（不在 policy 里）。
    """
    body: dict[str, Any] = {
        "schema": INTENT_SCHEMA,
        "task_id": task_id,
        "robot": {"platform": platform, "sn": robot_id},
        "world_ref": {"map_id": map_id, "revision": int(revision), "frame_id": frame_id},
        "skills": skills,
        "execution": {"priority": int(priority), "preemptible": bool(preemptible)},
    }
    if deadline_s is not None and float(deadline_s) > 0.0:
        body["policy"] = {"deadline_s": float(deadline_s)}
    return body
