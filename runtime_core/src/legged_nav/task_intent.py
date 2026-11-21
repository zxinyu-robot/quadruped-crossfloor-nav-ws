"""Task Intent 契约校验与准入前过滤（无 ROS 依赖，可单测）。

schema: motionslam.task_intent.v1（字段集与 quadruped-crossfloor-nav-ws 保持一致）
{
  "schema": "motionslam.task_intent.v1",
  "task_id": "task-...",
  "robot":   {"platform": "unitree_go2_edu", "sn": "go2_001"},
  "world_ref": {"map_id": "map_01", "floor_id": "floor_01",
                "frame_id": "world", "revision": 1},
  "skills":  [{"name": "Navigate", "goal": {"x": 2.0, "y": 0.5, "z": 0.35}},
              {"name": "ReturnHome"}],
  "policy":  {"deadline_s": 120.0},
  "execution":  {"priority": 0, "preemptible": false},
  "navigation": {"forbidden_areas": ["restricted_C"]}
}

安全隔离：意图中禁止携带任何底层控制字段（cmd_vel / twist / 路径话题名），
模型决策只允许到达技能层，不允许直达电机/速度层。

契约常量唯一来源：contracts/runtime_contract.json（生成物
src/legged_nav/contract_gen.py，与 C++ 侧 contract_gen.hpp 同源）。归一化与类型
语义与 C++ 权威实现一致：字符串去 ASCII 首尾空白，非字符串视为缺失，数值字段只
接受 JSON number（字符串不做隐式转换），保证两侧判定逐一等价。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .contract_gen import (
    ALLOWED_PLATFORMS,
    ALLOWED_SKILLS,
    FEEDBACK_SCHEMA,
    FORBIDDEN_CONTROL_KEYS,
    INTENT_SCHEMA,
)

SCHEMA_INTENT = INTENT_SCHEMA
SCHEMA_FEEDBACK = FEEDBACK_SCHEMA

_ASCII_WS = " \t\n\r\f\v"


def _norm(value: Any) -> str:
    """契约归一化：仅接受 str，去 ASCII 首尾空白；其余类型视为缺失。"""
    return value.strip(_ASCII_WS) if isinstance(value, str) else ""


def _num(value: Any) -> float | None:
    """契约数值：只接受 JSON number（bool 不算数字）；字符串不做隐式转换。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _has_forbidden_control(obj: Any) -> str | None:
    if isinstance(obj, dict):
        for key, val in obj.items():
            if str(key) in FORBIDDEN_CONTROL_KEYS:
                return f"REJECT_FORBIDDEN_CONTROL:{key}"
            nested = _has_forbidden_control(val)
            if nested:
                return nested
    elif isinstance(obj, list):
        for item in obj:
            nested = _has_forbidden_control(item)
            if nested:
                return nested
    return None


@dataclass
class IntentGoal:
    x: float
    y: float
    z: float
    frame_id: str = "world"


@dataclass
class IntentSkill:
    name: str
    goal: IntentGoal | None = None


@dataclass
class TaskIntent:
    task_id: str = ""
    platform: str = ""
    robot_id: str = ""
    map_id: str = ""
    floor_id: str = ""
    frame_id: str = "world"
    revision: int = -1
    skills: list[IntentSkill] = field(default_factory=list)
    deadline_s: float = 0.0
    priority: int = 0
    preemptible: bool = False
    forbidden_areas: list[str] = field(default_factory=list)


def validate_task_intent(
    raw: Any,
    *,
    allowed_skills: set[str] | None = None,
    local_platform: str = "",
    local_robot_id: str = "",
) -> tuple[TaskIntent | None, str | None]:
    """校验 Task Intent；返回 (intent, None) 或 (None, REJECT_* 错误码)。"""
    if not isinstance(raw, dict):
        return None, "REJECT_INTENT_TYPE"
    forbidden = _has_forbidden_control(raw)
    if forbidden:
        return None, forbidden
    schema = _norm(raw.get("schema"))
    if schema != SCHEMA_INTENT:
        return None, f"REJECT_SCHEMA:{schema or 'missing'}"

    task_id = _norm(raw.get("task_id"))
    if not task_id:
        return None, "REJECT_MISSING_TASK_ID"

    robot = raw.get("robot") if isinstance(raw.get("robot"), dict) else {}
    platform = _norm(robot.get("platform"))
    if not platform:
        return None, "REJECT_MISSING_ROBOT"
    if platform not in ALLOWED_PLATFORMS:
        return None, f"REJECT_ROBOT_PLATFORM:{platform}"
    sn = _norm(robot.get("sn")) or _norm(robot.get("robot_id"))
    if not sn:
        return None, "REJECT_MISSING_ROBOT_ID"
    if local_platform and platform != _norm(local_platform):
        return None, "REJECT_ROBOT_MISMATCH"
    if local_robot_id and sn != _norm(local_robot_id):
        return None, "REJECT_ROBOT_MISMATCH"

    world = raw.get("world_ref")
    if not isinstance(world, dict):
        return None, "REJECT_MISSING_WORLD_REF"
    map_id = _norm(world.get("map_id"))
    if not map_id:
        return None, "REJECT_MISSING_MAP_ID"
    revision_raw = _num(world.get("revision"))
    if revision_raw is None or revision_raw < 0:
        return None, "REJECT_BAD_REVISION"
    revision = int(revision_raw)

    skills_raw = raw.get("skills")
    if not isinstance(skills_raw, list) or not skills_raw:
        return None, "REJECT_MISSING_SKILLS"
    cap = allowed_skills if allowed_skills is not None else set(ALLOWED_SKILLS)
    skills: list[IntentSkill] = []
    for i, item in enumerate(skills_raw):
        if not isinstance(item, dict):
            return None, f"REJECT_SKILL_{i}_TYPE"
        name = _norm(item.get("name")) or _norm(item.get("skill"))
        if not name:
            return None, f"REJECT_SKILL_{i}_MISSING_NAME"
        if name not in ALLOWED_SKILLS or name not in cap:
            return None, f"REJECT_SKILL_UNSUPPORTED:{name}"
        goal: IntentGoal | None = None
        if name == "Navigate":
            g = item.get("goal")
            if not isinstance(g, dict):
                return None, f"REJECT_SKILL_{i}_MISSING_GOAL"
            gx, gy, gz = _num(g.get("x")), _num(g.get("y")), _num(g.get("z"))
            if gx is None or gy is None or gz is None:
                return None, f"REJECT_SKILL_{i}_BAD_GOAL"
            frame = _norm(g.get("frame_id")) or _norm(world.get("frame_id")) or "world"
            goal = IntentGoal(gx, gy, gz, frame)
        skills.append(IntentSkill(name=name, goal=goal))

    policy = raw.get("policy") if isinstance(raw.get("policy"), dict) else {}
    deadline = 0.0
    if policy.get("deadline_s") is not None:
        deadline_raw = _num(policy["deadline_s"])
        if deadline_raw is None or deadline_raw <= 0:
            return None, "REJECT_BAD_DEADLINE"
        deadline = deadline_raw

    execution = raw.get("execution") if isinstance(raw.get("execution"), dict) else {}
    priority = 0
    if execution.get("priority") is not None:
        priority_raw = _num(execution["priority"])
        if priority_raw is None:
            return None, "REJECT_BAD_PRIORITY"
        priority = int(priority_raw)
    preemptible = execution.get("preemptible", False)
    if not isinstance(preemptible, bool):
        return None, "REJECT_BAD_PREEMPTIBLE"

    navigation = raw.get("navigation") if isinstance(raw.get("navigation"), dict) else {}
    forbidden_areas: list[str] = []
    areas_raw = navigation.get("forbidden_areas")
    if areas_raw is not None:
        if not isinstance(areas_raw, list):
            return None, "REJECT_BAD_FORBIDDEN_AREAS"
        for area in areas_raw:
            area_id = _norm(area)
            if not area_id:
                return None, "REJECT_BAD_FORBIDDEN_AREAS"
            forbidden_areas.append(area_id)

    return (
        TaskIntent(
            task_id=task_id,
            platform=platform,
            robot_id=sn,
            map_id=map_id,
            floor_id=_norm(world.get("floor_id")),
            frame_id=_norm(world.get("frame_id")) or "world",
            revision=revision,
            skills=skills,
            deadline_s=deadline,
            priority=priority,
            preemptible=preemptible,
            forbidden_areas=forbidden_areas,
        ),
        None,
    )


Polygon = list[tuple[float, float]]


def _point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def check_forbidden_areas(intent: TaskIntent, areas: dict[str, Polygon]) -> str | None:
    """禁区由端侧本地配置解析；未知区域 ID 失败即拒（fail closed）。

    仅校验 Navigate 目标点，不约束全局路径穿越禁区（局部规划尚无禁行区输入）。
    """
    for area_id in intent.forbidden_areas:
        poly = areas.get(area_id)
        if poly is None or len(poly) < 3:
            return f"REJECT_UNKNOWN_AREA:{area_id}"
        for i, skill in enumerate(intent.skills):
            if skill.goal is not None and _point_in_polygon(skill.goal.x, skill.goal.y, poly):
                return f"REJECT_GOAL_IN_FORBIDDEN_AREA:{area_id}@skill{i}"
    return None
