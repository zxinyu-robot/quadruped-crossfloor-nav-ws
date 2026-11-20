"""技能层：IntentSkill → 可执行动作序列。

技能是模型决策与底层控制之间的唯一通道。每个技能把自己编译成
一组 ActionStep（高层运动原语），交给执行层；技能本身不产生 cmd_vel。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .contract_gen import SKILL_KINDS, SKILL_PARAMS, SKILL_TIMEOUTS_S
from .task_intent import IntentSkill


@dataclass
class ActionStep:
    """执行层最小动作单元（由 Forwarder 适配到平台 API）。"""

    kind: str  # navigate_to | return_home | explore | stand | stop
    goal: dict[str, Any] | None = None
    timeout_s: float = 30.0
    params: dict[str, Any] | None = None


class SkillError(ValueError):
    pass


def compile_skill(skill: IntentSkill) -> list[ActionStep]:
    """技能 → 动作序列；未知技能 fail closed。

    技能种类、超时与特有参数全部来自 contracts/runtime_contract.json
    （生成物 ``contract_gen``），与 C++ ``compile_skill`` 共用同一份契约。
    """
    kind = SKILL_KINDS.get(skill.name)
    if kind is None:
        raise SkillError(f"unsupported skill: {skill.name}")
    if kind == "navigate_to" and skill.goal is None:
        raise SkillError("Navigate requires goal")
    goal = None
    if skill.goal is not None:
        goal = {
            "x": skill.goal.x,
            "y": skill.goal.y,
            "z": skill.goal.z,
            "frame_id": skill.goal.frame_id,
        }
    params = dict(SKILL_PARAMS.get(skill.name, {})) or None
    return [
        ActionStep(
            kind=kind,
            goal=goal,
            timeout_s=SKILL_TIMEOUTS_S[skill.name],
            params=params,
        )
    ]
