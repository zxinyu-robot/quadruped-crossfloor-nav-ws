"""视觉驱动的闭环编排：RGB 帧流 → VLM 语义观测 → TaskIntent 合成 → TaskRuntime。

语义决策规则（保守、可审计）：
- 视野含危险标签 → 生成 Stop 技能意图；
- 目标可见且置信度达标 → 生成 Navigate 意图（目标位姿由任务语言携带的空间参数提供）；
- 无目标、无危险 → 不生成意图（保持观察）。

语义观测只影响技能选择与参数校验后的意图合成；
空间状态门控（定位/版本一致性）仍由 TaskRuntime 独立裁决。
"""
from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from .runtime import TaskRuntime
from .spatial_context import StreamContext, VisionToken
from .task_intent import SCHEMA_INTENT
from .vlm_adapter import SemanticObservation, VLMBackend


@dataclass
class VisionContext:
    """任务上下文：任务语言 + 目标空间参数 + 世界引用。"""

    task_prompt: str
    robot: dict[str, str]
    world_ref: dict[str, Any]
    goal: dict[str, float] | None = None          # Navigate 目标位姿（world 系，米）
    deadline_s: float = 120.0
    min_confidence: float = 0.6                      # 观测置信度下限，低于不发意图


@dataclass
class VisionDecision:
    """一次视觉步的决策留痕。"""

    observation: SemanticObservation
    intent: dict[str, Any] | None
    feedback: dict[str, Any] | None
    reason: str = ""


def observation_to_intent(
    obs: SemanticObservation,
    ctx: VisionContext,
    seq: int,
) -> tuple[dict[str, Any] | None, str]:
    """语义观测 → TaskIntent（或 None + 原因）。保守规则，逐条可解释。"""
    if obs.confidence < ctx.min_confidence:
        return None, f"low_observation_confidence:{obs.confidence:.2f}"
    if obs.hazards:
        skills = [{"name": "Stop"}]
        reason = f"hazard:{','.join(obs.hazards)}"
    elif obs.target_visible and ctx.goal is not None:
        skills = [{"name": "Navigate", "goal": dict(ctx.goal)}]
        reason = "target_visible"
    else:
        return None, "no_actionable_observation"

    return (
        {
            "schema": SCHEMA_INTENT,
            "task_id": f"vision-{seq:04d}",
            "robot": ctx.robot,
            "world_ref": ctx.world_ref,
            "skills": skills,
            "policy": {"deadline_s": ctx.deadline_s},
            "execution": {"priority": 0, "preemptible": True},
        },
        reason,
    )


class VisionLoop:
    """RGB 帧流 → VLM → 意图 → Runtime 的端到端编排。"""

    def __init__(
        self,
        backend: VLMBackend,
        runtime: TaskRuntime,
        ctx: VisionContext,
        stream: StreamContext | None = None,
    ) -> None:
        self.backend = backend
        self.runtime = runtime
        self.ctx = ctx
        self.stream = stream   # Fast-Slow 流式上下文（可选；LIO 位姿去重）
        self.decisions: list[VisionDecision] = []

    def step(
        self, frame: Any, frame_id: str = "", seq: int = 0,
        pose: tuple[float, float, float] | None = None,
    ) -> VisionDecision:
        obs = self.backend.observe(frame, self.ctx.task_prompt, frame_id=frame_id)
        if self.stream is not None and pose is not None:
            tags = [obs.scene_class] + obs.landmarks + obs.hazards
            self.stream.push(VisionToken(
                frame_id=frame_id, summary=",".join(t for t in tags if t), pose=pose,
            ))
        intent, reason = observation_to_intent(obs, self.ctx, seq)
        feedback = self.runtime.submit(intent, state=None) if intent is not None else None
        dec = VisionDecision(observation=obs, intent=intent, feedback=feedback, reason=reason)
        self.decisions.append(dec)
        return dec

    def run(self, frames: Iterable[Any]) -> list[VisionDecision]:
        for i, frame in enumerate(frames):
            self.step(frame, frame_id=f"frame-{i:04d}", seq=i)
        return self.decisions
