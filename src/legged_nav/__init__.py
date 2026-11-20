"""legged_nav：端侧自主导航闭环核心（ROS-free）。

链路：TaskIntent 契约准入 → 技能映射 → 执行编排 → 执行反馈。
模型决策只允许到达技能层，底层运动指令不属于意图字段。
"""

from .admission import AdmitDecision, TaskAdmission
from .executor import Forwarder, SimForwarder, StepResult
from .feedback import build_feedback
from .runtime import TaskRuntime, TaskStatus
from .spatial_context import SpatialStateStore, StreamContext, VisionToken
from .spatial_state import SpatialState, StateGate
from .task_intent import (
    SCHEMA_FEEDBACK,
    SCHEMA_INTENT,
    TaskIntent,
    check_forbidden_areas,
    validate_task_intent,
)
from .transport import CHANNELS, InProcessTransport, Message, Transport
from .vision_loop import VisionContext, VisionLoop, observation_to_intent
from .vlm_adapter import (
    QwenVLBackend,
    SemanticObservation,
    StubVLMBackend,
    VLMBackend,
)

__all__ = [
    "AdmitDecision",
    "CHANNELS",
    "Forwarder",
    "InProcessTransport",
    "Message",
    "QwenVLBackend",
    "SemanticObservation",
    "StubVLMBackend",
    "VLMBackend",
    "VisionContext",
    "VisionLoop",
    "VisionToken",
    "SCHEMA_FEEDBACK",
    "SCHEMA_INTENT",
    "SimForwarder",
    "SpatialState",
    "SpatialStateStore",
    "StateGate",
    "StreamContext",
    "StepResult",
    "TaskAdmission",
    "TaskIntent",
    "TaskRuntime",
    "TaskStatus",
    "Transport",
    "build_feedback",
    "check_forbidden_areas",
    "observation_to_intent",
    "validate_task_intent",
]
