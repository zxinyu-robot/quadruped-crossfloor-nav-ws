"""视觉闭环端到端：RGB 帧 → VLM(stub) → 意图 → Runtime → 反馈。"""

import time

from legged_nav.executor import SimForwarder
from legged_nav.runtime import TaskRuntime
from legged_nav.spatial_state import SpatialState
from legged_nav.vision_loop import VisionContext, VisionLoop, observation_to_intent
from legged_nav.vlm_adapter import QwenVLBackend, SemanticObservation, StubVLMBackend

NOW = time.time_ns()

CTX = VisionContext(
    task_prompt="导航到厨房门口",
    robot={"platform": "unitree_go2_edu", "sn": "go2_001"},
    world_ref={"map_id": "map_01", "revision": 3},
    goal={"x": 2.0, "y": 0.5, "z": 0.35},
    deadline_s=120.0,
    min_confidence=0.6,
)


def make_state() -> SpatialState:
    return SpatialState(
        map_id="map_01", revision=3, localization_confidence=0.9,
        valid=True, timestamp_ns=time.time_ns(),
    )


def make_loop(backend):
    fwd = SimForwarder(initial_state=make_state())
    rt = TaskRuntime(fwd, local_platform="unitree_go2_edu", local_robot_id="go2_001")
    return VisionLoop(backend, rt, CTX), fwd, rt


def test_target_visible_synthesizes_navigate_and_succeeds():
    backend = StubVLMBackend(preset=SemanticObservation(
        scene_class="kitchen", confidence=0.9, target_visible=True,
        landmarks=["door_03"],
    ))
    loop, fwd, rt = make_loop(backend)
    dec = loop.step(b"\xff\xd8fake_jpeg", frame_id="f0", seq=0)
    assert dec.reason == "target_visible"
    assert dec.intent["skills"][0]["name"] == "Navigate"
    assert dec.feedback["event"] == "succeeded"
    assert fwd.steps[0]["kind"] == "navigate_to"
    assert fwd.state().x == 2.0


def test_hazard_synthesizes_stop():
    backend = StubVLMBackend(preset=SemanticObservation(
        scene_class="corridor", confidence=0.9, hazards=["dynamic_obstacle"],
    ))
    loop, fwd, _ = make_loop(backend)
    dec = loop.step(b"frame", seq=1)
    assert dec.reason.startswith("hazard:")
    assert dec.intent["skills"][0]["name"] == "Stop"
    assert dec.feedback["event"] == "succeeded"
    assert fwd.steps[0]["kind"] == "stop"


def test_low_confidence_and_empty_observation_no_intent():
    backend = StubVLMBackend(preset=SemanticObservation(confidence=0.2))
    loop, fwd, _ = make_loop(backend)
    dec = loop.step(b"f", seq=0)
    assert dec.intent is None
    assert dec.reason.startswith("low_observation_confidence")
    assert fwd.steps == []

    backend2 = StubVLMBackend(preset=SemanticObservation(confidence=0.9))
    loop2, fwd2, _ = make_loop(backend2)
    dec2 = loop2.step(b"f", seq=0)
    assert dec2.reason == "no_actionable_observation"
    assert fwd2.steps == []


def test_frame_stream_run_records_all_decisions():
    frames = [b"f0", b"f1", b"f2"]
    presets = [
        SemanticObservation(confidence=0.9, target_visible=True),
        SemanticObservation(confidence=0.9, hazards=["person"]),
        SemanticObservation(confidence=0.1),
    ]
    it = iter(presets)
    backend = StubVLMBackend(preset=lambda frame, prompt: next(it))
    loop, _, rt = make_loop(backend)
    decs = loop.run(frames)
    assert len(decs) == 3
    assert decs[0].intent is not None and decs[1].intent is not None
    assert decs[2].intent is None
    # 两个意图的 task_id 不同，各自完整走完
    assert decs[0].intent["task_id"] == "vision-0000"
    assert decs[1].intent["task_id"] == "vision-0001"
    assert decs[0].feedback["event"] == "succeeded"


def test_vlm_cannot_inject_low_level_control():
    """即使观测层被污染，合成意图不含禁控字段（契约层兜底也覆盖）。"""
    obs = SemanticObservation(confidence=0.9, target_visible=True,
                              hazards=[], landmarks=[])
    intent, reason = observation_to_intent(obs, CTX, 0)
    assert intent is not None
    assert "cmd_vel" not in str(intent)


def test_qwen_backend_lazy_error_without_torch():
    backend = QwenVLBackend(model_path="Qwen/Qwen2.5-VL-7B-Instruct")
    import pytest

    try:
        import torch  # noqa: F401
        import transformers  # noqa: F401
    except ImportError:
        with pytest.raises(RuntimeError, match="StubVLMBackend"):
            backend.observe(b"x", "task")
    else:
        pytest.skip("torch/transformers available — real GPU path not tested in CI")
