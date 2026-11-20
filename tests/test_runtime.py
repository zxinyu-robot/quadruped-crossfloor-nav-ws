"""端侧闭环端到端：准入 → 门控 → 编排 → 执行 → 反馈。"""

import time

from legged_nav.executor import SimForwarder
from legged_nav.runtime import TaskRuntime, TaskStatus
from legged_nav.spatial_state import SpatialState, StateGate

NOW = time.time_ns()


def make_state(**kw) -> SpatialState:
    base = dict(
        x=0.0, y=0.0, z=0.0, map_id="map_01", revision=3,
        localization_confidence=0.9, valid=True, timestamp_ns=NOW,
    )
    base.update(kw)
    return SpatialState(**base)


def make_intent(task_id="t1", **over):
    raw = {
        "schema": "motionslam.task_intent.v1",
        "task_id": task_id,
        "robot": {"platform": "unitree_go2_edu", "sn": "go2_001"},
        "world_ref": {"map_id": "map_01", "revision": 3},
        "skills": [{"name": "Navigate", "goal": {"x": 2.0, "y": 0.5, "z": 0.35}},
                   {"name": "ReturnHome"}],
        "policy": {"deadline_s": 120.0},
        "execution": {"priority": 0, "preemptible": False},
    }
    raw.update(over)
    return raw


def make_rt(**kw):
    fwd = SimForwarder(initial_state=make_state())
    return TaskRuntime(fwd, local_platform="unitree_go2_edu", local_robot_id="go2_001", **kw), fwd


def test_closed_loop_success_emits_full_feedback_chain():
    rt, fwd = make_rt()
    fb = rt.submit(make_intent(), state=make_state())
    assert fb["event"] == "succeeded"
    assert fb["schema"] == "motionslam.execution_feedback.v1"
    events = [e["event"] for e in rt.events]
    assert events[0] == "accepted" and events[1] == "started" and events[-1] == "succeeded"
    assert [s["kind"] for s in fwd.steps] == ["navigate_to", "return_home"]
    assert fwd.state().x == 0.0  # ReturnHome 归位


def test_reject_does_not_dispatch_to_forwarder():
    rt, fwd = make_rt()
    fb = rt.submit({"schema": "bad"})
    assert fb["state"] == "rejected"
    assert fwd.steps == []


def test_expired_state_holds_task():
    rt, _ = make_rt(state_gate=StateGate(ttl_ns=1_000))
    fb = rt.submit(make_intent(), state=make_state(timestamp_ns=NOW - 10_000))
    assert fb["event"] == "holding"
    assert "state_expired" in fb["detail"]


def test_revision_mismatch_interlocks():
    rt, _ = make_rt()
    fb = rt.submit(make_intent(), state=make_state(revision=9))
    assert fb["state"] == "holding"
    assert "revision_mismatch" in fb["detail"]


def test_low_confidence_interlocks():
    rt, _ = make_rt()
    fb = rt.submit(make_intent(), state=make_state(localization_confidence=0.1))
    assert "low_confidence" in fb["detail"]


def test_duplicate_task_id_rejected():
    rt, _ = make_rt()
    rt.submit(make_intent(), state=make_state())
    fb = rt.submit(make_intent(), state=make_state())
    assert "REJECT_DUPLICATE_TASK_ID" in fb["detail"]


def test_busy_rejects_and_preemptible_allows_preempt():
    rt, fwd = make_rt()
    # 让 Navigate 失败前保持 active：直接构造一个 active 句柄
    rt.submit(make_intent("t1", execution={"priority": 0, "preemptible": True}),
              state=make_state())
    # t1 已完成；再造忙态需手工：提交一个会失败的任务不适用，改为直接测 admission
    from legged_nav.admission import TaskAdmission
    from legged_nav.task_intent import validate_task_intent

    adm = TaskAdmission()
    i1, _ = validate_task_intent(make_intent("a", execution={"preemptible": True, "priority": 1}))
    i2, _ = validate_task_intent(make_intent("b", execution={"priority": 5}))
    i3, _ = validate_task_intent(make_intent("c", execution={"priority": 0}))
    assert adm.decide(i2, i1) == "preempt"
    assert adm.decide(i3, i1) == "REJECT_BUSY"
    assert adm.decide(i1, i1) == "idempotent"


def test_execution_failure_aborts_and_fails():
    rt, fwd = make_rt()
    fwd.fail_kinds.add("navigate_to")
    fb = rt.submit(make_intent(), state=make_state())
    assert fb["event"] == "failed"
    assert "step_failed:navigate_to" in fb["detail"]
    assert fwd.steps[-1] == {"kind": "abort", "reason": "sim_fail:navigate_to"}


def test_deadline_exceeded_fails():
    t = [NOW]

    def clock():
        return t[0]

    class SlowForwarder(SimForwarder):
        def execute(self, step, deadline_ns):
            t[0] += 200 * 10**9
            return super().execute(step, deadline_ns)

    rt = TaskRuntime(SlowForwarder(initial_state=make_state(timestamp_ns=t[0])), clock_ns=clock)
    fb = rt.submit(
        make_intent("t-dl", policy={"deadline_s": 120.0}),
        state=make_state(timestamp_ns=t[0]),
    )
    assert fb["event"] == "failed"
    assert fb["detail"] == "deadline_exceeded"


def test_cancel_active_task():
    rt, fwd = make_rt()
    from legged_nav.task_intent import validate_task_intent

    intent, _ = validate_task_intent(make_intent("tc"))
    from legged_nav.runtime import TaskHandle

    rt.active = TaskHandle(intent=intent, status=TaskStatus.RUNNING)
    fb = rt.cancel("tc")
    assert fb["event"] == "canceled"
    assert fwd.steps[-1]["kind"] == "abort"
    fb2 = rt.cancel("tc")
    assert fb2["event"] == "cancel_noop"


def test_forbidden_area_rejected_at_runtime():
    areas = {"zone_A": [(1.0, 0.0), (3.0, 0.0), (3.0, 1.0), (1.0, 1.0)]}
    rt, fwd = make_rt(forbidden_areas=areas)
    fb = rt.submit(
        make_intent(navigation={"forbidden_areas": ["zone_A"]}), state=make_state()
    )
    assert fb["state"] == "rejected"
    assert "REJECT_GOAL_IN_FORBIDDEN_AREA" in fb["detail"]
    assert fwd.steps == []
