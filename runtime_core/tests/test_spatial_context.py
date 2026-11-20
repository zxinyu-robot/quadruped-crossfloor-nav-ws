"""统一空间状态契约 + Fast-Slow 流式上下文。"""

import time

from legged_nav.spatial_context import (
    SpatialStateStore,
    StreamContext,
    VisionToken,
)
from legged_nav.spatial_state import SpatialState

NOW = time.time_ns()


def make_state(**kw):
    base = dict(map_id="map_01", revision=1, localization_confidence=0.9,
                valid=True, timestamp_ns=NOW)
    base.update(kw)
    return SpatialState(**base)


# ---------- SpatialStateStore ----------


def test_store_incremental_update_bumps_revision():
    s = SpatialStateStore()
    e1 = s.update("pose", make_state(), now_ns=NOW)
    assert e1.revision == 1
    e2 = s.update("pose", make_state(x=1.0), now_ns=NOW + 1)
    assert e2.revision == 2 and e2.state.x == 1.0
    s.update("goal", None, goal={"x": 3.0, "y": 0.0, "z": 0.0}, now_ns=NOW + 2)
    assert s.revision == 3 and len(s.snapshot()) == 2


def test_expired_or_invalid_entries_filtered():
    s = SpatialStateStore()
    s.update("pose", make_state(), ttl_ns=1_000, now_ns=NOW)
    s.update("pose2", make_state(), ttl_ns=10**9, now_ns=NOW)
    live = s.entries(now_ns=NOW + 5_000)
    assert "pose" not in live and "pose2" in live
    s.update("pose3", make_state(valid=False), now_ns=NOW)
    assert "pose3" not in s.entries(now_ns=NOW)


# ---------- StreamContext ----------


def test_sliding_window_compresses_overflow():
    sc = StreamContext(window=3, dedup_dist_m=0.0)
    for i in range(6):
        sc.push(VisionToken(frame_id=f"f{i}", summary=f"scene{i}", pose=(float(i), 0, 0)))
    stats = sc.stats()
    assert stats["fast_len"] == 3
    assert stats["slow_compressed"] == 3      # 3 条滑出并压缩
    assert stats["slow_scenes"]["scene0"] == 1
    assert stats["kv_cached"] == 6


def test_lio_pose_dedup_drops_redundant_observations():
    sc = StreamContext(window=8, dedup_dist_m=0.3)
    ok1 = sc.push(VisionToken(frame_id="a", summary="kitchen", pose=(0, 0, 0)))
    ok2 = sc.push(VisionToken(frame_id="b", summary="kitchen", pose=(0.1, 0.05, 0)))  # 0.11m < 0.3m
    ok3 = sc.push(VisionToken(frame_id="c", summary="corridor", pose=(2.0, 0, 0)))
    assert ok1 and not ok2 and ok3
    assert sc.stats()["dedup_dropped"] == 1
    assert sc.stats()["kv_cached"] == 2


def test_kv_reuse_ratio_and_odom():
    sc = StreamContext(window=4, dedup_dist_m=0.1)
    for i in range(4):
        sc.push(VisionToken(frame_id=f"f{i}", summary="s", pose=(float(i), 0, 0)))
    st = sc.stats()
    assert st["kv_cached"] == 4 and st["fast_len"] == 4
    assert st["slow_odom_m"] == 3.0  # 1+1+1
    sc.push(VisionToken(frame_id="f4", summary="s", pose=(4.0, 0, 0)))
    st2 = sc.stats()
    assert st2["kv_cached"] == 5 and st2["fast_len"] == 4
    assert st2["slow_compressed"] == 1
    assert st2["kv_reuse_ratio"] == 1.0 - 4 / 5


def test_context_tokens_and_compress_oldest():
    sc = StreamContext(window=5)
    sc.push(VisionToken(frame_id="f0", summary="a,b", pose=(0, 0, 0)))
    evicted = sc.compress_oldest()
    assert evicted and evicted.frame_id == "f0"
    assert sc.stats()["slow_scenes"] == {"a": 1, "b": 1}
    assert sc.compress_oldest() is None


def test_vision_loop_feeds_stream_tokens():
    from legged_nav.executor import SimForwarder
    from legged_nav.runtime import TaskRuntime
    from legged_nav.vision_loop import VisionContext, VisionLoop
    from legged_nav.vlm_adapter import SemanticObservation, StubVLMBackend

    fwd = SimForwarder(initial_state=make_state(revision=3))
    rt = TaskRuntime(fwd, local_platform="unitree_go2_edu", local_robot_id="go2_001")
    ctx = VisionContext(
        task_prompt="去厨房", robot={"platform": "unitree_go2_edu", "sn": "go2_001"},
        world_ref={"map_id": "map_01", "revision": 3},
        goal={"x": 2.0, "y": 0.5, "z": 0.35},
    )
    sc = StreamContext(window=3, dedup_dist_m=0.3)
    loop = VisionLoop(
        StubVLMBackend(preset=SemanticObservation(
            scene_class="kitchen", confidence=0.9, target_visible=True,
        )), rt, ctx, stream=sc,
    )
    loop.step(b"f", frame_id="f0", seq=0, pose=(0, 0, 0))
    loop.step(b"f", frame_id="f1", seq=1, pose=(0.05, 0, 0))  # 去重丢弃
    loop.step(b"f", frame_id="f2", seq=2, pose=(1.0, 0, 0))
    st = sc.stats()
    assert st["fast_len"] == 2 and st["dedup_dropped"] == 1


def test_yaw_from_axis_angle():
    """traj 轴角旋转 → yaw：绕 z 轴 90°、零向量归零。"""
    import math

    from legged_nav.spatial_state import yaw_from_axis_angle
    yaw = yaw_from_axis_angle(0.0, 0.0, math.pi / 2)
    assert abs(yaw - 90.0) < 0.5
    assert yaw_from_axis_angle(0.0, 0.0, 0.0) == 0.0


def test_blocking_hazards():
    """深度门控：近场走廊障碍→阻挡；远场/无效→放行或保守（纯 Python 嵌套列表）。"""
    from legged_nav.spatial_state import blocking_hazards

    hz = ["cup", "bottle"]
    near = [[3000] * 10 for _ in range(10)]
    for row in near:
        row[4] = row[5] = 500
    far = [[3000] * 10 for _ in range(10)]
    zeros = [[0] * 10 for _ in range(10)]
    assert blocking_hazards(near, hz) == hz            # 走廊内 0.5m → 阻挡
    assert blocking_hazards(far, hz) == []             # 3m → 放行
    assert blocking_hazards(zeros, hz) == hz           # 无效深度→保守
    assert blocking_hazards(None, hz) == hz
    assert blocking_hazards([], hz) == hz
    assert blocking_hazards([[3000] * 10, [3000] * 3], hz) == hz  # 行宽不齐→保守
