"""通信中间件层：通道表、QoS、禁控 payload、pub/sub。"""

import pytest

from legged_nav.transport import CHANNELS, CycloneDDSTransport, InProcessTransport, QoS


def test_channel_table_covers_four_domains():
    groups = {c.name.split("/")[0] for c in CHANNELS.values()}
    assert groups == {"rgb", "sensor", "spatial", "perception", "task", "motion", "exec"}
    assert CHANNELS["task/intent"].qos == QoS.RELIABLE
    assert CHANNELS["rgb/frame"].qos == QoS.BEST_EFFORT


def test_pubsub_inprocess():
    t = InProcessTransport()
    got = []
    t.subscribe("task/intent", got.append)
    err = t.publish("task/intent", {"task_id": "x", "schema": "motionslam.task_intent.v1"})
    assert err is None
    assert len(got) == 1 and got[0].payload["task_id"] == "x"
    assert len(t.log) == 1  # 留痕


def test_unknown_channel_and_forbidden_payload_rejected():
    t = InProcessTransport()
    assert t.publish("nope/x", {}).startswith("REJECT_UNKNOWN_CHANNEL")
    err = t.publish("motion/action_step", {"kind": "x", "params": {"cmd_vel": 1.0}})
    assert err == "REJECT_FORBIDDEN_CONTROL:cmd_vel"
    err2 = t.publish("task/intent", {"skills": [{"twist": {}}]})
    assert err2 == "REJECT_FORBIDDEN_CONTROL:twist"


def test_cyclone_backend_lazy_error_without_binding():
    try:
        import cyclonedds  # noqa: F401
    except ImportError:
        with pytest.raises(RuntimeError, match="InProcessTransport"):
            CycloneDDSTransport()
    else:
        pytest.skip("cyclonedds available — real DDS path not tested in CI")
