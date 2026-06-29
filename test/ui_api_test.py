"""Tests for the front-end-agnostic UI API (issue #54), Phase 0.

Covers:
* the ``_get_ui_content`` source fix -- specialized callbacks must return
  JSON-serializable content (no numpy arrays).
* ``ros_sugar.ui_node.api.build_interfaces`` -- the discovery document.
* the optional-dependency import boundary -- importing the main package must
  not pull in the web stack (fasthtml/starlette/uvicorn).
"""

import json
import subprocess
import sys

import numpy as np
import pytest

from ros_sugar.io.topic import Topic


# ---------------------------------------------------------------------------
# _get_ui_content source fix: no numpy in the UI payloads
# ---------------------------------------------------------------------------
def test_geometry_callbacks_return_jsonable_ui_content():
    """Point/Pose/Odom/MultiArray ``_get_ui_content`` must be JSON-safe."""
    from geometry_msgs.msg import Point as ROSPoint, Pose as ROSPose
    from std_msgs.msg import Float64MultiArray

    from ros_sugar.io.callbacks import (
        PointCallback,
        PoseCallback,
        StdMsgArrayCallback,
    )

    point_cb = PointCallback(Topic(name="p", msg_type="Point"))
    point_cb.msg = ROSPoint(x=1.0, y=2.0, z=3.0)
    point_content = point_cb._get_ui_content()
    assert point_content["data"] == [1.0, 2.0, 3.0]
    assert not isinstance(point_content["data"], np.ndarray)
    json.dumps(point_content)

    pose_cb = PoseCallback(Topic(name="po", msg_type="Pose"))
    pose_cb.msg = ROSPose()
    pose_content = pose_cb._get_ui_content()
    assert isinstance(pose_content["data"], list)
    json.dumps(pose_content)

    arr_cb = StdMsgArrayCallback(Topic(name="a", msg_type="Float64MultiArray"))
    msg = Float64MultiArray()
    msg.data = [1.0, 2.0, 3.0]
    arr_cb.msg = msg
    arr_content = arr_cb._get_ui_content()
    assert isinstance(arr_content, list)
    json.dumps(arr_content)


# ---------------------------------------------------------------------------
# Discovery document
# ---------------------------------------------------------------------------
class _FakeConfig:
    api_stream_default_rate = 10.0
    api_max_stream_rate = 30.0


class _FakeNode:
    """Minimal stand-in for a UINode for testing build_interfaces."""

    def __init__(self, in_topics, out_topics, srv=None, act=None):
        self.in_topics = in_topics  # API outputs (robot -> client)
        self.out_topics = out_topics  # API inputs (client -> robot)
        self._srv = srv or []
        self._act = act or []
        self.config = _FakeConfig()

    def srv_clients_inputs_dicts(self):
        return self._srv

    def action_clients_inputs_dicts(self):
        return self._act


def test_build_interfaces_document():
    pytest.importorskip("starlette")
    from ros_sugar.ui_node.api import build_interfaces

    node = _FakeNode(
        in_topics=[Topic(name="odom", msg_type="Odometry"), Topic(name="cam", msg_type="Image")],
        out_topics=[Topic(name="cmd_vel", msg_type="Twist")],
        srv=[{"name": "dock", "type": "Trigger", "fields": {}}],
        act=[{"name": "move", "type": "NavigateToPose", "fields": {"x": "float"}}],
    )
    doc = build_interfaces(node)

    # Inputs (client -> robot) come from out_topics
    assert [i["name"] for i in doc["inputs"]] == ["cmd_vel"]
    assert doc["inputs"][0]["publish"] == "POST /api/inputs/cmd_vel"

    # Outputs (robot -> client) come from in_topics; Image is a binary stream
    outputs = {o["name"]: o for o in doc["outputs"]}
    assert outputs["odom"]["binary"] is False
    assert outputs["cam"]["binary"] is True
    assert outputs["odom"]["stream"] == "WS /api/outputs/odom"
    assert outputs["odom"]["latest"] == "GET /api/outputs/odom/latest"

    # Services / actions
    assert doc["services"][0]["call"] == "POST /api/services/dock"
    assert doc["actions"][0]["feedback"] == "WS /api/actions/move/feedback"
    assert doc["actions"][0]["goal_schema"] == {"x": "float"}

    # Stream rates surface the config
    assert doc["stream"] == {"default_rate": 10.0, "max_rate": 30.0}

    # The whole document must be JSON-serializable
    json.dumps(doc)


# ---------------------------------------------------------------------------
# Optional-dependency import boundary
# ---------------------------------------------------------------------------
def test_main_import_does_not_pull_web_stack():
    """Importing the main package must not require fasthtml/starlette/uvicorn.

    These remain optional so ``ros_sugar`` runs on systems (e.g. Python 3.8)
    where the web UI stack is unavailable. Only the UI node executable and the
    guarded web-tier modules (frontend/elements/api) may import them.
    """
    code = (
        "import sys\n"
        "import ros_sugar\n"
        "import ros_sugar.ui_node\n"
        "import ros_sugar.launch.launcher\n"
        "import ros_sugar.io.callbacks\n"
        "leaked = [m for m in ('fasthtml', 'monsterui', 'starlette', 'uvicorn')\n"
        "          if m in sys.modules]\n"
        "assert not leaked, f'web stack leaked into main import path: {leaked}'\n"
    )
    subprocess.check_call([sys.executable, "-c", code])


# ---------------------------------------------------------------------------
# Phase 1: inputs + services REST endpoints
# ---------------------------------------------------------------------------
class _ApiNode:
    """Fake UINode exposing just the surface the API routes use."""

    def __init__(self):
        self.in_topics = [Topic(name="odom", msg_type="Odometry")]
        self.out_topics = [Topic(name="cmd_vel", msg_type="Twist")]
        self.config = _FakeConfig()
        self.published = []
        self.publish_error = None  # exception to raise from publish_data
        self.service_response = None  # raw ROS response to return
        self.service_error = None  # exception to raise from send_srv_call

    def srv_clients_inputs_dicts(self):
        return [{"name": "reset", "type": "Trigger", "fields": {}}]

    def action_clients_inputs_dicts(self):
        return []

    def publish_data(self, data):
        if self.publish_error is not None:
            raise self.publish_error
        self.published.append(data)
        return 2

    def send_srv_call(self, data):
        if self.service_error is not None:
            raise self.service_error
        return self.service_response


def _make_client(node):
    pytest.importorskip("httpx")
    pytest.importorskip("fasthtml")
    from fasthtml.common import fast_app
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import register_api

    app, _ = fast_app()
    register_api(app, node)
    return TestClient(app)


def test_publish_input_ok():
    node = _ApiNode()
    client = _make_client(node)
    resp = client.post("/api/inputs/cmd_vel", json={"linear": {"x": 1.0}})
    assert resp.status_code == 200
    assert resp.json() == {"published": "cmd_vel", "subscribers": 2}
    # publish_data receives the topic name folded into the field dict
    assert node.published == [{"topic_name": "cmd_vel", "linear": {"x": 1.0}}]


def test_publish_input_unknown_topic_404():
    client = _make_client(_ApiNode())
    resp = client.post("/api/inputs/nope", json={})
    assert resp.status_code == 404


def test_publish_input_not_ready_503():
    node = _ApiNode()
    node.publish_error = RuntimeError("Publisher for input topic 'cmd_vel' is not ready")
    client = _make_client(node)
    resp = client.post("/api/inputs/cmd_vel", json={})
    assert resp.status_code == 503


def test_publish_input_bad_fields_400():
    node = _ApiNode()
    node.publish_error = ValueError("Cannot build a Twist message")
    client = _make_client(node)
    resp = client.post("/api/inputs/cmd_vel", json={"bogus": 1})
    assert resp.status_code == 400


def test_call_service_ok():
    from std_srvs.srv import Trigger

    node = _ApiNode()
    response = Trigger.Response()
    response.success = True
    response.message = "ok"
    node.service_response = response
    client = _make_client(node)

    resp = client.post("/api/services/reset", json={})
    assert resp.status_code == 200
    assert resp.json() == {"service": "reset", "response": {"success": True, "message": "ok"}}


def test_call_service_unknown_404():
    client = _make_client(_ApiNode())
    resp = client.post("/api/services/nope", json={})
    assert resp.status_code == 404


def test_call_service_no_response_502():
    node = _ApiNode()
    node.service_response = None  # simulates no response received
    client = _make_client(node)
    resp = client.post("/api/services/reset", json={})
    assert resp.status_code == 502


def test_msg_to_jsonable_handles_arrays():
    pytest.importorskip("starlette")
    from sensor_msgs.msg import LaserScan

    from ros_sugar.ui_node.api import _msg_to_jsonable

    scan = LaserScan()
    scan.ranges = [0.1, 0.2, 0.3]  # float32[] -> array.array internally
    out = _msg_to_jsonable(scan)
    assert out["ranges"] == [pytest.approx(0.1), pytest.approx(0.2), pytest.approx(0.3)]
    json.dumps(out)  # must not raise
