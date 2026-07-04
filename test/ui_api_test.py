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

    # Outputs (robot -> client) come from in_topics; Image samples, Odometry pushes
    outputs = {o["name"]: o for o in doc["outputs"]}
    assert outputs["odom"]["mode"] == "push"
    assert outputs["cam"]["mode"] == "sampled"
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


def test_api_module_does_not_pull_fasthtml():
    """The API tier depends on Starlette, never on the browser stack.

    Importing ``ros_sugar.ui_node.api`` must not pull in fasthtml/monsterui, so
    the API can run on systems where only Starlette + uvicorn are installed.
    """
    pytest.importorskip("starlette")
    code = (
        "import sys\n"
        "import ros_sugar.ui_node.api\n"
        "leaked = [m for m in ('fasthtml', 'monsterui') if m in sys.modules]\n"
        "assert not leaked, f'browser stack leaked into the API module: {leaked}'\n"
    )
    subprocess.check_call([sys.executable, "-c", code])


# ---------------------------------------------------------------------------
# Phase 1: inputs + services REST endpoints
# ---------------------------------------------------------------------------
class _ApiNode:
    """Fake UINode exposing just the surface the API routes use."""

    def __init__(self):
        self.in_topics = [
            Topic(name="odom", msg_type="Odometry"),
            Topic(name="map", msg_type="OccupancyGrid"),
            Topic(name="plan", msg_type="Path"),
        ]
        self.out_topics = [
            Topic(name="cmd_vel", msg_type="Twist"),
            Topic(name="speech", msg_type="Audio"),
        ]
        self.config = _FakeConfig()
        self.published = []
        self.audio_published = []  # (name, base64) recorded by publish_audio
        self.publish_error = None  # exception to raise from publish_data
        self.service_response = None  # raw ROS response to return
        self.service_error = None  # exception to raise from send_srv_call
        self.latest = {}  # topic_name -> get_latest_output result
        self.goals = []  # recorded action goals
        self.goal_accepted = True  # send_action_goal return value
        self.goal_error = None  # exception to raise from send_action_goal
        self.cancel_result = (True, "Action goal cancelled successfully")
        self.cancel_error = None  # exception to raise from cancel_action
        self.feedback = None  # get_action_feedback return value
        self.feedback_ready = True  # add_action_feedback_listener return value
        self.feedback_listeners = {}  # action_name -> set of push listeners
        self.output_ready = True  # add_output_listener return value
        self.output_listeners = {}  # topic_name -> set of push listeners

    def srv_clients_inputs_dicts(self):
        return [{"name": "reset", "type": "Trigger", "fields": {}}]

    def action_clients_inputs_dicts(self):
        return [{"name": "navigate", "type": "NavigateToPose", "fields": {}}]

    def get_latest_output(self, name):
        return self.latest.get(name)

    def publish_data(self, data):
        if self.publish_error is not None:
            raise self.publish_error
        self.published.append(data)
        return 2

    def publish_audio(self, name, audio_b64):
        self.audio_published.append((name, audio_b64))
        return 1

    def send_srv_call(self, data):
        if self.service_error is not None:
            raise self.service_error
        return self.service_response

    def send_action_goal(self, data):
        if self.goal_error is not None:
            raise self.goal_error
        self.goals.append(data)
        return self.goal_accepted

    def cancel_action(self, name):
        if self.cancel_error is not None:
            raise self.cancel_error
        return self.cancel_result

    def get_action_feedback(self, name):
        return self.feedback

    def add_action_feedback_listener(self, name, listener):
        if not self.feedback_ready:
            return False
        self.feedback_listeners.setdefault(name, set()).add(listener)
        return True

    def remove_action_feedback_listener(self, name, listener):
        listeners = self.feedback_listeners.get(name)
        if listeners:
            listeners.discard(listener)

    def fire_action_feedback(self, name):
        """Simulate feedback arriving (invoke listeners, as the ROS thread would)."""
        for listener in list(self.feedback_listeners.get(name, ())):
            listener()

    def add_output_listener(self, name, listener):
        if not self.output_ready:
            return False
        self.output_listeners.setdefault(name, set()).add(listener)
        return True

    def remove_output_listener(self, name, listener):
        listeners = self.output_listeners.get(name)
        if listeners:
            listeners.discard(listener)

    def fire_output(self, name):
        """Simulate a message arriving on an output topic (invoke push listeners)."""
        for listener in list(self.output_listeners.get(name, ())):
            listener()


def _make_client(node):
    # The API is a standalone Starlette app -- no FastHTML needed to test it.
    pytest.importorskip("httpx")
    pytest.importorskip("starlette")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import build_api_app

    return TestClient(build_api_app(node))


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


# ---------------------------------------------------------------------------
# Phase 2: output streaming + latest value
# ---------------------------------------------------------------------------
def test_output_latest_ok():
    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}
    client = _make_client(node)
    resp = client.get("/api/outputs/odom/latest")
    assert resp.status_code == 200
    assert resp.json() == {"topic": "odom", "payload": {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}}


def test_output_latest_unknown_404():
    client = _make_client(_ApiNode())
    resp = client.get("/api/outputs/nope/latest")
    assert resp.status_code == 404


def test_output_latest_no_data_404():
    client = _make_client(_ApiNode())  # latest is empty
    resp = client.get("/api/outputs/odom/latest")
    assert resp.status_code == 404


def test_output_stream_ok():
    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/odom?rate=50") as ws:
        data = ws.receive_json()
    assert data == {"topic": "odom", "payload": {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}}


def test_output_stream_unknown_closes():
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/outputs/nope") as ws:
            ws.receive_json()


def test_output_stream_default_push_for_light_type():
    """A light type (Odometry) defaults to lossless event push -- no ?rate needed."""
    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0]}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/odom") as ws:
        # current value on connect
        assert ws.receive_json() == {
            "topic": "odom",
            "payload": {"frame_id": "odom", "data": [1.0]},
        }
        assert node.output_listeners.get("odom")  # push path registered a listener
        # a new message arrives -> pushed immediately (no poll wait)
        node.latest["odom"] = {"frame_id": "odom", "data": [2.0]}
        node.fire_output("odom")
        assert ws.receive_json() == {
            "topic": "odom",
            "payload": {"frame_id": "odom", "data": [2.0]},
        }


def test_output_stream_visual_type_samples_by_default():
    """A continuous/heavy type (OccupancyGrid) is rate-sampled by default (no push listener)."""
    node = _ApiNode()
    node.latest["map"] = {"frame_id": "map", "data": "AAAA"}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/map") as ws:
        assert ws.receive_json()["topic"] == "map"  # latest sent on connect
        assert not node.output_listeners.get("map")  # sampled path -> no listener


def test_output_stream_rate_overrides_to_sample():
    """?rate=<hz> forces sampling even for a push-by-default type (no push listener)."""
    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0]}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/odom?rate=50") as ws:
        assert ws.receive_json()["topic"] == "odom"
        assert not node.output_listeners.get("odom")  # sampled path -> no listener


def test_output_stream_push_not_ready_closes():
    from starlette.websockets import WebSocketDisconnect

    node = _ApiNode()
    node.output_ready = False  # no callback for this topic yet
    client = _make_client(node)
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/outputs/odom") as ws:  # default push
            ws.receive_json()


def test_interfaces_advertises_stream_mode():
    node = _ApiNode()
    client = _make_client(node)
    outputs = {o["name"]: o for o in client.get("/api/interfaces").json()["outputs"]}
    assert outputs["odom"]["stream"] == "WS /api/outputs/odom"
    assert outputs["odom"]["mode"] == "push"  # light type -> push default
    assert outputs["map"]["mode"] == "sampled"  # OccupancyGrid -> sampled default


def test_content_to_jsonable_passes_through_and_converts():
    pytest.importorskip("starlette")
    from sensor_msgs.msg import LaserScan

    from ros_sugar.ui_node.api import _content_to_jsonable

    # JSON-native content (from a specialized _get_ui_content) passes through
    assert _content_to_jsonable({"data": [1.0, 2.0]}) == {"data": [1.0, 2.0]}
    # A raw ROS message (unspecialized type) is faithfully converted
    scan = LaserScan()
    scan.ranges = [0.5]
    out = _content_to_jsonable(scan)
    assert out["ranges"] == [pytest.approx(0.5)]
    json.dumps(out)


# ---------------------------------------------------------------------------
# Phase 3: actions
# ---------------------------------------------------------------------------
def test_send_goal_accepted_202():
    node = _ApiNode()
    client = _make_client(node)
    resp = client.post("/api/actions/navigate", json={"x": 1.0})
    assert resp.status_code == 202
    assert resp.json() == {
        "accepted": True,
        "action": "navigate",
        "feedback": "/api/actions/navigate/feedback",
    }
    assert node.goals == [{"action_name": "navigate", "x": 1.0}]


def test_send_goal_unknown_404():
    client = _make_client(_ApiNode())
    resp = client.post("/api/actions/nope", json={})
    assert resp.status_code == 404


def test_send_goal_not_ready_503():
    node = _ApiNode()
    node.goal_error = RuntimeError("Action client 'navigate' is not ready")
    client = _make_client(node)
    resp = client.post("/api/actions/navigate", json={})
    assert resp.status_code == 503


def test_send_goal_rejected_502():
    node = _ApiNode()
    node.goal_accepted = False
    client = _make_client(node)
    resp = client.post("/api/actions/navigate", json={})
    assert resp.status_code == 502


def test_cancel_goal_ok():
    node = _ApiNode()
    node.cancel_result = (True, "Action goal cancelled successfully")
    client = _make_client(node)
    resp = client.post("/api/actions/navigate/cancel")
    assert resp.status_code == 200
    assert resp.json() == {
        "cancelled": True,
        "message": "Action goal cancelled successfully",
    }


def test_cancel_goal_unknown_404():
    client = _make_client(_ApiNode())
    resp = client.post("/api/actions/nope/cancel")
    assert resp.status_code == 404


def test_action_feedback_emits_current_state_on_connect():
    from geometry_msgs.msg import Point as ROSPoint

    node = _ApiNode()
    node.feedback = {
        "status": "running",
        "feedback": ROSPoint(x=1.0, y=2.0, z=3.0),  # a raw ROS message
        "timestep": 5,
        "feedback_timeout": False,
        "duration_secs": 2.0,
    }
    client = _make_client(node)
    with client.websocket_connect("/api/actions/navigate/feedback") as ws:
        data = ws.receive_json()
    assert data == {
        "status": "running",
        "feedback": {"x": 1.0, "y": 2.0, "z": 3.0},
        "timestep": 5,
        "duration_secs": 2.0,
    }


def test_action_feedback_pushed_on_arrival():
    """Each feedback is pushed the moment it arrives (event-driven, not polled)."""
    from starlette.websockets import WebSocketDisconnect

    node = _ApiNode()
    node.feedback = {
        "status": "running",
        "feedback": None,
        "timestep": 1,
        "feedback_timeout": False,
        "duration_secs": 0.5,
    }
    client = _make_client(node)
    with client.websocket_connect("/api/actions/navigate/feedback") as ws:
        assert ws.receive_json()["timestep"] == 1  # current state on connect

        # New feedback arrives -> push it.
        node.feedback = {
            "status": "running",
            "feedback": None,
            "timestep": 2,
            "feedback_timeout": False,
            "duration_secs": 1.0,
        }
        node.fire_action_feedback("navigate")
        assert ws.receive_json()["timestep"] == 2

        # Terminal feedback -> push, then the server closes the stream.
        node.feedback = {
            "status": "completed",
            "feedback": None,
            "timestep": 3,
            "feedback_timeout": False,
            "duration_secs": 1.5,
        }
        node.fire_action_feedback("navigate")
        assert ws.receive_json()["status"] == "completed"
        with pytest.raises(WebSocketDisconnect):
            ws.receive_json()


def test_action_feedback_not_ready_closes():
    from starlette.websockets import WebSocketDisconnect

    node = _ApiNode()
    node.feedback_ready = False  # action client not initialized yet
    client = _make_client(node)
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/actions/navigate/feedback") as ws:
            ws.receive_json()


def test_action_feedback_unknown_closes():
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/actions/nope/feedback") as ws:
            ws.receive_json()


# ---------------------------------------------------------------------------
# Phase 4: /api/world composable map stream
# ---------------------------------------------------------------------------
def test_interfaces_includes_worlds():
    doc = _make_client(_ApiNode()).get("/api/interfaces").json()
    worlds = {w["name"]: w for w in doc["worlds"]}
    assert "map" in worlds
    assert worlds["map"]["stream"] == "WS /api/world/map"
    overlay_names = {o["name"] for o in worlds["map"]["overlays"]}
    assert {"odom", "plan"} <= overlay_names


def test_world_stream_emits_grid_and_markers():
    node = _ApiNode()
    # The grid's UI content is the flat map metadata + data the callback now
    # produces; the world wraps it in a publish op.
    node.latest["map"] = {
        "frame_id": "map",
        "resolution": 0.5,
        "width": 2,
        "height": 2,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "origin_yaw": 0.0,
        "data": "AAECAw==",
    }
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0, 2.0, 0.0, 0.5, 0.0]}
    node.latest["plan"] = {"frame_id": "map", "data": [0.0, 0.0, 1.0, 1.0]}

    client = _make_client(node)
    with client.websocket_connect("/api/world/map") as ws:
        frames = [ws.receive_json() for _ in range(3)]

    by_op = {}
    for frame in frames:
        by_op.setdefault(frame["op"], []).append(frame)

    grid_msg = by_op["publish"][0]["msg"]
    assert grid_msg["width"] == 2
    assert grid_msg["data"] == "AAECAw=="  # raw occupancy data, never a JPEG

    overlays = {m["id"]: m for m in by_op.get("overlay", [])}
    paths = {m["id"]: m for m in by_op.get("path", [])}
    assert overlays["odom"]["x"] == 1.0 and overlays["odom"]["theta"] == 0.5
    assert paths["plan"]["points"] == [0.0, 0.0, 1.0, 1.0]


def test_occupancy_grid_ui_content_is_raw_grid_not_jpeg():
    import array as _array
    import base64

    from nav_msgs.msg import OccupancyGrid

    from ros_sugar.io.callbacks import OccupancyGridCallback

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 2
    grid.info.height = 2
    grid.info.resolution = 0.5
    grid.data = [0, 100, -1, 0]

    cb = OccupancyGridCallback(Topic(name="map", msg_type="OccupancyGrid"))
    cb.callback(grid)  # sets msg + frame_id (from header), as in production
    content = cb._get_ui_content()

    assert content["frame_id"] == "map"
    assert content["width"] == 2
    assert content["resolution"] == 0.5
    # data is the raw int8 grid, base64-encoded (not a rendered image)
    assert list(_array.array("b", base64.b64decode(content["data"]))) == [0, 100, -1, 0]
    json.dumps(content)  # JSON-safe


def test_world_unknown_closes():
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    # 'odom' is an Odometry output, not an occupancy grid -> reject.
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/world/odom") as ws:
            ws.receive_json()


# ---------------------------------------------------------------------------
# Phase 5: audio input WS
# ---------------------------------------------------------------------------
def test_interfaces_advertises_audio_stream():
    doc = _make_client(_ApiNode()).get("/api/interfaces").json()
    inputs = {i["name"]: i for i in doc["inputs"]}
    assert inputs["speech"]["audio_stream"] == "WS /api/inputs/speech/audio"
    assert "audio_stream" not in inputs["cmd_vel"]  # non-audio inputs don't get it


def test_audio_input_stream_publishes():
    node = _ApiNode()
    client = _make_client(node)
    with client.websocket_connect("/api/inputs/speech/audio") as ws:
        ws.send_json({"payload": "QUJD"})  # base64 for "ABC"
        ack = ws.receive_json()
    assert ack == {"published": "speech"}
    assert node.audio_published == [("speech", "QUJD")]


def test_audio_input_unknown_closes():
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    # 'cmd_vel' is a Twist input, not Audio -> reject.
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/inputs/cmd_vel/audio") as ws:
            ws.receive_json()
