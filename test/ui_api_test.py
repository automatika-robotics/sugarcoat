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
        self.service_calls = []  # recorded service call data
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
        from std_srvs.srv import Trigger

        return [
            {"name": "reset", "type": "Trigger", "fields": {}, "request_class": Trigger.Request}
        ]

    def action_clients_inputs_dicts(self):
        # Point stands in for a goal message with an 'x' field
        from geometry_msgs.msg import Point

        return [
            {"name": "navigate", "type": "NavigateToPose", "fields": {}, "goal_class": Point}
        ]

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
        self.service_calls.append(data)
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
    resp = client.post("/api/inputs/cmd_vel", json={"linear": {"x": 1.0}})
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

    from ros_sugar.ui_node.api_utils import msg_to_jsonable

    scan = LaserScan()
    scan.ranges = [0.1, 0.2, 0.3]  # float32[] -> array.array internally
    out = msg_to_jsonable(scan)
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
    """An undeclared name is accepted, then closed with 1008 and a reason, so a
    client learns why instead of seeing a bare HTTP 403 on the handshake."""
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    with client.websocket_connect("/api/outputs/nope") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Unknown output topic")


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


def test_sampled_stream_does_not_resend_unchanged_content():
    """Sampling faster than the source must not repeat the same frame"""
    import time

    node = _ApiNode()
    node.latest["map"] = {"frame_id": "map", "data": "AAAA"}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/map?rate=30") as ws:
        assert ws.receive_json()["payload"]["data"] == "AAAA"
        time.sleep(0.2)  # several ticks with no new message
        node.latest["map"] = {"frame_id": "map", "data": "BBBB"}
        # The next frame is the new message, not a repeat of the first
        assert ws.receive_json()["payload"]["data"] == "BBBB"


def test_push_stream_sends_every_message():
    """A push stream sends each message, even one carrying the same value"""
    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0]}
    client = _make_client(node)
    with client.websocket_connect("/api/outputs/odom") as ws:
        first = ws.receive_json()
        node.fire_output("odom")  # a new message with the same content
        assert ws.receive_json() == first


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


def test_namespaced_topic_names_are_addressable():
    """Topics with interior slashes (ns/topic) resolve through the {name:path}
    routes; a leading slash in the URL is normalized like Topic names."""
    node = _ApiNode()
    node.out_topics.append(Topic(name="/ns/cmd", msg_type="String"))  # -> "ns/cmd"
    node.in_topics.append(Topic(name="ns/status", msg_type="String"))
    client = _make_client(node)

    resp = client.post("/api/inputs/ns/cmd", json={"data": "hi"})
    assert resp.status_code == 200
    assert node.published[-1]["topic_name"] == "ns/cmd"

    # A client that naively prepends the ROS-style leading slash still resolves
    resp = client.post("/api/inputs//ns/cmd", json={"data": "again"})
    assert resp.status_code == 200
    assert node.published[-1]["topic_name"] == "ns/cmd"

    node.latest["ns/status"] = "ok"
    resp = client.get("/api/outputs/ns/status/latest")
    assert resp.status_code == 200
    assert resp.json() == {"topic": "ns/status", "payload": "ok"}
    with client.websocket_connect("/api/outputs/ns/status?rate=50") as ws:
        assert ws.receive_json()["topic"] == "ns/status"


def test_action_cancel_not_swallowed_by_goal_route():
    """/actions/{name:path} is greedy: the cancel route must win for .../cancel."""
    node = _ApiNode()
    client = _make_client(node)
    resp = client.post("/api/actions/navigate/cancel")
    assert resp.status_code == 200
    assert resp.json()["cancelled"] is True
    assert node.goals == []  # the goal handler never ran


def test_body_cannot_redirect_to_another_interface():
    """The URL names the interface. A body key naming another input, service
    or action is not a message field, so the call is refused and nothing is
    sent to either interface."""
    node = _ApiNode()
    client = _make_client(node)

    resp = client.post("/api/inputs/cmd_vel", json={"topic_name": "speech", "linear": {"x": 1.0}})
    assert resp.status_code == 400
    assert client.post("/api/services/reset", json={"srv_name": "other"}).status_code == 400
    resp = client.post("/api/actions/navigate", json={"action_name": "other", "x": 1.0})
    assert resp.status_code == 400

    assert node.published == [] and node.service_calls == [] and node.goals == []


def test_unknown_fields_are_rejected():
    """A misspelt field must not be dropped silently, which would send that
    part of the message at its default value. The error names the field and
    the ones that exist, at any nesting depth."""
    node = _ApiNode()
    client = _make_client(node)

    resp = client.post("/api/inputs/cmd_vel", json={"linear": {"x": 1.0, "q": 2.0}})
    assert resp.status_code == 400
    assert "'q'" in resp.json()["error"] and "x, y, z" in resp.json()["error"]
    assert client.post("/api/services/reset", json={"bogus": 1}).status_code == 400
    assert client.post("/api/actions/navigate", json={"yaw": 1.0}).status_code == 400

    assert node.published == [] and node.service_calls == [] and node.goals == []


def test_misspelt_fields_inside_lists_of_messages_are_found():
    """A waypoint in a list is a message too. A misspelt field in one would
    leave that pose at zero"""
    from nav_msgs.msg import Path

    from ros_sugar.io.supported_types import validate_msg_fields

    pose = {"pose": {"position": {"x": 1.0}}}
    validate_msg_fields(Path, {"poses": [pose, pose]})

    misspelt = {"pose": {"positon": {"x": 1.0}}}
    with pytest.raises(
        ValueError,
        match=r"'poses\[1\]\.pose' has no field 'positon'\. It has: orientation, position",
    ):
        validate_msg_fields(Path, {"poses": [pose, misspelt]})


def test_a_goal_with_a_misspelt_waypoint_field_is_rejected():
    from nav_msgs.msg import Path

    node = _ApiNode()
    # A goal holding a list of poses, like a mission's waypoints
    node.action_clients_inputs_dicts = lambda: [
        {"name": "navigate", "type": "Path", "fields": {}, "goal_class": Path}
    ]
    client = _make_client(node)

    resp = client.post(
        "/api/actions/navigate", json={"poses": [{"pose": {"positon": {"x": 1.0}}}]}
    )

    assert resp.status_code == 400
    assert "'poses[0].pose' has no field 'positon'" in resp.json()["error"]
    assert node.goals == []


def test_malformed_json_body_is_rejected():
    """A body that is not valid JSON must not reach ROS as an empty request,
    which would publish a default message (e.g. a goal at the origin)."""
    node = _ApiNode()
    client = _make_client(node)
    malformed = {"content": b"{x: 9", "headers": {"content-type": "application/json"}}

    for url in ("/api/inputs/cmd_vel", "/api/services/reset", "/api/actions/navigate"):
        resp = client.post(url, **malformed)
        assert resp.status_code == 400, url
        assert "JSON object" in resp.json()["error"]

    assert node.published == [] and node.service_calls == [] and node.goals == []


def test_empty_body_is_an_empty_request():
    """A body-less call, such as a Trigger service, is still sent"""
    node = _ApiNode()
    client = _make_client(node)
    client.post("/api/services/reset")
    assert node.service_calls == [{"srv_name": "reset"}]


def test_set_ros_msg_from_dict_converts_by_declared_type():
    from std_msgs.msg import ByteMultiArray, Float32MultiArray, String as ROSString

    from ros_sugar.io.supported_types import set_ros_msg_from_dict

    # octet sequences become length-1 bytes elements (native-serializable)
    msg = set_ros_msg_from_dict(ByteMultiArray, {"data": [1, 2, 255]})
    assert msg.data == [b"\x01", b"\x02", b"\xff"]
    # numeric sequences coerce per element
    msg = set_ros_msg_from_dict(Float32MultiArray, {"data": [1, "2.5"]})
    assert msg.data == pytest.approx([1.0, 2.5])
    # scalar strings still coerce leniently
    assert set_ros_msg_from_dict(ROSString, {"data": 42}).data == "42"
    # nested complex sequences ('sequence<pkg/Type>') recurse correctly
    msg = set_ros_msg_from_dict(
        ByteMultiArray,
        {"layout": {"dim": [{"label": "x", "size": 1, "stride": 1}], "data_offset": 0}, "data": [7]},
    )
    assert msg.layout.dim[0].label == "x" and msg.layout.dim[0].size == 1


def test_boolean_fields_take_only_unambiguous_values():
    """bool() is True for any non-empty string, so 'false' used to publish true"""
    from std_msgs.msg import Bool

    from ros_sugar.io.supported_types import set_ros_msg_from_dict

    for value, expected in (
        (True, True), (False, False), ("true", True), ("False", False), (1, True), (0, False)
    ):
        assert set_ros_msg_from_dict(Bool, {"data": value}).data is expected, value
    for value in ("yes", "0", "", 2, 0.5, [1], None):
        with pytest.raises(ValueError, match="data"):
            set_ros_msg_from_dict(Bool, {"data": value})


def test_integer_fields_take_only_whole_numbers_in_range():
    """int() truncated 2.9 to 2 and took True as 1, and an out of range value
    wrapped when serialized: 300 as a uint8 arrived as 44"""
    import numpy as np
    from std_msgs.msg import Int32, UInt8

    from ros_sugar.io.supported_types import set_ros_msg_from_dict

    for value, expected in (
        (3, 3), (-3, -3), (3.0, 3), ("3", 3), (" 3.0 ", 3), (np.int64(3), 3)
    ):
        assert set_ros_msg_from_dict(Int32, {"data": value}).data == expected, value
    for value in (2.9, True, "2.9", "abc", None, float("nan"), 1e20):
        with pytest.raises(ValueError, match="data"):
            set_ros_msg_from_dict(Int32, {"data": value})

    assert set_ros_msg_from_dict(UInt8, {"data": 255}).data == 255
    for value in (300, -1):
        with pytest.raises(ValueError, match=r"in \[0, 255\]"):
            set_ros_msg_from_dict(UInt8, {"data": value})


def test_float_fields_refuse_booleans():
    """float() took True as 1.0"""
    from std_msgs.msg import Float64

    from ros_sugar.io.supported_types import set_ros_msg_from_dict

    for value, expected in ((2, 2.0), (2.5, 2.5), ("2.5", 2.5)):
        assert set_ros_msg_from_dict(Float64, {"data": value}).data == expected
    for value in (True, "abc", None):
        with pytest.raises(ValueError, match="data"):
            set_ros_msg_from_dict(Float64, {"data": value})


def test_set_ros_msg_from_dict_rejects_native_fatal_values():
    """Values that would build a Python-plausible but rmw-fatal message must
    raise ValueError (-> HTTP 400) instead of reaching the serializer."""
    from std_msgs.msg import ByteMultiArray, Float32MultiArray

    from ros_sugar.io.supported_types import set_ros_msg_from_dict

    # a string is NOT a valid octet sequence (this exact payload used to
    # crash the UI node process in the C typesupport layer)
    with pytest.raises(ValueError, match="data"):
        set_ros_msg_from_dict(ByteMultiArray, {"data": "zzz"})
    # a scalar is not a sequence
    with pytest.raises(ValueError, match="data"):
        set_ros_msg_from_dict(ByteMultiArray, {"data": 42})
    # unconvertible element
    with pytest.raises(ValueError, match="data"):
        set_ros_msg_from_dict(Float32MultiArray, {"data": ["abc"]})


def test_content_to_jsonable_passes_through_and_converts():
    pytest.importorskip("starlette")
    from sensor_msgs.msg import LaserScan

    from ros_sugar.ui_node.api_utils import content_to_jsonable

    # JSON-native content (from a specialized _get_ui_content) passes through
    assert content_to_jsonable({"data": [1.0, 2.0]}) == {"data": [1.0, 2.0]}
    # A raw ROS message (unspecialized type) is faithfully converted
    scan = LaserScan()
    scan.ranges = [0.5]
    out = content_to_jsonable(scan)
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


def test_send_goal_while_one_runs_409():
    from ros_sugar.ui_node.utils import GoalInProgressError

    node = _ApiNode()
    node.goal_error = GoalInProgressError("Action 'navigate' is still running a goal")
    client = _make_client(node)
    resp = client.post("/api/actions/navigate", json={})
    assert resp.status_code == 409
    assert "still running" in resp.json()["error"]


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
        "feedback_timeout": False,
        "result": None,
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

        # Terminal feedback -> push it with the goal's result, then the server
        # closes the stream.
        from geometry_msgs.msg import Point as ROSPoint

        node.feedback = {
            "status": "completed",
            "feedback": None,
            "timestep": 3,
            "feedback_timeout": False,
            "duration_secs": 1.5,
            "result": ROSPoint(x=5.0, y=6.0, z=0.0),  # a raw ROS message
        }
        node.fire_action_feedback("navigate")
        terminal = ws.receive_json()
        assert terminal["status"] == "completed"
        assert terminal["result"] == {"x": 5.0, "y": 6.0, "z": 0.0}
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
    """An undeclared name is accepted, then closed with 1008 and a reason, so a
    client learns why instead of seeing a bare HTTP 403 on the handshake."""
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    with client.websocket_connect("/api/actions/nope/feedback") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Unknown action")


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
    """An undeclared name is accepted, then closed with 1008 and a reason, so a
    client learns why instead of seeing a bare HTTP 403 on the handshake."""
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    # 'odom' is an Odometry output, not an occupancy grid
    with client.websocket_connect("/api/world/odom") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Not a declared OccupancyGrid output")


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
    """An undeclared name is accepted, then closed with 1008 and a reason, so a
    client learns why instead of seeing a bare HTTP 403 on the handshake."""
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_ApiNode())
    # 'cmd_vel' is a Twist input, not Audio
    with client.websocket_connect("/api/inputs/cmd_vel/audio") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Not a declared Audio input")


def test_a_command_from_another_site_is_refused():
    """A page on another site can make a browser post plain text here without
    asking first. Clients without an Origin, and pages served by the robot
    itself or through a proxy, are unaffected."""
    node = _ApiNode()
    client = _make_client(node)

    foreign = client.post(
        "/api/inputs/cmd_vel",
        content=b'{"linear": {"x": 1.0}}',
        headers={"Content-Type": "text/plain", "Origin": "http://evil.example"},
    )
    assert foreign.status_code == 403
    assert not node.published

    for headers in (
        {},
        {"Origin": "http://testserver"},
        {"Origin": "https://robot.example", "X-Forwarded-Host": "robot.example"},
    ):
        resp = client.post("/api/inputs/cmd_vel", json={"linear": {"x": 1.0}}, headers=headers)
        assert resp.status_code == 200, headers
    assert len(node.published) == 3


def test_a_command_stream_from_another_site_is_refused():
    """Browsers open WebSockets to any site, so a stream that publishes is
    refused for another site's page. A stream that only sends data out is not."""
    from starlette.websockets import WebSocketDisconnect

    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}
    client = _make_client(node)
    foreign = {"Origin": "http://evil.example"}

    with client.websocket_connect("/api/inputs/speech/audio", headers=foreign) as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            # Without the refusal this is published and acknowledged
            ws.send_json({"payload": "QUJD"})
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (
        1008,
        "Cross-origin connections are not allowed",
    )
    assert not node.audio_published

    with client.websocket_connect("/api/outputs/odom?rate=50", headers=foreign) as ws:
        assert ws.receive_json()["topic"] == "odom"


# ---------------------------------------------------------------------------
# UI node lifecycle
# ---------------------------------------------------------------------------
@pytest.fixture
def ui_node():
    """A real, activated UI node with one service and one action client, neither
    of which has a server running."""
    import rclpy
    from std_srvs.srv import Trigger
    from tf2_msgs.action import LookupTransform

    from ros_sugar.base_clients import ActionClientConfig, ServiceClientConfig
    from ros_sugar.ui_node.ui_node import UINode, UINodeConfig

    if not rclpy.ok():
        rclpy.init()
    node = UINode(
        config=UINodeConfig(),
        inputs=[
            ServiceClientConfig(srv_type=Trigger, name="ui_node_test/reset"),
            ActionClientConfig(action_type=LookupTransform, name="ui_node_test/lookup"),
        ],
    )
    node.rclpy_init_node()
    node.custom_on_activate()
    try:
        yield node
    finally:
        node.destroy_node()


def test_enable_ui_rejects_an_output_type_without_a_callback(monkeypatch):
    """The UI node could not read such an output, so the recipe fails at once
    with a clear error instead of the UI failing to start"""
    from ros_sugar import Launcher
    from ros_sugar.io.supported_types import String

    monkeypatch.setattr(String, "callback", None)  # a type without a callback

    with pytest.raises(TypeError, match="'tracks' has type 'String'"):
        Launcher().enable_ui(outputs=[Topic(name="tracks", msg_type="String")])


def test_slow_output_content_does_not_block_other_requests():
    """Computing an output's content (e.g. a JPEG encode) must not hold up
    the rest of the server, for a latest read or a stream"""
    import threading
    import time

    node = _ApiNode()

    def _slow_latest(name):
        time.sleep(1.0)
        return {"data": 1}

    node.get_latest_output = _slow_latest

    def _health_secs(client):
        start = time.monotonic()
        assert client.get("/api/health").status_code == 200
        return time.monotonic() - start

    with _make_client(node) as client:  # one event loop for all requests
        reader = threading.Thread(
            target=client.get, args=("/api/outputs/map/latest",), daemon=True
        )
        reader.start()
        time.sleep(0.2)
        assert _health_secs(client) < 0.5
        reader.join()

        with client.websocket_connect("/api/outputs/map"):  # a sampled stream
            time.sleep(0.2)
            assert _health_secs(client) < 0.5


def test_action_duration_has_fractions_of_a_second(ui_node):
    """A goal running for part of a second reports that part, not 0"""
    import time
    from unittest.mock import MagicMock

    handler = ui_node._ros_action_clients["ui_node_test/lookup"]
    handler.client.wait_for_server = MagicMock(return_value=True)

    def _accept(goal, feedback_callback):
        handler.goal_accepted = True
        return MagicMock()

    handler.client.send_goal_async = _accept
    assert handler.send_request(handler.config.action_type.Goal())
    time.sleep(0.3)

    assert 0.3 <= handler.get_ui_elements()["duration_secs"] < 1.0
    handler.reset()


def test_ui_node_deactivates_with_service_and_action_clients(ui_node):
    """Deactivating releases the node's service and action clients, and the
    API then reports them as not ready until the node is activated again."""
    service_client = ui_node._ros_service_clients["ui_node_test/reset"].client
    action_client = ui_node._ros_action_clients["ui_node_test/lookup"].client

    ui_node.custom_on_deactivate()

    assert service_client not in list(ui_node.clients)
    assert action_client not in list(ui_node.waitables)
    with pytest.raises(RuntimeError, match="not ready"):
        ui_node.send_srv_call({"srv_name": "ui_node_test/reset"})
    with pytest.raises(RuntimeError, match="not ready"):
        ui_node.send_action_goal({"action_name": "ui_node_test/lookup"})


def test_a_real_ui_node_reports_values_its_fields_cannot_hold():
    """The UI node builds the messages, so a value a field cannot hold reaches the
    client as 400 naming it, for a topic, a service and an action alike. The
    clients used to log that error and return nothing, which the API reported as
    a missing response or a rejected goal"""
    from unittest.mock import MagicMock

    import rclpy
    from starlette.testclient import TestClient
    from std_srvs.srv import SetBool
    from tf2_msgs.action import LookupTransform

    from ros_sugar.base_clients import ActionClientConfig, ServiceClientConfig
    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.ui_node import UINode, UINodeConfig

    if not rclpy.ok():
        rclpy.init()
    node = UINode(
        config=UINodeConfig(),
        inputs=[
            Topic(name="ui_node_test/flag", msg_type="Bool"),
            ServiceClientConfig(srv_type=SetBool, name="ui_node_test/set_flag"),
            ActionClientConfig(action_type=LookupTransform, name="ui_node_test/lookup"),
        ],
        outputs=[Topic(name="ui_node_test/status", msg_type="String")],
    )
    node.rclpy_init_node()
    node.create_all_publishers()
    node.custom_on_activate()
    service = node._ros_service_clients["ui_node_test/set_flag"]
    service.client.wait_for_service = MagicMock(return_value=True)
    service.send_request = MagicMock(return_value=SetBool.Response(success=True))
    action = node._ros_action_clients["ui_node_test/lookup"]
    action.client.wait_for_server = MagicMock(return_value=True)
    action.send_request = MagicMock(return_value=True)
    try:
        client = TestClient(build_api_app(node))
        for url, body, reason in (
            ("/api/inputs/ui_node_test/flag", {"data": "yes"}, "expected true or false"),
            ("/api/services/ui_node_test/set_flag", {"data": "yes"}, "expected true or false"),
            ("/api/actions/ui_node_test/lookup", {"timeout": {"sec": 2.5}}, "expected a whole number"),
        ):
            resp = client.post(url, json=body)
            assert resp.status_code == 400, url
            assert reason in resp.json()["error"], url
        service.send_request.assert_not_called()
        action.send_request.assert_not_called()

        for url, body, status in (
            ("/api/inputs/ui_node_test/flag", {"data": "false"}, 200),
            ("/api/services/ui_node_test/set_flag", {"data": True}, 200),
            ("/api/actions/ui_node_test/lookup", {"timeout": {"sec": 2}}, 202),
        ):
            assert client.post(url, json=body).status_code == status, url
    finally:
        node.destroy_node()


def test_a_second_goal_leaves_the_running_goal_alone(ui_node):
    """While a goal runs, another is refused before it can clear the running
    goal's state. Once the goal returns, or its feedback times out, goals are
    accepted again"""
    from unittest.mock import MagicMock

    from ros_sugar.ui_node.utils import GoalInProgressError

    name = "ui_node_test/lookup"
    handler = ui_node._ros_action_clients[name]
    handler.client.wait_for_server = MagicMock(return_value=True)
    handler.send_request_from_dict = MagicMock(return_value=True)
    handler.goal_accepted = True  # a running goal

    with pytest.raises(GoalInProgressError):
        ui_node.send_action_goal({"action_name": name})
    handler.send_request_from_dict.assert_not_called()
    assert handler.goal_accepted

    handler._feedback_timeout = True  # its server went quiet
    assert ui_node.send_action_goal({"action_name": name})
    handler._feedback_timeout = False
    handler.action_returned = True  # it finished
    assert ui_node.send_action_goal({"action_name": name})


def test_missing_server_is_reported_at_once(ui_node):
    """With no server behind a declared client, the call fails after a brief
    wait for discovery instead of holding the request for the client's full
    timeout (30 s by default)."""
    import time

    calls = (
        (ui_node.send_srv_call, {"srv_name": "ui_node_test/reset"}),
        (ui_node.send_action_goal, {"action_name": "ui_node_test/lookup"}),
    )
    for call, data in calls:
        start = time.monotonic()
        with pytest.raises(RuntimeError, match="not available"):
            call(data)
        assert time.monotonic() - start < 5.0


# ---------------------------------------------------------------------------
# UI server TLS
# ---------------------------------------------------------------------------
def test_a_certificate_is_minted_once_and_kept(tmp_path, monkeypatch):
    """Without an environment certificate, Sugarcoat mints its own,
    readable by the owner only, and reuses it so pinned clients keep working"""
    import stat

    from ros_sugar.ui_node import security

    monkeypatch.setenv(security.DATA_DIR_ENV, str(tmp_path / "ui"))
    monkeypatch.delenv(security.TLS_CERT_ENV, raising=False)
    monkeypatch.delenv(security.TLS_KEY_ENV, raising=False)

    first = security.resolve_certificate()

    assert first.source == "minted"
    assert stat.S_IMODE((tmp_path / "ui").stat().st_mode) == 0o700
    assert stat.S_IMODE(first.key.stat().st_mode) == 0o600
    assert {"127.0.0.1", "::1"} <= set(first.addresses)
    assert security.uncovered_addresses(first) == []
    assert len(first.fingerprint.split(":")) == 32
    assert f"Fingerprint (SHA-256): {first.fingerprint}" in security.banner(first)

    again = security.resolve_certificate()

    assert (again.source, again.fingerprint) == ("stored", first.fingerprint)
    assert security.banner(again) == []


def test_a_certificate_near_expiry_is_renewed(tmp_path):
    import datetime

    from ros_sugar.ui_node import security

    security._mint(tmp_path / "tls.crt", tmp_path / "tls.key", datetime.timedelta(days=10))
    expiring = security.load_certificate(tmp_path / "tls.crt", tmp_path / "tls.key", "minted")

    renewed = security.minted_certificate(tmp_path)

    assert renewed.fingerprint != expiring.fingerprint
    assert renewed.expires - expiring.expires > datetime.timedelta(days=600)


def test_an_environment_certificate_comes_first(tmp_path, monkeypatch):
    from ros_sugar.ui_node import security

    monkeypatch.setenv(security.DATA_DIR_ENV, str(tmp_path / "ui"))
    emos = security.minted_certificate(tmp_path / "emos")
    other = security.minted_certificate(tmp_path / "other")

    monkeypatch.setenv(security.TLS_CERT_ENV, str(emos.certificate))
    monkeypatch.setenv(security.TLS_KEY_ENV, str(emos.key))
    chosen = security.resolve_certificate()
    assert (chosen.source, chosen.fingerprint) == ("environment", emos.fingerprint)

    # A configured certificate that cannot be used is an error, not a fallback
    monkeypatch.setenv(security.TLS_KEY_ENV, str(other.key))
    with pytest.raises(security.CertificateError, match="cannot be used"):
        security.resolve_certificate()
    monkeypatch.setenv(security.TLS_CERT_ENV, str(tmp_path / "missing.crt"))
    with pytest.raises(security.CertificateError, match="Cannot read"):
        security.resolve_certificate()
    monkeypatch.delenv(security.TLS_KEY_ENV)
    with pytest.raises(security.CertificateError, match="set together"):
        security.resolve_certificate()


def test_the_server_speaks_only_modern_tls(tmp_path):
    """TLS 1.2 or later, TLS 1.2 ciphers with forward secrecy and authenticated
    encryption only, and a client that pins the certificate connects"""
    import socket
    import ssl
    import threading
    import time
    import urllib.request

    import uvicorn
    from starlette.applications import Starlette
    from starlette.responses import PlainTextResponse
    from starlette.routing import Route

    from ros_sugar.ui_node import security
    from ros_sugar.ui_node.api_utils import server_config

    certificate = security.minted_certificate(tmp_path)
    app = Starlette(routes=[Route("/", lambda request: PlainTextResponse("ok"))])
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = server_config(app, port, certificate)
    assert config.ssl.minimum_version == ssl.TLSVersion.TLSv1_2
    for cipher in config.ssl.get_ciphers():
        if cipher["protocol"] == "TLSv1.2":
            assert cipher["kea"] == "kx-ecdhe", cipher["name"]
            assert "GCM" in cipher["name"] or "CHACHA20" in cipher["name"], cipher["name"]
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 10
        while not server.started and time.time() < deadline:
            time.sleep(0.05)

        pinned = ssl.create_default_context(cafile=str(certificate.certificate))
        with urllib.request.urlopen(f"https://localhost:{port}/", context=pinned) as resp:
            assert resp.read() == b"ok"
        with socket.create_connection(("localhost", port)) as raw:
            with pinned.wrap_socket(raw, server_hostname="localhost") as tls:
                assert tls.version() in ("TLSv1.2", "TLSv1.3")

    finally:
        server.should_exit = True
        thread.join(timeout=10)


def test_enable_ui_is_secure_by_default():
    from ros_sugar import Launcher

    launcher = Launcher()
    launcher.enable_ui(serve_browser=False)
    assert launcher._ui_node_config.secure is True

    launcher.enable_ui(serve_browser=False, secure=False)
    assert launcher._ui_node_config.secure is False
