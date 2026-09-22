"""Tests for the front-end-agnostic UI API (issue #54), Phase 0.

Covers:
* the ``_get_ui_content`` source fix -- specialized callbacks must return
  JSON-serializable content (no numpy arrays).
* ``ros_sugar.ui_node.api.build_interfaces`` -- the discovery document.
* the optional-dependency import boundary -- importing the main package must
  not pull in the web stack (fasthtml/starlette/uvicorn).
"""

import datetime
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

    def __init__(self, in_topics, out_topics, srv=None, act=None, routines=None):
        self.in_topics = in_topics  # API outputs (robot -> client)
        self.out_topics = out_topics  # API inputs (client -> robot)
        self._srv = srv or []
        self._act = act or []
        self._routines = routines or []
        self.config = _FakeConfig()

    def srv_clients_inputs_dicts(self):
        return self._srv

    def action_clients_inputs_dicts(self):
        return self._act

    def routine_names(self):
        return list(self._routines)


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
        self.logged = []  # (level, message) logged through get_logger
        self.routines = []  # routine_names result
        self.routine_states = {}  # routine name -> get_routine_state result
        self.routine_listeners = {}  # routine name -> set of push listeners
        self.routine_result = (True, "done")  # control_routine return value
        self.routine_error = None  # exception to raise from control_routine
        self.routine_calls = []  # (name, command, reason) given to control_routine

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

    def routine_names(self):
        return list(self.routines)

    def get_routine_state(self, name):
        return self.routine_states.get(name)

    def add_routine_listener(self, name, listener):
        if name not in self.routines:
            return False
        self.routine_listeners.setdefault(name, set()).add(listener)
        return True

    def remove_routine_listener(self, name, listener):
        self.routine_listeners.get(name, set()).discard(listener)

    def control_routine(self, name, command, reason=None):
        self.routine_calls.append((name, command, reason))
        if self.routine_error is not None:
            raise self.routine_error
        return self.routine_result

    def set_routine_state(self, routine, state):
        """Simulate the Monitor publishing a new state for a routine"""
        self.routine_states[routine] = state
        for listener in list(self.routine_listeners.get(routine, ())):
            listener()

    def get_logger(self):
        node = self

        class _Logger:
            def info(self, message):
                node.logged.append(("info", message))

            def warning(self, message):
                node.logged.append(("warning", message))

        return _Logger()


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
def ui_node(request):
    """A real, activated UI node with one service and one action client, neither
    of which has a server running. Named after the test, since a lifecycle node
    stays in the graph once destroyed"""
    import rclpy
    from std_srvs.srv import Trigger
    from tf2_msgs.action import LookupTransform

    from ros_sugar.base_clients import ActionClientConfig, ServiceClientConfig
    from ros_sugar.ui_node.ui_node import UINode, UINodeConfig

    if not rclpy.ok():
        rclpy.init()
    node = UINode(
        component_name=f"ui_{request.node.name}",
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
    the rest of the server, for a latest read or a stream.

    The content is held until the test lets it go, and another request has to
    be answered in the meantime. Timing that request instead failed whenever
    the machine was busy enough to slow it down on its own
    """
    import threading

    node = _ApiNode()
    entered = threading.Event()
    release = threading.Event()

    def _held_latest(name):
        entered.set()
        release.wait(10.0)
        return {"data": 1}

    node.get_latest_output = _held_latest

    def _answered_meanwhile(client) -> bool:
        """Whether /api/health answers while the content is still held"""
        answers = []
        asker = threading.Thread(
            target=lambda: answers.append(client.get("/api/health").status_code),
            daemon=True,
        )
        asker.start()
        asker.join(5.0)
        return answers == [200]

    with _make_client(node) as client:  # one event loop for all requests
        reader = threading.Thread(
            target=client.get, args=("/api/outputs/map/latest",), daemon=True
        )
        reader.start()
        assert entered.wait(5.0), "the latest read never got to the content"
        try:
            assert _answered_meanwhile(client), "a latest read held up the server"
        finally:
            release.set()
            reader.join()

        entered.clear()
        release.clear()
        with client.websocket_connect("/api/outputs/map"):  # a sampled stream
            assert entered.wait(5.0), "the stream never got to the content"
            try:
                assert _answered_meanwhile(client), "a stream held up the server"
            finally:
                release.set()


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
    started = time.monotonic()
    assert handler.send_request(handler.config.action_type.Goal())
    time.sleep(0.3)

    duration = handler.get_ui_elements()["duration_secs"]
    elapsed = time.monotonic() - started
    # Between the sleep and the time that really went by, however busy the
    # machine was, and not rounded to whole seconds. The small margin covers
    # the node's clock and this one being read a moment apart
    assert 0.3 <= duration <= elapsed + 0.05
    assert duration != round(duration)
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
        component_name="ui_field_errors",
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


# ---------------------------------------------------------------------------
# API keys
# ---------------------------------------------------------------------------
def test_api_keys_are_stored_hashed_and_checked(tmp_path):
    import stat

    from ros_sugar.ui_node.security import ApiKeyError, ApiKeys

    keys = ApiKeys(tmp_path)
    stored, key = keys.create("mission-control", ["command", "read"])

    assert key.startswith("sk_ui_")
    assert key not in keys.path.read_text()
    assert stat.S_IMODE(keys.path.stat().st_mode) == 0o600
    assert stored.scopes == ("read", "command")
    assert ApiKeys(tmp_path).find(key) == stored
    assert keys.find(key + "x") is None
    assert keys.find("") is None

    for name, scopes, expires_at, error in (
        ("mission-control", ["read"], None, "already exists"),
        ("viewer", ["write"], None, "Scopes must be"),
        ("viewer", [], None, "Scopes must be"),
        ("viewer", ["read"], datetime.date(2020, 1, 1), "has passed"),
        (" ", ["read"], None, "needs a name"),
    ):
        with pytest.raises(ApiKeyError, match=error):
            keys.create(name, scopes, expires_at)


def test_a_key_is_valid_through_its_expiry_date(tmp_path):
    import attrs

    from ros_sugar.ui_node.security import ApiKeys

    keys = ApiKeys(tmp_path)
    today = datetime.date.today()
    stored, key = keys.create("mission-control", ["read"], today)
    assert keys.find(key) == stored

    keys._save([attrs.evolve(stored, expires_at=today - datetime.timedelta(days=1))])
    assert keys.find(key) is None


def test_keys_created_or_revoked_reach_a_running_ui(tmp_path):
    """Revoking one key and creating another in quick succession can leave the
    file's size and times unchanged, so the content is what is compared"""
    from ros_sugar.ui_node.security import ApiKeys

    running, command = ApiKeys(tmp_path), ApiKeys(tmp_path)
    first, first_key = command.create("aaaa", ["read"])
    assert running.find(first_key) == first

    command.revoke(first.id)
    second, second_key = command.create("bbbb", ["read"])

    assert running.find(first_key) is None
    assert running.find(second_key) == second


def _make_secure_client(node, tmp_path):
    """A client of an API that needs keys, and a command key and a read key"""
    pytest.importorskip("httpx")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.security import ApiKeys

    keys = ApiKeys(tmp_path)
    _, command_key = keys.create("mission-control", ["read", "command"])
    _, read_key = keys.create("dashboard", ["read"])
    client = TestClient(build_api_app(node, keys=keys), base_url="https://testserver")
    return client, keys, command_key, read_key


def test_the_api_needs_a_key_with_the_scope(tmp_path):
    node = _ApiNode()
    client, _, command_key, read_key = _make_secure_client(node, tmp_path)
    body = {"linear": {"x": 1.0}}

    assert client.get("/api/health").status_code == 200
    for headers in ({}, {"Authorization": "Bearer sk_ui_wrong"}, {"Authorization": read_key}):
        refused = client.get("/api/interfaces", headers=headers)
        assert refused.status_code == 401, headers
        assert refused.json() == {"error": "Missing or invalid API key"}
        assert refused.headers["www-authenticate"] == "Bearer"

    as_reader = {"Authorization": f"Bearer {read_key}"}
    assert client.get("/api/interfaces", headers=as_reader).status_code == 200
    refused = client.post("/api/inputs/cmd_vel", json=body, headers=as_reader)
    assert refused.status_code == 403
    assert refused.json() == {"error": "This key does not have the 'command' scope"}
    assert not node.published

    as_commander = {"Authorization": f"bearer {command_key}"}
    for _ in range(2):
        assert client.post("/api/inputs/cmd_vel", json=body, headers=as_commander).status_code == 200
    assert len(node.published) == 2

    # Once per key and address, not once per request
    assert node.logged.count(("info", "API key 'mission-control' used from testclient")) == 1
    assert [m for level, m in node.logged if level == "warning"] == [
        "Refused API access from testclient: Missing or invalid API key"
    ]


def test_api_streams_need_a_key_with_the_scope(tmp_path):
    from starlette.websockets import WebSocketDisconnect

    node = _ApiNode()
    node.latest["odom"] = {"frame_id": "odom", "data": [1.0, 2.0, 3.0]}
    client, _, _, read_key = _make_secure_client(node, tmp_path)
    as_reader = {"Authorization": f"Bearer {read_key}"}

    with client.websocket_connect("/api/outputs/odom?rate=50") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Missing or invalid API key")

    with client.websocket_connect("/api/outputs/odom?rate=50", headers=as_reader) as ws:
        assert ws.receive_json()["topic"] == "odom"

    with client.websocket_connect("/api/inputs/speech/audio", headers=as_reader) as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            # Without the refusal this is published and acknowledged
            ws.send_json({"payload": "QUJD"})
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (
        1008,
        "This key does not have the 'command' scope",
    )
    assert not node.audio_published


def test_a_revoked_key_is_refused_at_once(tmp_path):
    node = _ApiNode()
    client, keys, command_key, _ = _make_secure_client(node, tmp_path)
    as_commander = {"Authorization": f"Bearer {command_key}"}
    assert client.get("/api/interfaces", headers=as_commander).status_code == 200

    keys.revoke(next(k.id for k in keys.all() if k.name == "mission-control"))

    assert client.get("/api/interfaces", headers=as_commander).status_code == 401


def test_an_api_without_a_front_end_gives_no_cookie(tmp_path):
    """The cookie grants the API to the browser that loaded the robot's page,
    so an API served without the front end never sets it"""
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.security import ApiKeys, session_key

    app = build_api_app(_ApiNode(), keys=ApiKeys(tmp_path), session_key=session_key(tmp_path))
    client = TestClient(app, base_url="https://testserver")

    assert "set-cookie" not in client.get("/").headers
    assert client.get("/api/interfaces").status_code == 401


def test_ui_security_manages_keys(tmp_path):
    import os
    import re
    from pathlib import Path

    from ros_sugar.ui_node.security import ApiKeys

    script = Path(__file__).resolve().parents[1] / "scripts" / "ui_security"
    env = {**os.environ, "SUGARCOAT_UI_DATA_DIR": str(tmp_path)}

    def ui_security(*args):
        return subprocess.run(
            [sys.executable, str(script), *args], env=env, capture_output=True, text=True
        )

    created = ui_security("keys", "create", "--name", "mission-control", "--scopes", "read,command")
    assert created.returncode == 0, created.stderr
    key = re.search(r"sk_ui_\S+", created.stdout).group()
    stored = ApiKeys(tmp_path).find(key)
    assert (stored.name, stored.scopes) == ("mission-control", ("read", "command"))

    duplicate = ui_security("keys", "create", "--name", "mission-control")
    assert duplicate.returncode != 0
    assert "already exists" in duplicate.stderr

    listed = ui_security("keys", "list")
    assert stored.id in listed.stdout and key not in listed.stdout

    assert ui_security("keys", "revoke", stored.id).returncode == 0
    assert ApiKeys(tmp_path).find(key) is None


def test_logging_refusals_and_key_uses_with_a_ros_logger(tmp_path):
    """An rclpy logger refuses different severities from one line of code, which
    turned the first accepted request after a refusal into a 500"""
    import rclpy.logging

    node = _ApiNode()
    node.get_logger = lambda: rclpy.logging.get_logger("ui_api_test")
    client, _, command_key, _ = _make_secure_client(node, tmp_path)

    assert client.get("/api/interfaces").status_code == 401
    as_commander = {"Authorization": f"Bearer {command_key}"}
    assert client.get("/api/interfaces", headers=as_commander).status_code == 200


# ---------------------------------------------------------------------------
# Routines
# ---------------------------------------------------------------------------
_RUNNING = {
    "name": "patrol",
    "status": "running",
    "index": 1,
    "active_step": "scan",
    "steps": ["go_home", "scan", "back_home"],
    "message": "",
    "elapsed": 4.2,
}


def _routine_node():
    node = _ApiNode()
    node.routines = ["patrol", "dock"]
    node.routine_states = {"patrol": dict(_RUNNING)}
    return node


def test_the_routines_are_listed_with_where_each_has_got_to():
    listed = _make_client(_routine_node()).get("/api/routines").json()

    assert listed == [
        {"name": "patrol", "state": _RUNNING},
        # Declared, but no state has arrived for it yet
        {"name": "dock", "state": None},
    ]


def test_a_routine_is_read_by_name():
    client = _make_client(_routine_node())

    found = client.get("/api/routines/patrol").json()
    assert found == {"name": "patrol", "state": _RUNNING}
    missing = client.get("/api/routines/nowhere")
    assert missing.status_code == 404
    assert "nowhere" in missing.json()["error"]


def test_each_routine_control_reaches_the_monitor():
    node = _routine_node()
    client = _make_client(node)
    commands = ("start", "pause", "resume", "abort")

    for command in commands:
        node.routine_result = (True, f"{command} done")
        response = client.post(f"/api/routines/patrol/{command}")
        assert response.status_code == 200, response.json()
        assert response.json() == {
            "routine": "patrol",
            command: True,
            "message": f"{command} done",
        }

    assert node.routine_calls == [("patrol", command, None) for command in commands]


def test_an_abort_carries_its_reason():
    node = _routine_node()

    client = _make_client(node)
    client.post("/api/routines/patrol/abort", json={"reason": "operator stop"})

    assert node.routine_calls == [("patrol", "abort", "operator stop")]


def test_a_command_the_routine_cannot_take_is_a_conflict():
    """Pausing a routine that is not running: the Monitor says why"""
    node = _routine_node()
    node.routine_result = (False, "Routine 'dock' is not running")

    response = _make_client(node).post("/api/routines/dock/pause")

    assert response.status_code == 409
    assert response.json() == {"error": "Routine 'dock' is not running"}


def test_an_unreachable_monitor_is_reported():
    node = _routine_node()
    node.routine_error = RuntimeError("The Monitor is not available")

    response = _make_client(node).post("/api/routines/patrol/start")

    assert response.status_code == 503
    assert response.json() == {"error": "The Monitor is not available"}


def test_an_unknown_routine_cannot_be_controlled():
    node = _routine_node()

    response = _make_client(node).post("/api/routines/nowhere/start")

    assert response.status_code == 404
    assert not node.routine_calls


def test_a_routine_state_is_pushed_as_it_changes():
    node = _routine_node()
    client = _make_client(node)

    with client.websocket_connect("/api/routines/patrol/state") as ws:
        # Where it stands, as soon as the stream opens
        assert ws.receive_json() == _RUNNING
        ended = {**_RUNNING, "status": "completed", "index": 3}
        node.set_routine_state("patrol", ended)
        assert ws.receive_json()["status"] == "completed"
        # A run that ended keeps the stream open, since the routine can start again
        node.set_routine_state("patrol", dict(_RUNNING))
        assert ws.receive_json()["status"] == "running"


def test_an_unknown_routine_state_stream_is_refused():
    from starlette.websockets import WebSocketDisconnect

    client = _make_client(_routine_node())
    with client.websocket_connect("/api/routines/nowhere/state") as ws:
        with pytest.raises(WebSocketDisconnect) as closed:
            ws.receive_json()
    assert (closed.value.code, closed.value.reason) == (1008, "Unknown routine")


def test_the_interfaces_advertise_the_routines():
    doc = _make_client(_routine_node()).get("/api/interfaces").json()

    assert doc["routines"][0] == {
        "name": "patrol",
        "state": "GET /api/routines/patrol",
        "stream": "WS /api/routines/patrol/state",
        "start": "POST /api/routines/patrol/start",
        "pause": "POST /api/routines/patrol/pause",
        "resume": "POST /api/routines/patrol/resume",
        "abort": "POST /api/routines/patrol/abort",
    }
    assert [r["name"] for r in doc["routines"]] == ["patrol", "dock"]


def test_a_routine_is_followed_with_a_read_key_and_controlled_with_a_command_key(
    tmp_path,
):
    node = _routine_node()
    client, _, command_key, read_key = _make_secure_client(node, tmp_path)
    as_reader = {"Authorization": f"Bearer {read_key}"}
    as_commander = {"Authorization": f"Bearer {command_key}"}

    assert client.get("/api/routines", headers=as_reader).status_code == 200
    stream = "/api/routines/patrol/state"
    with client.websocket_connect(stream, headers=as_reader) as ws:
        assert ws.receive_json()["status"] == "running"
    refused = client.post("/api/routines/patrol/start", headers=as_reader)
    assert refused.status_code == 403
    assert not node.routine_calls

    allowed = client.post("/api/routines/patrol/start", headers=as_commander)
    assert allowed.status_code == 200
    assert node.routine_calls == [("patrol", "start", None)]


def test_a_real_ui_node_follows_the_state_a_routine_publishes():
    """The state topic is latched, so a state published before the UI node
    subscribed arrives all the same"""
    import time

    import rclpy
    from rclpy.qos import DurabilityPolicy
    from std_msgs.msg import String

    from ros_sugar.config import QoSConfig
    from ros_sugar.ui_node.ui_node import UINode, UINodeConfig

    if not rclpy.ok():
        rclpy.init()
    publisher_node = rclpy.create_node("ui_routine_state_publisher")
    latched = QoSConfig(durability=DurabilityPolicy.TRANSIENT_LOCAL, queue_size=1)
    publisher = publisher_node.create_publisher(
        String, "routine/ui_followed/state", latched.to_ros()
    )
    state = {
        "name": "ui_followed",
        "status": "running",
        "index": 0,
        "active_step": "go",
        "steps": ["go"],
        "message": "",
        "elapsed": 1.0,
    }
    publisher.publish(String(data=json.dumps(state)))

    node = UINode(
        component_name="ui_routine_follower",
        config=UINodeConfig(routines=["ui_followed"]),
    )
    node.rclpy_init_node()
    node.custom_on_activate()
    heard = []
    try:
        assert node.add_routine_listener("ui_followed", lambda: heard.append(1))
        assert not node.add_routine_listener("not_shown", lambda: None)

        deadline = time.time() + 10.0
        while node.get_routine_state("ui_followed") is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        followed = node.get_routine_state("ui_followed")
        assert followed is not None, "the latched state never arrived"
        assert followed["active_step"] == "go"
        assert heard, "the listener was not told"
        # Nothing was published since, and the routine is still running
        time.sleep(0.2)
        assert node.get_routine_state("ui_followed")["elapsed"] > 1.0
    finally:
        node.destroy_node()
        publisher_node.destroy_node()


def test_a_real_ui_node_controls_a_routine_through_the_monitor():
    from unittest.mock import MagicMock

    import rclpy
    from automatika_ros_sugar.srv import ExecuteMethod

    from ros_sugar.ui_node.ui_node import UINode, UINodeConfig

    if not rclpy.ok():
        rclpy.init()
    node = UINode(
        component_name="ui_routine_controller",
        config=UINodeConfig(routines=["ui_controlled"]),
    )
    node.rclpy_init_node()
    node.custom_on_activate()
    client = node._runtime_api_client
    client.client.wait_for_service = MagicMock(return_value=True)
    client.send_request = MagicMock(
        return_value=ExecuteMethod.Response(
            success=True, response_json="Routine 'ui_controlled' aborted"
        )
    )
    try:
        answer = node.control_routine("ui_controlled", "abort", reason="operator stop")
        assert answer == (True, "Routine 'ui_controlled' aborted")
        request = client.send_request.call_args.args[0]
        assert request.name == "abort_routine"
        assert json.loads(request.kwargs_json) == {
            "routine_name": "ui_controlled",
            "reason": "operator stop",
        }

        client.send_request.return_value = ExecuteMethod.Response(
            success=False, error_msg="Routine 'ui_controlled' is not running"
        )
        assert node.control_routine("ui_controlled", "pause") == (
            False,
            "Routine 'ui_controlled' is not running",
        )
        assert json.loads(client.send_request.call_args.args[0].kwargs_json) == {
            "routine_name": "ui_controlled"
        }

        with pytest.raises(ValueError, match="not shown in the UI"):
            node.control_routine("elsewhere", "start")
        with pytest.raises(ValueError, match="Unknown routine command"):
            node.control_routine("ui_controlled", "explode")
        client.client.wait_for_service.return_value = False
        with pytest.raises(RuntimeError, match="not available"):
            node.control_routine("ui_controlled", "start")
    finally:
        node.destroy_node()


def test_enable_ui_hands_its_routines_to_the_ui_and_the_monitor():
    from ros_sugar import Launcher
    from ros_sugar.core import Action, BaseComponent, Routine
    from ros_sugar.utils import ActionReturnType

    class _Arm(BaseComponent):
        def _execution_step(self):
            pass

        def home(self, **_) -> ActionReturnType:
            return True, "home"

    arm = _Arm(component_name="ui_routine_arm")
    patrol = Routine("ui_patrol", steps=[Action(arm.home)])
    launcher = Launcher()
    launcher.add_pkg(components=[arm])

    launcher.enable_ui(
        routines=[patrol, "ui_mission"], serve_browser=False, secure=False
    )
    launcher.setup_launch_description()

    assert launcher._ui_node_config.routines == ["ui_patrol", "ui_mission"]
    # Hosted though no event triggers it. A name is left to whoever registers it
    assert launcher.monitor_node._standalone_routines == [patrol]


def test_enable_ui_takes_each_routine_once_as_a_routine_or_a_name():
    from ros_sugar import Launcher

    launcher = Launcher()
    with pytest.raises(TypeError, match="Routine or the name of one"):
        launcher.enable_ui(routines=[42], serve_browser=False)
    with pytest.raises(ValueError, match="given to the UI twice"):
        launcher.enable_ui(routines=["patrol", "patrol"], serve_browser=False)
