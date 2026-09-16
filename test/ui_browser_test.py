"""Tests for the FastHTML browser front-end of the UI node."""

import pytest

from ros_sugar.io.topic import Topic


class _BrowserNode:
    """Fake UINode exposing the surface the browser log pane uses."""

    def __init__(self, in_topics):
        self.in_topics = in_topics  # recipe outputs, shown in the log
        self.out_topics = []
        self.config = type("Config", (), {"components": {}, "hide_settings": True})()
        self.latest = {}  # topic_name -> get_latest_output result
        self.output_listeners = {}  # topic_name -> set of push listeners
        self.actions = []  # action_clients_inputs_dicts result
        self.feedback = None  # get_action_feedback result
        self.feedback_listeners = {}  # action_name -> set of push listeners

    def srv_clients_inputs_dicts(self):
        return []

    def action_clients_inputs_dicts(self):
        return self.actions

    def get_action_feedback(self, name):
        return self.feedback

    def add_action_feedback_listener(self, name, listener):
        self.feedback_listeners.setdefault(name, set()).add(listener)
        return True

    def remove_action_feedback_listener(self, name, listener):
        self.feedback_listeners.get(name, set()).discard(listener)

    def set_feedback(self, name, **feedback):
        """Simulate an action client update"""
        self.feedback = feedback
        for listener in list(self.feedback_listeners.get(name, ())):
            listener()

    def get_latest_output(self, name):
        return self.latest.get(name)

    def add_output_listener(self, name, listener):
        self.output_listeners.setdefault(name, set()).add(listener)
        return True

    def remove_output_listener(self, name, listener):
        self.output_listeners.get(name, set()).discard(listener)

    def publish(self, name, content):
        """Simulate a message arriving on an output topic"""
        self.latest[name] = content
        for listener in list(self.output_listeners.get(name, ())):
            listener()


def test_log_shows_false_and_zero(tmp_path, monkeypatch):
    """False and 0 are values: the log must show them, not skip them as empty"""
    import time

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([
        Topic(name="flag", msg_type="Bool"),
        Topic(name="level", msg_type="Float32"),
        Topic(name="note", msg_type="String"),
    ])
    client = TestClient(build_browser_app(node))
    with client.websocket_connect("/ws") as ws:
        # The log pane registers its listeners once the socket is connected
        deadline = time.time() + 5.0
        while len(node.output_listeners) < 3 and time.time() < deadline:
            time.sleep(0.01)
        node.publish("flag", False)
        node.publish("level", 0.0)
        # A non-empty message last, so a log that skips False and 0 still
        # sends something and the test fails instead of waiting forever
        node.publish("note", "done")
        log = ws.receive_text()
        while "done" not in log:
            log = ws.receive_text()

    # Each frame is the whole log card, so the last one holds every entry
    assert "</strong>False</span>" in log
    assert "</strong>0.0</span>" in log


def test_log_leaves_out_a_type_without_a_log_element(tmp_path, monkeypatch):
    """A type with no log element is not subscribed, so it cannot flood or
    stop the log, and the other topics are still logged"""
    import time

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([
        Topic(name="joints", msg_type="JointState"),
        Topic(name="note", msg_type="String"),
    ])
    client = TestClient(build_browser_app(node))
    with client.websocket_connect("/ws") as ws:
        deadline = time.time() + 5.0
        while not node.output_listeners and time.time() < deadline:
            time.sleep(0.01)
        assert set(node.output_listeners) == {"note"}
        node.publish("note", "done")
        log = ws.receive_text()

    assert "done" in log


def test_action_result_is_logged_once_when_it_arrives_after_the_end(
    tmp_path, monkeypatch
):
    """The status can end before the result arrives: the result is still
    logged, and only once"""
    import threading
    import time

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient
    from tf2_msgs.action import LookupTransform

    from ros_sugar.io.supported_types import get_ros_msg_fields_dict
    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    name = "tf/lookup"
    node = _BrowserNode([])
    node.actions = [{
        "name": name,
        "type": "LookupTransform",
        "fields": get_ros_msg_fields_dict(LookupTransform.Goal),
    }]
    client = TestClient(build_browser_app(node))
    ended = {"status": "completed", "timestep": 3, "duration_secs": 1.0, "feedback": None}
    result = LookupTransform.Result()
    result.transform.child_frame_id = "gripper"

    with client.websocket_connect("/ws_actions") as ws:

        def _receive_until(text):
            """Frames up to the first holding text, or those received in 5 s"""
            frames = []

            def _read():
                while not frames or text not in frames[-1]:
                    frames.append(ws.receive_text())

            # The test client has no receive timeout, so read from a thread
            reader = threading.Thread(target=_read, daemon=True)
            reader.start()
            reader.join(timeout=5.0)
            return frames

        deadline = time.time() + 5.0
        while not node.feedback_listeners and time.time() < deadline:
            time.sleep(0.01)
        node.set_feedback(name, result=None, **ended)
        assert _receive_until("completed"), "no card for the ended goal"
        node.set_feedback(name, result=result, **ended)
        logged = _receive_until("result")
        # Nothing new: the result must not be logged again
        node.set_feedback(name, result=result, **ended)
        time.sleep(0.3)  # handled on its own, not merged with the next update
        node.set_feedback(
            name, status="running", result=None, timestep=0, duration_secs=0.0,
            feedback=None,
        )
        later = _receive_until("running")

    assert logged and "child_frame_id: gripper" in logged[-1], "the result was not logged"
    assert not [f for f in later if "result" in f], "the result was logged twice"


def test_a_browser_command_from_another_site_is_refused(tmp_path, monkeypatch):
    """The front end's form routes take commands, and a page on another site can
    make a browser post a form to them. The robot's own page is unaffected."""
    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient
    from tf2_msgs.action import LookupTransform

    from ros_sugar.io.supported_types import get_ros_msg_fields_dict
    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([])
    node.actions = [{
        "name": "tf/lookup",
        "type": "LookupTransform",
        "fields": get_ros_msg_fields_dict(LookupTransform.Goal),
        "goal_class": LookupTransform.Goal,
    }]
    cancelled = []
    node.cancel_action = lambda name: (cancelled.append(name), (True, "cancelled"))[1]
    # Served the way the UI node serves it: the front end mounted under the API
    client = TestClient(build_api_app(node, build_browser_app(node)))

    form = {"action_name": "tf/lookup"}
    foreign = client.post("/action/cancel", data=form, headers={"Origin": "http://evil.example"})
    assert foreign.status_code == 403
    assert not cancelled

    own = client.post("/action/cancel", data=form, headers={"Origin": "http://testserver"})
    assert own.status_code == 200
    assert cancelled == ["tf/lookup"]


def test_other_sites_cannot_frame_the_ui(tmp_path, monkeypatch):
    """Another site could show the UI in a hidden frame and trick an operator
    into clicking its controls, so browsers are told to allow only the UI's own
    pages to frame it"""
    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([])
    # Served the way the UI node serves it: the front end mounted under the API
    client = TestClient(build_api_app(node, build_browser_app(node)))

    page = client.get("/")

    assert page.status_code == 200
    assert page.headers["x-frame-options"] == "SAMEORIGIN"
    assert page.headers["content-security-policy"] == "frame-ancestors 'self'"
