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


def test_a_boolean_request_field_is_a_checkbox_sending_true():
    """The schema names the type 'boolean'. Matched as 'bool', it rendered as a
    text box, where typing 'false' sent true"""
    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from fasthtml.common import to_xml

    from ros_sugar.ui_node.elements import _generic_message_form

    form = to_xml(_generic_message_form({"data": "boolean"}))

    assert 'type="checkbox"' in form
    assert 'name="data"' in form and 'value="true"' in form


def test_a_form_value_a_field_cannot_hold_is_shown(tmp_path, monkeypatch):
    """The error names the field, instead of the goal being reported as rejected
    or the page failing"""
    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient
    from tf2_msgs.action import LookupTransform

    from ros_sugar.io.supported_types import get_ros_msg_fields_dict
    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([])
    node.actions = [{
        "name": "tf/lookup",
        "type": "LookupTransform",
        "fields": get_ros_msg_fields_dict(LookupTransform.Goal),
    }]

    def _refuse(data):
        raise ValueError(
            "Invalid value for field 'sec' (int32): expected a whole number, got '2.5'"
        )

    node.send_action_goal = _refuse
    client = TestClient(build_browser_app(node))

    page = client.post("/action/goal", data={"action_name": "tf/lookup", "sec": "2.5"})

    assert page.status_code == 200
    assert "expected a whole number" in page.text


def test_the_robots_page_gives_its_browser_the_api(tmp_path, monkeypatch):
    """A secure UI needs keys on its API, but the front end's scripts call the API
    too: loading the page gives the browser a cookie for it, which other sites'
    pages and other clients do not have"""
    import logging

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.api import build_api_app
    from ros_sugar.ui_node.browser import build_browser_app
    from ros_sugar.ui_node.security import ApiKeys, session_key

    monkeypatch.chdir(tmp_path)
    state = tmp_path / "state"
    state.mkdir()
    secret = session_key(state)
    node = _BrowserNode([Topic(name="note", msg_type="String")])
    node.get_logger = lambda: logging.getLogger("ui_browser_test")
    node.publish("note", "hello")
    app = build_api_app(
        node,
        build_browser_app(node, session_key=secret),
        keys=ApiKeys(state),
        session_key=secret,
    )
    browser = TestClient(app, base_url="https://testserver")

    assert browser.get("/api/outputs/note/latest").status_code == 401

    page = browser.get("/")
    assert page.status_code == 200
    cookie = next(
        c for c in page.headers.get_list("set-cookie") if c.startswith("sugarcoat_ui=")
    )
    assert {"HttpOnly", "Secure", "SameSite=Strict"} <= set(cookie.split("; "))

    assert browser.get("/api/outputs/note/latest").json()["payload"] == "hello"
    # The test client opens WebSockets over ws://, where it holds back a Secure
    # cookie that a browser sends over wss://, so the cookie is given explicitly
    with browser.websocket_connect(
        "/api/outputs/note", headers={"Cookie": cookie.split("; ")[0]}
    ) as ws:
        assert ws.receive_json()["payload"] == "hello"

    forged = TestClient(app, base_url="https://testserver", cookies={"sugarcoat_ui": "forged"})
    assert forged.get("/api/outputs/note/latest").status_code == 401

    # The front end's session is signed with the given secret, not a key file
    # written to the working directory
    assert not (tmp_path / ".sesskey").exists()


def test_the_page_loads_nothing_from_other_hosts(tmp_path, monkeypatch):
    """Scripts on the page can use the robot's API, and robots are often offline,
    so every script, style and image comes from the UI itself"""
    import re
    from urllib.parse import urlsplit

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    node = _BrowserNode([
        Topic(name="map", msg_type="OccupancyGrid"),
        Topic(name="camera", msg_type="Image"),
    ])
    client = TestClient(build_browser_app(node))
    page = client.get("/").text

    urls = re.findall(r'<(?:script|link|img)\b[^>]*?\b(?:src|href)="([^"]+)"', page)
    assert len(urls) > 10
    for url in urls:
        if url.startswith(("http://", "https://")):
            # The canonical link names the page itself
            assert urlsplit(url).hostname == "testserver", url
        else:
            assert client.get("/" + url.lstrip("/")).status_code == 200, url


def test_the_page_serves_no_file_outside_its_static_folder(tmp_path, monkeypatch):
    """A URL climbing out of the static folder with '..' must not reach other
    files, as FastHTML's own static route lets it"""
    import os

    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from starlette.testclient import TestClient

    import ros_sugar.ui_node.frontend as frontend
    from ros_sugar.ui_node.browser import build_browser_app

    monkeypatch.chdir(tmp_path)  # FastHTML writes its session key to the cwd
    secret = tmp_path / "secret.txt"
    secret.write_text("not for the network")
    static = os.path.join(os.path.dirname(os.path.realpath(frontend.__file__)), "static")
    climb = os.path.relpath(secret, static)
    client = TestClient(build_browser_app(_BrowserNode([])))

    assert client.get("/custom.css").status_code == 200
    for path in (
        climb.replace("..", "%2E%2E"),
        climb.replace("/", "%2F"),
        "%2F" + str(secret).lstrip("/").replace("/", "%2F"),
    ):
        response = client.get(f"/{path}")
        assert response.status_code == 404, path
        assert "not for the network" not in response.text


def test_fasthtml_and_monsterui_ask_for_the_bundled_front_end_files():
    """The browser front end serves copies of the files FastHTML and MonsterUI
    load from CDNs. Their Python code generates markup for those exact files, so
    a release asking for other ones may no longer fit the copies, and needs the
    copies updated or its version bounded"""
    pytest.importorskip("fasthtml")
    pytest.importorskip("monsterui")
    from fasthtml.core import def_hdrs, htmx_exts
    from monsterui.all import Theme

    from ros_sugar.ui_node.frontend import _THEME_FILES

    def urls(headers):
        return {
            h.attrs.get("src") or h.attrs.get("href")
            for h in headers
            if (h.attrs.get("src") or h.attrs.get("href") or "").startswith("http")
        }

    fasthtml_files = urls(def_hdrs()) | {htmx_exts["ws"]}
    assert fasthtml_files == {
        "https://cdn.jsdelivr.net/npm/htmx.org@2.0.7/dist/htmx.js",
        "https://cdn.jsdelivr.net/gh/answerdotai/fasthtml-js@1.0.12/fasthtml.js",
        "https://cdn.jsdelivr.net/gh/answerdotai/surreal@main/surreal.js",
        "https://cdn.jsdelivr.net/gh/gnat/css-scope-inline@main/script.js",
        "https://cdn.jsdelivr.net/npm/htmx-ext-ws@2.0.3/ws.js",
    }, "FastHTML loads other files than static/vendor has copies of"
    assert urls(Theme.red.headers()) == _THEME_FILES, (
        "MonsterUI's theme loads other files than static/vendor has copies of"
    )
