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

    def srv_clients_inputs_dicts(self):
        return []

    def action_clients_inputs_dicts(self):
        return []

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
