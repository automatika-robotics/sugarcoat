"""Tests for the robot plugin framework (``ros_sugar.robot``).

Covers transports, the feedback bus, plugin spec serialization, the
action/event registries, introspection, and end-to-end feedback/command flow
through a mock UDP robot — both at the plugin level and wired into a
``BaseComponent``.
"""

import json
import os
import socket
import subprocess
import sys
import time

import pytest
import rclpy
from std_msgs.msg import Int32 as RosInt32

from ros_sugar.io.topic import Topic
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    Feedback,
    InProcessFeedbackBus,
    PluginMetadata,
    RobotCommand,
    RobotPlugin,
    RobotPluginHost,
    SdkCallbackTransport,
    SocketFeedbackBus,
    UdpTransport,
    create_supported_type,
)

# ---------------------------------------------------------------------------
# Shared fixtures / helpers
# ---------------------------------------------------------------------------


def _free_port() -> int:
    """Grab a free UDP port from the OS."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# A SupportedType wrapping std_msgs/Int32, registered once at import time.
def _int32_callback(msg: RosInt32) -> int:
    return msg.data


RobotInt32 = create_supported_type(RosInt32, callback=_int32_callback)


def _decode_int32(raw: bytes):
    """UDP wire bytes -> RosInt32 (returns None for malformed packets)."""
    try:
        msg = RosInt32()
        msg.data = int(raw.decode())
        return msg
    except (ValueError, UnicodeDecodeError):
        return None


def _encode_int32(output) -> bytes:
    """Component output -> UDP wire bytes."""
    return str(int(output)).encode()


class MockPlugin(RobotPlugin):
    """A minimal robot plugin: one UDP feedback stream and one UDP command,
    plus one action factory and one event factory."""

    def __init__(self, host: str = "127.0.0.1", state_port: int = 0, cmd_port: int = 0):
        self.metadata = PluginMetadata(name="MockPlugin", vendor="test", version="0.1")
        state_transport = UdpTransport(
            "state", send_to=(host, state_port), bind=(host, state_port)
        )
        cmd_transport = UdpTransport("cmd", send_to=(host, cmd_port))
        self.transports = {"state": state_transport, "cmd": cmd_transport}
        self.feedbacks = {
            "Int32": Feedback(
                key="Int32",
                msg_type=RobotInt32,
                transport=state_transport,
                decoder=_decode_int32,
                rate_hz=10.0,
                description="Mock robot integer state",
            )
        }
        self.commands = {
            "Int32": RobotCommand(
                key="Int32",
                transport=cmd_transport,
                encoder=_encode_int32,
                description="Mock robot integer command",
            )
        }
        self.actions = ActionRegistry(
            {"ping": lambda: self._send_cmd(1)}
        )
        self.events = EventRegistry(
            {
                "state_high": lambda threshold=100: self.feedbacks["Int32"]
                .as_topic()
                .msg.data
                > threshold
            }
        )

    def _send_cmd(self, value: int):
        from ros_sugar.core.action import Action

        return Action(method=lambda: self.commands["Int32"].transport.send(
            _encode_int32(value)
        ))


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------


def test_udp_transport_roundtrip():
    """A UDP transport bound to a port receives what it sends to itself."""
    port = _free_port()
    received = []
    transport = UdpTransport(
        "loop", send_to=("127.0.0.1", port), bind=("127.0.0.1", port)
    )
    transport.subscribe(lambda data: received.append(data))
    transport.open()
    try:
        time.sleep(0.1)
        assert transport.send(b"ping-1")
        assert transport.send(b"ping-2")
        deadline = time.time() + 2.0
        while len(received) < 2 and time.time() < deadline:
            time.sleep(0.02)
        assert received == [b"ping-1", b"ping-2"]
    finally:
        transport.close()
    assert not transport.is_open()


def test_udp_transport_egress_only():
    """A send-only UDP transport (no bind) still sends; another socket receives."""
    port = _free_port()
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", port))
    rx.settimeout(2.0)
    transport = UdpTransport("tx", send_to=("127.0.0.1", port))
    transport.open_egress()
    try:
        assert transport.send(b"hello")
        data, _ = rx.recvfrom(1024)
        assert data == b"hello"
    finally:
        transport.close()
        rx.close()


def test_sdk_callback_transport():
    """The SDK transport wires register/unregister/send through to plain callables."""
    inbound = []
    registered = {}

    def register(cb):
        registered["cb"] = cb
        return "handle-1"

    def unregister(handle):
        registered["unregistered"] = handle

    def send(payload):
        inbound.append(("sent", payload))

    transport = SdkCallbackTransport(
        "sdk", register_fn=register, unregister_fn=unregister, send_fn=send
    )
    transport.subscribe(lambda msg: inbound.append(("recv", msg)))
    transport.open()
    # The SDK delivers a message via the registered callback
    registered["cb"]({"x": 1})
    assert transport.send("cmd")
    transport.close()

    assert ("recv", {"x": 1}) in inbound
    assert ("sent", "cmd") in inbound
    assert registered["unregistered"] == "handle-1"


# ---------------------------------------------------------------------------
# Feedback bus
# ---------------------------------------------------------------------------


def test_in_process_feedback_bus():
    """In-process bus delivers published bytes to channel subscribers."""
    bus = InProcessFeedbackBus()
    bus.start()
    seen_a, seen_b = [], []
    handle = bus.subscribe("chan/a", lambda d: seen_a.append(d))
    bus.subscribe("chan/b", lambda d: seen_b.append(d))
    bus.publish("chan/a", b"one")
    bus.publish("chan/b", b"two")
    bus.publish("chan/a", b"three")
    assert seen_a == [b"one", b"three"]
    assert seen_b == [b"two"]
    # Unsubscribing stops delivery
    handle.unsubscribe()
    bus.publish("chan/a", b"four")
    assert seen_a == [b"one", b"three"]
    bus.close()


def test_socket_feedback_bus_bidirectional():
    """Socket bus relays HOST->client (feedback) and client->HOST (commands)."""
    server = SocketFeedbackBus()
    server.start()
    assert server.endpoint is not None
    client = SocketFeedbackBus(server.endpoint)
    client.connect()
    try:
        client_seen, server_seen = [], []
        client.subscribe("feedback/x", lambda d: client_seen.append(d))
        server.subscribe("command/y", lambda d: server_seen.append(d))
        # Give the client's SUBSCRIBE frame time to register on the server
        time.sleep(0.3)
        server.publish("feedback/x", b"telemetry")
        client.publish("command/y", b"command")
        deadline = time.time() + 2.0
        while (not client_seen or not server_seen) and time.time() < deadline:
            time.sleep(0.02)
        assert client_seen == [b"telemetry"]
        assert server_seen == [b"command"]
    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------------
# Plugin spec serialization & introspection
# ---------------------------------------------------------------------------


def test_plugin_spec_roundtrip():
    """A plugin's spec is JSON-serializable and rebuilds an equivalent instance."""
    plugin = MockPlugin(state_port=46000, cmd_port=46001)
    spec = plugin.to_spec()
    # spec must be JSON-serializable
    json.dumps(spec)
    assert spec["class"].endswith(":MockPlugin")
    assert spec["kwargs"] == {
        "host": "127.0.0.1",
        "state_port": 46000,
        "cmd_port": 46001,
    }
    rebuilt = RobotPlugin.from_spec(spec)
    assert set(rebuilt.transports) == {"state", "cmd"}
    assert set(rebuilt.feedbacks) == {"Int32"}
    assert set(rebuilt.commands) == {"Int32"}


def test_plugin_introspection():
    """``describe`` / ``list_*`` expose the plugin surface."""
    plugin = MockPlugin(state_port=46010, cmd_port=46011)
    desc = plugin.describe()
    assert desc["metadata"]["name"] == "MockPlugin"
    assert desc["transports"] == {"state": "UdpTransport", "cmd": "UdpTransport"}
    assert [f["key"] for f in desc["feedbacks"]] == ["Int32"]
    assert desc["feedbacks"][0]["transport_kind"] == "UdpTransport"
    assert desc["feedbacks"][0]["channel"] == "robot/feedback/Int32"
    assert [c["key"] for c in desc["commands"]] == ["Int32"]
    assert {a["name"] for a in desc["actions"]} == {"ping"}
    assert {e["name"] for e in desc["events"]} == {"state_high"}


def test_inspect_cli():
    """``python -m ros_sugar.robot inspect`` emits the plugin's JSON surface."""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(test_dir)
    env = dict(os.environ)
    # Make both `ros_sugar` (repo root) and `robot_plugin_test` (test dir)
    # importable by the subprocess.
    env["PYTHONPATH"] = os.pathsep.join(
        [repo_root, test_dir, env.get("PYTHONPATH", "")]
    )
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ros_sugar.robot",
            "inspect",
            "robot_plugin_test:MockPlugin",
        ],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload["metadata"]["name"] == "MockPlugin"
    assert [f["key"] for f in payload["feedbacks"]] == ["Int32"]


# ---------------------------------------------------------------------------
# Registries
# ---------------------------------------------------------------------------


def test_registries_attribute_access_and_listing():
    """Registries expose factories by attribute and via ``list``/``names``."""
    plugin = MockPlugin(state_port=46020, cmd_port=46021)
    assert "ping" in plugin.actions
    assert "state_high" in plugin.events
    assert plugin.actions.names() == ["ping"]
    assert callable(plugin.actions.ping)
    # Event factory builds a fresh Condition-bearing object each call
    cond_default = plugin.events.state_high()
    cond_custom = plugin.events.state_high(threshold=5)
    assert cond_default is not cond_custom
    specs = plugin.events.list()
    assert specs[0].name == "state_high"
    with pytest.raises(AttributeError):
        _ = plugin.actions.nonexistent


# ---------------------------------------------------------------------------
# End-to-end: plugin HOST against a mock UDP robot
# ---------------------------------------------------------------------------


def test_plugin_host_feedback_and_command_flow():
    """A HOST plugin decodes UDP telemetry onto the bus and sends UDP commands."""
    state_port = _free_port()
    cmd_port = _free_port()

    # Mock robot: a socket that receives commands sent by the plugin
    robot_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_cmd_sock.bind(("127.0.0.1", cmd_port))
    robot_cmd_sock.settimeout(2.0)

    plugin = MockPlugin(state_port=state_port, cmd_port=cmd_port)
    bus = InProcessFeedbackBus()

    monitor_feed = []
    host = RobotPluginHost(
        plugin,
        node=None,
        bus=bus,
        monitor_feed=lambda name, msg: monitor_feed.append((name, msg.data)),
    )
    host.open()
    try:
        # A consumer subscribes to the feedback channel as a component would
        decoded = []
        from rclpy.serialization import deserialize_message

        bus.subscribe(
            "robot/feedback/Int32",
            lambda data: decoded.append(deserialize_message(data, RosInt32).data),
        )

        # The robot streams a telemetry packet into the plugin's bound port
        robot_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot_tx.sendto(b"77", ("127.0.0.1", state_port))
        deadline = time.time() + 2.0
        while not decoded and time.time() < deadline:
            time.sleep(0.02)
        assert decoded == [77]
        assert monitor_feed == [("robot/feedback/Int32", 77)]

        # The plugin sends a command; the mock robot receives the encoded bytes
        assert plugin.send_command(plugin.commands["Int32"], _encode_int32(42))
        data, _ = robot_cmd_sock.recvfrom(1024)
        assert data == b"42"
        robot_tx.close()
    finally:
        host.close()
        robot_cmd_sock.close()


# ---------------------------------------------------------------------------
# Component integration
# ---------------------------------------------------------------------------


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_component_use_robot_plugin(rclpy_context):
    """``BaseComponent._use_robot_plugin`` rewires inputs/outputs to the plugin."""
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.robot.adapters import RobotCommandPublisher

    state_port = _free_port()
    cmd_port = _free_port()

    robot_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_cmd_sock.bind(("127.0.0.1", cmd_port))
    robot_cmd_sock.settimeout(2.0)

    plugin = MockPlugin(state_port=state_port, cmd_port=cmd_port)
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()

    component = BaseComponent(
        component_name="robot_plugin_test_component",
        inputs=[Topic(name="robot_state", msg_type="Int32", use_plugin=True)],
        outputs=[Topic(name="robot_cmd", msg_type="Int32", use_plugin=True)],
    )
    component.rclpy_init_node()
    component._robot_plugin = plugin
    try:
        component._use_robot_plugin()

        # Both topics were bound to the plugin's non-ROS transports
        assert component._external_topics == {"robot_state", "robot_cmd"}
        assert isinstance(
            component.publishers_dict["robot_cmd"], RobotCommandPublisher
        )

        # Telemetry pushed by the plugin reaches the component's callback slot
        robot_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot_tx.sendto(b"55", ("127.0.0.1", state_port))
        callback = component.callbacks["robot_state"]
        deadline = time.time() + 2.0
        while callback.msg is None and time.time() < deadline:
            time.sleep(0.02)
        assert callback.msg is not None
        assert callback.get_output() == 55
        robot_tx.close()

        # Publishing on the component output sends an encoded UDP command
        component.publishers_dict["robot_cmd"].publish(99)
        data, _ = robot_cmd_sock.recvfrom(1024)
        assert data == b"99"
    finally:
        host.close()
        robot_cmd_sock.close()
        component.destroy_node()


# ---------------------------------------------------------------------------
# use_plugin opt-in and key-based disambiguation
# ---------------------------------------------------------------------------


def test_topic_use_plugin_default_is_false():
    """Without explicit opt-in, a Topic does not route through the plugin."""
    topic = Topic(name="raw", msg_type="Int32")
    assert topic.use_plugin is False


def test_topic_without_use_plugin_is_not_claimed(rclpy_context):
    """A topic that doesn't opt in stays as a plain ROS subscriber/publisher
    even when the plugin has a matching message type."""
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.robot.adapters import RobotCommandPublisher

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()
    component = BaseComponent(
        component_name="optout_component",
        inputs=[Topic(name="internal_state", msg_type="Int32")],   # no use_plugin
        outputs=[Topic(name="internal_cmd", msg_type="Int32")],    # no use_plugin
    )
    component.rclpy_init_node()
    component._robot_plugin = plugin
    try:
        component._use_robot_plugin()
        # Neither topic is claimed by the plugin
        assert "internal_state" not in component._external_topics
        assert "internal_cmd" not in component._external_topics
        assert not isinstance(
            component.publishers_dict.get("internal_cmd"), RobotCommandPublisher
        )
    finally:
        host.close()
        component.destroy_node()


def test_resolve_feedback_by_topic_name_disambiguates():
    """Recipe authors disambiguate by setting ``Topic.name`` to match the
    plugin's registry key (no separate string param)."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    extra_transport = UdpTransport("extra", send_to=("127.0.0.1", _free_port()))
    plugin.feedbacks["left_arm"] = Feedback(
        key="left_arm",
        msg_type=RobotInt32,
        transport=extra_transport,
        decoder=_decode_int32,
    )
    plugin.feedbacks["right_arm"] = Feedback(
        key="right_arm",
        msg_type=RobotInt32,
        transport=extra_transport,
        decoder=_decode_int32,
    )
    # Drop the type-named entry so the type-scan path is ambiguous.
    plugin.feedbacks.pop("Int32")

    left = plugin.resolve_feedback("left_arm", "Int32")
    right = plugin.resolve_feedback("right_arm", "Int32")
    assert left is plugin.feedbacks["left_arm"]
    assert right is plugin.feedbacks["right_arm"]


def test_resolve_feedback_ambiguous_type_raises():
    """When the topic name doesn't match any key and multiple feedbacks share
    the type, the framework raises with the available keys."""
    from ros_sugar.robot import AmbiguousPluginEntryError

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    extra_transport = UdpTransport("extra", send_to=("127.0.0.1", _free_port()))
    plugin.feedbacks["left_arm"] = Feedback(
        key="left_arm",
        msg_type=RobotInt32,
        transport=extra_transport,
        decoder=_decode_int32,
    )
    plugin.feedbacks["right_arm"] = Feedback(
        key="right_arm",
        msg_type=RobotInt32,
        transport=extra_transport,
        decoder=_decode_int32,
    )
    plugin.feedbacks.pop("Int32")

    with pytest.raises(AmbiguousPluginEntryError) as excinfo:
        plugin.resolve_feedback("nonmatching_topic", "Int32")
    msg = str(excinfo.value)
    assert "left_arm" in msg and "right_arm" in msg


def test_resolve_feedback_type_mismatch_on_key_match_raises():
    """When ``Topic.name`` matches a plugin key but the message types
    disagree, the recipe is mis-wired -- surface it loudly."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    # The MockPlugin has ``feedbacks["Int32"]`` of type RobotInt32. Asking
    # for it by key with a *different* type should fail.
    with pytest.raises(TypeError) as excinfo:
        plugin.resolve_feedback("Int32", "Float64")
    msg = str(excinfo.value)
    assert "Int32" in msg and "Float64" in msg


def test_resolve_feedback_unique_type_match_succeeds():
    """When the topic name doesn't match any key and only one feedback has
    the requested type, that feedback is returned (the common case)."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    # Drop the type-named key so resolution falls through to the type-scan path
    plugin.feedbacks["only_one"] = plugin.feedbacks.pop("Int32")

    feedback = plugin.resolve_feedback("any_topic_name", "Int32")
    assert feedback is plugin.feedbacks["only_one"]


def test_feedback_spec_exposes_key():
    """``FeedbackSpec.key`` is the value to pass to ``Topic(use_plugin=)``."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    desc = plugin.describe()
    keys = {f["key"] for f in desc["feedbacks"]}
    assert keys == {"Int32"}
    cmd_keys = {c["key"] for c in desc["commands"]}
    assert cmd_keys == {"Int32"}


def test_warn_orphaned_plugin_topics(rclpy_context, caplog):
    """When no plugin is attached, ``use_plugin=True`` topics still work but
    emit a warning so the recipe-vs-deployment mismatch is visible."""
    import logging
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(
        component_name="orphan_warn_component",
        inputs=[Topic(name="cmd", msg_type="Int32", use_plugin=True)],
    )
    component.rclpy_init_node()
    try:
        # No plugin attached
        assert component._robot_plugin is None
        with caplog.at_level(logging.WARNING):
            component._warn_orphaned_plugin_topics()
        # Topic stays as an ordinary ROS topic; no _external_topics entry
        assert "cmd" not in component._external_topics
    finally:
        component.destroy_node()
