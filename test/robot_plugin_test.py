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
                name="Int32",
                msg_type=RobotInt32,
                transport=state_transport,
                decoder=_decode_int32,
                rate_hz=10.0,
                description="Mock robot integer state",
            )
        }
        self.commands = {
            "Int32": RobotCommand(
                name="Int32",
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
    assert [f["name"] for f in desc["feedbacks"]] == ["Int32"]
    assert desc["feedbacks"][0]["transport_kind"] == "UdpTransport"
    assert desc["feedbacks"][0]["channel"] == "robot/feedback/Int32"
    assert [c["name"] for c in desc["commands"]] == ["Int32"]
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
    assert [f["name"] for f in payload["feedbacks"]] == ["Int32"]


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
        inputs=[Topic(name="robot_state", msg_type="Int32")],
        outputs=[Topic(name="robot_cmd", msg_type="Int32")],
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
