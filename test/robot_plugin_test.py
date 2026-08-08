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
    HttpTransport,
    InProcessFeedbackBus,
    PluginMetadata,
    RobotCommand,
    RobotPlugin,
    RobotPluginHost,
    SensorPlugin,
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


class MockSensor(SensorPlugin):
    """A sensor plugin with no I/O of its own -- enough to be attached."""

    def __init__(self, cam_name: str = "camera"):
        self.metadata = PluginMetadata(name=cam_name)


class MockUdpSensor(SensorPlugin):
    """A sensor plugin with one UDP feedback -- a camera-shaped stand-in."""

    def __init__(self, host: str = "127.0.0.1", state_port: int = 0):
        self.metadata = PluginMetadata(name="MockSensor", vendor="test")
        transport = UdpTransport(
            "state", send_to=(host, state_port), bind=(host, state_port)
        )
        self.transports = {"state": transport}
        self.feedbacks = {
            "Int32": Feedback(
                key="Int32",
                msg_type=RobotInt32,
                transport=transport,
                decoder=_decode_int32,
            )
        }


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


def test_http_transport_send_and_poll():
    """``HttpTransport`` POSTs commands and polls telemetry against a real
    local HTTP server -- exercises both directions end to end.

    HttpTransport is a public extension point with no in-repo robot using it
    yet; this keeps it verified rather than unexercised.
    """
    import threading
    from http.server import BaseHTTPRequestHandler, HTTPServer

    received_posts = []

    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):  # silence default stderr logging
            pass

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            received_posts.append(self.rfile.read(length))
            self.send_response(200)
            self.end_headers()

        def do_GET(self):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"telemetry-data")

    server = HTTPServer(("127.0.0.1", 0), _Handler)
    port = server.server_address[1]
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    transport = HttpTransport(
        "http",
        base_url=f"http://127.0.0.1:{port}",
        send_path="cmd",
        poll_path="telemetry",
        poll_rate_hz=50.0,
    )
    polled = []
    transport.subscribe(lambda body: polled.append(body))
    transport.open()
    try:
        # Outbound: a POST command reaches the server with the exact body.
        assert transport.send(b"go")
        deadline = time.time() + 2.0
        while not received_posts and time.time() < deadline:
            time.sleep(0.02)
        assert received_posts and received_posts[0] == b"go"

        # Inbound: the polled GET body is dispatched to the subscriber.
        deadline = time.time() + 2.0
        while not polled and time.time() < deadline:
            time.sleep(0.02)
        assert polled and polled[0] == b"telemetry-data"
    finally:
        transport.close()
        server.shutdown()
        server_thread.join(timeout=2.0)


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


def test_socket_feedback_bus_concurrent_client_publishes():
    """Many threads publishing on one client socket must not interleave frames.

    ``sendall`` isn't atomic across threads; without a send lock concurrent
    publishes corrupt the length-prefixed stream and the server desyncs. Each
    payload is distinct and large enough to span multiple ``send`` syscalls,
    so an unserialized writer would reliably scramble framing.
    """
    import threading

    server = SocketFeedbackBus()
    server.start()
    client = SocketFeedbackBus(server.endpoint)
    client.connect()
    try:
        # Shrink the client send buffer so each large publish forces ``sendall``
        # to loop over multiple ``send`` syscalls -- that loop is where an
        # unserialized concurrent writer interleaves bytes. Without this, a
        # single-syscall send on loopback is atomic by luck and the race hides.
        client._client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048)

        received = []
        server.subscribe("cmd", lambda d: received.append(d))
        time.sleep(0.3)  # let the SUBSCRIBE register

        n_threads, per_thread = 8, 15
        # Distinct payloads, each far larger than the send buffer (so sendall
        # makes many syscalls). A single byte-level interleave yields a payload
        # that isn't in the valid set.
        payloads = {
            t: bytes([65 + t]) * (256 * 1024 + t) for t in range(n_threads)
        }

        def publisher(t):
            for _ in range(per_thread):
                client.publish("cmd", payloads[t])

        threads = [threading.Thread(target=publisher, args=(t,)) for t in range(n_threads)]
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        deadline = time.time() + 15.0
        total = n_threads * per_thread
        while len(received) < total and time.time() < deadline:
            time.sleep(0.02)

        assert len(received) == total, f"lost frames: {len(received)}/{total}"
        # Every received frame must be one of the intact payloads -- a single
        # interleaved/desynced frame would not match any.
        valid = set(payloads.values())
        assert all(r in valid for r in received), "frame corruption / interleave detected"
        # And each payload arrived the expected number of times.
        from collections import Counter
        counts = Counter(received)
        assert all(counts[p] == per_thread for p in payloads.values())
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


class _RouteViaHostPlugin(RobotPlugin):
    """Plugin whose command transport is marked ``route_via_host`` -- the
    client publishes the command to the bus and the HOST forwards it to the
    transport. Exercises the route-via-host path (for command transports that
    can only live in the host process, e.g. a single-connection vendor SDK)."""

    def __init__(self, cmd_port: int = 0):
        self.metadata = PluginMetadata(name="RouteViaHost", vendor="test")
        cmd = UdpTransport(
            "cmd", send_to=("127.0.0.1", cmd_port), route_via_host=True
        )
        self.transports = {"cmd": cmd}
        self.commands = {
            "Int32": RobotCommand(key="Int32", transport=cmd, encoder=_encode_int32)
        }


def test_route_via_host_command_forwarding():
    """A ``route_via_host`` command published from the client side is forwarded
    by the HOST to the underlying transport."""
    cmd_port = _free_port()
    robot_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_cmd_sock.bind(("127.0.0.1", cmd_port))
    robot_cmd_sock.settimeout(2.0)

    plugin = _RouteViaHostPlugin(cmd_port=cmd_port)
    assert plugin.commands["Int32"].transport.route_via_host is True

    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()  # attaches the bus and registers the host-side forwarder
    try:
        # send_command sees route_via_host -> publishes to the bus channel;
        # the HOST's forwarder receives it and calls transport.send.
        assert plugin.send_command(plugin.commands["Int32"], _encode_int32(7))
        data, _ = robot_cmd_sock.recvfrom(1024)
        assert data == b"7"
    finally:
        host.close()
        robot_cmd_sock.close()


def test_lifecycle_hooks_fire_around_the_transports(monkeypatch):
    """`on_detached` has to run while the transports are still open, so a
    plugin can leave hardware in a safe state -- a spinning motor is not
    stopped by closing the port."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    events = []

    monkeypatch.setattr(
        type(plugin),
        "on_attached",
        lambda self, node, bus: events.append("attached"),
        raising=False,
    )
    monkeypatch.setattr(
        type(plugin),
        "on_detached",
        lambda self, node, bus: events.append(
            # every transport must still be usable at this point
            f"detached:{all(t._open for t in self.transports.values())}"
        ),
        raising=False,
    )

    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()
    assert events == ["attached"]
    host.close()
    assert events == ["attached", "detached:True"]

    # Both hooks are idempotent, since close() is
    host.close()
    assert events == ["attached", "detached:True"]


def test_a_raising_detach_hook_does_not_block_teardown():
    """Teardown must finish even if the device has already gone away."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())

    def _boom(self, node, bus):
        raise RuntimeError("device unplugged")

    monkeypatch_target = type(plugin)
    original = getattr(monkeypatch_target, "on_detached", None)
    monkeypatch_target.on_detached = _boom
    try:
        host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
        host.open()
        host.close()  # must not raise
        assert all(not t._open for t in plugin.transports.values())
    finally:
        if original is not None:
            monkeypatch_target.on_detached = original
        else:
            del monkeypatch_target.on_detached


# ---------------------------------------------------------------------------
# Component integration
# ---------------------------------------------------------------------------


@pytest.fixture
def rclpy_context():
    # Other tests in the suite (e.g. launcher tests) may leave the default
    # rclpy context initialized -- Launcher.__init__ does `if not rclpy.ok():
    # rclpy.init()` and never shuts down. Mirror that guard so we tolerate a
    # pre-initialized context and only tear down what we own.
    own = not rclpy.ok()
    if own:
        rclpy.init()
    yield
    if own and rclpy.ok():
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


def test_use_robot_plugin_survives_deactivate_reactivate(rclpy_context):
    """A deactivate/activate cycle must re-bind plugin feedback and commands.

    Regression: ``destroy_all_subscribers`` released the feedback-bus handles
    but ``_external_topics`` was never cleared, so the re-activation
    ``_use_robot_plugin`` skipped re-binding and the component went deaf to
    plugin feedback.
    """
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.robot.adapters import RobotCommandPublisher

    state_port = _free_port()
    cmd_port = _free_port()
    robot_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_cmd_sock.bind(("127.0.0.1", cmd_port))
    robot_cmd_sock.settimeout(2.0)

    plugin = MockPlugin(state_port=state_port, cmd_port=cmd_port)
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()

    component = BaseComponent(
        component_name="reactivate_test_component",
        inputs=[Topic(name="robot_state", msg_type="Int32", use_plugin=True)],
        outputs=[Topic(name="robot_cmd", msg_type="Int32", use_plugin=True)],
    )
    component.rclpy_init_node()
    component._robot_plugin = plugin

    def _feedback_roundtrip(expected: int):
        """Push one telemetry value through the plugin and assert it lands."""
        cb = component.callbacks["robot_state"]
        cb.msg = None
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(str(expected).encode(), ("127.0.0.1", state_port))
        deadline = time.time() + 2.0
        while cb.msg is None and time.time() < deadline:
            time.sleep(0.02)
        tx.close()
        assert cb.msg is not None, "plugin feedback not received"
        assert cb.get_output() == expected

    try:
        # --- first activation ---
        component._use_robot_plugin()
        assert component._external_topics == {"robot_state", "robot_cmd"}
        _feedback_roundtrip(11)

        # --- deactivate ---
        component.destroy_all_subscribers()
        # bus handles released and the plugin-topic set reset
        assert component._robot_plugin_bus_handles == []
        assert component._external_topics == set()

        # --- re-activate ---
        component._use_robot_plugin()
        assert component._external_topics == {"robot_state", "robot_cmd"}

        # Feedback flows again (the regression: it didn't).
        _feedback_roundtrip(22)

        # Command still routes through the plugin adapter.
        assert isinstance(
            component.publishers_dict["robot_cmd"], RobotCommandPublisher
        )
        component.publishers_dict["robot_cmd"].publish(88)
        data, _ = robot_cmd_sock.recvfrom(1024)
        assert data == b"88"
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


def test_use_robot_plugin_component_without_in_topics(rclpy_context):
    """A component that builds ``callbacks`` directly without passing
    ``inputs=`` to ``BaseComponent`` -- like the EmbodiedAgents Memory
    component -- still gets its ``use_plugin`` topics rewired.

    Regression: ``_use_robot_plugin`` used to iterate ``in_topics``, which is
    only populated when ``inputs`` is passed through ``__init__``. Components
    that build ``callbacks`` directly were silently skipped.
    """
    from ros_sugar.core.component import BaseComponent

    state_port = _free_port()
    plugin = MockPlugin(state_port=state_port, cmd_port=_free_port())
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()

    # Memory-style construction: no inputs/outputs at __init__ time.
    component = BaseComponent(component_name="no_in_topics_component")
    component.rclpy_init_node()
    # Precondition: the component genuinely has no in_topics list.
    assert not hasattr(component, "in_topics")

    # Callbacks built directly, the way Memory._layers does it.
    topic = Topic(name="robot_state", msg_type="Int32", use_plugin=True)
    component.callbacks = {
        topic.name: topic.msg_type.callback(topic, node_name=component.node_name)
    }
    component._robot_plugin = plugin
    try:
        component._use_robot_plugin()

        # The use_plugin topic was discovered and bound despite no in_topics.
        assert "robot_state" in component._external_topics

        # Plugin telemetry reaches the (swapped) callback slot.
        robot_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot_tx.sendto(b"77", ("127.0.0.1", state_port))
        callback = component.callbacks["robot_state"]
        deadline = time.time() + 2.0
        while callback.msg is None and time.time() < deadline:
            time.sleep(0.02)
        robot_tx.close()
        assert callback.msg is not None
        assert callback.get_output() == 77
    finally:
        host.close()
        component.destroy_node()


def test_use_robot_plugin_mismatch_is_contained(rclpy_context):
    """A mis-wired plugin topic (type mismatch) is logged and skipped, not
    fatal -- other correctly-wired topics on the same component still bind.

    Regression: ``_resolve_entry`` raises ``TypeError`` on a key match with
    disagreeing types; that used to propagate out of ``_use_robot_plugin``
    and abort the whole component's activation.
    """
    from ros_sugar.core.component import BaseComponent

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()

    component = BaseComponent(
        component_name="mismatch_component",
        inputs=[
            # Correctly wired: resolves to the plugin's Int32 feedback.
            Topic(name="robot_state", msg_type="Int32", use_plugin=True),
            # Mis-wired: name matches the plugin feedback key "Int32" but the
            # declared type disagrees -> _resolve_entry raises TypeError.
            Topic(name="Int32", msg_type="Float64", use_plugin=True),
        ],
    )
    component.rclpy_init_node()
    component._robot_plugin = plugin
    try:
        # Must not raise despite the mis-wired topic.
        component._use_robot_plugin()

        # The valid topic was bound to the plugin...
        assert "robot_state" in component._external_topics
        # ...and the mis-wired one fell back to an ordinary ROS topic.
        assert "Int32" not in component._external_topics
        assert "Int32" in component.callbacks  # still a normal callback slot
    finally:
        host.close()
        component.destroy_node()


# ---------------------------------------------------------------------------
# launcher.robot auto-apply from plugin.robot_config
# ---------------------------------------------------------------------------


class _FakeComponentConfig:
    """Bare component config with the two slots the launcher broadcasts into."""

    def __init__(self):
        from ros_sugar.config import RobotFrames

        self.robot = None
        self.frames = RobotFrames()


class _FakeComponent:
    """Minimum surface the launcher's robot-broadcast path needs."""

    def __init__(self, node_name: str):
        self.node_name = node_name
        self.config = _FakeComponentConfig()


def test_plugin_robot_config_auto_applied_on_bringup_hook():
    """When the recipe doesn't set ``launcher.robot``, the plugin's
    ``robot_config`` is broadcast to every component with a ``config.robot``
    slot."""
    from ros_sugar import Launcher

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    plugin.robot_config = {"sentinel": "from_plugin"}

    launcher = Launcher(robot_plugin=plugin)
    comp_a = _FakeComponent("a")
    comp_b = _FakeComponent("b")
    launcher._components = [comp_a, comp_b]

    launcher._apply_plugin_robot_config()

    assert comp_a.config.robot == {"sentinel": "from_plugin"}
    assert comp_b.config.robot == {"sentinel": "from_plugin"}


def test_recipe_override_wins_over_plugin():
    """An explicit ``launcher.robot = ...`` sets the sentinel and the
    auto-apply step at bringup is a no-op (recipe wins)."""
    from ros_sugar import Launcher

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    plugin.robot_config = {"sentinel": "from_plugin"}

    launcher = Launcher(robot_plugin=plugin)
    comp = _FakeComponent("c")
    launcher._components = [comp]

    launcher.robot = {"sentinel": "from_recipe"}
    assert launcher._robot_explicitly_set is True
    assert comp.config.robot == {"sentinel": "from_recipe"}

    launcher._apply_plugin_robot_config()  # would normally fire at bringup
    assert comp.config.robot == {"sentinel": "from_recipe"}  # untouched


def test_plugin_that_describes_no_robot_is_noop():
    """Every plugin declares ``robot_config``, but one that describes no robot
    leaves it None -- the auto-apply path stays opt-in."""
    from ros_sugar import Launcher

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    assert plugin.robot_config is None

    launcher = Launcher(robot_plugin=plugin)
    comp = _FakeComponent("d")
    launcher._components = [comp]

    launcher._apply_plugin_robot_config()
    assert comp.config.robot is None


def test_no_plugin_attached_is_noop():
    """Launcher with no plugin attached -- auto-apply is a no-op."""
    from ros_sugar import Launcher

    launcher = Launcher()
    comp = _FakeComponent("e")
    launcher._components = [comp]

    launcher._apply_plugin_robot_config()
    assert comp.config.robot is None


# ---------------------------------------------------------------------------
# component events on plugin-fed topics
# ---------------------------------------------------------------------------


def _event_component(plugin, topic, name):
    """A component whose event watches a topic served by the plugin."""
    from ros_sugar.core.action import Action
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.core.event import Event

    component = BaseComponent(component_name=name, inputs=[topic])
    component.rclpy_init_node()
    component._robot_plugin = plugin
    component._use_robot_plugin()
    component._add_event_action_pair(
        Event(topic.msg.data > 10), Action(method=lambda: None)
    )
    return component


def test_event_on_plugin_topic_creates_no_ros_subscription(rclpy_context):
    """The data never travels over ROS, so a ROS subscription would sit empty."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()

    topic = Topic(name="robot_state", msg_type="Int32", use_plugin=True)
    component = _event_component(plugin, topic, "plugin_event_no_sub")
    try:
        component._turn_on_events_management()
        assert component._external_topics == {"robot_state"}
        assert component._BaseComponent__event_listeners == [], (
            "a plugin-fed event topic must not get a dead ROS subscription"
        )
    finally:
        host.close()
        component.destroy_node()


def test_event_on_plugin_topic_fires_from_feedback_bus(rclpy_context):
    """Regression: events on plugin feedback never fired, because the component
    only ever subscribed to ROS."""
    state_port = _free_port()
    plugin = MockPlugin(state_port=state_port, cmd_port=_free_port())
    host = RobotPluginHost(plugin, node=None, bus=InProcessFeedbackBus())
    host.open()

    topic = Topic(name="robot_state", msg_type="Int32", use_plugin=True)
    component = _event_component(plugin, topic, "plugin_event_bus")
    try:
        component._turn_on_events_management()

        robot_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot_tx.sendto(b"55", ("127.0.0.1", state_port))
        deadline = time.time() + 2.0
        while (
            "robot_state" not in component._events_topics_blackboard
            and time.time() < deadline
        ):
            time.sleep(0.02)
        robot_tx.close()

        entry = component._events_topics_blackboard.get("robot_state")
        assert entry is not None, "plugin feedback never reached the event handler"
        assert entry.msg.data == 55
    finally:
        host.close()
        component.destroy_node()


def test_event_on_a_sensor_topic_binds_to_that_sensor(rclpy_context):
    """Regression: the event path resolved against the robot plugin whatever
    the topic named, so a sensor's event fired on the robot's telemetry."""
    from ros_sugar.core.action import Action
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.core.event import Event

    robot_port, cam_port = _free_port(), _free_port()
    robot = MockPlugin(state_port=robot_port, cmd_port=_free_port(), id="lite3")
    camera = MockUdpSensor(state_port=cam_port, id="front_cam")
    robot._bind_identity()
    camera._bind_identity()

    bus = InProcessFeedbackBus()
    bus.start()
    hosts = [
        RobotPluginHost(p, node=None, bus=bus, owns_bus=False) for p in (robot, camera)
    ]
    for host in hosts:
        host.open()

    topic = Topic(name="cam_state", msg_type="Int32", use_plugin=camera.id)
    component = BaseComponent(component_name="sensor_event", inputs=[topic])
    component.rclpy_init_node()
    component.add_plugin(robot)
    component.add_plugin(camera)
    try:
        component._use_robot_plugin()
        component._add_event_action_pair(
            Event(topic.msg.data > 10), Action(method=lambda: None)
        )
        component._turn_on_events_management()

        # Only the robot transmits. Resolving against the robot plugin would
        # deliver its telemetry to the camera's event.
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"55", ("127.0.0.1", robot_port))
        time.sleep(0.5)
        assert component._events_topics_blackboard.get("cam_state") is None, (
            "the camera's event received the robot's telemetry"
        )

        # The camera's own data does reach it
        tx.sendto(b"22", ("127.0.0.1", cam_port))
        deadline = time.time() + 2.0
        while (
            "cam_state" not in component._events_topics_blackboard
            and time.time() < deadline
        ):
            time.sleep(0.02)
        tx.close()

        entry = component._events_topics_blackboard.get("cam_state")
        assert entry is not None, "the camera's feedback never reached its event"
        assert entry.msg.data == 22
    finally:
        for host in hosts:
            host.close()
        bus.close()
        component.destroy_node()


def test_event_on_a_sensor_topic_binds_with_no_robot_plugin(rclpy_context):
    """A sensor-only recipe has no robot plugin, which used to mean the event
    path returned early and wired up nothing at all -- silently."""
    from ros_sugar.core.action import Action
    from ros_sugar.core.component import BaseComponent
    from ros_sugar.core.event import Event

    cam_port = _free_port()
    camera = MockUdpSensor(state_port=cam_port, id="front_cam")
    camera._bind_identity()

    bus = InProcessFeedbackBus()
    bus.start()
    host = RobotPluginHost(camera, node=None, bus=bus, owns_bus=False)
    host.open()

    topic = Topic(name="cam_state", msg_type="Int32", use_plugin=camera.id)
    component = BaseComponent(component_name="sensor_only_event", inputs=[topic])
    component.rclpy_init_node()
    component.add_plugin(camera)
    try:
        assert component._robot_plugin is None, "no robot plugin in this recipe"
        component._use_robot_plugin()
        component._add_event_action_pair(
            Event(topic.msg.data > 10), Action(method=lambda: None)
        )
        component._turn_on_events_management()

        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"22", ("127.0.0.1", cam_port))
        deadline = time.time() + 2.0
        while (
            "cam_state" not in component._events_topics_blackboard
            and time.time() < deadline
        ):
            time.sleep(0.02)
        tx.close()

        entry = component._events_topics_blackboard.get("cam_state")
        assert entry is not None, "sensor event got no listeners at all"
        assert entry.msg.data == 22
    finally:
        host.close()
        bus.close()
        component.destroy_node()


# ---------------------------------------------------------------------------
# plugin-supplied robot description
# ---------------------------------------------------------------------------


class _DescribingPlugin(RobotPlugin):
    """A plugin that knows the robot it drives."""

    def __init__(self):
        from ros_sugar.config import (
            AngularCtrlLimits,
            LinearCtrlLimits,
            RobotConfig,
            RobotGeometryType,
            RobotType,
        )
        import numpy as np

        self.metadata = PluginMetadata(name="DescribingPlugin")
        self.robot_config = RobotConfig(
            model_type=RobotType.DIFFERENTIAL_DRIVE,
            geometry_type=RobotGeometryType.BOX,
            geometry_params=np.array([0.61, 0.37, 0.4]),
            ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=2.5, max_decel=7.5),
            ctrl_omega_limits=AngularCtrlLimits(
                max_vel=1.5, max_acc=2.5, max_decel=4.0, max_steer=1.57
            ),
        )
        self.base_frame = "lite3_base"


def test_plugin_describes_robot_via_declared_fields():
    """Both are declared members, so a plugin author has something to populate
    and the launcher has something to type against."""
    plugin = _DescribingPlugin()
    assert plugin.robot_config is not None
    assert plugin.base_frame == "lite3_base"

    bare = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    assert bare.robot_config is None and bare.base_frame is None


def test_plugin_base_frame_applied_but_not_world_frame():
    """A robot knows its own body frame; where it has been placed is not its
    call, so the world frame must be left alone."""
    from ros_sugar import Launcher

    launcher = Launcher(robot_plugin=_DescribingPlugin())
    comp = _FakeComponent("base_frame_component")
    launcher._components = [comp]

    launcher._apply_plugin_robot_config()
    launcher._apply_plugin_base_frame()

    assert comp.config.robot is not None
    assert comp.config.frames.robot_base == "lite3_base"
    assert comp.config.frames.world == "map", "world frame is not the robot's to set"


def test_plugin_base_frame_wins_over_recipe():
    """The plugin stamps its telemetry in its own body frame and mounts are
    parented on it, so a recipe cannot rename it from the outside."""
    from ros_sugar import Launcher
    from ros_sugar.config import RobotFrames

    launcher = Launcher(robot_plugin=_DescribingPlugin())
    comp = _FakeComponent("plugin_wins_component")
    launcher._components = [comp]

    launcher.frames = RobotFrames(robot_base="from_recipe", world="office")
    launcher._apply_plugin_base_frame()  # would normally fire at bringup

    assert comp.config.frames.robot_base == "lite3_base"
    assert comp.config.frames.world == "office", "world is still the recipe's"


def test_naming_only_the_world_frame_keeps_the_plugin_base_frame():
    """The regression this precedence exists for: a recipe that assigns
    `frames` to choose a world frame used to silently drop the plugin's body
    frame, leaving mounts parented on a frame nothing looked up."""
    from ros_sugar import Launcher
    from ros_sugar.config import RobotFrames

    launcher = Launcher(robot_plugin=_DescribingPlugin())
    comp = _FakeComponent("world_only_component")
    launcher._components = [comp]

    launcher.frames = RobotFrames(world="odom")
    launcher._apply_plugin_base_frame()

    assert comp.config.frames.robot_base == "lite3_base"
    assert comp.config.frames.world == "odom"


def test_renaming_the_frame_on_the_plugin_is_the_way_to_override():
    """The escape hatch: rename it where telemetry and mounts both read it."""
    from ros_sugar import Launcher

    plugin = _DescribingPlugin()
    plugin.base_frame = "base_link"
    launcher = Launcher(robot_plugin=plugin)
    comp = _FakeComponent("renamed_frame_component")
    launcher._components = [comp]

    launcher._apply_plugin_base_frame()

    assert comp.config.frames.robot_base == "base_link"


def test_world_frame_setter_leaves_the_robot_frame_alone():
    from ros_sugar import Launcher

    launcher = Launcher(robot_plugin=_DescribingPlugin())
    comp = _FakeComponent("world_setter_component")
    launcher._components = [comp]

    launcher.world_frame = "odom"
    launcher._apply_plugin_base_frame()

    assert comp.config.frames.world == "odom"
    assert comp.config.frames.robot_base == "lite3_base"
    assert launcher.world_frame == {"world_setter_component": "odom"}


def test_robot_frame_setter_applies_without_a_plugin():
    from ros_sugar import Launcher

    launcher = Launcher()
    comp = _FakeComponent("robot_setter_component")
    launcher._components = [comp]

    launcher.robot_frame = "chassis"
    launcher._apply_plugin_base_frame()  # no plugin attached, so a no-op

    assert comp.config.frames.robot_base == "chassis"
    assert launcher.robot_frame == {"robot_setter_component": "chassis"}


@pytest.mark.parametrize("bad", ["", None, 3])
def test_frame_setters_reject_non_frame_names(bad):
    from ros_sugar import Launcher

    launcher = Launcher()
    launcher._components = [_FakeComponent("bad_frame_component")]

    with pytest.raises(ValueError):
        launcher.world_frame = bad


def test_plugin_without_base_frame_leaves_frames_untouched():
    from ros_sugar import Launcher

    launcher = Launcher(robot_plugin=MockPlugin(
        state_port=_free_port(), cmd_port=_free_port()
    ))
    comp = _FakeComponent("no_base_frame_component")
    launcher._components = [comp]

    launcher._apply_plugin_base_frame()

    assert comp.config.frames.robot_base == "base_link"


# ---------------------------------------------------------------------------
# plugin taxonomy
# ---------------------------------------------------------------------------


def test_role_comes_from_the_base_class_not_the_recipe():
    """Role is intrinsic: a plugin *is* a robot or a sensor by construction, so
    a recipe cannot attach one under the wrong role."""
    from ros_sugar.robot import Plugin, PluginRole, RobotPlugin, SensorPlugin

    assert RobotPlugin._role is PluginRole.ROBOT
    assert SensorPlugin._role is PluginRole.SENSOR
    assert Plugin._role is PluginRole.SENSOR

    class MyRobot(RobotPlugin):
        pass

    class MyCamera(SensorPlugin):
        pass

    assert MyRobot().role is PluginRole.ROBOT
    assert MyCamera().role is PluginRole.SENSOR

    # read-only: a recipe cannot reassign what a plugin is
    with pytest.raises(AttributeError):
        MyCamera().role = PluginRole.ROBOT


def test_only_robot_plugins_describe_a_robot():
    """A camera has no geometry or velocity envelope of its own, so those slots
    exist only where they mean something."""
    from ros_sugar.robot import RobotPlugin, SensorPlugin

    class MyRobot(RobotPlugin):
        pass

    class MyCamera(SensorPlugin):
        pass

    robot = MyRobot()
    assert robot.robot_config is None and robot.base_frame is None

    camera = MyCamera()
    assert not hasattr(camera, "robot_config")
    assert not hasattr(camera, "base_frame")


def test_subclass_without_its_own_init_still_reports_its_own_name():
    """Regression guard for the __init_subclass__ move: the framework's own
    base classes must stay unwrapped, or a plugin that defines no ``__init__``
    would be initialized against its base and take the base's name."""
    from ros_sugar.robot import RobotPlugin, SensorPlugin

    class BareRobot(RobotPlugin):
        pass

    class BareSensor(SensorPlugin):
        pass

    assert BareRobot().metadata.name == "BareRobot"
    assert BareSensor().metadata.name == "BareSensor"


def test_describe_reports_the_role():
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    assert plugin.describe()["role"] == "robot"


def test_role_survives_the_spec_round_trip():
    """Components rebuild plugins from a spec in their own process; the role
    has to come back with them."""
    from ros_sugar.robot import Plugin, PluginRole

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    rebuilt = Plugin.from_spec(plugin.to_spec())
    assert rebuilt.role is PluginRole.ROBOT


# ---------------------------------------------------------------------------
# shared bus lifecycle
# ---------------------------------------------------------------------------


def test_second_host_on_a_shared_bus_opens(rclpy_context):
    """Regression: with each host starting the bus itself, the second one
    re-bound the same address and died with 'Address already in use'."""
    bus = SocketFeedbackBus()
    bus.start()
    hosts = []
    try:
        for _ in range(2):
            plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
            host = RobotPluginHost(plugin, node=None, bus=bus, owns_bus=False)
            host.open()
            hosts.append(host)
        assert all(h._active for h in hosts)
    finally:
        for host in hosts:
            host.close()
        bus.close()


def test_one_host_closing_leaves_the_shared_bus_up(rclpy_context):
    """Regression: the first host to close tore the bus down for every other
    plugin still using it."""
    state_port = _free_port()
    bus = InProcessFeedbackBus()
    bus.start()

    first = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    second = MockPlugin(state_port=state_port, cmd_port=_free_port())
    host_a = RobotPluginHost(first, node=None, bus=bus, owns_bus=False)
    host_b = RobotPluginHost(second, node=None, bus=bus, owns_bus=False)
    host_a.open()
    host_b.open()

    received = []
    handle = bus.subscribe(second.feedbacks["Int32"].channel, received.append)
    try:
        host_a.close()  # the other plugin must keep working

        robot_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot_tx.sendto(b"77", ("127.0.0.1", state_port))
        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            time.sleep(0.02)
        robot_tx.close()
        assert received, "closing one host silenced the surviving plugin"
    finally:
        handle.unsubscribe()
        host_b.close()
        bus.close()


def test_standalone_host_still_owns_its_bus(rclpy_context):
    """A lone host with no launcher around it must remain self-contained."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()
    assert host._active
    host.close()
    assert not host._active


# ---------------------------------------------------------------------------
# plugin identity
# ---------------------------------------------------------------------------


def test_unattached_plugin_keeps_the_bare_channel_form():
    """A plugin that was never attached has no id, and its channels stay in the
    form a single-plugin recipe has always used."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    assert plugin.feedbacks["Int32"].channel == "robot/feedback/Int32"
    assert plugin.commands["Int32"].channel == "robot/command/Int32"


def test_binding_identity_namespaces_every_channel():
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="front_cam")
    plugin._bind_identity()

    assert plugin.id == "front_cam"
    assert plugin.feedbacks["Int32"].channel == "plugin/front_cam/feedback/Int32"
    assert plugin.commands["Int32"].channel == "plugin/front_cam/command/Int32"


def test_two_plugins_of_the_same_class_do_not_collide():
    """The whole point: several plugins share one bus, so identical feedback
    keys must resolve to different channels."""
    a = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="cam_a")
    b = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="cam_b")
    a._bind_identity()
    b._bind_identity()

    assert a.feedbacks["Int32"].channel != b.feedbacks["Int32"].channel
    assert a.commands["Int32"].channel != b.commands["Int32"].channel


def test_namespaced_channel_is_a_valid_ros_topic_name():
    """Channels become ROS topic names for events, and an invalid one fails at
    activation rather than when the recipe runs."""
    from rclpy.validate_topic_name import validate_topic_name

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="front_cam")
    plugin._bind_identity()
    validate_topic_name(f"/{plugin.feedbacks['Int32'].channel}")


@pytest.mark.parametrize("bad_id", ["front-cam", "office.cam", "2nd_cam", ""])
def test_ids_that_are_not_valid_ros_tokens_are_rejected(bad_id):
    with pytest.raises(ValueError, match="not a usable plugin id"):
        MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id=bad_id)


def test_rebinding_after_a_topic_was_handed_out_raises():
    """Silent-failure guard: an event built before attaching embeds the old
    channel name, and the Monitor blackboard is keyed by topic name -- so a
    later rename would leave the event listening to nothing, with no error."""
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="front_cam")
    plugin.feedbacks["Int32"].as_topic()  # what building an event does

    with pytest.raises(ValueError, match="has already been referenced"):
        plugin._bind_identity()


def test_binding_the_same_identity_twice_is_allowed():
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="front_cam")
    plugin._bind_identity()
    plugin.feedbacks["Int32"].as_topic()
    plugin._bind_identity()  # no-op, must not raise


def test_identity_survives_the_spec_round_trip():
    """The component subprocess must subscribe to the same channels the host
    publishes on."""
    from ros_sugar.robot import Plugin

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="front_cam")
    plugin._bind_identity()

    rebuilt = Plugin.from_spec(plugin.to_spec())
    assert rebuilt.id == "front_cam"
    assert rebuilt.feedbacks["Int32"].channel == plugin.feedbacks["Int32"].channel


@pytest.mark.parametrize(
    "name, expected",
    [
        ("Lite3", "lite3"),
        ("HikVision PTZ-2", "hikvision_ptz_2"),
        ("  spaced  name  ", "spaced_name"),
        ("2nd camera", "_2nd_camera"),
    ],
)
def test_display_names_become_usable_ids(name, expected):
    from ros_sugar.robot.plugin import _slug

    assert _slug(name) == expected


# ---------------------------------------------------------------------------
# component holds several plugins
# ---------------------------------------------------------------------------


def _sensor_plugin(name: str) -> "MockSensor":
    return MockSensor(cam_name=name)


def test_component_keeps_plugins_in_a_dict(rclpy_context):
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="multi_plugin_component")
    component.rclpy_init_node()
    try:
        robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
        camera = _sensor_plugin("front_cam")
        component.add_plugin(robot)
        component.add_plugin(camera)

        assert set(component._plugins) == {"mockplugin", "front_cam"}
    finally:
        component.destroy_node()


def test_robot_plugin_property_reads_and_writes_through(rclpy_context):
    """Existing code and tests assign ``component._robot_plugin`` directly, and
    the out-of-tree Lite3 republisher reads it."""
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="robot_plugin_property_component")
    component.rclpy_init_node()
    try:
        assert component._robot_plugin is None

        robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
        component._robot_plugin = robot
        assert component._robot_plugin is robot

        component._robot_plugin = None
        assert component._robot_plugin is None
        assert component._plugins == {}
    finally:
        component.destroy_node()


def test_setting_the_robot_plugin_leaves_sensors_attached(rclpy_context):
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="mixed_plugin_component")
    component.rclpy_init_node()
    try:
        camera = _sensor_plugin("front_cam")
        component.add_plugin(camera)
        component._robot_plugin = MockPlugin(
            state_port=_free_port(), cmd_port=_free_port()
        )
        component._robot_plugin = None  # clearing the robot must not drop the camera

        assert component._plugins == {"front_cam": camera}
    finally:
        component.destroy_node()


def test_a_sensor_only_component_still_binds_its_plugins(rclpy_context):
    """The plugin gates used to key on the *robot* plugin, so a recipe with
    only sensors would have skipped plugin wiring entirely."""
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="sensor_only_component")
    component.rclpy_init_node()
    try:
        component.add_plugin(_sensor_plugin("front_cam"))
        assert component._robot_plugin is None
        assert component._plugins, "gates must key on _plugins, not the robot plugin"
        assert component._plugins_json != "{}"
    finally:
        component.destroy_node()


def test_plugins_round_trip_across_the_process_boundary(rclpy_context):
    """Multiprocess launch serializes plugins into argv and rebuilds them in
    the component subprocess."""
    from ros_sugar.core.component import BaseComponent

    sender = BaseComponent(component_name="plugins_json_sender")
    sender.rclpy_init_node()
    receiver = BaseComponent(component_name="plugins_json_receiver")
    receiver.rclpy_init_node()
    try:
        robot = MockPlugin(
            state_port=_free_port(), cmd_port=_free_port(), id="lite3"
        )
        robot._bind_identity()
        sender.add_plugin(robot)
        sender.add_plugin(MockSensor(cam_name="HikVision", id="front_cam"))

        receiver._plugins_json = sender._plugins_json

        assert set(receiver._plugins) == {"lite3", "front_cam"}
        assert receiver._robot_plugin is not None
        # the identity has to come back, or the subprocess subscribes to
        # channels the host never publishes on
        assert receiver._robot_plugin.id == "lite3"
        assert (
            receiver._robot_plugin.feedbacks["Int32"].channel
            == "plugin/lite3/feedback/Int32"
        )
    finally:
        sender.destroy_node()
        receiver.destroy_node()


# ---------------------------------------------------------------------------
# launcher holds several plugins
# ---------------------------------------------------------------------------


def _launcher_with(components):
    from ros_sugar import Launcher

    launcher = Launcher()
    launcher._components = components
    return launcher


def test_add_plugin_order_does_not_matter(rclpy_context):
    """Regression: plugins used to be handed out inside add_pkg, so a recipe
    that called add_pkg first left those components with nothing."""
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="ordering_component")
    component.rclpy_init_node()
    try:
        # components registered BEFORE any plugin is attached
        launcher = _launcher_with([component])
        launcher.add_plugin(MockPlugin(state_port=_free_port(), cmd_port=_free_port()))
        launcher.add_plugin(MockSensor(cam_name="front_cam"))

        assert component._plugins == {}, "distribution happens at bringup"
        launcher._distribute_plugins()
        assert set(component._plugins) == {"mockplugin", "front_cam"}
    finally:
        component.destroy_node()


def test_only_one_robot_plugin_per_recipe(rclpy_context):
    launcher = _launcher_with([])
    launcher.add_plugin(MockPlugin(state_port=_free_port(), cmd_port=_free_port()))

    with pytest.raises(ValueError, match="a recipe describes one robot"):
        launcher.add_plugin(
            MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="second")
        )


def test_several_sensor_plugins_are_allowed(rclpy_context):
    launcher = _launcher_with([])
    launcher.add_plugin(MockSensor(cam_name="cam_a"))
    launcher.add_plugin(MockSensor(cam_name="cam_b"))

    assert set(launcher._plugins) == {"cam_a", "cam_b"}


def test_duplicate_names_are_rejected(rclpy_context):
    launcher = _launcher_with([])
    launcher.add_plugin(MockSensor(cam_name="cam"))

    with pytest.raises(ValueError, match="already taken"):
        launcher.add_plugin(MockSensor(cam_name="cam"))


def test_two_plugins_of_one_kind_need_distinct_names(rclpy_context):
    """Two identical cameras default to the same slug, so the recipe names
    them -- and that name is what namespaces their channels."""
    launcher = _launcher_with([])
    launcher.add_plugin(MockSensor(cam_name="HikVision PTZ", id="front_cam"))
    launcher.add_plugin(MockSensor(cam_name="HikVision PTZ", id="rear_cam"))

    assert set(launcher._plugins) == {"front_cam", "rear_cam"}
    assert launcher._plugins["front_cam"].id == "front_cam"
    assert launcher._plugins["rear_cam"].id == "rear_cam"


def test_attaching_binds_identity_immediately(rclpy_context):
    """Identity must be final when add_plugin returns: events built from the
    plugin freeze their topic names before plugin setup runs."""
    launcher = _launcher_with([])
    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    launcher.add_plugin(plugin)

    assert plugin.id == "mockplugin"
    assert plugin.feedbacks["Int32"].channel == "plugin/mockplugin/feedback/Int32"


def test_launcher_robot_plugin_property_round_trips(rclpy_context):
    """ros-agents reads and writes ``launcher._robot_plugin`` directly."""
    from ros_sugar import Launcher

    plugin = MockPlugin(state_port=_free_port(), cmd_port=_free_port())
    launcher = Launcher(robot_plugin=plugin)
    assert launcher._robot_plugin is plugin

    launcher._robot_plugin = None
    assert launcher._robot_plugin is None


# ---------------------------------------------------------------------------
# addressing a specific plugin from a topic
# ---------------------------------------------------------------------------


def test_topic_rejects_an_empty_plugin_id():
    """`use_plugin` is tested for truthiness, so "" would silently mean off."""
    with pytest.raises(ValueError, match="cannot be an empty string"):
        Topic(name="x", msg_type="Int32", use_plugin="")


def test_use_plugin_survives_the_topic_round_trip():
    """Topics are rebuilt from JSON in every component subprocess."""
    original = Topic(name="x", msg_type="Int32", use_plugin="front_cam")
    rebuilt = Topic(**{
        **json.loads(original.to_json()),
        "qos_profile": original.qos_profile,
        "additional_types": [],
    })
    assert rebuilt.use_plugin == "front_cam"

    plain = Topic(name="y", msg_type="Int32", use_plugin=True)
    assert json.loads(plain.to_json())["use_plugin"] is True


def test_component_binds_each_topic_to_the_named_plugin(rclpy_context):
    """The point of step 8: two plugins attached, each topic reaching the one
    it names."""
    from ros_sugar.core.component import BaseComponent

    robot_port, cam_port = _free_port(), _free_port()
    robot = MockPlugin(state_port=robot_port, cmd_port=_free_port(), id="lite3")
    camera = MockPlugin(state_port=cam_port, cmd_port=_free_port(), id="front_cam")

    # what Launcher.add_plugin does: namespace each plugin's channels so two
    # plugins exposing the same feedback key cannot collide on the shared bus
    robot._bind_identity()
    camera._bind_identity()

    bus = InProcessFeedbackBus()
    bus.start()
    hosts = [
        RobotPluginHost(p, node=None, bus=bus, owns_bus=False) for p in (robot, camera)
    ]
    for host in hosts:
        host.open()

    component = BaseComponent(
        component_name="two_plugin_component",
        inputs=[
            Topic(name="robot_state", msg_type="Int32", use_plugin=True),
            Topic(name="cam_state", msg_type="Int32", use_plugin=camera.id),
        ],
    )
    component.rclpy_init_node()
    component.add_plugin(robot)
    component.add_plugin(camera)
    try:
        component._use_robot_plugin()
        assert component._external_topics == {"robot_state", "cam_state"}

        # each input receives only its own plugin's telemetry
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"11", ("127.0.0.1", robot_port))
        tx.sendto(b"22", ("127.0.0.1", cam_port))
        deadline = time.time() + 2.0
        while (
            component.callbacks["robot_state"].msg is None
            or component.callbacks["cam_state"].msg is None
        ) and time.time() < deadline:
            time.sleep(0.02)
        tx.close()

        assert component.callbacks["robot_state"].get_output() == 11
        assert component.callbacks["cam_state"].get_output() == 22
    finally:
        for host in hosts:
            host.close()
        bus.close()
        component.destroy_node()


def test_topic_naming_an_unattached_plugin_is_caught_at_bringup(rclpy_context):
    """Falling back to a ROS topic nothing publishes leaves the component
    looking healthy while receiving nothing, so fail in the recipe instead."""
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(
        component_name="typo_component",
        inputs=[Topic(name="Image", msg_type="Int32", use_plugin="front_cm")],
    )
    component.rclpy_init_node()
    try:
        launcher = _launcher_with([component])
        launcher.add_plugin(MockSensor(cam_name="HikVision", id="front_cam"))

        with pytest.raises(ValueError, match="not attached to this recipe"):
            launcher._validate_plugin_references()
    finally:
        component.destroy_node()


def test_use_plugin_true_without_a_robot_plugin_is_caught_at_bringup(rclpy_context):
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(
        component_name="no_robot_component",
        inputs=[Topic(name="Image", msg_type="Int32", use_plugin=True)],
    )
    component.rclpy_init_node()
    try:
        launcher = _launcher_with([component])
        launcher.add_plugin(MockSensor(cam_name="HikVision", id="front_cam"))

        with pytest.raises(ValueError, match="no robot plugin is attached"):
            launcher._validate_plugin_references()
    finally:
        component.destroy_node()


# ---------------------------------------------------------------------------
# sensor frames and mounts
# ---------------------------------------------------------------------------


def test_sensor_frame_defaults_to_its_id():
    """Two instances of one sensor get distinct frames with no author effort."""
    a = MockSensor(cam_name="HikVision", id="front_cam")
    b = MockSensor(cam_name="HikVision", id="rear_cam")
    assert a.frame_id == "front_cam_frame"
    assert b.frame_id == "rear_cam_frame"


def test_sensor_frame_can_be_named_by_the_recipe():
    cam = MockSensor(cam_name="HikVision", id="front_cam", frame_id="front_optical")
    assert cam.frame_id == "front_optical"


def test_sensor_frame_survives_the_spec_round_trip():
    from ros_sugar.robot import Plugin

    cam = MockSensor(cam_name="HikVision", id="front_cam", frame_id="front_optical")
    rebuilt = Plugin.from_spec(cam.to_spec())
    assert rebuilt.frame_id == "front_optical"


def test_unstamped_feedback_is_stamped_with_the_plugin_frame(rclpy_context):
    """Regression: an unstamped message is silently never transformed -- the
    component's frame resolver returns None on an empty frame_id."""
    from std_msgs.msg import Header
    from sensor_msgs.msg import Imu

    port = _free_port()

    def _decode_imu(raw: bytes):
        msg = Imu()
        msg.header = Header()  # deliberately unstamped, as a decoder often is
        msg.linear_acceleration.x = float(raw.decode())
        return msg

    RobotImu = create_supported_type(Imu)

    class ImuSensor(SensorPlugin):
        def __init__(self, imu_port: int):
            self.metadata = PluginMetadata(name="IMU")
            transport = UdpTransport(
                "imu", send_to=("127.0.0.1", imu_port), bind=("127.0.0.1", imu_port)
            )
            self.transports = {"imu": transport}
            self.feedbacks = {
                "Imu": Feedback(
                    key="Imu",
                    msg_type=RobotImu,
                    transport=transport,
                    decoder=_decode_imu,
                )
            }

    sensor = ImuSensor(imu_port=port, id="waist_imu")
    sensor._bind_identity()
    bus = InProcessFeedbackBus()
    bus.start()
    host = RobotPluginHost(sensor, node=None, bus=bus, owns_bus=False)
    host.open()

    received = []
    handle = bus.subscribe(sensor.feedbacks["Imu"].channel, received.append)
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"9", ("127.0.0.1", port))
        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            time.sleep(0.02)
        tx.close()
        assert received, "no feedback arrived"

        from rclpy.serialization import deserialize_message

        msg = deserialize_message(received[0], Imu)
        assert msg.header.frame_id == "waist_imu_frame"
    finally:
        handle.unsubscribe()
        host.close()
        bus.close()


def test_a_decoder_that_stamps_its_own_frame_is_never_overridden(rclpy_context):
    """A robot's odometry is in the localization frame, not a body frame --
    only the decoder knows that, so the framework must not overwrite it."""
    from sensor_msgs.msg import Imu

    RobotImu2 = create_supported_type(Imu, module=__name__)

    feedback = Feedback(
        key="Imu",
        msg_type=RobotImu2,
        transport=UdpTransport("x", send_to=("127.0.0.1", 1)),
        decoder=lambda _: None,
        frame_id="declared_frame",
    )
    host = RobotPluginHost(
        MockSensor(cam_name="s", id="s"), node=None, bus=InProcessFeedbackBus()
    )

    msg = Imu()
    msg.header.frame_id = "author_stamped"
    host._stamp_frame(feedback, msg)
    assert msg.header.frame_id == "author_stamped"

    fresh = Imu()
    host._stamp_frame(feedback, fresh)
    assert fresh.header.frame_id == "declared_frame", "per-feedback frame wins"


# ---- Mount ----------------------------------------------------------------


def test_mount_resolves_frames_from_plugins():
    from ros_sugar.robot import Mount

    robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="lite3")
    robot.base_frame = "base_link"
    cam = MockSensor(cam_name="HikVision", id="front_cam")

    mount = Mount(parent=robot, xyz=(0.15, 0.0, 0.35))
    mount.child = cam
    assert mount.parent_frame == "base_link"
    assert mount.child_frame == "front_cam_frame"


def test_mount_accepts_a_plain_frame_for_a_fixed_sensor():
    """A sensor watching a room is mounted on the world, not on the robot."""
    from ros_sugar.robot import Mount

    cam = MockSensor(cam_name="Lobby", id="lobby_cam")
    mount = Mount(parent="map", xyz=(2.0, 3.0, 2.5))
    mount.child = cam
    assert mount.parent_frame == "map"


def test_mount_rejects_something_with_no_frame():
    from ros_sugar.robot import Mount

    with pytest.raises(ValueError, match="Cannot mount against"):
        Mount(parent=object()).parent_frame


def test_euler_to_quaternion_matches_known_rotations():
    from ros_sugar.robot.mount import quaternion_from_euler

    x, y, z, w = quaternion_from_euler(0.0, 0.0, 0.0)
    assert (x, y, z, w) == (0.0, 0.0, 0.0, 1.0)

    # 90 degrees about Z
    x, y, z, w = quaternion_from_euler(0.0, 0.0, 3.141592653589793 / 2)
    assert abs(z - 0.7071067811865476) < 1e-9
    assert abs(w - 0.7071067811865476) < 1e-9


def test_launcher_collects_mounts_from_add_plugin(rclpy_context):
    from ros_sugar.robot import Mount

    robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="lite3")
    robot.base_frame = "base_link"
    cam = MockSensor(cam_name="HikVision", id="front_cam")

    launcher = _launcher_with([])
    launcher.add_plugin(robot)
    launcher.add_plugin(cam, mount=Mount(parent=robot, xyz=(0.15, 0.0, 0.35)))

    assert len(launcher._mounts) == 1
    mount = launcher._mounts[0]
    assert mount.parent_frame == "base_link"
    # the child is filled in from the plugin, so a recipe never repeats it
    assert mount.child_frame == "front_cam_frame"


def test_a_sensor_needs_no_mount_when_tf_already_has_its_frame(rclpy_context):
    """Mode B: a URDF or the sensor's own driver already publishes the frame,
    so the recipe declares nothing and Sugarcoat just consumes it."""
    cam = MockSensor(cam_name="HikVision", id="front_cam")
    launcher = _launcher_with([])
    launcher.add_plugin(cam)

    assert launcher._mounts == []


# ---------------------------------------------------------------------------
# launcher-side plugin bring-up
# ---------------------------------------------------------------------------


class _StubMonitor:
    """Stands in for the Monitor node in ``_setup_plugins``.

    Only the two methods the launcher wires plugin feedback through are
    needed; building a real Monitor would drag in the whole launch machinery.
    """

    def __init__(self):
        self.registered = []

    def register_external_topic(self, topic):
        self.registered.append(topic.name)

    def feed_external_topic(self, name, msg):
        pass


def test_setup_plugins_hosts_every_plugin_on_one_bus(rclpy_context):
    """Each plugin needs its own host -- otherwise a sensor plugin's transports
    are never opened and it is attached in name only."""
    launcher = _launcher_with([])
    launcher.monitor_node = _StubMonitor()

    robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="lite3")
    camera = MockUdpSensor(state_port=_free_port(), id="front_cam")
    launcher.add_plugin(robot)
    launcher.add_plugin(camera)

    try:
        launcher._setup_plugins()

        assert len(launcher._plugin_hosts) == 2
        assert all(host._active for host in launcher._plugin_hosts)
        # one bus, owned by the launcher rather than by any host
        assert launcher._plugin_bus is not None
        assert all(not host._owns_bus for host in launcher._plugin_hosts)
        assert all(
            host.bus is launcher._plugin_bus for host in launcher._plugin_hosts
        )
    finally:
        for host in launcher._plugin_hosts:
            host.close()
        if launcher._plugin_bus:
            launcher._plugin_bus.close()


def test_setup_plugins_registers_each_plugins_feedback_with_the_monitor(rclpy_context):
    """Events over non-ROS feedback are tracked without a ROS subscription, so
    every plugin's channels have to reach the Monitor -- namespaced, or two
    plugins would register the same topic."""
    launcher = _launcher_with([])
    launcher.monitor_node = _StubMonitor()

    launcher.add_plugin(
        MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="lite3")
    )
    launcher.add_plugin(MockUdpSensor(state_port=_free_port(), id="front_cam"))

    try:
        launcher._setup_plugins()
        assert set(launcher.monitor_node.registered) == {
            "plugin/lite3/feedback/Int32",
            "plugin/front_cam/feedback/Int32",
        }
    finally:
        for host in launcher._plugin_hosts:
            host.close()
        if launcher._plugin_bus:
            launcher._plugin_bus.close()


def test_setup_plugins_is_a_noop_without_plugins(rclpy_context):
    launcher = _launcher_with([])
    launcher.monitor_node = _StubMonitor()

    launcher._setup_plugins()

    assert launcher._plugin_hosts == []
    assert launcher._plugin_bus is None


def test_mounts_are_published_as_static_transforms(rclpy_context):
    """The mount only matters if it actually reaches the TF tree."""
    import math

    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    from ros_sugar.robot import Mount

    publisher_node = Node("mount_publisher")
    listener_node = Node("mount_listener")
    received = []
    listener_node.create_subscription(
        TFMessage,
        "/tf_static",
        lambda m: received.extend(m.transforms),
        QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    launcher = _launcher_with([])
    # The real Monitor is not rclpy-initialized when mounts are registered, so
    # it only holds them; a live node stands in for the broadcast half here.
    launcher.monitor_node = publisher_node
    publisher_node.set_static_transforms = lambda t: setattr(
        publisher_node, "_pending", t
    )

    robot = MockPlugin(state_port=_free_port(), cmd_port=_free_port(), id="lite3")
    robot.base_frame = "base_link"
    camera = MockSensor(cam_name="HikVision", id="front_cam")
    launcher.add_plugin(robot)
    launcher.add_plugin(
        camera,
        mount=Mount(parent=robot, xyz=(0.15, 0.0, 0.35), rpy=(0.0, 0.0, math.pi / 2)),
    )

    try:
        launcher._publish_mounts()

        # what Monitor.activate() does once the node is up
        from tf2_ros import StaticTransformBroadcaster

        stamp = publisher_node.get_clock().now().to_msg()
        for transform in publisher_node._pending:
            transform.header.stamp = stamp
        broadcaster = StaticTransformBroadcaster(publisher_node)
        broadcaster.sendTransform(publisher_node._pending)

        deadline = time.time() + 3.0
        while not received and time.time() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.05)
            rclpy.spin_once(listener_node, timeout_sec=0.05)

        assert received, "the mount never reached /tf_static"
        transform = received[0]
        assert transform.header.frame_id == "base_link"
        assert transform.child_frame_id == "front_cam_frame"
        assert abs(transform.transform.translation.x - 0.15) < 1e-6
        assert abs(transform.transform.translation.z - 0.35) < 1e-6
        # 90 degrees about Z
        assert abs(transform.transform.rotation.z - 0.7071067811865476) < 1e-6
        assert abs(transform.transform.rotation.w - 0.7071067811865476) < 1e-6
    finally:
        publisher_node.destroy_node()
        listener_node.destroy_node()


def test_publishing_mounts_is_a_noop_without_any(rclpy_context):
    from rclpy.node import Node

    node = Node("no_mounts")
    launcher = _launcher_with([])
    launcher.monitor_node = node
    node.set_static_transforms = lambda t: setattr(node, "_pending", t)
    try:
        launcher._publish_mounts()  # must not register anything or raise
        assert not hasattr(node, "_pending")
    finally:
        node.destroy_node()
