"""Tests for plugin-declared driver processes and recipe demand propagation.

Covers `Plugin.requested_feedbacks` / `requested_commands` — what a recipe
actually asked a plugin for — and `Plugin.required_processes`, which lets a
plugin have the launcher bring up the driver node its data depends on.
"""

import pytest
from launch_ros.actions import Node as NodeLaunchAction

from ros_sugar import Launcher
from ros_sugar.io.topic import Topic
from ros_sugar.robot import (
    Feedback,
    PluginMetadata,
    ProcessSpec,
    RobotCommand,
    RobotPlugin,
)
from ros_sugar.robot.transports.udp import UdpTransport

from ros_sugar.io.supported_types import LaserScan, Odometry, Twist


class _DriverPlugin(RobotPlugin):
    """A robot plugin with two same-typed feedbacks and one command.

    Two `LaserScan` feedbacks is the interesting case: it forces recipes to
    name the plugin's key rather than lean on the unique-type fallback, which
    is exactly the shape of a robot with a front and a rear lidar.
    """

    def __init__(self):
        self.metadata = PluginMetadata(name="DriverBot", vendor="test")
        transport = UdpTransport("state", send_to=("127.0.0.1", 45999))
        self.transports = {"robot": transport}
        self.feedbacks = {
            "scan_front": Feedback(
                key="scan_front", msg_type=LaserScan, transport=transport,
                decoder=lambda raw: None,
            ),
            "scan_back": Feedback(
                key="scan_back", msg_type=LaserScan, transport=transport,
                decoder=lambda raw: None,
            ),
            "odom": Feedback(
                key="odom", msg_type=Odometry, transport=transport,
                decoder=lambda raw: None,
            ),
        }
        self.commands = {
            "processes_cmd_vel": RobotCommand(
                key="processes_cmd_vel", msg_type=Twist, transport=transport,
                encoder=lambda out: b"",
            )
        }
        # Test hooks
        self.seen_at_attach = None
        self.precondition_result = True
        self.declare_raises = False

    def required_processes(self):
        if self.declare_raises:
            raise RuntimeError("boom")
        if not {"scan_front", "scan_back"} & self.requested_feedbacks:
            return []
        return [
            ProcessSpec(
                package="fake_lidar_pkg",
                executable="fake_lidar_node",
                name="lidar_driver",
                precondition=lambda: self.precondition_result,
            )
        ]

    def on_attached(self, node, bus) -> None:
        # Demand must already be populated by the time this runs.
        self.seen_at_attach = self.requested_feedbacks


class _FakeComponent:
    """Minimum surface the launcher's demand pass needs."""

    def __init__(self, node_name, in_topics=None, out_topics=None):
        self.node_name = node_name
        self.in_topics = in_topics or []
        self.out_topics = out_topics or []


def _launcher_with(plugin, components):
    launcher = Launcher(robot_plugin=plugin)
    launcher._components = components
    return launcher


@pytest.fixture
def driver_installed(monkeypatch):
    """Treat the tests' fake driver package as installed."""
    monkeypatch.setattr(
        "ros_sugar.launch.launcher._check_ros_executable", lambda *_: None
    )


# --- demand resolution ---------------------------------------------------


def test_demand_resolves_by_plugin_key():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)]
    )
    _launcher_with(plugin, [comp])._resolve_plugin_demand()

    assert plugin.requested_feedbacks == frozenset({"scan_front"})
    assert plugin.requested_commands == frozenset()


def test_demand_resolves_by_unique_type_fallback():
    """``Odometry`` is unique on this plugin, so the topic name need not match."""
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="whatever", msg_type="Odometry", use_plugin=True)]
    )
    _launcher_with(plugin, [comp])._resolve_plugin_demand()

    assert plugin.requested_feedbacks == frozenset({"odom"})


def test_demand_resolves_commands_from_out_topics():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a",
        out_topics=[
            Topic(name="processes_cmd_vel", msg_type="Twist", use_plugin=True)
        ],
    )
    _launcher_with(plugin, [comp])._resolve_plugin_demand()

    assert plugin.requested_commands == frozenset({"processes_cmd_vel"})
    assert plugin.requested_feedbacks == frozenset()


def test_demand_unions_across_components():
    plugin = _DriverPlugin()
    comps = [
        _FakeComponent(
            "a",
            in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)],
        ),
        _FakeComponent(
            "b",
            in_topics=[Topic(name="scan_back", msg_type="LaserScan", use_plugin=True)],
        ),
    ]
    _launcher_with(plugin, comps)._resolve_plugin_demand()

    assert plugin.requested_feedbacks == frozenset({"scan_front", "scan_back"})


def test_topics_not_bound_to_a_plugin_are_ignored():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan")]
    )
    _launcher_with(plugin, [comp])._resolve_plugin_demand()

    assert plugin.requested_feedbacks == frozenset()


def test_ambiguous_reference_does_not_escape_the_demand_pass():
    """Two feedbacks share ``LaserScan``; a topic named after neither is
    ambiguous. The component reports that and falls back — resolving demand
    must not turn it into a bringup failure."""
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="unmatched", msg_type="LaserScan", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])

    launcher._resolve_plugin_demand()  # must not raise

    assert plugin.requested_feedbacks == frozenset()


def test_type_mismatch_on_key_match_does_not_escape():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="Odometry", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])

    launcher._resolve_plugin_demand()  # must not raise

    assert plugin.requested_feedbacks == frozenset()


def test_demand_defaults_empty_without_a_launcher():
    """A plugin nobody asked anything of requests nothing — notably not
    'everything', which would have a standalone host start every driver."""
    plugin = _DriverPlugin()

    assert plugin.requested_feedbacks == frozenset()
    assert plugin.requested_commands == frozenset()
    assert plugin.required_processes() == []


# --- process declaration -------------------------------------------------


def test_declared_driver_becomes_a_launch_action(driver_installed):
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])
    launcher._resolve_plugin_demand()
    launcher._launch_plugin_processes(plugin)

    nodes = [e for e in launcher._description.entities
             if isinstance(e, NodeLaunchAction)]
    assert len(nodes) == 1


def test_no_driver_when_no_component_wants_the_feedback():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a",
        out_topics=[
            Topic(name="processes_cmd_vel", msg_type="Twist", use_plugin=True)
        ],
    )
    launcher = _launcher_with(plugin, [comp])
    launcher._resolve_plugin_demand()
    launcher._launch_plugin_processes(plugin)

    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


def test_failing_precondition_skips_the_driver():
    """The already-running case: the vendor's own driver holds the port."""
    plugin = _DriverPlugin()
    plugin.precondition_result = False
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])
    launcher._resolve_plugin_demand()
    launcher._launch_plugin_processes(plugin)

    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


def test_a_raising_declaration_does_not_stop_bringup():
    plugin = _DriverPlugin()
    plugin.declare_raises = True
    launcher = _launcher_with(plugin, [])

    launcher._launch_plugin_processes(plugin)  # must not raise

    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


def test_a_driver_that_is_not_installed_fails_bringup():
    """Found by the launch system only once running, it stopped every node with
    an error naming neither the plugin nor the fix."""
    from ament_index_python.packages import PackageNotFoundError

    plugin = _DriverPlugin()  # declares 'fake_lidar_pkg', which is not installed
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])
    launcher._resolve_plugin_demand()

    with pytest.raises(PackageNotFoundError) as raised:
        launcher._launch_plugin_processes(plugin)

    message = raised.value.args[0]
    assert plugin.id in message
    assert "lidar_driver" in message, "names the driver"
    assert "scan_front" in message, "names what the recipe used it for"
    assert "not installed" in message


def test_a_driver_missing_its_executable_fails_bringup():
    from ros_sugar.launch.launcher import _check_ros_executable

    with pytest.raises(FileNotFoundError, match="no executable 'no_such_node'"):
        _check_ros_executable("tf2_ros", "no_such_node")


def test_an_installed_driver_passes_the_check():
    from ros_sugar.launch.launcher import _check_ros_executable

    _check_ros_executable("tf2_ros", "static_transform_publisher")


def test_add_ros_node_rejects_a_package_that_is_not_installed():
    """The same failure for a node a recipe adds by hand."""
    from ament_index_python.packages import PackageNotFoundError

    launcher = Launcher()
    with pytest.raises(PackageNotFoundError, match="not installed"):
        launcher.add_ros_node(package="no_such_pkg", executable="node")
    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


def test_a_missing_driver_fails_before_anything_is_opened():
    """Raised before the bus starts or any host opens, so a failed bringup
    leaves no transports or bus behind."""
    from ament_index_python.packages import PackageNotFoundError

    plugin = _RosDriverPlugin()  # declares 'fake_lidar_pkg'
    launcher = Launcher(robot_plugin=plugin)
    launcher._components = [
        _FakeComponent(
            "a",
            in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)],
        )
    ]
    launcher.monitor_node = _FakeMonitorNode()

    with pytest.raises(PackageNotFoundError):
        launcher._setup_plugins()

    assert launcher._plugin_hosts == []
    assert launcher._plugin_bus is None
    assert plugin.seen_at_attach is None, "on_attached must not have run"


def test_a_raising_precondition_skips_the_driver():
    plugin = _DriverPlugin()
    comp = _FakeComponent(
        "a", in_topics=[Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)]
    )
    launcher = _launcher_with(plugin, [comp])
    launcher._resolve_plugin_demand()

    def _broken():
        raise OSError("cannot probe the port")

    plugin.required_processes = lambda: [
        ProcessSpec(package="fake_lidar_pkg", executable="n", precondition=_broken)
    ]
    launcher._launch_plugin_processes(plugin)  # must not raise

    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


def test_a_single_spec_instead_of_a_list_does_not_stop_bringup():
    plugin = _DriverPlugin()
    launcher = _launcher_with(plugin, [])
    plugin.required_processes = lambda: ProcessSpec(package="p", executable="e")

    launcher._launch_plugin_processes(plugin)  # logged as a declaration fault


def test_plugins_without_the_hook_are_unaffected():
    """The whole feature is a no-op for a plugin that does not opt in."""

    class _Plain(RobotPlugin):
        def __init__(self):
            self.metadata = PluginMetadata(name="Plain")

    plugin = _Plain()
    launcher = _launcher_with(plugin, [])
    launcher._launch_plugin_processes(plugin)

    assert plugin.required_processes() == []
    assert not [e for e in launcher._description.entities
                if isinstance(e, NodeLaunchAction)]


# --- spec ----------------------------------------------------------------


def test_launch_kwargs_omits_precondition():
    spec = ProcessSpec(package="p", executable="e", precondition=lambda: True)
    kwargs = spec.launch_kwargs()

    assert "precondition" not in kwargs
    assert kwargs["package"] == "p"
    assert kwargs["respawn"] is True


def test_spec_label_falls_back_to_package_and_executable():
    assert ProcessSpec(package="p", executable="e").label == "p/e"
    assert ProcessSpec(package="p", executable="e", name="n").label == "n"


# --- ordering, through the real _setup_plugins path ----------------------


class _RosDriverPlugin(RobotPlugin):
    """Same shape as `_DriverPlugin` but on ROS transports, which the host
    skips — so `_setup_plugins` can be run whole without opening a socket."""

    def __init__(self):
        from ros_sugar.robot import RosTopicTransport

        self.metadata = PluginMetadata(name="RosDriverBot", vendor="test")
        transport = RosTopicTransport(
            "scan", topic_name="/scan", msg_type=LaserScan
        )
        self.transports = {"scan": transport}
        self.feedbacks = {
            "scan_front": Feedback(
                key="scan_front", msg_type=LaserScan, transport=transport
            )
        }
        self.seen_at_attach = None

    def required_processes(self):
        if "scan_front" not in self.requested_feedbacks:
            return []
        return [ProcessSpec(package="fake_lidar_pkg", executable="fake_lidar_node")]

    def on_attached(self, node, bus) -> None:
        self.seen_at_attach = self.requested_feedbacks


class _FakeMonitorNode:
    def feed_external_topic(self, channel, msg):
        pass

    def register_external_topic(self, topic):
        pass


@pytest.fixture
def wired_launcher(driver_installed):
    plugin = _RosDriverPlugin()
    launcher = Launcher(robot_plugin=plugin)
    launcher._components = [
        _FakeComponent(
            "a",
            in_topics=[
                Topic(name="scan_front", msg_type="LaserScan", use_plugin=True)
            ],
        )
    ]
    launcher.monitor_node = _FakeMonitorNode()
    yield launcher, plugin
    for host in launcher._plugin_hosts:
        host.close()


def _node_actions(launcher):
    return [e for e in launcher._description.entities
            if isinstance(e, NodeLaunchAction)]


def test_on_attached_sees_the_resolved_demand(wired_launcher):
    """The ordering the whole design rests on: demand is populated before any
    plugin hook runs, so a plugin can act on it."""
    launcher, plugin = wired_launcher

    launcher._setup_plugins()

    assert plugin.seen_at_attach == frozenset({"scan_front"})


def test_setup_plugins_starts_the_declared_driver(wired_launcher):
    launcher, _ = wired_launcher

    launcher._setup_plugins()

    assert len(_node_actions(launcher)) == 1


def test_setup_plugins_is_idempotent(wired_launcher):
    """``setup_launch_description`` is public; running the plugin setup twice
    must not start the driver twice or open a second host."""
    launcher, _ = wired_launcher

    launcher._setup_plugins()
    launcher._setup_plugins()

    assert len(_node_actions(launcher)) == 1
    assert len(launcher._plugin_hosts) == 1


# --- inputs: feedback a driver reads on ROS ------------------------------


def _capture_ros_nodes(launcher, monkeypatch):
    """Record what the launcher hands ``add_ros_node``, without launching."""
    added = []
    monkeypatch.setattr(launcher, "add_ros_node", lambda **kwargs: added.append(kwargs))
    return added


def test_launch_kwargs_omits_inputs():
    """``inputs`` is delivered by the launcher, not passed to launch_ros."""
    spec = ProcessSpec(package="p", executable="e", inputs={"odom": "/odom"})

    assert "inputs" not in spec.launch_kwargs()


def test_inputs_decoded_by_the_host_are_published_by_it(monkeypatch):
    """A feedback that only lives on the bus is handed to the host to publish,
    and the driver is left subscribing on its own topic name."""
    plugin = _DriverPlugin()  # 'odom' arrives over UDP
    plugin.required_processes = lambda: [
        ProcessSpec(package="ekf_pkg", executable="ekf", inputs={"odom": "/odom"})
    ]
    launcher = _launcher_with(plugin, [])
    added = _capture_ros_nodes(launcher, monkeypatch)

    published = launcher._launch_plugin_processes(plugin)

    assert published == [("odom", "/odom")]
    assert len(added) == 1
    assert added[0]["remappings"] is None


def test_inputs_already_on_ros_are_remapped_not_published(monkeypatch):
    """A feedback on a ROS topic is on ROS already: republishing it would
    duplicate the stream, so the driver is pointed at the real topic."""
    plugin = _RosDriverPlugin()  # 'scan_front' is the ROS topic /scan
    plugin.required_processes = lambda: [
        ProcessSpec(
            package="p",
            executable="e",
            remappings=[("/in", "/out")],
            inputs={"scan_front": "/points"},
        )
    ]
    launcher = _launcher_with(plugin, [])
    added = _capture_ros_nodes(launcher, monkeypatch)

    published = launcher._launch_plugin_processes(plugin)

    assert published == []
    assert added[0]["remappings"] == [("/in", "/out"), ("/points", "/scan")]


def test_an_input_on_its_own_topic_needs_no_remapping(monkeypatch):
    plugin = _RosDriverPlugin()
    plugin.required_processes = lambda: [
        ProcessSpec(package="p", executable="e", inputs={"scan_front": "/scan"})
    ]
    launcher = _launcher_with(plugin, [])
    added = _capture_ros_nodes(launcher, monkeypatch)

    launcher._launch_plugin_processes(plugin)

    assert added[0]["remappings"] is None


def test_an_input_naming_no_feedback_fails_bringup(monkeypatch):
    """The driver would start and never receive anything -- a silent failure
    that reads as the driver being broken."""
    plugin = _DriverPlugin()
    plugin.required_processes = lambda: [
        ProcessSpec(package="p", executable="e", name="ekf", inputs={"nope": "/x"})
    ]
    launcher = _launcher_with(plugin, [])
    added = _capture_ros_nodes(launcher, monkeypatch)

    with pytest.raises(ValueError) as raised:
        launcher._launch_plugin_processes(plugin)

    message = raised.value.args[0]
    assert "'ekf'" in message and "'nope'" in message and "odom" in message
    assert added == [], "no driver started"


def test_inputs_of_a_skipped_driver_are_not_published(monkeypatch):
    """A driver whose precondition fails is not running, so nothing reads the
    topic and the host has no reason to publish it."""
    plugin = _DriverPlugin()
    plugin.required_processes = lambda: [
        ProcessSpec(
            package="p", executable="e",
            inputs={"odom": "/odom"}, precondition=lambda: False,
        )
    ]
    launcher = _launcher_with(plugin, [])
    _capture_ros_nodes(launcher, monkeypatch)

    assert launcher._launch_plugin_processes(plugin) == []


def test_setup_plugins_has_the_host_publish_the_inputs(driver_installed):
    plugin = _DriverPlugin()
    plugin.required_processes = lambda: [
        ProcessSpec(package="ekf_pkg", executable="ekf", inputs={"odom": "/odom"})
    ]
    launcher = Launcher(robot_plugin=plugin)
    launcher._components = []
    launcher.monitor_node = _FakeMonitorNode()
    try:
        launcher._setup_plugins()

        (host,) = launcher._plugin_hosts
        assert host._ros_outputs.topics == {"odom": ["/odom"]}
    finally:
        for host in launcher._plugin_hosts:
            host.close()
