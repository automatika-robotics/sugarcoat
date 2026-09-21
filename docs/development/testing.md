# Testing Guide

This document covers how the Sugarcoat test suite is organized, how to run it, and the patterns used to test configuration, events, actions, fallbacks, components, robot plugins and whole recipes. Every snippet below is a trimmed copy of something in `test/`.

## Prerequisites

Build and source a workspace containing Sugarcoat, then install pytest:

```bash
cd ~/ros_ws
colcon build --packages-select automatika_ros_sugar
source install/setup.bash
pip install pytest
```

The UI API tests additionally need `starlette` and `httpx` and are skipped when those are missing. `launch_testing` ships with ROS 2.

## Running the Suite

Tests live in `test/` and are named `*_test.py`; the launch-based component tests are under `test/component/`. Run them with pytest from the repository root, as CI does:

```bash
python3 -m pytest test/ -p no:anyio -q

# one file, one test
python3 -m pytest test/robot_plugin_test.py -p no:anyio -q
python3 -m pytest test/io_types_test.py -p no:anyio -q -k image
```

`colcon test` runs nothing for this package: the CMake testing block is disabled until the package's test dependencies are available on the ROS build farm.

Things worth knowing:

- Launch-based tests print the whole launch log. Use `-q` and redirect to a file when running the full suite.
- Launch-based tests discover nodes by name over DDS. Each run therefore picks a `ROS_DOMAIN_ID` of its own, shown in the pytest header, so it never shares the graph with a stack running on the machine or with another run. Set `ROS_DOMAIN_ID` yourself to rerun on a given domain.
- Every node, service, action, routine and topic name a test module brings up belongs to that module alone. The modules share one process and one domain, and a module's lifecycle nodes stay in the graph after its launch ends, where the Monitor could take one for a later module's component of the same name. `test/unique_names_test.py` fails when two modules share a name. A component that a fixture builds for each test is named after the test.
- A step that has to outlast its timeout, or still be running when the test checks on it, waits on a `threading.Event` that the test sets in a `finally`, rather than sleeping. A busy machine can stall a thread for over a second, and a sleep then loses the race it was meant to win.
- ROS 2 Humble's generated message classes check field types on every assignment, newer distributions only when `ROS_PYTHON_CHECK_FIELDS=1` is set. Run with that variable before pushing to catch mistakes such as assigning `0` to a `bool` field, which fails only on Humble in CI otherwise.
- CI (`.github/workflows/tests.yml`) runs the suite inside `ros:humble`, `jazzy`, `kilted`, `lyrical` and `rolling` containers after a `colcon build`.

## rclpy Fixtures

Each test module owns its rclpy setup; the shared `test/conftest.py` only picks the run's ROS domain and arms a per-test watchdog. Other modules in the same session may already have initialized rclpy (`Launcher.__init__` does, and never shuts it down), so fixtures tolerate a live context instead of asserting on it:

```python
import pytest
import rclpy


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
```

Tests that only exercise configuration, conditions, actions, fallbacks, serialization or the shared-memory ring need no ROS context at all.

## Testing Configuration

`BaseAttrs` configs load from a file section named after the component and validate on every assignment:

```python
import pytest
from attrs import define, field

from ros_sugar.config import BaseComponentConfig, base_validators


@define(kw_only=True)
class MyConfig(BaseComponentConfig):
    threshold: float = field(default=0.5, validator=base_validators.in_range(0.0, 1.0))


def test_config_from_file(tmp_path):
    config_file = tmp_path / "config.yaml"
    config_file.write_text("my_component:\n  loop_rate: 50.0\n  threshold: 0.8\n")
    config = MyConfig()
    assert config.from_file(str(config_file), nested_root_name="my_component")
    assert config.loop_rate == 50.0
    assert config.threshold == 0.8


def test_config_rejects_out_of_range():
    with pytest.raises(ValueError):
        MyConfig(threshold=2.0)
```

`from_file` returns `False` when the file has no section for the component. Round-trips through `to_json()` / `from_json()` are worth a test for any config that crosses the multiprocess boundary; `test/robot_config_test.py` covers the robot description this way.

## Testing Events and Actions

### Event conditions

An `Event` evaluates against a blackboard: a dict from topic name to `EventBlackboardEntry`. Key it by `topic.name`, which has the leading slash stripped:

```python
import time

from std_msgs.msg import Float32 as ROSFloat32

from ros_sugar.core import Event
from ros_sugar.core.event import EventBlackboardEntry
from ros_sugar.io import Topic
from ros_sugar.io.supported_types import Float32


def _blackboard(topic: Topic, value: float):
    msg = ROSFloat32()
    msg.data = value
    return {topic.name: EventBlackboardEntry(msg=msg, timestamp=time.time())}


def test_event_triggers():
    battery = Topic(name="/battery", msg_type=Float32)
    event = Event(battery.msg.data < 10.0)
    event.check_condition(_blackboard(battery, 5.0))
    assert event.trigger is True


def test_event_does_not_trigger():
    battery = Topic(name="/battery", msg_type=Float32)
    event = Event(battery.msg.data < 10.0)
    event.check_condition(_blackboard(battery, 95.0))
    assert event.trigger is False
```

### Serialization round-trip

Events cross the multiprocess boundary as JSON. Test that a restored event still names its topics and evaluates:

```python
def test_event_serialization_round_trip():
    temp = Topic(name="/temp", msg_type=Float32)
    original = Event(temp.msg.data > 100.0)
    restored = Event.from_json(original.to_json())
    assert [t.name for t in restored.get_involved_topics()] == ["temp"]
    restored.check_condition(_blackboard(temp, 150.0))
    assert restored.trigger is True
```

### Actions

An `Action` is called with the triggering messages as `topics`; arguments given as `topic.msg.<field>` expressions are filled from them:

```python
from ros_sugar.core import Action


def test_action_execution():
    called = {}

    def my_handler():
        called["yes"] = True
        return True

    action = Action(my_handler)
    action(topics={})
    assert called["yes"] is True


def test_action_pulls_arguments_from_the_topic():
    sensor = Topic(name="/sensor", msg_type=Float32)
    seen = []
    action = Action(method=seen.append, args=(sensor.msg.data,))
    msg = ROSFloat32()
    msg.data = 3.5
    action(topics={sensor.name: msg})
    assert seen == [3.5]
```

### Fallbacks

`ComponentFallbacks.execute_*_fallback()` runs the next step of the chain and returns whether it has given up:

```python
from ros_sugar.core import Action, ComponentFallbacks, Fallback


def test_fallback_retry():
    calls = {"n": 0}

    def failing_action():
        calls["n"] += 1
        return False, "simulated failure"  # actions return (success, message)

    fallbacks = ComponentFallbacks(
        on_component_fail=Fallback(action=Action(failing_action), max_retries=3)
    )
    for _ in range(3):
        assert fallbacks.execute_component_fallback() is False
    assert fallbacks.execute_component_fallback() is True
    assert calls["n"] == 3
```

## Testing a Component Without a Launcher

A component can be driven directly on a live rclpy context: initialize its node, create the resources under test, and spin it by hand. This is how the plugin binding, TF lifecycle and topic replacement tests work (`test/robot_plugin_test.py`, `test/tf_lifecycle_test.py`):

```python
import time

import pytest
import rclpy
from std_msgs.msg import Float32 as ROSFloat32

from ros_sugar.core import BaseComponent
from ros_sugar.io import Topic


def test_component_receives_its_input():
    component = BaseComponent(
        component_name="doc_example_component",
        inputs=[Topic(name="sensor", msg_type="Float32")],
    )
    component.rclpy_init_node()
    try:
        component.create_all_subscribers()
        publisher = component.create_publisher(ROSFloat32, "sensor", 10)
        callback = component.callbacks["sensor"]
        msg = ROSFloat32()
        msg.data = 4.2
        deadline = time.time() + 2.0
        while not callback.got_msg and time.time() < deadline:
            publisher.publish(msg)
            rclpy.spin_once(component, timeout_sec=0.05)
        assert component.got_all_inputs()
        assert callback.get_output() == pytest.approx(4.2)
    finally:
        component.destroy_node()
```

Always destroy the node in a `finally` block; a leaked node keeps its name on the graph for the rest of the session.

## Testing a Robot Plugin

Plugins are tested against a mock robot on the loopback interface. A `RobotPluginHost` on an `InProcessFeedbackBus` opens the plugin's transports exactly as the Launcher would, with `node=None` since no ROS node is needed for non-ROS transports. Bind to a free port rather than a fixed one so tests can run in parallel:

```python
import socket
import time

from std_msgs.msg import Int32 as RosInt32

from ros_sugar.robot import (
    Feedback,
    InProcessFeedbackBus,
    PluginMetadata,
    RobotPlugin,
    RobotPluginHost,
    UdpTransport,
    create_supported_type,
)


def _int32_callback(msg: RosInt32) -> int:
    return msg.data


RobotInt32 = create_supported_type(RosInt32, callback=_int32_callback)


def _decode(raw: bytes):
    msg = RosInt32()
    msg.data = int(raw.decode())
    return msg


class MockRobot(RobotPlugin):
    def __init__(self, port: int = 0):
        self.metadata = PluginMetadata(name="MockRobot")
        telemetry = UdpTransport("telemetry", bind=("127.0.0.1", port))
        self.transports = {"telemetry": telemetry}
        self.feedbacks = {
            "Int32": Feedback(key="Int32", msg_type=RobotInt32, transport=telemetry, decoder=_decode)
        }


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_plugin_host_decodes_telemetry():
    port = _free_udp_port()
    plugin = MockRobot(port=port)
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()
    received = []
    handle = plugin.subscribe_feedback(plugin.feedbacks["Int32"], received.append)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as robot:
            robot.sendto(b"42", ("127.0.0.1", port))
        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            time.sleep(0.02)
        assert received and received[0].data == 42
    finally:
        handle.unsubscribe()
        host.close()
```

`test/robot_plugin_test.py` extends this pattern to commands, multiprocess specs, the socket bus and the shared-memory fast path; `test/plugin_processes_test.py` covers driver-process declarations against a launcher with fake components; `test/robot_shm_test.py` tests the shared-memory ring on its own, including a cross-process read.

## Testing a Recipe with launch_testing

Whole recipes run under `launch_testing`. Build the launch description with the `Launcher` but do not call `bringup()`: `setup_launch_description()` assembles it, `ReadyToTest()` hands control to the test class, and module-level `threading.Event`s carry results out of components and actions:

```python
import unittest
from threading import Event as ThreadingEvent

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar import Launcher
from ros_sugar.core import BaseComponent

ran_once = ThreadingEvent()


class TickingComponent(BaseComponent):
    def _execution_step(self):
        ran_once.set()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    component = TickingComponent(component_name="ticking_component")
    component.loop_rate = 10.0

    launcher = Launcher()
    launcher.add_pkg(components=[component])
    launcher.setup_launch_description()          # build, but do not run
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


class TestRecipe(unittest.TestCase):
    def test_component_executes(self):
        assert ran_once.wait(10.0), "the component never ran"
```

The event tests (`test/simple_events_test.py`, `test/composed_events_test.py`, `test/generic_events_test.py`) and the component run-type tests under `test/component/` all follow this shape: a component publishes, the recipe's events fire actions that set threading events, and the test class waits on them with a timeout.

The components here run in launcher threads. Multiprocess launch needs an installed package with an executable entry point, so the pieces it relies on are tested directly instead: the process launch action and its prefix in `test/external_launch_test.py`, the socket bus in `test/robot_plugin_test.py`, and the shared-memory ring in `test/robot_shm_test.py`.
