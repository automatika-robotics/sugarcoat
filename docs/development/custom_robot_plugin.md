# Creating a Robot Plugin

A **robot plugin** adapts a Sugarcoat-based stack to one specific robot. It maps
generic component I/O (`Twist`, `Odometry`, `Imu`, …) to whatever the robot
actually exposes — ROS topics, ROS services, UDP, HTTP, or a vendor SDK — and
can additionally contribute high-level **actions** (`stand_up`, `backflip`) and
**events** (`fall_detected`, `low_battery`) that recipes consume directly.
Component code never changes.

A plugin author subclasses {py:class}`~ros_sugar.robot.RobotPlugin`. One plugin
instance is passed to the `Launcher`.

```{tip}
A complete, runnable reference plugin lives at
[automatika-robotics/robot-plugin-example](https://github.com/automatika-robotics/robot-plugin-example).
It implements one robot across **all** the transport families covered here
(UDP telemetry + command, a ROS-topic feedback, and a ROS-service action) and
is the recommended starting point — copy it and adapt. The steps below explain
the pieces it is built from.
```

## When to Use a Plugin

- Your robot uses custom ROS message types instead of standard ones.
- Your robot's control surface is **not ROS** — UDP/TCP/HTTP or a vendor SDK.
- Your robot has named one-shot behaviours or emits hardware events you want as
  first-class triggers.
- You want one recipe to run on different robots by swapping a single object.

## Architecture

The plugin has two roles, decided automatically by the `Launcher`:

- **HOST** — lives in the launcher process. Owns the real transports (binds
  sockets, opens sessions, runs heartbeats), decodes telemetry once, and
  publishes it on a feedback bus.
- **CLIENT** — lives in a component process under multiprocess launch, rebuilt
  from a serializable spec. It opens no vendor sockets; it consumes decoded
  feedback from the bus and sends commands.

Under multithreaded launch there is a single HOST instance shared by every
component thread — the bus is in-process and there is no CLIENT role.

```{important}
Because the plugin must be reconstructable in component subprocesses, its
`__init__` must be **declarative** — accept only JSON-serializable keyword
arguments and perform **no I/O**. Open sockets and start threads in
`on_activate()`, not `__init__()`.
```

## Building Blocks

| Class | Role |
|:------|:-----|
| {py:class}`~ros_sugar.robot.RobotPlugin` | The plugin itself — subclass this. |
| {py:class}`~ros_sugar.robot.Transport` | Where data comes from / goes to: `UdpTransport`, `HttpTransport`, `SdkCallbackTransport`, `RosTopicTransport`, `RosServiceTransport`. |
| {py:class}`~ros_sugar.robot.Feedback` | One telemetry stream — a standard type name, a `SupportedType`, a transport, and a decoder. |
| {py:class}`~ros_sugar.robot.RobotCommand` | One command surface — a standard type name, a transport, and an encoder. |
| {py:class}`~ros_sugar.robot.ActionRegistry` / {py:class}`~ros_sugar.robot.EventRegistry` | Named factories producing `Action` / `Event` objects. |
| {py:func}`~ros_sugar.robot.create_supported_type` | Wraps a robot's custom ROS message as a `SupportedType`. |

## Step 1: Wrap Custom Message Types

Use `create_supported_type()` to turn a robot's ROS message into a
`SupportedType`, optionally with a `callback` (ROS message → Python value, for
feedback) and/or `converter` (Python value → ROS message, for commands):

```python
import numpy as np
from ros_sugar.robot import create_supported_type
from myrobot_msgs.msg import CustomOdom


def _odom_callback(msg: CustomOdom) -> np.ndarray:
    return np.array([msg.x, msg.y, msg.yaw])


RobotOdometry = create_supported_type(CustomOdom, callback=_odom_callback)
```

## Step 2: Define Transports, Feedbacks and Commands

A `Transport` carries raw payloads. A `Feedback` pairs a transport with a
**decoder** (`raw payload → ROS message`, or `None` to ignore a packet). A
`RobotCommand` pairs a transport with an **encoder** (`component output → wire
payload`).

```python
import struct
from std_msgs.msg import Float32 as RosFloat32
from ros_sugar.robot import (
    RobotPlugin, PluginMetadata, Feedback, RobotCommand,
    UdpTransport, create_supported_type,
)

RobotBattery = create_supported_type(RosFloat32, callback=lambda m: m.data)


def _decode_battery(raw: bytes):
    if raw[:1] != b"\x10":            # not a battery packet — ignore
        return None
    msg = RosFloat32()
    msg.data = struct.unpack("<f", raw[1:5])[0]
    return msg


def _encode_twist(output) -> bytes:
    return struct.pack("<3f", output[0], output[1], output[2])
```

Transports may declare a heartbeat with `keep_alive_fn` / `keep_alive_rate_hz`;
the plugin HOST runs it on a background thread while active. A transport may set
`route_via_host=True` so commands are forwarded through the HOST (use for
session-bound protocols where a single client must own the connection).

## Step 3: Subclass `RobotPlugin`

Construction is **zero-argument**: a `RobotPlugin` represents a *specific* robot,
so every endpoint and tuning knob is part of the plugin's identity — declared as
class attributes, not constructor parameters. Recipe authors write
`MyRobotPlugin()` and nothing more.

```python
class MyRobotPlugin(RobotPlugin):
    # Robot-specific configuration — the plugin's identity, not recipe concerns.
    ROBOT_IP = "192.168.1.120"        # control board's IP on the robot's LAN
    COMMAND_PORT = 43893
    TELEMETRY_PORT = 43897
    BIND_HOST = "0.0.0.0"             # local NIC to listen on

    def __init__(self):
        self.metadata = PluginMetadata(name="MyRobot", vendor="Acme", version="1.0")

        telemetry = UdpTransport(
            "telemetry", bind=(self.BIND_HOST, self.TELEMETRY_PORT),
        )
        commands = UdpTransport(
            "commands", send_to=(self.ROBOT_IP, self.COMMAND_PORT),
            keep_alive_fn=self._heartbeat, keep_alive_rate_hz=2.0,
        )
        self.transports = {"telemetry": telemetry, "commands": commands}

        self.feedbacks = {
            "Float32": Feedback("Float32", RobotBattery, telemetry,
                                decoder=_decode_battery, rate_hz=50.0),
        }
        self.commands = {
            "Twist": RobotCommand("Twist", commands, encoder=_encode_twist),
        }
        self.actions = ActionRegistry({
            "stand_up": lambda: Action(method=lambda: commands.send(b"\x21\x01\x02\x02")),
            "sit":      lambda: Action(method=lambda: commands.send(b"\x21\x01\x02\x03")),
        })
        self.events = EventRegistry({
            "low_battery": lambda threshold=0.2: Event(
                event_condition=self.feedbacks["Float32"].as_topic().msg.data < threshold,
                on_change=True,
            ),
        })

    def _heartbeat(self):
        self.transports["commands"].send(b"\x00")
```

Notes:

- `feedbacks` / `commands` are keyed by the **standard message-type name** they
  stand in for — matched against component input/output topic message types.
- `actions` entries are **factories** returning fresh `Action` objects; `events`
  entries are factories returning `Event` objects, so recipes can pass per-call
  arguments.
- `Feedback.as_topic()` yields the topic events reference — the real ROS topic
  for `RosTopicTransport` feedbacks, or a synthetic bus channel for everything
  else.
- Override `on_load(node)`, `on_activate()`, `on_deactivate()` only if you need
  custom HOST-side setup; the defaults open transports, start the bus and run
  heartbeats for you.

### Overriding for non-default deployments

If a particular deployment needs different endpoints (alternate subnet, custom
port, calibration tweak), **subclass** rather than parameterizing the recipe.
Set the override as an instance attribute *before* `super().__init__()` so the
production constructor reads it in place of the class default:

```python
class MyRobotOnLAN(MyRobotPlugin):
    """A MyRobot deployed on the 10.0.0.x subnet."""
    ROBOT_IP = "10.0.0.42"   # static class-attribute override


class _MyRobotForTest(MyRobotPlugin):
    """Test-only override with dynamic ports."""
    def __init__(self, *, command_port: int, telemetry_port: int):
        self.ROBOT_IP = "127.0.0.1"
        self.COMMAND_PORT = command_port
        self.TELEMETRY_PORT = telemetry_port
        super().__init__()
```

The serializable plugin spec captures whichever subclass and kwargs were used,
so the same override survives the multiprocess launch boundary.

## Step 4: Use the Plugin

```python
from ros_sugar.launch import Launcher
from my_robot_plugin import MyRobotPlugin

plugin = MyRobotPlugin()                     # declarative, all args defaulted
launcher = Launcher(robot_plugin=plugin)
launcher.add_pkg(components=[planner, controller], multiprocessing=True)

# Plugin-provided event + action factories, wired with the Launcher.on() sugar
launcher.on(plugin.events.low_battery(0.15), plugin.actions.sit())

launcher.bringup()
```

The launcher hosts the plugin, propagates it to every component, and tears it
down on shutdown. During activation each component looks up its input/output
topic types in the plugin: a match on a `RosTopicTransport` re-uses the native
subscriber/publisher-swap path, any other transport is bridged through the
feedback bus — components remain unaware their data is not ROS.

## How Replacement Works

During component activation, for every input topic the plugin provides a
`Feedback` for:

- `RosTopicTransport` → the component's subscriber is re-pointed at the robot's
  topic and type.
- any other transport → no ROS subscription is created; decoded ROS messages
  from the feedback bus are pushed straight into the component's callback slot.

For every output topic the plugin provides a `RobotCommand` for:

- `RosTopicTransport` → the publisher is re-pointed at the robot's topic.
- any other transport → the publisher is replaced by an adapter that runs the
  component's pre-processors, encodes the output, and sends it on the command
  transport (directly, or via the HOST when `route_via_host` is set).

Decoded feedback is also injected into the Monitor's event blackboard, so
`Event`s and `Condition`s over plugin feedback evaluate exactly as they would
for a real ROS topic.

## Introspection

List everything a plugin exposes, programmatically or from the command line:

```python
plugin.list_feedbacks()   # -> [FeedbackSpec, ...]
plugin.list_commands()    # -> [CommandSpec, ...]
plugin.list_actions()     # -> [ActionSpec, ...]
plugin.list_events()      # -> [EventSpec, ...]
plugin.describe()         # -> a JSON-serializable tree of all of the above
```

```bash
python -m ros_sugar.robot inspect my_robot_plugin:MyRobotPlugin
```

## Full Example

[automatika-robotics/robot-plugin-example](https://github.com/automatika-robotics/robot-plugin-example)
is the reference template: one robot spanning UDP, ROS-topic and ROS-service
interfaces, with a mock robot and an end-to-end test suite. Copy it and adapt.
