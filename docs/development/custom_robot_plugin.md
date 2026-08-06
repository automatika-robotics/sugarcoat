# Creating a Plugin

A **plugin** adapts a Sugarcoat-based stack to one specific piece of hardware. It
maps generic component I/O (`Twist`, `Odometry`, `Imu`, …) to whatever the
hardware actually exposes — ROS topics, ROS services, UDP, HTTP, or a vendor SDK
— and can additionally contribute high-level **actions** (`stand_up`, `backflip`)
and **events** (`fall_detected`, `low_battery`) that recipes consume directly.
Component code never changes.

Plugins come in two roles:

| Base class | Represents | Per recipe |
|:--|:--|:--|
| {py:class}`~ros_sugar.robot.RobotPlugin` | The robot itself | exactly one |
| {py:class}`~ros_sugar.robot.SensorPlugin` | A sensor that is not part of the robot's own hardware — an inspection camera bolted on, a fixed camera watching a room | any number |

The role is **intrinsic**: it follows from the base class you subclass, is
read-only, and a recipe cannot attach a plugin under a different role.

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
arguments and perform **no I/O**. Sockets, threads and heartbeats are opened
for you by the plugin HOST; if you need node-dependent setup of your own,
override `on_attached(node, bus)`.
```

## Building Blocks

| Class | Role |
|:------|:-----|
| {py:class}`~ros_sugar.robot.RobotPlugin` / {py:class}`~ros_sugar.robot.SensorPlugin` | The plugin itself — subclass one of these. |
| {py:class}`~ros_sugar.robot.Mount` | Where a sensor sits, when nothing else publishes its frame into TF. |
| {py:class}`~ros_sugar.robot.Transport` | Where data comes from / goes to: `UdpTransport`, `HttpTransport`, `SdkCallbackTransport`, `RosTopicTransport`, `RosServiceTransport`. |
| {py:class}`~ros_sugar.robot.Feedback` | One telemetry stream — a standard type name, a `SupportedType`, a transport, and a decoder. |
| {py:class}`~ros_sugar.robot.RobotCommand` | One command surface — a standard type name, a transport, and an encoder. |
| {py:class}`~ros_sugar.robot.ActionRegistry` / {py:class}`~ros_sugar.robot.EventRegistry` | Named factories producing `Action` / `Event` objects. |
| {py:func}`~ros_sugar.robot.create_supported_type` | Wraps a robot's custom ROS message as a `SupportedType`. |

## Step 1: Wrap Custom Message Types

Use `create_supported_type()` to turn a robot's ROS message into a
`SupportedType`, optionally with a `callback` (ROS message → Python value, for
feedback) and/or `converter` (Python value → ROS message, for commands).

Both are checked against their annotations, so they must be annotated functions
rather than bare lambdas: a `callback`'s first argument must be annotated with
the ROS message type, and its return with a non-ROS Python type.

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

def _battery_callback(msg: RosFloat32) -> float:
    return msg.data


RobotBattery = create_supported_type(RosFloat32, callback=_battery_callback)


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

## Step 3: Subclass a Plugin Base

A plugin represents a *specific* piece of hardware, so its endpoints and tuning
knobs are part of its identity — declared as class attributes rather than
constructor parameters. Recipe authors normally write `MyRobotPlugin()` and
nothing more.

Two keyword arguments are **reserved by the framework** and always available,
whether or not your `__init__` declares them:

| Argument | Purpose |
|:--|:--|
| `id` | Identity within a recipe. Defaults to a slug of the metadata name. Needed only when two plugins of the same kind are attached. |
| `frame_id` | Name of the frame this hardware's data is in. Sensors default to `<id>_frame`. |

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
            "Float32": Feedback(
                key="Float32", msg_type=RobotBattery, transport=telemetry,
                decoder=_decode_battery, rate_hz=50.0,
            ),
        }
        self.commands = {
            "Twist": RobotCommand(
                key="Twist", transport=commands, encoder=_encode_twist,
            ),
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

- `feedbacks` / `commands` are resolved against a component topic **by name
  first** — a topic named after one of your registry keys binds to that entry —
  falling back to a unique match on message type. So keying by message-type name
  (`"Twist"`, `"Odometry"`) is the convention when you expose one entry per type;
  when you expose several of the same type (a humanoid's `"left_arm"` /
  `"right_arm"` JointStates) use descriptive keys and have the recipe name its
  topics to match.
- `actions` entries are **factories** returning fresh `Action` objects; `events`
  entries are factories returning `Event` objects, so recipes can pass per-call
  arguments.
- `Feedback.as_topic()` yields the topic events reference — the real ROS topic
  for `RosTopicTransport` feedbacks, or a synthetic bus channel for everything
  else.
- Override `on_attached(node, bus)` only if you need custom HOST-side setup —
  binding a `RosServiceTransport` client is the canonical case. The HOST already
  opens transports, wires decoders to the bus and runs heartbeats for you. It is
  the **only** author hook.

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

## Describing the Robot

A `RobotPlugin` knows the robot it drives, so it can configure the whole stack:

```python
class MyRobotPlugin(RobotPlugin):
    def __init__(self):
        ...
        self.robot_config = RobotConfig(
            model_type=RobotType.DIFFERENTIAL_DRIVE,
            geometry_type=RobotGeometryType.BOX,
            geometry_params=[0.61, 0.37, 0.4],
            ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=2.5, max_decel=7.5),
            ctrl_omega_limits=AngularCtrlLimits(
                max_vel=1.5, max_acc=2.5, max_decel=4.0, max_steer=1.57
            ),
        )
        self.base_frame = "base_link"
```

Both are broadcast to every component at bringup. A recipe that sets its own
`launcher.robot` wins, but `base_frame` does **not** work that way: the plugin's
frame always applies, because its telemetry is stamped in that frame and
anything mounted on the plugin is placed relative to it. The two cannot
disagree. To publish under a different name, rename it on the plugin:

```python
robot = MyRobotPlugin()
robot.base_frame = "chassis"
launcher.add_plugin(robot)
```

```{note}
A robot plugin supplies only `base_frame` — the frame attached to the robot's
body. The **world** frame describes where the robot has been *placed*, which is
a property of the deployment rather than of the robot, so it stays with the
recipe.
```

Name the two independently, so taking one from a plugin does not mean restating
the other:

```python
launcher.world_frame = "odom"       # leaves the body frame to the plugin
launcher.robot_frame = "base_link"  # only meaningful with no robot plugin attached
```

Assigning `launcher.frames = RobotFrames(...)` still works and sets both at
once — but `RobotFrames` carries a default body frame, so naming only the world
frame that way would hand the plugin a frame it has to override. The
single-frame setters avoid the question.

## Sensor Plugins and Frames

A sensor sits somewhere, so it has a frame. Which frame is the plugin's business;
where that frame *sits* is the recipe's.

```python
class InspectionCamera(SensorPlugin):
    def __init__(self, host: str = "192.168.1.50"):
        self.metadata = PluginMetadata(name="HikVision PTZ", vendor="HikVision")
        ...
```

Every sensor gets a frame automatically — `<id>_frame` — so two of the same
camera never collide:

```python
front = InspectionCamera(id="front_cam")                          # front_cam_frame
rear  = InspectionCamera(id="rear_cam", frame_id="rear_optical")  # rear_optical
```

The HOST stamps that frame onto any decoded message whose `header.frame_id` is
empty, so components can place the data without you hard-coding frame names in
your decoders. A decoder that stamps its own frame is never overridden — a
robot's odometry is in the localization frame, and only the decoder knows that.

### Getting the frame into TF

Components transform incoming data from `header.frame_id` into the robot body
frame, which requires that transform to exist. There are two ways to provide it:

**Something else already publishes it** — a URDF and `robot_state_publisher`, or
the sensor's own driver. Declare nothing:

```python
launcher.add_plugin(InspectionCamera(id="front_cam", frame_id="front_camera_link"))
```

**Nothing publishes it** — you bolted a camera on and there is no model of it.
Give the recipe a `Mount` and Sugarcoat publishes the static transform:

```python
from ros_sugar.robot import Mount

launcher.add_plugin(
    camera,
    mount=Mount(parent=robot, xyz=(0.15, 0.0, 0.35), rpy=(0.0, 0.0, 1.57)),
)
```

`parent` takes the robot plugin (using its `base_frame`), another sensor plugin,
or a plain frame name such as the world frame for a fixed sensor. The child is
filled in from the plugin being attached, so a recipe never repeats it.

```{warning}
Mounts are **static**. A sensor with moving parts publishes the dynamic
transform from its own base frame to the moving one *itself*; the recipe then
only describes where that base frame sits, and the two compose in the TF tree.
```

```{note}
In the first form, if nothing actually publishes the frame the lookup never
resolves and the data is used untransformed — with no error. Prefer a `Mount`
unless you are sure the frame is in the tree.
```

## Step 4: Use the Plugin

```python
from ros_sugar.launch import Launcher
from my_robot_plugin import MyRobotPlugin

plugin = MyRobotPlugin()                     # declarative, all args defaulted
launcher = Launcher()
launcher.add_plugin(plugin)
launcher.add_pkg(components=[planner, controller], multiprocessing=True)

# Plugin-provided event + action factories, wired with the Launcher.on() sugar
launcher.on(plugin.events.low_battery(0.15), plugin.actions.sit())

launcher.bringup()
```

`add_pkg` and `add_plugin` may be called in either order — every component
receives every plugin at bringup.

The launcher hosts each plugin, propagates them to every component, and tears
them down on shutdown. During activation each component looks up its
input/output topics in the plugin: a match on a `RosTopicTransport` re-uses the
native subscriber/publisher-swap path, any other transport is bridged through
the feedback bus — components remain unaware their data is not ROS.

## Recipes With Several Plugins

A robot augmented with an inspection camera attaches both. Topics say which
plugin they come from with `use_plugin`:

```python
robot  = MyRobotPlugin()
camera = InspectionCamera(id="insp_cam")

detector = VisionDetector(
    component_name="detector",
    inputs=[
        # the robot plugin
        Topic(name="odom", msg_type="Odometry", use_plugin=True),
        # a specific plugin, by id
        Topic(name="image", msg_type="Image", use_plugin=camera.id),
    ],
)

launcher = Launcher()
launcher.add_plugin(robot)
launcher.add_plugin(camera, mount=Mount(parent=robot, xyz=(0.15, 0.0, 0.35)))
launcher.add_pkg(components=[detector], multiprocessing=True)
launcher.bringup()
```

`use_plugin=True` means the robot plugin, so single-robot recipes are unchanged.
Passing `camera.id` rather than a literal string means a rename cannot leave a
topic pointing at nothing; a name that matches no attached plugin is rejected at
bringup, listing the plugins that are attached.

Each plugin's bus channels are namespaced by its id, so two instances of the same
camera never collide.

```{note}
Namespacing covers the feedback bus, not the ROS graph. If your plugin uses a
`RosTopicTransport`, the topic name belongs to the driver — two instances must
be told which topic each reads, usually with a constructor argument.
```

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
plugin.describe()         # -> a JSON-serializable tree of all of the above,
                          #    including the plugin's role
plugin.id                 # -> identity within a recipe
plugin.role               # -> PluginRole.ROBOT | PluginRole.SENSOR (read-only)
```

```bash
python -m ros_sugar.robot inspect my_robot_plugin:MyRobotPlugin
```

## Full Example

[automatika-robotics/robot-plugin-example](https://github.com/automatika-robotics/robot-plugin-example)
is the reference template: one robot spanning UDP, ROS-topic and ROS-service
interfaces, with a mock robot and an end-to-end test suite. Copy it and adapt.
