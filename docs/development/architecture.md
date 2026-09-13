# Architecture Overview

This document describes the internal architecture of Sugarcoat for developers contributing to or extending the framework.

## Package Layout

| Package | Contents |
|:--------|:---------|
| `ros_sugar.core` | `BaseComponent`, `Monitor`, `Event`, `Action`, `Status`, `Fallback` / `ComponentFallbacks` |
| `ros_sugar.io` | `Topic`, `Publisher`, the `SupportedType` registry and callbacks, and the `LaserScanData` / `PointCloudData` / `CameraIntrinsics` containers |
| `ros_sugar.config` | `attrs`-based configuration: `BaseAttrs`, `BaseComponentConfig`, `QoSConfig`, validators, and the robot description (`RobotConfig`, `RobotFrames`, control limits) |
| `ros_sugar.launch` | `Launcher`, the multiprocess entry point (`executable_main`), the in-process launch actions, and the system-info serializer the UI reads |
| `ros_sugar.robot` | The plugin framework: `RobotPlugin` / `SensorPlugin`, transports, the feedback bus and shared-memory ring, mounts, driver-process and mapping declarations |
| `ros_sugar.ui_node` | The UI node, its JSON/WebSocket API, and the optional FastHTML browser front-end |
| `ros_sugar.condition`, `ros_sugar.actions`, `ros_sugar.base_clients`, `ros_sugar.tf`, `ros_sugar.utils` | Condition expressions, the standard action factories, service/action client handlers, TF lookup helpers, and the decorators |

## Core Module Structure

The `ros_sugar.core` package exposes the primary building blocks:

| Class | Base Class | Role |
|:------|:-----------|:-----|
| `BaseComponent` | `rclpy.lifecycle.Node` | Managed lifecycle execution unit |
| `Monitor` | `rclpy.node.Node` | Event evaluation and component supervision |
| `Event` | _(standalone)_ | Condition-based trigger on topic data |
| `Action` | _(standalone)_ | Callable dispatched when an event fires |
| `Status` | _(standalone)_ | Health status wrapper around `ComponentStatus` msg |
| `Fallback` / `ComponentFallbacks` | _(attrs / standalone)_ | Failure recovery actions |

### BaseComponent

`BaseComponent` extends `rclpy.lifecycle.Node` and is the primary unit of execution. It wraps lifecycle management, declarative I/O wiring, type-safe configuration via `attrs`, health status broadcasting, and fallback handling into a single class.

Key constructor parameters:

```python
BaseComponent(
    component_name: str,
    inputs: Optional[Sequence[Topic]] = None,
    outputs: Optional[Sequence[Topic]] = None,
    config: Optional[BaseComponentConfig] = None,
    config_file: Optional[str] = None,
    callback_group: Optional[CallbackGroup] = None,
    fallbacks: Optional[ComponentFallbacks] = None,
    main_action_type: Optional[type] = None,
    main_srv_type: Optional[type] = None,
)
```

A component is a plain Python object until its ROS node is initialized. The `Launcher` constructs components in the launcher process, then either initializes the node in a thread of that process or serializes the component's construction arguments (config, topics, events, fallbacks, plugins) into the command line of a separate process, where `executable_main` rebuilds it. Recipe-level knobs that are not part of the config live directly on the component, such as `launch_prefix`, a command prefix (`taskset`, `nice`, `chrt`, a profiler) applied to the component's process under multiprocess launch.

### Monitor

`Monitor` extends `rclpy.node.Node` (a standard, non-lifecycle node). It is responsible for:

- Subscribing to all registered `Event` topics and evaluating conditions via an event blackboard (`EventBlackboardEntry` cache).
- Receiving decoded robot-plugin feedback for topics that have no ROS subscription: the plugin HOST registers them with `register_external_topic` and injects each decoded message through `feed_external_topic`, so events over plugin telemetry evaluate exactly as over ROS topics.
- Creating service clients for component reconfiguration and lifecycle transitions, and activating components on start once they are discovered on the graph.
- Invoking component methods at runtime via `ExecuteMethod` service clients.
- Emitting `InternalEvent` instances back to the `Launcher` so the corresponding `Action` can be dispatched via the ROS launch event system.
- Broadcasting the static transforms declared by mounts on `/tf_static`.

Health status is not consumed by the Monitor. Each component evaluates its own status and runs its fallbacks on its own timer (see {doc}`event_system`); process-level crash recovery is the Launcher's `on_process_fail`. When using the `Launcher`, the `Monitor` is created and configured automatically; users do not need to instantiate it directly.

### Launcher

`Launcher` (in `ros_sugar.launch.launcher`) provides a Pythonic alternative to `ros2 launch`. Its recipe-facing API:

| Method / property | Purpose |
|:------------------|:--------|
| `add_pkg(components, package_name, executable_entry_point, events_actions, multiprocessing, ...)` | Add components from one package; `multiprocessing=True` runs each in its own process (requires the package name and entry point) |
| `add_plugin(plugin, mount=None)` | Attach a robot or sensor plugin; a `Mount` places a sensor plugin in TF |
| `add_ros_node(package, executable, ...)` / `include_launch_file(package, launch_file, launch_args)` | Bring up external ROS nodes and launch files alongside the components, each in its own process |
| `on(event, action)` | Register an event/action pair; sugar for `events_actions` |
| `on_process_fail(max_retries)` | Respawn a multiprocess component that exits unexpectedly |
| `enable_ui(inputs, outputs, port, serve_browser, ...)` | Start the UI node with its JSON/WebSocket API and, optionally, the browser front-end |
| `robot`, `frames`, `robot_frame`, `world_frame` | Broadcast the robot description and frame names to every component; a robot plugin supplies defaults for `robot` and the body frame |
| `bringup(config_file=None, introspect=False, launch_debug=False)` | Build the launch description and run it, blocking until shutdown |

`bringup()` proceeds in this order: validate that every `Topic(use_plugin=...)` names an attached plugin; hand every plugin to every component; apply the robot plugin's `robot_config` and `base_frame` unless the recipe set its own; route events to their owners; create the UI node and the Monitor; start the shared feedback bus, then for each plugin its declared driver processes and its HOST (transports, decoders and heartbeats); publish mounts as static TF; build one launch action per component (thread or process); run the `LaunchService`. On exit it closes every plugin HOST, then the shared bus and the shared-memory segments.

## Component Lifecycle

`BaseComponent` follows the ROS 2 managed lifecycle with four transition callbacks:

```
[Unconfigured] --on_configure--> [Inactive] --on_activate--> [Active]
     ^                               |                          |
     |                               |<---on_deactivate---------|
     |<------on_cleanup--------------|
```

### on_configure

Called when the component transitions from **Unconfigured** to **Inactive**. It loads the configuration file if one was given, calls `custom_on_configure()`, and resets the health status to healthy.

### on_activate

Called when the component transitions from **Inactive** to **Active**. This is where the ROS resources are created, in this order:

1. Robot plugin adaptation: inputs and outputs that opted in with `use_plugin` are bound to the plugin's feedback and commands (see {doc}`custom_robot_plugin`). This runs first because it may replace entries in `self.callbacks` and `self.publishers_dict`.
2. `init_variables()`, the hook for component state and for declaring which frame each input should be transformed into.
3. Subscriptions for the declared `inputs` (each `Topic` wired to a `GenericCallback`), publishers for the declared `outputs`, the built-in services (parameter change, topic replacement, configure-from-file, execute-method), service and action clients, the main action server or service, the subscriptions that feed fallbacks, and finally the execution timer and the fallback-check timer.
4. Event management and external processors are attached, and `custom_on_activate()` runs.

### on_deactivate

Transitions from **Active** back to **Inactive**. Timers, servers, clients, subscriptions and publishers are destroyed and TF lookups are paused; `custom_on_deactivate()` runs.

### on_cleanup

Transitions from **Inactive** back to **Unconfigured**. Override `custom_on_cleanup()` to release resources and reset internal state.

## IO Module

The `ros_sugar.io` package handles typed topic communication.

### Topic

`Topic` is a descriptor that binds a ROS topic name to a `SupportedType`. It carries the topic name (a leading `/` is stripped), message type (a class or its name as a string), QoS profile, `data_timeout` (how long the event system holds a message before treating it as stale), and `use_plugin`: `True` binds the topic to the robot plugin, a string binds it to the plugin with that id, `False` keeps it a plain ROS topic. Topics are declared on components as `inputs` and `outputs` and are wired during activation.

### Publisher

`Publisher` wraps `rclpy.publisher.Publisher` and adds the `SupportedType.convert()` step so that components can publish Python-native data (e.g., `numpy` arrays) without manually constructing ROS messages. After conversion it stamps the header with the node clock and the `frame_id` passed to `publish()`. Pre-processors can be attached to transform data before conversion.

### SupportedType and callbacks

`SupportedType` is the base class for the type system. Each subclass maps a ROS message type (`_ros_type`), a callback class that turns messages into Python data (`callback`), a conversion function that produces the ROS message from Python data (`convert`), a UI streaming mode (`_ui_rate_sampled`), and optional shared-memory hooks for large payloads. The sensor callbacks return the containers in `ros_sugar.io.datatypes` (`LaserScanData`, `PointCloudData`, `CameraIntrinsics`). See {doc}`custom_types` for details on extending it.

### Frames

A component asks for an input in a given frame with `transform_input_to(topic_name, goal_frame)`. The source frame is read from each message header, so nothing about sensor frames is configured; the component's callback is handed a transform resolver, and the spatial callbacks return their data already transformed. All frame pairs share one TF buffer per component, so the node subscribes to `/tf` and `/tf_static` exactly once. Frame names come from `config.frames` (`RobotFrames`: `robot_base` and `world`), which the Launcher sets from the recipe or from the robot plugin.

## Robot Plugins

A plugin adapts a recipe to specific hardware without changing component code. A `RobotPlugin` (exactly one per recipe) or `SensorPlugin` (any number) declares transports (ROS topic, ROS service, UDP, HTTP, vendor SDK), feedbacks (decoders producing ROS messages), commands (encoders producing wire payloads), action and event factories, and optionally the robot description, the placement of its built-in sensors, the driver processes it depends on, and how the robot is mapped.

At bringup the Launcher runs a **HOST** for each plugin in its own process: it opens the transports, decodes telemetry once, publishes it on a feedback bus, and feeds the Monitor's blackboard. Under multithreaded launch the bus is in-process and components consume the HOST's decoded messages directly. Under multiprocess launch the bus is a Unix socket, each component process rebuilds a **CLIENT** plugin from a serialized spec, and large feedbacks (images, point clouds) cross the boundary through a shared-memory ring rather than CDR over the socket. Components bind inputs and outputs to plugin feedbacks and commands during activation. See {doc}`custom_robot_plugin`.

## Callback Groups

`BaseComponent` uses ROS 2 callback groups to control concurrency:

- **`MutuallyExclusiveCallbackGroup`** -- Default for service callbacks; ensures serial execution.
- **`ReentrantCallbackGroup`** -- Used when the component needs concurrent subscription callbacks (e.g., multiple sensor streams processed in parallel).

The callback group can be specified at construction via the `callback_group` parameter.

## Key Decorators

### @component_action

Defined in `ros_sugar.utils.component_action`. Marks a method as an action that can be dispatched by the event system. The decorator enforces:

1. The method belongs to a `LifecycleNode` instance.
2. The return type annotation is `bool` or `None`.
3. If `active=True`, the component must be in the **Active** lifecycle state.

Can be used bare (`@component_action`) or with parameters (`@component_action(description={...}, active=True)`). The optional `description` parameter accepts an OpenAI-compatible tool/function description dict, used when actions are exposed as tools to an orchestrating LLM.

```python
from ros_sugar.utils import component_action

class MyComponent(BaseComponent):
    @component_action
    def stop_motors(self) -> bool:
        # ... stop logic ...
        return True

    @component_action(description={
        "type": "function",
        "function": {
            "name": "stop_motors",
            "description": "Immediately stop all motors.",
        },
    })
    def stop_motors_with_desc(self) -> bool:
        ...
```

### @component_fallback

Defined in `ros_sugar.utils.component_fallback`. Marks a method as a fallback handler. The decorator verifies that rclpy is initialized and the component is at least in the **Inactive** state (i.e., configured or active). This allows fallbacks to fire even when the component has been deactivated due to an error.

Like `@component_action`, it can be used bare or with a `description` parameter for LLM tool descriptions.

```python
from ros_sugar.utils import component_fallback

class MyComponent(BaseComponent):
    @component_fallback
    def restart(self) -> None:
        self.trigger_deactivate()
        self.trigger_activate()
```

### @action_handler

Defined in `ros_sugar.utils.action_handler`. Used internally to validate that a function returns `SomeEntitiesType` (the ROS launch entity type). This is primarily for functions that integrate directly with the launch event system.

## Monitor Orchestration

At runtime the `Monitor` operates a tight evaluation loop:

1. **Receive** -- Subscription callbacks, and `feed_external_topic` calls from plugin HOSTs, write incoming messages into a shared `Dict[str, EventBlackboardEntry]` (the "blackboard"). Each entry carries a UUID and timestamp for staleness detection.
2. **Evaluate** -- For every registered `Event`, `Monitor` calls `event.check_condition(blackboard)`. The `Condition` tree is evaluated against the cached topic messages. Composite conditions (AND / OR / NOT via `ConditionLogicOp`) are resolved recursively.
3. **Trigger** -- If a condition evaluates to `True`, the event's registered actions are submitted to a shared `ThreadPoolExecutor` for non-blocking execution.
4. **Emit** -- For actions that must be handled at the launch level (lifecycle transitions, process restarts), the `Monitor` emits an `InternalEvent` which is caught by an `OnInternalEvent` handler registered by the `Launcher`.

## Launcher Process Graph

The `Launcher` supports two execution modes, chosen per `add_pkg` call:

### Multi-Threaded

All components run in the same process. Each component gets its own callback group. The `Launcher` uses a `MultiThreadedExecutor` to spin all nodes concurrently. This is simpler but shares a single fault domain. Plugin feedback uses an in-process bus, and a `launch_prefix` set on such a component has no effect (the launcher warns).

### Multi-Process

Each component is launched as a separate ROS 2 process via `ExecuteProcess`. The `Launcher` communicates with components through ROS services and the `Monitor`'s topic subscriptions. This provides process isolation -- a crash in one component does not bring down the others, and `on_process_fail` can respawn it. Plugin feedback uses a socket bus with the shared-memory fast path for large messages, and `launch_prefix` applies.

External nodes added with `add_ros_node`, launch files included with `include_launch_file`, and driver processes declared by plugins always run in their own processes, in either mode.

In both modes, the `Monitor` node runs in the main launcher process and coordinates lifecycle transitions via `LifecycleTransition` launch actions.
