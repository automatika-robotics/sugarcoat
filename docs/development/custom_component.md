# Creating a New BaseComponent

This guide walks through subclassing `BaseComponent` to create a new component type. This is the primary extension point in Sugarcoat — both [EmbodiedAgents](https://github.com/automatika-robotics/embodied-agents) and [Kompass](https://github.com/automatika-robotics/kompass) build their component layers by subclassing `BaseComponent`. Read {doc}`architecture` first for the overall design.

## Choosing the Right Base

`BaseComponent` wraps a ROS 2 Lifecycle Node with declarative I/O, type-safe configuration, health status, and fallback recovery. Subclass it when you need a new component abstraction that downstream packages will further extend.

If you are building an end-user component (not a framework layer), consider subclassing one of the higher-level components from EmbodiedAgents or Kompass instead.

## Constructor

Your subclass constructor should accept inputs, outputs, config, and a trigger, then call `super().__init__()`:

```python
from typing import Optional, Sequence
from ros_sugar.core import BaseComponent
from ros_sugar.io import Topic
from ros_sugar.config import BaseComponentConfig

class MyComponent(BaseComponent):
    def __init__(
        self,
        component_name: str = "my_component",
        inputs: Optional[Sequence[Topic]] = None,
        outputs: Optional[Sequence[Topic]] = None,
        config: Optional[BaseComponentConfig] = None,
        config_file: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            component_name=component_name,
            inputs=inputs,
            outputs=outputs,
            config=config or BaseComponentConfig(),
            config_file=config_file,
            **kwargs,
        )
```

### Constructor Parameters

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `component_name` | `str` | ROS 2 node name |
| `inputs` | `Sequence[Topic]` | Input topics the component subscribes to |
| `outputs` | `Sequence[Topic]` | Output topics the component publishes to |
| `config` | `BaseComponentConfig` | Component configuration (loop rate, run type, etc.) |
| `config_file` | `str` | Path to YAML/JSON/TOML config file (alternative to `config`) |
| `callback_group` | `CallbackGroup` | ROS 2 callback group; defaults to `ReentrantCallbackGroup` |
| `main_action_type` | `type` | ROS 2 action type (required when run type is `ACTION_SERVER`) |
| `main_srv_type` | `type` | ROS 2 service type (required when run type is `SERVER`) |

## Execution Methods

These are the methods your subclass must or can implement. The one you **must** implement depends on the component's run type.

### `_execution_step()` — TIMED and EVENT modes

Called every timer cycle at `loop_rate` frequency. This is where your core logic lives:

```python
def _execution_step(self):
    if not self.got_all_inputs():
        missing = self.get_missing_inputs()
        self.health_status.set_fail_system(topic_names=missing)
        return

    sensor = self.callbacks["sensor"].get_output()
    if sensor is None:
        return

    result = self.process(sensor)
    self.health_status.set_healthy()
    self.publishers_dict["output"].publish(result)
```

### `main_action_callback()` — ACTION_SERVER mode

Called when an action goal is received. Required when `run_type == ComponentRunType.ACTION_SERVER`:

```python
def main_action_callback(self, goal_handle):
    result = self.process_goal(goal_handle.request)
    goal_handle.succeed()
    return result
```

### `main_service_callback()` — SERVER mode

Called when a service request is received. Required when `run_type == ComponentRunType.SERVER`:

```python
def main_service_callback(self, request, response):
    response.result = self.compute(request.data)
    return response
```

## Lifecycle Hooks

Override these to run custom logic during lifecycle transitions. All are optional with empty defaults:

| Hook | Called when | Common use |
|:-----|:-----------|:-----------|
| `init_variables()` | Start of activation, after plugin adaptation and before subscribers exist | Initialize state variables; declare input frames with `transform_input_to()` |
| `custom_on_configure()` | After configuration | Set up internal resources |
| `custom_on_activate()` | After activation | Start background tasks, create TF listeners |
| `custom_on_deactivate()` | After deactivation | Pause background tasks |
| `custom_on_cleanup()` | During cleanup | Release resources |
| `custom_on_shutdown()` | During shutdown | Final cleanup |
| `custom_on_error()` | On transition error | Error-specific handling |

The base class resets health status to healthy on `configure`, `activate`, and `deactivate`. On `error`, it sets `set_fail_component()`. You generally don't need to manage status in lifecycle hooks.

```python
class MyComponent(BaseComponent):
    def init_variables(self):
        self.counter = 0
        self.buffer = []

    def custom_on_configure(self):
        self.get_logger().info("Configured")

    def custom_on_activate(self):
        self.get_logger().info("Active")
```

## Accessing Inputs and Outputs

### Reading from Inputs

Input data is available through `self.callbacks`, a dict mapping topic names to callback objects:

```python
# Check if all inputs have received at least one message
if self.got_all_inputs():
    data = self.callbacks["my_topic"].get_output()

# Check specific inputs only
if self.got_all_inputs(inputs_to_check=["critical_topic"]):
    ...

# Exclude optional inputs from the check
if self.got_all_inputs(inputs_to_exclude=["optional_topic"]):
    ...

# Get list of topics that haven't received data yet
missing = self.get_missing_inputs()
```

### Publishing to Outputs

Output publishers are available through `self.publishers_dict`:

```python
# Publish data (automatically converted via SupportedType.convert())
self.publishers_dict["output"].publish(result)

# With frame_id for stamped messages
self.publishers_dict["pose"].publish(pose_data, frame_id="map")
```

## Working with Frames

Every spatial input (scans, clouds, grids, odometry, paths, poses, points) can be delivered already expressed in the frame the component's algorithm needs. Declare the target frame in `init_variables()`, before the subscribers are created:

```python
def init_variables(self):
    # Scans arrive in whatever frame the sensor stamps them with; the
    # controller wants them in the robot body frame
    self.transform_input_to("scan", self.config.frames.robot_base)
    # A fixed map only needs one lookup
    self.transform_input_to("map", self.config.frames.world, static_tf=True)
```

The source frame is read from each message's `header.frame_id`, so no sensor frame is configured anywhere. From then on `self.callbacks["scan"].get_output()` returns the data in the body frame; while the transform is still unavailable the data is used untransformed. All frame pairs share one TF buffer per component, so the node subscribes to `/tf` and `/tf_static` once no matter how many sensors it tracks.

For lookups outside an input, `self.get_transform(source_frame, goal_frame, static_tf=False)` returns the `TransformStamped` between two frames, or `None` until it has been resolved. Repeated calls for the same pair are cheap; each pair gets a cached listener polling at `TFListenerConfig.lookup_rate`.

Frame names come from `config.frames`, a `RobotFrames` with `robot_base` (default `"base_link"`) and `world` (default `"map"`). The Launcher sets them for every component from `launcher.frames`, `launcher.robot_frame` / `launcher.world_frame`, or the attached robot plugin's `base_frame`, so a component should read them rather than hard-code frame names.

## Run Types

Set the run type via configuration to control how `_execution_step()` is triggered:

| Run Type | Behavior | Requires |
|:---------|:---------|:---------|
| `TIMED` | Fires `_execution_step()` at `loop_rate` Hz | Nothing extra |
| `EVENT` | Fires on topic/event trigger | Event wiring at Launcher level |
| `SERVER` | Fires `main_service_callback()` on service request | `main_srv_type` parameter |
| `ACTION_SERVER` | Fires `main_action_callback()` on action goal | `main_action_type` parameter |

```python
from ros_sugar.config import BaseComponentConfig, ComponentRunType

# Timed: execute at 50 Hz
config = BaseComponentConfig(loop_rate=50.0)

# Server: respond to service requests
component = MyComponent(
    config=BaseComponentConfig(_run_type=ComponentRunType.SERVER),
    main_srv_type=MyService,
)
```

## Configuration

`BaseComponentConfig` carries the parameters every component has:

| Field | Default | Meaning |
|:------|:--------|:--------|
| `loop_rate` | `100.0` | Execution timer rate in Hz (`TIMED` run type) and health-status publishing rate |
| `fallback_rate` | `100.0` | Rate at which the component checks its health status and runs fallbacks |
| `log_level` | `"info"` | Component logger level |
| `rclpy_log_level` | `"warn"` | rclpy / RMW logger level |
| `robot` | `None` | `RobotConfig`: model type, geometry and control limits, set by the Launcher from `launcher.robot` or the robot plugin |
| `frames` | `RobotFrames()` | Body and world frame names, set by the Launcher |
| `wait_for_restart_time` | `6000.0` | Seconds to wait for the node to come back after a `restart()` |
| `executor_spin_timeout` | `0.01` | Spin timeout of the in-process executor (multithreaded launch) |
| `_run_type` | `TIMED` | See [Run Types](#run-types) |
| `_lifecycle_state_transition_timeout` | `10.0` | Seconds to wait on a lifecycle transition |

### Extending BaseComponentConfig

Define a custom config class using `attrs` for component-specific parameters:

```python
from attrs import define, field
from ros_sugar.config import BaseComponentConfig, base_validators

@define(kw_only=True)
class MyConfig(BaseComponentConfig):
    threshold: float = field(default=0.5, validator=base_validators.in_range(0.0, 1.0))
    window_size: int = field(default=10, validator=base_validators.gt(0))
    mode: str = field(default="fast", validator=base_validators.in_(["fast", "accurate"]))
```

Key points:

- Always use `@define(kw_only=True)`.
- Use `base_validators` for field validation (`gt`, `in_range`, `in_`).
- Configs are serializable to YAML/JSON/TOML via `to_file()` / `from_file()`.

### Loading from File

```python
# At construction
component = MyComponent(config_file="/path/to/config.yaml")

# At runtime
component.config_from_file("/path/to/config.yaml")

# On a config object directly (YAML, JSON or TOML by extension)
config = MyConfig()
config.from_file("/path/to/config.yaml", nested_root_name="my_component", get_common=True)
```

YAML structure:

```yaml
/**:                     # Common parameters, merged into every component (get_common)
  fallback_rate: 10.0

my_component:            # Must match component_name
  loop_rate: 50.0
  threshold: 0.8
  window_size: 20
```

`from_file` returns `False` when the file has no section for the component. Configs also round-trip through `to_json()` / `from_json()`, which is how a component's configuration reaches its process under multiprocess launch. Algorithm configurations registered on a component travel as explicitly set fields only (`explicit_fields`), so values the component computes for itself are not overwritten by defaults.

## Restricting Allowed Topics

Use `AllowedTopics` to enforce which message types a component accepts:

```python
from ros_sugar.io import AllowedTopics
from ros_sugar.io.supported_types import Image, String, Float64

class MyComponent(BaseComponent):
    def __init__(self, **kwargs):
        self.allowed_inputs = {
            "Required": AllowedTopics(types=[Image], number_required=1),
            "Optional": AllowedTopics(types=[String], number_required=0, number_optional=1),
        }
        self.allowed_outputs = {
            "Required": AllowedTopics(types=[Float64], number_required=1),
        }
        super().__init__(**kwargs)
```

The validation runs during initialization and raises if required topics are missing or types don't match.

## Custom Actions and Fallbacks

### Defining Component Actions

Use the `@component_action` decorator to mark methods as dispatchable actions. These can be used as fallback targets or wired to events:

```python
from ros_sugar.utils import component_action, component_fallback

class MyComponent(BaseComponent):
    @component_action
    def reset_buffer(self) -> bool:
        self.buffer = []
        return True

    @component_fallback
    def emergency_stop(self):
        self.publishers_dict["velocity"].publish(0.0)
```

- `@component_action`: Validates lifecycle state before execution. Return type should be `bool` or `None`. When an action is invoked remotely through the `ExecuteMethod` service, `False` is reported as a failure and anything else as success; see the [built-in services](../advanced/srvs.md).
- `@component_fallback`: Validates the component is in a valid state (active, inactive, or activating).

### Tool Descriptions for LLM Orchestration

Both decorators accept an optional `description` parameter for providing an OpenAI-compatible tool/function description. This is used when component actions are exposed as tools to an orchestrating LLM (e.g. via EmbodiedAgents):

```python
class MyComponent(BaseComponent):
    @component_action(description={
        "type": "function",
        "function": {
            "name": "reset_buffer",
            "description": "Clears the internal data buffer and resets processing state.",
        },
    })
    def reset_buffer(self) -> bool:
        self.buffer = []
        return True

    @component_fallback(description={
        "type": "function",
        "function": {
            "name": "emergency_stop",
            "description": "Immediately stops all motor output.",
        },
    })
    def emergency_stop(self):
        self.publishers_dict["velocity"].publish(0.0)
```

When `description` is omitted, the method's docstring is used as the description. The `active` parameter is also supported on `@component_action` to require the Active lifecycle state:

```python
@component_action(description={...}, active=True)
def move_forward(self) -> bool:
    ...
```

### Built-in Actions

Every component inherits these actions that can be used directly in fallbacks or events:

| Action | Description |
|:-------|:------------|
| `start()` | Lifecycle activate |
| `stop()` | Lifecycle deactivate |
| `restart(*, wait_time=None)` | Stop then start (`wait_time` is keyword-only) |
| `reconfigure(new_config, keep_alive=False)` | Apply new config |
| `set_param(name, value, keep_alive=True)` | Change one parameter |
| `set_params(names, values, keep_alive=True)` | Change multiple parameters |
| `broadcast_status()` | Publish current health status |
| `inspect_component()` | Return a string summary of the component's config, inputs, and outputs |

### Custom Action/Service Names

By default, the main action server and service names are derived from the type name (e.g. `component_name/my_action_type`). You can override them with the `main_action_name` and `main_srv_name` setters:

```python
component = MyComponent(
    main_action_type=MyAction,
    config=BaseComponentConfig(_run_type=ComponentRunType.ACTION_SERVER),
)
component.main_action_name = "custom/action_name"
component.main_srv_name = "custom/service_name"
```

### Extension Points for Derived Packages

Subclasses can override these methods to support dynamic I/O reconfiguration at the Launcher level:

| Method | Description |
|:-------|:------------|
| `set_input(**kwargs) -> bool` | Update an input topic by keyword. Return `True` if the input was found and updated. |
| `set_output(**kwargs) -> bool` | Update an output topic by keyword. Return `True` if the output was found and updated. |
| `get_ros_entrypoints() -> Dict` | Return a dict of additional ROS services and actions the component exposes. |

These are called by the `Launcher.inputs()` and `Launcher.outputs()` methods to propagate settings across all components.

## Placing the Process

Under multiprocess launch every component has a process of its own, and `launch_prefix` prepends a command to it: CPU pinning, a scheduling class, or a profiler.

```python
vision = VisionComponent(component_name="vision")
vision.launch_prefix = "taskset -c 4-7"        # keep it on the performance cores
logger_comp.launch_prefix = "nice -n 10"

launcher.add_pkg(components=[vision, logger_comp], package_name="my_pkg", multiprocessing=True)
```

The prefix has no effect on a component running in a launcher thread; the Launcher warns if one is set there.

## Complete Skeleton

```python
from typing import Optional, Sequence
from attrs import define, field
from ros_sugar.core import BaseComponent, Action
from ros_sugar.io import Topic
from ros_sugar.io.supported_types import Float64, String
from ros_sugar.config import BaseComponentConfig, base_validators
from ros_sugar.utils import component_action
from ros_sugar.launch import Launcher


# --- Config ---
@define(kw_only=True)
class FilterConfig(BaseComponentConfig):
    alpha: float = field(default=0.5, validator=base_validators.in_range(0.0, 1.0))


# --- Component ---
class ExponentialFilter(BaseComponent):
    """Low-pass exponential filter on a float stream."""

    def __init__(
        self,
        component_name: str = "exp_filter",
        inputs: Optional[Sequence[Topic]] = None,
        outputs: Optional[Sequence[Topic]] = None,
        config: Optional[FilterConfig] = None,
        **kwargs,
    ):
        super().__init__(
            component_name=component_name,
            inputs=inputs,
            outputs=outputs,
            config=config or FilterConfig(),
            **kwargs,
        )

    def init_variables(self):
        self._filtered = 0.0

    def _execution_step(self):
        if not self.got_all_inputs():
            self.health_status.set_fail_system(
                topic_names=self.get_missing_inputs()
            )
            return

        raw = self.callbacks["raw_signal"].get_output()
        if raw is None:
            return

        alpha = self.config.alpha
        self._filtered = alpha * raw + (1 - alpha) * self._filtered
        self.health_status.set_healthy()
        self.publishers_dict["filtered_signal"].publish(self._filtered)

    @component_action
    def reset_filter(self) -> bool:
        self._filtered = 0.0
        return True


# --- Usage ---
raw = Topic(name="raw_signal", msg_type=Float64)
filtered = Topic(name="filtered_signal", msg_type=Float64)

filt = ExponentialFilter(
    inputs=[raw],
    outputs=[filtered],
    config=FilterConfig(loop_rate=100.0, alpha=0.3),
)

# Fallback: restart on failure
filt.on_algorithm_fail(
    action=Action(filt.restart),
    max_retries=3,
)

launcher = Launcher()
launcher.add_pkg(components=[filt])
launcher.bringup()
```
