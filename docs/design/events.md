# Events

**Dynamic behavior switching based on real-time environmental context.**

Sugarcoat's Event-Driven architecture enables dynamic behavior switching based on real-time environmental context. This allows robots to react instantly to changes in their internal state or external environment without complex, brittle if/else chains.

An Event in Sugarcoat monitors a specific **ROS2 Topic**, and defines a triggering condition based on the incoming topic data. You can write natural Python expressions (e.g., topic.msg.data > 5) to define exactly when an event should trigger the associated Action(s).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`hub;1.5em;sd-text-primary` Compose Logic - </span> Combine triggers using simple Pythonic syntax (`(lidar_clear) & (goal_seen)`).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`sync;1.5em;sd-text-primary` Fuse Data - </span> Monitor multiple topics simultaneously via a synchronized **Blackboard** that ensures data freshness.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`speed;1.5em;sd-text-primary` Stay Fast - </span> All evaluation happens asynchronously in a dedicated worker pool. Your main component loop **never blocks**.


:::{admonition} Think in Behaviors
:class: tip
Events are designed to be read like a sentence:
*"If the battery is low AND we are far from home, THEN navigate to the charging dock."*
:::

:::{tip} Events can be paired with Sugarcoat [`Action`](actions.md)(s) or with any standard [ROS2 Launch Action](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
:::

## Defining Events

The Event API uses a fluent, expressive syntax that allows you to access ROS2 message attributes directly via `topic.msg`.

### Basic Single-Topic Event

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

# 1. Define the Source
# `data_timeout` parameter is optional. It ensures data is considered "stale" after 0.5s
battery = Topic(name="/battery_level", msg_type="Float32", data_timeout=0.5)

# 2. Define the Event
# Triggers when percentage drops below 20%
low_batt_event = Event(battery.msg.data < 20.0)
```

### Composed Conditions (Logic & Multi-Topic)

You can combine multiple conditions using standard Python bitwise operators (`&`, `|`, `~`) to create complex behavioral triggers. Events can also span multiple different topics. Sugarcoat automatically manages a "Blackboard" of the latest messages from all involved topics, ensuring synchronization and data "freshness".

- **Example**: Trigger a "Stop" event only if an obstacle is detected AND the robot is currently in "Auto" mode.

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

lidar_topic = Topic(name="/person_detected", msg_type="Bool", data_timeout=0.5)
status_topic = Topic(name="/robot_mode", msg_type="String", data_timeout=60.0)

# Complex Multi-Topic Condition
emergency_stop_event = Event((lidar_topic.msg.data.is_true()) & (status_topic.msg.data == "AUTO"))
```

:::{admonition} Handling Stale Data
:class: warning
When combining multiple topics, data synchronization is critical. Use the `data_timeout` parameter on your `Topic` definition to ensure you never act on old sensor data.
:::

## Event Configuration

Refine *when* and *how* the event triggers using these parameters:

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`change_circle` On Change (`on_change=True`)</span> - Triggers **only** when the condition transitions from `False` to `True` (Edge Trigger). Useful for state transitions (e.g., "Goal Reached") rather than continuous firing.
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`all_inclusive` On Any (`Topic`)</span> - If you pass the `Topic` object itself as the condition, the event triggers on **every received message**, regardless of content.
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`looks_one` Handle Once (`handle_once=True`)</span> - The event will fire exactly one time during the lifecycle of the system. Useful for initialization sequences.
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`timer` Event Delay (`keep_event_delay=2.0`)</span> - Prevents rapid firing (debouncing). Ignores subsequent triggers for the specified duration (in seconds).


## Supported Conditional Operators

You can use standard Python operators or specific helper methods on any topic attribute to define the <span class="text-blue">**event triggering condition**</span>

| Operator / Method | Description | Example |
| :--- | :--- | :--- |
| **`==`**, **`!=`** | Equality checks. | `topic.msg.status == "IDLE"` |
| **`>`**, **`>=`**, **`<`**, **`<=`** | Numeric comparisons. | `topic.msg.temperature > 75.0` |
| **`.is_true()`** | Boolean True check. | `topic.msg.is_ready.is_true()` |
| **`.is_false()`**, **`~`** | Boolean False check. | `topic.msg.is_ready.is_false()` or `~topic.msg.is_ready` |
| **`.is_in(list)`** | Value exists in a list. | `topic.msg.mode.is_in(["AUTO", "TELEOP"])` |
| **`.not_in(list)`** | Value is not in a list. | `topic.msg.id.not_in([0, 1])` |
| **`.contains(val)`** | String/List contains a value. | `topic.msg.description.contains("error")` |
| **`.contains_any(list)`** | List contains *at least one* of the values. | `topic.msg.error_codes.contains_any([404, 500])` |
| **`.contains_all(list)`** | List contains *all* of the values. | `topic.msg.detections.labels.contains_all(["window", "desk"])` |
| **`.not_contains_any(list)`** | List contains *none* of the values. | `topic.msg.active_ids.not_contains_any([99, 100])` |


## Usage Examples

### 1. Automatic Adaptation (Terrain Switching)
Scenario: A perception or ML node publishes a string to `/terrain_type`. We want to change the robot's gait when the terrain changes.

```{code-block} python
:caption: quadruped_controller.py
:linenos:

from typing import Literal
from ros_sugar.component import BaseComponent

class QuadrupedController(BaseComponent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Some logic

    def switch_gait_controller(self, controller_type: Literal['stairs', 'sand', 'snow', 'gravel']):
        self.get_logger().info("New terrain detected! Switching gait.")
        # Logic to change controller parameters...
```

```{code-block} python
:caption: quadruped_controller_recipe.py
:linenos:

from my_pkg.components import QuadrupedController
from ros_sugar.core import Event, Action
from ros_sugar.io import Topic
from ros_sugar import Launcher

quad_controller = QuadrupedController(component_name="quadruped_controller")

# Define the Event Topic
terrain_topic = Topic(name="/terrain_type", msg_type="String")

# Define the Event
# Logic: Trigger when the detected terrain changes
# on_change=True ensures we only trigger the switch the FIRST time stairs are seen.
# Add an optional delay to prevent rapid event triggering
event_terrain_changed = Event(terrain_topic,on_change=True, keep_event_delay=60.0)

# Define the Action
# Call self.switch_gait_controller() when triggered and pass the detected terrain to the method
change_gait_action = Action(method=self.activate_stairs_controller, args=(terrain_topic.msg.data))

# Register
my_launcher = Launcher()
my_launcher.add_pkg(
            components=[quad_controller],
            events_actions={stairs_event: change_gait_action},
        )
```


### 2. An Autonomous Drone
Scenario: An autonomous drone **stops** if an obstacle is close OR the bumper is hit. It also sends a warning if the battery is low AND we are far from the land.

```python
from ros_sugar.core import Event, Action
from ros_sugar.io import Topic

# --- Topics ---
proximity_sensor   = Topic(name="/radar_front", msg_type="Float32", data_timeout=0.2)
bumper  = Topic(name="/bumper", msg_type="Bool", data_timeout=0.1)
battery = Topic(name="/battery", msg_type="Float32")
location = Topic(name="/pose", msg_type="Pose")

# --- Conditions ---
# 1. Safety Condition (Composite OR)
# Stop if proximity_sensor < 0.2m OR Bumper is Hit
is_danger = (proximity_sensor.msg.data < 0.2) | (bumper.msg.data.is_true())

# 2. Return Home Condition (Composite AND)
# Return if Battery < 20% AND Distance > 100m
needs_return = (battery.msg.data < 20.0) & (location.position.z > 100.0)

# --- Events ---
safety_event = Event(is_danger)

return_event = Event(needs_return, on_change=True)
```


## Next Steps

Now that you understand Events, learn how to attach them to Actions.

::::{grid} 1
:gutter: 3

:::{grid-item-card} {material-regular}`call_to_action;1.5em;sd-text-primary` Actions
:link: actions
:link-type: doc
:class-card: sugar-card

Learn how to execute context-aware methods when an Event triggers.
:::
::::
