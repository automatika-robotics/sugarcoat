# Events

Sugarcoat's Event-Driven architecture enables dynamic behavior switching based on real-time environmental context. This allows robots to react instantly to changes in their internal state or external environment without complex, brittle if/else chains.

An Event in Sugarcoat monitors a specific **ROS2 Topic**, and defines a triggering condition based on the incoming topic data. You can write natural Python expressions (e.g., topic.msg.data > 5) to define exactly when an event should trigger the associated Action(s).

:::{tip} Events can be paired with Sugarcoat [`Action`](actions.md)(s) or with any standard [ROS2 Launch Action](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
:::

## Defining Events

The pythonic Event API uses a fluent syntax that allows you to access ROS2 message attributes directly via `topic.msg`.

- Basic Syntax

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

# 1. Define the Event Source Topic
sensor = Topic(name="/geometry_point", msg_type="Point")

# 2. Define the Event using an Expression
low_batt_event = Event(
    event_name="high_altitude",
    event_condition=sensor.msg.z >= 20.0
)
```

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

## Event Configuration
The Event class accepts additional arguments to refine the event triggering behavior:

### 1. On Change (on_change=True)
Triggers the event only when the new result of the condition changes.

This is useful for state transitions (e.g., triggering "Goal Reached" only the moment it happens, not continuously while the robot is there).

- Syntax:  ```Event(..., event_condition=(x == y), on_change=True)```

### 2. On Any (event_condition=Topic)
If you pass the Topic object itself instead of a condition, the event triggers on every message received.


### 3. Handle Once (handle_once=True)
If an event should only fire a single time during the lifetime of the system (e.g., initialization triggers).

### 4. Event Delay (keep_event_delay=2.0)
To prevent an event from firing too rapidly (debouncing), use `keep_event_delay` to ignore subsequent triggers for a set duration (in seconds).

## Usage Examples

### 1. Automatic Adaptation (Terrain Switching)
Scenario: A perception or ML node publishes a string to `/terrain_type`. We want to change the robot's gait when it sees stairs.

```{code-block} python
:caption: quadruped_controller.py
:linenos:

from typing import Literal
from ros_sugar.component import BaseComponent

class QuadrupedController(BaseComponent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Some logic

    def activate_stairs_controller(self):
        self.get_logger().info("Stairs detected! Switching gait.")
        # Logic to change controller parameters...

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
# Logic: Trigger when data equals "stairs".
# on_change=True ensures we only trigger the switch the FIRST time stairs are seen.
stairs_event = Event(
    event_name="stairs_detected",
    event_condition=terrain_topic.msg.data == "stairs",
    on_change=True
)

# Define the Action
# Call self.activate_stairs_controller() when triggered
change_gait_action = Action(method=self.activate_stairs_controller)

# Register
my_launcher = Launcher()
my_launcher.add_pkg(
            components=[quad_controller],
            events_actions={stairs_event: change_gait_action},
        )
```

### 2. Intelligent Interaction (Follow to Patrol)
Scenario: A vision system tracks a target. If the target is lost (`target_visible` becomes False, or label 'person' is no longer in the detections list, etc.), the robot should switch to a search/patrol pattern.

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

tracking_status_topic = Topic(name='target_visible', msg_type="Bool")

# Trigger ONLY when target_visible changes from True to False
target_lost_event = Event(
    event_name="target_lost",
    event_condition=tracking_status_topic.msg.data.is_false(),
    on_change=True
)
```


## Dynamic Event Parsers

While basic Events trigger a pre-defined action, (e.g., detecting an obstacle triggers a stop() command), **real-world autonomy often requires the data that triggered the event to determine how to react**.

<span class="text-blue">**Event Parsers** allow you to extract specific information from the triggering ROS2 message, process it, and inject it dynamically as arguments into your Action function.</span>

```{tip}
Your Actions are context-aware by default! The triggering message is always available to any action method via the `msg` argument, enabling **data-driven actions** out of the box. For even greater flexibility, use **Event Parsers** to transform this message data and inject it into specific function arguments, keeping your component methods generic and reusable.
```

### Why use Event Parsers?
- **Data-Driven Actions**: Instead of just knowing that an event occurred, your component receives context about what happened (e.g., knowing the specific "Terrain Type" detected, rather than just "Terrain Changed").

- **Code Reusability**: You can write a single, generic action method (e.g., switch_controller(mode_name)) and use it for dozens of different triggers, rather than writing a separate wrapper function for every possible state.

- **Separation of Concerns**: The logic for extracting data (the parser) is kept separate from the logic for acting on data (the component method).

### How it works

The pipeline transforms a standard event trigger into a parameterized function call:

1. **Trigger**: The Event detects a condition on a Topic.

2. **Parse**: The `add_event_parser` function receives the raw ROS2 message. It extracts the relevant data (e.g., a string, a coordinate, an ID).

3. **Map**: The extracted data is mapped to a specific keyword argument, or a specified argument index of the target Action.

4. **Execute**: The Action is executed with the dynamic data passed in.

### Example

In [the previous example](#1-automatic-adaptation-terrain-switching), actions were hard-coded (e.g., "If stairs, run activate_stairs_controller"). However, often you want a single generic method (e.g., switch_controller) that dynamically adapts based on the data received in the event.

You can achieve this using the `add_event_parser` method on an Action.

Scenario: The perception system publishes various terrain types ("sand", "gravel", "stairs") to `/terrain_type`. We want to trigger the generic `switch_gait_controller` method and pass the detected terrain type as an argument.

```python
# Import the component
from my_pkg.components import QuadrupedController
from ros_sugar.events import OnChange, OnEqual
from ros_sugar.action import Action
from ros_sugar.io import Topic
from ros_sugar import Launcher

quad_controller = QuadrupedController(component_name="quadruped_controller")

# Define the Event Topic (Can be the output of some perception system or an ML model)
terrain_topic = Topic(name="/terrain_type", msg_type="String")

# Define the Event
# Trigger when the terrain changes
terrain_change_event = Event(
    event_name="terrain_changed",
    event_source=terrain_topic,
    on_change=True
)

# Option 1: Define a Helper Parser Function Explicitly
# The Event automatically passes the triggering 'msg' to the action/parser.
def parse_terrain_data(msg: String) -> str:
    """Extracts the data string from the ROS message."""
    return msg.data

# Define the Action with a Parser
# First, define the action targeting the generic method
dynamic_switch_action = Action(method=quad_controller.switch_gait_controller)

# Next, attach the parser.
# - method: The function that processes the incoming ROS msg.
# - output_mapping: The name of the argument in 'switch_gait_controller'
#   that receives the return value of 'parse_terrain_data'.
dynamic_switch_action.add_event_parser(
    method=parse_terrain_data,
    keyword_argument_name="controller_type"
)

# Option2: use the automatic build-in parser
# This option works on types with direct mappings
# This wil automatically get the value for msg.data and pass it to the action method
dynamic_switch_action.add_automatic_parser_from_msg_type(
            input_topic_msg_type=terrain_topic.ros_msg_type, action_argument_type=str
        )

# Register
my_launcher = Launcher()
my_launcher.add_pkg(
            components=[quad_controller],
            events_actions={terrain_change_event: dynamic_switch_action},
        )
```
