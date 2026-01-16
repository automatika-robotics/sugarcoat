# Events

Sugarcoat's Event-Driven architecture enables dynamic behavior switching based on real-time environmental context. This system allows robots to react instantly to changes in their internal state or external environment without complex, brittle if/else chains in your main loop.

An Event in Sugarcoat monitors a specific **ROS2 Topic**. It inspects a specific *Attribute* within that message and compares it against a *Trigger Value*. When the condition is met, the Event triggers its associated [`Action`](actions.md)(s).

:::{tip} Events can be paired with Sugarcoat [`Action`](actions.md)(s) or with any standard [ROS2 Launch Action](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
:::

## Available Event Types

| Event Class | Description | Use Case |
| :--- | :--- | :--- |
| **`OnAny`** | Triggers on **every** message received. | Logging, heartbeat monitoring, continuous data processing. |
| **`OnEqual`** | Triggers when the value **equals** the trigger value. | State matching (e.g., `status == "IDLE"`), detecting specific object IDs. |
| **`OnDifferent`** | Triggers when the value is **not equal** to the trigger value. | Detecting configuration changes or mode mismatches. |
| **`OnGreater`** | Triggers when value **>** trigger value (supports `or_equal=True`). | Altitude limits, temperature warnings, speed thresholds. |
| **`OnLess`** | Triggers when value **<** trigger value (supports `or_equal=True`). | Low battery, proximity alerts, signal strength drop. |
| **`OnChange`** | Triggers whenever the value **changes** from its previous reading. | Reacting to any new command, mode switch, or distinct sensor reading. |
| **`OnChangeEqual`** | Triggers when the value changes and becomes equal to the trigger (change from `!=` to `==`). | Goal reaching (trigger *only* the moment status becomes "ARRIVED"). |
| **`OnContainsAny`** | Triggers if the attribute (list) contains **any** of the trigger values. | Checking if *any* error code in a list matches a known critical error. |
| **`OnContainsAll`** | Triggers if the attribute (list) contains **all** trigger values. | Verifying all required subsystems are present in a status list. |
| **`OnChangeContainsAny`** | Triggers **once** when the list changes to contain **any** of the trigger values (after not containing any of the values). | Alerting when a specific hazard enters a detected objects list. |
| **`OnChangeContainsAll`** | Triggers **once** when the list changes to contain **all** of the trigger values (after not containing all of the values). | Confirming a complex condition is fully met after being partial. |
| **`OnChangeNotContain`** | Triggers **once** when the list changes to **not** contain the trigger values (after containing some or all of the values). | Detecting when a tracked object is lost or a required resource is removed. |

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
from ros_sugar.events import OnChange, OnEqual
from ros_sugar.action import Action
from ros_sugar.io import Topic
from ros_sugar import Launcher

quad_controller = QuadrupedController(component_name="quadruped_controller")

# Define the Event Topic (Can be the output of some perception system or an ML model)
terrain_topic = Topic(name="/terrain_type", msg_type="String")

# Define the Event
# Trigger when the terrain changes to 'stairs'. Uses 'OnChangeEqual' to trigger the switch only the first time stairs are detected
stairs_event = OnChangeEqual(
    event_name="stairs_detected",
    event_source=terrain_topic,
    nested_attributes="data",
    trigger_value="stairs"
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
from ros_sugar.events import OnChangeEqual
from ros_sugar.io import Topic

tracking_status_topic = Topic(name='target_visible', msg_type="Bool")

# Trigger ONLY when target_visible changes from True to False
target_lost_event = OnChangeEqual(
    event_name="target_lost",
    event_source=tracking_status_topic,
    nested_attributes="data",
    trigger_value=False
)
```

### 3. Nested Attributes

You can access deeply nested fields in ROS messages using a list of strings for nested_attributes.

**Example**: Checking the Z position in a *geometry_msgs/PoseStamped* (i.e. access msg.pose.position.z)

```python
high_altitude_event = OnGreater(
    event_name="altitude_limit",
    event_source=pose_topic,
    # Access msg.pose -> .position -> .z
    nested_attributes=["pose", "position", "z"],
    trigger_value=50.0 # meters
)
```

## Advanced Configuration

### Handling Once
If an event should only fire a single time during the lifecycle of the system (e.g., initialization triggers), set handle_once=True.

```python
init_event = OnEqual(..., handle_once=True)
```

### Event Delay (Debouncing)
To prevent an event from firing too rapidly (e.g., sensor noise flickering around a threshold), use keep_event_delay.

```python
# Once triggered, ignore subsequent triggers for 2.0 seconds
stable_event = OnGreater(..., keep_event_delay=2.0)
```

## Dynamic Event Parsers

While basic Events trigger a pre-defined action (e.g., detecting an obstacle triggers a stop() command), **real-world autonomy often requires the data that triggered the event to determine how to react**.

<span class="text-blue">**Event Parsers** allow you to extract specific information from the triggering ROS2 message and inject it dynamically as arguments into your Action function.</span>

### Why use Event Parsers?
- **Data-Driven Actions**: Instead of just knowing that an event occurred, your component receives context about what happened (e.g., knowing the specific "Terrain Type" detected, rather than just "Terrain Changed").

- **Code Reusability**: You can write a single, generic action method (e.g., switch_controller(mode_name)) and use it for dozens of different triggers, rather than writing a separate wrapper function for every possible state.

- **Separation of Concerns**: The logic for extracting data (the parser) is kept separate from the logic for acting on data (the component method).

### How it works

The pipeline transforms a standard event trigger into a parameterized function call:

1. **Trigger**: The Event detects a condition on a Topic.

2. **Parse**: The `event_parser` function receives the raw ROS2 message. It extracts the relevant data (e.g., a string, a coordinate, an ID).

3. **Map**: The extracted data is mapped to a specific keyword argument (`output_mapping`) of the target Action.

4. **Execute**: The Action is executed with the dynamic data passed in.

### Example

In [the previous example](#1-automatic-adaptation-terrain-switching), actions were hard-coded (e.g., "If stairs, run activate_stairs_controller"). However, often you want a single generic method (e.g., switch_controller) that dynamically adapts based on the data received in the event.

You can achieve this using the `event_parser` method on an Action.

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
terrain_change_event = OnChange(
    event_name="terrain_changed",
    event_source=terrain_topic,
    nested_attributes="data",
)

# Define a Helper Parser Function
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
dynamic_switch_action.event_parser(
    method=parse_terrain_data,
    output_mapping="controller_type"
)

# Alternatively, since this was a simple parser we could have used a lambda function as well
# dynamic_switch_action.event_parser(
#     method=(lambda msg: msg.data),
#     output_mapping="controller_type"
# )

# Register
my_launcher = Launcher()
my_launcher.add_pkg(
            components=[quad_controller],
            events_actions={terrain_change_event: dynamic_switch_action},
        )
```
