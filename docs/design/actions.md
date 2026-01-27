# Actions

Actions are methods or routines executed by a Component or by the System Monitor. They are the "Outputs" of the Event-Driven system.

Actions can be triggered in two ways:

- **Event-Driven**: Executed when a specific Event is detected.

- **Fallback-Driven**: Executed by a Component when an internal Health Status failure is detected.

## The `Action` Class
The `Action` class is a generic wrapper for any `callable`. It allows you to package a function and its arguments to be executed later.

```python
class Action:
    def __init__(self, method: Callable, args: tuple = (), kwargs: Optional[Dict] = None):
```

- method: The function to be executed.

- args: Tuple of positional arguments.

- kwargs: Dictionary of keyword arguments.


## Basic Usage
```python
from ros_sugar.component import BaseComponent
from ros_sugar.core import Action
import logging

def custom_routine():
    logging.info("I am executing an action!")

my_component = BaseComponent(node_name='test_component')

# 1. Component Method
action1 = Action(method=my_component.start)

# 2. Method with keyword arguments
action2 = Action(method=my_component.update_parameter, kwargs={"param_name": "fallback_rate", "new_value": 1000})

# 3. External Function
action3 = Action(method=custom_routine)
```

## Pre-defined Actions

While you can wrap any `function` in an Action, Sugarcoat provides the [`Actions`](../apidocs/ros_sugar/ros_sugar.actions.md) module with a suite of pre-defined, thread-safe actions for managing components and system resources.

These actions are divided into Component-Level (affecting a specific component's lifecycle or config) and System-Level (general ROS2 utilities).

Sugarcoat comes with a set of pre-defined component level actions and system level actions

### Component-Level Actions

These actions directly manipulate the state or configuration of a specific `BaseComponent` derived object.

| Action Method | Arguments | Description |
| :--- | :--- | :--- |
| **`start`** | `component` | Triggers the component's Lifecycle transition to **Active**. |
| **`stop`** | `component` | Triggers the component's Lifecycle transition to **Inactive**. |
| **`restart`** | `component`<br>`wait_time` (opt) | Stops the component, waits `wait_time` seconds (default 0), and Starts it again. |
| **`reconfigure`** | `component`<br>`new_config`<br>`keep_alive` | Reloads the component with a new configuration object or file path. <br>`keep_alive=True` (default) keeps the node running during update. |
| **`update_parameter`** | `component`<br>`param_name`<br>`new_value`<br>`keep_alive` | Updates a **single** configuration parameter. |
| **`update_parameters`** | `component`<br>`params_names`<br>`new_values`<br>`keep_alive` | Updates **multiple** configuration parameters simultaneously. |
| **`trigger_component_service`** | `component` | Triggers the component's main service. <br>Sets up dynamic typing to allow parsing the request from an Event. |
| **`trigger_component_action_server`** | `component` | Triggers the component's main action server. <br>Sets up dynamic typing to allow parsing the goal from an Event. |

### System-Level Actions

These actions interact with the broader ROS2 system and are executed by the central `Monitor`.

| Action Method | Arguments | Description |
| :--- | :--- | :--- |
| **`log`** | `msg`<br>`logger_name` (opt) | Logs a message to the ROS console. |
| **`publish_message`** | `topic`<br>`msg`<br>`publish_rate`/`period` | Publishes a specific message to a topic. Can be single-shot or periodic. |
| **`publish_message_from_parsed_topic`** | `in_topic`<br>`out_topic`<br>`publish_rate`/`period`<br>`custom_parser` (opt) | Publishes to `out_topic` by parsing data from `in_topic`. Automatically creates a parser if `custom_parser` is not provided. |
| **`send_srv_request`** | `srv_name`<br>`srv_type`<br>`srv_request_msg` | Sends a request to a ROS 2 Service with a specific message. |
| **`trigger_service`** | `srv_name`<br>`srv_type` | Prepares a Service request. Sets up dynamic typing to allow parsing the request message from an Event later. |
| **`send_srv_request_from_topic`** | `srv_name`<br>`srv_type`<br>`topic`<br>`custom_parser` (opt) | Sends a Service request by parsing the `topic` data into the request message. |
| **`send_action_goal`** | `server_name`<br>`server_type`<br>`request_msg` | Sends a specific goal to a ROS 2 Action Server. |
| **`trigger_action_server`** | `server_name`<br>`server_type` | Prepares an Action Goal. Sets up dynamic typing to allow parsing the goal message from an Event later. |
| **`send_action_goal_from_topic`** | `server_name`<br>`server_type`<br>`topic`<br>`custom_parser` (opt) | Sends an Action Goal by parsing the `topic` data into the goal message. |


:::{tip} The pre-defined Actions are all keyword only and can be imported from `ros_sugar.actions' module
:::

## Dynamic Arguments (Event Parsers)

In standard usage, action methods arguments are fixed at definition time (e.g., `new_value=1000`). However, when paired with an Event, you often need to use data from the triggering message (e.g., "Set speed to X" where X comes from a sensor topic).

Sugarcoat provides two ways to map Event data to Action arguments: Automatic Parsers and Custom Parsers.

:::{seealso} See more on how the event parser affects the event management [here](events.md)
:::

### 1. Automatic Parsers
If your Event Topic data type is compatible with your Action's argument type, you can use the built-in auto-parser. This effectively "wires" the topic data directly into the function argument.

The system automatically handles:

1. ROS-to-Python: e.g., `std_msgs/String` $\rightarrow$ `str`, `std_msgs/Int32` $\rightarrow$ `int`.
2. Container Unwrapping: e.g., `std_msgs/Float32MultiArray` $\rightarrow$ `list` or `numpy.ndarray`.
3. Duck Typing: e.g., `PoseStamped` topic $\rightarrow$ `Pose` argument (extracts matching fields automatically).

#### Example: Passing a String from Topic to Function

```python
from std_msgs.msg import String

# 1. The Function: Expects a python string
def switch_gait(controller_type: str):
    print(f"Switching to {controller_type}")

# 2. The Action
gait_action = Action(method=switch_gait)

# 3. The Auto-Parser
# Automatically extracts .data from the String msg and passes it as 'controller_type'
gait_action.add_automatic_parser_from_msg_type(
    input_topic_msg_type=String,
)
```

#### Example: Passing a List/Array

```python
from std_msgs.msg import Float32MultiArray

def update_waypoints(transformation, coords: list):
    # coords will be a standard python list [x, y, z...]
    pass

# some_transformation = ...
action = Action(method=update_waypoints, args=(some_transformation,))

action.add_automatic_parser_from_msg_type(
    input_topic_msg_type=Float32MultiArray,
    keyword_argument_name="coords",
    # Alternatively pass the positional argument index
    # argument_index=1
    action_argument_type=list  # Explicitly request a list conversion
)
```

### 2. Custom Parsers

For complex logic—such as coordinate transformations, unit conversions, or constructing complex ROS2 objects—use a custom parser method.

The parser receives the raw Event message and returns the value to be injected into the Action.


### Example: Converting PointStamped to Action Goal

Let's see how this can work in a small example: We will take the example used in [Kompass tutorial](https://automatika-robotics.github.io/kompass/tutorials/events_actions.html) where a `send_action_goal` action is used to send a ROS2 ActionServer goal by parsing a value from a published topic.

First we define the action that sends the action server goal:

```python
from ros_sugar.actions import send_action_goal
from geometry_msgs.msg import PointStamped
from kompass.components import Planner
from kompass_interfaces.action import PlanPath

# define the planner component running as an Action Server
planner = Planner("kompass_planner")
planner.run_type = "ActionServer"

# Define Rviz clicked point topic
clicked_point_topic = Topic(name="/clicked_point", msg_type="PointStamped")

# Define a method to parse a message of type PointStamped to the planner PlanPath Goal
def goal_point_parser(*, msg: PointStamped, **_):
    action_request = PlanPath.Goal()
    goal = Pose()
    goal.position.x = msg.point.x
    goal.position.y = msg.point.y
    action_request.goal = goal
    end_tolerance = PathTrackingError()
    end_tolerance.orientation_error = 0.2
    end_tolerance.lateral_distance_error = 0.05
    action_request.end_tolerance = end_tolerance
    return action_request

# Define an Action to send a goal to the planner ActionServer
send_goal: actions.Action = actions.send_action_goal_from_topic(
    server_name=planner.main_action_name,
    server_type=planner.action_type,
    topic=clicked_point_topic,
    custom_parser=goal_point_parser,  # Adds the parser method to convert the topic message to the action request
)
```

As we see the defined `goal_point_parser` method takes the PointStamped message and turns it into a `PlanPath` goal request. Then at each event trigger the value of the `action_request` will be passed to the `send_action_goal` executable as the `action_request_msg`

:::{important}
Parsers are executed immediately when the Event triggers, before the Action is launched. Ensure your custom parsers are lightweight to avoid blocking the event loop.
:::
