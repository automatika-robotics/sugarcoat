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


## Usage Example:
```python
    from ros_sugar.core import BaseComponent
    from ros_sugar.config import BaseComponentConfig
    from ros_sugar.actions import Action
    import logging

    def function():
        logging.info("I am executing an action!")

    my_component = BaseComponent(node_name='test_component')
    new_config = BaseComponentConfig(loop_rate=50.0)
    action1 = Action(method=my_component.start)
    action2 = Action(method=my_component.reconfigure, args=(new_config, True),)
    action3 = Action(method=function)
```

## Pre-defined Actions

While you can wrap any `function` in an Action, Sugarcoat provides the [`ComponentActions`](../apidocs/ros_sugar/ros_sugar.core.component_actions.md) module with a suite of pre-defined, thread-safe actions for managing components and system resources.

These actions are divided into Component-Level (affecting a specific component's lifecycle or config) and System-Level (general ROS2 utilities).

Sugarcoat comes with a set of pre-defined component level actions and system level actions

### Component-Level Actions

These actions directly manipulate the state or configuration of a specific `BaseComponent` derived object.

| Action Method | Arguments | Description |
| :--- | :--- | :--- |
| **`start`** | `component` | Triggers the component's Lifecycle transition to **Active**. |
| **`stop`** | `component` | Triggers the component's Lifecycle transition to **Inactive** (stops execution loops). |
| **`restart`** | `component`<br>`wait_time` (opt) | Stops the component, waits `wait_time` seconds (default 0), and Starts it again. |
| **`reconfigure`** | `component`<br>`new_config`<br>`keep_alive` | Reloads the component with a new configuration object or any configuration file path. |
| **`update_parameter`** | `component`<br>`param_name`<br>`new_value`<br>`keep_alive` | Updates a **single** configuration parameter. <br>`keep_alive=True` (default) keeps the node running during update. |
| **`update_parameters`** | `component`<br>`params_names`<br>`new_values`<br>`keep_alive` | Updates **multiple** configuration parameters simultaneously. |

### System-Level Actions

These actions interact with the broader ROS2 system and are executed by the central `Monitor`.

| Action Method | Arguments | Description |
| :--- | :--- | :--- |
| **`log`** | `msg`<br>`logger_name` (opt) | Logs a message to the ROS console. |
| **`publish_message`** | `topic`<br>`msg`<br>`publish_rate`/`period` | Publishes a message to a specific topic. Can be single-shot or periodic. |
| **`send_srv_request`** | `srv_name`<br>`srv_type`<br>`srv_request_msg` | Sends a request to a ROS 2 Service. |
| **`send_action_goal`** | `action_name`<br>`action_type`<br>`action_request_msg` | Sends a goal to a ROS 2 Action Server. |

:::{tip} The pre-defined Actions are all keyword only
:::

### Usage Example:
```python
    from ros_sugar.actions import ComponentActions

    my_component = BaseComponent(node_name='test_component')
    action1 = ComponentActions.start(component=my_component)
    action2 = ComponentActions.log(msg="I am executing a cool action!")
```

## Dynamic Arguments (Event Parsers)

In standard usage, arguments are fixed at definition time. However, when paired with an `Event`, you often need to use data from the triggering message (e.g., "Go to this location" where this is the location defined in the triggering event message).

:::{seealso} See more on how the event parser affects the event management [here](events.md)
:::

You can use the `.event_parser()` method to map data from the event to the action's arguments.


### Example

Let's see how this can work in a small example: We will take the example used in [Kompass tutorial](https://automatika-robotics.github.io/kompass/tutorials/events_actions.html) where a `send_action_goal` action is used to send a ROS2 ActionServer goal by parsing a value from a published topic.

First we define the action that sends the action server goal:

```python
from ros_sugar.actions import ComponentActions
from kompass_interfaces.action import PlanPath

# Define an Action to send a goal to the planner ActionServer
send_goal: Action = ComponentActions.send_action_goal(
    action_name="/planner/plan_path",
    action_type=PlanPath,
    action_request_msg=PlanPath.Goal(),
)
```
Then a parser is defined to parse a `PointStamped` message into the required ROS2 goal message:

```python
from kompass_interfaces.msg import PathTrackingError
from geomerty_msgs.msg import PointStamped

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

# Adds the parser method as an Event parser of the send_goal action
send_goal.event_parser(goal_point_parser, output_mapping="action_request_msg")
```

As we see the defined `goal_point_parser` method takes the PointStamped message and turns it into a `PlanPath` goal request. Then at each event trigger the value of the `action_request` will be passed to the `send_action_goal` executable as the `action_request_msg`
