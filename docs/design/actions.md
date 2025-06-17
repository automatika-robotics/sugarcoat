# Actions

Actions are methods or routines executed by a component or by the system monitor.

Actions can either be:
- Actions paired with Events: in this case the Action is executed by a system monitor when an event is detected (see more details in [Monitor](monitor.md), [Launcher](launcher.md))
- Actions paired with Fallbacks: in this case the Action is executed by a Component when a failure is detected

Actions are defined with:
- method to be executed (Callable)
- args: Arguments to be passed to the method when executing the action
- kwargs: Keyword arguments to be passed to the method when executing the action

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

## Available Defined Actions:

Sugarcoat comes with a set of pre-defined component level actions and system level actions

### Component-level Actions:
- stop: Deactivate the lifecycle Component
- start: Activate the lifecycle Component
- restart: stop then start
- reconfigure: Send new ComponentConfig class object to the Component
- update_parameter: Update the value of one parameter in the ComponentConfig
- update_parameters: Update the value of a set of parameters in the ComponentConfig

### System-level Actions:
- log: Log a message
- publish_message: Publish a ROS2 message to a given topic
- send_srv_request: Send a ROS2 service request
- send_action_goal: Send a ROS2 action goal

:::{tip} The previous pre-defined Actions are all keyword only
:::

### Usage Example:
```python
    from ros_sugar.actions import ComponentActions

    my_component = BaseComponent(node_name='test_component')
    action1 = ComponentActions.start(component=my_component)
    action2 = ComponentActions.log(msg="I am executing a cool action!")
```

## Events Parsers in Actions

In the previous examples ComponentActions are defined with fixed arguments, however, when ComponentActions are used with Events, it might be required in some applications to update an argument during runtime based on the value on the event's topic. For this purpose, each event passes the full value of the incoming ROS2 message and it's trigger attribute value to the action using two keyword argument: `msg` and `trigger`. An Action can then be configured to use these value with its **`event_parser`**.

We configure an `event_parser` using a parsing `Callable` and an `output_mapping: str`. This allows us to add a method that will be executed before the main action executable. The returned value from the method will be passed to the action executable as a keyword argument using the `output_mapping`.

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
