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
| **`send_component_service_request`** | `component`<br>`srv_request_msg` | Sends a request to the component's main service with a specific message. |
| **`trigger_component_service`** | `component` | Triggers the component's main service. <br>Creates the request message dynamically during runtime from the incoming Event topic data. |
| **`send_component_action_server_goal`** | `component`<br>`request_msg` | Sends a goal to the component's main action server with a specific message. |
| **`trigger_component_action_server`** | `component` | Triggers the component's main action server. <br>Creates the request message dynamically during runtime from the incoming Event topic data. |

### System-Level Actions

These actions interact with the broader ROS2 system and are executed by the central `Monitor`.

| Action Method | Arguments | Description |
| :--- | :--- | :--- |
| **`log`** | `msg`<br>`logger_name` (opt) | Logs a message to the ROS console. |
| **`publish_message`** | `topic`<br>`msg`<br>`publish_rate`/`period` | Publishes a specific message to a topic. Can be single-shot or periodic. |
| **`send_srv_request`** | `srv_name`<br>`srv_type`<br>`srv_request_msg` | Sends a request to a ROS 2 Service with a specific message. |
| **`trigger_service`** | `srv_name`<br>`srv_type` | Triggers the a given ROS2 service. |
| **`send_action_goal`** | `server_name`<br>`server_type`<br>`request_msg` | Sends a specific goal to a ROS 2 Action Server. |
| **`trigger_action_server`** | `server_name`<br>`server_type` | Triggers the a given ROS2 action server. |

:::{tip} The pre-defined Actions are all keyword only and can be imported from `ros_sugar.actions' module
:::

:::{note} When the *trigger* actions for both ROS2 services and ROS2 action servers are paired with an Event, the action attempts to create the required service request from the incoming Event topic data. If automatic conversion (duck taping) is not possible or if the action is not paired with an Event, the *trigger* action will send a default (empty) request
:::
