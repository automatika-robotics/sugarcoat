# Actions

Actions represent the **executable behaviors** of your systems. They can represent:

- **Behaviors defined within your components** that you wish to execute at runtime such as stopping your robot, executing a specific motion pattern or saying a sentence.
- or **system level behaviors** like reconfiguring a component, turning component on/off or replacing some input streams.
- or **arbitrary user-defined behaviors** like sending a call to an external API.

In Sugarcoat, Actions are not just static function calls; they are <span class="text-red">dynamic, context-aware routines that can adapt their parameters in real-time based on live system data.</span>

**How are Actions Triggered?** Actions sit dormant until activated by one of two mechanisms:

- Event-Driven (Reflexive): Triggered instantly when a specific Event condition is met (e.g., "Obstacle Detected" $\rightarrow$ stop_robot).

- Fallback-Driven (Restorative): Triggered automatically by a Component when its internal Health Status degrades (e.g., "Camera Driver Failed" $\rightarrow$ restart_driver).

## The `Action` Class

At its core, the `Action` class is a wrapper around any Python callable. It packages a function along with its arguments, preparing them for execution at runtime.

But unlike standard Python functions, Sugarcoat Actions possess a superpower: [Dynamic Data Injection](#dynamic-data-injection). You can bind their arguments directly to live ROS2 Topics, allowing the Action to fetch the latest topic message or a specific message argument the moment it triggers.

```python
class Action:
    def __init__(self, method: Callable, args: tuple = (), kwargs: Optional[Dict] = None):
```

- method: The function or routine to execute.

- args: Positional arguments (can be static values OR dynamic Topic values).

- kwargs: Keyword arguments (can be static values OR dynamic Topic values).

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

## Dynamic Data Injection

Sugarcoat Actions are not limited to static arguments. You can bind arguments directly to live Topic data.

When the Action is triggered, the system automatically resolves these bindings—fetching the current value from the specified Topic attributes—and injects them into your function.

This allows you to **create complex, context-aware behaviors without writing any "glue code" or custom parsers**.

### Example: Cross-Topic Data Access

**Scenario**: An event occurs on Topic 1. You want to log a message that includes the current status from Topic 2 and a sensor reading from Topic 3.

```python
from ros_sugar.core import Event, Action
from ros_sugar.io import Topic

# 1. Define Topics
topic_1 = Topic(name="system_alarm", msg_type="Bool")
topic_2 = Topic(name="robot_mode", msg_type="String")
topic_3 = Topic(name="battery_voltage", msg_type="Float32")

# 2. Define the Event
# Trigger when Topic 1 becomes True
event_on_first_topic = Event(topic_1.msg.data.is_true())

# 3. Define the Target Function
def log_context_message(mode, voltage):
    print(f"System Alarm! Current Mode: {mode}, Voltage: {voltage}V")

# 4. Define the Dynamic Action
# We bind the function arguments directly to the data fields of Topic 2 and Topic 3
my_action = Action(
    method=log_context_message,
    # At runtime, these are replaced by the actual values from the topics
    args=(topic_2.msg.data, topic_3.msg.data)
)
```

## Pre-defined Actions

While you can wrap any `function` in an Action, Sugarcoat provides the [`Actions`](../apidocs/ros_sugar/ros_sugar.actions.md) module with a suite of pre-defined, thread-safe actions for managing components and system resources.

These actions are divided into Component-Level (affecting a specific component's lifecycle or config) and System-Level (general ROS2 utilities).

Sugarcoat comes with a set of pre-defined component level actions and system level actions

### Component-Level Actions

These actions directly manipulate the state or configuration of a specific `BaseComponent` derived object.

| Action Method                           | Arguments                                                     | Description                                                                                                                                 |
| :-------------------------------------- | :------------------------------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------ |
| **`start`**                             | `component`                                                   | Triggers the component's Lifecycle transition to **Active**.                                                                                |
| **`stop`**                              | `component`                                                   | Triggers the component's Lifecycle transition to **Inactive**.                                                                              |
| **`restart`**                           | `component`<br>`wait_time` (opt)                              | Stops the component, waits `wait_time` seconds (default 0), and Starts it again.                                                            |
| **`reconfigure`**                       | `component`<br>`new_config`<br>`keep_alive`                   | Reloads the component with a new configuration object or file path. <br>`keep_alive=True` (default) keeps the node running during update.   |
| **`update_parameter`**                  | `component`<br>`param_name`<br>`new_value`<br>`keep_alive`    | Updates a **single** configuration parameter.                                                                                               |
| **`update_parameters`**                 | `component`<br>`params_names`<br>`new_values`<br>`keep_alive` | Updates **multiple** configuration parameters simultaneously.                                                                               |
| **`send_component_service_request`**    | `component`<br>`srv_request_msg`                              | Sends a request to the component's main service with a specific message.                                                                    |
| **`trigger_component_service`**         | `component`                                                   | Triggers the component's main service. <br>Creates the request message dynamically during runtime from the incoming Event topic data.       |
| **`send_component_action_server_goal`** | `component`<br>`request_msg`                                  | Sends a goal to the component's main action server with a specific message.                                                                 |
| **`trigger_component_action_server`**   | `component`                                                   | Triggers the component's main action server. <br>Creates the request message dynamically during runtime from the incoming Event topic data. |

### System-Level Actions

These actions interact with the broader ROS2 system and are executed by the central `Monitor`.

| Action Method               | Arguments                                       | Description                                                              |
| :-------------------------- | :---------------------------------------------- | :----------------------------------------------------------------------- |
| **`log`**                   | `msg`<br>`logger_name` (opt)                    | Logs a message to the ROS console.                                       |
| **`publish_message`**       | `topic`<br>`msg`<br>`publish_rate`/`period`     | Publishes a specific message to a topic. Can be single-shot or periodic. |
| **`send_srv_request`**      | `srv_name`<br>`srv_type`<br>`srv_request_msg`   | Sends a request to a ROS 2 Service with a specific message.              |
| **`trigger_service`**       | `srv_name`<br>`srv_type`                        | Triggers the a given ROS2 service.                                       |
| **`send_action_goal`**      | `server_name`<br>`server_type`<br>`request_msg` | Sends a specific goal to a ROS 2 Action Server.                          |
| **`trigger_action_server`** | `server_name`<br>`server_type`                  | Triggers a given ROS2 action server.                                 |

:::{tip} The pre-defined Actions are all keyword only and can be imported from `ros_sugar.actions' module.
:::

:::{note} When the _trigger_service_, _trigger_action_server_, etc. actions are paired with an Event, the action attempts to create the required service request from the incoming Event topic data automatically via duck typing. If automatic conversion is not possible or if the action is not paired with an Event, the action will send the default (empty) request message.
:::
