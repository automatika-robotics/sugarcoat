# Custom Robot Plugins

Sugarcoat introduces Robot Plugins to **seamlessley bridge your automation recipes with diverse robot hardware**.

By abstracting manufacturer-specific ROS2 interfaces, <span class="text-red">plugins allow you to write generic, portable automation logic that runs on any robot without code changes 🎉</span>

## What are Robot Plugins?

Different robot manufacturers often use custom messages, or services in their ROS2 interfaces to handle basic operations like sending robot actions or getting diverse low-level feedback such as odometry, battery info, etc. With traditional ROS2 packages, you will need to do code changes to handle each new message/service type. This creates a "lock-in" where your code becomes tightly coupled to a specific robot.

Sugarcoat Robot Plugins acts as a translation layer. It sits between your Sugarcoat application and the robot's hardware with all its custom types.

## Why Robot Plugins?

* <span class="text-blue">**Portability:**</span> Write your automation recipe once using standard types. Switch robots by simply changing the plugin configuration.

* <span class="text-blue">**Simplicity:**</span> The plugin handles all the complex type conversions and service calls behind the scenes.

* <span class="text-blue">**Modularity:**</span> Keep hardware-specific logic isolated in a separate package.

## How to Create a Robot Plugin

To create your own custom plugin, you can create a ROS2 package based on the example [myrobot_plugin_interface](https://github.com/automatika-robotics/robot-plugin-example) that we have created.

Any plugin package must export a Python module containing two specific dictionaries in its `__init__.py`:

1.  `robot_feedback`: Maps standard types to the robot's specific feedback topics (e.g., getting `IMU` or `Odometry` like information).
2.  `robot_action`: Maps standard types to the robot's specific action topics or service clients (e.g., sending `Twist` like commands).

The main steps to creating the plugin:

0.  (Optional) **Define Custom ROS Interfaces**:

    If your robot's manufacturer-specific messages or services are not available to import from another package, define them in `msg/` and `srv/` folders.

1.  **Implement Type Converters (`types.py`)**:

    Create a `types.py` module to handle data translation.
    * **For Each Feedback:** Define a callback function that transforms the custom ROS2 message into a standard Python type (like a NumPy array) which you can use directly in your system. Register it using `create_supported_type`.
    * **For Each Action:** Define a converter function that transforms standard Python inputs into the custom ROS2 message.

2.  **Handle Service Clients (`clients.py`)**:

    If your robot actions require calling a ROS2 service, create a class inheriting from `RobotPluginServiceClient` in `clients.py`. Implement the `_publish` method to construct and send the service request.

3.  **Register the Plugin (`__init__.py`)**:

    Expose your new capabilities in `__init__.py` by defining two dictionaries:
    * `robot_feedback`: Map standard names to `Topic` objects using your custom types.
    * `robot_action`: Map standard names to `Topic` objects (for topics) or Client classes (for services).

4.  **Configure the Build**:

    Use the same `CMakeLists.txt` and `package.xml` for your new plugin package. Make sure to add any additional used dependencies.

Check the [robot plugin example details](#an-example-robot-plugin) below for a complete breakdown of the process.


## How to Use a Plugin in your Recipe

Using a robot plugin in your Sugarcoat automation recipe is extremly straightforward. After building and installing your plugin package, all you need to do is to specify the plugin package name when initializing the `Launcher` and Sugarcoat will take it from there!

```python
from ros_sugar import Launcher

# ... Define your components/events/actions/fallbacks here ...

# Initialize the launcher with your specific robot plugin
launcher = Launcher(robot_plugin="myrobot_plugin")

# ... Add it all to the launcher and bringup the system ...
```

## An Example Robot Plugin

Lets explore an example of a custom plugin called [`myrobot_plugin`](https://github.com/automatika-robotics/robot-plugin-example). The plugin is made to bridge a standard robot commands (`Twist`) and a standard feedback (`Odometry`) to two custom interfaces.

### 1. Custom Interfaces

The plugin defines three ROS2 interfaces as an example of custom interfaces to communicate with the robot's ROS2 interface exposed by the manufacturer:

* **`CustomOdom.msg`**: A feedback message containing position (x, y, z) and orientation (pitch, roll, yaw).
* **`CustomTwist.msg`**: A command message for 2D velocity (vx, vy) and angular velocity (vyaw).
* **`RobotActionCall.srv`**: A service definition used to trigger actions on the robot, returning a success boolean.

### 2. Plugin Implementation

The core logic resides in the `myrobot_plugin` Python module.

#### Supported Types (`types.py`)

This module defines how to translate between the custom robot types (`CustomOdom` and `CustomTwist`) and standard python types

* **Feedback (Callbacks):** Functions that convert incoming ROS messages into standard types.
    * *Example:* `_odom_callback` converts `CustomOdom` into a NumPy array `[x, y, yaw]`.
* **Actions (Converters):** Functions that convert standard commands (like `vx`, `vy`, `omega`) into custom ROS2 messages.
    * *Example:* `_ctr_converter` converts velocity inputs into a `CustomTwist` message.

```python
from ros_sugar.robot_plugin import create_supported_type
# Example: Creating a supported type for feedback
RobotOdometry = create_supported_type(CustomOdom, callback=_odom_callback)
```

#### Service Clients (`clients.py`)

For robots that handle actions via ROS services (instead of topics), this module defines custom client wrappers inheriting from RobotPluginServiceClient.

* **CustomTwistClient**: Wraps the RobotActionCall service. It implements the _publish method to populate the service request (vx, vy, vyaw) and send it to the robot.

#### Plugin Entry Point (`__init__.py`)

This file exposes the plugin capabilities to the framework using two specific dictionaries:

1. **robot_feedback**: Maps standard feedback names (e.g., "Odometry") to Topic instances using the custom types defined in types.py.

2. **robot_action**: Maps standard action names (e.g., "Twist") to either a Topic or a ServiceClientHandler (like CustomTwistClient).

```python
from . import types, clients

# Example configuration
robot_feedback = {
    "Odometry": Topic(name="myrobot_odom", msg_type=types.RobotOdometry),
}

robot_action = {
    "Twist": clients.CustomTwistClient
}
```

### 3. Testing

A `server_node.py` is provided to simulate the robot's ROS2 server. It spins a minimal node that listens to `robot_control_service` requests and logs the received velocity commands, allowing you to test the `CustomTwistClient` functionality.


## See the Example Robot Plugin in Action

Here the example plugin is tested with [**Kompass**](https://github.com/automatika-robotics/kompass) which is an event-driven navigation framework build on top of Sugarcoat.

- To recplicate this test, start by installing Kompass from source by following the instructions [here](https://automatika-robotics.github.io/kompass/install.html)

- Pull the [myrobot_plugin_interface](https://github.com/automatika-robotics/robot-plugin-example) to your ROS2 workspace and build it.

Start by running the [`turtlebot3_test`](https://github.com/automatika-robotics/kompass/blob/main/kompass/recipes/turtlebot3.py) without the plugin and observe the subscribed and published topics. You will see that the components are subscribed to `/odometry/filtered` topic of type `Odometry`. The `DriveManager` component will also be publishing the robot commands as `Twist` messages on `/cmd_vel` topic (or `TwistStamped` based on your ROS2 distribution).

To enable the plugin, just edit one line in the recipe to change the exisitng launcher initialization from the following:
```python

launcher = Launcher(robot_plugin="myrobot_plugin")

```
to:

```python

launcher = Launcher(config_file=config_file, robot_plugin="myrobot_plugin")

```

Re-run the recipe after enabling the plugin, and Voila! The components now will be expecting the plugin odometry topic of type `CustomOdometry`. Moreover, the `DriveManager` will no longer publish the `/cmd_vel` topic. Instead, it has created a service client in accordance with our custom plugin.


```{youtube} oZN6pcJKgfY
:width: 600
:height: 338
:align: center
```
