# Why Sugarcoat?

Sugarcoat is designed to **streamline development** and **reduce boilerplate** while developing ROS2 packages and nodes.
Built with modern software engineering principles, it transforms how developers build, manage, and orchestrate complex ROS applications. With **intuitive Python APIs**, **built-in runtime control**, **health monitoring**, and a **dynamic web UI generation**, Sugarcoat lets you focus on the logic that matters — not the glue code.

Whether you're building scalable robotic systems, deploying distributed edge applications, or iterating fast on prototypes, Sugarcoat gives you the tools to move faster, write cleaner code, and ship more reliable robots with automatic, real-time visualization and control.

## 🚀 Advantages of Using Sugarcoat

### Intuitive Python API with Event-Driven Architecture

No need to write verbose callbacks just to check a sensor value. Sugarcoat allows you to define complex system behaviors using natural Python expressions to execute data-driven actions

* **Standard ROS2:** Write a custom subscriber class in your component $\rightarrow$ hardcode a custom callback $\rightarrow$ check `if msg.data <= 20.0` $\rightarrow$ call function.

* **Sugarcoat:**
    ```python
    # Trigger an action when battery is low
    Event(battery.msg.data <= 20.0)
    ```


### ROS2 Nodes Reimagined as Components

Sugarcoat wraps standard ROS2 nodes into powerful **Components** that come pre-equipped with:

* **Configurable Inputs/Outputs** with automatic handling of subscribers and publishers creation.
* **Managed Lifecycle:** Automatic handling of Configure, Activate, Deactivate, and Cleanup states.
* **Configuration Management:** Type-safe configuration validation using `attrs`.
* **Health Monitoring:** Built-in heartbeats and status tracking.
* **Fallbacks:** Define specific recovery behaviors if a component crashes or stalls.

With Sugarcoat Components you can forget about boilerplate code and focus on logic implementation, not plumbing.

### Zero-Code Dynamic Web UI

The **Dynamic Web UI** automatically generates a fully functional control interface for your entire system — **no front-end coding required.**. This feature instantly transforms your complex, multinode ROS2 system into a **monitorable and configurable web application**.

* **Auto I/O Visualization:** Inputs/Outputs are automatically visualized (Text, Images, Audio, Pose, Maps, etc.).
* **Live Configuration:** Tweak component parameters in real-time directly from the UI.
* **Bidirectional Streaming:** Low-latency WebSocket streaming for and to the robot.
- **Responsive Layouts:** Components are organized in adaptive, grid-based layouts for clear visibility.
- **Extensible Design:** Add new message types or custom widgets through lightweight extensions.

:::{seealso} Check how you can enable the dynamic UI for your recipe [here](./advanced/web_ui.md)
:::

### Hardware Agnosticism (Robot Plugins)
Sugarcoat introduces [**Robot Plugins**](./advanced/robot_plugins.md) to seamlessly bridge your automation recipes with diverse robot hardware.

* **No Vendor Lock-in:** Different manufacturers use custom ROS2 interfaces (messages/services). Sugarcoat Plugins act as a translation layer, handling type conversions behind the scenes.
* **True Portability:** Write your automation logic once using standard types. Switch from a simulation to a physical robot simply by changing the `robot_plugin` configuration—**zero code changes required**.

## Standard ROS2 vs. Sugarcoat

### 🔧 Code, Architecture and Developer Experience

| **Standard ROS2 Pattern** | **With Sugarcoat** |
| :--- | :--- |
| **Verbose Boilerplate**<br>Requires manual setup of Publishers, Subscribers, Service Clients, and Parameter servers for every node. | <span class="text-red">**Declarative & Concise**</span><br>Define Inputs, Outputs, and Configs declaratively. The framework handles the setup, threading, and cleanup. |
| **Manual Logic Glue**<br>Writing custom "Manager Nodes" just to wire the output of Node A to the input of Node B's action. | <span class="text-red">**Auto-Parsers & Events**</span><br>Directly wire Topic data into Action arguments in your launch recipe without writing intermediate glue code. |
| **Steep learning curve**<br>Limited examples in the docs                                                   | <span class="text-red">**Intuitive Pythonic interface**</span><br>Full developer docs, tutorials, and API references.                                       |


### ⚙️ Runtime and Reliability

| **Standard ROS2 Pattern** | **With Sugarcoat** |
| :--- | :--- |
| **Static Launch Files**<br>XML/Python launch files are often static and hard to introspect at runtime. | <span class="text-red">**Dynamic Universal Recipes**</span><br>Hardware-agnostic applications that auto-generate a Web UI and allow full runtime introspection, live reconfiguration, and interactive control. |
| **Opaque Failures**<br>When a node dies, the system often keeps running in an undefined state unless you write custom watchdogs. | <span class="text-red">**Built-in Health Monitor**</span><br>A central Monitor tracks every component. If one fails, defined **Fallback Actions** (e.g., specific restarts or safety stops) trigger automatically. |
| **Manual Lifecycle Management**<br>Implementing the `LifecycleNode` state machine correctly is complex and often skipped. | <span class="text-red">**Native Lifecycle Support**</span><br>Every Sugarcoat Component is a Lifecycle Node by default, with automated transitions and error handling. |
| **No Native UI**<br>Visualizing custom data requires writing custom RQt plugins or web bridges. | <span class="text-red">**Instant Web Interface**</span><br>Every topic, parameter, and log is accessible via an auto-generated, responsive Web UI immediately upon launch. |



## See it in Action

-  Before (Standard ROS2 Approach):

*You write a "Battery Monitor" node.*

1.  Import rclpy, create class.
2.  Create subscription to `/battery`.
3.  Create client for `/navigation/stop`.
4.  In callback: check voltage.
5.  If low: wait for service, create request, call service.
6.  Handle service errors / timeouts.

- After (Sugarcoat Approach):

*You write a Recipe!*

```python
# 1. Define Source & Action
batt_topic = Topic(name="/battery", msg_type="Float32")
stop_action = Action(method=nav_component.stop)

# 2. Define Logic
emergency_event = Event(batt_topic.msg.data <= 15.0,
    on_change=True # Trigger once when threshold is crossed
)

# 3. Launch
launcher.add_pkg(
    components=[nav_component],
    events_actions={emergency_event: stop_action}
)
```
