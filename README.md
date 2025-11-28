<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="docs/_static/SUGARCOAT_DARK.png"  width="50%">
</picture>

<br/>


🇨🇳 [简体中文](docs/README.zh.md) | 🇯🇵 [日本語](docs/README.ja.md)


## The Sweetest Way to Build ROS2 Systems

**Sugarcoat** is a meta-framework that injects a whole lot of **syntactic sugar** into building complex, event-driven multinode systems in **ROS2**, all through an intuitive **Python API**.

## Key Features

| Feature | Description |
| :--- | :--- |
| **Event-Driven Core** | Built-in primitives for **Events** and **Actions** enables dynamic runtime configuration and control over your system's **Components**. |
| **Built-in Resilience** | **Fallbacks** and **Fault Tolerance** are core design concepts, ensuring your systems are robust and reliable. |
| **Intuitive Python API** | Design your entire system—nodes, events, and actions—using clean, readable Python code. |
| **Dynamic Web UI** | Automatically generate a fully dynamic, extensible web interface for monitoring and configuring your system. |
| **Universal Applications [Using Robot Plugins](https://www.youtube.com/watch?v=oZN6pcJKgfY) (!NEW)** | Allows you to write generic, portable automation logic that runs on any robot without code changes |
| **Launch Replacement** | A more pythonic alternative to the ROS2 Launch API, providing greater flexibility and runtime control for real-world applications. |

## Packages Built with Sugarcoat

- [**Kompass**](https://automatikarobotics.com/kompass/): A framework for building robust and comprehensive event-driven navigation stacks using an easy-to-use and intuitive Python API.
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/): A fully-loaded framework for creating interactive physical agents that can understand, remember, and act upon contextual information from their environment.


## Get Started

- Learn more about the [**design concepts**](https://automatika-robotics.github.io/sugarcoat/design/index.html) in Sugarcoat 📚
- Learn how to [**create your own ROS2 package**](https://automatika-robotics.github.io/sugarcoat/use.html) using Sugarcoat 🚀
- [**Port your automation recipes across different hardware**](https://automatika-robotics.github.io/sugarcoat/advanced/robot_plugins.html) using **Robot Plugins**
- Explore the [**Dynamic Web UI**](https://automatika-robotics.github.io/sugarcoat/advanced/web_ui.html) for real-time system visualization and control

## Dynamic Web UI for Sugarcoat Recipes

The **Dynamic Web UI** feature takes system visibility and control to the next level. Built with [**FastHTML**](https://www.fastht.ml/) and [**MonsterUI**](https://monsterui.answer.ai/), it is designed to automatically generate a fully dynamic, extensible web interface for any Sugarcoat recipe, completely eliminating the need for manual front-end development.

This feature instantly transforms your complex, multinode ROS2 system into a monitorable and configurable web application.

### Automatic UI Generation in Action

See how the Web UI effortlessly generates interfaces for different types of Sugarcoat recipes:

- **Example 1: General Q\&A MLLM Recipe**
  A fully functional interface generated for an MLLM agent recipe from [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/), automatically providing controls for settings and real-time text I/O with the robot.

<p align="center">
<picture align="center">
  <img alt="EmbodiedAgents UI Example GIF" src="docs/_static/images/agents_ui.gif" width="60%">
</picture>
</p>

- **Example 2: Point Navigation Recipe**
  An example for an automatically generated UI for a point navigation system from [**Kompass**](https://automatikarobotics.com/kompass/). The UI automatically renders map data, and sends navigation goals to the robot.

<p align="center">
<picture align="center">
  <img alt="KOMPASS UI Example GIF" src="docs/_static/images/nav_ui.gif" width="60%">
</picture>
</p>


### What's Inside?

- **Automatic Settings UI**: Interfaces for configuring the settings of all **Components** used in your recipe are generated on the fly.
- **Auto I/O Visualization**: Front-end controls and data visualizations for UI **Inputs** and **Outputs** are created automatically.
- **WebSocket-Based Streaming**: Features bidirectional, low-latency communication for streaming **text, image, and audio** messages.
- **Responsive Layouts**: Input and output elements are presented in clear, adaptable grid layouts.
- **Extensible Design**: Easily add support for new message types and custom visualizations through extensions.


## How Sugarcoat Works

The core of Sugarcoat revolves around a few concepts:

- **Component**: The main execution unit (a ROS2 lifecycle node abstraction) configured with **Inputs/Outputs** and **Fallback** behaviors. Each component reports its **Health Status**. [Learn More about Components](https://automatika-robotics.github.io/sugarcoat/design/component.html)

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/component_light.png">
  <img alt="Base Component Diagram" src="docs/_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

- **Events & Actions**: Mechanisms to handle and reconfigure components dynamically at runtime. [Learn More about Events](https://automatika-robotics.github.io/sugarcoat/design/events.html) | [Learn More about Actions](https://automatika-robotics.github.io/sugarcoat/design/actions.html)
- **Launcher**: Takes your defined Components, Events, and Actions, and executes the system using multi-threaded or multi-process execution. It works with an internal **Monitor** to manage component lifecycles and track events. [Learn More about the Launcher](https://automatika-robotics.github.io/sugarcoat/design/launcher.html)


<p align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/multi_process_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/multi_process_light.png">
  <img alt="Multi-process execution Diagram" src="docs/_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>


## 🛠️ Installation

Sugarcoat is available for ROS versions $\ge$ **Humble**.

### Using your Package Manager (Recommended)

On Ubuntu, for example:

`sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar`

Alternatively, you can install a specific deb package from the [release page](https://github.com/automatika-robotics/sugarcoat/releases):

`sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb`

> **Note:** If your package manager's version of `attrs` is older than 23.2, you may need to update it via pip:
> `pip install 'attrs>=23.2.0'`

### Building from Source

```shell
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src
git clone [https://github.com/automatika-robotics/sugarcoat](https://github.com/automatika-robotics/sugarcoat) && cd ..

# Install dependencies (ensure attrs>=23.2.0 is included)
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml

colcon build
source install/setup.bash
```

## Copyright

The code in this distribution is Copyright (c) 2024 Automatika Robotics unless explicitly indicated otherwise.

Sugarcoat is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.

## Contributions

Sugarcoat has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.

## 🎩 Hat Tip

The **Dynamic Web UI** is powered by two cool open-source projects. A big thank you to Answers.ai for their work on:

- [**FastHTML**](https://www.fastht.ml/): The HTMX based framework that enables automatic generation of our dynamic web interfaces.
- [**MonsterUI**](https://monsterui.answer.ai/): The styled UI components that make the interface intuitive.
