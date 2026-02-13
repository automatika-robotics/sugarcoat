<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="docs/_static/SUGARCOAT_DARK.png"  width="50%">
</picture>

<br/>

🇨🇳 [简体中文](docs/README.zh.md) | 🇯🇵 [日本語](docs/README.ja.md)

## The Orchestration Layer for Event-Driven ROS2 Systems

**Sugarcoat** is a meta-framework that replaces fragmented _ROS2_ development with a unified workflow, offering a high-level API to build robust components and orchestrate them into cohesive, self-healing systems.

By replacing verbose boilerplate and static launch files with an **Event-Driven API**, Sugarcoat allows you to orchestrate complex robotic behaviors with the elegance of modern Python.

## Why Sugarcoat? Bridging the Orchestration Gap

In the standard ROS2 ecosystem, developers are given powerful tools to create individual "bricks" (Nodes), but very few tools to create the "building" (the System). As robotic systems scale, they inevitably face the Orchestration Gap: a void between low-level drivers and high-level mission planning.

- **Standard ROS2**: Leads to a "Manager Node" problem. To coordinate nodes, developers write a manager node that quickly becomes a "spaghetti" of callbacks, timers, and hardcoded logic that is difficult to test and prone to failure.
- **Behavior Trees (e.g., Nav2)**: Rely on sequential polling mechanisms ("ticks") that process logic sequentially. They are latency-prone, can block the system's ability to react during complex actions, and make global scope safety triggers (like a universal killswitch) notoriously difficult to implement.

**The Sugarcoat Solution:** Sugarcoat provides an imperative, event-driven middle layer. It operates on a **Parallel Event Engine** that doesn't "tick" through a list; it listens to the entire system at once, offering true distributed automation with immediate microsecond reaction times.

## Key Features & Core Pillars

| Feature                       | Description                                                                                                                                                                                                                    |
| :---------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Smart Components**          | Every component is a managed lifecycle node (Configure, Activate, Deactivate) out of the box. It features type-safe configurations via `attrs` and declarative auto-wiring for inputs/outputs.                                 |
| **Active Resilience**         | <br> Built-in "Immune System" for ROS2 nodes. Components actively report their **Health Status** (Algorithm, Component, or System failures) and automatically trigger distributed **Fallbacks** to self-heal without crashing. |
| **Event-Driven Behavior**     | Define global **Events** (e.g., `Event(battery < 10.0)`) and **Actions** in pure, readable Python. These act as triggers that monitor ROS2 topics natively and execute instantly regardless of current system state.           |
| **Centralized Orchestration** | A powerful **Launcher** acts as a pythonic alternative to `ros2 launch`. It supports multi-threaded or multi-process execution, actively supervising component lifecycles at runtime.                                          |
| **Universal Applications**    | **Robot Plugins** act as a translation layer. This allows you to write generic, portable automation logic (recipes) that run on any robot without code changes.                                                                |
| **Dynamic Web UI**            | Auto-generates a fully functional web frontend for every topic, parameter, and event instantly.                                                                                                                                |

## Packages Built with Sugarcoat

- [**Kompass**](https://automatikarobotics.com/kompass/): A framework for building robust and comprehensive event-driven navigation stacks using an easy-to-use and intuitive Python API.
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/): A fully-loaded framework for creating interactive embodied agents that can think, understand and act.

## Get Started

- Learn more about the [**design concepts**](https://automatika-robotics.github.io/sugarcoat/design/index.html) in Sugarcoat
- Learn how to [**create your own ROS2 package**](https://automatika-robotics.github.io/sugarcoat/use.html) using Sugarcoat
- [**Port your automation recipes across different hardware**](https://automatika-robotics.github.io/sugarcoat/features/robot_plugins.html) using **Robot Plugins**
- Explore the [**Dynamic Web UI**](https://automatika-robotics.github.io/sugarcoat/features/web_ui.html) for real-time system visualization and control

## How Sugarcoat Works

The core of Sugarcoat revolves around bringing seemless orchestration and reactive autonomy to your robot.

### 1. Components (Smart Execution)

A `Component` is your main execution unit, replacing the standard ROS2 Node. It validates its own configuration, automatically wires topics declaratively, and manages its own lifecycle natively.

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/component_light.png">
  <img alt="Base Component Diagram" src="docs/_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

### 2. Events & Actions (Reactive Middle Layer)

Define dynamic behavior using pure Python expressions. Events monitor topics continuously in parallel, completely independent of the execution state of your components.

```python
from ros_sugar.core import Event, Action

# A global event monitored in parallel, executing instantly with zero polling latency
collision_risk = Event(sensor.msg.min_dist < 0.5)

# Trigger a System-Wide Action instantly
launcher.add_pkg(
    components=[...],
    events_actions={collision_risk: Action(stop_motors)}
)

```

### 3. Launcher (Orchestration)

Takes your defined Components, Events, and Actions, and executes the system. The Launcher actively tracks health statuses and orchestrates multi-threaded or multi-process execution cleanly.

<p align="center">
<picture>
<source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/multi_process_dark.png">
<source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/multi_process_light.png">
<img alt="Multi-process execution Diagram" src="docs/_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>

## Dynamic Web UI for Sugarcoat Recipes

The **Dynamic Web UI** feature takes system visibility and control to the next level. Built with **[FastHTML](https://www.fastht.ml/)** and **[MonsterUI](https://monsterui.answer.ai/)**, it is designed to automatically generate a fully dynamic, extensible web interface for any Sugarcoat recipe, completely eliminating the need for manual front-end development.

### Automatic UI Generation in Action

See how the Web UI effortlessly generates interfaces for different types of Sugarcoat recipes:

- **Example 1: General Q&A MLLM Recipe**
  A fully functional interface generated for an MLLM agent recipe from **[EmbodiedAgents](https://automatika-robotics.github.io/embodied-agents/)**, automatically providing controls for settings and real-time text I/O with the robot.

<p align="center">
<picture align="center">
<img alt="EmbodiedAgents UI Example GIF" src="./docs/_static/videos/ui_agents.gif" width="60%">
</picture>
</p>

- **Example 2: Point Navigation Recipe**
  An example for an automatically generated UI for a point navigation system from **[Kompass](https://automatikarobotics.com/kompass/)**. The UI automatically renders map data, and sends navigation goals to the robot.

<p align="center">
<picture align="center">
<img alt="Navigation System UI Example GIF" src="./docs/_static/videos/ui_navigation.gif" width="60%">
</picture>
</p>

### What's Inside?

- **Automatic Settings UI**: Interfaces for configuring the settings of all **Components** used in your recipe are generated on the fly.
- **Auto I/O Visualization**: Front-end controls and data visualizations for UI **Inputs** and **Outputs** are created automatically.
- **WebSocket-Based Streaming**: Features bidirectional, low-latency communication for streaming **text, image, and audio** messages.
- **Responsive Layouts**: Input and output elements are presented in clear, adaptable grid layouts.
- **Extensible Design**: Easily add support for new message types and custom visualizations through extensions.

## Installation

Sugarcoat is available for ROS versions **Humble**.

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

Sugarcoat is made available under the MIT license. Details can be found in the [LICENSE](https://www.google.com/search?q=LICENSE) file.

## Contributions

Sugarcoat has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.

## Hat Tip

The **Dynamic Web UI** is powered by two awesome open-source projects. A big thank you to Answers.ai for their work on:

- **[FastHTML](https://www.fastht.ml/)**: The HTMX based framework that enables automatic generation of our dynamic web interfaces.
- **[MonsterUI](https://monsterui.answer.ai/)**: The styled UI components that make the interface intuitive.
