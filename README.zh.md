<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="_static/SUGARCOAT_DARK.png"  width="50%">
</picture>
<br/><br/>

> 🌐 [English Version](../README.md) | 🇯🇵 [日本語版](README.ja.md)

## 构建 ROS2 系统的最甜方式

**Sugarcoat** 是一个为 **ROS2** 提供强大**语法糖**的元框架，通过直观的 **Python API**，让你能够轻松构建复杂的、事件驱动的多节点系统。

## 主要特性

| 特性 | 描述 |
| :--- | :--- |
| **事件驱动核心** | 内置的 **事件（Events）** 和 **动作（Actions）** 原语，使你能够在运行时动态配置和控制系统的 **组件（Components）**。 |
| **内置韧性** | **回退机制（Fallbacks）** 和 **容错设计（Fault Tolerance）** 是核心概念，确保系统的健壮性与可靠性。 |
| **直观的 Python API** | 使用简洁、可读的 Python 代码设计整个系统——包括节点、事件和动作。 |
| **动态 Web UI** | 自动生成可动态扩展的 Web 界面，用于监控和配置系统。 |
| **通用应用 [使用机器人插件](https://www.youtube.com/watch?v=oZN6pcJKgfY) （!新）** | 允许您编写可在任何机器人上运行而无需更改代码的通用、可移植的自动化逻辑 |
| **Launch 替代方案** | 一个比 ROS2 Launch API 更加 Pythonic 的替代方案，为真实应用提供更灵活的运行时控制能力。 |

## 基于 Sugarcoat 构建的框架

- [**Kompass**](https://automatikarobotics.com/kompass/): 一个基于事件驱动的导航栈框架，使用简单直观的 Python API 构建强健且全面的导航系统。
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/): 一个用于创建交互式物理智能体的完整框架，使其能够理解、记忆并基于环境上下文采取行动。

## 快速上手

- 了解 Sugarcoat 的[**设计概念**](https://automatika-robotics.github.io/sugarcoat/design/index.html) 📚
- 学习如何使用 Sugarcoat [**创建你自己的 ROS2 包**](https://automatika-robotics.github.io/sugarcoat/use.html) 🚀
- [**将您的自动化配方移植到不同的硬件上**](https://automatika-robotics.github.io/sugarcoat/advanced/robot_plugins.html) 使用 **机器人插件**
- 探索 [**动态 Web UI**](https://automatika-robotics.github.io/sugarcoat/advanced/web_ui.html) 以实现实时系统可视化和控制

## **（全新！）** 介绍 Sugarcoat Recipes 的动态 Web UI

全新的 **动态 Web UI** 功能将系统的可视化与控制提升到新的高度。
它基于 [**FastHTML**](https://www.fastht.ml/) 和 [**MonsterUI**](https://monsterui.answer.ai/) 构建，能够为任何 Sugarcoat recipe 自动生成动态、可扩展的 Web 界面，完全消除手动前端开发的需求。

这一特性可即时将复杂的多节点 ROS2 系统转化为可监控、可配置的 Web 应用。

### 自动 UI 生成演示

看看 Web UI 如何为不同类型的 Sugarcoat recipe 自动生成界面：

- **示例 1：通用问答型 MLLM Recipe**
  为来自 [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/) 的 MLLM 智能体 recipe 自动生成完整的交互界面，提供设置控制以及与机器人实时文本交互的功能。

<p align="center">
<picture align="center">
  <img alt="EmbodiedAgents UI 示例 GIF" src="_static/images/agents_ui.gif" width="60%">
</picture>
</p>

- **示例 2：视觉跟随 Recipe**
  一个使用 [**Kompass**](https://automatikarobotics.com/kompass/) 与 [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/) 组件的复杂系统，用于控制机器人运动并跟踪视觉目标。
  UI 会自动渲染图像数据、检测结果和动作指令，展示其在多媒体与复杂组件交互场景中的强大能力。

<p align="center">
<picture align="center">
  <img alt="KOMPASS UI 示例 GIF" src="_static/images/follow_ui.gif" width="60%">
</picture>
</p>

### 功能概览

- **自动设置界面**：为所有 **组件（Components）** 的配置选项即时生成界面。
- **自动 I/O 可视化**：前端控件与数据可视化会根据 **输入（Inputs）** 与 **输出（Outputs）** 自动创建。
- **基于 WebSocket 的流式通信**：支持双向低延迟的 **文本、图像与音频** 数据流。
- **响应式布局**：输入与输出元素以清晰、可适配的网格布局呈现。
- **可扩展设计**：通过扩展机制轻松添加新消息类型与自定义可视化模块。

## Sugarcoat 的工作原理

Sugarcoat 的核心围绕以下几个概念：

- **组件（Component）**：主要执行单元（ROS2 生命周期节点的抽象），通过 **输入/输出** 与 **回退行为（Fallback）** 配置。每个组件都会报告其 **健康状态（Health Status）**。
  [了解更多关于组件的内容](https://automatika-robotics.github.io/sugarcoat/design/component.html)

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/component_light.png">
  <img alt="基础组件结构图" src="_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

- **事件与动作（Events & Actions）**：在运行时动态处理与重配置组件的机制。
  [了解更多关于事件](https://automatika-robotics.github.io/sugarcoat/design/events.html) ｜ [了解更多关于动作](https://automatika-robotics.github.io/sugarcoat/design/actions.html)

- **启动器（Launcher）**：执行你定义的组件、事件与动作，可通过多线程或多进程运行。
  内部的 **监控器（Monitor）** 负责管理组件生命周期并跟踪事件。
  [了解更多关于启动器的内容](https://automatika-robotics.github.io/sugarcoat/design/launcher.html)

<p align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/multi_process_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/multi_process_light.png">
  <img alt="多进程执行结构图" src="_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>

## 🛠️ 安装

Sugarcoat 适用于 ROS 版本 $\ge$ **Humble**。

### 使用包管理器（推荐）

以 Ubuntu 为例：

`sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar`

或者，从[发布页面](https://github.com/automatika-robotics/sugarcoat/releases)安装指定版本的 deb 包：

`sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb`

> **注意：** 如果你的包管理器中的 `attrs` 版本低于 23.2，请使用 pip 更新：
> `pip install 'attrs>=23.2.0'`

## 从源代码构建

```shell
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src
git clone [https://github.com/automatika-robotics/sugarcoat](https://github.com/automatika-robotics/sugarcoat) && cd ..

# Install dependencies (ensure attrs>=23.2.0 is included)
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml

colcon build
source install/setup.bash
```

## 版权

除非另有明确说明，本发行版中的代码版权所有 (c) 2024 Automatika Robotics。

Sugarcoat 根据 MIT 许可证提供。详细信息可在 [LICENSE](LICENSE) 文件中找到。

## 贡献

Sugarcoat 是由 [Automatika Robotics](https://automatikarobotics.com/) 和 [Inria](https://inria.fr/) 合作开发的。欢迎社区贡献。

## 🎩 致敬

**动态 Web UI** 由两个非常出色的开源项目驱动。
特别感谢 Answers.ai 团队的卓越工作：

- [**FastHTML**](https://www.fastht.ml/): 基于 HTMX 的框架，使我们的动态 Web 界面能够自动生成。
- [**MonsterUI**](https://monsterui.answer.ai/): 提供优雅且直观的 UI 组件，让界面更具可用性与美感。

