<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="_static/SUGARCOAT_DARK.png"  width="50%">
</picture>

<br/>

🇨🇳 [简体中文](README.zh.md) | 🇯🇵 [日本語](README.ja.md)

## ROS2 事件驱动系统的编排层

**Sugarcoat** 是一个元框架，它用统一的工作流取代了碎片化的 _ROS2_ 开发，提供了一个高级 API 来构建健壮的组件，并将它们编排成具有凝聚力、能够自我修复的系统。

通过用声明式的 **事件驱动 API (Event-Driven API)** 取代冗长的样板代码和静态启动文件，Sugarcoat 让您能够以现代 Python 的优雅来编排复杂的机器人行为。

## 为什么选择 Sugarcoat？弥合编排鸿沟

在标准的 ROS2 生态系统中，开发者拥有强大的工具来创建单个的“砖块”（节点），但几乎没有工具来建造“大楼”（系统）。随着机器人系统规模的扩大，它们不可避免地会面临“编排鸿沟 (Orchestration Gap)”：即底层驱动与高级任务规划之间的空白。

- **标准 ROS2**：会导致“管理器节点”问题。为了协调各个节点，开发者需要编写一个管理器节点，这很快就会变成由回调、定时器和硬编码逻辑组成的“意大利面条式”代码，难以测试且容易出错。
- **行为树 (例如 Nav2)**：依赖于顺序轮询机制（"ticks"）来按顺序处理逻辑。它们容易产生延迟，可能在复杂动作期间阻塞系统的反应能力，并且使得全局范围的安全触发器（如通用紧急停止开关）极难实现。

**Sugarcoat 的解决方案**：Sugarcoat 提供了一个命令式的、事件驱动的中间层。它运行在一个**并行事件引擎**上，该引擎不通过列表进行“tick”轮询；它同时监听整个系统，提供真正的分布式自动化和微秒级的即时反应时间。

## 核心特性与支柱

| 特性                                     | 描述                                                                                                                                                                              |
| :--------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **智能组件 (Smart Components)**          | 开箱即用，每个组件都是一个受管的生命周期节点（配置、激活、停用）。它通过 `attrs` 提供类型安全的配置，并为输入/输出提供声明式的自动连线。                                          |
| **主动恢复 (Active Resilience)**         | <br> 为 ROS2 节点内置的“免疫系统”。组件会主动报告其**健康状态 (Health Status)**（算法、组件或系统故障），并自动触发分布式**后备方案 (Fallbacks)** 以进行自我修复而不会崩溃。      |
| **事件驱动行为 (Event-Driven Behavior)** | 使用纯粹且易读的 Python 表达式定义全局**事件 (Events)**（例如 `Event(battery < 10.0)`）和**动作 (Actions)**。它们作为触发器原生监控 ROS2 话题，无论当前系统状态如何都能立即执行。 |
| **集中编排 (Centralized Orchestration)** | 强大的**启动器 (Launcher)** 作为 `ros2 launch` 的 Pythonic 替代方案。它支持多线程或多进程执行，在运行时主动监督组件的生命周期。                                                   |
| **通用应用 (Universal Applications)**    | **机器人插件 (Robot Plugins)** 充当翻译层。这使得您可以编写通用的、可移植的自动化逻辑（配方 / recipes），且无需修改代码即可在任何机器人上运行。                                   |
| **动态 Web UI**                          | 实时自动为每个话题、参数和事件生成全功能的 Web 前端界面。                                                                                                                         |

## 基于 Sugarcoat 构建的软件包

- [**Kompass**](https://automatikarobotics.com/kompass/)：一个用于构建健壮且全面的事件驱动导航栈的框架，使用易于使用且直观的 Python API。
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/)：一个全功能的框架，用于创建能够思考、理解和行动的交互式具身智能体 (Embodied Agents)。

## 快速入门

- 了解更多关于 Sugarcoat 的[**设计理念**](https://automatika-robotics.github.io/sugarcoat/design/index.html)
- 学习如何使用 Sugarcoat [**创建您自己的 ROS2 软件包**](https://automatika-robotics.github.io/sugarcoat/use.html)
- 使用 **机器人插件** [**跨不同硬件移植您的自动化配方**](https://automatika-robotics.github.io/sugarcoat/features/robot_plugins.html)
- 探索 [**动态 Web UI**](https://automatika-robotics.github.io/sugarcoat/features/web_ui.html) 以进行实时系统可视化和控制

## Sugarcoat 如何工作

Sugarcoat 的核心围绕着为您的机器人引入集中式编排和反应式自主性。

### 1. 组件 (智能执行)

`Component` (组件) 是您的主要执行单元，取代了标准的 ROS2 节点。它会验证自身的配置，声明式地自动连接话题，并原生管理自身的生命周期。

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/component_light.png">
  <img alt="Base Component Diagram" src="_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

### 2. 事件与动作 (反应式中间层)

使用纯 Python 表达式定义动态行为。事件在并行中持续监控话题，完全独立于组件的执行状态。

```python
from ros_sugar.core import Event, Action

# 在并行中监控全局事件，以零轮询延迟即时执行
collision_risk = Event(sensor.msg.min_dist < 0.5)

# 即时触发系统级动作
launcher.add_pkg(
    components=[...],
    events_actions={collision_risk: Action(stop_motors)}
)
```

### 3. 启动器 (编排)

获取您定义的组件、事件和动作，并执行系统。启动器主动跟踪健康状态，并清晰地编排多线程或多进程执行。

<p align="center">
<picture>
<source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/multi_process_dark.png">
<source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/multi_process_light.png">
<img alt="Multi-process execution Diagram" src="_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>

## Sugarcoat 配方的动态 Web UI

**动态 Web UI** 功能将系统可见性和控制力提升到了一个新水平。该功能基于 **[FastHTML](https://www.fastht.ml/)** 和 **[MonsterUI](https://monsterui.answer.ai/)** 构建，旨在为任何 Sugarcoat 配方自动生成完全动态、可扩展的 Web 界面，从而彻底消除了手动进行前端开发的需要。

此功能可瞬间将复杂的、多节点的 ROS2 系统转变为可监控和可配置的 Web 应用程序。

### 自动生成 UI 演示

查看 Web UI 如何毫不费力地为不同类型的 Sugarcoat 配方生成界面：

- **示例 1：通用问答 MLLM 配方**
  为 **[EmbodiedAgents](https://automatika-robotics.github.io/embodied-agents/)** 中的 MLLM 智能体配方自动生成的全功能界面，自动提供设置控制以及与机器人的实时文本 I/O。

<p align="center">
<picture align="center">
<img alt="EmbodiedAgents UI Example GIF" src="_static/videos/ui_agents.gif" width="60%">
</picture>
</p>

- **示例 2：定点导航配方**
  为 **[Kompass](https://automatikarobotics.com/kompass/)** 中的点到点导航系统自动生成的 UI 示例。该 UI 自动渲染地图数据，并向机器人发送导航目标。

<p align="center">
<picture align="center">
<img alt="Navigation System UI Example GIF" src="_static/videos/ui_navigation.gif" width="60%">
</picture>
</p>

### 包含哪些功能？

- **自动设置 UI**：实时生成界面，用于配置配方中使用的所有**组件**的设置。
- **自动 I/O 可视化**：自动创建用于 UI **输入**和**输出**的前端控件和数据可视化。
- **基于 WebSocket 的流传输**：具有双向、低延迟通信功能，可流式传输**文本、图像和音频**消息。
- **响应式布局**：输入和输出元素以清晰、适应性强的网格布局呈现。
- **可扩展设计**：通过扩展轻松添加对新消息类型和自定义可视化的支持。

## 安装

Sugarcoat 支持 **Humble** 的 ROS 版本。

### 使用包管理器 (推荐)

例如，在 Ubuntu 上：

`sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar`

或者，您可以从 [Releases 页面](https://github.com/automatika-robotics/sugarcoat/releases) 安装特定的 deb 包：

`sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb`

> **注意：** 如果您的包管理器中的 `attrs` 版本低于 23.2，您可能需要通过 pip 更新它：
> `pip install 'attrs>=23.2.0'`

### 源码编译

```shell
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src
git clone [https://github.com/automatika-robotics/sugarcoat](https://github.com/automatika-robotics/sugarcoat) && cd ..

# 安装依赖项 (确保包含 attrs>=23.2.0)
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml

colcon build
source install/setup.bash
```

## 版权声明

除非另有说明，本发行版中的代码版权归 Automatika Robotics (c) 2024 所有。

Sugarcoat 基于 MIT 许可证提供。详情请参阅 [LICENSE](https://www.google.com/search?q=LICENSE) 文件。

## 贡献

Sugarcoat 由 [Automatika Robotics](https://automatikarobotics.com/) 和 [Inria](https://inria.fr/) 合作开发。非常欢迎社区的贡献。

## 致谢

**动态 Web UI** 由两个出色的开源项目提供支持。非常感谢 Answers.ai 在以下项目中的工作：

- **[FastHTML](https://www.fastht.ml/)**：基于 HTMX 的框架，使我们能够自动生成动态 Web 界面。
- **[MonsterUI](https://monsterui.answer.ai/)**：样式化的 UI 组件，使界面直观易用。
