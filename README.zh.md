<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="_static/SUGARCOAT_DARK.png"  width="50%">
</picture>
<br/><br/>

> 🌐 [English Version](../README.md) | 🇯🇵 [日本語版](README.ja.md)

Sugarcoat 🍬 是一个元框架，它为在 ROS2 中创建事件驱动的多节点系统提供了大量的语法糖，并使用直观的 Python API。

- 📚 了解更多关于 Sugarcoat 的[**设计概念**](https://automatika-robotics.github.io/sugarcoat/design/index.html)
- 🚀 学习如何使用 Sugarcoat [**创建你自己的 ROS2 包**](https://automatika-robotics.github.io/sugarcoat/use.html)

## 使用 Sugarcoat 创建的包

- [**Kompass**](https://automatikarobotics.com/kompass/)：一个用于构建健壮和全面的事件驱动导航堆栈的框架，它使用易于使用和直观的 Python API
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/)：一个功能齐全的框架，用于创建交互式物理代理，这些代理可以理解、记住并根据其环境中的上下文信息采取行动。

## 概述

Sugarcoat 专为 ROS2 开发者而设计，他们希望创建易于使用、内置回退和容错功能，并且可以通过直观的 Python API 进行配置和启动的事件驱动多节点系统。它提供了编写 ROS 节点以及启动/停止/修改节点的事件/动作的原语，秉承了事件驱动软件的精神。Sugarcoat 也可以替代 ROS Launch API。

[组件](https://automatika-robotics.github.io/sugarcoat/design/component.html) 是 Sugarcoat 中的主要执行单元，每个组件都配置有[输入/输出](https://automatika-robotics.github.io/sugarcoat/design/topics.md) 和 [回退](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html) 行为。此外，每个组件都会更新其自身的 [健康状态](https://automatika-robotics.github.io/sugarcoat/design/status.html)。组件可以在运行时使用[事件](https://automatika-robotics.github.io/sugarcoat/design/events.html)和[动作](https://automatika-robotics.github.io/sugarcoat/design/actions.html) 进行动态处理和重新配置。事件、动作和组件被传递给[启动器](https://automatika-robotics.github.io/sugarcoat/design/launcher.html)，启动器使用多线程或多进程执行来运行组件集。启动器还使用内部[监视器](https://automatika-robotics.github.io/sugarcoat/design/monitor.html)来跟踪组件并监视事件。

## 基础组件

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/component_light.png">
  <img alt="Base Component" src="_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

## 多进程执行

<p align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/multi_process_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/multi_process_light.png">
  <img alt="Multi-process execution" src="_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>

## 安装

对于 ROS 版本大于等于 _humble_ 的用户，可以通过包管理器安装 Sugarcoat。例如，在 Ubuntu 上：

`sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar`

或者，也可以从 [发布页面](https://github.com/automatika-robotics/sugarcoat/releases) 下载你喜欢的 deb 安装包，并使用以下命令进行安装：

`sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb`

如果你使用的包管理器中的 attrs 版本小于 23.2，请使用 pip 安装如下：

`pip install 'attrs>=23.2.0'`

## 从源代码构建

```shell
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src
git clone https://github.com/automatika-robotics/sugarcoat && cd ..
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml
colcon build
source install/setup.bash
```

## 版权

除非另有明确说明，本发行版中的代码版权所有 (c) 2024 Automatika Robotics。

Sugarcoat 根据 MIT 许可证提供。详细信息可在 [LICENSE](LICENSE) 文件中找到。

## 贡献

Sugarcoat 是由 [Automatika Robotics](https://automatikarobotics.com/) 和 [Inria](https://inria.fr/) 合作开发的。欢迎社区贡献。
