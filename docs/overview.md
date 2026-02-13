# Sugarcoat

**The Orchestration Layer for Event-Driven ROS2 Systems**

<p style="font-size: 1.2em; line-height: 1.6; opacity: 0.9;">
  Sugarcoat is a <b>meta-framework</b> that replaces fragmented *ROS2* development with a unified workflow, offering a high-level API to build robust components and orchestrate them into cohesive, self-healing systems. It replaces verbose boilerplate and static launch files with a declarative <b>Event-Driven API</b>, allowing you to orchestrate complex robotic behaviors with the elegance of modern Python.
</p>

[Get Started](install.md) • [Why Sugarcoat?](why.md) • [View on GitHub](https://github.com/automatika-robotics/sugarcoat)


- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`auto_awesome;1.5em;sd-text-primary` Syntactic Sugar</span> -
  Write clean, imperative code. Define components, topics, and events without the repetitive ROS2 boilerplate.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`hub;1.5em;sd-text-primary` Event-Driven Core</span> -
  Connect system states to actions. Trigger safety protocols or mode changes using native Python expressions.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`health_and_safety;1.5em;sd-text-primary` Built-in Resilience</span> -
  Native support for health status, automated node recovery, and fallback actions.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`terminal;1.5em;sd-text-primary` Beyond Static Launching</span> -
  A more flexible, Python-native alternative to `ros2 launch` that gives you full runtime control.



::::{grid} 1 2 2 3
:gutter: 2

:::{grid-item-card} {material-regular}`download;1.5em;sd-text-primary` Installation
:link: install
:link-type: doc

Install Sugarcoat to start building with it
:::

:::{grid-item-card} {material-regular}`bolt;1.5em;sd-text-primary` Why Sugarcoat?
:link: why
:link-type: doc

Discover the advantages of _Sugarcoat-ing_ your standard ROS2 nodes
:::

:::{grid-item-card} {material-regular}`extension;1.5em;sd-text-primary` Design Concepts
:link: design/concepts_overview
:link-type: doc

Learn about Components, Launchers, and behind the scenes architecture
:::

:::{grid-item-card} {material-regular}`rocket_launch;1.5em;sd-text-primary` Create a Package
:link: advanced/use
:link-type: doc

Step-by-step guide to creating your own ROS2 package using Sugarcoat
:::

:::{grid-item-card} {material-regular}`power;1.5em;sd-text-primary` Robot Plugins
:link: features/robot_plugins
:link-type: doc

Learn how your ROS2 based system runs on differents hardware seemlessly
:::

:::{grid-item-card} {material-regular}`desktop_windows;1.5em;sd-text-primary` Web UI
:link: features/web_ui
:link-type: doc

Explore the Dynamic Web UI for real-time visualization and control
:::
::::


## Ecosystem

Frameworks built using the Sugarcoat standard:

::::{grid} 1 1 2 2
:gutter: 3

:::{grid-item-card} <span class="text-red-strong">Kompass</span>
:link: https://automatikarobotics.com/kompass/
:class-card: sugar-card

A framework for building robust and comprehensive event-driven navigation stacks.
:::

:::{grid-item-card}  <span class="text-red-strong">EmbodiedAgents</span>
:link: https://automatika-robotics.github.io/embodied-agents/
:class-card: sugar-card

A fully-loaded framework for creating interactive embodied agents that can think, understand and act.
:::
::::



## Contributions

Sugarcoat has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.

