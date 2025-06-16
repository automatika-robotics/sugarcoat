# Concepts Overview

A [Component](component.md) is the main execution unit, each component is configured with [Inputs/Outputs](topics.md) and [Fallback](fallbacks.md) behaviors. Additionally, each component updates its own [Health Status](status.md). Components can be handled and reconfigured dynamically at runtime using [Events](events.md) and [Actions](actions.md). Events, Actions and Components are passed to the [Launcher](launcher.md) which runs the set of components as using multi-threaded or multi-process execution. The Launcher also uses an internal [Monitor](monitor.md) to keep track of the components and monitor events.

:::{figure-md} fig-comp-design

<img src="../_static/images/diagrams/component_light.png" alt="Base Component" width="700px">

Component Structure
:::

:::{figure-md} fig-multi-thread-design

<img src="../_static/images/diagrams/multi_threaded_light.png" alt="Multi-threaded execution" width="500px">

Multi-threaded execution
:::

:::{figure-md} fig-multi-process-design

<img src="../_static/images/diagrams/multi_process_light.png" alt="Multi-process execution" width="500px">

Multi-process execution
:::
