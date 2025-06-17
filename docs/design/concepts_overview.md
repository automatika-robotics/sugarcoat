# Concepts Overview

A [Component](component.md) is the main execution unit, each component is configured with [Inputs/Outputs](topics.md) and [Fallback](fallbacks.md) behaviors. Additionally, each component updates its own [Health Status](status.md). Components can be handled and reconfigured dynamically at runtime using [Events](events.md) and [Actions](actions.md). Events, Actions and Components are passed to the [Launcher](launcher.md) which runs the set of components as using multi-threaded or multi-process execution. The Launcher also uses an internal [Monitor](monitor.md) to keep track of the components and monitor events.

```{figure} /_static/images/diagrams/component_dark.png
:class: only-dark
:alt: component
:align: center

Component Structure
```
```{figure} /_static/images/diagrams/component_light.png
:class: only-light
:alt: component
:align: center

Component Structure
```

```{figure} /_static/images/diagrams/multi_threaded_dark.png
:class: only-dark
:alt: multi-threaded
:align: center

Multi-threaded execution
```
```{figure} /_static/images/diagrams/multi_threaded_light.png
:class: only-light
:alt: multi-threaded
:align: center

Multi-threaded execution
```

```{figure} /_static/images/diagrams/multi_process_dark.png
:class: only-dark
:alt: multi-process
:align: center

Multi-process execution
```
```{figure} /_static/images/diagrams/multi_process_light.png
:class: only-light
:alt: multi-process
:align: center

Multi-process execution
```
