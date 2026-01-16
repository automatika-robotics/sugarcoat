# Design Concepts Overview

Sugarcoat is designed to transform standard ROS2 nodes into robust, self-healing, and dynamic building blocks for autonomous systems. The architecture is centered around four key pillars: [**Modular Execution**](#1-the-component-smart-execution), [**Active Resilience**](#2-active-resilience-status--fallbacks), [**Event-Driven Behavior**](#3-dynamic-behavior-events--actions), and [**Centralized Orchestration**](#4-orchestration-launcher--monitor).

The following diagram illustrates how a single Component is structured to handle inputs, outputs, health monitoring, and recovery behaviors internally.


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

## 1. The Component: Smart Execution
At the heart of the system is the [Component](component.md). Unlike a standard ROS2 node, a Sugarcoat Component is Lifecycle-managed and Health-aware by default. It executes a specific logic (e.g., a path planner, a driver, or a controller), and validates its own configuration automatically.

## 2. Active Resilience: Status & Fallbacks
Robots fail. Sugarcoat Components are designed to handle failure gracefully rather than crashing.

- [**Health Status**](status.md): Every component continuously reports its internal state, distinguishing between Algorithm failures (e.g., "no path found"), Component failures (e.g., "driver crash"), or System failures (e.g., "missing input").

- [**Fallbacks**](fallbacks.md): When a failure is detected, the component automatically triggers pre-configured recovery strategies—such as retrying an operation, re-initializing a driver, or safely shutting down—without requiring external intervention.

## 3. Dynamic Behavior: Events & Actions
To create reactive autonomy, Sugarcoat layers an **Event-Driven system** on top of standard data flows.

- [**Events**](events.md): These act as triggers that monitor ROS2 topics for specific conditions (e.g., "Battery < 20%", "Target Lost", or "New Terrain Detected").

- [**Actions**](actions.md): When an Event triggers (or a Fallback activates), an Action is executed. Actions can reconfigure components, switch controllers, start/stop processes, or send goals to other nodes.

## 4. Orchestration: Launcher & Monitor
Finally, the system is brought to life and supervised by the Launcher and Monitor.

- [**Launcher**](launcher.md): A Pythonic interface to define and deploy your system. It supports running components in Multi-threaded or Multi-process modes, managing their lifecycles automatically.

- [**Monitor**](monitor.md) An internal node that runs alongside your components. It acts as the system supervisor, listening to global Events and Component Health Statuses to coordinate system-wide responses.


## Execution Models
Sugarcoat supports flexible execution models to suit your performance needs:

### Multi-threaded Execution:

Components run as threads within a single process, sharing memory for low-latency communication.


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

### Multi-process Execution

Components run in separate processes for isolation and stability, coordinated by the Launcher.

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
