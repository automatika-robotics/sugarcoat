# Monitor

:::{note} The Monitor is an internal element of Sugarcoat launch system that is configured automatically for you. The following documentation aims only to explain its internal usage.
:::

Monitor is a ROS2 Node (not Lifecycle) responsible of monitoring the status of the stack (rest of the running nodes) and managing requests/responses from the Orchestrator.



## Main Functionalities:
- Creates Subscribers to registered Events. The Monitor is configured to declare an InternalEvent back to the Launcher so the corresponding Action can be executed (see source implementation in launch_actions.py)


```{figure} /_static/images/diagrams/events_actions_config_dark.png
:class: dark-only
:alt: Monitoring events diagram
:align: center
:scale: 70

```

```{figure} /_static/images/diagrams/events_actions_config_light.png
:class: light-only
:alt: Monitoring events diagram
:align: center
:scale: 70

Monitoring events
```



```{figure} /_static/images/diagrams/events_actions_exec_dark.png
:class: dark-only
:alt: An Event Trigger diagram
:align: center
:scale: 70

```

```{figure} /_static/images/diagrams/events_actions_exec_light.png
:class: light-only
:alt: An Event Trigger diagram
:align: center
:scale: 70

An Event Trigger
```



- Creates Subscribers to all registered Components health status topics
- Creates clients for all components main services and main action servers
- Creates service clients to components reconfiguration services to handle actions sent from the Launcher
