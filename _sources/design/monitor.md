# Monitor

:::{note} The Monitor is an internal element of Sugarcoat launch system that is configured automatically for you. The following documentation aims only to explain its internal usage.
:::

Monitor is a ROS2 Node (not Lifecycle) responsible of monitoring the status of the stack (rest of the running nodes) and managing requests/responses from the Orchestrator.



## Main Functionalities:
- Creates Subscribers to registered Events. The Monitor is configured to declare an InternalEvent back to the Launcher so the corresponding Action can be executed (see source implementation in launch_actions.py)


<!-- ![Monitoring events](../_static/images/diagrams/events_actions_config_light.png)
![An Event Trigger](../_static/images/diagrams/events_actions_exec_light.png) -->

:::{figure-md} fig-monitor_event_config

<img src="../_static/images/diagrams/events_actions_config_light.png" alt="Monitoring events" width="400px">

Monitoring events
:::


:::{figure-md} fig-monitor_event_exec

<img src="../_static/images/diagrams/events_actions_exec_light.png" alt="An Event Trigger" width="400px">

An Event Trigger
:::


- Creates Subscribers to all registered Components health status topics
- Creates clients for all components main services and main action servers
- Creates service clients to components reconfiguration services to handle actions sent from the Launcher

