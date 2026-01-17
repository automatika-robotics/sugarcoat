# {py:mod}`ros_sugar.core.monitor`

```{py:module} ros_sugar.core.monitor
```

```{autodoc2-docstring} ros_sugar.core.monitor
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Monitor <ros_sugar.core.monitor.Monitor>`
  - ```{autodoc2-docstring} ros_sugar.core.monitor.Monitor
    :summary:
    ```
````

### API

`````{py:class} Monitor(components_names: typing.List[str], enable_health_status_monitoring: bool = True, events_actions: typing.Optional[typing.Dict[str, typing.List[ros_sugar.core.action.Action]]] = None, events_to_emit: typing.Optional[typing.List[ros_sugar.core.event.Event]] = None, config: typing.Optional[ros_sugar.config.BaseConfig] = None, services_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, action_servers_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, activate_on_start: typing.Optional[typing.List[str]] = None, activation_timeout: typing.Optional[float] = None, activation_attempt_time: float = 1.0, component_name: str = 'monitor', **_)
:canonical: ros_sugar.core.monitor.Monitor

Bases: {py:obj}`rclpy.node.Node`

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor
```

````{py:method} rclpy_init_node(*args, **kwargs)
:canonical: ros_sugar.core.monitor.Monitor.rclpy_init_node

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.rclpy_init_node
```

````

````{py:method} add_components_activation_event(method) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_components_activation_event

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.add_components_activation_event
```

````

````{py:method} activate()
:canonical: ros_sugar.core.monitor.Monitor.activate

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.activate
```

````

````{py:method} configure_component(component: ros_sugar.core.component.BaseComponent, new_config: typing.Union[object, str], keep_alive: bool) -> None
:canonical: ros_sugar.core.monitor.Monitor.configure_component

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.configure_component
```

````

````{py:method} update_parameter(component: ros_sugar.core.component.BaseComponent, param_name: str, new_value: typing.Any, keep_alive: bool = True) -> None
:canonical: ros_sugar.core.monitor.Monitor.update_parameter

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.update_parameter
```

````

````{py:method} update_parameters(component: ros_sugar.core.component.BaseComponent, params_names: typing.List[str], new_values: typing.List, keep_alive: bool = True, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.update_parameters

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.update_parameters
```

````

````{py:method} send_srv_request(srv_name: str, srv_type: type, srv_request_msg: typing.Any, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.send_srv_request

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.send_srv_request
```

````

````{py:method} send_action_goal(action_name: str, action_type: type, action_request_msg: typing.Any, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.send_action_goal

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.send_action_goal
```

````

````{py:method} publish_message(topic: ros_sugar.io.topic.Topic, msg: typing.Any, publish_rate: typing.Optional[float] = None, publish_period: typing.Optional[float] = None, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.publish_message

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.publish_message
```

````

`````
