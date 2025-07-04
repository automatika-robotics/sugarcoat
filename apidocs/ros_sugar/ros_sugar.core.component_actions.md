# {py:mod}`ros_sugar.core.component_actions`

```{py:module} ros_sugar.core.component_actions
```

```{autodoc2-docstring} ros_sugar.core.component_actions
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ComponentActions <ros_sugar.core.component_actions.ComponentActions>`
  - ```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions
    :summary:
    ```
````

### API

`````{py:class} ComponentActions
:canonical: ros_sugar.core.component_actions.ComponentActions

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions
```

````{py:method} send_srv_request(*, srv_name: str, srv_type: type, srv_request_msg: typing.Any) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.send_srv_request
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.send_srv_request
```

````

````{py:method} send_action_goal(*, action_name: str, action_type: type, action_request_msg: typing.Any) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.send_action_goal
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.send_action_goal
```

````

````{py:method} publish_message(*, topic: ros_sugar.io.topic.Topic, msg: typing.Any, publish_rate: typing.Optional[float] = None, publish_period: typing.Optional[float] = None) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.publish_message
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.publish_message
```

````

````{py:method} start(*, component: ros_sugar.core.component.BaseComponent) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.start
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.start
```

````

````{py:method} stop(*, component: ros_sugar.core.component.BaseComponent) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.stop
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.stop
```

````

````{py:method} restart(*, component: ros_sugar.core.component.BaseComponent, wait_time: typing.Optional[float] = None) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.restart
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.restart
```

````

````{py:method} reconfigure(*, component: ros_sugar.core.component.BaseComponent, new_config: typing.Union[str, object], keep_alive: bool = False) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.reconfigure
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.reconfigure
```

````

````{py:method} update_parameter(*, component: ros_sugar.core.component.BaseComponent, param_name: str, new_value: typing.Any, keep_alive: bool = True) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.update_parameter
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.update_parameter
```

````

````{py:method} update_parameters(*, component: ros_sugar.core.component.BaseComponent, params_names: typing.List[str], new_values: typing.List, keep_alive: bool = True) -> ros_sugar.core.action.Action
:canonical: ros_sugar.core.component_actions.ComponentActions.update_parameters
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.update_parameters
```

````

````{py:method} log(*, msg: str, logger_name: typing.Optional[str] = None) -> ros_sugar.core.action.LogInfo
:canonical: ros_sugar.core.component_actions.ComponentActions.log
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component_actions.ComponentActions.log
```

````

`````
