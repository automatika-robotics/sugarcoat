---
orphan: true
---

# {py:mod}`ros_sugar.launch.launch_actions`

```{py:module} ros_sugar.launch.launch_actions
```

```{autodoc2-docstring} ros_sugar.launch.launch_actions
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ComponentLaunchAction <ros_sugar.launch.launch_actions.ComponentLaunchAction>`
  - ```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction
    :summary:
    ```
````

### API

`````{py:class} ComponentLaunchAction(*, node: typing.Union[ros_sugar.core.component.BaseComponent, ros_sugar.core.monitor.Monitor], name: typing.Union[str, typing.List[launch.Substitution], None] = 'node_name', namespace: typing.Union[str, typing.List[launch.Substitution], None] = None, log_level: rclpy.impl.logging_severity.LoggingSeverity = LoggingSeverity.INFO, **kwargs)
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction

Bases: {py:obj}`launch_ros.actions.Node`

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction
```

````{py:property} name
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.name
:type: typing.Union[str, typing.List[launch.Substitution], None]

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.name
```

````

````{py:method} execute(context: launch.LaunchContext) -> typing.Optional[typing.List[launch.action.Action]]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.execute

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.execute
```

````

````{py:property} executor
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.executor

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.executor
```

````

````{py:method} shutdown()
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.shutdown

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.shutdown
```

````

`````
