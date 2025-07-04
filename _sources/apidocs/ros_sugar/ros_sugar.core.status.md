# {py:mod}`ros_sugar.core.status`

```{py:module} ros_sugar.core.status
```

```{autodoc2-docstring} ros_sugar.core.status
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Status <ros_sugar.core.status.Status>`
  - ```{autodoc2-docstring} ros_sugar.core.status.Status
    :summary:
    ```
````

### API

`````{py:class} Status(msg: typing.Optional[automatika_ros_sugar.msg.ComponentStatus] = None)
:canonical: ros_sugar.core.status.Status

```{autodoc2-docstring} ros_sugar.core.status.Status
```

````{py:property} value
:canonical: ros_sugar.core.status.Status.value

```{autodoc2-docstring} ros_sugar.core.status.Status.value
```

````

````{py:method} set_healthy()
:canonical: ros_sugar.core.status.Status.set_healthy

```{autodoc2-docstring} ros_sugar.core.status.Status.set_healthy
```

````

````{py:method} set_failure()
:canonical: ros_sugar.core.status.Status.set_failure

```{autodoc2-docstring} ros_sugar.core.status.Status.set_failure
```

````

````{py:method} set_fail_algorithm(algorithm_names: typing.Optional[typing.List[str]] = None)
:canonical: ros_sugar.core.status.Status.set_fail_algorithm

```{autodoc2-docstring} ros_sugar.core.status.Status.set_fail_algorithm
```

````

````{py:method} set_fail_component(component_names: typing.Optional[typing.List[str]] = None)
:canonical: ros_sugar.core.status.Status.set_fail_component

```{autodoc2-docstring} ros_sugar.core.status.Status.set_fail_component
```

````

````{py:method} set_fail_system(component_names: typing.Optional[typing.List[str]] = None, topic_names: typing.Optional[typing.List[str]] = None)
:canonical: ros_sugar.core.status.Status.set_fail_system

```{autodoc2-docstring} ros_sugar.core.status.Status.set_fail_system
```

````

````{py:property} is_healthy
:canonical: ros_sugar.core.status.Status.is_healthy
:type: bool

```{autodoc2-docstring} ros_sugar.core.status.Status.is_healthy
```

````

````{py:property} is_component_fail
:canonical: ros_sugar.core.status.Status.is_component_fail
:type: bool

```{autodoc2-docstring} ros_sugar.core.status.Status.is_component_fail
```

````

````{py:property} is_algorithm_fail
:canonical: ros_sugar.core.status.Status.is_algorithm_fail
:type: bool

```{autodoc2-docstring} ros_sugar.core.status.Status.is_algorithm_fail
```

````

````{py:property} is_system_fail
:canonical: ros_sugar.core.status.Status.is_system_fail
:type: bool

```{autodoc2-docstring} ros_sugar.core.status.Status.is_system_fail
```

````

`````
