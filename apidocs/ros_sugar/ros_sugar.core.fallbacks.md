# {py:mod}`ros_sugar.core.fallbacks`

```{py:module} ros_sugar.core.fallbacks
```

```{autodoc2-docstring} ros_sugar.core.fallbacks
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Fallback <ros_sugar.core.fallbacks.Fallback>`
  - ```{autodoc2-docstring} ros_sugar.core.fallbacks.Fallback
    :summary:
    ```
* - {py:obj}`ComponentFallbacks <ros_sugar.core.fallbacks.ComponentFallbacks>`
  - ```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks
    :summary:
    ```
````

### API

`````{py:class} Fallback
:canonical: ros_sugar.core.fallbacks.Fallback

```{autodoc2-docstring} ros_sugar.core.fallbacks.Fallback
```

````{py:method} reset_current_idx()
:canonical: ros_sugar.core.fallbacks.Fallback.reset_current_idx

```{autodoc2-docstring} ros_sugar.core.fallbacks.Fallback.reset_current_idx
```

````

````{py:method} reset_retries()
:canonical: ros_sugar.core.fallbacks.Fallback.reset_retries

```{autodoc2-docstring} ros_sugar.core.fallbacks.Fallback.reset_retries
```

````

````{py:method} reset()
:canonical: ros_sugar.core.fallbacks.Fallback.reset

```{autodoc2-docstring} ros_sugar.core.fallbacks.Fallback.reset
```

````

`````

`````{py:class} ComponentFallbacks(on_any_fail: typing.Optional[ros_sugar.core.fallbacks.Fallback] = None, on_component_fail: typing.Optional[ros_sugar.core.fallbacks.Fallback] = None, on_algorithm_fail: typing.Optional[ros_sugar.core.fallbacks.Fallback] = None, on_system_fail: typing.Optional[ros_sugar.core.fallbacks.Fallback] = None, on_giveup: typing.Optional[ros_sugar.core.fallbacks.Fallback] = None)
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks
```

````{py:property} giveup
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.giveup
:type: bool

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.giveup
```

````

````{py:method} reset() -> None
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.reset

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.reset
```

````

````{py:method} reset_execution_indices() -> None
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.reset_execution_indices

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.reset_execution_indices
```

````

````{py:method} reset_retries()
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.reset_retries

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.reset_retries
```

````

````{py:method} execute_giveup()
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.execute_giveup

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.execute_giveup
```

````

````{py:method} execute_component_fallback() -> bool
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.execute_component_fallback

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.execute_component_fallback
```

````

````{py:method} execute_algorithm_fallback() -> bool
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.execute_algorithm_fallback

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.execute_algorithm_fallback
```

````

````{py:method} execute_system_fallback() -> bool
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.execute_system_fallback

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.execute_system_fallback
```

````

````{py:method} execute_generic_fallback() -> bool
:canonical: ros_sugar.core.fallbacks.ComponentFallbacks.execute_generic_fallback

```{autodoc2-docstring} ros_sugar.core.fallbacks.ComponentFallbacks.execute_generic_fallback
```

````

`````
