# {py:mod}`ros_sugar.tf`

```{py:module} ros_sugar.tf
```

```{autodoc2-docstring} ros_sugar.tf
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`TFListenerConfig <ros_sugar.tf.TFListenerConfig>`
  - ```{autodoc2-docstring} ros_sugar.tf.TFListenerConfig
    :summary:
    ```
* - {py:obj}`TFListener <ros_sugar.tf.TFListener>`
  - ```{autodoc2-docstring} ros_sugar.tf.TFListener
    :summary:
    ```
````

### API

`````{py:class} TFListenerConfig
:canonical: ros_sugar.tf.TFListenerConfig

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.tf.TFListenerConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.tf.TFListenerConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.tf.TFListenerConfig.from_dict

````

````{py:method} from_file(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> bool
:canonical: ros_sugar.tf.TFListenerConfig.from_file

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.tf.TFListenerConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.tf.TFListenerConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.tf.TFListenerConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.tf.TFListenerConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.tf.TFListenerConfig.update_value

````

`````

`````{py:class} TFListener(tf_config: typing.Optional[ros_sugar.tf.TFListenerConfig] = None, node_name: str = '')
:canonical: ros_sugar.tf.TFListener

```{autodoc2-docstring} ros_sugar.tf.TFListener
```

````{py:property} translation
:canonical: ros_sugar.tf.TFListener.translation
:type: typing.Optional[numpy.ndarray]

```{autodoc2-docstring} ros_sugar.tf.TFListener.translation
```

````

````{py:property} rotation
:canonical: ros_sugar.tf.TFListener.rotation
:type: typing.Optional[numpy.ndarray]

```{autodoc2-docstring} ros_sugar.tf.TFListener.rotation
```

````

````{py:property} tf_buffer
:canonical: ros_sugar.tf.TFListener.tf_buffer

```{autodoc2-docstring} ros_sugar.tf.TFListener.tf_buffer
```

````

````{py:method} set_listener(tf_listener: tf2_ros.transform_listener.TransformListener)
:canonical: ros_sugar.tf.TFListener.set_listener

```{autodoc2-docstring} ros_sugar.tf.TFListener.set_listener
```

````

````{py:property} timer
:canonical: ros_sugar.tf.TFListener.timer
:type: typing.Optional[rclpy.timer.Timer]

```{autodoc2-docstring} ros_sugar.tf.TFListener.timer
```

````

````{py:method} timer_callback()
:canonical: ros_sugar.tf.TFListener.timer_callback

```{autodoc2-docstring} ros_sugar.tf.TFListener.timer_callback
```

````

````{py:method} check_tf() -> bool
:canonical: ros_sugar.tf.TFListener.check_tf

```{autodoc2-docstring} ros_sugar.tf.TFListener.check_tf
```

````

`````
