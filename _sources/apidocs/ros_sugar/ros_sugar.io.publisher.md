# {py:mod}`ros_sugar.io.publisher`

```{py:module} ros_sugar.io.publisher
```

```{autodoc2-docstring} ros_sugar.io.publisher
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Publisher <ros_sugar.io.publisher.Publisher>`
  - ```{autodoc2-docstring} ros_sugar.io.publisher.Publisher
    :summary:
    ```
````

### API

`````{py:class} Publisher(output_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.publisher.Publisher

```{autodoc2-docstring} ros_sugar.io.publisher.Publisher
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.publisher.Publisher.set_node_name

```{autodoc2-docstring} ros_sugar.io.publisher.Publisher.set_node_name
```

````

````{py:method} set_publisher(publisher: rclpy.publisher.Publisher) -> None
:canonical: ros_sugar.io.publisher.Publisher.set_publisher

```{autodoc2-docstring} ros_sugar.io.publisher.Publisher.set_publisher
```

````

````{py:method} add_pre_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.publisher.Publisher.add_pre_processors

```{autodoc2-docstring} ros_sugar.io.publisher.Publisher.add_pre_processors
```

````

````{py:method} publish(output: typing.Any, *args, frame_id: typing.Optional[str] = None, time_stamp: typing.Optional[builtin_interfaces.msg.Time] = None, **kwargs) -> None
:canonical: ros_sugar.io.publisher.Publisher.publish

```{autodoc2-docstring} ros_sugar.io.publisher.Publisher.publish
```

````

`````
