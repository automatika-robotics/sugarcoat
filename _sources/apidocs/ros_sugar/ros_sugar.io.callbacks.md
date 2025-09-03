# {py:mod}`ros_sugar.io.callbacks`

```{py:module} ros_sugar.io.callbacks
```

```{autodoc2-docstring} ros_sugar.io.callbacks
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`GenericCallback <ros_sugar.io.callbacks.GenericCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback
    :summary:
    ```
* - {py:obj}`StdMsgCallback <ros_sugar.io.callbacks.StdMsgCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.StdMsgCallback
    :summary:
    ```
* - {py:obj}`StdMsgArrayCallback <ros_sugar.io.callbacks.StdMsgArrayCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.StdMsgArrayCallback
    :summary:
    ```
* - {py:obj}`ImageCallback <ros_sugar.io.callbacks.ImageCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.ImageCallback
    :summary:
    ```
* - {py:obj}`CompressedImageCallback <ros_sugar.io.callbacks.CompressedImageCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.CompressedImageCallback
    :summary:
    ```
* - {py:obj}`TextCallback <ros_sugar.io.callbacks.TextCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.TextCallback
    :summary:
    ```
* - {py:obj}`AudioCallback <ros_sugar.io.callbacks.AudioCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.AudioCallback
    :summary:
    ```
* - {py:obj}`MapMetaDataCallback <ros_sugar.io.callbacks.MapMetaDataCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.MapMetaDataCallback
    :summary:
    ```
* - {py:obj}`OdomCallback <ros_sugar.io.callbacks.OdomCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.OdomCallback
    :summary:
    ```
* - {py:obj}`PointCallback <ros_sugar.io.callbacks.PointCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.PointCallback
    :summary:
    ```
* - {py:obj}`PointStampedCallback <ros_sugar.io.callbacks.PointStampedCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.PointStampedCallback
    :summary:
    ```
* - {py:obj}`PoseCallback <ros_sugar.io.callbacks.PoseCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.PoseCallback
    :summary:
    ```
* - {py:obj}`PoseStampedCallback <ros_sugar.io.callbacks.PoseStampedCallback>`
  - ```{autodoc2-docstring} ros_sugar.io.callbacks.PoseStampedCallback
    :summary:
    ```
* - {py:obj}`OccupancyGridCallback <ros_sugar.io.callbacks.OccupancyGridCallback>`
  -
````

### API

`````{py:class} GenericCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.GenericCallback

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.GenericCallback.frame_id
:type: typing.Optional[str]

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.frame_id
```

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.GenericCallback.set_node_name

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.set_node_name
```

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.GenericCallback.set_subscriber

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.set_subscriber
```

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.GenericCallback.on_callback_execute

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.on_callback_execute
```

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.GenericCallback.callback

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.callback
```

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.GenericCallback.add_post_processors

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.add_post_processors
```

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.GenericCallback.get_output

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.get_output
```

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.GenericCallback.got_msg

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.got_msg
```

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.GenericCallback.clear_last_msg

```{autodoc2-docstring} ros_sugar.io.callbacks.GenericCallback.clear_last_msg
```

````

`````

`````{py:class} StdMsgCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.StdMsgCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.StdMsgCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.StdMsgCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.StdMsgCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.StdMsgCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.StdMsgCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.StdMsgCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.StdMsgCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.StdMsgCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.StdMsgCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.StdMsgCallback.clear_last_msg

````

`````

`````{py:class} StdMsgArrayCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.StdMsgArrayCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.StdMsgArrayCallback.clear_last_msg

````

`````

`````{py:class} ImageCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.ImageCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.ImageCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.ImageCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.ImageCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.ImageCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.ImageCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.ImageCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.ImageCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.ImageCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.ImageCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.ImageCallback.clear_last_msg

````

`````

`````{py:class} CompressedImageCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.CompressedImageCallback

Bases: {py:obj}`ros_sugar.io.callbacks.ImageCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.CompressedImageCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.CompressedImageCallback.clear_last_msg

````

`````

`````{py:class} TextCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.TextCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.TextCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.TextCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.TextCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.TextCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.TextCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.TextCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.TextCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.TextCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.TextCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.TextCallback.clear_last_msg

````

`````

`````{py:class} AudioCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.AudioCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.AudioCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.AudioCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.AudioCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.AudioCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.AudioCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.AudioCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.AudioCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.AudioCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.AudioCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.AudioCallback.clear_last_msg

````

`````

`````{py:class} MapMetaDataCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.MapMetaDataCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.MapMetaDataCallback.clear_last_msg

````

`````

`````{py:class} OdomCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.OdomCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.OdomCallback
```

````{py:property} transformation
:canonical: ros_sugar.io.callbacks.OdomCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} ros_sugar.io.callbacks.OdomCallback.transformation
```

````

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.OdomCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.OdomCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.OdomCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.OdomCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.OdomCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.OdomCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.OdomCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.OdomCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.OdomCallback.clear_last_msg

````

`````

`````{py:class} PointCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.PointCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.PointCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.PointCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.PointCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.PointCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.PointCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.PointCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.PointCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.PointCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.PointCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.PointCallback.clear_last_msg

````

`````

`````{py:class} PointStampedCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.PointStampedCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.PointStampedCallback
```

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.PointStampedCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.PointStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.PointStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.PointStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.PointStampedCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.PointStampedCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.PointStampedCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.PointStampedCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.PointStampedCallback.clear_last_msg

````

`````

`````{py:class} PoseCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.PoseCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.PoseCallback
```

````{py:property} transformation
:canonical: ros_sugar.io.callbacks.PoseCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} ros_sugar.io.callbacks.PoseCallback.transformation
```

````

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.PoseCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.PoseCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.PoseCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.PoseCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.PoseCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.PoseCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.PoseCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.PoseCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.PoseCallback.clear_last_msg

````

`````

`````{py:class} PoseStampedCallback(input_topic, node_name: typing.Optional[str] = None)
:canonical: ros_sugar.io.callbacks.PoseStampedCallback

Bases: {py:obj}`ros_sugar.io.callbacks.PoseCallback`

```{autodoc2-docstring} ros_sugar.io.callbacks.PoseStampedCallback
```

````{py:property} transformation
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

````

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.PoseStampedCallback.clear_last_msg

````

`````

`````{py:class} OccupancyGridCallback(input_topic, node_name: typing.Optional[str] = None, to_numpy: typing.Optional[bool] = True, twoD_to_threeD_conversion_height: typing.Optional[float] = 0.01)
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback

Bases: {py:obj}`ros_sugar.io.callbacks.GenericCallback`

````{py:property} frame_id
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.frame_id
:type: typing.Optional[str]

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.callback

````

````{py:method} add_post_processors(processors: typing.List[typing.Union[typing.Callable, socket.socket]])
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.add_post_processors

````

````{py:method} get_output(clear_last: bool = False, **kwargs) -> typing.Any
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.get_output

````

````{py:property} got_msg
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.got_msg

````

````{py:method} clear_last_msg()
:canonical: ros_sugar.io.callbacks.OccupancyGridCallback.clear_last_msg

````

`````
