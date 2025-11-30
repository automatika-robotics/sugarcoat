---
orphan: true
---

# {py:mod}`ros_sugar.robot_plugin`

```{py:module} ros_sugar.robot_plugin
```

```{autodoc2-docstring} ros_sugar.robot_plugin
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`RobotPluginServiceClient <ros_sugar.robot_plugin.RobotPluginServiceClient>`
  - ```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`create_supported_type <ros_sugar.robot_plugin.create_supported_type>`
  - ```{autodoc2-docstring} ros_sugar.robot_plugin.create_supported_type
    :summary:
    ```
````

### API

````{py:function} create_supported_type(ros_msg_type: type, converter: typing.Optional[typing.Callable] = None, callback: typing.Optional[typing.Callable] = None) -> type[ros_sugar.io.supported_types.SupportedType]
:canonical: ros_sugar.robot_plugin.create_supported_type

```{autodoc2-docstring} ros_sugar.robot_plugin.create_supported_type
```
````

`````{py:class} RobotPluginServiceClient(client_node: rclpy.node.Node, srv_name: str = 'robot_serice_name', srv_type: typing.Optional[type] = None)
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient

Bases: {py:obj}`ros_sugar.base_clients.ServiceClientHandler`

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient
```

````{py:method} replace_publisher(current_publisher: ros_sugar.io.publisher.Publisher) -> bool
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.replace_publisher

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.replace_publisher
```

````

````{py:property} name
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.name
:type: str

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.name
```

````

````{py:property} msg_type
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.msg_type
:type: str

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.msg_type
```

````

````{py:method} publish(*args, **kwargs) -> bool
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.publish

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.publish
```

````

````{py:method} start(*_) -> bool
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.start
:abstractmethod:

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.start
```

````

````{py:method} end(*_) -> bool
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.end
:abstractmethod:

```{autodoc2-docstring} ros_sugar.robot_plugin.RobotPluginServiceClient.end
```

````

````{py:method} send_request(req_msg, executor: typing.Optional[rclpy.executors.Executor] = None)
:canonical: ros_sugar.robot_plugin.RobotPluginServiceClient.send_request

````

`````
