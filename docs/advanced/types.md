# Supported ROS2 Messages

Sugarcoat components create automatic subscribers and callbacks to all inputs and publishers to all outputs.

```{tip}
Access all callbacks in a `BaseComponent` in `self.callbacks: Dict[str, GenericCallback]` and get the topic incoming message using `get_ouput` method in the [`GenericCallback`](../apidocs/ros_sugar/ros_sugar.io.callbacks.md#classes) class
```

```{tip}
Access all publishers in a `BaseComponent` in `self.publishers_dict: Dict[str, Publisher]` and publish a new message to the topic using `publish` method in the [`Publisher`](../apidocs/ros_sugar/ros_sugar.io.publisher.md#classes) class
```

Many supported message types in Sugarcoat come with pre-defined callback and publisher classes to convert ROS2 messages to Python types. Below is a list of supported messages and the types accepted by their publishers `publish` method and returned by their callback `get_output` method:

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Message
  - ROS2 package
  - Callback return type
  - Publisher converts from

* - **[String](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `str`
  - `str`

* - **[Bool](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `bool`
  - `bool`

* - **[Float32](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `float`
  - `float`

* - **[Float32MultiArray](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `numpy.ndarray[float]`
  - `numpy.ndarray[float]`

* - **[Float64](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `float`
  - `float`

* - **[Float64MultiArray](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - std_msgs
  - `numpy.ndarray[float]`
  - `numpy.ndarray[float]`

* - **[Point](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - geometry_msgs
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[PointStamped](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - geometry_msgs
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[Pose](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - geometry_msgs
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[PoseStamped](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - geometry_msgs
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[Twist](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - geometry_msgs
  - `geometry_msgs.msg.Twist`
  - `geometry_msgs.msg.Twist`

* - **[Image](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - sensor_msgs
  - `numpy.ndarray`
  - `sensor_msgs.msg.Image | numpy.ndarray`


* - **[CompressedImage](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - sensor_msgs
  - `numpy.ndarray`
  - `sensor_msgs.msg.CompressedImage | numpy.ndarray`

* - **[Audio](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - sensor_msgs
  - `bytes`
  - `str | bytes`

* - **[Odometry](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - nav_msgs
  - `numpy.ndarray`
  - `nav_msgs.msg.Odometry`

* - **[LaserScan](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - sensor_msgs
  - `sensor_msgs.msg.LaserScan`
  - `sensor_msgs.msg.LaserScan`

* - **[Path](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - nav_msgs
  - `nav_msgs.msg.Path`
  - `nav_msgs.msg.Path`

* - **[MapMetaData](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - nav_msgs
  - `Dict`
  - `nav_msgs.msg.MapMetaData`

* - **[OccupancyGrid](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - nav_msgs
  - `nav_msgs.msg.OccupancyGrid | np.ndarray | Dict`
  - `numpy.ndarray`

* - **[ComponentStatus](../apidocs/ros_sugar/ros_sugar.io.supported_types.md#classes)**
  - ros_sugar_interfaces
  - `ros_sugar_interfaces.msg.ComponentStatus`
  - `ros_sugar_interfaces.msg.ComponentStatus`

```
