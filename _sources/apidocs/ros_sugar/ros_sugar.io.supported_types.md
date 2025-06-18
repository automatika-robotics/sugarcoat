# {py:mod}`ros_sugar.io.supported_types`

```{py:module} ros_sugar.io.supported_types
```

```{autodoc2-docstring} ros_sugar.io.supported_types
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Meta <ros_sugar.io.supported_types.Meta>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Meta
    :summary:
    ```
* - {py:obj}`SupportedType <ros_sugar.io.supported_types.SupportedType>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.SupportedType
    :summary:
    ```
* - {py:obj}`String <ros_sugar.io.supported_types.String>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.String
    :summary:
    ```
* - {py:obj}`Bool <ros_sugar.io.supported_types.Bool>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Bool
    :summary:
    ```
* - {py:obj}`Float32 <ros_sugar.io.supported_types.Float32>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Float32
    :summary:
    ```
* - {py:obj}`Float32MultiArray <ros_sugar.io.supported_types.Float32MultiArray>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Float32MultiArray
    :summary:
    ```
* - {py:obj}`Float64 <ros_sugar.io.supported_types.Float64>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Float64
    :summary:
    ```
* - {py:obj}`Float64MultiArray <ros_sugar.io.supported_types.Float64MultiArray>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Float64MultiArray
    :summary:
    ```
* - {py:obj}`Image <ros_sugar.io.supported_types.Image>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Image
    :summary:
    ```
* - {py:obj}`CompressedImage <ros_sugar.io.supported_types.CompressedImage>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.CompressedImage
    :summary:
    ```
* - {py:obj}`Audio <ros_sugar.io.supported_types.Audio>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Audio
    :summary:
    ```
* - {py:obj}`MapMetaData <ros_sugar.io.supported_types.MapMetaData>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.MapMetaData
    :summary:
    ```
* - {py:obj}`Odometry <ros_sugar.io.supported_types.Odometry>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Odometry
    :summary:
    ```
* - {py:obj}`LaserScan <ros_sugar.io.supported_types.LaserScan>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.LaserScan
    :summary:
    ```
* - {py:obj}`Path <ros_sugar.io.supported_types.Path>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Path
    :summary:
    ```
* - {py:obj}`OccupancyGrid <ros_sugar.io.supported_types.OccupancyGrid>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.OccupancyGrid
    :summary:
    ```
* - {py:obj}`Point <ros_sugar.io.supported_types.Point>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Point
    :summary:
    ```
* - {py:obj}`PointStamped <ros_sugar.io.supported_types.PointStamped>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.PointStamped
    :summary:
    ```
* - {py:obj}`Pose <ros_sugar.io.supported_types.Pose>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Pose
    :summary:
    ```
* - {py:obj}`PoseStamped <ros_sugar.io.supported_types.PoseStamped>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.PoseStamped
    :summary:
    ```
* - {py:obj}`ComponentStatus <ros_sugar.io.supported_types.ComponentStatus>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.ComponentStatus
    :summary:
    ```
* - {py:obj}`Twist <ros_sugar.io.supported_types.Twist>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.Twist
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`add_additional_datatypes <ros_sugar.io.supported_types.add_additional_datatypes>`
  - ```{autodoc2-docstring} ros_sugar.io.supported_types.add_additional_datatypes
    :summary:
    ```
````

### API

````{py:function} add_additional_datatypes(types: typing.List[type]) -> None
:canonical: ros_sugar.io.supported_types.add_additional_datatypes

```{autodoc2-docstring} ros_sugar.io.supported_types.add_additional_datatypes
```
````

````{py:class} Meta
:canonical: ros_sugar.io.supported_types.Meta

Bases: {py:obj}`type`

```{autodoc2-docstring} ros_sugar.io.supported_types.Meta
```

````

`````{py:class} SupportedType
:canonical: ros_sugar.io.supported_types.SupportedType

```{autodoc2-docstring} ros_sugar.io.supported_types.SupportedType
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.SupportedType.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.SupportedType.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.SupportedType.get_ros_type
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.SupportedType.get_ros_type
```

````

`````

`````{py:class} String
:canonical: ros_sugar.io.supported_types.String

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.String
```

````{py:method} convert(output: str, **_) -> std_msgs.msg.String
:canonical: ros_sugar.io.supported_types.String.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.String.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.String.get_ros_type
:classmethod:

````

`````

`````{py:class} Bool
:canonical: ros_sugar.io.supported_types.Bool

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Bool
```

````{py:method} convert(output: bool, **_) -> std_msgs.msg.Bool
:canonical: ros_sugar.io.supported_types.Bool.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Bool.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Bool.get_ros_type
:classmethod:

````

`````

`````{py:class} Float32
:canonical: ros_sugar.io.supported_types.Float32

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Float32
```

````{py:method} convert(output: float, **_) -> std_msgs.msg.Float32
:canonical: ros_sugar.io.supported_types.Float32.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Float32.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Float32.get_ros_type
:classmethod:

````

`````

`````{py:class} Float32MultiArray
:canonical: ros_sugar.io.supported_types.Float32MultiArray

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Float32MultiArray
```

````{py:method} convert(output: numpy.ndarray, **_) -> std_msgs.msg.Float32MultiArray
:canonical: ros_sugar.io.supported_types.Float32MultiArray.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Float32MultiArray.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Float32MultiArray.get_ros_type
:classmethod:

````

`````

`````{py:class} Float64
:canonical: ros_sugar.io.supported_types.Float64

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Float64
```

````{py:method} convert(output: float, **_) -> std_msgs.msg.Float64
:canonical: ros_sugar.io.supported_types.Float64.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Float64.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Float64.get_ros_type
:classmethod:

````

`````

`````{py:class} Float64MultiArray
:canonical: ros_sugar.io.supported_types.Float64MultiArray

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Float64MultiArray
```

````{py:method} convert(output: numpy.ndarray, **_) -> std_msgs.msg.Float64MultiArray
:canonical: ros_sugar.io.supported_types.Float64MultiArray.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Float64MultiArray.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Float64MultiArray.get_ros_type
:classmethod:

````

`````

`````{py:class} Image
:canonical: ros_sugar.io.supported_types.Image

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Image
```

````{py:method} convert(output: typing.Union[sensor_msgs.msg.Image, numpy.ndarray], **_) -> sensor_msgs.msg.Image
:canonical: ros_sugar.io.supported_types.Image.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Image.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Image.get_ros_type
:classmethod:

````

`````

`````{py:class} CompressedImage
:canonical: ros_sugar.io.supported_types.CompressedImage

Bases: {py:obj}`ros_sugar.io.supported_types.Image`

```{autodoc2-docstring} ros_sugar.io.supported_types.CompressedImage
```

````{py:method} convert(output: typing.Union[sensor_msgs.msg.CompressedImage, numpy.ndarray], **_) -> sensor_msgs.msg.CompressedImage
:canonical: ros_sugar.io.supported_types.CompressedImage.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.CompressedImage.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.CompressedImage.get_ros_type
:classmethod:

````

`````

`````{py:class} Audio
:canonical: ros_sugar.io.supported_types.Audio

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Audio
```

````{py:method} convert(output: typing.Union[str, bytes], **_) -> std_msgs.msg.ByteMultiArray
:canonical: ros_sugar.io.supported_types.Audio.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Audio.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Audio.get_ros_type
:classmethod:

````

`````

`````{py:class} MapMetaData
:canonical: ros_sugar.io.supported_types.MapMetaData

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.MapMetaData
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.MapMetaData.convert
:classmethod:

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.MapMetaData.get_ros_type
:classmethod:

````

`````

`````{py:class} Odometry
:canonical: ros_sugar.io.supported_types.Odometry

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Odometry
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.Odometry.convert
:classmethod:

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Odometry.get_ros_type
:classmethod:

````

`````

`````{py:class} LaserScan
:canonical: ros_sugar.io.supported_types.LaserScan

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.LaserScan
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.LaserScan.convert
:classmethod:

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.LaserScan.get_ros_type
:classmethod:

````

`````

`````{py:class} Path
:canonical: ros_sugar.io.supported_types.Path

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Path
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.Path.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Path.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Path.get_ros_type
:classmethod:

````

`````

`````{py:class} OccupancyGrid
:canonical: ros_sugar.io.supported_types.OccupancyGrid

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.OccupancyGrid
```

````{py:method} convert(output: numpy.ndarray, resolution: float, origin: typing.Optional[geometry_msgs.msg.Pose] = None, msg_header: typing.Optional[std_msgs.msg.Header] = None, **_) -> nav_msgs.msg.OccupancyGrid
:canonical: ros_sugar.io.supported_types.OccupancyGrid.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.OccupancyGrid.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.OccupancyGrid.get_ros_type
:classmethod:

````

`````

`````{py:class} Point
:canonical: ros_sugar.io.supported_types.Point

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Point
```

````{py:method} convert(output: numpy.ndarray, **_) -> geometry_msgs.msg.Point
:canonical: ros_sugar.io.supported_types.Point.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Point.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Point.get_ros_type
:classmethod:

````

`````

`````{py:class} PointStamped
:canonical: ros_sugar.io.supported_types.PointStamped

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.PointStamped
```

````{py:method} convert(output: numpy.ndarray, frame_id=None, ros_time=None, **_) -> geometry_msgs.msg.PointStamped
:canonical: ros_sugar.io.supported_types.PointStamped.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.PointStamped.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.PointStamped.get_ros_type
:classmethod:

````

`````

`````{py:class} Pose
:canonical: ros_sugar.io.supported_types.Pose

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Pose
```

````{py:method} convert(output: numpy.ndarray, frame_id=None, ros_time=None, **_) -> geometry_msgs.msg.Pose
:canonical: ros_sugar.io.supported_types.Pose.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.Pose.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Pose.get_ros_type
:classmethod:

````

`````

`````{py:class} PoseStamped
:canonical: ros_sugar.io.supported_types.PoseStamped

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.PoseStamped
```

````{py:method} convert(output: numpy.ndarray, frame_id=None, ros_time=None, **_) -> geometry_msgs.msg.PoseStamped
:canonical: ros_sugar.io.supported_types.PoseStamped.convert
:classmethod:

```{autodoc2-docstring} ros_sugar.io.supported_types.PoseStamped.convert
```

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.PoseStamped.get_ros_type
:classmethod:

````

`````

`````{py:class} ComponentStatus
:canonical: ros_sugar.io.supported_types.ComponentStatus

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.ComponentStatus
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.ComponentStatus.convert
:classmethod:

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.ComponentStatus.get_ros_type
:classmethod:

````

`````

`````{py:class} Twist
:canonical: ros_sugar.io.supported_types.Twist

Bases: {py:obj}`ros_sugar.io.supported_types.SupportedType`

```{autodoc2-docstring} ros_sugar.io.supported_types.Twist
```

````{py:method} convert(output, **_) -> typing.Any
:canonical: ros_sugar.io.supported_types.Twist.convert
:classmethod:

````

````{py:method} get_ros_type() -> type
:canonical: ros_sugar.io.supported_types.Twist.get_ros_type
:classmethod:

````

`````
