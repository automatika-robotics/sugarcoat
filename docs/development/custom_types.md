# Extending the Type System

Sugarcoat uses a type system built on `SupportedType` to bridge ROS 2 message types with Python-native data. This document explains how the system works, what a type can opt into (UI streaming mode, the shared-memory fast path), and how to extend it with custom types.

## SupportedType Base Class

Every supported message type is a subclass of `ros_sugar.io.supported_types.SupportedType`. The base class defines these extension points:

```python
class SupportedType:
    # The ROS 2 message class (e.g., std_msgs.msg.String)
    _ros_type: type

    # Callback class that turns incoming messages into Python data
    callback = callbacks.GenericCallback

    # Whether UI/API clients receive this stream rate-sampled (True)
    # or pushed per message (False)
    _ui_rate_sampled: bool = False

    @classmethod
    def convert(cls, output, **_) -> Any:
        """Convert Python data into a ROS message instance."""

    @classmethod
    def get_ros_type(cls) -> type:
        """Return the underlying ROS 2 message class."""

    @classmethod
    def to_shm_payload(cls, msg) -> Optional[Tuple[Dict, memoryview]]:
        """Zero-copy hand-off for the shared-memory fast path. Default: None."""

    @classmethod
    def from_shm_payload(cls, meta: Dict, buffer: bytes) -> Any:
        """Rebuild the message from `meta` and a copy of its buffer."""
```

### _ros_type

Class attribute holding the ROS 2 message class. It is used to create subscriptions and publishers, validate topic compatibility, and generate UI schemas.

### callback

A `GenericCallback` subclass that turns each incoming ROS message into the Python value a component reads through `self.callbacks[name].get_output()`. Different types use specialized callbacks: `ImageCallback` returns a numpy array, `OdomCallback` a position/heading/speed array, `LaserScanCallback` a `LaserScanData` container, `StdMsgCallback` the bare `data` field. See [Custom Callbacks](#custom-callbacks) for the contract.

### convert

A classmethod that takes Python-native data and returns a ROS message instance. The first positional argument must be named `output`. `Publisher.publish()` calls it and then stamps the message header (frame and time) itself, so a converter does not need to fill headers on the publish path.

Converters may accept extra keyword arguments, which reach them from `publish(output, **kwargs)` or from direct calls. Two built-in examples:

- `Image.convert(array, encoding=None, stamp=None, frame_id="")` builds a complete `sensor_msgs/Image` from an `(H, W)` or `(H, W, C)` array. `encoding` is inferred from the dtype and channel count when not given (`mono8`, `rgb8`, `rgba8`, or the CvMat form such as `16UC1`), and `step` and `is_bigendian` are filled in. A decoder producing another layout, such as BGR from OpenCV, must pass `encoding="bgr8"`.
- `CameraInfo.convert(intrinsics, stamp=None, frame_id="")` builds a `sensor_msgs/CameraInfo` from a `CameraIntrinsics`, the inverse of `read_camera_info`.

`stamp` (seconds) and `frame_id` exist for callers building messages outside a publisher, such as a robot plugin decoder.

### _ui_rate_sampled

How the web UI and the JSON/WebSocket API stream an output of this type to clients:

- `False` (default): every message is pushed as it arrives. Lossless, and cheap for low-rate data such as text, poses or status.
- `True`: the latest value is sampled at a rate (`enable_ui(api_stream_default_rate=...)`, capped by `api_max_stream_rate`). Use it for continuous high-bandwidth streams where a client never wants every raw frame.

The built-in types set it on `Image`, `CompressedImage`, `LaserScan`, `PointCloud2` and `OccupancyGrid`. A derived package should set it on its own heavy streaming types. A client can override the default per connection with `?rate=<hz>` (`?rate=0` forces push), and `GET /api/interfaces` reports the default of each output as `"mode": "sampled"` or `"push"`.

### to_shm_payload / from_shm_payload

Under multiprocess launch, feedback decoded by a robot plugin HOST crosses into component processes over a socket bus, CDR-serialized. For large messages that serialization is the dominant cost, so a type can opt into a shared-memory ring instead: the HOST writes the raw buffer into shared memory and only a small descriptor crosses the socket.

- `to_shm_payload(msg)` returns `(meta, view)`: `view` is a zero-copy `memoryview` of the message's dominant buffer and `meta` a msgpack-serializable dict carrying everything else needed to rebuild the message. Return `None` (the default) when the type is not a good fit.
- `from_shm_payload(meta, buffer)` rebuilds the message from `meta` and a `bytes` copy of the buffer.

The fast path is taken only when a payload is at least `ros_sugar.robot.plugin.SHM_MIN_BYTES` (32 KiB). Smaller messages, types that return `None`, and any shared-memory failure fall back to CDR transparently. `Image`, `CompressedImage` and `PointCloud2` implement the hooks; the `Image` implementation shows the shape:

```python
@classmethod
def to_shm_payload(cls, msg: ROSImage):
    meta = {
        "h": msg.height, "w": msg.width, "e": msg.encoding,
        "b": int(msg.is_bigendian), "st": msg.step,
        "s": msg.header.stamp.sec, "ns": msg.header.stamp.nanosec,
        "f": msg.header.frame_id,
    }
    return meta, memoryview(msg.data)

@classmethod
def from_shm_payload(cls, meta, buffer: bytes) -> ROSImage:
    msg = ROSImage()
    msg.header.stamp.sec = meta["s"]
    msg.header.stamp.nanosec = meta["ns"]
    msg.header.frame_id = meta["f"]
    msg.height, msg.width = meta["h"], meta["w"]
    msg.encoding, msg.step = meta["e"], meta["st"]
    msg.is_bigendian = bool(meta["b"])
    msg.data = bytes_to_array(buffer)
    return msg
```

See {doc}`custom_robot_plugin` for where this sits in the plugin data path.

## Building Messages Efficiently

The Python message classes generated by rosidl take an `array.array` of the field's typecode for a sequence field as is. Anything else (a list, `bytes`, a numpy array) is walked element by element in Python, which dominates the cost of building an image or a point cloud. Use `ros_sugar.io.utils.bytes_to_array(buffer, typecode="B")` for large fields:

```python
import numpy as np
from ros_sugar.io.utils import bytes_to_array

msg.data = bytes_to_array(frame.tobytes())                        # uint8[] field
grid.data = bytes_to_array(cells.astype(np.int8).tobytes(), "b")  # int8[] field
```

`numpy_to_multiarray` does the same for the `*MultiArray` types. Two related rules:

- Assign the declared Python type to scalar fields. ROS 2 Humble's generated setters check field types on every assignment (`is_bigendian = 0` on a `bool` field raises an `AssertionError` there), while newer distributions only check when `ROS_PYTHON_CHECK_FIELDS=1` is set. Run the tests with that variable to catch such mistakes before CI does.
- Leave header stamping to the publisher on the publish path, as described under `convert`.

## Built-in Types

Sugarcoat ships with the following built-in types in `ros_sugar.io.supported_types`. "Sampled" marks types whose `_ui_rate_sampled` is `True`; "SHM" marks types implementing the shared-memory hooks.

| Type | ROS Message | Callback | Sampled | SHM |
|:-----|:------------|:---------|:-------:|:---:|
| `String` | `std_msgs/String` | `TextCallback` | | |
| `Bool` | `std_msgs/Bool` | `StdMsgCallback` | | |
| `Float32` | `std_msgs/Float32` | `StdMsgCallback` | | |
| `Float64` | `std_msgs/Float64` | `StdMsgCallback` | | |
| `Float32MultiArray` | `std_msgs/Float32MultiArray` | `StdMsgArrayCallback` | | |
| `Float64MultiArray` | `std_msgs/Float64MultiArray` | `StdMsgArrayCallback` | | |
| `Audio` | `std_msgs/ByteMultiArray` | `AudioCallback` | | |
| `Image` | `sensor_msgs/Image` | `ImageCallback` | yes | yes |
| `CompressedImage` | `sensor_msgs/CompressedImage` | `CompressedImageCallback` | yes | yes |
| `CameraInfo` | `sensor_msgs/CameraInfo` | `CameraInfoCallback` | | |
| `LaserScan` | `sensor_msgs/LaserScan` | `LaserScanCallback` | yes | |
| `PointCloud2` | `sensor_msgs/PointCloud2` | `PointCloudCallback` | yes | yes |
| `Imu` | `sensor_msgs/Imu` | `ImuCallback` | | |
| `JointState` | `sensor_msgs/JointState` | `JointStateCallback` | | |
| `NavSatFix` | `sensor_msgs/NavSatFix` | `NavSatFixCallback` | | |
| `Range` | `sensor_msgs/Range` | `RangeCallback` | | |
| `Odometry` | `nav_msgs/Odometry` | `OdomCallback` | | |
| `Path` | `nav_msgs/Path` | `PathCallback` | | |
| `OccupancyGrid` | `nav_msgs/OccupancyGrid` | `OccupancyGridCallback` | yes | |
| `MapMetaData` | `nav_msgs/MapMetaData` | `MapMetaDataCallback` | | |
| `Point` | `geometry_msgs/Point` | `PointCallback` | | |
| `PointStamped` | `geometry_msgs/PointStamped` | `PointStampedCallback` | | |
| `Pose` | `geometry_msgs/Pose` | `PoseCallback` | | |
| `PoseStamped` | `geometry_msgs/PoseStamped` | `PoseStampedCallback` | | |
| `PoseArray` | `geometry_msgs/PoseArray` | `PoseArrayCallback` | | |
| `Twist` | `geometry_msgs/Twist` | `GenericCallback` | | |
| `ComponentStatus` | `automatika_ros_sugar/ComponentStatus` | `GenericCallback` | | |

The sensor callbacks return the containers in `ros_sugar.io.datatypes`: `LaserScanCallback` a `LaserScanData` (ranges and angles as float32 arrays), `PointCloudCallback` a `PointCloudData` (the raw point buffer and its layout, with `xyz` decoded lazily on first use), `CameraInfoCallback` a `CameraIntrinsics`. The spatial callbacks (scan, cloud, grid, odometry, path, poses and points) return their data in the frame the component asked for; see the frames section of {doc}`custom_component`.

## Registering Additional Types

Use `add_additional_datatypes()` to register custom types at runtime:

```python
from ros_sugar.io.supported_types import add_additional_datatypes

add_additional_datatypes([MyCustomType, AnotherType])
```

The function maintains a global `_additional_types` dictionary keyed by the type's module and class name. When a type with the same class name is already registered, the function merges callbacks and conversion functions into the existing entry rather than replacing it (see [Merging Behavior](#merging-behavior)). Once registered, a type can be named in a `Topic` either by class or by its name as a string: `Topic(name="/t", msg_type="MyCustomType")`.

Under multiprocess launch, the Launcher passes the modules of all registered types to every component process, which imports them before rebuilding its topics from the launch arguments. Registering at import time of your package, as shown below, is what makes that work.

## Adding a Custom Type

There are two ways to add a type. The quick way covers the common case of wrapping a message with a decoder and an encoder; subclassing gives access to everything else a type can declare.

### The quick way: `create_supported_type`

`create_supported_type(ros_msg_type, converter=None, callback=None)` builds and registers a `SupportedType` subclass from two annotated functions:

```python
from sensor_msgs.msg import Temperature as ROSTemperature
from ros_sugar import create_supported_type


def _temperature_callback(msg: ROSTemperature) -> float:
    return msg.temperature


def _temperature_converter(output: float) -> ROSTemperature:
    msg = ROSTemperature()
    msg.temperature = float(output)
    return msg


Temperature = create_supported_type(
    ROSTemperature,
    callback=_temperature_callback,
    converter=_temperature_converter,
)
```

The annotations are checked: the callback's first parameter must be annotated with the ROS message type and its return with a non-ROS Python type; the converter's return annotation must be the ROS message type. Either function may be omitted. The type is registered under the calling module and can be used right away:

```python
from ros_sugar.io import Topic

temperature_topic = Topic(name="/temperature", msg_type=Temperature)
```

This is the form robot plugins use to wrap a manufacturer's custom messages; see {doc}`custom_robot_plugin`.

### The full way: subclass `SupportedType`

Subclass when the type needs a custom UI payload, a streaming mode, the shared-memory hooks, or frame handling:

```python
from typing import Optional

from sensor_msgs.msg import Temperature as ROSTemperature

from ros_sugar.io.callbacks import GenericCallback
from ros_sugar.io.supported_types import SupportedType, add_additional_datatypes


class TemperatureCallback(GenericCallback):
    def _get_output(self, **_) -> Optional[float]:
        if self.msg is None:
            return None
        return self.msg.temperature

    def _get_ui_content(self, **_):
        # JSON-serializable content for the web UI and the API
        return {"celsius": self._get_output()}


class Temperature(SupportedType):
    _ros_type = ROSTemperature
    callback = TemperatureCallback

    @classmethod
    def convert(cls, output: float, **_) -> ROSTemperature:
        msg = ROSTemperature()
        msg.temperature = float(output)
        return msg


add_additional_datatypes([Temperature])
```

## Custom Callbacks

A callback class inherits from `GenericCallback` and is constructed by the component with the input `Topic`. The contract:

- `callback(msg)` is the ROS subscription callback. It stores the message in `self.msg`, records the message frame in `self.frame_id`, refreshes `self.transformation`, and fires any attached extra callback. Do not override it to compute outputs; if you need per-message work, override it and call `super().callback(msg)` first.
- `_get_output(**kwargs)` returns the Python value for the component, computed from `self.msg`. Return `None` when no message has arrived. Keyword arguments are whatever the component passes to `get_output(**kwargs)`; the built-in `OccupancyGridCallback`, for example, accepts `get_obstacles` and `get_three_d`.
- `get_output(**kwargs)` is what components call. It runs `_get_output` and then any post-processors attached with `add_post_processors` (see {doc}`custom_processing`). Do not override it.
- `_get_ui_content(**_)` returns JSON-serializable content (a `str` or a `dict`) for the web UI and the API. The default returns `get_output()`, which is fine for scalars and strings. Heavy types return a summary; the scan and cloud callbacks send metadata only.
- `self.frame_id` and `self.transformation` support spatial data. When a component asked for this input in a frame (`transform_input_to`), `self.transformation` holds the `TransformStamped` from the message frame to that frame, or `None` while it is unresolved. A callback for spatial data applies it in `_get_output`.
- `self.got_msg` is `True` once a message has been received; `got_all_inputs()` on the component is built on it.

When two packages register the same type name with different callbacks, the merged type carries a list of callback classes, and a component picks the one defined in its own package.

## How Derived Packages Register Types

Packages built on Sugarcoat (such as Kompass or EmbodiedAgents) register their own types by calling `add_additional_datatypes()` at import time. For example, a navigation package might add:

```python
# In my_nav_package/__init__.py
from ros_sugar.io.supported_types import add_additional_datatypes
from .types import CostMap, Waypoint, TrajectoryArray

add_additional_datatypes([CostMap, Waypoint, TrajectoryArray])
```

This ensures that when any component from `my_nav_package` is imported, the types are immediately available for topic wiring and event conditions.

### Merging Behavior

If two packages register a type with the same class name, `add_additional_datatypes()` merges them:

- **callback**: If the existing type has no callback, the new one is used. If both have callbacks, they are combined into a list.
- **_ros_type**: Only set if the existing type has no `_ros_type`.
- **convert**: Merged using the same list-accumulation logic as callbacks.

This allows, for example, one package to define the `_ros_type` and another to supply a specialized `convert` function for the same message type.
