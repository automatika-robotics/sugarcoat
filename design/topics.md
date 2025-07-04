# Topics

Sugarcoat provides classes to configure a ROS2 topics as a Component Input/Output.

## Topic Configuration Class

- Configured using:

1. name: [str], ROS2 topic name.

2. msg_type: [Union[ros_sugar.supported_types.SupportedType, str]], ROS2 message type, passed as a string or as a type.

3. qos_profile: [QoSConfig](../apidocs/ros_sugar/ros_sugar.config.base_config.md#classes), See usage in example below.

- Provides:

ros_msg_type: [type], Provides the ROS2 message type of the topic.

## Usage Example

```python
from ros_sugar.config import Topic, QoSConfig

qos_conf = QoSConfig(
    history=qos.HistoryPolicy.KEEP_LAST,
    queue_size=20,
    reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    durability=qos.DurabilityPolicy.TRANSIENT_LOCAL
)

topic = Topic(name='/local_map', msg_type='OccupancyGrid', qos_profile=qos_conf)
```
