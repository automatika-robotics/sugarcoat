# Topics

In the Sugarcoat ecosystem, Topics act as the connective pipes that link your Components together and bridge them with the Event-Driven system.

Sugarcoat Topics are exposed in the Python API as configuration objects that defines the name (ROS2 topic name) and the Data Contract (Message Type) for communication, along with the possibility to set the QoS configuration of the topic.

This abstraction ensures that topic can:

- **Connect Components**: Topics serve as the interface between different parts of your system. Instead of hardcoding strings inside your nodes, you define your Topics as shared resources.

- **Connect Events (The Input Pipe)**: When defining an [Event](events.md), the Topic acts as the input pipe. The Event attaches to this pipe, listening for data flowing through it to trigger specific conditions.


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

## Advanced: Type Resolution

Because Topic objects are "smart," they handle the complexity of ROS2 message types for you. With the **dynamic resolution** of topics, you don't need to manually import message classes at the top of every file. You can define topics using string representations of the type, and Sugarcoat resolves them at runtime. See a list of supported messages [here](../advanced/types.md).

```python
from ros_sugar.config import Topic
from std_msgs.msg import String

# Method 1: import ROS2 message and pass it as the message type
topic_1 = Topic(name='/message', msg_type=String)

# Method 2: pass the message type as a string corresponding to the class name
topic_2 = Topic(name='/message', msg_type='String')
```

