# Events

Events are created to alert a robot software stack to any dynamic change at runtime. An Event is defined by a change in a ROS2 message value on a specific topic.

Events are used by matching them to 'Actions'; an Action is meant to be executed at runtime once the Event is triggered.

## Available Events:
- [OnEqual](#onequal-event)
- [OnDifferent](#ondifferent-event)
- [OnChange](#onchange-event)
- [OnChangeEqual](#onchangeequal-event)
- [OnGreater](#ongreater-event)
- [OnLess](#onless-event)

## OnEqual Event

OnEqual Event is triggered when a given topic attribute value is equal to a given trigger value.

*Example usage scenario:*
- Event when the detection id (object type) in an object detection topic is equal to a specific object (to raise an event on detecting another robot, a human, etc.)

```python
from ros_sugar.events import OnEqual
from ros_sugar.io import Topic
from vision_msgs.msg import Detection2D

example_person_id_from_db : int = 3

# Raise event as long as a person is detected
person_detected = OnEqual(
    "person_detected",
    Topic(name="/person_detection", msg_type=Detection2D),
    example_person_id_from_db,
    ("results", "id"),
)
```

## OnDifferent Event
OnDifferent Event is triggered when a given topic attribute value is different from a given trigger value.


## OnChange Event
OnChange Event is triggered when a given topic attribute changes in value from any initial value to any new value. The target attribute value is registered on the first recept of a message on the target topic, then the event is triggered on a change in that value. After a change the new value is registered and the event is triggered again on any new change, ...etc.

*Example usage scenario:*
- Event on a change in the number of detected people of the robot by a vision system to play a friendly message.

```python
from ros_sugar.events import OnChange
from ros_sugar.io import Topic

# Raise event when the number of detected people change
number_people_change = OnChange(
    "number_people_change",
    Topic(name="/people_count", msg_type="Int"),
)
```

## OnChangeEqual Event
OnChangeEqual Event is a combination of OnChange and OnEqual events. OnChangeEqual is triggered when a given topic attribute changes in value from any initial value to *given trigger goal* value.

:::{note} The difference between using OnChangeEqual as opposite to OnEqual or OnChange is that:
- OnEqual will keep getting triggered every time a new message value is received that is equal to the trigger.
- OnChange will keep getting triggered every time a new message value is received that is different from a previous value
- OnChangeEqual will get triggered once when the topic message value reaches the trigger, making it convenient for many applications
:::

*Example usage scenarios:*
- Event on the robot reaching a navigation goal point: reach_end Boolean topic OnChangeEqual to True (triggered once when reaching, does not trigger again if the robot is static and staying in 'goal reaching' state)
- Event on an Enum value of a message attribute to detect reaching a given state.
- Event of reaching 100% charge level of a robot to end charging.


```python
from ros_sugar.events import OnChangeEqual
from ros_sugar.io import Topic

# Raise event when end is reached
reached_end = OnChangeEqual(
    "reached_end",
    Topic(name="/reach_end", msg_type="Bool"),
    True,
    ("data")
)
```

## OnGreater Event

OnGreater Event is triggered when a given topic attribute value is greater than a given trigger value.

*Example usage scenario:*
- Event when a drone is higher than a certain allowed elevation (location z coordinate > elevation level), to bring the drone down into allowed limits.

```python
from ros_sugar.events import OnGreater
from ros_sugar.io import Topic

# Raise event when elevation is more than 100 meters
crossed_100m_elevation = OnGreater(
    "crossed_elevation",
    Topic(name="/odom", msg_type="Odometry"),
    100.0,
    ("pose", "pose", "position", "z")
)
```

## OnLess Event

OnLess Event is triggered when a given topic attribute value is less than a given trigger value.

*Example usage scenario:*
- Event when the robot battery level falls under a certain low limit, to go back to the charging station, for example.

```python
from ros_sugar.events import OnLess
from ros_sugar.io import Topic

# Raise event when battery is low
low_battery = OnLess(
    "low_battery",
    Topic(name="/battery_level", msg_type="Int"),
    15,
    ("data")
)
```
