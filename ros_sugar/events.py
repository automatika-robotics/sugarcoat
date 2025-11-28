"""Available Events"""

from typing import Union, Dict, List, Any
import json
from copy import deepcopy
from .io.topic import Topic
from .core.event import Event


def event_from_json(
    json_obj: Union[str, bytes, bytearray],
) -> Event:
    # Check if the serialized event contains a class name
    event_as_dict = json.loads(json_obj)
    if "event_class" not in event_as_dict.keys():
        raise ValueError(
            "Cannot convert json object to Events Dictionary. Json item is not a valid serialized Event"
        )

    # Get and check event class
    event_class_name: str = event_as_dict["event_class"]
    events_classes = [
        event.__name__
        for event in globals().values()
        if type(event) is type and issubclass(event, Event)
    ]
    if event_class_name not in events_classes:
        raise ValueError(
            f"Cannot convert json object to Event. Unknown event class '{event_class_name}'"
        )

    # Construct new event
    event_type = globals()[event_class_name]
    event: Event = event_type(
        event_as_dict["event_name"],
        event_as_dict,
        trigger_value=event_as_dict["trigger_ref_value"]
        if event_as_dict.get("trigger_ref_value")
        else None,
        nested_attributes=[],
    )
    return event


def json_to_events_list(
    json_obj: Union[str, bytes, bytearray],
) -> List:
    """
    Loads a list of events from a JSON object

    :param json_obj: JSON object containing a set of events
    :type json_obj: str | bytes | bytearray

    :raises ValueError: If the provided json object cannot be converted to an events list

    :return: Events list
    :rtype: List[Event]
    """
    list_obj = json.loads(json_obj)
    events_list = []
    for event_serialized in list_obj:
        new_event = event_from_json(event_serialized)
        events_list.append(
            deepcopy(new_event)
        )  # deepcopy is needed to avoid copying the previous event
    return events_list


class OnAny(Event):
    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **_,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=None,
            nested_attributes=[],
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
        )

    def callback(self, msg: Any) -> None:
        """
        Event topic listener callback

        :param msg: Event trigger topic message
        :type msg: Any
        """
        self._event_value = msg

        self.trigger = True

        # Process event on any incoming message
        if not self.under_processing:
            self.under_processing = True
            self._call_on_trigger(msg=msg)
            self.under_processing = False


class OnChange(Event):
    """
    OnChange Event is triggered when a given topic attribute changes in value from any initial value to any new value. The target attribute value is registered on the first recept of a message on the target topic, then the event is triggered on a change in that value. After a change the new value is registered and the event is triggered again on any new change, ...etc.

    ## Example usage scenario:
    - Event on a change in the number of detected people of the robot by a vision system to play a friendly welcome message.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=None,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs,
        )
        self._previous_event_value = None

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        if (
            self._previous_event_value is not None
            and self._event_value != self._previous_event_value
        ):
            self.trigger = True
        else:
            self.trigger = False


class OnChangeEqual(Event):
    """
    OnChangeEqual Event is a combination of OnChange and OnEqual events. OnChangeEqual is triggered when a given topic attribute changes in value from any initial value to <b>given trigger goal</b> value.

    :::{note} The difference between using OnChangeEqual as opposite to OnEqual or OnChange is that:
    - OnEqual will keep getting triggered every time a new message value is received that is equal to the trigger.
    - OnChange will keep getting triggered every time a new message value is received that is different from a previous value
    - OnChangeEqual will get triggered once when the topic message value reaches the trigger, making it convenient for many applications
    :::

    ## Some example usage scenarios:
    - Event on the robot reaching a navigation goal point: reach_end Boolean topic OnChangeEqual to True (triggered once when reaching, does not trigger again if the robot is static and staying in 'goal reaching' state)
    - Event on an Enum value of a message attribute to detect reaching a given state.
    - Event of reaching 100% charge level of a robot to end charging.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, List],
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """Init OnChangeEqual Event

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str, List]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs,
        )
        self._previous_event_value = None

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        if self._previous_event_value is not None:
            if (
                self._event_value != self._previous_event_value
                and self._event_value == self.trigger_ref_value
            ):
                self.trigger = True
            else:
                self.trigger = False
        else:
            self.trigger = False


class OnEqual(Event):
    """
    OnEqual Event is triggered when a given topic attribute value is equal to a given trigger value.

    ## Example usage scenario:
    - Event when the detection id (object type) in an object detection topic is equal to a specific object (to raise an event on detecting another robot, a human, etc.)
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, List],
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str, List]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        self.trigger = self._event_value == self.trigger_ref_value


class OnContainsAll(Event):
    """
    OnContainsAll Event is triggered when a given topic attribute value contains all of the given trigger list value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: List,
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: List
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value contains all of the reference values
        """
        if isinstance(self.trigger_ref_value, List):
            self.trigger = all(
                val in self._event_value for val in self.trigger_ref_value
            )
        else:
            self.trigger = self.trigger_ref_value in self._event_value


class OnChangeContainsAll(Event):
    """
    OnChangeContainsAll Event is a combination of OnChange and OnContainsAll events. OnChangeContainsAll is triggered when a given topic attribute contains a <b>given trigger goal</b> value after not containing it.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: List,
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: List
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs
        )
        self._previous_event_value = None

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if reference value is contained in the event value after not being contained in the previous event value
        """
        if self._previous_event_value is not None:
            if (
                isinstance(self.trigger_ref_value, List)
                and self._event_value != self._previous_event_value
                and all(val in self._event_value for val in self.trigger_ref_value)
            ):
                self.trigger = True
            else:
                self.trigger = False
        else:
            self.trigger = False


class OnChangeNotContain(Event):
    """
    OnChangeNotContain Event
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: List,
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: List
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs
        )
        self._previous_event_value = None

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if reference value is NOT contained in the event value after being contained in the previous event value
        """
        if self._previous_event_value is not None:
            if (
                isinstance(self.trigger_ref_value, List)
                and any(
                    val in self._previous_event_value for val in self.trigger_ref_value
                )
                and not any(val in self._event_value for val in self.trigger_ref_value)
            ):
                self.trigger = True
            else:
                self.trigger = False
        else:
            self.trigger = False


class OnContainsAny(Event):
    """
    OnContainsAny Event is triggered when a given topic attribute value contains one of the given trigger list value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: List,
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: List
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value contains any of the reference values
        """
        if isinstance(self.trigger_ref_value, List):
            self.trigger = any(
                val in self._event_value for val in self.trigger_ref_value
            )
        else:
            self.trigger = self.trigger_ref_value in self._event_value


class OnChangeContainsAny(Event):
    """
    OnChangeContainsAny Event is a combination of OnChange and OnContainsAny events. OnChangeContainsAny is triggered when a given topic attribute contains any <b>given trigger goal</b> value after not containing it.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: List,
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: List
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs,
        )
        self._previous_event_value = None

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if reference value is contained in the event value after not being contained in the previous event value
        """
        if self._previous_event_value is not None:
            if (
                isinstance(self.trigger_ref_value, List)
                and not any(
                    val in self._previous_event_value for val in self.trigger_ref_value
                )
                and any(val in self._event_value for val in self.trigger_ref_value)
            ):
                self.trigger = True
            else:
                self.trigger = False
        else:
            self.trigger = False


class OnDifferent(Event):
    """
    OnDifferent Event is triggered when a given topic attribute value is different from a given trigger value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, List],
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str, List]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs,
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is not equal to reference value
        """
        self.trigger = self._event_value != self.trigger_ref_value


class OnGreater(Event):
    """
    OnGreater Event is triggered when a given topic attribute value is greater than a given trigger value.

    ## Example usage scenario:
    - Event when a drone is higher than a certain allowed elevation (location z coordinate > elevation level), to bring the drone down into allowed limits.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str],
        nested_attributes: Union[str, List[str]],
        or_equal: bool = False,
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param or_equal: Trigger the event if greater or equal to the reference value, defaults to False
        :type or_equal: bool
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            **kwargs,
        )
        if isinstance(event_source, Topic):
            self._or_equal = or_equal

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is greater than reference value
        """
        if self._or_equal:
            self.trigger = self._event_value >= self.trigger_ref_value
        else:
            self.trigger = self._event_value > self.trigger_ref_value

    @property
    def dictionary(self) -> Dict:
        """
        Property to parse the event into a dictionary

        :return: Event description dictionary
        :rtype: Dict
        """
        event_dict = Event.dictionary.fget(self)
        event_dict["or_equal"] = self._or_equal
        return event_dict

    @dictionary.setter
    def dictionary(self, dict_obj: Dict) -> None:
        """
        Setter of the event using a dictionary

        :param dict_obj: Event description dictionary
        :type dict_obj: Dict
        """
        Event.dictionary.fset(self, dict_obj)
        self._or_equal = dict_obj["or_equal"]


class OnLess(OnGreater):
    """
    OnGreater Event is triggered when a given topic attribute value is less than a given trigger value.

    ## Example usage scenario:
    - Event when the robot battery level falls under a certain low limit, to go back to the charging station, for example.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str],
        nested_attributes: Union[str, List[str]],
        or_equal: bool = False,
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param or_equal: Trigger the event if less or equal to the reference value, defaults to False
        :type or_equal: bool
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional
        """
        super().__init__(
            event_name=event_name,
            event_source=event_source,
            trigger_value=trigger_value,
            nested_attributes=nested_attributes,
            handle_once=handle_once,
            keep_event_delay=keep_event_delay,
            or_equal=or_equal,
            **kwargs,
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is less than reference value
        """
        if self._or_equal:
            self.trigger = self._event_value <= self.trigger_ref_value
        else:
            self.trigger = self._event_value < self.trigger_ref_value
