"""Event"""

import json
import time
import logging
from abc import abstractmethod
from typing import Any, Callable, Dict, List, Union
from launch.event import Event as ROSLaunchEvent
from launch.event_handler import EventHandler as ROSLaunchEventHandler
import array
import numpy as np

from ..io.topic import Topic
from .action import Action
from ..utils import SomeEntitiesType


def _access_attribute(obj: Any, nested_attributes: List[str]):
    """
    Access nested attribute (specified by attrs) in a given object

    :param obj: Object
    :type obj: Any

    :raises AttributeError: If nested attribute does not exist in object

    :return: Nested attribute
    :rtype: Any
    """
    try:
        result = obj
        for attr in nested_attributes:
            result = getattr(result, attr)
        return result
    except AttributeError as e:
        raise AttributeError(f"Given attribute is not part of class {type(obj)}") from e


def _get_attribute_type(cls: Any, attrs: tuple):
    """
    Gets the type of a nested attribute (specified by attrs) in given object

    :param obj: Object
    :type obj: Any

    :raises AttributeError: If nested attribute does not exist in object

    :return: Type of nested attribute
    :rtype: Any
    """
    try:
        result = cls()
        for attr in attrs:
            result = getattr(result, str(attr))
        return type(result)
    except AttributeError as e:
        raise AttributeError(
            f"Given nested attributes '{attrs}' are not part of class {cls}"
        ) from e


def _check_attribute(cls, expected_type, attrs: tuple):
    """
    Checks if the given class has the nested attribute specified by attrs
    """
    try:
        current_cls = cls()
        for attr in attrs:
            if not hasattr(current_cls, attr):
                return False
            current_cls = getattr(current_cls, str(attr))
        # Handle the case of MultiArray data type
        if isinstance(current_cls, array.array) and (
            expected_type in [List, np.ndarray, list]
        ):
            return True
        return isinstance(current_cls, expected_type)
    except AttributeError:
        return False


class InternalEvent(ROSLaunchEvent):
    """
    Class to transform a Kompass event to ROS launch event using event key name
    """

    def __init__(self, event_name: str) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :rtype: None
        """
        super().__init__()
        self.__event_name = event_name

    @property
    def event_name(self):
        """
        Getter of internal event name

        :return: Event name
        :rtype: str
        """
        return self.__event_name


class OnInternalEvent(ROSLaunchEventHandler):
    """ROS EventHandler for InternalEvent."""

    def __init__(
        self,
        *,
        internal_event_name: str,
        entities: SomeEntitiesType,
        handle_once: bool = False,
    ) -> None:
        """__init__.

        :param internal_event_name:
        :type internal_event_name: str
        :param entities:
        :type entities: SomeEntitiesType
        :param handle_once:
        :type handle_once: bool
        :rtype: None
        """

        self.__matcher: Callable[[ROSLaunchEvent], bool] = lambda event: (
            isinstance(event, InternalEvent) and event.event_name == internal_event_name
        )

        super().__init__(
            matcher=self.__matcher, entities=entities, handle_once=handle_once
        )


class Operand:
    """
    Class to dynamically access nested attributes of an object and perform value Comparisons.
    """

    def __init__(self, ros_message: Any, attributes: List[str]) -> None:
        """

        :param ros_message: _description_
        :type ros_message: Any
        :param *attrs: Strings forming a chain of nested attribute accesses.
        :type *attrs: str
        """
        self._message = ros_message
        self._attrs = attributes
        # Get attribute from ros message
        self.value: Union[float, int, bool, str, List] = _access_attribute(
            ros_message, attributes
        )
        # Handle array case for all std MultiArray messages
        if isinstance(self.value, array.array):
            self.value = self.value.tolist()

        self.type_error_msg: str = "Cannot compare values of different types"

    def _check_similar_types(self, __value) -> None:
        """
        Checks for similar values types

        :param __value: Given value
        :type __value: Any

        :raises TypeError: If attributes are of different types
        """
        if type(self.value) is not type(__value):
            raise TypeError(
                f"{self.type_error_msg}: {type(self.value)} and {type(__value)}"
            )

    def _check_is_numerical_type(self) -> None:
        """
        Checks if the value is numerical

        :raises TypeError: If value is a not of type int or float
        """
        if type(self.value) not in [int, float]:
            raise TypeError(
                f"Unsupported operator for type {type(self.value)}. Supported operators are: [==, !=]"
            )

    def __contains__(self, __value: Union[float, int, str, bool, List]) -> bool:
        """If __value in self

        :param __value: _description_
        :type __value: List
        :return: Operand has value (or list of values)
        :rtype: bool
        """
        if isinstance(self.value, List):
            return __value in self.value
        if isinstance(__value, List):
            return any(a == self.value for a in __value)
        return self.value == __value

    def __eq__(self, __value: object) -> bool:
        """
        Operand == value

        :param __value: Comparison value
        :type __value: object

        :return: If Operand.value == __value
        :rtype: bool
        """
        self._check_similar_types(__value)
        return self.value == __value

    def __ne__(self, __value: object) -> bool:
        """
        Operand != value

        :param __value: Comparison value
        :type __value: Union[float, int, bool, str]

        :return: If Operand.value != __value
        :rtype: bool
        """
        self._check_similar_types(__value)
        return self.value is not __value

    def __lt__(self, __value: object) -> bool:
        """
        Operand < value

        :param __value: Comparison value
        :type __value: object

        :return: If Operand.value < __value
        :rtype: bool
        """
        self._check_is_numerical_type()
        self._check_similar_types(__value)
        return self.value < __value

    def __gt__(self, __value: object) -> bool:
        """
        Operand > value

        :param __value: Comparison value
        :type __value: object

        :return: If Operand.value == __value
        :rtype: bool
        """
        self._check_is_numerical_type()
        self._check_similar_types(__value)
        return self.value > __value

    def __le__(self, __value: object) -> bool:
        """
        Operand <= value

        :param __value: Comparison value
        :type __value: object

        :return: If Operand.value == __value
        :rtype: bool
        """
        self._check_is_numerical_type()
        self._check_similar_types(__value)
        return self.value <= __value

    def __ge__(self, __value: object) -> bool:
        """
        Operand >= value

        :param __value: Comparison value
        :type __value: object

        :return: If Operand.value == __value
        :rtype: bool
        """
        self._check_is_numerical_type()
        self._check_similar_types(__value)
        return self.value >= __value

    def __str__(self) -> str:
        """
        str(Operand)

        :rtype: str
        """
        return f"{type(self._message)}.{self._attrs}: {self.value}"


class Event:
    """An Event is defined by a change in a ROS2 message value on a specific topic. Events are created to alert a robot software stack to any dynamic change at runtime.

    Events are used by matching them to 'Actions'; an Action is meant to be executed at runtime once the Event is triggered.

    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, List, None],
        nested_attributes: Union[str, List[str]],
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
    ) -> None:
        """Creates an event

        :param event_name: Event key name
        :type event_name: str
        :param event_source: Event source configured using a Topic instance or a valid json/dict config
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value: Triggers event using this reference value
        :type trigger_value: Union[float, int, bool, str, List, None]
        :param nested_attributes: Attribute names to access within the event_source Topic
        :type nested_attributes: Union[str, List[str]]
        :param handle_once: Handle the event only once during the node lifetime, defaults to False
        :type handle_once: bool, optional
        :param keep_event_delay: Add a time delay between consecutive event handling instances, defaults to 0.0
        :type keep_event_delay: float, optional

        :raises AttributeError: If a non-valid event_source is provided

        :raises TypeError: If the provided nested_attributes cannot be accessed in the Topic message type
        """
        self.__name = event_name
        self._handle_once: bool = handle_once
        self._keep_event_delay: float = keep_event_delay
        # Init the event from the json values
        if isinstance(event_source, str):
            self.json = event_source

        # Init from dictionary values
        elif isinstance(event_source, Dict):
            self.dictionary = event_source

        elif isinstance(event_source, Topic):
            self.event_topic = event_source
            if nested_attributes is not None:
                self._attrs: List[str] = (
                    nested_attributes
                    if isinstance(nested_attributes, List)
                    else [nested_attributes]
                )
            if trigger_value is not None:
                self.trigger_ref_value = trigger_value

        else:
            raise AttributeError(
                "Cannot initialize Event class. Must provide 'event_source' as a Topic or a valid config from json or dictionary"
            )

        # Check if given trigger is of valid type
        if trigger_value is not None and not _check_attribute(
            self.event_topic.msg_type.get_ros_type(),
            type(self.trigger_ref_value),
            self._attrs,
        ):
            raise TypeError(
                f"Event Initialization error. Cannot initiate nested attribute '{self._attrs}' for class '{self.event_topic.msg_type.get_ros_type()}' with trigger of type '{type(trigger_value)}'. Should be '{_get_attribute_type(self.event_topic.msg_type.get_ros_type(), self._attrs)}'"
            )

        # Init trigger as False
        self.trigger: bool = False

        # Register for on trigger methods
        self._registered_on_trigger_methods: Dict[str, Callable[..., Any]] = {}

        # Register for on trigger actions
        self._registered_on_trigger_actions: List[Action] = []

        self.__under_processing = False

        self._processed_once: bool = False

    @property
    def under_processing(self) -> bool:
        """If event is triggered and associated action is getting executed

        :return: Event under processing flag
        :rtype: bool
        """
        return self.__under_processing

    @under_processing.setter
    def under_processing(self, value: bool) -> None:
        """If event is triggered and associated action is getting executed

        :param value: Event under processing flag
        :type value: bool
        """
        self.__under_processing = value

    def reset(self):
        """Reset event processing"""
        self._processed_once = False
        self.under_processing = False
        self.trigger = False

    @property
    def name(self) -> str:
        """
        Getter of event name

        :return: Name
        :rtype: str
        """
        return self.__name

    def clear(self) -> None:
        """
        Clear event trigger
        """
        self.trigger = False

    def trig(self) -> None:
        """
        Raise event trigger
        """
        self.trigger = True

    @property
    def dictionary(self) -> Dict:
        """
        Property to parse the event into a dictionary

        :return: Event description dictionary
        :rtype: Dict
        """
        event_dict = {
            "event_name": self.name,
            "event_class": self.__class__.__name__,
            "topic": self.event_topic.to_json(),
            "handle_once": self._handle_once,
            "event_delay": self._keep_event_delay,
        }
        if hasattr(self, "trigger_ref_value"):
            event_dict["trigger_ref_value"] = self.trigger_ref_value
        if hasattr(self, "_attrs"):
            event_dict["_attrs"] = self._attrs
        return event_dict

    @dictionary.setter
    def dictionary(self, dict_obj: Dict) -> None:
        """
        Setter of the event using a dictionary

        :param dict_obj: Event description dictionary
        :type dict_obj: Dict
        """
        try:
            self.__name = dict_obj["event_name"]
            if not hasattr(self, "event_topic"):
                self.event_topic = Topic(
                    name="dummy_init", msg_type="String"
                )  # Dummy init to set from json
            self.event_topic.from_json(dict_obj["topic"])
            self._handle_once = dict_obj["handle_once"]
            self._keep_event_delay = dict_obj["event_delay"]
            if dict_obj.get("trigger_ref_value") is not None:
                self.trigger_ref_value = dict_obj["trigger_ref_value"]
            if dict_obj.get("_attrs") is not None:
                self._attrs = dict_obj["_attrs"]
        except Exception as e:
            logging.error(f"Cannot set Event from incompatible dictionary. {e}")
            raise

    def set_dictionary(self, dict_obj, topic_template: Topic):
        """
        Set event using a dictionary and a Topic template
        Note: The template topic is added to pass a child class of the Topic class that exists in auto_ros

        :param dict_obj: Event description dictionary
        :type dict_obj: Dict
        :param topic_template: Template for the event topic
        :type topic_template: Topic
        """
        # Set event topic from the dictionary using the template as an initial value
        topic_template.from_json(dict_obj["topic"])
        self.event_topic = topic_template
        self.dictionary = dict_obj

    @property
    def json(self) -> str:
        """
        Property to get/set the event using a json

        :return: Event description dictionary as json
        :rtype: str
        """
        return json.dumps(self.dictionary)

    @json.setter
    def json(self, json_obj: Union[str, bytes, bytearray]):
        """
        Property to get/set the event using a json

        :param json_obj: Event description dictionary as json
        :type json_obj: Union[str, bytes, bytearray]
        """
        dict_obj = json.loads(json_obj)
        self.dictionary = dict_obj

    def callback(self, msg: Any) -> None:
        """
        Event topic listener callback

        :param msg: Event trigger topic message
        :type msg: Any
        """
        if self._handle_once and self._processed_once:
            return

        self._event_value = Operand(msg, self._attrs)

        self._update_trigger()
        # Process event if trigger is up and the event is not already under processing
        if self.trigger and not self.under_processing:
            self.under_processing = True
            self._call_on_trigger(msg=msg, trigger=self._event_value)
            self._processed_once = True
            # If a delay is provided start a timer and set the event under_processing flag to False only when the delay expires
            if self._keep_event_delay:
                time.sleep(self._keep_event_delay)
            self.under_processing = False

    def register_method(self, method_name: str, method: Callable[..., Any]) -> None:
        """
        Adds a new method to the on trigger register

        :param method_name: Key name of the method
        :type method_name: str

        :param method: Method to be executed
        :type method: Callable[..., Any]
        """
        self._registered_on_trigger_methods[method_name] = method

    def register_actions(self, actions: Union[Action, List[Action]]) -> None:
        """Register an Action or a set of Actions to execute on trigger

        :param actions: Action or a list of Actions
        :type actions: Union[Action, List[Action]]
        """
        self._registered_on_trigger_actions = (
            actions if isinstance(actions, List) else [actions]
        )

    def clear_actions(self) -> None:
        """Clear all registered on trigger Actions"""
        self._registered_on_trigger_actions = []

    def remove_method(self, method_name: str):
        """
        Adds a new method to the on trigger register

        :param method_name: Key name of the method
        :type method_name: str

        :param method: Method to be executed
        :type method: Callable[..., Any]
        """
        if method_name in self._registered_on_trigger_methods.keys():
            del self._registered_on_trigger_methods[method_name]

    def _call_on_trigger(self, *args, **kwargs):
        """
        Executes all the registered on trigger methods
        """
        for method in self._registered_on_trigger_methods.values():
            method(*args, **kwargs)

        # Execute all actions
        for action in self._registered_on_trigger_actions:
            action(*args, **kwargs)

    @abstractmethod
    def _update_trigger(self, *_, **__) -> None:
        """
        Custom trigger update
        """
        raise NotImplementedError

    def __bool__(self) -> bool:
        """
        Bool method for Event object
        """
        return self.trigger

    def __and2__(self, __value) -> bool:
        """
        AND for 2 Event objects
        """
        return isinstance(__value, Event) and self.trigger and __value.trigger

    def __or2__(self, __value) -> bool:
        """
        OR for 2 Event objects
        """
        return isinstance(__value, Event) and (self.trigger or __value.trigger)

    def __invert__(self) -> bool:
        """
        ~ for Event object
        """
        return not self.trigger

    def __str__(self) -> str:
        """
        str for Event object
        """
        return f"'{self.name}' ({super().__str__()})"
