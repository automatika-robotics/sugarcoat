"""Event"""

import inspect
import json
import time
import uuid
from attrs import define, field
from typing import Any, Callable, Dict, List, Union, Optional
from launch.event import Event as ROSLaunchEvent
from launch.event_handler import EventHandler as ROSLaunchEventHandler
from copy import copy
from concurrent.futures import ThreadPoolExecutor

from ..io.topic import Topic
from .action import Action, OpaqueCoroutine, OpaqueFunction
from ..condition import Condition
from ..utils import SomeEntitiesType
from ..utils import logger


class InternalEvent(ROSLaunchEvent):
    """
    Class to transform a Kompass event to ROS launch event using event key name
    """

    def __init__(self, event_name: str, topics_value: Dict) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :rtype: None
        """
        super().__init__()
        self.__event_name = event_name
        self.__topics_value = topics_value

    @property
    def event_name(self):
        """
        Getter of internal event name

        :return: Event name
        :rtype: str
        """
        return self.__event_name

    @property
    def topics_value(self):
        """
        Getter of internal event name

        :return: Event name
        :rtype: str
        """
        return self.__topics_value

    @topics_value.setter
    def topics_value(self, value):
        """
        Getter of internal event name

        :return: Event name
        :rtype: str
        """
        self.__topics_value = value


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

    def handle(self, event: ROSLaunchEvent, context) -> Optional[SomeEntitiesType]:
        """
        Overriding handle to inject event data into the entities.
        """
        # Capture the entities defined in __init__ (via super())
        entities = super().handle(event, context)
        new_entities = []
        # Safety check: Ensure we are dealing with internal event type
        if entities is not None and isinstance(event, InternalEvent):
            data = event.topics_value

            # Iterate through entities and inject the data
            for entity in entities:
                if isinstance(entity, OpaqueFunction):
                    # We use partial to inject 'topics_value' into the inner function
                    # This assumes your inner functions are ready to accept 'topics_value'
                    kwargs = {**entity.kwargs, "topics": data}
                    new_entities.append(
                        OpaqueFunction(
                            function=entity.function, args=entity.args, kwargs=kwargs
                        )
                    )
                elif isinstance(entity, OpaqueCoroutine):
                    # We use partial to inject 'topics_value' into the inner function
                    # This assumes your inner functions are ready to accept 'topics_value'
                    kwargs = {**entity.kwargs, "topics": data}
                    new_entities.append(
                        OpaqueCoroutine(
                            coroutine=entity.coroutine, args=entity.args, kwargs=kwargs
                        )
                    )
                else:
                    new_entities.append(entity)

        return new_entities


@define
class EventBlackboardEntry:
    """
    A container for timestamped messages stored in the Event Blackboard.

    This class wraps raw ROS messages with metadata to enable:
    1. **Time-To-Live (TTL) Checks:** Using ``timestamp`` to invalidate old data.
    2. **Idempotency:** Using a unique ``id`` to prevent the same message instance
       from triggering the same event multiple times.

    :param msg: The actual data payload (e.g., a ROS message).
    :type msg: Any
    :param timestamp: The standard Unix timestamp (float) when the message was received.
    :type timestamp: float
    :param id: A unique UUID4 string identifying this specific message reception instance.
               Defaults to a new UUID if not provided.
    :type id: str
    """

    msg: Any
    timestamp: float
    # A unique identifier for this specific reception instance
    id: str = field(factory=lambda: str(uuid.uuid4()))

    def validate(self, timeout: Optional[float] = None, stale_id: Optional[str] = None):
        """Validate the data freshness

        :param timeout: Maximum lifetime, defaults to None
        :type timeout: Optional[float], optional
        :param stale_id: ID of the latest stale message, defaults to None
        :type stale_id: Optional[str], optional
        """
        age = time.time() - self.timestamp
        if (stale_id and self.id == stale_id) or (age >= timeout if timeout else False):
            # Return none as this is already stale
            self.msg = None
            self.id = str(uuid.uuid4())
            return

    @classmethod
    def get(
        cls,
        entries_dict: Dict[str, "EventBlackboardEntry"],
        topic_name: str,
        timeout: Optional[float],
        stale_id: Optional[str] = None,
    ) -> Optional[Any]:
        """
        Retrieves data from entries_dict:
        - If data is missing: returns None
        - If data is present but expired: Deletes it and returns None (Lazy Expiration)
        - If data is valid: returns the entry
        """
        entry = entries_dict.get(topic_name)

        if entry is None:
            return None

        if stale_id and entry.id == stale_id:
            # Return none as this is already stale for one event,
            # but do not delete as it might be required by others
            return None

        # Check Timeout (if defined)
        if timeout is not None:
            age = time.time() - entry.timestamp
            if age > timeout:
                # LAZY DELETION: The data is dead, clean it up now.
                del entries_dict[topic_name]
                return None

        return entry


class Event:
    """A runtime trigger driven by ROS2 topic data or a callable condition.

    An Event monitors one of three condition types and fires registered Actions
    when the condition becomes true:

    1. **Condition expression** – a declarative predicate on topic message
       attributes (e.g. ``topic.msg.speed > 5.0``). The condition is evaluated
       each time new data arrives on the involved topics.
    2. **Topic (on-any)** – triggers whenever a valid message is available on
       the given topic(s), regardless of content.
    3. **Callable** – a user-supplied function polled at ``check_rate`` whose
       boolean return value determines the trigger state.

    Trigger behaviour can be further refined with:

    * ``on_change`` – fire only on a *rising edge* (condition transitions from
      false to true), rather than every evaluation cycle.
    * ``handle_once`` – fire at most once and then become inert.
    * ``keep_event_delay`` – hold the *under-processing* flag for a fixed
      duration after actions complete, throttling re-triggers.

    Actions are executed asynchronously via a shared ``ThreadPoolExecutor``.
    """

    # SHARED EXECUTOR across all Event instances to prevent thread explosion.
    # TODO: How to adjust max_workers? based on system capabilities?
    _action_executor = ThreadPoolExecutor(
        max_workers=10, thread_name_prefix="action_worker"
    )

    def __init__(
        self,
        event_condition: Union[Topic, Condition, Callable],
        on_change: bool = False,
        handle_once: bool = False,
        keep_event_delay: float = 0.0,
        check_rate: Optional[float] = None,
    ) -> None:
        """Create a new Event.

        :param event_condition: The source that drives this event. Can be a
            ``Condition`` expression on topic attributes, a ``Topic`` for
            on-any monitoring, or a ``Callable`` that returns ``bool`` and is
            polled at ``check_rate``.
        :type event_condition: Union[Topic, Condition, Callable]
        :param on_change: If True, trigger only on a rising edge (condition
            transitions from False to True), defaults to False.
        :type on_change: bool, optional
        :param handle_once: If True, execute registered actions at most once
            across the lifetime of this event, defaults to False.
        :type handle_once: bool, optional
        :param keep_event_delay: Minimum delay in seconds to hold the
            under-processing flag after actions complete, throttling
            re-triggers, defaults to 0.0.
        :type keep_event_delay: float, optional
        :param check_rate: Polling rate in Hz for callable-based conditions. If not provided, the component loop_rate is used instead.
            Only used when ``event_condition`` is a Callable, defaults to None.
        :type check_rate: Optional[float], optional
        :raises TypeError: If a callable-based condition is a component-bound
            method or does not return bool.
        :raises AttributeError: If ``event_condition`` is not a supported type.
        """
        # Unique event ID
        self.__id = str(uuid.uuid4())
        self._handle_once: bool = handle_once
        self._keep_event_delay: float = keep_event_delay
        self._on_change: bool = on_change
        self._on_any: bool = False
        self._previous_trigger = None
        self.__under_processing = False
        self._processed_once: bool = False

        # Case 1: Init from Condition Expression (topic.msg.data > 5)
        if isinstance(event_condition, Condition):
            self._condition = event_condition
            self._action_condition: Optional[Action] = None
            self._is_action_based: bool = False
            self.check_rate: Optional[float] = None

        # Case 2: Topics are passed for on_any event
        elif isinstance(event_condition, Topic):
            self._action_condition: Optional[Action] = None
            self._is_action_based: bool = False
            self.check_rate: Optional[float] = None
            self._condition = Condition(
                topic_name=event_condition.name,
                topic_msg_type=event_condition.msg_type.__name__,
                topic_qos_config=event_condition.qos_profile.to_dict(),
                attribute_path=[],
                operator_func=None,
                ref_value=None,
            )
            self._on_any = True
        # Case 3: Callable-based polling: action return value is the boolean condition
        elif isinstance(event_condition, Callable):
            self._condition = None
            self._action_condition = Action(method=event_condition)
            self._is_action_based = True
            self.check_rate = check_rate
            # Validate: must not be a @component_action (bound to a component lifecycle)
            if self._action_condition.component_action:
                raise TypeError(
                    f"Cannot use a component method '{event_condition.__name__}' as an event_condition. "
                    f"Use a plain callable instead."
                )
            # Validate: return type must be bool
            return_type = inspect.signature(event_condition).return_annotation
            if return_type is not inspect.Parameter.empty and return_type is not bool:
                raise TypeError(
                    f"event_condition callable must return bool, "
                    f"but '{event_condition.__name__}' has return annotation '{return_type}'."
                )
        else:
            raise AttributeError(
                f"Cannot initialize Event class. Must provide 'event_condition' as a Topic, a Condition on a Topic or a valid Method to be checked at check_rate, got {type(event_condition)}"
            )

        # Init trigger as False
        self.trigger: bool = False

        # Register for on trigger actions
        self._registered_on_trigger_actions: List[Union[Callable, Action]] = []

        # Required topics registry
        self.__required_topics: List[Topic] = []
        if not self._is_action_based:
            required_topics_dict: Dict = self._condition._get_involved_topics()
            for topic_name, topic_dict in required_topics_dict.items():
                self.__required_topics.append(Topic(name=topic_name, **topic_dict))

        # Additional Action Topics
        self.__additional_action_topics: List[Topic] = []

        # Stores the ID of the last processed message for each topic involved
        self.__last_processed_ids: Dict[str, str] = {}

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

    @property
    def id(self) -> str:
        """Getter of the event unique id

        :return: Unique ID
        :rtype: str
        """
        return self.__id

    def reset(self):
        """Reset event processing"""
        self._processed_once = False
        self.under_processing = False
        self.trigger = False
        self._previous_trigger = None

    def clear(self) -> None:
        """
        Clear event trigger
        """
        self.trigger = False

    def raise_event_trigger(self) -> None:
        """
        Raise event trigger
        """
        self.trigger = True

    def to_dict(self) -> Dict:
        """
        Property to parse the event into a dictionary

        :return: Event description dictionary
        :rtype: Dict
        """
        # NOTE: callable-based events are always handled by the Monitor (main process). Events with callable conditions will never be ran in a different process, so the serialization/deserialization of action_condition is not needed
        event_dict = {
            "name": self.__id,
            "condition": self._condition.to_json() if self._condition else None,
            "handle_once": self._handle_once,
            "keep_event_delay": self._keep_event_delay,
            "on_change": self._on_change,
        }
        return event_dict

    @classmethod
    def from_dict(cls, dict_obj: Dict):
        """
        Setter of the event using a dictionary

        :param dict_obj: Event description dictionary
        :type dict_obj: Dict
        """
        try:
            event_condition = Condition.from_dict(json.loads(dict_obj["condition"]))
            event = cls(
                event_condition=event_condition,
                on_change=dict_obj["on_change"],
                handle_once=dict_obj["handle_once"],
                keep_event_delay=dict_obj["keep_event_delay"],
            )
            # Set the same ID to the event
            event.__id = dict_obj["name"]
            return event
        except Exception as e:
            logger.error(f"Cannot set Event from incompatible dictionary. {e}")
            raise

    def to_json(self) -> str:
        """
        Property to get/set the event using a json

        :return: Event description dictionary as json
        :rtype: str
        """
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, json_obj: Union[str, bytes, bytearray]):
        """
        Property to get/set the event using a json

        :param json_obj: Event description dictionary as json
        :type json_obj: Union[str, bytes, bytearray]
        """
        dict_obj = json.loads(json_obj)
        return cls.from_dict(dict_obj)

    def get_involved_topics(self) -> List[Topic]:
        """Get all the topics required for monitoring this event

        :return: Required topics
        :rtype: List[Topic]
        """
        return self.__required_topics + self.__additional_action_topics

    def get_last_processed_id(self, topic_name: str) -> Optional[str]:
        """Get the unique ID of the last processed message for a given topic

        :param topic_name: Topic name
        :type topic_name: str
        :return: The last unique ID if the topic was processed earlier, otherwise None
        :rtype: Optional[str]
        """
        return self.__last_processed_ids.get(topic_name, None)

    def verify_required_action_topics(self, action: Action) -> None:
        """Verify the action topic parsers (if present) against an event.
           Raises a 'ValueError' if there is a mismatch.

        :param event: Event to verify against
        :type event: Event
        """

        action_topics: List[Topic] = action.get_required_topics()
        event_topics = self.get_involved_topics()
        for topic in action_topics:
            if topic.name not in [event_topic.name for event_topic in event_topics]:
                # Add the topic required by the action to the event conditions (as on any)
                # to ensure getting its message value
                self.__additional_action_topics.append(topic)

    def _execute_actions(self, global_topic_cache: Dict) -> None:
        """
        Event topic listener callback

        :param msg: Event trigger topic message
        :type msg: Any
        """
        if self._handle_once and self._processed_once:
            return

        # Process event if trigger is up and the event is not already under processing
        if self.trigger and not self.under_processing:
            self.under_processing = True
            # Offload the actual execution to the executor
            Event._action_executor.submit(
                self._async_action_wrapper, global_topic_cache
            )

    def _async_action_wrapper(self, global_topic_cache: Dict) -> None:
        """
        The actual execution logic running in the background thread.
        Handles the execution, delay, and flag resetting.
        """
        try:
            # Execute all actions
            for action in self._registered_on_trigger_actions:
                action(topics=global_topic_cache)

            # Handle the blocking delay inside the thread (so main loop isn't blocked)
            if self._keep_event_delay > 0:
                # If a delay is provided start a timer and
                # set the event under_processing flag to False only when the delay expires
                time.sleep(self._keep_event_delay)

        except Exception as e:
            logger.error(f"Error executing actions for event '{self.name}': {e}")
        finally:
            # Reset the flag only after work + delay are done
            self.under_processing = False
            self._processed_once = True

    def register_actions(
        self, actions: Union[Action, Callable, List[Union[Action, Callable]]]
    ) -> None:
        """Register an Action or a set of Actions to execute on trigger

        :param actions: Action or a list of Actions
        :type actions: Union[Action, List[Action]]
        """
        actions = actions if isinstance(actions, List) else [actions]
        # If it is a simple condition
        topics = self.get_involved_topics()
        for act in actions:
            if len(topics) == 1 and isinstance(act, Action):
                # Setup any required automatic conversion from the event message type to the action inputs
                act._setup_conversions(topics[0].name, topics[0].ros_msg_type)
            self._registered_on_trigger_actions.append(act)

    def clear_actions(self) -> None:
        """Clear all registered on trigger Actions"""
        self._registered_on_trigger_actions = []

    def check_condition(
        self, global_topic_cache: Dict[str, EventBlackboardEntry]
    ) -> None:
        """
        Replaces existing trigger logic.
        Evaluates the root Condition tree against the global cache.
        """
        topics_dict = {key: value.msg for key, value in global_topic_cache.items()}

        if self._on_any:
            # Check that all involved topics has values
            topics_names = [topic.name for topic in self.get_involved_topics()]
            self.trigger = all(topics_dict.get(key, None) for key in topics_names)
            self._previous_trigger = copy(self.trigger)
        else:
            # This assumes self.event_condition is now the root Condition object
            triggered = self._condition.evaluate(topics_dict)

            # If the event is to be checked only 'on_change' in the value
            # then check if:
            # 1. the event previous value is different from the event current value (there is a change)
            # and 2. if the new_trigger is on
            # If on_change and 1 and 2 -> activate the trigger
            if (
                self._on_change
                and self._previous_trigger is not None
                and not self._previous_trigger
                and triggered
            ):
                self.trigger = True
            else:
                # If:
                # 1. on_change is not required
                # or 2. the event previous value is the same as the current value (no change happened)
                # then just directly update the trigger
                self.trigger = triggered
            self._previous_trigger = copy(triggered)

        if self.trigger:
            # If triggered update the last processed IDs
            # This prevents handling the same topic data twice in the period before its 'stale'
            self.__last_processed_ids = {
                key: value.id for key, value in global_topic_cache.items()
            }
            self._execute_actions(topics_dict)
        return

    def check_action_condition(
        self, global_topic_cache: Dict[str, EventBlackboardEntry]
    ) -> None:
        """Evaluate the action-based condition and execute registered actions if triggered.
        Called periodically by a component timer; should only be used on action-based events.
        """
        triggered = bool(self._action_condition())

        if self._on_change and self._previous_trigger is not None:
            self.trigger = triggered and not self._previous_trigger
        else:
            self.trigger = triggered

        if self.trigger:
            topics_dict = {key: value.msg for key, value in global_topic_cache.items()}
            self.__last_processed_ids = {
                key: value.id for key, value in global_topic_cache.items()
            }
            self._execute_actions(topics_dict)

        self._previous_trigger = copy(triggered)

    def __str__(self) -> str:
        """
        str for Event object
        """
        if self._is_action_based:
            return f"Callable Based Event(On '{self._action_condition.action_name}' returns true polled at {self.check_rate if self.check_rate else 'loop_rate'}Hz) (ID {self.__id})"
        return f"{self._condition._readable()} (ID {self.__id})"
