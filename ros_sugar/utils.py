import inspect
from enum import IntEnum as BaseIntEnum
from functools import wraps
from typing import Callable, List, Union, TypeVar, Type, Any
from attrs import define
import operator

from rclpy.utilities import ok as rclpy_is_ok
from rclpy.lifecycle import Node as LifecycleNode
from launch import LaunchContext
from launch.actions import OpaqueFunction
import os
import logging

from .config.base_attrs import BaseAttrs


# Get ROS distro
__installed_distro = os.environ.get("ROS_DISTRO", "").lower()

try:
    if __installed_distro in ["humble", "galactic", "foxy"]:
        # Get some_action_type for older distributions
        from launch.some_actions_type import SomeActionsType as SomeEntitiesType
    else:
        from launch.some_entities_type import SomeEntitiesType
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "Could not determine correct ROS version. Make sure ROS_DISTRO variable is set and ROS_DISTRO is >= `humble`"
    ) from e


# logger for utils
logger = logging.getLogger("Sugarcoat")


# Define a generic type variable for topic message types
MsgT = TypeVar("MsgT")


@define
class Condition(BaseAttrs):
    """Holds the parsed condition from an event expression."""

    topic_source: Any  # The Topic object
    attribute_path: List[str]
    operator_func: Callable
    ref_value: Any


class _MsgConditionBuilder:
    """Helper class to build paths for accessing ROS message attributes in topics.
    Used for parsing topic message attributes for Actions and Event parsers.
    """

    def __init__(self, topic, path=None):
        self._name = topic.name
        self._type = topic.ros_msg_type
        self._topic = topic
        self._base = path or []

    def __getattr__(self, name):
        # Validate that the attribute exists in the message type
        augmented_base = self._base + [name]
        start = self._type()
        for key, attribute_name in enumerate(augmented_base):
            if not hasattr(start, attribute_name):
                old_bases = ".".join(augmented_base[:key])
                error = (
                    f"Available attributes: {start.get_fields_and_field_types()}"
                    if hasattr(start, "get_fields_and_field_types")
                    else ""
                )
                raise AttributeError(
                    f"Message '{self._type.__name__}.{old_bases}' has no attribute: '{augmented_base[key]}'. "
                    + error
                )
            start = getattr(start, attribute_name)
        return _MsgConditionBuilder(self._topic, augmented_base)

    def as_tuple(self) -> tuple:
        return tuple(self._base)

    @property
    def name(self) -> str:
        return self._name

    @property
    def type(self) -> Type[MsgT]:
        return self._type

    def _get_value(self) -> Any:
        val = self._type()
        for attribute_name in self._base:
            val = getattr(val, attribute_name)
        return val

    # --- Dunder Methods for Conditional Parsing ---

    def _check_similar_type(self, other):
        """Helper method to insure compatible condition types
        """
        if (val_type := type(self._get_value())) is not type(other):
            raise TypeError(
                f"Cannot construct condition from incompatible types: Topic attribute type is '{val_type}', and reference type is '{type(other)}'"
            )

    def _check_similar_list_type(self, other):
        """Helper method to insure compatible condition types"""
        if not other or not isinstance(other, List):
            return
        if (val_type := type(self._get_value())) is not type(other[0]):
            raise TypeError(
                f"Cannot construct condition from incompatible types: Topic attribute type is '{val_type}', and reference type is list containing '{type(other[0])}' values"
            )

    def _make_condition(self, op: Callable, value: Any) -> Condition:
        return Condition(
            topic_source=self._topic,
            attribute_path=self._base,
            operator_func=op,
            ref_value=value,
        )

    def __eq__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.eq, other)

    def __ne__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.ne, other)

    def __lt__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.lt, other)

    def __le__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.le, other)

    def __gt__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.gt, other)

    def __ge__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(operator.ge, other)

    def is_in(self, other: Union[List, tuple]) -> Condition:
        """
        Check if the topic value is inside the provided list/tuple.
        Usage: topic.msg.status.is_in([1, 2, 3])
        """
        # We use operator.contains.
        # When Event executes: operator.contains(Operand(msg_val), [1,2,3])
        # This calls Operand.__contains__([1,2,3])
        self._check_similar_list_type(other)
        return self._make_condition(operator.contains, other)

    def not_in(self, other: Union[List, tuple]) -> Condition:
        """
        Check if the topic value is NOT inside the provided list/tuple.
        Usage: topic.msg.status.not_in([0, -1])
        """
        self._check_similar_list_type(other)

        # We define a custom method for 'not in'
        def not_contained_in(value, other: List) -> bool:
            if not isinstance(value, List):
                return value not in other
            else:
                return not all(a in other for a in value)

        return self._make_condition(not_contained_in, other)

    # --- Helper to safely get list from Operand ---
    @staticmethod
    def _get_value_as_list(op_obj) -> list:
        # Helper to ensure we are comparing against a list, even if the topic is scalar
        val = op_obj.value
        return val if isinstance(val, (list, tuple)) else [val]

    def contains_any(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value contains AT LEAST ONE of the values in 'other'.
        Equivalent to: set(topic) & set(other) is not empty
        """

        def _check_any(op_obj, ref_list):
            val_list = self._get_value_as_list(op_obj)
            return any(item in val_list for item in ref_list)

        return self._make_condition(_check_any, other)

    def contains_all(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value contains ALL of the values in 'other'.
        Equivalent to: set(other).issubset(set(topic))
        """

        def _check_all(op_obj, ref_list):
            val_list = self._get_value_as_list(op_obj)
            return all(item in val_list for item in ref_list)

        return self._make_condition(_check_all, other)

    def not_contains_any(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value contains NONE of the values in 'other'.
        Equivalent to: set(topic).isdisjoint(set(other))
        """

        def _check_none(op_obj, ref_list):
            val_list = self._get_value_as_list(op_obj)
            return not any(item in val_list for item in ref_list)

        return self._make_condition(_check_none, other)

    def not_contains_all(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value is MISSING at least one value from 'other'.
        Inverse of contains_all.
        """

        def _check_not_all(op_obj, ref_list):
            val_list = self._get_value_as_list(op_obj)
            return not all(item in val_list for item in ref_list)

        return self._make_condition(_check_not_all, other)


class IncompatibleSetup(Exception):
    """Exception raised when a component is configured with incompatible parameter values"""

    pass


class InvalidAction(Exception):
    """Exception raised when an Action is invalid or configured with incompatible values"""

    pass


class IntEnum(BaseIntEnum):
    """
    Extends enum.IntEnum class with methods to get all integer values and get enum value corresponding to given int value
    """

    @classmethod
    def get_enum(cls, __value: int) -> Union[int, None]:
        """
        Get Enum members equal to given values

        :param __value: Reference value
        :type __value: int
        :return: Enum value
        :rtype: int | None
        """
        for enum_member in cls:
            if enum_member.value == __value:
                return enum_member
        return None

    @classmethod
    def values(cls):
        """
        Get all enum values
        """
        return [member.value for member in cls]


def action_handler(function: Callable):
    """
    Decorator for components action handlers
    Verifies that the return type is a ros launch some_entities_type required for event handling

    :param function:
    :type function: Callable
    """

    @wraps(function)
    def _wrapper(*args, **kwargs):
        """_wrapper.
        :param a:
        :param kw:
        """
        return_type = inspect.signature(function).return_annotation
        if return_type is not SomeEntitiesType and return_type != "SomeEntitiesType":
            raise TypeError(
                f"Action handlers must return launch event handlers 'launch.some_entities_type.SomeEntitiesType'. Method '{function.__name__}' cannot have '@action_handler' decorator"
            )

        return function(*args, **kwargs)

    return _wrapper


def component_action(function: Callable, active: bool = False):
    """
    Decorator for components actions
    Verifies that the function is a valid Component method, returns a boolean or None, and that the Component is active

    :param function:
    :type function: Callable
    """

    @wraps(function)
    def _wrapper(*args, **kwargs):
        """_wrapper.
        :param a:
        :param kw:
        """
        if not args:
            raise TypeError(f"'{function.__name__}' is not a valid Component method")

        self = args[0]
        if not isinstance(self, LifecycleNode):
            raise TypeError(f"'{function.__name__}' is not a valid Component method")

        # Check return type
        return_type = inspect.signature(function).return_annotation
        if return_type is not bool and return_type:
            raise TypeError(
                f"Action methods must return boolean or None. Method '{function.__name__}' cannot have '@component_action' decorator"
            )

        # Check Component is active
        if rclpy_is_ok() and hasattr(self, "_state_machine"):
            # check for active flag and if the flag is True, check lifecycle_state is 3 i.e. active
            if not active or self._state_machine.current_state[1] == "active":
                return function(*args, **kwargs)
            else:
                logger.error(
                    f"Cannot use component action method '{function.__name__}' without activating the Component"
                )
                return None
        else:
            logger.error(
                f"Cannot use component action method '{function.__name__}' without initializing rclpy and the Component"
            )

    _wrapper.__name__ = function.__name__

    return _wrapper


def component_fallback(function: Callable):
    """
    Decorator for components fallback methods
    Verifies that rcply is initialized and component is configured or active

    :param function:
    :type function: Callable
    """

    @wraps(function)
    def _wrapper(*args, **kwargs):
        """_wrapper.
        :param a:
        :param kw:
        """
        if not args:
            raise TypeError(f"'{function.__name__}' is not a valid Component method")

        self = args[0]
        if not isinstance(self, LifecycleNode):
            raise TypeError(f"'{function.__name__}' is not a valid Component method")

        # Check Component is active
        if rclpy_is_ok() and hasattr(self, "_state_machine"):
            if self._state_machine.current_state[1] in [
                "active",
                "inactive",
                "activating",
            ]:
                return function(*args, **kwargs)
            else:
                logger.error(
                    f"{self._state_machine.current_state[1]} Cannot use component fallback method '{function.__name__}' without activating or configuring the Component"
                )
                return None
        else:
            logger.error(
                f"Cannot use component action method '{function.__name__}' without initializing rclpy and the Component"
            )

    _wrapper.__name__ = function.__name__

    return _wrapper


def launch_action(function: Callable):
    """
    Decorator to add LaunchCotext to a method to be used as a ros launch action

    :param function:
    :type function: Callable
    """

    @wraps(function)
    def _wrapper(*args, **kwargs):
        """_wrapper.
        :param a:
        :param kw:
        """

        def new_function(_: LaunchContext, *args, **kwargs):
            return function(*args, **kwargs)

        return OpaqueFunction(function=new_function, args=args, kwargs=kwargs)

    function_parameters = inspect.signature(function).parameters
    new_parameters = list(function_parameters.values())
    new_parameters.insert(
        0,
        inspect.Parameter(
            "context",
            kind=inspect.Parameter.POSITIONAL_ONLY,
            annotation=LaunchContext,
        ),
    )
    _wrapper.__signature__ = inspect.signature(function).replace(
        parameters=new_parameters
    )
    return _wrapper


def log_srv(srv_callback: Callable):
    """
    Decorator for components service callback methods to log request/response of the service call

    :param srv_callback:
    :type srv_callback: Callable
    """

    @wraps(srv_callback)
    def _wrapper(*args, **kwargs):
        """_wrapper.
        :param a:
        :param kw:
        """
        self = args[0]
        parameters = inspect.signature(srv_callback).parameters
        self.get_logger().info(
            f"Got New Service Request: {parameters['request'].annotation}"
        )
        response = srv_callback(*args, **kwargs)
        self.get_logger().info(f"Service returned response: {response}")
        return response

    return _wrapper


def has_decorator(method: Callable, decorator_name: str):
    """Helper method to check if a callable is decorated with given decorator
    :param method:
    :type method: Callable
    :param decorator_name:
    :type decorator_name: str

    :rtype: bool
    """
    if decorator_name.startswith("@"):
        decorator_name = decorator_name[1:]

    decorators = [
        i.strip()
        for i in inspect.getsource(method).split("\n")
        if i.strip().startswith("@")
    ]
    return f"@{decorator_name}" in decorators


def get_methods_with_decorator(obj, decorator_name: str) -> List[str]:
    """Helper method to get all object method names decorated with given decorator
    :param obj:
    :type obj: Any
    :param decorator_name:
    :type decorator_name: str

    :rtype: List[str]
    """
    method_names = []
    for name, method in inspect.getmembers(obj.__class__, predicate=inspect.isfunction):
        if has_decorator(method, decorator_name):
            method_names.append(name)
    return method_names


def camel_to_snake_case(text: str) -> str:
    """
    Turns given string from camel case to snake case
    used to automatically assign names to ros services and actions from type names

    :param text: _description_
    :type text: str
    :return: _description_
    :rtype: str
    """
    result = ""
    for char in text:
        if char.isupper():
            result += "_" + char.lower()
        else:
            result += char
    return result.lstrip("_")
