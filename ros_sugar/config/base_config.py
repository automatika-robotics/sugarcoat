"""Config Classes for Components and Topics"""

from enum import Enum
from typing import Union, Optional, Dict, Any
import functools

from attrs import define, field
from rclpy import qos
import rclpy.callback_groups as ros_callback_groups
from rclpy.logging import LoggingSeverity

from . import base_validators
from .base_attrs import BaseAttrs

__all__ = ["QoSConfig", "BaseComponentConfig", "BaseConfig", "ComponentRunType"]


# mapping base_validator function names to ui friendly operation names
FUNC_NAME_MAP = {
    "__gt": "greater_than",
    "__lt": "less_than",
    "__in_": "in",
    "__list_contained_in": "list_contained_in",
    "__in_range_validator": "in_range",
    "__in_range_discretized_validator": "in_range_discretized",
}


def _get_enum_value(enm_val):
    """
    Converter to get value from Enum
    """
    if isinstance(enm_val, Enum):
        return enm_val.value
    return enm_val


@define(kw_only=True)
class QoSConfig(BaseAttrs):
    """
    Class for quality of service (QoS) configuration in ROS2

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **history**
      - `int`, `[qos](https://docs.ros2.org/foxy/api/rclpy/api/qos.html).HistoryPolicy.KEEP_LAST`
      - Sample store type: ALL or LAST (up to N samples, configurable via the queue depth option)

    * - **queue_size**
      - `int`, `10`
      - Used only if the “history” policy was set to “keep last”

    * - **reliability**
      - `int`, `qos.ReliabilityPolicy.RELIABLE`
      - Level of reliability in delivering samples

    * - **durability**
      - `int`, `qos.DurabilityPolicy.VOLATILE`
      - Determines if the publisher will be persisting samples for “late-joining” subscriptions (Transit Local) or not (Volatile)
    ```
    """

    # Keep last: only store up to N samples, configurable via the queue depth option.
    # Keep all: store all samples, subject to the configured resource limits of the underlying middleware.
    history: int = field(
        converter=_get_enum_value,
        default=qos.HistoryPolicy.KEEP_LAST,
        validator=base_validators.in_(list(qos.HistoryPolicy)),
    )

    # used only if the “history” policy was set to “keep last”
    queue_size: int = field(
        default=10, validator=base_validators.in_range(min_value=0, max_value=1e3)
    )

    # Best effort: attempt to deliver samples, but may loose them if the network is not robust
    # Reliable: guarantee that samples are delivered, may retry multiple times
    reliability: int = field(
        converter=_get_enum_value,
        default=qos.ReliabilityPolicy.RELIABLE,
        validator=base_validators.in_(list(qos.ReliabilityPolicy)),
    )

    # Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscriptions
    # Volatile: no attempt is made to persist samples
    durability: int = field(
        converter=_get_enum_value,
        default=qos.DurabilityPolicy.VOLATILE,
        validator=base_validators.in_(list(qos.DurabilityPolicy)),
    )

    # TODO: Fix default values
    # the expected maximum amount of time between subsequent messages being published to a topic
    # deadline: Optional[qos.Duration] = field(default=None)

    # the maximum amount of time between the publishing and the reception of a message without the message being considered stale or expired (expired messages are silently dropped and are effectively never received).
    # lifespan: Optional[qos.Duration] = field(default=None)

    def to_ros(self) -> qos.QoSProfile:
        """Converts the config class to ROS2 QoS Profile

        :return: ROS2 QoS Profile
        :rtype: qos.QoSProfile
        """
        return qos.QoSProfile(
            reliability=self.reliability,
            history=self.history,
            depth=self.queue_size,
            durability=self.durability,
        )


@define(kw_only=True)
class BaseConfig(BaseAttrs):
    """
    Node General Parameters

    :param loop_rate: Rate (Hz) in which the node executes
    :type loop_rate: float
    :param visualization: To publish additional topics for visualization
    :type visualization: bool
    """

    # Runtime
    loop_rate: float = field(
        default=100.0, validator=base_validators.in_range(min_value=1e-4, max_value=1e9)
    )  # Hz


class ComponentRunType(Enum):
    """
    Component run type:
    - Timed: Executes main in a timed loop
    - Event: Executes based on a trigger topic/event
    - Server: Executes based on a ROS service request from a client
    - ActionServer: Executes based on a ROS action server request from a client
    """

    TIMED = "Timed"
    EVENT = "Event"
    SERVER = "Server"
    ACTION_SERVER = "ActionServer"

    @classmethod
    def values(cls):
        return [member.value for member in cls]

    def __str__(self) -> str:
        """
        Gets value of enum

        :return: Enum value
        :rtype: str
        """
        return self.value

    def __repr__(self) -> str:
        """
        Gets value of enum

        :return: Enum value
        :rtype: str
        """
        return self.value

    @classmethod
    def to_str(cls, enum_value) -> str:
        """
        Return string value corresponding to enum value if exists

        :param enum_value: _description_
        :type enum_value: ComponentRunType | str
        :raises ValueError: If the enum value is not from this class

        :return: String value
        :rtype: str
        """
        if isinstance(enum_value, ComponentRunType):
            return enum_value.value
        # If the value is already given as a string check if it valid and return it
        elif isinstance(enum_value, str):
            if enum_value in cls.values():
                return enum_value
        raise ValueError(f"{enum_value} is not a valid ComponentRunType value")


def _convert_runtype_to_enum(
    value: Union[ComponentRunType, str],
) -> ComponentRunType:
    """
    Converter for ComponentRunType to set the value from strings

    :param value: Runtype value
    :type value: Union[ComponentRunType, str]

    :raises ValueError: If string value is not one of the ComponentRunType Enum values

    :return: Enum value
    :rtype: ComponentRunType
    """
    if isinstance(value, ComponentRunType):
        return value
    if isinstance(value, str):
        for value_enum in ComponentRunType:
            if value_enum.value == value:
                return value_enum
        raise ValueError(f"Unsupported ComponentRunTime value '{value}'")


def _convert_logging_severity_to_str(
    value: Union[LoggingSeverity, str],
) -> str:
    """
    Converter for ComponentRunType to set the value from strings

    :param value: Runtype value
    :type value: Union[ComponentRunType, str]

    :raises ValueError: If string value is not one of the ComponentRunType Enum values

    :return: Enum value
    :rtype: ComponentRunType
    """
    if isinstance(value, LoggingSeverity):
        return value.name.lower()
    if isinstance(value, str):
        # Validate string value
        for value_enum in LoggingSeverity:
            if value_enum.name.lower() == value.lower():
                return value
        raise ValueError(f"Unsupported Logging Severity Value '{value}'")


def _get_str_from_callbackgroup(
    callback_group: Union[str, ros_callback_groups.CallbackGroup],
) -> Optional[str]:
    """
    Get callback group from string
    """
    if not callback_group:
        return

    if isinstance(callback_group, ros_callback_groups.CallbackGroup):
        return callback_group.__class__.__name__

    return callback_group


@define(kw_only=True)
class BaseComponentConfig(BaseConfig):
    """
    Component configuration parameters

    :param use_without_launcher: To use the component without the Launcher. When True, it initializes a ROS node on component init.
    :type use_without_launcher: bool

    :param fallback_rate: Rate (Hz) at which the component checks for fallbacks and executes actions if a failure is detected.
    :type fallback_rate: float

    :param log_level: Logging level for the component's internal logs. Can be a string or `LoggingSeverity` enum.
    :type log_level: Union[str, LoggingSeverity]

    :param rclpy_log_level: Logging level for rclpy (ROS client library) logs. Can be a string or `LoggingSeverity` enum.
    :type rclpy_log_level: Union[str, LoggingSeverity]

    :param run_type: Component run type, e.g., TIMED or EVENT. Can be a string or `ComponentRunType` enum.
    :type run_type: Union[ComponentRunType, str]

    :param _callback_group: (Optional) Callback group used by the component. Can be a string or `ros_callback_groups.CallbackGroup` instance.
    :type _callback_group: Optional[Union[ros_callback_groups.CallbackGroup, str]]

    :param wait_for_restart_time: Time (in seconds) the component waits for a node to come back online after restart. Used to avoid infinite restart loops. Recommended to use a high value.
    :type wait_for_restart_time: float
    """

    use_without_launcher: bool = field(default=False)

    # NOTE: Layer ID to be added in coming updates
    # layer_id: int = field(
    #     default=0, validator=base_validators.in_range(min_value=0, max_value=1e3)
    # )

    fallback_rate: float = field(
        default=100.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )

    log_level: Union[str, LoggingSeverity] = field(
        default=LoggingSeverity.INFO, converter=_convert_logging_severity_to_str
    )

    rclpy_log_level: Union[str, LoggingSeverity] = field(
        default=LoggingSeverity.WARN, converter=_convert_logging_severity_to_str
    )

    run_type: Union[ComponentRunType, str] = field(
        default=ComponentRunType.TIMED, converter=_convert_runtype_to_enum
    )

    _callback_group: Optional[Union[ros_callback_groups.CallbackGroup, str]] = field(
        default=None, converter=_get_str_from_callbackgroup, alias="_callback_group"
    )

    wait_for_restart_time: float = field(
        default=6000.0,
        validator=base_validators.in_range(min_value=10.0, max_value=1e9),
    )  # Component wait time for node to come back online after restart (used to avoid infinite loops). Recommended to use a high value

    @staticmethod
    def _parse_validator(validator: object) -> Dict[str, Optional[Any]]:
        """
        Introspects an attrs validator object to extract its parameters.

        :param validator: The validator object to parse.
        :return: A dictionary with the validator's name and parameters.
        """
        validator_name = validator.__class__.__name__

        # Handle attrs built-in range validators (gt, ge, lt, le)
        if hasattr(validator, "bound"):
            op_name = {
                "_GreaterThenValidator": "greater_than",
                "_GreaterEqualValidator": "greater_or_equal",
                "_LessThenValidator": "less_than",
                "_LessEqualValidator": "less_or_equal",
            }.get(validator_name, "bound")
            return {op_name: validator.bound}

        # Handle functions of base_validators
        if isinstance(validator, functools.partial):
            func = validator.func
            func_name = func.__name__
            # ensure function is from base_validator
            if hasattr(base_validators, func_name):
                # map name to standard name
                op_name = FUNC_NAME_MAP.get(func_name, func_name)
                # collect bounds/parameters
                bounds = {}
                # keyword args in partial
                bounds.update(validator.keywords or {})
                return {op_name: bounds}

        # Fallback for unknown validators
        return {"unknown": "unknown"}

    @classmethod
    def get_fields_info(cls) -> Dict[str, Dict[str, Any]]:
        """
        Returns a dictionary with metadata about each field in the class.

        This includes the field's name, type annotation, and parsed validator info.

        :return: A dictionary where keys are field names and values are dicts
                 of metadata.
        """
        fields_info = {}
        # Iterate over all attributes defined by attrs
        for attr_field in cls.__attrs_attrs__:
            if (
                attr_field in BaseComponentConfig.__attrs_attrs__
                or attr_field.name.startswith("_")
            ):
                continue
            validators_list = []
            # Check if a validator exists for the field
            if attr_field.validator:
                # A validator can be a single callable or a list/tuple of them
                # if they are wrapped in attrs.validators.and_()
                # Check for composite validators (like and_())
                if hasattr(attr_field.validator, "validators"):
                    for v in attr_field.validator.validators:
                        validators_list.append(cls._parse_validator(v))

                else:  # It's a single validator
                    validators_list.append(cls._parse_validator(attr_field.validator))

            fields_info[attr_field.name] = {
                "type": attr_field.type,
                "validators": validators_list,
            }
        return fields_info
