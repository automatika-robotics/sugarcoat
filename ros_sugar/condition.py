from typing import Any, List, Callable, Union, Type
from attrs import define
import operator
import array
import numpy as np

from .config.base_attrs import BaseAttrs
from .utils import MsgT


@define
class Condition(BaseAttrs):
    """Condition class used for defining events on incoming topic data.
    It captures the logic of an expression like `topic.msg.data > 5` to be evaluated at runtime.

    ```{list-table}
    :widths: 15 20 65
    :header-rows: 1

    * - Attribute
      - Type
      - Description

    * - **topic_source**
      - `Topic`
      - The source Topic object that provides the data stream.

    * - **attribute_path**
      - `List[str | int]`
      - The traversal path to access the specific field within the ROS message (e.g., `['header', 'stamp', 'sec']`).

    * - **operator_func**
      - `Callable`
      - The comparison function (from `ConditionOperators`) used to evaluate the message field against the reference value.

    * - **ref_value**
      - `Any`
      - The reference value (or list of values) to compare the message field against.
    ```
    """

    topic_source: Any  # The Topic object
    attribute_path: List[str]
    operator_func: Callable
    ref_value: Any

    @classmethod
    def serialized_operator(cls, operator_func: Callable) -> str:
        """Serialize condition operator"""
        return ConditionOperators.get_name(operator_func)

    @classmethod
    def deserialized_operator(cls, serialized_operator: str) -> Callable:
        """
        Deserialize from dictionary.

        :param topic_lookup: Dict mapping topic names to actual Topic objects
        """
        return ConditionOperators.get_operator(serialized_operator)


class ConditionOperators:
    """
    Registry of supported operators for Conditions.
    Enables serialization by mapping function names to callables.
    """

    _registry = {}

    # --- Internal Decorator ---
    def _register(func, registry=_registry):
        registry[func.__name__] = func
        return func

    @classmethod
    def get_operator(cls, name: str) -> Callable:
        """Retrieve function by name (for Deserialization)."""
        if name not in cls._registry:
            raise ValueError(f"Unknown operator: {name}")
        return cls._registry[name]

    @classmethod
    def get_name(cls, func: Callable) -> str:
        """Retrieve name by function (for Serialization)."""
        # Search registry for the function
        for name, registered_func in cls._registry.items():
            if registered_func == func:
                return name
        # Fallback for standard operators if passed directly
        if func == operator.eq:
            return "equals"
        if func == operator.ne:
            return "not_equals"
        if func == operator.gt:
            return "greater_than"
        if func == operator.ge:
            return "greater_or_equal"
        if func == operator.lt:
            return "less_than"
        if func == operator.le:
            return "less_or_equal"

        raise ValueError(
            f"Operator function {func} is not registered in ConditionOperators"
        )

    # --- Standard Comparison Operators ---

    @staticmethod
    @_register
    def equals(op_obj, ref):
        return op_obj == ref

    @staticmethod
    @_register
    def not_equals(op_obj, ref):
        return op_obj != ref

    @staticmethod
    @_register
    def greater_than(op_obj, ref):
        return op_obj > ref

    @staticmethod
    @_register
    def greater_or_equal(op_obj, ref):
        return op_obj >= ref

    @staticmethod
    @_register
    def less_than(op_obj, ref):
        return op_obj < ref

    @staticmethod
    @_register
    def less_or_equal(op_obj, ref):
        return op_obj <= ref

    # --- Container / Set Logic Operators ---
    # Helper to extract raw value from Operand if needed
    @staticmethod
    def _unwrap(op_obj):
        return op_obj.value if hasattr(op_obj, "value") else op_obj

    @staticmethod
    def _unwrap_list(op_obj) -> list:
        val = ConditionOperators._unwrap(op_obj)
        return val if isinstance(val, (list, tuple)) else [val]

    @staticmethod
    @_register
    def is_in(op_obj, ref_list):
        """Checks if Topic Value is IN Reference List"""
        # Note: We unwrap because 'Operand in List' checks for the object instance, not the value
        return ConditionOperators._unwrap(op_obj) in ref_list

    @staticmethod
    @_register
    def not_in(op_obj, ref_list):
        """Checks if Topic Value is NOT IN Reference List"""
        return ConditionOperators._unwrap(op_obj) not in ref_list

    @staticmethod
    @_register
    def contains(op_obj, ref_val):
        """Topic (string) contains a reference string"""
        return ref_val in op_obj

    @staticmethod
    @_register
    def not_contains(op_obj, ref_val):
        """Topic (string) does not contain a reference string"""
        return ref_val not in op_obj

    @staticmethod
    @_register
    def contains_any(op_obj, ref_list):
        """Topic (list) contains ANY of Reference (list)"""
        val_list = ConditionOperators._unwrap_list(op_obj)
        return any(item in val_list for item in ref_list)

    @staticmethod
    @_register
    def contains_all(op_obj, ref_list):
        """Topic (list) contains ALL of Reference (list)"""
        val_list = ConditionOperators._unwrap_list(op_obj)
        return all(item in val_list for item in ref_list)

    @staticmethod
    @_register
    def not_contains_any(op_obj, ref_list):
        """Topic (list) contains NONE of Reference (list)"""
        val_list = ConditionOperators._unwrap_list(op_obj)
        return not any(item in val_list for item in ref_list)

    @staticmethod
    @_register
    def not_contains_all(op_obj, ref_list):
        """Topic (list) does MISSING at least one of Reference (list)"""
        val_list = ConditionOperators._unwrap_list(op_obj)
        return not all(item in val_list for item in ref_list)


class MsgConditionBuilder:
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
        return MsgConditionBuilder(self._topic, augmented_base)

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

    def _check_similar_type(self, other):
        """Helper method to ensure compatible condition types"""
        val = self._get_value()

        # Allow Exact Match
        if type(val) is type(other):
            return

        # Allow Numeric Compatibility (int vs float)
        if isinstance(val, (int, float)) and isinstance(other, (int, float)):
            return

        # Allow Sequence Compatibility (list vs array.array vs tuple)
        # We explicitly check for list-like types excluding strings
        sequence_types = (list, tuple, array.array, np.ndarray)
        if isinstance(val, sequence_types) and isinstance(other, sequence_types):
            return

        # If none of the above passed, types are incompatible
        raise TypeError(
            f"Cannot construct condition from incompatible types: "
            f"Topic attribute type is '{type(val).__name__}', "
            f"and reference type is '{type(other).__name__}'"
        )

    def _make_condition(self, op: Callable, value: Any) -> Condition:
        return Condition(
            topic_source=self._topic,
            attribute_path=self._base,
            operator_func=op,
            ref_value=value,
        )

    # --- Dunder Methods for Conditional Parsing ---

    def __eq__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.equals, other)

    def __ne__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.not_equals, other)

    def __lt__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.less_than, other)

    def __le__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.less_or_equal, other)

    def __gt__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.greater_than, other)

    def __ge__(self, other) -> Condition:
        self._check_similar_type(other)
        return self._make_condition(ConditionOperators.greater_or_equal, other)

    def __invert__(self) -> Condition:
        """
        Shorthand for checking if a boolean is False.
        Usage: ~topic.msg.is_enabled  (Equivalent to is_enabled == False)
        """
        self._check_similar_type(False)
        # You can map this to == False, or a specific NOT operator if you have one
        return self._make_condition(ConditionOperators.equals, False)

    def is_in(self, other: Union[List, tuple, str]) -> Condition:
        """
        Check if the topic value is inside the provided list/tuple.
        Usage: topic.msg.status.is_in([1, 2, 3])
        """
        # We use operator.contains.
        # When Event executes: operator.contains(Operand(msg_val), [1,2,3])
        # This calls Operand.__contains__([1,2,3])
        if isinstance(other, str):
            self._check_similar_type(other)
            return self._make_condition(ConditionOperators.is_in, other)
        return self._make_condition(ConditionOperators.is_in, other)

    def not_in(self, other: Union[List, tuple, str]) -> Condition:
        """
        Check if the topic value is NOT inside the provided list/tuple.
        Usage: topic.msg.status.not_in([0, -1])
        """
        if isinstance(other, str):
            self._check_similar_type(other)
            return self._make_condition(ConditionOperators.not_in, other)

        return self._make_condition(ConditionOperators.not_in, other)

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
        return self._make_condition(ConditionOperators.contains_any, other)

    def contains_all(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value contains ALL of the values in 'other'.
        Equivalent to: set(other).issubset(set(topic))
        """
        return self._make_condition(ConditionOperators.contains_all, other)

    def contains(self, other: Union[List, tuple, str]) -> Condition:
        """
        If value contains another string
        If other is a list: works same as contains_all
        """
        # We use operator.contains.
        # When Event executes: operator.contains(Operand(msg_val), "some string")
        if isinstance(other, str):
            self._check_similar_type(other)
            return self._make_condition(ConditionOperators.contains, other)
        return self.contains_all(other)

    def not_contains_any(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value contains NONE of the values in 'other'.
        Equivalent to: set(topic).isdisjoint(set(other))
        """
        return self._make_condition(ConditionOperators.not_contains_any, other)

    def not_contains_all(self, other: Union[List, tuple]) -> Condition:
        """
        True if the topic value is MISSING at least one value from 'other'.
        Inverse of contains_all.
        """
        return self._make_condition(ConditionOperators.not_contains_all, other)

    def not_contains(self, other: Union[List, tuple, str]) -> Condition:
        """
        If value does not contain another string
        If other is a list: works same as not_contains_all
        """
        # We use operator.contains.
        # When Event executes: operator.contains(Operand(msg_val), "some string")
        if isinstance(other, str):
            self._check_similar_type(other)
            return self._make_condition(ConditionOperators.not_contains, other)
        return self.not_contains_all(other)

    def is_true(self) -> Condition:
        """
        Create a condition checking if the boolean attribute is True.
        Usage: topic.msg.is_enabled.is_true()
        """
        # Ensure we are comparing against a boolean type in the msg
        self._check_similar_type(True)
        return self._make_condition(ConditionOperators.equals, True)

    def is_false(self) -> Condition:
        """
        Create a condition checking if the boolean attribute is False.
        Usage: topic.msg.is_enabled.is_false()
        """
        self._check_similar_type(False)
        return self._make_condition(ConditionOperators.equals, False)

    def __getitem__(self, key: int):
        """
        Allow array indexing in the path.
        Usage: topic.msg.data[0] > 5
        """
        if not isinstance(key, int):
            raise TypeError("Only integer indices are supported for arrays.")

        # Add the integer index to the path
        augmented_base = self._base + [key]

        # Validation Logic (Optional but recommended)
        # We need to temporarily check if the current object is indexable
        # This is harder to validate statically with types, but we can try:
        # start = self._get_value_at_current_path()
        # if not hasattr(start, '__getitem__'): error...

        return MsgConditionBuilder(self._topic, augmented_base)
