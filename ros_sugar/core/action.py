"""Actions"""

import inspect
import json
import numpy as np
from rclpy.lifecycle import Node as LifecycleNode
from launch.actions import OpaqueCoroutine, OpaqueFunction
from functools import partial, wraps
import array
from typing import (
    Any,
    Callable,
    Dict,
    Optional,
    Union,
    List,
    Type,
    get_origin,
    get_args,
)

from launch import LaunchContext
import launch
from launch.actions import LogInfo as LogInfoROSAction

from ..launch import logger
from ..condition import MsgConditionBuilder


def _create_auto_topic_parser(input_msg_type: Type, target_type: Type) -> Callable:
    """
    Factory function to create an automatic parser from a ROS message type
    to a target type (either another ROS message or a Python primitive/structure).

    :param input_msg_type: The source ROS message class.
    :param target_type: The destination type (ROS msg class, int, float, list, etc.).
    :return: A callable parser function: (msg=...) -> target_type
    """
    if not hasattr(input_msg_type, "get_fields_and_field_types"):
        raise ValueError(
            f"Cannot create an automatic parser. 'input_msg_type' should be a valid ROS2 message type, but got {input_msg_type}"
        )
    # Check if target is a ROS Message (has field types method)
    if hasattr(target_type, "get_fields_and_field_types"):
        return _create_auto_ros_msg_parser(input_msg_type, target_type)
    else:
        return _parse_to_python_type(input_msg_type, target_type)


def _get_ros_field_type_map(msg_cls: Type) -> Dict[str, str]:
    """Helper to safely get field types map from a ROS message class."""
    if hasattr(msg_cls, "get_fields_and_field_types"):
        return msg_cls.get_fields_and_field_types()
    return {}


def _is_compatible_primitive(ros_type_str: str, python_type: Type) -> bool:
    """
    Check if a ROS type string (e.g., 'int32', 'boolean') is compatible with a Python type.
    """
    # Clean ROS type string (remove 'sequence<...>', 'string<...>')
    base_ros_type = ros_type_str.split("<")[0]

    # Mapping ROS string types to Python types
    type_map = {
        "boolean": bool,
        "bool": bool,
        "octet": (int, bytes),
        "float": float,
        "double": float,
        "int": int,
        "uint": int,
        "string": str,
        "char": (int, str),
    }

    # Handle standard python types
    for key, val in type_map.items():
        if key in base_ros_type:
            if isinstance(val, tuple):
                if python_type in val:
                    return True
            elif python_type == val:
                return True

            # Allow float -> int (if desired, though risky for precision) or int -> float
            if python_type is float and val is int:
                return True

            # Allow numpy types
            if python_type.__module__ == "numpy":
                if np.issubdtype(python_type, np.integer) and val is int:
                    return True
                if np.issubdtype(python_type, np.floating) and val is float:
                    return True

    return False


def _parse_to_python_type(input_msg_type: Type, python_type: Type) -> Callable:
    """
    Creates a parser to extract a standard Python type (int, float, list, etc.)
    from a ROS message.
    """
    msg_dummy = input_msg_type()

    # 0. Handle 'Any' or specific ignore cases if needed
    if python_type == Any:
        return lambda *, msg, **_: msg

    # Inspect ROS message fields
    fields = _get_ros_field_type_map(input_msg_type)

    # 2. Single Field Extraction (The "data" pattern)
    # Common in std_msgs (e.g. Int32.data, String.data) or simple messages.
    # We look for a field that matches the target python type.

    candidates = []

    # Handle generic List[T]
    origin_type = get_origin(python_type)
    args_type = get_args(python_type)
    is_list_target = origin_type in (list, List) or python_type is list
    is_numpy_target = inspect.isclass(python_type) and issubclass(
        python_type, np.ndarray
    )

    for field_name, field_ros_type in fields.items():
        # Get the actual python attribute value from the dummy message to check its type
        # This is more reliable than parsing the ROS string type manually
        field_value = getattr(msg_dummy, field_name)

        # A. Exact Python Type Match
        if type(field_value) is python_type:
            candidates.append(field_name)
            continue

        # B. Compatible Primitives (e.g. ROS int32 -> Python int)
        if _is_compatible_primitive(field_ros_type, python_type):
            candidates.append(field_name)
            continue

        # C. List / Array Handling
        if is_list_target or is_numpy_target:
            # Check if field is a sequence/array
            if isinstance(field_value, (list, tuple, np.ndarray, array.array)):
                # If target is generic List (no subtypes specified), accept it
                if not args_type and is_list_target:
                    candidates.append(field_name)
                # If target is specific List[int], check inner types if list is not empty
                # (Hard to check on empty dummy msg, so we rely on ROS string type usually)
                elif is_list_target and args_type:
                    # Check if 'sequence' or '[' is in ros type definition
                    if "sequence" in field_ros_type or "[" in field_ros_type:
                        # Heuristic: assume it matches if the container matches
                        candidates.append(field_name)
                elif is_numpy_target:
                    candidates.append(field_name)

    # Decision Logic
    if len(candidates) == 1:
        field_name = candidates[0]
        logger.info(
            f"Auto-parser created: Extracting field '{field_name}' from '{input_msg_type.__name__}' "
            f"to satisfy target '{python_type.__name__ if hasattr(python_type, '__name__') else str(python_type)}'."
        )

        def extraction_parser(*, msg: Any, **_) -> Any:
            val = getattr(msg, field_name)

            # Post-processing for List/Numpy conversions
            if is_list_target and isinstance(val, (np.ndarray, array.array)):
                return val.tolist()
            if is_numpy_target and isinstance(val, list):
                return np.array(val)
            if is_numpy_target and isinstance(val, np.ndarray):
                return val  # Already numpy

            return val

        return extraction_parser

    elif len(candidates) > 1:
        # Ambiguity - prefer 'data' if it exists (standard ROS convention)
        if "data" in candidates:
            logger.warning(
                f"Ambiguous fields {candidates} found for target type '{python_type}'. "
                f"Defaulting to 'data' field."
            )
            return lambda *, msg, **_: getattr(msg, "data", None)

        raise ValueError(
            f"Auto-parsing ambiguous: Multiple fields {candidates} in '{input_msg_type.__name__}' "
            f"match target type '{python_type}'. Please provide a custom parser."
        )

    # 3. Last Resort: Recursion / Duck Typing for wrapped types
    # e.g., input: Wrapper(data=Int32), target: int
    # This is complex to implement generically without infinite recursion risks,
    # but we can try 1 level deep if there is only 1 field in the message.
    if len(fields) == 1:
        single_field = list(fields.keys())[0]
        field_val = getattr(msg_dummy, single_field)

        # If the single field is a ROS Message, try to recurse
        if hasattr(field_val, "get_fields_and_field_types"):
            try:
                # Recursively create a parser from that inner field to our target
                inner_parser = _parse_to_python_type(type(field_val), python_type)

                logger.info(
                    f"Auto-parser recursive: unwrapping '{single_field}' to match target."
                )

                def recursive_parser(*, msg: Any, **kwargs) -> Any:
                    inner_msg = getattr(msg, single_field)
                    return inner_parser(msg=inner_msg, **kwargs)

                return recursive_parser
            except ValueError:
                pass  # Recursion failed, continue to raise error

    raise ValueError(
        f"Auto-parsing failed: No field in '{input_msg_type.__name__}' matches Python type "
        f"'{python_type}'. Matches attempted on fields: {list(fields.keys())}."
    )


def _create_auto_ros_msg_parser(
    input_msg_type: Type, target_msg_type: Type
) -> Callable:
    """
    Creates a parser to map one ROS message type to another ROS message type.
    Logic:
    1. Exact Match.
    2. Duck Typing (Same field names & types).
    3. Type Matching (Different names, same types - heuristic).
    """
    msg_dummy = input_msg_type()

    # Case 1: Direct Type Match
    if isinstance(msg_dummy, target_msg_type):
        return lambda *, msg, **_: msg

    target_fields = _get_ros_field_type_map(target_msg_type)
    input_fields = _get_ros_field_type_map(input_msg_type)

    # Case 2: Duck Typing (Name & Type Match)
    # Check if all fields in target exist in input with same type string
    common_fields = []
    for key, f_type in target_fields.items():
        if key in input_fields and input_fields[key] == f_type:
            common_fields.append(key)

    # If we found matches for ALL target fields, this is a strong match subset
    # Or if we found matches for ALL input fields
    # Let's be strict: If target has fields, we need to fill them.
    # If target is fully covered by input:
    if (len(common_fields) == len(target_fields) and len(target_fields) > 0) or (
        len(common_fields) == len(input_fields)
        and len(input_fields) < len(target_fields)
    ):
        logger.info(
            f"Added name-based parser from '{input_msg_type.__name__}' to '{target_msg_type.__name__}'."
        )

        def duck_parser(*, msg: Any, **_) -> Any:
            out = target_msg_type()
            for key in common_fields:
                setattr(out, key, getattr(msg, key))
            return out

        return duck_parser

    # Case 3: Type-based matching (Heuristic)
    # If names don't match, but types do uniquely
    # e.g. Input(x: float), Target(data: float)
    matching_map = {}  # target_field -> input_field
    used_inputs = set()

    possible_match = True

    for tgt_key, tgt_type in target_fields.items():
        found_source = None
        for inp_key, inp_type in input_fields.items():
            if inp_key in used_inputs:
                continue
            if inp_type == tgt_type:
                found_source = inp_key
                break

        if found_source:
            matching_map[tgt_key] = found_source
            used_inputs.add(found_source)
        else:
            possible_match = False
            break

    if possible_match and matching_map:
        logger.warning(
            f"Added type-based parser (heuristic) from '{input_msg_type.__name__}' to '{target_msg_type.__name__}' "
            f"mapping: {matching_map}. Verify this logic."
        )

        def type_parser(*, msg: Any, **_) -> Any:
            out = target_msg_type()
            for t_key, i_key in matching_map.items():
                setattr(out, t_key, getattr(msg, i_key))
            return out

        return type_parser

    # Case 4: No Match
    raise ValueError(
        f"Auto-parsing failed: Could not map ROS message '{input_msg_type.__name__}' "
        f"to target ROS message '{target_msg_type.__name__}'."
    )


class Action:
    """
    Actions are used by Components and by the Launcher to execute specific methods.

    Actions can either be:
    - Actions paired with Events: in this case the Action is executed by a Launcher when an event is detected
    - Actions paired with Fallbacks: in this case the Action is executed by a Component when a failure is detected

    Actions are defined with:
    - method (Callable)
    - args: Arguments to be passed to the method when executing the action
    - kwargs: Keyword arguments to be passed to the method when executing the action

    ## Usage Example:
    ```python
        from ros_sugar.component import BaseComponent
        from ros_sugar.config import BaseComponentConfig

        def function():
            print("I am executing action!")

        my_component = BaseComponent(node_name='test_component')
        new_config = BaseComponentConfig(loop_rate=50.0)
        action1 = Action(method=my_component.start)
        action2 = Action(method=my_component.reconfigure, args=(new_config, True)),)
        action3 = Action(method=function)
    ```
    """

    def __init__(
        self, method: Callable, args: tuple = (), kwargs: Optional[Dict] = None
    ) -> None:
        """
        Action

        :param method: Action function
        :type method: callable
        :param args: function arguments, defaults to ()
        :type args: tuple, optional
        :param kwargs: function keyword arguments, defaults to {}
        :type kwargs: dict, optional
        """
        self.__component_action: bool = False
        self.__parent_component: Optional[str] = None
        self.__action_keyname: Optional[str] = (
            None  # contains the name of the component action as a string
        )
        self._is_monitor_action: bool = False
        self._is_lifecycle_action: bool = False
        self._function = method
        self._args = args if isinstance(args, tuple) else (args,)
        self._kwargs = kwargs if kwargs else {}

        # List of registered parsers to execute before the main method
        # Each entry is a dict: {'method': Callable, 'output_mapping': str|None, 'arg_index': int|None}
        # TODO: Handle parser method execution in multi-processing
        self._event_parsers: List[Dict[str, Any]] = []
        self.__parsed_topics: Dict[str, type] = {}

        self.__verify_args_kwargs()

        # Check if it is a component action and update parent and keyname
        if hasattr(self._function, "__self__"):
            action_object = self._function.__self__

            if hasattr(action_object, "node_name") and isinstance(
                action_object, LifecycleNode
            ):
                self.parent_component = action_object.node_name
                self.action_name = self._function.__name__
                self.__component_action = True

    def __verify_args_kwargs(self):
        """
        Verify that args and kwargs are correct for the action executable

        :raises ValueError: If args or kwargs are not compatible with the action executable
        """
        _topic_parsing: Dict[Union[int, str], MsgConditionBuilder] = {}
        function_parameters = inspect.signature(self.executable).parameters

        # Check args
        if len(list(self.args)) > len(function_parameters):
            raise ValueError(
                f"Too many arguments provided for action '{self.action_name}': expected maximum {len(function_parameters)}, got {len(self.args)}"
            )

        for idx, value in enumerate(self.args):
            if isinstance(value, MsgConditionBuilder):
                _topic_parsing[idx] = value

        # Check kwargs
        for key, value in self.kwargs.items():
            if isinstance(value, MsgConditionBuilder):
                _topic_parsing[key] = value

        for position, topic_path in _topic_parsing.items():
            topic_msg_attributes = topic_path.as_tuple()
            self.__parsed_topics[topic_path.name] = topic_path.type

            # Parser method to extract attribute from msg
            def parser_method(topic_msg_attributes, msg, **_):
                attribute_value = msg
                for attr in topic_msg_attributes:
                    attribute_value = getattr(attribute_value, attr)
                return attribute_value

            if isinstance(position, int):
                # Positional argument replacement
                self.add_event_parser(
                    method=partial(parser_method, topic_msg_attributes),
                    argument_index=position,
                )
            else:
                # Keyword argument replacement
                self.add_event_parser(
                    method=partial(parser_method, topic_msg_attributes),
                    keyword_argument_name=position,
                )

    def __call__(self, **kwargs):
        """
        Execute the action.
        Iterates through all parsers to prepare dynamic arguments based on the event (kwargs).
        """
        # Create mutable copies of args and kwargs for this specific execution
        call_args = list(self.args)
        call_kwargs = self.kwargs.copy()

        # Values to prepend to args (legacy support for parsers with no mapping)
        # Using a list to collect them, then we will prepend them in reverse order or bulk
        prepend_values = []

        # Iterate over all registered parsers
        for parser in self._event_parsers:
            method = parser["method"]
            output_mapping = parser["keyword_argument_name"]
            arg_index = parser["argument_index"]

            # Execute the parser method with the event context (e.g. msg)
            try:
                output = method(**kwargs)
            except TypeError:
                # Fallback if parser doesn't accept kwargs, though parsers created internally handle it
                output = method()

            if output_mapping is not None:
                # Update specific keyword argument
                call_kwargs[output_mapping] = output
            elif arg_index is not None:
                # Replace specific positional argument (previously a placeholder)
                # Ensure we don't go out of bounds
                if arg_index < len(call_args):
                    call_args[arg_index] = output
                else:
                    raise IndexError(
                        f"Parser argument index {arg_index} out of range for args {call_args}"
                    )
            else:
                # No specific target: Prepend to arguments (Stack behavior)
                prepend_values.append(output)

        # Assemble final arguments
        final_args = tuple(prepend_values) + tuple(call_args)

        return self.executable(*final_args, **call_kwargs)

    def add_event_parser(
        self,
        method: Callable,
        keyword_argument_name: Optional[str] = None,
        argument_index: Optional[int] = None,
        **new_kwargs,
    ):
        """
        Add an event parser to the action.

        :param method: Method to be executed before the main action executable
        :param output_mapping: Name of the keyword argument to replace/inject
        :param arg_index: Index of the positional argument to replace
        :param new_kwargs: Additional static kwargs to add to the action configuration
        """
        if new_kwargs:
            self.kwargs.update(new_kwargs)

        self._event_parsers.append({
            "method": method,
            "keyword_argument_name": keyword_argument_name,
            "argument_index": argument_index,
        })

    def _set_dynamic_argument_types(self, types: Dict[str, Union[Type, List[Type]]]):
        """Used to set the missing (dynamic) argument type in derived pre-built actions

        :param types: Required types by the action method
        :type types: Dict[str, Union[Type, List[Type]]]
        """
        self._dynamic_argument_types = types

    def add_automatic_parser_from_msg_type(
        self,
        input_topic_msg_type: Type,
        action_argument_type: Optional[Type] = None,
        keyword_argument_name: Optional[str] = None,
        arg_index: Optional[int] = None,
        **new_kwargs,
    ):
        """
        Add an automatic parser from a given topic to the required dynamic type

        :param input_topic_msg_type: Type of the input topic message (to be parsed to the dynamic argument type)
        :param keyword_argument_name: Name of the keyword argument to replace/inject
        :param arg_index: Index of the positional argument to replace
        :param new_kwargs: Additional static kwargs to add to the action configuration
        """
        if not hasattr(self, "_dynamic_argument_types") and not action_argument_type:
            logger.debug(
                "Action does not have any required 'dynamic_argument_types' and 'action_argument_type' is None. Skipping automatic parser..."
            )
            return

        if action_argument_type:
            parser_method = _create_auto_topic_parser(
                input_msg_type=input_topic_msg_type,
                target_msg_type=action_argument_type,
            )
        else:
            if keyword_argument_name:
                dynamic_type = self._dynamic_argument_types.get(
                    keyword_argument_name, None
                )
            elif arg_index is not None:
                dynamic_type = self._dynamic_argument_types.values()[arg_index]
            else:
                # Get the first type by default
                dynamic_type = self._dynamic_argument_types.values()[0]
            parser_method = _create_auto_topic_parser(
                input_msg_type=input_topic_msg_type, target_msg_type=dynamic_type
            )
        self.add_event_parser(
            method=parser_method,
            keyword_argument_name=keyword_argument_name,
            argument_index=arg_index,
            **new_kwargs,
        )

    def _verify_against_event_topic(self, event_topic) -> None:
        """Verify the action topic parsers (if present) against an event.
           Raises a 'ValueError' if there is a mismatch.

        :param event_topic: Event topic to verify against
        :type event_topic: Topic
        :raises ValueError: If the action parser topics are different from the event topic
        """
        # TODO: Support additional subscribers to required topics other than the event topic?
        # For example: If battery is low event triggers an action that reads from /odom topic?
        for topic_name, topic_ros_type in self.__parsed_topics.items():
            if (
                topic_name != event_topic.name
                or topic_ros_type != event_topic.ros_msg_type
            ):
                raise ValueError(
                    f"Action parser topic mismatch. Actions only have access to associated events topics. Action '{self.action_name}' is associated with event topic '{event_topic.name}', but got topic '{topic_name}'"
                )

    @property
    def executable(self):
        """
        Get the action callable

        :return: _description_
        :rtype: _type_
        """
        return self._function

    @executable.setter
    def executable(self, value: Callable):
        """
        Getter of action executable

        :param value: _description_
        :type value: Callable
        """
        self._function = value

    @property
    def args(self):
        """
        Getter of action arguments

        :return: _description_
        :rtype: _type_
        """
        return self._args

    @property
    def kwargs(self):
        """
        Getter of action keyword arguments

        :return: _description_
        :rtype: _type_
        """
        return self._kwargs

    @property
    def parent_component(self):
        """
        Getter of parent component class name if it is a component action, else None

        :return: _description_
        :rtype: str | None
        """
        return self.__parent_component

    @parent_component.setter
    def parent_component(self, component_name: str):
        """
        Setter of parent component name

        :param component_name: _description_
        :type component_name: str
        """
        self.__parent_component = component_name

    @property
    def action_name(self) -> str:
        """
        Getter of the action name
        Equals exact executable name if it is not a component action
        Equals method name in the component if it is a component action

        :return: _description_
        :rtype: str
        """
        return self.__action_keyname or self._function.__name__

    @action_name.setter
    def action_name(self, value: str) -> None:
        """
        Getter of action name

        :param value: _description_
        :type value: str
        """
        self.__action_keyname = value

    @property
    def component_action(self) -> bool:
        """component_action.

        :rtype: bool
        """
        return self.__component_action

    @component_action.setter
    def component_action(self, value: bool) -> None:
        """component_action.

        :param value:
        :type value: bool
        :rtype: None
        """
        self.__component_action = value

    @property
    def monitor_action(self) -> bool:
        return self._is_monitor_action

    @property
    def lifecycle_action(self) -> bool:
        return self._is_lifecycle_action

    @property
    def dictionary(self) -> Dict:
        """
        Property to get/set the event using a dictionary

        :return: Event description dictionary
        :rtype: Dict
        """
        # TODO: Handle parsers?
        return {
            "action_name": self.action_name,
            "parent_name": self.parent_component,
            "args": self.args,
            "kwargs": self.kwargs,
        }

    @property
    def json(self) -> str:
        """
        Property to get/set the event using a json

        :return: Event description dictionary as json
        :rtype: str
        """
        json_dict = self.dictionary
        return json.dumps(json_dict)

    def launch_action(
        self, monitor_node=None
    ) -> Union[OpaqueCoroutine, OpaqueFunction]:
        """
        Get the ros launch action

        :return: _description_
        :rtype: OpaqueCoroutine | OpaqueFunction
        """
        # Check if it is a stack action and update the executable from the monitor node
        if self.monitor_action and monitor_node:
            if not hasattr(monitor_node, self.action_name):
                raise ValueError(f"Unknown stack action: {self.action_name}")
            # Get executable from monitor
            self.executable = getattr(monitor_node, self.action_name)

        elif self.monitor_action and not monitor_node:
            raise ValueError("Monitor node should be provided to parse stack action")

        function_parameters = inspect.signature(self.executable).parameters

        # Wrap the function to add LaunchContext attribute to it required by ROS Launch
        @wraps(self.executable)
        def new_function(_: LaunchContext, *args, **kwargs):
            """
            Create new_function - Add context + No return from original function

            :param context: ROS Launch Context
            :type context: LaunchContext
            """
            self.executable(*args, **kwargs)

        # HACK: Update function signature - as ROS Launch uses inspect to check signature
        new_parameters = list(function_parameters.values())
        new_parameters.insert(
            0,
            inspect.Parameter(
                "context",
                kind=inspect.Parameter.POSITIONAL_ONLY,
                annotation=LaunchContext,
            ),
        )
        new_function.__signature__ = inspect.signature(self.executable).replace(
            parameters=new_parameters
        )

        if inspect.iscoroutine(self.executable):
            return OpaqueCoroutine(
                coroutine=new_function, args=self._args, kwargs=self._kwargs
            )
        else:
            return OpaqueFunction(
                function=new_function, args=self._args, kwargs=self._kwargs
            )


class LogInfo(LogInfoROSAction):
    """Overrides the LogInfo Action for ros2 launch to change the hard-codded logger name

    :param LogInfoROSAction: Action that logs a message when executed
    :type LogInfoROSAction: LogInfoROSAction
    """

    def __init__(self, *, msg: str, logger_name: Optional[str] = None, **kwargs):
        """Setup the LogInfo Action

        :param msg: Logged message
        :type msg: str
        :param logger_name: Logger name, defaults to None. If not provided the message is logged with the package logger name 'Launcher'
        :type logger_name: Optional[str], optional
        """
        super().__init__(msg=msg, **kwargs)
        if logger_name:
            self.__logger = launch.logging.get_logger(logger_name)
        else:
            self.__logger = logger

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        self.__logger.info(
            "".join([context.perform_substitution(sub) for sub in self.msg])
        )
        return None
