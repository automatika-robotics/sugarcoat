"""Actions"""

import inspect
import json
from rclpy.lifecycle import Node as LifecycleNode
from launch.actions import OpaqueCoroutine, OpaqueFunction
from functools import partial, wraps
from typing import Any, Callable, Dict, Optional, Union, List, Type

from launch import LaunchContext
import launch
from launch.actions import LogInfo as LogInfoROSAction

from ..launch import logger
from ..utils import _MsgConditionBuilder


def _create_auto_topic_parser(input_msg_type: Type, target_msg_type: Type) -> Callable:
    """Creates a parser function that attempts to convert an input_msg_type
    into the target_msg_type.

    :param input_msg_type: Input message type
    :type input_msg_type: Type
    :param target_msg_type: Target type after parsing
    :type target_msg_type: Type
    :raises ValueError: If not automatic parsing is found

    :return: automatic parsing method
    :rtype: Callable
    """
    target_msg = target_msg_type()
    msg = input_msg_type()

    # Case 1: Direct Type Match
    # If the input message is exactly the target type, return it directly.
    if isinstance(msg, target_msg_type):
        logger.info(
            f"Added direct types match parser from topic message type '{input_msg_type.__name__}' to target type '{target_msg_type.__name__}'"
        )
        return lambda *, msg, **_: msg

    # Otherwise, Inspect fields
    target_msg_fields_dict = target_msg.get_fields_and_field_types()
    input_msg_fields_dict = msg.get_fields_and_field_types()
    matches_found = True

    # Case 2: Same Field Names and Types (Duck Typing)
    for key, field_type in input_msg_fields_dict.items():
        if (
            key not in target_msg_fields_dict
            or field_type != target_msg_fields_dict[key]
        ):
            matches_found = False
            break

    if matches_found:
        logger.info(
            f"Added direct types parser from topic message type '{input_msg_type.__name__}' to target type '{target_msg_type.__name__}'"
        )

        # Define the duck taping parser
        def auto_parser(*, msg: Any, **_) -> Any:
            for key in input_msg_fields_dict.keys():
                setattr(target_msg, key, getattr(msg, key))
            return target_msg

        return auto_parser

    matching_dict = {}
    matches_found = True
    # Case 3: Different Field Names but same unique Types
    for key, field_type in input_msg_fields_dict.items():
        if key not in target_msg_fields_dict:
            # Check if any field in input has the same type
            type_matched = False
            for out_key, in_field_type in target_msg_fields_dict.items():
                if (
                    field_type == in_field_type
                    and out_key not in matching_dict.values()
                ):
                    matching_dict[key] = out_key
                    type_matched = True
                    break
            if not type_matched:
                matches_found = False
                break

    if matches_found:
        logger.warning(
            f"Added type-based parser from topic message type '{input_msg_type.__name__}' to target type '{target_msg_type.__name__}' matching the following fields: {matching_dict}. Re-write a custom parser if a different mapping is desired."
        )

        # Define the type-based parser
        def auto_parser(*, msg: Any, **_) -> Any:
            for key, out_key in matching_dict.items():
                setattr(target_msg, out_key, getattr(msg, key))
            return target_msg

        return auto_parser

    # Case 4: No Match Found
    raise ValueError(
        f"Auto-parsing failed: Could not map input message of type '{input_msg_type.__name__}' "
        f"to target action/service type '{target_msg_type.__name__}'. "
        f"Fields do not match by name or type. Please provide a custom parser method that takes in an input 'msg' of type '{input_msg_type.__name__}' and return a parsed message of type '{target_msg_type.__name__}'."
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
        _topic_parsing: Dict[Union[int, str], _MsgConditionBuilder] = {}
        function_parameters = inspect.signature(self.executable).parameters

        # Check args
        if len(list(self.args)) > len(function_parameters):
            raise ValueError(
                f"Too many arguments provided for action '{self.action_name}': expected maximum {len(function_parameters)}, got {len(self.args)}"
            )

        for idx, value in enumerate(self.args):
            if isinstance(value, _MsgConditionBuilder):
                _topic_parsing[idx] = value

        # Check kwargs
        for key, value in self.kwargs.items():
            if isinstance(value, _MsgConditionBuilder):
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
                    arg_index=position,
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
            output_mapping = parser["output_mapping"]
            arg_index = parser["arg_index"]

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
        arg_index: Optional[int] = None,
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
            "arg_index": arg_index,
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
            arg_index=arg_index,
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
