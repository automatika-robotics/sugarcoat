"""Component Actions"""

from functools import wraps
from typing import Any, Callable, List, Optional, Union, Type

from .action import Action
from .component import BaseComponent
from ..utils import InvalidAction
from ..io.topic import Topic
from .action import LogInfo
from ..launch import logger


def _validate_component_action(function: Callable):
    """
    Decorator for to validate that a given action is a supported component action

    :param function:
    :type function: Callable
    """

    # NOTE: Although the validator is used as a decorated for methods taking only keyword arguments, *args is added to get cls/self arguments
    @wraps(function)
    def _wrapper(*args, **kwargs):
        """_summary_

        :param component: _description_
        :type component: BaseComponent
        :raises TypeError: _description_
        :return: _description_
        :rtype: _type_
        """
        if not kwargs.get("component"):
            raise InvalidAction(
                f"Component should be provided to use component action '{function.__name__}'"
            )
        component: Optional[BaseComponent] = kwargs.get("component")

        if component and not hasattr(component, function.__name__):
            raise InvalidAction(
                f"Component '{component.node_name}' does not support '{function.__name__}' action"
            )

        return function(*args, **kwargs)

    return _wrapper


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
        return lambda *, msg: msg

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


class ComponentActions:
    """Pre-defined component level actions and system level actions

    ## Component-level Actions:
    - stop
    - start
    - restart
    - reconfigure
    - update_parameter
    - update_parameters

    ## System-level Actions:
    - log
    - publish_message
    - send_srv_request
    - send_action_goal

    ## Usage Example:
    ```python
        from ros_sugar.actions import Actions

        my_component = BaseComponent(node_name='test_component')
        action1 = Actions.start(component=my_component)
        action2 = Actions.log(msg="I am executing a cool action!")
    ```

    """

    @classmethod
    def send_srv_request(
        cls, *, srv_name: str, srv_type: type, srv_request_msg: Any
    ) -> Action:
        """Action to send a ROS2 service request using the Monitor

        :param srv_name: Service name
        :type srv_name: str
        :param srv_type: Service type (ROS2 service)
        :type srv_type: type
        :param srv_request_msg: Service request message
        :type srv_request_msg: Any

        :return: Sending request action
        :rtype: Action
        """
        # Combine positional arguments and keyword arguments
        kwargs = {
            "srv_name": srv_name,
            "srv_type": srv_type,
            "srv_request_msg": srv_request_msg,
        }
        # Action with an empty callable
        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "send_srv_request"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def send_srv_request_from_topic(
        cls,
        *,
        srv_name: str,
        srv_type: type,
        topic: Any,
        custom_parser: Optional[Callable] = None,
    ) -> "Action":
        """
        Creates an Action to send a ROS2 service request, automatically parsing the
        request from the event topic message.

        :param srv_name: Service name
        :param srv_type: Service type
        :param topic: The topic object containing the message to parse
        :param custom_parser: Optional custom parser method
        :return: Sending request action with parser attached
        """
        request_msg_type = srv_type.Request

        # Create the base action
        action = cls.send_srv_request(
            srv_name=srv_name,
            srv_type=srv_type,
            srv_request_msg=request_msg_type(),  # Default empty request
        )

        input_msg_type = topic.ros_msg_type

        # Use provided parser if provided, otherwise create an automatic parser
        auto_parser_method = custom_parser or _create_auto_topic_parser(
            input_msg_type, request_msg_type
        )

        # Attach the parser
        action.event_parser(auto_parser_method, output_mapping="srv_request_msg")

        return action

    @classmethod
    def send_action_goal(
        cls, *, action_name: str, action_type: type, action_request_msg: Any
    ) -> Action:
        """Action to send a ROS2 action goal using the Monitor

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any

        :return: Sending goal action
        :rtype: Action
        """
        # Combine positional arguments and keyword arguments
        kwargs = {
            "action_name": action_name,
            "action_type": action_type,
            "action_request_msg": action_request_msg,
        }

        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "send_action_goal"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def send_action_goal_from_topic(
        cls,
        *,
        action_name: str,
        action_type: type,
        topic: Any,
        custom_parser: Optional[Callable] = None,
    ) -> "Action":
        """
        Creates an Action to send a ROS2 action goal, automatically parsing the
        goal from the event topic message.

        :param action_name: Action name
        :param action_type: Action type
        :param topic: The topic object containing the message to parse
        :param custom_parser: Optional custom parser method
        :return: Sending request action with parser attached
        """
        request_msg_type = action_type.Goal

        # Create the base action
        action = cls.send_action_goal(
            action_name=action_name,
            action_type=action_type,
            action_request_msg=request_msg_type(),  # Default empty request
        )

        input_msg_type = topic.ros_msg_type

        # Use provided parser if provided, otherwise create an automatic parser
        auto_parser_method = custom_parser or _create_auto_topic_parser(
            input_msg_type, request_msg_type
        )

        # Attach the parser
        action.event_parser(auto_parser_method, output_mapping="action_request_msg")

        return action

    @classmethod
    def publish_message(
        cls,
        *,
        topic: Topic,
        msg: Any,
        publish_rate: Optional[float] = None,
        publish_period: Optional[float] = None,
    ) -> Action:
        """Action to send a ROS2 action goal using the Monitor

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any

        :return: Sending goal action
        :rtype: Action
        """
        # Combine positional arguments and keyword arguments
        kwargs = {
            "topic": topic,
            "msg": msg,
            "publish_rate": publish_rate,
            "publish_period": publish_period,
        }

        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "publish_message"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def publish_message_from_parsed_topic(
        cls,
        *,
        in_topic: Topic,
        out_topic: Topic,
        publish_rate: Optional[float] = None,
        publish_period: Optional[float] = None,
        custom_parser: Optional[Callable] = None,
    ) -> Action:
        """Action to send a ROS2 action goal using the Monitor

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any

        :return: Sending goal action
        :rtype: Action
        """
        # Combine positional arguments and keyword arguments
        kwargs = {
            "topic": out_topic,
            "msg": out_topic.ros_msg_type(),
            "publish_rate": publish_rate,
            "publish_period": publish_period,
        }

        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "publish_message"
        stack_action._is_monitor_action = True

        # Create the automatic parser logic
        parser_method = custom_parser or _create_auto_topic_parser(
            input_msg_type=in_topic.ros_msg_type,
            target_msg_type=out_topic.ros_msg_type,
        )
        stack_action.event_parser(parser_method, output_mapping="msg")

        return stack_action

    @classmethod
    @_validate_component_action
    def start(cls, *, component: BaseComponent) -> Action:
        """Action to start a given component

        :param component: Component
        :type component: BaseComponent

        :return: Component start action
        :rtype: Action
        """
        # Action with an empty callable
        stack_action = Action(method=component.start)
        stack_action._is_lifecycle_action = True
        return stack_action

    @classmethod
    @_validate_component_action
    def stop(cls, *, component: BaseComponent) -> Action:
        """Action to stop a given component

        :param component: Component
        :type component: BaseComponent

        :return: Component stop action
        :rtype: Action
        """
        # Action with an empty callable
        stack_action = Action(method=component.stop)
        stack_action._is_lifecycle_action = True
        return stack_action

    @classmethod
    @_validate_component_action
    def restart(
        cls, *, component: BaseComponent, wait_time: Optional[float] = None
    ) -> Action:
        """Action to restart a given component

        :param component: Component
        :type component: BaseComponent
        :param wait_time: Optional wait time n seconds between stop and start, defaults to None (i.e. no wait)
        :type wait_time: Optional[float]

        :return: Component restart action
        :rtype: Action
        """
        # Action with an empty callable
        stack_action = Action(method=component.restart, kwargs={"wait_time": wait_time})
        stack_action._is_lifecycle_action = True
        return stack_action

    @classmethod
    def reconfigure(
        cls,
        *,
        component: BaseComponent,
        new_config: Union[str, object],
        keep_alive: bool = False,
    ) -> Action:
        """Action to reconfigure a given component

        :param component: Component
        :type component: BaseComponent
        :param new_config: Component config class or path to config file
        :type new_config: Union[str, object]
        :param keep_alive: To keep the component running when reconfiguring, defaults to False
        :type keep_alive: bool, optional

        :return: Component reconfigure action
        :rtype: Action
        """
        kwargs = {
            "component": component,
            "new_config": new_config,
            "keep_alive": keep_alive,
        }

        if not isinstance(new_config, str) and not isinstance(
            new_config, component.config.__class__
        ):
            raise TypeError(
                f"Incompatible config type '{type(new_config)}'. Cannot reconfigure {component.node_name}. config should be either a '{component.config.__class__}' instance or 'str' with path to valid config file (yaml, json, toml)"
            )
        # Action with an empty callable
        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "configure_component"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def update_parameter(
        cls,
        *,
        component: BaseComponent,
        param_name: str,
        new_value: Any,
        keep_alive: bool = True,
    ) -> Action:
        """Action to update (change) the value of a component config parameter

        :param component: Component
        :type component: BaseComponent
        :param param_name: Parameter name
        :type param_name: str
        :param new_value: Parameter value
        :type new_value: Any
        :param keep_alive: To keep the component running when updating the value, defaults to True
        :type keep_alive: bool, optional

        :return: Component parameter update action
        :rtype: Action
        """
        kwargs = {
            "component": component,
            "param_name": param_name,
            "new_value": new_value,
            "keep_alive": keep_alive,
        }
        # Action with an empty callable
        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        stack_action.action_name = "update_parameter"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def update_parameters(
        cls,
        *,
        component: BaseComponent,
        params_names: List[str],
        new_values: List,
        keep_alive: bool = True,
    ) -> Action:
        """Action to update (change) the values of a list of component config parameters

        :param component: Component
        :type component: BaseComponent
        :param params_names: Parameters names
        :type params_names: List[str]
        :param new_values: Parameters values
        :type new_values: List
        :param keep_alive: To keep the component running when updating the value, defaults to True
        :type keep_alive: bool, optional

        :return: Component parameter update action
        :rtype: Action
        """
        kwargs = {
            "component": component,
            "params_names": params_names,
            "new_values": new_values,
            "keep_alive": keep_alive,
        }

        # Action with an empty callable
        stack_action = Action(method=lambda *_: None, kwargs=kwargs)
        # Setup Monitor action
        stack_action.action_name = "update_parameters"
        stack_action._is_monitor_action = True
        return stack_action

    @classmethod
    def log(cls, *, msg: str, logger_name: Optional[str] = None) -> LogInfo:
        """Action to log a message.

        :param msg:
        :type msg: str
        :param logger_name:
        :type logger_name: Optional[str]
        :rtype: LogInfo
        """
        return LogInfo(msg=msg, logger_name=logger_name)
