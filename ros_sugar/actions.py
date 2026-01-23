"""Actions

Pre-defined component and system level actions

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

from functools import wraps
from typing import Any, Callable, List, Optional, Union

from .core.action import Action, LogInfo, _create_auto_topic_parser
from .core.component import BaseComponent
from .utils import InvalidAction
from .io.topic import Topic
from .launch import logger

__all__ = [
    "Action",
    "start",
    "stop",
    "restart",
    "reconfigure",
    "update_parameter",
    "update_parameters",
    "log",
    "send_srv_request",
    "send_srv_request_from_topic",
    "send_action_goal",
    "send_action_goal_from_topic",
    "publish_message",
    "publish_message_from_parsed_topic",
]


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


def send_srv_request(*, srv_name: str, srv_type: type, srv_request_msg: Any) -> Action:
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
    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_srv_request"
    stack_action._is_monitor_action = True
    return stack_action


def trigger_service(*, srv_name: str, srv_type: type) -> Action:
    """Action to trigger a ROS2 service

    :param srv_name: Service name
    :type srv_name: str
    :param srv_type: Service type (ROS2 service)
    :type srv_type: type

    :return: Sending request action
    :rtype: Action
    """
    # Combine positional arguments and keyword arguments
    kwargs = {
        "srv_name": srv_name,
        "srv_type": srv_type,
    }

    # Action with an empty callable
    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_srv_request"
    stack_action._is_monitor_action = True
    # Set the type of the required argument (service request)
    # This is done to enable easily setting an "automatic parser" from within components
    stack_action._set_dynamic_argument_types({"srv_request_msg": srv_type.Request})
    return stack_action


def send_srv_request_from_topic(
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
    action = send_srv_request(
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
    action.add_event_parser(auto_parser_method, keyword_argument_name="srv_request_msg")

    return action


def trigger_component_service(*, component: BaseComponent) -> Action:
    """Action to trigger a component's main service

    :param component: Sugarcoat Component
    :type component: BaseComponent

    :return: Sending request action
    :rtype: Action
    """
    if not component.main_srv_name or not component.service_type:
        raise NotImplementedError(
            f"Cannot use the action 'trigger_component_service' on component '{component.node_name}'. Component {component.node_name} does not have a main service implemented."
        )
    # Combine positional arguments and keyword arguments
    kwargs = {
        "srv_name": component.main_srv_name,
        "srv_type": component.service_type,
    }

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_srv_request"
    stack_action._is_monitor_action = True
    # Set the type of the required argument (service request)
    stack_action._set_dynamic_argument_types({
        "srv_request_msg": component.service_type.Request
    })
    return stack_action


def send_action_goal(
    *, server_name: str, server_type: type, request_msg: Any
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
        "action_name": server_name,
        "action_type": server_type,
        "action_request_msg": request_msg,
    }

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_action_goal"
    stack_action._is_monitor_action = True
    return stack_action


def trigger_action_server(*, server_name: str, server_type: type) -> Action:
    """Action to trigger a ROS2 action server

    :param action_name: ROS2 action name
    :type action_name: str
    :param action_type: ROS2 action type
    :type action_type: type

    :return: Sending goal action
    :rtype: Action
    """
    # Combine positional arguments and keyword arguments
    kwargs = {
        "action_name": server_name,
        "action_type": server_type,
    }

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_action_goal"
    stack_action._is_monitor_action = True
    # Set the type of the required argument (service request)
    stack_action._set_dynamic_argument_types({"action_request_msg": server_type.Goal})
    return stack_action


def trigger_component_action_server(*, component: BaseComponent) -> Action:
    """Action to trigger a component's action server

    :param component: Sugarcoat Component
    :type component: BaseComponent

    :return: Sending goal action
    :rtype: Action
    """
    if not component.main_action_name or not component.action_type:
        raise NotImplementedError(
            f"Cannot use the action 'trigger_main_action_server' on component '{component.node_name}'. Component {component.node_name} does not have a main action server implemented."
        )
    # Combine positional arguments and keyword arguments
    kwargs = {
        "action_name": component.main_action_name,
        "action_type": component.action_type,
    }

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "send_action_goal"
    stack_action._is_monitor_action = True
    # Set the type of the required argument (service request)
    stack_action._set_dynamic_argument_types({
        "action_request_msg": component.action_type.Goal
    })
    return stack_action


def send_action_goal_from_topic(
    *,
    server_name: str,
    server_type: type,
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
    request_msg_type = server_type.Goal

    # Create the base action
    action = send_action_goal(
        server_name=server_name,
        server_type=server_type,
        request_msg=request_msg_type(),  # Default empty request
    )

    input_msg_type = topic.ros_msg_type

    # Use provided parser if provided, otherwise create an automatic parser
    auto_parser_method = custom_parser or _create_auto_topic_parser(
        input_msg_type, request_msg_type
    )

    # Attach the parser
    action.add_event_parser(
        auto_parser_method, keyword_argument_name="action_request_msg"
    )

    return action


def publish_message(
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

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "publish_message"
    stack_action._is_monitor_action = True
    return stack_action


def publish_message_from_parsed_topic(
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

    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "publish_message"
    stack_action._is_monitor_action = True

    # Create the automatic parser logic
    parser_method = custom_parser or _create_auto_topic_parser(
        input_msg_type=in_topic.ros_msg_type,
        target_msg_type=out_topic.ros_msg_type,
    )
    stack_action.add_event_parser(parser_method, keyword_argument_name="msg")

    return stack_action


@_validate_component_action
def start(*, component: BaseComponent) -> Action:
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


@_validate_component_action
def stop(*, component: BaseComponent) -> Action:
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


@_validate_component_action
def restart(*, component: BaseComponent, wait_time: Optional[float] = None) -> Action:
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


def reconfigure(
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
    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "configure_component"
    stack_action._is_monitor_action = True
    return stack_action


def update_parameter(
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
    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    stack_action.action_name = "update_parameter"
    stack_action._is_monitor_action = True
    return stack_action


def update_parameters(
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
    stack_action = Action(method=lambda *args, **kwargs: None, kwargs=kwargs)
    # Setup Monitor action
    stack_action.action_name = "update_parameters"
    stack_action._is_monitor_action = True
    return stack_action


def log(*, msg: str, logger_name: Optional[str] = None) -> LogInfo:
    """Action to log a message.

    :param msg:
    :type msg: str
    :param logger_name:
    :type logger_name: Optional[str]
    :rtype: LogInfo
    """
    return LogInfo(msg=msg, logger_name=logger_name)
