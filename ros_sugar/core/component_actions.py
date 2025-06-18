"""Component Actions"""

from functools import wraps
from typing import Any, Callable, List, Optional, Union

from .action import Action
from .component import BaseComponent
from ..utils import InvalidAction
from ..io.topic import Topic
from .action import LogInfo


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
