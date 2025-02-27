"""Actions"""

import inspect
import json
from rclpy.lifecycle import Node as LifecycleNode
from launch.actions import OpaqueCoroutine, OpaqueFunction
from functools import wraps
from typing import Callable, Dict, Optional, Union

from launch import LaunchContext
import launch
from launch.actions import LogInfo as LogInfoROSAction

from ..launch import logger


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
        self._args = args
        self._kwargs = kwargs if kwargs else {}

        # Check if it is a component action and update parent and keyname
        if hasattr(self._function, "__self__"):
            action_object = self._function.__self__

            if hasattr(action_object, "node_name") and isinstance(
                action_object, LifecycleNode
            ):
                self.parent_component = action_object.node_name
                self.action_name = self._function.__name__
                self.__component_action = True

    def __call__(self, **kwargs):
        """
        Execute the action

        :return: _description_
        :rtype: _type_
        """
        if hasattr(self, "_event_parser_method"):
            output = self._event_parser_method(**kwargs)
            if self._event_parser_mapping:
                self.kwargs.update({self._event_parser_mapping: output})
        return self.executable(*self.args, **self.kwargs)

    def event_parser(
        self, method: Callable, output_mapping: Optional[str] = None, **new_kwargs
    ):
        """Add an event parser to the action. This method will be executed before the main action executable. The returned value from the method will be passed to the action executable as a keyword argument using output_mapping

        :param method: Method to be executed before the main action executable
        :type method: callable
        :param output_mapping: keyword argument name to pass the parser returned value to the action main method, defaults to None
        :type output_mapping: Optional[str], optional
        """
        if new_kwargs:
            self.kwargs.update(new_kwargs)

        self._event_parser_method = method
        self._event_parser_mapping = output_mapping

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
