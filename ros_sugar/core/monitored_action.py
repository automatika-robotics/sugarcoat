"""Action with a closed loop: dispatch, watch for success, retry on failure"""

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Union

from ..condition import Condition
from ..io import Topic
from ..utils import ActionResult, logger, parse_action_result
from .action import Action
from .event import Event

# Accepted policies for what a timeout means
ON_TIMEOUT_POLICIES = ("fail", "succeed", "retry")

# Granularity of the success wait. The latch is set from a subscription
# callback, so this only bounds how quickly a failed dispatch short-circuits
# the remaining wait
_WAIT_SLICE_SECS = 0.05


def bind_monitored_actions(actions: Iterable, host) -> None:
    """Give any monitored actions among `actions` a handle to their host node.

    The host is the node that owns the action, and is what a monitored action
    asks to start watching its success condition. Watching starts on the first
    dispatch, so a success topic is never subscribed by a host
    whose action is never triggered.

    :param actions: The actions registered on the host, monitored or not.
        Entries may be single actions or lists of actions
    :param host: The node monitoring these actions, exposing `add_event_listener`
    """
    for entry in actions:
        for action in entry if isinstance(entry, list) else [entry]:
            if isinstance(action, MonitoredAction):
                action.set_host(host)


class MonitoredAction(Action):
    """Action that verifies its own outcome and retries until it is achieved.

    A plain `Action` is fire and forget: it dispatches a method and nothing can
    tell afterwards whether the method actually worked. `MonitoredAction` closes
    that loop by dispatching, waiting for a verdict, then re-dispatching while
    the verdict is negative and the retry budget allows.

    The verdict comes from one of two sources:

    - **A success condition** (`success`), a `Condition` on live topic data such
      as ``gripper_state.msg.closed.is_true()``. World state is authoritative: if
      the condition becomes true the action succeeded, even if the dispatched
      method reported otherwise.
    - **The return value**, when no success condition is given. Returning `False`
      or raising is a failure; `True`, `None` or any other value is a success,
      matching how component actions are reported over the `ExecuteMethod`
      service.

    A `MonitoredAction` is an `Action`, so it is registered, routed and
    serialized exactly like one and the monitoring runs in whichever process
    owns the action.

    ```python
    grasp = MonitoredAction(
        gripper.close,
        success=gripper_state.msg.closed.is_true(),
        timeout=3.0,
        max_retries=3,
    )
    launcher.on(grasp_requested, grasp)
    ```

    :param method: Method to dispatch
    :param args: Positional arguments, may contain `topic.msg.x` expressions
    :param kwargs: Keyword arguments, may contain `topic.msg.x` expressions
    :param success: Condition proving the action worked. If omitted, the return
        value of the method is used instead
    :param timeout: Seconds to wait for the verdict on each attempt
    :param on_timeout: What a timeout means, one of "fail", "succeed" or "retry"
    :param max_retries: Number of *re*-dispatches, so the total number of
        attempts is `max_retries + 1`
    :param retry_delay: Seconds to wait between attempts
    :param description: Optional action description
    """

    # Dispatches run here rather than on the shared Event pool, whose workers
    # are used by every event in the system and would be starved by a few
    # long running monitored actions
    _dispatch_executor = ThreadPoolExecutor(
        max_workers=10, thread_name_prefix="monitored_action_worker"
    )

    def __init__(
        self,
        method: Callable,
        args: Optional[Union[Tuple, List, Any]] = None,
        kwargs: Optional[Dict] = None,
        success: Optional[Union[Condition, Topic]] = None,
        timeout: Optional[float] = None,
        on_timeout: str = "retry",
        max_retries: int = 0,
        retry_delay: float = 0.0,
        description: Optional[str] = None,
    ) -> None:
        super().__init__(
            method=method, args=args, kwargs=kwargs, description=description
        )
        self._set_monitoring_policy(
            success=success,
            timeout=timeout,
            on_timeout=on_timeout,
            max_retries=max_retries,
            retry_delay=retry_delay,
        )

    def _set_monitoring_policy(
        self,
        success: Optional[Union[Condition, Topic]] = None,
        timeout: Optional[float] = None,
        on_timeout: str = "retry",
        max_retries: int = 0,
        retry_delay: float = 0.0,
    ) -> None:
        """Validate and apply the monitoring policy.

        Kept separate from `__init__` so that deserialization can restore the
        policy onto an already reconstructed action.
        """
        if on_timeout not in ON_TIMEOUT_POLICIES:
            raise ValueError(
                f"Got 'on_timeout': '{on_timeout}', which is not a valid policy. "
                f"Expected one of {ON_TIMEOUT_POLICIES}"
            )
        if max_retries < 0:
            raise ValueError(f"'max_retries' cannot be negative, got {max_retries}")
        if timeout is not None and timeout <= 0.0:
            raise ValueError(f"'timeout' must be a positive number, got {timeout}")
        if retry_delay < 0.0:
            raise ValueError(f"'retry_delay' cannot be negative, got {retry_delay}")

        self._success_condition = self.__parse_success(success)
        self._timeout = timeout
        self._on_timeout = on_timeout
        self._max_retries = max_retries
        self._retry_delay = retry_delay

        if self._success_condition is not None and self._timeout is None:
            logger.warning(
                f"MonitoredAction '{self.action_name}' has a success condition but no "
                "'timeout'. If the condition is never met the action will block its "
                "worker indefinitely. Setting a timeout is strongly recommended."
            )

        # Set by the success event when the condition holds.
        self._success_latch = threading.Event()

        # The success condition is monitored as an ordinary Event, so the host
        # evaluates it on message arrival using the machinery it already has.
        self._success_event: Optional[Event] = None
        if self._success_condition is not None:
            self._success_event = Event(event_condition=self._success_condition)
            self._success_event.register_actions(self.set_success)

        # Host node and whether it has been asked to monitor the success event.
        # Both are runtime state and are never serialized
        self._host = None
        self._watching = False

    @staticmethod
    def __parse_success(
        success: Optional[Union[Condition, Topic]],
    ) -> Optional[Condition]:
        """Normalize the success argument into a Condition"""
        if success is None or isinstance(success, Condition):
            return success
        if isinstance(success, Topic):
            # Topic = On Any
            return Condition(
                topic_name=success.name,
                topic_msg_type=success.msg_type.__name__,
                topic_qos_config=success.qos_profile.to_dict(),
                topic_use_plugin=success.use_plugin,
                attribute_path=[],
                operator_func=None,
                ref_value=None,
            )
        raise TypeError(
            "'success' must be a Condition on a Topic (e.g. topic.msg.data.is_true()) "
            f"or a Topic, got {type(success)}"
        )

    @property
    def success_event(self) -> Optional[Event]:
        """Event monitored to detect success, `None` in return value mode.

        NOTE: this is deliberately a separate event rather than extra topics on
        the triggering event. An 'on any' event fires only once all of its
        involved topics have data, so a success topic added there would hold the
        action back until the state it is meant to bring about had already been
        published at least once.

        :rtype: Optional[Event]
        """
        return self._success_event

    def as_plain_action(self) -> Action:
        """An unmonitored Action wrapping the same method, args and kwargs.

        Used where the dispatch has to happen somewhere other than where the
        monitoring runs. A launcher owned action executes in the launch context,
        while its watch and retry loop stays in the Monitor, which must never
        block the launch loop.

        :rtype: Action
        """
        plain = Action(method=self.executable, description=self._description)
        plain.action_name = self.action_name
        plain.parent_component = self.parent_component
        plain._is_monitor_action = self._is_monitor_action
        plain._is_lifecycle_action = self._is_lifecycle_action
        plain._reset_args_kwargs(
            self._args, self._kwargs, self._dynamic_input_topics
        )
        return plain

    def set_host(self, host) -> None:
        """Set the node that will monitor this action's success condition.

        :param host: A node exposing `add_event_listener`
        """
        self._host = host

    def set_success(self, **_) -> None:
        """Mark the current attempt as successful.

        Registered as the action of `success_event`, so it is called by the host
        when the success condition holds.
        """
        self._success_latch.set()

    def __start_watching(self) -> None:
        """Ask the host to start monitoring the success event.

        Deferred to the first dispatch so that a host never subscribes to a
        success topic for an action that is never triggered. The subscription is
        then kept, rather than torn down after every attempt, to avoid
        subscription churn across retries and repeat triggers.
        """
        if self._success_event is None or self._watching:
            return
        if self._host is None:
            raise RuntimeError(
                f"MonitoredAction '{self.action_name}' has a success condition but no "
                "host to monitor it. The action must be registered on a Component or "
                "the Monitor before being triggered."
            )
        self._host.add_runtime_event_listener(self._success_event)
        self._watching = True

    def __call__(self, **kwargs) -> ActionResult:
        """Dispatch the action and verify the outcome, retrying as configured.

        Blocks until the outcome is decided. This runs on an Event worker
        thread, and the triggering Event will not re-fire while its actions are
        in flight, so no separate preemption handling is needed.

        :return: (success, message) per the action contract, where the message
            explains the final verdict across all attempts
        :rtype: ActionResult
        """
        self.__start_watching()
        attempt = 0
        while True:
            succeeded, timed_out, message = self.__run_attempt(**kwargs)
            if succeeded:
                return True, message

            if timed_out and self._on_timeout == "succeed":
                logger.warning(
                    f"MonitoredAction '{self.action_name}' timed out, reporting success "
                    "as configured by on_timeout='succeed'"
                )
                return True, f"{message}, reported as success by on_timeout='succeed'"
            if timed_out and self._on_timeout == "fail":
                logger.error(
                    f"MonitoredAction '{self.action_name}' timed out and on_timeout='fail'"
                )
                return False, message

            if attempt >= self._max_retries:
                error = (
                    f"MonitoredAction '{self.action_name}' failed after "
                    f"{attempt + 1} attempt(s): {message}"
                )
                logger.error(error)
                return False, error

            attempt += 1
            logger.warning(
                f"MonitoredAction '{self.action_name}' failed, retrying "
                f"({attempt}/{self._max_retries}): {message}"
            )
            if self._retry_delay > 0.0:
                time.sleep(self._retry_delay)

    def __run_attempt(self, **kwargs) -> Tuple[bool, bool, str]:
        """Dispatch once and wait for the verdict.

        :return: (succeeded, timed_out, message)
        :rtype: Tuple[bool, bool, str]
        """
        # Cleared before dispatching so a success can only ever be credited to
        # data arriving after this attempt started. Otherwise a condition that
        # already held beforehand would report instant success without the
        # action having done anything
        self._success_latch.clear()

        call_args, call_kwargs = self._prepare_call(**kwargs)
        future = self._dispatch_executor.submit(self.__dispatch, call_args, call_kwargs)

        if self._success_condition is None:
            return self.__await_return_value(future)
        return self.__await_success_condition(future)

    def __dispatch(self, call_args: List, call_kwargs: Dict) -> ActionResult:
        """Run the executable and read its verdict off the (bool, str) contract"""
        try:
            result = self.executable(*call_args, **call_kwargs)
        except Exception as e:
            error = f"Error executing action '{self.action_name}': {e}"
            logger.error(error)
            return False, error
        succeeded, message = parse_action_result(result, self.action_name)
        if not succeeded:
            logger.warning(
                f"MonitoredAction '{self.action_name}' reported failure: {message}"
            )
        return succeeded, message

    def __await_return_value(self, future) -> Tuple[bool, bool, str]:
        """Verdict comes from the dispatched method itself"""
        try:
            succeeded, message = future.result(timeout=self._timeout)
            return succeeded, False, message
        except FutureTimeoutError:
            error = (
                f"MonitoredAction '{self.action_name}' did not return within "
                f"{self._timeout} secs"
            )
            logger.warning(error)
            # NOTE: the call is still running. A method that is already
            # executing cannot be cancelled
            return False, True, error

    def __await_success_condition(self, future) -> Tuple[bool, bool, str]:
        """Verdict comes from the world reaching the expected state"""
        waited = 0.0
        while self._timeout is None or waited < self._timeout:
            slice_secs = _WAIT_SLICE_SECS
            if self._timeout is not None:
                slice_secs = min(slice_secs, self._timeout - waited)
            if self._success_latch.wait(slice_secs):
                return True, False, "Success condition met"
            waited += slice_secs
            # A dispatch that already reported failure will not bring the
            # condition about, so stop waiting out the rest of the timeout
            if future.done():
                dispatched, message = future.result()
                if not dispatched:
                    return False, False, message
        return (
            False,
            True,
            f"Success condition not met within {self._timeout} secs",
        )

    @property
    def dictionary(self) -> Dict:
        """Serialized form, extending the Action payload with the monitoring policy

        :rtype: Dict
        """
        dict_value = super().dictionary
        dict_value["monitored"] = True
        dict_value["success"] = (
            self._success_condition.to_json() if self._success_condition else None
        )
        dict_value["timeout"] = self._timeout
        dict_value["on_timeout"] = self._on_timeout
        dict_value["max_retries"] = self._max_retries
        dict_value["retry_delay"] = self._retry_delay
        return dict_value

    @classmethod
    def deserialize_action(
        cls,
        serialized_action_dict: Dict,
        deserialized_method: Callable,
    ) -> "MonitoredAction":
        """Reconstruct a MonitoredAction from serialized action data

        :param serialized_action_dict: Serialized action data
        :param deserialized_method: Deserialized action method
        :rtype: MonitoredAction
        """
        # Action.deserialize_action builds via cls(), so this is a MonitoredAction
        reconstructed: "MonitoredAction" = super().deserialize_action(  # type: ignore[assignment]
            serialized_action_dict, deserialized_method
        )
        serialized_condition = serialized_action_dict.get("success", None)
        reconstructed._set_monitoring_policy(
            success=(
                Condition.from_dict(json.loads(serialized_condition))
                if serialized_condition
                else None
            ),
            timeout=serialized_action_dict.get("timeout", None),
            on_timeout=serialized_action_dict.get("on_timeout", "retry"),
            max_retries=serialized_action_dict.get("max_retries", 0),
            retry_delay=serialized_action_dict.get("retry_delay", 0.0),
        )
        return reconstructed
