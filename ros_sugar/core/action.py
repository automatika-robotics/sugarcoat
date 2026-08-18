"""Actions: fire-and-forget by default, closed-loop when told what success means"""

import json
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from functools import partial
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Union

from ..condition import Condition
from ..config import StrEnum
from ..io import Topic
from ..utils import ActionResult, logger, parse_action_result
from .base_action import BaseAction, LogInfo, OpaqueCoroutine, OpaqueFunction
from .event import Event

__all__ = [
    "Action",
    "ActionOutcome",
    "LogInfo",
    "OpaqueCoroutine",
    "OpaqueFunction",
    "bind_monitored_actions",
]

# Accepted policies for what a timeout means
ON_TIMEOUT_POLICIES = ("fail", "succeed", "retry")

# Accepted policies for what a sequence does with an action that has failed for
# good. Only a Routine reads these.
ON_FAIL_POLICIES = ("abort", "skip", "fallback")


class ActionOutcome(StrEnum):
    """Why a run ended.

    The action contract carries only (bool, str), so a caller that has to tell
    a preemption apart from a genuine failure - a `Routine` choosing what to do
    with a step - reads the outcome alongside the result. A `StrEnum`, so each
    member compares and serializes as its plain string value.
    """

    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    PREEMPTED = "preempted"


def bind_monitored_actions(actions: Iterable, host) -> None:
    """Give any monitored actions among `actions` a handle to their host node.

    The host is the node that owns the action, and is what a "monitored" action
    asks to start watching its success condition. Watching starts on the first
    dispatch, so a success topic is never subscribed by a host
    whose action is never triggered.

    :param actions: The actions registered on the host, monitored or not.
        Entries may be single actions or lists of actions
    :param host: The node monitoring these actions, exposing `add_event_listener`
    """
    for entry in actions:
        for action in entry if isinstance(entry, list) else [entry]:
            if isinstance(action, Action):
                action.set_host(host)


class Action(BaseAction):
    """An executable method, optionally monitored until it verifiably worked.

    In its plain form an action is fire and forget: it dispatches a method and
    nothing afterwards can tell whether the method actually worked.

    ```python
    Action(my_component.start)
    ```

    Passing any of the monitoring parameters **activates monitoring**: the
    action dispatches, waits for a verdict, then re-dispatches while the verdict
    is negative and the retry budget allows.

    ```python
    grasp = Action(
        gripper.close,
        success=gripper_state.msg.closed.is_true(),
        timeout=3.0,
        max_retries=3,
    )
    launcher.on(grasp_requested, grasp)
    ```

    The verdict comes from one of two sources:

    - **A success condition** (`success`), a `Condition` on live topic data such
      as ``gripper_state.msg.closed.is_true()``. World state is authoritative: if
      the condition becomes true the action succeeded, even if the dispatched
      method reported otherwise.
    - **The return value**, when no success condition is given. The success half
      of the action's `(bool, str)` result decides the verdict. A raised
      exception, or a return that does not follow the contract, is a failure.

    There are two ways to run one. Calling it executes the action and returns
    its `(bool, str)` result - a monitored action blocks until its outcome is
    decided. `start()` runs the same logic without blocking anything, reporting
    the outcome to a callback instead; this is what `Routine` uses to sequence
    steps without parking a worker thread per active routine.

    :param method: Method to dispatch
    :param args: Positional arguments, may contain `topic.msg.x` expressions
    :param kwargs: Keyword arguments, may contain `topic.msg.x` expressions
    :param success: Condition proving the action worked. If omitted, the return
        value of the method is used instead. Activates monitoring
    :param timeout: Seconds to wait for the verdict on each attempt. Activates
        monitoring
    :param on_timeout: What a timeout means, one of "fail", "succeed" or "retry"
    :param max_retries: Number of *re*-dispatches, so the total number of
        attempts is `max_retries + 1`. Activates monitoring
    :param retry_delay: Seconds to wait between attempts. Activates monitoring
    :param cancel_method: Called by `halt()` to preempt a run in flight, for an
        action that can be told to stop, such as an arm motion. Activates
        monitoring
    :param on_fail: Only read when this action is a step of a `Routine`: what
        the routine does with the step once its retries are spent, one of
        "abort", "skip" or "fallback"
    :param fallback: Only read when this action is a step of a `Routine`: the
        recovery action run when `on_fail="fallback"`
    :param name: Name this action is known by, defaulting to the method name.
        This is what a routine's cursor reports
    :param description: Optional action description
    """

    # NOTE: Dispatches of monitored runs go here rather than on the shared Event
    # pool, whose workers are used by every event in the system and would be
    # starved by a few long running monitored actions
    _dispatch_executor = ThreadPoolExecutor(
        max_workers=10, thread_name_prefix="action_dispatch_worker"
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
        cancel_method: Optional[Callable] = None,
        on_fail: str = "abort",
        fallback: Optional[Union[BaseAction, Callable]] = None,
        name: Optional[str] = None,
        description: Optional[str] = None,
    ) -> None:
        super().__init__(
            method=method, args=args, kwargs=kwargs, description=description
        )
        if name:
            self.action_name = name
        self._set_monitoring_policy(
            success=success,
            timeout=timeout,
            on_timeout=on_timeout,
            max_retries=max_retries,
            retry_delay=retry_delay,
            cancel_method=cancel_method,
            on_fail=on_fail,
            fallback=fallback,
        )

    def _set_monitoring_policy(
        self,
        success: Optional[Union[Condition, Topic]] = None,
        timeout: Optional[float] = None,
        on_timeout: str = "retry",
        max_retries: int = 0,
        retry_delay: float = 0.0,
        cancel_method: Optional[Callable] = None,
        on_fail: str = "abort",
        fallback: Optional[Union[BaseAction, Callable]] = None,
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
        if cancel_method is not None and not callable(cancel_method):
            raise TypeError(
                f"'cancel_method' must be callable, got {type(cancel_method)}"
            )
        if on_timeout != "retry" and timeout is None:
            raise ValueError(
                f"Action '{self.action_name}' sets on_timeout='{on_timeout}' without a "
                "'timeout', so it could never take effect. Set a timeout or drop "
                "'on_timeout'"
            )
        if on_fail not in ON_FAIL_POLICIES:
            raise ValueError(
                f"Action '{self.action_name}' got 'on_fail': '{on_fail}', which is not "
                f"a valid policy. Expected one of {ON_FAIL_POLICIES}"
            )
        if on_fail == "fallback" and fallback is None:
            raise ValueError(
                f"Action '{self.action_name}' has on_fail='fallback' but no 'fallback' "
                "action"
            )
        if fallback is not None and on_fail != "fallback":
            logger.warning(
                f"Action '{self.action_name}' declares a 'fallback' that will never "
                f"run, because on_fail='{on_fail}'"
            )

        self._success_condition = self.__parse_success(success)
        self._timeout = timeout
        self._on_timeout = on_timeout
        self._max_retries = max_retries
        self._retry_delay = retry_delay
        self._cancel_method = cancel_method

        # Fallback is read only by a Routine.
        self.on_fail = on_fail
        self.fallback = self.coerce(fallback, f"The fallback of '{self.action_name}'")

        # Monitoring is activated by declaring any part of a watch or retry
        # policy. The flag also routes the action in the Launcher
        self._is_monitored = (
            self._success_condition is not None
            or timeout is not None
            or max_retries > 0
            or cancel_method is not None
        )

        if self._max_retries == 0 and self._retry_delay > 0:
            logger.warning(
                f"Action '{self.action_name}' has a retry delay but no retries. "
                "The delay will never be used"
            )

        if self._success_condition is not None and self._timeout is None:
            logger.warning(
                f"Action '{self.action_name}' has a success condition but no "
                "'timeout'. If the condition is never met the action will never "
                "settle. Setting a timeout is strongly recommended."
            )

        # The success condition is monitored as an ordinary Event
        self._success_event: Optional[Event] = None
        if self._success_condition is not None:
            self._success_event = Event(event_condition=self._success_condition)
            self._success_event.register_actions(self.set_success)

        # Run state, none of which is serialized. Every transition below happens
        # on a dispatch worker, a timer thread or a subscription callback, so all
        # of it is guarded by the lock
        self._run_lock = threading.RLock()
        self._on_done: Optional[Callable[[ActionResult, ActionOutcome], None]] = None
        self._run_kwargs: Dict = {}
        self._running = False
        self._attempt = 0

        # _attempt_id: bumped on every attempt
        self._attempt_id = 0
        self._attempt_open = False
        self._timer: Optional[threading.Timer] = None

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
    def is_monitored(self) -> bool:
        """Whether this action watches its own outcome.

        True as soon as any part of a watch or retry policy is declared:
        `success`, `timeout`, `max_retries`, `retry_delay` or `cancel_method`.

        :rtype: bool
        """
        return self._is_monitored

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

    @property
    def running(self) -> bool:
        """Whether a run is in flight

        :rtype: bool
        """
        with self._run_lock:
            return self._running

    @classmethod
    def coerce(
        cls, action: Union[BaseAction, Callable, None], owner: str
    ) -> Optional["Action"]:
        """Normalize whatever was declared into an Action.

        Anything a `Routine` runs, and any fallback, goes through the same
        dispatch machinery, so there is one implementation of 'dispatch, wait
        for a verdict, retry' rather than one per kind of thing being run.

        :param action: An Action, a BaseAction or a plain callable
        :param owner: Name used in error messages
        :rtype: Optional[Action]
        """
        if action is None:
            return None
        if getattr(action, "_is_routine", False):
            raise TypeError(
                f"{owner} cannot be a Routine. Nesting routines is not supported yet"
            )
        if isinstance(action, Action):
            return action
        if isinstance(action, BaseAction):
            return cls.from_base_action(action)
        if callable(action):
            return cls(method=action)
        raise TypeError(
            f"{owner} must be an Action or a callable, got {type(action)}"
        )

    @classmethod
    def from_base_action(cls, action: BaseAction, **policy) -> "Action":
        """Lift a BaseAction into a full Action with the given policy.

        The method, arguments and routing flags are carried over, so an action
        built by internal machinery can be used wherever a full Action is
        expected without being redeclared.

        :param action: The action to lift
        :param policy: Monitoring policy, as accepted by the constructor
        :rtype: Action
        """
        if isinstance(action, cls):
            raise TypeError(f"Action '{action.action_name}' is already a full Action")
        lifted = cls(
            method=action.executable, description=action.description, **policy
        )
        lifted.action_name = action.action_name
        lifted.parent_component = action.parent_component
        lifted._is_monitor_action = action._is_monitor_action
        lifted._is_lifecycle_action = action._is_lifecycle_action
        lifted._reset_args_kwargs(
            action._args, action._kwargs, action._dynamic_input_topics
        )
        return lifted

    def set_host(self, host) -> None:
        """Set the node that will monitor this action's success condition.

        :param host: A node exposing `add_event_listener`
        """
        self._host = host

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
                f"Action '{self.action_name}' has a success condition but no "
                "host to monitor it. The action must be registered on a Component or "
                "the Monitor before being triggered."
            )
        self._host.add_runtime_event_listener(self._success_event)
        self._watching = True

    # ---- Running the action ------------------------------------------------

    def start(
        self, on_done: Callable[[ActionResult, ActionOutcome], None], **kwargs
    ) -> None:
        """Dispatch and watch the outcome without blocking the caller.

        The whole watch and retry policy runs on dispatch workers and timer
        threads, and `on_done` is called exactly once, from whichever of those
        threads settles the run. Nothing is parked in the meantime, which is
        what lets a `Routine` hold many steps in flight without holding a
        thread per step.

        :param on_done: Called once with the (success, message) result and the
            outcome that produced it, an `ActionOutcome`
        :param kwargs: Passed to the dispatched method, typically `topics`
        """
        with self._run_lock:
            already_running = self._running
            if not already_running:
                self._running = True
                self._on_done = on_done
                self._run_kwargs = kwargs
                self._attempt = 0

        if already_running:
            error = (
                f"Action '{self.action_name}' is already running. Ignoring "
                "this dispatch rather than interleaving two runs of the same action"
            )
            logger.warning(error)
            on_done((False, error), ActionOutcome.FAILURE)
            return

        try:
            self.__start_watching()
        except RuntimeError as e:
            logger.error(str(e))
            self.__finish((False, str(e)), ActionOutcome.FAILURE)
            return
        self.__begin_attempt()

    def __call__(self, **kwargs) -> ActionResult:
        """Execute the action and return its result.

        An unmonitored action runs inline on the calling thread, exactly as a
        plain method call: no worker is used and nothing is watched. A monitored
        action is the blocking face of `start()`: it parks the calling thread
        until the outcome is decided. Either way the return follows the action
        contract.

        :return: (success, message) per the action contract, where a monitored
            action's message explains the final verdict across all attempts
        :rtype: ActionResult
        """
        if not self._is_monitored:
            return super().__call__(**kwargs)

        settled = threading.Event()
        verdict: List[ActionResult] = []

        def _on_done(result: ActionResult, _outcome: ActionOutcome) -> None:
            verdict.append(result)
            settled.set()

        self.start(_on_done, **kwargs)
        settled.wait()
        return verdict[0]

    def halt(self) -> ActionResult:
        """Preempt a run in flight.

        Stops the watch and retry loop, and invokes `cancel_method` if one was
        given so the action can also be told to stop acting.

        NOTE: a dispatched call that is already executing cannot be interrupted.
        `cancel_method` is the only way to affect it, which is why an action used
        in a preemptible context should provide one.

        :return: (success, message), where the message reports what was halted
        :rtype: ActionResult
        """
        with self._run_lock:
            if not self._running:
                return True, f"Action '{self.action_name}' was not running"
            # Invalidate any verdict still to arrive from the current attempt
            self._attempt_id += 1
            self._attempt_open = False
        self.__cancel_timer()

        message = f"Action '{self.action_name}' was preempted"
        if self._cancel_method is not None:
            try:
                cancelled, cancel_message = parse_action_result(
                    self._cancel_method(), f"{self.action_name} cancel method"
                )
            except Exception as e:
                cancelled, cancel_message = False, str(e)
            if not cancelled:
                logger.error(
                    f"Cancel method of '{self.action_name}' failed: {cancel_message}"
                )
            message = f"{message}: {cancel_message}"

        self.__finish((False, message), ActionOutcome.PREEMPTED)
        return True, message

    def set_success(self, **_) -> None:
        """Settle the current attempt as successful.

        Registered as the action of `success_event`, so it is called by the host
        when the success condition holds. A condition that holds while no
        attempt is in flight is ignored: nothing this action did brought it
        about, so it cannot be credited to it.
        """
        with self._run_lock:
            attempt_id = self._attempt_id if self._attempt_open else None
        if attempt_id is None:
            return
        self.__settle_attempt(
            attempt_id, True, ActionOutcome.SUCCESS, "Success condition met"
        )

    def __begin_attempt(self) -> None:
        """Dispatch once and arm the wait for its verdict"""
        with self._run_lock:
            if not self._running:
                return
            self._attempt_id += 1
            self._attempt_open = True
            attempt_id = self._attempt_id
            call_kwargs = dict(self._run_kwargs)

        try:
            prepared_args, prepared_kwargs = self._prepare_call(**call_kwargs)
        except Exception as e:
            error = f"Error preparing arguments for action '{self.action_name}': {e}"
            logger.error(error)
            self.__settle_attempt(attempt_id, False, ActionOutcome.FAILURE, error)
            return

        # Armed before dispatching so that a method which blocks forever is
        # still bounded by the timeout
        self.__arm_timer(self._timeout, self.__on_timeout, attempt_id)
        future = self._dispatch_executor.submit(
            self.__dispatch, prepared_args, prepared_kwargs
        )
        future.add_done_callback(partial(self.__on_dispatch_done, attempt_id))

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
                f"Action '{self.action_name}' reported failure: {message}"
            )
        return succeeded, message

    def __on_dispatch_done(self, attempt_id: int, future: Future) -> None:
        """Read the dispatched method's own report of what happened"""
        try:
            succeeded, message = future.result()
        except Exception as e:
            succeeded, message = False, f"Error executing '{self.action_name}': {e}"

        if succeeded and self._success_condition is not None:
            # Keep waiting for the condition or the timeout
            return
        # A dispatch reported failure
        self.__settle_attempt(
            attempt_id,
            succeeded,
            ActionOutcome.SUCCESS if succeeded else ActionOutcome.FAILURE,
            message,
        )

    def __on_timeout(self, attempt_id: int) -> None:
        """The attempt ran out of time before anything settled it"""
        self.__settle_attempt(
            attempt_id,
            False,
            ActionOutcome.TIMEOUT,
            f"Action '{self.action_name}' did not settle within "
            f"{self._timeout} secs",
        )

    def __settle_attempt(
        self, attempt_id: int, succeeded: bool, outcome: ActionOutcome, message: str
    ) -> None:
        """Close one attempt and either finish the run or start the next one"""
        with self._run_lock:
            stale = (
                not self._running
                or not self._attempt_open
                or attempt_id != self._attempt_id
            )
            if stale:
                return
            self._attempt_open = False
            attempt = self._attempt
        self.__cancel_timer()

        if succeeded:
            self.__finish((True, message), ActionOutcome.SUCCESS)
            return

        if outcome == ActionOutcome.TIMEOUT and self._on_timeout == "succeed":
            logger.warning(
                f"Action '{self.action_name}' timed out, reporting success "
                "as configured by on_timeout='succeed'"
            )
            self.__finish(
                (True, f"{message}, reported as success by on_timeout='succeed'"),
                ActionOutcome.SUCCESS,
            )
            return

        if outcome == ActionOutcome.TIMEOUT and self._on_timeout == "fail":
            logger.error(
                f"Action '{self.action_name}' timed out and on_timeout='fail'"
            )
            self.__finish((False, message), ActionOutcome.TIMEOUT)
            return

        if attempt >= self._max_retries:
            error = (
                f"Action '{self.action_name}' failed after "
                f"{attempt + 1} attempt(s): {message}"
            )
            logger.error(error)
            self.__finish((False, error), outcome)
            return

        with self._run_lock:
            self._attempt += 1
            attempt = self._attempt
        logger.warning(
            f"Action '{self.action_name}' failed, retrying "
            f"({attempt}/{self._max_retries}): {message}"
        )
        if self._retry_delay > 0.0:
            self.__arm_timer(self._retry_delay, self.__begin_attempt)
            return
        self.__begin_attempt()

    def __finish(self, result: ActionResult, outcome: ActionOutcome) -> None:
        """End the run and report the verdict to whoever started it"""
        with self._run_lock:
            if not self._running:
                return
            self._running = False
            self._attempt_open = False
            on_done = self._on_done
            self._on_done = None
        if on_done is None:
            return
        try:
            on_done(result, outcome)
        except Exception as e:
            logger.error(
                f"Error in the completion callback of '{self.action_name}': {e}"
            )

    def __arm_timer(
        self, delay: Optional[float], callback: Callable, *args
    ) -> None:
        """Replace the run's pending timer with a new one"""
        self.__cancel_timer()
        if delay is None:
            return
        timer = threading.Timer(delay, callback, args=args)
        timer.daemon = True
        with self._run_lock:
            if not self._running:
                return
            self._timer = timer
        timer.start()

    def __cancel_timer(self) -> None:
        """Cancel the run's pending timer, if any"""
        with self._run_lock:
            timer, self._timer = self._timer, None
        if timer is not None:
            timer.cancel()

    # ---- Serialization -----------------------------------------------------

    @property
    def dictionary(self) -> Dict:
        """Serialized form, extending the base payload with the monitoring policy

        :rtype: Dict
        """
        dict_value = super().dictionary
        dict_value["monitored"] = self._is_monitored
        dict_value["success"] = (
            self._success_condition.to_json() if self._success_condition else None
        )
        dict_value["timeout"] = self._timeout
        dict_value["on_timeout"] = self._on_timeout
        dict_value["max_retries"] = self._max_retries
        dict_value["retry_delay"] = self._retry_delay
        dict_value["on_fail"] = self.on_fail
        # NOTE: 'fallback' is not serialized. It is a whole action of its own,
        # and it is only read by a Routine, which does not round-trip yet
        # Only the name travels.
        dict_value["cancel"] = (
            self._cancel_method.__name__ if self._cancel_method else None
        )
        return dict_value

    @classmethod
    def deserialize_action(
        cls,
        serialized_action_dict: Dict,
        deserialized_method: Callable,
    ) -> "Action":
        """Reconstruct an Action from serialized action data

        :param serialized_action_dict: Serialized action data
        :param deserialized_method: Deserialized action method
        :rtype: Action
        """
        reconstructed: "Action" = super().deserialize_action(  # type: ignore[assignment]
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
            on_fail=serialized_action_dict.get("on_fail", "abort"),
            cancel_method=cls.__deserialize_cancel_method(
                serialized_action_dict.get("cancel", None), deserialized_method
            ),
        )
        return reconstructed

    @staticmethod
    def __deserialize_cancel_method(
        cancel_name: Optional[str], deserialized_method: Callable
    ) -> Optional[Callable]:
        """Resolve the cancel method against the owner of the action's method"""
        if not cancel_name:
            return None
        owner = getattr(deserialized_method, "__self__", None)
        if owner is None or not hasattr(owner, cancel_name):
            logger.error(
                f"Cannot restore cancel method '{cancel_name}': it is not available on "
                f"the owner of '{getattr(deserialized_method, '__name__', '')}'. The "
                "action will not be cancellable in this process."
            )
            return None
        return getattr(owner, cancel_name)
