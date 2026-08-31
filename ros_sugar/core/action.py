"""Actions: fire-and-forget by default, closed-loop when told what success means"""

import json
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from functools import partial
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Union

from rclpy.action.server import GoalStatus

from ..condition import Condition
from ..config import StrEnum
from ..io import Topic
from ..utils import ActionReturnType, logger, parse_action_result
from .base_action import BaseAction, LogInfo, OpaqueCoroutine, OpaqueFunction
from .event import Event

__all__ = [
    "Action",
    "ActionOutcome",
    "ActionServerGoal",
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
        self._on_done: Optional[Callable[[ActionReturnType, ActionOutcome], None]] = None
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
        `success`, `timeout`, `max_retries` or `cancel_method`.

        NOTE: not `retry_delay`. It only delays a retry, so with no retries to
        delay it can never take effect, and arming a watch loop for it would
        monitor an action that has nothing to monitor.

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
            # A component action has already been checked: the decorator
            # validated that it returns (bool, str) when the class was defined.
            # A loose function has not, and wrapping it silently would let a
            # step whose verdict cannot be read reach a routine
            if hasattr(action, "_action_description"):
                return cls(method=action)
            # A lambda has no name worth quoting back, so do not tell someone
            # to write Action(<lambda>)
            name = getattr(action, "__name__", "")
            wrapped = f"Action({name})" if name.isidentifier() else "an Action"
            raise TypeError(
                f"{owner} is a plain callable. Wrap it as {wrapped} to say what "
                "should happen to its result, or declare it on a component with "
                "@component_action"
            )
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
        self, on_done: Callable[[ActionReturnType, ActionOutcome], None], **kwargs
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

    def __call__(self, **kwargs) -> ActionReturnType:
        """Execute the action and return its result.

        An unmonitored action runs inline on the calling thread, exactly as a
        plain method call: no worker is used and nothing is watched. A monitored
        action is the blocking face of `start()`: it parks the calling thread
        until the outcome is decided. Either way the return follows the action
        contract.

        :return: (success, message) per the action contract, where a monitored
            action's message explains the final verdict across all attempts
        :rtype: ActionReturnType
        """
        if not self._is_monitored:
            return super().__call__(**kwargs)

        settled = threading.Event()
        verdict: List[ActionReturnType] = []

        def _on_done(result: ActionReturnType, _outcome: ActionOutcome) -> None:
            verdict.append(result)
            settled.set()

        self.start(_on_done, **kwargs)
        settled.wait()
        return verdict[0]

    def halt(self) -> ActionReturnType:
        """Preempt a run in flight.

        Stops the watch and retry loop, and invokes `cancel_method` if one was
        given so the action can also be told to stop acting.

        NOTE: a dispatched call that is already executing cannot be interrupted.
        `cancel_method` is the only way to affect it, which is why an action used
        in a preemptible context should provide one.

        :return: (success, message), where the message reports what was halted
        :rtype: ActionReturnType
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

    def __dispatch(self, call_args: List, call_kwargs: Dict) -> ActionReturnType:
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

    def _abandon_attempt(self) -> None:
        """Drop work an attempt left in flight. No-op for a plain method call.

        Overridden by kinds whose dispatch outlives the call, so a timeout does
        not leave the real work running.
        """
        return None

    def get_feedback(self) -> Optional[Dict]:
        """Progress of the attempt in flight, or None when there is none.

        Read by a Routine to put the active step's progress in its cursor.
        """
        return None

    def set_feedback_sink(self, sink: Optional[Callable[[], None]]) -> None:
        """Install a zero-arg callable to ping when feedback arrives.

        No-op unless the action actually produces feedback.
        """
        return None

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

        if outcome == ActionOutcome.TIMEOUT:
            # The dispatch never reported back, so whatever it started is still
            # running. Retrying or failing on top of it would leave it there.
            self._abandon_attempt()

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

    def __finish(self, result: ActionReturnType, outcome: ActionOutcome) -> None:
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
        cancel_method: Optional[Callable] = None,
        fallback: Optional["Action"] = None,
    ) -> "Action":
        """Reconstruct an Action from serialized action data

        :param serialized_action_dict: Serialized action data
        :param deserialized_method: Deserialized action method
        :param cancel_method: Resolved cancel method, overriding the serialized
            name. Needed when the executable is not a bound method, so there is
            no owner to resolve that name against
        :param fallback: Resolved fallback action. `dictionary` cannot carry one
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
            fallback=fallback,
            cancel_method=(
                cancel_method
                if cancel_method is not None
                else cls.__deserialize_cancel_method(
                    serialized_action_dict.get("cancel", None), deserialized_method
                )
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


class ActionServerGoal(Action):
    """A step that sends a goal to a ROS action server and waits for its result.

    Unlike a method step, the dispatch outlives the call: the goal runs on the
    server and its result is the verdict. So this kind reads the server's own
    outcome instead of needing a success topic, cancels natively on `halt()`,
    and reports the server's feedback into a routine's cursor.

    ```python
    Routine("patrol", steps=[
        ActionServerGoal(component="planner", goal=goal_msg, timeout=120.0),
        ActionServerGoal(component="planner", goal=other_goal, on_fail="skip"),
    ])
    ```

    The client is resolved from the host at dispatch, not held here: steps are
    built in a recipe or from JSON, both before any node exists.

    :param component: Node name of a component whose main action server to drive
    :param server_name: Action server name, when not naming a component
    :param server_type: Action type, required with `server_name`
    :param goal: A ready Goal message, a dict of goal fields, or None to take
        one from the call arguments
    :param success: Condition proving the goal did what was wanted. When given
        it decides the verdict: met at any point during execution, or within
        `success_grace` of the goal returning, means success. The server's own
        outcome then only bounds the window
    :param success_grace: Seconds to keep checking `success` after the goal
        returns, for a condition topic that lags the server
    """

    #: How often the wait loop re-checks the success condition and the client
    _POLL_PERIOD = 0.2

    def __init__(
        self,
        *,
        component: Optional[str] = None,
        server_name: Optional[str] = None,
        server_type: Optional[type] = None,
        goal: Optional[Any] = None,
        success: Optional[Union[Condition, Topic]] = None,
        success_grace: float = 1.0,
        timeout: Optional[float] = None,
        on_timeout: str = "fail",
        max_retries: int = 0,
        retry_delay: float = 0.0,
        on_fail: str = "abort",
        fallback: Optional[Union[BaseAction, Callable]] = None,
        name: Optional[str] = None,
        description: Optional[str] = None,
    ) -> None:
        if not component and not (server_name and server_type):
            raise ValueError(
                "An action server step needs either 'component' or both "
                "'server_name' and 'server_type'"
            )
        self._component = component
        self._server_name = server_name
        self._server_type = server_type
        self._goal_spec = goal
        # NOTE: kept here rather than handed to the monitoring policy. As a
        # policy condition it would mean "server succeeded AND then this holds",
        # which for a goal that already reports its own outcome is a wait for
        # nothing. Here it is checked during the goal and briefly after it.
        self._success_check = self.__as_condition(success)
        self._success_grace = success_grace

        # Per-dispatch state
        self._client = None
        self._settled = threading.Event()
        self._abandoned = False
        self._feedback_sink: Optional[Callable[[], None]] = None

        super().__init__(
            method=self._send_and_wait,
            timeout=timeout,
            # If timeout is set but no timeout policy -> retry be default
            on_timeout=on_timeout if timeout is not None else "retry",
            max_retries=max_retries,
            retry_delay=retry_delay,
            cancel_method=self._cancel,
            on_fail=on_fail,
            fallback=fallback,
            name=name or component or server_name,
            description=description,
        )

    @staticmethod
    def __as_condition(
        success: Optional[Union[Condition, Topic]],
    ) -> Optional[Condition]:
        """Take a Condition as given, or a Topic as 'anything on this topic'"""
        if success is None or isinstance(success, Condition):
            return success
        if isinstance(success, Topic):
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
            f"'success' must be a Condition or a Topic, got {type(success)}"
        )

    @property
    def target(self) -> str:
        """Readable name of the server this step drives"""
        return self._component or self._server_name or "unknown"

    def get_required_topics(self) -> List[Topic]:
        """Topics the step reads, including the success condition's.

        A routine reports these so its host subscribes them before the step
        runs; without that the condition would never see any data.
        """
        topics = list(super().get_required_topics())
        if self._success_check is not None:
            known = {topic.name for topic in topics}
            for name, spec in self._success_check._get_involved_topics().items():
                if name not in known:
                    topics.append(Topic(name=name, **spec))
        return topics

    # ---- Dispatch ---------------------------------------------------------

    def _resolve_client(self):
        """Get the client for this step's server from the host"""
        if self._host is None:
            raise RuntimeError(
                f"Action server step '{self.action_name}' has no host to get a "
                "client from. It must be registered on a Monitor before it runs"
            )
        if self._component:
            return self._host.get_component_action_client(self._component)
        return self._host.get_action_client(self._server_name, self._server_type)

    def _dispatch_goal(self, client, call_kwargs: Dict) -> bool:
        """Send the goal, in whichever of the three shapes it was given"""
        goal = self._goal_spec
        if goal is None:
            # Filled from the call arguments, or an empty goal
            goal = call_kwargs.get("goal", None)
        if goal is None:
            return client.send_request(client.config.action_type.Goal())
        if isinstance(goal, dict):
            return client.send_request_from_dict(goal)
        return client.send_request(goal)

    def _send_and_wait(self, **kwargs) -> ActionReturnType:
        """Send the goal and block until it, or the success condition, settles.

        Blocking is safe here: dispatches run on their own worker pool, never
        on the ROS executor. The deadline belongs to this action's own timeout.
        """
        try:
            client = self._resolve_client()
        except Exception as e:
            return False, str(e)

        self._client = client
        self._abandoned = False
        self._settled.clear()
        client.add_feedback_listener(self._on_client_event)
        try:
            if not self._dispatch_goal(client, kwargs):
                if client.goal_rejected:
                    return False, f"Server '{self.target}' rejected the goal"
                return False, f"Server '{self.target}' did not accept the goal"

            while not self._settled.wait(self._POLL_PERIOD):
                if self._condition_met():
                    # Succeeded early: stop the goal rather than leave it
                    # running while the routine moves on
                    self._cancel()
                    return True, f"Success condition met while '{self.target}' ran"
                if client.action_returned or client.goal_rejected or self._abandoned:
                    break

            if self._condition_met():
                return True, f"Success condition met as '{self.target}' returned"
            if self._success_check is not None:
                return self._verdict_from_condition(client)
            return self._verdict_from_status(client)
        finally:
            client.remove_feedback_listener(self._on_client_event)

    def _condition_met(self) -> bool:
        """Whether the success condition holds against the host's latest data"""
        if self._success_check is None or self._host is None:
            return False
        snapshot = getattr(self._host, "get_topics_snapshot", None)
        if snapshot is None:
            return False
        try:
            return self._success_check.evaluate(snapshot())
        except Exception:
            return False

    def _verdict_from_condition(self, client) -> ActionReturnType:
        """Keep checking the condition for a grace period after the goal ends.

        A condition topic often lags the server finishing, so deciding at the
        instant of return would fail a goal that did work.
        """
        deadline = time.time() + self._success_grace
        while time.time() < deadline:
            if self._settled.wait(min(self._POLL_PERIOD, self._success_grace)):
                pass
            if self._condition_met():
                return True, f"Success condition met after '{self.target}' returned"
            if self._abandoned:
                break
        return (
            False,
            f"Success condition not met within {self._success_grace}s of "
            f"'{self.target}' returning ({self._status_name(client)})",
        )

    @staticmethod
    def _status_name(client) -> str:
        return {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }.get(getattr(client, "action_status", None), "no terminal status")

    def _verdict_from_status(self, client) -> ActionReturnType:
        """The server's own outcome, when no success condition was given"""
        status = getattr(client, "action_status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, f"Server '{self.target}' succeeded"
        if status == GoalStatus.STATUS_ABORTED:
            return False, f"Server '{self.target}' aborted the goal"
        if status == GoalStatus.STATUS_CANCELED:
            return False, f"Goal on '{self.target}' was canceled"
        return False, f"No terminal status from '{self.target}'"

    # ---- Preemption -------------------------------------------------------

    def _cancel(self, **_) -> ActionReturnType:
        """Cancel the goal in flight and release the waiting dispatch"""
        self._abandoned = True
        client = self._client
        result = (
            client.cancel_request() if client is not None else (True, "nothing to cancel")
        )
        self._settled.set()
        return result

    def _abandon_attempt(self) -> None:
        """A timed out attempt leaves a live goal on the server; take it back"""
        self._cancel()

    # ---- Feedback ---------------------------------------------------------

    def _on_client_event(self) -> None:
        """Fired by the client on every feedback message and on terminal state"""
        client = self._client
        if client is not None and (client.action_returned or client.goal_rejected):
            self._settled.set()
        if self._feedback_sink is not None:
            self._feedback_sink()

    def set_feedback_sink(self, sink: Optional[Callable[[], None]]) -> None:
        """Install the callable pinged when the server sends feedback"""
        self._feedback_sink = sink

    def get_feedback(self) -> Optional[Dict]:
        """Progress of the goal in flight, for a routine's cursor"""
        client = self._client
        if client is None:
            return None
        return {
            "target": self.target,
            "server_status": client._status,
            "feedback_count": client.feedback_count,
        }

    # ---- Serialization ----------------------------------------------------

    @property
    def dictionary(self) -> Dict:
        """Serialized form, adding what identifies the server and the goal"""
        dict_value = super().dictionary
        dict_value["kind"] = "action_server"
        dict_value["component"] = self._component
        dict_value["server_name"] = self._server_name
        dict_value["goal"] = self._goal_spec if isinstance(self._goal_spec, dict) else None
        dict_value["success_grace"] = self._success_grace
        dict_value["success"] = (
            self._success_check.to_json() if self._success_check else None
        )
        return dict_value
