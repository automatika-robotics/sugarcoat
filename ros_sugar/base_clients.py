"""ROS Service/Action Client Wrapper"""

import threading
import time
from functools import partial
from typing import Any, Callable, Optional, Dict, Tuple
from attrs import Factory, define, field

from rclpy.action.client import ActionClient
from rclpy.action.server import GoalStatus
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup

from .config import BaseAttrs, base_validators
from .supported_types import set_ros_msg_from_dict


@define
class ServiceClientConfig(BaseAttrs):
    """
    Basic configuration for any ROS service client
    """

    srv_type: type = field()
    name: str = field()
    timeout_secs: float = field(
        default=30.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout after calling the service
    attempt_period_secs: float = field(
        default=1.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to attempt to call the service again


@define
class ActionClientConfig(BaseAttrs):
    """
    Basic configuration for any ROS action client
    """

    action_type: type = field()
    name: str = field()
    timeout_secs: float = field(
        default=30.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout after calling the action
    attempt_period_secs: float = field(
        default=1.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to attempt to call the action again
    feedback_check_period: float = field(
        default=0.05, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to check for the action feedback
    feedback_check_timeout: float = field(
        default=60.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout if feedback is not received after x seconds
    cancel_on_feedback_timeout: bool = field(
        default=True
    )  # cancel the goal when no new feedback arrives within feedback_check_timeout.
    # Set False for action servers that legitimately publish no feedback, whose
    # goals would otherwise be cancelled mid-execution
    callback_group: CallbackGroup = field(
        default=Factory(ReentrantCallbackGroup)
    )  # callback group for the feedback callback of the action


class ServiceClientHandler:
    """
    General purpose service client class
    """

    def __init__(
        self,
        client_node: Node,
        config: Optional[ServiceClientConfig] = None,
        srv_name: Optional[str] = None,
        srv_type: Optional[type] = None,
        callback_group: Optional[CallbackGroup] = None,
    ) -> None:
        """
        Init the client

        :param client_node: ROS node used to run the client
        :type client_node: Node
        :param config: Service client configuration
        :type srv_name: ServiceClientConfig
        :param callback_group: Group the responses are processed in, defaults
            to the node's default group. A client called while that group is
            busy, for instance from a lifecycle transition, needs its own
        :type callback_group: Optional[CallbackGroup]

        """
        if not config and not srv_name and not srv_type:
            raise ValueError(
                "Cannot initialize service client. Provide a valid config or a valid service name and service type"
            )
        if not config and (srv_name and srv_type):
            config = ServiceClientConfig(name=srv_name, srv_type=srv_type)

        # If config is provided plus additional name or type -> update name or type
        if srv_name:
            config.name = srv_name

        if srv_type:
            config.srv_type = srv_type

        self.config = config
        self.node = client_node
        self.node.get_logger().debug(
            f"creating client for {self.config.name} of type {self.config.srv_type}"
        )
        self.client = self.node.create_client(
            self.config.srv_type, self.config.name, callback_group=callback_group
        )

    def send_request_from_dict(
        self,
        request_fields: Dict[str, Any],
    ):
        """Send a service request using a serialized Dict request data

        :param request_fields: Request data [key, value]
        :type request_fields: Dict[str, Any]
        :raises ValueError: If a field cannot be set from its value, naming both
        :return: Service result
        :rtype: Any
        """
        updated_message = set_ros_msg_from_dict(
            msg_class=self.config.srv_type.Request, data_dict=request_fields
        )
        self.node.get_logger().debug(f"sending request {updated_message}")
        return self.send_request(updated_message)

    def send_request(self, req_msg, timeout: Optional[float] = None):
        """
        Sends a request to the service returns the response
        In case of failure, the method attempts sending the request again multiple time according to the given config

        :param req_msg: Service request msg
        :type req_msg: Any
        :param timeout: Seconds to wait for the response, instead of the
            configured timeout. `math.inf` waits for as long as the node lives,
            for a caller that has a deadline of its own and whose call may
            legitimately take longer than any fixed budget
        :type timeout: Optional[float]
        :return: Service result
        :rtype: Any
        """
        response_timeout = self.config.timeout_secs if timeout is None else timeout
        _timeout_count: float = 0.0  # timeout counter

        # Check if the service is available every attempt_period_secs
        while not self.client.wait_for_service(
            timeout_sec=self.config.attempt_period_secs
        ):
            # If the service is not available give warning
            self.node.get_logger().warning(
                f"Service {self.config.name} not available, Waiting... timeout in {(self.config.timeout_secs - _timeout_count):.2f} secs"
            )
            _timeout_count += self.config.attempt_period_secs

            # Check for service request timeout
            if _timeout_count > self.config.timeout_secs:
                self.node.get_logger().warning(
                    f"Service {self.config.name} is not available, Cancelling"
                )
                return None

        # Service is available
        self.node.get_logger().debug(
            f"Service {self.config.name} is available, Sending request..."
        )

        # Check request type
        if not isinstance(req_msg, self.config.srv_type.Request):
            self.node.get_logger().error(
                f"Invalid request message for service '{self.config.name}'. Service takes request message of type '{self.config.srv_type.Request}', got '{type(req_msg)}'"
            )
            return None

        # send request. The future is kept locally as well as on the handler:
        # several callers can have a request in flight on one client, and each
        # has to wait for its own response rather than for the latest one
        self.request = req_msg
        future = self.client.call_async(req_msg)
        self.future = future

        # Wait for service response, bounded by the timeout this call was given
        _response_wait: float = 0.0
        while not future.done():
            if _response_wait > response_timeout:
                self.node.get_logger().error(
                    f"Service {self.config.name} did not respond within {response_timeout} secs, Cancelling"
                )
                future.cancel()
                return None
            if not self.node.context.ok():
                # Nothing will deliver the response now. Without this, a call
                # with no deadline would hold its thread past shutdown
                return None
            time.sleep(0.01)
            _response_wait += 0.01

        # return response
        return future.result()


class ActionClientHandler:
    """
    General purpose action client class
    """

    def __init__(
        self,
        client_node: Node,
        config: Optional[ActionClientConfig] = None,
        action_name: Optional[str] = None,
        action_type: Optional[type] = None,
    ):
        """
        Init an action client handler

        :param client_node: ROS node using the client
        :type client_node: rclpy.node.Node
        :param config: Client config
        :type config: ActionClientConfig
        """
        self._check_server_alive_timer = None
        # Zero-arg listeners fired (in the ROS executor thread) on every feedback
        # message and on terminal state, so feedback can be pushed to multiple
        # consumers can push feedback as it arrives instead of polling.
        self._feedback_listeners = set()
        # Identify the goal this handler is tracking
        self._goal_generation = 0
        # Guards claiming and releasing the one goal this handler tracks.
        self._goal_lock = threading.Lock()
        self.reset()

        if not config and (action_name and action_type):
            config = ActionClientConfig(name=action_name, action_type=action_type)
        elif not config:
            raise ValueError(
                "Cannot initialize action client. Provide a valid config or a valid action name and action type"
            )

        # If config is provided plus additional name or type -> update name or type
        if action_name:
            config.name = action_name

        if action_type:
            config.action_type = action_type

        self.config = config
        self.node = client_node
        self.client = ActionClient(
            self.node,
            self.config.action_type,
            self.config.name,
            callback_group=self.config.callback_group,
        )

    def reset(self):
        """
        Reset the client handler
        """
        self.old_feedback_count: int = 0
        self.feedback_count: int = 0
        self.feedback_msg = None
        self.goal_rejected = False
        self.goal_accepted = False
        self.action_returned = False
        self.action_result = None
        # Terminal status from the result. The goal handle's own status comes
        # from the status topic, so it races the result future.
        self.action_status: int = GoalStatus.STATUS_UNKNOWN
        self._feedback_timeout = False
        self._goal_handle = None
        # If any goal is currently accepted, a second goal may not be sent over the top of it
        self._goal_in_flight = False
        # A cancel that arrived before the server answered, carried out as soon
        # as there is a goal handle to cancel
        self._cancel_pending = False
        self._old_status = self._status
        self._start_time_secs = None
        self._stop_alive_timer()

    def _stop_alive_timer(self) -> None:
        """Destroy the feedback watchdog, if one is running"""
        if self._check_server_alive_timer:
            self.node.destroy_timer(self._check_server_alive_timer)
            self._check_server_alive_timer = None

    @property
    def _status(self) -> str:
        """Goal handle status getter

        :return: _description_
        :rtype: str
        """
        if (
            not self._goal_handle
            or self._goal_handle.status == GoalStatus.STATUS_UNKNOWN
        ):
            return "inactive"
        if self._goal_handle.status == GoalStatus.STATUS_ABORTED:
            return "aborted"
        if self._goal_handle.status in [
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
        ]:
            if self.feedback_msg:
                return "running"
            else:
                return "accepted"
        if self._goal_handle.status in [
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ]:
            return "canceled"
        if self._goal_handle.status in [GoalStatus.STATUS_SUCCEEDED]:
            return "completed"
        return "inactive"

    def send_request_from_dict(
        self,
        request_fields: Dict[str, Any],
        wait_until_first_feedback: bool = False,
        still_wanted: Optional[Callable[[], bool]] = None,
    ) -> Optional[bool]:
        """Send an action request using a serialized Dict request data

        :param request_fields: Request data [key, value]
        :type request_fields: Dict[str, Any]
        :param still_wanted: See `send_request`
        :raises ValueError: If a field cannot be set from its value, naming both.
            Raised before anything is sent, so a running goal is left alone
        """
        updated_message = set_ros_msg_from_dict(
            msg_class=self.config.action_type.Goal, data_dict=request_fields
        )
        return self.send_request(
            updated_message, wait_until_first_feedback, still_wanted=still_wanted
        )

    def __for_goal(self, generation: int, callback: Callable, payload: Any) -> None:
        """Run a callback only while it is still about the goal in hand.

        A cancelled goal answers after the client has been taken for the next
        one. Read as the current goal's, that answer overwrites its handle and
        its result, and the new goal becomes untrackable.
        """
        if generation != self._goal_generation:
            return
        callback(payload)

    def __claim_for_new_goal(self, still_wanted: Callable[[], bool]) -> bool:
        """Take the client for a new goal, once the last one has finished.

        This handler tracks one goal: its handle, its feedback and its result.
        Clearing that while a goal is still running used to lose the handle of
        a goal the server was still executing, so nothing could cancel it any
        more. A goal on its way out is waited for - a cancelled step re-sending
        is the ordinary case, and a cancel takes as long as the server needs to
        notice it - and a goal still running past the feedback timeout, by
        which point the server counts as unresponsive anyway, keeps the client.

        :param still_wanted: Asked while waiting. A caller that has stopped in
            the meantime gives up the wait rather than send once the client is
            free
        :return: Whether the client is now this caller's
        :rtype: bool
        """
        waited: float = 0.0
        while self._goal_in_flight and waited < self.config.feedback_check_timeout:
            if not still_wanted():
                return False
            time.sleep(self.config.feedback_check_period)
            waited += self.config.feedback_check_period
        # Asked outside the lock: it takes the caller's own lock
        if not still_wanted():
            return False
        with self._goal_lock:
            if self._goal_in_flight:
                return False
            # Clears the previous goal's terminal state, so this one is not
            # read off it
            self.reset()
            self._goal_in_flight = True
            # Everything the server says about the previous goal arrives after
            # this, and belongs to a goal this client no longer tracks
            self._goal_generation += 1
        return True

    def wait_until_idle(self, timeout: float) -> bool:
        """Wait for the goal this client has on the server to end.

        For a caller tearing things down: the cancels it sent are on their way,
        and a server told to stop is worth waiting for before its node goes.

        :param timeout: Seconds to wait
        :return: Whether the client ended up with no goal in flight
        :rtype: bool
        """
        waited: float = 0.0
        while self._goal_in_flight and waited < timeout:
            time.sleep(self.config.feedback_check_period)
            waited += self.config.feedback_check_period
        return not self._goal_in_flight

    def __release_claim(self) -> None:
        """Give the client back without a goal having reached the server.

        Every way out of a send that leaves nothing running has to come through
        here: a claim held for a goal that was never sent refuses every later
        goal on this client, for good.
        """
        with self._goal_lock:
            self._goal_in_flight = False
            self._cancel_pending = False

    def send_request(
        self,
        request_msg: Any,
        wait_until_first_feedback: bool = False,
        still_wanted: Optional[Callable[[], bool]] = None,
    ) -> bool:
        """
        Sends a request to an action server

        :param request_msg: Action request message
        :type request_msg: Action_Type.Goal
        :param wait_until_first_feedback: Wait until the server returns its first feedback, defaults to True
        :type wait_until_first_feedback: bool, optional
        :param still_wanted: Asked while the request waits for the client and
            for the server, and once more just before it is sent. Once it says
            no, nothing is sent: a caller that stopped while it waited would
            otherwise start a goal that nothing is watching any more
        :type still_wanted: Callable[[], bool], optional

        :return: If action server is available
        :rtype: bool
        """
        wanted = still_wanted or (lambda: True)
        if not self.__claim_for_new_goal(wanted):
            if not wanted():
                self.node.get_logger().debug(
                    f"Not sending a goal to '{self.config.name}': its caller "
                    "stopped while it waited"
                )
                return False
            self.node.get_logger().error(
                f"Cannot send a goal to '{self.config.name}': a goal of this "
                "client's is still running on the server. Cancel it first"
            )
            return False
        # Making request to the server
        _path_timeout_count: float = 0.0
        # Wait until the server is available
        while not self.client.wait_for_server(
            timeout_sec=self.config.attempt_period_secs
        ):
            if not wanted():
                self.__release_claim()
                return False
            self.node.get_logger().info(
                "Waiting for Server node to become available...", once=True
            )

            _path_timeout_count += self.config.attempt_period_secs

            # timeout in attempt_period_secs
            if _path_timeout_count > self.config.timeout_secs:
                self.node.get_logger().error(
                    "Server node is not available - cannot start action service"
                )
                self.__release_claim()
                return False

        self.node.get_logger().debug(f"Sending request to {self.config.name}")

        # Check request type
        if not isinstance(request_msg, self.config.action_type.Goal):
            self.node.get_logger().error(
                f"Invalid request message for action '{self.config.name}'. Service takes request message of type '{self.config.action_type.Goal}', got '{type(request_msg)}'"
            )
            self.__release_claim()
            return False

        if not wanted():
            self.__release_claim()
            return False

        # If available, send request and get future response, and feedback callback method
        goal = self._goal_generation
        self._send_goal_future = self.client.send_goal_async(
            request_msg,
            feedback_callback=partial(
                self.__for_goal, goal, self.action_feedback_callback
            ),
        )

        self._start_time_secs = self.node.get_clock().now().nanoseconds / 1e9

        # Add method when action is done
        self._send_goal_future.add_done_callback(
            partial(self.__for_goal, goal, self.action_response_callback)
        )

        self._check_server_alive_timer = self.node.create_timer(
            timer_period_sec=self.config.feedback_check_timeout,
            callback=self._check_alive_callback,
        )

        _timeout_counter = 0
        while (
            not self.goal_accepted
            and not self.goal_rejected
            and _timeout_counter < self.config.feedback_check_timeout
        ):
            _timeout_counter += self.config.feedback_check_period
            time.sleep(self.config.feedback_check_period)

        if not self.goal_accepted and not self.goal_rejected:
            # The server never answered. Nothing of ours is running there, and
            # holding the client for it would refuse every later goal
            self.node.get_logger().error(
                f"No answer from '{self.config.name}' within "
                f"{self.config.feedback_check_timeout}s of sending the goal"
            )
            self.__release_claim()
            return False

        if wait_until_first_feedback:
            # Wait until the server sent the first feedback message
            _timeout_counter = 0
            while (
                not self.feedback_msg
                and _timeout_counter < self.config.feedback_check_timeout
            ):
                _timeout_counter += self.config.feedback_check_period
                time.sleep(self.config.feedback_check_period)
            if not self.feedback_msg:
                self.cancel_request()
                return False

        return self.goal_accepted

    # METHOD WHEN ACTION IS DONE
    def action_response_callback(self, future):
        """
        Callback when getting the action server responses

        :param future: Action result future
        :type future: Any
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.goal_rejected = True
            # Rejection is terminal: wake anyone parked on the result
            with self._goal_lock:
                self._goal_in_flight = False
                self._cancel_pending = False
            self._stop_alive_timer()
            self._notify_feedback_listeners()
            return
        self.goal_accepted = True

        with self._goal_lock:
            cancel_now = self._cancel_pending
            self._cancel_pending = False
        if cancel_now:
            # Cancelled while it was on its way to the server. Without this the
            # goal runs on, with whoever cancelled it believing it stopped
            self.node.get_logger().warning(
                f"Cancelling the goal on '{self.config.name}': it was cancelled "
                "before the server accepted it"
            )
            self._goal_handle.cancel_goal_async()

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            partial(self.__for_goal, self._goal_generation, self.action_result_callback)
        )
        return

    # METHOD TO GET THE RESULT WHEN DONE
    def action_result_callback(self, future):
        """
        Treats the path tracker action result

        :param future: Action result future
        :type future: Any
        """
        response = future.result()
        self.action_status = getattr(response, "status", GoalStatus.STATUS_UNKNOWN)
        self.action_result = response.result
        self.action_returned = True
        with self._goal_lock:
            self._goal_in_flight = False
        self._stop_alive_timer()
        # Notify listeners of the terminal transition (no further feedback).
        self._notify_feedback_listeners()

    def add_feedback_listener(self, listener: Callable[[], None]) -> None:
        """Register a zero-arg callback fired on every feedback message and on
        terminal state (in the ROS executor thread)."""
        self._feedback_listeners.add(listener)

    def remove_feedback_listener(self, listener: Callable[[], None]) -> None:
        """Remove a previously registered feedback listener."""
        self._feedback_listeners.discard(listener)

    def _notify_feedback_listeners(self) -> None:
        for listener in list(self._feedback_listeners):
            try:
                listener()
            except Exception:
                pass

    def action_feedback_callback(self, feedback_msg: Any):
        """
        Handles feedback messages received during action execution.
        :param feedback_msg: Action feedback message
        :type feedback_msg: Any
        """
        # Increase the feedback counter
        self.goal_accepted = True
        self.feedback_count += 1
        self.feedback_msg = feedback_msg
        self._notify_feedback_listeners()

    def _check_alive_callback(self):
        """Timed callback to check if server is sending a feedback"""
        # The goal already reached a terminal state; nothing left to watch
        if self.action_returned or self.goal_rejected:
            self._stop_alive_timer()
            return
        # New feedback got received within the timeout
        if self.feedback_count > self.old_feedback_count:
            self.old_feedback_count = self.feedback_count
        else:
            # No feedback is received
            self._feedback_timeout = True
            if self.config.cancel_on_feedback_timeout:
                self.cancel_request()

    def got_new_feedback(self) -> bool:
        """
        Checks if the client got a new feedback from the server within a specified time limit

        :return: Feedback updated on time
        :rtype: bool
        """
        # if did not get back wait and check
        _check_counter: float = 0.0
        while _check_counter < self.config.feedback_check_timeout:
            if self.feedback_count > self.old_feedback_count:
                self.old_feedback_count = self.feedback_count
                return True
            _check_counter += self.config.feedback_check_period
            time.sleep(self.config.feedback_check_period)
        return False

    def cancel_request(self, wait: bool = True) -> Tuple[bool, str]:
        """Cancel an active action goal and return result

        :param wait: Wait for the goal to return before reporting. Without it
            the cancel request is only sent, which is all that can be done once
            nothing spins to deliver the server's answer, e.g. at shutdown
        :type wait: bool
        :return: If cancellation is successful
        :rtype: Tuple[bool, str]
        """
        with self._goal_lock:
            if not self._goal_in_flight:
                # Nothing was sent, or it has already finished
                return (True, "No ongoing action goal to cancel")
            handle = self._goal_handle if self.goal_accepted else None
            if handle is None:
                # Sent, but the server has not answered yet, so there is no
                # handle to cancel. Cancelled the moment one arrives, rather
                # than reported as nothing to do while the goal starts running
                self._cancel_pending = True

        if handle is not None:
            handle.cancel_goal_async()
        if not wait:
            return (True, "Action goal cancel requested")

        # Wait for the goal to end, however it ends: cancelled, returned or
        # rejected. Whichever it is, the client is free afterwards
        _check_counter: float = 0.0
        while (
            not self.action_returned
            and not self.goal_rejected
            and _check_counter < self.config.feedback_check_timeout
        ):
            _check_counter += self.config.feedback_check_period
            time.sleep(self.config.feedback_check_period)
        if not self.action_returned and not self.goal_rejected:
            return (False, "Failed to cancel goal")
        # Wake parked waiters before reset() clears the terminal state
        self._notify_feedback_listeners()
        self.reset()
        return (True, "Action goal cancelled successfully")

    def get_ui_elements(self) -> Dict:
        """Get updated client elements for the UI

        :return: _description_
        :rtype: Dict
        """
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        ui_dict = {
            "status": self._status,
            "feedback": self.feedback_msg.feedback
            if self.feedback_msg and hasattr(self.feedback_msg, "feedback")
            else None,
            "timestep": self.feedback_count,
            "feedback_timeout": self._feedback_timeout,
            "duration_secs": (current_time - self._start_time_secs)
            if self._start_time_secs is not None
            else 0.0,
            # The goal's result message, None until the goal ends
            "result": self.action_result,
        }
        self._old_status = self._status
        return ui_dict
