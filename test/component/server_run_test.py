import time
import unittest
from threading import Event as threadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from tf2_msgs.action import LookupTransform
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from ros_sugar.core import BaseComponent, Event
from ros_sugar import Launcher
from ros_sugar.config import ComponentRunType
from ros_sugar import actions
from ros_sugar.io import Topic

# Dummy service type for testing
from nav_msgs.srv import SetMap

# Threading Events
execution_service_py_event = threadingEvent()

#: Goals the action server component started executing, by count
started_goals = []


class ChildComponent(BaseComponent):
    """Child component to test component action"""

    def __init__(
        self,
        component_name,
        inputs=None,
        outputs=None,
        config=None,
        config_file=None,
        callback_group=None,
        fallbacks=None,
        main_action_type=None,
        main_srv_type=None,
        **kwargs,
    ):
        super().__init__(
            component_name,
            inputs,
            outputs,
            config,
            config_file,
            callback_group,
            fallbacks,
            main_action_type,
            main_srv_type,
            **kwargs,
        )
        self.service_type = SetMap

    def main_service_callback(self, _, response):
        global execution_service_py_event
        execution_service_py_event.set()
        return response


class CountingComponent(BaseComponent):
    """Counts slowly, so a goal is still ongoing when the next one arrives.

    tf2_msgs/LookupTransform is the action type only because it ships with
    tf2_ros, a declared dependency. The count travels as text in `target_frame`,
    and doubles as the goal's identity in `started_goals`.
    """

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = LookupTransform
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        count = int(goal_handle.request.target_frame)
        started_goals.append(count)
        result = LookupTransform.Result()
        for _ in range(count):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)
        goal_handle.succeed()
        return result


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Component publishing to the event topic
    component = ChildComponent(component_name="test_component")

    component.loop_rate = 10.0  # Hz
    component.run_type = ComponentRunType.SERVER

    # health status topic
    status_topic = Topic(name="test_component/status", msg_type="ComponentStatus")

    # Dummy event to send an automatic service call to the component main service post launch
    event_on_health_status = Event(status_topic, handle_once=True)
    srv_call = actions.send_srv_request(
        srv_name="test_component/set_map",
        srv_request_msg=SetMap.Request(),
        srv_type=SetMap,
    )

    launcher = Launcher()

    launcher.add_pkg(
        components=[component, CountingComponent(component_name="counter")],
        events_actions={event_on_health_status: srv_call},
        multiprocessing=False,
        ros_log_level="debug",
        rclpy_log_level="debug",
    )

    # Setup launch description without bringup for testing
    launcher.setup_launch_description()

    # Add ready for test action
    launcher._description.add_action(launch_testing.actions.ReadyToTest())

    # Return the launcher description for launch_testing
    return launcher._description


class TestActions(unittest.TestCase):
    """Tests that Component runtype SERVER works correctly"""

    wait_time = 10.0  # seconds

    def test_server_component(cls):
        global execution_service_py_event
        assert execution_service_py_event.wait(
            cls.wait_time
        ), "Server component did not run correctly"


class TestActionServer(unittest.TestCase):
    """Tests that 'cancel_main_action' cancels the ongoing goal of a Component
    with runtype ACTION_SERVER for any caller.

    The server takes one goal at a time, so a goal is only ever sent once the
    previous one has ended.
    """

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        cls.context.init()
        cls.node = rclpy.create_node("action_server_client", context=cls.context)
        cls.executor = rclpy.executors.SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.client = ActionClient(cls.node, LookupTransform, "counter/count")
        cls.cancel = cls.node.create_client(Trigger, "counter/cancel_main_action")
        assert cls.client.wait_for_server(timeout_sec=30.0), "no action server"
        assert cls.cancel.wait_for_service(timeout_sec=30.0), "no cancel service"

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        cls.context.try_shutdown()

    def setUp(self):
        started_goals.clear()
        #: Result of the goal this test sent last
        self.result = None

    def tearDown(self):
        # A goal left running by a failed test is canceled here, not carried
        # into the next test
        if self.result is not None and not self.result.done():
            self.cancel_ongoing()
            self.wait(self.result)

    def wait(self, future, timeout: float = 15.0):
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout, executor=self.executor
        )
        assert future.done(), "timed out"
        return future.result()

    def send(self, count: int):
        """Sends a goal and returns its result future"""
        assert self.result is None or self.result.done(), "a goal is still ongoing"
        goal = LookupTransform.Goal(target_frame=str(count))
        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted, f"goal {count} was rejected"
        self.result = handle.get_result_async()
        return self.result

    def wait_until_started(self, count: int):
        deadline = time.time() + 15.0
        while count not in started_goals and time.time() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        assert count in started_goals, f"goal {count} never started"

    def cancel_ongoing(self) -> Trigger.Response:
        return self.wait(self.cancel.call_async(Trigger.Request()))

    def test_the_cancel_service_cancels_the_ongoing_goal(self):
        result = self.send(200)
        self.wait_until_started(200)

        response = self.cancel_ongoing()
        assert response.success, response.message
        # Canceled through the regular path, so its own client sees it canceled
        assert self.wait(result).status == GoalStatus.STATUS_CANCELED

        # Once canceled, the server takes the next goal
        assert self.wait(self.send(2)).status == GoalStatus.STATUS_SUCCEEDED

    def test_canceling_with_no_ongoing_goal_says_so(self):
        response = self.cancel_ongoing()
        assert not response.success
        assert "No ongoing goal" in response.message
