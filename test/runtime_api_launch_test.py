"""Drives the Monitor's runtime API the way an external caller would.

Everything else registered behaviour by holding the Monitor object. Nothing
outside the launch process can do that, so this goes over ROS: one
`ExecuteMethod` service on a fixed name, taking a method name and JSON keyword
arguments. The point of the test is that a caller with only strings and a
service client can describe a mission, register it and watch it run.

The failure paths carry as much weight as the success one. A caller who cannot
read the recipe needs to be told what it got wrong: which names exist, which
actions a component offers, and which arguments a method takes.
"""

import json
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from automatika_ros_sugar.srv import ExecuteMethod
from example_interfaces.action import Fibonacci

from ros_sugar import Launcher
from ros_sugar.config import ComponentRunType
from ros_sugar.core import BaseComponent, Monitor
from ros_sugar.io import Topic
from ros_sugar.utils import ActionReturnType, component_action

READING_TOPIC = "reading"

# What the driver was asked to do, so a registration can be shown to have acted
driver_calls = []


class DriverComponent(BaseComponent):
    """What a mission described over the service reaches for"""

    def _execution_step(self):
        pass

    @component_action
    def note(self, value=None, **_) -> ActionReturnType:
        """Records what it was called with"""
        driver_calls.append(value)
        return True, f"noted {value}"


class ReadingPublisher(BaseComponent):
    """Publishes the topic a registered event watches"""

    def _execution_step(self):
        if self.publishers_dict.get(READING_TOPIC):
            self.publishers_dict[READING_TOPIC].publish(2.5)


class CountingComponent(BaseComponent):
    """A main action server, so a mission step can be a real goal"""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = Fibonacci
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        result = Fibonacci.Result()
        for _ in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)
        result.sequence = [goal_handle.request.order]
        goal_handle.succeed()
        return result


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    reading_topic = Topic(name=READING_TOPIC, msg_type="Float32")
    launcher = Launcher()
    launcher.add_pkg(
        components=[
            DriverComponent(component_name="driver"),
            CountingComponent(component_name="counter"),
            ReadingPublisher(component_name="publisher", outputs=[reading_topic]),
        ]
    )
    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


class TestRuntimeApi(unittest.TestCase):
    """A caller outside the launch, holding nothing but strings"""

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        cls.context.init()
        cls.node = rclpy.create_node("test_runtime_api_client", context=cls.context)
        cls.client = cls.node.create_client(
            ExecuteMethod, Monitor.RUNTIME_API_SERVICE
        )
        assert cls.client.wait_for_service(timeout_sec=30.0), (
            f"'{Monitor.RUNTIME_API_SERVICE}' was not available within timeout"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.context.try_shutdown()

    def call(self, name: str, **kwargs):
        request = ExecuteMethod.Request()
        request.name = name
        request.kwargs_json = json.dumps(kwargs) if kwargs else ""
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=15.0,
            executor=rclpy.executors.SingleThreadedExecutor(context=self.context),
        )
        assert future.done(), f"'{name}' did not complete"
        return future.result()

    def call_raw(self, name: str, kwargs_json: str):
        """For payloads that are not valid keyword arguments"""
        request = ExecuteMethod.Request()
        request.name = name
        request.kwargs_json = kwargs_json
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=15.0,
            executor=rclpy.executors.SingleThreadedExecutor(context=self.context),
        )
        assert future.done(), f"'{name}' did not complete"
        return future.result()

    def wait_for(self, predicate, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.2)
        return predicate()

    # ---- Finding out what is there ------------------------------------

    def test_a_caller_can_ask_what_the_stack_offers(self):
        """The entry point: with only strings, this is the way in"""
        response = self.call("list_actions")
        assert response.success, response.error_msg
        refs = {entry["ref"] for entry in json.loads(response.response_json)}
        assert "driver/note" in refs
        assert "counter/count" in refs
        assert "monitor/start_routine" in refs

    def test_the_listing_is_readable_without_decoding_twice(self):
        """`ros2 service call` has to show something a person can read"""
        response = self.call("list_routines")
        assert response.success, response.error_msg
        assert isinstance(json.loads(response.response_json), list)

    # ---- Registering and running a mission ----------------------------

    def test_a_routine_described_as_json_runs(self):
        """The whole point: a mission described from outside, with no recipe"""
        response = self.call(
            "add_routine",
            routine={
                "name": "over_the_wire",
                "steps": [
                    {"ref": "driver/note", "kwargs": {"value": "one"}, "name": "first"},
                    {"ref": "counter/count", "goal": {"order": 2}, "name": "count"},
                    {"ref": "driver/note", "kwargs": {"value": "two"}, "name": "last"},
                ],
            },
        )
        assert response.success, response.error_msg

        response = self.call("start_routine", routine_name="over_the_wire")
        assert response.success, response.error_msg

        def _completed() -> bool:
            state = self.call("get_routine_state", routine_name="over_the_wire")
            return state.success and json.loads(state.response_json)["status"] == "completed"

        assert self.wait_for(_completed), (
            self.call("get_routine_state", routine_name="over_the_wire").response_json
        )
        assert "one" in driver_calls and "two" in driver_calls

    def test_a_registered_routine_can_be_driven_and_dropped(self):
        assert self.call(
            "add_routine",
            routine={
                "name": "controllable",
                "steps": [{"ref": "counter/count", "goal": {"order": 40}}],
            },
        ).success

        assert self.call("start_routine", routine_name="controllable").success
        assert self.wait_for(
            lambda: json.loads(
                self.call("get_routine_state", routine_name="controllable").response_json
            )["status"]
            == "running"
        )

        assert self.call("pause_routine", routine_name="controllable").success
        assert self.call("resume_routine", routine_name="controllable").success
        assert self.call("abort_routine", routine_name="controllable").success

        response = self.call("remove_routine", routine_name="controllable", force=True)
        assert response.success, response.error_msg
        assert not self.call(
            "get_routine_state", routine_name="controllable"
        ).success

    def test_an_event_described_as_json_fires(self):
        """A condition on live data, named from outside the process"""
        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        response = self.call(
            "add_event",
            # The condition as a plain dict. It is accepted as JSON text too,
            # since that is what Event.to_dict emits
            event={
                "condition": (reading.msg.data > 1.0).to_dict(),
                "handle_once": True,
            },
            actions={"ref": "driver/note", "kwargs": {"value": "from_service"}},
            event_id="reading_high",
        )
        assert response.success, response.error_msg
        assert self.wait_for(lambda: "from_service" in driver_calls)
        assert "reading_high" in json.loads(self.call("list_events").response_json)

        assert self.call("remove_event", event_id="reading_high").success
        assert "reading_high" not in json.loads(self.call("list_events").response_json)

    def test_a_routine_can_dwell_between_steps(self):
        """A mission pauses at a waypoint, which needs a step that waits"""
        registered = self.call(
            "add_routine",
            routine={
                "name": "dwells",
                "steps": [
                    {"ref": "driver/note", "kwargs": {"value": "before"}, "name": "before"},
                    {"ref": "monitor/wait", "kwargs": {"duration": 1.0}, "name": "dwell"},
                    {"ref": "driver/note", "kwargs": {"value": "after"}, "name": "after"},
                ],
            },
        )
        assert registered.success, registered.error_msg

        started = time.time()
        assert self.call("start_routine", routine_name="dwells").success
        assert self.wait_for(
            lambda: json.loads(
                self.call("get_routine_state", routine_name="dwells").response_json
            )["status"]
            == "completed"
        )
        assert time.time() - started >= 1.0, "the routine did not actually wait"
        assert "after" in driver_calls

    # ---- Being told what is wrong -------------------------------------

    def test_an_unknown_api_method_lists_the_ones_that_exist(self):
        response = self.call("do_something_clever")
        assert not response.success
        assert "add_routine" in response.error_msg

    def test_an_unknown_action_ref_lists_what_the_component_offers(self):
        response = self.call(
            "add_routine",
            routine={"name": "bad_ref", "steps": [{"ref": "driver/fly"}]},
        )
        assert not response.success
        assert "driver/note" in response.error_msg

    def test_a_step_naming_nothing_is_refused(self):
        response = self.call(
            "add_routine", routine={"name": "no_ref", "steps": [{"kwargs": {}}]}
        )
        assert not response.success
        assert "ref" in response.error_msg

    def test_a_misspelt_goal_field_is_refused(self):
        """The silent case: set_ros_msg_from_dict would skip it and send a
        default constructed goal, so a mission would drive to the wrong place
        with nothing reported"""
        response = self.call(
            "add_routine",
            routine={
                "name": "typo",
                "steps": [{"ref": "counter/count", "goal": {"ordr": 2}}],
            },
        )
        assert not response.success
        assert "ordr" in response.error_msg
        assert "order" in response.error_msg

    def test_a_routine_with_no_steps_is_refused(self):
        response = self.call("add_routine", routine={"name": "empty", "steps": []})
        assert not response.success
        assert "no steps" in response.error_msg

    def test_malformed_json_is_refused_clearly(self):
        response = self.call_raw("list_actions", "{not json")
        assert not response.success
        assert "json" in response.error_msg.lower()

    def test_arguments_that_are_not_an_object_are_refused(self):
        response = self.call_raw("list_actions", "[1, 2, 3]")
        assert not response.success
        assert "json object" in response.error_msg

    def test_wrong_arguments_name_the_method(self):
        response = self.call("start_routine", not_a_parameter=1)
        assert not response.success
        assert "start_routine" in response.error_msg

    def test_a_callable_condition_cannot_be_described(self):
        """There is no way to put code in a payload, so say so"""
        response = self.call(
            "add_event",
            event={"handle_once": True},
            actions={"ref": "driver/note"},
            event_id="impossible",
        )
        assert not response.success
        assert "condition" in response.error_msg

    def test_the_api_does_not_reach_arbitrary_monitor_methods(self):
        """The allowlist is the whole security boundary of this service"""
        response = self.call("destroy_node")
        assert not response.success
        assert "Unknown runtime API method" in response.error_msg
