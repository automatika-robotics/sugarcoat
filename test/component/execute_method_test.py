"""Tests the ExecuteMethod service against the (bool, str) action contract.

An action returns `(success, message)`. Over the wire that maps onto the three
`ExecuteMethod.srv` response fields: `success` carries the bool, and the string
lands in `response_json` when the action succeeded or in `error_msg` when it
failed. An action that wants to return something structured serializes it into
the string itself, which is the `return_payload` case below.
"""

import json
import unittest
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy

from ros_sugar.core import BaseComponent
from ros_sugar import Launcher
from ros_sugar.utils import ActionReturnType, component_action
from automatika_ros_sugar.srv import ExecuteMethod


EXPECTED_PAYLOAD = {"status": "ok", "count": 3, "items": ["a", "b"]}
SUCCESS_MESSAGE = "did the thing"
FAILURE_MESSAGE = "could not do the thing"


class ReturningComponent(BaseComponent):
    """Component whose actions cover both arms of the action contract."""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)

    def _execution_step(self):
        return

    @component_action
    def succeed(self) -> ActionReturnType:
        return True, SUCCESS_MESSAGE

    @component_action
    def fail(self) -> ActionReturnType:
        return False, FAILURE_MESSAGE

    @component_action
    def return_payload(self) -> ActionReturnType:
        """Structured output is carried as JSON inside the message string"""
        return True, json.dumps(EXPECTED_PAYLOAD)

    @component_action
    def raise_error(self) -> ActionReturnType:
        raise RuntimeError("boom")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    component = ReturningComponent(component_name="returning_component")
    component.loop_rate = 10.0

    launcher = Launcher()
    launcher.add_pkg(components=[component])
    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


class TestExecuteMethodResponse(unittest.TestCase):
    """Tests that ExecuteMethod maps an action's (bool, str) onto the response."""

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        cls.context.init()
        cls.node = rclpy.create_node(
            "test_execute_method_client", context=cls.context
        )
        cls.client = cls.node.create_client(
            ExecuteMethod, "returning_component/execute_method"
        )
        assert cls.client.wait_for_service(timeout_sec=30.0), (
            "execute_method service was not available within timeout"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.context.try_shutdown()

    def _call(self, method_name: str):
        req = ExecuteMethod.Request()
        req.name = method_name
        req.kwargs_json = ""
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=10.0,
            executor=rclpy.executors.SingleThreadedExecutor(context=self.context),
        )
        self.assertTrue(future.done(), f"Service call '{method_name}' did not complete")
        return future.result()

    def test_success_carries_the_message_in_response_json(self):
        resp = self._call("succeed")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(resp.response_json), SUCCESS_MESSAGE)
        self.assertEqual(resp.error_msg, "")

    def test_failure_carries_the_message_in_error_msg(self):
        """Regression: a (False, msg) tuple used to be reported as success.

        Before the action contract, a tuple fell through the type dispatch to
        the generic branch, so `success` was set True and the error text was
        JSON-dumped into `response_json`.
        """
        resp = self._call("fail")
        self.assertFalse(resp.success)
        self.assertEqual(resp.error_msg, FAILURE_MESSAGE)

    def test_structured_output_travels_as_json_in_the_message(self):
        resp = self._call("return_payload")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(json.loads(resp.response_json)), EXPECTED_PAYLOAD)

    def test_raised_exception_is_reported_as_failure(self):
        resp = self._call("raise_error")
        self.assertFalse(resp.success)
        self.assertIn("boom", resp.error_msg)

    def test_unknown_method_fails(self):
        resp = self._call("this_method_does_not_exist")
        self.assertFalse(resp.success)
        self.assertIn("does not have a method", resp.error_msg)
