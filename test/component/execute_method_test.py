import json
import unittest
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy

from ros_sugar.core import BaseComponent
from ros_sugar import Launcher
from ros_sugar.utils import component_action
from automatika_ros_sugar.srv import ExecuteMethod


EXPECTED_DICT = {"status": "ok", "count": 3, "items": ["a", "b"]}
EXPECTED_INT = 42
EXPECTED_STRING = "hello"


class ReturningComponent(BaseComponent):
    """Component with component_action methods returning various non-bool types."""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)

    def _execution_step(self):
        return

    @component_action
    def return_dict(self):
        return EXPECTED_DICT

    @component_action
    def return_int(self):
        return EXPECTED_INT

    @component_action
    def return_string(self):
        return EXPECTED_STRING

    @component_action
    def return_none(self):
        return None

    @component_action
    def return_true(self) -> bool:
        return True

    @component_action
    def return_false(self) -> bool:
        return False

    @component_action
    def return_non_serializable(self):
        return object()


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
    """Tests that ExecuteMethod.srv response_json is populated for non-bool returns."""

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
            self.node, future, timeout_sec=10.0, executor=rclpy.executors.SingleThreadedExecutor(context=self.context)
        )
        self.assertTrue(future.done(), f"Service call '{method_name}' did not complete")
        return future.result()

    def test_dict_return_is_json_encoded(self):
        resp = self._call("return_dict")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(resp.response_json), EXPECTED_DICT)

    def test_int_return_is_json_encoded(self):
        resp = self._call("return_int")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(resp.response_json), EXPECTED_INT)

    def test_string_return_is_json_encoded(self):
        resp = self._call("return_string")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(resp.response_json), EXPECTED_STRING)

    def test_none_return_has_empty_response_json(self):
        resp = self._call("return_none")
        self.assertTrue(resp.success)
        self.assertEqual(resp.response_json, "")

    def test_true_bool_return(self):
        resp = self._call("return_true")
        self.assertTrue(resp.success)
        self.assertEqual(json.loads(resp.response_json), True)

    def test_false_bool_return(self):
        resp = self._call("return_false")
        self.assertFalse(resp.success)
        self.assertNotEqual(resp.error_msg, "")

    def test_non_serializable_return_sets_error(self):
        resp = self._call("return_non_serializable")
        self.assertTrue(resp.success)
        self.assertEqual(resp.response_json, "")
        self.assertIn("not JSON serializable", resp.error_msg)

    def test_unknown_method_fails(self):
        resp = self._call("this_method_does_not_exist")
        self.assertFalse(resp.success)
        self.assertIn("does not have a method", resp.error_msg)
