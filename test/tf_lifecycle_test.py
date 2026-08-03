"""Tests for the component's TF lookup lifecycle.

Transform lookups run on their own timers, one per frame pair, created lazily
as frames are discovered from incoming data. Those timers are not owned by
``destroy_all_timers`` (which only owns the execution timer), so the component
has to pause and resume them itself around deactivation.
"""

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped

from ros_sugar.config import BaseComponentConfig, RobotFrames
from ros_sugar.core.component import BaseComponent
from ros_sugar.io.topic import Topic


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    # Other suites in this package init rclpy and never shut down; tolerate it
    if not rclpy.ok():
        rclpy.init()
    yield


def make_tf(parent: str, child: str, x: float = 0.0) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    return transform


@pytest.fixture
def component(request):
    config = BaseComponentConfig()
    config.frames = RobotFrames(robot_base="base_link", world="map")
    comp = BaseComponent(
        component_name=request.node.name.replace("[", "_").replace("]", "_"),
        inputs=[Topic(name="/scan", msg_type="LaserScan")],
        config=config,
    )
    comp.rclpy_init_node()
    comp.activate()
    return comp


def test_lookup_timers_stop_on_deactivate(component):
    """Otherwise a deactivated component keeps polling TF forever."""
    dynamic = component.get_transform_listener("odom", "map", static_tf=False)
    static = component.get_transform_listener("lidar_link", "base_link", static_tf=True)
    assert not dynamic.timer.is_canceled()
    assert not static.timer.is_canceled()

    component.deactivate()

    assert dynamic.timer.is_canceled()
    assert static.timer.is_canceled()


def test_lookups_resume_on_reactivate(component):
    dynamic = component.get_transform_listener("odom", "map", static_tf=False)
    component.deactivate()
    assert dynamic.timer.is_canceled()

    component.activate()

    assert not dynamic.timer.is_canceled()


def test_reactivate_does_not_restart_an_acquired_static_lookup(component):
    """A static transform cancels its own timer once resolved; resuming must
    not undo that, or the saving is lost after the first restart."""
    static = component.get_transform_listener("lidar_link", "base_link", static_tf=True)
    component.tf_buffer.set_transform_static(
        make_tf("base_link", "lidar_link", x=0.3), "unit_test"
    )
    static.timer_callback()
    assert static.got_transform
    assert static.timer.is_canceled(), "static lookup should stop polling once acquired"

    component.deactivate()
    component.activate()

    assert static.timer.is_canceled()


def test_resolved_transforms_survive_a_restart(component):
    """Pausing must not discard the buffer or the listener cache, or every
    component restart would re-pay the lookup latency."""
    component.tf_buffer.set_transform(make_tf("map", "odom", x=5.0), "unit_test")
    dynamic = component.get_transform_listener("odom", "map", static_tf=False)
    dynamic.timer_callback()
    assert dynamic.got_transform

    buffer_before = component.tf_buffer
    component.deactivate()
    component.activate()

    assert component.tf_buffer is buffer_before
    assert component.get_transform_listener("odom", "map") is dynamic
    assert dynamic.got_transform
    assert dynamic.transform.transform.translation.x == 5.0


def test_no_tf_machinery_is_created_when_unused(component):
    """A component that never asks for a transform should not pay for a buffer
    or the /tf and /tf_static subscriptions that come with it."""
    assert component._tf_buffer is None
    component.deactivate()  # must not blow up with nothing to pause
    assert component._tf_buffer is None
