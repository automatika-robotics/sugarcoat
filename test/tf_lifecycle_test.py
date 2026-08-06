"""Tests for the component's TF lookup lifecycle.

Transform lookups run on their own timers, one per frame pair, created lazily
as frames are discovered from incoming data. Those timers are not owned by
``destroy_all_timers`` (which only owns the execution timer), so the component
has to pause and resume them itself around deactivation.
"""

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped

from ros_sugar.config import BaseComponentConfig, ComponentRunType, RobotFrames
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


def _make_component(name: str) -> BaseComponent:
    config = BaseComponentConfig()
    config.frames = RobotFrames(robot_base="base_link", world="map")
    comp = BaseComponent(
        component_name=name,
        inputs=[Topic(name="/scan", msg_type="LaserScan")],
        config=config,
    )
    comp.rclpy_init_node()
    return comp


@pytest.fixture
def make_component(request):
    """Build components and tear them down with the test.

    Nodes must not outlive their test: these suites share one rclpy context
    with the launch_testing integration tests that run later in the session,
    and leaked live nodes destabilise it.
    """
    created = []

    def _factory(suffix: str = "") -> BaseComponent:
        name = request.node.name.replace("[", "_").replace("]", "_") + suffix
        comp = _make_component(name)
        created.append(comp)
        return comp

    yield _factory

    for comp in created:
        comp.destroy_node()


@pytest.fixture
def component(make_component):
    comp = make_component()
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


def test_same_frame_lookup_resolves_to_identity_without_polling(component):
    """Source and goal being the same frame is a normal configuration -- data
    that already arrives where it is wanted -- not a missing transform. It must
    not spin a timer or leave callers waiting on a lookup that could only ever
    return the identity."""
    listener = component.get_transform_listener("map", "map")

    assert listener.is_identity
    assert listener.got_transform, "callers poll this and would wait forever"
    assert listener.timer is None, "nothing to look up, so nothing to poll"
    assert listener.transform.header.frame_id == "map"
    assert listener.translation == pytest.approx([0.0, 0.0, 0.0])
    assert listener.rotation == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_same_frame_listener_survives_the_lifecycle(component):
    """It has no timer, so pause/resume must step over it rather than trip."""
    listener = component.get_transform_listener("map", "map")

    component.deactivate()
    component.activate()

    assert listener.got_transform
    assert component.get_transform_listener("map", "map") is listener


def test_distinct_frames_still_poll(component):
    """The identity shortcut must not swallow a real lookup."""
    listener = component.get_transform_listener("odom", "map")

    assert not listener.is_identity
    assert not listener.got_transform
    assert listener.timer is not None


def test_no_tf_machinery_is_created_when_unused(component):
    """A component that never asks for a transform should not pay for a buffer
    or the /tf and /tf_static subscriptions that come with it."""
    assert component._tf_buffer is None
    component.deactivate()  # must not blow up with nothing to pause
    assert component._tf_buffer is None


@pytest.mark.parametrize(
    "run_type",
    [ComponentRunType.TIMED, ComponentRunType.SERVER, ComponentRunType.ACTION_SERVER],
)
def test_teardown_is_safe_without_a_prior_activation(make_component, run_type):
    """Cleanup must tolerate whatever state the component reached: an
    activation that failed part-way, or a lifecycle transition that never
    activated at all, should not turn into an AttributeError on the way down.
    """
    comp = make_component()
    comp.run_type = run_type

    comp.deactivate()  # never activated


def test_teardown_is_idempotent(component):
    component.deactivate()
    component.deactivate()

    component.activate()
    component.deactivate()
