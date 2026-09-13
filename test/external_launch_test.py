"""Tests for launching external ROS2 nodes and launch files via the Launcher.

Covers Launcher.add_ros_node and Launcher.include_launch_file — used by
recipes to bring up nodes such as a MoveIt move_group alongside components.
"""

import pytest
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node as NodeLaunchAction

from ros_sugar import Launcher


@pytest.fixture
def launcher():
    return Launcher()


def test_add_ros_node_appends_action(launcher):
    # A declared dependency, so it is installed wherever this suite runs;
    # `add_ros_node` now rejects a package that is not
    action = launcher.add_ros_node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="my_tf",
        remappings=[("tf_static", "/tf_static_test")],
    )
    assert isinstance(action, NodeLaunchAction)
    assert action in launcher._description.entities


def test_include_launch_file_from_path(launcher, tmp_path):
    launch_file = tmp_path / "external.launch.py"
    launch_file.write_text(
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription()\n"
    )
    action = launcher.include_launch_file(
        package=None, launch_file=str(launch_file), launch_args={"use_rviz": False}
    )
    assert isinstance(action, IncludeLaunchDescription)
    assert action in launcher._description.entities
    # launch args are stringified for the launch system
    assert ("use_rviz", "False") in [tuple(pair) for pair in action.launch_arguments]


def test_include_launch_file_missing_file_raises(launcher):
    with pytest.raises(FileNotFoundError):
        launcher.include_launch_file(
            package=None, launch_file="/nonexistent.launch.py"
        )


def test_include_launch_file_missing_in_package_raises(launcher):
    # automatika_ros_sugar is installed but has no such launch file
    with pytest.raises(FileNotFoundError):
        launcher.include_launch_file(
            package="automatika_ros_sugar", launch_file="no_such.launch.py"
        )


# ---- component launch_prefix (issue #63) -----------------------------------


def test_launch_prefix_reaches_the_component_process(launcher):
    """A prefix set on the component must end up on its process description,
    where ros2 launch prepends it to the command -- CPU pinning, priorities
    and profilers all hang off this."""
    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="pinned_component")
    component.launch_prefix = "taskset -c 4-7"

    action = launcher._build_component_launch_action(
        component, "automatika_ros_sugar", "executable"
    )

    prefix = action.process_description.prefix
    assert prefix is not None, "the prefix never reached the launch action"
    assert "".join(sub.text for sub in prefix) == "taskset -c 4-7"


def test_launch_prefix_defaults_to_no_prefix(launcher):
    """Passing prefix=None is upstream's own default: the prefix resolves to
    launch's global `launch-prefix` configuration, with no user text in it."""
    from launch.substitutions import TextSubstitution

    from ros_sugar.core.component import BaseComponent

    component = BaseComponent(component_name="unpinned_component")
    action = launcher._build_component_launch_action(
        component, "automatika_ros_sugar", "executable"
    )
    prefix = action.process_description.prefix
    assert not any(isinstance(sub, TextSubstitution) for sub in prefix)


def test_launch_prefix_in_thread_mode_warns(launcher, caplog, monkeypatch):
    """A component in a launcher thread has no process of its own, so a set
    prefix cannot apply -- that must be said, not silently ignored."""
    import logging

    from ros_sugar.core.component import BaseComponent
    from ros_sugar.launch import logger as launch_logger

    component = BaseComponent(component_name="threaded_pinned_component")
    component.launch_prefix = "taskset -c 4-7"

    # the launch logger does not propagate to the root logger caplog listens on
    monkeypatch.setattr(launch_logger, "propagate", True)
    with caplog.at_level(logging.WARNING, logger=launch_logger.name):
        launcher._setup_component_in_thread(component)

    assert "launch_prefix" in caplog.text
    assert "no effect" in caplog.text
