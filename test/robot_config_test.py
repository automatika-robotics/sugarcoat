"""Tests for ros_sugar.config.robot -- the robot description primitives.

These cover the two things that are easy to break silently: the serialization
round-trip (components are launched in their own process and rebuild their
config from JSON), and the string-enum behaviour that lets the description
cross package boundaries without every consumer importing the same enum class.
"""

import json
import math

import numpy as np
import pytest

from ros_sugar.config import (
    AngularCtrlLimits,
    BaseComponentConfig,
    LinearCtrlLimits,
    RobotConfig,
    RobotFrames,
    RobotGeometryType,
    RobotType,
)


def robot_config(**overrides) -> RobotConfig:
    """Build a RobotConfig, letting each test state only what it exercises.

    Every field is mandatory by design -- a robot's size and velocity envelope
    must not be silently defaulted -- so tests need a baseline to vary from.
    """
    return RobotConfig(**{
        "model_type": RobotType.ACKERMANN,
        "geometry_type": RobotGeometryType.CYLINDER,
        "geometry_params": [0.2, 1.0],
        "ctrl_vx_limits": LinearCtrlLimits(max_vel=1.0, max_acc=3.0, max_decel=5.0),
        "ctrl_omega_limits": AngularCtrlLimits(
            max_vel=5.0, max_steer=math.pi, max_acc=10.0, max_decel=10.0
        ),
        **overrides,
    })


# ---- Mandatory fields -----------------------------------------------------


def test_every_safety_relevant_field_is_mandatory():
    """A silently defaulted geometry or velocity envelope means collision
    checking and control run against a robot that does not exist."""
    with pytest.raises(TypeError, match="required keyword-only argument"):
        RobotConfig()


def test_no_state_is_shared_between_instances():
    """Mutable defaults evaluated once at class definition would be shared by
    every instance -- and with `robot` on every component config, that means
    cross-component contamination."""
    a, b = robot_config(), robot_config()
    assert a.geometry_params is not b.geometry_params
    assert a.ctrl_vy_limits is not b.ctrl_vy_limits

    a.geometry_params[0] = 99.0
    a.ctrl_vy_limits.max_vel = 42.0

    np.testing.assert_allclose(b.geometry_params, [0.2, 1.0])
    assert b.ctrl_vy_limits.max_vel == 0.0
    np.testing.assert_allclose(robot_config().geometry_params, [0.2, 1.0])


def test_caller_cannot_mutate_config_through_the_array_it_passed():
    params = np.array([0.2, 1.0])
    config = robot_config(geometry_params=params)
    params[0] = 77.0
    assert config.geometry_params[0] == 0.2


# ---- Frames ---------------------------------------------------------------


def test_frames_declare_only_what_must_be_chosen():
    """Sensor and localization frames come from the data, not from config."""
    frames = RobotFrames()
    assert frames.robot_base == "base_link"
    assert frames.world == "map"
    for derived in ("odom", "scan", "rgb", "depth", "point_cloud"):
        assert not hasattr(frames, derived), f"{derived} should be read from the data"


# ---- Enum behaviour -------------------------------------------------------


def test_model_type_compares_equal_to_its_name():
    """Regression: this comparison was always False when the config stored a
    plain string and consumers compared against a non-str Enum, so an Ackermann
    robot was allowed to attempt rotate-in-place."""
    config = robot_config(model_type=RobotType.ACKERMANN)
    assert config.model_type == RobotType.ACKERMANN
    assert config.model_type == "ACKERMANN"


def test_enums_serialize_as_plain_names():
    """Not as a Python repr like 'Type.CYLINDER', which is neither stable nor
    writable by hand in a config file."""
    payload = json.loads(robot_config().to_json())
    assert payload["model_type"] == "ACKERMANN"
    assert payload["geometry_type"] == "CYLINDER"


@pytest.mark.parametrize(
    "value", ["CYLINDER", RobotGeometryType.CYLINDER, "Type.CYLINDER"]
)
def test_geometry_type_accepts_equivalent_spellings(value):
    """Bare name, enum member, and the legacy serialized form all normalize.

    The legacy form matters because configs written by earlier releases stored
    the Python repr.
    """
    assert robot_config(geometry_type=value).geometry_type is RobotGeometryType.CYLINDER


def test_unknown_enum_value_is_rejected():
    with pytest.raises(ValueError, match="not a valid RobotGeometryType"):
        robot_config(geometry_type="TRIANGLE")


# ---- Geometry validation --------------------------------------------------


@pytest.mark.parametrize(
    "geometry, params",
    [
        (RobotGeometryType.BOX, [1.0, 2.0]),  # needs 3
        (RobotGeometryType.SPHERE, [1.0, 2.0]),  # needs 1
        (RobotGeometryType.CYLINDER, [0.2, -1.0]),  # must be positive
        (RobotGeometryType.CYLINDER, [0.2, 0.0]),  # strictly positive
    ],
)
def test_geometry_params_must_match_geometry(geometry, params):
    with pytest.raises(ValueError, match="incompatible with given robot geometry"):
        robot_config(geometry_type=geometry, geometry_params=params)


def test_valid_geometry_params_are_kept_as_array():
    config = robot_config(
        geometry_type=RobotGeometryType.BOX, geometry_params=[0.61, 0.37, 0.4]
    )
    assert isinstance(config.geometry_params, np.ndarray)
    np.testing.assert_allclose(config.geometry_params, [0.61, 0.37, 0.4])


# ---- Serialization round-trip (the multiprocess launch path) --------------


def test_round_trip_restores_types_not_just_values():
    """Components rebuild their config from JSON in a separate process, so the
    rebuilt object must be usable exactly like the original."""
    original = robot_config(
        model_type=RobotType.DIFFERENTIAL_DRIVE,
        geometry_type=RobotGeometryType.BOX,
        geometry_params=np.array([0.61, 0.37, 0.4]),
        ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=2.5, max_decel=7.5),
    )

    rebuilt = robot_config()
    rebuilt.from_json(original.to_json())

    assert isinstance(rebuilt.model_type, RobotType)
    assert isinstance(rebuilt.geometry_type, RobotGeometryType)
    assert isinstance(rebuilt.geometry_params, np.ndarray)
    assert rebuilt.model_type == original.model_type
    assert rebuilt.geometry_type == original.geometry_type
    np.testing.assert_allclose(rebuilt.geometry_params, original.geometry_params)
    assert rebuilt.ctrl_vx_limits.max_decel == 7.5


def test_component_config_defaults_to_no_robot():
    """A component that never reasons about the robot's body should not be
    forced to describe one, and must not be handed an invented default."""
    config = BaseComponentConfig()
    assert config.robot is None

    rebuilt = BaseComponentConfig()
    rebuilt.from_json(config.to_json())
    assert rebuilt.robot is None


def test_component_config_carries_robot_and_frames():
    """Both ride the same JSON payload into the component subprocess."""
    config = BaseComponentConfig()
    config.robot = robot_config(geometry_type=RobotGeometryType.SPHERE,
                                geometry_params=[0.5])
    config.frames = RobotFrames(robot_base="body", world="odom")

    rebuilt = BaseComponentConfig()
    rebuilt.from_json(config.to_json())

    assert rebuilt.robot.geometry_type is RobotGeometryType.SPHERE
    assert rebuilt.frames.robot_base == "body"
    assert rebuilt.frames.world == "odom"
