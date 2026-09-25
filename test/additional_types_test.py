"""Tests for additional types passed to a component launched in its own process.

The launcher passes the supported types added by derived packages to every
process by name (``--additional_types``). The types must then be usable in the
topics the process parses.
"""

import json

import pytest
from std_msgs.msg import String as ROSString

from ros_sugar.core.component import BaseComponent
from ros_sugar.io import supported_types
from ros_sugar.io.topic import Topic


class UnregisteredStatus(supported_types.SupportedType):
    """A type defined in a module that does not register it, as a derived
    package that registers its types in another module"""

    _ros_type = ROSString


@pytest.fixture
def registry(monkeypatch):
    """A registry without the type, restored after the test"""
    types = dict(supported_types._additional_types)
    monkeypatch.setattr(supported_types, "_additional_types", types)
    return types


def test_passed_types_can_be_used_in_topics(registry):
    """Regression: the types were only kept on the component, which topics do
    not read, so topics of these types were rejected in the new process"""
    component = BaseComponent(component_name="additional_types_test")
    serialized = f"{UnregisteredStatus.__module__}.{UnregisteredStatus.__qualname__}"

    component.set_additional_types(json.dumps([serialized]))

    topic = Topic(name="status", msg_type="UnregisteredStatus")
    assert topic.msg_type is UnregisteredStatus
    assert UnregisteredStatus in component._additional_types
