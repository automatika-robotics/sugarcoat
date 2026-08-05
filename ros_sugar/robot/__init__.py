"""General-purpose robot plugin framework for Sugarcoat.

A robot plugin adapts a specific robot's control surface; ROS topics, UDP/HTTP
endpoints, or a vendor SDK, to Sugarcoat's standard component I/O, and may
contribute high-level Action factories and pre-built Event factories that
recipes consume directly.

Authors subclass `RobotPlugin`; see ``docs/development/custom_robot_plugin.md``.
"""

from .bus import FeedbackBus, InProcessFeedbackBus, SocketFeedbackBus
from .command import CommandSpec, RobotCommand
from .feedback import Feedback, FeedbackSpec
from .mount import Mount
from .plugin import (
    AmbiguousPluginEntryError,
    Plugin,
    PluginMetadata,
    PluginRole,
    RobotPlugin,
    SensorPlugin,
    RobotPluginHost,
)
from .registries import (
    ActionRegistry,
    ActionSpec,
    EventRegistry,
    EventSpec,
    plugin_action,
)
from .transports import SubscriptionHandle, Transport
from .transports.http import HttpTransport
from .transports.ros import RosServiceTransport, RosTopicTransport
from .transports.sdk import SdkCallbackTransport
from .transports.udp import UdpTransport
from .types import create_supported_type

__all__ = [
    # plugin
    "Mount",
    "Plugin",
    "PluginRole",
    "RobotPlugin",
    "SensorPlugin",
    "RobotPluginHost",
    "PluginMetadata",
    "AmbiguousPluginEntryError",
    # descriptors
    "Feedback",
    "FeedbackSpec",
    "RobotCommand",
    "CommandSpec",
    # registries
    "ActionRegistry",
    "EventRegistry",
    "ActionSpec",
    "EventSpec",
    "plugin_action",
    # transports
    "Transport",
    "SubscriptionHandle",
    "RosTopicTransport",
    "RosServiceTransport",
    "UdpTransport",
    "HttpTransport",
    "SdkCallbackTransport",
    # bus
    "FeedbackBus",
    "InProcessFeedbackBus",
    "SocketFeedbackBus",
    # types
    "create_supported_type",
]
