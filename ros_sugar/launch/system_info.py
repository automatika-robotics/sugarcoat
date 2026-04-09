"""Utilities for building the system info manifest used by the UI visualization."""

from typing import Any, Dict, List, Optional, Sequence

from ..condition import ConditionLogicOp, ConditionOperators, Condition
from ..core.action import Action
from ..core.component import BaseComponent
from ..core.event import Event
from ..io.topic import Topic


# Internal ROS topics to exclude from the visualization
_INTERNAL_TOPIC_SUFFIXES = (
    "status",
    "transition_event",
    "describe_parameters",
    "get_parameters",
    "get_parameter_types",
    "list_parameters",
    "set_parameters",
    "set_parameters_atomically",
)
_INTERNAL_TOPIC_NAMES = {"parameter_events", "rosout", "clock"}


def is_internal_topic(name: str) -> bool:
    """Check if a topic name is a ROS2 internal/infrastructure topic."""
    stripped = name.lstrip("/")
    if stripped in _INTERNAL_TOPIC_NAMES:
        return True
    return any(stripped.endswith(s) for s in _INTERNAL_TOPIC_SUFFIXES)


def serialize_topics(topics: Optional[Sequence[Topic]]) -> List[Dict[str, str]]:
    """Serialize a list of Topic objects to compact dicts, filtering out internal topics."""
    if not topics:
        return []
    result = []
    for t in topics:
        if is_internal_topic(t.name):
            continue
        msg_name = (
            t.msg_type.__name__ if hasattr(t.msg_type, "__name__") else str(t.msg_type)
        )
        result.append({"name": t.name, "msg_type": msg_name})
    return result


def serialize_component(component: BaseComponent) -> Dict[str, Any]:
    """Serialize a single component's metadata for the UI visualization."""
    return {
        "run_type": str(component.run_type),
        "component_type": type(component).__name__,
        "main_action_name": component.main_action_name,
        "main_srv_name": component.main_srv_name,
        "in_topics": serialize_topics(getattr(component, "in_topics", None)),
        "out_topics": serialize_topics(getattr(component, "out_topics", None)),
    }


def get_condition_parts(condition: Condition) -> List[Dict[str, str]]:
    """Recursively extract structured parts from a Condition tree for UI display.

    Returns a flat list of typed parts:
    - {"type": "topic", "value": "topic_name"}
    - {"type": "attribute", "value": "data.field"}
    - {"type": "operator", "value": "greater than"}
    - {"type": "ref_value", "value": "5.0"}
    - {"type": "logic", "value": "AND"}
    """
    if condition.logic_operator != ConditionLogicOp.NONE:
        parts = []
        for i, sub in enumerate(condition.sub_conditions):
            if i > 0:
                parts.append({"type": "logic", "value": condition.logic_operator.name})
            parts.extend(get_condition_parts(sub))
        return parts

    topic_name = condition.topic_name or ""
    if condition.operator_func:
        op_name = ConditionOperators.get_name(condition.operator_func)
        attr_path = (
            ".".join(str(a) for a in condition.attribute_path)
            if condition.attribute_path
            else ""
        )
        return [
            {"type": "topic", "value": topic_name},
            {"type": "attribute", "value": attr_path},
            {"type": "operator", "value": op_name.replace("_", " ")},
            {"type": "ref_value", "value": str(condition.ref_value)},
        ]
    return [
        {"type": "topic", "value": topic_name},
        {"type": "attribute", "value": "any"},
    ]


def classify_action(action) -> Dict[str, Any]:
    """Classify an action and return a compact serializable dict."""
    from launch.action import Action as ROSLaunchAction

    if isinstance(action, ROSLaunchAction) and not isinstance(action, Action):
        return {"name": type(action).__name__, "component": None, "type": "launch"}

    if isinstance(action, Action):
        if action._is_lifecycle_action:
            a_type = "lifecycle"
        elif action._is_monitor_action:
            a_type = "monitor"
        elif action.component_action:
            a_type = "component"
        else:
            a_type = "launch"
        return {
            "name": action.action_name,
            "component": action.parent_component,
            "type": a_type,
        }

    return {"name": str(action), "component": None, "type": "unknown"}


def serialize_event(event: Event, actions_list: list) -> Dict[str, Any]:
    """Serialize a single event and its associated actions for UI visualization."""
    return {
        "id": event.id,
        "readable": event._condition._readable(),
        "condition_parts": get_condition_parts(event._condition),
        "handle_once": event._handle_once,
        "on_change": event._on_change,
        "keep_event_delay": event._keep_event_delay,
        "involved_topics": [t.name for t in event.get_involved_topics()],
        "actions": [classify_action(a) for a in actions_list],
    }


_FALLBACK_TIERS = (
    "on_any_fail",
    "on_component_fail",
    "on_algorithm_fail",
    "on_system_fail",
    "on_giveup",
)


def serialize_fallbacks(component: BaseComponent) -> Dict[str, Any]:
    """Serialize a component's fallback configuration for UI visualization."""
    comp_fallbacks = component._BaseComponent__fallbacks
    result = {}
    for tier in _FALLBACK_TIERS:
        fallback = getattr(comp_fallbacks, tier, None)
        if fallback is None:
            result[tier] = None
            continue
        try:
            actions = fallback.action
            if isinstance(actions, list):
                action_names = [a.action_name for a in actions]
            else:
                action_names = [actions.action_name]
        except Exception:
            action_names = ["unknown"]
        result[tier] = {"actions": action_names, "max_retries": fallback.max_retries}
    return result
