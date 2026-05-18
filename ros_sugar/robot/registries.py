"""Action and Event registries for robot plugins.

A plugin exposes high-level, user-triggerable behaviours as **factories**:
calling ``plugin.actions.stand_up()`` constructs a fresh
`core.action.Action`, and ``plugin.events.low_battery(0.15)`` constructs a fresh
`core.event.Event`. Factories (rather than pre-built instances) let recipe authors
pass per-call arguments, plain values or ``MsgConditionBuilder`` references
from any topic.

Action factories can carry an OpenAI-style tool description (decorate them
with `plugin_action` or stamp ``_tool_description`` directly). Downstream
LLM-driven components (e.g. in EmbodiedAgents) consume these via
`ActionRegistry.tool_descriptions`.
"""

import functools
import inspect
import json
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Union

from ..core.action import Action
from ..core.event import Event


def plugin_action(
    function: Optional[Callable] = None,
    description: Optional[Union[str, Dict]] = None,
):
    """Decorator that stamps an LLM-tool description on a plugin action factory.

    Mirrors ``utils.component_action`` for actions exposed through a robot
    plugin. The decorated callable is unchanged at runtime; it carries a
    ``_tool_description`` attribute that `ActionRegistry` surfaces.

    Usage::

        @plugin_action(description="Sit down or stand up from a sitting position")
        def _make_sit_stand(self) -> Action:
            ...

        @plugin_action(description={
            "name": "move_to",
            "description": "Drive to an absolute pose",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                },
                "required": ["x", "y"],
            },
        })
        def _make_move_to(self) -> Action:
            ...

    A plain string becomes the function description with a zero-arg parameter
    schema. A dict is taken as the OpenAI ``function`` block; missing fields
    are filled in by `ActionRegistry.tool_descriptions`.
    """

    def _decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def _wrapper(*args, **kwargs):
            return func(*args, **kwargs)

        _wrapper._tool_description = description  # type: ignore[attr-defined]
        return _wrapper

    if function is not None:
        return _decorator(function)
    return _decorator


@dataclass
class ActionSpec:
    """Introspection record for one action factory."""

    name: str
    description: str
    signature: str
    tool_description: Optional[Union[str, Dict]] = None


@dataclass
class EventSpec:
    """Introspection record for one event factory."""

    name: str
    description: str
    signature: str


class _FactoryRegistry:
    """Common base — attribute access returns the named factory callable."""

    _factories: Dict[str, Callable[..., Any]]

    def __init__(self, factories: Dict[str, Callable]) -> None:
        object.__setattr__(self, "_factories", dict(factories))

    def __getattr__(self, name: str) -> Callable:
        if name == "_factories":
            raise AttributeError(name)
        try:
            return self._factories[name]
        except KeyError:
            raise AttributeError(
                f"'{type(self).__name__}' has no entry '{name}'. "
                f"Available: {sorted(self._factories)}"
            ) from None

    def __contains__(self, name: str) -> bool:
        return name in self._factories

    def __dir__(self):
        return list(self._factories) + list(super().__dir__())

    def names(self) -> List[str]:
        """Return the registered factory names."""
        return list(self._factories)

    @staticmethod
    def _describe(factory: Callable) -> str:
        doc = inspect.getdoc(factory)
        return doc.splitlines()[0] if doc else ""

    @staticmethod
    def _signature(factory: Callable) -> str:
        try:
            return str(inspect.signature(factory))
        except (ValueError, TypeError):
            return "(...)"


class ActionRegistry(_FactoryRegistry):
    """Registry of factories producing `core.action.Action`.

    :param factories: Mapping of action name to a zero-or-more-arg callable that
        returns a fresh ``Action``.
    """

    def __init__(self, factories: Dict[str, Callable[..., Action]]) -> None:
        super().__init__(factories)

    def list(self) -> List[ActionSpec]:
        """Return introspection specs for every registered action."""
        return [
            ActionSpec(
                name=name,
                description=self._describe(factory),
                signature=self._signature(factory),
                tool_description=getattr(factory, "_tool_description", None),
            )
            for name, factory in self._factories.items()
        ]

    def tool_descriptions(
        self, namespace: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """Return OpenAI-style tool descriptions for every registered action.

        Each entry is a dict ready to drop into an LLM-tool list. The tool name
        is ``{namespace}.{action_name}`` when ``namespace`` is given.

        Sources, in order of preference:

        1. ``factory._tool_description`` stamped via `plugin_action`
           (string or OpenAI ``function`` dict). Missing schema fields are
           filled in with a zero-arg default.
        2. The factory's first docstring line — short description, zero-arg
           schema.
        3. A bare description string of ``"<no description>"``.
        """
        descriptions: List[Dict[str, Any]] = []
        for name, factory in self._factories.items():
            tool_name = f"{namespace}.{name}" if namespace else name
            raw = getattr(factory, "_tool_description", None)

            fn_block: Dict[str, Any]
            if isinstance(raw, dict):
                # Author supplied a function block (or already-wrapped tool dict)
                if "function" in raw and isinstance(raw["function"], dict):
                    fn_block = dict(raw["function"])
                else:
                    fn_block = dict(raw)
            elif isinstance(raw, str):
                fn_block = {"description": raw}
            else:
                fn_block = {
                    "description": self._describe(factory) or "<no description>"
                }

            fn_block["name"] = tool_name
            fn_block.setdefault(
                "parameters",
                {"type": "object", "properties": {}, "required": []},
            )
            descriptions.append({"type": "function", "function": fn_block})
        return descriptions

    def tool_descriptions_json(self, namespace: Optional[str] = None) -> str:
        """JSON-serialized form of `tool_descriptions`."""
        return json.dumps(self.tool_descriptions(namespace=namespace))


class EventRegistry(_FactoryRegistry):
    """Registry of factories producing `core.event.Event`.

    :param factories: Mapping of event name to a zero-or-more-arg callable that
        returns a fresh ``Event``.
    """

    def __init__(self, factories: Dict[str, Callable[..., Event]]) -> None:
        super().__init__(factories)

    def list(self) -> List[EventSpec]:
        """Return introspection specs for every registered event."""
        return [
            EventSpec(
                name=name,
                description=self._describe(factory),
                signature=self._signature(factory),
            )
            for name, factory in self._factories.items()
        ]


__all__ = [
    "ActionRegistry",
    "EventRegistry",
    "ActionSpec",
    "EventSpec",
    "plugin_action",
]
