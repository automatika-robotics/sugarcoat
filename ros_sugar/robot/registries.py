"""Action and Event registries for robot plugins.

A plugin exposes high-level, user-triggerable behaviours as **factories**:
calling ``plugin.actions.stand_up()`` constructs a fresh
`core.action.Action`, and ``plugin.events.low_battery(0.15)`` constructs a fresh
`core.event.Event`. Factories (rather than pre-built instances) let recipe authors
pass per-call arguments, plain values or ``MsgConditionBuilder`` references
from any topic.
"""

import inspect
from dataclasses import dataclass
from typing import Any, Callable, Dict, List

from ..core.action import Action
from ..core.event import Event


@dataclass
class ActionSpec:
    """Introspection record for one action factory."""

    name: str
    description: str
    signature: str


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
            )
            for name, factory in self._factories.items()
        ]


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


__all__ = ["ActionRegistry", "EventRegistry", "ActionSpec", "EventSpec"]
