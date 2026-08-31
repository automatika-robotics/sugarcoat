"""What a running stack can be asked to do, by name.

At launch an action is a Python callable a recipe already holds. At runtime a
caller has only a string, and no way of knowing which component owns the thing
it names. This registry is the translation between the two: it is built once
from the components the Launcher knows about, and every runtime reference is
resolved against it.

A reference is ``"owner/name"`` -- the component's node name and one of the
things it exposes:

```python
registry = SystemActionRegistry.from_components([planner, driver])
registry.get("driver/move_to_unblock")          # a @component_action method
registry.get("planner/plan_path")               # its main action server
registry.get("planner/save_plan_to_file")       # an additional service
registry.get("controller/track_vision_target")  # an additional action server
```

Resolution stops here: turning an entry into something callable needs the
Monitor's clients, so it lives on the Monitor. This module stays ROS free.

Everything a component exposes is registered, including the lifecycle actions
every component inherits: a mission routine legitimately wants
``driver/restart``. A consumer that must not offer some of them filters for
itself, on whatever it considers dangerous.

NOTE: unrelated to :class:`ros_sugar.robot.ActionRegistry`, which is a registry
of factories producing Actions for robot plugins.
"""

import inspect
import json
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from attrs import define, field

from ..config.base_attrs import BaseAttrs
from ..utils import get_methods_with_decorator

#: A method carrying @component_action or @component_fallback, run on its
#: component through its ExecuteMethod service
COMPONENT_METHOD = "component_method"
#: One of the component's action servers, main or additional
COMPONENT_ACTION_SERVER = "component_action_server"
#: One of the component's services, main or additional
COMPONENT_SERVICE = "component_service"
#: A method of the Monitor itself
MONITOR_METHOD = "monitor_method"

#: Owner of actions belonging to the Monitor rather than to any component
MONITOR_OWNER = "monitor"


def _describe(method: Optional[Callable]) -> str:
    """What this action says it does, in prose.

    ``@component_action(description=...)`` takes a dict, which the decorator
    stores as JSON. That form is written for an LLM tool schema, so the prose
    inside it is pulled out rather than handed on as a blob.
    """
    if method is None:
        return ""
    described = getattr(method, "_action_description", None)
    if described:
        text = str(described).strip()
        parsed = _description_schema(text)
        if parsed is not None:
            function = parsed.get("function", parsed)
            return str(function.get("description", "")).strip() or text
        return text
    return (getattr(method, "__doc__", "") or "").strip().split("\n")[0]


def _description_schema(described: Optional[str]) -> Optional[Dict]:
    """The tool schema behind a description, when it was given as one"""
    if not described:
        return None
    try:
        parsed = json.loads(described)
    except (json.JSONDecodeError, TypeError):
        return None
    return parsed if isinstance(parsed, dict) else None


def _signature(method: Optional[Callable]) -> str:
    """The call signature, as text, without the bound instance"""
    if method is None:
        return "(...)"
    try:
        parameters = inspect.signature(method).parameters
    except (TypeError, ValueError):
        return "(...)"
    return "(" + ", ".join(
        str(param) for name, param in parameters.items() if name != "self"
    ) + ")"


def _type_name(interface: Any) -> Optional[str]:
    """Readable name of an action or service type"""
    return getattr(interface, "__name__", None) if interface else None


@define(kw_only=True)
class RegisteredAction(BaseAttrs):
    """One thing the running stack can be asked to do.

    :param ref: How a caller names it, as "owner/name"
    :param owner: Node name of the component that runs it, or "monitor"
    :param name: The short name within that owner. Never contains a slash, so
        a reference always splits cleanly
    :param kind: Which resolution path applies, one of the module constants.
        The Monitor dispatches on this
    :param description: What it does, for a caller listing what is available
    :param signature: Its call signature, with the bound instance dropped
    :param interface_type: Name of the action or service type, for the kinds
        that have one
    :param server_name: The ROS name to address, for the kinds that have one.
        Needed because a server's name is not derivable from the reference
    :param in_process: False when the owner runs as its own process, which is
        the case a caller cannot reach by holding the object
    """

    ref: str = field()
    owner: str = field()
    name: str = field()
    kind: str = field()
    description: str = field(default="")
    signature: str = field(default="(...)")
    interface_type: Optional[str] = field(default=None)
    server_name: Optional[str] = field(default=None)
    in_process: bool = field(default=True)


class SystemActionRegistry:
    """Every action the running stack exposes, addressable by name.

    :param actions: Entries to start with, defaults to empty
    """

    def __init__(self, actions: Optional[Iterable[RegisteredAction]] = None) -> None:
        self._by_ref: Dict[str, RegisteredAction] = {}
        # The live action/service classes, kept beside the entries rather than
        # on them: an entry travels to a caller as JSON, and a class does not.
        # Whoever builds a client needs the class, and is always in-process
        self._interfaces: Dict[str, Any] = {}
        for action in actions or ():
            self.add(action)

    # ---- Reference grammar ------------------------------------------------

    @staticmethod
    def parse_ref(ref: str) -> Tuple[str, str]:
        """Split "owner/name" into its two halves.

        :param ref: The reference to split. A leading slash is tolerated, since
            a caller used to ROS topic names will write one
        :raises ValueError: If it is not exactly one owner and one name
        :rtype: Tuple[str, str]
        """
        if not isinstance(ref, str):
            raise ValueError(f"An action reference must be a string, got {type(ref)}")
        owner, separator, name = ref.strip().lstrip("/").partition("/")
        if not separator or not owner or not name or "/" in name:
            raise ValueError(
                f"'{ref}' is not a valid action reference. Expected "
                "'component_name/action_name'"
            )
        return owner, name

    @classmethod
    def normalize(cls, ref: str) -> str:
        """The canonical spelling of a reference, so lookups agree"""
        owner, name = cls.parse_ref(ref)
        return f"{owner}/{name}"

    @staticmethod
    def short_name(server_name: str, owner: str) -> str:
        """Reduce a ROS server name to something a reference can carry.

        Components name their entry points inconsistently: some prefix the node
        name, some do not. Both have to end up addressable, and a reference
        holds exactly one name, so what is left of the path is flattened.
        """
        name = server_name.strip().lstrip("/")
        prefix = f"{owner}/"
        if name.startswith(prefix):
            name = name[len(prefix):]
        return name.replace("/", "_")

    # ---- Contents ---------------------------------------------------------

    def add(self, action: RegisteredAction, interface: Any = None) -> None:
        """Register one action.

        :param action: What to register
        :param interface: The live action or service class, for the kinds that
            are reached through a client rather than by method name
        :raises ValueError: If the reference is taken by a different action.
            Silently keeping one of them would make which one a caller reaches
            a matter of ordering
        """
        ref = self.normalize(action.ref)
        existing = self._by_ref.get(ref)
        if existing is not None and existing != action:
            raise ValueError(
                f"Action reference '{ref}' is already registered as "
                f"{existing.kind}, and would be replaced by {action.kind}"
            )
        self._by_ref[ref] = action
        if interface is not None:
            self._interfaces[ref] = interface

    def interface_for(self, ref: str) -> Any:
        """The live action or service class behind a reference.

        :return: The class, or None for a kind that is reached by method name
        """
        return self._interfaces.get(self.normalize(ref), None)

    def get(self, ref: str) -> RegisteredAction:
        """Look one up.

        :raises KeyError: If unknown, naming what the owner does offer, or the
            known owners when the owner itself is unknown
        """
        key = self.normalize(ref)
        found = self._by_ref.get(key)
        if found is not None:
            return found
        owner, _ = self.parse_ref(key)
        known = self.refs(owner=owner)
        if known:
            raise KeyError(
                f"Unknown action '{ref}'. '{owner}' offers: {', '.join(known)}"
            )
        raise KeyError(
            f"Unknown action '{ref}'. Known owners: {', '.join(self.owners()) or 'none'}"
        )

    def __contains__(self, ref: str) -> bool:
        try:
            return self.normalize(ref) in self._by_ref
        except ValueError:
            return False

    def owners(self) -> List[str]:
        """Every owner that has at least one action, sorted"""
        return sorted({action.owner for action in self._by_ref.values()})

    def refs(self, owner: Optional[str] = None) -> List[str]:
        """Every reference, sorted, optionally only one owner's"""
        return sorted(
            ref
            for ref, action in self._by_ref.items()
            if owner is None or action.owner == owner
        )

    def list(
        self, owner: Optional[str] = None, kind: Optional[str] = None
    ) -> List[RegisteredAction]:
        """Every action, sorted by reference, narrowed by owner and/or kind"""
        return [
            self._by_ref[ref]
            for ref in self.refs(owner=owner)
            if kind is None or self._by_ref[ref].kind == kind
        ]

    @property
    def dictionary(self) -> List[Dict]:
        """Serialized form, for a caller asking what is available"""
        return [action.to_dict() for action in self.list()]

    # ---- Construction -----------------------------------------------------

    @classmethod
    def from_components(
        cls,
        components: Iterable,
        monitor_methods: Optional[Iterable[str]] = None,
        monitor_class: Optional[type] = None,
        out_of_process: Optional[Iterable[str]] = None,
    ) -> "SystemActionRegistry":
        """Build the registry from what the Launcher knows.

        :param components: The components in the stack
        :param monitor_methods: Names of Monitor methods a runtime caller may
            use. An allowlist, not introspection: the Monitor holds lifecycle
            power over every component, and most of its methods take Python
            objects no JSON payload could carry
        :param monitor_class: Class to read those methods' signatures off. The
            registry is built before the Monitor exists, so this is the class
            rather than an instance
        :param out_of_process: Node names launched as their own process, which
            is what makes an owner unreachable by holding its object
        :rtype: SystemActionRegistry
        """
        registry = cls()
        remote = set(out_of_process or ())
        seen_owners: set = set()

        for component in components:
            owner = getattr(component, "node_name", None)
            if not owner:
                continue
            if owner in seen_owners:
                # Every entry two same named components produce is identical,
                # so the clash is invisible at the reference level. It still
                # makes which component a reference reaches a matter of
                # ordering, and ROS will not run both under one node name
                raise ValueError(
                    f"Two components share the node name '{owner}'. An action "
                    "reference could not say which one it means"
                )
            seen_owners.add(owner)
            in_process = owner not in remote

            registry.__add_methods(component, owner, in_process)
            registry.__add_main_server(component, owner, in_process)
            registry.__add_entry_points(component, owner, in_process)

        for name in monitor_methods or ():
            method = getattr(monitor_class, name, None) if monitor_class else None
            registry.add(
                RegisteredAction(
                    ref=f"{MONITOR_OWNER}/{name}",
                    owner=MONITOR_OWNER,
                    name=name,
                    kind=MONITOR_METHOD,
                    description=_describe(method),
                    signature=_signature(method),
                )
            )
        return registry

    def __add_methods(self, component, owner: str, in_process: bool) -> None:
        """Register the component's decorated methods.

        Both decorators, because a fallback is a perfectly good thing to ask
        for deliberately; it is only its default use that is automatic.
        """
        actions = getattr(component, "available_actions", None) or []
        # Same mechanism available_actions itself uses, so if that worked this
        # does too
        fallbacks = get_methods_with_decorator(component, "component_fallback")

        for name in list(actions) + [f for f in fallbacks if f not in actions]:
            method = getattr(component, name, None)
            self.add(
                RegisteredAction(
                    ref=f"{owner}/{name}",
                    owner=owner,
                    name=name,
                    kind=COMPONENT_METHOD,
                    description=_describe(method),
                    signature=_signature(method),
                    in_process=in_process,
                )
            )

    def __add_main_server(self, component, owner: str, in_process: bool) -> None:
        """Register the one main server a component may run, if it runs one.

        Under its own name, like any other server. Being the component's main
        one is how it was declared, not something a caller addressing it needs
        to know.
        """
        run_type = str(getattr(component, "run_type", ""))

        if "ActionServer" in run_type and getattr(component, "action_type", None):
            self.__register_server(
                owner,
                component.main_action_name,
                component.action_type,
                COMPONENT_ACTION_SERVER,
                in_process,
            )
        elif "Server" in run_type and getattr(component, "service_type", None):
            self.__register_server(
                owner,
                component.main_srv_name,
                component.service_type,
                COMPONENT_SERVICE,
                in_process,
            )

    def __register_server(
        self,
        owner: str,
        server_name: str,
        interface: Any,
        kind: str,
        in_process: bool,
    ) -> None:
        """Register one of a component's servers, wherever it was declared from.

        :raises ValueError: If the name is already taken by something that is
            not this same server
        """
        name = self.short_name(server_name or "", owner)
        if not name:
            return
        ref = f"{owner}/{name}"

        existing = self._by_ref.get(ref, None)
        if existing is not None:
            if existing.kind == kind and existing.server_name == server_name:
                # A component is free to declare its main server in
                # get_ros_entrypoints too; that is the same server twice
                return
            raise ValueError(
                f"'{ref}' would name both the {existing.kind} "
                f"'{existing.server_name or existing.name}' and the {kind} "
                f"'{server_name}'. Rename one, so a caller can say which it means"
            )

        self.add(
            RegisteredAction(
                ref=ref,
                owner=owner,
                name=name,
                kind=kind,
                description=f"'{server_name}' of type {_type_name(interface)}",
                signature="(goal)" if kind == COMPONENT_ACTION_SERVER else "(request)",
                interface_type=_type_name(interface),
                server_name=server_name,
                in_process=in_process,
            ),
            interface=interface,
        )

    def __add_entry_points(self, component, owner: str, in_process: bool) -> None:
        """Register the servers a component runs beyond its main one.

        A component declares these itself, and they are often the interesting
        ones: a planner's file handling services, a controller's vision
        tracking action server.
        """
        getter = getattr(component, "get_ros_entrypoints", None)
        if not callable(getter):
            return
        # Deliberately not guarded: this is the component author's own
        # introspection, and a launch that fails here is better than a stack
        # that silently cannot reach half of what a component offers
        entry_points = getter() or {}

        for group, kind in (
            ("actions", COMPONENT_ACTION_SERVER),
            ("services", COMPONENT_SERVICE),
        ):
            for server_name, interface in (entry_points.get(group) or {}).items():
                self.__register_server(
                    owner, server_name, interface, kind, in_process
                )
