"""Transport abstractions for robot plugins.

A `Transport` abstracts where robot data comes from and goes to. The
robot plugin owns its transports; the plugin **HOST** (running in the launcher
process) opens them, while component processes either consume decoded feedback
from the `FeedbackBus` or send commands through a send-only (egress) transport.
"""

from abc import ABC, abstractmethod
from typing import Any, Callable, List, Optional


class SubscriptionHandle:
    """Returned by `Transport.subscribe` call `unsubscribe` to detach."""

    def __init__(self, unsubscribe: Callable[[], None]) -> None:
        self._unsubscribe = unsubscribe
        self._active = True

    def unsubscribe(self) -> None:
        """Detach the callback registered with `Transport.subscribe`."""
        if self._active:
            self._unsubscribe()
            self._active = False


class Transport(ABC):
    """Base class for all robot plugin transports.

    :param name: Unique transport name within the plugin.
    :type name: str
    :param keep_alive_fn: Optional callable invoked periodically by the launcher
        while the plugin is active (e.g. a heartbeat packet).
    :type keep_alive_fn: Optional[Callable[[], None]]
    :param keep_alive_rate_hz: Rate at which ``keep_alive_fn`` is invoked.
    :type keep_alive_rate_hz: Optional[float]
    :param route_via_host: If True, commands on this transport are forwarded to
        the plugin HOST over the feedback bus instead of being sent directly
        from the component process. Use for session/heartbeat-bound protocols
        where a single client must own the connection.
    :type route_via_host: bool
    """

    def __init__(
        self,
        name: str,
        *,
        keep_alive_fn: Optional[Callable[[], None]] = None,
        keep_alive_rate_hz: Optional[float] = None,
        route_via_host: bool = False,
    ) -> None:
        self.name = name
        self.keep_alive_fn = keep_alive_fn
        self.keep_alive_rate_hz = keep_alive_rate_hz
        self.route_via_host = route_via_host
        self._subscribers: List[Callable[[Any], None]] = []
        self._open = False

    # lifecycle
    @abstractmethod
    def open(self) -> None:
        """Open the transport fully (ingress + egress). Called once on the HOST."""
        raise NotImplementedError

    def open_egress(self) -> None:
        """Open the transport for sending only.

        Called in component processes that issue commands on this transport
        directly (i.e. ``route_via_host`` is False). Defaults to `open`
        for transports where there is no separate egress-only mode.
        """
        self.open()

    @abstractmethod
    def close(self) -> None:
        """Close the transport and release all resources."""
        raise NotImplementedError

    def is_open(self) -> bool:
        """Return whether the transport is currently open."""
        return self._open

    # data flow
    @abstractmethod
    def send(self, payload: Any) -> bool:
        """Send a payload to the robot. Returns True on success."""
        raise NotImplementedError

    def subscribe(self, on_msg: Callable[[Any], None]) -> SubscriptionHandle:
        """Register a callback invoked with each raw inbound payload.

        Only meaningful on the HOST. The raw payload type is transport-specific
        (``bytes`` for UDP, the HTTP response body for HTTP, the SDK object for
        SDK callbacks); a `Feedback` decoder turns it into a ROS message.
        """
        self._subscribers.append(on_msg)

        def _unsub() -> None:
            if on_msg in self._subscribers:
                self._subscribers.remove(on_msg)

        return SubscriptionHandle(_unsub)

    # helpers
    def _dispatch(self, payload: Any) -> None:
        """Fan a raw inbound payload out to every registered subscriber."""
        for on_msg in list(self._subscribers):
            on_msg(payload)

    @property
    def kind(self) -> str:
        """Short transport-kind label used for introspection."""
        return type(self).__name__


__all__ = ["Transport", "SubscriptionHandle"]
