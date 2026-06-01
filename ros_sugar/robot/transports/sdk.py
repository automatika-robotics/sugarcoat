"""Vendor-SDK callback transport for robot plugins."""

from typing import Any, Callable, Optional

from . import Transport


class SdkCallbackTransport(Transport):
    """Generic shim around a vendor SDK's listener/sender API.

    The plugin author passes the SDK's "register a listener" function, its
    matching "unregister" function, and a "send" function. The transport wires
    the SDK's inbound callback to `_dispatch` so feedback decoders see
    each SDK object.

    :param name: Unique transport name.
    :type name: str
    :param register_fn: ``register_fn(callback)`` — hands the SDK a callable it
        invokes with each inbound message; may return a handle.
    :type register_fn: Optional[Callable[[Callable[[Any], None]], Any]]
    :param unregister_fn: ``unregister_fn(handle)`` — undoes ``register_fn``.
    :type unregister_fn: Optional[Callable[[Any], None]]
    :param send_fn: ``send_fn(payload)`` — sends a command through the SDK.
    :type send_fn: Optional[Callable[[Any], Any]]
    """

    def __init__(
        self,
        name: str,
        *,
        register_fn: Optional[Callable[[Callable[[Any], None]], Any]] = None,
        unregister_fn: Optional[Callable[[Any], None]] = None,
        send_fn: Optional[Callable[[Any], Any]] = None,
        **kwargs,
    ) -> None:
        super().__init__(name, **kwargs)
        self._register_fn = register_fn
        self._unregister_fn = unregister_fn
        self._send_fn = send_fn
        self._handle: Any = None

    def open(self) -> None:
        """Register the inbound callback with the SDK."""
        if self._open:
            return
        if self._register_fn is not None:
            self._handle = self._register_fn(self._dispatch)
        self._open = True

    def open_egress(self) -> None:
        """Sending only needs ``send_fn``; nothing to register."""
        self._open = True

    def close(self) -> None:
        """Unregister the inbound callback."""
        if self._unregister_fn is not None and self._handle is not None:
            self._unregister_fn(self._handle)
        self._handle = None
        self._open = False

    def send(self, payload: Any) -> bool:
        """Send ``payload`` through the SDK's send function."""
        if self._send_fn is None:
            return False
        self._send_fn(payload)
        return True


__all__ = ["SdkCallbackTransport"]
