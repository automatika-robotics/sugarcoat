"""Dynamic ``SupportedType`` construction for robot plugins.

``create_supported_type`` wraps a robot's custom ROS message in a
`io.supported_types.SupportedType` subclass, optionally with a
``converter`` (python output -> ROS message, for commands) and a ``callback``
(ROS message -> python value, for feedback).
"""

import inspect
from typing import Callable, Optional, Type

from ..io.callbacks import GenericCallback
from ..io.supported_types import SupportedType, add_additional_datatypes


def create_supported_type(
    ros_msg_type: type,
    converter: Optional[Callable] = None,
    callback: Optional[Callable] = None,
) -> Type[SupportedType]:
    """Add a new ``SupportedType`` derived class wrapping ``ros_msg_type``.

    :param ros_msg_type: A valid ROS2 message type.
    :param converter: ``converter(output) -> ros_msg_type`` — python value to ROS
        message; its return annotation must equal ``ros_msg_type``.
    :param callback: ``callback(msg: ros_msg_type) -> <python type>`` — ROS
        message to python value; its first-arg annotation must equal
        ``ros_msg_type`` and its return annotation must be a non-ROS python type.
    :return: The newly registered ``SupportedType`` subclass.
    """
    if not hasattr(ros_msg_type, "SLOT_TYPES"):
        raise TypeError("ros_msg_type must be a valid ROS2 message type")

    # Dynamically create the new class
    class_name = f"{ros_msg_type.__name__}"
    class_bases = (SupportedType,)
    class_attrs = {
        "_ros_type": ros_msg_type,
    }

    if converter:
        converter_sig = inspect.signature(converter)
        c_params = list(converter_sig.parameters.keys())

        if len(c_params) < 1:
            raise TypeError("Converter must take at least one parameter")

        if converter_sig.return_annotation != ros_msg_type:
            raise TypeError(
                f"Converter return annotation ({converter_sig.return_annotation}) "
                f"must match ros_msg_type ({ros_msg_type})"
            )

        def _convert(cls: object, *args, **kwargs):
            return converter(*args, **kwargs)

        class_attrs["convert"] = classmethod(_convert)

    if callback:
        callback_sig = inspect.signature(callback)
        cb_params = list(callback_sig.parameters.values())

        if not cb_params:
            raise TypeError("Callback function must accept at least one argument.")

        if cb_params[0].annotation != ros_msg_type:
            raise TypeError(
                f"Callback's first argument annotation ({cb_params[0].annotation}) "
                f"must match ros_msg_type ({ros_msg_type})"
            )

        if (
            callback_sig.return_annotation == ros_msg_type
            or callback_sig.return_annotation == inspect.Parameter.empty
        ):
            raise TypeError(
                "Callback must have a return annotation of a non-ROS, Python type "
                "(e.g., str, int, Dict)."
            )

        def _get_output(self: object, **_):
            return callback(self.msg, **_)

        callback_class_attrs = {
            "_get_output": _get_output,
        }
        callback_class = type(
            f"{class_name}Callback", (GenericCallback,), callback_class_attrs
        )
        class_attrs["callback"] = callback_class

    new_type = type(class_name, class_bases, class_attrs)
    add_additional_datatypes([new_type])
    return new_type


__all__ = ["create_supported_type"]
