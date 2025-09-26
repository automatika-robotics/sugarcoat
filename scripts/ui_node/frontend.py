import logging
from datetime import datetime
from typing import Dict
from .utils import parse_type

try:
    from fasthtml.common import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML with `pip install python-fasthtml`"
    ) from e


class FHApp:
    def __init__(self, configs: Dict):
        # --- Application Setup ---
        self.app, self.rt = fast_app(hdrs=(picolink), exts="ws", live=True)
        if not configs:
            logging.warning("No component configs provided to the UI")

        # Create settings UI
        self.settings = self._create_settings_ui(configs)

    def get_app(self):
        """Get the FastHTML app"""
        return self.app, self.rt

    # -- Utility Functions --
    def _create_ui_element(self, setting_name, setting_details):
        """Creates a UI element based on the setting's type and validators."""
        field_type, type_args = parse_type(setting_details.get("type", ""))
        validators = setting_details.get("validators", [])
        value = setting_details.get("value")

        # Helper to generate select options with the correct item selected
        def generate_options(options_list, selected_value):
            opts = []
            for opt_val in options_list:
                if str(opt_val) == str(selected_value):
                    opts.append(Option(opt_val, value=opt_val, selected=True))
                else:
                    opts.append(Option(opt_val, value=opt_val))
            return opts

        # Handle validators first
        if validators:
            validator = validators[0]
            validator_name = list(validator.keys())[0]
            validator_props = validator[validator_name]

            if validator_name == "in":
                options = validator_props.get("ref_value", [])
                return Select(
                    Select(*generate_options(options, value), id=setting_name),
                    label=setting_name,
                )

            elif validator_name in ["in_range", "in_range_discretized"]:
                return Input(
                    type="range",
                    cl="slider",
                    label=setting_name,
                    id=setting_name,
                    min=validator_props.get("min_value", 0),
                    max=validator_props.get("max_value", 100),
                    step=validator_props.get("step", 1),
                    value=value,
                )

            elif validator_name == "greater_than":
                return Input(
                    label=setting_name,
                    id=setting_name,
                    type="number",
                    min=validator_props.get("ref_value", 0),
                    value=value,
                )

            elif validator_name == "less_than":
                return Input(
                    label=setting_name,
                    id=setting_name,
                    type="number",
                    max=validator_props.get("ref_value", 100),
                    value=value,
                )

        # Fallback to type if no validator is handled
        if field_type == "bool":
            # The 'checked' attribute is a boolean flag, so it doesn't need a value
            return CheckboxX(label=setting_name, id=setting_name, checked=bool(value))

        elif field_type in ["str", "unknown"]:
            return Input(label=setting_name, id=setting_name, type="text", value=value)

        elif field_type in ["int", "float"]:
            return Input(
                label=setting_name, id=setting_name, type="number", value=value
            )

        elif field_type == "literal":
            return Select(
                Select(*generate_options(type_args, value), id=setting_name),
                label=setting_name,
            )

        # Placeholder for complex types
        elif field_type in ["dict", "list"]:
            return Div(
                Label(
                    f"Complex type '{field_type}' for '{setting_name}' requires custom handling."
                ),
                cl="text-sm text-gray-500",
            )

        return Input(label=f"Unhandled: {setting_name}", id=setting_name, disabled=True)

    def _create_settings_ui(self, settings: Dict):
        """Creates a Div for component settings from a dictionary."""
        main_container = Div()

        all_component_divs = []
        for component_name, component_settings in settings.items():
            component_div = Div("settings", cl="p-4 border rounded-lg my-4")
            settings_grid = Div(cls="grid grid-cols-2 gap-4")
            ui_elements = [
                self._create_ui_element(setting_name, setting_details)
                for setting_name, setting_details in component_settings.items()
            ]
            settings_grid(*ui_elements)

            component_div(
                H3(component_name, cls="text-xl font-bold mb-2"), settings_grid
            )
            all_component_divs.append(component_div)
        main_container(*all_component_divs)

        return main_container

    def _get_current_time(self):
        """Returns the current time as HH:MM."""
        return datetime.now().strftime("%H:%M")

    # -- WS handling --
    async def on_disconn(self, send):
        """When a client disconnects, unregister its callback."""
        pass

    async def on_conn(self, send):
        """When a client connects, register its callback."""
        pass

    # -- UI elements --
    def ChatMessage(self, text, msg_class, label, timestamp):
        """Component for a single text message."""
        return Div(
            Div(label, cl="message-label"),
            Div(text, cl=f"message {msg_class}"),
            Div(timestamp, cl=f"message-date-{msg_class}"),
            cl="message-wrapper",
            style=f"align-self: {'flex-end' if 'user' in msg_class else 'flex-start'};",
        )

    def AudioMessage(self, audio_src, msg_class, label, timestamp):
        """Component for a single audio message."""
        return Div(
            Div(Span(label), Span(timestamp), cl="audio-message-label"),
            Div(
                Audio(controls=True, src=audio_src, style="width: 100%;"),
                cl="audio-message",
            ),
            cl=f"audio-message-wrapper-{msg_class}",
        )

    def StatusMessage(self, message, msg_class):
        """Component for error or success messages."""
        return Div(message, cl=msg_class)

    def get_settings(self):
        return self.settings

    def get_main_page(self):
        # The audio recording script remains, as it requires browser APIs.
        return Titled(
            "EMOS WebView",
            Div(
                Div(id="notifications"),
                Form(Input(id="msg", placeholder="Something"), id="form", ws_send=True),
                hx_ext="ws",
                ws_connect="/ws",
            ),
            Div(
                Button(
                    "Settings",
                    id="settings-button",
                    hx_get="/settings/show",
                    hx_target="#modal-container",
                    hx_swap="innerHTML",
                ),
                id="modal-container",
            ),
        )
