import logging
from datetime import datetime

from ros_sugar.io.topic import Topic
from .utils import parse_type
from . import elements

try:
    from fasthtml.common import *
    from monsterui.all import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML & MonsterUI with `pip install python-fasthtml MonsterUI`"
    ) from e


class FHApp:
    def __init__(
        self,
        configs: Dict,
        in_topics: Sequence[Topic],
        out_topics: Sequence[Topic],
    ):
        # --- Application Setup ---
        # Get theme from MonsterUI
        hdrs = Theme.red.headers()
        self.app, self.rt = fast_app(hdrs=hdrs, exts="ws", live=True)
        setup_toasts(self.app)

        if not configs:
            logging.warning("No component configs provided to the UI")

        # Create settings UI
        self.configs = configs
        self.settings = self._create_component_settings_ui(configs)
        self.inputs = self._create_input_topics_ui(in_topics)
        self.outputs = self._create_output_topics_ui(out_topics)
        self.outputs_log = elements.create_logging_card()

    def get_app(self):
        """Get the FastHTML app"""
        return self.app, self.rt

    # -- Utility Functions --
    def toasting(self, msg, session, toast_type="info"):
        add_toast(session, f"{msg}", toast_type)

    def _create_ui_element(self, setting_name, setting_details):
        """Creates a UI element based on the setting's type and validators."""
        field_type, type_args = parse_type(setting_details.get("type", ""))
        validators = setting_details.get("validators", [])
        value = setting_details.get("value")

        # Handle validators first
        if validators:
            return elements.validated_config(
                setting_name=setting_name, value=value, attrs_validators=validators
            )

        return elements.nonvalidated_config(
            setting_name=setting_name,
            value=value,
            field_type=field_type,
            type_args=type_args,
        )

    # TODO: Create nested ui element
    # def _create_nested_attrs_ui(self, nested_value_name, nested_value):
    #     # main_nested_container = DivLAligned()
    #     nested_ui_elements = [
    #         DivLAligned(
    #             self._create_ui_element(nested_setting_name, nested_setting_details),
    #         )
    #         for nested_setting_name, nested_setting_details in nested_value.items()
    #     ]
    #     return nested_ui_elements

    def _create_input_topics_ui(self, inputs: Sequence[Topic]):
        """Creates cards for Input Topics"""

        input_divs = []
        inputs_container = Card(H3("Inputs"), cls="p-4")
        for inp in inputs:
            input_divs.append(
                Card(
                    H4(inp.name),
                    Form(cls="space-y-4")(
                        elements.input_topic_card(inp.name, inp.msg_type),
                        DivCentered(
                            Button(
                                "Submit",
                                cls=ButtonT.primary,
                                ws_send=True,
                                hx_target="#outputs-log",
                            ),
                        ),
                        id=f"{inp.name}-form",
                    ),
                    cls="p-4",
                    id=inp.name,
                )
            )
        return inputs_container(*input_divs)

    def _create_output_topics_ui(self, outputs: Sequence[Topic]):
        """Creates cards for Output Topics"""
        pass

    def _create_component_settings_ui(self, settings: Dict):
        """Creates a Div for component settings from a dictionary."""
        main_container = DivLAligned()

        all_component_forms = []
        for component_name, component_settings in settings.items():
            component_div = Card(cls="p-4")
            ui_elements = [
                DivLAligned(self._create_ui_element(setting_name, setting_details))
                for setting_name, setting_details in component_settings.items()
            ]
            settings_grid = Grid(*ui_elements, cols=4, cls="space-y-3 gap-4 p-4")

            component_div(
                H3(component_name),
                Form(cls="space-y-4")(
                    Input(name="component_name", type="hidden", value=component_name),
                    settings_grid,
                    DivCentered(
                        Grid(
                            Button(
                                "Submit",
                                cls=ButtonT.primary,
                                hx_post="/settings/submit",
                                hx_target="#main",
                            ),
                            Button(
                                "Close",
                                cls=ButtonT.secondary,
                                hx_get="/",
                                hx_target="#main",
                            ),
                            cols=2,
                            cls="gap-2",
                        )
                    ),
                    id=component_name,
                ),
                DivCentered(id="notification"),
                header=Div(
                    CardTitle(component_name),
                ),
            )
            all_component_forms.append(component_div)
        main_container(*all_component_forms)
        return main_container

    def _get_current_time(self):
        """Returns the current time as HH:MM."""
        return datetime.now().strftime("%H:%M")

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
        """Get components settings"""
        return self.settings

    def update_settings(self):
        """Update components settings"""
        pass

    def get_main_page(self):
        """Serves the main page of the UI"""
        self.settings = self._create_component_settings_ui(self.configs)
        return Titled(
            "EMOS WebView",
            Main(
                Div(
                    # ThemePicker(
                    #     color=False,
                    #     radii=False,
                    #     shadows=False,
                    #     font=False,
                    #     mode=True,
                    #     cls="p-4",
                    # ),
                    self.inputs,
                    self.outputs_log,
                    Button(
                        "Settings",
                        id="settings-button",
                        hx_get="/settings/show",
                        hx_target="#modal-container",
                        hx_swap="innerHTML",
                    ),
                    hx_ext="ws",
                    ws_connect="/ws",
                    id="modal-container",
                ),
                Div(id="result"),
            ),
            id="main",
        )
