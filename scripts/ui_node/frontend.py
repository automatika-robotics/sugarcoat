import logging
from datetime import datetime

from ros_sugar.io.supported_types import String, Audio as SugarAudio
from ros_sugar.io.topic import Topic
from .utils import parse_type
from . import elements
from ament_index_python.packages import get_package_prefix

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
        package_root = Path(
            get_package_prefix("automatika_ros_sugar")
        )  # where the package is installed
        BASE_DIR = Path.cwd()  # where the recipe is running
        static_files_abs = package_root / "lib" / "automatika_ros_sugar" / "ui_node"
        static_files = static_files_abs.relative_to(BASE_DIR).as_posix()
        hdrs = (
            Theme.red.headers(),
            Link(
                rel="stylesheet",
                href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css",
                type="text/css",
            ),
            Script(
                src="https://unpkg.com/idiomorph@0.7.3/dist/idiomorph-ext.min.js",
                integrity="sha384-szktAZju9fwY15dZ6D2FKFN4eZoltuXiHStNDJWK9+FARrxJtquql828JzikODob",
                crossorigin="anonymous",
            ),
            Script(
                src=static_files / Path("custom.js"),
            ),
        )
        self.app, self.rt = fast_app(hdrs=hdrs, exts=["ws", "morph"])
        setup_toasts(self.app)

        if not configs:
            logging.warning("No component configs provided to the UI")

        # Create settings UI
        self.configs = configs
        self.settings = self._create_component_settings_ui(configs)
        self.inputs = self._create_input_topics_ui(in_topics)
        self.outputs = self._create_output_topics_ui(out_topics)

        # persistent elements
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
        inputs_container = Card(H3("Inputs"), cls=CardT.secondary)
        for inp in inputs:
            input_divs.append(
                elements.input_topic_card(inp.name, inp.msg_type),
            )
        return inputs_container(*input_divs)

    def _create_output_topics_ui(self, outputs: Sequence[Topic]):
        """Creates cards for Output Topics"""

        output_divs = []
        outputs_container = Card(H3("Outputs"), cls=CardT.secondary)
        for out in outputs:
            if out.msg_type in [String, SugarAudio]:
                continue  # String is displayed in log
            output_divs.append(
                Card(H4(out.name), elements.output_topic_card(out.name, out.msg_type))
            )
        return outputs_container(*output_divs)

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
                    hx_post="/settings/submit",
                    hx_target="#main",
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
        # Create settings on refresh
        self.settings = self._create_component_settings_ui(self.configs)

        # Serve main page
        return Container(
            NavBar(
                Button(
                    "Components Settings",
                    id="settings-button",
                    hx_get="/settings/show",
                    hx_target="#modal-container",
                    hx_swap="innerHTML",
                    cls=ButtonT.primary,
                ),
                brand=DivLAligned(
                    Img(
                        src="https://automatikarobotics.com/Emos_dark.png",
                        style="width:6vw",
                    )
                ),
                cls="p-2",
            ),
            Main(
                Grid(
                    # ThemePicker(
                    #     color=False,
                    #     radii=False,
                    #     shadows=False,
                    #     font=False,
                    #     mode=True,
                    #     cls="p-4",
                    # ),
                    Div(
                        Card(H3("Log"), self.outputs_log, cls=f"{CardT.secondary}"),
                    ),
                    Div(self.outputs),
                    Div(self.inputs, cls="col-span-full"),
                    id="modal-container",
                    cols=2,
                ),
                Div(id="result"),
                id="main",
                cls="pt-2 pb-2",
                # connect to the websocket
                hx_ext="ws",
                ws_connect="/ws",
            ),
            id="main",
        )
