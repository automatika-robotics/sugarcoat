import logging
from datetime import datetime

from ros_sugar.io.topic import Topic
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
        additional_input_elements: Optional[List[Tuple]] = None,
        additional_output_elements: Optional[List[Tuple]] = None,
    ):
        # --- Application Setup ---
        static_src = Path(__file__).resolve().parent / "static"

        hdrs = (
            Theme.red.headers(),  # Get theme from MonsterUI
            Script(
                src="custom.js",
            ),
            Link(rel="stylesheet", href="custom.css", type="text/css"),
        )
        self.app, self.rt = fast_app(
            hdrs=hdrs, exts=["ws"], static_path=str(static_src)
        )

        if not configs:
            logging.warning("No component configs provided to the UI")

        # Add any additional elements
        elements.add_additional_ui_elements(
            input_elements=additional_input_elements,
            output_elements=additional_output_elements,
        )

        # Create settings UI
        self.configs = configs
        self.settings = self._create_component_settings_ui(configs)
        self.in_topics = in_topics
        self.out_topics = out_topics
        self.inputs = self._create_input_topics_ui(in_topics) if in_topics else None
        self.outputs = self._create_output_topics_ui(out_topics) if out_topics else None
        self.toggle_settings = False
        setup_toasts(self.app)

        # persistent elements
        self.outputs_log = elements.initial_logging_card()

    @property
    def _settings_button(self) -> str:
        return "Exit Settings" if self.toggle_settings else "Components Settings"

    def get_app(self):
        """Get the FastHTML app"""
        return self.app, self.rt

    def toasting(self, msg, session, toast_type="info", duration: int = 5000):
        session["duration"] = duration
        dismiss = (
            duration >= 20000
        )  # Id duration is more than 20seconds -> activate dismiss by default
        add_toast(session, f"{msg}", toast_type, dismiss)

    def get_all_stream_outputs(self) -> List[Tuple]:
        """Return all topics that connect to their own websocket for streaming"""
        return [
            (o.name, o.msg_type.__name__)
            for o in self.out_topics
            if (
                elements._OUTPUT_ELEMENTS.get(o.msg_type.__name__, None)
                == elements._out_image_element
            )
        ]

    def update_configs_from_data(self, data: Dict):
        """Update configs from a UI form data dict

        :param data: Component settings form data
        :type data: Dict
        """
        component_to_update = data["component_name"]
        for param in self.configs[component_to_update].keys():
            if param_value := data.get(param):
                self.configs[component_to_update][param]["value"] = param_value

    def _create_input_topics_ui(self, inputs: Sequence[Topic]):
        """Creates cards for Input Topics"""

        input_divs = []
        grid_id = "inputs-grid"
        inputs_container = elements.styled_main_inputs_container(grid_id)

        # Create a styled grid
        input_grid, inputs_columns_cls = elements.styled_inputs_grid(
            number_of_inputs=len(inputs)
        )

        for idx, inp in enumerate(inputs):
            input_divs.append(
                elements.input_topic_card(
                    inp.name, inp.msg_type.__name__, inputs_columns_cls[idx]
                ),
            )
        return inputs_container(input_grid(*input_divs, id=grid_id))

    def _create_output_topics_ui(self, outputs: Sequence[Topic]):
        """Creates cards for Output Topics"""
        displayed_outputs = [
            out
            for out in outputs
            if elements._OUTPUT_ELEMENTS[out.msg_type.__name__].__name__.startswith(
                "_out"
            )
        ]  # Get output elements that have specific display cards
        if not displayed_outputs:
            return None
        output_divs = []
        grid_id = "outputs-grid"
        outputs_container = elements.styled_main_outputs_container(grid_id)
        # Create a grid with max 2 outputs per line (1 per line for small views)
        output_grid, outputs_columns_cls = elements.styled_outputs_grid(
            number_of_outputs=len(displayed_outputs)
        )

        for idx, out in enumerate(displayed_outputs):
            output_divs.append(
                elements.output_topic_card(
                    out.name, out.msg_type.__name__, outputs_columns_cls[idx]
                )
            )
        return outputs_container(output_grid(*output_divs, id=grid_id))

    def _create_component_settings_ui(self, settings: Dict):
        """Creates a Div for component settings from a dictionary."""
        # Parse number of grid columns and column span based on the number of components
        # if len(settings) > 2:
        #     grid_num_cols = 2
        #     item_col_cls = "col-1"
        #     # Make the last component span over the whole width if we have an odd number of components
        #     last_col_cls = "col-span-full" if len(settings) % 2 == 1 else "col-1"
        # else:
        grid_num_cols = 1
        item_col_cls = "col-span-full"
        last_col_cls = "col-span-full"

        main_container = Grid(cols=grid_num_cols)

        all_component_forms = []
        for count, (component_name, component_settings) in enumerate(settings.items()):
            simple_ui_elements = []
            nested_ui_elements = []
            for setting_name, setting_details in component_settings.items():
                elements.parse_ui_elements_to_simple_and_nested(
                    component_name,
                    setting_name,
                    setting_details,
                    simple_ui_elements,
                    nested_ui_elements,
                )

            # Create an element for each component and add to the settings display
            all_component_forms.append(
                elements.component_settings_div(
                    component_name,
                    settings_col_cls=item_col_cls
                    if count < len(settings) - 1
                    else last_col_cls,
                    ui_elements=simple_ui_elements,
                    nested_ui_elements=nested_ui_elements,
                )
            )
        main_container(*all_component_forms)
        return main_container

    def _get_current_time(self):
        """Returns the current time as HH:MM."""
        return datetime.now().strftime("%H:%M")

    def get_settings(self):
        """Get components settings"""
        return self.settings

    @property
    def _main(self):
        if not self.toggle_settings:
            main_grid = Grid(
                id="modal-container",
                cols=2,
            )
            if self.outputs:
                main_grid(
                    Div(elements.output_logging_card(self.outputs_log)),
                    Div(self.outputs),
                )
            else:
                main_grid(
                    Div(
                        elements.output_logging_card(self.outputs_log),
                        cls="col-span-full",
                    ),
                )
            if self.inputs:
                main_grid(
                    Div(self.inputs, cls="col-span-full"),
                )

            return Main(
                main_grid,
                Div(id="result"),
                id="main",
                cls="pt-2 pb-2",
                # connect to the default websocket
                hx_ext="ws",
                ws_connect="/ws",
                # NOTE: Function defined in custon js to reconnect streaming websockets
                # HTMX fires "htmx:afterSwap" after content is swapped into the DOM,
                # and "htmx:afterOnLoad" / "htmx:afterRequest" for other lifecycle stages
                # Respond to afterSwap and afterOnLoad to be safe.
                hx_on__after_on_load="ensureConnectionsForPresentFrames",
                hx_on__after_swap="ensureConnectionsForPresentFrames",
            )
        return self.settings

    def get_main_page(self):
        """Serves the main page of the UI"""
        # Serve main page
        self.settings = self._create_component_settings_ui(self.configs)
        return (
            Favicon(light_icon="automatika-icon.png", dark_icon="automatika-icon.png"),
            Title("EMOS UI"),
            Div(
                Container(
                    NavBar(
                        Button(
                            self._settings_button,
                            id="settings-button",
                            hx_get="/settings/show",
                            hx_target="#main",
                            hx_swap="outerHTML",
                            cls="primary-button",
                        ),
                        brand=DivLAligned(
                            Img(
                                src="https://automatikarobotics.com/Emos_dark.png",
                                style="width:6vw",
                            )
                        ),
                    ),
                    self._main,
                    id="main",
                    cls="mt-0 mb-0",
                ),
                cls="grid-section",
            ),
        )
