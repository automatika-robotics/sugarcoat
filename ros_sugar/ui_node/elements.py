import importlib
from typing import List, Dict, Any
import logging
from ..io.supported_types import SupportedType

from .utils import parse_type

try:
    from fasthtml.common import *
    from monsterui.all import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML & MonsterUI with `pip install python-fasthtml MonsterUI`"
    ) from e


def _in_text_element(topic_name: str, topic_type: str):
    """FastHTML element for input String type"""
    field_type = "number" if topic_type in ["Float32", "Float64"] else "text"
    return (
        Form(cls="mb-1 p-1")(
            Input(name="topic_name", type="hidden", value=topic_name),
            Input(name="topic_type", type="hidden", value=topic_type),
            Input(
                name="data",
                placeholder="String data...",
                type=field_type,
                required=True,
                autocomplete="off",
            ),
            id=f"{topic_name}-form",
            ws_send=True,
            hx_on__ws_after_send="this.reset(); return false;",
        ),
    )


def _in_bool_element(topic_name: str, topic_type: str):
    """FastHTML element for input String type"""
    return (
        Form(cls="mb-1 p-1")(
            Input(name="topic_name", type="hidden", value=topic_name),
            Input(name="topic_type", type="hidden", value=topic_type),
            DivLAligned(
                Switch(
                    id=topic_name,
                    checked=False,
                    name="data",
                ),
                Button("Send", type="submit", title="Send", cls="primary-button"),
                cls="space-x-4 ml-2",
            ),
            id=f"{topic_name}-form",
            ws_send=True,
            hx_on__ws_after_send="this.reset(); return false;",
        ),
    )


def _in_audio_element(topic_name: str, **_):
    """FastHTML element for input Audio type"""
    return DivCentered(
        DivHStacked(
            P("Send audio: "),
            Button(
                UkIcon(icon="mic"),
                id=topic_name,
                onclick="startAudioRecording(this)",
                title="Record",
                cls=f"{AT.primary}",
            ),
        ),
        cls="no-drag ",
    )


def _in_point_element(topic_name: str, topic_type: str):
    """FastHTML element for 3D point type"""
    return (
        Form(cls="space-x-2 space-y-2 mr-2 mb-2")(
            DivVStacked(
                DivFullySpaced(
                    Input(name="topic_name", type="hidden", value=topic_name),
                    Input(name="topic_type", type="hidden", value=topic_type),
                    Input(
                        placeholder="X",
                        name="x",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    Input(
                        placeholder="Y",
                        name="y",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    Input(
                        placeholder="Z",
                        name="z",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    cls="space-x-2",
                ),
                Button("Submit", cls="primary-button"),
            ),
            id=f"{topic_name}-form",
            ws_send=True,
            hx_on__ws_after_send="this.reset(); return false;",
        ),
    )


def _in_pose_element(topic_name: str, topic_type: str):
    """FastHTML element for 3D point type"""
    return (
        Form(cls="space-x-2 space-y-2 mr-2 mb-2")(
            DivVStacked(
                P("Position:"),
                DivFullySpaced(
                    Input(name="topic_name", type="hidden", value=topic_name),
                    Input(name="topic_type", type="hidden", value=topic_type),
                    Input(
                        placeholder="X",
                        name="x",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    Input(
                        placeholder="Y",
                        name="y",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    Input(
                        placeholder="Z",
                        name="z",
                        type="number",
                        required=True,
                        autocomplete="off",
                    ),
                    cls="space-x-2",
                ),
                DivHStacked(
                    P("Orientation (Optional):"),
                    _toggle_button(
                        onclick="""
                                for (let i = 6; i < this.form.length -1 ; i++)
                                {{this.form[i].hidden = !this.form[i].hidden;}}
                                """
                    ),
                    cls="space-x-0",
                ),
                DivFullySpaced(
                    Input(
                        placeholder="W",
                        name="ori_w",
                        type="number",
                        autocomplete="off",
                        hidden=True,
                    ),
                    Input(
                        placeholder="X",
                        name="ori_x",
                        type="number",
                        autocomplete="off",
                        hidden=True,
                    ),
                    Input(
                        placeholder="Y",
                        name="ori_y",
                        type="number",
                        autocomplete="off",
                        hidden=True,
                    ),
                    Input(
                        placeholder="Z",
                        name="ori_z",
                        type="number",
                        autocomplete="off",
                        hidden=True,
                    ),
                    cls="space-x-2",
                ),
                Button("Submit", cls="primary-button"),
            ),
            id=f"{topic_name}-form",
            ws_send=True,
            hx_on__ws_after_send="this.reset(); return false;",
        ),
    )


def _out_image_element(topic_name: str, **_):
    """FastHTML element for output Image/CompressedImage type"""
    return DivCentered(
        Img(id=topic_name, name="video-frame", src="", cls="h-[40vh] w-auto")
    )


def _log_audio_element(logging_card, output, data_src: str, id: str = "audio"):
    return logging_card(_styled_logging_audio(output, data_src, id))


def _log_text_element(logging_card, output, data_src: str, id: str = "text"):
    return logging_card(_styled_logging_text(output, data_src, id))


_INPUT_ELEMENTS: Dict = {
    "String": _in_text_element,
    "Float32": _in_text_element,
    "Float64": _in_text_element,
    "Bool": _in_bool_element,
    "Audio": _in_audio_element,
    "Point": _in_point_element,
    "PointStamped": _in_point_element,
    "Pose": _in_pose_element,
    "PoseStamped": _in_pose_element,
}

_OUTPUT_ELEMENTS: Dict = {
    "String": _log_text_element,
    "Float32": _log_text_element,
    "Float64": _log_text_element,
    "Bool": _log_text_element,
    "Audio": _log_audio_element,
    "Image": _out_image_element,
    "CompressedImage": _out_image_element,
    "OccupancyGrid": _out_image_element,
}


def _deserialize_additional_element(k_t: str, i_t: str) -> Optional[Tuple]:
    """Deserialize one additional element"""
    # Get key type
    module_name_key, _, type_name = k_t.rpartition(".")
    if not module_name_key:
        logging.error(f"Could not find module name for {k_t}")
        return
    # Get item func
    module_name_item, _, func_name = i_t.rpartition(".")
    if not module_name_item:
        logging.error(f"Could not find module name for {i_t}")
        return
    module_key = importlib.import_module(module_name_key)
    module_item = importlib.import_module(module_name_item)
    key = getattr(module_key, type_name)
    if not issubclass(key, SupportedType):
        logging.error(f"Could not find {type_name} name in {module_key} module")
        return
    item = getattr(module_item, func_name)
    if not callable(item):
        logging.error(f"Could not find {func_name} name in {module_item} module")
        return
    return key, item


def add_additional_ui_elements(
    input_elements: Optional[List[Tuple]], output_elements: Optional[List[Tuple]]
):
    """Deserialize additional elements and add them"""
    global _INPUT_ELEMENTS, _OUTPUT_ELEMENTS

    # Add input elements
    if input_elements:
        for k_t, i_t in input_elements:
            deserialized = _deserialize_additional_element(k_t, i_t)
            if deserialized:
                _INPUT_ELEMENTS[deserialized[0].__name__] = deserialized[1]

    # Add output elements
    if output_elements:
        for k_t, i_t in output_elements:
            deserialized = _deserialize_additional_element(k_t, i_t)
            if deserialized:
                _OUTPUT_ELEMENTS[deserialized[0].__name__] = deserialized[1]


def _toggle_button(div_to_toggle: Optional[str] = None, **kwargs):
    _arrow_down = UkIcon("chevrons-down")
    _arrow_up = UkIcon("chevrons-up")
    onclick = f"""
                if (this.name == 'down')
                {{
                    this.innerHTML=`{_arrow_up}`;
                    this.name = 'up';
                }}
                else{{
                    this.innerHTML=`{_arrow_down}`;
                    this.name = 'down'}};
                """
    if div_to_toggle:
        toggle_click = f"""
                    let toggleDiv = document.getElementById('{div_to_toggle}');
                    toggleDiv.hidden = ! toggleDiv.hidden;
                    if (toggleDiv.hidden){{
                        toggleDiv.style.display = "none";
                    }} else {{
                        toggleDiv.style.display = "";
                    }}
                """
        onclick = f"{onclick}\n{toggle_click}"
    if kwargs.get("onclick", None):
        onclick = f"{onclick}\n{kwargs.get('onclick')}"
        kwargs.pop("onclick")
    return Button(
        _arrow_down,
        type="button",
        name="down",
        cls=f"no-drag {AT.primary}",
        onclick=onclick,
        **kwargs,
    )


def input_topic_card(topic_name: str, topic_type: str, column_class: str = "") -> FT:
    """Creates a UI element for an input topic

    :param topic_name: Topic name
    :type topic_name: str
    :param topic_type: Topic message type
    :type topic_type: str
    :return: Input topic UI element
    """
    card = Card(
        H4(topic_name),
        cls=f"m-2 {column_class} max-h-[20vh] overflow-y-auto inner-main-card",
        id=topic_name,
    )
    return card(_INPUT_ELEMENTS[topic_type](topic_name, topic_type=topic_type))


def styled_main_inputs_container(inputs_grid_div_id: str) -> FT:
    """Creates main section for all the UI inputs

    :return: Main inputs card
    :rtype: FT Card
    """
    return Card(
        DivHStacked(
            H4("Inputs", cls="cool-subtitle-mini"),
            _toggle_button(div_to_toggle=inputs_grid_div_id),
            cls="space-x-0",
        ),
        cls="draggable main-card max-h-[25vh] overflow-y-auto",
        body_cls="space-y-0",
    )


def styled_inputs_grid(number_of_inputs: int) -> tuple:
    """Creates a styled grid for the number of inputs

    :param number_of_inputs: Number of input cards
    :type number_of_inputs: int
    :return: Styled Grid, Style class to use for each element in the grid
    :rtype: tuple
    """
    # Create a grid with max 3 inputs per line (1 per line for small views)
    input_grid = Grid(cls="gap-0", cols_lg=min(3, number_of_inputs), cols_sm=1)
    inputs_columns_span = ["col-1"] * number_of_inputs
    # Adjust the column span of the remaining inputs
    if remaining_items := number_of_inputs % 3:
        if remaining_items == 1:
            inputs_columns_span[-1] = "col-span-full"
        elif remaining_items == 2:
            inputs_columns_span[-2:] = "flex flex-col items-center justify-center"
    return (input_grid, inputs_columns_span)


def output_topic_card(topic_name: str, topic_type: str, column_class: str = "") -> FT:
    """Creates a UI element for an output topic

    :param topic_name: Topic name
    :type topic_name: str
    :param topic_type: Topic message type
    :type topic_type: str
    :return: Output topic UI element
    """
    card = Card(
        H4(topic_name),
        cls=f"m-2 {column_class} h-[48vh] inner-main-card",
        id=topic_name,
    )
    return card(_OUTPUT_ELEMENTS[topic_type](topic_name))


def styled_main_outputs_container(outputs_grid_div_id: str) -> FT:
    """Creates main section for all the UI outputs

    :return: Main outputs card
    :rtype: FT Card
    """
    return Card(
        DivHStacked(
            H4("Outputs", cls="cool-subtitle-mini"),
            _toggle_button(div_to_toggle=outputs_grid_div_id),
            cls="space-x-0",
        ),
        cls="draggable main-card overflow-y-auto max-h-[60vh]",
        body_cls="space-y-0",
    )


def styled_outputs_grid(number_of_outputs: int) -> tuple:
    """Creates a styled grid for the number of outputs

    :param number_of_outputs: Number of output cards
    :type number_of_outputs: int
    :return: Styled Grid, Style class to use for each element in the grid
    :rtype: tuple
    """
    # Create a grid with max 2 outputs per line (1 per line for small views)
    output_grid = Grid(cls="gap-0", cols_lg=min(2, number_of_outputs), cols_sm=1)
    outputs_columns_span = ["col-1"] * number_of_outputs
    if number_of_outputs % 2:
        outputs_columns_span[-1] = "col-span-full"
    return (output_grid, outputs_columns_span)


def settings_ui_element(
    setting_name: str, setting_details: dict, field_type, type_args, input_name=None
):
    """Creates a UI element based on the setting's type and validators

    :param setting_name: Config parameter name
    :type setting_name: str
    :param setting_details: Details of the parsed config
    :type setting_details: dict

    :return: Setting parameter UI element
    """
    try:
        field_type, type_args = parse_type(setting_details.get("type", ""))
    except Exception as e:
        logging.error(
            f"Could not render setting: {setting_name}, with value: {setting_details} due to error: {e}"
        )
        field_type, type_args = None, None
    validators = setting_details.get("validators", [])
    value = setting_details.get("value")

    if not input_name:
        input_name = setting_name

    # Handle validators first
    if validators:
        return validated_config(
            setting_name=setting_name,
            value=value,
            attrs_validators=validators,
            field_type=field_type,
            type_args=type_args,
            input_name=input_name,
        )
    if field_type:
        return nonvalidated_config(
            setting_name=setting_name,
            value=value,
            field_type=field_type,
            type_args=type_args,
            input_name=input_name,
        )


def component_settings_div(
    component_name: str, settings_col_cls: str, ui_elements, nested_ui_elements
):
    """Creates a UI element for a component to show and update the config parameters

    :param component_name: Name of the component (ROS2 node name)
    :type component_name: str
    :param settings_col_cls: UI Div columns span in the display grid
    :type settings_col_cls: str
    :param ui_elements: A set ot UI elements for each parameter in the component config
    :type ui_elements: List
    :return: Component config UI element
    """
    component_div = Card(
        cls=f"p-4 {settings_col_cls} main-card",
    )
    settings_grid = Grid(*ui_elements, cols=4, cls="space-y-3 gap-4 p-4")
    nested_settings_grid = Grid(*nested_ui_elements, cols=1, cls="space-y-3 gap-4 p-4")
    _loading_content = DivHStacked(
        Loading(cls=(LoadingT.spinner, LoadingT.md)), P(" Sending")
    )
    return component_div(
        H3(component_name),
        Form(cls="space-y-4")(
            Input(name="component_name", type="hidden", value=component_name),
            settings_grid,
            nested_settings_grid,
            DivCentered(
                Grid(
                    Button(
                        "Submit",
                        cls="primary-button",
                        hx_post="/settings/submit",
                        hx_target="#main",
                        hx_on__before_request=f"""
                        this.innerHTML = `{_loading_content}`;
                        this.disabled = true;
                        """,
                    ),
                    Button(
                        "Close",
                        cls="secondary-button",
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


LOG_STYLES = {
    "alert": {"prefix": ">>>", "cls": f"{TextT.lg} tomorrow-night-green"},
    "error": {"prefix": ">>> ERROR: ", "cls": f"{TextT.lg} tomorrow-night-red"},
    "warn": {"prefix": ">>> WARNNING: ", "cls": f"{TextT.lg} tomorrow-night-yellow"},
    "user": {
        "prefix": "> User:",
        "cls": f"{TextT.medium} font-bold tomorrow-night-blue",
    },
    "robot": {
        "prefix": "> Robot:",
        "cls": f"{TextT.medium} font-bold tomorrow-night-green",
    },
}
# Default style for "info" or any other unspecified source
DEFAULT_STYLE = {"prefix": ">", "cls": ""}


def _styled_logging_text(text: str, output_src: str = "info", div_id: str = "text"):
    """Builds a styled text log component."""
    container = Div(cls="whitespace-pre-wrap ml-2 p-2 flex items-start", id=div_id)
    style = LOG_STYLES.get(output_src, DEFAULT_STYLE)

    if output_src in ["user", "robot"]:
        prefix_element = Strong(style["prefix"] + " ", cls=style["cls"])
        content_element = Span(prefix_element, f"{text}", id="inner-text")
        container(content_element)
    # All other types have the text inside the main Strong tag
    else:
        full_text = f"{style['prefix']} {text}"
        if output_src in ["error", "warn"]:
            full_text += "!"  # Add the original exclamation mark

        container(Strong(full_text, cls=style["cls"]))

    return container


def _styled_logging_audio(output, output_src: str = "info", div_id: str = "audio"):
    """Builds a styled audio log component."""
    container = DivLAligned(cls="whitespace-pre-wrap ml-2 p-2", id=div_id)
    style = LOG_STYLES.get(output_src, DEFAULT_STYLE)

    audio_element = Audio(
        src=f"data:audio/wav;base64,{output}",
        type="audio/wav",
        controls=True,
        style="border-radius:0.5rem;outline:none;",
    )

    prefix_element = Strong(style["prefix"] + " ", cls=style["cls"])
    container(prefix_element, audio_element)

    return container


def output_logging_card(current_log):
    return Card(
        DivHStacked(
            H4("Log", cls="cool-subtitle-mini"),
            current_log,
            cls="space-x-0",
        ),
        cls="fix-size draggable main-card h-[60vh] max-h-[60vh]",
        id="logging-card-parent",
    )


def initial_logging_card():
    output_card = Card(
        cls="terminal-container absolute top-20 inset-x-2 bottom-2 overflow-y-auto",
        id="outputs-log",
    )
    return output_card(_styled_logging_text("Log Started ...", output_src="alert"))


def remove_child_from_logging_card(logging_card, target_id="loading-dots"):
    """Remove the last child in logging_card.children with a matching id."""
    children = logging_card.children

    for i in range(len(children) - 1, -1, -1):
        if getattr(children[i], "id", None) == target_id:
            logging_card.children = children[:i] + children[i + 1 :]
            break


def augment_text_in_logging_card(
    logging_card,
    new_txt: str,
    target_id="text",
):
    """Update the inner text of a child in logging_card.children with a matching id."""
    children = logging_card.children
    target_child = None
    for i in range(len(children) - 1, -1, -1):
        if getattr(children[i], "id", None) == target_id:
            target_child = children[i]
            break
    if not target_child:
        return logging_card
    # Update inner text
    for i in range(len(target_child.children) - 1, -1, -1):
        if getattr(target_child.children[i], "id", None) == "inner-text":
            # Append the new text
            target_child.children[i](Span(f"{new_txt}"))
            return logging_card
    return logging_card


def update_logging_card(
    logging_card, output: str, data_type: str, data_src: str = "info"
):
    remove_child_from_logging_card(logging_card)
    # Handle errors originating from ROS node
    if data_type == "error":
        data_type = "String"
        data_src = "error"
    return _OUTPUT_ELEMENTS[data_type](logging_card, output, data_src)


def update_logging_card_with_loading(logging_card):
    """Update logging card"""
    return logging_card(
        Div(
            Strong(
                "> Robot:", cls=f"{TextT.medium} font-bold tomorrow-night-green mr-2"
            ),
            Loading(cls=(LoadingT.dots, LoadingT.md)),
            cls="whitespace-pre-wrap ml-2 p-2 flex items-start",
            id="loading-dots",
            name="loading-dots",
        )
    )


def nonvalidated_config(
    setting_name: str, value: Any, field_type: str, type_args, input_name: str
):
    if field_type == "bool":
        # The 'checked' attribute is a boolean flag, so it doesn't need a value
        return LabelSwitch(
            label=setting_name,
            id=setting_name,
            checked=int(value),
            name=input_name,
            cls="form-input",
        )

    elif field_type in ["str", "unknown"]:
        return LabelInput(
            label=setting_name,
            id=setting_name,
            type="text",
            value=value,
            name=input_name,
            cls="form-input",
        )

    elif field_type in ["int", "float"]:
        return LabelInput(
            label=setting_name,
            id=setting_name,
            type="number",
            value=str(value),
            name=input_name,
            cls="form-input",
        )

    elif field_type == "literal":
        # Parsing to hanlde enum literals
        parsed_value = value
        if value not in type_args and type(value) is str and value.upper() in type_args:
            parsed_value = value.upper()
        return LabelSelect(
            map(Option, type_args),
            id=setting_name,
            label=setting_name,
            value=parsed_value,
            name=input_name,
            cls="form-input",
        )

    return None


def validated_config(
    setting_name: str,
    value: Any,
    attrs_validators: List[Dict],
    field_type,
    type_args,
    input_name,
):
    validator = attrs_validators[0]
    validator_name = list(validator.keys())[0]
    validator_props = validator[validator_name]
    if validator_name in ["in_range", "in_range_discretized"]:
        return LabelInput(
            label=setting_name,
            id=setting_name,
            min=validator_props.get("min_value", 1e-9),
            max=validator_props.get("max_value", 1e9),
            step=validator_props.get("step", 1),
            value=str(value),
            name=input_name,
            cls="form-input",
        )

    elif validator_name == "in":
        options = validator_props.get("ref_value", [])
        return LabelSelect(
            map(Option, options),
            label=setting_name,
            id=setting_name,
            value=value,
            name=input_name,
            cls="form-input",
        )

    elif validator_name == "less_than":
        return LabelInput(
            setting_name,
            id=setting_name,
            type="number",
            max=validator_props.get("ref_value", None),
            value=str(value),
            name=input_name,
            cls="form-input",
        )
    elif validator_name == "greater_than":
        return LabelInput(
            setting_name,
            id=setting_name,
            type="number",
            min=validator_props.get("ref_value", None),
            value=str(value),
            name=input_name,
            cls="form-input",
        )
    return nonvalidated_config(
        setting_name,
        value,
        field_type,
        type_args,
        input_name,
    )


def _parse_nested_settings_dict(
    nested_dict: Dict, all_elements_list: List, nested_root_name: str
):
    """Parse a UI elemnt for nested settings (config) classses

    :param nested_dict: Nested config dictionary
    :type nested_dict: Dict
    :param all_elements_list: List to be populated with nested ui elements
    :type all_elements_list: List
    :param nested_root_name: Root name of the nested config attribute
    :type nested_root_name: str
    """
    for nested_setting_name, nested_setting_details in nested_dict.items():
        field_type, type_args = parse_type(nested_setting_details.get("type", ""))
        if field_type == "BaseAttrs":
            all_elements_list.append(H5(nested_setting_name, cls="col-span-4"))
            value = nested_setting_details.get("value", None)
            if value:
                _parse_nested_settings_dict(
                    nested_setting_details.get("value", {}),
                    all_elements_list,
                    f"{nested_root_name}.{nested_setting_name}",
                )
        else:
            single_div = DivLAligned(
                settings_ui_element(
                    nested_setting_name,
                    nested_setting_details,
                    field_type,
                    type_args,
                    input_name=f"{nested_root_name}.{nested_setting_name}",
                ),
                id=f"{nested_setting_name}",
            )
            all_elements_list.append(single_div)
    return


def parse_ui_elements_to_simple_and_nested(
    component_name,
    setting_name,
    setting_details,
    simple_ui_elements: list,
    nested_ui_elements: list,
):
    field_type, type_args = parse_type(setting_details.get("type", ""))
    nested_root_name = f"{setting_name}"

    if field_type == "BaseAttrs":
        section_id = f"{component_name}-{setting_name}-settings-grid"
        main_nested_container = Card(
            DivLAligned(
                H4(setting_name),
                _toggle_button(div_to_toggle=section_id),
            ),
            cls="p-4 main-card",
        )
        all_elements = Grid(
            id=section_id,
            cols=4,
            cls="space-y-3 gap-4 p-4",
            hidden=True,
            style="display: none;",
        )
        all_elements_list = []
        _parse_nested_settings_dict(
            setting_details.get("value", {}), all_elements_list, nested_root_name
        )
        nested_ui_elements.append(
            main_nested_container(all_elements(*all_elements_list))
        )

    else:
        # simple type: create and add the simple UI element
        simple_ui_elements.append(
            DivLAligned(
                settings_ui_element(
                    setting_name,
                    setting_details,
                    field_type,
                    type_args,
                    nested_root_name,
                ),
            )
        )
