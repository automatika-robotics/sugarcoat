from typing import List, Dict, Any

from ros_sugar.io.supported_types import (
    String,
    Image,
    CompressedImage,
    Audio as SugarAudio,
)

try:
    from fasthtml.common import *
    from monsterui.all import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML & MonsterUI with `pip install python-fasthtml MonsterUI`"
    ) from e


def input_topic_card(topic_name: str, topic_type: type):
    card = Card(
        H4(topic_name),
        cls="m-2",
        id=topic_name,
    )
    if topic_type is String:
        return card(
            Form(cls="space-y-4")(
                Input(
                    name=topic_name,
                    placeholder="String data...",
                    type="text",
                    required=True,
                    autocomplete="off",
                    hx_target="#outputs-log",
                ),
                id=f"{topic_name}-form",
                ws_send=True,
                hx_on__ws_after_send=f"this.{topic_name}.value=''; return false;",
            ),
        )
    if topic_type is SugarAudio:
        return card(
            Button(
                UkIcon("mic"),
                id=topic_name,
                onclick="startAudioRecording(this)",
            )
        )


def output_topic_card(topic_name: str, topic_type: type):
    if topic_type in [Image, CompressedImage]:
        return Img(id=topic_name, name="video-frame", src="", cls="h-96")


LOG_STYLES = {
    "alert": {"prefix": ">>>", "cls": f"{TextT.lg} text-green-500"},
    "error": {"prefix": ">>> ERROR:", "cls": f"{TextT.lg} text-red-500"},
    "warn": {"prefix": ">>> WARNNING:", "cls": f"{TextT.lg} text-orange-500"},
    "user": {"prefix": "> User:", "cls": f"{TextT.medium} text-blue-500"},
    "robot": {"prefix": "> Robot:", "cls": f"{TextT.medium} font-bold text-purple-500"},
}
# Default style for "info" or any other unspecified source
DEFAULT_STYLE = {"prefix": ">", "cls": ""}


def _styled_logging_text(text: str, output_src: str = "info"):
    """Builds a styled text log component."""
    container = DivLAligned(cls="whitespace-pre-wrap ml-2 p-2")
    style = LOG_STYLES.get(output_src, DEFAULT_STYLE)

    if output_src in ["user", "robot"]:
        prefix_element = Strong(style["prefix"] + " ", cls=style["cls"])
        content_element = P(f"{text}")
        container(prefix_element, content_element)
    # All other types have the text inside the main Strong tag
    else:
        full_text = f"{style['prefix']} {text}"
        if output_src in ["error", "warn"]:
            full_text += "!"  # Add the original exclamation mark

        container(Strong(full_text, cls=style["cls"]))

    return container


def _styled_logging_audio(output, output_src: str = "info"):
    """Builds a styled audio log component."""
    container = DivLAligned(cls="whitespace-pre-wrap ml-2 p-2")
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


def create_logging_card():
    output_card = Card(
        cls="overflow-y-auto h-96",
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


def update_logging_card(
    logging_card, output: str, output_src: str = "info", is_audio: bool = False
):
    remove_child_from_logging_card(logging_card)
    if is_audio:
        return logging_card(_styled_logging_audio(output, output_src))
    return logging_card(_styled_logging_text(output, output_src))


def update_logging_card_with_loading(logging_card):
    """Update logging card"""
    return logging_card(
        DivLAligned(
            Strong("> Robot: ", cls=f"{TextT.medium} font-bold text-purple-500"),
            Loading(cls=(LoadingT.dots, LoadingT.md)),
            cls="whitespace-pre-wrap ml-2 p-2",
            id="loading-dots",
            name="loading-dots",
        )
    )


def nonvalidated_config(setting_name: str, value: Any, field_type: str, type_args):
    if field_type == "bool":
        # The 'checked' attribute is a boolean flag, so it doesn't need a value
        return LabelSwitch(label=setting_name, id=setting_name, checked=bool(value))

    elif field_type in ["str", "unknown"]:
        return LabelInput(label=setting_name, id=setting_name, type="text", value=value)

    elif field_type in ["int", "float"]:
        return LabelInput(
            label=setting_name, id=setting_name, type="number", value=str(value)
        )

    elif field_type == "literal":
        return LabelSelect(
            map(Option, type_args), id=setting_name, label=setting_name, value=value
        )

    # TODO: handle BaseAttrs
    # elif field_type == "BaseAttrs":
    #     print(f"Returning None for {setting_name} with: {value}")
    #     # Handle nested attrs
    #     return None

    # Placeholder for complex types
    # TODO: handle dict and list
    # elif field_type in ["dict", "list"]:

    return LabelInput(
        label=f"Unhandled: {setting_name}", id=setting_name, disabled=True
    )


def validated_config(setting_name: str, value: Any, attrs_validators: List[Dict]):
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
        )

    elif validator_name == "in":
        options = validator_props.get("ref_value", [])
        return LabelSelect(
            map(Option, options), label=setting_name, id=setting_name, value=value
        )

    elif validator_name in ["greater_than", "less_than"]:
        return LabelInput(
            setting_name,
            id=setting_name,
            type="number",
            max=validator_props.get("ref_value", None),
            min=validator_props.get("ref_value", None),
            value=str(value),
        )
