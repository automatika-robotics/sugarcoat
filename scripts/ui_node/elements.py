from typing import List, Dict, Any

from ros_sugar.io.supported_types import String, Image, CompressedImage

try:
    from fasthtml.common import *
    from monsterui.all import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML & MonsterUI with `pip install python-fasthtml MonsterUI`"
    ) from e


def input_topic_card(topic_name: str, topic_type: type):
    if topic_type is String:
        return Input(
            name=topic_name,
            placeholder="String data...",
            type="text",
            required=True,
        )


def output_topic_card(topic_name: str, topic_type: type):
    if topic_type in [Image, CompressedImage]:
        return Img(id=topic_name, name="video-frame", src="", cls="h-96")


def _styled_logging_text(text: str, txt_type: str = "info"):
    new_txt = DivLAligned(cls="whitespace-pre-wrap ml-2 p-2")
    if txt_type == "alert":
        new_txt(Strong(f">>> {text}", cls=f"{TextT.lg} text-green-500"))
    elif txt_type == "error":
        new_txt(
            Strong(
                f">>> ERROR: {text}!",
                cls=f"{TextT.lg} text-red-500",
            )
        )
    elif txt_type == "warn":
        new_txt(
            Strong(
                f">>> WARNNING: {text}!",
                cls=f"{TextT.lg} text-orange-500",
            )
        )
    elif txt_type == "user":
        new_txt(
            Strong("> User: ", cls=f"{TextT.medium} text-blue-500"),
            P(f"{text}"),
        )
    elif txt_type == "robot":
        new_txt(
            Strong("> Robot: ", cls=f"{TextT.medium} font-bold text-purple-500"),
            P(f"{text}"),
        )
    elif txt_type == "info":
        new_txt(Strong(f"> {text}"))
    return new_txt


def create_logging_card():
    output_card = Card(
        cls="overflow-y-auto h-96",
        id="outputs-log",
    )
    return output_card(_styled_logging_text("Log Started ...", txt_type="alert"))


def update_logging_card(logging_card, text: str, txt_type: str = "info"):
    return logging_card(_styled_logging_text(text, txt_type))


def update_logging_card_with_loading(logging_card):
    """Update logging card"""
    return logging_card(
        DivLAligned(
            Strong("> Robot: ", cls=f"{TextT.medium} font-bold text-purple-500"),
            Loading(cls=(LoadingT.dots, LoadingT.md)),
            cls="whitespace-pre-wrap ml-2 p-2",
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
