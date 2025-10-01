from typing import List, Dict, Any
try:
    from fasthtml.common import *
    from monsterui.all import *

except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to use the dynamic web UI for your recipe, please install FastHTML & MonsterUI with `pip install python-fasthtml MonsterUI`"
    ) from e


def nonvalidated_input(setting_name: str, value: Any, field_type: str, type_args):
    if field_type == "bool":
        # The 'checked' attribute is a boolean flag, so it doesn't need a value
        return LabelSwitch(label=setting_name, id=setting_name, checked=bool(value))

    elif field_type in ["str", "unknown"]:
        return LabelInput(
            label=setting_name, id=setting_name, type="text", value=value
        )

    elif field_type in ["int", "float"]:
        return LabelInput(
            label=setting_name, id=setting_name, type="number", value=str(value)
        )

    elif field_type == "literal":
        return LabelSelect(
            map(Option, type_args),
            id=setting_name,
            label=setting_name,
            value=value
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


def validated_input(setting_name: str, value: Any, attrs_validators: List[Dict]):
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
