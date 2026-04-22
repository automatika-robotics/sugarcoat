"""Tests for ros_sugar.config.base_attrs.BaseAttrs.

Covers the serialization / deserialization path that config loading relies on:
from_dict (and its type gate), from_json round-trips, __setattr__ validator
re-run on mutation, and from_file for YAML (the primary config format).
"""
from typing import List, Literal, Optional, Union, Any

import json
import numpy as np
import pytest
from attrs import define, field

from ros_sugar.config.base_attrs import BaseAttrs
from ros_sugar.config import base_validators


# ---- Helper attrs classes used across the tests ---------------------------


@define(kw_only=True)
class NestedCfg(BaseAttrs):
    count: int = field(default=1)
    label: str = field(default="x")


@define(kw_only=True)
class ListItemCfg(BaseAttrs):
    name: str = field(default="item")
    value: int = field(default=0)


@define(kw_only=True)
class SampleCfg(BaseAttrs):
    rate: float = field(
        default=10.0, validator=base_validators.in_range(min_value=0.0, max_value=1e3)
    )
    name: str = field(default="default")
    flag: bool = field(default=False)
    optional_int: Optional[int] = field(default=None)
    mode: Literal["a", "b", "c"] = field(default="a")
    either: Union[int, str] = field(default=0)
    array: np.ndarray = field(default=np.array([0.0, 0.0]))
    nested: NestedCfg = field(factory=NestedCfg)
    items: List[ListItemCfg] = field(factory=lambda: [ListItemCfg()])


@define(kw_only=True)
class AnyFieldCfg(BaseAttrs):
    """Isolated fixture for verifying Any-field rejection."""
    free: Any = field(default=None)


# ---- from_dict: the main deserialization path ----------------------------


def test_from_dict_simple_types_roundtrip():
    cfg = SampleCfg()
    cfg.from_dict({"rate": 25.0, "name": "hello", "flag": True})
    assert cfg.rate == 25.0
    assert cfg.name == "hello"
    assert cfg.flag is True


def test_from_dict_union_accepts_either_arm():
    cfg = SampleCfg()
    cfg.from_dict({"either": 7})
    assert cfg.either == 7
    cfg.from_dict({"either": "seven"})
    assert cfg.either == "seven"


def test_from_dict_union_rejects_third_type():
    cfg = SampleCfg()
    with pytest.raises(TypeError):
        cfg.from_dict({"either": [1, 2, 3]})


def test_from_dict_optional_accepts_none_and_value():
    cfg = SampleCfg()
    cfg.from_dict({"optional_int": 5})
    assert cfg.optional_int == 5
    cfg.from_dict({"optional_int": None})
    assert cfg.optional_int is None


def test_from_dict_literal_accepts_allowed_rejects_disallowed():
    cfg = SampleCfg()
    cfg.from_dict({"mode": "b"})
    assert cfg.mode == "b"
    with pytest.raises(TypeError):
        cfg.from_dict({"mode": "not_in_literal"})


def test_from_dict_list_to_ndarray_conversion():
    cfg = SampleCfg()
    cfg.from_dict({"array": [1.0, 2.0, 3.0]})
    assert isinstance(cfg.array, np.ndarray)
    assert cfg.array.tolist() == [1.0, 2.0, 3.0]


def test_from_dict_nested_baseattrs_recurses_not_replaced():
    cfg = SampleCfg()
    nested_before = cfg.nested
    cfg.from_dict({"nested": {"count": 42, "label": "z"}})
    assert cfg.nested is nested_before  # same object, mutated in place
    assert cfg.nested.count == 42
    assert cfg.nested.label == "z"


def test_from_dict_nested_baseattrs_rejects_non_dict():
    cfg = SampleCfg()
    with pytest.raises(TypeError):
        cfg.from_dict({"nested": "not a dict"})


def test_from_dict_list_of_baseattrs_parsed_per_element():
    cfg = SampleCfg()
    cfg.from_dict({
        "items": [
            {"name": "first", "value": 1},
            {"name": "second", "value": 2},
        ]
    })
    assert len(cfg.items) == 2
    assert cfg.items[0].name == "first" and cfg.items[0].value == 1
    assert cfg.items[1].name == "second" and cfg.items[1].value == 2


def test_from_dict_list_of_baseattrs_rejects_non_dict_element():
    cfg = SampleCfg()
    with pytest.raises(TypeError):
        cfg.from_dict({"items": ["not a dict"]})


def test_from_dict_unknown_keys_silently_skipped():
    cfg = SampleCfg()
    cfg.from_dict({"rate": 1.0, "not_a_field": "ignored"})
    assert cfg.rate == 1.0
    assert not hasattr(cfg, "not_a_field")


def test_from_dict_any_field_raises():
    """Any-typed fields defeat attrs validation, so from_dict refuses to
    deserialize them. Configs must declare concrete types on their fields."""
    cfg = AnyFieldCfg()
    with pytest.raises(TypeError, match="Any"):
        cfg.from_dict({"free": 123})


def test_from_dict_silent_typeerror_caught_by_validator():
    """Regression guard: the type gate is lenient for simple types (see NOTE
    in __check_value_against_attr_type), so a wrong type for ``rate`` is not
    caught here. The attrs validator on __setattr__ catches the mismatch
    instead. If someone re-enables strict isinstance checking in from_dict,
    this test will flip and force a deliberate decision."""
    cfg = SampleCfg()
    with pytest.raises(TypeError):
        # in_range validator rejects non-numeric; proves the safety net exists
        cfg.from_dict({"rate": "not a number"})


# ---- from_json round-trip ------------------------------------------------


def test_from_json_roundtrip_simple():
    cfg = SampleCfg(rate=42.0, name="hi", flag=True, optional_int=3, mode="c")
    dumped = cfg.to_json()
    restored = SampleCfg()
    restored.from_json(dumped)
    assert restored.rate == 42.0
    assert restored.name == "hi"
    assert restored.flag is True
    assert restored.optional_int == 3
    assert restored.mode == "c"


def test_from_json_roundtrip_nested_and_ndarray():
    cfg = SampleCfg()
    cfg.nested.count = 7
    cfg.nested.label = "nested"
    cfg.array = np.array([1.5, 2.5, 3.5])

    restored = SampleCfg()
    restored.from_json(cfg.to_json())

    assert restored.nested.count == 7
    assert restored.nested.label == "nested"
    assert isinstance(restored.array, np.ndarray)
    assert restored.array.tolist() == [1.5, 2.5, 3.5]


# ---- __setattr__ re-runs validators on mutation --------------------------


def test_setattr_reruns_validator_on_mutation():
    cfg = SampleCfg(rate=10.0)
    cfg.rate = 500.0  # still in range
    assert cfg.rate == 500.0
    with pytest.raises(ValueError):
        cfg.rate = -1.0  # out of range [0, 1e3]


# ---- from_file: YAML happy path (+ json/toml smoke) ---------------------


def test_from_file_yaml_loads_nested_config(tmp_path):
    import yaml
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump({
        "my_component": {
            "rate": 33.0,
            "name": "yaml-cfg",
            "nested": {"count": 9},
        }
    }))
    cfg = SampleCfg()
    ok = cfg.from_file(str(path), nested_root_name="my_component")
    assert ok is True
    assert cfg.rate == 33.0
    assert cfg.name == "yaml-cfg"
    assert cfg.nested.count == 9


def test_from_file_yaml_get_common_merges_globals(tmp_path):
    import yaml
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump({
        "/**": {"name": "from_common"},
        "my_component": {"rate": 12.0},
    }))
    cfg = SampleCfg()
    ok = cfg.from_file(str(path), nested_root_name="my_component", get_common=True)
    assert ok is True
    assert cfg.rate == 12.0
    assert cfg.name == "from_common"


def test_from_file_unsupported_extension_raises(tmp_path):
    path = tmp_path / "cfg.ini"
    path.write_text("not really ini")
    cfg = SampleCfg()
    with pytest.raises(ValueError):
        cfg.from_file(str(path))


def test_from_file_empty_nested_root_returns_false(tmp_path):
    import yaml
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump({"other_component": {"rate": 1.0}}))
    cfg = SampleCfg()
    ok = cfg.from_file(str(path), nested_root_name="my_component")
    assert ok is False


def test_from_file_json_happy_path(tmp_path):
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps({"rate": 77.0, "name": "json-cfg"}))
    cfg = SampleCfg()
    ok = cfg.from_file(str(path))
    assert ok is True
    assert cfg.rate == 77.0
    assert cfg.name == "json-cfg"
