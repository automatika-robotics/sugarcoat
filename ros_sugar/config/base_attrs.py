import json
from typing import (
    Any,
    Callable,
    Dict,
    Optional,
    Union,
    get_args,
    List,
    get_origin,
    _GenericAlias,
)
from copy import deepcopy
import numpy as np
from attrs import asdict, define, fields_dict
from attrs import Attribute
import yaml
import toml
import os


def skip_no_init(a: Attribute, _) -> bool:
    return a.init


@define
class BaseAttrs:
    """
    Implements setattr method to re-use validators at set time
    """

    def __setattr__(self, name: str, value: Any) -> None:
        """Call the validator when we set the field (by default it only runs on __init__)"""
        for attribute in [
            a for a in getattr(self.__class__, "__attrs_attrs__", []) if a.name == name
        ]:
            if attribute.validator is not None:
                attribute.validator(self, attribute, value)
        super().__setattr__(name, value)

    def __str__(self) -> str:
        """
        Pretty print of class attributes/values

        :return: _description_
        :rtype: str
        """
        print_statement = "\n"

        first_level_keys = [attr.name for attr in self.__attrs_attrs__]
        first_level_values = [getattr(self, key) for key in first_level_keys]

        for name, value in zip(first_level_keys, first_level_values):
            # Do not display private attributes
            if not name.startswith("_"):
                print_statement += f"{name}: {value}\n"

        return print_statement

    @classmethod
    def __is_union_type(cls, some_type) -> bool:
        """
        Helper method to check if a type is from typing.Union

        :param some_type: Some type to check
        :type some_type: type

        :return: If type is from typing.Union
        :rtype: bool
        """
        return getattr(some_type, "__origin__", None) is Union

    @classmethod
    def __is_valid_arg_of_union_type(cls, obj, union_types) -> bool:
        """
        Helper method to check if a type is from typing.Union

        :param obj: _description_
        :type obj: _type_
        :param union_types: _description_
        :type union_types: _type_
        :return: _description_
        :rtype: _type_
        """
        _types = [
            get_origin(t) if isinstance(t, _GenericAlias) else t
            for t in get_args(union_types)
        ]
        return any(isinstance(obj, t) for t in _types)

    def asdict(self, filter: Optional[Callable] = None) -> Dict:
        """Convert class to dict.
        :rtype: dict
        """
        return asdict(self, filter=filter)

    def __check_value_against_attr_type(
        self, key: str, value: Any, attribute_to_set: Any, attribute_type: type
    ):
        """Helper method to check if a given value can be parsed to a class attribute

        :param key: Attribute key name
        :type key: str
        :param value: New value to set
        :type value: Any
        :param attribute_to_set: Class attribute to set
        :type attribute_to_set: Any
        :param attribute_type: Class attribute type
        :type attribute_type: type

        :raises TypeError: If value is of incompatible type

        :return: Updated value
        :rtype: Any
        """
        # Union typing requires special treatment
        if self.__is_union_type(attribute_type):
            if not self.__is_valid_arg_of_union_type(value, attribute_type):
                raise TypeError(
                    f"Trying to set with incompatible type. Attribute {key} expecting '{type(attribute_to_set)}' got '{type(value)}'"
                )
        elif isinstance(value, List) and attribute_type is np.ndarray:
            # Turn list into numpy array
            value = np.array(value)

        else:
            # If not a Union type -> check using isinstance
            # Handles only the origin of GenericAlias (dict, list)
            _attribute_type = (
                get_origin(attribute_type)
                if isinstance(attribute_type, _GenericAlias)
                else attribute_type
            )
            if not isinstance(value, _attribute_type):
                raise TypeError(
                    f"Trying to set with incompatible type. Attribute {key} expecting '{type(attribute_to_set)}' got '{type(value)}'"
                )
        return value

    def __parse_from_serialized_list(self, list_attr: List, value: List) -> List:
        """Helper method to parse attribute value from a list of serialized values

        :param list_attr: _description_
        :type list_attr: list
        :param value: _description_
        :type value: list
        :raises TypeError: _description_
        :return: _description_
        :rtype: list
        """
        new_list = []
        attr_val = list_attr[0]
        for val in value:
            if hasattr(attr_val, "__attrs_attrs__"):
                if not isinstance(val, Dict):
                    raise TypeError(
                        f"Trying to set with incompatible type. Attribute expecting list of dictionaries got list of '{type(val)}'"
                    )
                attr_val.from_dict(val)
            new_list.append(deepcopy(attr_val))
        return new_list

    def from_dict(self, dict_obj: Dict) -> None:
        """
        Gets attributes values from given dictionary

        :param dict_obj: Dictionary {attribute_name: attribute_value}
        :type dict_obj: Dict

        :raises ValueError: If attribute_name in dictionary does not exists in class attributes
        :raises TypeError: If attribute_value type in dictionary does not correspond to class attribute type
        """
        for key, value in dict_obj.items():
            if key not in self.asdict().keys():
                continue
            attribute_to_set = getattr(self, key)
            attribute_type = fields_dict(self.__class__)[key].type
            # Check for nested classes
            if hasattr(attribute_to_set, "__attrs_attrs__"):
                if not isinstance(value, Dict):
                    raise TypeError(
                        f"Trying to set with incompatible type. Attribute {key} expecting dictionary got '{type(value)}'"
                    )
                attribute_to_set.from_dict(value)
            elif isinstance(attribute_to_set, List):
                setattr(
                    self,
                    key,
                    self.__parse_from_serialized_list(attribute_to_set, value),
                )
            else:
                # Handle Any typing as it cannot be checked with isinstance
                if attribute_type is Any:
                    continue
                elif attribute_type:
                    value = self.__check_value_against_attr_type(
                        key, value, attribute_to_set, attribute_type
                    )
                setattr(self, key, value)

    def _select_nested_config(
        cls, config: Dict[str, Any], key_path: Optional[str]
    ) -> Dict[str, Any]:
        if not key_path:
            return config
        keys = key_path.split(".")
        for key in keys:
            config = config.get(key, {})
        return config

    def from_file(
        self,
        file_path: str,
        nested_root_name: Union[str, None] = None,
        get_common: bool = False,
    ) -> bool:
        """
        Update class attributes from yaml, json, or toml

        :param file_path: Path to config file (.yaml, .json, .toml)
        :param nested_root_name: Nested root name for the config, defaults to None
        :param get_common: Whether to get extra config root (for merging), defaults to False
        """
        ext: str = os.path.splitext(file_path)[1].lower()

        with open(file_path, "r", encoding="utf-8") as f:
            if ext in [".yaml", ".yml"]:
                raw_config: Dict[str, Any] = yaml.safe_load(f)
            elif ext == ".json":
                raw_config = json.load(f)
            elif ext == ".toml":
                raw_config = toml.load(f)
            else:
                raise ValueError(f"Unsupported config format: {ext}")

        # Extract specific and common config sections
        config = self._select_nested_config(raw_config, nested_root_name)
        extra_config = (
            self._select_nested_config(raw_config, "/**") if get_common else {}
        )

        if not config and not extra_config:
            return False

        merged_config = {**extra_config, **config}
        # Set attributes from final merged config
        self.from_dict(merged_config)
        return True

    def to_json(self) -> Union[str, bytes, bytearray]:
        """
        Dump to json

        :return: _description_
        :rtype: str | bytes | bytearray
        """
        dictionary = self.asdict(filter=skip_no_init)
        serialized_dict = self.__dict_to_serialized_dict(dictionary)
        return json.dumps(serialized_dict)

    def __list_to_serialized_list(self, list_items: List) -> List:
        """Serialize List object items

        :param list_items: _description_
        :type list_items: list
        :return: _description_
        :rtype: list
        """
        serialized_list = []
        for item in list_items:
            # Convert numpy array to list
            if isinstance(item, np.ndarray):
                item = item.tolist()
            if isinstance(item, BaseAttrs):
                serialized_list.append(item.to_json())
            elif isinstance(item, List):
                serialized_list.append(self.__list_to_serialized_list(item))
            elif isinstance(item, tuple):
                serialized_list.append(
                    tuple(self.__list_to_serialized_list(list(item)))
                )
            elif isinstance(item, Dict):
                serialized_list.append(self.__dict_to_serialized_dict(item))
            elif type(item) not in [float, int, str, bool, type(None)]:
                serialized_list.append(str(item))
            else:
                serialized_list.append(item)
        return serialized_list

    def __dict_to_serialized_dict(self, dictionary):
        """Serialize Dictionary object items

        :param dictionary: _description_
        :type dictionary: _type_
        :return: _description_
        :rtype: _type_
        """
        for name, value in dictionary.items():
            # Convert numpy array to list
            if isinstance(value, np.ndarray):
                value = value.tolist()

            if isinstance(value, List):
                dictionary[name] = self.__list_to_serialized_list(value)
            elif isinstance(value, tuple):
                dictionary[name] = tuple(self.__list_to_serialized_list(list(value)))
            elif isinstance(value, Dict):
                dictionary[name] = self.__dict_to_serialized_dict(value)
            elif type(value) not in [float, int, str, bool, type(None)]:
                dictionary[name] = str(value)
        return dictionary

    def from_json(self, json_obj: Union[str, bytes, bytearray]) -> None:
        """
        Gets attributes values from given json

        :param json_obj: Json object
        :type json_obj: str | bytes | bytearray
        """
        dict_obj = json.loads(json_obj)
        self.from_dict(dict_obj)

    def has_attribute(self, attr_name: str) -> bool:
        """
        Checks if class object has attribute with given name

        :param attr_name: _description_
        :type attr_name: str

        :return: If object has attribute with given name
        :rtype: bool
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        obj_to_set = self
        for name in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name):
                return False
            obj_to_set = getattr(obj_to_set, name)
        return True

    def get_attribute_type(self, attr_name: str) -> Optional[type]:
        """
        Gets type of given attribute name

        :param attr_name: _description_
        :type attr_name: str

        :raises AttributeError: If class does not have attribute with given name

        :return: Attribute type
        :rtype: type
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        name_to_set = nested_names[0]
        obj_to_set = self
        obj_class = self
        for name_to_set in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name_to_set):
                raise AttributeError(
                    f"Class '{self.__class__.__name__}' does not have an attribute '{attr_name}'"
                )
            obj_class = obj_to_set
            obj_to_set = getattr(obj_to_set, name_to_set)

        complex_type = fields_dict(obj_class.__class__)[name_to_set].type

        # Extract the simple type
        SIMPLE_TYPES = {int, float, str, bool}
        origin = get_origin(complex_type)
        args = get_args(complex_type)

        # If it's directly a simple type
        if complex_type in SIMPLE_TYPES:
            return complex_type

        # If it's a Union (including Optional)
        if origin is Union:
            for arg in args:
                if arg in SIMPLE_TYPES:
                    return arg
        return None  # No simple type found

    def update_value(self, attr_name: str, attr_value: Any) -> bool:
        """
        Updates the value of an attribute in the class

        :param attr_name: Attribute name - can be nested name
        :type attr_name: str
        :param attr_value: Attribute value
        :type attr_value: Any

        :raises AttributeError: If class does not contain attribute with given name
        :raises TypeError: If class attribute with given name if of different type

        :return: If attribute value is updated
        :rtype: bool
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        name_to_set = nested_names[0]
        obj_to_set = self
        obj_class = self
        for name_to_set in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name_to_set):
                raise AttributeError(
                    f"Class '{self.__class__.__name__}' does not have an attribute '{attr_name}'"
                )
            obj_class = obj_to_set
            obj_to_set = getattr(obj_to_set, name_to_set)

        attribute_type = fields_dict(obj_class.__class__)[name_to_set].type

        if not attribute_type:
            raise TypeError(
                f"Class '{self.__class__.__name__}' attribute '{attr_name}' type unknown"
            )

        if not isinstance(attr_value, attribute_type):
            raise TypeError(
                f"Class '{self.__class__.__name__}' attribute '{attr_name}' expecting type '{attribute_type}', got {type(attr_value)}"
            )
        setattr(obj_class, name_to_set, attr_value)
        return True
