---
orphan: true
---

# {py:mod}`ros_sugar.ui_node.ui_node`

```{py:module} ros_sugar.ui_node.ui_node
```

```{autodoc2-docstring} ros_sugar.ui_node.ui_node
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`UINodeConfig <ros_sugar.ui_node.ui_node.UINodeConfig>`
  -
````

### API

`````{py:class} UINodeConfig
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig

Bases: {py:obj}`ros_sugar.core.component.BaseComponentConfig`

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.from_dict

````

````{py:method} from_file(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> bool
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.from_file

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.update_value

````

````{py:method} get_fields_info(class_object) -> typing.Dict[str, typing.Dict[str, typing.Any]]
:canonical: ros_sugar.ui_node.ui_node.UINodeConfig.get_fields_info
:classmethod:

````

`````
