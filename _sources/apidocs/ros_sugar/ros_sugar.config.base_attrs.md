---
orphan: true
---

# {py:mod}`ros_sugar.config.base_attrs`

```{py:module} ros_sugar.config.base_attrs
```

```{autodoc2-docstring} ros_sugar.config.base_attrs
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`BaseAttrs <ros_sugar.config.base_attrs.BaseAttrs>`
  - ```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs
    :summary:
    ```
````

### API

`````{py:class} BaseAttrs
:canonical: ros_sugar.config.base_attrs.BaseAttrs

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.config.base_attrs.BaseAttrs.asdict

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.asdict
```

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.config.base_attrs.BaseAttrs.from_dict

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.from_dict
```

````

````{py:method} from_file(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> bool
:canonical: ros_sugar.config.base_attrs.BaseAttrs.from_file

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.from_file
```

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.config.base_attrs.BaseAttrs.to_json

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.to_json
```

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.config.base_attrs.BaseAttrs.from_json

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.from_json
```

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.config.base_attrs.BaseAttrs.has_attribute

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.has_attribute
```

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.config.base_attrs.BaseAttrs.get_attribute_type

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.get_attribute_type
```

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.config.base_attrs.BaseAttrs.update_value

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.update_value
```

````

````{py:method} get_fields_info(class_object) -> typing.Dict[str, typing.Dict[str, typing.Any]]
:canonical: ros_sugar.config.base_attrs.BaseAttrs.get_fields_info
:classmethod:

```{autodoc2-docstring} ros_sugar.config.base_attrs.BaseAttrs.get_fields_info
```

````

`````
