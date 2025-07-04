# {py:mod}`ros_sugar.io.topic`

```{py:module} ros_sugar.io.topic
```

```{autodoc2-docstring} ros_sugar.io.topic
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Topic <ros_sugar.io.topic.Topic>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.Topic
    :summary:
    ```
* - {py:obj}`AllowedTopics <ros_sugar.io.topic.AllowedTopics>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.AllowedTopics
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`get_all_msg_types <ros_sugar.io.topic.get_all_msg_types>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.get_all_msg_types
    :summary:
    ```
* - {py:obj}`get_msg_type <ros_sugar.io.topic.get_msg_type>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.get_msg_type
    :summary:
    ```
* - {py:obj}`msg_type_validator <ros_sugar.io.topic.msg_type_validator>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.msg_type_validator
    :summary:
    ```
````

### API

````{py:function} get_all_msg_types(msg_types_module: types.ModuleType = supported_types) -> typing.List[typing.Type[ros_sugar.io.supported_types.SupportedType]]
:canonical: ros_sugar.io.topic.get_all_msg_types

```{autodoc2-docstring} ros_sugar.io.topic.get_all_msg_types
```
````

````{py:function} get_msg_type(type_name: typing.Union[typing.Type[ros_sugar.io.supported_types.SupportedType], str], msg_types_module: typing.Optional[types.ModuleType] = supported_types) -> typing.Union[typing.Type[ros_sugar.io.supported_types.SupportedType], str]
:canonical: ros_sugar.io.topic.get_msg_type

```{autodoc2-docstring} ros_sugar.io.topic.get_msg_type
```
````

````{py:function} msg_type_validator(*_: typing.Any, value)
:canonical: ros_sugar.io.topic.msg_type_validator

```{autodoc2-docstring} ros_sugar.io.topic.msg_type_validator
```
````

`````{py:class} Topic
:canonical: ros_sugar.io.topic.Topic

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.io.topic.Topic
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.io.topic.Topic.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.io.topic.Topic.from_dict

````

````{py:method} from_file(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> bool
:canonical: ros_sugar.io.topic.Topic.from_file

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.io.topic.Topic.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.io.topic.Topic.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.io.topic.Topic.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.io.topic.Topic.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.io.topic.Topic.update_value

````

`````

`````{py:class} AllowedTopics
:canonical: ros_sugar.io.topic.AllowedTopics

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.io.topic.AllowedTopics
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.io.topic.AllowedTopics.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.io.topic.AllowedTopics.from_dict

````

````{py:method} from_file(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> bool
:canonical: ros_sugar.io.topic.AllowedTopics.from_file

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.io.topic.AllowedTopics.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.io.topic.AllowedTopics.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.io.topic.AllowedTopics.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.io.topic.AllowedTopics.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.io.topic.AllowedTopics.update_value

````

`````
