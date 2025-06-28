# {py:mod}`ros_sugar.events`

```{py:module} ros_sugar.events
```

```{autodoc2-docstring} ros_sugar.events
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`OnAny <ros_sugar.events.OnAny>`
  -
* - {py:obj}`OnChange <ros_sugar.events.OnChange>`
  - ```{autodoc2-docstring} ros_sugar.events.OnChange
    :summary:
    ```
* - {py:obj}`OnChangeEqual <ros_sugar.events.OnChangeEqual>`
  - ```{autodoc2-docstring} ros_sugar.events.OnChangeEqual
    :summary:
    ```
* - {py:obj}`OnEqual <ros_sugar.events.OnEqual>`
  - ```{autodoc2-docstring} ros_sugar.events.OnEqual
    :summary:
    ```
* - {py:obj}`OnContainsAll <ros_sugar.events.OnContainsAll>`
  - ```{autodoc2-docstring} ros_sugar.events.OnContainsAll
    :summary:
    ```
* - {py:obj}`OnContainsAny <ros_sugar.events.OnContainsAny>`
  - ```{autodoc2-docstring} ros_sugar.events.OnContainsAny
    :summary:
    ```
* - {py:obj}`OnDifferent <ros_sugar.events.OnDifferent>`
  - ```{autodoc2-docstring} ros_sugar.events.OnDifferent
    :summary:
    ```
* - {py:obj}`OnGreater <ros_sugar.events.OnGreater>`
  - ```{autodoc2-docstring} ros_sugar.events.OnGreater
    :summary:
    ```
* - {py:obj}`OnLess <ros_sugar.events.OnLess>`
  - ```{autodoc2-docstring} ros_sugar.events.OnLess
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`json_to_events_list <ros_sugar.events.json_to_events_list>`
  - ```{autodoc2-docstring} ros_sugar.events.json_to_events_list
    :summary:
    ```
````

### API

````{py:function} json_to_events_list(json_obj: typing.Union[str, bytes, bytearray]) -> typing.List
:canonical: ros_sugar.events.json_to_events_list

```{autodoc2-docstring} ros_sugar.events.json_to_events_list
```
````

`````{py:class} OnAny(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], **kwargs)
:canonical: ros_sugar.events.OnAny

Bases: {py:obj}`ros_sugar.core.event.Event`

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnAny.callback

```{autodoc2-docstring} ros_sugar.events.OnAny.callback
```

````

````{py:property} under_processing
:canonical: ros_sugar.events.OnAny.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnAny.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnAny.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnAny.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnAny.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnAny.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnAny.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnAny.json
:type: str

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnAny.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnAny.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnAny.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnAny.remove_method

````

`````

`````{py:class} OnChange(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnChange

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnChange
```

````{py:method} callback(msg) -> None
:canonical: ros_sugar.events.OnChange.callback

```{autodoc2-docstring} ros_sugar.events.OnChange.callback
```

````

````{py:property} under_processing
:canonical: ros_sugar.events.OnChange.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnChange.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnChange.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnChange.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnChange.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnChange.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnChange.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnChange.json
:type: str

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnChange.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnChange.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnChange.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnChange.remove_method

````

`````

`````{py:class} OnChangeEqual(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str, typing.List], nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnChangeEqual

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnChangeEqual
```

````{py:method} callback(msg) -> None
:canonical: ros_sugar.events.OnChangeEqual.callback

```{autodoc2-docstring} ros_sugar.events.OnChangeEqual.callback
```

````

````{py:property} under_processing
:canonical: ros_sugar.events.OnChangeEqual.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnChangeEqual.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnChangeEqual.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnChangeEqual.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnChangeEqual.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnChangeEqual.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnChangeEqual.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnChangeEqual.json
:type: str

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnChangeEqual.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnChangeEqual.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnChangeEqual.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnChangeEqual.remove_method

````

`````

`````{py:class} OnEqual(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str, typing.List], nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnEqual

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnEqual
```

````{py:property} under_processing
:canonical: ros_sugar.events.OnEqual.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnEqual.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnEqual.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnEqual.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnEqual.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnEqual.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnEqual.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnEqual.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnEqual.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnEqual.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnEqual.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnEqual.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnEqual.remove_method

````

`````

`````{py:class} OnContainsAll(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.List, nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnContainsAll

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnContainsAll
```

````{py:property} under_processing
:canonical: ros_sugar.events.OnContainsAll.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnContainsAll.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnContainsAll.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnContainsAll.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnContainsAll.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnContainsAll.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnContainsAll.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnContainsAll.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnContainsAll.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnContainsAll.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnContainsAll.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnContainsAll.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnContainsAll.remove_method

````

`````

`````{py:class} OnContainsAny(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.List, nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnContainsAny

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnContainsAny
```

````{py:property} under_processing
:canonical: ros_sugar.events.OnContainsAny.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnContainsAny.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnContainsAny.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnContainsAny.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnContainsAny.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnContainsAny.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnContainsAny.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnContainsAny.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnContainsAny.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnContainsAny.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnContainsAny.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnContainsAny.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnContainsAny.remove_method

````

`````

`````{py:class} OnDifferent(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str, typing.List], nested_attributes: typing.Union[str, typing.List[str]], **kwargs)
:canonical: ros_sugar.events.OnDifferent

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnDifferent
```

````{py:property} under_processing
:canonical: ros_sugar.events.OnDifferent.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnDifferent.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnDifferent.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnDifferent.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnDifferent.trig

````

````{py:property} dictionary
:canonical: ros_sugar.events.OnDifferent.dictionary
:type: typing.Dict

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnDifferent.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnDifferent.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnDifferent.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnDifferent.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnDifferent.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnDifferent.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnDifferent.remove_method

````

`````

`````{py:class} OnGreater(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str], nested_attributes: typing.Union[str, typing.List[str]], or_equal: bool = False, **kwargs)
:canonical: ros_sugar.events.OnGreater

Bases: {py:obj}`ros_sugar.core.event.Event`

```{autodoc2-docstring} ros_sugar.events.OnGreater
```

````{py:property} dictionary
:canonical: ros_sugar.events.OnGreater.dictionary
:type: typing.Dict

```{autodoc2-docstring} ros_sugar.events.OnGreater.dictionary
```

````

````{py:property} under_processing
:canonical: ros_sugar.events.OnGreater.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnGreater.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnGreater.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnGreater.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnGreater.trig

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnGreater.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnGreater.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnGreater.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnGreater.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnGreater.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnGreater.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnGreater.remove_method

````

`````

`````{py:class} OnLess(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str], nested_attributes: typing.Union[str, typing.List[str]], or_equal: bool = False, **kwargs)
:canonical: ros_sugar.events.OnLess

Bases: {py:obj}`ros_sugar.events.OnGreater`

```{autodoc2-docstring} ros_sugar.events.OnLess
```

````{py:property} dictionary
:canonical: ros_sugar.events.OnLess.dictionary
:type: typing.Dict

````

````{py:property} under_processing
:canonical: ros_sugar.events.OnLess.under_processing
:type: bool

````

````{py:method} reset()
:canonical: ros_sugar.events.OnLess.reset

````

````{py:property} name
:canonical: ros_sugar.events.OnLess.name
:type: str

````

````{py:method} clear() -> None
:canonical: ros_sugar.events.OnLess.clear

````

````{py:method} trig() -> None
:canonical: ros_sugar.events.OnLess.trig

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.events.OnLess.set_dictionary

````

````{py:property} json
:canonical: ros_sugar.events.OnLess.json
:type: str

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.events.OnLess.callback

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.events.OnLess.register_method

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.events.OnLess.register_actions

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.events.OnLess.clear_actions

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.events.OnLess.remove_method

````

`````
