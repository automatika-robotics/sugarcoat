---
orphan: true
---

# {py:mod}`ros_sugar.ui_node.elements`

```{py:module} ros_sugar.ui_node.elements
```

```{autodoc2-docstring} ros_sugar.ui_node.elements
:allowtitles:
```

## Module Contents

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`add_additional_ui_elements <ros_sugar.ui_node.elements.add_additional_ui_elements>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.add_additional_ui_elements
    :summary:
    ```
* - {py:obj}`input_topic_card <ros_sugar.ui_node.elements.input_topic_card>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.input_topic_card
    :summary:
    ```
* - {py:obj}`styled_main_inputs_container <ros_sugar.ui_node.elements.styled_main_inputs_container>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_main_inputs_container
    :summary:
    ```
* - {py:obj}`styled_inputs_grid <ros_sugar.ui_node.elements.styled_inputs_grid>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_inputs_grid
    :summary:
    ```
* - {py:obj}`output_topic_card <ros_sugar.ui_node.elements.output_topic_card>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.output_topic_card
    :summary:
    ```
* - {py:obj}`styled_main_outputs_container <ros_sugar.ui_node.elements.styled_main_outputs_container>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_main_outputs_container
    :summary:
    ```
* - {py:obj}`styled_outputs_grid <ros_sugar.ui_node.elements.styled_outputs_grid>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_outputs_grid
    :summary:
    ```
* - {py:obj}`settings_ui_element <ros_sugar.ui_node.elements.settings_ui_element>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.settings_ui_element
    :summary:
    ```
* - {py:obj}`component_settings_div <ros_sugar.ui_node.elements.component_settings_div>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.component_settings_div
    :summary:
    ```
* - {py:obj}`remove_child_from_logging_card <ros_sugar.ui_node.elements.remove_child_from_logging_card>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.remove_child_from_logging_card
    :summary:
    ```
* - {py:obj}`update_logging_card_with_loading <ros_sugar.ui_node.elements.update_logging_card_with_loading>`
  - ```{autodoc2-docstring} ros_sugar.ui_node.elements.update_logging_card_with_loading
    :summary:
    ```
````

### API

````{py:function} add_additional_ui_elements(input_elements: Optional[typing.List[Tuple]], output_elements: Optional[typing.List[Tuple]])
:canonical: ros_sugar.ui_node.elements.add_additional_ui_elements

```{autodoc2-docstring} ros_sugar.ui_node.elements.add_additional_ui_elements
```
````

````{py:function} input_topic_card(topic_name: str, topic_type: str, column_class: str = '') -> FT
:canonical: ros_sugar.ui_node.elements.input_topic_card

```{autodoc2-docstring} ros_sugar.ui_node.elements.input_topic_card
```
````

````{py:function} styled_main_inputs_container(inputs_grid_div_id: str) -> FT
:canonical: ros_sugar.ui_node.elements.styled_main_inputs_container

```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_main_inputs_container
```
````

````{py:function} styled_inputs_grid(number_of_inputs: int) -> tuple
:canonical: ros_sugar.ui_node.elements.styled_inputs_grid

```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_inputs_grid
```
````

````{py:function} output_topic_card(topic_name: str, topic_type: str, column_class: str = '') -> FT
:canonical: ros_sugar.ui_node.elements.output_topic_card

```{autodoc2-docstring} ros_sugar.ui_node.elements.output_topic_card
```
````

````{py:function} styled_main_outputs_container(outputs_grid_div_id: str) -> FT
:canonical: ros_sugar.ui_node.elements.styled_main_outputs_container

```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_main_outputs_container
```
````

````{py:function} styled_outputs_grid(number_of_outputs: int) -> tuple
:canonical: ros_sugar.ui_node.elements.styled_outputs_grid

```{autodoc2-docstring} ros_sugar.ui_node.elements.styled_outputs_grid
```
````

````{py:function} settings_ui_element(setting_name: str, setting_details: dict, field_type, type_args, input_name=None)
:canonical: ros_sugar.ui_node.elements.settings_ui_element

```{autodoc2-docstring} ros_sugar.ui_node.elements.settings_ui_element
```
````

````{py:function} component_settings_div(component_name: str, settings_col_cls: str, ui_elements, nested_ui_elements)
:canonical: ros_sugar.ui_node.elements.component_settings_div

```{autodoc2-docstring} ros_sugar.ui_node.elements.component_settings_div
```
````

````{py:function} remove_child_from_logging_card(logging_card, target_id='loading-dots')
:canonical: ros_sugar.ui_node.elements.remove_child_from_logging_card

```{autodoc2-docstring} ros_sugar.ui_node.elements.remove_child_from_logging_card
```
````

````{py:function} update_logging_card_with_loading(logging_card)
:canonical: ros_sugar.ui_node.elements.update_logging_card_with_loading

```{autodoc2-docstring} ros_sugar.ui_node.elements.update_logging_card_with_loading
```
````
