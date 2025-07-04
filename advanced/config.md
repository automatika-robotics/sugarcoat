# Don't Like YAML? We've Got You 😎

Tired of YAML indentation gymnastics? **We feel your pain — and we fixed it.**

In addition the Pythonic configuration API, Sugarcoat allows you to configure your Components using **YAML**, **TOML**, or **JSON** — just clean, intuitive config files written your way.


## YAML

Still love YAML? No problem — just drop the `ros__parameters` noise:

```yaml
/**: # Common parameters for all components
  common_int_param: 0

my_component_name:
  float_param: 1.5
  boolean_param: true
```

## TOML

More of a TOML fan? We've got you covered:

```toml
["/**"]
common_int_param = 0


[my_component_name]
float_param = 1.5
boolean_param = true
```

## JSON

Prefer curly braces? Configure just as easily with JSON:

```json
{
    "/**": {
        "common_int_param": 0,
    },
    "my_component_name": {
        "float_param": 1.5,
        "boolean_param": true,
    }
}
```
:::{seealso} You can check complete examples of detailed configuration files in [kompass source code](https://github.com/automatika-robotics/kompass/tree/main/kompass/params)
:::
