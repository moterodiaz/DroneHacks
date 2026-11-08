from dronesim.rosfake.rclpy_impl import Parameter, PARAMETER_SEPARATOR_STRING, parameter_value_to_python  # noqa: F401


def get_parameter_value(string_value):
    import yaml
    from rcl_interfaces.msg import ParameterValue
    value = yaml.safe_load(string_value)
    return Parameter("x", value=value).get_parameter_value() if value is not None else ParameterValue()
