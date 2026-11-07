"""rosidl_runtime_py helpers for generated messages."""
from collections import OrderedDict


def _value(v):
    if hasattr(v, 'get_fields_and_field_types'):
        return message_to_ordereddict(v)
    if hasattr(v, 'tolist'):
        return v.tolist()
    if isinstance(v, (list, tuple)):
        return [_value(x) for x in v]
    if hasattr(v, 'typecode'):
        return list(v)
    return v


def message_to_ordereddict(msg, *, truncate_length=None, no_arr=False, no_str=False):
    return OrderedDict((k, _value(getattr(msg, k))) for k in msg.get_fields_and_field_types())


def message_to_yaml(msg, **kwargs):
    import yaml
    return yaml.safe_dump(dict(message_to_ordereddict(msg)), default_flow_style=False)


def set_message_fields(msg, values, expand_header_auto=False, expand_time_now=False):
    for name, value in values.items():
        current = getattr(msg, name)
        if hasattr(current, 'get_fields_and_field_types') and isinstance(value, dict):
            set_message_fields(current, value)
        else:
            if isinstance(current, float) and isinstance(value, int) and not isinstance(value, bool):
                value = float(value)
            setattr(msg, name, value)
