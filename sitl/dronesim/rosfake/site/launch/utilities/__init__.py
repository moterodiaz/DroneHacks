from dronesim.rosfake.launch_impl import normalize_to_list_of_substitutions, perform_substitutions  # noqa: F401


def is_a(obj, entity_type):
    return isinstance(obj, entity_type)


def is_a_subclass(obj, entity_type):
    return isinstance(obj, entity_type)


def ensure_argument_type(argument, types, argument_name, caller=None):
    if not isinstance(argument, types):
        raise TypeError(f"Expected '{argument_name}' to be of type {types}, got {type(argument)}")
