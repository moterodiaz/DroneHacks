"""Parameter helpers shared by the nodes.

rclpy types a parameter by its default, so one declared ``20.0`` rejects an override written
``20``. That's an easy slip in a launch file or a ``-p`` argument. So numeric parameters are
declared with dynamic typing, and :func:`number` reads them back as floats, refusing anything
that isn't a number. Strings and booleans keep strict typing, so a misspelt ``true`` is still an
error.
"""

from __future__ import annotations

from typing import Any, List


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def declare(node: Any, name: str, default: Any) -> Any:
    """Declare *name* on *node*. A number, or a list of numbers, accepts int and float overrides."""
    if _is_number(default) or (isinstance(default, (list, tuple)) and default and all(_is_number(v) for v in default)):
        from rcl_interfaces.msg import ParameterDescriptor
        return node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return node.declare_parameter(name, default)


def number(node: Any, name: str) -> float:
    """The numeric parameter *name* as a float. Raises ValueError for anything else."""
    value = node.get_parameter(name).value
    if not _is_number(value):
        raise ValueError("parameter {} must be a number (got {!r})".format(name, value))
    return float(value)


def numbers(node: Any, name: str) -> List[float]:
    """The numeric list parameter *name* as floats. Raises ValueError for anything else."""
    value = node.get_parameter(name).value
    if not isinstance(value, (list, tuple)) or not all(_is_number(v) for v in value):
        raise ValueError("parameter {} must be a list of numbers (got {!r})".format(name, value))
    return [float(v) for v in value]
