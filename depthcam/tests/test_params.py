"""dronehacks_lidar.params: numeric parameters take int or float overrides. No ROS: a stand-in node.

rclpy types a parameter by its default, so a double declared without dynamic typing rejects
``start_timeout_s:=20``. ``declare`` asks for dynamic typing on numbers only; ``number`` then
refuses anything that isn't a number, so a string can't slip through.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest

from dronehacks_lidar.params import declare, number, numbers


class FakeNode:
    def __init__(self, overrides=None):
        self.overrides = dict(overrides or {})
        self.declared = {}

    def declare_parameter(self, name, value, descriptor=None):
        self.declared[name] = descriptor
        self.overrides.setdefault(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.overrides[name])


@pytest.fixture
def descriptor(monkeypatch):
    """A stand-in for rcl_interfaces.msg.ParameterDescriptor."""
    msg = types.ModuleType("rcl_interfaces.msg")
    msg.ParameterDescriptor = lambda **kw: SimpleNamespace(**kw)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", types.ModuleType("rcl_interfaces"))
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", msg)


def test_numbers_get_dynamic_typing_and_others_do_not(descriptor):
    node = FakeNode()
    for name, default in (("timeout_s", 20.0), ("count", 3), ("heights", [0.1, 0.2]), ("topic", "/x"),
                          ("land", True), ("empty", [])):
        declare(node, name, default)
    assert node.declared["timeout_s"].dynamic_typing and node.declared["count"].dynamic_typing
    assert node.declared["heights"].dynamic_typing
    assert node.declared["topic"] is None and node.declared["land"] is None and node.declared["empty"] is None


def test_an_int_override_reads_as_a_float(descriptor):
    node = FakeNode({"start_timeout_s": 20, "heights": [1, 0.5]})
    declare(node, "start_timeout_s", 20.0)
    declare(node, "heights", [0.1, 0.2])
    assert number(node, "start_timeout_s") == 20.0 and isinstance(number(node, "start_timeout_s"), float)
    assert numbers(node, "heights") == [1.0, 0.5]


@pytest.mark.parametrize("value", ["20", True, None, [1.0]])
def test_anything_but_a_number_is_refused(value):
    with pytest.raises(ValueError, match="must be a number"):
        number(FakeNode({"x": value}), "x")


def test_a_list_with_a_non_number_is_refused():
    with pytest.raises(ValueError, match="list of numbers"):
        numbers(FakeNode({"x": [0.1, "a"]}), "x")
