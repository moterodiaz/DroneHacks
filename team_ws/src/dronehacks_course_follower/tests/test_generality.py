"""Rule 9.5 guard (AGENTS.md §3.9): the package's code holds no course coordinate and no course
feature name. Every place on the course comes from the map file, so the code flies another layout."""
from __future__ import annotations

import ast
import re
from collections.abc import Iterator
from typing import Any

from conftest import PACKAGE_ROOT

# The code the guard reads: the library, the node and the launch file
_SOURCES = sorted([*(PACKAGE_ROOT / "dronehacks_course_follower").glob("*.py"), *(PACKAGE_ROOT / "launch").glob("*.py")])

# Numbers this small are tolerances and steps, not places on a 6 m course
_SMALLEST_COURSE_NUMBER = 0.05

# The goal names are the map schema's own keys, which the code must use to read the map
_SCHEMA_GOAL_NAMES = {"tunnel", "window", "pad"}


def _numbers(value: Any) -> Iterator[float]:
    """Every number in a nested map value."""
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)):
        yield float(value)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _numbers(item)
    elif isinstance(value, dict):
        for item in value.values():
            yield from _numbers(item)


def _course_numbers(course_map: dict[str, Any]) -> set[float]:
    """The map's coordinates and sizes, to the millimetre: launch, walls, net, goals and boulders."""
    places = [course_map["launch"][:2], course_map["segments"], course_map["bounds"], course_map["goals"],
              course_map["boulders"]]
    return {round(abs(number), 3) for number in _numbers(places) if abs(number) > _SMALLEST_COURSE_NUMBER}


def _feature_words(course_map: dict[str, Any]) -> set[str]:
    """The names the map gives its walls, whole and by their leading label (such as a wall's code)."""
    names = {unit for unit in course_map["units"] if unit not in _SCHEMA_GOAL_NAMES}
    return names | {name.split()[0] for name in names}


def test_no_course_coordinate_appears_in_the_package_code(course_map: dict[str, Any]) -> None:
    course = _course_numbers(course_map)
    found = [f"{path.name}:{node.lineno}: {node.value}" for path in _SOURCES
             for node in ast.walk(ast.parse(path.read_text(encoding="utf-8")))
             if isinstance(node, ast.Constant) and isinstance(node.value, (int, float))
             and not isinstance(node.value, bool) and round(abs(node.value), 3) in course]
    assert found == [], "course coordinates in package code"


def test_no_course_feature_name_appears_in_the_package_code(course_map: dict[str, Any]) -> None:
    words = _feature_words(course_map)
    found = [f"{path.name}: {word}" for path in _SOURCES for word in sorted(words)
             if re.search(rf"\b{re.escape(word)}\b", path.read_text(encoding="utf-8"))]
    assert found == [], "course feature names in package code"


def test_the_guard_reads_some_code() -> None:
    assert len(_SOURCES) >= 6
