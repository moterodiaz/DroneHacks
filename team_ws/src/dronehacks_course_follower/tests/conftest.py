"""Shared fixtures: the packaged course map and its plan. No ROS: the package root goes on the path."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from dronehacks_course_follower.planner import PlannedCourse, load_course_map, plan_flight  # noqa: E402

# The course map shipped in the package
MAP_PATH = PACKAGE_ROOT / "maps" / "course_map.json"


@pytest.fixture(scope="session")
def course_map() -> dict[str, Any]:
    """The packaged course map."""
    return load_course_map(MAP_PATH)


@pytest.fixture(scope="session")
def planned(course_map: dict[str, Any]) -> PlannedCourse:
    """The packaged map's plan, window allowed."""
    return plan_flight(course_map, allow_window=True)
