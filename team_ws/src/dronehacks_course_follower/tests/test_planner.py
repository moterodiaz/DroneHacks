"""The planner on the packaged course map, and on that map turned, so nothing in it is course-specific."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dronehacks_course_follower.follower import flown_path
from dronehacks_course_follower.planner import (BOULDER_MARGIN_M, MARGIN_M, MIN_BOULDER_OVERHEAD_M, RANGEFINDER_CONE_DEG,
                                                REACH_M, PlannedCourse, boulder_lethal_m, clearance_m, load_course_map,
                                                plan_flight, window_passable)

# A route waypoint counts as passing a goal within one grid cell's diagonal
_CELL_DIAGONAL_M = 0.05 * math.sqrt(2)

# The rangefinder cone's footprint radius per metre below the sensor
_HALF_CONE = math.tan(math.radians(RANGEFINDER_CONE_DEG / 2))


def _path(course_map: dict[str, Any], planned: PlannedCourse) -> np.ndarray:
    """The route as flown from the launch mark, sampled every 2 cm."""
    return flown_path(tuple(course_map["launch"][:2]), planned.route, planned.walls)


def _first_pass(path: np.ndarray, point: list[float]) -> int | None:
    """The first path sample within a grid cell of `point`; None when the path never comes that near."""
    near = np.flatnonzero(np.linalg.norm(path - np.asarray(point), axis=1) <= _CELL_DIAGONAL_M)
    return int(near[0]) if len(near) else None


def _turn(point: list[float], turn_rad: float) -> list[float]:
    """One (east, north) point turned counter-clockwise about the origin."""
    cos, sin = math.cos(turn_rad), math.sin(turn_rad)
    return [cos * point[0] - sin * point[1], sin * point[0] + cos * point[1]]


def _turned(course_map: dict[str, Any], turn_rad: float) -> dict[str, Any]:
    """The course map turned about its origin by `turn_rad`; the net is square about the origin, so
    its bounds stay as they are for a quarter turn."""
    turned = copy.deepcopy(course_map)
    turned["segments"] = [[*_turn(segment[:2], turn_rad), *_turn(segment[2:], turn_rad)]
                          for segment in course_map["segments"]]
    turned["boulders"] = [[*_turn(boulder[:2], turn_rad), *boulder[2:]] for boulder in course_map["boulders"]]
    turned["launch"] = [*_turn(course_map["launch"][:2], turn_rad), course_map["launch"][2] + math.degrees(turn_rad)]
    for goal in turned["goals"].values():
        goal["at"] = _turn(goal["at"], turn_rad)
        if "along" in goal:
            goal["along"] = _turn(goal["along"], turn_rad)
        if "ends" in goal:
            goal["ends"] = [_turn(end, turn_rad) for end in goal["ends"]]

    return turned


def test_the_route_passes_every_goal_in_the_maps_order_and_ends_on_the_last(course_map: dict[str, Any],
                                                                              planned: PlannedCourse) -> None:
    goals = [course_map["goals"][name]["at"] for name in course_map["order"]]
    passes = [_first_pass(_path(course_map, planned), goal) for goal in goals]
    assert planned.window_flown
    assert None not in passes and passes == sorted(passes)
    assert math.dist(planned.route[-1], goals[-1]) <= _CELL_DIAGONAL_M


def test_the_window_is_flown_square_through_its_middle(course_map: dict[str, Any], planned: PlannedCourse) -> None:
    window = course_map["goals"]["window"]
    along = np.asarray(window["along"])
    normal = np.array([-along[1], along[0]])
    offsets = _path(course_map, planned) - np.asarray(window["at"])

    # Near the opening, the path runs on the window's normal: no turn is cut into the frame there
    beside = np.abs(offsets @ along) < window["size_m"][0]
    near_plane = beside & (np.abs(offsets @ normal) < 0.3)
    assert near_plane.any()
    assert np.abs(offsets[near_plane] @ along).max() < 0.02


def test_the_flown_path_keeps_the_props_off_every_wall(course_map: dict[str, Any], planned: PlannedCourse) -> None:
    launch = tuple(course_map["launch"][:2])
    path = flown_path(launch, planned.route, planned.walls)
    assert clearance_m(path, planned.walls).min() >= REACH_M


def test_the_flown_path_keeps_the_rangefinder_cone_off_every_boulder_top(course_map: dict[str, Any],
                                                                         planned: PlannedCourse) -> None:
    path = _path(course_map, planned)
    height_m = course_map["height_m"]
    for east, north, radius_m, top_m in course_map["boulders"]:
        # The cone's edge at the top's height must stay outside the top's disc
        edge_m = np.linalg.norm(path - np.array([east, north]), axis=1).min() - radius_m
        assert edge_m - (height_m - top_m) * _HALF_CONE > 0


def test_a_boulder_with_its_height_is_cleared_by_the_cone_at_its_top() -> None:
    # A boulder 0.3 m below the flight height: the cone's footprint at its top, not on the floor
    with_top = boulder_lethal_m([0.0, 0.0, 0.2, 0.5], height_m=0.8)
    assert with_top == pytest.approx(0.2 + 0.3 * _HALF_CONE + BOULDER_MARGIN_M)
    assert with_top < boulder_lethal_m([0.0, 0.0, 0.2], height_m=0.8)


def test_a_boulder_without_its_height_keeps_the_floor_footprint() -> None:
    assert boulder_lethal_m([0.0, 0.0, 0.2], height_m=0.8) == pytest.approx(0.2 + 0.8 * _HALF_CONE + MARGIN_M)


def test_a_boulder_too_near_the_flight_height_keeps_the_floor_footprint() -> None:
    # Its top a cell under the minimum below the flight height: the props would pass too near it
    too_high = 0.8 - MIN_BOULDER_OVERHEAD_M + 0.05
    floor = boulder_lethal_m([0.0, 0.0, 0.2], height_m=0.8)
    assert boulder_lethal_m([0.0, 0.0, 0.2, too_high], height_m=0.8) == pytest.approx(floor)


def test_a_refused_window_is_walled_across_and_the_route_goes_round(course_map: dict[str, Any]) -> None:
    window = course_map["goals"]["window"]["at"]
    closed = plan_flight(course_map, allow_window=False)
    assert not closed.window_flown
    assert min(math.dist(waypoint, window) for waypoint in closed.route) > 0.5
    assert math.dist(closed.route[-1], course_map["goals"]["pad"]["at"]) <= _CELL_DIAGONAL_M


def test_a_window_the_flight_height_misses_is_not_passable(course_map: dict[str, Any]) -> None:
    # Just under the opening's sill: the map says the window cannot be flown at this height
    low = course_map | {"height_m": course_map["goals"]["window"]["span_m"][0]}
    assert window_passable(course_map)
    assert not window_passable(low)
    assert not plan_flight(low, allow_window=True).window_flown


def test_a_turned_course_plans_a_route_as_long_through_its_turned_goals(course_map: dict[str, Any],
                                                                        planned: PlannedCourse) -> None:
    turned_map = _turned(course_map, math.pi / 2)
    turned = plan_flight(turned_map, allow_window=True)
    passes = [_first_pass(_path(turned_map, turned), turned_map["goals"][name]["at"]) for name in turned_map["order"]]
    assert turned.window_flown
    assert None not in passes and passes == sorted(passes)

    # The grid does not turn with the course, so corners land a cell apart: within 2 % of the length
    launch, turned_launch = course_map["launch"][:2], turned_map["launch"][:2]
    length_m = _length_m([launch, *planned.route])
    assert abs(_length_m([turned_launch, *turned.route]) - length_m) < 0.02 * length_m

    path = flown_path(tuple(turned_launch), turned.route, turned.walls)
    assert clearance_m(path, turned.walls).min() >= REACH_M


def _length_m(points: list[tuple[float, float]]) -> float:
    """The length of the polyline through `points`."""
    return float(np.linalg.norm(np.diff(np.array(points, dtype=float), axis=0), axis=1).sum())


def test_a_map_missing_its_goal_order_is_refused(tmp_path: Path, course_map: dict[str, Any]) -> None:
    path = tmp_path / "no_order.json"
    path.write_text(json.dumps({key: value for key, value in course_map.items() if key != "order"}), encoding="utf-8")
    with pytest.raises(ValueError, match="order"):
        load_course_map(path)
