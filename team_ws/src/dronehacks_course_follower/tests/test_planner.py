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
from dronehacks_course_follower.planner import (BOULDER_HEIGHT_ALLOWANCE_M, BOULDER_MARGIN_M, MIN_LEG_M,
                                                RANGEFINDER_CONE_DEG, REACH_M, PlannedCourse, _folded_onto,
                                                build_grid, clearance_m, load_course_map, plan_flight,
                                                window_passable)

# A route waypoint counts as passing a goal within one grid cell's diagonal
_CELL_DIAGONAL_M = 0.05 * math.sqrt(2)


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
    """The course map turned about its origin by `turn_rad`, a whole number of quarter turns so the
    net stays a rectangle on the grid's axes."""
    turned = copy.deepcopy(course_map)
    west, east, south, north = course_map["bounds"]
    corners = np.array([_turn([x, y], turn_rad) for x in (west, east) for y in (south, north)])
    turned["bounds"] = [float(corners[:, 0].min()), float(corners[:, 0].max()), float(corners[:, 1].min()),
                        float(corners[:, 1].max())]
    turned["segments"] = [[*_turn(segment[:2], turn_rad), *_turn(segment[2:], turn_rad)]
                          for segment in course_map["segments"]]
    turned["boulders"] = [[*_turn(boulder[:2], turn_rad), boulder[2]] for boulder in course_map["boulders"]]
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


@pytest.mark.xfail(reason="at the measured window's height the west rock's ring shortens the window leg's exit "
                          "below the smooth follower's corner arc; stop-and-go's straight legs are unaffected",
                   strict=False)
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


@pytest.mark.xfail(reason="the wider boulder ring shortens the window leg's exit below the follower's corner "
                          "arc, so its hook stays until the window leg keeps its arc off the plane", strict=False)
@pytest.mark.parametrize("quarter_turns", [0, 1, 2, 3])
def test_the_route_turns_onto_and_off_the_windows_leg_once_from_any_side(course_map: dict[str, Any],
                                                                         quarter_turns: int) -> None:
    turned_map = _turned(course_map, quarter_turns * math.pi / 2)
    route = [tuple(turned_map["launch"][:2]), *plan_flight(turned_map, allow_window=True).route]
    entry = _window_leg_start(turned_map, route)

    # A leg shorter than the planner's own shortest beside either end is a hook: two tight turns
    # the follower can only round with tiny arcs, flown almost at rest
    assert math.dist(route[entry - 1], route[entry]) >= MIN_LEG_M
    if entry + 2 < len(route):
        assert math.dist(route[entry + 1], route[entry + 2]) >= MIN_LEG_M


def _window_leg_start(course_map: dict[str, Any], route: list[tuple[float, float]]) -> int:
    """The index of the route point starting the leg that crosses the window's plane inside its opening."""
    window = course_map["goals"]["window"]
    along = np.asarray(window["along"])
    offsets = np.array(route) - np.asarray(window["at"])
    across, beside = offsets @ np.array([-along[1], along[0]]), offsets @ along
    for index in range(len(route) - 1):
        if across[index] * across[index + 1] < 0 and max(abs(beside[index:index + 2])) < window["size_m"][0]:
            return index

    raise AssertionError("the route never crosses the window's opening")


def test_a_corner_a_cell_before_a_legs_entry_is_folded_onto_the_legs_line() -> None:
    grid = build_grid(np.empty((0, 4)), [], (-2.0, 3.0, -2.0, 2.0), 0.0)

    # Arriving from the north-east through a last cell just north of a leg flown east
    approach = [(1.0, 0.8), (0.5, 0.05), (0.5, 0.0)]
    folded = _folded_onto(grid, approach, (1.9, 0.0))

    # The corner is gone and the entry slid back along the leg's line to where the approach's
    # line meets it: 5 cm further down a line falling 0.75 for every 0.5 it runs west
    assert len(folded) == 2 and folded[0] == approach[0]
    assert folded[1][1] == pytest.approx(0.0, abs=1e-9)
    assert folded[1][0] == pytest.approx(0.5 - 0.05 * 0.5 / 0.75)


def test_a_fold_never_moves_a_legs_entry_towards_its_far_end() -> None:
    grid = build_grid(np.empty((0, 4)), [], (-2.0, 3.0, -2.0, 2.0), 0.0)

    # A shallow approach meets the leg's line 11 cm past its entry, more than a cell: folding would
    # shorten the straight the leg exists to give, so the approach is left as planned
    approach = [(-1.0, 0.5), (0.45, 0.05), (0.5, 0.0)]
    assert _folded_onto(grid, approach, (1.9, 0.0)) == approach


def _length_m(points: list[tuple[float, float]]) -> float:
    """The length of the polyline through `points`."""
    return float(np.linalg.norm(np.diff(np.array(points, dtype=float), axis=0), axis=1).sum())


def test_the_flown_path_keeps_the_rangefinder_cone_off_every_boulder_flown_high(course_map: dict[str, Any],
                                                                                planned: PlannedCourse) -> None:
    # The drone holds height from its rest pose, so it flies above the map's height; at the higher
    # height the cone's floor footprint is wider, and the path's centre must stay outside it
    high_m = float(course_map["height_m"]) + BOULDER_HEIGHT_ALLOWANCE_M
    footprint_m = high_m * math.tan(math.radians(RANGEFINDER_CONE_DEG / 2))
    path = flown_path(tuple(course_map["launch"][:2]), planned.route, planned.walls)
    for east, north, radius_m in course_map["boulders"]:
        nearest_m = float(np.min(np.hypot(path[:, 0] - east, path[:, 1] - north)))
        assert nearest_m > radius_m + footprint_m


def test_a_boulder_is_lethal_out_to_the_cone_at_the_higher_height_plus_the_margin() -> None:
    footprint_m = 0.25
    grid = build_grid(np.empty((0, 4)), [[0.0, 0.0, 0.2]], (-2.0, 2.0, -2.0, 2.0), footprint_m)
    xs = grid.origin_m[0] + np.arange(grid.cost.shape[1]) * 0.05
    ys = grid.origin_m[1] + np.arange(grid.cost.shape[0]) * 0.05
    grid_x, grid_y = np.meshgrid(xs, ys)
    lethal = np.isinf(grid.cost) & (np.hypot(grid_x, grid_y) < 1.5)

    # The ring is the radius, the footprint and the boulder margin, to within a cell
    ring_m = float(np.max(np.hypot(grid_x, grid_y)[lethal]))
    assert ring_m == pytest.approx(0.2 + footprint_m + BOULDER_MARGIN_M, abs=0.05)


def test_a_map_missing_its_goal_order_is_refused(tmp_path: Path, course_map: dict[str, Any]) -> None:
    path = tmp_path / "no_order.json"
    path.write_text(json.dumps({key: value for key, value in course_map.items() if key != "order"}), encoding="utf-8")
    with pytest.raises(ValueError, match="order"):
        load_course_map(path)
