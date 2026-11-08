"""Plans a route through a known course: an occupancy grid from the course map's walls at flight
height, inflated as Nav2's inflation layer inflates a costmap, and A* between the places the route
must pass, in the order the map gives.

Coordinates are the course map's ENU metres (x east, y north) on the floor plane. A cell is lethal
within the drone's reach (plus a margin) of a wall or the net; beyond, its cost decays as
252 exp(-k (d - lethal)), Nav2's `InflationLayer::computeCost`, so the cheapest path keeps to the
middle of a corridor instead of hugging a wall. Nothing here knows a course: every place comes
from the map file.
"""
from __future__ import annotations

import heapq
import itertools
import json
import math
import warnings
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np

# Grid cell size, a fifth of the narrowest free band the course leaves the drone
CELL_M = 0.05

# The event drone's prop tips reach this far from its centre; a cell nearer a wall than this plus
# the margin is lethal
REACH_M = 0.28
MARGIN_M = 0.04

# The kit's VL53L1X sees a cone this wide. A boulder is lethal out to its radius plus the cone's
# footprint where it would first meet the boulder, so the drone never reads it as the floor rising
RANGEFINDER_CONE_DEG = 27.0

# A boulder whose top is this far below the flight height has its props pass over its edge; the
# cone then only has to miss its top. Nearer, or of unknown height, the cone's floor footprint holds
MIN_BOULDER_OVERHEAD_M = 0.20

# Past the cone's footprint at a boulder's top, room for the drone's sideways tracking error: twice
# the 5 cm the real drone held its corners to, and within the 5-15 cm SITL's tracker strayed
BOULDER_MARGIN_M = 0.10

# A window is flown only with this much opening above and below the flight height: the airframe's
# half-height plus the rangefinder-held height's error
WINDOW_VERTICAL_CLEARANCE_M = 0.25

# A window is flown square through as one straight leg on its normal, from this far before the
# opening to this far past it. PX4 cuts a turn into the frame when it comes late; past the
# follower's 0.4 m corner arc, the drone is straight for the last 0.3 m before the frame
WINDOW_LEAD_M = 0.7

# Nav2's inflation decay: the cost falls by e every 1/k metres past the lethal distance
COST_SCALING_PER_M = 3.0

# How much a cell's cost lengthens a step through it, Nav2 Smac 2D's `cost_travel_multiplier`: a
# step costs its length times 1 + this x cost / 252. Above Smac's 2, maze corners are not cut
COST_WEIGHT = 4.0

# Nav2's highest cost below lethal
_MAX_COST = 252.0

# A straight leg replacing part of the A* path may cross cells this much costlier than the path's
# own: at 3 per metre, about 3 cm nearer a wall. Without it, sampling a diagonal line rounds onto
# neighbouring cells and leaves the path a staircase of tiny legs
SHORTCUT_COST_TOLERANCE = 1.1

# A corner closer than this to the one before is dropped where the shortcut allows
MIN_LEG_M = 0.15

# The keys every course map must carry
_REQUIRED_KEYS = ("height_m", "launch", "segments", "bounds", "goals", "order", "boulders")

# The goal a route can fly through or around, and the keys that say whether it can be flown
WINDOW_GOAL = "window"
_WINDOW_KEYS = ("at", "along", "size_m", "span_m")


class GoalMovedWarning(UserWarning):
    """A start or goal inside a wall's lethal ring was planned from the nearest free cell instead."""


@dataclass(frozen=True, slots=True)
class CourseGrid:
    """The course as a cost grid."""

    origin_m: tuple[float, float]       # east, north of cell (0, 0)'s centre
    cost: np.ndarray                    # (rows, columns) north by east; inf where lethal


class PlannedCourse(NamedTuple):
    """A route through the course map and the walls it was planned among."""

    route: list[tuple[float, float]]    # (east, north) waypoints from the launch mark to the last goal
    walls: np.ndarray                   # (n, 4) wall and net segments the route keeps clear of
    window_flown: bool                  # whether the route passes through the window


def load_course_map(path: Path) -> dict[str, Any]:
    """The course map at `path`. Raises `ValueError` for a map missing a key the planner needs, or
    whose goal order names a goal it lacks."""
    course_map = json.loads(Path(path).read_text(encoding="utf-8"))
    missing = [key for key in _REQUIRED_KEYS if key not in course_map]
    if missing:
        raise ValueError(f"the course map {path} lacks {missing}")

    unknown = [name for name in course_map["order"] if name not in course_map["goals"]]
    if unknown:
        raise ValueError(f"the course map's order names goals it lacks: {unknown}")

    return course_map


def plan_flight(course_map: Mapping[str, Any], allow_window: bool, tunnel_lead_m: float = 0.0) -> PlannedCourse:
    """The route through the course map's goals in its order, through the window only when
    `allow_window` and the map says the window is passable; otherwise the window's opening is
    closed and the route finds its own way to the next goal. Raises `ValueError` when no route plans."""
    window_flown = allow_window and window_passable(course_map)
    if WINDOW_GOAL in course_map["goals"] and not window_flown:
        course_map = without_window(course_map)

    route = plan_course(course_map, tunnel_lead_m)
    return PlannedCourse(route=route, walls=wall_segments(course_map),
                         window_flown=window_flown and WINDOW_GOAL in course_map["order"])


def window_passable(course_map: Mapping[str, Any]) -> bool:
    """Whether the map's window can be flown at the map's height: the opening spans the height with
    `WINDOW_VERTICAL_CLEARANCE_M` either side, and is wider than the drone's lethal ring twice."""
    window = course_map["goals"].get(WINDOW_GOAL)
    if window is None or any(key not in window for key in _WINDOW_KEYS):
        return False

    bottom_m, top_m = (float(value) for value in window["span_m"])
    height_m = float(course_map["height_m"])
    spans_height = bottom_m + WINDOW_VERTICAL_CLEARANCE_M <= height_m <= top_m - WINDOW_VERTICAL_CLEARANCE_M
    return spans_height and float(window["size_m"][0]) > 2 * (REACH_M + MARGIN_M)


def without_window(course_map: Mapping[str, Any]) -> dict[str, Any]:
    """The course map with the window out of the goal order and its opening walled across, so no
    route passes through it. Raises `ValueError` for a window whose opening the map does not lay."""
    window = course_map["goals"][WINDOW_GOAL]
    if "along" not in window or "size_m" not in window:
        raise ValueError("the course map's window has no wall direction or size to close it with")

    centre = np.asarray(window["at"], dtype=float)
    half_width = float(window["size_m"][0]) / 2 * np.asarray(window["along"], dtype=float)
    closing = [*(centre - half_width).tolist(), *(centre + half_width).tolist()]
    order = [name for name in course_map["order"] if name != WINDOW_GOAL]
    return dict(course_map) | {"segments": [*course_map["segments"], closing], "order": order}


def plan_course(course_map: Mapping[str, Any], tunnel_lead_m: float = 0.0) -> list[tuple[float, float]]:
    """The route from the launch mark through every goal of the course map, in the map's own
    order, as (east, north) waypoints at the map's height. Raises `ValueError` when no route plans.

    With `tunnel_lead_m`, each goal with two ends, such as the tunnel, is flown as one straight
    leg on its axis, from that far outside one end to that far outside the other, each in
    whichever direction makes the shortest route.
    """
    grid = build_grid(np.array(course_map["segments"], dtype=float), course_map["boulders"],
                      tuple(course_map["bounds"]), float(course_map["height_m"]))

    stops = [_stop(grid, course_map["goals"][name], tunnel_lead_m) for name in course_map["order"]]
    launch = (float(course_map["launch"][0]), float(course_map["launch"][1]))
    return _shortest(grid, launch, stops)


def _stop(grid: CourseGrid, goal: Mapping[str, Any], lead_m: float) -> list[tuple[float, float]]:
    """The points a route passes for one goal: a window's two lead-in points on its normal; its
    middle; or with `lead_m` and two ends, the points on its axis `lead_m` outside each end."""
    if "along" in goal:
        return _window_leg(grid, goal)

    if lead_m <= 0 or "ends" not in goal:
        return [(float(goal["at"][0]), float(goal["at"][1]))]

    middle = np.asarray(goal["at"], dtype=float)
    points = []
    for end in np.asarray(goal["ends"], dtype=float):
        outward = (end - middle) / max(float(np.linalg.norm(end - middle)), 1e-9)
        approach = end + lead_m * outward
        points.append((float(approach[0]), float(approach[1])))

    return points


def _window_leg(grid: CourseGrid, window: Mapping[str, Any]) -> list[tuple[float, float]]:
    """The window's straight leg: the points on its normal through the opening's middle, each side
    as far out as `WINDOW_LEAD_M` and clear of every lethal ring, stepping in a cell at a time."""
    centre = np.asarray(window["at"], dtype=float)
    normal = np.array([-window["along"][1], window["along"][0]], dtype=float)
    normal /= max(float(np.linalg.norm(normal)), 1e-9)
    points = []
    for side in (-1.0, 1.0):
        lead_m = WINDOW_LEAD_M
        while lead_m > CELL_M and not _free(grid, centre + side * lead_m * normal):
            lead_m -= CELL_M

        point = centre + side * lead_m * normal
        points.append((float(point[0]), float(point[1])))

    return points


def _free(grid: CourseGrid, point_m: np.ndarray) -> bool:
    """Whether a point lies on a free cell inside the grid."""
    column = int(round((point_m[0] - grid.origin_m[0]) / CELL_M))
    row = int(round((point_m[1] - grid.origin_m[1]) / CELL_M))
    rows, columns = grid.cost.shape
    return 0 <= row < rows and 0 <= column < columns and bool(np.isfinite(grid.cost[row, column]))


def _shortest(grid: CourseGrid, launch: tuple[float, float],
              stops: list[list[tuple[float, float]]]) -> list[tuple[float, float]]:
    """The shortest route through the `stops` in order, each two-point stop a straight leg flown
    in either direction. Raises `ValueError` when no choice of directions plans."""
    passages = [index for index, stop in enumerate(stops) if len(stop) == 2]
    routes, failures = [], []
    for flips in itertools.product((False, True), repeat=len(passages)):
        directed = list(stops)
        for index, flip in zip(passages, flips, strict=True):
            directed[index] = stops[index][::-1] if flip else stops[index]

        try:
            routes.append(_through_stops(grid, launch, directed))
        except ValueError as failure:
            failures.append(str(failure))

    if not routes:
        raise ValueError("; ".join(failures))

    lengths_m = [_length_m([launch, *route]) for route in routes]
    return routes[int(np.argmin(lengths_m))]


def _through_stops(grid: CourseGrid, launch: tuple[float, float],
                   stops: list[list[tuple[float, float]]]) -> list[tuple[float, float]]:
    """A* through the stops in order, a two-point stop's points joined by its straight leg.
    Raises `ValueError` when a straight leg crosses a lethal cell or a stop cannot be reached."""
    route, start, pending = [], launch, []
    for stop in stops:
        pending.append(stop[0])
        if len(stop) == 1:
            continue

        entry_cell, leaving_cell = _cell_of(grid, stop[0]), _cell_of(grid, stop[1])
        if not math.isfinite(_line_cost(grid, entry_cell, leaving_cell)):
            raise ValueError(f"the leg from {stop[0]} to {stop[1]} meets a wall")

        # A* up to the leg's entry, then straight along it; the next stretch starts where it leaves
        leaving_point = _point_of(grid, leaving_cell)
        route += [*plan_route(grid, start, pending), leaving_point]
        start, pending = leaving_point, []

    return [*route, *plan_route(grid, start, pending)]


def _length_m(points: list[tuple[float, float]]) -> float:
    """The length of the polyline through `points`."""
    return sum(math.dist(first, second) for first, second in zip(points[:-1], points[1:], strict=True))


def wall_segments(course_map: Mapping[str, Any]) -> np.ndarray:
    """The course map's walls at flight height and the net's four sides, as (x1, y1, x2, y2) rows."""
    west, east, south, north = (float(value) for value in course_map["bounds"])
    net = [[west, south, east, south], [east, south, east, north], [east, north, west, north],
           [west, north, west, south]]
    return np.array([*course_map["segments"], *net], dtype=float).reshape(-1, 4)


def clearance_m(points: np.ndarray, segments: np.ndarray) -> np.ndarray:
    """The distance from each of (n, 2) points to the nearest of the (x1, y1, x2, y2) segments;
    infinite with none."""
    if len(segments) == 0:
        return np.full(len(points), np.inf)

    # Each point's foot on each segment, clamped to the segment's ends
    starts, directions = segments[:, :2], segments[:, 2:] - segments[:, :2]
    length2 = np.maximum((directions ** 2).sum(axis=1), 1e-12)
    offsets = points[:, None, :] - starts[None, :, :]
    along = np.clip((offsets * directions).sum(axis=2) / length2, 0.0, 1.0)
    feet = starts + along[..., None] * directions
    return np.linalg.norm(points[:, None, :] - feet, axis=2).min(axis=1)


def boulder_lethal_m(boulder: list[float], height_m: float) -> float:
    """How near a boulder's centre the drone's centre may not come at `height_m`: the boulder's
    radius, the rangefinder cone's footprint at its top and `BOULDER_MARGIN_M`, for a boulder (east,
    north, radius, height) whose top is `MIN_BOULDER_OVERHEAD_M` below; otherwise its radius, the
    footprint on the floor and the walls' margin."""
    half_cone = math.tan(math.radians(RANGEFINDER_CONE_DEG / 2))
    radius_m = float(boulder[2])
    if len(boulder) > 3 and height_m - float(boulder[3]) >= MIN_BOULDER_OVERHEAD_M:
        return radius_m + (height_m - float(boulder[3])) * half_cone + BOULDER_MARGIN_M

    return radius_m + height_m * half_cone + MARGIN_M


def build_grid(segments: np.ndarray, boulders: list[list[float]], bounds_m: tuple[float, float, float, float],
               height_m: float) -> CourseGrid:
    """The cost grid at flight height `height_m` over `bounds_m` (west, east, south, north): walls
    from `segments` (x1, y1, x2, y2), the bounds themselves as the net, and `boulders` (east, north,
    radius, optional height), each lethal out to `boulder_lethal_m`."""
    lethal_m = REACH_M + MARGIN_M
    west, east, south, north = bounds_m
    xs = np.arange(west + CELL_M / 2, east, CELL_M)
    ys = np.arange(south + CELL_M / 2, north, CELL_M)
    grid_x, grid_y = np.meshgrid(xs, ys)

    # The distance from each cell to the nearest wall segment, or to the net around the course
    distance = np.minimum.reduce([grid_x - west, east - grid_x, grid_y - south, north - grid_y])
    for x1, y1, x2, y2 in np.asarray(segments, dtype=float).reshape(-1, 4):
        distance = np.minimum(distance, _distance_to_segment(grid_x, grid_y, (x1, y1), (x2, y2)))

    cost = _MAX_COST * np.exp(-COST_SCALING_PER_M * (distance - lethal_m))
    cost[distance < lethal_m] = np.inf

    # Boulders: lethal wherever the rangefinder's cone could meet one
    for boulder in boulders:
        cost[np.hypot(grid_x - boulder[0], grid_y - boulder[1]) < boulder_lethal_m(boulder, height_m)] = np.inf

    return CourseGrid(origin_m=(float(xs[0]), float(ys[0])), cost=cost)


def _distance_to_segment(grid_x: np.ndarray, grid_y: np.ndarray, start: tuple[float, float],
                         end: tuple[float, float]) -> np.ndarray:
    """The distance from every grid point to one segment."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = max(dx * dx + dy * dy, 1e-12)
    along = np.clip(((grid_x - start[0]) * dx + (grid_y - start[1]) * dy) / length2, 0.0, 1.0)
    return np.hypot(grid_x - (start[0] + along * dx), grid_y - (start[1] + along * dy))


def plan_route(grid: CourseGrid, start_m: tuple[float, float],
               goals_m: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The route from `start_m` through every goal in order, as waypoints: A* on the grid between
    each pair, then shortened wherever a straight leg comes no nearer a wall than the A* path did.
    Each goal stays a waypoint, so a route that doubles back from one cannot shortcut past it.

    Raises `ValueError` when a point is lethal or a goal cannot be reached.
    """
    start = _cell_of(grid, start_m)
    waypoints = []
    for goal in goals_m:
        cells = _astar(grid, start, _cell_of(grid, goal))
        waypoints += [_point_of(grid, cell) for cell in _without_short_legs(grid, _shortened(grid, cells))[1:]]
        start = cells[-1]

    return waypoints


def _cell_of(grid: CourseGrid, point_m: tuple[float, float]) -> tuple[int, int]:
    """The (row, column) of the free cell nearest a point: its own, or one at most the lethal
    distance away; warns with `GoalMovedWarning` when it moves the point. Raises when the point is
    outside the course or no free cell is that near."""
    column = int(round((point_m[0] - grid.origin_m[0]) / CELL_M))
    row = int(round((point_m[1] - grid.origin_m[1]) / CELL_M))
    rows, columns = grid.cost.shape
    if not (0 <= row < rows and 0 <= column < columns):
        raise ValueError(f"{point_m} is outside the course")

    if np.isfinite(grid.cost[row, column]):
        return row, column

    # The nearest free cell, within the distance a wall's lethal ring reaches
    free_rows, free_columns = np.nonzero(np.isfinite(grid.cost))
    distance_m = np.hypot(free_rows - row, free_columns - column) * CELL_M
    nearest = int(np.argmin(distance_m)) if len(distance_m) else -1
    if nearest < 0 or distance_m[nearest] > REACH_M + MARGIN_M:
        raise ValueError(f"{point_m} is too near a wall to plan from")

    # A goal moved off a window's or tunnel's axis is a route that misses it: say so
    moved = _point_of(grid, (int(free_rows[nearest]), int(free_columns[nearest])))
    warnings.warn(f"({point_m[0]:.1f}, {point_m[1]:.1f}) moved {math.dist(point_m, moved):.2f} m to "
                  f"({moved[0]:.2f}, {moved[1]:.2f}), out of a wall's lethal ring", GoalMovedWarning, stacklevel=2)
    return int(free_rows[nearest]), int(free_columns[nearest])


def _point_of(grid: CourseGrid, cell: tuple[int, int]) -> tuple[float, float]:
    """The centre of a cell, east and north."""
    return grid.origin_m[0] + cell[1] * CELL_M, grid.origin_m[1] + cell[0] * CELL_M


def _astar(grid: CourseGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    """The cheapest 8-connected cell path from `start` to `goal`, a step's cost its length times
    one plus the weighted cost of the cell it enters."""
    rows, columns = grid.cost.shape
    weight = 1.0 + COST_WEIGHT * np.where(np.isfinite(grid.cost), grid.cost, 0.0) / _MAX_COST
    best = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    frontier = [(0.0, start)]
    steps = [(row_step, column_step) for row_step in (-1, 0, 1) for column_step in (-1, 0, 1)
             if (row_step, column_step) != (0, 0)]

    while frontier:
        _, cell = heapq.heappop(frontier)
        if cell == goal:
            return _walk_back(came_from, goal)

        for row_step, column_step in steps:
            row, column = cell[0] + row_step, cell[1] + column_step
            if not (0 <= row < rows and 0 <= column < columns) or not np.isfinite(grid.cost[row, column]):
                continue

            cost = best[cell] + math.hypot(row_step, column_step) * CELL_M * float(weight[row, column])
            if cost < best.get((row, column), math.inf):
                best[(row, column)] = cost
                came_from[(row, column)] = cell
                heuristic = math.hypot(goal[0] - row, goal[1] - column) * CELL_M
                heapq.heappush(frontier, (cost + heuristic, (row, column)))

    raise ValueError(f"no path from cell {start} to cell {goal}")


def _walk_back(came_from: dict[tuple[int, int], tuple[int, int]], goal: tuple[int, int]) -> list[tuple[int, int]]:
    """The path to `goal` read back through each cell's predecessor."""
    path = [goal]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])

    return path[::-1]


def _shortened(grid: CourseGrid, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """The cell path cut to its corners: from each kept cell, the furthest later cell a straight
    line reaches without crossing a cell much costlier than the path's own between them."""
    kept = [cells[0]]
    index = 0
    while index < len(cells) - 1:
        reach = index + 1
        for later in range(len(cells) - 1, index + 1, -1):
            allowed = SHORTCUT_COST_TOLERANCE * max(float(grid.cost[cell]) for cell in cells[index:later + 1])
            if _line_cost(grid, cells[index], cells[later]) <= allowed:
                reach = later
                break

        kept.append(cells[reach])
        index = reach

    return kept


def _without_short_legs(grid: CourseGrid, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """The corners with any closer than `MIN_LEG_M` to the one before dropped, where the straight
    line past it is allowed as a shortcut: the follower rounds every corner by up to half its legs,
    and a tiny leg makes a tight, slow arc."""
    kept = list(cells)
    index = 1
    while index < len(kept) - 1:
        short = math.dist(kept[index - 1], kept[index]) * CELL_M < MIN_LEG_M
        allowed = SHORTCUT_COST_TOLERANCE * max(_line_cost(grid, kept[index - 1], kept[index]),
                                                _line_cost(grid, kept[index], kept[index + 1]))
        if short and _line_cost(grid, kept[index - 1], kept[index + 1]) <= allowed:
            del kept[index]
        else:
            index += 1

    return kept


def _line_cost(grid: CourseGrid, start: tuple[int, int], end: tuple[int, int]) -> float:
    """The highest cell cost along the straight line between two cells."""
    samples = max(abs(end[0] - start[0]), abs(end[1] - start[1])) * 2 + 1
    rows = np.rint(np.linspace(start[0], end[0], samples)).astype(int)
    columns = np.rint(np.linspace(start[1], end[1], samples)).astype(int)
    return float(grid.cost[rows, columns].max())
