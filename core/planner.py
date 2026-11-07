"""Grid planning and cost-aware line-of-sight shortcutting."""

from __future__ import annotations

import heapq
import math

import numpy as np

from .costmap import Costmap


def _goal_centres(course_map):
    goals = {}
    for goal in course_map.get("goals", []):
        name = goal.get("name")
        if name in goals:
            raise ValueError(f"duplicate goal {name}")
        goals[name] = np.asarray(goal.get("centre"), dtype=float)
    required = ("launch", "tunnel", "window", "pad")
    if any(name not in goals or goals[name].shape != (2,) or not np.all(np.isfinite(goals[name])) for name in required):
        raise ValueError("map requires finite launch, tunnel, window, and pad centres")
    return goals


def _astar(costmap: Costmap, start, target):
    costs = costmap.costs
    if not np.isfinite(costs[start]) or not np.isfinite(costs[target]):
        raise ValueError("route endpoint lies in a lethal cell")
    queue = [(0.0, start)]
    came = {}
    score = {start: 0.0}
    while queue:
        _, current = heapq.heappop(queue)
        if current == target:
            path = [current]
            while current != start:
                current = came[current]
                path.append(current)
            return path[::-1]
        for dr, dc in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
            nxt = current[0] + dr, current[1] + dc
            if not (0 <= nxt[0] < costs.shape[0] and 0 <= nxt[1] < costs.shape[1]) or not np.isfinite(costs[nxt]):
                continue
            step = math.hypot(dr, dc) * costmap.resolution
            candidate = score[current] + step * (1.0 + costs[nxt] / 100.0)
            if candidate < score.get(nxt, math.inf):
                score[nxt] = candidate
                came[nxt] = current
                heuristic = math.hypot(nxt[0] - target[0], nxt[1] - target[1]) * costmap.resolution
                heapq.heappush(queue, (candidate + heuristic, nxt))
    raise ValueError("no collision-free route to goal")


def _crossed_cells(start, end):
    """Yield every unit grid cell the segment ``start`` -> ``end`` crosses.

    ``start``/``end`` are float grid coordinates; cell ``i`` spans ``[i, i+1)``.
    Sampling rounded points at unit steps can skip a cell whose interior the
    segment crosses when it hugs a row or column boundary, so a chord could
    pass through a blocked cell that was never sampled. This boundary-
    crossing walk visits every crossed cell. Cells touched only at a grid
    corner are included, which rejects more chords, never fewer.
    """
    x0, y0 = float(start[0]), float(start[1])
    x1, y1 = float(end[0]), float(end[1])
    ix, iy = math.floor(x0), math.floor(y0)
    final = (math.floor(x1), math.floor(y1))
    yield (ix, iy)
    if (ix, iy) == final:
        return
    dx, dy = x1 - x0, y1 - y0
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    t_max_x = ((ix + (sx > 0)) - x0) / dx if dx else math.inf
    t_max_y = ((iy + (sy > 0)) - y0) / dy if dy else math.inf
    t_dx = abs(1.0 / dx) if dx else math.inf
    t_dy = abs(1.0 / dy) if dy else math.inf
    while (ix, iy) != final:
        if t_max_x < t_max_y:
            ix += sx
            t_max_x += t_dx
        else:
            iy += sy
            t_max_y += t_dy
        yield (ix, iy)


def _line_cells(a, b):
    """Every cell crossed by the cell-centre-to-cell-centre chord ``a`` -> ``b``."""
    return list(_crossed_cells((a[0] + 0.5, a[1] + 0.5), (b[0] + 0.5, b[1] + 0.5)))


def shortcut_path(cells, costs):
    """Use the farthest chord whose maximum cost does not exceed the replaced path's."""
    result = [cells[0]]
    start = 0
    while start < len(cells) - 1:
        chosen = start + 1
        for end in range(len(cells) - 1, start, -1):
            chord = _line_cells(cells[start], cells[end])
            chord_max = max(costs[cell] for cell in chord)
            subpath_max = max(costs[cell] for cell in cells[start : end + 1])
            if np.isfinite(chord_max) and chord_max <= subpath_max + 1e-12:
                chosen = end
                break
        result.append(cells[chosen])
        start = chosen
    return result


def _passage_axis(goal):
    """Return ``(axis_start, axis_end)`` for a directed passage goal, else ``None``.

    Directed passages (tunnel, window) declare a traversal axis in their
    params; any goal that declares one is routed through it.
    """
    params = goal.get("params") or {}
    start, end = params.get("axis_start"), params.get("axis_end")
    if start is None and end is None:
        return None
    if start is None or end is None:
        raise ValueError(f"goal {goal.get('name')!r} must declare both axis_start and axis_end")
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    if (
        start.shape != (2,)
        or end.shape != (2,)
        or not np.all(np.isfinite(start))
        or not np.all(np.isfinite(end))
        or np.allclose(start, end)
    ):
        raise ValueError(f"goal {goal.get('name')!r} needs distinct finite 2-vector axis_start/axis_end")
    return start, end


def _segment_cells(costmap: Costmap, a, b):
    """Every costmap cell the world-space segment ``a`` -> ``b`` crosses."""
    costmap.world_to_grid(a)  # bounds check with the usual error
    costmap.world_to_grid(b)
    lo, res = costmap.lower_left, costmap.resolution
    # Grid floats in (row, col) order, matching world_to_grid's cell tuples.
    start = ((float(a[1]) - lo[1]) / res, (float(a[0]) - lo[0]) / res)
    end = ((float(b[1]) - lo[1]) / res, (float(b[0]) - lo[0]) / res)
    return _crossed_cells(start, end)


def _polyline_max_cost(costmap: Costmap, points):
    worst = 0.0
    for a, b in zip(points, points[1:]):
        for cell in _segment_cells(costmap, a, b):
            worst = max(worst, float(costmap.costs[cell]))
    return worst


def _chord_acceptable(costmap: Costmap, a, b, replaced_max):
    """Same rule as :func:`shortcut_path`: every crossed cell is finite and no
    costlier than the subpath the chord replaces."""
    for cell in _segment_cells(costmap, a, b):
        cost = float(costmap.costs[cell])
        if not math.isfinite(cost) or cost > replaced_max + 1e-12:
            return False
    return True


def _prune_goal_snap(points, costmap: Costmap, resolution):
    """Drop trailing snapped cell centres within one grid cell of the exact goal.

    ``shortcut_path`` works on cell indices while legs end at exact goal
    points, so the last kept interior cell can sit a few centimetres from the
    goal. Those micro-legs cannot fit any allowed fillet radius. A point is
    dropped only when the straight replacement chord is acceptable under the
    same cost rule as ``shortcut_path``.
    """
    while len(points) > 2 and float(np.linalg.norm(points[-2] - points[-1])) <= resolution + 1e-12:
        subpath = [points[-3], points[-2], points[-1]]
        if not _chord_acceptable(costmap, points[-3], points[-1], _polyline_max_cost(costmap, subpath)):
            break
        points.pop(-2)
    return points


def plan_route(course_map: dict, costmap: Costmap) -> np.ndarray:
    """Plan launch -> tunnel middle -> window -> pad, with no typed geometry.

    Directed passages are entered and exited along their traversal axis
    (axis_start -> centre -> axis_end), so a tunnel or window can no longer be
    approached from its side.
    """
    goals = _goal_centres(course_map)
    by_name = {goal.get("name"): goal for goal in course_map.get("goals", [])}
    visits = []
    for name in ("launch", "tunnel", "window", "pad"):
        centre = goals[name]
        axis = _passage_axis(by_name[name])
        if axis is None:
            visits.append(centre)
        else:
            visits.extend((axis[0], centre, axis[1]))
    result = []
    for start, target in zip(visits, visits[1:]):
        cells = _astar(costmap, costmap.world_to_grid(start), costmap.world_to_grid(target))
        cells = shortcut_path(cells, costmap.costs)
        points = [start] + [costmap.grid_to_world(cell) for cell in cells[1:-1]] + [target]
        points = _prune_goal_snap(points, costmap, costmap.resolution)
        if result:
            points = points[1:]
        result.extend(points)
    route = np.asarray(result, dtype=float)
    # Drop consecutive coincident visits (e.g. one passage's axis_end on the next's axis_start).
    keep = np.concatenate(([True], np.linalg.norm(np.diff(route, axis=0), axis=1) > 1e-12))
    return route[keep]
