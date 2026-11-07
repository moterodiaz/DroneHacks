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


def _line_cells(a, b):
    distance = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
    if distance == 0:
        return [a]
    return list(dict.fromkeys((int(round(a[0] + (b[0] - a[0]) * i / distance)), int(round(a[1] + (b[1] - a[1]) * i / distance))) for i in range(distance + 1)))


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


def _sampled_cells(costmap: Costmap, a, b):
    count = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / (costmap.resolution / 2.0))) + 1)
    for t in np.linspace(0.0, 1.0, count):
        yield costmap.world_to_grid(a + t * (b - a))


def _polyline_max_cost(costmap: Costmap, points):
    worst = 0.0
    for a, b in zip(points, points[1:]):
        for cell in _sampled_cells(costmap, a, b):
            worst = max(worst, float(costmap.costs[cell]))
    return worst


def _chord_acceptable(costmap: Costmap, a, b, replaced_max):
    """Same rule as :func:`shortcut_path`: every crossed cell is finite and no
    costlier than the subpath the chord replaces."""
    for cell in _sampled_cells(costmap, a, b):
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
