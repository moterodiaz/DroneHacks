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


def plan_route(course_map: dict, costmap: Costmap) -> np.ndarray:
    """Plan launch -> tunnel middle -> window -> pad, with no typed geometry."""
    goals = _goal_centres(course_map)
    ordered = [goals[name] for name in ("launch", "tunnel", "window", "pad")]
    result = []
    for start, target in zip(ordered, ordered[1:]):
        cells = _astar(costmap, costmap.world_to_grid(start), costmap.world_to_grid(target))
        cells = shortcut_path(cells, costmap.costs)
        points = [start] + [costmap.grid_to_world(cell) for cell in cells[1:-1]] + [target]
        if result:
            points = points[1:]
        result.extend(points)
    return np.asarray(result, dtype=float)
