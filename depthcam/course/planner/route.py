"""Gates and the shortest route through them: A* on the configuration-space grid, then shortcuts.

A gate is a segment the drone's centre must cross, in order.  Only the part of the segment that is
free in configuration space can be used (:func:`free_interval`); that part is also how much room
the drone has there, which the planner reports.

The route is planned leg by leg (launch -> gate 1 -> gate 2 -> ... -> pad).  Each leg's A* stops on
the first free cell of the next gate, so the crossing points are greedy; the smoother slides them
along their gates afterwards, so this does not cost the final path anything.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from course.course_map import CourseMap
from course.geometry import closest_points_between
from course.planner.cspace import CSpace

SQRT2 = math.sqrt(2.0)


class PlanError(RuntimeError):
    """The course cannot be flown with these settings (blocked gate, no route, ...)."""


@dataclass
class Gate:
    name: str
    a: np.ndarray
    b: np.ndarray
    lo: float = 0.0  # free part of the segment, as fractions of a -> b
    hi: float = 1.0

    def point(self, u: float) -> np.ndarray:
        return self.a + u * (self.b - self.a)

    def project(self, p: np.ndarray) -> np.ndarray:
        e = self.b - self.a
        u = float(np.dot(p - self.a, e) / max(float(np.dot(e, e)), 1e-18))
        return self.point(min(max(u, self.lo), self.hi))

    def project_unclamped(self, p: np.ndarray) -> np.ndarray:
        """Closest point on the whole segment (ignoring which part is free)."""
        e = self.b - self.a
        u = float(np.dot(p - self.a, e) / max(float(np.dot(e, e)), 1e-18))
        return self.point(min(max(u, 0.0), 1.0))

    @property
    def free_width(self) -> float:
        return (self.hi - self.lo) * float(np.linalg.norm(self.b - self.a))


def resolve_gate(spec: Dict[str, Any], name: str, cmap: CourseMap, z_band: Tuple[float, float]) -> Gate:
    """A rules-file gate spec -> segment in the map frame."""
    if "segment" in spec:
        a, b = spec["segment"]
        return Gate(name, np.array(a, dtype=float), np.array(b, dtype=float))
    if "between" in spec:
        first, second = spec["between"]
        pieces = cmap.obstacles_2d(*z_band, rocks_full_height=True)
        pa_list = [o.polygon for o in pieces if o.name == first]
        pb_list = [o.polygon for o in pieces if o.name == second]
        if not pa_list or not pb_list:
            raise PlanError("gate {!r}: {!r} or {!r} is not an obstacle at the flight height".format(name, first, second))
        best = min((closest_points_between(pa, pb) for pa in pa_list for pb in pb_list), key=lambda c: c[2])
        return Gate(name, best[0], best[1])
    if "opening" in spec:
        wall = cmap.feature(spec["opening"])
        if wall["kind"] != "wall" or not wall.get("openings"):
            raise PlanError("gate {!r}: {!r} has no opening".format(name, spec["opening"]))
        opening = wall["openings"][int(spec.get("index", 0))]
        p0, p1 = np.array(wall["p0"]), np.array(wall["p1"])
        d = (p1 - p0) / np.linalg.norm(p1 - p0)
        return Gate(name, p0 + d * opening["s"][0], p0 + d * opening["s"][1])
    if "tube_end" in spec:
        tube = cmap.feature(spec["tube_end"])
        end = np.array(tube[spec.get("end", "p0")])[:2]
        axis = np.array(tube["p1"])[:2] - np.array(tube["p0"])[:2]
        w = np.array([-axis[1], axis[0]]) / np.linalg.norm(axis)
        half = tube["outer"]["half_width"]
        return Gate(name, end - half * w, end + half * w)
    raise PlanError("gate {!r}: unknown gate spec {}".format(name, spec))


def free_interval(gate: Gate, cs: CSpace, step: float = 0.002) -> Gate:
    """Restrict a gate to its longest free stretch in configuration space."""
    length = float(np.linalg.norm(gate.b - gate.a))
    n = max(2, int(math.ceil(length / step)) + 1)
    u = np.linspace(0.0, 1.0, n)
    d, _ = cs.distance(gate.a[None] + u[:, None] * (gate.b - gate.a)[None])
    free = d > 0.0
    best, start = (0, -1, -1), None
    for i, ok in enumerate(list(free) + [False]):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            if i - start > best[0]:
                best = (i - start, start, i - 1)
            start = None
    if best[0] < 2:
        raise PlanError("gate {!r} is blocked for this drone at this heading and height "
                        "(no free point on it)".format(gate.name))
    return Gate(gate.name, gate.a, gate.b, float(u[best[1]]), float(u[best[2]]))


def _segment_distance(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    e = b - a
    t = np.clip(((points - a) @ e) / max(float(e @ e), 1e-18), 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * e), axis=1)


def astar(cs: CSpace, start: Tuple[int, int], goal: np.ndarray, heuristic: np.ndarray) -> List[Tuple[int, int]]:
    """8-connected A* on the grid (no corner cutting).  ``goal`` and ``heuristic`` are (ny, nx)."""
    ny, nx = cs.shape
    blocked = cs.blocked.ravel().tolist()
    is_goal = goal.ravel().tolist()
    h = heuristic.ravel().tolist()
    s = start[0] * nx + start[1]
    if blocked[s]:
        raise PlanError("route start cell {} is blocked".format(start))
    g = [math.inf] * (nx * ny)
    parent = [-1] * (nx * ny)
    closed = bytearray(nx * ny)
    g[s] = 0.0
    heap = [(h[s], 0.0, s)]
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        if is_goal[cur]:
            path = [cur]
            while parent[path[-1]] >= 0:
                path.append(parent[path[-1]])
            return [divmod(c, nx) for c in reversed(path)]
        closed[cur] = 1
        cy, cx = divmod(cur, nx)
        for dx, dy, cost in moves:
            x, y = cx + dx, cy + dy
            if x < 0 or y < 0 or x >= nx or y >= ny:
                continue
            n = y * nx + x
            if blocked[n] or closed[n]:
                continue
            if dx and dy and (blocked[cy * nx + x] or blocked[y * nx + cx]):
                continue
            ng = gc + cost
            if ng < g[n]:
                g[n] = ng
                parent[n] = cur
                heapq.heappush(heap, (ng + h[n], ng, n))
    raise PlanError("no route on the grid (is a gate unreachable for this drone?)")


def _cell_centres(cs: CSpace) -> np.ndarray:
    ny, nx = cs.shape
    cols, rows = np.meshgrid(np.arange(nx), np.arange(ny))
    return np.stack([cs.x0 + (cols + 0.5) * cs.res, cs.y0 + (rows + 0.5) * cs.res], axis=2).reshape(-1, 2)


def route_through(cs: CSpace, start: np.ndarray, gates: Sequence[Gate], goal: np.ndarray) -> Tuple[np.ndarray, List[int]]:
    """Grid route launch -> gates -> goal.  Returns points and the indices of the anchors
    (start, each gate crossing, goal) in that polyline."""
    centres = _cell_centres(cs)
    ny, nx = cs.shape
    points: List[np.ndarray] = [np.asarray(start, dtype=float)]
    anchors = [0]
    cur = cs.cell(start)
    targets: List[Any] = list(gates) + [np.asarray(goal, dtype=float)]
    names = ["the start"] + [g.name for g in gates] + ["the goal"]
    for leg, target in enumerate(targets):
        if isinstance(target, Gate):
            a, b = target.point(target.lo), target.point(target.hi)
            dist = _segment_distance(centres, a, b)
            goal_mask = (dist <= cs.res * 0.75).reshape(ny, nx) & ~cs.blocked
            if not goal_mask.any():
                raise PlanError("gate {!r} has no free grid cell; try a finer --grid".format(target.name))
        else:
            dist = np.linalg.norm(centres - target, axis=1)
            goal_mask = np.zeros((ny, nx), dtype=bool)
            goal_mask[cs.cell(target)] = True
        try:
            cells = astar(cs, cur, goal_mask, (dist / cs.res).reshape(ny, nx))
        except PlanError:
            raise PlanError("no way from {} to {} for this drone (size + margin) at this heading and height; "
                            "something on that stretch is narrower than the drone".format(names[leg], names[leg + 1]))
        points.extend(cs.centre(r, c) for r, c in cells[1:])
        points[-1] = target.project(points[-1]) if isinstance(target, Gate) else target.copy()
        anchors.append(len(points) - 1)
        cur = cells[-1]
    return np.array(points), anchors


def line_of_sight(cs: CSpace, a: np.ndarray, b: np.ndarray) -> bool:
    n = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / (cs.res / 2.0))) + 1)
    pts = a[None] + np.linspace(0.0, 1.0, n)[:, None] * (b - a)[None]
    return not cs.blocked_at(pts[1:-1]).any()


def shortcut(cs: CSpace, points: np.ndarray, anchors: Sequence[int]) -> Tuple[np.ndarray, List[int]]:
    """Greedy line-of-sight simplification of each section between anchors (anchors are kept)."""
    out: List[np.ndarray] = [points[0]]
    new_anchors = [0]
    for s0, s1 in zip(anchors[:-1], anchors[1:]):
        i = s0
        while i < s1:
            j = i + 1
            while j + 1 <= s1 and line_of_sight(cs, points[i], points[j + 1]):
                j += 1
            out.append(points[j])
            i = j
        new_anchors.append(len(out) - 1)
    return np.array(out), new_anchors
