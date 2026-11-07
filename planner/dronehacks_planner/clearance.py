"""Exact distances to the course's obstacles, and the cost grid A* searches.

Distances are computed from the map's geometry, not from a rasterised image, so
the clearance a route reports does not depend on the grid resolution. The grid
only decides where A* may step; every final route is re-checked here at
centimetre spacing.

Two quantities, both in metres:

* **surface distance**: from a point to the nearest obstacle surface
  (negative inside one). This is the "clearance" a route reports.
* **margin**: surface distance minus that obstacle's inscribed radius (the
  closest the drone's centre may come: one radius for walls and the net, one
  per boulder). A point is lethal when its margin is below 0. Cost is a
  function of margin.

The cost follows Nav2's inflation layer: lethal inside the inscribed radius,
then ``exp(-k * margin)`` out to the inflation radius, then 0. A step's
traversal cost is its length times ``1 + weight * cost``.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .course_map import CourseMap, Point

_CHUNK = 4096  # points per vectorised batch; bounds memory on the Jetson


@dataclasses.dataclass(frozen=True)
class CostModel:
    inscribed_radius_m: float
    inflation_radius_m: float
    cost_scaling_per_m: float
    cost_weight: float

    def multiplier(self, margin: np.ndarray) -> np.ndarray:
        """Per-metre traversal multiplier for each margin: 1 in the open, inf if lethal."""
        margin = np.asarray(margin, dtype=float)
        cutoff = self.inflation_radius_m - self.inscribed_radius_m
        cost = np.where(margin <= cutoff, np.exp(-self.cost_scaling_per_m * np.maximum(margin, 0.0)), 0.0)
        mult = 1.0 + self.cost_weight * cost
        return np.where(margin < 0.0, np.inf, mult)


class ObstacleField:
    """Exact distance queries against a map's segments and boulders.

    ``boulder_radii`` is each boulder's inscribed radius, in map order.
    """

    def __init__(self, cmap: CourseMap, model: CostModel, boulder_radii: Sequence[float]):
        self.model = model
        if len(boulder_radii) != len(cmap.boulders):
            raise ValueError("need one inscribed radius per boulder")
        self.boulder_radii = np.asarray(boulder_radii, dtype=float)
        if cmap.segments:
            self._a = np.array([s.start for s in cmap.segments], dtype=float)
            b = np.array([s.end for s in cmap.segments], dtype=float)
            self._d = b - self._a
            self._len2 = np.einsum("ij,ij->i", self._d, self._d)
            self._half = np.array([s.thickness / 2.0 for s in cmap.segments], dtype=float)
        else:
            self._a = None
        if cmap.boulders:
            self._c = np.array([b.centre for b in cmap.boulders], dtype=float)
            self._r = np.array([b.radius for b in cmap.boulders], dtype=float)
        else:
            self._c = None

    def wall_distance(self, pts: np.ndarray) -> np.ndarray:
        pts = _as_points(pts)
        if self._a is None:
            return np.full(len(pts), np.inf)
        out = np.empty(len(pts))
        safe_len2 = np.where(self._len2 > 0, self._len2, 1.0)
        for lo in range(0, len(pts), _CHUNK):
            p = pts[lo:lo + _CHUNK, None, :]                      # (n, 1, 2)
            ap = p - self._a[None, :, :]                          # (n, m, 2)
            t = np.einsum("nmk,mk->nm", ap, self._d) / safe_len2  # (n, m)
            t = np.clip(np.where(self._len2 > 0, t, 0.0), 0.0, 1.0)
            closest = self._a[None, :, :] + t[:, :, None] * self._d[None, :, :]
            dist = np.hypot(*(p - closest).transpose(2, 0, 1)) - self._half[None, :]
            out[lo:lo + _CHUNK] = dist.min(axis=1)
        return out

    def _boulders(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Per point: nearest boulder surface distance, and smallest boulder margin."""
        if self._c is None:
            inf = np.full(len(pts), np.inf)
            return inf, inf
        surface = np.empty(len(pts))
        margin = np.empty(len(pts))
        for lo in range(0, len(pts), _CHUNK):
            p = pts[lo:lo + _CHUNK, None, :]
            dist = np.hypot(*(p - self._c[None, :, :]).transpose(2, 0, 1)) - self._r[None, :]
            surface[lo:lo + _CHUNK] = dist.min(axis=1)
            margin[lo:lo + _CHUNK] = (dist - self.boulder_radii[None, :]).min(axis=1)
        return surface, margin

    def query(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """``(surface_distance, margin)`` for each point."""
        pts = _as_points(pts)
        wall = self.wall_distance(pts)
        boulder_surface, boulder_margin = self._boulders(pts)
        surface = np.minimum(wall, boulder_surface)
        margin = np.minimum(wall - self.model.inscribed_radius_m, boulder_margin)
        return surface, margin

    def margin(self, pts: np.ndarray) -> np.ndarray:
        return self.query(pts)[1]

    def segment_query(self, a: Point, b: Point) -> Tuple[float, float, Point]:
        """Exact ``(surface distance, margin, nearest point on a-b)`` for the
        straight segment ``a -> b``, with no sampling.

        Segment-to-segment distance in 2D is 0 if they cross, else the
        smallest of the four endpoint-to-segment distances. Collinear
        overlapping segments count as crossing, which errs safe.
        """
        a_ = np.asarray(a, dtype=float)
        b_ = np.asarray(b, dtype=float)
        e = b_ - a_
        e2 = float(e @ e)
        best_surface, best_margin, at = math.inf, math.inf, (float(a_[0]), float(a_[1]))
        if self._a is not None:
            A, D, half = self._a, self._d, self._half
            B = A + D
            m = len(A)
            d1, _ = _point_segment(np.broadcast_to(a_, (m, 2)), A, D, self._len2)
            d2, _ = _point_segment(np.broadcast_to(b_, (m, 2)), A, D, self._len2)
            ee = np.broadcast_to(e, (m, 2))
            e2s = np.full(m, e2)
            d3, c3 = _point_segment(A, np.broadcast_to(a_, (m, 2)), ee, e2s)
            d4, c4 = _point_segment(B, np.broadcast_to(a_, (m, 2)), ee, e2s)
            o1 = _cross(e, A - a_)
            o2 = _cross(e, B - a_)
            o3 = _cross(D, a_ - A)
            o4 = _cross(D, b_ - A)
            crossing = (o1 * o2 <= 0) & (o3 * o4 <= 0)
            stacked = np.stack([d1, d2, d3, d4])                       # (4, m)
            which = np.argmin(stacked, axis=0)
            dist = np.where(crossing, 0.0, stacked[which, np.arange(m)])
            near = np.where((which == 0)[:, None], a_, np.where((which == 1)[:, None], b_,
                            np.where((which == 2)[:, None], c3, c4)))
            denom = _cross(e, D)
            safe = np.where(np.abs(denom) > 1e-15, denom, 1.0)
            t = np.clip(_cross(A - a_, D) / safe, 0.0, 1.0)
            near = np.where(crossing[:, None], a_ + t[:, None] * e, near)
            surface = dist - half
            margin = surface - self.model.inscribed_radius_m
            i = int(np.argmin(surface))
            best_surface, at = float(surface[i]), (float(near[i, 0]), float(near[i, 1]))
            best_margin = float(margin.min())
        if self._c is not None:
            k = len(self._c)
            dc, cc = _point_segment(self._c, np.broadcast_to(a_, (k, 2)), np.broadcast_to(e, (k, 2)),
                                    np.full(k, e2))
            surface = dc - self._r
            i = int(np.argmin(surface))
            if surface[i] < best_surface:
                best_surface, at = float(surface[i]), (float(cc[i, 0]), float(cc[i, 1]))
            best_margin = min(best_margin, float((surface - self.boulder_radii).min()))
        return best_surface, best_margin, at

    def wall_free_distance(self, pts: np.ndarray, dirs: np.ndarray, stop_at: float,
                           max_range: float = 10.0, iterations: int = 200) -> np.ndarray:
        """How far each point can travel along its unit direction before it is
        within ``stop_at`` of a wall or the net (boulders are ignored: they are
        lower than the drone). Sphere tracing on the exact wall distance.
        Where tracing has not converged, the distance so far is returned,
        which is an underestimate and so errs safe."""
        pts = _as_points(pts)
        dirs = _as_points(dirs)
        t = np.zeros(len(pts))
        active = np.ones(len(pts), dtype=bool)
        for _ in range(iterations):
            if not active.any():
                break
            d = self.wall_distance(pts[active] + t[active, None] * dirs[active]) - stop_at
            idx = np.flatnonzero(active)
            hit = d < 1e-3
            t[idx] += np.where(hit, 0.0, d)
            done = hit | (t[idx] >= max_range)
            active[idx[done]] = False
        return np.minimum(t, max_range)


@dataclasses.dataclass
class CostGrid:
    """A regular grid over the course; cell ``(iy, ix)`` has its centre at
    ``(x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res)``."""

    x0: float
    y0: float
    res: float
    nx: int
    ny: int
    surface: np.ndarray  # (ny, nx) metres
    margin: np.ndarray   # (ny, nx) metres
    mult: np.ndarray     # (ny, nx) per-metre traversal multiplier, inf = lethal

    def cell(self, p: Point) -> Optional[int]:
        """Flat index of the cell containing ``p``, or None outside the grid."""
        ix = int(math.floor((p[0] - self.x0) / self.res))
        iy = int(math.floor((p[1] - self.y0) / self.res))
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return iy * self.nx + ix
        return None

    def centre(self, flat: int) -> Point:
        iy, ix = divmod(flat, self.nx)
        return (self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res)


def build_grid(cmap: CourseMap, field: ObstacleField, res: float) -> CostGrid:
    """Grid covering the whole map plus two cells of border."""
    min_x, min_y, max_x, max_y = cmap.bounds()
    x0 = math.floor(min_x / res) * res - 2 * res
    y0 = math.floor(min_y / res) * res - 2 * res
    nx = int(math.ceil((max_x - x0) / res)) + 2
    ny = int(math.ceil((max_y - y0) / res)) + 2
    xs = x0 + (np.arange(nx) + 0.5) * res
    ys = y0 + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    surface, margin = field.query(pts)
    mult = field.model.multiplier(margin)
    return CostGrid(
        x0=x0, y0=y0, res=res, nx=nx, ny=ny,
        surface=surface.reshape(ny, nx),
        margin=margin.reshape(ny, nx),
        mult=mult.reshape(ny, nx),
    )


def sample_segment(a: Point, b: Point, step: float) -> np.ndarray:
    """Points from ``a`` to ``b`` inclusive, no more than ``step`` apart."""
    n = max(1, int(math.ceil(math.dist(a, b) / step)))
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return np.asarray(a, dtype=float)[None, :] * (1 - t) + np.asarray(b, dtype=float)[None, :] * t


def segment_cost(field: ObstacleField, a: Point, b: Point, step: float) -> Tuple[float, float]:
    """``(traversal cost, min margin)`` of the straight segment ``a -> b``.

    Cost is the trapezoid-rule integral of the multiplier along the segment
    (inf if any sample is lethal), so a straight segment costs the same,
    to sampling error, whether or not it is split.
    """
    pts = sample_segment(a, b, step)
    margin = field.margin(pts)
    mult = field.model.multiplier(margin)
    length = math.dist(a, b)
    if not np.all(np.isfinite(mult)):
        return math.inf, float(margin.min())
    return length * float(((mult[:-1] + mult[1:]) * 0.5).mean()), float(margin.min())


def _cross(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]


def _point_segment(p: np.ndarray, s0: np.ndarray, d: np.ndarray, len2: np.ndarray):
    """Row-wise distance from points ``p`` to segments ``s0 + t*d``, and the nearest points."""
    safe = np.where(len2 > 0, len2, 1.0)
    t = np.clip(np.where(len2 > 0, np.einsum("ij,ij->i", p - s0, d) / safe, 0.0), 0.0, 1.0)
    near = s0 + t[:, None] * d
    return np.hypot(*(p - near).T), near


def conservative_min_margin(field: "ObstacleField", pts: np.ndarray) -> Tuple[float, int]:
    """A lower bound on the margin anywhere on the polyline through ``pts``
    (and on any curve whose points between samples stay within half a sample
    spacing of them): the smallest sampled margin minus half the largest
    spacing. Margin is 1-Lipschitz, so nothing between samples can be lower.
    Returns ``(bound, index of the worst sample)``."""
    pts = _as_points(pts)
    margin = field.margin(pts)
    gap = float(np.max(np.hypot(*np.diff(pts, axis=0).T))) if len(pts) > 1 else 0.0
    i = int(np.argmin(margin))
    return float(margin[i]) - gap / 2.0, i


def _as_points(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts[None, :]
    return pts
