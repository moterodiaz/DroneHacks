"""2D geometry helpers (numpy only): convex hulls, oriented rectangles, Minkowski sums, distances.

Every polygon here is an ``(K, 2)`` float array of vertices in counter-clockwise order with no
repeated closing vertex.  The planner only ever needs *convex* polygons: walls are rectangles,
rocks are regular polygons, and anything else is replaced by its convex hull.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np


def convex_hull(points: np.ndarray) -> np.ndarray:
    """Counter-clockwise convex hull of 2D points (Andrew's monotone chain); collinear points dropped."""
    pts = np.unique(np.round(np.asarray(points, dtype=float)[:, :2], 9), axis=0)
    if len(pts) <= 2:
        return pts

    def cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    lower: List[np.ndarray] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: List[np.ndarray] = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def polygon_area(poly: np.ndarray) -> float:
    """Signed area (positive for counter-clockwise)."""
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def min_area_rect(points: np.ndarray) -> Tuple[np.ndarray, float, float, float]:
    """Smallest enclosing rectangle: ``(centre, angle, length, width)``.

    ``angle`` is the direction of the long side in [0, pi); ``length >= width``.
    """
    hull = convex_hull(points)
    if len(hull) == 1:
        return hull[0].copy(), 0.0, 0.0, 0.0
    best = None
    for i in range(len(hull)):
        edge = hull[(i + 1) % len(hull)] - hull[i]
        theta = math.atan2(edge[1], edge[0])
        c, s = math.cos(theta), math.sin(theta)
        to_local = np.array([[c, s], [-s, c]])
        q = hull @ to_local.T
        lo, hi = q.min(axis=0), q.max(axis=0)
        area = float(np.prod(hi - lo))
        if best is None or area < best[0] - 1e-12:
            best = (area, theta, lo, hi, to_local)
    _, theta, lo, hi, to_local = best
    centre = to_local.T @ ((lo + hi) / 2.0)
    ext = hi - lo
    if ext[0] >= ext[1]:
        angle, length, width = theta, float(ext[0]), float(ext[1])
    else:
        angle, length, width = theta + math.pi / 2.0, float(ext[1]), float(ext[0])
    return centre, angle % math.pi, length, width


def rect_polygon(centre: Sequence[float], angle: float, length: float, width: float) -> np.ndarray:
    """Counter-clockwise corners of a rectangle with its long side along ``angle``."""
    d = np.array([math.cos(angle), math.sin(angle)])
    n = np.array([-d[1], d[0]])
    c = np.asarray(centre, dtype=float)[:2]
    hl, hw = length / 2.0, width / 2.0
    return np.array([c - hl * d - hw * n, c + hl * d - hw * n, c + hl * d + hw * n, c - hl * d + hw * n])


def regular_polygon(centre: Sequence[float], radius: float, sides: int = 16) -> np.ndarray:
    """Counter-clockwise regular polygon that *contains* the circle of ``radius`` (circumscribed)."""
    r = radius / math.cos(math.pi / sides)
    a = np.arange(sides) * 2.0 * math.pi / sides + math.pi / sides
    c = np.asarray(centre, dtype=float)[:2]
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], axis=1)


def minkowski_sum(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minkowski sum of two convex polygons (convex hull of all vertex sums)."""
    return convex_hull((a[:, None, :] + b[None, :, :]).reshape(-1, 2))


def points_in_convex(points: np.ndarray, poly: np.ndarray, tol: float = 0.0) -> np.ndarray:
    """Boolean mask: which points lie inside (or within ``tol`` outside) a CCW convex polygon."""
    p = np.asarray(points, dtype=float)
    inside = np.ones(len(p), dtype=bool)
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        e = b - a
        length = math.hypot(e[0], e[1])
        if length == 0.0:
            continue
        cross = (e[0] * (p[:, 1] - a[1]) - e[1] * (p[:, 0] - a[0])) / length
        inside &= cross >= -tol
    return inside


def signed_distance(points: np.ndarray, polys: Sequence[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    """Signed distance from points to a union of CCW convex polygons, and its gradient.

    Positive outside every polygon, negative inside one (minus the depth to that polygon's
    boundary).  The gradient is the unit direction in which the distance grows: away from the
    nearest boundary point outside, towards it inside.  Inside overlapping polygons the value is
    the most negative one, so it is exact outside and a good push direction inside.
    """
    p = np.asarray(points, dtype=float)
    best_d = np.full(len(p), np.inf)
    best_g = np.zeros((len(p), 2))
    for poly in polys:
        a = poly
        ab = np.roll(poly, -1, axis=0) - poly
        ab2 = np.maximum((ab ** 2).sum(axis=1), 1e-18)
        ap = p[:, None, :] - a[None, :, :]
        t = np.clip((ap * ab[None]).sum(axis=2) / ab2[None], 0.0, 1.0)
        q = a[None] + t[..., None] * ab[None]
        diff = p[:, None, :] - q
        dist = np.sqrt((diff ** 2).sum(axis=2))
        k = dist.argmin(axis=1)
        rows = np.arange(len(p))
        dmin = dist[rows, k]
        cross = ab[None, :, 0] * ap[..., 1] - ab[None, :, 1] * ap[..., 0]
        inside = (cross >= 0.0).all(axis=1)
        d = np.where(inside, -dmin, dmin)
        safe = dmin > 1e-12
        direction = np.zeros((len(p), 2))
        direction[safe] = diff[rows, k][safe] / dmin[safe, None]
        edge = ab[k]
        outward = np.stack([edge[:, 1], -edge[:, 0]], axis=1) / np.sqrt(ab2[k])[:, None]
        direction[~safe] = outward[~safe]
        g = np.where(inside[:, None] & safe[:, None], -direction, direction)
        better = d < best_d
        best_d[better] = d[better]
        best_g[better] = g[better]
    return best_d, best_g


class PolygonSet:
    """Many convex polygons packed edge by edge, for a fast :func:`signed_distance` over all of them.

    Same result as ``signed_distance(points, polys)`` but one set of array operations over every
    edge at once instead of a Python loop over polygons (about 20x faster for a course).
    """

    def __init__(self, polys: Sequence[np.ndarray]) -> None:
        self.polys = [np.asarray(p, dtype=float) for p in polys]
        self.a = np.concatenate([p for p in self.polys], axis=0)
        self.ab = np.concatenate([np.roll(p, -1, axis=0) - p for p in self.polys], axis=0)
        self.ab2 = np.maximum((self.ab ** 2).sum(axis=1), 1e-18)
        sizes = [len(p) for p in self.polys]
        self.starts = np.concatenate([[0], np.cumsum(sizes)[:-1]]).astype(int)
        self.owner = np.repeat(np.arange(len(self.polys)), sizes)
        self.outward = np.stack([self.ab[:, 1], -self.ab[:, 0]], axis=1) / np.sqrt(self.ab2)[:, None]

    def distance(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        p = np.atleast_2d(np.asarray(points, dtype=float))
        ap = p[:, None, :] - self.a[None]
        t = np.clip((ap * self.ab[None]).sum(axis=2) / self.ab2[None], 0.0, 1.0)
        diff = ap - t[..., None] * self.ab[None]
        dist = np.sqrt((diff ** 2).sum(axis=2))                                  # (N, E)
        cross = self.ab[None, :, 0] * ap[..., 1] - self.ab[None, :, 1] * ap[..., 0]
        inside = np.logical_and.reduceat(cross >= 0.0, self.starts, axis=1)      # (N, P)
        dmin = np.minimum.reduceat(dist, self.starts, axis=1)                     # (N, P)
        d_poly = np.where(inside, -dmin, dmin)
        best = d_poly.argmin(axis=1)
        rows = np.arange(len(p))
        masked = np.where(self.owner[None, :] == best[:, None], dist, np.inf)
        edge = masked.argmin(axis=1)
        d = d_poly[rows, best]
        dm = dist[rows, edge]
        safe = dm > 1e-12
        direction = np.where(safe[:, None], diff[rows, edge] / np.maximum(dm, 1e-12)[:, None], self.outward[edge])
        grad = np.where((inside[rows, best] & safe)[:, None], -direction, direction)
        return d, grad


def segment_points(a: Sequence[float], b: Sequence[float], step: float) -> np.ndarray:
    """Points from ``a`` to ``b`` (inclusive) no more than ``step`` apart."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / step)) + 1)
    return a[None] + np.linspace(0.0, 1.0, n)[:, None] * (b - a)[None]


def closest_points_between(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Closest pair of points between two disjoint convex polygons.

    When the closest features are parallel edges every pair along the overlap is closest; this
    returns the middle of the overlap, which is what a "gap between two walls" should mean.
    """
    pairs: List[Tuple[float, np.ndarray, np.ndarray]] = []
    for src, dst, flip in ((a, b, False), (b, a, True)):
        for v in src:
            for s0, s1 in zip(dst, np.roll(dst, -1, axis=0)):
                e = s1 - s0
                t = float(np.clip(np.dot(v - s0, e) / max(float(np.dot(e, e)), 1e-18), 0.0, 1.0))
                q = s0 + t * e
                d = float(np.linalg.norm(v - q))
                pairs.append((d, q, v) if flip else (d, v, q))
    dmin = min(p[0] for p in pairs)
    near = [p for p in pairs if p[0] <= dmin + 1e-6]
    pa = np.mean([p[1] for p in near], axis=0)
    pb = np.mean([p[2] for p in near], axis=0)
    return pa, pb, dmin
