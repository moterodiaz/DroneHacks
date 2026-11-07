"""Configuration space: grow the 2D obstacle slice by the drone's footprint so the drone is a point.

The drone is a ``length`` x ``width`` rectangle (nose to tail, side to side) that keeps one heading
for the whole flight, grown by ``margin`` on every side.  At a fixed heading the grown obstacle
is the Minkowski sum of each convex obstacle with that rectangle, so "the centre is outside every
grown polygon" means "the whole drone plus margin clears every obstacle".  Holding the heading is
also what the VIO prefers, and it lets one plan be checked exactly.

Heights: the body occupies ``cruise_z +- (height/2 + margin)``; that band selects which openings
the drone fits through and which low obstacles count (see ``CourseMap.obstacles_2d``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from course.course_map import CourseMap, Obstacle2D
from course.geometry import PolygonSet, minkowski_sum, points_in_convex, rect_polygon


@dataclass
class DroneSpec:
    length_m: float = 0.40
    width_m: float = 0.50
    height_m: float = 0.20
    margin_m: float = 0.05

    def footprint(self, yaw_rad: float, extra_m: float = 0.0) -> np.ndarray:
        """The grown body outline centred on the origin, nose along ``yaw_rad`` (ENU)."""
        grow = 2 * (self.margin_m + extra_m)
        return rect_polygon((0.0, 0.0), yaw_rad, self.length_m + grow, self.width_m + grow)

    def band(self, z: float) -> Tuple[float, float]:
        half = self.height_m / 2.0 + self.margin_m
        return z - half, z + half


@dataclass
class CSpace:
    obstacles: List[Obstacle2D]
    polygons: List[np.ndarray]
    x0: float
    y0: float
    res: float
    blocked: np.ndarray  # (ny, nx), True where the drone centre may not be

    def __post_init__(self) -> None:
        self.packed = PolygonSet(self.polygons)
        self._sdf: Optional[np.ndarray] = None
        self._sdf_grad: Optional[Tuple[np.ndarray, np.ndarray]] = None

    @property
    def shape(self) -> Tuple[int, int]:
        return self.blocked.shape

    def cell(self, xy: np.ndarray) -> Tuple[int, int]:
        """(row, column) of the cell holding a point."""
        return (int(math.floor((xy[1] - self.y0) / self.res)), int(math.floor((xy[0] - self.x0) / self.res)))

    def centre(self, row: int, col: int) -> np.ndarray:
        return np.array([self.x0 + (col + 0.5) * self.res, self.y0 + (row + 0.5) * self.res])

    def blocked_at(self, points: np.ndarray) -> np.ndarray:
        """Grid lookup (conservative to one cell); outside the grid counts as blocked."""
        p = np.atleast_2d(points)
        col = np.floor((p[:, 0] - self.x0) / self.res).astype(int)
        row = np.floor((p[:, 1] - self.y0) / self.res).astype(int)
        ny, nx = self.blocked.shape
        inside = (row >= 0) & (row < ny) & (col >= 0) & (col < nx)
        out = np.ones(len(p), dtype=bool)
        out[inside] = self.blocked[row[inside], col[inside]]
        return out

    def distance(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Exact signed distance of the drone centre to the grown obstacles, and its gradient."""
        return self.packed.distance(points)

    def distance_fast(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Bilinear lookup in a signed-distance grid sampled at the cell centres (built on first use).

        Within a few millimetres of :meth:`distance` (slightly optimistic right at convex corners),
        and about 100x faster, which is what the smoother's inner loop needs; final checks use
        :meth:`distance`.
        """
        if self._sdf is None:
            ny, nx = self.blocked.shape
            cols, rows = np.meshgrid(np.arange(nx), np.arange(ny))
            centres = np.stack([self.x0 + (cols.ravel() + 0.5) * self.res,
                                self.y0 + (rows.ravel() + 0.5) * self.res], axis=1)
            values = np.concatenate([self.packed.distance(chunk)[0] for chunk in np.array_split(centres, 64)])
            self._sdf = values.reshape(ny, nx)
            gy, gx = np.gradient(self._sdf, self.res)
            self._sdf_grad = (gx, gy)
        p = np.atleast_2d(points)
        ny, nx = self._sdf.shape
        fx = np.clip((p[:, 0] - self.x0) / self.res - 0.5, 0.0, nx - 1.000001)
        fy = np.clip((p[:, 1] - self.y0) / self.res - 0.5, 0.0, ny - 1.000001)
        ix, iy = np.floor(fx).astype(int), np.floor(fy).astype(int)
        tx, ty = fx - ix, fy - iy

        def lerp(grid: np.ndarray) -> np.ndarray:
            return ((1 - tx) * (1 - ty) * grid[iy, ix] + tx * (1 - ty) * grid[iy, ix + 1]
                    + (1 - tx) * ty * grid[iy + 1, ix] + tx * ty * grid[iy + 1, ix + 1])

        grad = np.stack([lerp(self._sdf_grad[0]), lerp(self._sdf_grad[1])], axis=1)
        norm = np.linalg.norm(grad, axis=1, keepdims=True)
        return lerp(self._sdf), grad / np.maximum(norm, 1e-9)


def build_cspace(cmap: CourseMap, drone: DroneSpec, yaw_rad: float, z: float, res: float = 0.02,
                 rocks_full_height: bool = True, extra_margin: Optional[Dict[str, float]] = None) -> CSpace:
    """Configuration space at cruise height ``z`` for a drone holding heading ``yaw_rad``.

    ``extra_margin`` maps feature names to additional clearance (m) on top of the drone's margin,
    e.g. more room inside a dark tunnel where the localisation is weakest.
    """
    z_lo, z_hi = drone.band(z)
    obstacles = cmap.obstacles_2d(z_lo, z_hi, rocks_full_height=rocks_full_height)
    extra_margin = extra_margin or {}
    unknown = set(extra_margin) - {f["name"] for f in cmap.features}
    if unknown:
        raise KeyError("extra margin for unknown feature(s): {}".format(", ".join(sorted(unknown))))
    footprint = drone.footprint(yaw_rad)
    polygons = [minkowski_sum(o.polygon, drone.footprint(yaw_rad, extra_margin[o.name]) if o.name in extra_margin
                              else footprint) for o in obstacles]

    b = cmap.boundary
    x0, x1 = b["x"]
    y0, y1 = b["y"]
    # The boundary itself, in case the map has no nets: four slabs just outside it, grown too.
    thick = 1.0
    frame = [rect_polygon(((x0 + x1) / 2, y0 - thick / 2), 0.0, x1 - x0 + 2 * thick, thick),
             rect_polygon(((x0 + x1) / 2, y1 + thick / 2), 0.0, x1 - x0 + 2 * thick, thick),
             rect_polygon((x0 - thick / 2, (y0 + y1) / 2), 0.0, thick, y1 - y0 + 2 * thick),
             rect_polygon((x1 + thick / 2, (y0 + y1) / 2), 0.0, thick, y1 - y0 + 2 * thick)]
    polygons += [minkowski_sum(f, footprint) for f in frame]

    nx, ny = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    cols, rows = np.meshgrid(np.arange(nx), np.arange(ny))
    centres = np.stack([x0 + (cols.ravel() + 0.5) * res, y0 + (rows.ravel() + 0.5) * res], axis=1)
    blocked = np.zeros(len(centres), dtype=bool)
    for poly in polygons:
        blocked |= points_in_convex(centres, poly, tol=1e-9)
    return CSpace(obstacles, polygons, x0, y0, res, blocked.reshape(ny, nx))
