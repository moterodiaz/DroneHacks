"""Build a metric obstacle costmap from the contracted course-map schema."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

GRID_SIZE = 122
RESOLUTION = 0.05
LETHAL_CLEARANCE = 0.32  # 0.28 m prop reach + 0.04 m safety margin.
INFLATION_LIMIT = 0.80
RASTER_BUFFER = RESOLUTION / math.sqrt(2.0)  # Covers every point in a lethal cell, not only its centre.
# Navigation tuning values; unlike geometry and clearance, this decay is not contract-pinned.
C_MAX = 100.0
INFLATION_LAMBDA = 5.0


@dataclass(frozen=True)
class Costmap:
    costs: np.ndarray
    origin: tuple[float, float]
    resolution: float = RESOLUTION

    @property
    def lower_left(self) -> np.ndarray:
        return np.asarray(self.origin) - self.resolution * np.array(self.costs.shape[::-1]) / 2

    def world_to_grid(self, point) -> tuple[int, int]:
        xy = (np.asarray(point, dtype=float) - self.lower_left) / self.resolution
        col, row = np.floor(xy).astype(int)
        if not (0 <= row < self.costs.shape[0] and 0 <= col < self.costs.shape[1]):
            raise ValueError(f"point {tuple(point)} is outside the costmap")
        return int(row), int(col)

    def grid_to_world(self, cell) -> np.ndarray:
        row, col = cell
        return self.lower_left + self.resolution * np.array([col + 0.5, row + 0.5])


def _finite_pair(value, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (2,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{label} must be a finite 2-vector")
    return a


def _point_segment_distance(points, start, end):
    delta = end - start
    length2 = float(delta @ delta)
    if length2 <= 0:
        raise ValueError("wall segment must have nonzero length")
    t = np.clip(np.sum((points - start) * delta, axis=-1) / length2, 0.0, 1.0)
    return np.linalg.norm(points - (start + t[..., None] * delta), axis=-1)


def _validated_window(course_map, segments):
    windows = [g for g in course_map.get("goals", []) if g.get("name") == "window"]
    if len(windows) != 1:
        raise ValueError("map must contain exactly one window goal")
    goal = windows[0]
    params = goal.get("params") or {}
    required = ("wall_segment_index", "axis_start", "axis_end", "side", "sill_height", "head_height", "clearance_radius")
    missing = [key for key in required if params.get(key) is None]
    if missing:
        raise ValueError(f"window is missing required params: {', '.join(missing)}")
    index = params["wall_segment_index"]
    if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < len(segments):
        raise ValueError("window wall_segment_index is invalid")
    centre = _finite_pair(goal.get("centre"), "window centre")
    a0 = _finite_pair(params["axis_start"], "window axis_start")
    a1 = _finite_pair(params["axis_end"], "window axis_end")
    wall0, wall1, _ = segments[index]
    axis = a1 - a0
    wall = wall1 - wall0
    scalars = [float(params[k]) for k in ("side", "sill_height", "head_height", "clearance_radius")]
    if not np.all(np.isfinite(scalars)) or scalars[0] <= 0 or scalars[3] <= 0 or scalars[2] <= scalars[1]:
        raise ValueError("window dimensions must be finite and positive")
    if np.linalg.norm(axis) == 0 or abs(float(axis @ wall)) > 1e-6 * np.linalg.norm(axis) * np.linalg.norm(wall):
        raise ValueError("window traversal axis must be nonzero and normal to its wall")
    # Both the wall and traversal lines must pass through the declared opening centre.
    if _point_segment_distance(centre[None, :], wall0, wall1)[0] > 1e-6:
        raise ValueError("window centre is not on its referenced wall")
    cross = abs(np.cross(axis, centre - a0)) / np.linalg.norm(axis)
    if cross > 1e-6:
        raise ValueError("window axis does not cross its centre")
    slice_height = float(course_map.get("slice_height", math.nan))
    if not scalars[1] < slice_height < scalars[2]:
        raise ValueError("map slice_height is outside the window opening")
    return index, centre, float(params["side"]), float(params["clearance_radius"]), wall / np.linalg.norm(wall)


def build_costmap(course_map: dict) -> Costmap:
    """Parse and rasterize a contracted course map."""
    origin = _finite_pair(course_map.get("origin"), "origin")
    segments = []
    for i, segment in enumerate(course_map.get("segments", [])):
        start = _finite_pair(segment.get("start"), f"segment {i} start")
        end = _finite_pair(segment.get("end"), f"segment {i} end")
        thickness = float(segment.get("thickness", 0.102))
        if not np.isfinite(thickness) or thickness <= 0:
            raise ValueError(f"segment {i} thickness must be positive")
        segments.append((start, end, thickness))
    window_index, window_centre, window_side, clearance_radius, wall_unit = _validated_window(course_map, segments)
    # clearance_radius is a reviewed robot-centre exclusion radius. It must fit the portal.
    if clearance_radius >= window_side / 2:
        raise ValueError("window clearance_radius must be smaller than side / 2")

    offsets = -GRID_SIZE * RESOLUTION / 2 + RESOLUTION * (np.arange(GRID_SIZE) + 0.5)
    xx, yy = np.meshgrid(origin[0] + offsets, origin[1] + offsets)
    points = np.stack((xx, yy), axis=-1)
    costs = np.zeros((GRID_SIZE, GRID_SIZE), dtype=float)

    def add_obstacle(surface_distance):
        lethal = surface_distance <= LETHAL_CLEARANCE + RASTER_BUFFER
        inflated = (surface_distance > LETHAL_CLEARANCE + RASTER_BUFFER) & (surface_distance <= INFLATION_LIMIT)
        costs[lethal] = np.inf
        values = C_MAX * np.exp(-INFLATION_LAMBDA * (surface_distance[inflated] - LETHAL_CLEARANCE))
        costs[inflated] = np.maximum(costs[inflated], values)

    for i, (start, end, thickness) in enumerate(segments):
        distance = _point_segment_distance(points, start, end) - thickness / 2
        if i == window_index:
            along = np.abs(np.sum((points - window_centre) * wall_unit, axis=-1))
            # The portal removes this wall's footprint across its physical side. The reviewed
            # clearance radius is retained as the minimum remaining lateral centre margin.
            portal = along <= window_side / 2 - clearance_radius
            distance = np.where(portal, np.inf, distance)
        add_obstacle(distance)

    for i, boulder in enumerate(course_map.get("boulders", [])):
        centre = _finite_pair(boulder.get("centre"), f"boulder {i} centre")
        radius = float(boulder.get("radius", math.nan))
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError(f"boulder {i} radius must be positive")
        add_obstacle(np.linalg.norm(points - centre, axis=-1) - radius)
    return Costmap(costs, tuple(origin))


def obstacle_clearance(point, course_map: dict) -> float:
    """Return distance from a point to the nearest wall/boulder surface."""
    p = _finite_pair(point, "point")[None, :]
    distances = []
    for segment in course_map.get("segments", []):
        distances.append(float(_point_segment_distance(p, _finite_pair(segment["start"], "start"), _finite_pair(segment["end"], "end"))[0] - float(segment.get("thickness", 0.102)) / 2))
    for boulder in course_map.get("boulders", []):
        distances.append(float(np.linalg.norm(p[0] - _finite_pair(boulder["centre"], "centre")) - float(boulder["radius"])))
    return min(distances, default=math.inf)
