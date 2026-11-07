"""Arc-length-sampled, feed-forward trajectory generation.

The public sampling methods accept path arc length ``s`` in metres. Calling cadence is
intentionally outside this Layer-B module, and no cross-track feedback is applied.
"""

from __future__ import annotations

import math

import numpy as np

R_MIN, R_MAX = 0.20, 0.35
V_MAX = 1.0  # contracts/px4.md documents the organizer MPC_XY_VEL_MAX cap.
MAZE_FIRST_CORNER_MAX = 0.75  # PX4 1.16 attitude tune overshoots there; rule 8.5 forbids retuning it.
YAW_LOOKAHEAD = 0.80


class Trajectory:
    """Continuous piecewise-linear geometry with smooth jerk-limited speed samples."""

    def __init__(self, waypoints, tunnel_goal, clearance_at=None, a_lat=1.5, ds=0.02):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            raise ValueError("waypoints must be an Nx2 array with N >= 2")
        # Circular fillets provide continuous position and tangent at each retained corner.
        # Radius is clamped to the requested band and locally available free space beyond
        # the 0.32 m lethal boundary.
        radii = []
        for point in points[1:-1]:
            available = R_MAX if clearance_at is None else max(0.0, float(clearance_at(point)) - 0.32)
            if available < R_MIN:
                raise ValueError("local clearance cannot fit the minimum 0.20 m corner fillet")
            radii.append(min(R_MAX, available))
        self.corner_radii = np.asarray(radii)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(lengths == 0):
            raise ValueError("consecutive waypoints must differ")
        geometry = [points[0]]
        corner_positions = []
        for i, vertex in enumerate(points[1:-1], start=1):
            incoming = (vertex - points[i - 1]) / lengths[i - 1]
            outgoing = (points[i + 1] - vertex) / lengths[i]
            turn = math.acos(float(np.clip(incoming @ outgoing, -1.0, 1.0)))
            cross = float(np.cross(incoming, outgoing))
            if turn < 1e-5 or abs(cross) < 1e-8:
                geometry.append(vertex)
                corner_positions.append(vertex)
                continue
            radius = self.corner_radii[i - 1]
            tangent_distance = radius * math.tan(turn / 2)
            if tangent_distance >= min(lengths[i - 1], lengths[i]) / 2:
                raise ValueError("waypoint spacing cannot fit the requested corner fillet")
            entry = vertex - incoming * tangent_distance
            exit_point = vertex + outgoing * tangent_distance
            left = np.array([-incoming[1], incoming[0]])
            centre = entry + math.copysign(radius, cross) * left
            start_angle = math.atan2(*(entry - centre)[::-1])
            sweep = math.copysign(turn, cross)
            arc_count = max(3, int(math.ceil(radius * turn / ds)))
            geometry.append(entry)
            geometry.extend(
                centre + radius * np.array([math.cos(start_angle + sweep * j / arc_count), math.sin(start_angle + sweep * j / arc_count)])
                for j in range(1, arc_count + 1)
            )
            corner_positions.append(centre + radius * np.array([math.cos(start_angle + sweep / 2), math.sin(start_angle + sweep / 2)]))
        geometry.append(points[-1])
        geometry = np.asarray(geometry)
        geometry_s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(geometry, axis=0), axis=1))))
        self.s = np.arange(0.0, geometry_s[-1], ds)
        self.s = np.append(self.s, geometry_s[-1])
        self.points = np.column_stack([np.interp(self.s, geometry_s, geometry[:, axis]) for axis in range(2)])
        self.length = float(self.s[-1])
        tangent = np.gradient(self.points, self.s, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        self.tangent = tangent
        limits = np.full(len(self.s), V_MAX)
        for i, corner_point in enumerate(corner_positions):
            corner_s = self.s[np.argmin(np.linalg.norm(self.points - corner_point, axis=1))]
            radius = self.corner_radii[i]
            local = np.abs(self.s - corner_s) <= radius
            limits[local] = np.minimum(limits[local], math.sqrt(a_lat * radius))
            if i == 0:
                limits[local] = np.minimum(limits[local], MAZE_FIRST_CORNER_MAX)
        # Symmetric smoothstep envelope has continuous acceleration and bounded jerk.
        ramp = np.minimum(self.s, self.length - self.s)
        x = np.clip(ramp / 0.5, 0.0, 1.0)
        self.speed = limits * (3 * x**2 - 2 * x**3)
        params = tunnel_goal.get("params") or {}
        self.tunnel_start = np.asarray(params.get("axis_start"), dtype=float)
        self.tunnel_end = np.asarray(params.get("axis_end"), dtype=float)
        if self.tunnel_start.shape != (2,) or self.tunnel_end.shape != (2,) or np.allclose(self.tunnel_start, self.tunnel_end):
            raise ValueError("tunnel goal needs a nonzero axis_start/axis_end")

    def _index(self, s):
        return int(np.clip(np.searchsorted(self.s, np.clip(s, 0, self.length)), 0, len(self.s) - 1))

    def position_at(self, s):
        s = float(np.clip(s, 0, self.length))
        return np.array([np.interp(s, self.s, self.points[:, i]) for i in range(2)])

    def velocity_at(self, s):
        i = self._index(s)
        return self.tangent[i] * self.speed[i]

    def yaw_at(self, s):
        position = self.position_at(s)
        axis = self.tunnel_end - self.tunnel_start
        projection = float((position - self.tunnel_start) @ axis / (axis @ axis))
        lateral = abs(float(np.cross(axis, position - self.tunnel_start))) / np.linalg.norm(axis)
        if 0.0 <= projection <= 1.0 and lateral <= 0.5:
            return math.atan2(axis[1], axis[0])
        look = self.position_at(min(self.length, float(s) + YAW_LOOKAHEAD)) - position
        if np.linalg.norm(look) < 1e-9:
            look = self.tangent[self._index(s)]
        return math.atan2(look[1], look[0])


def build_trajectory(waypoints, course_map, clearance_at=None, **kwargs):
    tunnels = [g for g in course_map.get("goals", []) if g.get("name") == "tunnel"]
    if len(tunnels) != 1:
        raise ValueError("map must contain exactly one tunnel goal")
    return Trajectory(waypoints, tunnels[0], clearance_at=clearance_at, **kwargs)
