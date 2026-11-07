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
TIME_STEP = 1e-3  # numerical integration step for the opt-in time profile; not a flight rate
MAX_PROFILE_STEPS = 2_000_000  # integration guard: a stalled loop fails closed instead of hanging


def _cross2(a, b) -> float:
    """Portable 2-D scalar cross product (NumPy 2 removed 2-D ``np.cross``)."""
    return float(a[0] * b[1] - a[1] * b[0])


class Trajectory:
    """Continuous piecewise-linear geometry with an acceleration-bounded speed profile."""

    def __init__(self, waypoints, tunnel_goal, clearance_at=None, a_lat=1.5, ds=0.02):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            raise ValueError("waypoints must be an Nx2 array with N >= 2")
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(lengths == 0):
            raise ValueError("consecutive waypoints must differ")
        # Circular fillets provide continuous position and tangent at each corner that
        # actually turns. Straight (collinear) interior waypoints are passed through
        # untouched: no clearance gate, no fillet, no corner speed limit.
        radii = []
        geometry = [points[0]]
        corner_positions = []
        for i, vertex in enumerate(points[1:-1], start=1):
            incoming = (vertex - points[i - 1]) / lengths[i - 1]
            outgoing = (points[i + 1] - vertex) / lengths[i]
            turn = math.acos(float(np.clip(incoming @ outgoing, -1.0, 1.0)))
            cross = _cross2(incoming, outgoing)
            if turn < 1e-5 or abs(cross) < 1e-8:
                geometry.append(vertex)
                continue
            available = R_MAX if clearance_at is None else max(0.0, float(clearance_at(vertex)) - 0.32)
            if available < R_MIN:
                raise ValueError("local clearance cannot fit the minimum 0.20 m corner fillet")
            # Clamp the radius down so the fillet's tangent points stay on both
            # neighbouring legs; only genuinely infeasible geometry is rejected.
            spacing = min(lengths[i - 1], lengths[i]) / 2.0 * (1.0 - 1e-9)
            fitting = spacing / math.tan(turn / 2.0) if turn < math.pi - 1e-9 else 0.0
            radius = min(R_MAX, available, fitting)
            if radius < R_MIN:
                raise ValueError("waypoint spacing cannot fit any fillet radius in the allowed 0.20-0.35 m band")
            radii.append(radius)
            tangent_distance = radius * math.tan(turn / 2.0)
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
        self.corner_radii = np.asarray(radii)
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
        self._speed_ceiling = limits  # raw caps before end ramps and smoothing passes
        # Symmetric smoothstep envelope has continuous acceleration and bounded jerk.
        ramp = np.minimum(self.s, self.length - self.s)
        x = np.clip(ramp / 0.5, 0.0, 1.0)
        speed = limits * (3 * x**2 - 2 * x**3)
        # Forward/backward passes bound the implied acceleration by a_lat and remove
        # the step-function jumps the raw corner bands had at their edges.
        for i in range(len(speed) - 1):
            step = float(self.s[i + 1] - self.s[i])
            speed[i + 1] = min(speed[i + 1], math.sqrt(speed[i] ** 2 + 2.0 * a_lat * step))
        for i in range(len(speed) - 2, -1, -1):
            step = float(self.s[i + 1] - self.s[i])
            speed[i] = min(speed[i], math.sqrt(speed[i + 1] ** 2 + 2.0 * a_lat * step))
        self.speed = speed
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

    def build_time_profile(self, a_lon, jerk_max):
        """Return ``(t, s, v)`` arrays of a time-parameterized profile from rest.

        OPT-IN API for callers that integrate in time instead of arc length.
        Both the longitudinal acceleration bound ``a_lon`` and the jerk bound
        ``jerk_max`` are required caller-supplied arguments; this module invents
        no longitudinal defaults for them. The ceiling reuses the existing
        bounds only (1.0 m/s cap, corner speeds derived from ``a_lat``, the
        0.75 m/s first-corner cap) and the profile starts and ends at rest.
        """
        a_lon = float(a_lon)
        jerk_max = float(jerk_max)
        if not math.isfinite(a_lon) or a_lon <= 0.0:
            raise ValueError("a_lon must be a positive finite longitudinal acceleration bound")
        if not math.isfinite(jerk_max) or jerk_max <= 0.0:
            raise ValueError("jerk_max must be a positive finite jerk bound")
        ceiling = self._speed_ceiling.copy()
        ceiling[-1] = 0.0  # end at rest
        for i in range(len(ceiling) - 2, -1, -1):
            step = float(self.s[i + 1] - self.s[i])
            ceiling[i] = min(ceiling[i], math.sqrt(ceiling[i + 1] ** 2 + 2.0 * a_lon * step))
        # Jerk-aware ceiling: the max speed at each sample from which a
        # jerk-limited brake can still meet every future cap. Braking from v
        # with decel ramping at jerk_max covers v^2/(2 a_lon) plus a ramp
        # excess of about a_lon^2/jerk_max * v, so the feasible speed at
        # distance d ahead of cap c solves v^2 + (a_lon^2/jerk_max) v <=
        # c^2 + 2 a_lon d.
        ramp_term = a_lon ** 2 / jerk_max
        grid = float(self.s[1] - self.s[0])
        brake_range = V_MAX ** 2 / (2.0 * a_lon) + V_MAX * ramp_term / 2.0 + 2.0 * grid
        jerk_ceiling = ceiling.copy()
        for i in range(len(ceiling) - 2, -1, -1):
            limit = ceiling[i]
            for j in range(i + 1, len(ceiling)):
                distance = float(self.s[j] - self.s[i])
                if distance > brake_range:
                    break
                allowed = 0.5 * (-ramp_term + math.sqrt(ramp_term ** 2 + 4.0 * (ceiling[j] ** 2 + 2.0 * a_lon * distance)))
                if allowed < limit:
                    limit = allowed
            jerk_ceiling[i] = limit
        times = [0.0]
        positions = [0.0]
        speeds = [0.0]
        s_cur, v_cur, a_cur = 0.0, 0.0, 0.0
        # Terminal-decay gain: bounding a from below by -(jerk_max/a_lon)*v
        # caps the low-speed decel smoothly. The floor equals -a_lon exactly at
        # v = a_lon**2/jerk_max, so it binds without a jump, and the implied
        # jerk K^2*v peaks at K*a_lon = jerk_max there.
        decay_gain = min(jerk_max / a_lon, 0.5 / TIME_STEP)  # keep K*TIME_STEP < 1 so the floor never overshoots zero
        while s_cur < self.length - 1e-6:
            if len(times) > MAX_PROFILE_STEPS:
                raise ValueError("time profile integration failed to converge")
            # Track the ceiling several jerk-ramp distances ahead so the decel
            # ramp (worst case from +a_lon to -a_lon) completes before the
            # ceiling declines instead of overshooting it.
            offset = 3.0 * v_cur * a_lon / jerk_max + 0.5 * a_lon * TIME_STEP ** 2
            v_target = float(np.interp(min(self.length, s_cur + offset), self.s, jerk_ceiling))
            a_target = min(a_lon, max(-a_lon, (v_target - v_cur) / (5.0 * TIME_STEP)))
            a_cur += min(jerk_max * TIME_STEP, max(-jerk_max * TIME_STEP, a_target - a_cur))
            a_cur = max(a_cur, -decay_gain * v_cur)
            v_next = v_cur + a_cur * TIME_STEP
            if v_next < 0.0:
                v_next = 0.0  # float-precision guard only
            s_next = s_cur + 0.5 * (v_cur + v_next) * TIME_STEP
            s_cur, v_cur = s_next, v_next
            times.append(times[-1] + TIME_STEP)
            positions.append(min(s_cur, self.length))
            speeds.append(v_cur)
        return np.asarray(times), np.asarray(positions), np.asarray(speeds)

    def yaw_at(self, s):
        position = self.position_at(s)
        axis = self.tunnel_end - self.tunnel_start
        projection = float((position - self.tunnel_start) @ axis / (axis @ axis))
        lateral = abs(_cross2(axis, position - self.tunnel_start)) / np.linalg.norm(axis)
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
