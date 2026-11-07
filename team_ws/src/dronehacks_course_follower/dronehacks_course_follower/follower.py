"""Follows a planned route with a setpoint that slides along it: corners rounded into arcs, a speed
profile within PX4's own acceleration and jerk limits, the camera facing a point ahead on the
path, and the pose led so that PX4's proportional position loops fly the setpoint's own motion.

Everything is in the course frame: (east, north) metres and headings counter-clockwise from east,
in radians. A path is an (n, 2) array of samples; "along" is the distance along it from its start.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .frames import wrap_angle
from .planner import MARGIN_M, REACH_M, clearance_m

# The camera faces a point this far ahead of the setpoint. A maze corner leaves half a corridor in
# front of the drone, and facing that end wall starves cuVSLAM, so it turns into the next corridor early
LOOKAHEAD_M = 0.8

# The commanded heading's turn rate: PX4's own auto-mode limit, MPC_YAWRAUTO_MAX at its v1.16 default
TURN_RATE_DPS = 45.0

# The commanded heading closes on the look-ahead bearing with this time constant, a smooth turn
# whose rate is fed forward through the led heading
HEADING_TIME_CONSTANT_S = 0.25

# The setpoint slows as the drone uses up the setpoint's room to a wall, falls behind it along the
# path, or its heading trails the commanded one, and stops at these limits
MAX_ALONG_LAG_M = 1.0
MAX_HEADING_LAG_RAD = math.radians(30.0)

# Each corner is flown as an arc tangent to both legs, starting this far before the corner: a
# sharp corner has no speed at which a drone can follow it
CORNER_CUT_M = 0.4

# An arc passes inside its corner: it is cut tighter where it would come nearer a wall than the
# planner's lethal distance, which the planned corners keep
ROUNDED_CLEARANCE_M = REACH_M + MARGIN_M

# Spacing of the rounded path's samples
PATH_STEP_M = 0.02

# The speed profile copies PX4's own auto-mode limits at their v1.16 defaults: the acceleration
# MPC_ACC_HOR, scaled by MPC_XY_TRAJ_P in turns, and the jerk MPC_JERK_AUTO
ACCEL_M_S2 = 3.0
TURN_ACCEL_SCALE = 0.5
JERK_M_S3 = 4.0

# PX4's v1.16 default gains on the loops a pose setpoint passes through: position to velocity
# (MPC_XY_P), velocity to acceleration (MPC_XY_VEL_P_ACC) and heading to yaw rate (MC_YAW_P)
PX4_XY_P = 0.95
PX4_XY_VEL_P_ACC = 1.8
PX4_YAW_P = 2.8

# The path's velocity and curvature are read over this step ahead of the setpoint
_MOTION_STEP_M = 0.05


class EasedSpeed(NamedTuple):
    """The setpoint's speed along the path and its rate of change."""

    speed_mps: float          # along the path
    accel_m_s2: float         # its rate of change, fed forward along the tangent


class PathMotion(NamedTuple):
    """The velocity and acceleration of a point moving along a path, (east, north) each."""

    velocity: tuple[float, float]           # m/s
    acceleration: tuple[float, float]       # m/s²


class LedPose(NamedTuple):
    """A pose setpoint led ahead of its reference."""

    east: float               # metres
    north: float              # metres
    yaw_rad: float            # NaN for no heading


@dataclass(frozen=True, slots=True)
class Drone:
    """The drone's estimated pose in the course frame."""

    east: float               # metres
    north: float              # metres
    height_m: float           # above the course floor
    yaw_rad: float            # counter-clockwise from east


class Follower:
    """A setpoint sliding along the rounded route from `start`, stepped once per tick."""

    def __init__(self, start: tuple[float, float], route: list[tuple[float, float]], walls: np.ndarray,
                 yaw_rad: float, top_speed_mps: float, turn_rate_rad_s: float = math.radians(TURN_RATE_DPS),
                 lead: bool = True) -> None:
        """Rounds the route among `walls` and plans its speed profile; `yaw_rad` is the drone's heading now."""
        if top_speed_mps <= 0.0:
            raise ValueError(f"top speed must be positive, not {top_speed_mps}")

        self.path = flown_path(start, route, walls)
        self.lengths = path_lengths(self.path)
        self.speeds = speed_profile(self.path, top_speed_mps)
        self.walls = walls
        self.turn_rate_rad_s = turn_rate_rad_s
        self.lead = lead
        self.yaw_rad = yaw_rad
        self.along_m = 0.0
        self.motion = EasedSpeed(0.0, 0.0)
        self.pace = 0.0

    @property
    def finished(self) -> bool:
        """Whether the setpoint has reached the path's end."""
        return self.along_m >= float(self.lengths[-1]) - PATH_STEP_M

    @property
    def end(self) -> tuple[float, float]:
        """The path's last point."""
        return float(self.path[-1, 0]), float(self.path[-1, 1])

    def step(self, elapsed_s: float, drone: Drone | None) -> LedPose:
        """The pose to command now, then the setpoint moved on by `elapsed_s` at a pace the drone's
        estimate allows; with no estimate the setpoint waits."""
        setpoint = point_along(self.path, self.along_m)

        # The heading closes on the look-ahead point's bearing, no faster than the turn rate
        target_rad = target_heading(self.path, self.along_m, LOOKAHEAD_M)
        turn_rad, yaw_rate = 0.0, 0.0
        if target_rad is not None:
            turn_rad = wrap_angle(target_rad - self.yaw_rad)
            yaw_rate = min(max(turn_rad / HEADING_TIME_CONSTANT_S, -self.turn_rate_rad_s), self.turn_rate_rad_s)
            self.yaw_rad = wrap_angle(self.yaw_rad + yaw_rate * elapsed_s)

        # The setpoint's own motion, led into the pose so PX4's loops command it
        motion = path_motion(self.path, self.along_m, self.motion.speed_mps, self.motion.accel_m_s2)
        command = LedPose(float(setpoint[0]), float(setpoint[1]), self.yaw_rad)
        if self.lead:
            command = led_pose(command, motion, yaw_rate)

        # The pace the drone's estimate allows, eased within PX4's acceleration and jerk
        tangent = point_along(self.path, self.along_m + _MOTION_STEP_M) - setpoint
        self.pace = self._pace(setpoint, tangent, drone, turn_rad)
        self.motion = paced_speed(self.lengths, self.speeds, self.along_m, self.pace, self.motion, elapsed_s)
        if self.motion.speed_mps > 0.0:
            self.along_m = min(self.along_m + self.motion.speed_mps * elapsed_s, float(self.lengths[-1]))

        return command

    def reference(self) -> tuple[float, float]:
        """The un-led setpoint on the path now, (east, north)."""
        point = point_along(self.path, self.along_m)
        return float(point[0]), float(point[1])

    def _pace(self, setpoint: np.ndarray, tangent: np.ndarray, drone: Drone | None, turn_rad: float) -> float:
        """The share of the profile's speed allowed: cut as the drone strays nearer a wall than the
        path, trails the setpoint, or its heading trails the commanded one, or that trails its target."""
        if drone is None:
            return 0.0

        position = np.array([drone.east, drone.north])
        return (wall_slack(setpoint, position, self.walls) * _slack(behind_m(position, setpoint, tangent), MAX_ALONG_LAG_M)
                * _slack(abs(wrap_angle(drone.yaw_rad - self.yaw_rad)), MAX_HEADING_LAG_RAD)
                * heading_slack(turn_rad, self.turn_rate_rad_s))


def led_pose(pose: LedPose, motion: PathMotion, yaw_rate: float) -> LedPose:
    """A pose setpoint led so that PX4's proportional loops command a reference's motion.

    PX4 flies a pose as velocity = `PX4_XY_P` x (pose - position) and acceleration =
    `PX4_XY_VEL_P_ACC` x (velocity setpoint - velocity), with no feed-forward through the flight
    interface. A pose led by v / P + a / (P x P_acc) makes those terms equal the reference's own
    velocity and acceleration, and a heading led by yaw rate / `PX4_YAW_P` its turn rate. Exact
    while no loop saturates.
    """
    (east_mps, north_mps), (east_m_s2, north_m_s2) = motion
    east = pose.east + east_mps / PX4_XY_P + east_m_s2 / (PX4_XY_P * PX4_XY_VEL_P_ACC)
    north = pose.north + north_mps / PX4_XY_P + north_m_s2 / (PX4_XY_P * PX4_XY_VEL_P_ACC)
    yaw_rad = pose.yaw_rad if math.isnan(pose.yaw_rad) else wrap_angle(pose.yaw_rad + yaw_rate / PX4_YAW_P)
    return LedPose(east, north, yaw_rad)


def behind_m(drone: np.ndarray, setpoint: np.ndarray, tangent: np.ndarray) -> float:
    """How far the drone trails a moving setpoint along its direction of travel `tangent`;
    infinite where the path has no direction."""
    length = float(np.hypot(*tangent))
    if length < 1e-9:
        return math.inf

    return max(0.0, -float((drone - setpoint) @ (tangent / length)))


def wall_slack(setpoint: np.ndarray, drone: np.ndarray, walls: np.ndarray) -> float:
    """The share of the setpoint's room to a wall, past the drone's reach, that the drone has not
    used up by straying nearer one: 1 with no walls."""
    if len(walls) == 0:
        return 1.0

    path_room_m, drone_room_m = clearance_m(np.array([setpoint, drone]), walls) - REACH_M
    if path_room_m <= 0.0:
        return 0.0

    return _slack(max(path_room_m - drone_room_m, 0.0), path_room_m)


def _slack(lag: float, limit: float) -> float:
    """How much of the full pace a lag leaves: 1 with none, falling to 0 at the limit."""
    return min(max(1.0 - lag / limit, 0.0), 1.0)


def heading_slack(turn_rad: float, turn_rate_rad_s: float) -> float:
    """How much of the full pace the commanded heading's lag `turn_rad` behind its target leaves:
    the lag the time constant gives at full turn rate is free; past it the pace falls to 0 over
    `MAX_HEADING_LAG_RAD`, so the setpoint waits for the camera."""
    free_rad = HEADING_TIME_CONSTANT_S * turn_rate_rad_s
    return _slack(max(0.0, abs(turn_rad) - free_rad), MAX_HEADING_LAG_RAD)


def flown_path(start: tuple[float, float], route: list[tuple[float, float]], walls: np.ndarray) -> np.ndarray:
    """The path flown from `start` through the route: its corners rounded, each kept
    `ROUNDED_CLEARANCE_M` from the walls (segment rows) where it can be."""
    return rounded_route(np.array([start, *route], dtype=float), CORNER_CUT_M, walls=walls if len(walls) else None,
                         lethal_m=ROUNDED_CLEARANCE_M)


def path_lengths(path: np.ndarray) -> np.ndarray:
    """The distance along the path to each of its samples."""
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])


def rounded_route(points: np.ndarray, cut_m: float, step_m: float = PATH_STEP_M, walls: np.ndarray | None = None,
                  lethal_m: float = 0.0) -> np.ndarray:
    """The route sampled every `step_m`, each corner replaced by an arc tangent to both legs.

    An arc starts `cut_m` before its corner, or halfway along a shorter leg. For a turn of theta
    it has radius cut * cot(theta / 2), the geometry of PX4's `computeMaxSpeedInWaypoint`. With
    `walls` as (x1, y1, x2, y2) rows, a corner whose arc would come nearer them than `lethal_m`,
    and nearer than its sharp corner does, is cut by half again until it does not.
    """
    samples = [points[0]]
    start = points[0]
    for corner, after in zip(points[1:-1], points[2:], strict=True):
        incoming, outgoing = corner - start, after - corner
        if np.linalg.norm(incoming) < 1e-9 or np.linalg.norm(outgoing) < 1e-9:
            continue

        into, out_of = incoming / np.linalg.norm(incoming), outgoing / np.linalg.norm(outgoing)
        turn_rad = math.acos(min(max(float(into @ out_of), -1.0), 1.0))
        cut = min(cut_m, float(np.linalg.norm(incoming)) / 2, float(np.linalg.norm(outgoing)) / 2)
        if turn_rad < 1e-3:
            continue

        # The arc inside the corner passes nearest whatever stands there: a tighter one until it clears
        arc = _corner_arc(corner, into, out_of, turn_rad, cut, step_m)
        while walls is not None and cut > step_m and not _arc_clear(arc, corner, walls, lethal_m, step_m):
            cut /= 2
            arc = _corner_arc(corner, into, out_of, turn_rad, cut, step_m)

        # Straight to where the arc starts, then round the corner on the arc
        samples += _segment(samples[-1], arc[0], step_m)
        samples += arc[1:]
        start = corner

    samples += _segment(samples[-1], points[-1], step_m)
    return np.array(samples)


def _corner_arc(corner: np.ndarray, into: np.ndarray, out_of: np.ndarray, turn_rad: float, cut_m: float,
                step_m: float) -> list[np.ndarray]:
    """The arc rounding a corner `cut_m` before and after it: its start, then its samples."""
    arc_start = corner - cut_m * into
    return [arc_start, *_arc(arc_start, into, out_of, cut_m / math.tan(turn_rad / 2), turn_rad, step_m)]


def _arc_clear(arc: list[np.ndarray], corner: np.ndarray, walls: np.ndarray, lethal_m: float, step_m: float) -> bool:
    """Whether an arc keeps `lethal_m` from the walls, or as far as the sharp corner it replaces."""
    sharp = [arc[0], *_segment(arc[0], corner, step_m), *_segment(corner, arc[-1], step_m)]
    needed_m = min(lethal_m, float(clearance_m(np.array(sharp), walls).min()))
    return float(clearance_m(np.array(arc), walls).min()) >= needed_m


def _segment(start: np.ndarray, end: np.ndarray, step_m: float) -> list[np.ndarray]:
    """Points from just past `start` to `end`, about `step_m` apart."""
    count = max(1, math.ceil(float(np.linalg.norm(end - start)) / step_m))
    return [start + (end - start) * index / count for index in range(1, count + 1)]


def _arc(start: np.ndarray, into: np.ndarray, out_of: np.ndarray, radius_m: float, turn_rad: float,
         step_m: float) -> list[np.ndarray]:
    """Points along a circular arc from `start`, tangent to `into` there, turning by `turn_rad`."""
    side = 1.0 if into[0] * out_of[1] - into[1] * out_of[0] > 0 else -1.0
    centre = start + radius_m * side * np.array([-into[1], into[0]])
    first = math.atan2(*(start - centre)[::-1])
    count = max(1, math.ceil(radius_m * turn_rad / step_m))
    angles = [first + side * turn_rad * index / count for index in range(1, count + 1)]
    return [centre + radius_m * np.array([math.cos(angle), math.sin(angle)]) for angle in angles]


def speed_profile(path: np.ndarray, top_speed_mps: float) -> np.ndarray:
    """The fastest speed at each sample of a path: PX4's turn limit on its curvature, at rest at
    both ends, and no faster than it can brake to every later limit or have gained since every
    earlier one."""
    return _reachable_speeds(path_lengths(path), _turn_limits(path, top_speed_mps), top_speed_mps)


def _turn_limits(path: np.ndarray, top_speed_mps: float) -> np.ndarray:
    """Each sample's own speed limit: lateral acceleration v^2 * curvature within PX4's scaled
    acceleration, and at rest at both ends."""
    limits = np.full(len(path), top_speed_mps)
    for index in range(1, len(path) - 1):
        curvature = _curvature(path[index - 1], path[index], path[index + 1])
        if curvature > 1e-6:
            limits[index] = min(top_speed_mps, math.sqrt(ACCEL_M_S2 * TURN_ACCEL_SCALE / curvature))

    limits[0] = limits[-1] = 0.0
    return limits


def _reachable_speeds(along_m: np.ndarray, limits: np.ndarray, top_speed_mps: float) -> np.ndarray:
    """The fastest speeds within `limits` that can still brake to every later limit and could have
    gained since every earlier one.

    PX4's jerk-limited braking assumes it starts from zero acceleration, so it holds only over the
    whole distance to a limit, never chained from sample to sample. Gaining speed uses the plain
    acceleration limit: near zero distance the braking formula is linear in distance, and a
    setpoint started from rest on it creeps away exponentially, slower than any real drone.
    """
    speeds = limits.copy()
    for index in np.flatnonzero(limits < top_speed_mps):
        before, after = slice(0, index), slice(index + 1, len(limits))
        speeds[before] = np.minimum(speeds[before], braking_speed(along_m[index] - along_m[before], limits[index]))
        gained = np.sqrt(limits[index] ** 2 + 2.0 * ACCEL_M_S2 * (along_m[after] - along_m[index]))
        speeds[after] = np.minimum(speeds[after], gained)

    return speeds


def braking_speed(distance_m: np.ndarray, final_speed_mps: float) -> np.ndarray:
    """The fastest speeds that can still brake to `final_speed_mps` within each distance, under the
    acceleration and jerk limits: PX4's `computeMaxSpeedFromDistance`."""
    b = 4.0 * ACCEL_M_S2 ** 2 / JERK_M_S3
    c = -2.0 * ACCEL_M_S2 * np.asarray(distance_m) - final_speed_mps ** 2
    return np.maximum(0.5 * (-b + np.sqrt(b * b - 4.0 * c)), final_speed_mps)


def _curvature(before: np.ndarray, at: np.ndarray, after: np.ndarray) -> float:
    """The curvature of the circle through three points: 4 x area / product of the sides."""
    sides = np.linalg.norm(at - before) * np.linalg.norm(after - at) * np.linalg.norm(after - before)
    cross = (at - before)[0] * (after - before)[1] - (at - before)[1] * (after - before)[0]
    return float(2.0 * abs(cross) / sides) if sides > 1e-12 else 0.0


def eased_speed(speed_mps: float, accel_m_s2: float, target_mps: float, elapsed_s: float) -> EasedSpeed:
    """The speed one step on towards `target_mps`, within `ACCEL_M_S2` and `JERK_M_S3`.

    The acceleration aims at what, ramped to zero at the jerk limit, closes the error just as it
    ends: sqrt(2 J |error|), less half a step's jerk so the discrete ramp does not start late. It
    moves towards that by at most the jerk limit. A step that would pass the target lands on it
    instead, and a speed on its target holds there.
    """
    error_mps = target_mps - speed_mps
    if abs(error_mps) < 1e-9:
        return EasedSpeed(target_mps, 0.0)

    if elapsed_s <= 0.0:
        return EasedSpeed(speed_mps, accel_m_s2)

    # The jerk-limited approach, then no further than the target
    half_step_m_s2 = JERK_M_S3 * elapsed_s / 2
    ramp_m_s2 = math.sqrt(2.0 * JERK_M_S3 * abs(error_mps) + half_step_m_s2 ** 2) - half_step_m_s2
    wanted = math.copysign(min(ACCEL_M_S2, ramp_m_s2), error_mps)
    accel = min(max(wanted, accel_m_s2 - JERK_M_S3 * elapsed_s), accel_m_s2 + JERK_M_S3 * elapsed_s)
    if (speed_mps + accel * elapsed_s - target_mps) * error_mps >= 0.0:
        return EasedSpeed(target_mps, error_mps / elapsed_s)

    return EasedSpeed(speed_mps + accel * elapsed_s, accel)


def paced_speed(lengths: np.ndarray, speeds: np.ndarray, along_m: float, pace: float, current: EasedSpeed,
                elapsed_s: float) -> EasedSpeed:
    """The setpoint's next speed: eased from `current` towards the profile's speed times `pace`,
    and never over the profile, whose own braking into a corner then sets the acceleration."""
    profile_mps = float(np.interp(along_m, lengths, speeds))

    # From rest the profile's first sample is 0: aim at the speed one step in allows
    if along_m < lengths[1]:
        profile_mps = max(profile_mps, float(speeds[1]))

    eased = eased_speed(current.speed_mps, current.accel_m_s2, profile_mps * pace, elapsed_s)
    if eased.speed_mps <= profile_mps:
        return eased

    # Riding the profile, its slope gives the change of speed: v dv/ds
    slope = (float(np.interp(along_m + _MOTION_STEP_M, lengths, speeds)) - profile_mps) / _MOTION_STEP_M
    return EasedSpeed(profile_mps, profile_mps * slope)


def path_motion(path: np.ndarray, along_m: float, speed_mps: float, accel_m_s2: float) -> PathMotion:
    """The velocity and acceleration of a point moving along the path at `speed_mps`, its speed
    changing by `accel_m_s2`: the turn's v² dt/ds plus the change of speed along the tangent, what
    PX4 needs to hold a curve, and to slow on it, without first building a position error."""
    here = point_along(path, along_m)
    ahead = point_along(path, along_m + _MOTION_STEP_M)
    further = point_along(path, along_m + 2 * _MOTION_STEP_M)
    if np.hypot(*(ahead - here)) < 0.01:
        return PathMotion((0.0, 0.0), (0.0, 0.0))

    tangent = (ahead - here) / np.hypot(*(ahead - here))
    next_tangent = tangent
    if np.hypot(*(further - ahead)) > 0.01:
        next_tangent = (further - ahead) / np.hypot(*(further - ahead))

    acceleration = speed_mps ** 2 * (next_tangent - tangent) / _MOTION_STEP_M + accel_m_s2 * tangent
    velocity = tangent * speed_mps
    return PathMotion((float(velocity[0]), float(velocity[1])), (float(acceleration[0]), float(acceleration[1])))


def point_along(path: np.ndarray, distance_m: float) -> np.ndarray:
    """The point `distance_m` along a polyline of (n, 2) points, clamped to its two ends."""
    lengths = path_lengths(path)
    distance_m = min(max(distance_m, 0.0), float(lengths[-1]))
    leg = min(int(np.searchsorted(lengths, distance_m, side="right")) - 1, len(path) - 2)
    leg_m = lengths[leg + 1] - lengths[leg]
    fraction = (distance_m - lengths[leg]) / leg_m if leg_m > 0 else 0.0
    return path[leg] + fraction * (path[leg + 1] - path[leg])


def target_heading(path: np.ndarray, along_m: float, lookahead_m: float) -> float | None:
    """The bearing from the point `along_m` on the path to the point `lookahead_m` further on;
    None at the path's end, where the two meet."""
    ahead = point_along(path, along_m + lookahead_m) - point_along(path, along_m)
    return math.atan2(ahead[1], ahead[0]) if np.hypot(*ahead) > 0.01 else None
