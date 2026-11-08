"""The course frame and cuVSLAM's odom frame, and headings as the flight interface reads them.

Both frames are ENU (x east, y north, z up, headings counter-clockwise from +x), but odom's origin
is wherever the tracker started and its axes are whatever the rig faced then. contracts/frames.md
defines the measured rigid transform between them,

    p_odom = o_odom + R(yaw) (p_course - o_course)

with `o_course` the course map's launch mark and `o_odom` its measured odom reading, the same
parameters the team's manual navigator takes (`course_to_odom_yaw/x/y`). Heights are the course's
above the floor; odom's z is above the odom origin, which sits `origin_height_m` above the floor.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# How long the drone must rest on the launch mark before arming for its odometry to place the course
REST_WINDOW_S = 2.0

# Spread in that window beyond which the drone was moving, not resting: a hand placing it, a slide
REST_SPREAD_M = 0.02
REST_SPREAD_RAD = math.radians(1.0)

# Fewer odometry samples than this in the window means the tracker is not running at its rate
REST_MIN_SAMPLES = 10


# PX4's offset from the odometry at rest beyond which a flight is refused: more than the
# rangefinder's error on the floor, so the drone was powered up somewhere else
OFFSET_LIMIT_M = 0.3

# How fast the offset follows PX4's frame in flight, as the rangefinder comes into range
OFFSET_TAU_S = 1.0

# How fast the height offset follows PX4's local position paired with the odometry: EKF2 moves its
# height over a rangefinder step in 0.2-0.8 s, and the tracker's latency and the interface's 0.1 s
# resend already lag it, so a shorter constant gains little and only passes on more noise
OFFSET_HEIGHT_TAU_S = 0.1


class NotAtRest(RuntimeError):
    """The odometry does not show the drone resting still on the launch mark."""


@dataclass(frozen=True, slots=True, kw_only=True)
class CourseToOdom:
    """The measured course-to-odom transform."""

    yaw_rad: float                          # turn from course axes to odom axes, about +z
    odom_origin: tuple[float, float]        # odom (x, y) of the launch reference
    course_origin: tuple[float, float]      # course (east, north) of the launch reference
    origin_height_m: float                  # height of the odom origin above the course floor

    def __post_init__(self) -> None:
        """Raises `ValueError` for any value not measured: a NaN or infinite one."""
        values = (self.yaw_rad, *self.odom_origin, *self.course_origin, self.origin_height_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"the course-to-odom transform must be measured, not {values}; see contracts/frames.md")

    def point_to_odom(self, east: float, north: float) -> tuple[float, float]:
        """A course point as odom (x, y)."""
        x, y = self.vector_to_odom(east - self.course_origin[0], north - self.course_origin[1])
        return self.odom_origin[0] + x, self.odom_origin[1] + y

    def point_to_course(self, x: float, y: float) -> tuple[float, float]:
        """An odom point as course (east, north)."""
        east, north = self.vector_to_course(x - self.odom_origin[0], y - self.odom_origin[1])
        return self.course_origin[0] + east, self.course_origin[1] + north

    def vector_to_odom(self, east: float, north: float) -> tuple[float, float]:
        """A course direction or velocity turned into odom axes."""
        cos, sin = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return cos * east - sin * north, sin * east + cos * north

    def vector_to_course(self, x: float, y: float) -> tuple[float, float]:
        """An odom direction or velocity turned into course axes."""
        cos, sin = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return cos * x + sin * y, -sin * x + cos * y

    def yaw_to_odom(self, yaw_rad: float) -> float:
        """A course heading as an odom heading; NaN, for no heading, stays NaN."""
        return wrap_angle(yaw_rad + self.yaw_rad)

    def yaw_to_course(self, yaw_rad: float) -> float:
        """An odom heading as a course heading."""
        return wrap_angle(yaw_rad - self.yaw_rad)

    def height_to_odom(self, height_m: float) -> float:
        """A height above the course floor as odom z."""
        return height_m - self.origin_height_m

    def height_to_course(self, z_m: float) -> float:
        """Odom z as a height above the course floor."""
        return z_m + self.origin_height_m


class RestOnMark:
    """The course-to-odom transform read from the odometry of a drone resting on the launch mark,
    facing the launch heading: the mark is where it sits, the heading where its nose points. It is
    re-read at every tracking start, since the odom origin is wherever the tracker started."""

    def __init__(self, launch: tuple[float, float], launch_heading_rad: float, origin_height_m: float) -> None:
        """Keeps the mark, its heading (counter-clockwise from east) and the odom origin's height."""
        self.launch = launch
        self.launch_heading_rad = launch_heading_rad
        self.origin_height_m = origin_height_m
        self.samples: deque[tuple[float, float, float, float]] = deque()

    def add(self, time_s: float, x: float, y: float, yaw_rad: float) -> None:
        """Adds one odometry reading, forgetting those older than the rest window."""
        self.samples.append((time_s, x, y, yaw_rad))
        while self.samples and time_s - self.samples[0][0] > REST_WINDOW_S:
            self.samples.popleft()

    def transform(self) -> CourseToOdom:
        """The transform from the window's mean pose; raises `NotAtRest` if it is short or moving."""
        if len(self.samples) < REST_MIN_SAMPLES:
            raise NotAtRest(f"only {len(self.samples)} odometry samples in {REST_WINDOW_S:.0f} s "
                            f"(need {REST_MIN_SAMPLES}): is the tracker running?")

        _, xs, ys, yaws = zip(*self.samples, strict=True)
        spread_m = max(max(xs) - min(xs), max(ys) - min(ys))
        if spread_m > REST_SPREAD_M:
            raise NotAtRest(f"the drone moved {spread_m * 100:.1f} cm in the last {REST_WINDOW_S:.0f} s")

        # The circular mean, so headings either side of the wrap average correctly
        yaw_rad = math.atan2(sum(math.sin(yaw) for yaw in yaws), sum(math.cos(yaw) for yaw in yaws))
        turn_rad = max(abs(wrap_angle(yaw - yaw_rad)) for yaw in yaws)
        if turn_rad > REST_SPREAD_RAD:
            raise NotAtRest(f"the drone turned {math.degrees(turn_rad):.1f} deg in the last {REST_WINDOW_S:.0f} s")

        return CourseToOdom(yaw_rad=wrap_angle(yaw_rad - self.launch_heading_rad),
                            odom_origin=(sum(xs) / len(xs), sum(ys) / len(ys)), course_origin=self.launch,
                            origin_height_m=self.origin_height_m)


class Px4Offset:
    """PX4's local position minus the odometry, both ENU, which every pose setpoint carries.

    The kit's interface hands PX4 an odom setpoint unanchored, so PX4 flies it in its own frame.
    PX4 takes x, y and yaw from vision but its height from the rangefinder, with its origin where
    the flight controller powered up, and the rangefinder reads wrong below its range on the floor
    and true in the air. So the offset is measured at rest, refused when too large to be the floor's
    error, and followed, smoothed, through the flight. Both inputs measure the same drone, so
    following it moves no setpoint toward the drone: it is not a control loop. The height part is
    PX4's error over whatever the rangefinder sees, so with PX4's local position paired to the
    odometry it is followed fast, at `OFFSET_HEIGHT_TAU_S`, and the drone holds the tracker's height."""

    def __init__(self) -> None:
        """Starts with no offset."""
        self.enu = (0.0, 0.0, 0.0)

    def measure(self, px4_enu: tuple[float, float, float], odom_enu: tuple[float, float, float]) -> None:
        """Takes the offset as it is now: at rest, before Offboard."""
        self.enu = offset_between(px4_enu, odom_enu)

    def follow(self, target_enu: tuple[float, float, float], elapsed_s: float,
               taus_s: tuple[float, float, float] = (OFFSET_TAU_S,) * 3) -> None:
        """Moves the offset toward `target_enu`, its value now, each axis at its time constant in
        `taus_s`; an infinite one holds that axis."""
        self.enu = tuple(offset + min(1.0, elapsed_s / tau_s) * (target - offset)
                         for offset, target, tau_s in zip(self.enu, target_enu, taus_s, strict=True))

    def too_far(self, limit_m: float) -> str | None:
        """Why the offset is too large to fly on, or None."""
        size_m = math.hypot(*self.enu)
        return f"PX4 and cuVSLAM disagree by {size_m:.2f} m" if size_m > limit_m else None


def offset_between(px4_enu: tuple[float, float, float],
                   odom_enu: tuple[float, float, float]) -> tuple[float, float, float]:
    """PX4's position minus the odometry's, ENU."""
    return tuple(px4 - odom for px4, odom in zip(px4_enu, odom_enu, strict=True))


def wrap_angle(angle_rad: float) -> float:
    """The angle brought into [-pi, pi); NaN stays NaN."""
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """The level orientation facing `yaw_rad` as (x, y, z, w); for NaN, the all-zero quaternion the
    flight interface reads as "heading not commanded"."""
    if math.isnan(yaw_rad):
        return 0.0, 0.0, 0.0, 0.0

    return 0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """The heading of an orientation (x, y, z, w): its rotation about +z, counter-clockwise from +x."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
