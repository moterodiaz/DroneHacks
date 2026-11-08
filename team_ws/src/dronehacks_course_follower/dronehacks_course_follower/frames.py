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


def frames_disagree(odom_enu: tuple[float, float, float], px4_ned: tuple[float, float, float],
                    limit_m: float) -> str | None:
    """Why PX4's local position and the tracker's odometry disagree by more than `limit_m`, or None.

    The kit's interface hands PX4 an odom ENU setpoint as NED (x and y swapped, z negated), which is
    only right while both frames share an origin. PX4 takes x, y and yaw from vision but its height
    from the rangefinder, with its origin where the flight controller powered up; odom's origin is
    where the stack started. A drone powered up off its take-off spot flies every setpoint shifted."""
    north_m, east_m, down_m = px4_ned
    x_m, y_m, z_m = odom_enu
    horizontal_m = math.hypot(east_m - x_m, north_m - y_m)
    if horizontal_m > limit_m:
        return f"PX4 position and cuVSLAM disagree by {horizontal_m:.2f} m"

    vertical_m = abs(-down_m - z_m)
    if vertical_m > limit_m:
        return f"PX4 height and cuVSLAM disagree by {vertical_m:.2f} m"

    return None


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
