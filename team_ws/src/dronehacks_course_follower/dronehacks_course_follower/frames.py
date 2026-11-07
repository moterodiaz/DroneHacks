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
from dataclasses import dataclass


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
