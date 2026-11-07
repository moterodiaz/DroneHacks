"""Course frame <-> the ``odom`` frame the kit's flight interface takes.

``contracts/frames.md`` (PR #5), "Origin": the tracker's frame starts at the
drone's body resting on the launch mark, facing up the first corridor. The
organisers' cuVSLAM ``odom`` frame is exactly that (competitor_guide.md §1):
ENU-labelled, but "east" is whatever the vehicle's x axis pointed at when the
tracker started, and the origin is the start pose. So:

* odom +x = the launch heading (the map's ``launch.heading``: north on the published course);
* odom +y = 90 degrees to its left;
* odom +z = up, from the body resting on the floor.

With the drone facing north this is the contract's transform: subtract the
launch mark, then ``x = north``, ``y = -east`` (FLU, the ROS side of the
contract's FRD step). A heading measured counter-clockwise from east becomes
an odom yaw ``heading - launch_heading``. Setpoint height is
``flight height - body resting height`` (frames.md step 3).

No coordinate is written here: the launch mark and heading come from the map
file (AGENTS.md §3.7 on the contracts branch).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Sequence, Tuple

Vec = Tuple[float, float]


@dataclasses.dataclass(frozen=True)
class OdomFrame:
    """The odom frame, as placed on the course: its origin and x direction in course coordinates."""

    east: float
    north: float
    heading: float  # course heading of odom +x, radians counter-clockwise from east

    @classmethod
    def from_launch(cls, position: Sequence[float], heading_vec: Sequence[float]) -> "OdomFrame":
        hx, hy = float(heading_vec[0]), float(heading_vec[1])
        if math.hypot(hx, hy) < 1e-9:
            raise ValueError("launch heading has zero length")
        return cls(float(position[0]), float(position[1]), math.atan2(hy, hx))

    # -- positions and directions --------------------------------------------------

    def to_odom(self, east: float, north: float) -> Vec:
        de, dn = east - self.east, north - self.north
        return self.rotate_to_odom(de, dn)

    def to_course(self, x: float, y: float) -> Vec:
        de, dn = self.rotate_to_course(x, y)
        return self.east + de, self.north + dn

    def rotate_to_odom(self, ve: float, vn: float) -> Vec:
        c, s = math.cos(self.heading), math.sin(self.heading)
        return ve * c + vn * s, -ve * s + vn * c

    def rotate_to_course(self, vx: float, vy: float) -> Vec:
        c, s = math.cos(self.heading), math.sin(self.heading)
        return vx * c - vy * s, vx * s + vy * c

    # -- headings ------------------------------------------------------------------------

    def yaw_to_odom(self, course_heading: float) -> float:
        return wrap(course_heading - self.heading)

    def yaw_to_course(self, odom_yaw: float) -> float:
        return wrap(odom_yaw + self.heading)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (about +z) of an xyzw quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """xyzw quaternion of a pure yaw."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def body_to_world(vx: float, vy: float, yaw: float) -> Vec:
    """A body-frame (FLU) horizontal velocity, as cuVSLAM's odometry twist gives it,
    in the odom frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return vx * c - vy * s, vx * s + vy * c


def ned_yaw_to_enu(yaw_ned: float) -> float:
    """PX4's heading (NED, from north, clockwise) as an ENU yaw (competitor_guide.md §1: its own inverse)."""
    return wrap(math.pi / 2.0 - yaw_ned)


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
