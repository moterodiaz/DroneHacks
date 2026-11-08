"""Course frame <-> the ``odom`` frame the kit's flight interface takes.

The transform between them is **measured**, not assumed
(``contracts/frames.md``, "Measure a rigid course-to-odom transform at each
tracking initialization"):

    p_odom = o_odom + R_odom_course * p_course

``o_odom`` is the odom reading simultaneous with the course reference pose,
and ``R_odom_course`` rotates course-world vectors into odom-world vectors.
frames.md: "Do not assume an identity rotation from the launch heading" --
stereo-only cuVSLAM starts at the rig's startup pose, without compass or
gravity alignment, so the map's launch heading does not place the odom frame.
A measured identity (yaw 0, ``o_odom`` zero) may still be *passed explicitly*;
nothing here supplies a default. The node's parameters are required and it
fails closed without them (``mission_node.py``).

Setpoint height is ``flight height - body resting height`` (frames.md step 3);
the resting height is measured too. No coordinate is written here: the course
geometry comes from the map file (AGENTS.md §3.7 on the contracts branch).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Sequence, Tuple

Vec = Tuple[float, float]


@dataclasses.dataclass(frozen=True)
class OdomFrame:
    """The measured course->odom transform, contracts/frames.md:
    ``p_odom = o_odom + R_odom_course * p_course``.

    ``yaw`` is the rotation of ``R_odom_course``, radians counter-clockwise
    from east: a course direction at heading ``a`` points at odom heading
    ``a + yaw``. ``(x, y)`` is ``o_odom``, the odom reading of the course
    origin, in metres. Both are measured at each tracking initialisation and
    passed in -- never derived, never defaulted. A measured identity
    (``yaw`` 0, ``o_odom`` zero) is passed explicitly.
    """

    yaw: float
    x: float
    y: float

    @classmethod
    def from_measured(cls, yaw: float, o_odom: Sequence[float]) -> "OdomFrame":
        """The transform as measured (contracts/frames.md). No defaults: every
        caller passes measured numbers, and anything non-finite is refused."""
        if not (math.isfinite(yaw) and math.isfinite(o_odom[0]) and math.isfinite(o_odom[1])):
            raise ValueError("the measured course->odom transform must be three finite numbers")
        return cls(float(yaw), float(o_odom[0]), float(o_odom[1]))

    # -- positions and directions --------------------------------------------------

    def to_odom(self, east: float, north: float) -> Vec:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return east * c - north * s + self.x, east * s + north * c + self.y

    def to_course(self, x: float, y: float) -> Vec:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.x, y - self.y
        return dx * c + dy * s, -dx * s + dy * c

    def rotate_to_odom(self, ve: float, vn: float) -> Vec:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return ve * c - vn * s, ve * s + vn * c

    def rotate_to_course(self, vx: float, vy: float) -> Vec:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return vx * c + vy * s, -vx * s + vy * c

    # -- headings ------------------------------------------------------------------------

    def yaw_to_odom(self, course_heading: float) -> float:
        return wrap(course_heading + self.yaw)

    def yaw_to_course(self, odom_yaw: float) -> float:
        return wrap(odom_yaw - self.yaw)


def model_odom_frame(position: Sequence[float], heading_vec: Sequence[float]) -> OdomFrame:
    """The odom frame the offline model flies in (``vehicle_sim``, ``dry_run``, the tests):
    cuVSLAM starts at the drone's pose on the launch mark, so the *model's* odom origin
    is the map's launch position and its +x is the launch heading.

    A model, for offline runs only. The node's frame is measured (contracts/frames.md)
    and is never derived from the launch heading."""
    hx, hy = float(heading_vec[0]), float(heading_vec[1])
    if math.hypot(hx, hy) < 1e-9:
        raise ValueError("launch heading has zero length")
    h = math.atan2(hy, hx)               # odom +x's course heading, so R rotates by -h
    c, s = math.cos(-h), math.sin(-h)
    e, n = float(position[0]), float(position[1])
    return OdomFrame.from_measured(-h, (-(e * c - n * s), -(e * s + n * c)))   # o_odom = -R * launch



def frame_from_mark(position: Sequence[float], heading_vec: Sequence[float],
                    odom_x: float, odom_y: float, odom_yaw: float) -> OdomFrame:
    """The transform measured from an odometry reading taken with the drone placed on the
    launch mark, facing the launch heading (``position`` and ``heading_vec`` in course
    coordinates, from the map). This is contracts/frames.md's ``o_course`` and simultaneous
    ``o_odom``: the odom yaw is read, so no rotation is assumed."""
    hx, hy = float(heading_vec[0]), float(heading_vec[1])
    if math.hypot(hx, hy) < 1e-9:
        raise ValueError("launch heading has zero length")
    yaw = wrap(odom_yaw - math.atan2(hy, hx))
    c, s = math.cos(yaw), math.sin(yaw)
    e, n = float(position[0]), float(position[1])
    return OdomFrame.from_measured(yaw, (odom_x - (e * c - n * s), odom_y - (e * s + n * c)))


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
