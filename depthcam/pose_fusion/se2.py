"""Planar pose algebra (SE(2)) and the quaternion helpers the lidar/VO fusion needs.

Conventions:

* A :class:`Pose2` ``(x, y, yaw)`` maps points from its child frame into its parent frame:
  ``p_parent = R(yaw) @ p_child + (x, y)``.
* ``a.compose(b)`` is ``a ∘ b``: apply ``b`` first, then ``a``.
* Quaternions are ``(x, y, z, w)`` tuples with the Hamilton product, the same order as ROS
  messages.

Standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

Quat = Tuple[float, float, float, float]

TWO_PI = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """*angle* wrapped into ``[-pi, pi)``."""
    return (angle + math.pi) % TWO_PI - math.pi


@dataclass(frozen=True)
class Pose2:
    """A planar rigid transform: translation ``(x, y)`` in metres, rotation ``yaw`` in radians."""

    x: float
    y: float
    yaw: float

    def compose(self, other: "Pose2") -> "Pose2":
        """``self ∘ other``."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(self.x + c * other.x - s * other.y,
                     self.y + s * other.x + c * other.y,
                     wrap_angle(self.yaw + other.yaw))

    def inverse(self) -> "Pose2":
        """The transform that undoes ``self``: ``self.compose(self.inverse())`` is the identity."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(-c * self.x - s * self.y, s * self.x - c * self.y, wrap_angle(-self.yaw))

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


IDENTITY = Pose2(0.0, 0.0, 0.0)


def interpolate(a: Pose2, b: Pose2, alpha: float) -> Pose2:
    """Linear interpolation from *a* (``alpha = 0``) to *b* (``alpha = 1``), yaw along the short way."""
    return Pose2(a.x + alpha * (b.x - a.x),
                 a.y + alpha * (b.y - a.y),
                 wrap_angle(a.yaw + alpha * wrap_angle(b.yaw - a.yaw)))


def quat_yaw(q: Quat) -> float:
    """Yaw (rotation about z, the Z of a ZYX Euler sequence) of the unit quaternion *q*."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_about_z(yaw: float) -> Quat:
    """The unit quaternion of a rotation by *yaw* about z."""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product ``a ⊗ b`` (apply ``b``, then ``a``)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate_quat_about_z(q: Quat, yaw: float) -> Quat:
    """Orientation *q* turned by *yaw* about the parent frame's z axis: ``Rz(yaw) ⊗ q``.

    Roll and pitch (ZYX) are unchanged and yaw grows by *yaw*. A zero turn returns *q* itself,
    so a pass-through stays bit-exact. *q* is assumed to be unit length; it is not renormalised.
    """
    if yaw == 0.0:
        return (q[0], q[1], q[2], q[3])
    return quat_multiply(quat_about_z(yaw), q)
