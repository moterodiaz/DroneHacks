"""Correct the flight commands, not PX4's estimate.

PX4 flies on raw cuVSLAM, in its ``odom`` frame, through the organisers' bridge, which team code
cannot sit in front of. The lidar says where the drone really is:
``truth ≈ C ∘ odom`` (``pose_fusion.fusion``). To put the drone at a true pose ``S``, PX4 must
be asked for the odom pose that ``C`` maps onto ``S``:

    setpoint_odom = C⁻¹ ∘ S

PX4 then servoes on ``C⁻¹ ∘ S − odom = C⁻¹ (S − C ∘ odom)``, which is the error in the true frame,
seen through the same correction. The route stays on the walls while PX4's own estimate keeps
drifting.

Only the plane is corrected, the same planar ``C`` as the fusion applies to poses:

* pose setpoints: x and y are mapped through ``C⁻¹``; z is unchanged. The yaw turns by ``-C.yaw``,
  except that an **all-zero quaternion stays all-zero**. The kit's flight interface reads it as
  "yaw is not being commanded" (``skybeta_flight_interface.flight_interface.pose_setpoint_cb``).
* twist setpoints: the ENU linear x and y are rotated by ``-C.yaw``; z and the yaw rate are
  unchanged.

A setpoint with non-finite values is returned unchanged, so the interface rejects it as it
always does. ``C`` is the slew-limited ``PoseFusion.correction_applied``: it never jumps, so the
commands never jump.

Standard library only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .se2 import Pose2, Quat, rotate_quat_about_z


@dataclass(frozen=True)
class PoseSetpoint:
    x: float
    y: float
    z: float
    q: Quat                     # (x, y, z, w); all zeros = yaw not commanded


@dataclass(frozen=True)
class TwistSetpoint:
    vx: float                   # ENU world, m/s
    vy: float
    vz: float
    yaw_rate: float             # rad/s about up


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def to_odom_pose(setpoint: PoseSetpoint, correction: Pose2) -> PoseSetpoint:
    """The odom-frame pose PX4 must fly to for the drone to reach *setpoint* in the true frame."""
    if not _finite(setpoint.x, setpoint.y, setpoint.z, *setpoint.q):
        return setpoint
    inv = correction.inverse()
    c, s = math.cos(inv.yaw), math.sin(inv.yaw)
    x = c * setpoint.x - s * setpoint.y + inv.x
    y = s * setpoint.x + c * setpoint.y + inv.y
    if all(v == 0.0 for v in setpoint.q):
        q = setpoint.q
    else:
        q = rotate_quat_about_z(setpoint.q, inv.yaw)
    return PoseSetpoint(x, y, setpoint.z, q)


def to_odom_twist(setpoint: TwistSetpoint, correction: Pose2) -> TwistSetpoint:
    """The odom-frame velocity for a true-frame *setpoint* velocity."""
    if not _finite(setpoint.vx, setpoint.vy, setpoint.vz, setpoint.yaw_rate):
        return setpoint
    c, s = math.cos(-correction.yaw), math.sin(-correction.yaw)
    return TwistSetpoint(c * setpoint.vx - s * setpoint.vy, s * setpoint.vx + c * setpoint.vy,
                         setpoint.vz, setpoint.yaw_rate)


def correction_moved(a: Pose2, b: Pose2, pos_m: float = 0.002, yaw_rad: float = math.radians(0.1)) -> bool:
    """Whether two corrections differ enough to re-send a held pose setpoint."""
    return math.hypot(a.x - b.x, a.y - b.y) > pos_m or abs(math.remainder(a.yaw - b.yaw, 2 * math.pi)) > yaw_rad


def refresh_due(correction: Pose2, sent_with: Pose2, setpoint_age_s: Optional[float], max_age_s: float) -> bool:
    """Whether to re-send the follower's last pose setpoint with a newer correction.

    Only while the follower is still streaming, i.e. its last pose setpoint is at most *max_age_s*
    old. When the follower falls silent on purpose (before Offboard while its start checks fail,
    BLOCKED, a stale pose), a re-sent setpoint would make it look live: the kit enters Offboard at
    the pilot's switch to Position only on a team setpoint under 1 s old (``auto_offboard.py``).
    """
    if setpoint_age_s is None or not setpoint_age_s <= max_age_s:
        return False
    return correction_moved(correction, sent_with)


def as_tuple(p: PoseSetpoint) -> Tuple[float, ...]:
    return (p.x, p.y, p.z) + tuple(p.q)
