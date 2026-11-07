"""Course frame -> tracker frame, following contracts/frames.md ("The transform (normative)").

contracts/frames.md defines the map from course ``(east, north)`` to a PX4 NED/FRD setpoint:

1. subtract the launch mark;
2. world north -> PX4 +x (forward), world east -> PX4 +y (right).

The fusion works in the frame the tracker publishes to ``px4_odom_bridge.py``: the ROS FLU
frame of the body resting on the launch mark, facing north (contracts/frames.md, "Origin").
That is the FRD frame above with y negated (frame table, FRD vs FLU): x forward = north,
y left = west. This module applies the contract's steps 1-2 unchanged, then the FRD -> FLU flip
from the frame table. It adds no rotation or sign of its own, so contracts/frames.md stays the
only source of truth. Step 3 (height) is not used: a 2D lidar does not measure height, and EKF2
takes height from the rangefinder.

The launch mark is a required value with no default. AGENTS.md §3.7 forbids course geometry
literals in code, so it comes from contracts/course.md (Geometry, "Launch mark") at run time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .se2 import Pose2, wrap_angle


@dataclass(frozen=True)
class LaunchMark:
    """Course-frame position of the launch mark, in metres. The value comes from contracts/course.md."""

    east: float
    north: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.east) and math.isfinite(self.north)):
            raise ValueError("launch mark must be finite (east={!r}, north={!r}); take it from "
                             "contracts/course.md".format(self.east, self.north))


def course_to_frd_xy(east: float, north: float, mark: LaunchMark) -> Tuple[float, float]:
    """contracts/frames.md, "The transform (normative)", steps 1 and 2: course -> PX4 (forward, right)."""
    d_east = east - mark.east          # step 1: subtract the launch mark
    d_north = north - mark.north
    return d_north, d_east             # step 2: north -> PX4 +x (forward), east -> PX4 +y (right)


def frd_to_flu_xy(forward: float, right: float) -> Tuple[float, float]:
    """contracts/frames.md frame table: x is forward in both, y is right in FRD and left in FLU."""
    return forward, -right


def course_pose_to_tracker(east: float, north: float, heading: float, mark: LaunchMark) -> Pose2:
    """A body pose in the course frame, as a :class:`Pose2` in the tracker frame.

    *heading* is the course-frame yaw, counter-clockwise from east (course x = east, y = north).
    The tracker frame starts facing north, so its yaw is counter-clockwise from north:
    ``yaw = heading - pi/2``.
    """
    x, y = frd_to_flu_xy(*course_to_frd_xy(east, north, mark))
    return Pose2(x, y, wrap_angle(heading - 0.5 * math.pi))


#: d(tracker x, y, yaw) / d(course east, north, heading) for :func:`course_pose_to_tracker`.
COURSE_TO_TRACKER_JACOBIAN = np.array([[0.0, 1.0, 0.0],
                                       [-1.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0]])


def course_cov_to_tracker(cov: np.ndarray) -> np.ndarray:
    """A 3x3 (east, north, heading) covariance as a 3x3 (x, y, yaw) covariance in the tracker frame."""
    cov = np.asarray(cov, dtype=float)
    if cov.shape != (3, 3):
        raise ValueError("expected a 3x3 covariance, got shape {}".format(cov.shape))
    jac = COURSE_TO_TRACKER_JACOBIAN
    return jac @ cov @ jac.T
