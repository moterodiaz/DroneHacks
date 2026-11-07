"""Course frame -> tracker frame, following contracts/frames.md ("Course → odom (normative)").

contracts/frames.md requires a **measured** rigid course-to-odom transform at each tracking
initialisation:

    p_odom = o_odom + R_odom_course * (p_course - o_course)

and says "Do not assume an identity rotation from the launch heading". The tracker frame here is
the frame the tracker publishes to ``px4_odom_bridge.py``: the frame cuVSLAM starts on the body
at its reference pose. :class:`CourseToTracker` holds that transform, measured per run with no
defaults (issue #29, recommendation B): ``mark`` is the surveyed ``o_course`` (the launch mark,
from contracts/course.md at run time, AGENTS.md §3.7), ``odom_yaw_rad`` is the planar
``R_odom_course`` and ``odom_x``/``odom_y`` are the measured ``o_odom``.

Conventions differ between the contract's prose and this stack's internal course frame, and
the conversion happens in exactly one place, :func:`bearing_to_course_yaw`:

* contracts/frames.md states course heading **clockwise from north** (an operator's compass
  bearing, degrees) - the convention the ROS parameters take;
* the course frame the stack works in (contracts/course.md, the map, the route files, this
  module) is **counter-clockwise from east** (90 deg = nose north).

Step 3 of the contract's transform (height) is not used: a 2D lidar does not measure height,
and EKF2 takes height from the rangefinder.

The follower's 0.25 m / 10 deg start-tolerance check cross-checks that this measured transform
and the physical placement agree; it is not the source of the transform.
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


@dataclass(frozen=True)
class CourseToTracker:
    """The measured course -> tracker (odom) transform (contracts/frames.md, "Course → odom").

    Every measured value is required and has no default: a node built without one must fail
    closed rather than fly on an assumed transform.

    * ``mark`` - ``o_course``, the surveyed course-frame position of the tracked body at the
      reference pose: the launch mark, read from the rules file's start marking in the map
      (``pose_fusion.course_files.launch_mark_from_files``).
    * ``odom_yaw_rad`` - the planar ``R_odom_course``: the course-frame heading of the tracker
      frame's +x axis. **Convention: the course frame's own, counter-clockwise from east, in
      radians (0.5*pi = north).** contracts/frames.md's "Course heading is clockwise from
      north" is an operator's compass bearing; convert it with
      :func:`bearing_to_course_yaw` before it reaches here. cuVSLAM's odom frame starts on the
      body at the reference pose, so this yaw is also the drone's startup course heading - the
      localiser's first-guess heading.
    * ``odom_x``, ``odom_y`` - ``o_odom``, the tracker-frame reading of the reference pose, in
      metres (x along the frame's +x axis, y 90 deg counter-clockwise from it). Measured; do
      not assume the origin is the reference pose.
    """

    mark: LaunchMark
    odom_yaw_rad: float
    odom_x: float
    odom_y: float

    def __post_init__(self) -> None:
        for name, value in (("odom_yaw_rad", self.odom_yaw_rad), ("odom_x", self.odom_x),
                            ("odom_y", self.odom_y)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("CourseToTracker.{} must be a finite measured number (got {!r}); "
                                 "contracts/frames.md requires a measured course->odom transform, "
                                 "not an assumed one".format(name, value))


def bearing_to_course_yaw(bearing_deg: float) -> float:
    """An operator's measured heading, clockwise from course north in degrees
    (contracts/frames.md's "Course heading is clockwise from north"), as the course frame's
    internal yaw: counter-clockwise from east, radians (90 deg = nose north). internal = 90 - bearing.
    """
    if isinstance(bearing_deg, bool) or not isinstance(bearing_deg, (int, float)) \
            or not math.isfinite(bearing_deg):
        raise ValueError("the measured bearing must be a finite number of degrees (got {!r})".format(bearing_deg))
    return wrap_angle(0.5 * math.pi - math.radians(float(bearing_deg)))


def course_pose_to_tracker(east: float, north: float, heading: float, tf: CourseToTracker) -> Pose2:
    """A body pose in the course frame, as a :class:`Pose2` in the tracker frame.

    ``p_tracker = o_odom + R_odom_course @ (p_course - o_course)``, planar: the displacement from
    the surveyed reference position is rotated into the tracker frame by the measured yaw and
    shifted by the measured ``o_odom``. *heading* is the course-frame yaw, counter-clockwise
    from east; the tracker yaw is ``heading - odom_yaw`` (the translation does not change it).
    """
    d_east = east - tf.mark.east          # p_course - o_course
    d_north = north - tf.mark.north
    c, s = math.cos(tf.odom_yaw_rad), math.sin(tf.odom_yaw_rad)
    x = c * d_east + s * d_north + tf.odom_x
    y = -s * d_east + c * d_north + tf.odom_y
    return Pose2(x, y, wrap_angle(heading - tf.odom_yaw_rad))


def course_cov_to_tracker(cov: np.ndarray, tf: CourseToTracker) -> np.ndarray:
    """A 3x3 (east, north, heading) covariance as a 3x3 (x, y, yaw) covariance in the tracker frame.

    The rotation of :func:`course_pose_to_tracker` applies; the ``o_odom`` shift does not.
    """
    cov = np.asarray(cov, dtype=float)
    if cov.shape != (3, 3):
        raise ValueError("expected a 3x3 covariance, got shape {}".format(cov.shape))
    c, s = math.cos(tf.odom_yaw_rad), math.sin(tf.odom_yaw_rad)
    jac = np.array([[c, s, 0.0],
                    [-s, c, 0.0],
                    [0.0, 0.0, 1.0]])
    return jac @ cov @ jac.T
