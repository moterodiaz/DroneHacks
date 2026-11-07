"""Conversions between ROS 2 messages and the fusion's plain types.

There is no rclpy import here. The functions only read and write message attributes, so the tests
can pass simple stand-in objects, and the logic is checked on hosts without ROS.

Expected inputs:

* Guard output: ``nav_msgs/Odometry`` in the tracker frame (contracts/frames.md, "Origin"),
  with ``header.stamp`` on the ROS clock.
* Lidar localiser: ``geometry_msgs/PoseWithCovarianceStamped`` of the **body** (not the lidar
  head) in the course frame (x east, y north, origin at the floor centre; contracts/course.md).
  ``header.frame_id`` must equal the configured ``lidar_frame_id``, and the stamp is the scan's
  time after deskewing, on the same clock as the Guard. The x, y and yaw variances must be
  positive: a localiser that reports zero means "unknown", and is refused rather than trusted.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import numpy as np

from .frames import CourseToTracker, course_cov_to_tracker, course_pose_to_tracker
from .fusion import FusedPose, LidarFix, VoSample
from .se2 import quat_yaw

if TYPE_CHECKING:
    from .setpoints import PoseSetpoint, TwistSetpoint

#: Rows/columns of x, y and yaw (rotation about z) in a ROS 6x6 pose covariance (x, y, z, rot x, rot y, rot z).
POSE_COV_XY_YAW = (0, 1, 5)


class LidarInputError(ValueError):
    """A lidar pose message the fusion refuses: wrong frame, unusable covariance, non-finite values."""


def stamp_to_s(stamp: Any) -> float:
    """A ``builtin_interfaces/Time`` in seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def odometry_to_vo_sample(msg: Any) -> VoSample:
    """The Guard's ``nav_msgs/Odometry`` as a :class:`VoSample`."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return VoSample(stamp_to_s(msg.header.stamp), float(p.x), float(p.y), float(p.z),
                    (float(q.x), float(q.y), float(q.z), float(q.w)))


def lidar_pose_to_fix(msg: Any, expected_frame_id: str, tf: CourseToTracker) -> LidarFix:
    """The localiser's course-frame ``geometry_msgs/PoseWithCovarianceStamped`` as a tracker-frame :class:`LidarFix`,
    through the measured course->odom transform (contracts/frames.md "Course → odom")."""
    frame_id = msg.header.frame_id
    if frame_id != expected_frame_id:
        raise LidarInputError("lidar pose is in frame {!r}, expected {!r} (the course frame: x east, "
                              "y north)".format(frame_id, expected_frame_id))
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    values = (float(p.x), float(p.y), float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(v) for v in values):
        raise LidarInputError("lidar pose has non-finite values: {}".format(values))
    cov6 = np.asarray(msg.pose.covariance, dtype=float)
    if cov6.size != 36:
        raise LidarInputError("lidar pose covariance has {} entries, expected 36".format(cov6.size))
    cov_course = cov6.reshape(6, 6)[np.ix_(POSE_COV_XY_YAW, POSE_COV_XY_YAW)]
    if not np.all(np.isfinite(cov_course)) or np.any(np.diag(cov_course) <= 0.0):
        raise LidarInputError("lidar pose covariance needs finite, positive x, y and yaw variances "
                              "(diagonal {})".format(np.diag(cov_course).tolist()))
    heading = quat_yaw((values[2], values[3], values[4], values[5]))
    pose = course_pose_to_tracker(values[0], values[1], heading, tf)
    return LidarFix(stamp_to_s(msg.header.stamp), pose, course_cov_to_tracker(cov_course, tf))


#: Variance written for the axes a 2D lidar does not measure (z, roll, pitch): "unknown".
UNKNOWN_VARIANCE = 1.0e6


def s_to_stamp(t: float) -> "tuple[int, int]":
    """Seconds as ``builtin_interfaces/Time`` ``(sec, nanosec)``."""
    sec = math.floor(t)
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return int(sec), nanosec


def fill_pose_with_covariance(msg: Any, x: float, y: float, z: float, yaw: float, cov_xy_yaw: np.ndarray) -> None:
    """Write a course-frame body pose into a ``geometry_msgs/PoseWithCovarianceStamped`` (not the header).

    x, y and yaw carry *cov_xy_yaw* at rows/columns 0, 1 and 5. z, roll and pitch are marked
    unknown. This is the inverse of :func:`lidar_pose_to_fix`, which reads it back.
    """
    cov = np.asarray(cov_xy_yaw, dtype=float)
    if cov.shape != (3, 3) or not np.all(np.isfinite(cov)):
        raise ValueError("cov_xy_yaw must be a finite 3x3 matrix")
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    full = np.zeros((6, 6))
    for k in (2, 3, 4):
        full[k, k] = UNKNOWN_VARIANCE
    full[np.ix_(POSE_COV_XY_YAW, POSE_COV_XY_YAW)] = 0.5 * (cov + cov.T)
    msg.pose.covariance = [float(v) for v in full.ravel()]


def pose_setpoint_from_msg(msg: Any) -> "PoseSetpoint":
    """A ``geometry_msgs/PoseStamped`` setpoint as a :class:`pose_fusion.setpoints.PoseSetpoint`."""
    from .setpoints import PoseSetpoint

    p, q = msg.pose.position, msg.pose.orientation
    return PoseSetpoint(float(p.x), float(p.y), float(p.z), (float(q.x), float(q.y), float(q.z), float(q.w)))


def fill_pose_setpoint(msg: Any, setpoint: "PoseSetpoint") -> None:
    """Write *setpoint* into a ``geometry_msgs/PoseStamped`` (not the header)."""
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(setpoint.x), float(setpoint.y),
                                                                     float(setpoint.z))
    (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
     msg.pose.orientation.w) = (float(v) for v in setpoint.q)


def twist_setpoint_from_msg(msg: Any) -> "TwistSetpoint":
    """A ``geometry_msgs/TwistStamped`` setpoint as a :class:`pose_fusion.setpoints.TwistSetpoint`."""
    from .setpoints import TwistSetpoint

    lin = msg.twist.linear
    return TwistSetpoint(float(lin.x), float(lin.y), float(lin.z), float(msg.twist.angular.z))


def fill_twist_setpoint(msg: Any, setpoint: "TwistSetpoint", source: Any) -> None:
    """Write *setpoint* into a ``geometry_msgs/TwistStamped``; angular x/y are copied from *source*."""
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(setpoint.vx), float(setpoint.vy),
                                                                   float(setpoint.vz))
    msg.twist.angular.x = float(source.twist.angular.x)
    msg.twist.angular.y = float(source.twist.angular.y)
    msg.twist.angular.z = float(setpoint.yaw_rate)


def fill_pose(pose_msg: Any, fused: FusedPose) -> None:
    """Write *fused* into a ``geometry_msgs/Pose`` (position and orientation)."""
    pose_msg.position.x = float(fused.x)
    pose_msg.position.y = float(fused.y)
    pose_msg.position.z = float(fused.z)
    pose_msg.orientation.x = float(fused.q[0])
    pose_msg.orientation.y = float(fused.q[1])
    pose_msg.orientation.z = float(fused.q[2])
    pose_msg.orientation.w = float(fused.q[3])
