"""Odometry guard node.

Subscribes to /visual_slam/tracking/odometry (skybeta's cuVSLAM output),
withholds poses when the tracker is failing or the step implies a reset,
then republishes survivors on /visual_slam/tracking/odometry_mapped with
fixed covariances.

A settling window (settle_s) discards the first good messages after any
break, since cuVSLAM can emit plausible poses as it recovers before fully
re-locking.

Topics
------
  Sub: /visual_slam/tracking/odometry    nav_msgs/Odometry
  Sub: /visual_slam/status               VisualSlamStatus (optional)
  Sub: /oak/left/image_raw               sensor_msgs/Image
  Pub: /visual_slam/tracking/odometry_mapped  nav_msgs/Odometry
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    _VO_STATUS_AVAILABLE = True
except ImportError:
    VisualSlamStatus = None
    _VO_STATUS_AVAILABLE = False

# Default guard thresholds — tunable via ROS parameters
_MIN_BRIGHTNESS_DN = 15.0   # mean grey level; below this the image is too dark to track
_MAX_SPEED_MPS = 10.0       # above this the step is a tracker jump, not flight
_MAX_ROT_DPS = 720.0        # above this the rotation is a tracker jump
_SETTLE_S = 0.2             # good messages needed before re-publishing after a break

# Fixed covariance sent instead of cuVSLAM's: (x,y,z) position, (r,p,y) orientation variances
_POS_VAR = (0.01, 0.01, 0.02)
_ORI_VAR = (0.01, 0.01, 0.02)
_COV_IDX = (0, 7, 14, 21, 28, 35)  # diagonal indices of a row-major 6x6 matrix


def _fixed_covariance() -> list[float]:
    cov = [0.0] * 36
    for idx, var in zip(_COV_IDX, (*_POS_VAR, *_ORI_VAR)):
        cov[idx] = var
    return cov


def _mean_brightness(msg: Image) -> float:
    """Mean of all bytes in a bgr8 image — cheap proxy for luminance."""
    if not msg.data:
        return 255.0
    return float(np.frombuffer(bytes(msg.data), dtype=np.uint8).mean())


def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Rotation angle in degrees between two xyzw unit quaternions."""
    dot = min(1.0, abs(float(np.dot(q1, q2))))
    return math.degrees(2.0 * math.acos(dot))


class OdometryGuard(Node):
    """Guards cuVSLAM odometry and republishes clean poses."""

    def __init__(self) -> None:
        super().__init__('odometry_guard')

        self.declare_parameter('odom_in', '/visual_slam/tracking/odometry')
        self.declare_parameter('odom_out', '/visual_slam/tracking/odometry_mapped')
        self.declare_parameter('min_brightness_dn', _MIN_BRIGHTNESS_DN)
        self.declare_parameter('max_speed_mps', _MAX_SPEED_MPS)
        self.declare_parameter('max_rot_dps', _MAX_ROT_DPS)
        self.declare_parameter('settle_s', _SETTLE_S)

        odom_in = self.get_parameter('odom_in').get_parameter_value().string_value
        odom_out = self.get_parameter('odom_out').get_parameter_value().string_value
        self._min_brightness = self.get_parameter('min_brightness_dn').get_parameter_value().double_value
        self._max_speed = self.get_parameter('max_speed_mps').get_parameter_value().double_value
        self._max_rot = self.get_parameter('max_rot_dps').get_parameter_value().double_value
        self._settle_s = self.get_parameter('settle_s').get_parameter_value().double_value

        self._pub = self.create_publisher(Odometry, odom_out, 10)
        self._fixed_cov = _fixed_covariance()

        self._vo_failed = False
        self._in_break = False
        self._settle_start_s: float | None = None
        self._previous: tuple[int, np.ndarray, np.ndarray] | None = None
        self._latest_image: Image | None = None
        self._withheld: dict[str, int] = {}
        self._relayed = 0

        self.create_subscription(Odometry, odom_in, self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(Image, '/oak/left/image_raw', self._on_image, qos_profile_sensor_data)

        if _VO_STATUS_AVAILABLE:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     self._on_status, qos_profile_sensor_data)
        else:
            self.get_logger().warning('isaac_ros_visual_slam_interfaces not available; '
                                      'tracker failures not visible from /visual_slam/status')

        self.create_timer(10.0, self._log_status)
        self.get_logger().info(f'guarding {odom_in} → {odom_out}')

    def _on_image(self, msg: Image) -> None:
        self._latest_image = msg

    def _on_status(self, msg) -> None:
        self._vo_failed = int(msg.vo_state) == 2  # VoState.FAILED

    def _on_odometry(self, msg: Odometry) -> None:
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        q_xyzw = np.array([q.x, q.y, q.z, q.w])

        reason = self._guard(ts_ns, position, q_xyzw)

        if reason is not None:
            self._withheld[reason] = self._withheld.get(reason, 0) + 1
            self._in_break = True
            self._settle_start_s = None
            return

        if self._in_break:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if self._settle_start_s is None:
                self._settle_start_s = now_s
            if now_s - self._settle_start_s < self._settle_s:
                self._withheld['settling'] = self._withheld.get('settling', 0) + 1
                return
            self._in_break = False
            self._settle_start_s = None

        self._previous = (ts_ns, position, q_xyzw)
        msg.pose.covariance = self._fixed_cov
        self._pub.publish(msg)
        self._relayed += 1

    def _guard(self, ts_ns: int, position: np.ndarray, q_xyzw: np.ndarray) -> str | None:
        if self._vo_failed:
            return 'vo_failed'

        if self._latest_image is not None:
            if _mean_brightness(self._latest_image) < self._min_brightness:
                return 'dark'

        if self._previous is not None:
            prev_ts, prev_pos, prev_q = self._previous
            dt_s = (ts_ns - prev_ts) / 1e9
            if dt_s > 0:
                if float(np.linalg.norm(position - prev_pos)) / dt_s > self._max_speed:
                    return 'jump'
                if _quat_angle_deg(q_xyzw, prev_q) / dt_s > self._max_rot:
                    return 'spin'

        return None

    def _log_status(self) -> None:
        total = sum(self._withheld.values())
        reasons = ', '.join(f'{r} {n}' for r, n in self._withheld.items()) or 'none'
        self.get_logger().info(f'relayed {self._relayed}, withheld {total} ({reasons})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
