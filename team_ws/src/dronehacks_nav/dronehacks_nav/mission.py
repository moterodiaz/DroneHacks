"""DroneHacks navigation node — forked from competitor_example/square_mission.py.

Same WAIT_FOR_OFFBOARD → HOVER → SQUARE → HOLD skeleton.  Adds subscriptions
to every available sensor topic so perception data is on hand when we replace
the square logic with real course navigation.

Subscriptions
-------------
  /visual_slam/tracking/odometry    nav_msgs/Odometry              30 Hz
  /visual_slam/status               VisualSlamStatus               30 Hz
  /skybeta/state                    std_msgs/String                 5 Hz
  /oak/left/image_raw               sensor_msgs/Image              30 Hz  bgr8
  /oak/right/image_raw              sensor_msgs/Image              30 Hz  bgr8
  /oak/left/image_mono              sensor_msgs/Image              30 Hz  mono8
  /oak/right/image_mono             sensor_msgs/Image              30 Hz  mono8
  /oak/left/camera_info             sensor_msgs/CameraInfo         30 Hz
  /oak/right/camera_info            sensor_msgs/CameraInfo         30 Hz
  /oak/imu/data                     sensor_msgs/Imu               ~200 Hz
  /fmu/out/vehicle_local_position   px4_msgs/VehicleLocalPosition  BEST_EFFORT
  /fmu/out/vehicle_status_v1        px4_msgs/VehicleStatus         BEST_EFFORT
  /fmu/out/vehicle_land_detected    px4_msgs/VehicleLandDetected   BEST_EFFORT
  /fmu/out/vehicle_attitude         px4_msgs/VehicleAttitude       BEST_EFFORT

  vehicle_status is suffixed _v1 because PX4 v1.16 declares MESSAGE_VERSION=1
  for that message type.  The other fmu/out topics are version 0 (bare names).

Publish
-------
  /skybeta/setpoint/pose            geometry_msgs/PoseStamped      10 Hz
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

try:
    from px4_msgs.msg import (
        VehicleAttitude,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )
    _PX4_AVAILABLE = True
except ImportError:
    _PX4_AVAILABLE = False

# BEST_EFFORT / VOLATILE / KEEP_LAST — required by uXRCE-DDS publishers.
_PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class Mission(Node):
    """Hover, fly a square, hold.  Four-state machine on a 10 Hz timer."""

    def __init__(self, **kwargs):
        super().__init__('dronehacks_nav', **kwargs)

        # --- parameters -------------------------------------------------------
        self.declare_parameter('hover_altitude_m', 1.5)
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('square_side_m', 5.0)
        self.declare_parameter('arrival_radius_m', 0.3)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('corner_timeout_s', 60.0)

        self.hover_altitude = float(self.get_parameter('hover_altitude_m').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        self.side = float(self.get_parameter('square_side_m').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius_m').value)
        self.corner_timeout = float(self.get_parameter('corner_timeout_s').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # --- publisher --------------------------------------------------------
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)

        # --- VIO / state subscriptions ----------------------------------------
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self._odom_cb, 10)
        self.create_subscription(
            String, '/skybeta/state', self._state_cb, 10)

        if VisualSlamStatus is not None:
            self.create_subscription(
                VisualSlamStatus, '/visual_slam/status', self._vslam_status_cb, 10)
        else:
            self.get_logger().warn(
                'isaac_ros_visual_slam_interfaces not available; '
                '/visual_slam/status will not be monitored.')

        # --- camera subscriptions ---------------------------------------------
        self.create_subscription(
            Image, '/oak/left/image_raw', self._left_raw_cb, 10)
        self.create_subscription(
            Image, '/oak/right/image_raw', self._right_raw_cb, 10)
        self.create_subscription(
            Image, '/oak/left/image_mono', self._left_mono_cb, 10)
        self.create_subscription(
            Image, '/oak/right/image_mono', self._right_mono_cb, 10)
        self.create_subscription(
            CameraInfo, '/oak/left/camera_info', self._left_info_cb, 10)
        self.create_subscription(
            CameraInfo, '/oak/right/camera_info', self._right_info_cb, 10)

        # --- IMU subscription -------------------------------------------------
        self.create_subscription(Imu, '/oak/imu/data', self._imu_cb, 10)

        # --- PX4 subscriptions (BEST_EFFORT) ----------------------------------
        if _PX4_AVAILABLE:
            self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self._vehicle_local_position_cb,
                _PX4_QOS,
            )
            self.create_subscription(
                VehicleStatus,
                '/fmu/out/vehicle_status_v1',
                self._vehicle_status_cb,
                _PX4_QOS,
            )
            self.create_subscription(
                VehicleLandDetected,
                '/fmu/out/vehicle_land_detected',
                self._vehicle_land_detected_cb,
                _PX4_QOS,
            )
            self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
                self._vehicle_attitude_cb,
                _PX4_QOS,
            )
        else:
            self.get_logger().warn(
                'px4_msgs not available; /fmu/out/* topics will not be subscribed.')

        # --- latest data from each topic --------------------------------------
        self.position = None           # (x, y, z) ENU from cuVSLAM
        self.vo_state = None           # 0=unknown, 1=tracking, 2=lost
        self.offboard_active = False
        self.left_image_raw = None
        self.right_image_raw = None
        self.left_image_mono = None
        self.right_image_mono = None
        self.left_camera_info = None
        self.right_camera_info = None
        self.imu = None
        self.vehicle_local_position = None
        self.vehicle_status = None
        self.vehicle_land_detected = None
        self.vehicle_attitude = None

        # --- mission state ----------------------------------------------------
        self.mission_state = 'WAIT_FOR_OFFBOARD'
        self.start = None
        self.corners = []
        self.corner_index = 0
        self.hold_target = None
        self.hover_started_at = None
        self.corner_started_at = None
        self.was_offboard = False
        self.log_once = set()

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'dronehacks_nav ready: hover {self.hover_altitude:.2f} m for '
            f'{self.hover_seconds:.0f} s, then a {self.side:.1f} m square. '
            'Waiting for the safety pilot to arm, then switch to Position...')

    # --------------------------------------------------------------------------
    # callbacks — VIO / state
    # --------------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def _state_cb(self, msg: String):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))
        if state.get('clamped'):
            self._info_once(
                f'clamped:{",".join(state["clamped"])}',
                f'flight interface is CLAMPING setpoints '
                f'({", ".join(state["clamped"])}) — outside safety envelope '
                f'{state.get("envelope")}')

    def _vslam_status_cb(self, msg):
        self.vo_state = int(msg.vo_state)

    # --------------------------------------------------------------------------
    # callbacks — camera
    # --------------------------------------------------------------------------
    def _left_raw_cb(self, msg: Image):
        self.left_image_raw = msg

    def _right_raw_cb(self, msg: Image):
        self.right_image_raw = msg

    def _left_mono_cb(self, msg: Image):
        self.left_image_mono = msg

    def _right_mono_cb(self, msg: Image):
        self.right_image_mono = msg

    def _left_info_cb(self, msg: CameraInfo):
        self.left_camera_info = msg

    def _right_info_cb(self, msg: CameraInfo):
        self.right_camera_info = msg

    # --------------------------------------------------------------------------
    # callbacks — IMU
    # --------------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        self.imu = msg

    # --------------------------------------------------------------------------
    # callbacks — PX4
    # --------------------------------------------------------------------------
    def _vehicle_local_position_cb(self, msg):
        self.vehicle_local_position = msg

    def _vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    def _vehicle_land_detected_cb(self, msg):
        self.vehicle_land_detected = msg

    def _vehicle_attitude_cb(self, msg):
        self.vehicle_attitude = msg

    # --------------------------------------------------------------------------
    # setpoint publisher
    # --------------------------------------------------------------------------
    def _send(self, x, y, z, yaw_enu=None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        if yaw_enu is None:
            msg.pose.orientation.w = 0.0
        else:
            msg.pose.orientation.z = math.sin(float(yaw_enu) / 2.0)
            msg.pose.orientation.w = math.cos(float(yaw_enu) / 2.0)
        self.setpoint_pub.publish(msg)

    # --------------------------------------------------------------------------
    # geometry helpers
    # --------------------------------------------------------------------------
    def _distance_to(self, target):
        if self.position is None:
            return float('inf')
        return math.dist(self.position, target)

    def _arrived_at(self, target):
        return self._distance_to(target) <= self.arrival_radius

    def _build_square(self):
        x0, y0, _ = self.start
        z = self.start[2] + self.hover_altitude
        s = self.side
        return [
            (x0 + s, y0,     z),
            (x0 + s, y0 + s, z),
            (x0,     y0 + s, z),
            (x0,     y0,     z),
        ]

    # --------------------------------------------------------------------------
    # 10 Hz state machine
    # --------------------------------------------------------------------------
    def _tick(self):
        if self.position is None:
            self._info_once(
                'no_odom',
                'no /visual_slam/tracking/odometry yet — is cuVSLAM up and '
                'tracking? Nothing will be commanded until it is.')
            return

        if self.was_offboard and not self.offboard_active:
            self.get_logger().warn(
                'Offboard lost — pilot has taken control. Mission paused: '
                'holding position until Offboard returns. '
                'Offboard does not return until disarmed and re-armed.')
            self.was_offboard = False
        if self.offboard_active and not self.was_offboard:
            self.was_offboard = True

        # Pause: with Offboard inactive the pilot has control — hold the
        # current position and advance nothing (the "mission paused" log
        # above must be true, not cosmetic).
        if not self.offboard_active and self.mission_state != 'WAIT_FOR_OFFBOARD':
            self._send(*self.position)
            return

        handler = getattr(self, f'_ms_{self.mission_state.lower()}')
        handler()

    # -- states ----------------------------------------------------------------
    def _ms_wait_for_offboard(self):
        if not self.offboard_active:
            self._info_once(
                'waiting',
                'holding at current position until the safety pilot arms '
                'and switches to Position.')
            self._send(*self.position)
            return
        self.start = self.position
        self.corners = self._build_square()
        self.hold_target = None
        self.hover_started_at = self._now_s()
        self.mission_state = 'HOVER'
        self.get_logger().info(
            f'Offboard active. Start ENU '
            f'({self.start[0]:.2f}, {self.start[1]:.2f}, {self.start[2]:.2f}); '
            f'climbing to {self.hover_altitude:.2f} m.')

    def _ms_hover(self):
        target = (self.start[0], self.start[1], self.start[2] + self.hover_altitude)
        self._send(*target)
        if not self._arrived_at(target):
            self.hover_started_at = self._now_s()
            return
        if self._now_s() - self.hover_started_at < self.hover_seconds:
            return
        self.corner_index = 0
        self.corner_started_at = self._now_s()
        self.mission_state = 'SQUARE'
        self.get_logger().info(
            f'hover complete — flying the {self.side:.1f} m square, '
            f'corner 1/{len(self.corners)}')

    def _ms_square(self):
        target = self.corners[self.corner_index]
        self._send(*target)
        if self._arrived_at(target):
            self.corner_index += 1
            self.corner_started_at = self._now_s()
            if self.corner_index >= len(self.corners):
                self.hold_target = self.position
                self.mission_state = 'HOLD'
                self.get_logger().info(
                    'square complete — holding. Safety pilot can land.')
                return
            self.get_logger().info(
                f'corner reached — next: {self.corner_index + 1}/'
                f'{len(self.corners)} at ENU '
                f'({self.corners[self.corner_index][0]:.2f}, '
                f'{self.corners[self.corner_index][1]:.2f}, '
                f'{self.corners[self.corner_index][2]:.2f})')
            return
        if self._now_s() - self.corner_started_at > self.corner_timeout:
            self.get_logger().error(
                f'corner {self.corner_index + 1} not reached within '
                f'{self.corner_timeout:.0f} s '
                f'({self._distance_to(target):.2f} m away) — holding. '
                'Check /skybeta/state for a clamp or cuVSLAM for tracking loss.')
            self.hold_target = self.position
            self.mission_state = 'HOLD'

    def _ms_hold(self):
        # Hold where the drone actually is when HOLD is entered — never
        # command a corner that was never reached.
        self._send(*(self.hold_target or self.position))

    # --------------------------------------------------------------------------
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _info_once(self, key, message):
        if key in self.log_once:
            return
        self.log_once.add(key)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
