"""dronehacks_manual — odometry-only maze waypoint follower.

Flies 7 ENU waypoints through the zig-zag maze, then holds at Gate A.
No camera or lidar — pure cuVSLAM odometry + fixed altitude + course-map.

Waypoints are corridor-centered and derived from the GLB course model
(Y-up, net-center origin).  Coordinate frame: ENU, origin = launch mark.
  ENU +x = east (drone's right at startup)
  ENU +y = north (drone faces the maze)
  ENU +z = up

Subscriptions:
  /visual_slam/tracking/odometry    nav_msgs/Odometry  30 Hz
  /skybeta/state                    std_msgs/String     5 Hz

Publish:
  /skybeta/setpoint/pose            geometry_msgs/PoseStamped  10 Hz
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# ENU XY centerline waypoints derived from the GLB course model (Y-up, net center origin).
# Wall geometry (ENU, launch mark = origin):
#   Net  : x=[-0.533, 5.563]  y=[-3.962, 2.134]
#   M1   : x=[0.533, 0.635]   y=[-0.406, 1.206]  — bottom-fixed, free at top
#   M2   : x=[1.651, 1.753]   y=[0.521,  2.134]  — top-fixed (north wall), free at bottom
#   M3   : x=[2.769, 2.871]   y=[-0.406, 1.206]  — bottom-fixed, free at top
# Corridor centers: west=0.000, M1-M2=1.143, M2-M3=2.261
# Clearances: 0.40 m above M1/M3 tops → y=1.606; 0.26 m below M2 south → y=0.261
_MAZE_XY: list[tuple[float, float]] = [
    (0.000, 0.000),   # WP0: ascend above launch mark (west corridor center)
    (0.000, 1.606),   # WP1: north past M1 top — 40 cm clear
    (1.143, 1.606),   # WP2: east into M1-M2 corridor, same altitude
    (1.143, 0.261),   # WP3: south past M2 bottom — 26 cm clear
    (2.261, 0.261),   # WP4: east into M2-M3 corridor
    (2.261, 1.606),   # WP5: north past M3 top — 40 cm clear
    (3.200, 1.606),   # WP6: east past M3 — Gate A checkpoint — HOLD
]


class ManualMission(Node):
    """WAIT_FOR_OFFBOARD → NAVIGATE (waypoints) → HOLD."""

    def __init__(self, **kwargs):
        super().__init__('dronehacks_manual_maze_mission', **kwargs)

        self.declare_parameter('odometry_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('hover_altitude_m', 0.80)
        self.declare_parameter('arrival_radius_m', 0.25)
        self.declare_parameter('publish_rate_hz', 10.0)

        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.hover_alt = float(self.get_parameter('hover_altitude_m').value)
        self.arrival_r = float(self.get_parameter('arrival_radius_m').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(String, '/skybeta/state', self._state_cb, 10)

        self.position: tuple[float, float, float] | None = None
        self.offboard_active = False
        self.was_offboard = False

        self.state = 'WAIT_FOR_OFFBOARD'
        self.start_z: float = 0.0
        self.waypoints: list[tuple[float, float, float]] = []
        self.wp_idx = 0
        self.log_once: set[str] = set()

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'dronehacks_manual ready — {len(_MAZE_XY)} maze waypoints, '
            f'hover {self.hover_alt:.2f} m, arrival ±{self.arrival_r:.2f} m')

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def _state_cb(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))

    def _send(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def _xy_dist(self, target: tuple[float, float, float]) -> float:
        if self.position is None:
            return float('inf')
        return math.hypot(self.position[0] - target[0], self.position[1] - target[1])

    def _tick(self) -> None:
        if self.position is None:
            self._once('no_odom', 'waiting for odometry ...')
            return

        if self.was_offboard and not self.offboard_active:
            self.get_logger().warning('Offboard lost — mission paused.')
            self.was_offboard = False
        if self.offboard_active and not self.was_offboard:
            self.was_offboard = True

        getattr(self, f'_ms_{self.state.lower()}')()

    def _ms_wait_for_offboard(self) -> None:
        self._once('waiting', 'waiting for safety pilot to arm and enter Position mode.')
        self._send(*self.position)  # echo current pos so interface stays happy
        if not self.offboard_active:
            return
        self.start_z = self.position[2]
        z = self.start_z + self.hover_alt
        self.waypoints = [(x, y, z) for x, y in _MAZE_XY]
        self.wp_idx = 0
        self.state = 'NAVIGATE'
        self.get_logger().info(
            f'Offboard active. Start ENU '
            f'({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}). '
            f'Flying to waypoint 0/{len(self.waypoints) - 1} '
            f'at ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f}, {z:.2f})')

    def _ms_navigate(self) -> None:
        target = self.waypoints[self.wp_idx]
        self._send(*target)

        if self._xy_dist(target) <= self.arrival_r:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.state = 'HOLD'
                self.get_logger().info(
                    'All waypoints reached — holding at Gate A. '
                    'Safety pilot can land.')
                return
            nxt = self.waypoints[self.wp_idx]
            self.get_logger().info(
                f'wp {self.wp_idx - 1} reached → '
                f'wp {self.wp_idx}/{len(self.waypoints) - 1} '
                f'ENU ({nxt[0]:.2f}, {nxt[1]:.2f}, {nxt[2]:.2f})')

    def _ms_hold(self) -> None:
        self._send(*self.waypoints[-1])

    def _once(self, key: str, msg: str) -> None:
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
