"""Full-course manual mission: Maze → Tunnel → Chicane (window) → Land.

All geometry is derived from the GLB course model (Y-up, net-center origin).
ENU frame: origin = launch mark; +x = east; +y = north; +z = up.

Wall geometry (ENU, metres):
  Net      x=[-0.533, 5.563]  y=[-3.962,  2.134]
  M1       x=[0.533,  0.635]  y=[-0.406,  1.206]  bottom-fixed, free top
  M2       x=[1.651,  1.753]  y=[0.521,   2.134]  top-fixed (N wall), free bottom
  M3       x=[2.769,  2.871]  y=[-0.406,  1.206]  bottom-fixed, free top
  C1       x=[3.440,  4.087]  y=[-2.060, -0.510]  solid block
  A1       x=[1.488,  1.590]  y=[-1.537, -0.508]  solid, N end at barrier
  C2b      x=[2.272,  2.719]  y=[-2.446, -1.444]  solid block
  C2a(win) diagonal; WINDOW opens at y∈[-3.633,-2.739], heights 0.305–1.219 m
  Tunnel   x=[4.089,  5.563]  y=[-1.219,  0.305]  center x=4.826

Stage 4 scores via the WINDOW route (+5 bonus pts vs Gate B).

States: WAIT_FOR_OFFBOARD → NAVIGATE → LAND → HOLD_LANDED

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

# ── Waypoint table ───────────────────────────────────────────────────────────
# (ENU_x, ENU_y) pairs; z = hover_altitude_m throughout except LAND state.
# Each entry has a comment explaining the geometric constraint it satisfies.
_WAYPOINTS_XY: list[tuple[float, float]] = [
    # MAZE ── zig-zag through M1/M2/M3 dividers ─────────────────────────────
    (0.000,  0.000),   # WP0  ascend above launch mark (west corridor centre x=0)
    (0.000,  1.606),   # WP1  north past M1 top (1.206) — 40 cm clear; stay x=0
    (1.143,  1.606),   # WP2  east past M1; centred in M1-M2 corridor (x=1.143)
    (1.143,  0.261),   # WP3  south past M2 south (0.521) — 26 cm clear
    (2.261,  0.261),   # WP4  east past M2; centred in M2-M3 corridor (x=2.261)
    (2.261,  1.606),   # WP5  north past M3 top (1.206) — 40 cm clear
    (3.200,  1.606),   # WP6  Gate A — east of M3 east (2.871)

    # PRE-TUNNEL ─────────────────────────────────────────────────────────────
    (4.826,  1.606),   # WP7  east; x-align with tunnel centre (4.826)
    (4.826,  0.600),   # WP8  south; stage 30 cm north of tunnel entrance (y=0.305)

    # TUNNEL — dark, VIO unreliable; dead-reckoning straight south ───────────
    # Tunnel: y=0.305 (N entrance) to y=-1.219 (S exit), length 1.524 m
    (4.826, -1.500),   # WP9  through tunnel — overshoot 28 cm past S exit

    # CHICANE — navigate C1 / C2b / C2a-window ───────────────────────────────
    (4.100, -1.500),   # WP10 west; east of C1 (C1 east face x=4.087), same y
    (4.100, -2.200),   # WP11 south past C1 bottom (y=-2.060); x=4.100>C1
    (2.750, -2.200),   # WP12 west; east of C2b (C2b east face x=2.719)
    (2.750, -2.900),   # WP13 south into window y-range (window y∈[-3.633,-2.739])
    # WP13→WP14: fly WEST through C2a window at y=-2.900 (within range), z=0.80 m
    # Window allows east-wall (x=2.272-2.367) + west-wall (x=1.720-1.816) passage
    (1.600, -2.900),   # WP14 through C2a window; x<1.720 = west of C2a — STAGE 4 ✓

    # LANDING APPROACH ───────────────────────────────────────────────────────
    (0.472, -2.900),   # WP15 west to landing pad x; y=-2.900 clears all boulders
    (0.472, -1.707),   # WP16 north to landing pad centre — begin LAND state
]

_LANDING_PAD_XY = (0.472, -1.707)   # landing pad centre (ENU)


class FullCourseMission(Node):
    """WAIT_FOR_OFFBOARD → NAVIGATE → LAND → HOLD_LANDED."""

    def __init__(self, **kwargs):
        super().__init__('dronehacks_full_mission', **kwargs)

        self.declare_parameter('odometry_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('hover_altitude_m', 0.80)
        self.declare_parameter('arrival_radius_m', 0.25)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('land_speed_mps', 0.10)   # descent rate m/s

        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.hover_alt  = float(self.get_parameter('hover_altitude_m').value)
        self.arrival_r  = float(self.get_parameter('arrival_radius_m').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        rate_hz         = float(self.get_parameter('publish_rate_hz').value)
        self._dt        = 1.0 / rate_hz

        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(String, '/skybeta/state', self._state_cb, 10)

        self.position: tuple[float, float, float] | None = None
        self.offboard_active = False
        self.was_offboard = False

        self.state    = 'WAIT_FOR_OFFBOARD'
        self.start_z  = 0.0
        self.waypoints: list[tuple[float, float, float]] = []
        self.wp_idx   = 0
        self.land_z   = 0.0   # current commanded z during LAND
        self.log_once: set[str] = set()

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'full-course mission ready — {len(_WAYPOINTS_XY)} waypoints, '
            f'hover {self.hover_alt:.2f} m, land {self.land_speed:.2f} m/s')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def _state_cb(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))

    # ── publish ───────────────────────────────────────────────────────────────

    def _send(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # w=0 → flight_interface ignores yaw; keep current heading throughout
        msg.pose.orientation.w = 0.0
        self.pub.publish(msg)

    # ── geometry ─────────────────────────────────────────────────────────────

    def _xy_dist(self, tx: float, ty: float) -> float:
        if self.position is None:
            return float('inf')
        return math.hypot(self.position[0] - tx, self.position[1] - ty)

    # ── state machine ─────────────────────────────────────────────────────────

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
        self._once('waiting', 'waiting for safety pilot — arm and enter Position mode.')
        self._send(*self.position)
        if not self.offboard_active:
            return
        self.start_z = self.position[2]
        z = self.start_z + self.hover_alt
        self.waypoints = [(x, y, z) for x, y in _WAYPOINTS_XY]
        self.land_z    = z
        self.wp_idx    = 0
        self.state     = 'NAVIGATE'
        self.get_logger().info(
            f'Offboard active. ENU start ({self.position[0]:.2f}, '
            f'{self.position[1]:.2f}, {self.position[2]:.2f}). '
            f'WP0 → ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f}, {z:.2f})')

    def _ms_navigate(self) -> None:
        target = self.waypoints[self.wp_idx]
        self._send(*target)

        if self._xy_dist(target[0], target[1]) <= self.arrival_r:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                # All course waypoints done — start landing descent
                self.land_z = self.waypoints[-1][2]
                self.state  = 'LAND'
                self.get_logger().info(
                    f'WP{self.wp_idx - 1} reached — all waypoints done, '
                    f'descending to land at pad centre.')
                return
            nxt = self.waypoints[self.wp_idx]
            label = ''
            if self.wp_idx == 7:  label = ' [TUNNEL APPROACH]'
            elif self.wp_idx == 9:  label = ' [TUNNEL EXIT]'
            elif self.wp_idx == 14: label = ' [STAGE 4 WINDOW ✓]'
            elif self.wp_idx == 16: label = ' [PAD CENTRE — LAND]'
            self.get_logger().info(
                f'WP{self.wp_idx - 1} reached → '
                f'WP{self.wp_idx}/{len(self.waypoints) - 1} '
                f'ENU ({nxt[0]:.2f}, {nxt[1]:.2f}, {nxt[2]:.2f}){label}')

    def _ms_land(self) -> None:
        """Descend at land_speed_mps while holding pad centre XY."""
        px, py, _ = _LANDING_PAD_XY[0], _LANDING_PAD_XY[1], 0
        self.land_z = max(0.0, self.land_z - self.land_speed * self._dt)
        self._send(px, py, self.land_z)
        if self.land_z <= 0.05:
            self.state = 'HOLD_LANDED'
            self.get_logger().info('Touchdown — holding at z=0. Disarm when ready.')

    def _ms_hold_landed(self) -> None:
        self._send(_LANDING_PAD_XY[0], _LANDING_PAD_XY[1], 0.0)

    def _once(self, key: str, msg: str) -> None:
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FullCourseMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
