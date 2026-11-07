"""Full-course manual mission: Maze → Tunnel → Chicane (Gate B) → hold on pad.

All course geometry is derived at run time from the contracts/course.md map
by :mod:`waypoints` (rule 9.5, AGENTS.md §3.9); this node holds no course
coordinate.

Contract anchors:
* ``contracts/px4.md`` — all setpoints go through ``skybeta_flight_interface``;
  autonomous landing stays blocked pending an organizer-supported procedure,
  so the final state holds over the pad and the pilot lands. The kit's 0.3 m
  altitude floor is not lowered (issue #34).
* ``contracts/frames.md`` — the course→odom transform is measured
  (``course_to_odom_yaw/x/y``), never assumed; arming does not reset the
  vision origin. The node fails closed before creating any publisher until
  the measured parameters are supplied (decision #25 point 6).
* ``contracts/ros-graph.md`` — the Guard is not in the shipped data path; the
  kit's flight interface consumes cuVSLAM odometry directly (decision #25
  point 5), so the kit defaults are used.

States (pure logic in ``waypoints.MissionEngine``):
WAIT_FOR_OFFBOARD → CLIMB → NAVIGATE → HOLD_OVER_PAD, plus HOLD_FAULT for
per-waypoint timeouts and stale/frozen tracking.

Subscriptions:
  <odometry_topic>                   nav_msgs/Odometry  ~30 Hz
  /skybeta/state                     std_msgs/String     5 Hz

Publish:
  /skybeta/setpoint/pose             geometry_msgs/PoseStamped  publish_rate_hz
"""
from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from . import waypoints as W


def _resolve_map_path(param_value):
    """Explicit parameter, else the packaged copy of the course map."""
    if param_value:
        return param_value
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory(
                'dronehacks_manual_maze_tunnel_chicane_gateb_landing'),
            'maps', 'course_v3_layout.yaml')
    except Exception:  # ament not available outside a sourced workspace
        return ''
    return candidate if os.path.isfile(candidate) else ''


class FullCourseMission(Node):
    """Drives ``W.MissionEngine`` from odometry and the kit state topic."""

    def __init__(self, **kwargs):
        super().__init__('dronehacks_full_mission', **kwargs)

        # Fail closed on the measured transform BEFORE creating any publisher
        # or subscription (contracts/frames.md; decision #25 point 6).
        def param(name, default):
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            return float(value) if isinstance(default, float) else value

        yaw = param('course_to_odom_yaw', float('nan'))
        ox = param('course_to_odom_x', float('nan'))
        oy = param('course_to_odom_y', float('nan'))
        if not (math.isfinite(yaw) and math.isfinite(ox) and math.isfinite(oy)):
            raise RuntimeError(
                'course_to_odom_yaw/x/y are required measured parameters '
                '(contracts/frames.md): the course→odom rotation and the odom '
                'reading of the launch reference. Arming does not define them. '
                'Measure per the frames.md acceptance check, then pass them; '
                'refusing to start with an assumed identity transform.')

        map_path = _resolve_map_path(param('course_map_yaml', ''))
        if not map_path:
            raise RuntimeError(
                'no course map: pass course_map_yaml (the surveyed map per '
                'contracts/course.md layout tolerance), or install this '
                "package's share/ maps copy.")
        data, goals = W.load_course_map(map_path)

        hover_alt = param('hover_altitude_m', 0.80)
        arrival_r = param('arrival_radius_m', W.DEFAULT_ARRIVAL_RADIUS_M)
        timeout_s = param('waypoint_timeout_s', 45.0)
        stale_after = param('tracking_stale_after_s', 0.5)
        frozen_window = param('tracking_frozen_window_s', 1.0)
        rate_hz = param('publish_rate_hz', 10.0)

        route = W.derive_route(data, goals, arrival_r)
        launch = tuple(map(float, goals['launch']['centre']))
        pad = tuple(map(float, goals['pad']['centre']))

        # course frame → odom frame (contracts/frames.md)
        odom_xy = [W.course_to_odom_xy((wp.x, wp.y), launch, (ox, oy), yaw)
                   for wp in route.waypoints]
        pad_odom = W.course_to_odom_xy(pad, launch, (ox, oy), yaw)

        self.engine = W.MissionEngine(
            odom_xy, hover_alt, pad_odom, arrival_r,
            waypoint_timeout_s=timeout_s)
        self.monitor = W.TrackingMonitor(stale_after_s=stale_after,
                                         frozen_window_s=frozen_window)
        self._dt = 1.0 / rate_hz
        self.position = None
        self.offboard_active = False

        odom_topic = param('odometry_topic', '/visual_slam/tracking/odometry')

        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(String, '/skybeta/state', self._state_cb, 10)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'full-course mission ready — {len(route.waypoints)} map-derived '
            f'waypoints, hover {hover_alt:.2f} m (odom frame), '
            f'arrival {arrival_r:.2f} m')
        if route.refusal_reason:
            self.get_logger().warning(route.refusal_reason)
        self.get_logger().warning(
            'heights are odom-frame and assume the vision origin is the '
            'measured launch reference (contracts/frames.md); verify with '
            'props off before flight')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        now = self.get_clock().now().nanoseconds * 1e-9
        self.position = (float(p.x), float(p.y), float(p.z))
        self.monitor.update(now, self.position)

    def _state_cb(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))

    # ── publish ─────────────────────────────────────────────────────────────

    def _send(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 0.0  # w=0 → flight_interface holds heading
        self.pub.publish(msg)

    # ── tick ─────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        tracking = (self.monitor.status(now) if self.position is not None
                    else 'stale')
        setpoint, events = self.engine.tick(
            now, self.offboard_active, self.position, tracking)
        for event in events:
            if event.startswith('ABORT'):
                self.get_logger().error(event)
            else:
                self.get_logger().info(event)
        if setpoint is not None:
            self._send(*setpoint)


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
