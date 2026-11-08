"""ROS 2 wrapper: feeds odometry and /skybeta/state to the Mission and
publishes its twists on /skybeta/setpoint/twist. All flight logic is in
mission.py, so it runs unchanged in the offline simulator (sim.py).

    subscribe  /visual_slam/tracking/odometry   nav_msgs/Odometry (ENU odom)
    subscribe  /skybeta/state                   std_msgs/String (JSON)
    publish    /skybeta/setpoint/twist          geometry_msgs/TwistStamped

This node never touches /fmu/*, never arms, never changes mode.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .mission import Config, Mission, Observation
from .path import load_path_csv, twist_for_interface


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_path_csv():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('dronehacks_pathfly'),
                        'data', 'path_dense.csv')


class PathFlyNode(Node):
    def __init__(self, **kwargs):
        super().__init__('dronehacks_pathfly', **kwargs)
        cfg = Config()
        # Booleans stay booleans (hold_heading: true); everything else is a
        # float, so write 2.0, not 2, in the YAML.
        defaults = {name: getattr(cfg, name) for name in Config.field_names()}
        for name, value in defaults.items():
            self.declare_parameter(
                name, value if isinstance(value, bool) else float(value))
        self.declare_parameter('path_csv', '')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('odometry_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('state_topic', '/skybeta/state')
        self.declare_parameter('twist_setpoint_topic', '/skybeta/setpoint/twist')
        # Measured heading of the vision frame relative to EKF2's NED, same
        # meaning and value as flight_interface's parameter of this name
        # (measure it with pathfly_check). Applied to our twists, which the
        # interface itself does not rotate.
        self.declare_parameter('odom_to_ned_yaw_offset_rad', 0.0)

        for name, default in defaults.items():
            value = self.get_parameter(name).value
            setattr(cfg, name,
                    bool(value) if isinstance(default, bool) else float(value))
        path_csv = self.get_parameter('path_csv').value or default_path_csv()
        points, margins = load_path_csv(path_csv)
        # Raises (and the node exits) on a bad config or path: better than
        # flying something half-configured. With no setpoints from us the
        # flight interface refuses Offboard, so the pilot keeps the vehicle.
        self.mission = Mission(points, margins, cfg)

        self.pub = self.create_publisher(
            TwistStamped, self.get_parameter('twist_setpoint_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self.odometry_cb, 10)
        self.create_subscription(
            String, self.get_parameter('state_topic').value, self.state_cb, 10)

        self.position = None
        self.yaw = None
        self.odom_t = None
        self.offboard = None
        self.landed = None
        self.state_t = None
        self.divergence = None
        self.clamp_logged = set()

        self.yaw_offset = float(
            self.get_parameter('odom_to_ned_yaw_offset_rad').value)
        if self.yaw_offset:
            self.get_logger().info(
                f'rotating twists by odom_to_ned_yaw_offset_rad = '
                f'{self.yaw_offset:.4f} rad ({math.degrees(self.yaw_offset):.2f} deg)')
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'path {path_csv}')
        self.get_logger().info(self.mission.summary())
        if self.mission.heading_warning:
            self.get_logger().warn(self.mission.heading_warning)
        self.get_logger().info(
            'Waiting for the safety pilot to arm (Altitude), then switch to '
            'Position. Zero-velocity twists are published while odometry is '
            'fresh, so the flight interface can enter Offboard.')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        pos = (float(p.x), float(p.y), float(p.z))
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not all(math.isfinite(v) for v in pos + (yaw,)):
            return
        self.position, self.yaw, self.odom_t = pos, yaw, self.now_s()

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(state.get('offboard_active', False))
        o, l = state.get('odom_enu'), state.get('local_enu')
        self.divergence = (tuple(float(a) - float(b) for a, b in zip(o, l))
                           if o and l and len(o) == 3 and len(l) == 3 else None)
        landed = state.get('landed')
        self.landed = None if landed is None else bool(landed)
        self.state_t = self.now_s()
        for reason in state.get('clamped') or []:
            if reason not in self.clamp_logged:
                self.clamp_logged.add(reason)
                self.get_logger().warn(
                    f'flight interface clamped our setpoint: {reason} '
                    f'(envelope {state.get("envelope")})')

    def tick(self):
        now = self.now_s()
        obs = Observation(
            position=self.position, yaw=self.yaw,
            odom_age_s=math.inf if self.odom_t is None else now - self.odom_t,
            offboard=self.offboard, landed=self.landed,
            divergence=self.divergence,
            state_age_s=math.inf if self.state_t is None else now - self.state_t)
        cmd = self.mission.update(now, obs)
        for line in self.mission.events:
            if any(w in line for w in ('HOLD', 'ABORTED', 'lost', 'stale',
                                       'timed out', 'not flying', 'did not')):
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        self.mission.events.clear()
        if cmd.twist is None:
            return
        twist = twist_for_interface(cmd.twist, self.yaw_offset)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(twist[0])
        msg.twist.linear.y = float(twist[1])
        msg.twist.linear.z = float(twist[2])
        msg.twist.angular.z = float(twist[3])
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
