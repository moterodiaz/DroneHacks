"""ROS wrapper for ``mission.Mission``: cuVSLAM pose + /skybeta/state in, kit setpoints out.

Same contract as the kit's ``competitor_example``: subscribes /visual_slam/tracking/odometry and
/skybeta/state, publishes /skybeta/setpoint/pose and /skybeta/setpoint/twist. Never touches
/fmu/*, never arms or changes mode. Status (state + reason) on /square_test/status.

Recording (``record``, default true): at the first arming it starts a rosbag2 recording with
dronehacks_telemetry's ``light`` profile (every topic but images, point clouds and visualisation)
into ``/root/team_ws/flights/<UTC time>-light-square/`` (host: ``~/team_ws/flights``), the layout
``python3 -m dronehacks_telemetry.analyze <folder>`` reads. When the mission is over and PX4 has
disarmed, it waits 5 s, closes the bag with SIGINT and exits. Cut power before that and the bag
is left open: ``ros2 bag reindex <folder>/bag -s mcap`` in the container before analysing it.
"""
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from dronehacks_square_test.mission import Config, Mission

FLIGHTS = '/root/team_ws/flights'
LIGHT_EXCLUDE = 'image|compressed|/depth|points|cloud|/visual_slam/vis/|_path$'  # = record_flight.sh's


class SquareTest(Node):
    def __init__(self):
        super().__init__('square_test')
        defaults = Config()
        cfg = {}
        for name, value in vars(defaults).items():
            # dynamic_typing: a launch override like side_m:=1 arrives as an int
            self.declare_parameter(name, float(value), ParameterDescriptor(dynamic_typing=True))
            cfg[name] = float(self.get_parameter(name).value)
        self.mission = Mission(Config(**cfg))
        self.declare_parameter('record', True, ParameterDescriptor(dynamic_typing=True))
        self.record = str(self.get_parameter('record').value).lower() in ('true', '1')
        self.recorder = None
        self.pose = None
        self.pose_t = None
        self.yaw = None
        self.offboard = False
        self.landed = None
        self.armed = None
        self.px4 = None               # PX4 local position, ENU, from /skybeta/state
        self.last_status = None
        self.finished = False        # main() closes the recording and exits on it
        self.end_t = None

        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.status_pub = self.create_publisher(String, '/square_test/status', 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)
        self.create_timer(0.05, self.tick)  # 20 Hz
        c = self.mission.cfg
        self.get_logger().info(
            'square_test ready: up {:.2f} m, {:.2f} m box turning {:.0f} deg/s at each corner, {:.0f} s holds, '
            'land, touch-and-go, land and disarm. Arm, then switch to Position.'.format(
                c.height_m, c.side_m, c.yaw_rate_dps, c.hold_s))

    def start_recording(self):
        started = time.gmtime()
        run = os.path.join(FLIGHTS, time.strftime('%Y%m%d-%H%M%S', started) + '-light-square')
        try:
            os.makedirs(os.path.join(run, 'meta'))
            os.makedirs(os.path.join(run, 'px4'))
            with open(os.path.join(run, 'meta', 'manifest.txt'), 'w') as f:
                f.write(''.join(line + chr(10) for line in (
                    'run_id: ' + os.path.basename(run), 'profile: light',
                    'started_utc: ' + time.strftime('%Y-%m-%dT%H:%M:%SZ', started),
                    'recorded_by: dronehacks_square_test (at arming)', 'exclude_regex: ' + LIGHT_EXCLUDE)))
            log = open(os.path.join(run, 'meta', 'recorder.log'), 'w')
            self.recorder = subprocess.Popen(
                ['ros2', 'bag', 'record', '-s', 'mcap', '-o', os.path.join(run, 'bag'), '-a', '-x', LIGHT_EXCLUDE],
                stdout=log, stderr=subprocess.STDOUT)
            self.get_logger().info('armed: recording into {}'.format(run))
        except OSError as exc:              # never let recording stop the flight
            self.record = False
            self.get_logger().error('recording not started: {}'.format(exc))

    def stop_recording(self):
        if self.recorder is None or self.recorder.poll() is not None:
            return
        self.recorder.send_signal(signal.SIGINT)
        try:
            self.recorder.wait(timeout=30)
            self.get_logger().info('recording closed')
        except subprocess.TimeoutExpired:
            self.get_logger().error('recorder did not stop in 30 s; reindex the bag before analysing')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if all(math.isfinite(v) for v in xyz + (yaw,)):
            self.pose, self.pose_t, self.yaw = xyz, self.now(), yaw

    def state_cb(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        self.landed = s.get('landed')
        self.armed = s.get('armed')
        local = s.get('local_enu')
        self.px4 = tuple(local) if isinstance(local, list) and len(local) == 3 else None
        if s.get('clamped'):
            self.get_logger().warn('kit is clamping our setpoint: {}'.format(s['clamped']),
                                   throttle_duration_sec=2.0)

    def tick(self):
        cmd = self.mission.step(self.now(), self.pose, self.pose_t, self.offboard, self.landed, self.armed,
                                self.yaw, self.px4)
        stamp = self.get_clock().now().to_msg()
        if cmd.kind == 'pose':
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cmd.x, cmd.y, cmd.z
            if cmd.yaw is None:
                m.pose.orientation.w = 0.0      # all-zero quaternion: do not command yaw
            else:
                m.pose.orientation.z = math.sin(cmd.yaw / 2.0)
                m.pose.orientation.w = math.cos(cmd.yaw / 2.0)
            self.pose_pub.publish(m)
        elif cmd.kind == 'twist':
            m = TwistStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = cmd.x, cmd.y, cmd.z
            m.twist.angular.z = 0.0
            self.twist_pub.publish(m)
        if self.record and self.armed and self.recorder is None:
            self.start_recording()
        status = '{}: {}'.format(self.mission.state, self.mission.reason)
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status
        self.status_pub.publish(String(data=status))
        # Done once the mission is over and PX4 has disarmed; exit 5 s later so the recording
        # keeps the touchdown, then the launch file stops the bag cleanly.
        if self.mission.state in ('DONE', 'STOPPED') and self.armed is False:
            if self.end_t is None:
                self.end_t = self.now()
                self.get_logger().info('mission over and disarmed: exiting in 5 s')
            elif self.now() - self.end_t >= 5.0:
                self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = SquareTest()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
