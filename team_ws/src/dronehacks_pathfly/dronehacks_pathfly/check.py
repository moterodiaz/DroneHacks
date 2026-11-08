"""pathfly_check: ground checks for drift, with the props off. It commands
nothing and publishes nothing, so it is safe to run next to the stack.

    ros2 run dronehacks_pathfly pathfly_check
    ros2 run dronehacks_pathfly pathfly_check --ros-args -p params_file:=/path/to.yaml

It reads the mission's settings from the same config/pathfly.yaml.

Start it with the drone sitting on START, nose east (as start_yaw_deg says).
It anchors the arena frame to the odometry pose at that moment, exactly as
the mission does when Offboard starts. Then it prints, twice a second:

- where the vision estimate says the drone is, in arena metres, and how high;
- how far that is from the planned path (cross-track) and the planner's wall
  margin there: what would be left if the drone flew where vision says it is;
- distance from START by vision and by EKF2 (they should agree);
- the frame rotation between vision and PX4's local frame, measured from
  motion (needs the drone carried 0.5 m+): this, not the attitude heading
  difference, is the value for odom_to_ned_yaw_offset_rad;
- vision height vs EKF2 height since the start.

Carry test: walk the drone by hand along the path at about 1.25 m and back
to START. On Ctrl-C it prints a summary: the closure error (how far vision
thinks it is from START when it is back on the marker) and the yaw offset.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .mission import Config, Mission
from .node import default_path_csv, yaw_from_quat
from .path import ArenaToOdom, Polyline, load_path_csv, wrap_pi


def default_params_file():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('dronehacks_pathfly'),
                        'config', 'pathfly.yaml')


def mission_params(path):
    """The dronehacks_pathfly node's ros__parameters from a params YAML."""
    import yaml
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    return (doc.get('dronehacks_pathfly') or {}).get('ros__parameters') or {}


def circular_mean(angles):
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


class PathflyCheck(Node):
    def __init__(self):
        super().__init__('dronehacks_pathfly_check')
        # Read the mission's own settings (same YAML), so the anchor matches.
        self.declare_parameter('params_file', default_params_file())
        self.declare_parameter('odometry_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('state_topic', '/skybeta/state')
        params_file = self.get_parameter('params_file').value
        params = mission_params(params_file)
        cfg = Config()
        for name in Config.field_names():
            if name in params:
                default = getattr(cfg, name)
                setattr(cfg, name, bool(params[name]) if isinstance(default, bool)
                        else float(params[name]))
        path_csv = params.get('path_csv') or default_path_csv()
        pts, margins = load_path_csv(path_csv)
        self.mission = Mission(pts, margins, cfg)   # for start point/yaw only
        self.line = Polyline(pts, margins)
        self.configured_offset = float(params.get('odom_to_ned_yaw_offset_rad', 0.0))
        self.get_logger().info(f'settings from {params_file}')

        self.tf = None
        self.anchor_z = None
        self.odom = None            # (x, y, z, yaw)
        self.state = None
        self.local0 = None
        self.odom0 = None
        self.s_hint = 0.0
        self.max_xtrack = 0.0
        self.min_clear = math.inf
        self.offsets = []
        self.rot_dot = self.rot_cross = self.scale_sum = 0.0
        self.rot_n = 0

        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value, self.odom_cb, 10)
        self.create_subscription(
            String, self.get_parameter('state_topic').value, self.state_cb, 10)
        self.attitude_ok = False
        try:
            from px4_msgs.msg import VehicleAttitude
            from skybeta_flight_interface.topics import fmu_topic
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            topic = fmu_topic('out', 'vehicle_attitude', 'VehicleAttitude')
            self.create_subscription(VehicleAttitude, topic, self.attitude_cb, qos)
            self.attitude_ok = True
            self.get_logger().info(f'yaw offset from {topic}')
        except Exception as e:  # noqa: BLE001 - optional
            self.get_logger().warn(f'no PX4 attitude ({e}); yaw offset not measured')

        self.create_timer(0.5, self.report)
        self.get_logger().info(
            f'START arena ({self.mission.arena_start[0]:.3f}, '
            f'{self.mission.arena_start[1]:.3f}), nose '
            f'{self.mission.start_yaw_deg:.1f} deg. Anchoring on the first '
            f'odometry: the drone must be on START now.')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z, yaw_from_quat(msg.pose.pose.orientation))
        if self.tf is None:
            self.tf = ArenaToOdom(self.mission.arena_start,
                                  math.radians(self.mission.start_yaw_deg),
                                  (p.x, p.y), self.odom[3])
            self.anchor_z = p.z
            self.get_logger().info(
                f'anchored at odom ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) yaw '
                f'{math.degrees(self.odom[3]):.1f} deg')

    def state_cb(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            return
        if self.local0 is None and self.state.get('local_enu') and self.odom:
            self.local0 = list(self.state['local_enu'])
            self.odom0 = list(self.odom[:3])
            return
        le = self.state.get('local_enu')
        if self.local0 is None or not le or not self.odom:
            return
        # Frame rotation from motion: how PX4's local frame is turned relative
        # to the vision frame, for horizontal VECTORS - exactly what decides
        # where a velocity command goes. Accumulated as sum(dot), sum(cross)
        # over samples where both have moved > 0.3 m from the start.
        dv = (self.odom[0] - self.odom0[0], self.odom[1] - self.odom0[1])
        de = (le[0] - self.local0[0], le[1] - self.local0[1])
        nv, ne = math.hypot(*dv), math.hypot(*de)
        if nv > 0.3 and ne > 0.3:
            self.rot_dot += dv[0] * de[0] + dv[1] * de[1]
            self.rot_cross += dv[0] * de[1] - dv[1] * de[0]
            self.rot_n += 1
            self.scale_sum += ne / nv

    def attitude_cb(self, msg):
        if self.odom is None:
            return
        w, x, y, z = (float(v) for v in msg.q)
        yaw_px4_ned = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_from_odom_ned = wrap_pi(math.pi / 2.0 - self.odom[3])
        # Same convention as flight_interface: target_yaw_ned = odom yaw + offset
        self.offsets.append(wrap_pi(yaw_px4_ned - yaw_from_odom_ned))
        self.offsets = self.offsets[-400:]

    def report(self):
        if self.tf is None or self.odom is None:
            self.get_logger().info('waiting for odometry...')
            return
        a = self.tf.inverse_point(self.odom[:2])
        h = self.odom[2] - self.anchor_z
        s = self.line.project(a, self.s_hint, window=1.5)
        self.s_hint = s
        q = self.line.at(s)
        xt = math.dist(a, q)
        m = self.line.margin_at(s)
        clear = m - xt if m is not None else float('nan')
        if h > 0.5:     # carried at flight height: count it
            self.max_xtrack = max(self.max_xtrack, xt)
            self.min_clear = min(self.min_clear, clear)
        nose = math.degrees(self.tf.yaw_to_arena(self.odom[3]))
        line = (f'arena ({a[0]:5.2f}, {a[1]:5.2f}) h {h:4.2f} m nose {nose:6.1f} | '
                f'path s {s:5.2f} m, off-path {xt*100:4.1f} cm, planner margin '
                f'{(m or 0)*100:4.1f} cm -> left {clear*100:5.1f} cm')
        if self.state and self.local0 and self.state.get('local_enu'):
            le = self.state['local_enu']
            dv = math.dist(self.odom[:2], self.odom0[:2])
            de = math.dist(le[:2], self.local0[:2])
            line += (f' | from start: vision {dv:4.2f} m, EKF2 {de:4.2f} m; '
                     f'height vision {self.odom[2] - self.odom0[2]:+.2f} m, '
                     f'EKF2 {le[2] - self.local0[2]:+.2f} m')
        if self.rot_n:
            line += (f' | frame rotation {math.degrees(self.offset_from_motion()):+.1f} deg'
                     f' (n={self.rot_n})')
        if self.offsets:
            line += (f' | attitude heading diff '
                     f'{math.degrees(circular_mean(self.offsets[-20:])):+.1f} deg (info)')
        self.get_logger().info(line)

    def offset_from_motion(self):
        """odom_to_ned_yaw_offset_rad implied by the motion so far: minus the
        CCW angle from the vision displacement to the EKF2 displacement."""
        return -math.atan2(self.rot_cross, self.rot_dot)

    def summary(self):
        if self.tf is None or self.odom is None:
            return 'no odometry received'
        a = self.tf.inverse_point(self.odom[:2])
        closure = math.dist(a, self.mission.arena_start)
        lines = [
            f'closure: vision puts the drone {closure*100:.1f} cm from START '
            f'(only meaningful if it is back on the marker)',
            f'carried at >0.5 m: max off-path {self.max_xtrack*100:.1f} cm, '
            f'least margin left {self.min_clear*100:.1f} cm '
            f'(this is your hand, not the controller: a sanity check only)',
            f'nose drift: {math.degrees(self.tf.yaw_to_arena(self.odom[3])) - self.mission.start_yaw_deg:+.2f} deg '
            f'(back on START, facing east again, this should be ~0)',
        ]
        if self.rot_n >= 10:
            off = self.offset_from_motion()
            lines.append(
                f'odom_to_ned_yaw_offset_rad from motion = {off:+.4f} '
                f'({math.degrees(off):+.2f} deg, {self.rot_n} samples, EKF2/vision '
                f'distance ratio {self.scale_sum / self.rot_n:.2f}); configured '
                f'{self.configured_offset:+.4f}. Use THIS value, and only if it is '
                f'steady over a 1-2 m carry and above ~1 deg.')
        else:
            lines.append('frame rotation: not measured - carry the drone at least '
                         '0.5 m from where this started (needs motion).')
        if self.offsets:
            mean = circular_mean(self.offsets)
            lines.append(
                f'attitude heading difference (PX4 vs vision) = {math.degrees(mean):+.1f} '
                f'deg. Information only: it includes how the camera is mounted '
                f'(~180 deg is normal here) and is NOT the offset to configure.')
        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = PathflyCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n=== pathfly_check summary ===\n' + node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
