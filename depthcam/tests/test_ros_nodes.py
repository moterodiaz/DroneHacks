"""The four nodes under ROS 2 Humble, in one process: real rclpy, real messages, a real serial port.

Skipped where rclpy or px4_msgs is missing (the Windows dev venv). Run on a ROS 2 Humble host with
px4_msgs built, from ``depthcam/``:

    python3 -m pytest tests/test_ros_nodes.py -q

On drone 11, in the kit's container (``~/skybeta/host/run.sh shell``), after ``team_build.sh``:

    source /root/team_ws/install/setup.bash
    cd /root/team_ws/src/dronehacks_lidar
    ROS_DOMAIN_ID=101 python3 -m pytest tests/test_ros_nodes.py -q

These tests publish setpoints on ``/skybeta/setpoint/pose``. So in the kit's container they refuse
to run unless DDS is loopback-only (``SKYBETA_DDS_SCOPE=local``, the kit's default) and
``ROS_DOMAIN_ID`` differs from the drone's own (``DRONE_ID``). That way the running stack's
flight interface never sees them.

What each test drives:

* the Guard: a cuVSLAM odometry stream with one jump, and one stamped on another clock;
* the fusion, forbidden: odometry and setpoints must come out unchanged;
* the fusion, allowed: lidar fixes that say the drone is 0.2 m further north than cuVSLAM thinks.
  The follower's setpoint must reach /skybeta/setpoint/pose shifted back by 0.2 m. After a VO
  gap, ~/aligned must turn false;
* the localiser: a pseudo-terminal stands in for the MS200K's USB serial port, fed a synthetic
  byte stream of the course scanned from the launch mark, plus a fake PX4 VehicleOdometry. Its
  pose must land on the launch mark;
* the follower, on the shipped window route, on the launch mark with pose_fusion ready: before
  Offboard it must stream the takeoff pose (the kit enters Offboard only on a setpoint under 1 s
  old). Once /skybeta/state says Offboard it climbs to the route height. While
  /pose_fusion/aligned is false it holds with a zero twist. It stops when Offboard ends.

Course positions come from the map and rules files (AGENTS.md §3.7).
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

if "SKYBETA_DDS_SCOPE" in os.environ:                # in the kit's container, on a drone
    _scope, _domain = os.environ.get("SKYBETA_DDS_SCOPE"), os.environ.get("ROS_DOMAIN_ID", "")
    if _scope != "local" or not _domain or _domain == os.environ.get("DRONE_ID", ""):
        raise RuntimeError(
            "refusing to run: these tests publish /skybeta/setpoint/pose. Run them on loopback DDS "
            "(SKYBETA_DDS_SCOPE=local, now {!r}) and a domain other than the drone's (ROS_DOMAIN_ID={!r}, "
            "DRONE_ID={!r}), e.g.  ROS_DOMAIN_ID=101 python3 -m pytest tests/test_ros_nodes.py -q".format(
                _scope, _domain, os.environ.get("DRONE_ID")))

rclpy = pytest.importorskip("rclpy")
px4_msgs = pytest.importorskip("px4_msgs.msg")

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from pose_fusion.course_files import launch_mark_from_files, load_course_map  # noqa: E402
from pose_fusion.policy import LidarPolicy  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP = str(REPO_ROOT / "course" / "maps" / "course_v3_map.json")
RULES = str(REPO_ROOT / "course" / "maps" / "course_v3_rules.json")
REST = 0.07


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def params(**kw):
    out = []
    for k, v in kw.items():
        out.append(Parameter(k, value=v))
    return out


def spin_until(executor, predicate, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return False


def odom(node: Node, x: float, y: float, z: float = 0.0, yaw: float = 0.0, t=None) -> Odometry:
    msg = Odometry()
    msg.header.stamp = (node.get_clock().now() if t is None else t).to_msg()
    msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


class Collector:
    def __init__(self, node: Node, msg_type, topic: str):
        self.msgs = []
        node.create_subscription(msg_type, topic, self.msgs.append, 10)


# ---------------------------------------------------------------------------------------------- the Guard

def test_guard_withholds_a_jump_and_settles():
    from dronehacks_lidar.vo_guard_node import VoGuardNode

    guard = VoGuardNode(parameter_overrides=params(px4_odometry_topic="", status_topic=""))
    test = Node("test_guard")
    pub = test.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
    out = Collector(test, Odometry, "/vo_guard/odometry")
    tracking = Collector(test, Bool, "/vo_guard/tracking")
    ex = SingleThreadedExecutor()
    ex.add_node(guard)
    ex.add_node(test)
    try:
        assert spin_until(ex, lambda: pub.get_subscription_count() > 0, 5.0)
        for i in range(40):
            jump = 0.5 if i >= 20 else 0.0                       # 0.5 m in one frame at i = 20
            pub.publish(odom(test, 0.5 * i / 30 + jump, 0.0, 0.8))   # stamped now, so the clock rule passes it
            spin_until(ex, lambda: False, 1.0 / 30.0)            # in real time: about 30 Hz
        spin_until(ex, lambda: len(out.msgs) >= 30, 2.0)
        assert guard.guard.withheld["jump"] == 1 and guard.guard.withheld["clock"] == 0
        assert 4 <= guard.guard.withheld["settling"] <= 7        # 0.2 s of poses at 30 Hz, less any scheduling delay
        assert len(out.msgs) == 40 - 1 - guard.guard.withheld["settling"]
        assert out.msgs[0].pose.covariance[0] == pytest.approx(0.01)
        # ~/tracking: one Bool per pose judged, false from the jump through the settling
        spin_until(ex, lambda: len(tracking.msgs) >= 40, 2.0)
        assert [m.data for m in tracking.msgs].count(False) == 1 + guard.guard.withheld["settling"]
        assert tracking.msgs[19].data and not tracking.msgs[20].data and tracking.msgs[-1].data
    finally:
        guard.destroy_node()
        test.destroy_node()


def test_guard_withholds_poses_stamped_on_another_clock():
    from dronehacks_lidar.vo_guard_node import VoGuardNode

    guard = VoGuardNode(parameter_overrides=params(px4_odometry_topic="", status_topic=""))
    test = Node("test_guard_clock")
    pub = test.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
    out = Collector(test, Odometry, "/vo_guard/odometry")
    ex = SingleThreadedExecutor()
    ex.add_node(guard)
    ex.add_node(test)
    try:
        assert spin_until(ex, lambda: pub.get_subscription_count() > 0, 5.0)
        old = test.get_clock().now() - rclpy.duration.Duration(seconds=100)
        for i in range(10):
            stamp = old + rclpy.duration.Duration(nanoseconds=int(i * 1e9 / 30))
            pub.publish(odom(test, 0.5 * i / 30, 0.0, 0.8, t=stamp))
            spin_until(ex, lambda: False, 1.0 / 30.0)
        spin_until(ex, lambda: guard.guard.withheld["clock"] >= 10, 2.0)
        assert guard.guard.withheld["clock"] == 10 and not out.msgs
        assert guard.guard.last_stamp_lag_s == pytest.approx(100.0, abs=1.0)
    finally:
        guard.destroy_node()
        test.destroy_node()


# ---------------------------------------------------------------------------------------------- the fusion

FUSION_TOPICS = dict(input_topic="/vo_guard/odometry", output_topic="/pose_fusion/odometry",
                     setpoint_pose_in="/follower/setpoint/pose", setpoint_twist_in="/follower/setpoint/twist")


def setpoint(node: Node, x: float, y: float, z: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = node.get_clock().now().to_msg(), "odom"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    return msg                                               # orientation all-zero: yaw not commanded


def test_fusion_forbidden_passes_poses_and_setpoints_through():
    from dronehacks_lidar.pose_fusion_node import PoseFusionNode

    fusion = PoseFusionNode(parameter_overrides=params(lidar_policy="forbidden", **FUSION_TOPICS))
    test = Node("test_forbidden")
    vo_pub = test.create_publisher(Odometry, "/vo_guard/odometry", 10)
    sp_pub = test.create_publisher(PoseStamped, "/follower/setpoint/pose", 10)
    poses = Collector(test, Odometry, "/pose_fusion/odometry")
    sps = Collector(test, PoseStamped, "/skybeta/setpoint/pose")
    ex = SingleThreadedExecutor()
    ex.add_node(fusion)
    ex.add_node(test)
    try:
        assert spin_until(ex, lambda: vo_pub.get_subscription_count() > 0 and sp_pub.get_subscription_count() > 0, 5.0)
        vo_pub.publish(odom(test, 1.25, -0.5, 0.8, 0.3))
        sp_pub.publish(setpoint(test, 2.0, 1.0, 0.73))
        assert spin_until(ex, lambda: poses.msgs and sps.msgs, 3.0)
        assert poses.msgs[0].pose.pose.position.x == 1.25 and poses.msgs[0].pose.pose.position.y == -0.5
        p = sps.msgs[0].pose
        assert (p.position.x, p.position.y, p.position.z) == (2.0, 1.0, 0.73)
        assert (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) == (0.0, 0.0, 0.0, 0.0)
        assert fusion.fusion.stats.lidar_received == 0
    finally:
        fusion.destroy_node()
        test.destroy_node()


def test_fusion_allowed_shifts_the_setpoints_by_the_drift():
    from dronehacks_lidar.pose_fusion_node import PoseFusionNode

    mark = launch_mark_from_files(MAP, RULES)
    fusion = PoseFusionNode(parameter_overrides=params(
        lidar_policy="allowed", lidar_topic="/lidar_localizer/pose", lidar_frame_id="course",
        map_file=MAP, rules_file=RULES,
        odom_yaw_deg=0.0, odom_origin_x=0.0, odom_origin_y=0.0,   # the measured identity, passed explicitly
        **FUSION_TOPICS))
    test = Node("test_allowed")
    vo_pub = test.create_publisher(Odometry, "/vo_guard/odometry", 10)
    lidar_pub = test.create_publisher(PoseWithCovarianceStamped, "/lidar_localizer/pose", 10)
    sp_pub = test.create_publisher(PoseStamped, "/follower/setpoint/pose", 10)
    sps = Collector(test, PoseStamped, "/skybeta/setpoint/pose")
    ex = SingleThreadedExecutor()
    ex.add_node(fusion)
    ex.add_node(test)

    def tick_vo():
        vo_pub.publish(odom(test, 0.0, 0.0, 0.73))               # cuVSLAM: still on the launch mark

    def tick_lidar():
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = (test.get_clock().now() - rclpy.duration.Duration(nanoseconds=20_000_000)).to_msg()
        msg.header.frame_id = "course"
        msg.pose.pose.position.x, msg.pose.pose.position.y = mark.east, mark.north + 0.2   # really 0.2 m north
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02 ** 2
        cov[14] = cov[21] = cov[28] = 1.0e6
        cov[35] = math.radians(0.5) ** 2
        msg.pose.covariance = cov
        lidar_pub.publish(msg)

    test.create_timer(1.0 / 30.0, tick_vo)
    test.create_timer(0.1, tick_lidar)
    try:
        assert spin_until(ex, lambda: lidar_pub.get_subscription_count() > 0, 5.0)
        spin_until(ex, lambda: False, 5.0)                        # converge, and walk C_applied there
        sps.msgs.clear()
        sp_pub.publish(setpoint(test, 1.0, 0.0, 0.73))
        assert spin_until(ex, lambda: bool(sps.msgs), 2.0)
        p = sps.msgs[-1].pose.position
        assert p.x == pytest.approx(0.8, abs=0.03) and p.y == pytest.approx(0.0, abs=0.03) and p.z == 0.73
        assert fusion.fusion.stats.lidar_fused > 20
    finally:
        fusion.destroy_node()
        test.destroy_node()


def test_fusion_reports_aligned_and_drops_it_at_a_vo_gap():
    from dronehacks_lidar.pose_fusion_node import PoseFusionNode

    mark = launch_mark_from_files(MAP, RULES)
    fusion = PoseFusionNode(parameter_overrides=params(
        lidar_policy="allowed", lidar_topic="/lidar_localizer/pose", lidar_frame_id="course",
        map_file=MAP, rules_file=RULES,
        odom_yaw_deg=0.0, odom_origin_x=0.0, odom_origin_y=0.0,   # the measured identity, passed explicitly
        **FUSION_TOPICS))
    test = Node("test_aligned")
    vo_pub = test.create_publisher(Odometry, "/vo_guard/odometry", 10)
    lidar_pub = test.create_publisher(PoseWithCovarianceStamped, "/lidar_localizer/pose", 10)
    aligned = Collector(test, Bool, "/pose_fusion/aligned")
    vo_on = {"on": True}
    ex = SingleThreadedExecutor()
    ex.add_node(fusion)
    ex.add_node(test)

    def tick_vo():
        if vo_on["on"]:
            vo_pub.publish(odom(test, 0.0, 0.0, 0.73))

    def tick_lidar():
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = (test.get_clock().now() - rclpy.duration.Duration(nanoseconds=20_000_000)).to_msg()
        msg.header.frame_id = "course"
        msg.pose.pose.position.x, msg.pose.pose.position.y = mark.east, mark.north
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02 ** 2
        cov[14] = cov[21] = cov[28] = 1.0e6
        cov[35] = math.radians(0.5) ** 2
        msg.pose.covariance = cov
        lidar_pub.publish(msg)

    test.create_timer(1.0 / 30.0, tick_vo)
    test.create_timer(0.1, tick_lidar)
    try:
        assert spin_until(ex, lambda: lidar_pub.get_subscription_count() > 0, 5.0)
        spin_until(ex, lambda: False, 3.0)
        assert aligned.msgs and aligned.msgs[-1].data is True
        vo_on["on"] = False                                       # the Guard withholds for half a second
        spin_until(ex, lambda: False, 0.5)
        n = len(aligned.msgs)
        vo_on["on"] = True
        assert spin_until(ex, lambda: any(not m.data for m in aligned.msgs[n:]), 2.0)
        assert fusion.fusion.stats.vo_gaps == 1
        assert spin_until(ex, lambda: fusion.fusion.aligned(), 3.0)   # the next fused fix brings it back
    finally:
        fusion.destroy_node()
        test.destroy_node()


# ---------------------------------------------------------------------------------------------- the localiser

@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="needs a Linux pseudo-terminal")
def test_localizer_reads_a_serial_port_and_lands_on_the_launch_mark():
    pytest.importorskip("serial")
    import pty

    from course import ms200
    from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate
    from dronehacks_lidar.lidar_localizer_node import LidarLocalizerNode
    from pose_fusion.course_files import START_HEADING_NORTH

    mark = launch_mark_from_files(MAP, RULES)
    cmap = load_course_map(MAP)
    seconds = 8.0
    t = np.arange(0.0, seconds + 1.0, 0.01)
    traj = Trajectory(t, np.tile([mark.east, mark.north, REST], (len(t), 1)),
                      np.tile([0.0, 0.0, START_HEADING_NORTH], (len(t), 1)))
    scans = simulate(traj, Scene.from_map(cmap), LidarSpec(mount=(0.0, 0.0, 0.035)), 0.0, seconds, seed=2)
    reads = ms200.synthetic_stream(np.concatenate([s.t for s in scans]), np.concatenate([s.angle for s in scans]),
                                   np.concatenate([s.range for s in scans]), rate_hz=10.0)
    master, slave = pty.openpty()

    def feed():
        start = time.monotonic()
        for arrival, data in reads:
            delay = start + arrival - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            os.write(master, data)

    loc = LidarLocalizerNode(LidarPolicy.ALLOWED, parameter_overrides=params(
        port=os.ttyname(slave), map_file=MAP, rules_file=RULES, yaw_offset_deg=0.0, mount_x=0.0, mount_y=0.0,
        mount_z=0.035, rest_height_m=REST, slice_heights=[0.2],
        odom_yaw_deg=0.0))        # the measured identity (bearing 0 = north), passed explicitly
    test = Node("test_localizer")
    px4 = test.create_publisher(px4_msgs.VehicleOdometry, "/fmu/out/vehicle_odometry", 10)
    poses = Collector(test, PoseWithCovarianceStamped, "/lidar_localizer/pose")

    def tick_px4():
        m = px4_msgs.VehicleOdometry()
        m.q = [1.0, 0.0, 0.0, 0.0]
        m.position = [0.0, 0.0, 0.0]
        m.velocity = [0.0, 0.0, 0.0]
        m.angular_velocity = [0.0, 0.0, 0.0]
        m.velocity_frame = px4_msgs.VehicleOdometry.VELOCITY_FRAME_NED
        px4.publish(m)

    test.create_timer(0.02, tick_px4)
    ex = SingleThreadedExecutor()
    ex.add_node(loc)
    ex.add_node(test)
    writer = threading.Thread(target=feed, daemon=True)
    try:
        spin_until(ex, lambda: px4.get_subscription_count() > 0, 5.0)
        writer.start()
        assert spin_until(ex, lambda: len(poses.msgs) >= 5, seconds + 3.0), loc.localizer.counts
        last = poses.msgs[-1]
        assert last.header.frame_id == "course"
        assert math.hypot(last.pose.pose.position.x - mark.east, last.pose.pose.position.y - mark.north) < 0.03
        assert last.pose.covariance[0] > 0.0 and last.pose.covariance[35] > 0.0
        assert loc.localizer.counts["lost"] == 0
    finally:
        loc.destroy_node()
        test.destroy_node()
        os.close(master)
        os.close(slave)


# ---------------------------------------------------------------------------------------------- the follower

def test_follower_takes_off_on_offboard_and_stops_when_it_ends():
    from std_msgs.msg import Bool

    from dronehacks_lidar.follower_node import FollowerNode

    route_file = str(REPO_ROOT / "routes" / "course_v3_window.json")
    follower = FollowerNode(parameter_overrides=params(route_file=route_file, map_file=MAP, rules_file=RULES,
                                                       rest_height_m=REST, land_detected_topic="",
                                                       odom_yaw_deg=0.0, odom_origin_x=0.0, odom_origin_y=0.0))
    test = Node("test_follower")
    state_pub = test.create_publisher(String, "/skybeta/state", 10)
    ready_pub = test.create_publisher(Bool, "/pose_fusion/ready", 10)
    aligned_pub = test.create_publisher(Bool, "/pose_fusion/aligned", 10)
    pose_pub = test.create_publisher(Odometry, "/pose_fusion/odometry", 10)
    sps = Collector(test, PoseStamped, "/follower/setpoint/pose")
    twists = Collector(test, TwistStamped, "/follower/setpoint/twist")
    offboard = {"on": False}
    aligned = {"on": True}
    test.create_timer(0.2, lambda: state_pub.publish(String(data=json.dumps({"offboard_active": offboard["on"]}))))
    test.create_timer(0.1, lambda: ready_pub.publish(Bool(data=True)))
    test.create_timer(0.1, lambda: aligned_pub.publish(Bool(data=aligned["on"])))
    test.create_timer(0.02, lambda: pose_pub.publish(odom(test, 0.0, 0.0, 0.0)))   # resting on the launch mark
    ex = SingleThreadedExecutor()
    ex.add_node(follower)
    ex.add_node(test)
    try:
        # Before Offboard, on the start and ready: the takeoff pose streams, so the kit can enter Offboard at all
        assert spin_until(ex, lambda: len(sps.msgs) >= 5, 3.0)
        p = sps.msgs[-1].pose.position
        assert p.z == pytest.approx(0.8 - REST) and abs(p.x) < 1e-9 and abs(p.y) < 1e-9
        assert follower.mission.state == "WAIT_FOR_OFFBOARD"
        offboard["on"] = True
        n = len(sps.msgs)
        assert spin_until(ex, lambda: len(sps.msgs) >= n + 5, 3.0)
        p = sps.msgs[-1].pose.position
        assert p.z == pytest.approx(0.8 - REST) and abs(p.x) < 1e-9 and abs(p.y) < 1e-9
        assert follower.mission.state == "TAKEOFF"
        aligned["on"] = False                                    # a frame break: hold still by velocity
        assert spin_until(ex, lambda: len(twists.msgs) >= 5, 3.0)
        tw = twists.msgs[-1].twist
        assert (tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z) == (0.0, 0.0, 0.0, 0.0)
        assert follower.mission.holds == 1
        aligned["on"] = True
        n = len(sps.msgs)
        assert spin_until(ex, lambda: len(sps.msgs) >= n + 5, 3.0)          # back to the takeoff pose
        offboard["on"] = False
        spin_until(ex, lambda: follower.mission.state == "STOPPED", 3.0)
        n = len(sps.msgs)
        spin_until(ex, lambda: False, 0.5)
        assert follower.mission.state == "STOPPED" and len(sps.msgs) == n
    finally:
        follower.destroy_node()
        test.destroy_node()


# ---------------------------------------------------------------------------------------------- the launch file

def test_launch_file_has_no_default_policy():
    pytest.importorskip("launch")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("dronehacks_lidar")
    except Exception:  # noqa: BLE001
        pytest.skip("dronehacks_lidar is not installed (colcon build it first)")
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    source = PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_nav.launch.py"))
    args = {a.name: a for a in source.get_launch_description(None).get_launch_arguments()}
    assert args["lidar_policy"].default_value is None
    assert args["route_file"].default_value is not None                  # the packaged route
    assert args["auto_offboard_on_arm"].default_value is not None
    assert os.path.exists(os.path.join(share, "routes", "course_v3_window.json"))
