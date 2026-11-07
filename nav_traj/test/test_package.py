"""Parameters, launch files and ROS message conversions. The ROS parts run only where rclpy exists."""

import ast
import dataclasses
import unittest

import _paths
import yaml

from dronehacks_traj.mission import MissionConfig

NODE_PARAMS = {"map_file", "rest_height_m", "v_max_mps", "odometry_topic", "state_topic", "land_detected_topic",
               "attitude_topic", "setpoint_pose_out", "setpoint_twist_out", "frame_id", "rate_hz",
               "state_timeout_s", "prefer"}


class ConfigTest(unittest.TestCase):
    def test_every_parameter_is_declared_with_the_right_type(self):
        doc = yaml.safe_load((_paths.PACKAGE_ROOT / "config" / "traj_nav.example.yaml").read_text(encoding="utf-8"))
        params = doc["mission"]["ros__parameters"]
        defaults = MissionConfig(rest_height_m=0.0)
        known = NODE_PARAMS | {f.name for f in dataclasses.fields(MissionConfig)}
        self.assertEqual(sorted(set(params) - known), [])
        for f in dataclasses.fields(MissionConfig):
            if f.name in params:
                # ROS 2 refuses an int for a float parameter (and the reverse)
                self.assertIs(type(params[f.name]), type(getattr(defaults, f.name)), f.name)
        MissionConfig(**{k: v for k, v in params.items() if k in {f.name for f in dataclasses.fields(MissionConfig)}})

    def test_node_declares_what_the_config_uses(self):
        src = (_paths.PACKAGE_ROOT / "dronehacks_traj" / "mission_node.py").read_text(encoding="utf-8")
        declared = {node.args[0].value for node in ast.walk(ast.parse(src))
                    if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "declare_parameter"
                    and node.args and isinstance(node.args[0], ast.Constant)}
        self.assertEqual(sorted(NODE_PARAMS - declared), [])


class LaunchFileTest(unittest.TestCase):
    def test_launch_files_parse_and_start_the_kit_interface_unchanged(self):
        for path in (_paths.PACKAGE_ROOT / "launch").glob("*.launch.py"):
            src = path.read_text(encoding="utf-8")
            tree = ast.parse(src, filename=str(path))
            # No lidar node, package or topic: every Node(...) names a non-lidar package and executable.
            for call in ast.walk(tree):
                if isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Node":
                    for kw in call.keywords:
                        if kw.arg in ("package", "executable") and isinstance(kw.value, ast.Constant):
                            self.assertNotIn("lidar", kw.value.value, path.name)
            for banned in ("dronehacks_lidar", "lidar_localizer", "lidar_policy", "pose_fusion", "/scan"):
                self.assertNotIn(banned, src, path.name)
        main = (_paths.PACKAGE_ROOT / "launch" / "traj_nav.launch.py").read_text(encoding="utf-8")
        for arg in ("max_altitude_m", "min_altitude_m", "max_speed_mps", "setpoint_rate_hz", "allow_mode_request",
                    "auto_offboard_on_arm", "odometry_topic"):
            self.assertIn(f'"{arg}"', main)
        self.assertIn('("allow_mode_request", "false"', main)
        self.assertIn('("auto_offboard_on_arm", "true"', main)


@unittest.skipUnless(_paths.HAVE_RCLPY, "needs ROS 2 (rclpy, nav_msgs, geometry_msgs)")
class RosConversionTest(unittest.TestCase):
    def test_odometry_and_setpoint_messages(self):
        import math

        from nav_msgs.msg import Odometry

        from dronehacks_traj.frames import yaw_to_quat
        from dronehacks_traj.mission import Command
        from dronehacks_traj.mission_node import odom_to_pose, pose_msg, twist_msg

        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 500_000_000
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1.0, 2.0, 0.7
        q = yaw_to_quat(math.pi / 2)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.twist.twist.linear.x = 1.0      # body forward, facing +y
        pose = odom_to_pose(msg)
        self.assertAlmostEqual(pose.t, 12.5)
        self.assertAlmostEqual(pose.vx, 0.0)
        self.assertAlmostEqual(pose.vy, 1.0)
        p = pose_msg(Command("pose", 1, 2, 0.73, None), "odom", msg.header.stamp)
        self.assertEqual((p.pose.orientation.w, p.pose.orientation.z), (0.0, 0.0))   # yaw not commanded
        t = twist_msg(Command("twist", vz=-0.3), "odom", msg.header.stamp)
        self.assertAlmostEqual(t.twist.linear.z, -0.3)


if __name__ == "__main__":
    unittest.main()
