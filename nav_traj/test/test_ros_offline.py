"""The real node and launch files, run against stand-in ROS modules (fake_ros.py): no ROS needed."""

import importlib
import importlib.util
import json
import math
import os
import unittest
from unittest import mock

import _paths
import fake_ros
import yaml

STEP_MAP = str(_paths.PACKAGE_ROOT / "maps" / "course_v3_step.yaml")

# A stand-in for the measured course->odom transform (contracts/frames.md), written for the
# STEP map's launch mark (position (-2.515, 0.914), facing north): R rotates by -pi/2 and
# o_odom is the course origin's odom reading, -R * launch_mark.
MEASURED = {"course_to_odom_yaw": -math.pi / 2, "course_to_odom_x": -0.914, "course_to_odom_y": -2.515}


def config_params():
    doc = yaml.safe_load((_paths.PACKAGE_ROOT / "config" / "traj_nav.example.yaml").read_text(encoding="utf-8"))
    return doc["mission"]["ros__parameters"]


class NodeTest(unittest.TestCase):
    def setUp(self):
        params = config_params()
        params["map_file"] = STEP_MAP
        params.update(MEASURED)
        params["rest_height_m"] = 0.07          # the config's .nan means "measure it"; stand in here
        self.handle = fake_ros.install(fake_ros.Handle(parameters=params))
        self.mod = importlib.import_module("dronehacks_traj.mission_node")
        self.node = self.mod.MissionNode()

    def tearDown(self):
        fake_ros.uninstall()

    def odom(self, t, x=0.0, y=0.0, z=0.0, yaw=0.0, vx_body=0.0):
        msg = fake_ros.Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.twist.twist.linear.x = vx_body
        return msg

    def feed(self, topic, msg):
        self.node.subscriptions[topic][1](msg)

    def tick(self):
        self.node.timers[0][1]()

    def test_wiring(self):
        n = self.node
        self.assertEqual(set(n.subscriptions), {"/vo_guard/odometry", "/skybeta/state",
                                                "/fmu/out/vehicle_land_detected_v1", "/fmu/out/vehicle_attitude_v1"})
        self.assertEqual(n.subscriptions["/vo_guard/odometry"][2].reliability, "best_effort")
        self.assertEqual(set(n.publishers), {"/skybeta/setpoint/pose", "/skybeta/setpoint/twist", "~/status"})
        self.assertAlmostEqual(n.timers[0][0], 1 / 50.0)
        self.assertTrue(any("route tunnel > gate_b > pad" in line for line in n.logged), n.logged)

    def test_publishes_only_once_the_prechecks_pass_then_takes_off(self):
        pose_pub = self.node.publishers["/skybeta/setpoint/pose"]
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": False})))
        self.feed("/vo_guard/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(pose_pub.sent, [])                     # no PX4 heading yet: the gate holds
        # PX4 attitude: facing the odom frame's +x (ENU yaw 0) = NED yaw 90 deg, q = (w, x, y, z) about +z down
        half = math.radians(90) / 2
        self.feed("/fmu/out/vehicle_attitude_v1", fake_ros.VehicleAttitude((math.cos(half), 0, 0, math.sin(half))))
        self.feed("/fmu/out/vehicle_land_detected_v1", fake_ros.VehicleLandDetected(True))
        self.tick()
        self.assertEqual(len(pose_pub.sent), 1)
        msg = pose_pub.sent[0]
        self.assertEqual(msg.header.frame_id, "odom")
        self.assertAlmostEqual(msg.pose.position.z, 0.8 - 0.07)
        self.assertAlmostEqual(msg.pose.orientation.w, 1.0)      # current heading, yaw 0
        # The pilot switches: Offboard
        self.handle.now_s += 0.1
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": True})))
        self.feed("/vo_guard/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "TAKEOFF")
        self.assertEqual(len(pose_pub.sent), 2)

    def test_stale_state_means_no_offboard(self):
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": True})))
        self.handle.now_s += 5.0                                  # /skybeta/state stopped arriving
        self.feed("/vo_guard/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "WAIT")

    def test_status_json(self):
        self.node.timers[1][1]()
        status = json.loads(self.node.publishers["~/status"].sent[-1].data)
        self.assertEqual(status["state"], "WAIT")
        self.assertEqual(status["route"], ["tunnel", "gate_b", "pad"])

    def test_body_velocity_is_rotated_into_odom(self):
        pose = self.mod.odom_to_pose(self.odom(1.5, yaw=math.pi / 2, vx_body=1.0))
        self.assertAlmostEqual(pose.t, 1.5)
        self.assertAlmostEqual(pose.vx, 0.0)
        self.assertAlmostEqual(pose.vy, 1.0)


class NodeConfigErrorTest(unittest.TestCase):
    def tearDown(self):
        fake_ros.uninstall()

    def make(self, **params):
        fake_ros.install(fake_ros.Handle(parameters=params))
        mod = importlib.import_module("dronehacks_traj.mission_node")
        return mod

    def test_missing_map_is_a_config_error(self):
        mod = self.make(rest_height_m=0.07, **MEASURED)
        with self.assertRaises(mod.ConfigError):
            mod.MissionNode()

    def test_missing_rest_height_is_a_config_error(self):
        mod = self.make(map_file=STEP_MAP, **MEASURED)
        with self.assertRaises(mod.ConfigError):
            mod.MissionNode()

    def test_missing_measured_transform_is_a_config_error(self):
        # contracts/frames.md: the course->odom transform is measured, with no defaults and no
        # derivation from the launch heading. Without it the node fails closed: it raises a
        # configuration error in __init__, before creating a single publisher or timer.
        mod = self.make(map_file=STEP_MAP, rest_height_m=0.07)
        with self.assertRaises(mod.ConfigError) as caught:
            mod.MissionNode()
        self.assertIn("course_to_odom", str(caught.exception))
        mod = self.make(map_file=STEP_MAP, rest_height_m=0.07,
                        course_to_odom_yaw=MEASURED["course_to_odom_yaw"], course_to_odom_x=-0.914)
        with self.assertRaises(mod.ConfigError) as caught:      # one of the three missing: still refuses
            mod.MissionNode()
        self.assertIn("course_to_odom", str(caught.exception))

    def test_int_for_a_float_parameter_is_refused(self):
        mod = self.make(map_file=STEP_MAP, rest_height_m=0.07, descent_mps=1, **MEASURED)
        with self.assertRaises(fake_ros.ParameterTypeError):
            mod.MissionNode()

    def test_main_returns_2_and_publishes_nothing(self):
        mod = self.make(map_file="/nonexistent.yaml", rest_height_m=0.07, **MEASURED)
        self.assertEqual(mod.main([]), 2)


def load_launch(name):
    path = _paths.PACKAGE_ROOT / "launch" / name
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class LaunchTest(unittest.TestCase):
    def tearDown(self):
        fake_ros.uninstall()

    def run_main(self, installed=(), **args):
        handle = fake_ros.install(fake_ros.Handle(installed=installed))
        handle.share["dronehacks_traj"] = str(_paths.PACKAGE_ROOT)
        desc = load_launch("traj_nav.launch.py").generate_launch_description()
        declared = {e.name: e for e in desc.entities if isinstance(e, fake_ros.DeclareLaunchArgument)}
        values = {k: d.default_value for k, d in declared.items()}
        values.update(args)
        ctx = type("Ctx", (), {"launch_configurations": values})()
        fi = next(e for e in desc.entities if isinstance(e, fake_ros.LaunchNode))
        opaque = next(e for e in desc.entities if isinstance(e, fake_ros.OpaqueFunction))
        return declared, fi, opaque.function(ctx)

    def test_defaults_fly_the_packaged_map_on_raw_cuvslam(self):
        # The Guard is off by default (contracts/ros-graph.md: cuVSLAM direct is the path).
        declared, fi, actions = self.run_main()
        self.assertEqual(declared["map_file"].default_value,
                         os.path.join(str(_paths.PACKAGE_ROOT), "maps", "course_v3_step.yaml"))
        self.assertTrue(os.path.isfile(declared["map_file"].default_value))
        self.assertEqual(fi.kwargs["package"], "skybeta_flight_interface")
        self.assertEqual(declared["auto_offboard_on_arm"].default_value, "true")
        self.assertEqual(declared["allow_mode_request"].default_value, "false")
        self.assertEqual(declared["guard_package"].default_value, "none")
        logs = [a.msg for a in actions if isinstance(a, fake_ros.LogInfo)]
        self.assertFalse(any("WARNING" in m for m in logs), logs)
        nodes = [a for a in actions if isinstance(a, fake_ros.LaunchNode)]
        self.assertEqual([n.kwargs["executable"] for n in nodes], ["mission"])
        self.assertEqual(nodes[0].kwargs["parameters"][1]["odometry_topic"], "/visual_slam/tracking/odometry")

    def test_auto_still_starts_the_guard_when_installed(self):
        _, _, actions = self.run_main(installed={"dronehacks_vo"}, guard_package="auto")
        nodes = {a.kwargs["executable"]: a for a in actions if isinstance(a, fake_ros.LaunchNode)}
        self.assertEqual(nodes["vo_guard"].kwargs["package"], "dronehacks_vo")
        self.assertEqual(nodes["vo_guard"].kwargs["parameters"][0]["output_topic"], "/vo_guard/odometry")
        self.assertEqual(nodes["mission"].kwargs["parameters"][1]["odometry_topic"], "/vo_guard/odometry")

    def test_auto_without_the_guard_warns_and_flies_raw(self):
        _, _, actions = self.run_main(guard_package="auto")
        logs = [a.msg for a in actions if isinstance(a, fake_ros.LogInfo)]
        self.assertTrue(any("WARNING dronehacks_vo is not installed" in m for m in logs), logs)
        nodes = [a for a in actions if isinstance(a, fake_ros.LaunchNode)]
        self.assertEqual(nodes[0].kwargs["parameters"][1]["odometry_topic"], "/visual_slam/tracking/odometry")

    def test_a_named_guard_that_is_missing_aborts_the_launch(self):
        with self.assertRaises(fake_ros.PackageNotFoundError):
            self.run_main(guard_package="dronehacks_vo")

    def test_competitor_launch_picks_the_packaged_map_and_no_guard(self):
        # The Guard is off by default (ros-graph.md); DRONEHACKS_GUARD=auto still requests it by hand.
        handle = fake_ros.install(fake_ros.Handle())
        handle.share["dronehacks_traj"] = str(_paths.PACKAGE_ROOT)
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("DRONEHACKS_MAP", None)
            os.environ.pop("DRONEHACKS_GUARD", None)
            inc = load_launch("traj_nav_competitor.launch.py").generate_launch_description().entities[0]
            raw = load_launch("traj_nav_competitor_raw.launch.py").generate_launch_description().entities[0]
        if not os.path.isfile("/root/team_ws/course_map.yaml"):
            self.assertTrue(inc.launch_arguments["map_file"].endswith(os.path.join("maps", "course_v3_step.yaml")))
        self.assertEqual(inc.launch_arguments["guard_package"], "none")
        self.assertEqual(raw.launch_arguments["guard_package"], "none")
        self.assertTrue(inc.source.path.endswith("traj_nav.launch.py"))


if __name__ == "__main__":
    unittest.main()
