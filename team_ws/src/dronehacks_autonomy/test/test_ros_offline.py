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

STEP_MAP = str(_paths.CONTRACT_MAP)

# A stand-in for the measured course->odom transform (contracts/frames.md), written for the
# contract map's launch mark (position (-2.515, 0.914), facing north): R rotates by -pi/2 and
# o_odom is the course origin's odom reading, -R * launch_mark.
MEASURED = {"course_to_odom_yaw": -math.pi / 2, "course_to_odom_x": -0.914, "course_to_odom_y": -2.515}


def config_params():
    doc = yaml.safe_load((_paths.PACKAGE_ROOT / "config" / "autonomy.example.yaml").read_text(encoding="utf-8"))
    return doc["mission"]["ros__parameters"]


class NodeTest(unittest.TestCase):
    def setUp(self):
        params = config_params()
        params["map_file"] = STEP_MAP
        params.update(MEASURED)
        params["rest_height_m"] = 0.07          # the config's .nan means "measure it"; stand in here
        self.handle = fake_ros.install(fake_ros.Handle(parameters=params))
        self.mod = importlib.import_module("dronehacks_autonomy.mission_node")
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
        self.assertEqual(set(n.subscriptions), {"/visual_slam/tracking/odometry", "/skybeta/state",
                                                "/fmu/out/vehicle_land_detected_v1", "/fmu/out/vehicle_attitude_v1"})
        self.assertEqual(n.subscriptions["/visual_slam/tracking/odometry"][2].reliability, "best_effort")
        self.assertEqual(set(n.publishers), {"/skybeta/setpoint/pose", "/skybeta/setpoint/twist", "~/status"})
        self.assertAlmostEqual(n.timers[0][0], 1 / 20.0)
        self.assertTrue(any("route tunnel > pad" in line for line in n.logged), n.logged)

    def test_px4_velocity_in_the_body_heading_frame(self):
        # PX4 heading east (NED yaw 90 deg), flying north at 1 m/s: north is the drone's left
        half = math.radians(90.0) / 2
        forward, left = self.mod.body_velocity((math.cos(half), 0.0, 0.0, math.sin(half)), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(forward, 0.0, places=9)
        self.assertAlmostEqual(left, 1.0, places=9)
        self.assertIsNone(self.mod.body_velocity((1.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)))

    def test_publishes_only_once_the_prechecks_pass_then_takes_off(self):
        pose_pub = self.node.publishers["/skybeta/setpoint/pose"]
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": False})))
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
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
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "TAKEOFF")
        self.assertEqual(len(pose_pub.sent), 2)

    def test_stale_state_means_no_offboard(self):
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": True})))
        self.handle.now_s += 5.0                                  # /skybeta/state stopped arriving
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "WAIT")

    def take_off(self):
        """Get the real node to Offboard over its topics; returns with the mission in TAKEOFF."""
        half = math.radians(90) / 2
        self.feed("/fmu/out/vehicle_attitude_v1", fake_ros.VehicleAttitude((math.cos(half), 0, 0, math.sin(half))))
        self.feed("/fmu/out/vehicle_land_detected_v1", fake_ros.VehicleLandDetected(True))
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": True})))
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "TAKEOFF")

    def test_losing_state_in_flight_is_unknown_not_a_mode_exit(self):
        self.take_off()
        self.handle.now_s += 5.0                                  # /skybeta/state stopped arriving
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
        self.tick()
        self.assertEqual(self.node.mission.state, "TAKEOFF")

    def test_losing_state_on_the_descent_keeps_descending(self):
        # Stopping here lets the last twist expire into the pose above: the drone climbs back up.
        self.take_off()
        self.node.mission.state = "LAND"
        self.handle.now_s += 5.0
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s, z=0.5))
        self.feed("/fmu/out/vehicle_land_detected_v1", fake_ros.VehicleLandDetected(False))
        self.tick()
        self.assertEqual(self.node.mission.state, "LAND")
        self.assertLess(self.node.publishers["/skybeta/setpoint/twist"].sent[-1].twist.linear.z, 0.0)

    def test_a_stale_land_detector_reading_is_unknown(self):
        # take_off fed "landed" at the launch; the detector then stops. Seconds later, over the pad,
        # that old reading must reach the mission as unknown, not as a touchdown.
        self.take_off()
        self.node.mission.state = "LAND"
        self.node.mission._airborne_seen = True
        self.handle.now_s += 5.0
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": True})))
        self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s, z=0.5))
        self.tick()
        self.assertEqual(self.node.mission.state, "LAND")
        # A fresh reading still counts.
        self.feed("/fmu/out/vehicle_land_detected_v1", fake_ros.VehicleLandDetected(True))
        self.tick()
        self.assertEqual(self.node.mission.state, "LANDED")

    def test_a_real_offboard_exit_still_stops(self):
        self.take_off()
        self.feed("/skybeta/state", fake_ros.String(json.dumps({"offboard_active": False})))
        self.tick()
        self.assertEqual(self.node.mission.state, "STOPPED")

    def test_malformed_state_is_ignored(self):
        self.take_off()
        for payload in ("null", "[]", "3", '"x"', "{}", '{"offboard_active": "no"}', "not json"):
            with self.subTest(payload=payload):
                self.feed("/skybeta/state", fake_ros.String(payload))
                self.feed("/visual_slam/tracking/odometry", self.odom(self.handle.now_s))
                self.tick()
                self.assertEqual(self.node.mission.state, "TAKEOFF")

    def test_status_json(self):
        self.node.timers[1][1]()
        status = json.loads(self.node.publishers["~/status"].sent[-1].data)
        self.assertEqual(status["state"], "WAIT")
        self.assertEqual(status["route"], ["tunnel", "pad"])

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
        mod = importlib.import_module("dronehacks_autonomy.mission_node")
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

    def make_return(self, **params):
        mod = self.make(map_file=STEP_MAP, rest_height_m=0.07, **MEASURED, **params)
        return mod, mod.MissionNode

    def test_return_leg_needs_the_land_detector(self):
        # The land detector's flag is what starts the return, so turning it off cannot be allowed.
        mod, node = self.make_return(land=True, return_leg=True, return_height_m=2.2, land_detected_topic="")
        with self.assertRaisesRegex(mod.ConfigError, "land_detected_topic"):
            node()

    def test_return_leg_without_land_is_a_config_error(self):
        mod, node = self.make_return(return_leg=True, return_height_m=2.2)
        with self.assertRaisesRegex(mod.ConfigError, "needs land"):
            node()

    def test_return_parameters_reach_the_mission(self):
        _, node = self.make_return(land=True, return_leg=True, return_height_m=2.2, return_speed_mps=0.4)
        mission = node().mission
        self.assertTrue(mission.config.return_leg)
        self.assertAlmostEqual(mission.z_return, 2.2 - 0.07)
        self.assertAlmostEqual(mission.config.return_speed_mps, 0.4)

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

    def describe(self):
        handle = fake_ros.install(fake_ros.Handle())
        handle.share["dronehacks_autonomy"] = str(_paths.PACKAGE_ROOT)
        mod = load_launch("competitor.launch.py")
        desc = mod.generate_launch_description()
        declared = {e.name: e for e in desc.entities if isinstance(e, fake_ros.DeclareLaunchArgument)}
        nodes = {e.kwargs["executable"]: e for e in desc.entities if isinstance(e, fake_ros.LaunchNode)}
        return mod, declared, nodes

    def test_starts_the_kit_interface_and_the_mission(self):
        _, declared, nodes = self.describe()
        self.assertEqual(set(nodes), {"flight_interface", "mission"})
        self.assertEqual(nodes["flight_interface"].kwargs["package"], "skybeta_flight_interface")
        self.assertEqual(declared["auto_offboard_on_arm"].default_value, "true")
        self.assertEqual(declared["allow_mode_request"].default_value, "false")

    def test_defaults_to_the_packaged_contract_map(self):
        _, declared, _ = self.describe()
        if not os.path.isfile("/root/team_ws/course_map.yaml"):
            self.assertEqual(declared["map_file"].default_value, str(_paths.CONTRACT_MAP))
        self.assertTrue(os.path.isfile(declared["map_file"].default_value))

    def test_the_teams_files_win_when_present(self):
        mod, _, _ = self.describe()
        with mock.patch.object(mod.os.path, "isfile", return_value=True):
            self.assertEqual(mod.choose_map("share"), mod.TEAM_MAP)
            params = mod.mission_params("share", "the-map")
        self.assertEqual(params[1], mod.TEAM_PARAMS)
        self.assertEqual(params[-1], {"map_file": "the-map"})

    def test_without_measured_values_the_defaults_refuse_to_start(self):
        # The packaged yaml leaves the measured values at .nan, so a fresh install stays with the pilot.
        params = config_params()
        for key in ("rest_height_m", "course_to_odom_yaw", "course_to_odom_x", "course_to_odom_y"):
            self.assertTrue(math.isnan(params[key]), key)
        self.assertIs(params["land"], False)
        self.assertIs(params["use_window"], False)


if __name__ == "__main__":
    unittest.main()
