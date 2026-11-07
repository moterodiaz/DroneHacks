"""The mission state machine, alone and flown whole in the stand-in vehicle.

The small course here is made up (AGENTS.md section 3.7); the last test uses
the provisional published map.
"""

import dataclasses
import math
import unittest

import _paths
import numpy as np

from dronehacks_planner import parse_map
from dronehacks_planner.course_map import CourseMap, load_map
from dronehacks_traj.factory import flight_checks, prepare
from dronehacks_traj.frames import model_odom_frame
from dronehacks_traj.mission import (FOLLOW, GROUNDED, HELD, HOLD, LAND, LANDED, STOPPED, TAKEOFF, WAIT,
                                     MissionConfig, OdomPose)
from dronehacks_traj.vehicle_sim import VehicleSimConfig, fly, summary

# These tests fly the landing too, so they opt into it: land is off by default (px4.md:
# the drone holds over the pad for the pilot unless a landing API is contracted).
CONFIG = MissionConfig(rest_height_m=0.07, land=True)


def prep(map_source, config, **kw):
    """prepare() with the offline model's odom frame, which starts at the launch pose
    (frames.model_odom_frame). The node's frame is measured; nothing derives it."""
    cmap = map_source if isinstance(map_source, CourseMap) else load_map(map_source)
    return prepare(cmap, config, frame=model_odom_frame(cmap.launch.position, cmap.launch.heading), **kw)


def l_course():
    """A 1.2 m wide L: north 4 m from the launch, then east to a pad. Launch facing north."""
    segs = [((-1.0, -0.6), (-1.0, 4.6)), ((0.2, -0.6), (0.2, 3.4)), ((-1.0, 4.6), (4.0, 4.6)),
            ((0.2, 3.4), (4.0, 3.4)), ((-1.0, -0.6), (0.2, -0.6)), ((4.0, 3.4), (4.0, 4.6))]
    doc = {
        "origin": [0.0, 0.0], "slice_height": 0.8,
        "launch": {"position": [-0.4, 0.0], "heading": [0.0, 1.0]},
        "segments": [{"start": list(a), "end": list(b), "thickness": 0.1} for a, b in segs],
        "goals": [{"name": "pad", "shape": "disc", "centre": [3.2, 4.0], "params": {"radius": 0.5}}],
    }
    return parse_map(doc, source="l_course")


class StateMachineTest(unittest.TestCase):
    def setUp(self):
        self.prepared = prep(l_course(), CONFIG)
        self.m = self.prepared.mission

    def pose(self, t=0.0, x=0.0, y=0.0, z=0.0, yaw=0.0):
        return OdomPose(t=t, x=x, y=y, z=z, yaw=yaw)

    def test_nothing_without_a_pose(self):
        self.assertIsNone(self.m.update(0.0, False, None, True, 0.0))
        self.assertIn("pose", self.m.reason)

    def test_nothing_when_off_the_start(self):
        self.assertIsNone(self.m.update(0.0, False, self.pose(x=0.3), True, 0.0))
        self.assertIn("route start", self.m.reason)

    def test_nothing_when_turned(self):
        self.assertIsNone(self.m.update(0.0, False, self.pose(yaw=math.radians(10)), True, math.radians(10)))
        self.assertIn("launch heading", self.m.reason)

    def test_px4_heading_gate(self):
        self.assertIsNone(self.m.update(0.0, False, self.pose(), True, None))
        self.assertIsNone(self.m.update(0.0, False, self.pose(), True, math.radians(3)))
        cmd = self.m.update(0.0, False, self.pose(), True, math.radians(1))
        self.assertEqual(cmd.kind, "pose")
        self.assertAlmostEqual(cmd.z, 0.8 - 0.07)
        self.assertEqual(self.m.state, WAIT)

    def test_gate_off(self):
        m = prep(l_course(), dataclasses.replace(CONFIG, px4_heading_gate_deg=0.0)).mission
        self.assertIsNotNone(m.update(0.0, False, self.pose(), True, None))

    def test_offboard_with_a_stale_pose_stays_grounded(self):
        self.assertIsNotNone(self.m.update(0.0, False, self.pose(), True, 0.0))   # takeoff pose offered
        cmd = self.m.update(1.0, True, self.pose(t=0.0), True, 0.0)               # 1 s old: stale
        self.assertEqual(self.m.state, GROUNDED)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy, cmd.vz), ("twist", 0.0, 0.0, 0.0))
        self.assertIsNone(self.m.update(1.1, False, self.pose(t=1.1), True, 0.0))  # Offboard ends
        self.assertEqual(self.m.state, STOPPED)

    def test_takeoff_then_follow(self):
        self.m.update(0.0, False, self.pose(), True, 0.0)
        cmd = self.m.update(0.02, True, self.pose(t=0.02), True, 0.0)
        self.assertEqual((self.m.state, cmd.kind), (TAKEOFF, "pose"))
        cmd = self.m.update(1.0, True, self.pose(t=1.0, z=0.70), False, 0.0)
        self.assertEqual(self.m.state, FOLLOW)
        self.assertEqual(cmd.kind, "pose")

    def test_far_off_the_path_holds_for_the_pilot(self):
        self.m.update(0.0, False, self.pose(), True, 0.0)
        self.m.update(0.02, True, self.pose(t=0.02), True, 0.0)
        self.m.update(1.0, True, self.pose(t=1.0, z=0.73), False, 0.0)
        cmd = self.m.update(1.1, True, self.pose(t=1.1, x=0.1, y=0.5, z=0.73), False, 0.0)
        self.assertEqual(self.m.state, HELD)
        self.assertAlmostEqual(cmd.x, 0.1)
        self.assertAlmostEqual(cmd.y, 0.5)
        self.assertEqual(self.m.update(1.2, True, self.pose(t=1.2), False, 0.0), cmd)

    def test_stale_pose_in_the_air_sends_nothing(self):
        self.m.update(0.0, False, self.pose(), True, 0.0)
        self.m.update(0.02, True, self.pose(t=0.02), True, 0.0)
        self.assertIsNone(self.m.update(2.0, True, self.pose(t=0.5, z=0.5), False, 0.0))
        self.assertEqual(self.m.state, TAKEOFF)

    def test_stale_pose_while_landing_keeps_descending(self):
        self.m.update(0.0, False, self.pose(), True, 0.0)
        self.m.update(0.02, True, self.pose(t=0.02), True, 0.0)       # Offboard: TAKEOFF
        self.m.state = LAND                                             # skip ahead to the descent
        cmd = self.m.update(2.0, True, self.pose(t=0.5, z=0.4), False, 0.0)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy), ("twist", 0.0, 0.0))
        self.assertAlmostEqual(cmd.vz, -CONFIG.descent_mps)
        self.assertIn("no fresh pose", self.m.reason)
        cmd = self.m.update(2.1, True, None, True, 0.0)                 # PX4: landed
        self.assertEqual(self.m.state, LANDED)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy), ("twist", 0.0, 0.0))
        self.assertAlmostEqual(cmd.vz, -0.3 * CONFIG.descent_mps)

    def test_rest_height_must_be_below_flight_height(self):
        with self.assertRaises(ValueError):
            prep(l_course(), MissionConfig(rest_height_m=0.9))
        with self.assertRaises(ValueError):
            MissionConfig(rest_height_m=float("nan"))

    def test_land_defaults_to_false(self):
        # px4.md blocks landing until a landing API is contracted: the default holds over
        # the pad for the pilot.
        self.assertIs(MissionConfig(rest_height_m=0.07).land, False)


class WholeFlightTest(unittest.TestCase):
    def test_nominal_flight(self):
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission)
        s = summary(log)
        self.assertEqual(s["states"], [WAIT, TAKEOFF, FOLLOW, HOLD, LAND, LANDED])
        self.assertEqual(log.final_state, STOPPED)
        self.assertIn("disarm", log.events)
        checks = flight_checks(prepared, log.pos)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertTrue(checks["on_pad"])

    def test_failed_heading_gate_means_no_offboard(self):
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission, cfg=VehicleSimConfig(px4_yaw_offset_rad=math.radians(5)))
        self.assertNotIn("offboard", log.events)
        self.assertEqual(log.offboard_refused, "no team setpoint under 1 s old")
        self.assertNotIn("takeoff", log.events)
        self.assertIn("disarm_preflight", log.events)
        self.assertTrue(any("PX4's heading" in r for r in log.reasons), log.reasons)

    def test_pilot_takeover_stops_the_mission(self):
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission, cfg=VehicleSimConfig(pilot_takeover_s=5.0))
        self.assertEqual(log.final_state, STOPPED)
        self.assertIn("pilot", log.final_reason)
        self.assertIsNone(prepared.mission.update(6.0, True, OdomPose(6.0, 0, 0, 0.7, 0), False, 0.0))

    def test_odometry_dropout_mid_flight(self):
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(5.0, 5.6)))
        self.assertIn("disarm", log.events)
        self.assertEqual(flight_checks(prepared, log.pos)["wall_touches"], 0)

    def test_odometry_dropout_while_landing_never_climbs(self):
        # If the twist stopped, the interface would expire it into its last pose, the pad at
        # flight height, and the drone would climb (review of #26 at a616fdf, mission.py:187-189).
        nominal = fly(prep(l_course(), CONFIG).mission)
        t_land = next(t for t, s in zip(nominal.t, nominal.state) if s == LAND)
        start = t_land + 0.5
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(start, start + 2.0)))
        z = log.pos[log.t >= t_land, 2]
        climb = float((z - np.minimum.accumulate(z)).max())          # height regained since LAND began
        self.assertLess(climb, 0.02)
        self.assertIn("disarm", log.events)
        self.assertTrue(flight_checks(prepared, log.pos)["on_pad"])
        self.assertTrue(any("no fresh pose" in r and "straight down" in r for r in log.reasons), log.reasons)

    def test_the_default_holds_over_the_pad_for_the_pilot(self):
        # land defaults to False (px4.md): the drone never descends; it holds over the pad
        # until the pilot takes it back.
        prepared = prep(l_course(), MissionConfig(rest_height_m=0.07))
        log = fly(prepared.mission, cfg=VehicleSimConfig(pilot_takeover_s=30.0))
        self.assertEqual(summary(log)["states"], [WAIT, TAKEOFF, FOLLOW, HOLD, STOPPED])
        self.assertIn("pilot_takeover", log.events)
        self.assertNotIn("touchdown", log.events)
        self.assertTrue(flight_checks(prepared, log.pos)["on_pad"])

    def test_published_map(self):
        prepared = prep(str(_paths.PUBLISHED_MAP), CONFIG)
        log = fly(prepared.mission)
        checks = flight_checks(prepared, log.pos)
        self.assertEqual(summary(log)["states"], [WAIT, TAKEOFF, FOLLOW, HOLD, LAND, LANDED])
        self.assertEqual(checks["wall_touches"], 0)
        self.assertEqual(checks["openings"], {"tunnel": True, "gate_b": True})
        self.assertTrue(checks["on_pad"])


if __name__ == "__main__":
    unittest.main()
