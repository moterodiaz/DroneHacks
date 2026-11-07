"""The mission state machine, alone and flown whole in the stand-in vehicle.

The small course here is made up (AGENTS.md section 3.7); the last test uses
the provisional published map.
"""

import dataclasses
import math
import unittest

import _paths

from dronehacks_planner import parse_map
from dronehacks_traj.factory import flight_checks, prepare
from dronehacks_traj.mission import (FOLLOW, GROUNDED, HELD, HOLD, LAND, LANDED, STOPPED, TAKEOFF, WAIT,
                                     MissionConfig, OdomPose)
from dronehacks_traj.vehicle_sim import VehicleSimConfig, fly, summary

CONFIG = MissionConfig(rest_height_m=0.07)


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
        self.prep = prepare(l_course(), CONFIG)
        self.m = self.prep.mission

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
        m = prepare(l_course(), dataclasses.replace(CONFIG, px4_heading_gate_deg=0.0)).mission
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

    def test_rest_height_must_be_below_flight_height(self):
        with self.assertRaises(ValueError):
            prepare(l_course(), MissionConfig(rest_height_m=0.9))
        with self.assertRaises(ValueError):
            MissionConfig(rest_height_m=float("nan"))


class WholeFlightTest(unittest.TestCase):
    def test_nominal_flight(self):
        prep = prepare(l_course(), CONFIG)
        log = fly(prep.mission)
        s = summary(log)
        self.assertEqual(s["states"], [WAIT, TAKEOFF, FOLLOW, HOLD, LAND, LANDED])
        self.assertEqual(log.final_state, STOPPED)
        self.assertIn("disarm", log.events)
        checks = flight_checks(prep, log.pos)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertTrue(checks["on_pad"])

    def test_failed_heading_gate_means_no_offboard(self):
        prep = prepare(l_course(), CONFIG)
        log = fly(prep.mission, cfg=VehicleSimConfig(px4_yaw_offset_rad=math.radians(5)))
        self.assertNotIn("offboard", log.events)
        self.assertEqual(log.offboard_refused, "no team setpoint under 1 s old")
        self.assertNotIn("takeoff", log.events)
        self.assertIn("disarm_preflight", log.events)
        self.assertTrue(any("PX4's heading" in r for r in log.reasons), log.reasons)

    def test_pilot_takeover_stops_the_mission(self):
        prep = prepare(l_course(), CONFIG)
        log = fly(prep.mission, cfg=VehicleSimConfig(pilot_takeover_s=5.0))
        self.assertEqual(log.final_state, STOPPED)
        self.assertIn("pilot", log.final_reason)
        self.assertIsNone(prep.mission.update(6.0, True, OdomPose(6.0, 0, 0, 0.7, 0), False, 0.0))

    def test_odometry_dropout_mid_flight(self):
        prep = prepare(l_course(), CONFIG)
        log = fly(prep.mission, cfg=VehicleSimConfig(dropout_s=(5.0, 5.6)))
        self.assertIn("disarm", log.events)
        self.assertEqual(flight_checks(prep, log.pos)["wall_touches"], 0)

    def test_published_map(self):
        prep = prepare(str(_paths.PUBLISHED_MAP), CONFIG)
        log = fly(prep.mission)
        checks = flight_checks(prep, log.pos)
        self.assertEqual(summary(log)["states"], [WAIT, TAKEOFF, FOLLOW, HOLD, LAND, LANDED])
        self.assertEqual(checks["wall_touches"], 0)
        self.assertEqual(checks["openings"], {"tunnel": True, "window": True})
        self.assertTrue(checks["on_pad"])


if __name__ == "__main__":
    unittest.main()
