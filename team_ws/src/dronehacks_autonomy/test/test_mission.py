"""The mission state machine, alone and flown whole in the stand-in vehicle.

The small course here is made up (AGENTS.md section 3.7); the last test uses
the provisional published map.
"""

import dataclasses
import math
import unittest

import _paths
import numpy as np

from dronehacks_autonomy.planning import parse_map
from dronehacks_autonomy.planning.course_map import CourseMap, load_map
from dronehacks_autonomy.planning.follower import Follower
from dronehacks_autonomy.contract_map import load_contract_map
from dronehacks_autonomy.factory import flight_checks, prepare
from dronehacks_autonomy.frames import model_odom_frame
from dronehacks_autonomy.mission import (FOLLOW, GROUNDED, HELD, HOLD, LAND, LANDED, STOPPED, TAKEOFF, WAIT,
                                     MissionConfig, OdomPose)
from dronehacks_autonomy.vehicle_sim import VehicleSimConfig, fly, summary

# These tests fly the landing too, so they opt into it: land is off by default (px4.md:
# the drone holds over the pad for the pilot unless a landing API is contracted).
CONFIG = MissionConfig(rest_height_m=0.07, land=True)
COAST = dataclasses.replace(CONFIG, coast_max_s=4.0)


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

    def test_stale_pose_while_waiting_replaces_the_offered_takeoff_pose(self):
        # The kit's pose setpoint never expires, and Offboard is still granted up to 1 s
        # after the last team setpoint (flight_interface.py, auto_offboard.py). So once
        # the takeoff pose has been offered, a failed precheck must replace it with the
        # zero-velocity hold on the next tick, not just stop publishing.
        cmd = self.m.update(0.0, False, self.pose(), True, 0.0)                  # offered
        self.assertEqual(cmd.kind, "pose")
        cmd = self.m.update(0.6, False, self.pose(t=0.0), True, 0.0)            # 0.6 s old: stale
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy, cmd.vz), ("twist", 0.0, 0.0, 0.0))
        self.assertIn("no fresh pose", self.m.reason)
        self.assertEqual(self.m.update(0.7, False, self.pose(t=0.0), True, 0.0).kind, "twist")
        # prechecks recover: the takeoff pose is offered again, replacing the twist
        cmd = self.m.update(1.2, False, self.pose(t=1.2), True, 0.0)
        self.assertEqual(cmd.kind, "pose")
        self.assertAlmostEqual(cmd.z, 0.8 - 0.07)

    def test_a_failed_precheck_while_waiting_replaces_the_offered_takeoff_pose(self):
        # Same replacement for a precheck that fails with a fresh pose: the drone was
        # nudged off the launch mark after the takeoff pose was offered.
        self.m.update(0.0, False, self.pose(), True, 0.0)                       # offered
        cmd = self.m.update(0.1, False, self.pose(t=0.1, x=0.3), True, 0.0)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy, cmd.vz), ("twist", 0.0, 0.0, 0.0))
        self.assertIn("route start", self.m.reason)

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

    def test_coasting_is_off_by_default_so_a_dropout_sends_nothing(self):
        # contracts/ros-graph.md blocks tunnel traversal during vision loss
        self.assertEqual(CONFIG.coast_max_s, 0.0)
        log = fly(prep(l_course(), CONFIG).mission, cfg=VehicleSimConfig(dropout_s=(4.0, 6.5)))
        self.assertFalse(any("coasting" in r for r in log.reasons), log.reasons)
        self.assertTrue(any("sending nothing" in r for r in log.reasons), log.reasons)

    def test_a_long_dropout_mid_follow_is_flown_through(self):
        # the dark tube: 2.5 s without odometry would be a stop and a PX4 failsafe; it coasts on
        prepared = prep(l_course(), COAST)
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(4.0, 6.5)))
        self.assertIn("disarm", log.events)
        checks = flight_checks(prepared, log.pos)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertTrue(checks["on_pad"])
        self.assertTrue(any("coasting" in r for r in log.reasons), log.reasons)
        blind = log.pos[(log.t > 4.1) & (log.t < 6.4), :2]
        self.assertGreater(float(np.hypot(*(blind[-1] - blind[0]))), 0.8)      # it kept flying

    def test_after_a_frozen_dropout_the_frame_is_stitched_and_the_pad_reached(self):
        # cuVSLAM resumes from where it froze: without re-anchoring the rest of the flight
        # would be the coast's length off; the dead reckoning gives the jump
        prepared = prep(l_course(), COAST)
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(4.0, 5.5), freeze_on_dropout=True))
        m = prepared.mission
        self.assertEqual((m.coasts, len(m.stitches)), (1, 1))
        self.assertGreater(math.hypot(*m.offset), 0.5)
        checks = flight_checks(prepared, log.pos)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertTrue(checks["on_pad"])
        self.assertIn("disarm", log.events)

    def test_the_coast_dead_reckons_on_px4s_velocity_when_it_has_one(self):
        # a wildly wrong response model still stitches the frozen frame's jump on PX4's velocity
        prepared = prep(l_course(), dataclasses.replace(COAST, coast_response_s=3.0))
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(6.0, 7.5), freeze_on_dropout=True))
        checks = flight_checks(prepared, log.pos)
        self.assertEqual((len(prepared.mission.stitches), checks["wall_touches"], checks["on_pad"]), (1, 0, True))

    def test_a_coast_longer_than_its_limit_holds_still_by_velocity(self):
        prepared = prep(l_course(), dataclasses.replace(COAST, coast_max_s=0.5))
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(4.0, 6.0)))
        self.assertTrue(any("holding still" in r for r in log.reasons), log.reasons)
        self.assertTrue(flight_checks(prepared, log.pos)["on_pad"])

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


class CoastSafetyTest(unittest.TestCase):
    """A coast driven by hand: the follower's safety stops and the heading the dead reckoning uses."""

    def coasting(self, px4_yaw=0.0, odom_yaw=0.0):
        m = prep(l_course(), COAST).mission
        m._was_offboard = True
        m.state = FOLLOW
        m.follower = Follower(m.plan.trajectory, m.field, m.limits, m.follower_config)
        x, y = m.start_odom
        m.update(1.0, True, OdomPose(t=1.0, x=x, y=y, z=m.z_fly, yaw=odom_yaw, received=1.0), None, px4_yaw)
        return m

    def doctored(self, m, **fields):
        """Make the follower's next setpoints report *fields* (a wall ahead, off the path)."""
        real = m.follower.update
        m.follower.update = lambda *a, **k: dataclasses.replace(real(*a, **k), **fields)

    def test_no_safe_target_stops_the_coast_by_zero_velocity(self):
        m = self.coasting()
        self.doctored(m, flags=("no_safe_target",))
        for t in (1.3, 1.4, 1.5):
            cmd = m.update(t, True, None, None, 0.0)
            self.assertEqual((cmd.kind, cmd.vx, cmd.vy), ("twist", 0.0, 0.0))
        self.assertIn("no safe target", m.reason)
        self.assertEqual(m.state, FOLLOW)              # the pose coming back resumes the flight

    def test_off_the_path_while_coasting_holds_for_the_pilot(self):
        m = self.coasting()
        self.doctored(m, cross_track_m=m.config.abort_cross_track_m + 0.1)
        cmd = m.update(1.3, True, None, None, 0.0)
        self.assertEqual(m.state, HELD)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy), ("twist", 0.0, 0.0))
        x, y = m.start_odom                            # even with the pose back: held for the pilot
        cmd = m.update(1.4, True, OdomPose(t=1.4, x=x, y=y, z=m.z_fly, yaw=0.0, received=1.4), None, 0.0)
        self.assertEqual((m.state, cmd.kind), (HELD, "twist"))

    def test_the_dead_reckoning_turns_with_px4s_heading(self):
        # Vision froze while the drone was turning: PX4 now heads 90 deg left of where vision last
        # saw it. Its forward velocity is the odom frame's +y, not the frozen heading's +x. The
        # vision-PX4 gap measured at the last good pose (0.1 rad here) is kept.
        m = self.coasting(px4_yaw=0.0, odom_yaw=0.1)
        m.update(1.3, True, None, None, math.pi / 2, px4_velocity=(1.3, 1.0, 0.0))
        vx, vy = m._dr.vx, m._dr.vy
        self.assertAlmostEqual(vx, math.cos(math.pi / 2 + 0.1), places=9)
        self.assertAlmostEqual(vy, math.sin(math.pi / 2 + 0.1), places=9)

    def test_without_px4s_heading_the_last_odometry_heading_stands(self):
        m = self.coasting(px4_yaw=None, odom_yaw=0.0)
        m.update(1.3, True, None, None, None, px4_velocity=(1.3, 1.0, 0.0))
        self.assertAlmostEqual(m._dr.vx, 1.0, places=9)
        self.assertAlmostEqual(m._dr.vy, 0.0, places=9)


class TrackingGapTest(unittest.TestCase):
    def test_a_gap_on_the_route_does_not_throw_the_reference_ahead(self):
        cmap = load_contract_map(_paths.CONTRACT_MAP)
        m = prepare(cmap, MissionConfig(rest_height_m=0.07),
                    frame=model_odom_frame(cmap.launch.position, cmap.launch.heading)).mission
        m._was_offboard = True
        m.state = "FOLLOW"
        m.follower = Follower(m.plan.trajectory, m.field, m.limits, m.follower_config)
        x, y = m.start_odom
        here = OdomPose(t=1.0, x=x, y=y, z=m.z_fly, yaw=0.0)
        m.update(1.0, True, here, None, 0.0)
        tau = m.follower._tau
        self.assertIsNone(m.update(1.05, True, None, None, 0.0))
        # Short enough that the jump stays on a clear straight: a longer one lands behind a wall,
        # where the follower's wall check hides it.
        m.update(2.0, True, None, None, 0.0)
        cmd = m.update(2.05, True, dataclasses.replace(here, t=2.05), None, 0.0)
        self.assertNotIn("no_safe_target", m.last_setpoint.flags)
        self.assertLess(m.follower._tau - tau, 0.2)
        self.assertEqual(cmd.kind, "pose")


if __name__ == "__main__":
    unittest.main()
