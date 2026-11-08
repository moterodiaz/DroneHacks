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

    def _following(self, mission):
        mission.update(0.0, False, self.pose(), True, 0.0)
        mission.update(0.02, True, self.pose(t=0.02), True, 0.0)
        mission.update(1.0, True, self.pose(t=1.0, z=0.73), False, 0.0)
        self.assertEqual(mission.state, FOLLOW)

    def test_stale_pose_while_following_coasts_on_by_velocity(self):
        self._following(self.m)
        t, x, y = 1.0, 0.0, 0.0
        for _ in range(100):                              # 2 s on the reference: the drone is at its setpoints
            t += 0.02
            cmd = self.m.update(t, True, self.pose(t=t, x=x, y=y, z=0.73), False, 0.0)
            x, y = cmd.x, cmd.y
        cmd = self.m.update(t + 0.6, True, self.pose(t=t, x=x, y=y, z=0.73), False, 0.0)
        self.assertEqual(cmd.kind, "twist")
        self.assertGreater(math.hypot(cmd.vx, cmd.vy), 0.2)
        self.assertGreater(cmd.vx, 0.0)                   # on along the route: north is odom +x here
        self.assertEqual(self.m.coasts, 1)
        self.assertIn("coasting", self.m.reason)

    def test_a_quiet_stream_is_a_dropout_within_the_pose_gap(self):
        self._following(self.m)
        last = OdomPose(t=1.0, x=0.0, y=0.0, z=0.73, yaw=0.0, received=1.0)
        self.assertEqual(self.m.update(1.0 + CONFIG.pose_gap_s - 0.01, True, last, False, 0.0).kind, "pose")
        self.assertEqual(self.m.update(1.0 + CONFIG.pose_gap_s + 0.02, True, last, False, 0.0).kind, "twist")

    def test_with_coasting_off_a_dropout_holds_still_by_velocity(self):
        m = prep(l_course(), dataclasses.replace(CONFIG, coast_max_s=0.0)).mission
        self._following(m)
        cmd = m.update(1.6, True, self.pose(t=1.0, z=0.73), False, 0.0)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy, cmd.vz), ("twist", 0.0, 0.0, 0.0))
        self.assertIn("holding still", m.reason)

    def test_held_brakes_before_holding_where_it_stopped(self):
        # at speed, a pose hold where the abort fired makes PX4 overshoot it and fly back
        self.m.update(0.0, False, self.pose(), True, 0.0)
        self.m.update(0.02, True, self.pose(t=0.02), True, 0.0)
        self.m.update(1.0, True, self.pose(t=1.0, z=0.73), False, 0.0)
        fast = OdomPose(t=1.1, x=0.1, y=0.5, z=0.73, yaw=0.0, vx=1.0, vy=0.0)
        cmd = self.m.update(1.1, True, fast, False, 0.0)
        self.assertEqual(self.m.state, HELD)
        self.assertEqual((cmd.kind, cmd.vx, cmd.vy), ("twist", 0.0, 0.0))          # braking
        stopped = OdomPose(t=1.6, x=0.45, y=0.5, z=0.73, yaw=0.0)
        cmd = self.m.update(1.6, True, stopped, False, 0.0)
        self.assertEqual(cmd.kind, "pose")
        self.assertAlmostEqual(cmd.x, 0.45)                                       # where it stopped
        self.assertEqual(self.m.update(1.7, True, self.pose(t=1.7), False, 0.0), cmd)

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

    def test_a_long_dropout_mid_follow_is_flown_through(self):
        # the dark tube: 2.5 s without odometry would be a stop and a PX4 failsafe; it coasts on
        prepared = prep(l_course(), CONFIG)
        log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(4.0, 6.5)))
        self.assertIn("disarm", log.events)
        self.assertEqual(flight_checks(prepared, log.pos)["wall_touches"], 0)
        self.assertTrue(flight_checks(prepared, log.pos)["on_pad"])
        self.assertTrue(any("coasting" in r for r in log.reasons), log.reasons)
        blind = log.pos[(log.t > 4.1) & (log.t < 6.4), :2]
        self.assertGreater(float(np.hypot(*(blind[-1] - blind[0]))), 0.8)      # it kept flying

    def test_after_a_frozen_dropout_the_frame_is_stitched_and_the_pad_reached(self):
        # cuVSLAM resumes from where it froze: without re-anchoring the rest of the flight
        # would be the coast's length off; the dead reckoning gives the jump
        prepared = prep(l_course(), CONFIG)
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
        errors = {}
        for px4 in (False, True):
            cfg = dataclasses.replace(CONFIG, coast_response_s=3.0)
            prepared = prep(l_course(), cfg)
            log = fly(prepared.mission, cfg=VehicleSimConfig(dropout_s=(6.0, 7.5), freeze_on_dropout=True,
                                                            px4_velocity=px4))
            checks = flight_checks(prepared, log.pos)
            errors[px4] = (len(prepared.mission.stitches), checks["wall_touches"], checks["on_pad"])
        self.assertEqual(errors[True], (1, 0, True), errors)

    def test_a_track_speed_flies_the_whole_course_faster(self):
        # Reference poses alone are held near 0.3 m/s by the governor (PX4 trails them by v/kp);
        # with a track speed FOLLOW flies the trajectory by velocity and keeps up with it.
        from dronehacks_traj.factory import for_track_speed
        from dronehacks_planner.limits import VehicleLimits
        limits, cfg = for_track_speed(0.8, VehicleLimits(), CONFIG)
        self.assertEqual((limits.v_max_mps, cfg.velocity_follow), (0.8, True))
        self.assertEqual(for_track_speed(3.0, VehicleLimits(), CONFIG)[0].v_max_mps, 1.0)   # PX4's cap
        fast = dataclasses.replace(VehicleLimits(), v_max_mps=2.0)
        self.assertEqual(for_track_speed(2.0, fast, CONFIG)[0].v_max_mps, 1.5)  # the interface's twist clamp
        times = {}
        for track in (0.0, 0.8):
            prepared = prep(l_course(), CONFIG, track_speed_mps=track)
            log = fly(prepared.mission)
            checks = flight_checks(prepared, log.pos)
            self.assertIn("disarm", log.events)
            self.assertEqual(checks["wall_touches"], 0, track)
            self.assertTrue(checks["on_pad"], track)
            times[track] = log.events["disarm"] - log.events["takeoff"]
        self.assertLess(times[0.8], 0.8 * times[0.0], times)
        self.assertTrue(any("following by velocity" in r for r in log.reasons))

    def test_the_window_2mps_line_flies_the_step_map_through_the_window(self):
        # config/flight_line_window_2mps.yaml, as the node applies it over the shipped config
        import yaml
        from dronehacks_planner.limits import VehicleLimits
        from dronehacks_planner.route import PlannerConfig
        line = yaml.safe_load((_paths.PACKAGE_ROOT / "config" / "flight_line_window_2mps.yaml")
                              .read_text(encoding="utf-8"))["mission"]["ros__parameters"]
        cfg = dataclasses.replace(CONFIG, twist_speed_max_mps=line["twist_speed_max_mps"])
        prepared = prep(_paths.PACKAGE_ROOT / "maps" / "course_v3_step.yaml", cfg,
                        limits=dataclasses.replace(VehicleLimits(), v_max_mps=line["v_max_mps"]),
                        planner=dataclasses.replace(PlannerConfig(), prefer=tuple(line["prefer"])),
                        track_speed_mps=line["track_speed_mps"])
        self.assertEqual(prepared.plan.route.goals, ["cp2_gate_a", "tunnel", "window", "pad"])
        self.assertEqual(prepared.v_max_mps, 2.0)       # verified at 2 m/s, not planned slower
        # PX4 at MPC_XY_VEL_MAX 2.0 and the interface at max_speed_mps 2.0 (the line needs both)
        log = fly(prepared.mission, dataclasses.replace(VehicleLimits(), v_max_mps=2.0),
                  VehicleSimConfig(twist_speed_cap_mps=2.0))
        checks = flight_checks(prepared, log.pos)
        self.assertIn("disarm", log.events)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertEqual(checks["openings"], {"cp2_gate_a": True, "tunnel": True, "window": True})
        self.assertTrue(checks["on_pad"])

    def test_a_plan_that_fails_its_checks_at_a_speed_is_planned_slower_not_refused(self):
        # the 2026-10-27 STEP map's window route over the boulders breaks the jerk limit at 1.5 m/s
        # and up (the 2026-11-07 layout's routes verify at every speed, so the old map is kept as a
        # fixture for this): refusing would leave the node silent, so the kit never grants Offboard
        from dronehacks_planner.limits import VehicleLimits
        from dronehacks_planner.route import PlannerConfig
        from dronehacks_traj.factory import prepare
        cmap = dataclasses.replace(load_map(_paths.HERE / "fixtures" / "course_v3_step_2026-10-27.yaml"), boulders=())
        prepared = prep(cmap, CONFIG, limits=dataclasses.replace(VehicleLimits(), v_max_mps=2.0),
                        planner=dataclasses.replace(PlannerConfig(), prefer=("window",)))
        self.assertEqual(prepared.plan.route.goals, ["tunnel", "window", "pad"])
        self.assertLess(prepared.v_max_mps, 2.0)
        self.assertIn("no plan verified at 2 m/s", prepared.speed_note)

    def test_a_velocity_follow_keeps_a_hold_pose_under_the_drone(self):
        # if the twists stop, the interface holds its last pose: it must be where the drone is,
        # never the takeoff pose behind the walls
        prepared = prep(l_course(), CONFIG, track_speed_mps=0.8)
        m = prepared.mission
        m.update(0.0, False, OdomPose(0.0, 0, 0, 0, 0), True, 0.0)
        m.update(0.02, True, OdomPose(0.02, 0, 0, 0, 0), True, 0.0)
        m.update(1.0, True, OdomPose(1.0, 0, 0, 0.73, 0), False, 0.0)
        cmds = [m.update(1.0 + k * 0.02, True, OdomPose(1.0 + k * 0.02, 0.01 * k, 0, 0.73, 0), False, 0.0)
                for k in range(1, 60)]
        self.assertTrue(all(c.kind == "twist" for c in cmds))
        holds = [c for c in cmds if c.fallback is not None]
        self.assertGreaterEqual(len(holds), 2)
        last = holds[-1]
        self.assertLess(abs(last.fallback[0] - 0.01 * (cmds.index(last) + 1)), 0.05)
        self.assertAlmostEqual(last.fallback[2], m.z_fly)

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
