"""The return leg: touch the pad, stay armed, fly back at height, land by the launch mark."""

import dataclasses
import math
import unittest

import _paths
import numpy as np
import yaml
from synthetic_courses import random_course

from dronehacks_autonomy.contract_map import from_contract, load_contract_map
from dronehacks_autonomy.dry_run import RETURN_RADIUS_M, main as dry_run
from dronehacks_autonomy.factory import flight_checks, prepare
from dronehacks_autonomy.frames import model_odom_frame
from dronehacks_autonomy.mission import (HELD, HOLD, LAND, LANDED, RETURN_CLIMB, RETURN_TRANSIT, STOPPED,
                                         MissionConfig, OdomPose)
from dronehacks_autonomy.vehicle_sim import VehicleSimConfig, fly, summary

CONFIG = MissionConfig(rest_height_m=0.07, land=True, return_leg=True, return_height_m=2.2)
PUBLISHED = yaml.safe_load(_paths.CONTRACT_MAP.read_text(encoding="utf-8"))
RUN_LIMIT_S = 180.0


def prep(doc, config=CONFIG):
    cmap = from_contract(doc)
    return prepare(cmap, config, frame=model_odom_frame(cmap.launch.position, cmap.launch.heading))


def fly_it(doc, config=CONFIG, sim=VehicleSimConfig()):
    prepared = prep(doc, config)
    log = fly(prepared.mission, cfg=sim)
    touch = log.pos[int(np.searchsorted(log.t, log.events["touchdown"]))] if "touchdown" in log.events else None
    return prepared, log, flight_checks(prepared, log.pos, pad_touch_odom=touch)


class FlownTest(unittest.TestCase):
    def assert_return(self, doc, sim=VehicleSimConfig()):
        prepared, log, checks = fly_it(doc, sim=sim)
        s = summary(log)
        self.assertEqual(log.final_state, STOPPED, log.final_reason)
        self.assertIn("done: landed", log.final_reason)
        self.assertEqual(s["states"][-5:], [RETURN_CLIMB, RETURN_TRANSIT, HOLD, LAND, LANDED])
        self.assertEqual(checks["wall_touches"], 0, checks)
        self.assertTrue(all(checks["openings"].values()), checks)
        self.assertTrue(checks["on_pad"], "touched down off the pad")
        self.assertLessEqual(checks["launch_miss_m"], RETURN_RADIUS_M)
        # Stayed armed through the pad: the only disarm follows the second landing.
        self.assertGreater(log.events["last_touchdown"], log.events["touchdown"])
        self.assertAlmostEqual(log.events["disarm"] - log.events["last_touchdown"], sim.disarm_land_s, delta=0.05)
        self.assertLess(log.events["disarm"], RUN_LIMIT_S)
        self.assertGreater(float(np.max(log.pos[:, 2])), prepared.mission.z_return - 0.15)
        return log

    def test_published(self):
        self.assert_return(PUBLISHED)

    def test_random_courses(self):
        for seed in range(6):
            with self.subTest(seed=seed):
                self.assert_return(random_course(seed))

    def test_slow_land_detector(self):
        # PX4's real detector is slower than the model's guess; the window runs from its flag.
        self.assert_return(PUBLISHED, VehicleSimConfig(land_detect_s=1.5))

    def test_vision_dropout_on_the_way_back(self):
        prepared = prep(PUBLISHED)
        probe = fly(prepared.mission)
        climb = probe.t[probe.state.index(RETURN_TRANSIT)]
        self.assert_return(PUBLISHED, VehicleSimConfig(dropout_s=(float(climb) + 0.5, float(climb) + 1.0)))

    def test_off_by_default_nothing_changes(self):
        config = dataclasses.replace(CONFIG, return_leg=False)
        _, log, checks = fly_it(PUBLISHED, config)
        states = summary(log)["states"]
        self.assertNotIn(RETURN_CLIMB, states)
        self.assertEqual(log.events["touchdown"], log.events["last_touchdown"])
        self.assertTrue(checks["on_pad"])

    def test_pilot_takeover_on_the_way_back_stops_for_good(self):
        prepared = prep(PUBLISHED)
        probe = fly(prepared.mission)
        t = float(probe.t[probe.state.index(RETURN_TRANSIT)]) + 1.0
        prepared = prep(PUBLISHED)
        log = fly(prepared.mission, cfg=VehicleSimConfig(pilot_takeover_s=t))
        self.assertEqual(log.final_state, STOPPED)
        self.assertIn("pilot", log.final_reason)

    def test_dry_run_with_return(self):
        self.assertEqual(dry_run([str(_paths.CONTRACT_MAP), "--rest-height", "0.07", "--return-height", "2.2"]), 0)


class ConfigTest(unittest.TestCase):
    def test_needs_land(self):
        with self.assertRaisesRegex(ValueError, "needs land"):
            prep(PUBLISHED, dataclasses.replace(CONFIG, land=False))

    def test_needs_a_height_above_flight_height(self):
        with self.assertRaisesRegex(ValueError, "return_height_m"):
            prep(PUBLISHED, dataclasses.replace(CONFIG, return_height_m=0.0))
        with self.assertRaisesRegex(ValueError, "return_height_m"):
            prep(PUBLISHED, dataclasses.replace(CONFIG, return_height_m=0.8))

    def test_needs_a_speed(self):
        with self.assertRaisesRegex(ValueError, "return_speed_mps"):
            prep(PUBLISHED, dataclasses.replace(CONFIG, return_speed_mps=0.0))

    def test_needs_a_lag(self):
        # With no lag the reference, which starts on the drone over the pad, can never move ahead.
        with self.assertRaisesRegex(ValueError, "return_lag_m"):
            prep(PUBLISHED, dataclasses.replace(CONFIG, return_lag_m=0.0))
        prep(PUBLISHED, dataclasses.replace(CONFIG, return_leg=False, return_lag_m=0.0))   # unused: fine


class StateTest(unittest.TestCase):
    """Driven by hand, to reach the cases a clean simulated flight does not."""

    def at_pad(self, airborne=True):
        m = prep(PUBLISHED).mission
        m._was_offboard = True
        m.state = LAND
        if airborne:                       # PX4 has said "not landed" since takeoff
            m.update(0.5, True, None, False, 0.0)
        return m

    def pose(self, m, t, xy=None, z=0.0):
        x, y = m.end_odom if xy is None else xy
        return OdomPose(t=t, x=x, y=y, z=z, yaw=0.0)

    def test_the_landed_flag_commands_a_climb_at_once(self):
        m = self.at_pad()
        cmd = m.update(1.0, True, self.pose(m, 1.0), True, 0.0)
        self.assertEqual(m.state, RETURN_CLIMB)
        self.assertEqual(cmd.kind, "pose")
        self.assertAlmostEqual(cmd.z, m.z_return)

    def test_the_climb_starts_even_with_a_stale_pose(self):
        # The window is short: a vision gap at touchdown must not lose it.
        m = self.at_pad()
        cmd = m.update(5.0, True, None, True, 0.0)
        self.assertEqual(m.state, RETURN_CLIMB)
        self.assertAlmostEqual(cmd.z, m.z_return)

    def test_a_landed_left_over_from_the_launch_is_not_a_touchdown(self):
        # The land detector went quiet after its launch reading: over the pad at flight height it
        # still says landed. That must not start the return climb.
        m = self.at_pad(airborne=False)
        cmd = m.update(1.0, True, self.pose(m, 1.0, z=m.z_fly), True, 0.0)
        self.assertEqual(m.state, LAND)
        self.assertEqual(cmd.kind, "twist")
        self.assertLess(cmd.vz, 0.0)
        m.update(1.1, True, self.pose(m, 1.1, z=0.3), False, 0.0)      # PX4: in the air
        m.update(1.2, True, self.pose(m, 1.2, z=0.0), True, 0.0)       # then down: a real touchdown
        self.assertEqual(m.state, RETURN_CLIMB)

    def test_the_landing_at_the_launch_mark_needs_its_own_liftoff(self):
        m = self.at_pad()
        m.update(1.0, True, self.pose(m, 1.0), True, 0.0)
        self.assertEqual(m.state, RETURN_CLIMB)
        m.state = LAND                     # straight to the second landing, with no "not landed" between
        m.update(1.1, True, self.pose(m, 1.1, m.start_odom), True, 0.0)
        self.assertEqual(m.state, LAND)

    def test_still_descending_until_px4_says_landed(self):
        m = self.at_pad()
        cmd = m.update(1.0, True, self.pose(m, 1.0, z=0.05), False, 0.0)
        self.assertEqual(m.state, LAND)
        self.assertEqual(cmd.kind, "twist")
        self.assertLess(cmd.vz, 0.0)

    def test_a_disarm_on_the_pad_stops_the_mission(self):
        # If the window is missed the mission ends there, rather than commanding a dead vehicle.
        m = self.at_pad()
        self.assertIsNone(m.update(1.0, False, self.pose(m, 1.0), True, 0.0))
        self.assertEqual(m.state, STOPPED)

    def test_the_reference_waits_for_the_drone(self):
        m = self.at_pad()
        m.update(1.0, True, self.pose(m, 1.0), True, 0.0)
        m.update(1.1, True, self.pose(m, 1.1, z=m.z_return), None, 0.0)
        self.assertEqual(m.state, RETURN_TRANSIT)
        for i in range(200):                       # the drone never moves
            t = 2.0 + i * 0.05
            cmd = m.update(t, True, self.pose(m, t, z=m.z_return), None, 0.0)
        lead = math.hypot(cmd.x - m.end_odom[0], cmd.y - m.end_odom[1])
        self.assertLessEqual(lead, m.config.return_lag_m + 0.05)

    def test_a_vision_gap_does_not_throw_the_reference_ahead(self):
        m = self.at_pad()
        m.update(1.0, True, self.pose(m, 1.0), True, 0.0)
        m.update(1.1, True, self.pose(m, 1.1, z=m.z_return), None, 0.0)
        self.assertIsNone(m.update(1.2, True, None, None, 0.0))        # stale: sends nothing
        cmd = m.update(8.0, True, self.pose(m, 8.0, z=m.z_return), None, 0.0)
        lead = math.hypot(cmd.x - m.end_odom[0], cmd.y - m.end_odom[1])
        self.assertLessEqual(lead, m.config.return_lag_m + 1e-9)

    def test_off_the_line_holds_for_the_pilot(self):
        m = self.at_pad()
        m.update(1.0, True, self.pose(m, 1.0), True, 0.0)
        m.update(1.1, True, self.pose(m, 1.1, z=m.z_return), None, 0.0)
        ax, ay = m.end_odom
        bx, by = m.start_odom
        n = math.hypot(bx - ax, by - ay)
        off = (ax - (by - ay) / n * 0.5, ay + (bx - ax) / n * 0.5)
        cmd = m.update(1.2, True, self.pose(m, 1.2, off, z=m.z_return), None, 0.0)
        self.assertEqual(m.state, HELD)
        self.assertAlmostEqual(cmd.z, m.z_return)


if __name__ == "__main__":
    unittest.main()
