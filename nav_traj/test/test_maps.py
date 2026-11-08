"""The packaged STEP map: regenerates byte for byte from its sources, and flies the whole mission."""

import json
import unittest

import _paths
import yaml

from dronehacks_planner.convert import convert
from dronehacks_planner.course_map import load_map
from dronehacks_traj.factory import flight_checks, prepare
from dronehacks_traj.frames import model_odom_frame
from dronehacks_traj.mission import MissionConfig
from dronehacks_traj.vehicle_sim import fly, summary

MAPS = _paths.PACKAGE_ROOT / "maps"


class StepMapTest(unittest.TestCase):
    def test_regenerates_from_its_sources(self):
        cmap = json.loads((MAPS / "source" / "course_v3_map.json").read_text(encoding="utf-8"))
        rules = json.loads((MAPS / "source" / "course_v3_rules.json").read_text(encoding="utf-8"))
        doc = convert(cmap, rules, (0, 1), route_gates={"gate_b": {"ends": ["C2b chicane wall", "A1 approach wall"]}})
        packaged = yaml.safe_load((MAPS / "course_v3_step.yaml").read_text(encoding="utf-8"))
        self.assertEqual(doc, packaged)
        self.assertEqual(cmap["source"]["sha256"], "2194664e852e341f86fb63a19ecc4f8f0ef34da6a51867e49beb3157cb90b876")

    def test_contract_geometry(self):
        doc = yaml.safe_load((MAPS / "course_v3_step.yaml").read_text(encoding="utf-8"))
        goals = {g["name"]: g for g in doc["goals"]}
        self.assertAlmostEqual(goals["gate_b"]["params"]["width"], 1.13, delta=0.01)   # course.md: 1.13 m
        self.assertAlmostEqual(goals["window"]["params"]["width"], 0.914, delta=0.001)  # 0.914 m square
        self.assertAlmostEqual(goals["tunnel"]["params"]["length"], 1.524, delta=0.001)
        self.assertEqual(goals["tunnel"]["params"]["axis"], [0.0, -1.0])               # out of the south end
        self.assertEqual(doc["launch"]["heading"], [0.0, 1.0])                         # frames.md: facing north

    def test_whole_mission_on_the_step_map(self):
        cmap = load_map(str(MAPS / "course_v3_step.yaml"))
        # The model's odom frame starts at the launch pose; the node's frame is measured.
        # land is off by default (px4.md): the dry run of a whole mission opts in explicitly.
        frame = model_odom_frame(cmap.launch.position, cmap.launch.heading)
        prep = prepare(cmap, MissionConfig(rest_height_m=0.07, land=True), frame=frame)
        log = fly(prep.mission)
        checks = flight_checks(prep, log.pos)
        self.assertEqual(summary(log)["states"], ["WAIT", "TAKEOFF", "FOLLOW", "HOLD", "LAND", "LANDED"])
        self.assertIn("disarm", log.events)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertEqual(checks["openings"], {"cp2_gate_a": True, "tunnel": True, "gate_b": True})
        self.assertTrue(checks["on_pad"])


if __name__ == "__main__":
    unittest.main()
