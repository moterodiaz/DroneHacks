"""The packaged maps: each regenerates from its sources and flies the whole mission.

* ``course_v3_step.yaml``: the STEP export (the default launches).
* ``course_arena_2026-11-08.yaml``: the course as the team taped it (Michael Bar On's map on
  core/flight-iteration), flown by the window flight lines.
"""

import dataclasses
import hashlib
import json
import math
import unittest

import _paths
import yaml

from dronehacks_planner.convert import apply_edits, convert
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


SOURCE = MAPS / "source"
EDITS = SOURCE / "arena_2026-11-08_edits.json"
TAPED = MAPS / "course_arena_2026-11-08.yaml"


def taped_sources():
    cmap = json.loads((SOURCE / "course_v3_map.json").read_text(encoding="utf-8"))
    rules = json.loads((SOURCE / "course_v3_rules.json").read_text(encoding="utf-8"))
    edits = json.loads(EDITS.read_text(encoding="utf-8"))
    return cmap, rules, edits


class TapedArenaMapTest(unittest.TestCase):
    def test_regenerates_from_the_step_extraction_and_the_edits(self):
        cmap, rules, edits = taped_sources()
        self.assertEqual(edits["applies_to"]["map_sha256"],
                         hashlib.sha256((SOURCE / "course_v3_map.json").read_bytes()).hexdigest())
        cmap, rules = apply_edits(cmap, rules, edits)
        doc = convert(cmap, rules, (0, 1), slice_height=1.23,
                      route_gates={"gate_b": {"ends": ["C2b chicane wall", "A1 approach wall"]}})
        self.assertEqual(doc, yaml.safe_load(TAPED.read_text(encoding="utf-8")))
        self.assertEqual([g["name"] for g in doc["goals"]], ["tunnel", "window", "gate_b", "pad"])

    def test_the_edits_carry_michaels_map(self):
        """Every taped number in the edits is the source map's own (core/flight-iteration b5d640e)."""
        _, _, edits = taped_sources()
        src = edits["source"]
        raw = (SOURCE / src["copy"]).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), src["file_sha256"])
        mb = json.loads(raw)
        add = {e["name"]: e for e in edits["add"]}
        moves = {n: e for e in edits["move"] for n in e["names"]}
        north = add["North wall (taped)"]
        self.assertAlmostEqual(north["p0"][1] - north["thickness"] / 2, mb["bounds"][3], places=6)
        # the diagonal's centreline: the middles of its two short sides in the source polygon
        i = mb["units"].index("D diagonal wall (measured)")
        corners = [s[:2] for s, u in zip(mb["segments"], mb["segment_units"]) if u == i]
        ends = sorted(([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] for a, b in zip(corners, corners[1:] + corners[:1])
                       if math.dist(a, b) < 0.2), key=lambda p: p[1])
        diag = add["Diagonal wall (taped)"]
        for want, got in zip(ends, (diag["p0"], diag["p1"])):
            self.assertLess(math.dist(want, got), 0.001)
        self.assertEqual(sorted(tuple(round(v, 4) for v in b[:2]) for b in mb["boulders"]),
                         sorted(tuple(round(v, 4) for v in moves[n]["to"]) if n in moves else
                                {"Boulder 4": (-2.5603, -2.4994)}[n]
                                for n in ("Boulder 1", "Boulder 3", "Boulder 4", "Boulder 5")))
        self.assertEqual(moves["Landing pad 5 ft"]["to"], mb["goals"]["pad"]["at"])
        window = edits["replace"][0]
        self.assertEqual(window["name"], mb["goals"]["window"]["unit"])
        self.assertEqual(window["openings"][0]["z"], mb["goals"]["window"]["span_m"])
        doc = yaml.safe_load(TAPED.read_text(encoding="utf-8"))
        self.assertEqual(doc["slice_height"], mb["height_m"])
        self.assertEqual(doc["launch"]["position"], mb["launch"][:2])

    def test_the_route_takes_the_whole_drone_across_gate_a_and_flies(self):
        """CP2 Gate A has no goal on the taped map (planner_goal false): the route must still take
        the whole drone across its line, from Barrier 3's face to the north wall. The leg to the
        tunnel is the same whichever way CP4 goes (test_mission flies the window lines)."""
        _, _, edits = taped_sources()
        (a, b), = [edits["rules"]["checkpoints"]["CP2 Gate A"]["gate"]["segment"]]
        cmap = dataclasses.replace(load_map(str(TAPED)), boulders=())      # avoid_boulders false
        frame = model_odom_frame(cmap.launch.position, cmap.launch.heading)
        prep = prepare(cmap, MissionConfig(rest_height_m=0.10, land=True), frame=frame)
        xy = prep.plan.trajectory.xy
        side = (xy[:, 0] - a[0]) * (b[1] - a[1]) - (xy[:, 1] - a[1]) * (b[0] - a[0])
        k = int(next(i for i in range(len(xy) - 1) if side[i] * side[i + 1] <= 0 and side[i] != side[i + 1]))
        y = xy[k, 1]
        self.assertTrue(min(a[1], b[1]) < y < max(a[1], b[1]), y)
        extent = 0.28                                                      # contracts/course.md prop reach
        self.assertTrue(any(abs(s) / math.dist(a, b) > extent for s in side[k:]))
        log = fly(prep.mission)
        checks = flight_checks(prep, log.pos)
        self.assertIn("disarm", log.events)
        self.assertEqual(checks["wall_touches"], 0)
        self.assertTrue(checks["on_pad"])

    def test_the_window_route_is_smoothed_not_left_as_its_polyline(self):
        """The tight gap at the diagonal's top end needs 22 smoothing repairs. At the old limit of 20
        every corner fell back to sharp, and the speed profile stopped at each one (planned 36.9 s
        at 0.5 m/s instead of 27.2 s)."""
        from dronehacks_planner.pipeline import build
        from dronehacks_planner.route import PlannerConfig
        cmap = dataclasses.replace(load_map(str(TAPED)), boulders=())
        plan = build(cmap, dataclasses.replace(PlannerConfig(), prefer=("window",)))
        self.assertFalse(any("fell back" in n for n in plan.path.notes), plan.path.notes)
        self.assertGreaterEqual(sum(r > 0 for r in plan.path.radii), len(plan.path.radii) // 2)
        self.assertGreater(plan.trajectory.length / plan.trajectory.duration, 0.65)   # mean m/s at v_max 1.0


if __name__ == "__main__":
    unittest.main()
