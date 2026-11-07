"""The provisional published-layout fixture, perturbed layouts, and random mazes.

The perturbation and random-maze cases are the evidence for the generality
score (rule 9.5): the same code, with no layout knowledge, plans every one.
"""

import ast
import dataclasses
import math
import random
import unittest
from pathlib import Path

import yaml
from _maps import PUBLISHED, serpentine

from dronehacks_planner import PlannerConfig, load_map, parse_map, plan

CFG = PlannerConfig()
PACKAGE = Path(__file__).resolve().parent.parent / "dronehacks_planner"


class PublishedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmap = load_map(PUBLISHED)
        cls.route = plan(cls.cmap)

    def test_visits_every_stage_in_order(self):
        # The window is opt-in (contracts/course.md leaves its clearance_radius
        # unreviewed), so the default route is Gate B.
        self.assertEqual(self.route.goals, ["tunnel", "gate_b", "pad"])
        self.assertEqual(self.route.choices, {"landing_entry": "gate_b"})
        self.assertIn("window", self.route.rejected)
        self.assertGreaterEqual(self.route.min_margin_m, 0.0)

    def test_window_is_opt_in(self):
        route = plan(self.cmap, dataclasses.replace(CFG, allow_windows=True))
        self.assertEqual(route.goals, ["tunnel", "window", "pad"])
        self.assertGreaterEqual(route.min_margin_m, 0.0)

    def test_length_is_plausible(self):
        # The field guide's planner measured about 17 m launch to pad.
        self.assertGreater(self.route.length_m, 14.0)
        self.assertLess(self.route.length_m, 21.0)

    def test_tunnel_flown_on_its_axis_and_cleared(self):
        tunnel = next(g for g in self.cmap.goals if g.name == "tunnel")
        p = self.route.passes["tunnel"]
        self.assertEqual(p.direction, tunnel.axis)
        exit_ = next(w for w in self.route.waypoints if w.kind == "exit" and w.goal == "tunnel")
        along = ((exit_.position[0] - tunnel.centre[0]) * tunnel.axis[0]
                 + (exit_.position[1] - tunnel.centre[1]) * tunnel.axis[1])
        # Whole drone out of the far end (checkpoint 3), with the margin.
        self.assertGreaterEqual(along - tunnel.depth / 2 - CFG.drone_extent_m, CFG.crossing_margin_m - 1e-9)

    def test_gate_b_fallback(self):
        route = plan(self.cmap, dataclasses.replace(CFG, prefer=("gate_b",)))
        self.assertEqual(route.goals, ["tunnel", "gate_b", "pad"])
        self.assertGreaterEqual(route.min_margin_m, 0.0)

    def test_ends_on_the_pad_centre(self):
        pad = next(g for g in self.cmap.goals if g.name == "pad")
        self.assertEqual(self.route.waypoints[-1].position, pad.centre)


class PerturbedLayoutTest(unittest.TestCase):
    def test_boulders_moved(self):
        base = yaml.safe_load(PUBLISHED.read_text(encoding="utf-8"))
        for seed in range(8):
            rng = random.Random(seed)
            doc = dict(base)
            doc["boulders"] = [dict(b, centre=[b["centre"][0] + rng.uniform(-0.15, 0.15),
                                               b["centre"][1] + rng.uniform(-0.15, 0.15)])
                               for b in base["boulders"]]
            with self.subTest(seed=seed):
                route = plan(parse_map(doc))
                self.assertEqual(route.goals[0], "tunnel")
                self.assertIn(route.goals[1], ("window", "gate_b"))
                self.assertEqual(route.goals[-1], "pad")
                self.assertGreaterEqual(route.min_margin_m, 0.0)

    def test_blocked_window_falls_back_to_gate_b(self):
        # Move the boulder nearest the window onto the window's exit side.
        doc = yaml.safe_load(PUBLISHED.read_text(encoding="utf-8"))
        win = next(g for g in doc["goals"] if g["name"] == "window")
        (cx, cy), (ax, ay) = win["centre"], win["params"]["axis"]
        nearest = min(doc["boulders"], key=lambda b: math.dist(b["centre"], win["centre"]))
        nearest["centre"] = [cx + ax * 0.75, cy + ay * 0.75]
        route = plan(parse_map(doc))
        self.assertEqual(route.choices, {"landing_entry": "gate_b"})
        self.assertIn("window", route.rejected)
        self.assertGreaterEqual(route.min_margin_m, 0.0)

    def test_whole_course_rotated(self):
        # Rotate every coordinate and axis 90 degrees: the planner must not care.
        doc = yaml.safe_load(PUBLISHED.read_text(encoding="utf-8"))

        def rot(p):
            return [-p[1], p[0]]

        doc["launch"]["position"] = rot(doc["launch"]["position"])
        for s in doc["segments"]:
            s["start"], s["end"] = rot(s["start"]), rot(s["end"])
        for b in doc["boulders"]:
            b["centre"] = rot(b["centre"])
        for g in doc["goals"]:
            g["centre"] = rot(g["centre"])
            if "axis" in g["params"]:
                g["params"]["axis"] = rot(g["params"]["axis"])
        base = plan(load_map(PUBLISHED))
        route = plan(parse_map(doc))
        self.assertEqual(route.goals, base.goals)
        self.assertAlmostEqual(route.length_m, base.length_m, delta=0.5)
        self.assertGreaterEqual(route.min_margin_m, 0.0)


class RandomMazeTest(unittest.TestCase):
    def test_random_serpentines(self):
        for seed in range(8):
            with self.subTest(seed=seed):
                route = plan(serpentine(seed))
                self.assertEqual(route.goals, ["pad"])
                self.assertGreaterEqual(route.min_margin_m, 0.0)


class NoCourseLiteralsTest(unittest.TestCase):
    """AGENTS.md section 3.7 (contracts branch) / rule 9.5: no course geometry in code.

    Every number that places or sizes something on the published course (goal
    centres and dimensions, launch, boulders) must be absent from the package
    source. Drone and safety constants are allowed.
    """

    def test_package_has_no_course_numbers(self):
        doc = yaml.safe_load(PUBLISHED.read_text(encoding="utf-8"))
        course = set()

        def add(v):
            if isinstance(v, (int, float)) and not isinstance(v, bool) and abs(v) > 0.05:
                course.add(round(abs(float(v)), 3))
            elif isinstance(v, (list, tuple)):
                for x in v:
                    add(x)
            elif isinstance(v, dict):
                for x in v.values():
                    add(x)

        add(doc["launch"])
        add(doc["boulders"])
        for g in doc["goals"]:
            add(g["centre"])
            add({k: v for k, v in g["params"].items() if k != "axis"})
        course -= {1.0}
        found = []
        for path in sorted(PACKAGE.glob("*.py")):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.Constant) and isinstance(node.value, float):
                    if round(abs(node.value), 3) in course:
                        found.append(f"{path.name}:{node.lineno}: {node.value}")
        self.assertEqual(found, [], "course numbers in planner code")


if __name__ == "__main__":
    unittest.main()
