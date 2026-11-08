"""STEP-extraction JSON -> course.md map, on a made-up course (AGENTS.md section 3.7)."""

import math
import unittest

from dronehacks_planner import parse_map, plan
from dronehacks_planner.convert import ConvertError, convert


def wall(name, p0, p1, openings=(), t=0.1, z=(0.0, 1.7)):
    return {"name": name, "kind": "wall", "p0": list(p0), "p1": list(p1), "thickness": t, "z": list(z),
            "openings": [dict(o) for o in openings]}


def net(name, poly):
    return {"name": name, "kind": "net", "z": [0.0, 3.0], "footprint": [list(p) for p in poly]}


def marking(name, centre, r):
    return {"name": name, "kind": "marking", "z": [0.0, 0.001], "centre": list(centre), "half_size": [r, r]}


def course(window_z=(0.3, 1.2)):
    """A 6 x 6 m net; a wall across the middle (y = 0) with a window; a tube in the east; a pad north."""
    sq = [(-3, -3), (3, -3), (3, 3), (-3, 3)]
    feats = [net("Net", sq),
             wall("Middle", (-3.0, 0.0), (1.5, 0.0), openings=[{"s": [2.0, 2.9], "z": list(window_z)}]),
             wall("Stub", (1.5, 0.0), (1.5, -1.0)),
             {"name": "Tube", "kind": "tube", "z": [0.0, 1.8], "p0": [2.25, 1.0, 0.9], "p1": [2.25, -1.0, 0.9],
              "outer": {"half_width": 0.70, "half_height": 0.9}, "inner": {"half_width": 0.68, "half_height": 0.88}},
             {"name": "Rock", "kind": "rock", "z": [0.0, 0.5], "centre": [-2.0, 2.0], "radius": 0.2},
             {"name": "Cradle", "kind": "support", "z": [0.0, 0.25], "p0": [1.9, 0.0], "p1": [2.6, 0.0],
              "thickness": 0.02, "openings": []},
             marking("Start", (-2.0, -2.0), 0.08), marking("Pad", (-1.5, 1.8), 0.7)]
    cmap = {"format": "course-map/1", "features": feats, "source": {"step": "made-up"}}
    rules = {"format": "course-rules/1", "start": {"marking": "Start"},
             "finish": {"marking": "Pad", "inside_radius_m": 0.65},
             "checkpoints": [
                 {"name": "CP1", "gate": {"segment": [[-3, -1], [-1.5, -1]]}},
                 {"name": "CP2 tube", "gate": {"tube_end": "Tube", "end": "p1"}},
                 {"name": "CP3 entry", "routes": {"window": {"gate": {"opening": "Middle"}},
                                                  "gap": {"gate": {"between": ["Stub", "Middle"]}}}},
                 {"name": "CP4 land", "land": True}]}
    return cmap, rules


class ConvertTest(unittest.TestCase):
    def setUp(self):
        self.cmap, self.rules = course()
        self.doc = convert(self.cmap, self.rules, (0, 1))
        self.goals = {g["name"]: g for g in self.doc["goals"]}

    def test_window_splits_the_wall_and_becomes_a_rect_goal(self):
        middle = [s for s in self.doc["segments"] if abs(s["start"][1]) <= 0.05 and abs(s["end"][1]) <= 0.05
                  and min(s["start"][0], s["end"][0]) < 1.5]
        self.assertEqual(len(middle), 8)          # two rectangles, four edges each
        w = self.goals["window"]
        self.assertEqual(w["shape"], "rect")
        self.assertEqual(w["centre"], [-0.55, 0.0])
        self.assertEqual(w["params"]["axis"], [0.0, 1.0])    # towards the pad, north of the wall
        self.assertAlmostEqual(w["params"]["width"], 0.9)
        self.assertAlmostEqual(w["params"]["depth"], 0.1)
        self.assertEqual((w["params"]["bottom"], w["params"]["top"]), (0.3, 1.2))
        self.assertEqual(w["group"], self.goals["gap"]["group"])

    def test_opening_outside_the_band_keeps_the_wall_whole(self):
        cmap, rules = course(window_z=(1.0, 1.5))
        rules["checkpoints"][2]["routes"].pop("gap")
        with self.assertRaises(ConvertError):
            convert(cmap, rules, (0, 1))

    def test_tube_is_sliced_at_its_narrowest_in_the_band(self):
        t = self.goals["tunnel"]
        self.assertEqual(t["params"]["axis"], [0.0, -1.0])    # out through p1, the south end
        self.assertLess(t["params"]["width"], 2 * 0.68)
        dz = 0.9 - 0.7
        self.assertAlmostEqual(t["params"]["width"], 2 * 0.68 * math.sqrt(1 - (dz / 0.88) ** 2), places=4)

    def test_tube_outer_wall_uses_the_widest_in_band_width(self):
        # Review of #24: the outer wall used the narrowest width over the band
        # (0.70 * sqrt(1 - (0.2 / 0.9)^2) = 0.6825), shrinking the obstacle on
        # its outside and giving paths beside the tube ~1.75 cm of phantom
        # clearance. The band 0.7-0.9 contains the outer ellipse's centre (0.9),
        # so the outer wall must sit at the full half-width 0.70.
        long_x = sorted({round(s["start"][0], 4) for s in self.doc["segments"]
                         if {round(s["start"][1], 4), round(s["end"][1], 4)} == {1.0, -1.0}
                         and math.isclose(s["start"][0], s["end"][0])})
        self.assertEqual(len(long_x), 4)                       # inner and outer, both sides
        self.assertAlmostEqual(min(long_x), 2.25 - 0.70, places=3)
        self.assertAlmostEqual(max(long_x), 2.25 + 0.70, places=3)
        self.assertNotAlmostEqual(min(long_x), 2.25 - 0.6825, places=3)

    def test_rock_launch_pad_and_checkpoint_gate(self):
        self.assertEqual(self.doc["boulders"], [{"centre": [-2.0, 2.0], "radius": 0.2, "height": 0.5}])
        self.assertEqual(self.doc["launch"], {"position": [-2.0, -2.0], "heading": [0.0, 1.0]})
        self.assertEqual(self.goals["pad"]["params"]["radius"], 0.65)
        # a checkpoint line is a goal the route flies through, in the rules' order
        self.assertNotIn("checkpoints", self.doc)
        self.assertEqual([g["name"] for g in self.doc["goals"]], ["cp1", "tunnel", "window", "gap", "pad"])
        cp1 = self.goals["cp1"]
        self.assertEqual((cp1["shape"], cp1["centre"]), ("gate", [-2.25, -1.0]))
        self.assertEqual(cp1["params"], {"axis": [0.0, 1.0], "width": 1.5, "depth": 0.0})
        self.assertNotIn("group", cp1)

    def test_checkpoint_gate_is_flown_towards_the_next_goal_not_the_pad(self):
        # A line at x = -1 from the south net: the tunnel (next) is east of it, the pad west.
        self.rules["checkpoints"][0]["gate"] = {"segment": [[-1.0, -3.0], [-1.0, -1.0]]}
        cp1 = convert(self.cmap, self.rules, (0, 1))["goals"][0]
        self.assertEqual(cp1["params"]["axis"], [1.0, 0.0])

    def test_block_footprint_is_an_obstacle(self):
        self.cmap["features"].append({"name": "Block", "kind": "block", "z": [0.0, 1.7],
                                      "footprint": [[-3.0, 2.6], [-2.4, 2.6], [-2.4, 3.0], [-3.0, 3.0]]})
        doc = convert(self.cmap, self.rules, (0, 1))
        edges = {(tuple(s["start"]), tuple(s["end"])) for s in doc["segments"]}
        self.assertLessEqual({((-3.0, 2.6), (-2.4, 2.6)), ((-2.4, 2.6), (-2.4, 3.0))}, edges)
        low = dict(self.cmap["features"][-1], z=[0.0, 0.3], name="Low block")
        self.cmap["features"][-1] = low
        self.assertEqual(len(convert(self.cmap, self.rules, (0, 1))["segments"]), len(self.doc["segments"]))

    def test_low_support_is_left_out(self):
        self.assertFalse(any(abs(s["start"][1]) < 0.02 and 1.9 <= s["start"][0] <= 2.6 and s["start"][0] != 1.55
                             for s in self.doc["segments"] if s["start"][0] > 1.6))

    def test_touching_features_make_no_gate(self):
        with self.assertRaises(ConvertError):
            convert(self.cmap, self.rules, (0, 1), route_gates={"gap": {"ends": ["Stub", "Middle"]}})

    def test_between_and_ends_gates(self):
        # Two walls, A ending at (-1, -1) and B ending at (0.2, -1.2): a gap whose ends are 1.217 m apart,
        # and whose closest points (A's end face to B's side) are nearer than that.
        feats = [net("Net", [(-3, -3), (3, -3), (3, 3), (-3, 3)]),
                 wall("A", (-1.0, -3.0), (-1.0, -1.0)), wall("B", (0.2, -1.2), (2.0, 1.0)),
                 marking("Start", (-2.0, -2.0), 0.08), marking("Pad", (1.0, -2.0), 0.7)]
        cmap = {"format": "course-map/1", "features": feats}
        rules = {"format": "course-rules/1", "start": {"marking": "Start"}, "finish": {"marking": "Pad"},
                 "checkpoints": [{"name": "entry", "routes": {"gap": {"gate": {"ends": ["A", "B"]}}}},
                                 {"name": "land", "land": True}]}
        g = convert(cmap, rules, (0, 1))["goals"][0]
        self.assertEqual(g["shape"], "gate")
        self.assertAlmostEqual(g["params"]["width"], math.hypot(1.2, 0.2), places=4)
        self.assertEqual(g["centre"], [-0.4, -1.1])
        ax = g["params"]["axis"]
        self.assertGreater((1.0 - -0.4) * ax[0] + (-2.0 - -1.1) * ax[1], 0)   # towards the pad
        rules["checkpoints"][0]["routes"]["gap"]["gate"] = {"between": ["A", "B"]}
        g2 = convert(cmap, rules, (0, 1))["goals"][0]
        self.assertLess(g2["params"]["width"], g["params"]["width"])

    def test_bad_inputs(self):
        with self.assertRaises(ConvertError):
            convert(self.cmap, self.rules, (0, 0))
        with self.assertRaises(ConvertError):
            convert({"format": "other"}, self.rules, (0, 1))

    def test_result_is_a_valid_map_the_planner_flies(self):
        cmap = parse_map(self.doc)
        self.assertEqual(len(cmap.stages()), 4)
        route = plan(cmap)
        self.assertEqual(route.goals[:2], ["cp1", "tunnel"])
        self.assertEqual(route.goals[-1], "pad")
        self.assertGreaterEqual(route.min_margin_m, 0.0)


if __name__ == "__main__":
    unittest.main()
