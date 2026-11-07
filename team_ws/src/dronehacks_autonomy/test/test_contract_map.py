"""The contract map loader: it refuses bad maps and never substitutes a default (contracts/course.md)."""

import copy
import math
import unittest

import _paths
import numpy as np
import yaml
from synthetic_courses import random_course

from dronehacks_autonomy.contract_map import _open_window, _oval_width, from_contract, load_contract_map
from dronehacks_autonomy.factory import prepare
from dronehacks_autonomy.frames import model_odom_frame
from dronehacks_autonomy.mission import MissionConfig
from dronehacks_autonomy.planning.course_map import MapError
from dronehacks_autonomy.planning.route import PlannerConfig, make_field, plan

PUBLISHED = yaml.safe_load(_paths.CONTRACT_MAP.read_text(encoding="utf-8"))


def goal(doc, name):
    return next(g for g in doc["goals"] if g["name"] == name)


class RefusesTest(unittest.TestCase):
    def broken(self, edit):
        doc = copy.deepcopy(PUBLISHED)
        edit(doc)
        return doc

    def assert_refused(self, edit, message=""):
        with self.assertRaisesRegex(MapError, message):
            from_contract(self.broken(edit))

    def test_each_required_goal(self):
        for name in ("tunnel", "window", "pad", "launch"):
            with self.subTest(name=name):
                self.assert_refused(lambda d: d["goals"].remove(goal(d, name)), name)

    def test_unknown_goal(self):
        self.assert_refused(lambda d: d["goals"].append(
            {"name": "bonus", "shape": "disc", "centre": [0, 0], "params": {"radius": 1.0}}), "bonus")

    def test_missing_params(self):
        for name, key in (("tunnel", "width"), ("tunnel", "axis_end"), ("window", "clearance_radius"),
                          ("window", "side"), ("pad", "radius"), ("launch", "heading_deg")):
            with self.subTest(name=name, key=key):
                self.assert_refused(lambda d: goal(d, name)["params"].pop(key), key)

    def test_zero_length_axis(self):
        def edit(d):
            p = goal(d, "tunnel")["params"]
            p["axis_end"] = list(p["axis_start"])
        self.assert_refused(edit, "zero length")

    def test_bad_wall_index(self):
        self.assert_refused(lambda d: goal(d, "window")["params"].update(wall_segment_index=999), "wall_segment_index")

    def test_window_axis_not_normal_to_its_wall(self):
        def edit(d):
            p = goal(d, "window")["params"]
            wall = d["segments"][p["wall_segment_index"]]
            c = goal(d, "window")["centre"]
            dx, dy = wall["end"][0] - wall["start"][0], wall["end"][1] - wall["start"][1]
            p["axis_start"] = [c[0] - dx * 0.01, c[1] - dy * 0.01]
            p["axis_end"] = [c[0] + dx * 0.01, c[1] + dy * 0.01]
        self.assert_refused(edit, "not normal")

    def test_window_off_its_wall(self):
        def edit(d):
            g = goal(d, "window")
            ux, uy = (g["params"]["axis_end"][0] - g["params"]["axis_start"][0],
                      g["params"]["axis_end"][1] - g["params"]["axis_start"][1])
            n = math.hypot(ux, uy)
            g["centre"] = [g["centre"][0] + ux / n, g["centre"][1] + uy / n]
            g["params"]["axis_start"] = [g["params"]["axis_start"][0] + ux / n, g["params"]["axis_start"][1] + uy / n]
            g["params"]["axis_end"] = [g["params"]["axis_end"][0] + ux / n, g["params"]["axis_end"][1] + uy / n]
        self.assert_refused(edit, "not on its wall")

    def test_non_finite_geometry(self):
        self.assert_refused(lambda d: goal(d, "pad").update(centre=[float("nan"), 0.0]))


class WindowTest(unittest.TestCase):
    def test_published_window_is_blocked_and_refused_when_asked_for(self):
        self.assertIsNone(goal(PUBLISHED, "window")["params"]["clearance_radius"])
        with self.assertRaisesRegex(MapError, "clearance_radius is null"):
            load_contract_map(_paths.CONTRACT_MAP, use_window=True)

    def test_blocked_window_leaves_its_wall_solid(self):
        cmap = from_contract(PUBLISHED)
        self.assertEqual([g.name for g in cmap.goals], ["tunnel", "pad"])
        centre = np.array(goal(PUBLISHED, "window")["centre"])
        surface = make_field(cmap).query(centre)[0][0]
        self.assertLess(surface, 0.06)  # the opening's centre is inside the wall, not free space

    def test_opened_window_is_free_space(self):
        doc = random_course(7, clearance_radius=0.35)
        cmap = from_contract(doc, use_window=True)
        self.assertEqual([g.name for g in cmap.goals], ["tunnel", "window", "pad"])
        surface = make_field(cmap).query(np.array(goal(doc, "window")["centre"]))[0][0]
        self.assertGreater(surface, 0.4)

    def test_a_route_inside_the_reviewed_budget_is_refused(self):
        doc = random_course(7, clearance_radius=0.50)   # wider than half the opening
        cmap = from_contract(doc, use_window=True)
        frame = model_odom_frame(cmap.launch.position, cmap.launch.heading)
        with self.assertRaisesRegex(ValueError, "clearance_radius"):
            prepare(cmap, MissionConfig(rest_height_m=0.07), frame=frame)


class ConversionTest(unittest.TestCase):
    def test_launch_heading_is_clockwise_from_north(self):
        for degrees, want in ((0.0, (0.0, 1.0)), (90.0, (1.0, 0.0)), (180.0, (0.0, -1.0))):
            doc = copy.deepcopy(PUBLISHED)
            goal(doc, "launch")["params"]["heading_deg"] = degrees
            got = from_contract(doc).launch.heading
            self.assertAlmostEqual(got[0], want[0])
            self.assertAlmostEqual(got[1], want[1])

    def test_tunnel_walls_close_the_bore_sides(self):
        # Without its walls the tube is open space and the route could cut across it.
        cmap = from_contract(PUBLISHED)
        t = goal(PUBLISHED, "tunnel")
        a = np.array(t["params"]["axis_start"])
        b = np.array(t["params"]["axis_end"])
        u = (b - a) / np.linalg.norm(b - a)
        # At the oval's width over the drone's height at flight height, not its widest.
        half, h, z = PlannerConfig().drone_half_height_m, t["params"]["height"], PUBLISHED["slice_height"]
        width = min(_oval_width(t["params"]["width"], h, z - half), _oval_width(t["params"]["width"], h, z + half))
        self.assertLess(width, t["params"]["width"])
        side = np.array([-u[1], u[0]]) * width / 2.0
        field = make_field(cmap)
        for along in (0.1, 0.9):   # near each end of the tube, clear of the walls meeting its middle
            for sign in (1.0, -1.0):
                self.assertLess(field.query(a + (b - a) * along + sign * side)[0][0], 0.01)

    def test_a_tube_the_drone_cannot_fit_is_refused(self):
        for height in (0.85, 0.5):       # the roof is under the drone's top, or under the flight height
            doc = copy.deepcopy(PUBLISHED)
            goal(doc, "tunnel")["params"]["height"] = height
            with self.subTest(height=height), self.assertRaisesRegex(MapError, "does not fit"):
                from_contract(doc)

    def test_a_low_oval_narrows_the_walls_at_flight_height(self):
        # Fits, but the drone is high in the oval, where it is far narrower than its widest.
        self.assertAlmostEqual(_oval_width(1.4, 1.0, 0.5), 1.4)
        self.assertAlmostEqual(_oval_width(1.4, 1.0, 0.9), 1.4 * 0.6)

    def test_an_opening_as_wide_as_a_last_wall_leaves_no_wall(self):
        wall = {"start": [0.0, 0.0], "end": [1.0, 0.0], "thickness": 0.1}
        other = {"start": [0.0, 1.0], "end": [1.0, 1.0], "thickness": 0.1}
        window = {"side": 1.0, "along": 0.5, "wu": (1.0, 0.0), "w0": (0.0, 0.0), "w1": (1.0, 0.0), "index": 1,
                  "centre": (0.5, 0.0), "u": (0.0, 1.0), "sill": 0.3, "head": 1.2}
        g = {"params": {"clearance_radius": 0.3}}
        segments = [other, wall]
        _open_window(g, window, segments, "test")
        self.assertEqual(segments, [other])
        segments = [wall, other]
        _open_window(g, dict(window, index=0), segments, "test")
        self.assertEqual(segments, [other])

    def test_published_route_is_forced_through_the_tunnel_and_reaches_the_pad(self):
        route = plan(from_contract(PUBLISHED))
        self.assertEqual(route.goals, ["tunnel", "pad"])
        self.assertEqual(route.waypoints[-1].position, tuple(goal(PUBLISHED, "pad")["centre"]))
        self.assertGreaterEqual(route.min_margin_m, 0.0)

    def test_the_package_map_matches_the_contract_map(self):
        contract = _paths.REPO / "contracts" / "course_v3_layout.yaml"
        if not contract.is_file():
            self.skipTest("contracts/ is not next to this checkout (on the drone)")
        self.assertEqual(_paths.CONTRACT_MAP.read_bytes(), contract.read_bytes())


if __name__ == "__main__":
    unittest.main()
