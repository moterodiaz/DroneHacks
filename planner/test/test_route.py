import dataclasses
import json
import math
import unittest

import numpy as np
from _maps import box, make_map, pad, wall_with_gap

from dronehacks_planner import PlannerConfig, PlanningError, plan
from dronehacks_planner.clearance import sample_segment

CFG = PlannerConfig()


def assert_route_ok(tc, route):
    """Properties every route must have, whatever the map."""
    tc.assertGreaterEqual(route.min_margin_m, 0.0)
    json.dumps(route.to_dict())  # serialisable
    for a, b in zip(route.waypoints, route.waypoints[1:]):
        tc.assertGreater(math.dist(a.position, b.position), 1e-9, "repeated waypoint")


def window(name, centre, axis, width, **params):
    return {"name": name, "shape": "rect", "centre": list(centre),
            "params": dict({"axis": list(axis), "width": width, "depth": 0.1}, **params)}


def gate(name, centre, axis, width, **params):
    return {"name": name, "shape": "gate", "centre": list(centre),
            "params": dict({"axis": list(axis), "width": width, "depth": 0.1}, **params)}


class CorridorTest(unittest.TestCase):
    def test_route_rides_the_centreline(self):
        # A 1.0 m corridor (wall faces 1.0 m apart) running east.
        segs = [((-3, 0.55), (3, 0.55)), ((-3, -0.55), (3, -0.55)), ((-3, -0.55), (-3, 0.55)),
                ((3, -0.55), (3, 0.55))]
        cmap = make_map(segs, [pad((2.4, 0.0), radius=0.3)], launch=(-2.4, 0.0))
        route = plan(cmap)
        assert_route_ok(self, route)
        ys = [w.position[1] for w in route.waypoints]
        self.assertLess(max(abs(y) for y in ys), 1e-6)
        self.assertEqual(len(route.waypoints), 2)  # shortcut to a single straight leg
        self.assertAlmostEqual(route.min_clearance_m, 0.5, places=6)

    def test_open_room_is_one_straight_leg(self):
        cmap = make_map(box(5, 5), [pad((1.5, 1.5))], launch=(-1.5, -1.5))
        route = plan(cmap)
        assert_route_ok(self, route)
        self.assertEqual(len(route.waypoints), 2)
        self.assertAlmostEqual(route.length_m, math.dist((-1.5, -1.5), (1.5, 1.5)), places=6)


class GapTest(unittest.TestCase):
    def room(self, gap):
        segs = box(4, 4) + wall_with_gap(0.0, -2, 2, -gap / 2 - 0.05, gap / 2 + 0.05)
        return make_map(segs, [pad((1.0, 1.2), radius=0.3)], launch=(-1.0, -1.2))

    def test_narrowest_course_passage_is_passable(self):
        # contracts/course.md: narrowest passage 0.93 m.
        route = plan(self.room(0.93))
        assert_route_ok(self, route)
        crossing = [w for w in route.waypoints if abs(w.position[1]) < 0.4]
        self.assertTrue(all(abs(w.position[0]) < 0.06 for w in crossing), crossing)

    def test_too_narrow_gap_has_no_route(self):
        with self.assertRaises(PlanningError) as cm:
            plan(self.room(0.60))
        self.assertIn("cannot reach", str(cm.exception))

    def test_start_too_close_to_a_wall(self):
        cmap = self.room(0.93)
        with self.assertRaises(PlanningError) as cm:
            plan(cmap, start=(-1.0, -0.2))
        self.assertIn("inscribed radius", str(cm.exception))


class PassThroughTest(unittest.TestCase):
    def room(self, goal, launch=(-1.0, -1.4)):
        segs = box(4, 4) + wall_with_gap(0.0, -2, 2, -0.5, 0.5)
        return make_map(segs, [goal, pad((1.0, 1.4), radius=0.3)], launch=launch)

    def test_window_is_flown_straight_and_whole_drone_clears_it(self):
        route = plan(self.room(window("win", (0, 0), (0, 1), 0.9)))
        assert_route_ok(self, route)
        kinds = [(w.kind, w.goal) for w in route.waypoints]
        i = kinds.index(("entry", "win"))
        self.assertEqual(kinds[i + 1], ("exit", "win"))
        entry, exit_ = route.waypoints[i], route.waypoints[i + 1]
        self.assertTrue(entry.straight_to_next)
        self.assertAlmostEqual(entry.position[0], 0.0)
        self.assertAlmostEqual(exit_.position[0], 0.0)
        # Whole drone (centre + extent) past the far face, plus the crossing margin.
        self.assertGreaterEqual(exit_.position[1] - 0.05 - CFG.drone_extent_m, CFG.crossing_margin_m - 1e-9)
        self.assertLess(entry.position[1], -0.05 - CFG.drone_extent_m)
        self.assertEqual(route.passes["win"].direction, (0.0, 1.0))

    def test_direction_is_the_axis(self):
        # Start and pad both north of the wall; the window must still be flown northwards.
        route = plan(self.room(window("win", (0, 0), (0, 1), 0.9), launch=(-1.0, 1.4)))
        assert_route_ok(self, route)
        self.assertEqual(route.passes["win"].direction, (0.0, 1.0))

    def test_bidirectional_takes_the_cheaper_way(self):
        route = plan(self.room(window("win", (0, 0), (0, 1), 0.9, bidirectional=True), launch=(-1.0, 1.4)))
        assert_route_ok(self, route)
        # From the north side, going south first is the cheaper way to "visit" it.
        self.assertEqual(route.passes["win"].direction, (0.0, -1.0))

    def test_window_outside_flight_height_is_rejected(self):
        with self.assertRaises(PlanningError) as cm:
            plan(self.room(window("win", (0, 0), (0, 1), 0.9, bottom=1.0, top=1.9)))
        self.assertIn("usable band", str(cm.exception))

    def test_run_in_shrinks_to_fit(self):
        # A wall 0.95 m south of the window, reached round its east end, leaves
        # a 0.85 m corridor: no room for the full run-in below the window.
        segs = box(4, 4) + wall_with_gap(0.0, -2, 2, -0.5, 0.5) + [((-2, -0.95), (0.6, -0.95))]
        cmap = make_map(segs, [window("win", (0, 0), (0, 1), 0.9), pad((1.0, 1.4), radius=0.3)],
                        launch=(1.0, -1.4))
        route = plan(cmap)
        assert_route_ok(self, route)
        self.assertLess(route.passes["win"].approach_m, CFG.approach_m)


class AlternativesTest(unittest.TestCase):
    def room(self, near_width=0.9):
        # Wall across the room with a window right on the straight line and a
        # wide gate far to the east. Both lead north to the pad.
        segs = box(6, 4) + [((-3, 0), (-near_width / 2 - 0.05, 0)),
                            ((near_width / 2 + 0.05, 0), (1.8, 0)), ((2.8, 0), (3, 0))]
        goals = [dict(window("near", (0, 0), (0, 1), near_width), group="entry"),
                 dict(gate("far", (2.3, 0), (0, 1), 1.0), group="entry"),
                 pad((0, 1.4), radius=0.3)]
        return make_map(segs, goals, launch=(0, -1.4))

    def test_cheapest_alternative(self):
        route = plan(self.room())
        assert_route_ok(self, route)
        self.assertEqual(route.choices, {"entry": "near"})
        self.assertEqual(route.goals, ["near", "pad"])

    def test_preferred_alternative(self):
        route = plan(self.room(), dataclasses.replace(CFG, prefer=("far",)))
        assert_route_ok(self, route)
        self.assertEqual(route.choices, {"entry": "far"})

    def test_blocked_alternative_is_reported(self):
        route = plan(self.room(near_width=0.55))
        assert_route_ok(self, route)
        self.assertEqual(route.choices, {"entry": "far"})
        self.assertIn("near", route.rejected)

    def test_blocked_preferred_alternative_is_an_error(self):
        with self.assertRaises(PlanningError) as cm:
            plan(self.room(near_width=0.55), dataclasses.replace(CFG, prefer=("near",)))
        self.assertIn("preferred goal 'near'", str(cm.exception))


class BoulderTest(unittest.TestCase):
    def test_radius_rule(self):
        boulders = [{"centre": [0, 1], "radius": 0.2, "height": 0.533},
                    {"centre": [1, 1], "radius": 0.2},
                    {"centre": [-1, 1], "radius": 0.2, "height": 0.7}]
        cmap = make_map(box(4, 4), [pad((0, -1))], boulders=boulders)
        low = 0.8 * math.tan(math.radians(CFG.rangefinder_half_fov_deg)) + CFG.rangefinder_margin_m
        self.assertEqual(CFG.boulder_radii(cmap), [low, CFG.inscribed_radius_m, CFG.inscribed_radius_m])
        fixed = dataclasses.replace(CFG, boulder_inscribed_radius_m=0.4)
        self.assertEqual(fixed.boulder_radii(cmap), [0.4, 0.4, 0.4])

    def test_route_goes_around_a_boulder(self):
        cmap = make_map(box(5, 3), [pad((1.8, 0.0), radius=0.3)], launch=(-1.8, 0.0),
                        boulders=[{"centre": [0, 0], "radius": 0.23, "height": 0.533}])
        route = plan(cmap)
        assert_route_ok(self, route)
        pts = np.vstack([sample_segment(a.position, b.position, 0.01)
                         for a, b in zip(route.waypoints, route.waypoints[1:])])
        nearest = np.min(np.hypot(pts[:, 0], pts[:, 1])) - 0.23
        self.assertGreaterEqual(nearest, CFG.boulder_radii(cmap)[0] - 1e-9)


class StartTest(unittest.TestCase):
    def test_no_start_anywhere(self):
        cmap = make_map(box(4, 4), [pad((1, 1))])
        with self.assertRaises(PlanningError):
            plan(cmap)
        assert_route_ok(self, plan(cmap, start=(-1.0, -1.0)))


if __name__ == "__main__":
    unittest.main()
