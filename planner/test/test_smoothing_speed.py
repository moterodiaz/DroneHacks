"""Exact clearance, smoothing and speed profiling on synthetic paths."""

import dataclasses
import math
import random
import unittest

import numpy as np
from _maps import box, make_map, pad

from dronehacks_planner import PlannerConfig
from dronehacks_planner.clearance import conservative_min_margin, sample_segment
from dronehacks_planner.limits import VehicleLimits
from dronehacks_planner.route import Route, Waypoint, make_field, plan
from dronehacks_planner.smoothing import SmoothingConfig, smooth_route
from dronehacks_planner.speed import ProfileConfig, ProfileError, check_profile, speed_profile

PC = PlannerConfig()
LIM = VehicleLimits()


def polyline_route(points, cmap):
    """A route through fixed points (no planner), for testing later stages in isolation."""
    base = plan(cmap)
    kinds = ["start"] + ["via"] * (len(points) - 2) + ["goal"]
    wps = [Waypoint(tuple(map(float, p)), k, "pad" if k == "goal" else None, 1.0) for p, k in zip(points, kinds)]
    return dataclasses.replace(base, waypoints=wps, passes={})


def open_room(w=8.0, h=8.0, goal=(3.0, 3.0)):
    return make_map(box(w, h), [pad(goal, radius=0.3)], launch=(-3.0, -3.0))


class ExactClearanceTest(unittest.TestCase):
    def test_segment_query_matches_fine_sampling(self):
        cmap = make_map(box(6, 6) + [((-1, 0), (1, 0.5)), ((0, -2), (0, -1))], [pad((2, 2))],
                        boulders=[{"centre": [1.5, -1.5], "radius": 0.3}])
        field = make_field(cmap)
        rng = random.Random(3)
        for _ in range(200):
            a = (rng.uniform(-2.8, 2.8), rng.uniform(-2.8, 2.8))
            b = (rng.uniform(-2.8, 2.8), rng.uniform(-2.8, 2.8))
            surface, margin, _ = field.segment_query(a, b)
            s, m = field.query(sample_segment(a, b, 0.0005))
            self.assertLessEqual(surface, max(s.min(), 0.0) + 1e-9)       # never optimistic
            if s.min() > 0:
                self.assertAlmostEqual(margin, m.min(), delta=5e-4)

    def test_crossing_a_wall_is_zero_distance(self):
        cmap = make_map([((0, -1), (0, 1))], [pad((2, 0))], thickness=0.0)
        surface, margin, at = make_field(cmap).segment_query((-1, 0), (1, 0))
        self.assertAlmostEqual(surface, 0.0)
        self.assertAlmostEqual(at[0], 0.0, places=9)
        self.assertLess(margin, 0)

    def test_collinear_and_degenerate_segments_are_exact(self):
        # On one line, a leg that doesn't overlap the wall is measured by its gap,
        # not reported as crossing (review of #24 at f644ace, clearance.py:145).
        cmap = make_map([((0, 3), (2, 3)), ((5, -2), (5, -2))], [pad((0, 0))], thickness=0.2)
        field = make_field(cmap)
        cases = {
            ((4, 3), (6, 3)): 2.0 - 0.1,       # collinear, 2 m past the wall's end
            ((-3, 3), (-1, 3)): 1.0 - 0.1,     # collinear, 1 m before its start
            ((1, 3), (6, 3)): -0.1,            # collinear and overlapping: inside
            ((2, 3), (4, 3)): -0.1,            # collinear, touching at an end
            ((4, 3), (4, 3)): 2.0 - 0.1,       # a zero-length leg on the wall's line
            ((1, 3), (1, 3)): -0.1,            # a zero-length leg on the wall
            ((1, 2), (1, 4)): -0.1,            # a proper crossing
            ((2, 2), (2, 4)): -0.1,            # crossing through the wall's end point
            ((5, 0), (5, 1)): 2.0 - 0.1,       # in line with a post, past it
            ((5, -3), (5, -1)): -0.1,          # through the post
        }
        for (a, b), want in cases.items():
            surface, margin, _ = field.segment_query(a, b)
            s, _ = field.query(sample_segment(a, b, 0.0005))
            self.assertAlmostEqual(surface, want, places=9, msg=f"{a} -> {b}")
            self.assertAlmostEqual(surface, s.min(), delta=1e-3, msg=f"{a} -> {b}")
            self.assertAlmostEqual(margin, surface - PC.inscribed_radius_m, places=9)

    def test_conservative_bound_catches_what_samples_miss(self):
        # A thin post between two samples 0.3 m apart: both samples look clear.
        cmap = make_map([((0, 0), (0, 0))], [pad((2, 0))], thickness=0.0)
        field = make_field(cmap, dataclasses.replace(PC, inscribed_radius_m=0.1))
        pts = np.array([[-0.15, 0.0], [0.15, 0.0]])
        self.assertGreater(field.margin(pts).min(), 0.0)
        bound, _ = conservative_min_margin(field, pts)
        self.assertLess(bound, 0.0)

    def test_ray_to_a_wall(self):
        cmap = make_map([((2, -5), (2, 5))], [pad((0, 0))], thickness=0.0)
        free = make_field(cmap).wall_free_distance(np.array([[0.0, 0.0], [0.0, 1.0]]),
                                                   np.array([[1.0, 0.0], [0.0, 1.0]]), stop_at=0.28, max_range=5)
        self.assertAlmostEqual(free[0], 2.0 - 0.28, delta=2e-3)
        self.assertAlmostEqual(free[1], 5.0)   # parallel to the wall: never hits within range


class SmoothingTest(unittest.TestCase):
    def smooth(self, points, cmap=None, **cfg):
        cmap = cmap or open_room()
        field = make_field(cmap)
        return smooth_route(polyline_route(points, cmap), cmap, field, PC, SmoothingConfig(**cfg)), field

    def test_straight_stays_straight(self):
        path, _ = self.smooth([(-3, -3), (3, 3)])
        self.assertLess(np.abs(path.curvature).max(), 1e-9)
        np.testing.assert_allclose(path.xy[0], (-3, -3))
        np.testing.assert_allclose(path.xy[-1], (3, 3))
        self.assertAlmostEqual(path.length_m, math.dist((-3, -3), (3, 3)), places=6)

    def test_right_angle_is_continuous_and_bounded(self):
        path, _ = self.smooth([(-3, -3), (2, -3), (2, 3)])
        k = np.abs(path.curvature)
        self.assertAlmostEqual(k.max(), 1.0 / path.radii[1], delta=0.05 / path.radii[1])
        # Continuous curvature: no jump between neighbouring samples.
        self.assertLess(np.abs(np.diff(path.curvature)).max(), 0.05)
        self.assertGreaterEqual(path.min_margin_m, 0.0)
        np.testing.assert_allclose(path.xy[-1], (2, 3))

    def test_s_curve_and_close_turns(self):
        path, _ = self.smooth([(-3, -3), (-1, -3), (-1, -2.6), (1, -2.6), (1, -2.2), (3, -2.2), (3, 3)])
        self.assertGreaterEqual(path.min_margin_m, 0.0)
        self.assertLess(path.length_m, 6 + 0.4 + 2 + 0.4 + 2 + 5.2 + 1e-6)

    def test_fillets_are_no_riskier_than_the_polyline(self):
        # A hairpin round a wall end: every fillet cuts towards the wall end.
        segs = box(6, 6) + [((-3, 0), (0.5, 0))]
        cmap = make_map(segs, [pad((-2, 0.5), radius=0.3)], launch=(-2, -0.5))
        pts = [(-2, -0.5), (0.9, -0.5), (0.9, 0.5), (-2, 0.5)]
        path, field = self.smooth(pts, cmap=cmap)
        poly = min(field.segment_query(a, b)[1] for a, b in zip(pts, pts[1:]))
        comfort = PC.comfort_clearance_m - PC.inscribed_radius_m
        self.assertGreaterEqual(path.min_margin_m, 0.0)
        # The proven bound, not just the samples, keeps within 1 cm of the polyline.
        self.assertGreaterEqual(path.min_margin_m, min(comfort, poly) - SmoothingConfig().no_riskier_tolerance_m - 1e-9)

    def test_short_stub_is_merged(self):
        path, _ = self.smooth([(-3, -3), (0, 0), (0.05, 0.1), (3, 3)])
        self.assertLessEqual(len(path.radii), 3)

    def test_published_route_keeps_whole_drone_crossings(self):
        from _maps import PUBLISHED
        from dronehacks_planner import load_map
        from dronehacks_planner.sim import openings_crossed
        cmap = load_map(PUBLISHED)
        route = plan(cmap)          # Gate B by default; the window is opt-in
        path = smooth_route(route, cmap, make_field(cmap))
        self.assertGreaterEqual(path.min_margin_m, 0.0)
        self.assertEqual(openings_crossed(path.xy, route, cmap, PC.drone_extent_m), {"tunnel": True, "gate_b": True})


class SpeedTest(unittest.TestCase):
    def profile(self, points, limits=LIM, cmap=None, smoothing=None, **cfg):
        cmap = cmap or open_room(20, 20, goal=(9, 9))
        field = make_field(cmap)
        path = smooth_route(polyline_route(points, cmap), cmap, field, PC, smoothing or SmoothingConfig())
        return path, speed_profile(path, limits, field, PC, ProfileConfig(**cfg))

    def test_long_straight_reaches_v_max_and_stops(self):
        path, prof = self.profile([(-8, 0), (8, 0)])
        self.assertAlmostEqual(prof.v.max(), LIM.v_max_mps, places=3)
        self.assertEqual(prof.v[0], 0.0)
        self.assertAlmostEqual(prof.v[-1], 0.0, places=6)
        self.assertEqual(check_profile(path, prof, LIM)["violations"], [])

    def test_tight_curve_respects_lateral_acceleration(self):
        fast = dataclasses.replace(LIM, v_max_mps=3.0)
        path, prof = self.profile([(-8, -8), (0, -8), (0, 8)], limits=fast)
        lateral = prof.v ** 2 * np.abs(path.curvature)
        self.assertLessEqual(lateral.max(), fast.a_lat_mps2 * 1.02)
        self.assertEqual(check_profile(path, prof, fast)["violations"], [])

    def test_brakes_before_and_accelerates_after_a_corner(self):
        fast = dataclasses.replace(LIM, v_max_mps=2.0)
        path, prof = self.profile([(-8, -8), (0, -8), (0, 8)], limits=fast)
        apex = int(np.argmax(np.abs(path.curvature)))
        before = prof.v[apex - 300:apex]
        after = prof.v[apex:apex + 300]
        self.assertGreater(before[0], prof.v[apex] + 0.3)   # slowed into the corner
        self.assertGreater(after[-1], prof.v[apex] + 0.3)   # sped up out of it
        self.assertGreater(prof.v.max(), 1.9)

    def test_nonzero_start_speed(self):
        path, prof = self.profile([(-8, 0), (8, 0)], v_start_mps=0.8)
        self.assertAlmostEqual(prof.v[0], 0.8, places=3)
        self.assertEqual(check_profile(path, prof, LIM)["violations"], [])

    def test_start_too_fast_to_stop_is_infeasible(self):
        # 2.5 m/s needs 2.5^2 / (2 * 2) = 1.56 m to stop; the path is 0.6 m long.
        fast = dataclasses.replace(LIM, v_max_mps=3.0)
        with self.assertRaises(ProfileError):
            self.profile([(-0.3, 0), (0.3, 0)], limits=fast, v_start_mps=2.5)

    def test_end_speed_too_fast_to_reach_is_infeasible(self):
        # Review of #24: an unreachable v_end_mps used to be silently clamped,
        # so a caller joining trajectories got a different end speed than asked.
        # 3.0 m/s needs 9 / (2 * 2) = 2.25 m to brake from; the path is 0.6 m long.
        fast = dataclasses.replace(LIM, v_max_mps=5.0)
        with self.assertRaises(ProfileError):
            self.profile([(-0.3, 0), (0.3, 0)], limits=fast, v_end_mps=3.0)

    def test_reachable_end_speed_is_matched(self):
        path, prof = self.profile([(-8, 0), (8, 0)], v_end_mps=0.5)
        self.assertAlmostEqual(prof.v[-1], 0.5, places=3)
        self.assertEqual(check_profile(path, prof, LIM)["violations"], [])

    def test_coupled_acceleration(self):
        tight = dataclasses.replace(LIM, v_max_mps=2.0, a_lat_mps2=1.0, a_horizontal_mps2=1.2)
        path, prof = self.profile([(-8, -8), (0, -8), (0, 8)], limits=tight, longitudinal_jerk_smoothing=False,
                                  lateral_jerk_limit=False)
        total = np.hypot(prof.a_long, prof.a_lat)
        self.assertLessEqual(np.percentile(total, 99.5), tight.a_horizontal_mps2 * 1.05)

    def test_jerk_is_limited_and_violations_are_reported(self):
        path, prof = self.profile([(-8, -8), (0, -8), (0, 8)])
        rep = check_profile(path, prof, LIM)
        self.assertLessEqual(rep["max_jerk_long_mps3"], LIM.j_max_mps3 * 1.02)
        self.assertLessEqual(rep["max_jerk_lat_mps3"], LIM.j_max_mps3 * 1.02)
        raw = speed_profile(path, LIM, make_field(open_room(20, 20, goal=(9, 9))), PC,
                            ProfileConfig(longitudinal_jerk_smoothing=False, lateral_jerk_limit=False))
        self.assertTrue(any("jerk" in v for v in check_profile(path, raw, LIM)["violations"]))

    def test_braking_room_near_a_wall(self):
        # The path ends 0.6 m short of a wall it heads straight at.
        cmap = make_map(box(20, 20) + [((3.0, -2), (3.0, 2))], [pad((2.4, 0), radius=0.3)], launch=(-8, 0))
        fast = dataclasses.replace(LIM, v_max_mps=3.0)
        path, prof = self.profile([(-8, 0), (2.4, 0)], limits=fast, cmap=cmap)
        free = make_field(cmap).wall_free_distance(path.xy, path.tangent, PC.drone_extent_m, max_range=20)
        need = np.array([fast.stopping_distance(v) for v in prof.v])
        self.assertTrue(np.all(need <= free + 1e-3))

    def test_constant_speed_baseline(self):
        path, prof = self.profile([(-8, -8), (0, -8), (0, 8)], constant_speed=True)
        inner = prof.v[(prof.s > 2.0) & (prof.s < path.length_m - 2.0)]
        self.assertLess(inner.max() - inner.min(), 0.02)


if __name__ == "__main__":
    unittest.main()
