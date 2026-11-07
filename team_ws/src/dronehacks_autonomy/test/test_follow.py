"""Trajectory evaluation, the Follower, the simulator and the pipeline."""

import dataclasses
import unittest

import numpy as np
from _maps import PUBLISHED, box, make_map, pad, serpentine

from dronehacks_autonomy.planning import PlannerConfig, load_map
from dronehacks_autonomy.planning.follower import Follower, FollowerConfig, WaypointFollower
from dronehacks_autonomy.planning.limits import VehicleLimits
from dronehacks_autonomy.planning.pipeline import build, fastest
from dronehacks_autonomy.planning.route import make_field
from dronehacks_autonomy.planning.sim import flown_metrics, simulate

PC = PlannerConfig()
LIM = VehicleLimits()


def l_corridor():
    """A 1.2 m wide L-shaped corridor: east 5 m, then north 4 m."""
    segs = box(9, 9) + [((-4.5, 0.6), (1.0, 0.6)), ((-4.5, -0.6), (2.2, -0.6)),
                        ((1.0, 0.6), (1.0, 4.5)), ((2.2, -0.6), (2.2, 4.5))]
    return make_map(segs, [pad((1.6, 3.6), radius=0.3)], launch=(-3.8, 0.0))


class TrajectoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fp = build(l_corridor())

    def test_evaluation_between_samples_is_continuous(self):
        traj = self.fp.trajectory
        ts = np.linspace(0, traj.duration, 997)
        pos = np.array([traj.at(t).position for t in ts])
        vel = np.array([traj.at(t).velocity for t in ts])
        self.assertLess(np.hypot(*np.diff(pos, axis=0).T).max(), 0.05)
        # Velocity matches the finite difference of position.
        fd = np.diff(pos, axis=0) / np.diff(ts)[:, None]
        mid = (vel[:-1] + vel[1:]) / 2
        self.assertLess(np.percentile(np.hypot(*(fd - mid).T), 99), 0.05)
        self.assertAlmostEqual(traj.at(traj.duration).progress, 1.0)
        self.assertAlmostEqual(traj.at(-1.0).s, 0.0)

    def test_heading_looks_into_the_turn(self):
        traj = self.fp.trajectory
        # Halfway down the first leg the heading is east; near the end, north.
        self.assertAlmostEqual(traj.at(traj.duration * 0.2).yaw, 0.0, delta=0.15)
        self.assertAlmostEqual(np.cos(traj.at(traj.duration * 0.95).yaw), 0.0, delta=0.2)


class FollowerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmap = l_corridor()
        cls.field = make_field(cls.cmap)
        cls.fp = build(cls.cmap)

    def test_lead_never_points_through_a_wall(self):
        # The lead is off by default (withdrawn in contracts/ros-graph.md);
        # this tests the opt-in lead's own guard.
        f = Follower(self.fp.trajectory, self.field, LIM, FollowerConfig(lead=True))
        t, clipped = 0.0, 0
        while t < self.fp.duration:
            ref = self.fp.trajectory.at(t).position
            sp = f.update(t, ref)
            self.assertGreaterEqual(self.field.segment_query(tuple(sp.reference), tuple(sp.position))[1], 0.0)
            clipped += sp.lead_scale < 1.0
            t += 0.02
        self.assertGreater(clipped, 0)   # the corner does clip the lead

    def test_default_follower_publishes_plain_poses(self):
        f = Follower(self.fp.trajectory, self.field, LIM)
        sp = f.update(0.0, self.fp.trajectory.xy[0])
        self.assertEqual(sp.lead_m, 0.0)
        self.assertEqual(tuple(sp.position), tuple(sp.reference))

    def test_reference_waits_for_a_stuck_drone(self):
        f = Follower(self.fp.trajectory, self.field, LIM)
        start = self.fp.trajectory.xy[0]
        for k in range(500):
            sp = f.update(k * 0.02, start)
        self.assertLess(sp.ref_time, 3.0)
        self.assertLess(sp.clock_rate, 1e-3)    # approaches 0 as the lag approaches lag_stop
        self.assertIn("held", sp.flags)

    def test_flies_the_corridor_without_touching(self):
        # Plain poses (the default) lag a moving reference by v / MPC_XY_P, so
        # the governor holds the reference back and the flight takes longer
        # than the plan; it must still finish with no touch.
        res = simulate(Follower(self.fp.trajectory, self.field, LIM), self.fp.route.waypoints[0].position, LIM)
        m = flown_metrics(res, self.field, PC.drone_extent_m, self.fp.path.xy)
        self.assertTrue(m["finished"])
        self.assertEqual(m["wall_touches"], 0)
        self.assertLess(m["time_s"], self.fp.duration * 2.5)

    def test_lead_beats_plain_poses(self):
        start = self.fp.route.waypoints[0].position
        led = simulate(Follower(self.fp.trajectory, self.field, LIM, FollowerConfig(lead=True)), start, LIM)
        plain = simulate(Follower(self.fp.trajectory, self.field, LIM), start, LIM)
        self.assertTrue(led.finished and plain.finished)
        self.assertLess(led.time_s, plain.time_s * 0.8)

    def test_path_lead_mode_also_flies(self):
        res = simulate(Follower(self.fp.trajectory, self.field, LIM, FollowerConfig(lead=True, lead_mode="path")),
                       self.fp.route.waypoints[0].position, LIM)
        self.assertTrue(res.finished)
        self.assertEqual(flown_metrics(res, self.field, PC.drone_extent_m)["wall_touches"], 0)

    def test_drifted_behind_a_corner_gets_no_command_through_a_wall(self):
        # Review of #24: with every lead blocked, _clear_fraction returned 0 and
        # the follower commanded the reference pose itself, straight through
        # the wall from wherever the drone had drifted. It must hold instead.
        traj = self.fp.trajectory
        for cfg in (FollowerConfig(), FollowerConfig(lead=True)):   # plain poses and zero-offset lead too
            with self.subTest(lead=cfg.lead):
                f = Follower(traj, self.field, LIM, cfg)
                t = 0.0
                while t < traj.duration:      # fly normally until the reference is round the corner
                    sp = f.update(t, tuple(traj.at(t).position))
                    t += 0.02
                    if traj.at(f._tau).position[0] > 1.3 and traj.at(f._tau).position[1] > 1.0:
                        break
                # The drone is back in the south leg, clear of walls, but the
                # reference is up the north leg: every straight line from the
                # drone to it crosses the corner wall.
                est = (0.0, 0.2)
                self.assertGreaterEqual(self.field.query(np.asarray(est))[1][0], 0.0)   # itself clear
                ref_before = f._tau
                sp = f.update(t + 0.02, est)
                self.assertIn("no_safe_target", sp.flags)
                self.assertLessEqual(f._tau, ref_before + 1e-9)                        # clock frozen
                self.assertEqual(tuple(sp.position), est)                               # hold, don't chase the reference
                self.assertGreaterEqual(self.field.segment_query(est, tuple(sp.position))[1], 0.0)
                self.assertNotEqual(tuple(sp.position), tuple(sp.reference))

    def test_waypoint_baseline_advances_at_arrival_radius(self):
        wf = WaypointFollower([(0, 0), (1, 0), (2, 0)], arrival_radius_m=0.3)
        self.assertEqual(tuple(wf.update(0.0, (0.0, 0.0)).position), (1.0, 0.0))
        self.assertEqual(tuple(wf.update(0.1, (0.75, 0.0)).position), (2.0, 0.0))
        self.assertTrue(wf.update(0.2, (1.95, 0.0)).done)


class PipelineTest(unittest.TestCase):
    def test_fastest_defaults_to_gate_b(self):
        # The window is opt-in (contracts/course.md leaves its clearance_radius
        # unreviewed), so the default route is Gate B.
        cmap = load_map(PUBLISHED)
        best, table = fastest(cmap)
        self.assertEqual(best.route.goals, ["tunnel", "gate_b", "pad"])
        self.assertEqual(len(table), 1)
        self.assertEqual(best.report["violations"], [])
        self.assertGreaterEqual(best.path.min_margin_m, 0.0)
        self.assertIn("window", best.route.rejected)

    def test_fastest_picks_by_flight_time_when_windows_are_allowed(self):
        cmap = load_map(PUBLISHED)
        best, table = fastest(cmap, PlannerConfig(allow_windows=True))
        self.assertEqual(len(table), 2)
        times = {c["landing_entry"]: float(o.split()[0]) for c, o in table}
        self.assertLess(times["window"], times["gate_b"])
        self.assertEqual(best.route.goals, ["tunnel", "window", "pad"])

    def test_fastest_honors_an_explicitly_preferred_window(self):
        # route.plan keeps a window named in prefer; fastest must too, even
        # with allow_windows unset (an explicit choice, not a default).
        cmap = load_map(PUBLISHED)
        best, table = fastest(cmap, PlannerConfig(prefer=("window",)))
        self.assertEqual(len(table), 1)
        self.assertEqual(best.route.goals, ["tunnel", "window", "pad"])
        self.assertEqual(best.report["violations"], [])

    def test_random_mazes_build_cleanly(self):
        for seed in range(3):
            with self.subTest(seed=seed):
                fp = build(serpentine(seed))
                self.assertEqual(fp.report["violations"], [])
                self.assertGreaterEqual(fp.path.min_margin_m, 0.0)

    def test_faster_cap_is_not_slower(self):
        cmap = l_corridor()
        slow = build(cmap, limits=LIM)
        quick = build(cmap, limits=dataclasses.replace(LIM, v_max_mps=2.0))
        self.assertLessEqual(quick.duration, slow.duration + 1e-6)


if __name__ == "__main__":
    unittest.main()
