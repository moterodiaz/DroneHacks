"""Mission logic tests (no ROS). From the package directory:

    python3 -m pytest -q test/          or      python3 -m unittest discover -s test
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dronehacks_pathfly.mission import Config, Mission, Observation  # noqa: E402
from dronehacks_pathfly.path import (ArenaToOdom, Polyline,  # noqa: E402
                                     load_path_csv)
from dronehacks_pathfly.sim import DEFAULT_PATH, run  # noqa: E402

EXPECTED_PHASES = ['WAIT', 'TAKEOFF', 'PATH', 'PATH_END', 'LAND_1', 'GROUND_1',
                   'CLIMB_2', 'FORWARD', 'FORWARD_END', 'LAND_2', 'DONE',
                   'COMPLETE']
EXPECTED_PHASES_TURN = EXPECTED_PHASES[:7] + ['YAW'] + EXPECTED_PHASES[7:]


def dist_to_polyline(p, pts):
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        f = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx
                                                  + (p[1] - a[1]) * dy) / L2))
        best = min(best, math.hypot(p[0] - a[0] - f * dx, p[1] - a[1] - f * dy))
    return best


class TestPath(unittest.TestCase):
    def test_loads_planner_output(self):
        pts, margins = load_path_csv(DEFAULT_PATH)
        # path-planning 11db65f (tunnel walls tightened, margin-first plan,
        # east-facing footprint)
        self.assertEqual(len(pts), 135)
        self.assertEqual(pts[0], (0.580, 4.596))
        self.assertEqual(pts[-1], (0.900, 1.350))
        self.assertIsNotNone(margins)
        self.assertAlmostEqual(Polyline(pts).length, 13.31, delta=0.05)

    def test_warns_when_flown_off_the_planned_heading(self):
        pts, margins = load_path_csv(DEFAULT_PATH)
        self.assertIsNone(Mission(pts, margins, Config()).heading_warning)
        for cfg in (Config(start_yaw_deg=90.0), Config(hold_heading=False)):
            m = Mission(pts, margins, cfg)
            self.assertIn('east', m.heading_warning)
            self.assertIn('WARNING', m.summary())

    def test_flown_data_matches_the_path_planning_copy(self):
        """data/ is what flies; path_planning/ is the source repo copied
        unchanged (data/SOURCE.md). They must be byte-identical."""
        pkg = os.path.dirname(os.path.dirname(DEFAULT_PATH))
        for name in ('path_dense.csv', 'path_waypoints.csv'):
            with open(os.path.join(pkg, 'data', name), 'rb') as a, \
                    open(os.path.join(pkg, 'path_planning', name), 'rb') as b:
                self.assertEqual(a.read(), b.read(), name)
        for name in ('plan_path.py', 'STEP MAP.STEP', 'path.png'):
            self.assertTrue(os.path.exists(os.path.join(pkg, 'path_planning', name)))

    def test_waypoints_and_dense_path_agree(self):
        dense, _ = load_path_csv(DEFAULT_PATH)
        wps, _ = load_path_csv(os.path.join(os.path.dirname(DEFAULT_PATH),
                                            'path_waypoints.csv'))
        self.assertEqual(dense[0], wps[0])
        self.assertEqual(dense[-1], wps[-1])
        for w in wps:
            self.assertLess(dist_to_polyline(w, dense), 0.03)

    def test_transform_round_trip(self):
        tf = ArenaToOdom((0.58, 4.596), math.radians(30), (1.0, -2.0),
                         math.radians(-45))
        self.assertEqual(tf.point((0.58, 4.596)), (1.0, -2.0))
        q = tf.point((2.0, 5.0))
        back = tf.inverse_point(q)
        self.assertAlmostEqual(back[0], 2.0)
        self.assertAlmostEqual(back[1], 5.0)
        # Arena direction at the nose maps onto the odom nose direction.
        nose_a = (0.58 + math.cos(math.radians(30)), 4.596 + math.sin(math.radians(30)))
        n = tf.point(nose_a)
        self.assertAlmostEqual(math.atan2(n[1] + 2.0, n[0] - 1.0), math.radians(-45))


class TestFullMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pts, _ = load_path_csv(DEFAULT_PATH)
        cls.results = {}
        # key -> (start_yaw_deg, hold_heading); NaN = nose along the approach
        nan = float('nan')
        cls.cases = {'auto': (nan, True), 90.0: (90.0, True),
                     'turn': (nan, False), 'east': (0.0, True),
                     'east_turn': (0.0, False)}
        for key, (yaw, hold) in cls.cases.items():
            cls.results[key] = run(Config(start_yaw_deg=yaw, hold_heading=hold),
                                   quiet=True)
            cls.results[key] += (run.commands,)

    def rows(self, key, *phases):
        return [r for r in self.results[key][3] if r[1] in phases]

    def nose(self, key):
        return self.results[key][0].start_yaw_deg

    def test_phase_sequence_and_disarm(self):
        for key, (mission, veh, phases, track, _) in self.results.items():
            expected = EXPECTED_PHASES if self.cases[key][1] else EXPECTED_PHASES_TURN
            self.assertEqual([p for _, p in phases], expected, key)
            self.assertFalse(veh.armed)
            self.assertEqual(veh.disarm_reason, 'COM_DISARM_LAND')

    def test_default_never_yaws(self):
        """hold_heading (default): the nose stays on the start heading for the
        whole flight, and no yaw rate beyond small corrections is commanded."""
        for key in ('auto', 90.0):
            yaw0 = self.nose(key)
            for r in self.results[key][3]:
                self.assertAlmostEqual(r[5], yaw0, delta=1.0, msg=(key, r[:2]))
            for _, _, tw in self.results[key][4]:
                if tw is not None:
                    self.assertLess(abs(tw[3]), math.radians(2.0))

    def test_both_modes_land_in_the_same_place(self):
        a, b = self.results['auto'][3][-1], self.results['turn'][3][-1]
        self.assertLess(math.hypot(a[2] - b[2], a[3] - b[3]), 0.05)

    def test_liftoff_to_0p2_before_moving(self):
        take = self.rows('auto', 'TAKEOFF')
        self.assertAlmostEqual(take[-1][4], 0.2, delta=0.08)
        start = self.pts[0]
        for r in take:
            self.assertLess(math.hypot(r[2] - start[0], r[3] - start[1]), 0.02)

    def test_climbs_while_moving(self):
        path = self.rows('auto', 'PATH')
        start = self.pts[0]
        # Somewhere in the first metre: well above 0.2, well below 1.25,
        # and already moved along the path.
        mid = [r for r in path
               if 0.5 < math.hypot(r[2] - start[0], r[3] - start[1]) < 0.9]
        self.assertTrue(mid)
        for r in mid:
            self.assertGreater(r[4], 0.3)
            self.assertLess(r[4], 1.15)
        late = [r for r in path if r[0] > path[0][0] + 10.0]
        for r in late:
            self.assertAlmostEqual(r[4], 1.25, delta=0.06)
        self.assertLess(max(r[4] for r in path), 1.32)

    def test_follows_the_path(self):
        for yaw in ('auto', 90.0):
            for r in self.rows(yaw, 'PATH', 'PATH_END'):
                self.assertLess(dist_to_polyline((r[2], r[3]), self.pts), 0.05)

    def test_first_touchdown_at_path_end(self):
        g1 = self.rows('auto', 'GROUND_1')
        end = self.pts[-1]
        self.assertLess(math.hypot(g1[0][2] - end[0], g1[0][3] - end[1]), 0.05)
        self.assertLess(g1[0][4], 0.01)

    def test_reascends_quickly(self):
        """Back in the air well inside COM_DISARM_LAND (5 s) of touching down."""
        track = self.results['auto'][3]
        land_t = next(r[0] for r in track if r[1] == 'LAND_1' and r[4] <= 1e-6)
        up_t = next(r[0] for r in track if r[1] == 'CLIMB_2' and r[4] > 0.05)
        self.assertLess(up_t - land_t, 3.5)

    def test_default_nose_matches_the_drawing(self):
        """Default nose = along the final approach (162.3 deg); the leg from
        the goal back to START is then 66.6 deg clockwise of it (the
        drawing's 66.61 deg)."""
        m = self.results['auto'][0]
        self.assertAlmostEqual(m.start_yaw_deg, 162.3, delta=0.3)
        self.assertAlmostEqual(m.arena_leg_yaw_deg, 95.6, delta=0.2)
        self.assertAlmostEqual(m.start_yaw_deg - m.arena_leg_yaw_deg, 66.61,
                               delta=0.3)

    def test_hop_and_last_leg_back_to_start(self):
        start, goal = self.pts[0], self.pts[-1]
        leg_dir = math.degrees(math.atan2(start[1] - goal[1], start[0] - goal[0]))
        for key, (yaw, hold) in self.cases.items():
            fwd = self.rows(key, 'FORWARD', 'FORWARD_END')
            self.assertAlmostEqual(fwd[0][4], 2.0, delta=0.08)
            nose = self.nose(key) if hold else leg_dir
            for r in fwd:
                self.assertAlmostEqual(r[5], nose, delta=3.0, msg=key)
                self.assertAlmostEqual(r[4], 2.0, delta=0.1)
                # on the straight line goal -> start
                self.assertLess(dist_to_polyline((r[2], r[3]), [goal, start]), 0.06)
            a, b = fwd[0], fwd[-1]
            self.assertAlmostEqual(math.hypot(b[2] - a[2], b[3] - a[3]), 3.26,
                                   delta=0.12)
            direction = math.degrees(math.atan2(b[3] - a[3], b[2] - a[2]))
            self.assertAlmostEqual(direction, leg_dir, delta=2.0)

    def test_default_nose_is_east(self):
        self.assertEqual(Config().start_yaw_deg, 0.0)
        m = Mission(*load_path_csv(DEFAULT_PATH), Config())
        self.assertIn('east, along arena +x', m.summary())
        self.assertIn('95.6 deg counter-clockwise of the nose', m.summary())

    def test_east_turn_mode_turns_left(self):
        turn = self.rows('east_turn', 'YAW')
        self.assertAlmostEqual(turn[-1][5] - turn[0][5], 95.6, delta=3.0)

    def test_turn_mode_turns_66_clockwise(self):
        turn = self.rows('turn', 'YAW')
        self.assertAlmostEqual(turn[0][4], 2.0, delta=0.08)
        self.assertAlmostEqual(turn[0][5] - turn[-1][5], 66.6, delta=3.0)

    def test_final_touchdown_on_start(self):
        start = self.pts[0]
        for key in self.cases:
            last = self.results[key][3][-1]
            self.assertLess(math.hypot(last[2] - start[0], last[3] - start[1]),
                            0.08, key)
            self.assertLess(last[4], 0.01)

    def test_never_commands_up_when_landing_or_done(self):
        for t, phase, tw in self.results['auto'][4]:
            if phase in ('LAND_1', 'GROUND_1', 'LAND_2', 'DONE'):
                self.assertLessEqual(tw[2], 0.0, (t, phase))

    def test_command_speed_capped(self):
        for t, phase, tw in self.results['auto'][4]:
            if tw is not None:
                self.assertLessEqual(math.sqrt(sum(c * c for c in tw[:3])),
                                     1.0 + 1e-9)

    def test_noise_and_drift(self):
        mission, veh, phases, track = run(Config(), noise=0.02,
                                          wind=(0.05, -0.05), quiet=True)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)
        for r in track:
            if r[1] in ('PATH', 'PATH_END'):
                self.assertLess(dist_to_polyline((r[2], r[3]), self.pts), 0.08)


    def test_goal_dead_end_under_heavy_drift(self):
        """The planner leaves ~80 mm of margin at the goal. Even with 10 cm/s
        of drift PX4 does not see, stay within 60 mm of the path (and so of
        the walls' planned margin) while settling and descending there, and
        touch down within 100 mm of the goal (short of it along the corridor
        is fine; sideways is what the path check above bounds)."""
        mission, veh, phases, track = run(Config(start_yaw_deg=180.0),
                                          noise=0.03, wind=(0.1, 0.1),
                                          quiet=True)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)
        near = [r for r in track if r[1] in ('PATH_END', 'LAND_1') and r[4] > 0.01]
        self.assertTrue(near)
        for r in near:
            self.assertLess(dist_to_polyline((r[2], r[3]), self.pts), 0.06)
        end = self.pts[-1]
        g1 = next(r for r in track if r[1] == 'GROUND_1')
        self.assertLess(dist_to_polyline((g1[2], g1[3]), self.pts), 0.06)
        self.assertLess(math.hypot(g1[2] - end[0], g1[3] - end[1]), 0.10)


CONFIG_DIR = os.path.join(os.path.dirname(DEFAULT_PATH), '..', 'config')


def flat_yaml_params(name):
    """The `key: number` lines of a config file (no PyYAML in the CI image)."""
    import re
    out = {}
    with open(os.path.join(CONFIG_DIR, name), encoding='utf-8') as f:
        for line in f:
            m = re.match(r'^\s{4}(\w+):\s*([-+0-9.eE]+)\s*(#.*)?$', line)
            if m:
                out[m.group(1)] = float(m.group(2))
    return out


def goal_time(phases):
    """Seconds from Offboard (TAKEOFF) to touchdown 1 at the GOAL."""
    t0 = next(t for t, p in phases if p == 'TAKEOFF')
    return next(t for t, p in phases if p == 'GROUND_1') - t0


class TestFastProfile(unittest.TestCase):
    """config/fast.yaml: the same path, faster; touch the GOAL within 60 s."""

    @classmethod
    def setUpClass(cls):
        cls.pts, cls.margins = load_path_csv(DEFAULT_PATH)
        cls.fast = flat_yaml_params('fast.yaml')
        cls.safe = flat_yaml_params('pathfly.yaml')

    def cfg(self, **kw):
        c = Config(**{k: v for k, v in self.safe.items()
                      if k in Config.field_names()})
        for k, v in self.fast.items():
            setattr(c, k, v)
        for k, v in kw.items():
            setattr(c, k, v)
        return c

    def test_profile_has_the_expected_fields(self):
        self.assertEqual(self.fast['path_speed_mps'], 1.0)
        self.assertGreater(self.fast['ki_cross_xy'], 0.0)
        for k in self.fast:
            self.assertIn(k, Config.field_names(), k)
        self.cfg().validate()

    def test_same_path_and_nose(self):
        m = Mission(self.pts, self.margins, self.cfg())
        self.assertEqual(m.arena_points, Mission(self.pts, self.margins,
                                                 Config()).arena_points)
        self.assertEqual(m.start_yaw_deg, 0.0)
        self.assertIsNone(m.heading_warning)

    def test_goal_within_a_minute_and_faster_than_safe(self):
        _, _, phases, track = run(self.cfg(), quiet=True)
        _, _, safe_phases, _ = run(self.cfg(**{k: getattr(Config(), k)
                                               for k in self.fast}), quiet=True)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)
        self.assertLess(goal_time(phases), 42.0)
        self.assertLess(goal_time(phases), goal_time(safe_phases) - 10.0)
        g1 = next(r for r in track if r[1] == 'GROUND_1')
        end = self.pts[-1]
        self.assertLess(math.hypot(g1[2] - end[0], g1[3] - end[1]), 0.05)

    def test_goal_within_a_minute_with_lag_noise_and_push(self):
        """PX4 lag 0.7 s (measured in flight), 2 cm noise, 5 cm/s push: the
        drone never strays further from the path than the planner's wall
        margin there (less 5 mm)."""
        import dronehacks_pathfly.sim as sim
        line = Polyline(self.pts, self.margins)
        saved = sim.Vehicle.__init__.__defaults__
        sim.Vehicle.__init__.__defaults__ = (0.7,) + saved[1:]
        try:
            for wind in ((0.05, 0.0), (0.0, 0.05), (0.0, -0.05), (-0.05, 0.0)):
                _, _, phases, track = run(self.cfg(stop_after_landing_1=True),
                                          noise=0.02, wind=wind, quiet=True)
                self.assertNotIn('HOLD', [p for _, p in phases], wind)
                self.assertLess(goal_time(phases), 50.0, wind)
                s = 0.0
                for r in track:
                    if r[1] in ('PATH', 'PATH_END', 'LAND_1') and r[4] > 0.15:
                        s = line.project((r[2], r[3]), s, window=0.5)
                        dev = math.dist(line.at(s), (r[2], r[3]))
                        self.assertLess(dev, line.margin_at(s) - 0.005,
                                        (wind, r[:4], s))
        finally:
            sim.Vehicle.__init__.__defaults__ = saved

    def test_slow_where_the_margin_is_thin(self):
        m = Mission(self.pts, self.margins, self.cfg())
        line, prof = m.arena_line, m.profile
        for i, s in enumerate(line.s):
            if line.margin_at(s) <= 0.09:
                self.assertLessEqual(prof[i], 0.17, s)
        self.assertAlmostEqual(max(prof), 1.0, delta=1e-6)

    def test_cross_track_integral_cancels_a_steady_push(self):
        def worst(ki):
            _, _, _, track = run(self.cfg(ki_cross_xy=ki, stop_after_landing_1=True),
                                 wind=(0.0, 0.08), quiet=True)
            return max(dist_to_polyline((r[2], r[3]), self.pts)
                       for r in track if r[1] == 'PATH' and r[4] > 0.15)
        self.assertLess(worst(0.5), worst(0.0) - 0.01)

    def test_safe_defaults_unchanged(self):
        """The new options are off unless fast.yaml turns them on."""
        c = Config()
        self.assertEqual((c.ki_cross_xy, c.slow_margin_m, c.end_decel_mps2),
                         (0.0, 0.0, 0.0))
        for k in ('path_speed_mps', 'path_min_speed_mps', 'full_speed_margin_m',
                  'descent_speed_mps', 'final_descent_speed_mps'):
            self.assertEqual(self.safe[k], getattr(c, k), k)


class TestSpeedProfile(unittest.TestCase):
    def test_slow_margin_and_end_decel(self):
        from dronehacks_pathfly.path import speed_profile
        pts = [(0.0, 0.0), (10.0, 0.0)]
        line = Polyline(pts, [0.1, 0.1])
        v = speed_profile(line, 1.0, 0.2, 0.5, 0.5, 0.6, slow_margin=0.1)
        self.assertAlmostEqual(max(v), 0.2, delta=1e-6)
        line = Polyline(pts, [1.0, 1.0])
        hard = speed_profile(line, 1.0, 0.2, 0.5, 0.5, 0.6)
        soft = speed_profile(line, 1.0, 0.2, 0.5, 0.5, 0.6, end_decel=0.25,
                             end_decel_length=2.0)
        i = line.s.index(min(line.s, key=lambda s: abs(s - 9.5)))
        self.assertAlmostEqual(soft[i], math.sqrt(2 * 0.25 * 0.5), delta=0.01)
        self.assertGreater(hard[i], soft[i])
        j = line.s.index(min(line.s, key=lambda s: abs(s - 5.0)))
        self.assertEqual(hard[j], soft[j])


class TestBringUp(unittest.TestCase):
    """Staged tests, the heading-offset correction and the stage files."""

    @classmethod
    def setUpClass(cls):
        cls.pts, _ = load_path_csv(DEFAULT_PATH)

    def test_stage_files_are_valid_overrides(self):
        try:
            import yaml
        except ImportError:     # the repo CI image has no PyYAML; ROS does
            self.skipTest('PyYAML not installed')
        cfg_dir = os.path.join(os.path.dirname(DEFAULT_PATH), '..', 'config')
        names = set(Config.field_names()) | {
            'path_csv', 'odom_to_ned_yaw_offset_rad', 'rate_hz'}
        for f in ('pathfly.yaml', 'fast.yaml', 'stage1.yaml', 'stage2.yaml',
                  'stage3.yaml'):
            with open(os.path.join(cfg_dir, f), encoding='utf-8') as fh:
                params = yaml.safe_load(fh)['dronehacks_pathfly']['ros__parameters']
            for k, v in params.items():
                self.assertIn(k, names, (f, k))
                if k in Config.field_names():
                    want = type(getattr(Config(), k))
                    self.assertIsInstance(v, want, (f, k, v))

    def stage(self, **kw):
        return run(Config(**kw), quiet=True)

    def test_stage1_short_hop_and_stop(self):
        m, veh, phases, track = self.stage(path_limit_m=0.4,
                                           stop_after_landing_1=True)
        self.assertEqual([p for _, p in phases],
                         ['WAIT', 'TAKEOFF', 'PATH', 'PATH_END', 'LAND_1',
                          'GROUND_1', 'DONE', 'COMPLETE'])
        self.assertAlmostEqual(m.arena_line.length, 0.4, delta=0.01)
        self.assertIn('STAGED TEST', m.summary())
        last = track[-1]
        end = m.arena_points[-1]
        self.assertLess(math.hypot(last[2] - end[0], last[3] - end[1]), 0.05)
        self.assertEqual(veh.disarm_reason, 'COM_DISARM_LAND')
        self.assertLess(max(r[4] for r in track), 0.6)

    def test_stage2_through_the_north_corridor(self):
        m, veh, phases, track = self.stage(path_limit_m=2.6,
                                           stop_after_landing_1=True)
        self.assertEqual(phases[-1][1], 'COMPLETE')
        for r in track:
            if r[1] in ('PATH', 'PATH_END'):
                self.assertLess(dist_to_polyline((r[2], r[3]), self.pts), 0.04)

    def test_stage3_full_path_no_hop(self):
        m, veh, phases, track = self.stage(stop_after_landing_1=True)
        self.assertNotIn('CLIMB_2', [p for _, p in phases])
        end = self.pts[-1]
        self.assertLess(math.hypot(track[-1][2] - end[0], track[-1][3] - end[1]),
                        0.06)

    def test_yaw_offset_correction(self):
        """A 5 deg vision-vs-EKF2 heading offset pushes the drone off the path;
        setting the measured offset restores the no-offset tracking; the
        wrong sign makes it worse."""
        def worst(frame, param):
            _, _, _, tr = run(Config(), frame_yaw_deg=frame,
                              yaw_offset_param_deg=param, quiet=True)
            return max(dist_to_polyline((r[2], r[3]), self.pts)
                       for r in tr if r[1] in ('PATH', 'PATH_END'))
        base = worst(0.0, 0.0)
        self.assertGreater(worst(5.0, 0.0), base + 0.02)
        self.assertLess(worst(5.0, 5.0), base + 0.005)
        self.assertLess(worst(-5.0, -5.0), base + 0.005)
        self.assertGreater(worst(5.0, -5.0), worst(5.0, 0.0))


class TestFaults(unittest.TestCase):
    def test_offboard_lost_mid_path_stops_publishing(self):
        mission, veh, phases, track = run(Config(), drop_offboard_at=15.0,
                                          t_end=25.0, quiet=True)
        self.assertEqual(mission.phase, 'ABORTED')
        after = [c for c in run.commands if c[0] > 15.3]
        self.assertTrue(after)
        self.assertTrue(all(tw is None for _, _, tw in after))

    def test_offboard_already_active_never_flies(self):
        mission, veh, phases, track = run(Config(), offboard_at_boot=True,
                                          t_end=8.0, quiet=True)
        self.assertEqual(mission.phase, 'WAIT')
        self.assertTrue(all(tw is None for _, _, tw in run.commands))

    def test_no_odometry_no_setpoints(self):
        mission, veh, phases, track = run(Config(), odom_gap=(0.0, 100.0),
                                          t_end=8.0, quiet=True)
        self.assertEqual(mission.phase, 'WAIT')
        self.assertFalse(veh.offboard)
        self.assertTrue(all(tw is None for _, _, tw in run.commands))

    def test_odometry_loss_in_flight_holds(self):
        mission, veh, phases, track = run(Config(), odom_gap=(12.0, 15.0),
                                          t_end=20.0, quiet=True)
        self.assertEqual(mission.phase, 'HOLD')
        gap = [c for c in run.commands if 12.4 < c[0] < 15.0]
        self.assertTrue(gap)
        self.assertTrue(all(tw == (0.0, 0.0, 0.0, 0.0) for _, _, tw in gap))

    def test_short_odometry_dropout_continues(self):
        mission, veh, phases, track = run(Config(), odom_gap=(12.0, 12.8),
                                          quiet=True)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)

    def test_wait_publishes_zero_twist(self):
        m = Mission(*load_path_csv(DEFAULT_PATH), Config())
        cmd = m.update(0.0, Observation((0, 0, 0), 0.0, 0.01, False, True, 0.1,
                                        divergence=(0.02, -0.01, 0.03)))
        self.assertEqual(cmd.twist, (0.0, 0.0, 0.0, 0.0))

    def test_wait_without_px4_position_publishes_nothing(self):
        m = Mission(*load_path_csv(DEFAULT_PATH), Config())
        cmd = m.update(0.0, Observation((0, 0, 0), 0.0, 0.01, False, True, 0.1))
        self.assertIsNone(cmd.twist)

    def test_preflight_divergence_refuses_offboard(self):
        """The bench case of 2026-11-08: camera height 25.6 m off PX4's."""
        mission, veh, phases, track = run(
            Config(), divergence=lambda t: (0.0, 0.0, -25.6), t_end=12.0,
            quiet=True)
        self.assertEqual(mission.phase, 'WAIT')
        self.assertFalse(veh.offboard)
        self.assertTrue(all(tw is None for _, _, tw in run.commands))

    def test_divergence_that_clears_before_arming_flies(self):
        mission, veh, phases, track = run(
            Config(), divergence=lambda t: (0.0, 0.0, 2.0) if t < 2.0
            else (0.01, 0.0, 0.02), quiet=True)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)

    def test_inflight_horizontal_divergence_holds_zero(self):
        mission, veh, phases, track = run(
            Config(), divergence=lambda t: (max(0.0, t - 15.0) * 0.2, 0.0, 0.0),
            t_end=25.0, quiet=True)
        self.assertEqual(mission.phase, 'HOLD')
        held = [c for c in run.commands if c[1] == 'HOLD']
        self.assertTrue(held)
        self.assertTrue(all(tw == (0.0, 0.0, 0.0, 0.0) for _, _, tw in held))
        t_hold = held[0][0]
        self.assertGreater(t_hold, 17.0)    # 0.5 m at 0.2 m/s after t = 15
        self.assertLess(t_hold, 18.5)

    def test_inflight_height_divergence_on_path_holds(self):
        mission, veh, phases, track = run(
            Config(), divergence=lambda t: (0.0, 0.0, max(0.0, t - 12.0) * 0.2),
            t_end=20.0, quiet=True)
        self.assertEqual(mission.phase, 'HOLD')

    def test_height_divergence_over_the_walls_is_ignored(self):
        """On the return leg the rangefinder reads the wall tops (1.65 m),
        so PX4's height jumps; that must not stop the mission."""
        # The window CLIMB_2 .. FORWARD_END, from a clean run of the same path.
        _, _, clean, _ = run(Config(), quiet=True)
        ph0 = {p: t for t, p in clean}
        t0, t1 = ph0['CLIMB_2'] + 0.5, ph0['LAND_2'] - 0.5
        mission, veh, phases, track = run(
            Config(), divergence=lambda t: (0.0, 0.0, 1.65) if t0 < t < t1
            else (0.0, 0.0, 0.0), quiet=True)
        ph = {p: t for t, p in phases}
        self.assertLess(ph['CLIMB_2'], t0)
        self.assertGreater(ph['LAND_2'], t1)
        self.assertEqual([p for _, p in phases], EXPECTED_PHASES)

    def test_rejects_bad_config(self):
        pts, margins = load_path_csv(DEFAULT_PATH)
        for bad in (dict(touchdown_dwell_s=4.0), dict(max_height_m=3.5),
                    dict(hop_height_m=2.6), dict(liftoff_height_m=1.5),
                    dict(max_cmd_speed_mps=2.0), dict(start_x_m=2.0, start_y_m=2.0),
                    dict(slow_margin_m=0.5), dict(ki_cross_xy=-0.1)):
            with self.assertRaises(ValueError, msg=bad):
                Mission(pts, margins, Config(**bad))


if __name__ == '__main__':
    unittest.main()
