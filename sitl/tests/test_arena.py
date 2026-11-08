"""Arenas: course-edits/1 on the STEP model, the referee's arena gates, arena choice, and the plan observer.

Run from the ``sitl`` directory::

    py -B -m unittest tests.test_arena -v

The course tests load ``dronehacks-course-v3.step`` with the taped arena's edits through the shared
cache (``%LOCALAPPDATA%\\dronesim\\cache``). A new edits file builds its own heightmap and distance
field cache once, which takes minutes (see the README).
"""
import math
import os
import sys
import types
import unittest

import numpy as np

from dronesim.autonomy import observers
from dronesim.autonomy.catalog import VersionError, load_catalog, resolve_arena
from dronesim.world.arena import (ArenaError, apply_edits, apply_rule_edits, edits_path, load_edits,
                                  wall_triangles)
from dronesim.world.step_mesh import Solid, watertight_report

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
STEP = os.path.join(REPO, 'dronehacks-course-v3.step')
RULES = os.path.join(SITL, 'course', 'course_v3_rules.json')
TAPED = edits_path('taped-2026-11-08')
TEAM_COPY = os.path.join(REPO, 'nav_traj', 'maps', 'source', 'arena_2026-11-08_edits.json')
CATALOG = load_catalog(os.path.join(SITL, 'versions.yaml'))


def box(name, lo, hi):
    tris = wall_triangles((lo[0], (lo[1] + hi[1]) / 2), (hi[0], (lo[1] + hi[1]) / 2), hi[1] - lo[1], (lo[2], hi[2]))
    return Solid.from_triangles(name, tris)


class EditsFileTests(unittest.TestCase):
    @unittest.skipUnless(os.path.isfile(TEAM_COPY), 'nav_traj copy not in this checkout')
    def test_the_sitl_copy_is_the_team_maps_file_byte_for_byte(self):
        with open(TAPED, 'rb') as a, open(TEAM_COPY, 'rb') as b:
            self.assertEqual(a.read(), b.read(), 'sitl/course/arena_2026-11-08_edits.json must be a byte copy of '
                                                 'nav_traj/maps/source/arena_2026-11-08_edits.json: copy it again')

    def test_the_file_loads(self):
        edits, sha = load_edits(TAPED)
        self.assertEqual(edits['format'], 'course-edits/1')
        self.assertEqual(len(sha), 64)


class WallMeshTests(unittest.TestCase):
    P0, P1, T, Z = (-0.74676, -3.02971), (-0.19512, -1.51408), 0.1016, (0.0, 1.7272)
    WINDOW = {'s': [0.34925, 1.26365], 'z': [0.9144, 1.5494]}

    def length(self):
        return math.dist(self.P0, self.P1)

    def test_a_window_wall_is_closed_and_has_the_right_volume(self):
        tris = wall_triangles(self.P0, self.P1, self.T, self.Z, [self.WINDOW])
        self.assertTrue(watertight_report(tris).watertight)
        self.assertEqual(len(tris), 32)              # as the STEP meshes C2a
        s = Solid.from_triangles('w', tris)
        expected = self.length() * self.T * 1.7272 - 0.9144 * self.T * (1.5494 - 0.9144)
        self.assertAlmostEqual(s.volume(), expected, places=6)

    def test_only_the_openings_corners_lie_between_foot_and_top(self):
        v = wall_triangles(self.P0, self.P1, self.T, self.Z, [self.WINDOW]).reshape(-1, 3)
        inner = v[(v[:, 2] > 0.05) & (v[:, 2] < 1.7272 - 0.05)]
        u = (np.array(self.P1) - np.array(self.P0)) / self.length()
        s = (inner[:, :2] - np.array(self.P0)) @ u
        self.assertAlmostEqual(float(s.min()), 0.34925, places=6)
        self.assertAlmostEqual(float(s.max()), 1.26365, places=6)

    def test_a_plain_wall_and_the_grid_fallback_are_closed(self):
        plain = wall_triangles((0, 0), (2, 0), 0.1, (0, 1.7))
        self.assertTrue(watertight_report(plain).watertight)
        self.assertAlmostEqual(Solid.from_triangles('p', plain).volume(), 2 * 0.1 * 1.7, places=9)
        two = wall_triangles((0, 0), (3, 0), 0.1, (0, 1.7), [{'s': [0.5, 1.0], 'z': [0.5, 1.0]},
                                                             {'s': [2.0, 2.5], 'z': [0.0, 1.2]}])   # + a door
        self.assertTrue(watertight_report(two).watertight)
        self.assertAlmostEqual(Solid.from_triangles('g', two).volume(),
                               0.1 * (3 * 1.7 - 0.5 * 0.5 - 0.5 * 1.2), places=9)

    def test_normals_point_out(self):
        s = Solid.from_triangles('w', wall_triangles((0, 0), (2, 0), 0.1, (0, 1.7), [{'s': [0.5, 1.5], 'z': [0.5, 1.2]}]))
        self.assertGreater(s.volume(), 0)


class ApplyEditsTests(unittest.TestCase):
    def solids(self):
        return [box('Rock', (0.9, -0.1, 0), (1.1, 0.1, 0.5)), box('Old wall', (2, 0, 0), (3, 0.1, 1.7)),
                box('Window wall', (0, 2, 0), (2, 2.1, 1.7))]

    def test_remove_move_replace_add_in_order(self):
        edits = {'remove': [{'name': 'Old wall'}],
                 'move': [{'names': ['Rock'], 'from': [1.0, 0.0], 'to': [1.5, -0.5]}],
                 'replace': [{'name': 'Window wall', 'kind': 'wall', 'p0': [0, 2.05], 'p1': [2, 2.05],
                              'thickness': 0.1, 'z': [0, 1.7], 'openings': [{'s': [0.5, 1.5], 'z': [0.8, 1.4]}]}],
                 'add': [{'name': 'New wall', 'kind': 'wall', 'p0': [0, -1], 'p1': [1, -1], 'thickness': 0.1,
                          'z': [0, 1.7]}]}
        out, kinds, notes = apply_edits(self.solids(), edits)
        names = [s.name for s in out]
        self.assertEqual(names, ['Rock', 'Window wall', 'New wall'])
        rock = out[0]
        self.assertAlmostEqual((rock.bbox_min[0] + rock.bbox_max[0]) / 2, 1.5)
        self.assertAlmostEqual((rock.bbox_min[1] + rock.bbox_max[1]) / 2, -0.5)
        self.assertEqual(kinds, {'Window wall': 'wall', 'New wall': 'wall'})
        self.assertEqual(len(notes), 4)

    def test_a_move_from_the_wrong_place_is_refused(self):
        with self.assertRaises(ArenaError):
            apply_edits(self.solids(), {'move': [{'names': ['Rock'], 'from': [1.05, 0.0], 'to': [0, 0]}]})

    def test_unknown_or_duplicate_names_are_refused(self):
        with self.assertRaises(ArenaError):
            apply_edits(self.solids(), {'remove': [{'name': 'Nope'}]})
        with self.assertRaises(ArenaError):
            apply_edits(self.solids(), {'add': [{'name': 'Rock', 'p0': [0, 0], 'p1': [1, 0], 'thickness': 0.1,
                                                 'z': [0, 1]}]})

    def test_rule_edits_replace_a_gate_and_ignore_planner_keys(self):
        rules = {'checkpoints': [{'id': 2, 'name': 'CP2 Gate A', 'gate': {'segment': [[0, 2], [0, 3]]}},
                                 {'id': 5, 'name': 'CP5', 'land': True}]}
        out = apply_rule_edits(rules, {'rules': {'checkpoints': {'CP2 Gate A': {
            'gate': {'segment': [[0, 0.5], [0, 2.4]]}, 'planner_goal': False}}}})
        self.assertEqual(out['checkpoints'][0]['gate'], {'segment': [[0, 0.5], [0, 2.4]]})
        self.assertNotIn('planner_goal', out['checkpoints'][0])
        self.assertEqual(rules['checkpoints'][0]['gate'], {'segment': [[0, 2], [0, 3]]})   # not mutated
        with self.assertRaises(ArenaError):
            apply_rule_edits(rules, {'rules': {'checkpoints': {'CP9': {'gate': {}}}}})


class ArenaChoiceTests(unittest.TestCase):
    def test_the_flight_lines_planned_on_the_taped_course_fly_in_it(self):
        for line in ('window_max', 'window_0p5mps'):
            arena, why = resolve_arena(CATALOG['traj'], {'line': line})
            self.assertEqual(arena, 'taped-2026-11-08', line)
            self.assertIn(line, why)

    def test_the_shipped_line_and_other_versions_stay_on_the_step(self):
        self.assertEqual(resolve_arena(CATALOG['traj'], {})[0], 'step')
        self.assertEqual(resolve_arena(CATALOG['traj'], {'line': 'shipped'})[0], 'step')
        for vid in CATALOG:
            if vid != 'traj':
                self.assertEqual(resolve_arena(CATALOG[vid], {})[0], 'step', vid)

    def test_an_explicit_arena_wins(self):
        self.assertEqual(resolve_arena(CATALOG['traj'], {'line': 'window_max'}, 'step'),
                         ('step', 'chosen in the scenario'))
        self.assertEqual(resolve_arena(CATALOG['traj'], {}, 'taped-2026-11-08')[0], 'taped-2026-11-08')
        self.assertEqual(resolve_arena(CATALOG['traj'], {'line': 'window_max'}, 'auto')[0], 'taped-2026-11-08')
        with self.assertRaises(ArenaError):
            resolve_arena(CATALOG['traj'], {}, 'moon')

    def test_two_options_naming_different_arenas_are_an_error(self):
        v = {'id': 'x', 'options': {
            'a': {'default': 'one', 'choices': {'one': {'arena': 'step'}}},
            'b': {'default': 'two', 'choices': {'two': {'arena': 'taped-2026-11-08'}}}}}
        with self.assertRaises(VersionError):
            resolve_arena(v, {})


@unittest.skipUnless(os.path.isfile(STEP), 'course STEP not found')
class TapedCourseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dronesim.px4.params import load_profile
        from dronesim.scoring.referee import Referee
        from dronesim.vehicle.physics import AirframeConfig
        from dronesim.world.course import Course
        cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
        cls.course = Course(STEP, cache, TAPED)
        cls.step = Course(STEP, cache)
        af = AirframeConfig.from_params(load_profile(REPO, 'drone1-fleet'))
        cls.ref = Referee(cls.course, RULES, af)
        cls.step_ref = Referee(cls.step, RULES, af)

    def centre(self, name):
        return self.course.centre_xy(name)

    def test_removed_solids_are_gone(self):
        for name in ('Part67 / Boss-Extrude1[1]', 'Part67 / Boss-Extrude1[2]', 'Boulder 2'):
            self.assertNotIn(name, self.course.by_name)
            self.assertIn(name, self.step.by_name)

    def test_moved_solids_are_where_the_team_taped_them(self):
        for name, at in (('Boulder 1', (2.8194, -1.07)), ('Boulder 3', (0.79248, -2.8194)),
                         ('Boulder 5', (-1.34112, -2.7432)), ('Landing pad 5 ft', (-2.1336, -1.8288)),
                         ('Landing pad cross', (-2.1336, -1.8288)), ('Approach ring (stage 4 line)', (-2.1336, -1.8288))):
            c = self.centre(name)
            self.assertLess(math.dist(c, at), 1e-4, name)
        self.assertLess(math.dist(self.centre('Boulder 4'), (-2.56032, -2.49936)), 1e-4)   # unmoved

    def test_the_new_walls_are_walls_where_the_map_puts_them(self):
        self.assertEqual(self.course.kind('North wall (taped)'), 'wall')
        self.assertEqual(self.course.kind('Diagonal wall (taped)'), 'wall')
        north = self.course.solid('North wall (taped)')
        self.assertAlmostEqual(float(north.bbox_min[1]), 2.4384, places=6)     # its south face
        self.assertAlmostEqual(float(north.bbox_max[2]), 1.7272, places=6)
        self.assertEqual(self.course.kind('C2a chicane wall (window)'), 'wall')
        # the diagonal runs (-1.798, 0.508) -> (-0.813, 1.648): its centreline's middle is solid
        self.assertLess(self.course.clearance(-1.3055, 1.078, 1.0), 0.06)

    def test_the_window_is_open_at_the_taped_height_and_closed_below(self):
        wx, wy = -0.47094, -2.27189
        self.assertGreater(self.course.clearance(wx, wy, 1.23), 0.2)
        self.assertLess(self.course.clearance(wx, wy, 0.6), 0.1)
        self.assertGreater(self.step.clearance(wx, wy, 0.6), 0.2)   # the STEP's window: 12-48 in

    def test_referee_gates_follow_the_arena(self):
        gates = {(g['cp'], g['route']): g for g in self.ref.gates_for_display()}
        cp2 = gates[(2, 'main')]
        self.assertEqual((tuple(cp2['a']), tuple(cp2['b'])), ((0.3048, 0.508), (0.3048, 2.4384)))
        win = gates[(4, 'window')]
        self.assertAlmostEqual(win['z'][0], 0.9144, places=6)
        self.assertAlmostEqual(win['z'][1], 1.5494, places=6)
        step_win = {(g['cp'], g['route']): g for g in self.step_ref.gates_for_display()}[(4, 'window')]
        for k in ('a', 'b'):            # the opening's place along the wall is the STEP's
            self.assertLess(math.dist(win[k], step_win[k]), 1e-4)
        self.assertLess(math.dist(self.ref.pad_centre, (-2.1336, -1.8288)), 1e-6)
        step_gates = {(g['cp'], g['route']): g for g in self.step_ref.gates_for_display()}
        for key in ((3, 'main'), (4, 'gate_b')):        # the tube and Gate B are the STEP's
            self.assertEqual(gates[key], step_gates[key], key)

    def test_the_cache_and_description_name_the_arena(self):
        self.assertNotEqual(self.course._cache_path(), self.step._cache_path())
        self.assertIn(self.course.edits_sha256[:16], self.course._cache_path())
        d = self.course.describe()
        self.assertEqual(d['edits'], 'arena_2026-11-08_edits.json')
        self.assertIn('taped', d['arena'])
        self.assertIsNone(self.step.describe()['edits'])

    def test_the_step_is_unchanged_without_edits(self):
        gates = {(g['cp'], g['route']): g for g in self.step_ref.gates_for_display()}
        self.assertEqual(tuple(gates[(2, 'main')]['a']), (0.3048, 2.1209))
        self.assertIn('Part67 / Boss-Extrude1[2]', self.step.by_name)


class PlanObserverTests(unittest.TestCase):
    """The observer passes prepare() through unchanged and hands a copy of the plan to the sink."""

    def tearDown(self):
        observers.set_sink(None)

    def _fake_prepared(self):
        xy = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        traj = types.SimpleNamespace(xy=xy, t=np.array([0.0, 1.0, 2.0]), length=2.0)
        wp = types.SimpleNamespace(position=(0.0, 0.0), kind='start', goal=None)
        route = types.SimpleNamespace(goals=['tunnel', 'pad'], choices={}, waypoints=[wp])
        plan = types.SimpleNamespace(trajectory=traj, profile=types.SimpleNamespace(v=np.array([0, 1, 0.0])),
                                     route=route, duration=2.0)
        return types.SimpleNamespace(plan=plan, cmap=types.SimpleNamespace(slice_height=1.23), v_max_mps=0.5,
                                     speed_note=None, alternatives=[('cp4', 'window')])

    def test_with_a_stand_in_module(self):
        prepared = self._fake_prepared()
        calls = []
        pkg = types.ModuleType('dronehacks_traj')
        mod = types.ModuleType('dronehacks_traj.mission_node')

        def prepare(map_source, *a, **kw):
            calls.append((map_source, kw))
            return prepared
        mod.prepare = prepare
        pkg.mission_node = mod
        saved = {k: sys.modules.get(k) for k in ('dronehacks_traj', 'dronehacks_traj.mission_node')}
        sys.modules['dronehacks_traj'], sys.modules['dronehacks_traj.mission_node'] = pkg, mod
        try:
            got = []
            observers.set_sink(got.append)
            observers.traj_plan_observer()
            observers.traj_plan_observer()                 # idempotent: wrapped once
            self.assertIs(mod.prepare('map.yaml', follower='f'), prepared)
            self.assertEqual(calls, [('map.yaml', {'follower': 'f'})])
            self.assertEqual(len(got), 1)
            rec = got[0]
            self.assertEqual(rec['xy'], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
            self.assertEqual(rec['goals'], ['tunnel', 'pad'])
            self.assertEqual(rec['map_file'], 'map.yaml')
            self.assertEqual(rec['slice_height_m'], 1.23)
        finally:
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v

    def test_display_points_are_thinned_and_keep_the_end(self):
        plan = {'xy': [[i * 0.01, 0.0] for i in range(1001)]}
        pts = observers.display_points(plan, max_points=100)
        self.assertLessEqual(len(pts), 102)
        self.assertEqual(pts[0], [0.0, 0.0])
        self.assertEqual(pts[-1], [10.0, 0.0])

    def test_with_the_real_planner_on_the_step_map(self):
        paths = [os.path.join(REPO, 'planner'), os.path.join(REPO, 'nav_traj')]
        mp = os.path.join(REPO, 'nav_traj', 'maps', 'course_v3_step.yaml')
        if not all(os.path.isdir(p) for p in paths) or not os.path.isfile(mp):
            self.skipTest('planner / nav_traj not in this checkout')
        pkgs = ('dronehacks_planner', 'dronehacks_traj')
        before = {k for k in sys.modules if k.split('.')[0] in pkgs}
        added = [p for p in paths if p not in sys.path]
        sys.path[:0] = added
        try:
            from dronehacks_planner.course_map import load_map
            from dronehacks_traj.factory import prepare
            from dronehacks_traj.frames import model_odom_frame
            from dronehacks_traj.mission import MissionConfig
            cmap = load_map(mp)
            prepared = prepare(cmap, MissionConfig(rest_height_m=0.07),
                               frame=model_odom_frame(cmap.launch.position, cmap.launch.heading))
        except ImportError as exc:
            self.skipTest(f'cannot import the planner: {exc}')
        finally:
            for p in added:
                sys.path.remove(p)
            # later tests in this process build workspaces that import these from an archived ref
            for k in [k for k in sys.modules if k.split('.')[0] in pkgs and k not in before]:
                del sys.modules[k]
        rec = observers.traj_plan_summary(prepared, mp)
        self.assertEqual(rec['goals'], list(prepared.plan.route.goals))
        self.assertEqual(len(rec['xy']), len(prepared.plan.trajectory.xy))
        self.assertEqual(len(rec['t']), len(rec['xy']))
        self.assertAlmostEqual(rec['duration_s'], prepared.plan.duration, places=2)
        self.assertLess(math.dist(rec['xy'][0], cmap.launch.position), 0.05)   # course frame, from the mark


if __name__ == '__main__':
    unittest.main()
