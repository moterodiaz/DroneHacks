"""The 2026-11-07 course layout: solid kinds, the net box without a roof, and the rules file.

Run from the ``sitl`` directory::

    py -B -m unittest tests.test_course_layout -v

The course tests load ``dronehacks-course-v3.step`` from the repository root through the
shared cache (``%LOCALAPPDATA%\\dronesim\\cache``); the first run builds it (~40 s).
"""
import json
import math
import os
import unittest

import numpy as np

from dronesim.px4.params import load_profile
from dronesim.world.course import WALL_TOP_M, classify, classify_solid
from dronesim.world.step_mesh import Solid

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
STEP = os.path.join(REPO, 'dronehacks-course-v3.step')
RULES = os.path.join(SITL, 'course', 'course_v3_rules.json')


def box(name, lo, hi):
    """A box solid (12 outward triangles)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = np.array([[(lo, hi)[i & 1][0], (lo, hi)[(i >> 1) & 1][1], (lo, hi)[(i >> 2) & 1][2]] for i in range(8)])
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    tris = [t for a, b, cc, d in quads for t in ((c[a], c[b], c[cc]), (c[a], c[cc], c[d]))]
    return Solid.from_triangles(name, np.array(tris))


class GeometryWallFallbackTests(unittest.TestCase):
    """A solid whose name says nothing is a wall if it stands on the floor and reaches the wall top."""

    def test_unnamed_floor_to_wall_top_solid_is_a_wall(self):
        self.assertEqual(classify('Part12'), 'other')
        self.assertEqual(classify_solid(box('Part12', (0, 0, 0), (1.0, 0.1, WALL_TOP_M))), 'wall')
        # a diagonal footprint's bounding box is not thin
        self.assertEqual(classify_solid(box('Part12 / Boss-Extrude1[2]', (0, 0, 0), (1.0, 1.0, 1.7272))), 'wall')

    def test_what_is_not_a_wall(self):
        cases = {
            'raised off the floor': box('Part1', (0, 0, 0.30), (1.0, 0.1, WALL_TOP_M)),
            'too short': box('Part2', (0, 0, 0), (1.0, 0.1, 1.0)),
            'a thin sheet (a net panel)': box('Part3', (0, 0, 0), (6.0, 0.00127, 3.048)),
        }
        for why, s in cases.items():
            self.assertEqual(classify_solid(s), 'other', why)

    def test_the_name_wins(self):
        self.assertEqual(classify_solid(box('Net side north', (0, 0, 0), (6.0, 0.1, 3.048))), 'net')
        self.assertEqual(classify_solid(box('Boulder 9', (0, 0, 0), (0.5, 0.5, 1.8))), 'boulder')
        self.assertEqual(classify_solid(box('Tube oval', (0, 0, 0.0254), (1.5, 1.5, 1.85))), 'tunnel')


@unittest.skipUnless(os.path.isfile(STEP), 'course STEP not found')
class CourseLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dronesim.world.course import Course
        cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
        cls.course = Course(STEP, cache)

    def test_the_new_walls_are_walls(self):
        walls = sorted(n for n, k in self.course.kind_by_name.items() if k == 'wall')
        self.assertEqual(walls, sorted([
            'A1 approach wall', 'Barrier 1', 'Barrier 2', 'Barrier 3 (cut to 55 in)', 'C1 chicane wall',
            'C2a chicane wall (window)', 'C2b chicane wall', 'Part67 / Boss-Extrude1[1]',
            'Part67 / Boss-Extrude1[2]']))
        self.assertNotIn('other', self.course.kinds)
        # the index order the contact model uses carries the same kinds
        for n, k in zip(self.course.index.solid_names, self.course.solid_kind):
            self.assertEqual(k, self.course.kind(n), n)

    def test_the_net_box_is_the_height_limit_without_a_roof(self):
        self.assertNotIn('Net roof', self.course.by_name)
        nets = [s for s, k in zip(self.course.solids, self.course.kinds) if k == 'net']
        self.assertEqual(len(nets), 4)
        self.assertAlmostEqual(float(self.course.net_hi[2]), 3.048, delta=0.002)
        self.assertTrue(self.course.inside_net((0.0, 0.0, 3.0), margin=0.05))
        self.assertFalse(self.course.inside_net((0.0, 0.0, 3.2), margin=0.05))   # above the panels
        self.assertFalse(self.course.inside_net((0.0, 3.2, 1.0), margin=0.05))   # through the north net
        self.assertEqual(self.course.describe()['solids'], 31)


class RulesFileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(RULES, encoding='utf-8') as fh:
            cls.rules = json.load(fh)

    def test_checkpoint_1_is_removed_and_2_to_5_are_kept(self):
        cps = self.rules['checkpoints']
        self.assertEqual([(c['id'], c['name'], c['points']) for c in cps], [
            (2, 'CP2 Gate A', 7), (3, 'CP3 tunnel south end', 10), (4, 'CP4 landing area', 7),
            (5, 'CP5 land and disarm on the pad', 7)])
        text = json.dumps(cps)
        self.assertNotIn('CP1', text)
        self.assertNotIn('maze divider', text)          # the notes record the layout change

    def test_gate_a_is_an_explicit_segment(self):
        gate = self.rules['checkpoints'][0]['gate']
        self.assertEqual(list(gate), ['segment'])
        (a, b) = gate['segment']
        self.assertEqual(a[0], b[0])               # a north-south line
        self.assertLess(a[1], b[1])

    @unittest.skipUnless(os.path.isfile(STEP), 'course STEP not found')
    def test_every_name_resolves_and_every_gate_builds(self):
        from dronesim.scoring.referee import Referee
        from dronesim.vehicle.physics import AirframeConfig
        from dronesim.world.course import Course
        cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
        course = Course(STEP, cache)
        names = set(self.rules['extra_margin_m']) | {self.rules['start']['marking'], self.rules['finish']['marking'],
                                                     self.rules['return']['marking']}
        for cp in self.rules['checkpoints']:
            gates = [cp['gate']] if 'gate' in cp else [r['gate'] for r in cp.get('routes', {}).values()]
            for g in gates:
                names.update(g.get('between', []))
                for key in ('opening', 'tube_end'):
                    if key in g:
                        names.add(g[key])
        for n in names:
            self.assertIn(n, course.by_name, n)
        ref = Referee(course, RULES, AirframeConfig.from_params(load_profile(REPO, 'drone1-fleet')))
        gates = {(g['cp'], g['route']): g for g in ref.gates_for_display()}
        self.assertEqual(sorted(gates), [(2, 'main'), (3, 'main'), (4, 'gate_b'), (4, 'window')])
        a, b = gates[(2, 'main')]['a'], gates[(2, 'main')]['b']
        self.assertEqual([list(a), list(b)], self.rules['checkpoints'][0]['gate']['segment'])
        # Gate A reaches the north net's inner face
        self.assertAlmostEqual(b[1], float(course.solid('Net side north').bbox_min[1]), delta=0.002)
        gb = gates[(4, 'gate_b')]
        self.assertGreater(math.dist(gb['a'], gb['b']), 0.9)   # the C2b / Barrier 2 gap still resolves


if __name__ == '__main__':
    unittest.main()
