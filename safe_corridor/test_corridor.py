"""py -m unittest test_corridor  (from safe_corridor/)"""
import unittest

import numpy as np

from corridor import DEFAULTS, plan
from example_course import box


class TestCentreLine(unittest.TestCase):
    def test_duct_is_flown_clear_of_the_walls_through_the_opening_centre(self):
        # a 1.2 m wide duct whose middle is y = 0.3, between a floor and a 1.4 m roof
        tris = (box(-3, -0.4, 0, 3, -0.3, 1.4) + box(-3, 0.9, 0, 3, 1.0, 1.4)
                + box(-3, -0.4, -0.02, 3, 1.0, 0) + box(-3, -0.4, 1.4, 3, 1.0, 1.42))
        tris = np.array(tris, float)
        kinds = np.array(['wall'] * len(tris))
        route = {'start': [-2.0, 0.15], 'end': [2.0, 0.45], 'name': 'duct',
                 'gates': [{'name': 'mid', 'centre': [0.0, 0.3, 0.7]}]}
        cfg = dict(DEFAULTS, open_band_m=[0.2, 1.2])
        _, nodes, path, _, _, frames = plan(route, cfg, tris, kinds)
        gap = min(np.linalg.norm(np.array(q['p']) - nodes[1]['p']) for q in path)
        self.assertLess(gap, 0.03)                                  # through the opening's centre
        self.assertGreater(min(q['margin'] for q in path), 0.08)    # clear of the walls
        f = np.array(frames[0]['pts'])                              # the frame is the duct's section
        self.assertAlmostEqual(f[:, 1].max() - f[:, 1].min(), 1.2, delta=0.06)
        self.assertAlmostEqual(nodes[1]['band'][0], 0.0, delta=0.05)  # opening = floor..roof
        self.assertAlmostEqual(nodes[1]['band'][1], 1.4, delta=0.05)


if __name__ == '__main__':
    unittest.main()
