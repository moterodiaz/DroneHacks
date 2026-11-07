import math
import unittest

import numpy as np
from _maps import make_map, pad

from dronehacks_planner.clearance import CostModel, ObstacleField, segment_cost

MODEL = CostModel(inscribed_radius_m=0.32, inflation_radius_m=1.0, cost_scaling_per_m=3.0, cost_weight=4.0)


def field(segments=(), boulders=(), boulder_radii=None, thickness=0.2):
    segs = list(segments) or [((50, 50), (51, 50))]  # a map needs something; put it far away
    cmap = make_map(segs, [pad((0, 0))], boulders=boulders, thickness=thickness)
    radii = boulder_radii if boulder_radii is not None else [0.32] * len(boulders)
    return ObstacleField(cmap, MODEL, radii)


class DistanceTest(unittest.TestCase):
    def test_segment_distances(self):
        f = field([((0, 0), (1, 0))], thickness=0.2)
        surface, margin = f.query(np.array([[0.5, 1.0], [2.0, 0.0], [0.5, 0.0], [-0.6, 0.8]]))
        np.testing.assert_allclose(surface, [0.9, 0.9, -0.1, 0.9], atol=1e-12)
        np.testing.assert_allclose(margin, surface - 0.32, atol=1e-12)

    def test_boulder_has_its_own_radius(self):
        f = field(boulders=[{"centre": [0, 0], "radius": 0.5}], boulder_radii=[0.2])
        surface, margin = f.query(np.array([[1.0, 0.0]]))
        self.assertAlmostEqual(surface[0], 0.5)
        self.assertAlmostEqual(margin[0], 0.3)

    def test_zero_length_segment_is_a_point(self):
        f = field([((0, 0), (0, 0))], thickness=0.0)
        surface, _ = f.query(np.array([[3.0, 4.0]]))
        self.assertAlmostEqual(surface[0], 5.0)


class CostTest(unittest.TestCase):
    def test_multiplier(self):
        # The cutoff is inflation - inscribed = 0.68 m of margin.
        m = MODEL.multiplier(np.array([-0.01, 0.0, 0.1, 0.67, 0.69, 5.0]))
        self.assertTrue(math.isinf(m[0]))
        self.assertAlmostEqual(m[1], 5.0)
        self.assertAlmostEqual(m[2], 1 + 4 * math.exp(-0.3))
        self.assertGreater(m[3], 1.0)       # inside the inflation radius
        self.assertEqual(m[4], 1.0)         # beyond it
        self.assertEqual(m[5], 1.0)

    def test_open_space_costs_its_length(self):
        f = field()
        cost, _ = segment_cost(f, (0.0, 0.0), (3.0, 4.0), 0.02)
        self.assertAlmostEqual(cost, 5.0)

    def test_splitting_a_segment_keeps_its_cost(self):
        f = field([((0, 0.6), (4, 0.6))])
        whole, _ = segment_cost(f, (0.0, 0.0), (4.0, 0.0), 0.02)
        a, _ = segment_cost(f, (0.0, 0.0), (1.37, 0.0), 0.02)
        b, _ = segment_cost(f, (1.37, 0.0), (4.0, 0.0), 0.02)
        self.assertAlmostEqual(whole, a + b, delta=whole * 1e-3)

    def test_lethal_segment_costs_inf(self):
        f = field([((0, 0.3), (4, 0.3))], thickness=0.0)
        cost, margin = segment_cost(f, (0.0, 0.0), (4.0, 0.0), 0.02)
        self.assertTrue(math.isinf(cost))
        self.assertLess(margin, 0)


if __name__ == "__main__":
    unittest.main()
