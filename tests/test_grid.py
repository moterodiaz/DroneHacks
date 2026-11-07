"""Run the production grid walk in the existing dependency-free CI suite."""

import importlib.util
import itertools
import math
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / "core/grid.py"
spec = importlib.util.spec_from_file_location("grid", PATH)
grid = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grid)
crossed_cells = grid.crossed_cells


class GridWalkTests(unittest.TestCase):
    def cells(self, start, end):
        # Bound consumption so an endpoint regression fails rather than hangs.
        result = list(itertools.islice(crossed_cells(start, end), 100))
        self.assertLess(len(result), 100)
        return result

    def test_reported_endpoint_and_repeated_reverse(self):
        for _ in range(3):
            self.assertEqual(self.cells((0.5, 1.5), (1.0, 1.0)), [(0, 1), (1, 1)])
            self.assertEqual(self.cells((1.0, 1.0), (0.5, 1.5)), [(1, 1), (0, 1)])

    def test_tied_crossing_checks_both_neighbours(self):
        self.assertEqual(
            set(self.cells((0.5, 0.5), (1.5, 1.5))),
            {(0, 0), (1, 0), (0, 1), (1, 1)},
        )

    def test_boundary_orientations_against_intersection_oracle(self):
        # Independent slab intersection of each candidate cell with the segment,
        # not another grid walk or a sampled approximation of the segment.
        coordinates = (-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0)
        points = list(itertools.product(coordinates, repeat=2))
        for start, end in itertools.product(points, repeat=2):
            bounds = [
                range(min(math.floor(a), math.floor(b)), max(math.floor(a), math.floor(b)) + 1)
                for a, b in zip(start, end)
            ]
            expected = set()
            for cell in itertools.product(*bounds):
                low, high = 0.0, 1.0
                for a, b, c in zip(start, end, cell):
                    delta = b - a
                    if delta == 0:
                        if not c <= a <= c + 1:
                            high = -1.0
                            break
                    else:
                        t0, t1 = sorted(((c - a) / delta, (c + 1 - a) / delta))
                        low, high = max(low, t0), min(high, t1)
                if low <= high:
                    expected.add(cell)
            with self.subTest(start=start, end=end):
                self.assertEqual(set(self.cells(start, end)), expected)
