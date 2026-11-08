import math
import unittest

from dronehacks_autonomy.planning.grid_search import search

INF = math.inf


def grid(nx, ny, lethal=()):
    m = [1.0] * (nx * ny)
    for x, y in lethal:
        m[y * nx + x] = INF
    return m


def octile(dx, dy):
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


class SearchTest(unittest.TestCase):
    def test_open_grid_cost_is_octile(self):
        nx, ny = 20, 12
        r = search(grid(nx, ny), nx, ny, 0.05, 0, [5 * nx + 10])
        self.assertAlmostEqual(r.costs[5 * nx + 10], octile(10, 5) * 0.05)
        path = r.path(5 * nx + 10)
        self.assertEqual((path[0], path[-1]), (0, 5 * nx + 10))

    def test_unreachable(self):
        nx, ny = 10, 10
        r = search(grid(nx, ny, [(5, y) for y in range(ny)]), nx, ny, 1.0, 0, [9])
        self.assertTrue(math.isinf(r.costs[9]))
        with self.assertRaises(ValueError):
            r.path(9)

    def test_no_diagonal_squeeze(self):
        # Lethal cells at (1, 0) and (0, 1) touch at a corner. Going from (0, 0)
        # to (1, 1) diagonally would pass between them, which is not allowed.
        nx, ny = 3, 3
        r = search(grid(nx, ny, [(1, 0), (0, 1)]), nx, ny, 1.0, 0, [1 * nx + 1])
        self.assertTrue(math.isinf(r.costs[nx + 1]))

    def test_multi_target_costs_match_single(self):
        nx, ny = 30, 30
        m = grid(nx, ny, [(15, y) for y in range(5, 30)])
        targets = [29, 29 * nx + 29, 10 * nx + 3]
        multi = search(m, nx, ny, 0.05, 0, targets)
        for t in targets:
            single = search(m, nx, ny, 0.05, 0, [t])
            self.assertAlmostEqual(multi.costs[t], single.costs[t])

    def test_cost_weights_steer_the_path(self):
        # A costly band across the direct line: the path detours when the detour is cheaper.
        nx, ny = 21, 11
        m = grid(nx, ny)
        for y in range(0, 8):
            m[y * nx + 10] = 50.0
        r = search(m, nx, ny, 1.0, 0, [20])
        crossing_rows = [c // nx for c in r.path(20) if c % nx == 10]
        self.assertTrue(crossing_rows)
        self.assertTrue(all(row >= 8 for row in crossing_rows), crossing_rows)


if __name__ == "__main__":
    unittest.main()
