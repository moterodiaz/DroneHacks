import unittest

from _maps import PUBLISHED, box, make_map, pad

from dronehacks_planner import MapError, load_map, parse_map


def doc(**over):
    d = {
        "origin": [0.0, 0.0],
        "slice_height": 0.8,
        "segments": [{"start": [0, 0], "end": [1, 0], "thickness": 0.1}],
        "goals": [pad((0.5, 1.0))],
    }
    d.update(over)
    return d


class LoadTest(unittest.TestCase):
    def test_published_fixture_loads(self):
        cmap = load_map(PUBLISHED)
        self.assertEqual([g.name for g in cmap.goals], ["tunnel", "window", "gate_b", "pad"])
        self.assertEqual([[g.name for g in s] for s in cmap.stages()],
                         [["tunnel"], ["window", "gate_b"], ["pad"]])
        self.assertEqual(len(cmap.boulders), 5)
        self.assertIsNotNone(cmap.launch)
        self.assertEqual(len(cmap.sha256), 64)

    def test_contract_worked_example_pad(self):
        # contracts/course.md, "Worked example - pad goal"
        cmap = parse_map(doc(goals=[{"name": "pad", "shape": "disc", "centre": [-2.042, -0.792],
                                     "params": {"radius": 0.762}}]))
        self.assertEqual(cmap.goals[0].params["radius"], 0.762)
        self.assertFalse(cmap.goals[0].passes_through)

    def test_axis_is_normalised_and_directed(self):
        g = {"name": "w", "shape": "gate", "centre": [0, 0], "params": {"axis": [0, -2], "width": 1.0}}
        goal = parse_map(doc(goals=[g])).goals[0]
        self.assertEqual(goal.axis, (0.0, -1.0))
        self.assertFalse(goal.bidirectional)


class RejectTest(unittest.TestCase):
    def assertRejected(self, d, fragment):
        with self.assertRaises(MapError) as cm:
            parse_map(d)
        self.assertIn(fragment, str(cm.exception))

    def test_segment_needs_thickness(self):
        self.assertRejected(doc(segments=[{"start": [0, 0], "end": [1, 0]}]), "thickness")
        cmap = parse_map(doc(segments=[{"start": [0, 0], "end": [1, 0]}], default_thickness=0.1))
        self.assertEqual(cmap.segments[0].thickness, 0.1)

    def test_nonzero_origin(self):
        self.assertRejected(doc(origin=[1.0, 0.0]), "origin")

    def test_unknown_shape(self):
        self.assertRejected(doc(goals=[{"name": "x", "shape": "hoop", "centre": [0, 0]}]), "shape")

    def test_pass_shape_needs_axis(self):
        self.assertRejected(doc(goals=[{"name": "x", "shape": "rect", "centre": [0, 0],
                                        "params": {"width": 1}}]), "axis")

    def test_duplicate_goal_names(self):
        self.assertRejected(doc(goals=[pad((0, 1)), pad((0, 2))]), "twice")

    def test_group_must_be_contiguous(self):
        a = dict(pad((0, 1), name="a"), group="g")
        c = dict(pad((0, 3), name="c"), group="g")
        self.assertRejected(doc(goals=[a, pad((0, 2), name="b"), c]), "listed together")

    def test_window_band_order(self):
        g = {"name": "w", "shape": "rect", "centre": [0, 0],
             "params": {"axis": [0, 1], "width": 1, "bottom": 1.2, "top": 0.3}}
        self.assertRejected(doc(goals=[g]), "bottom")

    def test_bidirectional_must_be_bool(self):
        g = {"name": "w", "shape": "gate", "centre": [0, 0],
             "params": {"axis": [0, 1], "width": 1, "bidirectional": "yes"}}
        self.assertRejected(doc(goals=[g]), "bidirectional")

    def test_not_a_number(self):
        self.assertRejected(doc(slice_height="high"), "slice_height")

    def test_nothing_to_plan_around(self):
        self.assertRejected(doc(segments=[]), "nothing to plan around")


class MakeMapTest(unittest.TestCase):
    def test_helper_builds_valid_map(self):
        cmap = make_map(box(4, 4), [pad((1, 1))], launch=(-1, -1))
        self.assertEqual(len(cmap.segments), 4)
        self.assertEqual(cmap.launch.position, (-1.0, -1.0))


if __name__ == "__main__":
    unittest.main()
