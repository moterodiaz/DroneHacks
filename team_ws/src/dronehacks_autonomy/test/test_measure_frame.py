"""Measuring the course->odom transform from an odometry reading on the launch mark."""

import math
import tempfile
import unittest
from pathlib import Path

import _paths  # noqa: F401
import yaml

from dronehacks_autonomy.contract_map import load_contract_map
from dronehacks_autonomy.frames import frame_from_mark, model_odom_frame
from dronehacks_autonomy.measure_frame import MIN_SAMPLES, MeasureError, average, merge_into_file, parameters


class FrameFromMarkTest(unittest.TestCase):
    def test_the_mark_lands_on_the_reading_and_the_heading_on_the_yaw(self):
        for position, heading, x, y, yaw in (((-2.0, 1.0), (0.0, 1.0), 0.0, 0.0, 0.0),
                                             ((1.5, -0.5), (1.0, 0.0), 0.3, -0.2, 1.1),
                                             ((0.2, 3.0), (-0.6, 0.8), -4.0, 2.5, -2.7)):
            frame = frame_from_mark(position, heading, x, y, yaw)
            got = frame.to_odom(*position)
            self.assertAlmostEqual(got[0], x)
            self.assertAlmostEqual(got[1], y)
            self.assertAlmostEqual(math.cos(frame.yaw_to_odom(math.atan2(heading[1], heading[0]))), math.cos(yaw))
            self.assertAlmostEqual(math.sin(frame.yaw_to_odom(math.atan2(heading[1], heading[0]))), math.sin(yaw))

    def test_a_zero_reading_is_the_offline_model(self):
        a = frame_from_mark((-2.0, 1.0), (0.0, 1.0), 0.0, 0.0, 0.0)
        b = model_odom_frame((-2.0, 1.0), (0.0, 1.0))
        self.assertAlmostEqual(a.yaw, b.yaw)
        self.assertAlmostEqual(a.x, b.x)
        self.assertAlmostEqual(a.y, b.y)

    def test_zero_heading_is_refused(self):
        with self.assertRaises(ValueError):
            frame_from_mark((0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0.0)


class AverageTest(unittest.TestCase):
    def still(self, n=MIN_SAMPLES, x=1.0, y=2.0, yaw=0.5):
        return [(x, y, yaw)] * n

    def test_still_samples(self):
        x, y, yaw = average(self.still())
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, 0.5)

    def test_heading_across_the_wrap(self):
        samples = [(0.0, 0.0, math.pi - 0.001), (0.0, 0.0, -math.pi + 0.001)] * MIN_SAMPLES
        self.assertAlmostEqual(abs(average(samples)[2]), math.pi, places=3)

    def test_too_few_samples(self):
        with self.assertRaisesRegex(MeasureError, "samples"):
            average(self.still(MIN_SAMPLES - 1))

    def test_a_moving_drone_is_refused(self):
        with self.assertRaisesRegex(MeasureError, "moved"):
            average(self.still() + [(1.05, 2.0, 0.5)])
        with self.assertRaisesRegex(MeasureError, "heading"):
            average(self.still() + [(1.0, 2.0, 0.6)])


class FileTest(unittest.TestCase):
    def test_merge_keeps_other_keys(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "p.yaml"
            path.write_text(yaml.safe_dump({"mission": {"ros__parameters": {"rest_height_m": 0.07, "land": True}}}))
            cmap = load_contract_map(_paths.CONTRACT_MAP)
            merge_into_file(str(path), parameters(cmap, 0.0, 0.0, 0.0))
            got = yaml.safe_load(path.read_text())["mission"]["ros__parameters"]
        self.assertEqual(got["rest_height_m"], 0.07)
        self.assertIs(got["land"], True)
        for key in ("course_to_odom_yaw", "course_to_odom_x", "course_to_odom_y"):
            self.assertIsInstance(got[key], float)

    def test_written_values_put_the_mark_at_the_reading(self):
        cmap = load_contract_map(_paths.CONTRACT_MAP)
        v = parameters(cmap, 0.4, -0.1, 0.2)
        frame = frame_from_mark(cmap.launch.position, cmap.launch.heading, 0.4, -0.1, 0.2)
        self.assertAlmostEqual(v["course_to_odom_yaw"], frame.yaw)
        self.assertAlmostEqual(v["course_to_odom_x"], frame.x)
        self.assertAlmostEqual(v["course_to_odom_y"], frame.y)


if __name__ == "__main__":
    unittest.main()
