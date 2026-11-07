import math
import random
import unittest

import _paths  # noqa: F401

from dronehacks_traj.frames import (OdomFrame, body_to_world, ned_yaw_to_enu, quat_to_yaw, wrap,
                                    yaw_to_quat)


class OdomFrameTest(unittest.TestCase):
    def test_contract_worked_example(self):
        # contracts/frames.md (PR #5), "Worked example - landing pad": with the drone facing north
        # on the launch mark, the pad is 1.706 m behind (PX4 +x = -1.706) and 0.473 m right
        # (PX4 +y = +0.473). FLU, the odom side, has +y to the left: -0.473.
        frame = OdomFrame.from_launch((-2.515, 0.914), (0.0, 1.0))
        x, y = frame.to_odom(-2.042, -0.792)
        self.assertAlmostEqual(x, -1.706, places=6)
        self.assertAlmostEqual(y, -0.473, places=6)

    def test_round_trips(self):
        rng = random.Random(4)
        for _ in range(50):
            frame = OdomFrame(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi))
            e, n = rng.uniform(-3, 3), rng.uniform(-3, 3)
            back = frame.to_course(*frame.to_odom(e, n))
            self.assertAlmostEqual(back[0], e, places=9)
            self.assertAlmostEqual(back[1], n, places=9)
            h = rng.uniform(-math.pi, math.pi)
            self.assertAlmostEqual(wrap(frame.yaw_to_course(frame.yaw_to_odom(h)) - h), 0.0, places=9)
            v = frame.rotate_to_course(*frame.rotate_to_odom(0.3, -0.7))
            self.assertAlmostEqual(v[0], 0.3, places=9)
            self.assertAlmostEqual(v[1], -0.7, places=9)

    def test_launch_heading_is_odom_x(self):
        frame = OdomFrame.from_launch((1.0, 2.0), (0.0, 1.0))
        self.assertAlmostEqual(frame.yaw_to_odom(math.pi / 2), 0.0)      # facing north = odom yaw 0
        self.assertAlmostEqual(frame.yaw_to_odom(math.pi), math.pi / 2)  # facing west = odom +y (left)
        x, y = frame.to_odom(1.0, 3.0)                                    # 1 m north of the mark
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)

    def test_zero_heading_rejected(self):
        with self.assertRaises(ValueError):
            OdomFrame.from_launch((0, 0), (0, 0))


class AngleTest(unittest.TestCase):
    def test_quaternion_yaw(self):
        for yaw in (-3.0, -1.2, 0.0, 0.7, 3.1):
            self.assertAlmostEqual(quat_to_yaw(*yaw_to_quat(yaw)), yaw, places=9)

    def test_body_velocity_to_world(self):
        vx, vy = body_to_world(1.0, 0.0, math.pi / 2)   # moving forward while facing +y
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 1.0)

    def test_ned_heading_to_enu(self):
        self.assertAlmostEqual(ned_yaw_to_enu(0.0), math.pi / 2)            # north
        self.assertAlmostEqual(ned_yaw_to_enu(math.pi / 2), 0.0)            # east
        self.assertAlmostEqual(ned_yaw_to_enu(ned_yaw_to_enu(0.4)), 0.4)    # its own inverse


if __name__ == "__main__":
    unittest.main()
