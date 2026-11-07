import math
import random
import unittest

import _paths  # noqa: F401

from dronehacks_traj.frames import (OdomFrame, body_to_world, model_odom_frame, ned_yaw_to_enu, quat_to_yaw,
                                    wrap, yaw_to_quat)


class OdomFrameTest(unittest.TestCase):
    def test_contract_worked_example(self):
        # contracts/frames.md (PR #5), "Worked example - landing pad": with the drone facing north
        # on the launch mark, the pad is 1.706 m behind (PX4 +x = -1.706) and 0.473 m right
        # (PX4 +y = +0.473). FLU, the odom side, has +y to the left: -0.473. The transform below
        # is the model's measured form: odom +x is course north (R rotates by -pi/2) and the
        # course origin's odom reading is -R * launch_mark = -R * (-2.515, 0.914).
        frame = OdomFrame.from_measured(-math.pi / 2, (-0.914, -2.515))
        x, y = frame.to_odom(-2.042, -0.792)
        self.assertAlmostEqual(x, -1.706, places=6)
        self.assertAlmostEqual(y, -0.473, places=6)
        e, n = frame.to_course(x, y)
        self.assertAlmostEqual(e, -2.042, places=6)
        self.assertAlmostEqual(n, -0.792, places=6)

    def test_measured_identity_is_explicit(self):
        frame = OdomFrame.from_measured(0.0, (0.0, 0.0))
        x, y = frame.to_odom(0.473, -1.706)
        self.assertAlmostEqual(x, 0.473, places=9)
        self.assertAlmostEqual(y, -1.706, places=9)
        self.assertAlmostEqual(frame.yaw_to_odom(0.4), 0.4, places=9)
        self.assertAlmostEqual(frame.yaw_to_course(0.4), 0.4, places=9)

    def test_round_trips(self):
        rng = random.Random(4)
        for _ in range(50):
            frame = OdomFrame(rng.uniform(-math.pi, math.pi), rng.uniform(-3, 3), rng.uniform(-3, 3))
            e, n = rng.uniform(-3, 3), rng.uniform(-3, 3)
            back = frame.to_course(*frame.to_odom(e, n))
            self.assertAlmostEqual(back[0], e, places=9)
            self.assertAlmostEqual(back[1], n, places=9)
            h = rng.uniform(-math.pi, math.pi)
            self.assertAlmostEqual(wrap(frame.yaw_to_course(frame.yaw_to_odom(h)) - h), 0.0, places=9)
            v = frame.rotate_to_course(*frame.rotate_to_odom(0.3, -0.7))
            self.assertAlmostEqual(v[0], 0.3, places=9)
            self.assertAlmostEqual(v[1], -0.7, places=9)

    def test_the_models_odom_starts_at_the_launch_pose(self):
        # vehicle_sim's model: cuVSLAM starts at the drone's pose on the launch mark, so the
        # model's odom origin is the launch mark and odom +x is the launch heading.
        frame = model_odom_frame((1.0, 2.0), (0.0, 1.0))       # facing north
        self.assertAlmostEqual(frame.yaw_to_odom(math.pi / 2), 0.0)      # facing north = odom yaw 0
        self.assertAlmostEqual(frame.yaw_to_odom(math.pi), math.pi / 2)   # facing west = odom +y (left)
        x, y = frame.to_odom(1.0, 3.0)                                    # 1 m north of the mark
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)

    def test_nonfinite_measurements_are_refused(self):
        for bad_yaw, bad_o in ((float("nan"), (0.0, 0.0)), (0.0, (float("nan"), 0.0)),
                               (float("inf"), (0.0, 0.0))):
            with self.assertRaises(ValueError):
                OdomFrame.from_measured(bad_yaw, bad_o)
        with self.assertRaises(ValueError):
            model_odom_frame((0, 0), (0, 0))


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
