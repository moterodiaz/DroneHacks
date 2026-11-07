"""PR #39 review fixes: the range-loss fallback, external forces in the IMU, the lidar after an
outage, landing-gear touches, the window bonus and the optional return's scoring (rule 7.10)."""
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
sys.path.insert(0, SITL)

from dronesim.px4.params import load_profile  # noqa: E402
from dronesim.vehicle.physics import AirframeConfig  # noqa: E402


class ReviewFixTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = load_profile(REPO, 'drone1-fleet')
        cls.af = AirframeConfig.from_params(cls.params)

    def test_without_range_the_barometer_becomes_the_height_reference(self):
        from dronesim.px4.estimator import Ekf2Model
        ekf = Ekf2Model(self.params, random.Random(1), 0.0)
        t = 0.0
        for _ in range(40):                                  # 8 s of good range at 0.8 m
            t += 0.2
            ekf.fuse_range(t, 0.8, True, 1.0, False)
            ekf.fuse_baro(t, -0.8)
        self.assertTrue(ekf.rng_is_ref)
        ekf.fuse_range(t + 0.2, 0.0, False, 1.0, False)      # out of range: not yet timed out
        self.assertTrue(ekf.rng_is_ref)
        ekf.fuse_range(t + 0.8, 0.0, False, 1.0, False)      # still none after the timeout
        self.assertFalse(ekf.rng_is_ref)
        ekf.fuse_baro(t + 0.8, -0.8)
        self.assertTrue(ekf.baro_is_ref)

    def test_an_external_force_is_felt_by_the_imu(self):
        from dronesim.mathutil import q_from_euler
        from dronesim.vehicle.physics import QuadPhysics
        phys = QuadPhysics(self.af, (0.0, 0.0, -2.0), q_from_euler(0.0, 0.0, 0.0))
        phys.ext_force_world = (3.0, 0.0, 0.0)               # a 3 N push north, motors off
        phys.step(0.002, [0.0] * 4, False)
        sf = phys.state.specific_force_body
        self.assertAlmostEqual(sf[0], 3.0 / self.af.mass_kg, delta=0.02)
        self.assertGreater(phys.state.vel[0], 0.0)

    def test_the_lidar_resumes_with_fresh_frames_after_an_outage(self):
        from dronesim.sensors.ms200 import MS200Device

        class Rt:
            now_s = 0.0

            def sim_log(self, *args):
                pass
        dev = MS200Device(Rt(), None, None, random.Random(1), {})
        dev.opened(dev.cfg['baud'])
        dev.set_enabled(False, 1.0)
        dev.set_enabled(True, 3.0)                           # back after 2 s
        self.assertAlmostEqual(dev.next_frame_t, 3.0 + dev.frame_period)
        self.assertEqual(dev.bytes_until(int(3.0e9)), b'')   # nothing replayed from the outage

    def _referee(self):
        from dronesim.scoring.referee import Referee
        from dronesim.world.course import Course
        cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
        course = Course(os.path.join(REPO, 'dronehacks-course-v3.step'), cache)
        return Referee(course, os.path.join(SITL, 'course', 'course_v3_rules.json'), self.af)

    def test_landing_gear_on_a_boulder_is_a_touch(self):
        from dronesim.vehicle.contacts import Contact
        ref = self._referee()
        ref.t_start = 0.0
        foot = Contact('boulder', 'Boulder 3', 'gear', (0.79, -2.35, 0.53), (0, 0, 1), 0.1, 0.002)
        body = Contact('boulder', 'Boulder 3', 'body', (0.79, -2.35, 0.55), (0, 0, 1), 0.1, 0.002)
        ref.update(1.0, (0.79, -2.35, 0.6), 0.0, 0.53, True, False, [], [foot], 0.0)
        ref.update(1.1, (0.79, -2.35, 0.6), 0.0, 0.53, True, False, [body], [foot], 0.0)   # the same touch
        self.assertEqual(len(ref.touches), 1)

    def test_an_over_wall_checkpoint_keeps_the_window_bonus(self):
        ref = self._referee()
        ref.t_start, ref.t_finish = 0.0, 50.0
        for cp in ref.checkpoints:
            cp['done_t'] = cp['whole_t'] = 10.0
        ref.checkpoints[0]['over_wall'] = True               # CP1 over a wall: 15 % of its points
        ref.checkpoints[3]['route'] = 'window'
        summary = ref.summary()
        self.assertEqual(summary['window_bonus'], 5)
        self.assertAlmostEqual(summary['checkpoints'][0]['earned'], 0.15 * ref.checkpoints[0]['points'])

    def _returning(self):
        ref = self._referee()
        ref.t_start, ref.t_finish = 0.0, 60.0
        lx, ly = ref.launch_centre[0], ref.launch_centre[1]
        ref.update(65.0, (lx + 3.0, ly + 3.0, 0.8), 0.0, 0.7, True, False, [], [], 2.0)   # took off again
        self.assertEqual(ref.return_state, 'flying')
        return ref, lx, ly

    def test_a_return_disarming_on_the_launch_mark_succeeds(self):
        ref, lx, ly = self._returning()
        ref.update(90.0, (lx + 0.3, ly - 0.2, 0.1), 0.0, 0.0, False, True, [], [], 1.0)
        self.assertTrue(ref.return_over)
        ret = ref.summary()['return']
        self.assertEqual(ret['state'], 'success')
        self.assertEqual(ret['run_points_multiplier'], 1.25)

    def test_a_return_disarming_too_far_from_the_mark_fails_and_keeps_the_outbound_result(self):
        ref, lx, ly = self._returning()
        before = {k: v for k, v in ref.summary().items() if k != 'return'}
        ref.update(90.0, (lx + 0.8, ly, 0.1), 0.0, 0.0, False, True, [], [], 1.0)   # 0.8 m > 2 ft
        ret = ref.summary()['return']
        self.assertEqual(ret['state'], 'failed')
        self.assertEqual(ret['run_points_multiplier'], 1.0)
        self.assertEqual({k: v for k, v in ref.summary().items() if k != 'return'}, before)

    def test_no_return_is_not_attempted_once_the_time_limit_passes(self):
        ref = self._referee()
        ref.t_start, ref.t_finish = 0.0, 60.0
        lx, ly = ref.launch_centre[0], ref.launch_centre[1]
        ref.update(ref.return_time_limit_s + 1.0, (lx, ly, 0.1), 0.0, 0.0, False, True, [], [], 0.0)
        self.assertTrue(ref.return_over)
        self.assertEqual(ref.summary()['return']['state'], 'not attempted')


if __name__ == '__main__':
    unittest.main()
