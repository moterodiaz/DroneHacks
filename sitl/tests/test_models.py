"""PX4 control maths, the MS200 frame codec, parameters and the referee's gates."""
import math
import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
sys.path.insert(0, SITL)

from dronesim.px4 import control  # noqa: E402
from dronesim.px4.params import CONTRACT_PX4, PROFILES, load_profile  # noqa: E402
from dronesim.sensors.ms200 import POINTS_PER_FRAME, crc8, encode_frame  # noqa: E402


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = load_profile(REPO, 'drone1-fleet')

    def test_allocation_gives_equal_torque_free_hover(self):
        alloc = control.ControlAllocator(self.params)
        u = alloc.allocate((0.0, 0.0, 0.0), (0.0, 0.0, -0.5))
        # the rotor geometry is asymmetric (0.08 m front, 0.15 m rear): the front rotors must
        # work harder for zero pitch torque, and the result still produces no torque
        torque = alloc.effectiveness_scaled[:3] @ __import__('numpy').array(u)
        self.assertLess(max(abs(t) for t in torque), 1e-6)
        self.assertGreater(u[0], u[1])

    def test_constrain_xy_prioritises_first_vector(self):
        v = control.constrain_xy((0.6, 0.0), (0.0, 0.6), 1.0)          # under the limit: unchanged
        self.assertAlmostEqual(v[1], 0.6, places=6)
        v = control.constrain_xy((0.6, 0.0), (0.0, 1.0), 1.0)          # over: v0 kept, v1 scaled
        self.assertAlmostEqual(math.hypot(*v), 1.0, places=6)
        self.assertAlmostEqual(v[0], 0.6, places=6)
        self.assertAlmostEqual(v[1], 0.8, places=6)

    def test_position_control_holds_and_limits(self):
        pc = control.PositionControl()
        pc.set_params(self.params)
        pc.set_velocity_limits(1.0, 3.0, 1.5)
        pc.set_thrust_limits(0.12, 1.0)
        pc.set_state((0.0, 0.0, -1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        nan = float('nan')
        pc.set_input_setpoint((10.0, 0.0, -1.0), (nan,) * 3, (nan,) * 3, nan, nan)
        self.assertTrue(pc.update(0.01))
        self.assertAlmostEqual(math.hypot(pc.vel_sp[0], pc.vel_sp[1]), 1.0, places=6)   # MPC_XY_VEL_MAX
        q, thrust, _ = pc.attitude_setpoint()
        self.assertLess(thrust, 0.0)

    def test_takeoff_ramp_starts_below_zero(self):
        tk = control.TakeoffHandling()
        tk.set_params(self.params)
        tk.update_state(True, True, True, 0.0)
        tk.update_state(True, True, True, 2.0)
        self.assertEqual(tk.state, control.RAMPUP)
        v0 = tk.update_ramp(0.01, 3.0)
        self.assertLess(v0, 0.0)


class ParamTests(unittest.TestCase):
    def test_contract_overlay_and_flight_modes(self):
        p = load_profile(REPO, 'drone1-fleet')
        for k, v in CONTRACT_PX4.items():
            self.assertEqual(p.get(k), v, k)
        self.assertEqual(p.i('COM_FLTMODE6'), 2)          # Position on the switch
        self.assertEqual(p.i('MAV_SYS_ID'), 1)

    def test_drone1_dump_has_manual_on_slot_6(self):
        p = load_profile(REPO, 'drone1-as-dumped')
        self.assertEqual(p.i('COM_FLTMODE6'), 0)

    def test_every_profile_loads(self):
        for name in PROFILES:
            load_profile(REPO, name)


class Ms200Tests(unittest.TestCase):
    def test_frame_layout_and_crc(self):
        f = encode_frame(3600, 10.0, 19.5, [1000] * POINTS_PER_FRAME, [200] * POINTS_PER_FRAME, 29999)
        self.assertEqual(len(f), 47)
        self.assertEqual(f[:2], b'\x54\x2c')
        self.assertEqual(crc8(f[:-1]), f[-1])
        fields = struct.unpack('<BBHH' + 'HB' * 12 + 'HHB', f)
        self.assertEqual(fields[2], 3600)
        self.assertEqual(fields[3], 1000)
        self.assertEqual(fields[-2], 29999)


class RefereeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dronesim.scoring.referee import Referee
        from dronesim.vehicle.physics import AirframeConfig
        from dronesim.world.course import Course
        cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
        cls.course = Course(os.path.join(REPO, 'dronehacks-course-v3.step'), cache)
        cls.af = AirframeConfig.from_params(load_profile(REPO, 'drone1-fleet'))
        cls.Referee = Referee
        cls.rules = os.path.join(SITL, 'course', 'course_v3_rules.json')

    def test_gates_from_the_step_model(self):
        ref = self.Referee(self.course, self.rules, self.af)
        gates = {(g['cp'], g['route']): g for g in ref.gates_for_display()}
        win = gates[(4, 'window')]
        self.assertAlmostEqual(win['z'][0], 0.3048, places=3)
        self.assertAlmostEqual(win['z'][1], 1.2192, places=3)
        mid = ((win['a'][0] + win['b'][0]) / 2, (win['a'][1] + win['b'][1]) / 2)
        self.assertAlmostEqual(mid[0], -0.4709, delta=0.01)
        self.assertAlmostEqual(mid[1], -2.2719, delta=0.01)
        width = math.dist(win['a'], win['b'])
        self.assertAlmostEqual(width, 0.9144, delta=0.01)
        tube = gates[(3, 'main')]
        self.assertAlmostEqual(tube['a'][1], -0.3048, places=3)

    def test_whole_drone_must_pass(self):
        ref = self.Referee(self.course, self.rules, self.af)
        ref.t_start = 0.0
        y_turn = 2.70                      # centre beyond the CP1 line, tail not yet
        path = [(-2.5, 2.3 + 0.01 * i) for i in range(int((y_turn - 2.3) / 0.01))]
        t = 0.0
        for p in path:
            t += 0.02
            ref.update(t, (p[0], p[1], 0.8), math.pi / 2, 0.73, True, False, [], [], 0.0)
        cp1 = ref.checkpoints[0]
        self.assertIsNotNone(cp1['centre_t'])
        self.assertIsNone(cp1['whole_t'])
        for i in range(30):
            t += 0.02
            ref.update(t, (-2.5, y_turn + 0.01 * i, 0.8), math.pi / 2, 0.73, True, False, [], [], 0.0)
        self.assertIsNotNone(cp1['whole_t'])


if __name__ == '__main__':
    unittest.main()
