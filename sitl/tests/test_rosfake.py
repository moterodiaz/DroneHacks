"""The simulated ROS 2 runtime behaves like Humble where team code can tell."""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from dronesim import rosfake  # noqa: E402

rosfake.activate()

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from dronesim.rosfake import launch_impl, packages, runtime  # noqa: E402
from dronesim.rosfake.internal import BEST_EFFORT, RELIABLE, InternalNode  # noqa: E402


class MessageTests(unittest.TestCase):
    def test_float_field_rejects_int_and_float32(self):
        import numpy as np
        from geometry_msgs.msg import Point
        p = Point()
        with self.assertRaises(AssertionError):
            p.x = 1
        with self.assertRaises(AssertionError):
            p.x = np.float32(1.0)
        p.x = np.float64(1.0)          # a subclass of float: accepted, as in rosidl
        self.assertEqual(p.x, 1.0)

    def test_fixed_array_is_numpy_of_exact_dtype(self):
        from px4_msgs.msg import TrajectorySetpoint
        m = TrajectorySetpoint()
        m.position = [float('nan')] * 3
        self.assertEqual(str(m.position.dtype), 'float32')
        with self.assertRaises(AssertionError):
            m.position = [0, 0, 0]

    def test_constants_on_class_and_instance(self):
        from px4_msgs.msg import VehicleStatus
        self.assertEqual(VehicleStatus.NAVIGATION_STATE_OFFBOARD, 14)
        self.assertEqual(VehicleStatus().ARMING_STATE_ARMED, 2)
        self.assertEqual(VehicleStatus.MESSAGE_VERSION, 1)

    def test_service_halves(self):
        from std_srvs.srv import Trigger
        r = Trigger.Response(success=True, message='ok')
        self.assertTrue(r.success)
        self.assertEqual(type(Trigger.Request()).__name__, 'Trigger_Request')

    def test_copy_is_independent(self):
        from dronesim.rosfake.msggen import copy_message
        from nav_msgs.msg import Odometry
        a = Odometry()
        b = copy_message(a)
        b.pose.pose.position.x = 3.0
        self.assertEqual(a.pose.pose.position.x, 0.0)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime.SimRuntime()

    def tearDown(self):
        self.rt.close()

    def run_node(self, main):
        proc = self.rt.start_process('t-1', main)
        return proc

    def test_parameter_type_is_static(self):
        result = {}

        def main():
            rclpy.init()
            node = rclpy.create_node('n', parameter_overrides=[Parameter('a', value=3)])
            try:
                node.declare_parameter('a', 3.0)
            except Exception as exc:  # noqa: BLE001
                result['exc'] = type(exc).__name__
            result['empty'] = node.declare_parameter('b', []).type_
            rclpy.shutdown()
        self.run_node(main)
        self.rt.run_until(int(0.1 * 1e9))
        self.assertEqual(result['exc'], 'InvalidParameterTypeException')
        self.assertEqual(result['empty'], Parameter.Type.BYTE_ARRAY)

    def test_timer_and_spin_on_sim_time(self):
        ticks = []

        def main():
            rclpy.init()
            node = rclpy.create_node('timer_node')
            node.create_timer(0.1, lambda: ticks.append(node.get_clock().now().nanoseconds))
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
        proc = self.run_node(main)
        self.rt.run_until(int(1.05 * 1e9))
        self.assertEqual(len(ticks), 10)
        self.assertAlmostEqual((ticks[1] - ticks[0]) / 1e9, 0.1, places=6)
        self.rt.interrupt_process(proc)
        self.rt.run_until(int(1.1 * 1e9))
        self.assertFalse(proc.alive)

    def test_reliable_reader_never_hears_best_effort_writer(self):
        from std_msgs.msg import String
        got = {'reliable': 0, 'best_effort': 0}
        pub = InternalNode(self.rt, 'pub').publisher(String, '/x', BEST_EFFORT)
        sub = InternalNode(self.rt, 'sub')
        sub.subscribe(String, '/x', lambda m: got.__setitem__('reliable', got['reliable'] + 1), RELIABLE)
        sub.subscribe(String, '/x', lambda m: got.__setitem__('best_effort', got['best_effort'] + 1), BEST_EFFORT)
        self.rt.run_until(int(1e9))
        for _ in range(5):
            pub.publish(String(data='hi'))
        self.rt.run_until(int(2e9))
        self.assertEqual(got['reliable'], 0)
        self.assertEqual(got['best_effort'], 5)

    def test_time_clock_types_do_not_mix(self):
        from rclpy.clock import ClockType
        from rclpy.time import Time
        with self.assertRaises(TypeError):
            _ = Time(seconds=1) < Time(seconds=2, clock_type=ClockType.ROS_TIME)

    def test_qos_profile_needs_depth(self):
        from rclpy.qos import InvalidQoSProfileException, QoSProfile
        with self.assertRaises(InvalidQoSProfileException):
            QoSProfile()

    def test_a_callback_that_sleeps_delays_its_node_instead_of_failing(self):
        import time
        rosfake.patch_globals()
        ticks = []

        def main():
            rclpy.init()
            node = rclpy.create_node('sleeper')

            def cb():
                ticks.append(time.monotonic())
                time.sleep(0.5)                  # inside spin(): blocks this node's executor
            node.create_timer(0.1, cb)
            rclpy.spin(node)
        proc = self.run_node(main)
        self.rt.run_until(int(3e9))
        self.assertGreaterEqual(len(ticks), 3, getattr(proc, 'exit_code', None))
        gaps = [b - a for a, b in zip(ticks, ticks[1:])]
        self.assertGreaterEqual(min(gaps), 0.49)

    def test_time_sleep_is_simulated(self):
        import time
        rosfake.patch_globals()
        stamps = []

        def main():
            stamps.append(time.monotonic())
            time.sleep(2.0)
            stamps.append(time.monotonic())
        self.run_node(main)
        self.rt.run_until(int(3e9))
        self.assertAlmostEqual(stamps[1] - stamps[0], 2.0, places=6)


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime.SimRuntime()
        self.tmp = os.path.join(os.environ.get('TEMP', '/tmp'), 'dronesim_test_launch')
        os.makedirs(self.tmp, exist_ok=True)
        self.ws = packages.Workspace(os.path.join(self.tmp, 'ws'))
        packages.set_active(self.ws)
        self.ws.register_interfaces()

    def tearDown(self):
        self.rt.close()
        self.ws.close()

    def _write(self, text):
        path = os.path.join(self.tmp, 'x.launch.py')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
        return path

    def test_string_parameters_are_yaml_coerced(self):
        from launch.substitutions import TextSubstitution
        ctx = launch_impl.LaunchContext()
        out = launch_impl.evaluate_parameter_dict(ctx, {'a': '3.0', 'b': 'true', 'c': '/x/y',
                                                        'd': [TextSubstitution(text='7')]})
        self.assertEqual(out, {'a': 3.0, 'b': True, 'c': '/x/y', 'd': 7})

    def test_missing_required_argument_fails_the_launch(self):
        path = self._write('from launch import LaunchDescription\n'
                           'from launch.actions import DeclareLaunchArgument\n'
                           'def generate_launch_description():\n'
                           '    return LaunchDescription([DeclareLaunchArgument("policy")])\n')
        sess = launch_impl.LaunchSession(self.ws)
        self.assertFalse(sess.run_file(path))
        self.assertIn("missing required argument 'policy'", str(sess.failed))

    def test_path_join_nested_list(self):
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
        ctx = launch_impl.LaunchContext()
        ctx.launch_configurations['route'] = 'window'
        p = PathJoinSubstitution(['share', 'routes', ['course_v3_', LaunchConfiguration('route'), '.json']])
        self.assertEqual(p.perform(ctx), os.path.join('share', 'routes', 'course_v3_window.json'))


if __name__ == '__main__':
    unittest.main()
