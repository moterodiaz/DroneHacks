"""OAK-D-S2 side channels: the BNO086 IMU, camera_info, and the camera TF tree.

* ``/oak/imu/data`` (sensor_msgs/Imu) at 200 Hz, no fused orientation
  (``i_enable_rotation: false``), frame ``oak_imu_frame``.  Published only while
  someone subscribes, to keep the simulation fast.
* ``/oak/{left,right}/camera_info`` at 30 Hz, 1024x640, plumb_bob.  Images are
  not rendered (no team node in this repository consumes them); see README.
* ``/tf_static``: base_link -> oak_parent_frame from drone.env's CAMERA_X/Y/Z/
  ROLL/PITCH/YAW (what ``oakd_vslam.launch.py``'s static_transform_publisher
  sends), and the optical frames below it.
"""
from __future__ import annotations

import math

from ..mathutil import G, q_conj, q_rotate
from ..rosfake.internal import LATCHED, RELIABLE, InternalNode


class OakSide:
    def __init__(self, rt, physics, rng, camera_pose=(0.1053, 0.0, 0.0304, 0.0, 0.0, 0.0)):
        self.rt = rt
        self.physics = physics
        self.rng = rng
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, Imu
        from tf2_msgs.msg import TFMessage
        self.Imu, self.CameraInfo = Imu, CameraInfo
        self.node = InternalNode(rt, 'oak')
        self.pub_imu = self.node.publisher(Imu, '/oak/imu/data', RELIABLE)
        self.pub_info = [self.node.publisher(CameraInfo, f'/oak/{side}/camera_info', RELIABLE)
                         for side in ('left', 'right')]
        tf_node = InternalNode(rt, 'oak_static_tf')
        pub_static = tf_node.publisher(TFMessage, '/tf_static', LATCHED)
        x, y, z, roll, pitch, yaw = camera_pose
        transforms = []

        def tf(parent, child, xyz, rpy):
            t = TransformStamped()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
            cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
            cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
            cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
            t.transform.rotation.w = cr * cp * cy + sr * sp * sy
            t.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t.transform.rotation.z = cr * cp * sy - sr * sp * cy
            transforms.append(t)

        tf('base_link', 'oak_parent_frame', (x, y, z), (roll, pitch, yaw))
        tf('oak_parent_frame', 'oak', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        tf('oak', 'oak_left_camera_frame', (0.0, 0.0375, 0.0), (0.0, 0.0, 0.0))
        tf('oak', 'oak_right_camera_frame', (0.0, -0.0375, 0.0), (0.0, 0.0, 0.0))
        tf('oak_left_camera_frame', 'oak_left_camera_optical_frame', (0, 0, 0), (-math.pi / 2, 0.0, -math.pi / 2))
        tf('oak_right_camera_frame', 'oak_right_camera_optical_frame', (0, 0, 0), (-math.pi / 2, 0.0, -math.pi / 2))
        tf('oak', 'oak_imu_frame', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        rt.after(0.5, pub_static.publish, TFMessage(transforms=transforms))
        rt.after(1.0, self._imu_tick)
        rt.after(1.0, self._info_tick)

    def _imu_tick(self):
        self.rt.after(1.0 / 200.0, self._imu_tick)
        if self.pub_imu.subscription_count() == 0:
            return
        s = self.physics.state
        g = self.rng.gauss
        m = self.Imu()
        t = self.rt.ros_time_ns()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t // 1_000_000_000), int(t % 1_000_000_000)
        m.header.frame_id = 'oak_imu_frame'
        m.orientation_covariance = [-1.0] + [0.0] * 8
        w = s.omega
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
            float(w[0] + g(0, 0.003)), float(-w[1] + g(0, 0.003)), float(-w[2] + g(0, 0.003))
        sf = s.specific_force_body
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
            float(sf[0] + g(0, 0.05)), float(-sf[1] + g(0, 0.05)), float(-sf[2] + g(0, 0.05))
        self.pub_imu.publish(m)

    def _info_tick(self):
        self.rt.after(1.0 / 30.0, self._info_tick)
        for side, pub in zip(('left', 'right'), self.pub_info):
            if pub.subscription_count() == 0:
                continue
            m = self.CameraInfo()
            t = self.rt.ros_time_ns()
            m.header.stamp.sec, m.header.stamp.nanosec = int(t // 1_000_000_000), int(t % 1_000_000_000)
            m.header.frame_id = f'oak_{side}_camera_optical_frame'
            m.width, m.height = 1024, 640
            m.distortion_model = 'plumb_bob'
            m.d = [0.0] * 5
            fx = 610.0
            m.k = [fx, 0.0, 512.0, 0.0, fx, 320.0, 0.0, 0.0, 1.0]
            m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            tx = 0.0 if side == 'left' else -fx * 0.075
            m.p = [fx, 0.0, 512.0, tx, 0.0, fx, 320.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            pub.publish(m)


_unused = (G, q_conj, q_rotate)
