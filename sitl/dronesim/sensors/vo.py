"""The ``slam`` window: OAK-D-S2 stereo + cuVSLAM 3.2 (stereo-only), as a sensor model.

Publishes what Isaac ROS visual_slam publishes on the drone with the
organizers' launch settings (``oakd_vslam.launch.py``):

  /visual_slam/tracking/odometry          nav_msgs/Odometry, ENU ``odom`` -> FLU ``base_link``
  /visual_slam/tracking/vo_pose           geometry_msgs/PoseStamped
  /visual_slam/tracking/vo_pose_covariance geometry_msgs/PoseWithCovarianceStamped
  /visual_slam/status                     isaac_ros_visual_slam_interfaces/VisualSlamStatus
  /tf                                      map -> odom (identity), odom -> base_link

Model
-----
* **Frame.**  ``odom`` is base_link's pose when tracking first starts: x =
  the nose at startup, y = left, z = up (no gravity, no compass).
* **Timing.**  30 Hz frames; each odometry message carries the *capture* time in
  its header and is published ``latency`` later (the organizers measured image
  -> bridge publish p50 68.7 ms, p95 102 ms; the default here is 62 +- 10 ms
  for cuVSLAM alone).
* **Drift.**  Pose increments are integrated with a scale error, a heading
  error that rotates every later step (so it grows with distance), random-walk
  noise per metre travelled (z worst: stereo VO's weakest axis) and per radian
  turned, plus white output noise.
* **Tracking.**  Each frame casts a grid of rays through the camera frustum
  against the STEP mesh.  Surfaces inside the dark tunnel give no features, the
  black net few, the lit floor / walls / boulders / room beyond the net give
  features.  Too few features for ``lost_frames`` frames, or motion blur from
  fast rotation, loses tracking: no odometry, ``vo_state`` 2.  On re-acquisition
  the pose continues from where it froze (cuVSLAM VO cannot know the motion it
  missed) -- the odom frame has jumped, and the bridge bumps ``reset_counter``.
"""
from __future__ import annotations

import math

import numpy as np

from ..mathutil import q_conj, q_mul, q_rotate, q_yaw
from ..rosfake.internal import RELIABLE, InternalNode
from ..world.course import ned_to_course

DEFAULTS = {
    'rate_hz': 30.0,
    'start_delay_s': 2.5,
    'latency_mean_s': 0.062,
    'latency_std_s': 0.010,
    'drop_prob': 0.005,
    'scale_error': 0.006,
    'yaw_drift_deg_per_m': 0.25,
    'yaw_drift_deg_per_rad': 0.4,
    'pos_rw_per_sqrt_m': 0.004,
    'z_rw_per_sqrt_m': 0.012,
    'noise_pos_m': 0.002,
    'noise_yaw_deg': 0.05,
    'hfov_deg': 72.0,
    'vfov_deg': 49.0,
    'rays_x': 7,
    'rays_y': 5,
    'min_depth_m': 0.25,
    'max_depth_m': 8.0,
    'tunnel_lighting': 'dark',       # dark | dim | lit
    'min_feature_score': 0.12,       # below: most of the view is dark, blank or too close
    'recover_feature_score': 0.25,
    'lost_frames': 3,
    'recover_frames': 6,
    'blur_rate_rad_s': 4.0,
    'recovery': 'freeze',            # freeze: pose resumes from the last tracked pose
    'force_lost': False,
}

#: texture weight per surface kind; the black net is see-through netting, so a beam that
#: hits it sees the lit hall behind (same weight as a beam that leaves the course)
WEIGHTS = {'floor': 1.0, 'pad': 1.0, 'marking': 1.0, 'boulder': 1.0, 'wall': 0.8, 'tunnel_support': 0.7,
           'tunnel': 0.8, 'net': 0.6, 'other': 0.6}


class CuVslamModel:
    def __init__(self, rt, physics, course, rng, cfg=None, camera_flu=(0.1053, 0.0, 0.0304)):
        self.rt = rt
        self.physics = physics
        self.course = course
        self.rng = rng
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg or {})
        self.camera_flu = camera_flu
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        self.Odometry, self.PoseStamped, self.PoseCov = Odometry, PoseStamped, PoseWithCovarianceStamped
        self.Status, self.TFMessage = VisualSlamStatus, TFMessage
        self.node = InternalNode(rt, 'visual_slam')
        self.pub_odom = self.node.publisher(Odometry, '/visual_slam/tracking/odometry', RELIABLE)
        self.pub_pose = self.node.publisher(PoseStamped, '/visual_slam/tracking/vo_pose', RELIABLE)
        self.pub_pose_cov = self.node.publisher(PoseWithCovarianceStamped,
                                                '/visual_slam/tracking/vo_pose_covariance', RELIABLE)
        self.pub_status = self.node.publisher(VisualSlamStatus, '/visual_slam/status', RELIABLE)
        self.pub_tf = self.node.publisher(TFMessage, '/tf', RELIABLE)
        self.started = False
        self.origin = None                 # (pos_ned, q) of base_link at startup
        self.rep_pos = [0.0, 0.0, 0.0]     # reported odom position (FLU-of-start, i.e. ENU-labelled)
        self.rep_yaw_err = 0.0
        self.scale = 1.0 + rng.gauss(0.0, self.cfg['scale_error'])
        self.prev_true = None
        self.tracking = False
        self.vo_state = 0
        self.bad_frames = 0
        self.good_frames = 0
        self.frames = 0
        self.feature_score = 0.0
        self.lost_events = 0
        self.last_odom = None              # (t_capture, x, y, z, yaw) for telemetry
        self.errors = []                   # each published pose's horizontal error (m) at its capture
        self._tube = self._tube_geometry()
        self._ray_dirs_cam = self._make_rays()
        rt.after(self.cfg['start_delay_s'], self._frame)

    # -- geometry ---------------------------------------------------------------------------------
    def _tube_geometry(self):
        names = [n for n in self.course.by_name if 'Tube oval' in n]
        if not names:
            return None
        s = self.course.by_name[names[0]]
        cx = (s.bbox_min[0] + s.bbox_max[0]) / 2.0
        cz = (s.bbox_min[2] + s.bbox_max[2]) / 2.0
        y0, y1 = float(s.bbox_min[1]), float(s.bbox_max[1])
        # inner half-axes by raycast from the axis
        o = np.array([[cx, (y0 + y1) / 2, cz], [cx, (y0 + y1) / 2, cz]])
        d = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        dist, _sid, _n = self.course.raycast_course(o, d, 3.0)
        ax = float(dist[0]) if math.isfinite(dist[0]) else (s.bbox_max[0] - s.bbox_min[0]) / 2
        az = float(dist[1]) if math.isfinite(dist[1]) else (s.bbox_max[2] - s.bbox_min[2]) / 2
        return {'cx': float(cx), 'cz': float(cz), 'y0': y0, 'y1': y1, 'ax': ax, 'az': az}

    def in_tube(self, p):
        t = self._tube
        if t is None or not (t['y0'] <= p[1] <= t['y1']):
            return False
        return ((p[0] - t['cx']) / t['ax']) ** 2 + ((p[2] - t['cz']) / t['az']) ** 2 < 1.0

    def _make_rays(self):
        nx, ny = int(self.cfg['rays_x']), int(self.cfg['rays_y'])
        hf, vf = math.radians(self.cfg['hfov_deg']) / 2, math.radians(self.cfg['vfov_deg']) / 2
        dirs = []
        for i in range(nx):
            for j in range(ny):
                az = -hf + 2 * hf * (i + 0.5) / nx
                el = -vf + 2 * vf * (j + 0.5) / ny
                dirs.append((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))  # FLU
        return np.array(dirs)

    def _feature_score(self, pos_ned, q):
        """Fraction of the view that cuVSLAM could track features in (0..1)."""
        cam_frd = (self.camera_flu[0], -self.camera_flu[1], -self.camera_flu[2])
        cam_ned = tuple(pos_ned[i] + q_rotate(q, cam_frd)[i] for i in range(3))
        cam_c = ned_to_course(cam_ned)
        dirs_c = []
        for d in self._ray_dirs_cam:
            frd = (d[0], -d[1], -d[2])
            w = q_rotate(q, frd)
            dirs_c.append(ned_to_course(w))
        dirs_c = np.array(dirs_c)
        origins = np.repeat(np.array([cam_c]), len(dirs_c), axis=0)
        dist, sid, _n = self.course.raycast_course(origins, dirs_c, self.cfg['max_depth_m'] + 4.0)
        lighting = self.cfg['tunnel_lighting']
        dark_w = {'dark': 0.0, 'dim': 0.35, 'lit': 1.0}.get(lighting, 0.0)
        cam_in_tube = self.in_tube(cam_c)
        total = 0.0
        # From inside a dark tube, auto-exposure on the dark interior blows out the bright openings:
        # what lies beyond them gives almost no usable features.
        exit_w = {'dark': 0.15, 'dim': 0.5, 'lit': 1.0}.get(lighting, 0.15)
        for k in range(len(dirs_c)):
            d = float(dist[k])
            if not math.isfinite(d):
                w = 0.6 * (exit_w if cam_in_tube else 1.0)        # lit hall beyond the net
            elif d < self.cfg['min_depth_m']:
                w = 0.0
            else:
                kind = self.course.solid_kind[int(sid[k])] if sid[k] >= 0 else 'other'
                w = WEIGHTS.get(kind, 0.6)
                hit = cam_c + dirs_c[k] * d
                if kind == 'tunnel' and self._near_inner_wall(hit):
                    w *= dark_w          # the tube's inside: no light
                elif cam_in_tube:
                    w *= exit_w          # seen through an opening, overexposed
                if d > self.cfg['max_depth_m']:
                    w *= 0.4
            total += w
        return total / len(dirs_c)

    def _near_inner_wall(self, p):
        t = self._tube
        if t is None or not (t['y0'] - 0.01 <= p[1] <= t['y1'] + 0.01):
            return False
        r = ((p[0] - t['cx']) / t['ax']) ** 2 + ((p[2] - t['cz']) / t['az']) ** 2
        # inner surface sits at r = 1, the outer one ~1.4 % further out (1.27 cm wall)
        return r < 1.006

    # -- the frame loop ------------------------------------------------------------------------------
    def _frame(self):
        rt, cfg = self.rt, self.cfg
        rt.after(1.0 / cfg['rate_hz'], self._frame)
        s = self.physics.state
        pos, q = s.pos, s.q
        t_capture_ns = rt.ros_time_ns()
        self.frames += 1
        if not self.started:
            self.started = True
            self.origin = (pos, q)
            self.prev_true = (pos, q)
            self.tracking = True
            self.vo_state = 1
            self.node.log('INFO', 'cuVSLAM: tracking started; odom frame = base_link at this pose')
        # tracking quality
        score = self._feature_score(pos, q)
        self.feature_score = score
        rate = math.sqrt(sum(w * w for w in s.omega))
        blurred = rate > cfg['blur_rate_rad_s'] and self.rng.random() < 0.5
        bad = score < cfg['min_feature_score'] or blurred or cfg['force_lost']
        if self.tracking:
            self.bad_frames = self.bad_frames + 1 if bad else 0
            if self.bad_frames >= cfg['lost_frames']:
                self.tracking = False
                self.vo_state = 2
                self.good_frames = 0
                self.lost_events += 1
                self.node.log('WARN', 'Visual tracking is lost')
        else:
            good = score >= cfg['recover_feature_score'] and not blurred and not cfg['force_lost']
            self.good_frames = self.good_frames + 1 if good else 0
            if self.good_frames >= cfg['recover_frames']:
                self.tracking = True
                self.vo_state = 1
                self.bad_frames = 0
                self.prev_true = (pos, q)     # resume: the motion during the outage is lost
                self.node.log('INFO', 'Visual tracking re-acquired (pose continues from the last tracked pose)')
        self._publish_status(t_capture_ns)
        if not self.tracking:
            self.prev_true = None
            return
        if self.prev_true is None:
            self.prev_true = (pos, q)
        self._integrate(pos, q)
        if self.rng.random() < cfg['drop_prob']:
            return
        latency = max(0.03, self.rng.gauss(cfg['latency_mean_s'], cfg['latency_std_s']))
        msgs = self._build(t_capture_ns, s)
        # the pipeline is sequential: jitter delays frames but never reorders them
        publish_ns = max(rt.now_ns + int(latency * 1e9), getattr(self, '_last_publish_ns', 0) + 1_000_000)
        self._last_publish_ns = publish_ns
        rt.at(publish_ns, self._publish, msgs)

    def _integrate(self, pos, q):
        cfg = self.cfg
        p_prev, q_prev = self.prev_true
        p0, q0 = self.origin
        # step in the startup body frame (FRD) -> FLU (= odom's ENU labels)
        step_w = (pos[0] - p_prev[0], pos[1] - p_prev[1], pos[2] - p_prev[2])
        step_b = q_rotate(q_conj(q0), step_w)
        step = (step_b[0], -step_b[1], -step_b[2])
        dist = math.sqrt(sum(v * v for v in step))
        dyaw = abs(math.atan2(math.sin(q_yaw(q) - q_yaw(q_prev)), math.cos(q_yaw(q) - q_yaw(q_prev))))
        g = self.rng.gauss
        self.rep_yaw_err += g(0.0, math.radians(cfg['yaw_drift_deg_per_m']) * math.sqrt(dist + 1e-12)) + \
            g(0.0, math.radians(cfg['yaw_drift_deg_per_rad']) * math.sqrt(dyaw + 1e-12))
        c, s_ = math.cos(self.rep_yaw_err), math.sin(self.rep_yaw_err)
        sx = self.scale * (c * step[0] - s_ * step[1])
        sy = self.scale * (s_ * step[0] + c * step[1])
        sz = self.scale * step[2]
        rw = cfg['pos_rw_per_sqrt_m'] * math.sqrt(dist)
        self.rep_pos[0] += sx + g(0.0, rw)
        self.rep_pos[1] += sy + g(0.0, rw)
        self.rep_pos[2] += sz + g(0.0, cfg['z_rw_per_sqrt_m'] * math.sqrt(dist))
        self.prev_true = (pos, q)

    def odom_orientation(self, q):
        """Orientation of base_link (FLU) in odom, true part only."""
        q0 = self.origin[1]
        q_rel = q_mul(q_conj(q0), q)              # FRD relative rotation
        return (q_rel[0], q_rel[1], -q_rel[2], -q_rel[3])   # conjugate by 180 deg about x -> FLU

    def _build(self, t_ns, s):
        cfg = self.cfg
        g = self.rng.gauss
        x = self.rep_pos[0] + g(0.0, cfg['noise_pos_m'])
        y = self.rep_pos[1] + g(0.0, cfg['noise_pos_m'])
        z = self.rep_pos[2] + g(0.0, cfg['noise_pos_m'])
        qt = self.odom_orientation(s.q)
        ye = self.rep_yaw_err + math.radians(g(0.0, cfg['noise_yaw_deg']))
        qz = (math.cos(ye / 2), 0.0, 0.0, math.sin(ye / 2))
        q = q_mul(qz, qt)                                                 # wxyz
        v_frd = q_rotate(q_conj(s.q), s.vel)
        v_flu = (v_frd[0] + g(0, 0.01), -v_frd[1] + g(0, 0.01), -v_frd[2] + g(0, 0.01))
        w_flu = (s.omega[0], -s.omega[1], -s.omega[2])
        stamp_sec, stamp_ns = int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000)
        od = self.Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = stamp_sec, stamp_ns
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        pp = od.pose.pose
        pp.position.x, pp.position.y, pp.position.z = float(x), float(y), float(z)
        pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z = (float(v) for v in q)
        if self.frames <= 2:
            cov = [0.0] * 36
            for i in (0, 7, 14, 21, 28, 35):
                cov[i] = 1.0                       # cuVSLAM's identity fallback on the first frames
        else:
            cov = [0.0] * 36
            for i in (0, 7, 14):
                cov[i] = 2.5e-5
            for i in (21, 28, 35):
                cov[i] = 1.0e-5
        od.pose.covariance = cov
        tw = od.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = (float(v) for v in v_flu)
        tw.angular.x, tw.angular.y, tw.angular.z = (float(v) for v in w_flu)
        tcov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            tcov[i] = 1e-4
        od.twist.covariance = tcov
        ps = self.PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        pc = self.PoseCov()
        pc.header = od.header
        pc.pose = od.pose
        from geometry_msgs.msg import TransformStamped
        tf_map = TransformStamped()
        tf_map.header.stamp = od.header.stamp
        tf_map.header.frame_id = 'map'
        tf_map.child_frame_id = 'odom'
        tf_map.transform.rotation.w = 1.0
        tf_odom = TransformStamped()
        tf_odom.header.stamp = od.header.stamp
        tf_odom.header.frame_id = 'odom'
        tf_odom.child_frame_id = 'base_link'
        tf_odom.transform.translation.x, tf_odom.transform.translation.y, tf_odom.transform.translation.z = \
            float(x), float(y), float(z)
        tf_odom.transform.rotation = od.pose.pose.orientation
        tfm = self.TFMessage(transforms=[tf_map, tf_odom])
        self.last_odom = (t_ns / 1e9, x, y, z, 2 * math.atan2(q[3], q[0]))
        # the drift a consumer sees: this pose against the truth when it was captured (latency
        # does not add to it; it only decides whether PX4 still uses the sample)
        (ox, oy, _oz), heading = self.odom_frame()
        tc = ned_to_course(s.pos)
        ex = ox + math.cos(heading) * x - math.sin(heading) * y - tc[0]
        ey = oy + math.sin(heading) * x + math.cos(heading) * y - tc[1]
        self.errors.append(math.hypot(ex, ey))
        return od, ps, pc, tfm

    def _publish(self, msgs):
        od, ps, pc, tfm = msgs
        self.pub_odom.publish(od)
        self.pub_pose.publish(ps)
        self.pub_pose_cov.publish(pc)
        self.pub_tf.publish(tfm)

    def _publish_status(self, t_ns):
        st = self.Status()
        st.header.stamp.sec, st.header.stamp.nanosec = int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000)
        st.header.frame_id = 'odom'
        st.vo_state = int(self.vo_state)
        st.node_callback_execution_time = 0.018
        st.track_execution_time = 0.009
        st.track_execution_time_mean = 0.009
        st.track_execution_time_max = 0.02
        self.rt.after(0.035, self.pub_status.publish, st)

    # -- truth in the odom frame, for scoring / telemetry -------------------------------------------
    def odom_frame(self):
        """(origin course xy, odom +x heading in course, CCW from east) or None before start."""
        if self.origin is None:
            return None
        p0, q0 = self.origin
        c = ned_to_course(p0)
        yaw_ned = q_yaw(q0)                 # clockwise from course north
        return (c[0], c[1], c[2]), math.pi / 2 - yaw_ned
