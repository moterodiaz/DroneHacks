"""Behavioural model of PX4 v1.16 EKF2 for this airframe's sensor set.

This is not the 24-state filter.  It is a set of small Kalman filters with the
same inputs, delays, gates, resets and validity rules as EKF2 configured by the
drone's parameters (``EKF2_EV_CTRL`` = 9, ``EKF2_HGT_REF`` = 2 range,
``EKF2_RNG_CTRL`` = 1 conditional, ``EKF2_BARO_CTRL`` = 1, no GNSS, no flow):

* **Local frame.**  At boot the heading comes from the magnetometer.  When
  external-vision yaw fusion starts (pose frame FRD, no GNSS), EKF2 resets its
  heading to the vision yaw and its horizontal position to the vision position:
  from then on PX4's local NED frame *is* cuVSLAM's frame (permuted ENU->NED by
  the bridge), with ``heading_reset_counter`` / ``xy_reset_counter`` bumped.
* **Delayed fusion.**  Each vision sample is fused at the time EKF2 believes it
  was taken: the message's ``timestamp_sample`` (the camera capture time,
  converted by the uXRCE time sync) *minus* ``EKF2_EV_DELAY``.  A delay that is
  wrong for the real pipeline therefore shows up as a velocity-dependent position
  error, as on the vehicle.
* **Gates and resets.**  Innovations beyond ``EKF2_EVP_GATE`` sigma are rejected;
  a change of the bridge's ``reset_counter`` (cuVSLAM re-acquired tracking)
  resets position and heading to the new measurement.
* **Dead reckoning.**  With no vision, horizontal position is propagated from
  the IMU, ``eph`` grows, and after ``EKF2_NOAID_TOUT`` the position is no longer
  valid -- the trigger for PX4's fall back to Altitude.
* **Height.**  The rangefinder is the reference only while horizontal speed is
  below ``EKF2_RNG_A_VMAX``, height below ``EKF2_RNG_A_HMAX`` and the reading is
  consistent (a step, e.g. flying over a boulder, is rejected and must stay
  consistent for ``EKF2_RNG_QLTY_T`` before it is used again).  Otherwise the
  barometer, bias-corrected while the range was in use, is the reference.  The
  local z origin is the floor under the takeoff point (range reference).
"""
from __future__ import annotations

import collections
import math

from ..mathutil import G, q_from_euler, q_rotate, q_to_euler, wrap_pi

NAN = float('nan')


class _Axis2:
    """Position / velocity / accelerometer-bias Kalman filter on one axis.

    EKF2 estimates delta-velocity biases; without that state a constant
    accelerometer bias would leave a steady velocity error that the position
    controller then tracks (a vehicle hovering short of its setpoint).
    """

    __slots__ = ('x', 'v', 'b', 'P')
    Q_BIAS = 0.003 ** 2          # EKF2_ACC_B_NOISE^2 per second

    def __init__(self, x=0.0, v=0.0, pvar=1.0, vvar=0.25):
        self.x, self.v, self.b = x, v, 0.0
        self.P = [[pvar, 0.0, 0.0], [0.0, vvar, 0.0], [0.0, 0.0, 0.05 ** 2]]

    @property
    def pxx(self):
        return self.P[0][0]

    @property
    def pxv(self):
        return self.P[0][1]

    @property
    def pvv(self):
        return self.P[1][1]

    def predict(self, a, dt, q_acc):
        a_corr = a - self.b
        self.x += self.v * dt + 0.5 * a_corr * dt * dt
        self.v += a_corr * dt
        # F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
        P = self.P
        h = -0.5 * dt * dt
        F = ((1.0, dt, h), (0.0, 1.0, -dt), (0.0, 0.0, 1.0))
        FP = [[sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        Pn = [[sum(FP[i][k] * F[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        Pn[0][0] += q_acc * dt ** 4 / 4.0
        Pn[0][1] += q_acc * dt ** 3 / 2.0
        Pn[1][0] += q_acc * dt ** 3 / 2.0
        Pn[1][1] += q_acc * dt * dt
        Pn[2][2] += self.Q_BIAS * dt
        self.P = Pn

    def update(self, innov, r):
        P = self.P
        s = P[0][0] + r
        k = (P[0][0] / s, P[1][0] / s, P[2][0] / s)
        self.x += k[0] * innov
        self.v += k[1] * innov
        self.b += k[2] * innov
        row0 = P[0]
        self.P = [[P[i][j] - k[i] * row0[j] for j in range(3)] for i in range(3)]
        return s

    def reset(self, x, pvar=None):
        self.x = x
        if pvar is not None:
            self.P[0] = [pvar, 0.0, 0.0]
            self.P[1][0] = 0.0
            self.P[2][0] = 0.0


class Ekf2Model:
    HIST_S = 0.6

    def __init__(self, params, rng, boot_heading_true, mag_noise_deg=3.0):
        self.p = params
        self.rng = rng
        # local frame L relative to the true world W (NED): rotation about z and origin
        self.frame_yaw = 0.0                    # yaw of L's x axis in W
        self.frame_origin = (0.0, 0.0, 0.0)     # W position of L's origin
        self.x = _Axis2(0.0, 0.0, 100.0, 1.0)
        self.y = _Axis2(0.0, 0.0, 100.0, 1.0)
        self.z = _Axis2(0.0, 0.0, 1.0, 0.25)
        self.yaw = boot_heading_true + math.radians(rng.gauss(0.0, mag_noise_deg))
        self.yaw_var = math.radians(5.0) ** 2
        self.gyro_bias_z = rng.gauss(0.0, math.radians(0.05))
        self.acc_bias = (rng.gauss(0.0, 0.02), rng.gauss(0.0, 0.02), rng.gauss(0.0, 0.03))
        self.hist = collections.deque()
        self.t = 0.0
        # vision
        self.ev_active = False
        self.ev_yaw_active = False
        self.last_ev_fuse_t = -1e9
        self.last_ev_sample_t = -1e9
        self.ev_reset_counter = None
        self.ev_rejects = 0
        self.ev_innov = (0.0, 0.0)
        self.ev_test_ratio = 0.0
        self.mag_heading = True
        # height
        self.terrain = 0.0                       # NED z of the ground under the vehicle
        self.baro_bias = None
        self.rng_hagl = None
        self.rng_t = -1e9
        self.rng_good_since = None
        self.rng_prev = None
        self.rng_is_ref = False
        self.baro_is_ref = False
        self.height_init = False
        # counters / deltas for vehicle_local_position
        self.xy_reset_counter = 0
        self.z_reset_counter = 0
        self.vxy_reset_counter = 0
        self.vz_reset_counter = 0
        self.heading_reset_counter = 0
        self.delta_xy = (0.0, 0.0)
        self.delta_z = 0.0
        self.delta_vxy = (0.0, 0.0)
        self.delta_vz = 0.0
        self.delta_heading = 0.0
        self.acc_local = (0.0, 0.0, 0.0)
        self.on_ground = True
        self.in_air = False
        self.tilt_aligned = False
        self._boot_t = 0.0

    # -- frame helpers -------------------------------------------------------------------
    def w_to_l(self, p):
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        dx, dy, dz = p[0] - self.frame_origin[0], p[1] - self.frame_origin[1], p[2] - self.frame_origin[2]
        return (c * dx + s * dy, -s * dx + c * dy, dz)

    def l_to_w_vec(self, v):
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])

    # -- prediction ----------------------------------------------------------------------
    def predict(self, t, dt, true_q, specific_force_body, gyro_z):
        """IMU step: rotate the measured specific force with the *estimated* attitude."""
        self.t = t
        roll, pitch, _ = q_to_euler(true_q)
        if t > 0.5:
            self.tilt_aligned = True
        self.yaw = wrap_pi(self.yaw + (gyro_z + self.gyro_bias_z) * dt)
        self.yaw_var += (math.radians(0.15) ** 2) * dt
        q_est = q_from_euler(roll, pitch, self.yaw)
        f = q_rotate(q_est, specific_force_body)
        a = (f[0] + self.acc_bias[0], f[1] + self.acc_bias[1], f[2] + G + self.acc_bias[2])
        if self.on_ground and not self.in_air:
            # zero-velocity behaviour on the ground (EKF2 fuses no motion while at rest)
            a = (0.0, 0.0, 0.0)
        self.acc_local = a
        q_acc = self.p.f('EKF2_ACC_NOISE') ** 2
        self.x.predict(a[0], dt, q_acc)
        self.y.predict(a[1], dt, q_acc)
        self.z.predict(a[2], dt, q_acc)
        if self.on_ground and not self.in_air:
            for ax in (self.x, self.y, self.z):
                ax.v *= 0.9
        self.hist.append((t, self.x.x, self.y.x, self.yaw, self.z.x))
        while self.hist and t - self.hist[0][0] > self.HIST_S:
            self.hist.popleft()

    def _hist_at(self, t):
        h = self.hist
        if not h or t < h[0][0]:
            return None
        if t >= h[-1][0]:
            return h[-1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = h[lo], h[hi]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                a[3] + f * wrap_pi(b[3] - a[3]), a[4] + f * (b[4] - a[4]))

    # -- external vision ---------------------------------------------------------------------
    def fuse_ev(self, t, sample_t, pos, yaw_ev, pos_var, yaw_var, reset_counter, quality):
        """One /fmu/in/vehicle_visual_odometry sample (already in PX4 time, NED/FRD)."""
        ctrl = self.p.i('EKF2_EV_CTRL')
        if quality < self.p.i('EKF2_EV_QMIN'):
            return 'quality'
        if not all(math.isfinite(v) for v in pos[:2]):
            return 'nan'
        self.last_ev_sample_t = t
        assumed = sample_t - self.p.f('EKF2_EV_DELAY') / 1000.0
        if t - assumed > self.p.f('EKF2_DELAY_MAX') / 1000.0 + 0.05:
            return 'too_old'
        r_pos = max(pos_var, self.p.f('EKF2_EVP_NOISE') ** 2) if self.p.i('EKF2_EV_NOISE_MD') == 0 \
            else self.p.f('EKF2_EVP_NOISE') ** 2
        r_yaw = max(yaw_var, self.p.f('EKF2_EVA_NOISE') ** 2) if self.p.i('EKF2_EV_NOISE_MD') == 0 \
            else self.p.f('EKF2_EVA_NOISE') ** 2
        counter_changed = self.ev_reset_counter is not None and reset_counter != self.ev_reset_counter
        self.ev_reset_counter = reset_counter
        starting = not self.ev_active or (t - self.last_ev_fuse_t) > 1.0
        hist = self._hist_at(assumed)
        if hist is None:
            hist = (assumed, self.x.x, self.y.x, self.yaw, self.z.x)
        if (ctrl & 8) and (starting and not self.ev_yaw_active or counter_changed):
            # heading reset to vision: rotate the local frame with it
            new_yaw = wrap_pi(yaw_ev + (self.yaw - hist[3]))
            delta = wrap_pi(new_yaw - self.yaw)
            self._rotate_local_frame(delta)
            self.yaw = new_yaw
            self.yaw_var = r_yaw
            self.heading_reset_counter = (self.heading_reset_counter + 1) % 256
            self.delta_heading = delta
            self.ev_yaw_active = True
            self.mag_heading = False
            hist = self._hist_at(assumed) or (assumed, self.x.x, self.y.x, self.yaw, self.z.x)
        if (ctrl & 1) and (starting or counter_changed):
            dx_now, dy_now = self.x.x - hist[1], self.y.x - hist[2]
            nx, ny = pos[0] + dx_now, pos[1] + dy_now
            self.delta_xy = (nx - self.x.x, ny - self.y.x)
            self.x.reset(nx, r_pos)
            self.y.reset(ny, r_pos)
            self.xy_reset_counter = (self.xy_reset_counter + 1) % 256
            self.ev_active = True
            self.last_ev_fuse_t = t
            self._shift_hist(self.delta_xy)
            return 'reset'
        if ctrl & 1:
            ix, iy = pos[0] - hist[1], pos[1] - hist[2]
            gate = self.p.f('EKF2_EVP_GATE')
            sx, sy = self.x.pxx + r_pos, self.y.pxx + r_pos
            ratio = max(ix * ix / (gate * gate * sx), iy * iy / (gate * gate * sy))
            self.ev_test_ratio = ratio
            self.ev_innov = (ix, iy)
            if ratio > 1.0:
                self.ev_rejects += 1
                if t - self.last_ev_fuse_t > 5.0:
                    # rejected for too long: EKF2 resets to the measurement
                    self.x.reset(pos[0] + self.x.x - hist[1], r_pos)
                    self.y.reset(pos[1] + self.y.x - hist[2], r_pos)
                    self.xy_reset_counter = (self.xy_reset_counter + 1) % 256
                    self.last_ev_fuse_t = t
                    return 'reset'
                return 'rejected'
            self.ev_rejects = 0
            self.x.update(ix, r_pos)
            self.y.update(iy, r_pos)
            self.last_ev_fuse_t = t
            self.ev_active = True
        if ctrl & 8 and self.ev_yaw_active:
            iyaw = wrap_pi(yaw_ev - hist[3])
            k = self.yaw_var / (self.yaw_var + r_yaw)
            self.yaw = wrap_pi(self.yaw + k * iyaw)
            self.yaw_var *= (1 - k)
        return 'fused'

    def _rotate_local_frame(self, delta):
        """Heading reset by ``delta``: position estimates rotate with the frame."""
        c, s = math.cos(-delta), math.sin(-delta)
        self.frame_yaw = wrap_pi(self.frame_yaw - delta)
        x, y = self.x.x, self.y.x
        self.x.x, self.y.x = c * x - s * y, s * x + c * y
        vx, vy = self.x.v, self.y.v
        self.x.v, self.y.v = c * vx - s * vy, s * vx + c * vy
        rotated = collections.deque()
        for (t, hx, hy, hyaw, hz) in self.hist:
            rotated.append((t, c * hx - s * hy, s * hx + c * hy, wrap_pi(hyaw + delta), hz))
        self.hist = rotated

    def _shift_hist(self, d):
        self.hist = collections.deque((t, hx + d[0], hy + d[1], hyaw, hz) for (t, hx, hy, hyaw, hz) in self.hist)

    # -- height ---------------------------------------------------------------------------------
    def fuse_range(self, t, range_m, valid, tilt_cos, landed):
        p = self.p
        if not valid or tilt_cos < 0.7:
            self.rng_hagl = None
            return
        if landed and range_m < p.f('EKF2_MIN_RNG'):
            range_m = p.f('EKF2_MIN_RNG')
        hagl = range_m * tilt_cos + p.f('EKF2_RNG_POS_Z') * tilt_cos
        # kinematic consistency: range rate vs estimated vertical speed
        consistent = True
        if self.rng_prev is not None:
            dt = t - self.rng_prev[0]
            if dt > 1e-3:
                rate = (hagl - self.rng_prev[1]) / dt
                if abs(rate + self.z.v) > max(1.0, 3.0 * p.f('EKF2_RNG_A_IGATE')) and dt < 0.5:
                    consistent = False
        self.rng_prev = (t, hagl)
        if not consistent:
            self.rng_good_since = None
        elif self.rng_good_since is None:
            self.rng_good_since = t
        self.rng_hagl = hagl
        self.rng_t = t
        speed = math.hypot(self.x.v, self.y.v)
        good = self.rng_good_since is not None and (t - self.rng_good_since) >= p.f('EKF2_RNG_QLTY_T') \
            or (self.rng_good_since is not None and not self.height_init)
        cond = speed < p.f('EKF2_RNG_A_VMAX') and hagl < p.f('EKF2_RNG_A_HMAX')
        rng_ctrl = p.i('EKF2_RNG_CTRL')
        use_as_ref = good and p.i('EKF2_HGT_REF') == 2 and (rng_ctrl == 2 or (rng_ctrl == 1 and cond))
        r = (p.f('EKF2_RNG_NOISE') + p.f('EKF2_RNG_SFE') * hagl) ** 2
        if not self.height_init:
            self.z.reset(self.terrain - hagl, r)
            self.z.v = 0.0
            self.height_init = True
            self.z_reset_counter = (self.z_reset_counter + 1) % 256
        if use_as_ref:
            innov = (self.terrain - hagl) - self.z.x
            gate = float(p.get('EKF2_RNG_GATE', 5.0))
            if innov * innov / (gate * gate * (self.z.pxx + r)) < 1.0:
                self.z.update(innov, r)
                self._rng_fused_t = t
            elif t - getattr(self, '_rng_fused_t', t) > 5.0:
                # EKF2: height reference rejected for hgt_fusion_timeout_max -> reset to it
                old = self.z.x
                self.z.reset(self.terrain - hagl, r)
                self.z.v = 0.0
                self.delta_z = self.z.x - old
                self.z_reset_counter = (self.z_reset_counter + 1) % 256
                self._rng_fused_t = t
            self.rng_is_ref = True
            self.baro_is_ref = False
        else:
            self.rng_is_ref = False
            if good and rng_ctrl != 0:
                # range only updates the terrain estimate (EKF2 terrain state)
                meas_terrain = self.z.x + hagl
                if abs(meas_terrain - self.terrain) < 2.0:
                    self.terrain += 0.05 * (meas_terrain - self.terrain)

    def fuse_baro(self, t, baro_z_ned):
        p = self.p
        if p.i('EKF2_BARO_CTRL') == 0:
            return
        if self.baro_bias is None:
            if self.height_init:
                self.baro_bias = baro_z_ned - self.z.x
            elif t > 3.0:
                # no rangefinder at all: the barometer becomes the height reference, origin at boot
                self.baro_bias = baro_z_ned
                self.z.reset(0.0, p.f('EKF2_BARO_NOISE') ** 2)
                self.z.v = 0.0
                self.height_init = True
                self.z_reset_counter = (self.z_reset_counter + 1) % 256
            else:
                return
        if self.rng_is_ref:
            # learn the baro offset while the rangefinder holds the height
            self.baro_bias += 0.004 * ((baro_z_ned - self.z.x) - self.baro_bias)
            return
        self.baro_is_ref = True
        r = p.f('EKF2_BARO_NOISE') ** 2
        innov = (baro_z_ned - self.baro_bias) - self.z.x
        self.z.update(innov, r)

    # -- outputs ----------------------------------------------------------------------------------
    @property
    def dead_reckoning_s(self):
        return self.t - self.last_ev_fuse_t if self.ev_active else float('inf')

    def xy_valid(self):
        if not self.ev_active:
            return False
        return self.dead_reckoning_s < self.p.f('EKF2_NOAID_TOUT') / 1e6

    def eph(self):
        return math.sqrt(max(self.x.pxx, self.y.pxx))

    def local_position(self):
        hagl_valid = self.rng_hagl is not None and self.t - self.rng_t < 0.3
        return {
            'x': self.x.x, 'y': self.y.x, 'z': self.z.x,
            'vx': self.x.v, 'vy': self.y.v, 'vz': self.z.v,
            'ax': self.acc_local[0], 'ay': self.acc_local[1], 'az': self.acc_local[2],
            'heading': self.yaw, 'xy_valid': self.xy_valid(), 'z_valid': self.height_init,
            'v_xy_valid': self.xy_valid(), 'v_z_valid': self.height_init,
            'eph': self.eph(), 'epv': math.sqrt(self.z.pxx),
            'evh': math.sqrt(max(self.x.pvv, self.y.pvv)), 'evv': math.sqrt(self.z.pvv),
            'dist_bottom': (self.terrain - self.z.x) if self.height_init else NAN,
            'dist_bottom_valid': hagl_valid,
            'dead_reckoning': self.ev_active and self.dead_reckoning_s > 0.5,
        }
