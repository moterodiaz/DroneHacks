"""PX4 v1.16 multicopter control libraries, ported.

Each class follows the PX4 source it is named after; comments give the file.
All frames are PX4's: world NED, body FRD.  Units SI, angles radians.

* :class:`PositionControl`   -- src/modules/mc_pos_control/PositionControl/PositionControl.cpp
* :class:`TakeoffHandling`   -- src/modules/mc_pos_control/Takeoff/Takeoff.cpp
* :func:`thrust_to_attitude` -- src/modules/mc_pos_control/PositionControl/ControlMath.cpp
* :class:`AttitudeControl`   -- src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
* :class:`RateControl`       -- src/lib/rate_control/rate_control.cpp
* :class:`ControlAllocator`  -- src/modules/control_allocator (rotor effectiveness,
  pseudo-inverse with PX4's normalisation, sequential desaturation, airmode off)
* :class:`HoverThrustEstimator` -- a simplified mc_hover_thrust_estimator
"""
from __future__ import annotations

import math

import numpy as np

from ..mathutil import (G, constrain, cross, dot, norm, normalized, q_canonical, q_conj, q_dcm_z,
                        q_from_dcm_columns, q_from_two_vectors, q_mul, q_rotate)

NAN = float('nan')


def _isf(x):
    return math.isfinite(x)


# ---------------------------------------------------------------------------
# ControlMath
# ---------------------------------------------------------------------------
def constrain_xy(v0, v1, vmax):
    """ControlMath::constrainXY: v0 has priority over v1, ||result|| <= vmax."""
    sx, sy = v0[0] + v1[0], v0[1] + v1[1]
    if math.hypot(sx, sy) <= vmax:
        return (sx, sy)
    n0 = math.hypot(v0[0], v0[1])
    if n0 >= vmax:
        return (v0[0] / n0 * vmax, v0[1] / n0 * vmax)
    if math.hypot(v1[0] - v0[0], v1[1] - v0[1]) < 0.001:
        return (v0[0] / n0 * vmax, v0[1] / n0 * vmax) if n0 > 0 else (0.0, 0.0)
    if n0 < 0.001:
        n1 = math.hypot(v1[0], v1[1])
        return (v1[0] / n1 * vmax, v1[1] / n1 * vmax)
    n1 = math.hypot(v1[0], v1[1])
    u1 = (v1[0] / n1, v1[1] / n1)
    m = u1[0] * v0[0] + u1[1] * v0[1]
    c = v0[0] * v0[0] + v0[1] * v0[1] - vmax * vmax
    s = -m + math.sqrt(max(0.0, m * m - c))
    return (v0[0] + u1[0] * s, v0[1] + u1[1] * s)


def limit_tilt(body_unit, world_unit, max_angle):
    """ControlMath::limitTilt."""
    d = dot(body_unit, world_unit)
    angle = math.acos(constrain(d, -1.0, 1.0))
    angle = min(angle, max_angle)
    rej = (body_unit[0] - d * world_unit[0], body_unit[1] - d * world_unit[1], body_unit[2] - d * world_unit[2])
    if dot(rej, rej) < 1e-12:
        rej = (1.0, rej[1], rej[2])
    rn = normalized(rej)
    ca, sa = math.cos(angle), math.sin(angle)
    return (ca * world_unit[0] + sa * rn[0], ca * world_unit[1] + sa * rn[1], ca * world_unit[2] + sa * rn[2])


def body_z_to_attitude(body_z, yaw_sp):
    """ControlMath::bodyzToAttitude -> quaternion."""
    if dot(body_z, body_z) < 1e-12:
        body_z = (body_z[0], body_z[1], 1.0)
    body_z = normalized(body_z)
    y_c = (-math.sin(yaw_sp), math.cos(yaw_sp), 0.0)
    body_x = cross(y_c, body_z)
    if body_z[2] < 0.0:
        body_x = (-body_x[0], -body_x[1], -body_x[2])
    if abs(body_z[2]) < 0.000001:
        body_x = (0.0, 0.0, 1.0)
    body_x = normalized(body_x)
    body_y = cross(body_z, body_x)
    return q_from_dcm_columns(body_x, body_y, body_z)


def thrust_to_attitude(thr_sp, yaw_sp):
    """ControlMath::thrustToAttitude -> (q_d, thrust_body_z)."""
    q = body_z_to_attitude((-thr_sp[0], -thr_sp[1], -thr_sp[2]), yaw_sp)
    return q, -norm(thr_sp)


# ---------------------------------------------------------------------------
# PositionControl
# ---------------------------------------------------------------------------
class PositionControl:
    def __init__(self):
        self.gain_pos_p = (0.95, 0.95, 1.0)
        self.gain_vel_p = (1.8, 1.8, 4.0)
        self.gain_vel_i = (0.4, 0.4, 2.0)
        self.gain_vel_d = (0.2, 0.2, 0.0)
        self.lim_vel_horizontal = 12.0
        self.lim_vel_up = 3.0
        self.lim_vel_down = 1.5
        self.lim_thr_min = 0.12
        self.lim_thr_max = 1.0
        self.lim_thr_xy_margin = 0.3
        self.lim_tilt = math.radians(45.0)
        self.hover_thrust = 0.5
        self.decouple = True
        self.vel_int = [0.0, 0.0, 0.0]
        # state
        self.pos = (NAN, NAN, NAN)
        self.vel = (NAN, NAN, NAN)
        self.vel_dot = (NAN, NAN, NAN)
        self.yaw = 0.0
        # setpoints
        self.pos_sp = (NAN, NAN, NAN)
        self.vel_sp = (NAN, NAN, NAN)
        self.acc_sp = (NAN, NAN, NAN)
        self.yaw_sp = NAN
        self.yawspeed_sp = NAN
        self.thr_sp = (0.0, 0.0, 0.0)

    def set_params(self, p):
        self.gain_pos_p = (p.f('MPC_XY_P'), p.f('MPC_XY_P'), p.f('MPC_Z_P'))
        self.gain_vel_p = (p.f('MPC_XY_VEL_P_ACC'), p.f('MPC_XY_VEL_P_ACC'), p.f('MPC_Z_VEL_P_ACC'))
        self.gain_vel_i = (p.f('MPC_XY_VEL_I_ACC'), p.f('MPC_XY_VEL_I_ACC'), p.f('MPC_Z_VEL_I_ACC'))
        self.gain_vel_d = (p.f('MPC_XY_VEL_D_ACC'), p.f('MPC_XY_VEL_D_ACC'), p.f('MPC_Z_VEL_D_ACC'))
        self.lim_thr_xy_margin = p.f('MPC_THR_XY_MARG')
        self.decouple = bool(p.i('MPC_ACC_DECOUPLE'))
        self.hover_thrust = p.f('MPC_THR_HOVER')

    def set_velocity_limits(self, horizontal, up, down):
        self.lim_vel_horizontal, self.lim_vel_up, self.lim_vel_down = horizontal, up, down

    def set_thrust_limits(self, tmin, tmax):
        self.lim_thr_min = max(tmin, 10e-4)
        self.lim_thr_max = tmax

    def update_hover_thrust(self, new):
        prev = self.hover_thrust
        self.hover_thrust = constrain(new, 0.05, 0.9)
        acc_z = self.acc_sp[2] if _isf(self.acc_sp[2]) else 0.0
        self.vel_int[2] += (acc_z - G) * prev / self.hover_thrust + G - acc_z

    def set_state(self, pos, vel, vel_dot, yaw):
        self.pos, self.vel, self.vel_dot, self.yaw = pos, vel, vel_dot, yaw

    def set_input_setpoint(self, pos, vel, acc, yaw, yawspeed):
        self.pos_sp, self.vel_sp, self.acc_sp = tuple(pos), tuple(vel), tuple(acc)
        self.yaw_sp, self.yawspeed_sp = yaw, yawspeed

    def reset_integral(self):
        self.vel_int = [0.0, 0.0, 0.0]

    def _input_valid(self):
        for i in range(3):
            if not (_isf(self.pos_sp[i]) or _isf(self.vel_sp[i]) or _isf(self.acc_sp[i])):
                return False
        if _isf(self.pos_sp[0]) != _isf(self.pos_sp[1]):
            return False
        if _isf(self.vel_sp[0]) != _isf(self.vel_sp[1]):
            return False
        if _isf(self.acc_sp[0]) != _isf(self.acc_sp[1]):
            return False
        for i in range(3):
            if _isf(self.pos_sp[i]) and not _isf(self.pos[i]):
                return False
            if _isf(self.vel_sp[i]) and not (_isf(self.vel[i]) and _isf(self.vel_dot[i])):
                return False
        return True

    def update(self, dt):
        valid = self._input_valid()
        if valid:
            self._position_control()
            self._velocity_control(dt)
            self.yawspeed_sp = self.yawspeed_sp if _isf(self.yawspeed_sp) else 0.0
            self.yaw_sp = self.yaw_sp if _isf(self.yaw_sp) else self.yaw
        return valid and all(_isf(v) for v in self.acc_sp) and all(_isf(v) for v in self.thr_sp)

    def _position_control(self):
        vsp_pos = [(self.pos_sp[i] - self.pos[i]) * self.gain_pos_p[i] for i in range(3)]
        vel_sp = list(self.vel_sp)
        for i in range(3):            # addIfNotNan
            if _isf(vsp_pos[i]):
                vel_sp[i] = vsp_pos[i] if not _isf(vel_sp[i]) else vel_sp[i] + vsp_pos[i]
        vsp_pos = [v if _isf(v) else 0.0 for v in vsp_pos]   # setZeroIfNan
        ff = (vel_sp[0] - vsp_pos[0], vel_sp[1] - vsp_pos[1])
        xy = constrain_xy((vsp_pos[0], vsp_pos[1]), ff, self.lim_vel_horizontal)
        vel_sp[0], vel_sp[1] = xy
        vel_sp[2] = constrain(vel_sp[2], -self.lim_vel_up, self.lim_vel_down)
        self.vel_sp = tuple(vel_sp)

    def _velocity_control(self, dt):
        self.vel_int[2] = constrain(self.vel_int[2], -G, G)
        vel_error = [self.vel_sp[i] - self.vel[i] for i in range(3)]
        acc_v = [vel_error[i] * self.gain_vel_p[i] + self.vel_int[i] - self.vel_dot[i] * self.gain_vel_d[i]
                 for i in range(3)]
        acc_sp = list(self.acc_sp)
        for i in range(3):
            if _isf(acc_v[i]):
                acc_sp[i] = acc_v[i] if not _isf(acc_sp[i]) else acc_sp[i] + acc_v[i]
        self.acc_sp = tuple(acc_sp)
        self._acceleration_control()
        thr = list(self.thr_sp)
        if (thr[2] >= -self.lim_thr_min and vel_error[2] >= 0.0) or \
                (thr[2] <= -self.lim_thr_max and vel_error[2] <= 0.0):
            vel_error[2] = 0.0
        thr_xy_norm = math.hypot(thr[0], thr[1])
        thr_max_sq = self.lim_thr_max * self.lim_thr_max
        alloc_h = min(thr_xy_norm, self.lim_thr_xy_margin)
        thr_z_max_sq = thr_max_sq - alloc_h * alloc_h
        thr[2] = max(thr[2], -math.sqrt(max(0.0, thr_z_max_sq)))
        thr_max_xy_sq = thr_max_sq - thr[2] * thr[2]
        thr_max_xy = math.sqrt(thr_max_xy_sq) if thr_max_xy_sq > 0 else 0.0
        if thr_xy_norm > thr_max_xy and thr_xy_norm > 0:
            thr[0] = thr[0] / thr_xy_norm * thr_max_xy
            thr[1] = thr[1] / thr_xy_norm * thr_max_xy
        self.thr_sp = tuple(thr)
        # tracking anti-windup, horizontal
        prod = (thr[0] * (G / self.hover_thrust), thr[1] * (G / self.hover_thrust))
        arw_gain = 2.0 / self.gain_vel_p[0]
        acc_xy = (self.acc_sp[0], self.acc_sp[1])
        if acc_xy[0] ** 2 + acc_xy[1] ** 2 > prod[0] ** 2 + prod[1] ** 2:
            lim = prod
        else:
            lim = acc_xy
        vel_error[0] -= arw_gain * (acc_xy[0] - lim[0])
        vel_error[1] -= arw_gain * (acc_xy[1] - lim[1])
        vel_error = [v if _isf(v) else 0.0 for v in vel_error]
        for i in range(3):
            self.vel_int[i] += vel_error[i] * self.gain_vel_i[i] * dt

    def _acceleration_control(self):
        z_specific_force = -G
        if not self.decouple:
            z_specific_force += self.acc_sp[2]
        body_z = normalized((-self.acc_sp[0], -self.acc_sp[1], -z_specific_force))
        body_z = limit_tilt(body_z, (0.0, 0.0, 1.0), self.lim_tilt)
        thrust_ned_z = self.acc_sp[2] * (self.hover_thrust / G) - self.hover_thrust
        cos_ned_body = body_z[2]
        collective = min(thrust_ned_z / cos_ned_body, -self.lim_thr_min) if cos_ned_body > 1e-6 else -self.lim_thr_min
        self.thr_sp = (body_z[0] * collective, body_z[1] * collective, body_z[2] * collective)

    def attitude_setpoint(self):
        q_d, thrust_z = thrust_to_attitude(self.thr_sp, self.yaw_sp)
        return q_d, thrust_z, self.yawspeed_sp


# ---------------------------------------------------------------------------
# Takeoff
# ---------------------------------------------------------------------------
DISARMED, SPOOLUP, READY_FOR_TAKEOFF, RAMPUP, FLIGHT = range(5)
TAKEOFF_STATE_NAMES = {DISARMED: 'disarmed', SPOOLUP: 'spoolup', READY_FOR_TAKEOFF: 'ready_for_takeoff',
                       RAMPUP: 'rampup', FLIGHT: 'flight'}


class TakeoffHandling:
    def __init__(self):
        self.state = DISARMED
        self.ramp_time = 3.0
        self.ramp_progress = 0.0
        self.ramp_vz_init = -G / 4.0
        self.spoolup_time = 1.0
        self._armed_since = None

    def set_params(self, p):
        self.ramp_time = p.f('MPC_TKO_RAMP_T')
        self.spoolup_time = p.f('COM_SPOOLUP_TIME')
        self.ramp_vz_init = -G / max(p.f('MPC_Z_VEL_P_ACC'), 0.01)

    def update_state(self, armed, landed, want_takeoff, now_s, skip_takeoff=False):
        if armed:
            if self._armed_since is None:
                self._armed_since = now_s
        else:
            self._armed_since = None
        spooled = armed and (now_s - self._armed_since) >= self.spoolup_time
        s = self.state
        if s == DISARMED:
            if armed:
                s = SPOOLUP
        if s == SPOOLUP:
            if spooled:
                s = READY_FOR_TAKEOFF
        if s == READY_FOR_TAKEOFF:
            if want_takeoff:
                s = RAMPUP
                self.ramp_progress = 0.0
        if s == RAMPUP:
            if self.ramp_progress >= 1.0:
                s = FLIGHT
        if s == FLIGHT:
            if landed:
                s = READY_FOR_TAKEOFF
        if armed and skip_takeoff:
            s = FLIGHT
        if not armed:
            s = DISARMED
        self.state = s

    def update_ramp(self, dt, takeoff_desired_vz):
        if self.state < RAMPUP:
            return self.ramp_vz_init
        if self.state == RAMPUP:
            if self.ramp_time > dt:
                self.ramp_progress += dt / self.ramp_time
            else:
                self.ramp_progress = 1.0
            if self.ramp_progress < 1.0:
                return self.ramp_vz_init + self.ramp_progress * (takeoff_desired_vz - self.ramp_vz_init)
        return takeoff_desired_vz


# ---------------------------------------------------------------------------
# AttitudeControl
# ---------------------------------------------------------------------------
class AttitudeControl:
    def __init__(self):
        self.gain = (6.5, 6.5, 2.8 / 0.4)
        self.yaw_w = 0.4
        self.rate_limit = (math.radians(220), math.radians(220), math.radians(200))
        self.q_sp = (1.0, 0.0, 0.0, 0.0)
        self.yawspeed_sp = 0.0

    def set_params(self, p):
        self.yaw_w = constrain(p.f('MC_YAW_WEIGHT'), 0.0, 1.0)
        yaw_p = p.f('MC_YAW_P')
        if self.yaw_w > 1e-4:
            yaw_p /= self.yaw_w
        self.gain = (p.f('MC_ROLL_P'), p.f('MC_PITCH_P'), yaw_p)
        self.rate_limit = (math.radians(p.f('MC_ROLLRATE_MAX')), math.radians(p.f('MC_PITCHRATE_MAX')),
                           math.radians(p.f('MC_YAWRATE_MAX')))

    def set_setpoint(self, q_sp, yawspeed):
        self.q_sp = q_sp
        self.yawspeed_sp = yawspeed

    def update(self, q):
        qd = self.q_sp
        e_z = q_dcm_z(q)
        e_z_d = q_dcm_z(qd)
        qd_red = q_from_two_vectors(e_z, e_z_d)
        if abs(qd_red[1]) > (1.0 - 1e-5) or abs(qd_red[2]) > (1.0 - 1e-5):
            qd_red = qd
        else:
            qd_red = q_mul(qd_red, q)
        q_mix = q_canonical(q_mul(q_conj(qd_red), qd))
        w = constrain(q_mix[0], -1.0, 1.0)
        z = constrain(q_mix[3], -1.0, 1.0)
        qd = q_mul(qd_red, (math.cos(self.yaw_w * math.acos(w)), 0.0, 0.0, math.sin(self.yaw_w * math.asin(z))))
        qe = q_canonical(q_mul(q_conj(q), qd))
        eq = (2.0 * qe[1], 2.0 * qe[2], 2.0 * qe[3])
        rate_sp = [eq[i] * self.gain[i] for i in range(3)]
        if _isf(self.yawspeed_sp):
            zw = q_rotate(q_conj(q), (0.0, 0.0, 1.0))
            for i in range(3):
                rate_sp[i] += zw[i] * self.yawspeed_sp
        return tuple(constrain(rate_sp[i], -self.rate_limit[i], self.rate_limit[i]) for i in range(3))


# ---------------------------------------------------------------------------
# RateControl
# ---------------------------------------------------------------------------
class RateControl:
    def __init__(self):
        self.kp = (0.15, 0.15, 0.2)
        self.ki = (0.2, 0.2, 0.1)
        self.kd = (0.003, 0.003, 0.0)
        self.kff = (0.0, 0.0, 0.0)
        self.lim_int = (0.3, 0.3, 0.3)
        self.rate_int = [0.0, 0.0, 0.0]
        self.sat_pos = [False, False, False]
        self.sat_neg = [False, False, False]

    def set_params(self, p):
        k = (p.f('MC_ROLLRATE_K'), p.f('MC_PITCHRATE_K'), p.f('MC_YAWRATE_K'))
        self.kp = (k[0] * p.f('MC_ROLLRATE_P'), k[1] * p.f('MC_PITCHRATE_P'), k[2] * p.f('MC_YAWRATE_P'))
        self.ki = (k[0] * p.f('MC_ROLLRATE_I'), k[1] * p.f('MC_PITCHRATE_I'), k[2] * p.f('MC_YAWRATE_I'))
        self.kd = (k[0] * p.f('MC_ROLLRATE_D'), k[1] * p.f('MC_PITCHRATE_D'), k[2] * p.f('MC_YAWRATE_D'))
        self.kff = (p.f('MC_ROLLRATE_FF'), p.f('MC_PITCHRATE_FF'), p.f('MC_YAWRATE_FF'))
        self.lim_int = (p.f('MC_RR_INT_LIM'), p.f('MC_PR_INT_LIM'), p.f('MC_YR_INT_LIM'))

    def reset_integral(self):
        self.rate_int = [0.0, 0.0, 0.0]

    def update(self, rate, rate_sp, angular_accel, dt, landed):
        err = [rate_sp[i] - rate[i] for i in range(3)]
        torque = tuple(self.kp[i] * err[i] + self.rate_int[i] - self.kd[i] * angular_accel[i] + self.kff[i] * rate_sp[i]
                       for i in range(3))
        if not landed:
            r400 = math.radians(400.0)
            for i in range(3):
                e = err[i]
                if self.sat_pos[i]:
                    e = min(e, 0.0)
                if self.sat_neg[i]:
                    e = max(e, 0.0)
                f = e / r400
                f = max(0.0, 1.0 - f * f)
                ri = self.rate_int[i] + f * self.ki[i] * e * dt
                if _isf(ri):
                    self.rate_int[i] = constrain(ri, -self.lim_int[i], self.lim_int[i])
        return torque


# ---------------------------------------------------------------------------
# Control allocation
# ---------------------------------------------------------------------------
class ControlAllocator:
    """Rotor effectiveness from CA_ROTOR* + PX4's normalised pseudo-inverse, airmode off."""

    MINIMUM_YAW_MARGIN = 0.15

    def __init__(self, params):
        n = int(params.i('CA_ROTOR_COUNT'))
        self.n = n
        self.rotors = []
        cols = []
        for i in range(n):
            px, py = params.f(f'CA_ROTOR{i}_PX'), params.f(f'CA_ROTOR{i}_PY')
            pz = float(params.get(f'CA_ROTOR{i}_PZ', 0.0))
            ct, km = params.f(f'CA_ROTOR{i}_CT'), params.f(f'CA_ROTOR{i}_KM')
            self.rotors.append((px, py, pz, ct, km))
            a = (0.0, 0.0, -1.0)
            p = (px, py, pz)
            m = cross(p, a)
            moment = (ct * m[0] - ct * km * a[0], ct * m[1] - ct * km * a[1], ct * m[2] - ct * km * a[2])
            force = (ct * a[0], ct * a[1], ct * a[2])
            cols.append(list(moment) + list(force))
        self.B = np.array(cols, dtype=float).T          # 6 x n
        mix = np.linalg.pinv(self.B)                    # n x 6
        scale = np.ones(6)
        roll_s = math.sqrt(float(np.sum(mix[:, 0] ** 2)) / (n / 2.0))
        pitch_s = math.sqrt(float(np.sum(mix[:, 1] ** 2)) / (n / 2.0))
        scale[0] = scale[1] = max(roll_s, pitch_s)
        scale[2] = float(np.max(mix[:, 2]))
        thrust_s = float(np.sum(-mix[:, 5])) / n if n else 1.0
        scale[3] = scale[4] = scale[5] = thrust_s
        for i in range(6):
            if scale[i] > 1e-9:
                mix[:, i] /= scale[i]
        self.mix = mix
        self.scale = scale
        # u = pinv(B) diag(1/scale) c  =>  c_achieved = diag(scale) B u
        self.effectiveness_scaled = self.B * scale[:, None]
        self.mix_list = [list(map(float, mix[i])) for i in range(n)]
        self.unallocated = (0.0, 0.0, 0.0)

    def _gain(self, vec, sp, lo, hi):
        k_min = k_max = 0.0
        for i in range(self.n):
            if abs(vec[i]) < 1e-7:
                continue
            if sp[i] < lo[i]:
                k = (lo[i] - sp[i]) / vec[i]
                k_min, k_max = min(k_min, k), max(k_max, k)
            if sp[i] > hi[i]:
                k = (hi[i] - sp[i]) / vec[i]
                k_min, k_max = min(k_min, k), max(k_max, k)
        return k_min + k_max

    def _desaturate(self, sp, vec, lo, hi, increase_only=False):
        g = self._gain(vec, sp, lo, hi)
        if increase_only and g < 0.0:
            return
        for i in range(self.n):
            sp[i] += g * vec[i]
        g = 0.5 * self._gain(vec, sp, lo, hi)
        if increase_only and g < 0.0:
            return
        for i in range(self.n):
            sp[i] += g * vec[i]

    def allocate(self, torque, thrust_body):
        """Normalised torque (3) + thrust (3, FRD, z<0 up) -> motor commands in [0, 1]."""
        c = (torque[0], torque[1], torque[2], thrust_body[0], thrust_body[1], thrust_body[2])
        mix = self.mix_list
        n = self.n
        lo = [0.0] * n
        hi = [1.0] * n
        sp = [mix[i][0] * c[0] + mix[i][1] * c[1] + mix[i][3] * c[3] + mix[i][4] * c[4] + mix[i][5] * c[5]
              for i in range(n)]
        thrust_z = [mix[i][5] for i in range(n)]
        self._desaturate(sp, thrust_z, lo, hi, increase_only=True)
        self._desaturate(sp, [mix[i][0] for i in range(n)], lo, hi)
        self._desaturate(sp, [mix[i][1] for i in range(n)], lo, hi)
        yaw = [mix[i][2] for i in range(n)]
        for i in range(n):
            sp[i] += yaw[i] * c[2]
        hi_y = [h + (h - l) * self.MINIMUM_YAW_MARGIN for h, l in zip(hi, lo)]
        self._desaturate(sp, yaw, lo, hi_y)
        self._desaturate(sp, thrust_z, lo, hi, increase_only=True)
        out = [constrain(v, 0.0, 1.0) for v in sp]
        # what was actually allocated, for the rate controller's anti-windup
        achieved = self.effectiveness_scaled @ np.array(out)
        self.unallocated = (c[0] - float(achieved[0]), c[1] - float(achieved[1]), c[2] - float(achieved[2]))
        return out

    def saturation_flags(self):
        pos = [u > 1e-3 for u in self.unallocated]
        neg = [u < -1e-3 for u in self.unallocated]
        return pos, neg


# ---------------------------------------------------------------------------
# Hover thrust estimator
# ---------------------------------------------------------------------------
class HoverThrustEstimator:
    """1-state Kalman filter on hover thrust from collective thrust and vertical acceleration.

    Same measurement model as mc_hover_thrust_estimator: a_z = g * (T / T_hover) - g
    (NED, up negative).  Valid once its variance is small and the vehicle flies.
    """

    def __init__(self, initial=0.5):
        self.hover = initial
        self.var = 0.05 ** 2
        self.q = (0.0036) ** 2           # process noise per second
        self.r = 2.0 ** 2                # acceleration noise variance
        self.valid = False
        self._flying_s = 0.0

    def update(self, thrust, acc_z_ned, dt, in_air):
        if not in_air or thrust < 0.05:
            self._flying_s = 0.0
            self.valid = False
            return
        self._flying_s += dt
        self.var += self.q * dt
        # a = g*T/h - g  ->  linearise in h
        pred = G * thrust / self.hover - G
        h_jac = -G * thrust / (self.hover * self.hover)
        innov = (-acc_z_ned) - pred
        s = h_jac * self.var * h_jac + self.r
        k = self.var * h_jac / s
        self.hover = constrain(self.hover + k * innov, 0.1, 0.9)
        self.var = (1.0 - k * h_jac) * self.var
        self.valid = self._flying_s > 1.5 and math.sqrt(self.var) < 0.02
