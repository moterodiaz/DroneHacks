"""The simulated flight controller: PX4 v1.16 modules on their real rates.

    500 Hz  gyro filtering, rate controller, control allocation  (IMU loop)
    250 Hz  attitude controller
    100 Hz  EKF2 + multicopter position controller + takeoff handling
     50 Hz  commander, land detector, battery
     5 Hz   VL53L1X rangefinder (one reading per 200 ms, organizer note in 03a)

Truth comes from :class:`dronesim.vehicle.physics.QuadPhysics` (world NED with
the origin at the course floor centre, x = course north).  PX4 only ever sees
noisy sensors and its own estimate; the position controller runs in the EKF's
local frame, which after vision fusion starts is cuVSLAM's frame.
"""
from __future__ import annotations

import collections
import math

from ..mathutil import (G, LowPass1, LowPass2, constrain, norm, q_from_euler, q_rotate, q_to_euler,
                        q_yaw)
from . import commander as C
from .commander import Commander, RcInput
from .control import (FLIGHT, RAMPUP, AttitudeControl, ControlAllocator, HoverThrustEstimator,
                      PositionControl, RateControl, TakeoffHandling)
from .estimator import Ekf2Model
from .land_detector import LandDetector
from .modes import FlightModes

NAN = float('nan')

# Rangefinder: PX4 VL53L1X driver -- laser, downward, 0..4 m.  Body position from
# EKF2_RNG_POS_X/Y/Z (FRD, metres from the IMU).  On its gear the sensor is ~3 cm off
# the floor; it still reports, and EKF2 replaces on-ground readings below EKF2_MIN_RNG.
RNG_MIN_M = 0.0
RNG_MAX_M = 4.0


class PX4:
    def __init__(self, params, physics, world, rng, log_fn=None, sensor_cfg=None):
        self.params = params
        self.physics = physics
        self.world = world
        self.rng = rng
        self._log_fn = log_fn
        self.cfg = sensor_cfg or {}
        self.t = 0.0
        self.rc = RcInput()
        s = physics.state
        boot_yaw = q_yaw(s.q)
        self.ekf = Ekf2Model(params, rng, boot_yaw, mag_noise_deg=self.cfg.get('mag_heading_noise_deg', 3.0))
        self.commander = Commander(self)
        self.land_detector = LandDetector(params)
        self.modes = FlightModes(self)
        self.pos_ctrl = PositionControl()
        self.pos_ctrl.set_params(params)
        self.att_ctrl = AttitudeControl()
        self.att_ctrl.set_params(params)
        self.rate_ctrl = RateControl()
        self.rate_ctrl.set_params(params)
        self.allocator = ControlAllocator(params)
        self.takeoff = TakeoffHandling()
        self.takeoff.set_params(params)
        self.hte = HoverThrustEstimator(params.f('MPC_THR_HOVER'))
        fs = 500.0
        self.gyro_lpf = LowPass2(fs, params.f('IMU_GYRO_CUTOFF'))
        self.dgyro_lpf = LowPass2(fs, params.f('IMU_DGYRO_CUTOFF'))
        self.yaw_torque_lpf = LowPass1(params.f('MC_YAW_TQ_CUTOFF'))
        self.vel_deriv_lpf = LowPass1(params.f('MPC_VELD_LP'))
        self._prev_gyro = None
        self._prev_vel = None
        self.tilt_limit = math.radians(params.f('MPC_TILTMAX_LND'))
        # interfaces (filled by the uXRCE bridge)
        self.offboard_mode = None
        self.traj_sp = None
        self._commands = collections.deque()
        self.acks = collections.deque()
        self.ev_queue = collections.deque()
        self.ev_status = ''
        # outputs
        self.lpos_out = None
        self.att_q = (1.0, 0.0, 0.0, 0.0)
        self.rates = (0.0, 0.0, 0.0)
        self.rates_sp = (0.0, 0.0, 0.0)
        self.att_sp_q = (1.0, 0.0, 0.0, 0.0)
        self.att_sp_thrust = 0.0
        self.att_sp_yawspeed = 0.0
        self.thrust_body = (0.0, 0.0, 0.0)
        self.torque_sp = (0.0, 0.0, 0.0)
        self.motor_cmds = [0.0] * self.allocator.n
        self.traj_out = None          # what the position controller used (for vehicle_local_position_setpoint)
        self.control_kind = 'pos'
        self.battery_warning = 0
        self.distance = None          # (t, range, valid)
        self.gyro_bias = (rng.gauss(0, 0.002), rng.gauss(0, 0.002), rng.gauss(0, 0.002))
        self.acc_noise = self.cfg.get('accel_noise', 0.06)
        self.gyro_noise = self.cfg.get('gyro_noise', 0.004)
        self.baro_offset = rng.gauss(0.0, 1.0)
        self.baro_drift = 0.0
        self.imu_gyro = (0.0, 0.0, 0.0)
        self.imu_accel = (0.0, 0.0, -G)
        self._imu_sum = [0.0, 0.0, 0.0, 0.0, 0]
        self._fast_count = 0
        self.home_set = False

    # -- helpers --------------------------------------------------------------------------
    def emit_log(self, severity, text):
        if self._log_fn:
            self._log_fn(severity, text)

    def pop_commands(self):
        out = list(self._commands)
        self._commands.clear()
        return out

    def push_command(self, cmd):
        self._commands.append(cmd)

    def ack_command(self, cmd, result):
        self.acks.append((cmd, result))

    def hover_thrust(self):
        if self.params.i('MPC_USE_HTE') and self.hte.valid:
            return self.hte.hover
        return self.params.f('MPC_THR_HOVER')

    def att_roll_pitch(self):
        r, p, _y = q_to_euler(self.physics.state.q)
        return r, p

    @property
    def nav_state(self):
        return self.commander.nav_state

    @property
    def armed(self):
        return self.commander.armed

    # -- 500 Hz ---------------------------------------------------------------------------------
    def step_fast(self, t, dt):
        self.t = t
        s = self.physics.state
        rng = self.rng
        gn = self.gyro_noise
        gyro = (s.omega[0] + self.gyro_bias[0] + rng.gauss(0, gn), s.omega[1] + self.gyro_bias[1] + rng.gauss(0, gn),
                s.omega[2] + self.gyro_bias[2] + rng.gauss(0, gn))
        an = self.acc_noise
        sf = s.specific_force_body
        self.imu_accel = (sf[0] + rng.gauss(0, an), sf[1] + rng.gauss(0, an), sf[2] + rng.gauss(0, an))
        self.imu_gyro = gyro
        # EKF2 consumes integrated delta velocity / delta angle, not point samples
        acc = self._imu_sum
        acc[0] += self.imu_accel[0]
        acc[1] += self.imu_accel[1]
        acc[2] += self.imu_accel[2]
        acc[3] += gyro[2]
        acc[4] += 1
        rates = self.gyro_lpf.apply(gyro)
        if self._prev_gyro is None:
            self._prev_gyro = rates
        dgy = tuple((rates[i] - self._prev_gyro[i]) / dt for i in range(3))
        self._prev_gyro = rates
        ang_acc = self.dgyro_lpf.apply(dgy)
        self.rates = rates
        self._fast_count += 1
        cm = self.commander
        if self.control_kind in ('pos', 'att') and self._fast_count % 2 == 0:
            self.att_ctrl.set_setpoint(self.att_sp_q, self.att_sp_yawspeed)
            self.rates_sp = self.att_ctrl.update(self.att_q)
        landed = self.land_detector.maybe_landed or self.takeoff.state < RAMPUP
        torque = self.rate_ctrl.update(rates, self.rates_sp, ang_acc, dt, landed)
        if self.takeoff.state < RAMPUP and self.control_kind == 'pos':
            self.rate_ctrl.reset_integral()
            torque = (0.0, 0.0, 0.0)
        tz = self.yaw_torque_lpf.update(torque[2], dt)
        torque = (constrain(torque[0], -1, 1), constrain(torque[1], -1, 1), constrain(tz, -1, 1))
        self.torque_sp = torque
        if cm.motors_enabled and self.control_kind != 'off':
            self.motor_cmds = self.allocator.allocate(torque, (0.0, 0.0, self.att_sp_thrust))
            pos, neg = self.allocator.saturation_flags()
            self.rate_ctrl.sat_pos, self.rate_ctrl.sat_neg = pos, neg
        else:
            self.motor_cmds = [0.0] * self.allocator.n
        self.physics.motor_enabled = cm.motors_enabled and self.control_kind != 'off'
        # attitude estimate (tilt from the IMU filter, heading from EKF2)
        roll, pitch, _ = q_to_euler(s.q)
        self.att_q = q_from_euler(roll + self._tilt_err[0], pitch + self._tilt_err[1], self.ekf.yaw)

    _tilt_err = (0.0, 0.0)

    # -- 100 Hz ------------------------------------------------------------------------------------
    def step_100(self, t, dt):
        p = self.params
        s = self.physics.state
        ekf = self.ekf
        ekf.on_ground = self.land_detector.landed
        ekf.in_air = not self.land_detector.landed
        acc = self._imu_sum
        n = max(1, acc[4])
        ekf.predict(t, dt, s.q, (acc[0] / n, acc[1] / n, acc[2] / n), acc[3] / n)
        self._imu_sum = [0.0, 0.0, 0.0, 0.0, 0]
        while self.ev_queue:
            ev = self.ev_queue.popleft()
            self.ev_status = ekf.fuse_ev(t, ev['sample_t'], ev['pos'], ev['yaw'], ev['pos_var'], ev['yaw_var'],
                                         ev['reset_counter'], ev['quality'])
        lpos = ekf.local_position()
        # velocity derivative for the D term (MPC_VELD_LP)
        vel = (lpos['vx'], lpos['vy'], lpos['vz'])
        if self._prev_vel is None:
            self._prev_vel = vel
        acc = tuple((vel[i] - self._prev_vel[i]) / dt for i in range(3))
        self._prev_vel = vel
        vel_dot = self.vel_deriv_lpf.update(acc, dt)
        lpos['vel_dot'] = vel_dot
        lpos['t'] = t
        self.lpos_out = lpos
        cm = self.commander
        if not self.home_set and lpos['z_valid'] and t > 1.0:
            self.home_set = True
            cm.home = lpos['z']
        sp = self.modes.update(t, dt, cm.nav_state, lpos, self.rc, self.traj_sp, cm.armed)
        self.control_kind = sp.kind
        self.traj_out = sp
        if sp.kind == 'off' or not cm.armed:
            self.takeoff.update_state(cm.armed, self.land_detector.landed, False, t)
            self.pos_ctrl.reset_integral()
            self.att_sp_thrust = 0.0
            self.att_sp_q = self.att_q
            self.hte.update(0.0, 0.0, dt, False)
            return
        if sp.kind == 'att':
            self.takeoff.update_state(cm.armed, self.land_detector.landed, sp.thrust > 0.15, t, skip_takeoff=True)
            self.att_sp_q = sp.q
            self.att_sp_thrust = -sp.thrust
            self.att_sp_yawspeed = 0.0
            return
        # position controller (mc_pos_control)
        self.takeoff.update_state(cm.armed, self.land_detector.landed, sp.want_takeoff, t)
        speed_up = self.takeoff.update_ramp(dt, min(sp.speed_up, p.f('MPC_Z_VEL_MAX_UP')))
        flying = self.takeoff.state >= FLIGHT
        not_taken_off = self.takeoff.state < RAMPUP
        target_tilt = math.radians(p.f('MPC_TILTMAX_AIR') if flying else p.f('MPC_TILTMAX_LND'))
        step = 0.2 * dt if target_tilt > self.tilt_limit else 10.0
        self.tilt_limit += constrain(target_tilt - self.tilt_limit, -step, step)
        self.pos_ctrl.lim_tilt = self.tilt_limit
        self.pos_ctrl.set_thrust_limits(p.f('MPC_THR_MIN') if flying else 0.0, p.f('MPC_THR_MAX'))
        self.pos_ctrl.set_velocity_limits(p.f('MPC_XY_VEL_MAX'), min(speed_up, p.f('MPC_Z_VEL_MAX_UP')),
                                          max(sp.speed_down, 0.0))
        if p.i('MPC_USE_HTE') and self.hte.valid:
            self.pos_ctrl.update_hover_thrust(self.hte.hover)
        acc_sp = sp.acc
        if self.takeoff.state == RAMPUP and math.isfinite(sp.vel[2]):
            acc_sp = (acc_sp[0], acc_sp[1], NAN)
        if not_taken_off:
            self.pos_ctrl.reset_integral()
            self.att_sp_thrust = 0.0
            self.att_sp_q = q_from_euler(0.0, 0.0, lpos['heading'])
            self.att_sp_yawspeed = 0.0
            self.hte.update(0.0, 0.0, dt, False)
            return
        pos = (lpos['x'], lpos['y'], lpos['z']) if lpos['xy_valid'] else (NAN, NAN, lpos['z'])
        vel_s = (lpos['vx'], lpos['vy'], lpos['vz']) if lpos['v_xy_valid'] else (NAN, NAN, lpos['vz'])
        if not lpos['z_valid']:
            pos = (pos[0], pos[1], NAN)
            vel_s = (vel_s[0], vel_s[1], NAN)
        self.pos_ctrl.set_state(pos, vel_s, vel_dot, lpos['heading'])
        self.pos_ctrl.set_input_setpoint(sp.pos, sp.vel, acc_sp, sp.yaw, sp.yawspeed)
        ok = self.pos_ctrl.update(dt)
        if not ok:
            # PositionControl::generateFailsafeSetpoint: level, gentle descent
            fs_vel = (0.0, 0.0, 0.3) if lpos['v_xy_valid'] else (NAN, NAN, 0.3 if lpos['z_valid'] else NAN)
            fs_acc = (NAN, NAN, NAN) if lpos['v_xy_valid'] else (0.0, 0.0, NAN if lpos['z_valid'] else 0.0)
            self.pos_ctrl.set_input_setpoint((NAN, NAN, NAN), fs_vel, fs_acc, NAN, 0.0)
            if not self.pos_ctrl.update(dt):
                self.att_sp_q = q_from_euler(0.0, 0.0, lpos['heading'])
                self.att_sp_thrust = -0.9 * self.hover_thrust()
                return
        q_d, thrust_z, yawspeed = self.pos_ctrl.attitude_setpoint()
        self.att_sp_q, self.att_sp_thrust, self.att_sp_yawspeed = q_d, thrust_z, yawspeed
        # hover thrust estimator: collective thrust projected on the vertical
        tilt_cos = math.cos(q_to_euler(s.q)[0]) * math.cos(q_to_euler(s.q)[1])
        self.hte.update(-thrust_z * tilt_cos, lpos['az'] if math.isfinite(lpos['az']) else 0.0, dt,
                        flying and not self.land_detector.landed)

    # -- 50 Hz ------------------------------------------------------------------------------------
    def step_50(self, t, dt):
        cm = self.commander
        lpos = self.lpos_out
        thrust = -self.att_sp_thrust
        vz_sp = None
        sp = self.traj_out
        if sp is not None and sp.kind == 'pos':
            vz_sp = self.pos_ctrl.vel_sp[2]
        climb = sp is not None and sp.kind == 'pos'
        self.land_detector.update(t, cm.armed, lpos, thrust, self.hover_thrust(), self.rates, vz_sp, climb,
                                  norm(self.imu_accel))
        cm.update(t, dt)
        soc = self.physics.battery.soc
        p = self.params
        self.battery_warning = 0 if soc > p.f('BAT_LOW_THR') else (1 if soc > p.f('BAT_CRIT_THR') else
                                                                     (2 if soc > p.f('BAT_EMERGEN_THR') else 3))

    def step_baro(self, t, dt):
        s = self.physics.state
        self.baro_drift += self.rng.gauss(0.0, 0.01 * math.sqrt(dt))
        z = s.pos[2] + self.baro_offset + self.baro_drift + self.rng.gauss(0.0, 0.06)
        self.ekf.fuse_baro(t, z)

    def step_range(self, t):
        """VL53L1X sample: one ray down the body z axis from the sensor."""
        s = self.physics.state
        p = self.params
        off = (float(p.get('EKF2_RNG_POS_X', 0.0)), float(p.get('EKF2_RNG_POS_Y', 0.0)), p.f('EKF2_RNG_POS_Z'))
        origin = tuple(s.pos[i] + q_rotate(s.q, off)[i] for i in range(3))
        direction = q_rotate(s.q, (0.0, 0.0, 1.0))
        d = self.world.raycast_ned(origin, direction, RNG_MAX_M + 1.0) if self.world is not None else math.inf
        max_r = self.cfg.get('range_max_m', RNG_MAX_M)
        valid = math.isfinite(d) and RNG_MIN_M <= d <= max_r and direction[2] > 0.7
        if valid:
            d = d + self.rng.gauss(0.0, self.cfg.get('range_noise_m', 0.005) + 0.01 * d)
        self.distance = (t, d if valid else NAN, valid)
        self.ekf.fuse_range(t, d if valid else 0.0, valid, direction[2], self.land_detector.landed)
