"""Setpoint generation per nav state: Offboard, the pilot's modes, and the navigator.

* OFFBOARD -- MulticopterPositionControl's offboard branch: the latest
  ``trajectory_setpoint`` as is (NaN fields unused), ``want_takeoff`` when the
  setpoint is less than 1 s old and asks for a higher position or an upward
  velocity/acceleration, speed limits ``MPC_Z_VEL_MAX_UP/DN``.
* POSCTL / ALTCTL / STAB / MANUAL -- the pilot's sticks
  (FlightTaskManualPosition / ManualAltitude, mc_att_control's manual path):
  throttle is a climb-rate command around a held altitude with ``MPC_HOLD_DZ``
  and ``MPC_Z_MAN_EXPO``; descending is limited to ``MPC_LAND_SPEED`` within
  ``MPC_LAND_ALT2`` of the ground, so a pilot who switches with the throttle low
  brings the vehicle down at 0.7 m/s.
* AUTO_LOITER (Hold), AUTO_LAND (with ``MPC_LAND_CRWL`` near the ground),
  AUTO_TAKEOFF (``MIS_TAKEOFF_ALT`` at ``MPC_TKO_SPEED``), DESCEND.
"""
from __future__ import annotations

import math

from . import commander as C

NAN = float('nan')


def _expo(v, e):
    e = max(0.0, min(1.0, e))
    return (1.0 - e) * v + e * v * v * v


def _deadzone(v, dz):
    if abs(v) <= dz:
        return 0.0
    return (v - math.copysign(dz, v)) / (1.0 - dz)


class Setpoint:
    __slots__ = ('kind', 'pos', 'vel', 'acc', 'yaw', 'yawspeed', 'want_takeoff', 'speed_up', 'speed_down',
                 'q', 'thrust', 'motors_off')

    def __init__(self):
        self.kind = 'pos'                 # 'pos' (position controller) | 'att' (attitude direct) | 'off'
        self.pos = (NAN, NAN, NAN)
        self.vel = (NAN, NAN, NAN)
        self.acc = (NAN, NAN, NAN)
        self.yaw = NAN
        self.yawspeed = NAN
        self.want_takeoff = False
        self.speed_up = 3.0
        self.speed_down = 1.5
        self.q = None
        self.thrust = 0.0
        self.motors_off = False


class FlightModes:
    def __init__(self, px4):
        self.px4 = px4
        self.p = px4.params
        self._prev_nav = None
        self._hold_pos = None
        self._hold_yaw = NAN
        self._alt_hold = None
        self._yaw_sp = None
        self._vel_xy = (0.0, 0.0)
        self._land_xy = None
        self._takeoff_target = None

    def _enter(self, nav, lpos):
        self._prev_nav = nav
        self._hold_pos = None
        self._alt_hold = None
        self._land_xy = None
        self._takeoff_target = None
        if lpos:
            self._yaw_sp = lpos['heading']
            v = (lpos['vx'], lpos['vy']) if lpos['v_xy_valid'] else (0.0, 0.0)
            self._vel_xy = v

    def update(self, t, dt, nav, lpos, rc, traj, armed):
        if nav != self._prev_nav:
            self._enter(nav, lpos)
        sp = Setpoint()
        p = self.p
        sp.speed_up = p.f('MPC_Z_VEL_MAX_UP')
        sp.speed_down = p.f('MPC_Z_VEL_MAX_DN')
        if nav == C.TERMINATION:
            sp.kind = 'off'
            sp.motors_off = True
            return sp
        if nav == C.OFFBOARD:
            return self._offboard(t, sp, lpos, traj, armed)
        if nav in (C.MANUAL, C.STAB, C.ACRO):
            return self._stabilized(t, dt, sp, lpos, rc)
        if nav == C.ALTCTL:
            return self._altitude(t, dt, sp, lpos, rc, position=False)
        if nav == C.POSCTL:
            return self._altitude(t, dt, sp, lpos, rc, position=True)
        if nav == C.AUTO_LOITER:
            return self._hold(t, sp, lpos)
        if nav == C.AUTO_LAND:
            return self._land(t, sp, lpos, position=True)
        if nav == C.DESCEND:
            return self._land(t, sp, lpos, position=False)
        if nav == C.AUTO_TAKEOFF:
            return self._takeoff(t, sp, lpos)
        return self._hold(t, sp, lpos)

    # -- offboard -------------------------------------------------------------------------
    def _offboard(self, t, sp, lpos, traj, armed):
        if traj is None:
            if lpos:
                sp.pos = (lpos['x'], lpos['y'], lpos['z'])
            return sp
        sp.pos, sp.vel, sp.acc = traj['position'], traj['velocity'], traj['acceleration']
        sp.yaw, sp.yawspeed = traj['yaw'], traj['yawspeed']
        fresh = armed and (t < traj['t'] + 1.0)
        z = lpos['z'] if lpos else NAN
        if fresh and math.isfinite(sp.pos[2]) and math.isfinite(z) and sp.pos[2] < z:
            sp.want_takeoff = True
        elif fresh and math.isfinite(sp.vel[2]) and sp.vel[2] < 0.0:
            sp.want_takeoff = True
        elif fresh and math.isfinite(sp.acc[2]) and sp.acc[2] < 0.0:
            sp.want_takeoff = True
        return sp

    # -- pilot modes ----------------------------------------------------------------------
    def _throttle_to_vz(self, rc, lpos):
        p = self.p
        stick = _expo(_deadzone(rc.throttle, p.f('MPC_HOLD_DZ')), p.f('MPC_Z_MAN_EXPO'))
        if stick >= 0:
            return -stick * p.f('MPC_Z_VEL_MAX_UP')
        down = p.f('MPC_Z_VEL_MAX_DN')
        if lpos and lpos['dist_bottom_valid'] and math.isfinite(lpos['dist_bottom']):
            alt2, alt1 = p.f('MPC_LAND_ALT2'), p.f('MPC_LAND_ALT1')
            h = lpos['dist_bottom']
            if h <= alt2:
                down = p.f('MPC_LAND_SPEED')
            elif h < alt1:
                down = p.f('MPC_LAND_SPEED') + (h - alt2) / (alt1 - alt2) * (down - p.f('MPC_LAND_SPEED'))
        elif lpos and lpos['z_valid']:
            down = min(down, p.f('MPC_LAND_SPEED')) if -lpos['z'] < p.f('MPC_LAND_ALT2') else down
        return -stick * down

    def _yaw(self, dt, rc, lpos):
        p = self.p
        rate = _expo(_deadzone(rc.yaw, 0.05), p.f('MPC_YAW_EXPO')) * math.radians(p.f('MPC_MAN_Y_MAX'))
        if self._yaw_sp is None:
            self._yaw_sp = lpos['heading'] if lpos else 0.0
        if abs(rate) > 1e-3:
            self._yaw_sp = math.atan2(math.sin(self._yaw_sp + rate * dt), math.cos(self._yaw_sp + rate * dt))
            return NAN, rate
        return self._yaw_sp, 0.0

    def _altitude(self, t, dt, sp, lpos, rc, position):
        p = self.p
        vz = self._throttle_to_vz(rc, lpos)
        z = lpos['z'] if lpos else NAN
        if abs(vz) < 1e-3 and math.isfinite(z):
            if self._alt_hold is None:
                self._alt_hold = z
            pz = self._alt_hold
            vzs = NAN
        else:
            self._alt_hold = None
            pz = NAN
            vzs = vz
        sp.yaw, sp.yawspeed = self._yaw(dt, rc, lpos)
        yaw = lpos['heading'] if lpos else 0.0
        if position:
            # acceleration-based manual position (MPC_POS_MODE 4), simplified to a
            # rate-limited velocity target with a position lock once stopped
            vmax = p.f('MPC_VEL_MANUAL')
            sx = _expo(_deadzone(rc.pitch, 0.05), p.f('MPC_XY_MAN_EXPO'))
            sy = _expo(_deadzone(rc.roll, 0.05), p.f('MPC_XY_MAN_EXPO'))
            tx = (sx * math.cos(yaw) - sy * math.sin(yaw)) * vmax
            ty = (sx * math.sin(yaw) + sy * math.cos(yaw)) * vmax
            amax = p.f('MPC_ACC_HOR') * dt
            vx = self._vel_xy[0] + max(-amax, min(amax, tx - self._vel_xy[0]))
            vy = self._vel_xy[1] + max(-amax, min(amax, ty - self._vel_xy[1]))
            self._vel_xy = (vx, vy)
            if abs(sx) < 1e-3 and abs(sy) < 1e-3 and math.hypot(vx, vy) < 0.05 and lpos:
                if self._hold_pos is None:
                    self._hold_pos = (lpos['x'], lpos['y'])
                sp.pos = (self._hold_pos[0], self._hold_pos[1], pz)
                sp.vel = (NAN, NAN, vzs)
            else:
                self._hold_pos = None
                sp.pos = (NAN, NAN, pz)
                sp.vel = (vx, vy, vzs)
        else:
            tilt = math.tan(math.radians(p.f('MPC_MAN_TILT_MAX'))) * 9.80665
            sx = _deadzone(rc.pitch, 0.05)
            sy = _deadzone(rc.roll, 0.05)
            ax = (sx * math.cos(yaw) - sy * math.sin(yaw)) * tilt
            ay = (sx * math.sin(yaw) + sy * math.cos(yaw)) * tilt
            sp.pos = (NAN, NAN, pz)
            sp.vel = (NAN, NAN, vzs)
            sp.acc = (ax, ay, NAN)
        sp.want_takeoff = rc.throttle > p.f('MPC_HOLD_DZ')
        return sp

    def _stabilized(self, t, dt, sp, lpos, rc):
        p = self.p
        from ..mathutil import q_from_euler
        tilt = math.radians(p.f('MPC_MAN_TILT_MAX'))
        yaw_sp, _rate = self._yaw(dt, rc, lpos)
        if not math.isfinite(yaw_sp):
            yaw_sp = self._yaw_sp
        sp.kind = 'att'
        sp.q = q_from_euler(rc.roll * tilt, -rc.pitch * tilt, yaw_sp)
        thr = (rc.throttle + 1.0) / 2.0
        hover = self.px4.hover_thrust()
        lo, hi = p.f('MPC_MANTHR_MIN'), p.f('MPC_THR_MAX')
        sp.thrust = lo + thr * 2.0 * (hover - lo) if thr <= 0.5 else hover + (thr - 0.5) * 2.0 * (hi - hover)
        return sp

    # -- navigator ------------------------------------------------------------------------
    def _hold(self, t, sp, lpos):
        if lpos is None or not lpos['xy_valid']:
            sp.vel = (0.0, 0.0, 0.0)
            return sp
        if self._hold_pos is None:
            brake = 0.4
            self._hold_pos = (lpos['x'] + lpos['vx'] * brake, lpos['y'] + lpos['vy'] * brake,
                              lpos['z'] + lpos['vz'] * brake)
            self._hold_yaw = lpos['heading']
        sp.pos = self._hold_pos
        sp.yaw = self._hold_yaw
        sp.speed_up = self.p.f('MPC_Z_V_AUTO_UP')
        sp.speed_down = self.p.f('MPC_Z_V_AUTO_DN')
        return sp

    def _land(self, t, sp, lpos, position):
        p = self.p
        speed = p.f('MPC_LAND_SPEED')
        if lpos and lpos['dist_bottom_valid'] and math.isfinite(lpos['dist_bottom']):
            if lpos['dist_bottom'] < p.f('MPC_LAND_ALT3'):
                speed = p.f('MPC_LAND_CRWL')
        if position and lpos and lpos['xy_valid']:
            if self._land_xy is None:
                self._land_xy = (lpos['x'] + lpos['vx'] * 0.4, lpos['y'] + lpos['vy'] * 0.4)
                self._hold_yaw = lpos['heading']
            sp.pos = (self._land_xy[0], self._land_xy[1], NAN)
            sp.vel = (NAN, NAN, speed)
            sp.yaw = self._hold_yaw
        else:
            sp.acc = (0.0, 0.0, NAN)
            sp.vel = (NAN, NAN, speed)
        return sp

    def _takeoff(self, t, sp, lpos):
        p = self.p
        if lpos is None:
            return sp
        if self._takeoff_target is None:
            home = self.px4.commander.home if self.px4.commander.home is not None else lpos['z']
            self._takeoff_target = (lpos['x'], lpos['y'], home - p.f('MIS_TAKEOFF_ALT'))
            self._hold_yaw = lpos['heading']
        sp.pos = self._takeoff_target
        sp.yaw = self._hold_yaw
        sp.speed_up = p.f('MPC_TKO_SPEED')
        sp.want_takeoff = True
        if abs(lpos['z'] - self._takeoff_target[2]) < 0.2:
            self.px4.commander.user_intended = C.AUTO_LOITER
        return sp
