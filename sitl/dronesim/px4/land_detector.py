"""PX4 v1.16 multicopter land detector (src/modules/land_detector/MulticopterLandDetector.cpp).

Three stages -- ground contact, maybe landed, landed -- each must hold for a
third of ``LNDMC_TRIG_TIME``; any violated condition clears the later stages
immediately.  A disarmed vehicle is landed.  The thresholds are the PX4 ones:
low thrust is below ``MPC_THR_MIN + 0.3 (hover - MPC_THR_MIN)``, vertical
movement above ``LNDMC_Z_VEL_MAX``, horizontal above ``LNDMC_XY_VEL_MAX``,
rotation above ``LNDMC_ROT_MAX``.
"""
from __future__ import annotations

import math


class _Hysteresis:
    def __init__(self, time_true_s):
        self.time_true = time_true_s
        self.state = True
        self._since = None

    def update(self, cond, t):
        if not cond:
            self.state = False
            self._since = None
            return False
        if self.state:
            return True
        if self._since is None:
            self._since = t
        if t - self._since >= self.time_true:
            self.state = True
        return self.state

    def force(self, value):
        self.state = value
        self._since = None


class LandDetector:
    def __init__(self, params):
        self.p = params
        trig = params.f('LNDMC_TRIG_TIME')
        self.ground_contact_h = _Hysteresis(trig / 3.0)
        self.maybe_landed_h = _Hysteresis(trig / 3.0)
        self.landed_h = _Hysteresis(trig / 3.0)
        self.landed = True
        self.maybe_landed = True
        self.ground_contact = True
        self.freefall = False
        self.in_descend = False
        self.has_low_throttle = True
        self.vertical_movement = False
        self.horizontal_movement = False
        self.rotational_movement = False
        self.at_rest = True
        self._freefall_since = None

    def update(self, t, armed, lpos, thrust, hover_thrust, rates, vz_setpoint, climb_rate_enabled, specific_force_norm):
        p = self.p
        if not armed:
            for h in (self.ground_contact_h, self.maybe_landed_h, self.landed_h):
                h.force(True)
            self.landed = self.maybe_landed = self.ground_contact = True
            self.freefall = False
            self.at_rest = True
            return
        thr_min = p.f('MPC_THR_MIN')
        low = thrust <= thr_min + (hover_thrust - thr_min) * 0.3
        self.has_low_throttle = low
        if lpos and lpos['z_valid']:
            self.vertical_movement = abs(lpos['vz']) > p.f('LNDMC_Z_VEL_MAX')
        else:
            self.vertical_movement = True
        if lpos and lpos['v_xy_valid']:
            self.horizontal_movement = math.hypot(lpos['vx'], lpos['vy']) > p.f('LNDMC_XY_VEL_MAX')
        else:
            self.horizontal_movement = False
        self.rotational_movement = math.sqrt(sum(r * r for r in rates)) > math.radians(p.f('LNDMC_ROT_MAX'))
        self.in_descend = climb_rate_enabled and vz_setpoint is not None and math.isfinite(vz_setpoint) and \
            vz_setpoint >= 1.1 * p.f('LNDMC_Z_VEL_MAX')
        gc_cond = not self.vertical_movement and not self.horizontal_movement and (low or self.in_descend)
        ml_cond = gc_cond and low and not self.rotational_movement
        self.ground_contact = self.ground_contact_h.update(gc_cond, t)
        self.maybe_landed = self.maybe_landed_h.update(ml_cond and self.ground_contact, t)
        self.landed = self.landed_h.update(ml_cond and self.maybe_landed, t)
        # free fall: specific force close to zero for 0.3 s
        if specific_force_norm < 2.0:
            if self._freefall_since is None:
                self._freefall_since = t
            self.freefall = t - self._freefall_since > 0.3
        else:
            self._freefall_since = None
            self.freefall = False
        self.at_rest = self.landed and not self.vertical_movement and not self.rotational_movement
