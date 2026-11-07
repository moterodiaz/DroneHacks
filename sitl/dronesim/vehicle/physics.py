"""Six-degree-of-freedom quadrotor model (world NED, body FRD, origin at the IMU / CoG).

Rotor positions come from the drone's own ``CA_ROTOR*_PX/PY`` parameters (the
organizers measured them on the fleet airframe), so the control allocator and
the physics agree on geometry, as they do on the real vehicle.  Everything else
that PX4 does *not* know is a physical property here and lives in
``sitl/config/vehicle.yaml``: mass, inertia, the real motor thrust curve and
time constants, rotor drag, ground effect and the battery.

PX4 assumes thrust is linear in the motor command (``THR_MDL_FAC`` = 0); real
propellers are closer to quadratic, so the model uses
``T = T_max * ((1 - k) u + k u^2)``.  That mismatch, the motor lag and the
estimated hover thrust are exactly the things a real tuning has to absorb.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..mathutil import G, add, cross, q_conj, q_integrate, q_rotate, scale


@dataclass
class AirframeConfig:
    mass_kg: float = 1.35
    inertia: tuple = (0.0125, 0.0140, 0.0230)        # Ixx, Iyy, Izz  kg m^2 (FRD)
    motor_t_max_n: float = 8.0                       # per motor at full command, nominal voltage
    motor_curve_k: float = 0.35                      # 0 = linear, 1 = quadratic in the command
    motor_tau_up_s: float = 0.025
    motor_tau_down_s: float = 0.045
    motor_idle_cmd: float = 0.05                     # armed idle ("spin when armed")
    yaw_moment_m: float = 0.022                      # rotor reaction torque per newton of thrust
    drag_linear: tuple = (0.28, 0.28, 0.45)          # N/(m/s), body FRD (rotor drag)
    drag_quadratic: float = 0.035                    # N/(m/s)^2
    rot_damping: float = 0.004                       # N m /(rad/s)
    prop_radius_m: float = 0.0762                    # 6 in props
    rotor_z_m: float = -0.035                        # rotor plane above the IMU (FRD z up = negative)
    rest_height_m: float = 0.07                      # IMU height above the floor on its gear
    gear_points: tuple = ((0.075, 0.105), (0.075, -0.105), (-0.13, 0.095), (-0.13, -0.095))
    gear_stiffness: float = 2200.0                   # N/m per foot
    gear_damping: float = 45.0                       # N s/m per foot
    ground_friction: float = 0.7
    camera_xyz_flu: tuple = (0.1053, 0.0, 0.0304)    # from drone.env CAMERA_X/Y/Z
    battery_cells: int = 4
    battery_capacity_mah: float = 2200.0
    battery_r_internal: float = 0.035                # ohm per pack
    ground_effect: bool = True
    rotors: list = field(default_factory=list)       # [(px, py, km_sign)] FRD, filled from params

    @classmethod
    def from_params(cls, params, overrides=None):
        cfg = cls(**(overrides or {}))
        rotors = []
        for i in range(params.i('CA_ROTOR_COUNT')):
            km = params.f(f'CA_ROTOR{i}_KM')
            rotors.append((params.f(f'CA_ROTOR{i}_PX'), params.f(f'CA_ROTOR{i}_PY'), 1.0 if km >= 0 else -1.0))
        cfg.rotors = rotors
        return cfg

    def footprint_points(self, n_per_prop=12):
        """Outline of the drone in body FRD for collision / scoring: prop tips and body."""
        pts = []
        for px, py, _ in self.rotors:
            for k in range(n_per_prop):
                a = 2 * math.pi * k / n_per_prop
                pts.append((px + self.prop_radius_m * math.cos(a), py + self.prop_radius_m * math.sin(a),
                            self.rotor_z_m))
        return pts


class Battery:
    def __init__(self, cfg, soc=0.95):
        self.cells = cfg.battery_cells
        self.capacity_c = cfg.battery_capacity_mah * 3.6       # coulombs
        self.r = cfg.battery_r_internal
        self.soc = soc
        self.current = 0.0

    def ocv(self):
        # LiPo open-circuit voltage curve per cell (approximate)
        s = max(0.0, min(1.0, self.soc))
        v = 3.27 + 0.6 * s + 0.35 * s ** 0.5 - 0.25 * (1 - s) ** 6
        return self.cells * v

    def voltage(self):
        return self.ocv() - self.current * self.r

    def step(self, power_w, dt):
        v = max(10.0, self.ocv())
        self.current = power_w / v + 0.6                        # + avionics (Jetson, camera)
        self.soc = max(0.0, self.soc - self.current * dt / self.capacity_c)


class VehicleState:
    __slots__ = ('pos', 'vel', 'q', 'omega', 'acc', 'motors', 'thrusts', 'specific_force_body')

    def __init__(self, pos, q):
        self.pos = pos
        self.vel = (0.0, 0.0, 0.0)
        self.q = q
        self.omega = (0.0, 0.0, 0.0)
        self.acc = (0.0, 0.0, 0.0)
        self.motors = [0.0, 0.0, 0.0, 0.0]
        self.thrusts = [0.0, 0.0, 0.0, 0.0]
        self.specific_force_body = (0.0, 0.0, -G)


class QuadPhysics:
    """Rigid-body integration plus motors; contacts come from a callback."""

    def __init__(self, cfg, pos, q, contact_model=None):
        self.cfg = cfg
        self.state = VehicleState(pos, q)
        self.contact_model = contact_model
        self.battery = Battery(cfg)
        self.motor_enabled = True             # False after a kill / lockdown / disarm
        self.last_contact = None
        n = len(cfg.rotors)
        self.state.motors = [0.0] * n
        self.state.thrusts = [0.0] * n
        self._v_nominal = cfg.battery_cells * 3.9

    def step(self, dt, motor_cmds, armed):
        cfg, s = self.cfg, self.state
        # -- motors -------------------------------------------------------------
        total_thrust = 0.0
        tx = ty = tz = 0.0
        vbat = self.battery.voltage()
        vscale = (vbat / self._v_nominal) ** 2
        height = None
        if cfg.ground_effect and self.contact_model is not None:
            height = self.contact_model.height_above_support(s.pos)
        power = 0.0
        for i, (px, py, km_sign) in enumerate(cfg.rotors):
            if not (armed and self.motor_enabled):
                target = 0.0
            else:
                target = max(cfg.motor_idle_cmd, min(1.0, motor_cmds[i]))
            u = s.motors[i]
            tau = cfg.motor_tau_up_s if target > u else cfg.motor_tau_down_s
            u += (target - u) * min(1.0, dt / tau)
            s.motors[i] = u
            thrust = cfg.motor_t_max_n * vscale * ((1.0 - cfg.motor_curve_k) * u + cfg.motor_curve_k * u * u)
            if height is not None and height > 0.02:
                ratio = cfg.prop_radius_m / (4.0 * max(height, cfg.prop_radius_m * 0.6))
                thrust /= max(0.6, 1.0 - ratio * ratio)
            s.thrusts[i] = thrust
            total_thrust += thrust
            tx += -py * thrust
            ty += px * thrust
            tz += km_sign * cfg.yaw_moment_m * thrust
            power += 11.0 * thrust ** 1.5            # ~ momentum-theory electrical power, W
        self.battery.step(power, dt)
        # -- forces ---------------------------------------------------------------
        q = s.q
        f_body = (0.0, 0.0, -total_thrust)
        v_body = q_rotate(q_conj(q), s.vel)
        speed = math.sqrt(v_body[0] ** 2 + v_body[1] ** 2 + v_body[2] ** 2)
        drag_body = tuple(-cfg.drag_linear[i] * v_body[i] - cfg.drag_quadratic * speed * v_body[i] for i in range(3))
        f_body = add(f_body, drag_body)
        f_world = q_rotate(q, f_body)
        m = cfg.mass_kg
        f_world = (f_world[0], f_world[1], f_world[2] + m * G)
        torque = (tx - cfg.rot_damping * s.omega[0], ty - cfg.rot_damping * s.omega[1],
                  tz - cfg.rot_damping * s.omega[2])
        if self.contact_model is not None:
            cf, ct, info = self.contact_model.forces(s, dt)
            f_world = add(f_world, cf)
            torque = add(torque, ct)
            self.last_contact = info
        # -- integrate --------------------------------------------------------------
        acc = scale(f_world, 1.0 / m)
        s.acc = acc
        vel = add(s.vel, scale(acc, dt))
        s.pos = add(s.pos, scale(vel, dt))
        s.vel = vel
        ixx, iyy, izz = cfg.inertia
        w = s.omega
        iw = (ixx * w[0], iyy * w[1], izz * w[2])
        gyro = cross(w, iw)
        dw = ((torque[0] - gyro[0]) / ixx, (torque[1] - gyro[1]) / iyy, (torque[2] - gyro[2]) / izz)
        w_new = (w[0] + dw[0] * dt, w[1] + dw[1] * dt, w[2] + dw[2] * dt)
        s.omega = w_new
        s.q = q_integrate(q, w_new, dt)
        # specific force measured by the IMU (body): R^T (a - g)
        sf_world = (acc[0], acc[1], acc[2] - G)
        s.specific_force_body = q_rotate(q_conj(s.q), sf_world)
        return s
