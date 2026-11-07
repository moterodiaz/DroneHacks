"""A small 2D stand-in for PX4 under ``skybeta_flight_interface``, for comparing
ways of following a route offline. It is a model, not the drone: every
number it produces needs a flight to confirm it.

The chain, per tick:

* Our node (``follower_rate_hz``) computes a pose from odometry (position
  and velocity) delayed by ``odom_latency_s``. The organisers measured a
  68.7 ms median vision latency.
* The flight interface forwards the latest pose to PX4 at
  ``interface_rate_hz``: 10 Hz, its ``setpoint_rate_hz`` default.
* PX4's position controller (``px4_rate_hz``) uses the parameter values in
  ``limits.py``:
  ``vel_sp = MPC_XY_P * (pose - p)``, limited to ``MPC_XY_VEL_MAX``;
  ``acc_sp = MPC_XY_VEL_P_ACC * e + I - MPC_XY_VEL_D_ACC * dv/dt``, where
  ``e = vel_sp - v`` and ``I`` integrates ``MPC_XY_VEL_I_ACC * e`` (PX4
  differentiates the measured velocity, not the error);
  then limited to ``g * tan(MPC_TILTMAX_AIR)``.
  It uses the true position (EKF2 is assumed exact).
* The airframe reaches the commanded acceleration as a first-order lag of
  ``accel_lag_s``. **That lag is guessed, not measured.**

Not modelled: height, yaw dynamics, wind or prop wash near walls, vision
dropouts, EKF2 error, battery sag.
"""

from __future__ import annotations

import collections
import dataclasses
from typing import List, Optional, Sequence

import numpy as np

from .limits import VehicleLimits, tilt_accel_limit


@dataclasses.dataclass(frozen=True)
class SimConfig:
    dt_s: float = 0.002
    px4_rate_hz: float = 100.0
    interface_rate_hz: float = 10.0
    follower_rate_hz: float = 50.0
    odom_latency_s: float = 0.0687
    #: GUESSED: how fast the airframe reaches a commanded acceleration.
    accel_lag_s: float = 0.10
    ki_vel: float = 0.4   # MPC_XY_VEL_I_ACC
    kd_vel: float = 0.2   # MPC_XY_VEL_D_ACC
    record_rate_hz: float = 50.0
    timeout_s: float = 120.0


@dataclasses.dataclass
class SimResult:
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    setpoint: np.ndarray       # pose PX4 was tracking at each record
    reference: np.ndarray      # the follower's reference at each record
    lag: np.ndarray
    cross_track: np.ndarray
    lead_scale: np.ndarray
    clock_rate: np.ndarray
    finished: bool
    time_s: float              # time to finish (or the timeout)


def simulate(follower, start: Sequence[float], limits: VehicleLimits = VehicleLimits(),
             cfg: SimConfig = SimConfig()) -> SimResult:
    """Run ``follower`` (anything with ``update(now, est_pos) -> Setpoint``)
    from rest at ``start`` until it reports done or ``timeout_s`` passes."""
    p = np.asarray(start, dtype=float).copy()
    v = np.zeros(2)
    a = np.zeros(2)
    integ = np.zeros(2)
    a_max = tilt_accel_limit(limits)

    every = lambda hz: max(1, int(round(1.0 / (hz * cfg.dt_s))))  # noqa: E731
    n_follow, n_iface, n_px4, n_rec = (every(cfg.follower_rate_hz), every(cfg.interface_rate_hz),
                                       every(cfg.px4_rate_hz), every(cfg.record_rate_hz))
    history = collections.deque(maxlen=int(cfg.odom_latency_s / (n_follow * cfg.dt_s)) + 3)
    published = p.copy()     # our node's latest pose
    forwarded = p.copy()     # what the interface last sent PX4
    acc_sp = np.zeros(2)
    last_sp = None
    rec = {k: [] for k in ("t", "pos", "vel", "acc", "setpoint", "reference", "lag", "cross", "scale", "rate")}
    finished = False
    steps = int(cfg.timeout_s / cfg.dt_s)
    k = 0
    for k in range(steps + 1):
        now = k * cfg.dt_s
        if k % n_follow == 0:
            history.append((now, p.copy(), v.copy()))
            est_p, est_v = _delayed(history, now - cfg.odom_latency_s)
            last_sp = follower.update(now, est_p, est_v)
            published = np.asarray(last_sp.position, dtype=float)
            if last_sp.done:
                finished = True
        if k % n_iface == 0:
            forwarded = published.copy()
        if k % n_px4 == 0:
            dt_c = n_px4 * cfg.dt_s
            vel_sp = limits.kp_pos * (forwarded - p)
            speed = float(np.hypot(*vel_sp))
            if speed > limits.v_max_mps:
                vel_sp *= limits.v_max_mps / speed
            err = vel_sp - v
            integ = np.clip(integ + cfg.ki_vel * err * dt_c, -a_max, a_max)
            acc_sp = limits.kp_vel * err + integ - cfg.kd_vel * a
            mag = float(np.hypot(*acc_sp))
            if mag > a_max:
                acc_sp *= a_max / mag
        if k % n_rec == 0 and last_sp is not None:
            for key, val in (("t", now), ("pos", p.copy()), ("vel", v.copy()), ("acc", a.copy()),
                             ("setpoint", forwarded.copy()), ("reference", np.asarray(last_sp.reference, dtype=float)),
                             ("lag", last_sp.lag_m), ("cross", last_sp.cross_track_m),
                             ("scale", last_sp.lead_scale), ("rate", last_sp.clock_rate)):
                rec[key].append(val)
        if finished:
            break
        a = a + (acc_sp - a) * (cfg.dt_s / cfg.accel_lag_s)
        v = v + a * cfg.dt_s
        p = p + v * cfg.dt_s
    arr = {key: np.asarray(val) for key, val in rec.items()}
    return SimResult(t=arr["t"], pos=arr["pos"], vel=arr["vel"], acc=arr["acc"], setpoint=arr["setpoint"],
                     reference=arr["reference"], lag=arr["lag"], cross_track=arr["cross"],
                     lead_scale=arr["scale"], clock_rate=arr["rate"], finished=finished,
                     time_s=k * cfg.dt_s)


def _delayed(history, when: float):
    """Position and velocity at time ``when``, interpolated from the recent history."""
    items: List = list(history)
    if when <= items[0][0]:
        return items[0][1], items[0][2]
    for (t0, p0, v0), (t1, p1, v1) in zip(items, items[1:]):
        if t0 <= when <= t1:
            f = 0.0 if t1 == t0 else (when - t0) / (t1 - t0)
            return p0 * (1 - f) + p1 * f, v0 * (1 - f) + v1 * f
    return items[-1][1], items[-1][2]


def flown_metrics(res: SimResult, field, drone_extent_m: float, path_xy: Optional[np.ndarray] = None) -> dict:
    """What happened in a simulated flight: wall touches (prop tips inside a
    wall), closest approaches, speeds, and cross-track error against
    ``path_xy`` if given (exact nearest-sample distance, not the follower's
    delayed estimate)."""
    pos = res.pos
    wall = field.wall_distance(pos)
    _, margin = field.query(pos)
    touching = wall < drone_extent_m
    touches = int(np.sum(touching[1:] & ~touching[:-1]) + (1 if touching[0] else 0))
    speed = np.hypot(*res.vel.T)
    out = {
        "finished": res.finished,
        "time_s": res.time_s,
        "flown_length_m": float(np.sum(np.hypot(*np.diff(pos, axis=0).T))),
        "max_speed_mps": float(speed.max()),
        "mean_speed_mps": float(np.sum(np.hypot(*np.diff(pos, axis=0).T)) / max(res.time_s, 1e-9)),
        "max_accel_mps2": float(np.hypot(*res.acc.T).max()),
        "min_wall_clearance_m": float(wall.min()),
        "min_margin_m": float(margin.min()),
        "wall_touches": touches,
    }
    if path_xy is not None:
        out["max_cross_track_m"] = float(_nearest_distance(pos, path_xy).max())
    return out


def _nearest_distance(pts: np.ndarray, path_xy: np.ndarray, chunk: int = 512) -> np.ndarray:
    out = np.empty(len(pts))
    for lo in range(0, len(pts), chunk):
        d = pts[lo:lo + chunk, None, :] - path_xy[None, :, :]
        out[lo:lo + chunk] = np.sqrt(np.min(np.einsum("ijk,ijk->ij", d, d), axis=1))
    return out


def openings_crossed(pos: np.ndarray, route, cmap, drone_extent_m: float) -> dict:
    """For each opening the route passes: did the simulated drone go through
    it (centre inside the opening, minus the drone's half-span, at the
    plane) and then get its whole body past the far face (rule 7.6)?"""
    goals = {g.name: g for g in cmap.goals}
    out = {}
    for name, p in route.passes.items():
        g = goals[name]
        axis = np.asarray(g.axis, dtype=float)
        if float(np.dot(axis, p.direction)) < 0:
            axis = -axis
        perp = np.array([-axis[1], axis[0]])
        rel = pos - np.asarray(g.centre, dtype=float)
        along, lateral = rel @ axis, np.abs(rel @ perp)
        ok = False
        for i in np.flatnonzero((along[:-1] < 0) & (along[1:] >= 0)):
            if lateral[i] <= float(g.params["width"]) / 2.0 - drone_extent_m and \
                    along[i:].max() >= g.depth / 2.0 + drone_extent_m:
                ok = True
                break
        out[name] = ok
    return out


def landed_on(pos_end: np.ndarray, cmap, pad_name: str, drone_extent_m: float) -> bool:
    """Whole drone inside the pad's outer ring at the end (rule: checkpoint 5)."""
    g = next(g for g in cmap.goals if g.name == pad_name)
    return float(np.hypot(*(pos_end - np.asarray(g.centre)))) + drone_extent_m <= float(g.params["radius"])
