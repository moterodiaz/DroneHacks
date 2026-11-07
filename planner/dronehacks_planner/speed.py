"""How fast to fly along a smooth path, and when to be where.

1. **Speed caps** per sample, the tightest one wins:
   * ``speed``: the speed limit ``v_max``.
   * ``lateral``: ``sqrt(a_lat / |kappa|)``, so curves don't exceed lateral acceleration.
   * ``lateral_jerk``: ``(j_max / |dkappa/ds|)^(1/3)``. Flying a curvature
     ramp at speed v takes lateral jerk ``v^3 |dkappa/ds|``.
   * ``braking``: the drone can always stop, straight ahead, before its props
     reach a wall: ``v^2/(2 a_brake) + v*latency + margin <= free distance``.
     Free distance comes from ray-marching the exact wall distance.
   * ``opening`` (optional): a cap through narrow openings (the window).
2. **Forward-backward pass** on ``u = v^2`` (Pham & Pham's reachability
   analysis, which on a single path coordinate reduces to this). Forward:
   ``u[i+1] <= u[i] + 2*a*ds``. Backward: ``u[i] <= u[i+1] + 2*b*ds``. Here
   ``a`` and ``b`` are the longitudinal limits, further limited by what the
   combined bound ``a_long^2 + a_lat^2 <= a_horizontal^2`` leaves.
3. **Longitudinal jerk.** The pass above is second-order, so acceleration
   switches instantly. Speed *over time* is replaced by its moving average
   over ``T = (a_acc + a_brake) / j_max`` seconds. The averaged acceleration
   is a mean of accelerations already inside their limits, so it stays
   inside them. Its jerk is ``(a(t + T/2) - a(t - T/2)) / T``, so it is at
   most ``(a_acc + a_brake) / T = j_max``. The start and end ramps are
   smoothed the same way. Averaging also lifts speed minima above their caps
   (the apex of a corner), so wherever that happens the cap is lowered by
   the overshoot and everything is redone, until nothing is over its cap.
   This costs about ``T`` overall; an erode-then-average scheme that
   guaranteed the same thing cost ten times that on the published course.
4. **Time**: ``dt = 2 ds / (v_i + v_{i+1})``.

``check_profile`` re-measures everything on the result. It reports any
violation; it never assumes the construction worked.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, Optional

import numpy as np

from .clearance import ObstacleField
from .limits import VehicleLimits
from .route import PlannerConfig
from .smoothing import SmoothPath


class ProfileError(RuntimeError):
    """No speed profile satisfies the limits (e.g. too fast to stop in time)."""


@dataclasses.dataclass(frozen=True)
class ProfileConfig:
    lateral_jerk_limit: bool = True
    lateral_jerk_iterations: int = 12
    braking_room_limit: bool = True
    longitudinal_jerk_smoothing: bool = True
    #: Extra cap through narrow openings: the whole straight pass, including
    #: the run-in. In simulation the turn onto the window's axis is where
    #: tracking error peaks. None: no extra cap.
    opening_speed_mps: Optional[float] = None
    #: Openings at least this wide are not capped (the 1.47 m tunnel does not need it).
    opening_cap_max_width_m: float = 1.0
    #: Baseline mode: one speed everywhere (the lowest cap), with ramps at the ends.
    constant_speed: bool = False
    v_start_mps: float = 0.0
    v_end_mps: float = 0.0


@dataclasses.dataclass
class SpeedProfile:
    s: np.ndarray
    v: np.ndarray
    a_long: np.ndarray          # along the path, m/s^2
    a_lat: np.ndarray           # v^2 * kappa, left positive
    t: np.ndarray
    caps: Dict[str, np.ndarray]  # each cap, m/s
    limit: np.ndarray            # the tightest cap at each sample

    @property
    def duration(self) -> float:
        return float(self.t[-1])


def speed_profile(path: SmoothPath, limits: VehicleLimits, field: ObstacleField,
                  planner: PlannerConfig = PlannerConfig(), cfg: ProfileConfig = ProfileConfig()) -> SpeedProfile:
    s = path.s
    ds = np.diff(s)
    kappa = np.abs(path.curvature)
    caps = _caps(path, limits, field, planner, cfg)
    limit = np.min(np.vstack(list(caps.values())), axis=0)
    if cfg.constant_speed:
        limit = np.full_like(limit, float(limit.min()))
    if cfg.v_start_mps > limit[0] + 1e-9:
        raise ProfileError(f"start speed {cfg.v_start_mps:.2f} m/s exceeds the cap there ({limit[0]:.2f} m/s)")

    # Lateral jerk d(v^2 kappa)/dt = 2 v a kappa + v^3 dkappa/ds. The cap
    # covers the second term; this loop measures both on the result and
    # lowers speed wherever the sum is over, then redoes the profile.
    extra = np.full_like(limit, np.inf)
    for _ in range(cfg.lateral_jerk_iterations if cfg.lateral_jerk_limit else 1):
        lim = np.minimum(limit, extra)
        u = _forward_backward(lim ** 2, ds, kappa, limits, cfg.v_start_mps ** 2, cfg.v_end_mps ** 2)
        v = np.sqrt(np.maximum(u, 0.0))
        if cfg.longitudinal_jerk_smoothing:
            v = _jerk_limit(s, ds, kappa, lim, v, limits, cfg)
        if not cfg.lateral_jerk_limit:
            break
        j_lat = np.abs(np.gradient(v ** 2 * path.curvature, s)) * v
        over = j_lat > limits.j_max_mps3 * 1.01
        if not over.any():
            break
        # Jerk scales roughly with v^3. Widen each fix by a few samples so
        # neighbouring samples don't take turns being over.
        want = np.full_like(v, np.inf)
        want[over] = v[over] * np.cbrt(limits.j_max_mps3 / j_lat[over]) * 0.98
        widened = want.copy()
        for k in range(1, 6):
            widened[k:] = np.minimum(widened[k:], want[:-k])
            widened[:-k] = np.minimum(widened[:-k], want[k:])
        extra = np.minimum(extra, widened)
    limit = np.minimum(limit, extra)
    t = _times(s, ds, v)
    u = v ** 2
    a_int = (u[1:] - u[:-1]) / (2.0 * ds)
    a_long = np.concatenate([[a_int[0]], (a_int[:-1] + a_int[1:]) / 2.0, [a_int[-1]]])
    return SpeedProfile(s=s, v=v, a_long=a_long, a_lat=u * path.curvature, t=t, caps=caps, limit=limit)


def _times(s: np.ndarray, ds: np.ndarray, v: np.ndarray) -> np.ndarray:
    vsum = v[:-1] + v[1:]
    if np.any(vsum <= 1e-9):
        i = int(np.argmax(vsum <= 1e-9))
        raise ProfileError(f"the profile stops dead at s={s[i]:.2f} m (a cap of 0 there)")
    return np.concatenate([[0.0], np.cumsum(2.0 * ds / vsum)])


def _jerk_limit(s: np.ndarray, ds: np.ndarray, kappa: np.ndarray, limit: np.ndarray, v: np.ndarray,
                limits: VehicleLimits, cfg: ProfileConfig, dt: float = 0.01, iterations: int = 30) -> np.ndarray:
    """Moving-average the speed over time, lowering caps the average overshoots (module docstring, step 3)."""
    window = (limits.a_acc_mps2 + limits.a_brake_mps2) / limits.j_max_mps3
    n = max(1, int(round(window / dt)))
    box = np.full(n, 1.0 / n)
    corrected = limit.copy()
    total = float(s[-1])
    smooth_v = v
    for _ in range(iterations):
        t = _times(s, ds, v)
        steps = max(2, int(math.ceil(t[-1] / dt)))
        tg = np.linspace(0.0, t[-1], steps + 1)
        h = tg[1] - tg[0]
        vg = np.interp(tg, t, v)
        padded = np.concatenate([np.full(n - 1, cfg.v_start_mps), vg, np.full(n - 1, cfg.v_end_mps)])
        vs = np.convolve(padded, box, mode="valid")
        travelled = np.concatenate([[0.0], np.cumsum((vs[:-1] + vs[1:]) / 2.0) * h])
        travelled *= total / travelled[-1]
        keep = np.concatenate([[True], np.diff(travelled) > 1e-12])
        smooth_v = np.interp(s, travelled[keep], vs[keep])
        ratio = smooth_v / np.maximum(limit, 1e-9)
        if ratio.max() <= 1.0 + 1e-3:
            return smooth_v
        corrected = np.where(ratio > 1.0, corrected / np.maximum(ratio, 1.0), corrected)
        v = np.sqrt(np.maximum(_forward_backward(corrected ** 2, ds, kappa, limits,
                                                 cfg.v_start_mps ** 2, cfg.v_end_mps ** 2), 0.0))
    # Did not converge: stay under every cap and let check_profile report the jerk.
    return np.minimum(smooth_v, limit)


def _caps(path: SmoothPath, limits: VehicleLimits, field: ObstacleField, planner: PlannerConfig,
          cfg: ProfileConfig) -> Dict[str, np.ndarray]:
    n = len(path.s)
    kappa = np.abs(path.curvature)
    caps = {"speed": np.full(n, limits.v_max_mps)}
    with np.errstate(divide="ignore"):
        caps["lateral"] = np.where(kappa > 1e-9, np.sqrt(limits.a_lat_mps2 / np.maximum(kappa, 1e-12)), np.inf)
        if cfg.lateral_jerk_limit:
            dk = np.abs(path.dcurvature)
            caps["lateral_jerk"] = np.where(dk > 1e-9, np.cbrt(limits.j_max_mps3 / np.maximum(dk, 1e-12)), np.inf)
    if cfg.braking_room_limit:
        free = field.wall_free_distance(path.xy, path.tangent, planner.drone_extent_m,
                                        max_range=limits.stopping_distance(limits.v_max_mps) + 1.0)
        room = np.maximum(free - limits.stop_margin_m, 0.0)
        a, tau = limits.a_brake_mps2, limits.latency_s
        caps["braking"] = a * (-tau + np.sqrt(tau * tau + 2.0 * room / a))
    if cfg.opening_speed_mps is not None:
        narrow = path.pass_width < cfg.opening_cap_max_width_m
        caps["opening"] = np.where(narrow, cfg.opening_speed_mps, np.inf)
    return caps


def _forward_backward(u_cap: np.ndarray, ds: np.ndarray, kappa: np.ndarray, lim: VehicleLimits,
                      u_start: Optional[float], u_end: Optional[float]) -> np.ndarray:
    """Largest u = v^2 under the caps and the acceleration limits. None for a
    boundary speed leaves that end free (only the caps apply)."""
    u = u_cap.astype(float).copy()
    ah2 = lim.a_horizontal_mps2 ** 2
    if u_start is not None:
        u[0] = min(u[0], u_start)
    for i in range(len(u) - 1):
        lat = u[i] * kappa[i]
        acc = min(lim.a_acc_mps2, math.sqrt(max(0.0, ah2 - lat * lat)))
        nxt = u[i] + 2.0 * acc * ds[i]
        if nxt < u[i + 1]:
            u[i + 1] = nxt
    if u_end is not None:
        u[-1] = min(u[-1], u_end)
    for i in range(len(u) - 2, -1, -1):
        lat = u[i + 1] * kappa[i + 1]
        brake = min(lim.a_brake_mps2, math.sqrt(max(0.0, ah2 - lat * lat)))
        prv = u[i + 1] + 2.0 * brake * ds[i]
        if prv < u[i]:
            u[i] = prv
    if u_start is not None and u[0] < u_start - 1e-9:
        raise ProfileError(f"cannot brake from the start speed {math.sqrt(u_start):.2f} m/s in time "
                           f"(at most {math.sqrt(max(u[0], 0.0)):.2f} m/s is stoppable)")
    return u


def acceleration_vectors(path: SmoothPath, prof: SpeedProfile) -> np.ndarray:
    """(N, 2) acceleration: a_long along the tangent plus v^2 kappa along the left normal."""
    normal = np.column_stack([-path.tangent[:, 1], path.tangent[:, 0]])
    return prof.a_long[:, None] * path.tangent + prof.a_lat[:, None] * normal


def check_profile(path: SmoothPath, prof: SpeedProfile, limits: VehicleLimits,
                  rel_tol: float = 0.02, jerk_dt_s: float = 0.1) -> Dict[str, object]:
    """Re-measure a profile. Returns maxima and a list of violated limits.

    Jerk is a finite difference on a uniform ``jerk_dt_s`` time grid. The
    limit ``j_max`` applies to each component (along the path and lateral);
    the jerk of the whole acceleration vector, which can reach the sum of
    the two, is reported for information.
    """
    v, s = prof.v, prof.s
    a_lat = np.abs(prof.a_lat)
    a_tot = np.hypot(prof.a_long, prof.a_lat)
    acc = acceleration_vectors(path, prof)
    tg = np.arange(0.0, prof.duration, jerk_dt_s)
    if len(tg) > 1:
        def rate(series):
            return np.abs(np.diff(np.interp(tg, prof.t, series))) / jerk_dt_s
        j_long = float(rate(prof.a_long).max())
        j_lat = float(rate(prof.a_lat).max())
        ag = np.column_stack([np.interp(tg, prof.t, acc[:, 0]), np.interp(tg, prof.t, acc[:, 1])])
        jerk = float((np.hypot(*np.diff(ag, axis=0).T) / jerk_dt_s).max())
    else:
        j_long = j_lat = jerk = 0.0
    report = {
        "duration_s": prof.duration,
        "length_m": float(s[-1]),
        "max_speed_mps": float(v.max()),
        "mean_speed_mps": float(s[-1] / prof.duration),
        "max_a_long_mps2": float(np.abs(prof.a_long).max()),
        "max_a_lat_mps2": float(a_lat.max()),
        "max_a_total_mps2": float(a_tot.max()),
        "max_jerk_long_mps3": j_long,
        "max_jerk_lat_mps3": j_lat,
        "max_jerk_vector_mps3": jerk,
        "max_curvature": float(np.abs(path.curvature).max()),
        "violations": [],
    }
    tol = 1.0 + rel_tol
    checks = [
        ("speed above its cap", float((v - prof.limit * (1.0 + 1e-3)).max()), 1e-9),
        ("speed above v_max", float(v.max() - limits.v_max_mps), 1e-6),
        ("acceleration above a_acc", float(prof.a_long.max() - limits.a_acc_mps2 * tol), 0.0),
        ("braking above a_brake", float(-prof.a_long.min() - limits.a_brake_mps2 * tol), 0.0),
        ("lateral acceleration above a_lat", float(a_lat.max() - limits.a_lat_mps2 * tol), 0.0),
        ("total acceleration above a_horizontal", float(a_tot.max() - limits.a_horizontal_mps2 * tol), 0.0),
        ("along-path jerk above j_max", j_long - limits.j_max_mps3 * tol, 0.0),
        ("lateral jerk above j_max", j_lat - limits.j_max_mps3 * tol, 0.0),
    ]
    report["violations"] = [f"{name} by {excess:.3g}" for name, excess, slack in checks if excess > slack]
    return report
