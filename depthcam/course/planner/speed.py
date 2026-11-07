"""Time-optimal speed along a fixed path for a point mass, and sampling it into setpoints.

Model: the drone's total acceleration (along the path plus sideways ``v^2 * curvature``) may not
exceed ``a_max``, and its speed may not exceed a per-point cap (``v_max``, or less near the ground
when landing).  The fastest speed profile under those limits is the classic forward/backward pass:

1. cap the speed at every point by ``sqrt(a_max / curvature)`` (all the acceleration spent turning);
2. forward: from the start speed, accelerate as hard as the leftover acceleration allows;
3. backward: from the end speed, do the same in reverse (this is braking);
4. the profile is the smaller of the two at every point.

This is the same answer TOPP-RA gives for these constraints, in a few lines of numpy.  What it
ignores: jerk limits and the flight controller's tracking lag, so treat ``a_max`` as what the
drone can *track*, not what its motors can produce, and set it from flight logs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

MAX_CURVATURE = 50.0  # 1/m: a 2 cm radius; caps the speed limit at kinks instead of dividing by zero


def signed_curvature(points: np.ndarray) -> np.ndarray:
    """Menger curvature at each interior point of a 2D polyline (positive = turning left)."""
    k = np.zeros(len(points))
    if len(points) < 3:
        return k
    a, b, c = points[:-2], points[1:-1], points[2:]
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    denom = np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1) * np.linalg.norm(ac, axis=1)
    k[1:-1] = np.where(denom > 1e-12, 2.0 * cross / np.maximum(denom, 1e-12), 0.0)
    return np.clip(k, -MAX_CURVATURE, MAX_CURVATURE)


def speed_profile(points: np.ndarray, a_max: float, v_cap: np.ndarray, curvature: Optional[np.ndarray] = None,
                  v_start: float = 0.0, v_end: float = 0.0) -> np.ndarray:
    """Fastest speed at every point of a polyline (2D or 3D) under the limits above."""
    n = len(points)
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    kappa = np.abs(curvature) if curvature is not None else np.zeros(n)
    v_lim = np.minimum(v_cap, np.sqrt(a_max / np.maximum(kappa, 1e-9)))

    def leftover(v: float, k: float) -> float:
        lateral = v * v * k
        return float(np.sqrt(max(a_max * a_max - lateral * lateral, 0.0)))

    v = v_lim.copy()
    v[0] = min(v[0], v_start)
    for i in range(n - 1):
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2.0 * leftover(v[i], kappa[i]) * ds[i]))
    v[-1] = min(v[-1], v_end)
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2.0 * leftover(v[i + 1], kappa[i + 1]) * ds[i]))
    return v


def time_stamps(points: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Time at each point, assuming constant acceleration between points."""
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    mean_v = np.maximum((v[:-1] + v[1:]) / 2.0, 1e-6)
    return np.concatenate([[0.0], np.cumsum(ds / mean_v)])


@dataclass
class Samples:
    t: np.ndarray    # (M,)
    pos: np.ndarray  # (M, 3)
    vel: np.ndarray  # (M, 3)
    acc: np.ndarray  # (M, 3)


def sample(points: np.ndarray, v: np.ndarray, t: np.ndarray, dt: float, curvature: Optional[np.ndarray] = None,
           t0: float = 0.0) -> Samples:
    """Setpoints every ``dt`` along a timed polyline (3D points): position, velocity, acceleration.

    Within a segment the speed changes at the constant rate that joins its end speeds; the
    direction is the segment's, and sideways acceleration ``v^2 * curvature`` is added for 2D turns
    (``curvature`` signed, from :func:`signed_curvature` on the xy path).
    """
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    acc_along = np.where(ds > 1e-12, (v[1:] ** 2 - v[:-1] ** 2) / (2.0 * np.maximum(ds, 1e-12)), 0.0)
    ts = np.arange(0.0, t[-1] + 1e-9, dt)
    if ts[-1] < t[-1] - 1e-9:
        ts = np.append(ts, t[-1])
    seg = np.clip(np.searchsorted(t, ts, side="right") - 1, 0, len(ds) - 1)
    tau = ts - t[seg]
    s = np.minimum(v[seg] * tau + 0.5 * acc_along[seg] * tau ** 2, ds[seg])
    direction = np.diff(points, axis=0) / np.maximum(ds, 1e-12)[:, None]
    pos = points[seg] + direction[seg] * s[:, None]
    speed = np.clip(v[seg] + acc_along[seg] * tau, 0.0, None)
    vel = direction[seg] * speed[:, None]
    acc = direction[seg] * acc_along[seg][:, None]
    if curvature is not None:
        k = 0.5 * (curvature[seg] + curvature[seg + 1])
        normal = np.stack([-direction[seg][:, 1], direction[seg][:, 0], np.zeros(len(seg))], axis=1)
        acc = acc + normal * (speed ** 2 * k)[:, None]
    return Samples(ts + t0, pos, vel, acc)


def profile_1d(start: np.ndarray, end: np.ndarray, v_max: float, a_max: float, dt: float,
               v_slow: Optional[float] = None, slow_last_m: float = 0.0, t0: float = 0.0) -> Tuple[Samples, float]:
    """Straight move from rest to rest (climb, descent, return leg), optionally slowing to ``v_slow``
    for the last ``slow_last_m`` metres (touchdown).  Returns the samples and the duration."""
    length = float(np.linalg.norm(end - start))
    n = max(2, int(np.ceil(length / 0.01)) + 1)
    pts = start[None] + np.linspace(0.0, 1.0, n)[:, None] * (end - start)[None]
    remaining = length - np.linspace(0.0, length, n)
    cap = np.full(n, v_max)
    if v_slow is not None and slow_last_m > 0.0:
        cap[remaining <= slow_last_m] = min(v_slow, v_max)
        cap[-1] = min(v_slow, v_max)  # touch down at the slow speed, then the motors stop
    v = speed_profile(pts, a_max, cap, v_start=0.0, v_end=cap[-1] if v_slow is not None else 0.0)
    t = time_stamps(pts, v)
    return sample(pts, v, t, dt, t0=t0), float(t[-1])
