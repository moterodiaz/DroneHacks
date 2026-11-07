"""A time-parameterised trajectory: where to be, how fast, at any time t.

Built from a ``SmoothPath`` and a ``SpeedProfile``. ``at(t)`` interpolates
position, velocity, acceleration, heading and progress, and says which
opening (if any) the reference is in.

Heading (yaw, course frame, radians from east, counter-clockwise) looks
``yaw_lookahead_m`` ahead along the path, so the camera turns into the next
corridor before the corner (field guide: "the nose pointed 0.8 m ahead so the
camera never faces a corridor's end wall"). Inside an opening it is locked to
the direction of travel: no turning in the tunnel.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from .smoothing import SmoothPath
from .speed import SpeedProfile, acceleration_vectors


@dataclasses.dataclass(frozen=True)
class TrajectorySample:
    t: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float
    s: float
    progress: float      # s / length, 0..1
    opening: str         # name of the opening the reference is in, or ""


class Trajectory:
    def __init__(self, path: SmoothPath, profile: SpeedProfile, yaw_lookahead_m: float = 0.8):
        self.path = path
        self.profile = profile
        self.t = profile.t
        self.s = path.s
        self.xy = path.xy
        self.vel = profile.v[:, None] * path.tangent
        self.acc = acceleration_vectors(path, profile)
        self.yaw = _headings(path, yaw_lookahead_m)
        self.length = float(path.s[-1])

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    def at(self, t: float) -> TrajectorySample:
        t = min(max(t, 0.0), self.duration)
        i = int(np.searchsorted(self.t, t, side="right")) - 1
        i = min(max(i, 0), len(self.t) - 2)
        span = self.t[i + 1] - self.t[i]
        f = 0.0 if span <= 0 else (t - self.t[i]) / span
        lerp = lambda arr: arr[i] * (1 - f) + arr[i + 1] * f  # noqa: E731
        s = float(lerp(self.s))
        return TrajectorySample(
            t=t,
            position=lerp(self.xy),
            velocity=lerp(self.vel),
            acceleration=lerp(self.acc),
            yaw=float(lerp(self.yaw)),
            s=s,
            progress=s / self.length if self.length > 0 else 1.0,
            opening=self.path.pass_name[i if f < 0.5 else i + 1],
        )

    def position_at_s(self, s: float) -> np.ndarray:
        return np.array([np.interp(s, self.s, self.xy[:, 0]), np.interp(s, self.s, self.xy[:, 1])])


def _headings(path: SmoothPath, lookahead: float) -> np.ndarray:
    s, xy = path.s, path.xy
    ahead = np.minimum(s + lookahead, s[-1])
    target = np.column_stack([np.interp(ahead, s, xy[:, 0]), np.interp(ahead, s, xy[:, 1])])
    d = target - xy
    near_end = np.hypot(*d.T) < 0.05
    d[near_end] = path.tangent[near_end]
    in_pass = np.hypot(*path.pass_dir.T) > 0
    d[in_pass] = path.pass_dir[in_pass]
    yaw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    return yaw


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
