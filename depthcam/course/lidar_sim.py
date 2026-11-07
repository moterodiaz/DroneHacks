"""Synthetic Oradar MS200K scans from the course map along a flown trajectory (numpy only).

Why
---
The Gazebo GPU lidar runs at 3 % of real time under WSL2 on the laptop, and it would not model the
two things that matter most for a 2D lidar on an agile drone anyway:

* **scan smear**: one revolution takes 1/rate s (0.1 s at 10 Hz); at 2.5 m/s the drone moves
  25 cm during it, so a scan treated as one snapshot is bent;
* **tilt**: a multirotor tilts to accelerate (17 deg at 3 m/s^2), so the "horizontal" scan plane
  dips into the floor on one side and climbs over the 1.73 m walls on the other.

So this casts every beam in 3D from the pose the drone had *at that beam's own time*.

Sensor model (:class:`LidarSpec`)
---------------------------------
From the MS200 SDK and listings: about 4500 samples/s, rotation 5-15 Hz (driver ``motor_speed``,
default 10), clockwise sweep seen from above (the driver mirrors the angles when
``clockwise=False``), range up to 12 m, 905 nm dToF.  Assumed, not measured: 1 cm range noise,
2 % dropouts, and that the thin black nets return a beam 20 % of the time.

Scene
-----
Walls are boxes (split around the window), the tube is its outer and inner elliptical surfaces,
boulders are cylinders, nets are thin boxes plus the roof plane, and the floor is the plane z = 0.
Coordinates: the course frame (ENU, metres).  Attitude: roll/pitch/yaw of the body in FLU/ENU
(:func:`frd_to_flu` converts PX4's NED/FRD angles).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from course.course_map import CourseMap
from course.geometry import min_area_rect
from course.worldgen import _wall_boxes

FLOOR, WALL, TUBE, ROCK, NET, NONE = 0, 1, 2, 3, 4, 5
KIND_NAMES = {FLOOR: "floor", WALL: "wall", TUBE: "tube", ROCK: "rock", NET: "net", NONE: "none"}


@dataclass
class LidarSpec:
    rate_hz: float = 10.0
    samples_hz: float = 4500.0
    range_min: float = 0.05
    range_max: float = 12.0
    range_sigma: float = 0.01
    dropout: float = 0.02
    net_hit_prob: float = 0.2
    clockwise: bool = True
    mount: Tuple[float, float, float] = (0.0, 0.0, 0.10)  # scan centre in the body frame (FLU), m

    @property
    def points_per_rev(self) -> int:
        return int(round(self.samples_hz / self.rate_hz))


@dataclass
class Scan:
    t: np.ndarray        # (n,) time of every beam, s
    angle: np.ndarray    # (n,) beam angle in the lidar/body frame, rad, CCW from forward
    range: np.ndarray    # (n,) metres, NaN = no return
    kind: np.ndarray     # (n,) surface hit (FLOOR, WALL, ...)
    true_pos: np.ndarray  # (n, 3) body position at each beam (truth, for evaluation)
    true_rpy: np.ndarray  # (n, 3)

    @property
    def t_end(self) -> float:
        return float(self.t[-1])


def rot(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """(..., 3, 3) body (FLU) -> world (ENU): Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    r = np.empty(np.shape(roll) + (3, 3))
    r[..., 0, 0] = cy * cp
    r[..., 0, 1] = cy * sp * sr - sy * cr
    r[..., 0, 2] = cy * sp * cr + sy * sr
    r[..., 1, 0] = sy * cp
    r[..., 1, 1] = sy * sp * sr + cy * cr
    r[..., 1, 2] = sy * sp * cr - cy * sr
    r[..., 2, 0] = -sp
    r[..., 2, 1] = cp * sr
    r[..., 2, 2] = cp * cr
    return r


def frd_to_flu(roll_frd: np.ndarray, pitch_frd: np.ndarray, yaw_ned: np.ndarray) -> np.ndarray:
    """PX4 attitude (FRD body in NED) -> (roll, pitch, yaw) of the FLU body in ENU."""
    return np.stack([roll_frd, -pitch_frd, np.pi / 2 - yaw_ned], axis=-1)


@dataclass
class Scene:
    box_c: np.ndarray        # (K, 3) centres
    box_yaw: np.ndarray      # (K,)
    box_h: np.ndarray        # (K, 3) half sizes
    box_kind: np.ndarray     # (K,)
    tubes: List[dict] = field(default_factory=list)
    rocks: List[Tuple[np.ndarray, float, float]] = field(default_factory=list)  # centre xy, radius, height
    roof: Optional[float] = None
    wall_top: float = 1.7272

    @classmethod
    def from_map(cls, cmap: CourseMap) -> "Scene":
        centres, yaws, halves, kinds = [], [], [], []
        tubes, rocks, tops = [], [], []
        mid = np.array([np.mean(cmap.boundary["x"]), np.mean(cmap.boundary["y"])])
        for f in cmap.features:
            kind = f["kind"]
            if kind == "wall":
                for pose, size in _wall_boxes(f):
                    centres.append(pose[:3]), yaws.append(pose[5]), halves.append(np.array(size) / 2), kinds.append(WALL)
                tops.append(f["z"][1])
            elif kind in ("block", "support"):
                c, angle, length, width = min_area_rect(cmap.footprint(f["name"]))
                z0, z1 = f["z"]
                centres.append((c[0], c[1], (z0 + z1) / 2)), yaws.append(angle)
                halves.append(np.array([length, width, z1 - z0]) / 2), kinds.append(WALL)
            elif kind == "net":
                fp = np.array(f["footprint"])
                c, angle, length, width = min_area_rect(fp)
                z0, z1 = f["z"]
                if z1 - z0 > 1.0:
                    n = np.array([-math.sin(angle), math.cos(angle)])
                    if np.dot(c - mid, n) < 0:
                        n = -n
                    cc = c + n * (0.01 - width / 2)
                    centres.append((cc[0], cc[1], (z0 + z1) / 2)), yaws.append(angle)
                    halves.append(np.array([length / 2, 0.01, (z1 - z0) / 2])), kinds.append(NET)
            elif kind == "tube":
                p0, p1 = np.array(f["p0"]), np.array(f["p1"])
                u = (p1 - p0) / np.linalg.norm(p1 - p0)
                w = np.cross([0.0, 0.0, 1.0], u)
                tubes.append({"p0": p0, "u": u, "w": w / np.linalg.norm(w), "length": float(np.linalg.norm(p1 - p0)),
                              "ellipses": [(f["outer"]["half_width"], f["outer"]["half_height"]),
                                           (f["inner"]["half_width"], f["inner"]["half_height"])]})
            elif kind == "rock":
                rocks.append((np.array(f["centre"]), f["radius"], f["z"][1]))
        return cls(np.array(centres, dtype=float), np.array(yaws), np.array(halves), np.array(kinds), tubes, rocks,
                   cmap.boundary.get("z", [0, None])[1], max(tops) if tops else 1.7272)

    def cast(self, o: np.ndarray, d: np.ndarray, rng: np.random.Generator, net_hit_prob: float
             ) -> Tuple[np.ndarray, np.ndarray]:
        """First hit distance (inf if none) and surface kind for rays o + t d (unit d)."""
        n = len(o)
        best = np.full(n, np.inf)
        kind = np.full(n, NONE)

        def take(t: np.ndarray, k: np.ndarray) -> None:
            better = t < best
            best[better] = t[better]
            kind[better] = k[better] if np.ndim(k) else k

        with np.errstate(divide="ignore", invalid="ignore"):
            # floor and roof
            t = np.where(d[:, 2] < -1e-9, -o[:, 2] / d[:, 2], np.inf)
            take(np.where(t > 1e-6, t, np.inf), FLOOR)
            if self.roof is not None:
                t = np.where(d[:, 2] > 1e-9, (self.roof - o[:, 2]) / d[:, 2], np.inf)
                t = np.where((t > 1e-6) & (rng.random(n) < net_hit_prob), t, np.inf)
                take(t, NET)
            # boxes (walls, nets): slab test in each box's frame
            if len(self.box_c):
                c, s = np.cos(self.box_yaw), np.sin(self.box_yaw)
                rel = o[:, None, :] - self.box_c[None]
                ox = c * rel[..., 0] + s * rel[..., 1]
                oy = -s * rel[..., 0] + c * rel[..., 1]
                dx = c * d[:, None, 0] + s * d[:, None, 1]
                dy = -s * d[:, None, 0] + c * d[:, None, 1]
                dz = np.broadcast_to(d[:, None, 2], dx.shape)
                lo = np.full(dx.shape, -np.inf)
                hi = np.full(dx.shape, np.inf)
                for oc, dc, h in ((ox, dx, self.box_h[:, 0]), (oy, dy, self.box_h[:, 1]),
                                  (rel[..., 2], dz, self.box_h[:, 2])):
                    t1 = (-h[None] - oc) / dc
                    t2 = (h[None] - oc) / dc
                    # A beam parallel to this pair of faces: no constraint if it runs between them,
                    # a miss if it runs outside (an empty interval: both bounds at +inf).
                    parallel = np.abs(dc) < 1e-12
                    inside = np.abs(oc) <= h[None]
                    t1 = np.where(parallel, np.where(inside, -np.inf, np.inf), t1)
                    t2 = np.where(parallel, np.inf, t2)
                    lo = np.maximum(lo, np.minimum(t1, t2))
                    hi = np.minimum(hi, np.maximum(t1, t2))
                t = np.where((hi >= lo) & (lo > 1e-6), lo, np.inf)
                is_net = self.box_kind[None, :] == NET
                t = np.where(is_net & (rng.random(t.shape) >= net_hit_prob), np.inf, t)
                j = np.argmin(t, axis=1)
                take(t[np.arange(n), j], self.box_kind[j])
            # tubes: both elliptical surfaces, within the tube's length
            for tube in self.tubes:
                rel = o - tube["p0"]
                s0, w0, z0 = rel @ tube["u"], rel @ tube["w"], rel[:, 2]
                ds, dw, dz = d @ tube["u"], d @ tube["w"], d[:, 2]
                for a, b in tube["ellipses"]:
                    qa = dw ** 2 / a ** 2 + dz ** 2 / b ** 2
                    qb = 2 * (w0 * dw / a ** 2 + z0 * dz / b ** 2)
                    qc = w0 ** 2 / a ** 2 + z0 ** 2 / b ** 2 - 1
                    disc = qb ** 2 - 4 * qa * qc
                    root = np.sqrt(np.maximum(disc, 0))
                    for t in ((-qb - root) / (2 * qa), (-qb + root) / (2 * qa)):
                        s = s0 + t * ds
                        ok = (disc >= 0) & (t > 1e-6) & (s >= 0) & (s <= tube["length"])
                        take(np.where(ok, t, np.inf), TUBE)
            # rocks: vertical cylinders
            for centre, r, h in self.rocks:
                ox, oy = o[:, 0] - centre[0], o[:, 1] - centre[1]
                qa = d[:, 0] ** 2 + d[:, 1] ** 2
                qb = 2 * (ox * d[:, 0] + oy * d[:, 1])
                qc = ox ** 2 + oy ** 2 - r * r
                disc = qb ** 2 - 4 * qa * qc
                t = (-qb - np.sqrt(np.maximum(disc, 0))) / (2 * qa)
                z = o[:, 2] + t * d[:, 2]
                ok = (disc >= 0) & (t > 1e-6) & (z >= 0) & (z <= h)
                take(np.where(ok, t, np.inf), ROCK)
        return best, kind


@dataclass
class Trajectory:
    """Body pose over time in the course frame: position ENU (body centre), roll/pitch/yaw FLU/ENU."""

    t: np.ndarray
    pos: np.ndarray
    rpy: np.ndarray

    def at(self, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pos = np.stack([np.interp(t, self.t, self.pos[:, k]) for k in range(3)], axis=-1)
        yaw = np.unwrap(self.rpy[:, 2])
        rpy = np.stack([np.interp(t, self.t, self.rpy[:, 0]), np.interp(t, self.t, self.rpy[:, 1]),
                        np.interp(t, self.t, yaw)], axis=-1)
        return pos, rpy


def simulate(traj: Trajectory, scene: Scene, spec: LidarSpec, t0: float, t1: float, seed: int = 0,
             freeze: bool = False, level: bool = False) -> List[Scan]:
    """Revolutions from t0 to t1.  ``freeze``: every beam of a revolution uses the pose at its end
    (no smear); ``level``: roll = pitch = 0 (no tilt).  Both are for isolating the two effects."""
    rng = np.random.default_rng(seed)
    n = spec.points_per_rev
    period = 1.0 / spec.rate_hz
    sweep = -1.0 if spec.clockwise else 1.0
    scans: List[Scan] = []
    start = t0
    while start + period <= t1:
        tb = start + np.arange(n) / spec.samples_hz
        tq = np.full(n, tb[-1]) if freeze else tb
        pos, rpy = traj.at(tq)
        if level:
            rpy = rpy.copy()
            rpy[:, :2] = 0.0
        angle = np.mod(sweep * 2 * np.pi * np.arange(n) / n, 2 * np.pi)
        r = rot(rpy[:, 0], rpy[:, 1], rpy[:, 2])
        origin = pos + r @ np.array(spec.mount)
        direction = np.einsum("nij,nj->ni", r, np.stack([np.cos(angle), np.sin(angle), np.zeros(n)], axis=1))
        dist, kind = scene.cast(origin, direction, rng, spec.net_hit_prob)
        dist = dist + rng.normal(0.0, spec.range_sigma, n)
        bad = (~np.isfinite(dist)) | (dist > spec.range_max) | (dist < spec.range_min) | (rng.random(n) < spec.dropout)
        dist[bad] = np.nan
        kind = np.where(bad, NONE, kind)
        scans.append(Scan(tb, angle, dist, kind, pos, rpy))
        start += period
    return scans
