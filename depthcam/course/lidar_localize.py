"""2D lidar scan -> pose against the known course map (point-to-plane scan matching).

The map is known (``course/maps/*_map.json``), so localisation is matching, not SLAM: find the
(x, y, yaw) that puts the scan's points on the walls.  :func:`match` minimises the robust (Huber)
sum of squared residuals by Gauss-Newton, starting from a prior (the previous estimate moved by the
EKF's velocity).  The residual (``MatchConfig.residual``) is either ``"beam"`` (default): the
point's distance to the face its own beam crosses, from the map's outline edges, or ``"field"``:
the distance to the nearest surface from :class:`DistanceField`, a grid.  The nearest surface can
be the back face of a 10 cm wall, which gives a stable false minimum one wall thickness off.

Two corrections, each switchable so their effect can be measured (``17_lidar_study.py``):

* ``deskew``: each beam has its own time; with the velocity and yaw rate of the prior, every point
  is placed where the drone was when that beam fired, not where it was at the end of the turn.
* ``tilt``: each beam is rotated by the roll and pitch at its time (from PX4's attitude), which
  gives its true height; beams that end below ``z_floor`` (floor hits) or above the wall tops
  (over-the-wall hits) are dropped before matching.

The pose estimated is the body pose at ``t_ref`` (the time of the last beam of the scan).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from course.course_map import CourseMap
from course.geometry import PolygonSet
from course.lidar_sim import rot


class DistanceField:
    """Signed distance to the nearest obstacle boundary at height ``z`` (negative inside a wall), on
    a grid, with gradient.  Signed, because the field is then linear through a wall face and
    bilinear interpolation is exact there; the unsigned one has a V at every face that the grid
    blurs into a ~2 cm bias.  Walls thinner than the grid (the tube's 6 mm shell) still blur: the
    field there never reaches zero, so :func:`_beam_residual` uses the outline edges
    (``edge_a`` -> ``edge_b``) exactly instead."""

    def __init__(self, cmap: CourseMap, z: float, res: float = 0.02, pad: float = 1.0, include_nets: bool = True):
        obstacles = [o for o in cmap.obstacles_2d(z - 0.02, z + 0.02, rocks_full_height=False)
                     if include_nets or o.kind != "net"]
        self.edge_a = np.concatenate([np.asarray(o.polygon, dtype=float) for o in obstacles])
        self.edge_b = np.concatenate([np.roll(np.asarray(o.polygon, dtype=float), -1, axis=0) for o in obstacles])
        packed = PolygonSet([o.polygon for o in obstacles])
        (x0, x1), (y0, y1) = cmap.boundary["x"], cmap.boundary["y"]
        self.x0, self.y0, self.res = x0 - pad, y0 - pad, res
        nx, ny = int(math.ceil((x1 - x0 + 2 * pad) / res)) + 1, int(math.ceil((y1 - y0 + 2 * pad) / res)) + 1
        xs, ys = self.x0 + np.arange(nx) * res, self.y0 + np.arange(ny) * res
        gx, gy = np.meshgrid(xs, ys)
        pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
        d = np.concatenate([packed.distance(c)[0] for c in np.array_split(pts, 64)])
        self.field = d.reshape(ny, nx)
        self.grad_y, self.grad_x = np.gradient(self.field, res)

    def lookup(self, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ny, nx = self.field.shape
        fx = np.clip((p[:, 0] - self.x0) / self.res, 0, nx - 1.000001)
        fy = np.clip((p[:, 1] - self.y0) / self.res, 0, ny - 1.000001)
        ix, iy = np.floor(fx).astype(int), np.floor(fy).astype(int)
        tx, ty = fx - ix, fy - iy

        def lerp(g: np.ndarray) -> np.ndarray:
            return ((1 - tx) * (1 - ty) * g[iy, ix] + tx * (1 - ty) * g[iy, ix + 1]
                    + (1 - tx) * ty * g[iy + 1, ix] + tx * ty * g[iy + 1, ix + 1])

        return lerp(self.field), np.stack([lerp(self.grad_x), lerp(self.grad_y)], axis=1)


@dataclass
class MatchConfig:
    deskew: bool = True
    tilt: bool = True
    z_floor: float = 0.15          # beams ending lower than this hit the floor: dropped (tilt mode)
    wall_top_margin: float = 0.05  # beams ending higher than wall top - this passed over a wall: dropped
    huber_m: float = 0.05
    outlier_m: float = 0.4
    iterations: int = 15
    mount: Tuple[float, float, float] = (0.0, 0.0, 0.10)
    residual: str = "beam"          # "beam": distance along the beam to the first surface; "field": nearest surface
    beam_window_m: float = 0.25     # how far before / after the measured end the beam residual searches


def _beam_residual(field: DistanceField, p: np.ndarray, origin: np.ndarray, window: float
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """Point-to-plane residual against the face each beam actually hits.  The beam (``origin`` ->
    ``p``) is intersected with the map's outline edges; the first face it crosses within
    +-``window`` of its measured end is the one it saw, and the residual is the end point's distance
    to that face along the face normal (the gradient, pointing back at the sensor).

    The nearest-surface distance instead pulls a point that sits past the middle of a 10 cm wall
    out through the back face, which a level lidar never sees: a stable false minimum one wall
    thickness off.  Beams that cross no face in the window keep the nearest-surface residual.
    (Using the beam direction itself as the gradient is wrong away from normal incidence: for
    grazing beams in the tube it slides the pose along the wall.)"""
    d = p - origin
    rng = np.linalg.norm(d, axis=1)
    u = d / np.maximum(rng, 1e-9)[:, None]
    e = field.edge_b - field.edge_a
    w = field.edge_a[None] - origin[:, None]
    den = u[:, None, 0] * e[None, :, 1] - u[:, None, 1] * e[None, :, 0]
    safe = np.where(np.abs(den) > 1e-12, den, np.inf)
    t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / safe
    s = (w[..., 0] * u[:, None, 1] - w[..., 1] * u[:, None, 0]) / safe
    lo = np.maximum(rng - window, 0.02)[:, None]
    t = np.where((s >= 0) & (s <= 1) & (t >= lo) & (t <= (rng + window)[:, None]), t, np.inf)
    k = np.argmin(t, axis=1)
    t_hit = t[np.arange(len(p)), k]
    found = np.isfinite(t_hit)
    n = np.stack([-e[k, 1], e[k, 0]], axis=1)
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    cos_in = np.sum(n * u, axis=1)
    n = np.where((cos_in > 0)[:, None], -n, n)  # the face normal that points back at the sensor
    r_near, g_near = field.lookup(p)
    r = np.where(found, (rng - np.where(found, t_hit, 0.0)) * -np.abs(cos_in), r_near)
    return r, np.where(found[:, None], n, g_near)


def scan_points(ranges: np.ndarray, angles: np.ndarray, rp: np.ndarray, cfg: MatchConfig) -> np.ndarray:
    """Beam end points in a yaw-free, level-referenced body frame: rotated by roll/pitch (tilt mode)
    or taken as horizontal; the mount offset included.  NaN ranges stay NaN."""
    local = np.stack([ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)], axis=1)
    local = local + np.array(cfg.mount)
    if cfg.tilt:
        r = rot(rp[:, 0], rp[:, 1], np.zeros(len(rp)))
        return np.einsum("nij,nj->ni", r, local)
    return local


def match(field: DistanceField, ranges: np.ndarray, angles: np.ndarray, dt: np.ndarray, rp: np.ndarray,
          prior: np.ndarray, z_body: float, vel_xy: np.ndarray, yaw_rate: float, cfg: MatchConfig,
          wall_top: float = 1.7272, band: Optional[Tuple[float, float]] = None) -> Tuple[np.ndarray, Dict[str, float]]:
    """Pose (x, y, yaw) at the scan's reference time.  ``dt`` = beam time - reference time (<= 0),
    ``rp`` = roll/pitch per beam (FLU), ``prior`` = (x, y, yaw) guess, ``z_body`` = body height.
    ``band`` = the beam end heights ``field`` describes (tilt mode drops the rest); by default from
    the floor (``cfg.z_floor``) to just under the wall tops."""
    q = scan_points(ranges, angles, rp, cfg)
    ok = np.isfinite(q[:, 0])
    stats: Dict[str, float] = {"beams": int(ok.sum())}
    if cfg.tilt:
        lo, hi = band if band is not None else (cfg.z_floor, wall_top - cfg.wall_top_margin)
        z = z_body + q[:, 2]
        floor = ok & (z < lo)
        over = ok & (z > hi)
        stats["floor_dropped"] = int(floor.sum())
        stats["over_dropped"] = int(over.sum())
        ok &= ~floor & ~over
    # the lidar centre in the same frame (the mount offset tilted with the body), for beam directions
    mount = np.array(cfg.mount)
    q_origin = (np.einsum("nij,j->ni", rot(rp[:, 0], rp[:, 1], np.zeros(len(rp))), mount) if cfg.tilt
                else np.tile(mount, (len(rp), 1)))
    q, dt, q_origin = q[ok, :2], dt[ok], q_origin[ok, :2]
    if not cfg.deskew:
        dt = np.zeros_like(dt)
    pose = np.array(prior, dtype=float)
    prev_step = None
    h = None
    for _ in range(cfg.iterations):
        yaw = pose[2] + yaw_rate * dt
        c, s = np.cos(yaw), np.sin(yaw)
        qw = np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]], axis=1)
        shift = pose[:2] + dt[:, None] * vel_xy[None]
        p = qw + shift
        if cfg.residual == "beam":
            origin = np.stack([c * q_origin[:, 0] - s * q_origin[:, 1], s * q_origin[:, 0] + c * q_origin[:, 1]], 1) + shift
            r, g = _beam_residual(field, p, origin, cfg.beam_window_m)
        else:
            r, g = field.lookup(p)
        a = np.abs(r)
        use = a < cfg.outlier_m
        if use.sum() < 20:
            stats["failed"] = 1
            break
        w = np.where(a < cfg.huber_m, 1.0, cfg.huber_m / np.maximum(a, 1e-9)) * use
        j = np.stack([g[:, 0], g[:, 1], -g[:, 0] * qw[:, 1] + g[:, 1] * qw[:, 0]], axis=1)
        h = (j * w[:, None]).T @ j + np.diag([1e-6, 1e-6, 1e-6])
        step = -np.linalg.solve(h, (j * w[:, None]).T @ r)
        pose += step
        if np.abs(step[:2]).max() < 1e-4 and abs(step[2]) < 1e-4:
            break
        # A step that undoes the last one: a few beams on a threshold (Huber knee, outlier cut, the face
        # a beam crosses) flip between iterations and the pose bounces between two fits < 1 mm apart.
        # Take the midpoint instead of bouncing to the iteration cap (that cost 90 ms on the Jetson).
        if prev_step is not None and np.all(np.abs(step + prev_step) <= 0.1 * np.abs(step) + 1e-6):
            pose -= step / 2
            break
        prev_step = step
    stats["used"] = int(use.sum())
    stats["rms_m"] = float(np.sqrt(np.mean(r[use] ** 2))) if use.any() else math.nan
    # DroneHacks: the last iteration's Gauss-Newton information J^T W J (x, y, yaw), without the
    # regulariser, and the weighted residual variance. pose_fusion.localizer turns them into a
    # covariance. A direction the scan cannot see (along a featureless corridor) has little
    # information, and so a large variance. Costs nothing: both were computed above.
    if "failed" not in stats and h is not None:
        stats["information"] = h - np.diag([1e-6, 1e-6, 1e-6])
        wsum = float(np.sum(w))
        stats["sigma2_m2"] = float(np.sum(w * r ** 2) / max(wsum - 3.0, 1.0))
    return pose, stats


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


#: :class:`Tracker` guards for the live tools (19's bridge, 18 localize): a wrong fit is "lost", never a pose.
LIVE_GUARDS = dict(max_rms_m=0.10, min_used_frac=0.5, acquire_jump_m=1.0, acquire_yaw_rad=math.radians(45.0),
                   reacquire_after=5, hold_when_lost=True)


class Tracker:
    """:func:`match` revolution after revolution, from the sensor alone.

    Each revolution starts from the last estimate moved by a constant-velocity guess, and the same
    velocity and yaw rate deskew it.  They come from the last two estimates, unless the caller passes
    better ones (PX4's velocity and gyro: after one bad match, a yaw rate differenced from the
    estimates is wild and bends the next scans).  A match that fails, or lands more than
    ``max_jump_m`` / ``max_yaw_jump_rad`` from the guess, is not trusted: the guess stands in for it
    and ``lost`` counts it.  ``field`` / ``wall_top`` / ``band`` can be swapped between revolutions
    (the map slice at the scan's height).

    The live tools (19's bridge, 18 localize) also set the guards below; the mapper leaves them off
    (while it explores, most of a scan has no map yet).  A bench run without them sent PX4 a false
    lock (24 cm fit, 17 % of the beams) for a minute:

    * ``max_rms_m`` / ``min_used_frac``: a match that fits worse, or uses less than this share of the
      beams left after the height filter, is a wrong fit and not trusted, however close to the guess.
      Good bench fits: <= 6.3 cm and >= 98 %; the false lock: 24 cm and 14-17 %.
    * ``acquire_jump_m`` / ``acquire_yaw_rad``: until the first trusted match, and again after
      ``reacquire_after`` untrusted revolutions in a row, a well-fitting match is accepted this far
      from the guess (``stats["acquired"]``), instead of ``max_jump_m`` / ``max_yaw_jump_rad``.
    * ``hold_when_lost``: while not trusted, the guess stays put (turned only by a gyro rate passed in)
      and a caller's velocity is not used before the first trusted match.  Otherwise the guess
      drifts with that velocity (PX4's, meaningless before it fuses the lidar: 0.1 m/s on the bench)
      until good matches are "too far" from it."""

    def __init__(self, field: DistanceField, cfg: MatchConfig, start, wall_top: float = 1.7272,
                 max_jump_m: float = 0.3, max_yaw_jump_rad: float = 0.1, max_rms_m: Optional[float] = None,
                 min_used_frac: float = 0.0, acquire_jump_m: Optional[float] = None,
                 acquire_yaw_rad: Optional[float] = None, reacquire_after: int = 5, hold_when_lost: bool = False) -> None:
        self.field, self.cfg, self.wall_top, self.max_jump_m = field, cfg, wall_top, max_jump_m
        self.max_yaw_jump_rad = max_yaw_jump_rad
        self.max_rms_m, self.min_used_frac = max_rms_m, min_used_frac
        self.acquire_jump_m, self.acquire_yaw_rad, self.reacquire_after = acquire_jump_m, acquire_yaw_rad, reacquire_after
        self.hold_when_lost = hold_when_lost
        self.band: Optional[Tuple[float, float]] = None
        self.pose = np.array(start, dtype=float)
        self.t_ref: Optional[float] = None
        self.vel = np.zeros(2)
        self.yaw_rate = 0.0
        self.lost = 0
        self.trusted = 0
        self.lost_in_row = 0

    @property
    def acquiring(self) -> bool:
        """No trusted match yet, or ``reacquire_after`` untrusted ones in a row."""
        return self.trusted == 0 or self.lost_in_row >= self.reacquire_after

    def update(self, t: np.ndarray, ranges: np.ndarray, angles: np.ndarray, rp: np.ndarray, z_body: float,
               vel_xy: Optional[np.ndarray] = None, yaw_rate: Optional[float] = None
               ) -> Tuple[np.ndarray, Dict[str, float]]:
        """Pose (x, y, yaw) at ``t[-1]``.  ``t`` = beam times (s, any clock), ``rp`` = roll/pitch per
        beam (FLU; zeros if level), ``z_body`` = body height."""
        t_ref = float(t[-1])
        acquiring = self.acquiring
        holding = self.hold_when_lost and (self.trusted == 0 or self.lost_in_row > 0)
        if self.hold_when_lost and self.trusted == 0:
            vel = np.zeros(2)  # a caller's velocity is not trusted before ours is
        else:
            vel = self.vel if vel_xy is None else np.asarray(vel_xy, dtype=float)
        rate = self.yaw_rate if yaw_rate is None else float(yaw_rate)
        prior = self.pose.copy()
        if self.t_ref is not None:
            if holding:  # stay put; only a gyro passed in may turn the guess
                prior[2] += (0.0 if yaw_rate is None else rate) * (t_ref - self.t_ref)
            else:
                prior += np.r_[vel, rate] * (t_ref - self.t_ref)
        pose, stats = match(self.field, ranges, angles, t - t_ref, rp, prior, z_body, vel, rate, self.cfg, self.wall_top,
                            self.band)
        stats["match_pose"] = pose.copy()
        stats["jump_m"] = float(np.hypot(*(pose[:2] - prior[:2])))
        stats["yaw_jump_rad"] = abs(_wrap(pose[2] - prior[2]))
        kept = stats.get("beams", 0) - stats.get("floor_dropped", 0) - stats.get("over_dropped", 0)
        stats["used_frac"] = stats.get("used", 0) / max(kept, 1)
        if acquiring and self.acquire_jump_m is not None:
            jump_limit = self.acquire_jump_m
            yaw_limit = self.acquire_yaw_rad if self.acquire_yaw_rad is not None else 3.0 * self.max_yaw_jump_rad
        else:
            jump_limit = self.max_jump_m
            yaw_limit = self.max_yaw_jump_rad * (1.0 if self.trusted else 3.0)  # the start heading is a guess
        if "failed" in stats:
            stats["reason"] = "failed"
        elif (self.max_rms_m is not None and not stats["rms_m"] <= self.max_rms_m) or stats["used_frac"] < self.min_used_frac:
            stats["reason"] = "poor fit"
        elif stats["jump_m"] > jump_limit or stats["yaw_jump_rad"] > yaw_limit:
            stats["reason"] = "jump"
        if "reason" in stats:
            self.lost += 1
            self.lost_in_row += 1
            stats["lost"] = 1
            pose = prior
        else:
            if acquiring and self.acquire_jump_m is not None:
                stats["acquired"] = 1
            if self.t_ref is not None and t_ref > self.t_ref and (self.lost_in_row == 0 or not self.hold_when_lost):
                dt = t_ref - self.t_ref  # (after a held guess the difference is not a velocity)
                self.vel = (pose[:2] - self.pose[:2]) / dt
                self.yaw_rate = _wrap(pose[2] - self.pose[2]) / dt
            self.trusted += 1
            self.lost_in_row = 0
        self.pose, self.t_ref = pose, t_ref
        return pose, stats
