"""Map a room from a recorded MS200K walk (2D lidar SLAM), for the bench tests of the lidar tools.

``lidar_localize`` matches scans to a known map.  Here the map is built from the scans themselves:

* :class:`PointMap` holds the points seen so far, each with the surface normal estimated from its
  neighbours along its own scan.  Its :meth:`PointMap.lookup` has :class:`DistanceField`'s
  signature, so :func:`lidar_localize.match` (``residual="field"``) does point-to-plane ICP against
  it with the same deskew, tilt filter, Huber weights and Gauss-Newton.
* :func:`build` runs a :class:`lidar_localize.Tracker` over the revolutions.  The first revolution
  defines the map frame (the start pose, nose +y); every ``keyframe_m`` / ``keyframe_deg`` of motion a
  trusted revolution's points join the map.  With ``closed=True`` (the walk ended where it began,
  facing the same way) the end-pose error is spread linearly over the keyframes; then ``refine_passes``
  re-run every revolution against the finished map and rebuild it from the new keyframe poses.
* :func:`extract` turns the final points into map features: straight runs become thin walls whose
  visible face lies on the measured surface, what is left becomes convex blocks.

Only numpy (plus OpenCV for the nearest-point grid and blob outlines), like the other course tools.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from course.lidar_localize import MatchConfig, Tracker, scan_points

#: One revolution for :func:`build`: beam times (s), ranges (m, NaN = none), angles (rad, CCW from the
#: nose), roll/pitch per beam (FLU, or None = level) and the gyro's heading rate (CCW rad/s, or None).
Rev = Tuple[np.ndarray, np.ndarray, np.ndarray, Optional[np.ndarray], Optional[float]]

OUTLIER_R = 10.0   # residual lookup() reports for a point with no map surface near it (match drops it)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def scan_normals(xy: np.ndarray, k: int = 5, max_gap_m: float = 0.10, flatness: float = 0.3) -> np.ndarray:
    """(n, 2) unit normals of the surface under each point of one scan (points in beam order, NaN =
    no return), from the principal axis of the up to ``2k + 1`` scan neighbours within ``max_gap_m``;
    NaN where fewer than 3 neighbours qualify or the neighbourhood is not line-like."""
    n = len(xy)
    idx = np.clip(np.arange(n)[:, None] + np.arange(-k, k + 1)[None], 0, n - 1)
    nb = xy[idx]                                              # (n, 2k+1, 2)
    close = np.linalg.norm(nb - xy[:, None], axis=2) <= max_gap_m
    close &= np.isfinite(nb[..., 0]) & np.isfinite(xy[:, None, 0])
    cnt = close.sum(axis=1)
    w = close[..., None]
    mean = np.where(w, nb, 0.0).sum(axis=1) / np.maximum(cnt, 1)[:, None]
    d = np.where(w, nb - mean[:, None], 0.0)
    sxx, syy, sxy = (d[..., 0] ** 2).sum(1), (d[..., 1] ** 2).sum(1), (d[..., 0] * d[..., 1]).sum(1)
    ang = 0.5 * np.arctan2(2 * sxy, sxx - syy)                # direction of the line
    tr = sxx + syy
    det = sxx * syy - sxy ** 2
    lam_small = tr / 2 - np.sqrt(np.maximum(tr ** 2 / 4 - det, 0.0))
    flat = lam_small <= flatness * np.maximum(tr - lam_small, 1e-12)  # spread across << along
    out = np.stack([-np.sin(ang), np.cos(ang)], axis=1)
    out[(cnt < 3) | ~flat] = np.nan
    return out


class PointMap:
    """Map points with normals on a grid: :meth:`lookup` is the point-to-plane residual to the nearest
    map point and that point's normal (:class:`lidar_localize.DistanceField`'s interface).

    Points are averaged per ``cell`` (normals by their principal axis, so sign does not matter);
    the nearest cell comes from OpenCV's labelled distance transform at ``cell`` resolution."""

    def __init__(self, points: np.ndarray, normals: np.ndarray, viewers: np.ndarray, cell: float = 0.02,
                 max_dist_m: float = 0.3, pad_m: float = 1.0) -> None:
        import cv2  # lazily, like worldgen: the module stays importable without OpenCV

        ok = np.isfinite(points).all(1) & np.isfinite(normals).all(1)
        points, normals, viewers = points[ok], normals[ok], viewers[ok]
        if len(points) < 10:
            raise ValueError("a map needs at least 10 points with normals, got {}".format(len(points)))
        self.cell, self.max_dist = cell, max_dist_m
        self.x0, self.y0 = points[:, 0].min() - pad_m, points[:, 1].min() - pad_m
        nx = int(math.ceil((points[:, 0].max() + pad_m - self.x0) / cell)) + 1
        ny = int(math.ceil((points[:, 1].max() + pad_m - self.y0) / cell)) + 1
        ix = ((points[:, 0] - self.x0) / cell).astype(int)
        iy = ((points[:, 1] - self.y0) / cell).astype(int)
        key = iy * nx + ix
        uniq, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
        m = len(uniq)

        def mean(v):
            s = np.zeros((m, v.shape[1]))
            np.add.at(s, inv, v)
            return s / cnt[:, None]

        self.points, self.viewers = mean(points), mean(viewers)
        nn = np.stack([normals[:, 0] ** 2, normals[:, 1] ** 2, normals[:, 0] * normals[:, 1]], axis=1)
        a = mean(nn)
        ang = 0.5 * np.arctan2(2 * a[:, 2], a[:, 0] - a[:, 1])   # principal axis of the normals
        self.normals = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        self.counts = cnt
        img = np.full((ny, nx), 255, np.uint8)
        cy, cx = uniq // nx, uniq % nx
        img[cy, cx] = 0
        dist, labels = cv2.distanceTransformWithLabels(img, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        label_to_point = np.full(labels.max() + 1, -1, np.int64)
        label_to_point[labels[cy, cx]] = np.arange(m)
        self.nearest = label_to_point[labels]                     # (ny, nx) index of the nearest map point
        self.shape = (ny, nx)

    def lookup(self, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ny, nx = self.shape
        ix = np.floor((p[:, 0] - self.x0) / self.cell).astype(int)
        iy = np.floor((p[:, 1] - self.y0) / self.cell).astype(int)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & np.isfinite(p).all(1)
        k = np.where(inside, self.nearest[np.clip(iy, 0, ny - 1), np.clip(ix, 0, nx - 1)], 0)
        q, n = self.points[k], self.normals[k]
        d = p - q
        r = np.sum(d * n, axis=1)
        far = ~inside | (np.hypot(d[:, 0], d[:, 1]) > self.max_dist)
        return np.where(far, OUTLIER_R, r), n


@dataclass
class MapConfig:
    mount_z: float = 0.10
    height: float = 0.95            # body centre above the floor while mapping (the floor-hit filter)
    ceiling: float = 2.5
    range_max: float = 8.0          # farther returns are not added to the map (they are sparse and noisy)
    keyframe_m: float = 0.10
    keyframe_deg: float = 5.0
    cell: float = 0.02
    max_jump_m: float = 0.3
    max_yaw_jump_deg: float = 8.0
    max_lost_in_row: int = 10       # a second of untrusted revolutions: the walk is not mappable as recorded
    keyframe_rms_m: float = 0.02    # only well-matched revolutions join the map
    keyframe_max_rate_dps: float = 20.0  # ... and only while turning slower than this: a 6 deg/rev sweep with a
                                         # lagging rate estimate misplaces far points by 5-20 cm
    refine_passes: int = 2
    closed: bool = False

    def match_config(self, tilt: bool) -> MatchConfig:
        return MatchConfig(deskew=True, tilt=tilt, mount=(0.0, 0.0, self.mount_z), residual="field",
                           huber_m=0.03, outlier_m=0.3, iterations=20)


@dataclass
class Keyframe:
    rev: int                        # index of its revolution
    pose: np.ndarray                # (x, y, yaw) at the revolution's last beam
    body: np.ndarray                # (n, 2) points in the level, yaw-free body frame, deskewed to that pose
    normals_body: np.ndarray        # (n, 2)
    origin_body: np.ndarray         # (n, 2) sensor centre for each point (deskewed), same frame


@dataclass
class MapResult:
    poses: np.ndarray               # (R, 3) every revolution's pose (x, y, yaw), map frame
    t: np.ndarray                   # (R,) the revolutions' last-beam times
    trusted: np.ndarray             # (R,) bool
    keyframes: List[Keyframe]
    pointmap: PointMap
    stats: Dict[str, float] = field(default_factory=dict)


class MappingError(RuntimeError):
    pass


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _keyframe(rev_i: int, rev: Rev, pose: np.ndarray, vel: np.ndarray, rate: float, mcfg: MatchConfig,
              cfg: MapConfig) -> Optional[Keyframe]:
    """A revolution's points, deskewed with ``vel`` / ``rate`` and expressed in the body frame of
    ``pose`` (the pose at its last beam), keeping only what the map should hold."""
    t, ranges, angles, rp, _ = rev
    rp = np.zeros((len(t), 2)) if rp is None else rp
    q = scan_points(ranges, angles, rp, mcfg)
    ok = np.isfinite(q[:, 0]) & (ranges <= cfg.range_max)
    if mcfg.tilt:
        z = cfg.height + q[:, 2]
        ok &= (z > mcfg.z_floor) & (z < cfg.ceiling - mcfg.wall_top_margin)
    dt = t - t[-1]
    yaw_rel = rate * dt                                       # each beam's heading relative to the pose
    c, s = np.cos(yaw_rel), np.sin(yaw_rel)
    shift = dt[:, None] * (_rot(-pose[2]) @ vel)[None]        # body-frame displacement since each beam
    qb = np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]], axis=1) + shift
    ob = np.tile(np.array([0.0, 0.0]), (len(t), 1)) + shift
    qb[~ok] = np.nan
    nb = scan_normals(qb)
    keep = np.isfinite(qb[:, 0]) & np.isfinite(nb[:, 0])
    if keep.sum() < 30:
        return None
    return Keyframe(rev_i, pose.copy(), qb[keep], nb[keep], ob[keep])


def _world(kfs: Sequence[Keyframe]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts, nrm, view = [], [], []
    for kf in kfs:
        r = _rot(kf.pose[2])
        pts.append(kf.body @ r.T + kf.pose[:2])
        nrm.append(kf.normals_body @ r.T)
        view.append(kf.origin_body @ r.T + kf.pose[:2])
    return np.vstack(pts), np.vstack(nrm), np.vstack(view)


def _pointmap(kfs: Sequence[Keyframe], cfg: MapConfig) -> PointMap:
    return PointMap(*_world(kfs), cell=cfg.cell)


def _track(revs: Sequence[Rev], cfg: MapConfig, mcfg: MatchConfig, start: np.ndarray, pmap: Optional[PointMap],
           grow: bool) -> Tuple[np.ndarray, np.ndarray, List[Keyframe], Dict[str, float]]:
    """One pass over the revolutions.  ``grow``: start the map from the first revolution and add
    keyframes as the walk goes (SLAM); otherwise localise against the fixed ``pmap`` and only collect
    the keyframes (refinement)."""
    tracker = Tracker(pmap, mcfg, start, wall_top=cfg.ceiling, max_jump_m=cfg.max_jump_m,
                      max_yaw_jump_rad=math.radians(cfg.max_yaw_jump_deg))
    tracker.band = (mcfg.z_floor, cfg.ceiling - mcfg.wall_top_margin)
    poses, trusted, kfs = np.zeros((len(revs), 3)), np.zeros(len(revs), bool), []
    last_kf: Optional[np.ndarray] = None
    in_row = worst_row = 0
    rms: List[float] = []
    for i, rev in enumerate(revs):
        t, ranges, angles, rp, gyro = rev
        rp_use = np.zeros((len(t), 2)) if rp is None else rp
        sharp = True
        if i == 0 and grow:
            pose, ok = np.array(start, dtype=float), True
            tracker.t_ref = float(t[-1])
        else:
            pose, st = tracker.update(t, ranges, angles, rp_use, cfg.height, yaw_rate=gyro)
            ok = "lost" not in st
            if ok:
                rms.append(st["rms_m"])
                sharp = st["rms_m"] <= cfg.keyframe_rms_m
        in_row = 0 if ok else in_row + 1
        worst_row = max(worst_row, in_row)
        if in_row > cfg.max_lost_in_row:
            raise MappingError("lost track for {} revolutions in a row at {:.1f} s into the walk (pose x {:+.2f} y "
                               "{:+.2f}): move slower, turn slower, or keep more walls in view".format(
                                   in_row, t[-1] - revs[0][0][-1], pose[0], pose[1]))
        poses[i], trusted[i] = pose, ok
        if not (ok and sharp):
            continue
        moved = last_kf is None or (np.hypot(*(pose[:2] - last_kf[:2])) >= cfg.keyframe_m
                                    or abs(_wrap(pose[2] - last_kf[2])) >= math.radians(cfg.keyframe_deg))
        rate = tracker.yaw_rate if gyro is None else gyro
        if not moved or abs(math.degrees(rate)) > cfg.keyframe_max_rate_dps:
            continue
        kf = _keyframe(i, rev, pose, tracker.vel, rate, mcfg, cfg)
        if kf is None:
            continue
        kfs.append(kf)
        last_kf = pose
        if grow:
            tracker.field = _pointmap(kfs, cfg)
    stats = {"revolutions": len(revs), "trusted": int(trusted.sum()), "lost_in_row_max": worst_row,
             "match_rms_m": float(np.median(rms)) if rms else math.nan}
    return poses, trusted, kfs, stats


class LiveMapper:
    """:func:`_track`'s growing pass, one revolution at a time, for a live run with no map: the first
    revolution is the origin (``start``), and every ``keyframe_m`` / ``keyframe_deg`` of motion a
    well-matched revolution turning slower than ``keyframe_max_rate_dps`` joins the map.  It never
    gives up: an untrusted revolution holds the last pose (``hold_when_lost``) and is reported lost.

    :meth:`update` only matches (a few ms); a new keyframe waits for :meth:`grow`, which rebuilds the
    map (up to ~60 ms, more with PX4's link running): call it after the pose has gone to PX4.  The map
    keeps the newest ``max_keyframes`` so a long session stays cheap to rebuild."""

    def __init__(self, cfg: MapConfig, tilt: bool, start: Sequence[float] = (0.0, 0.0, math.pi / 2),
                 max_keyframes: int = 200, max_rms_m: float = 0.10) -> None:
        self.cfg, self.mcfg = cfg, cfg.match_config(tilt)
        self.tracker = Tracker(None, self.mcfg, np.array(start, dtype=float), wall_top=cfg.ceiling,
                               max_jump_m=cfg.max_jump_m, max_yaw_jump_rad=math.radians(cfg.max_yaw_jump_deg),
                               max_rms_m=max_rms_m, hold_when_lost=True)
        self.tracker.band = (self.mcfg.z_floor, cfg.ceiling - self.mcfg.wall_top_margin)
        self.max_keyframes = max_keyframes
        self.kfs: List[Keyframe] = []
        self.last_kf: Optional[np.ndarray] = None
        self.pending: Optional[Keyframe] = None
        self.revs = 0

    def update(self, rev: Rev) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Pose (x, y, yaw) at the revolution's last beam and the match's stats."""
        t, ranges, angles, rp, gyro = rev
        rp_use = np.zeros((len(t), 2)) if rp is None else rp
        self.revs += 1
        if not self.kfs and self.pending is None:  # nothing to match yet: this revolution is the origin
            pose, st = self.tracker.pose.copy(), {"match_pose": self.tracker.pose.copy(), "first": 1}
            self.tracker.t_ref = float(t[-1])
            sharp = True
        else:
            pose, st = self.tracker.update(t, ranges, angles, rp_use, self.cfg.height, yaw_rate=gyro)
            sharp = "lost" not in st and st["rms_m"] <= self.cfg.keyframe_rms_m
        if not sharp:
            return pose, st
        moved = self.last_kf is None or (np.hypot(*(pose[:2] - self.last_kf[:2])) >= self.cfg.keyframe_m
                                         or abs(_wrap(pose[2] - self.last_kf[2])) >= math.radians(self.cfg.keyframe_deg))
        rate = self.tracker.yaw_rate if gyro is None else gyro
        if moved and abs(math.degrees(rate)) <= self.cfg.keyframe_max_rate_dps:
            self.pending = _keyframe(self.revs - 1, rev, pose, self.tracker.vel, rate, self.mcfg, self.cfg) or self.pending
        return pose, st

    def grow(self) -> Optional[np.ndarray]:
        """Add the keyframe :meth:`update` picked, if any; its world points (for a display) or None."""
        kf, self.pending = self.pending, None
        if kf is None:
            return None
        self.kfs.append(kf)
        del self.kfs[:-self.max_keyframes]
        self.last_kf = kf.pose.copy()
        self.tracker.field = _pointmap(self.kfs, self.cfg)
        return _world([kf])[0]


def build(revs: Iterable[Rev], cfg: MapConfig = MapConfig(), tilt: Optional[bool] = None) -> MapResult:
    """Map the walk.  ``tilt`` (default: any revolution has roll/pitch) filters floor and ceiling hits."""
    revs = list(revs)
    if len(revs) < 20:
        raise MappingError("{} revolutions is too short a walk (at least 2 s)".format(len(revs)))
    tilt = any(r[3] is not None for r in revs) if tilt is None else tilt
    mcfg = cfg.match_config(tilt)
    start = np.array([0.0, 0.0, math.pi / 2])
    poses, trusted, kfs, stats = _track(revs, cfg, mcfg, start, None, grow=True)
    stats = {"slam_" + k: v for k, v in stats.items()}
    t = np.array([r[0][-1] for r in revs])
    # the frame is the first revolution's pose, so the rig must be still then: how far the SLAM pass
    # (where the first revolution is the origin by definition) moved in the first second
    first = trusted & (t <= t[0] + 1.0)
    stats["start_motion_m"] = float(np.hypot(*(poses[first, :2] - start[:2]).T).max()) if first.any() else math.nan
    if len(kfs) < 3:
        raise MappingError("only {} keyframes: move the rig around the room".format(len(kfs)))
    end_err = np.r_[poses[-1, :2] - start[:2], _wrap(poses[-1, 2] - start[2])]
    stats.update(end_x_m=float(end_err[0]), end_y_m=float(end_err[1]), end_yaw_deg=math.degrees(end_err[2]))
    if cfg.closed:  # spread the end-pose error over the walk, linearly in time
        t0, t1 = revs[0][0][-1], revs[-1][0][-1]
        for kf in kfs:
            f = (revs[kf.rev][0][-1] - t0) / max(t1 - t0, 1e-9)
            kf.pose = kf.pose - f * end_err
    pmap = _pointmap(kfs, cfg)
    for p in range(cfg.refine_passes):
        poses, trusted, new_kfs, st = _track(revs, cfg, mcfg, start, pmap, grow=False)
        if len(new_kfs) >= 3:
            kfs = new_kfs
            pmap = _pointmap(kfs, cfg)
        stats.update({"refine_" + k: v for k, v in st.items()})
    stats["keyframes"] = len(kfs)
    stats["map_points"] = int(len(pmap.points))
    stats["walk_m"] = float(np.sum(np.hypot(*np.diff(poses[trusted][:, :2], axis=0).T))) if trusted.sum() > 1 else 0.0
    return MapResult(poses, t, trusted, kfs, pmap, stats)


# --- map features ---------------------------------------------------------------------------------

@dataclass
class Segment:
    a: np.ndarray                   # face end points (the measured surface)
    b: np.ndarray
    normal: np.ndarray              # unit, pointing to the side it was seen from
    points: int
    rms_m: float


def extract(pmap: PointMap, line_tol_m: float = 0.02, min_points: int = 15, min_length_m: float = 0.15,
            gap_m: float = 0.15, max_lines: int = 80, clear_tol: float = 3.0, seed: int = 0
            ) -> Tuple[List[Segment], List[np.ndarray]]:
    """Straight segments (sequential RANSAC, split where the points along a line have a gap wider
    than ``gap_m``) and convex outlines of what is left (blobs of at least 5 cells).  An accepted
    segment clears every point within ``clear_tol`` x ``line_tol_m`` of it, so the edge of a wall's
    noise band does not come back as a second wall a few cm behind the first."""
    rng = np.random.default_rng(seed)
    pts, view = pmap.points.copy(), pmap.viewers.copy()
    left = np.ones(len(pts), bool)
    segments: List[Segment] = []
    for _ in range(max_lines):
        idx = np.flatnonzero(left)
        if len(idx) < min_points:
            break
        p = pts[idx]
        best, best_n = None, 0
        for _ in range(400):
            i, j = rng.choice(len(p), 2, replace=False)
            d = p[j] - p[i]
            length = math.hypot(*d)
            if length < 0.05:
                continue
            n = np.array([-d[1], d[0]]) / length
            inl = np.abs((p - p[i]) @ n) < line_tol_m
            if inl.sum() > best_n:
                best, best_n = inl, int(inl.sum())
        if best is None or best_n < min_points:
            break
        q = p[best]
        c = q.mean(0)
        _, _, vt = np.linalg.svd(q - c)
        u, n = vt[0], vt[1]
        inl = np.abs((p - c) @ n) < line_tol_m                # refit's inliers
        q, qi = p[inl], idx[inl]
        s = (q - c) @ u
        order = np.argsort(s)
        s, qi = s[order], qi[order]
        cuts = np.flatnonzero(np.diff(s) > gap_m) + 1
        used_any = False
        band = np.abs((p - c) @ n) < clear_tol * line_tol_m   # the wall's whole noise band, not just the inliers
        s_all = (p - c) @ u
        for run in np.split(np.arange(len(s)), cuts):
            if len(run) < min_points or s[run[-1]] - s[run[0]] < min_length_m:
                continue
            rp_ = pts[qi[run]]
            cc = rp_.mean(0)
            _, _, vv = np.linalg.svd(rp_ - cc)
            uu, nn = vv[0], vv[1]
            ss = (rp_ - cc) @ uu
            if np.mean((view[qi[run]] - cc) @ nn) < 0:
                nn = -nn
            resid = (rp_ - cc) @ nn
            segments.append(Segment(cc + ss.min() * uu, cc + ss.max() * uu, nn, len(run),
                                    float(np.sqrt(np.mean(resid ** 2)))))
            used_any = True
            left[idx[band & (s_all >= s[run[0]] - line_tol_m) & (s_all <= s[run[-1]] + line_tol_m)]] = False
        left[qi] = False                                      # short runs are clutter, not lines
        if not used_any and best_n < 2 * min_points:
            break
    blobs = _blobs(pts[left], pmap.cell)
    return segments, blobs


def _blobs(pts: np.ndarray, cell: float, min_cells: int = 5, join_m: float = 0.08) -> List[np.ndarray]:
    if len(pts) < min_cells:
        return []
    import cv2

    x0, y0 = pts.min(0) - 0.2
    ix = ((pts[:, 0] - x0) / cell).astype(int)
    iy = ((pts[:, 1] - y0) / cell).astype(int)
    img = np.zeros((iy.max() + 10, ix.max() + 10), np.uint8)
    img[iy, ix] = 255
    r = max(1, int(round(join_m / cell)))
    joined = cv2.dilate(img, np.ones((2 * r + 1, 2 * r + 1), np.uint8))
    n, labels = cv2.connectedComponents(joined)
    out = []
    lab = labels[iy, ix]
    for k in range(1, n):
        q = pts[lab == k]
        if len(q) < min_cells:
            continue
        hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2).astype(float)
        if len(hull) < 3 or cv2.contourArea(hull.astype(np.float32)) < cell * cell:
            c = q.mean(0)  # a sliver: a small square around it
            h = max(cell, 0.5 * float(np.ptp(q, axis=0).max()))
            hull = np.array([c + [-h, -h], c + [h, -h], c + [h, h], c + [-h, h]])
        out.append(hull)
    return out
