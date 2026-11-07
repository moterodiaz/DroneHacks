"""Lidar localiser: one MS200K revolution in, the body pose in the course frame and its covariance out.

This is the localiser that :mod:`pose_fusion.fusion` expects upstream. The matching itself is the
ported, bench-tested ``course/lidar_localize.py``:

* Gauss-Newton, point-to-face against the map faces each beam actually crosses;
* Huber weights and an outlier cut;
* deskew: each beam is placed where the drone was when it fired;
* tilt: beams are rotated by roll and pitch, and those that hit the floor or passed over the
  walls are dropped;
* ``Tracker`` guards (``LIVE_GUARDS``): a poor fit, or a jump, counts as "lost", never as a pose.

This module adds what a flight needs around it:

* **map slices:** distance fields at several heights. Each revolution is matched against the
  slice nearest its scan plane, with the band of beam heights that slice describes. This is
  the scheme of the old ``course/19_lidar_px4_bridge.py``, which commanded PX4 directly and is
  not ported; its constants are kept.
* **PX4 state:** roll and pitch per beam, horizontal velocity and yaw rate for the deskew, and
  the body height for picking the slice. All of it comes from ``/fmu/out/vehicle_odometry``,
  converted here to frame-independent terms. PX4's heading reference is arbitrary
  (``pose_frame = FRD`` in the kit's bridge), so its velocity is resolved in the body's own
  heading frame, then turned by the lidar's course heading. No PX4-to-course rotation is
  assumed.
* **covariance:** ``cov_scale * sigma^2 * (J^T W J)^-1`` from the matcher's last iteration,
  through an eigendecomposition. A direction the scan barely constrains gets a large variance,
  capped at ``max_variance``, instead of a confident guess. ``cov_scale`` covers the fact that
  map and scan errors are correlated, which the textbook formula ignores (it would give
  millimetres).
* **start-up gate:** nothing is published until ``startup_good`` trusted matches in a row
  (3, as on the bench), and again after the tracker re-acquires.

Clock: every time passed in (beam times, PX4 sample times) must be on one clock. The node uses
the ROS clock.

numpy only, plus the ``course`` package; no ROS import.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, fields
from typing import Any, Deque, Dict, Optional, Sequence, Tuple

import numpy as np

from .course_files import wall_and_net_tops

#: Scan-plane heights (m) with a map slice, from course/19_lidar_px4_bridge.py.
SLICE_HEIGHTS = (0.2, 0.4, 0.65, 0.9, 1.2, 1.5, 2.0, 2.4)
#: Gauss-Newton iterations per revolution. 19's measurement: bench recordings never needed more
#: than 7, and each costs about 6 ms on the Jetson, so 8 bounds a match at about 50 ms.
MATCH_ITERATIONS = 8
#: Trusted matches in a row before the first pose goes out (19: STARTUP_GOOD).
STARTUP_GOOD = 3


class MapSlices:
    """Distance fields at several heights; :meth:`pick` returns the one nearest a scan plane.

    The band is the range of beam end heights a slice describes. Below the wall tops it runs from
    8 cm above the floor to 5 cm under the wall tops; above them, it runs from 5 cm over the wall
    tops, where only the nets are left. Matching a wall hit against a nets-only slice put the
    heading 28 deg off in SITL (19's notes).
    """

    FLOOR_M, EDGE_M = 0.08, 0.05

    def __init__(self, cmap: Any, heights: Sequence[float], wall_top: float, net_top: float) -> None:
        from course.lidar_localize import DistanceField  # lazy: building fields is the slow part

        self.z = np.array(heights, dtype=float)
        if self.z.ndim != 1 or len(self.z) == 0:
            raise ValueError("need at least one slice height")
        self.fields = [DistanceField(cmap, float(z)) for z in self.z]
        self.tops = [wall_top if z < wall_top else net_top for z in self.z]
        self.bands = [(self.FLOOR_M, wall_top - self.EDGE_M) if z < wall_top else (wall_top + self.EDGE_M, net_top)
                      for z in self.z]

    def pick(self, scan_z: float) -> Tuple[Any, float, float, Tuple[float, float]]:
        """(field, wall top for its beams, slice height, beam band) for a scan plane at *scan_z*."""
        k = int(np.argmin(np.abs(self.z - scan_z)))
        return self.fields[k], self.tops[k], float(self.z[k]), self.bands[k]


@dataclass(frozen=True)
class LocalizerConfig:
    """Mount and rest height are measured per drone and have no default.

    * ``mount_xyz`` - the lidar's scan centre in the body frame (FLU), m. course/docs/hardware-setup.md
      section 2.4 explains how to measure it.
    * ``rest_height_m`` - the body centre above the floor when landed (contracts/frames.md uses a
      tape-measured value).
    * ``cov_scale`` - variance multiplier on the Gauss-Newton covariance. 100 (sigma x 10) is a
      guess until bench truth calibrates it; course/docs/bench-results.md shows lidar-vs-mark
      errors of 5-12 cm, limited by hand placement.
    """

    mount_xyz: Tuple[float, float, float]
    rest_height_m: float
    slice_heights: Tuple[float, ...] = SLICE_HEIGHTS
    iterations: int = MATCH_ITERATIONS
    startup_good: int = STARTUP_GOOD
    cov_scale: float = 100.0
    max_variance: float = 1.0e4
    max_px4_age_s: float = 0.3

    def __post_init__(self) -> None:
        if len(self.mount_xyz) != 3 or not all(math.isfinite(float(v)) for v in self.mount_xyz):
            raise ValueError("mount_xyz must be three finite numbers (measure the lidar mount; there is no default)")
        if not (math.isfinite(self.rest_height_m) and self.rest_height_m >= 0.0):
            raise ValueError("rest_height_m must be a finite height >= 0 (measured; there is no default)")
        for f in fields(self):
            if f.name in ("mount_xyz", "rest_height_m", "slice_heights"):
                continue
            value = getattr(self, f.name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0):
                raise ValueError("LocalizerConfig.{} must be > 0 (got {!r})".format(f.name, value))


@dataclass(frozen=True)
class Px4Sample:
    """One ``VehicleOdometry`` sample, reduced to what the localiser uses."""

    t: float                         # receive time, on the scan's clock
    roll_flu: float                  # rad
    pitch_flu: float                 # rad (FRD pitch negated)
    yaw_ned: float                   # rad, PX4's heading (arbitrary reference)
    z_ned: float                     # m, down
    vel_ned: Optional[Tuple[float, float, float]]   # None: PX4 sent no NED velocity
    yaw_rate_flu: float              # rad/s about body up (FRD z rate negated)


def px4_sample_from_odometry(t: float, q_wxyz: Sequence[float], position_ned: Sequence[float],
                             velocity_ned: Optional[Sequence[float]], angular_velocity_frd: Sequence[float]) -> Px4Sample:
    """A :class:`Px4Sample` from ``VehicleOdometry`` fields. PX4 quaternions are ``[w, x, y, z]``,
    FRD body to NED; velocity must be in NED (``VELOCITY_FRAME_NED``), or ``None``."""
    w, x, y, z = (float(v) for v in q_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    vel = None if velocity_ned is None else tuple(float(v) for v in velocity_ned)
    if vel is not None and not all(math.isfinite(v) for v in vel):
        vel = None
    return Px4Sample(float(t), roll, -pitch, yaw, float(position_ned[2]), vel, -float(angular_velocity_frd[2]))


def course_velocity(sample: Px4Sample, course_yaw: float) -> Optional[np.ndarray]:
    """PX4's horizontal velocity in the course frame, using only the lidar's course heading.

    The NED velocity is resolved into the body's heading frame (forward, left) with PX4's own
    yaw, then turned by *course_yaw*. PX4's heading reference never enters, so it can be anything.
    Returns None without a PX4 velocity; the tracker then deskews with its own estimate.
    """
    if sample.vel_ned is None:
        return None
    vn, ve = sample.vel_ned[0], sample.vel_ned[1]
    c, s = math.cos(sample.yaw_ned), math.sin(sample.yaw_ned)
    forward, right = c * vn + s * ve, -s * vn + c * ve
    left = -right
    cc, sc = math.cos(course_yaw), math.sin(course_yaw)
    return np.array([cc * forward - sc * left, sc * forward + cc * left])


class Px4Buffer:
    """Recent :class:`Px4Sample` s: roll/pitch interpolated per beam, the rest from the latest sample."""

    def __init__(self, horizon_s: float = 2.0) -> None:
        self.horizon_s = horizon_s
        self._samples: Deque[Px4Sample] = deque()
        self.z0_ned: Optional[float] = None

    def add(self, sample: Px4Sample) -> None:
        if self._samples and sample.t <= self._samples[-1].t:
            return
        if self.z0_ned is None:
            self.z0_ned = sample.z_ned            # the floor: the localiser starts with the drone resting on it
        self._samples.append(sample)
        while self._samples and self._samples[0].t < sample.t - self.horizon_s:
            self._samples.popleft()

    def latest(self) -> Optional[Px4Sample]:
        return self._samples[-1] if self._samples else None

    def attitude_at(self, t: np.ndarray) -> np.ndarray:
        """(n, 2) roll, pitch (FLU) at the times *t*, held at the ends of the buffer."""
        ts = np.array([s.t for s in self._samples])
        roll = np.array([s.roll_flu for s in self._samples])
        pitch = np.array([s.pitch_flu for s in self._samples])
        return np.stack([np.interp(t, ts, roll), np.interp(t, ts, pitch)], axis=1)

    def body_height(self, rest_height_m: float) -> float:
        """Body height above the floor: the rest height plus how far PX4 has climbed since its first sample."""
        latest = self.latest()
        if latest is None or self.z0_ned is None:
            return rest_height_m
        return max(0.0, rest_height_m + (self.z0_ned - latest.z_ned))


def match_covariance(stats: Dict[str, Any], scale: float, max_variance: float) -> Optional[np.ndarray]:
    """``scale * sigma^2 * information^-1`` (x, y, yaw), with each principal variance capped at *max_variance*."""
    info = stats.get("information")
    sigma2 = stats.get("sigma2_m2")
    if info is None or sigma2 is None:
        return None
    info = np.asarray(info, dtype=float)
    if info.shape != (3, 3) or not np.all(np.isfinite(info)) or not math.isfinite(sigma2):
        return None
    info = 0.5 * (info + info.T)
    eigval, eigvec = np.linalg.eigh(info)
    var = scale * max(sigma2, 0.0) / np.maximum(eigval, 1e-12)
    var = np.minimum(var, max_variance)
    var = np.maximum(var, 1e-12)
    return (eigvec * var) @ eigvec.T


@dataclass
class LocalizerResult:
    t: float                                  # the scan's reference time (its last beam)
    x: float                                  # course frame, m
    y: float
    yaw: float                                # course frame, counter-clockwise from east, rad
    z_body: float                             # m above the floor (from PX4; not measured by the lidar)
    publish: bool
    cov: Optional[np.ndarray] = None          # 3x3 (x, y, yaw), course frame
    reason: Optional[str] = None              # why it is not published
    stats: Dict[str, Any] = field(default_factory=dict)


class LidarLocalizer:
    """Revolution after revolution, from the launch mark. Make a new one for each run (rule 6.4)."""

    def __init__(self, cmap: Any, start_xy: Tuple[float, float], config: LocalizerConfig, start_yaw: float,
                 slices: Optional[MapSlices] = None) -> None:
        """*start_xy* / *start_yaw* are the drone's startup pose guess in the course frame. The
        yaw is the measured course->odom yaw (contracts/frames.md "Course → odom", issue #29,
        recommendation B): cuVSLAM's odom frame starts on the body, so that measured yaw is the
        drone's startup course heading. It has no default - not the north the launch heading
        suggests; pass the measured value (``pose_fusion.frames.bearing_to_course_yaw`` converts
        an operator's compass bearing)."""
        from course.lidar_localize import LIVE_GUARDS, MatchConfig, Tracker

        self.config = config
        wall_top, net_top = wall_and_net_tops(cmap)
        self.slices = slices if slices is not None else MapSlices(cmap, config.slice_heights, wall_top, net_top)
        mount = tuple(float(v) for v in config.mount_xyz)
        fld, top, _, band = self.slices.pick(config.rest_height_m + mount[2])
        self.tracker = Tracker(fld, MatchConfig(deskew=True, tilt=True, mount=mount, iterations=config.iterations),
                               np.array([start_xy[0], start_xy[1], start_yaw], dtype=float), wall_top=top, **LIVE_GUARDS)
        self.tracker.band = band
        self.streak = 0
        self.ever_published = False
        self.counts = {"revolutions": 0, "published": 0, "lost": 0, "no_px4": 0, "no_covariance": 0}

    def update(self, t: np.ndarray, ranges: np.ndarray, angles: np.ndarray, px4: Px4Buffer) -> LocalizerResult:
        """Match one revolution: beam times *t* (s), *ranges* (m, NaN = none), *angles* (body FLU, rad)."""
        cfg = self.config
        self.counts["revolutions"] += 1
        t = np.asarray(t, dtype=float)
        t_ref = float(t[-1])
        latest = px4.latest()
        pose = self.tracker.pose
        if latest is None or abs(t_ref - latest.t) > cfg.max_px4_age_s:
            self.counts["no_px4"] += 1
            self.streak = 0
            return LocalizerResult(t_ref, float(pose[0]), float(pose[1]), float(pose[2]),
                                   px4.body_height(cfg.rest_height_m), False, reason="no fresh PX4 attitude")
        rp = px4.attitude_at(t)
        z_body = px4.body_height(cfg.rest_height_m)
        tr = self.tracker
        tr.field, tr.wall_top, slice_z, tr.band = self.slices.pick(z_body + float(cfg.mount_xyz[2]))
        vel = course_velocity(latest, float(tr.pose[2]))
        pose, st = tr.update(t, np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float), rp, z_body,
                             vel, latest.yaw_rate_flu)
        st["slice_z"] = slice_z
        good = "lost" not in st
        self.streak = self.streak + 1 if good else 0
        if st.get("acquired") and self.ever_published:
            self.ever_published = False            # re-acquired after a loss: earn the start-up streak again
        result = LocalizerResult(t_ref, float(pose[0]), float(pose[1]), float(pose[2]), z_body, False, stats=st)
        if not good:
            self.counts["lost"] += 1
            result.reason = "lost: {}".format(st.get("reason", "?"))
            return result
        cov = match_covariance(st, cfg.cov_scale, cfg.max_variance)
        if cov is None:
            self.counts["no_covariance"] += 1
            result.reason = "no covariance"
            return result
        result.cov = cov
        if self.streak >= cfg.startup_good or self.ever_published:
            result.publish = True
            self.ever_published = True
            self.counts["published"] += 1
        else:
            result.reason = "start-up: {} of {} good matches".format(self.streak, cfg.startup_good)
        return result
