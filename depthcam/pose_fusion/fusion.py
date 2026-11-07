"""Lidar-corrected visual odometry: the drift estimate behind the corrected flight commands.

Where it sits:

    cuVSLAM ─┬──────────────────────────────► kit px4_odom_bridge.py ─► EKF2   (organisers', unchanged)
             └► Guard ─► PoseFusion ─► corrected pose (true frame) ─► Follower
                             ▲     └─► C⁻¹ ∘ Follower setpoint ─► /skybeta/setpoint/* (pose_fusion.setpoints)
    360 lidar ─► localiser ──┘   (body pose in the course frame, scan-matched to the published map)

EKF2 has exactly one external-vision input, and the organisers' bridge feeds it from cuVSLAM
with no room for team code in front. So PX4 keeps flying on raw cuVSLAM. This stage uses the lidar
to estimate the drift ``C`` of the visual odometry (VO). Our nodes then use ``C`` twice: to
publish the drone's true pose to the Follower, and to correct the Follower's setpoints into PX4's
frame. The corrected pose keeps the Guard's message and covariance.

Model
-----
VO is smooth but drifts. A lidar fix doesn't drift, but it is noisier, slower (about 10 Hz) and
arrives late. The filter estimates one planar transform ``C = (tx, ty, yaw)`` from the VO frame to
the tracker frame of contracts/frames.md, such that ``truth ≈ C ∘ VO``:

* **predict**, on every VO sample: ``C`` is a random walk. Its noise grows with the distance
  travelled and the angle turned since the last sample (that is how VO drift builds up), plus a
  small term per second while hovering. The noise pivots at the drone's current position,
  because drift that builds up here bends the path from here on, not from the origin.
* **update**, on every lidar fix: the measurement model is ``z = C ∘ VO(t_fix)``, with VO
  interpolated at the fix's own timestamp, so lidar latency costs nothing. Each fix passes a
  3-DOF chi-square gate, then gets a Joseph-form EKF update.
* **output**: ``C`` is never applied in one step. A second transform, ``C_applied``, follows ``C``
  with the displacement it causes at the drone limited to ``max_correction_speed_mps`` and
  ``max_correction_yaw_rate_rps``. Neither the published pose nor the corrected setpoints ever
  jump; a setpoint jump would kick PX4's position controller.
* **re-acquire**: ``reacquire_after`` gated fixes in a row that agree with each other (within
  ``reacquire_tol_m`` / ``reacquire_tol_rad`` at the drone) mean ``C`` is wrong, not the lidar.
  That happens when cuVSLAM re-initialises and its frame jumps, or when the drone was placed
  further off the mark than the prior allows. ``C`` is then reset to what they agree on, and
  ``C_applied`` walks there at the slew limits.

Only x, y and yaw are corrected. z, roll and pitch pass through unchanged: a 2D lidar does not
observe them, and PX4 takes height from the rangefinder (``EKF2_HGT_REF = 2``).

Frame breaks: :meth:`PoseFusion.aligned`
----------------------------------------
When cuVSLAM re-initialises, its frame jumps. PX4's estimate jumps with it, but ``C_applied`` is
still the old frame's correction, so the corrected pose and the corrected setpoints are wrong by
the jump until ``C`` is re-learnt and applied. Flying the route on them would steer the drone
off its path near obstacles. So :meth:`PoseFusion.aligned` turns false:

* at every VO gap, because cuVSLAM may have re-initialised in it;
* after ``suspect_after_gated`` gated fixes in a row, because the lidar no longer agrees with
  ``C ∘ VO`` (a frame that jumped too little for the Guard to break, or a lidar fault).

It turns true again only once a lidar fix has passed the gate since then, and ``C_applied`` is
within ``aligned_tol_m`` / ``aligned_tol_rad`` of ``C`` at the drone. The follower holds still
by velocity while it is false (``follower.mission``). A zero velocity setpoint doesn't depend on
the correction, so while the follower holds, the node applies ``C`` at once
(:meth:`PoseFusion.snap_to_estimate`) instead of walking it at the slew limits.

VO gaps
-------
When the Guard withholds poses (blank view, tracking loss), no VO arrives. The Guard passes raw
cuVSLAM poses, in PX4's own ``odom`` frame, and never re-anchors them. So after a gap longer than
``vo_gap_s``, ``C`` is kept: the setpoints don't move. Its uncertainty grows for the coast, and
is held at least at ``gap_floor_*``, in case cuVSLAM re-initialised during the gap. The
re-acquire rule catches a frame that really jumped. ``vo_gap_s`` must be shorter than the Guard's
settle time (0.2 s), so every Guard break counts as a gap. ``reanchor_on_gap = True`` keeps the
older behaviour for a Guard that re-anchors on PX4's estimate in this stage's output frame. That
only applies when this stage feeds PX4's vision input, which the issued drone does not allow.

The lidar policy
----------------
Under :attr:`LidarPolicy.FORBIDDEN`, :meth:`PoseFusion.add_vo` is an exact pass-through and
:meth:`PoseFusion.add_lidar` raises :class:`PolicyViolation`. A run where the lidar is not
allowed can't be influenced by it, even through a wiring mistake.

numpy only; no ROS import. The ROS 2 node is ``methods/lidar-vo-fusion/11_pose_fusion_node.py``.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import asdict, dataclass, fields
from typing import Dict, List, Optional

import numpy as np

from .policy import LidarPolicy
from .se2 import IDENTITY, Pose2, Quat, interpolate, quat_yaw, rotate_quat_about_z, wrap_angle

#: 99 % point of the chi-square distribution with 3 degrees of freedom (x, y, yaw innovation).
CHI2_3DOF_99 = 11.345

#: At most this many lidar fixes wait for VO to cover their timestamps; the oldest are dropped.
MAX_PENDING_FIXES = 50


class PolicyViolation(RuntimeError):
    """A lidar fix reached the fusion in a run where the lidar is forbidden."""


@dataclass(frozen=True)
class FusionConfig:
    """Tuning values. Every default is a starting point, not a measurement, because nothing has flown with this yet.

    Drift (process noise of the VO -> tracker correction):

    * ``drift_pos_per_m`` - 1-sigma position drift per metre travelled (0.05 = 5 %; the
      cuvslam-vio walks drifted 2.3-30 % of the path, back to start)
    * ``drift_yaw_per_rad`` - 1-sigma heading drift per radian turned
    * ``hover_pos_sigma_per_sqrt_s``, ``hover_yaw_sigma_per_sqrt_s`` - random walk while not moving

    Start:

    * ``prior_pos_sigma_m``, ``prior_yaw_sigma_rad`` - how well the drone is placed on the launch
      mark, by hand, facing north by eye. A placement error that pushes the first innovation
      past ``gate_chi2`` (combined over x, y and yaw) is gated out for the whole run; see
      ``consecutive_gated`` in the status.

    Lidar fixes:

    * ``gate_chi2`` - Mahalanobis gate on the (x, y, yaw) innovation
    * ``min_lidar_sigma_pos_m``, ``min_lidar_sigma_yaw_rad`` - floors on the reported lidar
      uncertainty (scan matchers are usually overconfident)
    * ``max_lidar_wait_s`` - a fix stamped further ahead of the newest VO sample than this is
      dropped (it points to a clock mismatch)
    * ``vo_buffer_s`` - how much VO history is kept to interpolate at late fixes; it must be
      longer than the lidar pipeline's latency

    Output (what EKF2 sees):

    * ``max_correction_speed_mps``, ``max_correction_yaw_rate_rps`` - the fastest the correction
      may move the published pose

    VO gaps (Guard withholds):

    * ``vo_gap_s`` - a longer silence counts as a gap; keep it below the Guard's settle time
    * ``reanchor_on_gap`` - False: keep ``C`` (the Guard passes PX4's frame). True: the older
      behaviour for a Guard that re-anchors (see the module docstring)
    * ``gap_pos_sigma_per_s``, ``gap_yaw_sigma_per_s`` - uncertainty added per second of coasting
    * ``gap_floor_pos_sigma_m``, ``gap_floor_yaw_sigma_rad`` - least uncertainty after any gap

    Re-acquiring (``C`` is wrong, not the lidar):

    * ``reacquire_after`` - gated fixes in a row before they are considered
    * ``reacquire_tol_m``, ``reacquire_tol_rad`` - how closely they must agree, at the drone

    Frame breaks (see :meth:`PoseFusion.aligned`):

    * ``suspect_after_gated`` - gated fixes in a row that mean the frames may disagree
    * ``aligned_tol_m``, ``aligned_tol_rad`` - how close ``C_applied`` must be to ``C``, at the
      drone, for the frames to count as agreeing again
    """

    drift_pos_per_m: float = 0.05
    drift_yaw_per_rad: float = 0.03
    hover_pos_sigma_per_sqrt_s: float = 0.01
    hover_yaw_sigma_per_sqrt_s: float = math.radians(0.2)
    prior_pos_sigma_m: float = 0.10
    prior_yaw_sigma_rad: float = math.radians(5.0)
    gate_chi2: float = CHI2_3DOF_99
    min_lidar_sigma_pos_m: float = 0.02
    min_lidar_sigma_yaw_rad: float = math.radians(0.5)
    max_lidar_wait_s: float = 0.15
    vo_buffer_s: float = 2.0
    max_correction_speed_mps: float = 0.10
    max_correction_yaw_rate_rps: float = math.radians(3.0)
    vo_gap_s: float = 0.15
    reanchor_on_gap: bool = False
    gap_pos_sigma_per_s: float = 0.10
    gap_yaw_sigma_per_s: float = math.radians(2.0)
    gap_floor_pos_sigma_m: float = 0.10
    gap_floor_yaw_sigma_rad: float = math.radians(5.0)
    reacquire_after: int = 5
    reacquire_tol_m: float = 0.05
    reacquire_tol_rad: float = math.radians(2.0)
    suspect_after_gated: int = 2
    aligned_tol_m: float = 0.05
    aligned_tol_rad: float = math.radians(2.0)

    def __post_init__(self) -> None:
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool):
                continue
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0):
                raise ValueError("FusionConfig.{} must be a finite number > 0 (got {!r})".format(f.name, value))


@dataclass(frozen=True)
class VoSample:
    """One visual-odometry pose from the Guard: time (s, ROS clock), position (m), orientation."""

    t: float
    x: float
    y: float
    z: float
    q: Quat

    def planar(self) -> Pose2:
        return Pose2(self.x, self.y, quat_yaw(self.q))

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.t, self.x, self.y, self.z) + tuple(self.q))


@dataclass(frozen=True, eq=False)
class LidarFix:
    """One lidar localisation: the body pose in the tracker frame and its 3x3 (x, y, yaw) covariance.

    Convert the localiser's course-frame message with :func:`pose_fusion.ros_io.lidar_pose_to_fix`.
    ``t`` is the scan time after deskewing, on the same clock as the VO.
    """

    t: float
    pose: Pose2
    cov: np.ndarray


@dataclass(frozen=True)
class FusedPose:
    """The pose to publish: the VO sample with ``correction`` applied in the plane."""

    t: float
    x: float
    y: float
    z: float
    q: Quat
    correction: Pose2


@dataclass
class FusionStats:
    vo_received: int = 0
    vo_out_of_order: int = 0
    vo_gaps: int = 0
    lidar_received: int = 0
    lidar_fused: int = 0
    lidar_gated: int = 0
    consecutive_gated: int = 0
    lidar_stale: int = 0
    lidar_future: int = 0
    lidar_dropped: int = 0
    lidar_invalid: int = 0
    lidar_refused: int = 0
    reacquired: int = 0


def apply_correction(sample: VoSample, correction: Pose2) -> FusedPose:
    """*sample* with the planar *correction* applied: x, y and yaw change; z, roll and pitch do not.

    The identity correction returns the sample's values bit for bit.
    """
    c, s = math.cos(correction.yaw), math.sin(correction.yaw)
    x = c * sample.x - s * sample.y + correction.x
    y = s * sample.x + c * sample.y + correction.y
    return FusedPose(sample.t, x, y, sample.z, rotate_quat_about_z(sample.q, correction.yaw), correction)


def floor_covariance(cov: np.ndarray, min_pos_sigma: float, min_yaw_sigma: float) -> np.ndarray:
    """*cov* (3x3, symmetrised) with its diagonal raised to at least the given standard deviations."""
    cov = np.asarray(cov, dtype=float)
    cov = 0.5 * (cov + cov.T)
    floors = np.array([min_pos_sigma ** 2, min_pos_sigma ** 2, min_yaw_sigma ** 2])
    return cov + np.diag(np.maximum(0.0, floors - np.diag(cov)))


def _pose_dict(pose: Pose2) -> Dict[str, float]:
    return {"x_m": pose.x, "y_m": pose.y, "yaw_deg": math.degrees(pose.yaw)}


class PoseFusion:
    """The drift filter for one run. Make a new one for every run: it never carries state across runs (rule 6.4)."""

    def __init__(self, policy: LidarPolicy, config: Optional[FusionConfig] = None) -> None:
        if not isinstance(policy, LidarPolicy):
            raise TypeError("policy must be a LidarPolicy (see pose_fusion.policy.parse_lidar_policy)")
        self.policy = policy
        self.config = config if config is not None else FusionConfig()
        cfg = self.config
        self._c = np.zeros(3)
        self._p = np.diag([cfg.prior_pos_sigma_m ** 2, cfg.prior_pos_sigma_m ** 2, cfg.prior_yaw_sigma_rad ** 2])
        self._applied = IDENTITY
        self._buf_t: List[float] = []
        self._buf_pose: List[Pose2] = []
        self._pending: List[LidarFix] = []
        self._last_vo: Optional[VoSample] = None
        self._last_fix_t: Optional[float] = None
        self._last_innovation: Optional[List[float]] = None
        self._last_d2: Optional[float] = None
        self._gated: List[Pose2] = []          # the correction each gated fix in a row implies
        self._suspect = False                  # a VO gap or gated fixes since the last fused fix
        self.stats = FusionStats()

    # ------------------------------------------------------------------ public API

    @property
    def lidar_allowed(self) -> bool:
        return self.policy is LidarPolicy.ALLOWED

    @property
    def correction_applied(self) -> Pose2:
        """The correction applied to the latest output."""
        return self._applied

    @property
    def correction_estimate(self) -> Pose2:
        """The filter's current estimate of the correction, which the output follows."""
        return self._c_pose()

    @property
    def covariance(self) -> np.ndarray:
        return self._p.copy()

    def add_vo(self, sample: VoSample) -> Optional[FusedPose]:
        """Feed one Guard pose; returns the pose to publish, or ``None`` for an out-of-order sample.

        Under the forbidden policy this is an exact pass-through and nothing is filtered.
        """
        self.stats.vo_received += 1
        if not self.lidar_allowed:
            return FusedPose(sample.t, sample.x, sample.y, sample.z, sample.q, IDENTITY)
        if not sample.is_finite():
            raise ValueError("non-finite VO sample at t={!r}".format(sample.t))
        planar = sample.planar()
        last = self._last_vo
        dt = 0.0
        if last is not None:
            dt = sample.t - last.t
            if dt <= 0.0:
                self.stats.vo_out_of_order += 1
                return None
            if dt > self.config.vo_gap_s:
                self._on_gap(dt, last.planar())
                dt = 0.0
            else:
                self._predict(last.planar(), planar, dt)
        self._buffer_vo(sample.t, planar)
        self._last_vo = sample
        self._fuse_pending()
        self._slew(planar, dt)
        return apply_correction(sample, self._applied)

    def add_lidar(self, fix: LidarFix) -> None:
        """Feed one lidar localisation. It is fused as soon as VO covers its timestamp."""
        if not self.lidar_allowed:
            self.stats.lidar_refused += 1
            raise PolicyViolation("lidar fix received in a run where lidar_policy is 'forbidden'")
        self.stats.lidar_received += 1
        cov = np.asarray(fix.cov, dtype=float)
        if (cov.shape != (3, 3) or not np.all(np.isfinite(cov))
                or not all(math.isfinite(v) for v in (fix.t,) + fix.pose.as_tuple())):
            self.stats.lidar_invalid += 1
            return
        if self._buf_t and fix.t > self._buf_t[-1] + self.config.max_lidar_wait_s:
            self.stats.lidar_future += 1
            return
        self._pending.append(LidarFix(fix.t, fix.pose, cov))
        self._pending.sort(key=lambda f: f.t)
        if len(self._pending) > MAX_PENDING_FIXES:
            excess = len(self._pending) - MAX_PENDING_FIXES
            self._pending = self._pending[excess:]
            self.stats.lidar_dropped += excess
        self._fuse_pending()

    def settled(self, max_lidar_age_s: float = 0.5, tol_m: float = 0.02, tol_rad: float = math.radians(0.5),
                min_fused: int = 10) -> bool:
        """Whether the correction is ready to fly on: always under the forbidden policy. When allowed,
        at least *min_fused* fixes fused, the last no older than *max_lidar_age_s* (VO time), and the
        applied correction within *tol_m* / *tol_rad* of the estimate at the drone."""
        if not self.lidar_allowed:
            return True
        if self._suspect or self.stats.lidar_fused < min_fused or self._last_vo is None or self._last_fix_t is None:
            return False
        if self._last_vo.t - self._last_fix_t > max_lidar_age_s:
            return False
        return self._applied_within(tol_m, tol_rad)

    def aligned(self) -> bool:
        """Whether the corrected pose and setpoints can be flown on: always under the forbidden policy.

        False from a VO gap, or ``suspect_after_gated`` gated fixes in a row, until a fix passes the
        gate again and ``C_applied`` is within ``aligned_tol_*`` of ``C`` (the module docstring,
        "Frame breaks"). A lidar that only goes quiet doesn't make it false: VO alone flies on.
        """
        if not self.lidar_allowed:
            return True
        if self._suspect:
            return False
        return self._applied_within(self.config.aligned_tol_m, self.config.aligned_tol_rad)

    def snap_to_estimate(self) -> None:
        """Apply ``C`` at once, skipping the slew limits.

        Only for while no command PX4 gets depends on the correction: the follower holding still
        with a zero velocity setpoint. Then the published pose jumps, but nothing PX4 is told does.
        """
        if self.lidar_allowed:
            self._applied = self._c_pose()

    def _applied_within(self, tol_m: float, tol_rad: float) -> bool:
        if self._last_vo is None:
            return True
        here = self._last_vo.planar()
        target = self._c_pose().compose(here)
        current = self._applied.compose(here)
        return (math.hypot(target.x - current.x, target.y - current.y) <= tol_m
                and abs(wrap_angle(target.yaw - current.yaw)) <= tol_rad)

    def status(self) -> Dict[str, object]:
        """Counters and the current correction, for the node's ~/status topic and for logs."""
        out: Dict[str, object] = {"lidar_policy": self.policy.value, "ready": self.settled(),
                                  "aligned": self.aligned()}
        out.update(asdict(self.stats))
        if not self.lidar_allowed:
            return out
        sigma = np.sqrt(np.clip(np.diag(self._p), 0.0, None))
        out["correction_applied"] = _pose_dict(self._applied)
        out["correction_estimate"] = _pose_dict(self._c_pose())
        out["correction_sigma"] = {"x_m": float(sigma[0]), "y_m": float(sigma[1]),
                                   "yaw_deg": math.degrees(float(sigma[2]))}
        out["pending_fixes"] = len(self._pending)
        out["lidar_age_s"] = (self._last_vo.t - self._last_fix_t
                              if self._last_vo is not None and self._last_fix_t is not None else None)
        out["last_innovation"] = self._last_innovation
        out["last_d2"] = self._last_d2
        return out

    # ------------------------------------------------------------------ filter internals

    def _c_pose(self) -> Pose2:
        return Pose2(float(self._c[0]), float(self._c[1]), float(self._c[2]))

    def _buffer_vo(self, t: float, pose: Pose2) -> None:
        self._buf_t.append(t)
        self._buf_pose.append(pose)
        cutoff = t - self.config.vo_buffer_s
        drop = bisect.bisect_left(self._buf_t, cutoff)
        if drop:
            del self._buf_t[:drop]
            del self._buf_pose[:drop]

    def _vo_at(self, t: float) -> Pose2:
        """VO interpolated at *t*, which must lie within the buffer."""
        i = bisect.bisect_left(self._buf_t, t)
        if self._buf_t[i] == t:
            return self._buf_pose[i]
        t0, t1 = self._buf_t[i - 1], self._buf_t[i]
        return interpolate(self._buf_pose[i - 1], self._buf_pose[i], (t - t0) / (t1 - t0))

    def _inflate(self, var_pos: float, var_yaw: float, pivot: Pose2) -> None:
        """Add process noise that pivots at *pivot*, the drone's position in the tracker frame."""
        tx, ty = float(self._c[0]), float(self._c[1])
        g = np.array([[1.0, 0.0, pivot.y - ty],
                      [0.0, 1.0, tx - pivot.x],
                      [0.0, 0.0, 1.0]])
        self._p = self._p + g @ np.diag([var_pos, var_pos, var_yaw]) @ g.T

    def _predict(self, prev: Pose2, cur: Pose2, dt: float) -> None:
        cfg = self.config
        dist = math.hypot(cur.x - prev.x, cur.y - prev.y)
        turn = abs(wrap_angle(cur.yaw - prev.yaw))
        var_pos = (cfg.drift_pos_per_m * dist) ** 2 + cfg.hover_pos_sigma_per_sqrt_s ** 2 * dt
        var_yaw = (cfg.drift_yaw_per_rad * turn) ** 2 + cfg.hover_yaw_sigma_per_sqrt_s ** 2 * dt
        self._inflate(var_pos, var_yaw, self._c_pose().compose(cur))

    def _on_gap(self, dt: float, last_planar: Pose2) -> None:
        cfg = self.config
        self.stats.vo_gaps += 1
        self.stats.lidar_dropped += len(self._pending)
        self._pending = []
        self._buf_t.clear()
        self._buf_pose.clear()
        pivot = self._applied.compose(last_planar)        # where the output last put the drone
        if cfg.reanchor_on_gap:
            self._c = np.array(self._c_pose().compose(self._applied.inverse()).as_tuple())
            self._applied = IDENTITY
        self._inflate((cfg.gap_pos_sigma_per_s * dt) ** 2, (cfg.gap_yaw_sigma_per_s * dt) ** 2, pivot)
        # At least this uncertain after any gap: cuVSLAM may have re-initialised in it
        floors = np.array([cfg.gap_floor_pos_sigma_m ** 2, cfg.gap_floor_pos_sigma_m ** 2,
                           cfg.gap_floor_yaw_sigma_rad ** 2])
        self._p = self._p + np.diag(np.maximum(0.0, floors - np.diag(self._p)))
        self._gated = []
        self._suspect = True

    def _fuse_pending(self) -> None:
        if not self._buf_t:
            return
        keep: List[LidarFix] = []
        for fix in self._pending:
            if fix.t < self._buf_t[0]:
                self.stats.lidar_stale += 1
            elif fix.t > self._buf_t[-1]:
                keep.append(fix)
            else:
                self._update(fix, self._vo_at(fix.t))
        self._pending = keep

    def _update(self, fix: LidarFix, vo: Pose2) -> None:
        cfg = self.config
        tx, ty, th = float(self._c[0]), float(self._c[1]), float(self._c[2])
        c, s = math.cos(th), math.sin(th)
        predicted = (c * vo.x - s * vo.y + tx, s * vo.x + c * vo.y + ty, th + vo.yaw)
        h = np.array([[1.0, 0.0, -s * vo.x - c * vo.y],
                      [0.0, 1.0, c * vo.x - s * vo.y],
                      [0.0, 0.0, 1.0]])
        innovation = np.array([fix.pose.x - predicted[0],
                               fix.pose.y - predicted[1],
                               wrap_angle(fix.pose.yaw - predicted[2])])
        r = floor_covariance(fix.cov, cfg.min_lidar_sigma_pos_m, cfg.min_lidar_sigma_yaw_rad)
        s_mat = h @ self._p @ h.T + r
        try:
            s_inv = np.linalg.inv(s_mat)
        except np.linalg.LinAlgError:
            self.stats.lidar_invalid += 1
            return
        d2 = float(innovation @ s_inv @ innovation)
        self._last_innovation = [float(v) for v in innovation]
        self._last_d2 = d2
        if not math.isfinite(d2) or d2 > cfg.gate_chi2:
            self.stats.lidar_gated += 1
            self.stats.consecutive_gated += 1
            if self.stats.consecutive_gated >= int(cfg.suspect_after_gated):
                self._suspect = True
            if math.isfinite(d2):
                self._gated = (self._gated + [fix.pose.compose(vo.inverse())])[-int(cfg.reacquire_after):]
                self._maybe_reacquire()
            return
        self._gated = []
        gain = self._p @ h.T @ s_inv
        self._c = self._c + gain @ innovation
        self._c[2] = wrap_angle(float(self._c[2]))
        i_kh = np.eye(3) - gain @ h
        self._p = i_kh @ self._p @ i_kh.T + gain @ r @ gain.T
        self._p = 0.5 * (self._p + self._p.T)
        self.stats.lidar_fused += 1
        self.stats.consecutive_gated = 0
        self._last_fix_t = fix.t
        self._suspect = False

    def _maybe_reacquire(self) -> None:
        """Reset ``C`` when the last ``reacquire_after`` gated fixes agree with each other at the drone."""
        cfg = self.config
        if len(self._gated) < int(cfg.reacquire_after) or not self._buf_pose:
            return
        here = self._buf_pose[-1]
        placed = [c.compose(here) for c in self._gated]
        xs = np.array([p.x for p in placed])
        ys = np.array([p.y for p in placed])
        yaws = np.array([p.yaw for p in placed])
        mean_yaw = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))
        spread_m = float(np.max(np.hypot(xs - xs.mean(), ys - ys.mean())))
        spread_rad = float(np.max(np.abs([wrap_angle(float(a) - mean_yaw) for a in yaws])))
        if spread_m > cfg.reacquire_tol_m or spread_rad > cfg.reacquire_tol_rad:
            return
        target = Pose2(float(xs.mean()), float(ys.mean()), mean_yaw)
        self._c = np.array(target.compose(here.inverse()).as_tuple())
        self._p = np.diag([cfg.prior_pos_sigma_m ** 2, cfg.prior_pos_sigma_m ** 2, cfg.prior_yaw_sigma_rad ** 2])
        self._gated = []
        self.stats.reacquired += 1
        self.stats.consecutive_gated = 0

    def _slew(self, cur: Pose2, dt: float) -> None:
        """Move ``C_applied`` toward ``C``, limiting how far the published pose moves in *dt*."""
        if dt <= 0.0:
            return
        cfg = self.config
        target = self._c_pose().compose(cur)
        current = self._applied.compose(cur)
        dx, dy = target.x - current.x, target.y - current.y
        dyaw = wrap_angle(target.yaw - current.yaw)
        if dx == 0.0 and dy == 0.0 and dyaw == 0.0:
            return                                     # nothing pending: keep C_applied bit-exact
        alpha = 1.0
        dist = math.hypot(dx, dy)
        if dist > 0.0:
            alpha = min(alpha, cfg.max_correction_speed_mps * dt / dist)
        if dyaw != 0.0:
            alpha = min(alpha, cfg.max_correction_yaw_rate_rps * dt / abs(dyaw))
        stepped = Pose2(current.x + alpha * dx, current.y + alpha * dy, wrap_angle(current.yaw + alpha * dyaw))
        self._applied = stepped.compose(cur.inverse())
