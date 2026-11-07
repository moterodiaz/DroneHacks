"""The Guard: withholds cuVSLAM poses that can't be trusted, before our nodes use them.

What it guards
--------------
On the issued drone, PX4 flies on raw cuVSLAM through the organisers' bridge. Their
``run_bridge.sh`` launches it on the defaults, and no team code can sit in front of it
(``methods/lidar-vo-fusion/docs/kit-interfaces.md``). So this Guard protects *our* chain instead:

* the drift estimate in ``pose_fusion.fusion``, which must not learn a correction from poses the
  camera never really saw;
* the corrected setpoints sent to ``/skybeta/setpoint/*``.

Its output is the raw pose in cuVSLAM's ``odom`` frame. That is PX4's frame too, so no
re-anchoring is needed, and the frame never moves under PX4.

Rules (contracts/ros-graph.md, "Guard (ours) - withhold conditions")
---------------------------------------------------------------------
A pose is withheld, in this order:

1. ``failed``: cuVSLAM's own ``vo_state`` is FAILED.
2. ``jump`` / ``spin``: the step from the previous pose, withheld or not, implies more than
   10 m/s or 720 deg/s.
3. ``gyro``: over the last 0.5 s, cuVSLAM's rotation disagrees with PX4's gyro by more than 3 deg.
   On a blank view cuVSLAM reports confident poses from image noise. This rule catches the
   resulting attitude runaway.
4. ``settling``: after any break, good poses are withheld until they have run for 0.2 s,
   because the tracker returns plausible poses between garbage ones as it recovers.

The contract's image rule ("image texture under 1%") is **not** applied. The Guard reads no
camera images: the 1024x640 frames are 655 KB, more than Fast DDS's default 512 KB shared-memory
segment, and decoding them in Python at 30 fps costs a core. A blank or dark view that cuVSLAM
does not report as FAILED is therefore caught only when it shows up as a jump, a spin or a gyro
disagreement. On a still blank view that shows up late or never.

Every pose that passes gets fixed, honest variances, because cuVSLAM's own covariance is not an
uncertainty. The contract's "restart the tracker after 30 withheld frames" does not apply: the
tracker is the organisers' process, and restarting their stack re-zeroes PX4's vision origin.

Provenance
----------
The thresholds and the settle time are those measured in jsun4231/DroneDepthCamTesting, branch
``host-agnostic-cuvslam-setup`` @ ``ad28e40`` (``cuvslam_oak/tracking.py``,
``methods/cuvslam-vio/map_relay.py``). The variances are its ``px4_bridge/odometry.py``
defaults. The gyro rule is re-implemented here against PX4's body rates
(``/fmu/out/vehicle_odometry``) instead of the OAK's IMU.

numpy only; no ROS import. The node is ``dronehacks_lidar.vo_guard_node``.
"""

from __future__ import annotations

import collections
import math
from dataclasses import dataclass, fields
from typing import Deque, Optional, Sequence, Tuple

import numpy as np

#: Largest plausible speed between two poses: far above any flight speed, so it catches the engine's
#: metre-per-frame glitches and never the vehicle's own motion.
DEFAULT_MAX_SPEED_MPS = 10.0
#: Largest plausible rotation rate between two poses: 24 deg per frame at 30 fps.
DEFAULT_MAX_ROT_DPS = 720.0
#: Largest disagreement between the engine's and the gyro's rotation over the window.
DEFAULT_MAX_GYRO_DISAGREEMENT_DEG = 3.0
#: Seconds of rotation compared with the gyro: long enough for a slow attitude runaway to show,
#: short enough that the gyro's bias adds little.
DEFAULT_GYRO_WINDOW_S = 0.5
#: Seconds of good poses after a break before they are trusted again.
DEFAULT_SETTLE_S = 0.2
#: Variances of a passed pose: position (m^2) and orientation as roll, pitch, yaw (rad^2). Sigma
#: 0.1 m and 0.1 rad, EKF2's own EVP and EVA noise defaults, with height and yaw looser.
DEFAULT_POSITION_VARIANCE_M2 = (0.01, 0.01, 0.02)
DEFAULT_ORIENTATION_VARIANCE_RAD2 = (0.01, 0.01, 0.02)

#: ``isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.vo_state`` (visual_slam_impl.cpp).
VO_STATE_FAILED = 2


# --------------------------------------------------------------------------- rotation helpers

def quat_xyzw_to_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_angle_deg(r: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0))))


def exp_so3(w: np.ndarray) -> np.ndarray:
    """Rotation matrix of the rotation vector *w* (Rodrigues)."""
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * kx + (1.0 - math.cos(theta)) * (kx @ kx)


# --------------------------------------------------------------------------- the guard

@dataclass(frozen=True)
class GuardConfig:
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS
    max_rot_dps: float = DEFAULT_MAX_ROT_DPS
    max_gyro_disagreement_deg: float = DEFAULT_MAX_GYRO_DISAGREEMENT_DEG
    gyro_window_s: float = DEFAULT_GYRO_WINDOW_S
    settle_s: float = DEFAULT_SETTLE_S

    def __post_init__(self) -> None:
        for f in fields(self):
            value = getattr(self, f.name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value >= 0.0):
                raise ValueError("GuardConfig.{} must be a finite number >= 0 (got {!r})".format(f.name, value))


@dataclass(frozen=True)
class Verdict:
    publish: bool
    reason: Optional[str] = None       # failed | jump | spin | gyro | settling
    detail: str = ""
    after_break: bool = False          # the first pose published after a break


class Guard:
    """The Guard's bookkeeping, free of ROS. Feed status and gyro as they come, then judge each pose."""

    GYRO_KEPT_S = 2.0

    def __init__(self, config: Optional[GuardConfig] = None) -> None:
        self.config = config if config is not None else GuardConfig()
        self.withheld: collections.Counter = collections.Counter()
        self.published = 0
        self._failed = False
        self._previous: Optional[Tuple[float, np.ndarray, np.ndarray]] = None      # (t, position, rotation)
        self._accepted: Deque[Tuple[float, np.ndarray]] = collections.deque()        # (t, rotation) this stretch
        self._gyro: Deque[Tuple[float, np.ndarray]] = collections.deque()           # (t, body FLU rate)
        self._in_break = False
        self._settle_start: Optional[float] = None

    # ------------------------------------------------------------------ inputs

    def on_status(self, vo_state: int) -> None:
        self._failed = int(vo_state) == VO_STATE_FAILED

    def on_gyro(self, t: float, rate_flu: Sequence[float]) -> None:
        """A body angular rate in FLU (rad/s); PX4's FRD rates are (x, -y, -z) of these."""
        if self._gyro and t <= self._gyro[-1][0]:
            return
        self._gyro.append((float(t), np.asarray(rate_flu, dtype=float)))
        while self._gyro and self._gyro[0][0] < t - self.GYRO_KEPT_S:
            self._gyro.popleft()

    # ------------------------------------------------------------------ the verdict

    def judge(self, t: float, position: Sequence[float], q_xyzw: Sequence[float]) -> Verdict:
        """Whether the pose at *t* (``odom`` frame) may be used."""
        p = np.asarray(position, dtype=float)
        if not (np.all(np.isfinite(p)) and all(math.isfinite(float(v)) for v in q_xyzw) and math.isfinite(t)):
            return self._withhold(t, None, None, ("jump", "non-finite pose"))
        try:
            r = quat_xyzw_to_matrix(q_xyzw)
        except ValueError:
            return self._withhold(t, p, None, ("jump", "zero quaternion"))
        broken = self._rule_broken(t, p, r)
        if broken is not None:
            return self._withhold(t, p, r, broken)
        self._previous = (t, p, r)
        if self._in_break:
            if self._settle_start is None:
                self._settle_start = t
            settled = t - self._settle_start
            if settled < self.config.settle_s:
                self.withheld["settling"] += 1
                self._accepted.append((t, r))             # a settling stretch is the gyro rule's reference too
                return Verdict(False, "settling", "{:.2f} of {:g} s of good poses".format(settled, self.config.settle_s))
            self._in_break = False
            self._settle_start = None
            self._note_accepted(t, r)
            self.published += 1
            return Verdict(True, after_break=True)
        self._note_accepted(t, r)
        self.published += 1
        return Verdict(True)

    def _withhold(self, t: float, p: Optional[np.ndarray], r: Optional[np.ndarray], why: Tuple[str, str]) -> Verdict:
        self.withheld[why[0]] += 1
        if p is not None and r is not None:
            self._previous = (t, p, r)                    # the next step is judged from this one, accepted or not
        self._accepted.clear()                            # a break forgets the stretch: what follows may be a new frame
        self._in_break = True
        self._settle_start = None
        return Verdict(False, why[0], why[1])

    def _note_accepted(self, t: float, r: np.ndarray) -> None:
        self._accepted.append((t, r))
        while len(self._accepted) > 2 and self._accepted[1][0] < t - 2.0 * self.config.gyro_window_s:
            self._accepted.popleft()

    def _rule_broken(self, t: float, p: np.ndarray, r: np.ndarray) -> Optional[Tuple[str, str]]:
        cfg = self.config
        if self._failed:
            return "failed", "the tracker reports vo_state FAILED"
        if self._previous is not None:
            t0, p0, r0 = self._previous
            dt = t - t0
            if dt > 0.0:
                step = float(np.linalg.norm(p - p0))
                rot = rotation_angle_deg(r0.T @ r)
                if cfg.max_speed_mps > 0.0 and step / dt > cfg.max_speed_mps:
                    return "jump", "step {:.3f} m in {:.1f} ms = {:.1f} m/s > {:g} m/s".format(
                        step, dt * 1e3, step / dt, cfg.max_speed_mps)
                if cfg.max_rot_dps > 0.0 and rot / dt > cfg.max_rot_dps:
                    return "spin", "rotation {:.1f} deg in {:.1f} ms = {:.0f} deg/s > {:g} deg/s".format(
                        rot, dt * 1e3, rot / dt, cfg.max_rot_dps)
        return self._gyro_rule(t, r)

    def _gyro_rule(self, t: float, r: np.ndarray) -> Optional[Tuple[str, str]]:
        cfg = self.config
        if cfg.max_gyro_disagreement_deg <= 0.0 or not self._accepted or not self._gyro:
            return None
        reference = None
        for entry in self._accepted:                      # the newest accepted pose a window back, else the oldest
            if entry[0] > t - cfg.gyro_window_s and reference is not None:
                break
            if entry[0] < t:
                reference = entry
        if reference is None:
            return None
        t_ref, r_ref = reference
        samples = [(ts, w) for ts, w in self._gyro if t_ref <= ts <= t]
        if len(samples) < 2 or samples[0][0] - t_ref > 0.05 or t - samples[-1][0] > 0.05:
            return None                                   # the gyro doesn't cover the span: no verdict
        gyro = np.eye(3)
        prev_t = t_ref
        for ts, w in samples + [(t, samples[-1][1])]:
            gyro = gyro @ exp_so3(w * (ts - prev_t))
            prev_t = ts
        engine = r_ref.T @ r
        disagreement = rotation_angle_deg(engine.T @ gyro)
        if disagreement > cfg.max_gyro_disagreement_deg:
            return "gyro", "rotation disagrees with the gyro by {:.1f} deg over {:.2f} s > {:g} deg".format(
                disagreement, t - t_ref, cfg.max_gyro_disagreement_deg)
        return None


def fixed_pose_covariance() -> list:
    """The 6x6 row-major pose covariance a passed pose carries (x, y, z, roll, pitch, yaw)."""
    diag = list(DEFAULT_POSITION_VARIANCE_M2) + list(DEFAULT_ORIENTATION_VARIANCE_RAD2)
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 7] = float(v)
    return cov
