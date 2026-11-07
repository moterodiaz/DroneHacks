"""Tracking loop shared by live and dataset runs of cuVSLAM on the OAK-D S2 (PyCuVSLAM 17.0.0).

``run_tracking(vslam, rig, mode, frame_iter, imu_iter, cfg, on_pose, on_stats)`` drives one
``vslam.Tracker`` over a stream of stereo frames (plus BNO086 IMU samples in ``'vio'`` mode) and
returns a summary dict; :func:`write_tum` writes the accumulated trajectory in the TUM format that
``evo`` reads.  The same loop serves

* ``methods/cuvslam-vio/08_cuvslam_track.py`` replaying an EuRoC-layout recording
  (``cuvslam_oak.dataset.EurocReader.frames()`` + ``.imu_samples()``, or ``.merged()``), and
* the live path on the Jetson / WSL2 (``cuvslam_oak.oak_source`` yielding synced GRAY8 pairs
  and raw BNO086 packets from the camera).

``vslam`` is the module returned by :func:`cuvslam_oak.import_cuvslam` (the real wheel on WSL2 /
Jetson, :mod:`cuvslam_oak.fake_cuvslam` on Windows).  Nothing here imports ``cuvslam`` at module
import time; only ``numpy`` and :mod:`cuvslam_oak.rig` (stdlib + numpy) are imported.

What the loop does (``methods/cuvslam-vio/docs/cuvslam-api-v17.md`` sections 3, 5-7):

* builds ``Tracker.OdometryConfig`` like the official examples: ``rectified_stereo_camera=False``
  (our rigs carry unrectified intrinsics + Polynomial distortion, or host-undistorted Pinhole
  frames, never row-aligned rectified pairs), ``odometry_mode=Multicamera`` for ``'stereo'``
  (mirrors ``examples/oak-d/run_stereo.py``) and ``Inertial`` for ``'vio'`` (the rig must then
  hold exactly one ``ImuCalibration``, see ``rig.build_rig(mode='vio')``), and
  ``async_sba = cfg.realtime`` (``False`` = deterministic offline replay, what every NVIDIA
  example uses; ``True`` = the library defaults for a live run);
* skips ``cfg.warmup_frames`` frames before the first ``track()`` (60 in the official OAK-D
  example, auto-exposure settle; pass 0 for recordings, the recorder already dropped them);
  IMU samples that arrive during warm-up are kept for the first ``track()`` only as far back as
  ``cfg.warmup_imu_window_s`` before the last warm-up frame (default 0: none; older ones are
  discarded and counted as ``imu_skipped_warmup``).  A 1 s window (``run_vio.py`` registers
  everything buffered before its first ``track()``) made live ``vio`` diverge on the Jetson;
  see :data:`DEFAULT_WARMUP_IMU_WINDOW_S`;
* before each ``track()`` registers every IMU sample whose timestamp is ``<=`` the image
  timestamp and holds back the rest for the next frame (NVIDIA's
  ``examples/realsense/run_vio.py::register_imu_until`` rule); samples older than the last
  tracker timestamp are dropped and counted, never passed;
* counts tracking failures (``world_from_rig is None``) and continues; a valid pose after a
  failure starts a new segment (cuVSLAM restarts its world frame after a loss) and bumps
  ``resets``; ``cfg.stitch_resets`` chains each new segment onto the last good pose so the
  trajectory stays continuous (zero motion during the loss is assumed); a pose holding a
  non-finite value or a zero-norm quaternion is skipped and counted (``frames_nonfinite``)
  but is *not* a loss: the engine only restarts its world frame after a ``None`` pose, so no
  reset is recorded and stitching does not re-anchor; the quaternion of every row is
  unit-normalised;
* optionally runs cuVSLAM's SLAM layer next to the odometry (``cfg.slam``): loop closure and pose-graph
  optimisation in the same ``Tracker`` (``Tracker(rig, odom_cfg, slam_cfg)``).  The SLAM pose of every
  accepted frame goes into ``slam_rows`` (same anchoring as the odometry rows, so the two trajectories
  are comparable) and onto ``PoseEvent.slam_translation`` / ``slam_rotation_xyzw``; at the end (and
  before every guard engine reset, which destroys the map) ``get_all_slam_poses()`` gives the
  re-optimised trajectory, ``slam_optimized_rows``.  Loop closures and pose-graph optimisations are
  counted as rising edges of ``get_slam_metrics().lc_status`` / ``pgo_status``.  The SLAM pose jumps
  when a loop closes: feed a flight controller the odometry rows, use the SLAM rows at the mission
  level.  ``cfg.slam_save_map`` saves the map at the end;
* *coasts* while the engine gives no usable pose (``cfg.coast``, on by default): a guarded frame, a
  loss (``None``) or a non-finite pose still produces a row, estimated from the last accepted poses:
  the position continues at the median frame-to-frame velocity of the last ``cfg.coast_velocity_window_s``
  (decaying with time constant ``cfg.coast_decay_s``, capped at ``cfg.max_speed_mps``) and the
  rotation follows the gyroscope (``cfg.coast_gyro``; the IMU samples are read in ``stereo`` mode
  too, without being registered with the engine), so a turn during a blank-wall stretch keeps its
  heading.  After ``cfg.coast_max_s`` without vision no more rows are written until vision returns.
  Coasted rows are counted (``frames_coasted``, ``coast_gaps``) and flagged (``PoseEvent.coasted``),
  and the next accepted pose is re-anchored onto the coaster's prediction for its own time, also
  after a loss (with coasting on, a loss no longer restarts the world frame at identity; the
  reset is still counted and ``segments`` still lists it);
* guards against *starved* frames (``cfg.guard``, on by default): on a blank wall from close
  range cuVSLAM still returns a pose, but a garbage one (single-frame steps of 2-50 m and
  rotations of up to 60 deg measured on hand-held walks; in ``vio`` mode the inertial filter
  then runs away for hundreds of metres) and never reports a failure.  A returned pose is
  therefore *skipped* (no row, no ``on_pose``; counted as ``frames_guarded`` with its reason
  ``guard_starved`` / ``guard_degenerate`` / ``guard_jump`` / ``guard_spin``, the first that
  matches in that order) when fewer than ``cfg.min_observations`` features were tracked in
  camera 0 (``get_last_observations(0)``; off by default: on this camera the count did *not*
  drop on the glitch frames, it sat at the engine's cap of 450 because sensor noise on a blank
  wall is detected as features; an engine that cannot export observations never triggers the
  check either), when the position block of ``covariance_xyz_rpy`` is *exactly* zero
  (``cfg.guard_zero_covariance``: indices 0, 7, 14 all ``0.0``, never on the first accepted pose
  of a segment, where the engine always reports zeros; measured on about half of the
  glitch frames and on well under 1 % of normal ones; there is deliberately no magnitude
  threshold because the engine returns a placeholder variance of 1.0 per axis, sigma 1.732 m,
  on ~30 % of perfectly good frames of a still camera), or when the motion since the previously
  *accepted* pose of the same engine segment exceeds ``cfg.max_speed_mps`` or
  ``cfg.max_rot_dps`` (the primary defence: glitch steps measured 0.39-52 m per frame,
  rotations up to 63 deg, against 0.01-0.05 m and 0.3-0.7 deg at walking pace).  The first
  accepted pose after a guarded stretch is re-anchored onto the
  last good pose (zero motion across the gap is assumed, as with ``stitch_resets``; the gap is
  recorded in ``guard_gaps``), so the trajectory stays continuous even without
  ``stitch_resets``.  After ``cfg.guard_reset_after`` consecutive guarded frames the tracker is
  re-created with the same rig and config (``guard_resets``, also counted in ``resets`` and
  ``segments``; the IMU stream continues into the new tracker, no warm-up frames are skipped)
  and the next accepted pose is re-anchored onto the last good pose *regardless* of
  ``stitch_resets``, because the guard exists to keep the trajectory continuous;
* warns about and counts image gaps above ``cfg.jitter_threshold_ns`` (35 ms) and IMU gaps above
  ``cfg.imu_jitter_threshold_ns`` (10 ms); a frame whose timestamp is not strictly increasing is
  skipped and counted instead of being handed to the engine;
* emits per-interval statistics (``on_stats``) and per-pose events (``on_pose``), accumulates TUM
  rows and, on finish, computes total path length, end-point offset from the first pose (the
  drift proxy of the bench protocol), duration and mean fps (frames handed to ``track()`` per
  wall second from the first ``track()`` on, so warm-up frames do not deflate it).

Timestamps are integer nanoseconds on one clock for frames and IMU (the device monotonic clock
plus one epoch offset in our recordings, ``datasets/dataset-format.md``); the loop only ever
compares them.  Frames are ``(height, width)`` uint8 arrays in rig camera order (left = camera
0, right = camera 1); the loop checks dtype and shape against the rig on every frame and makes
strided views contiguous (cuVSLAM rejects both).  An image may also be given as a PNG path
(loaded through ``cuvslam_oak.dataset.load_gray_png``), so the tagged tuples of
``datasets/dataset-format.md`` section 12.3, ``('imu', t, gyro, accel)`` and
``('stereo', t, [path0, path1])``, are accepted next to the ``EurocReader.merged()`` objects.

A live IMU source must be a ``queue.Queue`` or travel inside the merged frame stream
(``OakStereoImuSource.events()``), never a blocking generator: the feeder reads one IMU sample
ahead of every image with no timeout, so a stalled blocking stream would block the loop forever.
"""

from __future__ import annotations

import collections
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Any, Callable, Deque, Dict, Iterable, Iterator, List, Mapping, NamedTuple,
                    Optional, Sequence, Tuple, Union)

import numpy as np

from . import imu_calib as _imu_calib
from . import is_fake as _is_fake
from . import rig as _rig

__all__ = [
    "MODES",
    "MODE_ALIASES",
    "ODOMETRY_MODE_NAME",
    "MULTICAM_MODES",
    "DEFAULT_WARMUP_FRAMES",
    "DEFAULT_WARMUP_IMU_WINDOW_S",
    "DEFAULT_JITTER_THRESHOLD_NS",
    "DEFAULT_IMU_JITTER_THRESHOLD_NS",
    "DEFAULT_MIN_OBSERVATIONS",
    "DEFAULT_MAX_SPEED_MPS",
    "DEFAULT_COAST_VELOCITY_WINDOW_S",
    "DEFAULT_COAST_DECAY_S",
    "DEFAULT_COAST_MAX_S",
    "DEFAULT_MAX_ROT_DPS",
    "DEFAULT_GUARD_RESET_AFTER",
    "TUM_HEADER",
    "TrackingError",
    "TrackConfig",
    "ImuRecord",
    "PoseEvent",
    "normalize_mode",
    "odometry_config",
    "apply_borders",
    "make_tracker",
    "as_frame",
    "as_imu_sample",
    "is_imu_item",
    "run_tracking",
    "trajectory_metrics",
    "format_tum_timestamp",
    "parse_tum_timestamp",
    "write_tum",
    "read_tum",
]

#: Tracking modes of this package: ``'stereo'`` = ``OdometryMode.Multicamera`` (cameras only),
#: ``'vio'`` = ``OdometryMode.Inertial`` (cameras + one BNO086 ``ImuCalibration`` in the rig).
MODES = ("stereo", "vio")

#: Accepted spellings of the two modes (case-insensitive), e.g. the ``--mode stereo-inertial``
#: of ``methods/cuvslam-vio/08_cuvslam_track.py``.
MODE_ALIASES: Dict[str, str] = {
    "stereo": "stereo", "stereo-only": "stereo", "multicamera": "stereo", "vo": "stereo",
    "vio": "vio", "inertial": "vio", "stereo-inertial": "vio", "imu": "vio",
}

#: ``Tracker.OdometryMode`` member used for each mode (same table as ``rig.rig_spec``).
ODOMETRY_MODE_NAME: Dict[str, str] = {"stereo": "Multicamera", "vio": "Inertial"}

#: Valid ``TrackConfig.multicam_mode`` names (``Tracker.MulticameraMode`` members).
MULTICAM_MODES = ("Performance", "Precision", "Moderate")

#: Frames skipped before the first ``track()`` (``WARMUP_FRAMES`` of the official OAK-D example).
DEFAULT_WARMUP_FRAMES = 60

#: Seconds of IMU history before the last warm-up frame that stay buffered and are registered
#: ahead of the first ``track()`` in ``'vio'`` mode (``TrackConfig.warmup_imu_window_s``).
#: 0: with 1.0, live ``vio`` on the Jetson (60 warm-up frames, camera still) tracked 37 % of frames
#: with 30+ resets; with 0 it tracked 100 % with 0 resets and 4 mm drift in 30 s (2026-10-31).
DEFAULT_WARMUP_IMU_WINDOW_S = 0.0

#: Image gap that is reported as a dropped frame (``IMAGE_JITTER_THRESHOLD_NS`` of the examples).
DEFAULT_JITTER_THRESHOLD_NS = 35_000_000

#: IMU gap that is reported as a dropped sample (``IMU_JITTER_THRESHOLD_NS`` of ``run_vio.py``).
DEFAULT_IMU_JITTER_THRESHOLD_NS = 10_000_000  # Isaac ROS node default (imu_jitter_threshold_ms 10); the BNO086 spaces 200 Hz samples bimodally at 3-9 ms

#: Starvation-guard defaults (``TrackConfig.guard``), tuned for a hand-held walk or an indoor
#: flight at 30 fps.  The observation-count rule is off (0): on the OAK-D S2 the glitch frames
#: still reported 450 observations (the engine's cap; normal frames 185-450), so the count is
#: not a starvation signal on this camera.  Kept as an option for other sensors.
DEFAULT_MIN_OBSERVATIONS = 0
#: Largest plausible speed between two accepted poses (m/s); a racing drone needs more.
DEFAULT_MAX_SPEED_MPS = 3.0
#: Largest plausible rotation rate between two accepted poses (deg/s; 720 = 24 deg per 30 fps frame).
DEFAULT_MAX_ROT_DPS = 720.0
#: Coasting (``TrackConfig.coast``): the window of accepted poses the velocity is fitted over, the time
#: constant of its decay and the longest gap that is still bridged.
DEFAULT_COAST_VELOCITY_WINDOW_S = 1.0
DEFAULT_COAST_DECAY_S = 1.0
DEFAULT_COAST_MAX_S = 3.0
#: Consecutive guarded frames after which the tracker is re-created (0 = never).
DEFAULT_GUARD_RESET_AFTER = 30

#: First line of every TUM file written here (``datasets/dataset-format.md`` section 11).
TUM_HEADER = "# timestamp_s tx ty tz qx qy qz qw"

_NS_PER_S = 1_000_000_000
_WARN_FIRST = 5      # occurrences of one warning kind printed in full ...
_WARN_EVERY = 100    # ... then every 100th (run_vio.py prints IMU drops the same way)


class TrackingError(RuntimeError):
    """A frame, an IMU sample, the rig or the environment could not be handed to cuVSLAM."""


# --------------------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------------------

@dataclass
class TrackConfig:
    """Knobs of :func:`run_tracking` (everything else comes from the rig and the mode).

    ``realtime``: ``True`` keeps cuVSLAM's defaults (``async_sba=True``, bundle adjustment on a
    background thread) for a live camera; ``False`` (default) sets ``async_sba=False`` for a
    deterministic, repeatable replay of a recording (what all NVIDIA examples use).
    ``warmup_frames``: frames discarded before the first ``track()`` (auto-exposure settle on a
    live camera; use 0 for recordings).  ``warmup_imu_window_s``: in ``'vio'`` mode the IMU
    samples of the last this-many seconds before the last warm-up frame stay buffered and are
    registered ahead of the first ``track()`` (default 0); older warm-up samples are discarded
    and counted as ``imu_skipped_warmup``; ``0`` keeps only the samples after the last warm-up
    frame.  ``jitter_threshold_ns`` / ``imu_jitter_threshold_ns``:
    gaps above these are counted (and logged when ``verbose``).  ``borders``: ``None`` keeps the
    rig's ``Camera.border_*``; otherwise any form :func:`cuvslam_oak.rig.normalize_borders`
    accepts (``0``/``'none'``, a dict, a ``(top, bottom, left, right)`` sequence) is applied to
    every camera before the tracker is built.  ``verbose``: log warnings and a status line every
    ``stats_interval_s`` seconds through ``log`` (default: stderr); with ``False`` everything is
    only counted.  ``max_frames``: stop after this many frames were handed to ``track()``
    (warm-up excluded; 0 = unlimited).  ``should_stop``: polled once per frame (e.g.
    ``RunClock.expired``) for a wall-clock limit.

    Engine options passed straight into ``Tracker.OdometryConfig``: ``multicam_mode``
    (``Precision`` default, ``Performance`` for a small Jetson power budget), ``use_gpu``,
    ``use_motion_model``, ``use_denoising``, ``max_frame_delta_s`` (a larger gap loses tracking),
    ``final_landmarks`` (``enable_final_landmarks_export``; the summary then reports their
    count), ``rectified_stereo_camera`` (keep ``False`` unless the frames are row-aligned
    rectified pairs), ``expert_parameters`` (``Odometry.apply_expert_parameters`` strings).
    ``verbosity`` sets ``cuvslam.set_verbosity`` when not ``None``; ``warm_up_gpu`` calls
    ``cuvslam.warm_up_gpu()`` before the tracker is built (it raises when CUDA is broken, which is
    turned into a :class:`TrackingError` with the check_env hint).  ``stitch_resets`` is described
    in the module docstring.

    Starvation guard (module docstring): ``guard`` (default ``True``) skips a returned pose when
    fewer than ``min_observations`` features were tracked in camera 0 (default 0 = off: the
    count did not drop on this camera's glitch frames, it sat at the engine's cap of 450; an
    engine that cannot export observations never triggers the check either), when
    ``guard_zero_covariance`` (default ``True``) finds the position variances of
    ``covariance_xyz_rpy`` all exactly ``0.0`` (no magnitude threshold exists on purpose: the
    engine reports a placeholder variance of 1.0 per axis on ~30 % of good frames of a still
    camera), or when the step from the previously accepted pose of the same engine segment
    exceeds ``max_speed_mps`` (3 m/s) or ``max_rot_dps`` (720 deg/s); after
    ``guard_reset_after`` consecutive guarded frames (30) the tracker is re-created (``0`` =
    never).  The defaults suit a hand-held walk or an indoor flight at 30 fps; a racing drone
    needs a higher ``max_speed_mps``.  The rates may be ``inf``.
    """

    realtime: bool = False
    warmup_frames: int = DEFAULT_WARMUP_FRAMES
    warmup_imu_window_s: float = DEFAULT_WARMUP_IMU_WINDOW_S
    jitter_threshold_ns: int = DEFAULT_JITTER_THRESHOLD_NS
    imu_jitter_threshold_ns: int = DEFAULT_IMU_JITTER_THRESHOLD_NS
    borders: Any = None
    verbose: bool = False
    max_frames: int = 0
    stats_interval_s: float = 1.0
    stitch_resets: bool = False
    multicam_mode: str = "Precision"
    use_gpu: bool = True
    use_motion_model: bool = True
    use_denoising: bool = False
    max_frame_delta_s: float = 1.0
    final_landmarks: bool = False
    rectified_stereo_camera: bool = False
    expert_parameters: Dict[str, str] = field(default_factory=dict)
    verbosity: Optional[int] = None
    warm_up_gpu: bool = True
    should_stop: Optional[Callable[[], bool]] = None
    log: Optional[Callable[[str], None]] = None
    guard: bool = True
    min_observations: int = DEFAULT_MIN_OBSERVATIONS
    guard_zero_covariance: bool = True
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS
    max_rot_dps: float = DEFAULT_MAX_ROT_DPS
    guard_reset_after: int = DEFAULT_GUARD_RESET_AFTER
    #: Optional :class:`cuvslam_oak.imu_calib.ImuCorrection` (or its dict) applied to every IMU
    #: sample before it is registered; ``None`` registers the raw values.
    imu_correction: Any = None
    #: Coasting (module docstring): estimate a pose while the engine gives none.
    coast: bool = True
    coast_velocity_window_s: float = DEFAULT_COAST_VELOCITY_WINDOW_S
    coast_decay_s: float = DEFAULT_COAST_DECAY_S
    coast_max_s: float = DEFAULT_COAST_MAX_S
    coast_gyro: bool = True
    #: When the engine tracked through a gap without restarting its world frame, keep its rotation at
    #: the re-anchor and correct only the position.  Off by default: measured on four walks
    #: (2026-09-29), re-anchoring onto the gyro-coasted rotation left a smaller heading error at the end
    #: of the walk (the engine's rotation through a blank-wall stretch is garbage too).
    coast_keep_engine_rotation: bool = False
    #: SLAM (module docstring): loop closure + pose graph in the same tracker.  ``slam_sync`` ``None``
    #: means synchronous (deterministic) unless ``realtime``; ``slam_max_map_size`` 0 = unlimited poses
    #: in the graph (NVIDIA's default is 300, which forgets the start of a run longer than a few
    #: minutes); ``slam_save_map`` = folder to save the map into at the end ('' = do not).
    slam: bool = False
    slam_sync: Optional[bool] = None
    slam_max_map_size: int = 0
    slam_planar: bool = False
    slam_save_map: str = ""
    #: Rotation of the IMU in the rig frame as a quaternion ``(x, y, z, w)`` (or a 3x3 / 4x4 matrix),
    #: needed for gyro coasting when the rig carries no ``ImuCalibration`` (``stereo`` mode); in
    #: ``vio`` mode it is read from the rig.
    coast_rig_from_imu: Any = None

    def __post_init__(self) -> None:
        self.warmup_frames = max(0, int(self.warmup_frames))
        self.warmup_imu_window_s = float(self.warmup_imu_window_s)
        if not (math.isfinite(self.warmup_imu_window_s) and self.warmup_imu_window_s >= 0.0):
            raise ValueError("warmup_imu_window_s must be a finite non-negative number, got {}".format(
                self.warmup_imu_window_s))
        self.jitter_threshold_ns = int(self.jitter_threshold_ns)
        self.imu_jitter_threshold_ns = int(self.imu_jitter_threshold_ns)
        self.max_frames = max(0, int(self.max_frames or 0))
        self.stats_interval_s = max(0.0, float(self.stats_interval_s))
        self.max_frame_delta_s = float(self.max_frame_delta_s)
        if self.max_frame_delta_s <= 0.0:
            raise ValueError("max_frame_delta_s must be positive, got {}".format(self.max_frame_delta_s))
        if self.multicam_mode not in MULTICAM_MODES:
            raise ValueError("multicam_mode must be one of {}, got {!r}".format(MULTICAM_MODES, self.multicam_mode))
        if self.should_stop is not None and not callable(self.should_stop):
            raise TypeError("should_stop must be callable or None")
        if self.log is not None and not callable(self.log):
            raise TypeError("log must be callable or None")
        self.expert_parameters = {str(k): str(v) for k, v in dict(self.expert_parameters or {}).items()}
        self.guard = bool(self.guard)
        self.guard_zero_covariance = bool(self.guard_zero_covariance)
        self.min_observations = int(self.min_observations)
        self.guard_reset_after = int(self.guard_reset_after)
        for name in ("min_observations", "guard_reset_after"):
            if getattr(self, name) < 0:
                raise ValueError("{} must be a non-negative integer, got {}".format(name, getattr(self, name)))
        self.max_speed_mps = float(self.max_speed_mps)
        self.max_rot_dps = float(self.max_rot_dps)
        for name in ("max_speed_mps", "max_rot_dps"):
            value = getattr(self, name)
            if math.isnan(value) or value < 0.0:
                raise ValueError("{} must be a non-negative number (inf allowed), got {}".format(name, value))
        self.slam = bool(self.slam)
        if self.slam_sync is not None:
            self.slam_sync = bool(self.slam_sync)
        self.slam_max_map_size = int(self.slam_max_map_size)
        if self.slam_max_map_size < 0:
            raise ValueError("slam_max_map_size must be >= 0 (0 = unlimited), got {}".format(self.slam_max_map_size))
        self.slam_planar = bool(self.slam_planar)
        self.slam_save_map = str(self.slam_save_map or "")
        self.coast = bool(self.coast)
        self.coast_gyro = bool(self.coast_gyro)
        self.coast_keep_engine_rotation = bool(self.coast_keep_engine_rotation)
        for name in ("coast_velocity_window_s", "coast_decay_s", "coast_max_s"):
            value = float(getattr(self, name))
            if math.isnan(value) or value < 0.0:
                raise ValueError("{} must be a non-negative number, got {}".format(name, value))
            setattr(self, name, value)
        if self.coast_rig_from_imu is not None:
            self.coast_rig_from_imu = [float(v) for v in _rig.matrix_to_quaternion_xyzw(
                _rotation_matrix_of(self.coast_rig_from_imu, "coast_rig_from_imu"))]
        if self.imu_correction is not None and not isinstance(self.imu_correction, _imu_calib.ImuCorrection):
            if not isinstance(self.imu_correction, dict):
                raise TypeError("imu_correction must be an ImuCorrection, its dict or None, got {}".format(
                    type(self.imu_correction).__name__))
            self.imu_correction = _imu_calib.ImuCorrection.from_dict(self.imu_correction)

    def to_dict(self) -> Dict[str, Any]:
        """JSON-serialisable copy (the two callables are reported as booleans)."""
        out: Dict[str, Any] = {}
        for name in self.__dataclass_fields__:  # type: ignore[attr-defined]
            value = getattr(self, name)
            if name in ("should_stop", "log"):
                out[name] = value is not None
            elif name == "borders":
                out[name] = _json_safe(value)
            elif name == "imu_correction":
                out[name] = None if value is None else value.to_dict()
            elif isinstance(value, dict):
                out[name] = dict(value)
            else:
                out[name] = value
        return out


def _rotation_matrix_of(value: Any, what: str) -> np.ndarray:
    """3x3 rotation from a quaternion ``(x, y, z, w)``, a 3x3 or a 4x4 matrix, or an object with
    ``.rotation`` (a ``vslam.Pose``)."""
    if hasattr(value, "rotation") and not isinstance(value, (list, tuple, np.ndarray)):
        value = value.rotation
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape == (4,):
        return _rig.quaternion_xyzw_to_matrix(arr)
    if arr.shape in ((3, 3), (4, 4)):
        return np.array(arr[:3, :3], dtype=np.float64)
    raise ValueError("{} must be a quaternion (x, y, z, w), a 3x3 or a 4x4 matrix, got shape {}".format(
        what, arr.shape))


def _exp_so3(rotvec: np.ndarray) -> np.ndarray:
    """Rotation matrix of a rotation vector (Rodrigues)."""
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    k = rotvec / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, Mapping):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def normalize_mode(mode: str) -> str:
    """``'stereo'`` or ``'vio'`` from any :data:`MODE_ALIASES` spelling; ``ValueError`` otherwise."""
    key = str(mode).strip().lower().replace("_", "-")
    try:
        return MODE_ALIASES[key]
    except KeyError:
        raise ValueError("mode must be one of {} (aliases: {}), got {!r}".format(
            MODES, ", ".join(sorted(MODE_ALIASES)), mode)) from None


# --------------------------------------------------------------------------------------
# Tracker construction
# --------------------------------------------------------------------------------------

def odometry_config(vslam: Any, mode: str, cfg: Optional[TrackConfig] = None) -> Any:
    """``vslam.Tracker.OdometryConfig`` for *mode* exactly as the official examples build it.

    ``stereo``: ``odometry_mode=Multicamera`` (``examples/oak-d/run_stereo.py``); ``vio``:
    ``odometry_mode=Inertial`` (``examples/euroc/track_euroc.py``, ``realsense/run_vio.py``).
    Always ``rectified_stereo_camera=False`` unless ``cfg.rectified_stereo_camera`` says
    otherwise, ``enable_observations_export=True`` (needed for ``get_last_observations``),
    ``enable_final_landmarks_export=cfg.final_landmarks``, ``async_sba=cfg.realtime``.
    """
    mode = normalize_mode(mode)
    cfg = cfg if cfg is not None else TrackConfig()
    tracker_cls = vslam.Tracker
    odometry_mode = getattr(tracker_cls.OdometryMode, ODOMETRY_MODE_NAME[mode])
    multicam_mode = getattr(tracker_cls.MulticameraMode, cfg.multicam_mode)
    return tracker_cls.OdometryConfig(
        odometry_mode=odometry_mode,
        multicam_mode=multicam_mode,
        async_sba=bool(cfg.realtime),
        use_gpu=bool(cfg.use_gpu),
        use_motion_model=bool(cfg.use_motion_model),
        use_denoising=bool(cfg.use_denoising),
        rectified_stereo_camera=bool(cfg.rectified_stereo_camera),
        enable_observations_export=True,
        enable_landmarks_export=True,
        enable_final_landmarks_export=bool(cfg.final_landmarks),
        max_frame_delta_s=float(cfg.max_frame_delta_s),
    )


def apply_borders(rig: Any, borders: Any, *, undistorted: bool = False) -> Any:
    """Set ``Camera.border_top/bottom/left/right`` on every camera of *rig* in place and return it.

    *borders* takes any form :func:`cuvslam_oak.rig.normalize_borders` accepts; ``None`` leaves
    the rig untouched.  The camera list is re-assigned as a whole because ``rig.cameras`` is a
    copy in the binding (``rig.cameras[0].border_top = 1`` alone would be lost).
    """
    if borders is None:
        return rig
    cameras = list(rig.cameras)
    for index, cam in enumerate(cameras):
        size = (int(cam.size[0]), int(cam.size[1]))
        if size[0] <= 0 or size[1] <= 0:
            raise ValueError("camera {} has no size; set Camera.size before applying borders".format(index))
        values = _rig.normalize_borders(borders, size, undistorted=undistorted)
        cam.border_top = int(values["top"])
        cam.border_bottom = int(values["bottom"])
        cam.border_left = int(values["left"])
        cam.border_right = int(values["right"])
    rig.cameras = cameras
    return rig


def make_tracker(vslam: Any, rig: Any, mode: str, cfg: Optional[TrackConfig] = None) -> Tuple[Any, Any]:
    """Build ``(vslam.Tracker, odometry_config)`` for *rig* and *mode* (v17.0.0 ``Tracker(rig, cfg)``).

    Checks the rig against the mode first (``vio`` needs exactly one IMU, the stereo rig needs at
    least one camera) so the error names the fix instead of the engine's message; applies
    ``cfg.borders``; sets the cuVSLAM verbosity; optionally warms up the GPU; then applies
    ``cfg.expert_parameters``.  Raises :class:`TrackingError` (the binding's ``ValueError`` /
    ``TypeError`` and the engine's ``RuntimeError`` are chained as ``__cause__``).
    """
    mode = normalize_mode(mode)
    cfg = cfg if cfg is not None else TrackConfig()
    cameras = list(rig.cameras)
    imus = list(rig.imus)
    if not cameras:
        raise TrackingError("the rig has no cameras; build it with cuvslam_oak.rig.build_rig()")
    if mode == "vio" and len(imus) != 1:
        raise TrackingError("'vio' mode needs exactly one ImuCalibration in the rig, got {}; build the rig "
                            "with build_rig(..., mode='vio') or track with mode='stereo'".format(len(imus)))
    if len(imus) > 1:
        raise TrackingError("cuVSLAM supports at most one IMU per rig, got {}".format(len(imus)))
    if cfg.borders is not None:
        apply_borders(rig, cfg.borders)
    if cfg.verbosity is not None:
        vslam.set_verbosity(int(cfg.verbosity))
    if cfg.warm_up_gpu:
        try:
            vslam.warm_up_gpu()
        except Exception as exc:  # noqa: BLE001 - the binding raises a plain RuntimeError
            raise TrackingError("cuvslam.warm_up_gpu() failed ({}: {}): CUDA toolkit 12 / cuBLAS / cuSOLVER "
                                "are missing or the driver is too old; run python methods/cuvslam-vio/tools/check_env.py".format(
                                    type(exc).__name__, exc)) from exc
    odom_cfg = odometry_config(vslam, mode, cfg)
    return _new_tracker(vslam, rig, mode, cfg, odom_cfg), odom_cfg


def slam_config(vslam: Any, cfg: TrackConfig) -> Any:
    """``Tracker.SlamConfig`` for *cfg* (``None`` when ``cfg.slam`` is off): synchronous loop closure
    unless ``cfg.slam_sync`` says otherwise (``None`` = synchronous when not ``realtime``), internals
    readable (pose graph, metrics), ``max_map_size`` / ``planar_constraints`` from *cfg*."""
    if not cfg.slam:
        return None
    sync = (not cfg.realtime) if cfg.slam_sync is None else bool(cfg.slam_sync)
    try:
        return vslam.Tracker.SlamConfig(sync_mode=sync, enable_reading_internals=True,
                                        max_map_size=int(cfg.slam_max_map_size),
                                        planar_constraints=bool(cfg.slam_planar))
    except (ValueError, TypeError, AttributeError) as exc:
        raise TrackingError("cuvslam rejected the SLAM config: {}: {}".format(type(exc).__name__, exc)) from exc


def _new_tracker(vslam: Any, rig: Any, mode: str, cfg: TrackConfig, odom_cfg: Any = None) -> Any:
    """``vslam.Tracker(rig, odom_cfg)`` plus ``cfg.expert_parameters``, errors wrapped as :class:`TrackingError`.

    The construction step of :func:`make_tracker` (no rig check, borders, verbosity or GPU
    warm-up), also used by :func:`run_tracking` to re-create the engine after
    ``cfg.guard_reset_after`` consecutive guarded frames.  *odom_cfg* ``None`` builds one with
    :func:`odometry_config` (a prebuilt tracker handed to ``run_tracking`` without its config).
    """
    if odom_cfg is None:
        odom_cfg = odometry_config(vslam, mode, cfg)
    try:
        slam_cfg = slam_config(vslam, cfg)
        tracker = vslam.Tracker(rig, odom_cfg, slam_cfg) if slam_cfg is not None else vslam.Tracker(rig, odom_cfg)
    except (ValueError, TypeError) as exc:
        raise TrackingError("cuvslam rejected the rig/config for mode '{}': {}: {}".format(
            mode, type(exc).__name__, exc)) from exc
    except RuntimeError as exc:  # the engine itself (not the binding's argument checks): CUDA / library
        raise TrackingError("cuvslam could not create the tracker for mode '{}' ({}: {}); if this is a CUDA "
                            "error run python methods/cuvslam-vio/tools/check_env.py".format(mode, type(exc).__name__, exc)) from exc
    if cfg.expert_parameters:
        try:
            tracker.odom.apply_expert_parameters(dict(cfg.expert_parameters))
        except (ValueError, TypeError, RuntimeError) as exc:  # invalid values raise RuntimeError in the binding
            raise TrackingError("cuvslam rejected expert_parameters {!r}: {}: {}".format(
                cfg.expert_parameters, type(exc).__name__, exc)) from exc
    return tracker


# --------------------------------------------------------------------------------------
# Input records
# --------------------------------------------------------------------------------------

class ImuRecord(NamedTuple):
    """One IMU sample as the loop sees it: gyro ``rad/s`` and accel ``m/s^2`` (raw, gravity included),
    the same field order as ``dataset.ImuSample`` and ``imu0/data.csv``."""

    ts_ns: int
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]


def _vec3(value: Any, what: str) -> Tuple[float, float, float]:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise TrackingError("{} must have 3 components, got {}".format(what, arr.shape))
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _timestamp_of(item: Any) -> Optional[int]:
    if isinstance(item, Mapping):
        value = item.get("ts_ns", item.get("timestamp_ns"))
    else:
        value = getattr(item, "ts_ns", None)
        if value is None:
            value = getattr(item, "timestamp_ns", None)
    return None if value is None else int(value)


def _tag_of(item: Sequence[Any]) -> Optional[str]:
    """Leading string tag of a tuple record (``('imu', ...)`` / ``('stereo', ...)``), else ``None``.

    A numeric string in first position is not a tag (a timestamp spelled as text stays a timestamp).
    """
    if len(item) > 0 and isinstance(item[0], str):
        text = item[0].strip()
        if text and not text.lstrip("+-").replace(".", "", 1).isdigit():
            return text.lower()
    return None


def is_imu_item(item: Any) -> bool:
    """``True`` for an IMU record in a merged stream: ``kind == 'imu'``, a ``gyro`` /
    ``angular_velocities`` field, or a tuple/list tagged ``('imu', ts_ns, gyro, accel)``
    (``datasets/dataset-format.md`` section 12.3); untagged tuples, lists and arrays are
    always frames there."""
    if isinstance(item, (tuple, list)):
        return _tag_of(item) == "imu"
    if isinstance(item, np.ndarray):
        return False
    kind = item.get("kind") if isinstance(item, Mapping) else getattr(item, "kind", None)
    if kind is not None:
        return str(kind) == "imu"
    if isinstance(item, Mapping):
        return "gyro" in item or "angular_velocities" in item
    return hasattr(item, "gyro") or hasattr(item, "angular_velocities")


def as_imu_sample(item: Any) -> ImuRecord:
    """Normalise one IMU sample to :class:`ImuRecord`.

    Accepted forms: ``dataset.ImuSample`` (``ts_ns``, ``gyro``, ``accel``); any object or dict
    with ``ts_ns``/``timestamp_ns`` and ``gyro``/``angular_velocities`` +
    ``accel``/``linear_accelerations`` (so a ``cuvslam.ImuMeasurement`` works too); a tuple
    ``(ts_ns, gyro_xyz, accel_xyz)`` (gyro first, like ``imu0/data.csv``), optionally tagged
    ``('imu', ts_ns, gyro_xyz, accel_xyz)`` (``datasets/dataset-format.md`` section 12.3).
    """
    if isinstance(item, ImuRecord):
        return item
    if isinstance(item, (tuple, list)):
        values = list(item)
        tag = _tag_of(values)
        if tag is not None:
            if tag != "imu":
                raise TrackingError("tuple record tagged {!r} is not an IMU sample (expected 'imu')".format(item[0]))
            values = values[1:]
        if len(values) != 3:
            raise TrackingError("an IMU tuple must be (ts_ns, gyro_xyz, accel_xyz) or ('imu', ts_ns, gyro_xyz, "
                                "accel_xyz), got {} elements".format(len(item)))
        ts, gyro, accel = values
        return ImuRecord(int(ts), _vec3(gyro, "gyro"), _vec3(accel, "accel"))
    ts = _timestamp_of(item)
    if isinstance(item, Mapping):
        gyro = item.get("gyro", item.get("angular_velocities"))
        accel = item.get("accel", item.get("linear_accelerations"))
    else:
        gyro = getattr(item, "gyro", None)
        if gyro is None:
            gyro = getattr(item, "angular_velocities", None)
        accel = getattr(item, "accel", None)
        if accel is None:
            accel = getattr(item, "linear_accelerations", None)
    if ts is None or gyro is None or accel is None:
        raise TrackingError("cannot read an IMU sample from {!r}: need ts_ns/timestamp_ns, gyro/angular_velocities "
                            "and accel/linear_accelerations".format(type(item).__name__))
    return ImuRecord(ts, _vec3(gyro, "gyro"), _vec3(accel, "accel"))


def as_frame(item: Any) -> Tuple[int, List[Any]]:
    """Normalise one stereo frame to ``(ts_ns, [left, right])`` (images in rig camera order).

    Accepted forms: ``dataset.StereoFrame`` (``ts_ns`` + ``images()``, loads the PNGs lazily);
    any object with ``ts_ns``/``timestamp_ns`` and either ``images()``/``images`` or
    ``left``/``right``; a dict with the same keys; a tuple ``(ts_ns, left, right)`` or
    ``(ts_ns, [left, right])``, optionally tagged ``('stereo', ts_ns, [left, right])``
    (``datasets/dataset-format.md`` section 12.3).  Each image may be an array or a PNG
    path (loaded by the loop through ``cuvslam_oak.dataset.load_gray_png``); the arrays are
    validated later against the rig.
    """
    images: Any
    if isinstance(item, (tuple, list)):
        values = list(item)
        tag = _tag_of(values)
        if tag is not None:
            if tag not in ("stereo", "frame"):
                raise TrackingError("tuple record tagged {!r} is not a stereo frame (expected 'stereo')".format(item[0]))
            values = values[1:]
        if len(values) == 3:
            ts, images = values[0], [values[1], values[2]]
        elif len(values) == 2:
            ts, images = values[0], values[1]
        else:
            raise TrackingError("a frame tuple must be (ts_ns, left, right), (ts_ns, images) or ('stereo', ts_ns, "
                                "images), got {} elements".format(len(item)))
        if ts is None:
            raise TrackingError("frame tuple has no timestamp")
        return int(ts), list(images)
    ts = _timestamp_of(item)
    if ts is None:
        raise TrackingError("cannot read a frame from {!r}: need ts_ns or timestamp_ns".format(type(item).__name__))
    if isinstance(item, Mapping):
        if "images" in item:
            images = item["images"]
        elif "left" in item and "right" in item:
            images = [item["left"], item["right"]]
        else:
            raise TrackingError("frame dict needs 'images' or 'left' and 'right'")
    else:
        images = getattr(item, "images", None)
        if callable(images):
            images = images()
        if images is None:
            left = getattr(item, "left", None)
            right = getattr(item, "right", None)
            if left is None or right is None:
                raise TrackingError("cannot read images from {!r}: need images()/images or left and right".format(
                    type(item).__name__))
            images = [left, right]
    return ts, list(images)


def _load_png(path: Union[str, Path], frame_no: int, index: int) -> np.ndarray:
    """GRAY8 PNG -> uint8 array via ``cuvslam_oak.dataset.load_gray_png`` (imports cv2 only when a path is handed in)."""
    from . import dataset as _dataset
    try:
        return _dataset.load_gray_png(path)
    except (OSError, ValueError) as exc:  # DatasetError is a ValueError
        raise TrackingError("frame #{} camera {}: cannot load image {}: {}".format(frame_no, index, path, exc)) from exc


def _prepare_images(images: Sequence[Any], sizes: Sequence[Tuple[int, int]], ts: int, frame_no: int) -> List[np.ndarray]:
    """Validate dtype/shape against the rig and make each image C-contiguous (cuVSLAM's contract).

    An image given as a ``str``/``Path`` is loaded first (:func:`_load_png`).
    """
    if len(images) != len(sizes):
        raise TrackingError("frame #{} (t={}) has {} image(s) but the rig has {} camera(s)".format(
            frame_no, ts, len(images), len(sizes)))
    out: List[np.ndarray] = []
    for index, (img, (width, height)) in enumerate(zip(images, sizes)):
        if isinstance(img, (str, Path)):
            img = _load_png(img, frame_no, index)
        arr = np.asarray(img)
        if arr.size == 0:
            out.append(arr)  # cuVSLAM: an empty array means "no image for this camera"
            continue
        if arr.dtype != np.uint8:
            raise TrackingError("frame #{} camera {}: dtype {} but cuVSLAM needs uint8 (GRAY8)".format(
                frame_no, index, arr.dtype))
        if arr.ndim not in (2, 3) or (arr.ndim == 3 and arr.shape[2] not in (1, 3)):
            raise TrackingError("frame #{} camera {}: shape {} but cuVSLAM needs (H, W), (H, W, 1) or (H, W, 3)".format(
                frame_no, index, arr.shape))
        if width > 0 and height > 0 and (arr.shape[0] != height or arr.shape[1] != width):
            raise TrackingError("frame #{} camera {}: image is {}x{} but the rig camera size is {}x{}; build the rig "
                                "for the recorded resolution".format(frame_no, index, arr.shape[1], arr.shape[0],
                                                                     width, height))
        if not arr.flags.c_contiguous:
            arr = np.ascontiguousarray(arr)
        out.append(arr)
    return out


# --------------------------------------------------------------------------------------
# Events
# --------------------------------------------------------------------------------------

@dataclass
class PoseEvent:
    """One successfully tracked frame, handed to the ``on_pose`` callback.

    ``translation``/``rotation_xyzw`` are what the TUM row holds (``world_from_rig``, stitched
    when ``cfg.stitch_resets``; the quaternion is unit-normalised in float64); ``pose`` is the
    raw ``vslam.Pose`` of this segment;
    ``covariance`` the 36 ``covariance_xyz_rpy`` floats; ``gravity`` the rig-frame gravity in
    ``vio`` mode (``None`` until cuVSLAM has estimated it); ``images`` the arrays that were
    tracked, ``tracker`` the live tracker (``tracker.get_last_observations(0)`` for overlays);
    ``coasted`` marks an estimate made without the engine.
    """

    index: int                       # 0-based count of frames handed to track()
    frame_no: int                    # 1-based count of frames seen (warm-up included)
    ts_ns: int
    translation: np.ndarray          # float64 (3,) metres
    rotation_xyzw: np.ndarray        # float64 (4,)
    segment: int
    track_ms: float
    pose: Any = field(repr=False, default=None)
    covariance: Optional[List[float]] = field(repr=False, default=None)
    gravity: Optional[List[float]] = None
    images: List[np.ndarray] = field(repr=False, default_factory=list)
    tracker: Any = field(repr=False, default=None)
    #: ``True`` for an estimate made while the engine gave no usable pose (module docstring); then
    #: ``pose``, ``covariance``, ``gravity`` and ``tracker`` are ``None`` and ``track_ms`` is 0.
    coasted: bool = False
    #: The SLAM pose of this frame (``cfg.slam``, anchored like ``translation``), else ``None``.
    slam_translation: Optional[np.ndarray] = None
    slam_rotation_xyzw: Optional[np.ndarray] = None

    @property
    def timestamp_s(self) -> float:
        return self.ts_ns / _NS_PER_S

    @property
    def tum_row(self) -> Tuple[int, float, float, float, float, float, float, float]:
        t, q = self.translation, self.rotation_xyzw
        return (int(self.ts_ns), float(t[0]), float(t[1]), float(t[2]),
                float(q[0]), float(q[1]), float(q[2]), float(q[3]))


# --------------------------------------------------------------------------------------
# Internals of the loop
# --------------------------------------------------------------------------------------

def _stderr_log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


class _Warnings:
    """Rate-limited warning channel: the first few of a kind in full, then every 100th."""

    def __init__(self, log: Optional[Callable[[str], None]]) -> None:
        self.log = log
        self.counts: Dict[str, int] = collections.defaultdict(int)

    def emit(self, kind: str, message: str) -> None:
        self.counts[kind] += 1
        n = self.counts[kind]
        if self.log is None:
            return
        if n <= _WARN_FIRST or n % _WARN_EVERY == 0:
            suffix = "" if n <= _WARN_FIRST else " ({} so far)".format(n)
            self.log("warning: {}{}".format(message, suffix))


@dataclass
class _Counters:
    frames_seen: int = 0
    frames_warmup: int = 0
    frames_attempted: int = 0
    frames_tracked: int = 0
    frames_failed: int = 0
    frames_nonfinite: int = 0
    frames_out_of_order: int = 0
    frames_without_imu: int = 0
    frame_gaps: int = 0
    frame_gaps_over_max: int = 0
    max_frame_gap_ns: int = 0
    resets: int = 0
    imu_registered: int = 0
    imu_dropped_old: int = 0
    imu_out_of_order: int = 0
    imu_skipped_warmup: int = 0
    imu_nonfinite: int = 0
    imu_ignored: int = 0
    imu_gaps: int = 0
    max_imu_gap_ns: int = 0
    frames_guarded: int = 0
    guard_starved: int = 0
    guard_degenerate: int = 0
    guard_jump: int = 0
    guard_spin: int = 0
    guard_resets: int = 0
    frames_coasted: int = 0
    loop_closures: int = 0
    pgo_events: int = 0
    slam_map_resets: int = 0


class _ImuFeeder:
    """Buffers IMU samples and registers those with ``ts <= image ts`` before each ``track()``.

    The source is an iterator (``next()`` is called until one sample newer than the image has
    been seen: a dataset generator) or a ``queue.Queue`` (drained without blocking, the
    ``run_vio.py`` pattern).  Samples arriving through :meth:`push` (merged streams) join the same
    deque.  Only one sample beyond the image timestamp is ever pulled ahead; it is held back
    for the next frame.  Because the iterator form reads that one sample ahead with no timeout,
    a *live* IMU stream must be a queue or part of the merged frame stream
    (``OakStereoImuSource.events()``), never a blocking generator: a stalled stream would block
    the loop forever.

    During warm-up (``register=False``) nothing is registered; the samples inside
    ``warmup_window_ns`` before the current frame stay buffered for the first ``track()``, older
    ones are discarded and counted (``imu_skipped_warmup``).
    """

    def __init__(self, source: Any, vslam: Any, tracker: Any, counters: _Counters,
                 warnings: _Warnings, imu_gap_ns: int, warmup_window_ns: int = 0,
                 correction: Any = None) -> None:
        self._iter: Optional[Iterator[Any]] = None
        self._queue: Any = None
        if source is not None:
            if hasattr(source, "get_nowait"):
                self._queue = source
            else:
                self._iter = iter(source)
        self.vslam = vslam
        self.tracker = tracker
        self.counters = counters
        self.warnings = warnings
        self.imu_gap_ns = int(imu_gap_ns)
        self.warmup_window_ns = max(0, int(warmup_window_ns))
        self.correction = correction  # imu_calib.ImuCorrection or None (raw values)
        #: ``(ts_ns, gyro)`` of the last samples taken from the buffer (corrected), for gyro coasting.
        self.recent: Deque[Tuple[int, np.ndarray]] = collections.deque(maxlen=4000)
        self.pending: Deque[ImuRecord] = collections.deque()
        self.exhausted = source is None
        self.last_registered_ts: Optional[int] = None

    @property
    def held_back(self) -> int:
        return len(self.pending)

    def push(self, record: ImuRecord) -> None:
        self.pending.append(record)

    def _fill_until(self, ts: int) -> None:
        if self._queue is not None:
            while True:
                try:
                    item = self._queue.get_nowait()
                except Exception:  # noqa: BLE001 - queue.Empty (or a closed queue)
                    break
                self.pending.append(as_imu_sample(item))
            return
        if self._iter is None:
            return
        while not self.exhausted and (not self.pending or self.pending[-1].ts_ns <= ts):
            try:
                item = next(self._iter)
            except StopIteration:
                self.exhausted = True
                break
            self.pending.append(as_imu_sample(item))

    def register_until(self, ts: int, last_tracker_ts: Optional[int], register: bool,
                       engine: bool = True) -> Tuple[int, int]:
        """Take every buffered sample with ``ts_ns <= ts`` and return ``(taken, arrived)``: how many
        were handed to the engine (or, with ``engine=False``, only kept in :attr:`recent` for gyro
        coasting in ``stereo`` mode) and how many were removed from the buffer for this interval
        (the difference was dropped as old / out of order / non-finite and counted).  During
        warm-up (``register=False``) nothing is taken (``(0, 0)``); the last ``warmup_window_ns``
        of the samples stay buffered and the older ones are discarded."""
        self._fill_until(ts)
        n = 0
        arrived = 0
        c = self.counters
        if not register:
            cutoff = ts - self.warmup_window_ns  # window 0: everything up to this frame is discarded
            while self.pending and self.pending[0].ts_ns <= cutoff:
                self.pending.popleft()
                c.imu_skipped_warmup += 1
            return 0, 0
        while self.pending and self.pending[0].ts_ns <= ts:
            rec = self.pending.popleft()
            arrived += 1
            if last_tracker_ts is not None and rec.ts_ns < last_tracker_ts:
                c.imu_dropped_old += 1
                continue
            if self.last_registered_ts is not None and rec.ts_ns < self.last_registered_ts:
                c.imu_out_of_order += 1
                self.warnings.emit("imu_order", "IMU sample at {} is older than the last registered {}; dropped".format(
                    rec.ts_ns, self.last_registered_ts))
                continue
            if not all(math.isfinite(v) for v in rec.gyro + rec.accel):
                c.imu_nonfinite += 1
                self.warnings.emit("imu_nonfinite", "IMU sample at {} has non-finite values; dropped".format(rec.ts_ns))
                continue
            if self.last_registered_ts is not None:
                gap = rec.ts_ns - self.last_registered_ts
                if gap > self.imu_gap_ns:
                    c.imu_gaps += 1
                    c.max_imu_gap_ns = max(c.max_imu_gap_ns, gap)
                    self.warnings.emit("imu_gap", "IMU gap {:.2f} ms exceeds {:.2f} ms".format(
                        gap / 1e6, self.imu_gap_ns / 1e6))
            gyro, accel = rec.gyro, rec.accel
            if self.correction is not None:
                gyro, accel = self.correction.apply(gyro, accel)
            self.recent.append((int(rec.ts_ns), np.asarray(gyro, dtype=np.float64)))
            if engine:
                measurement = self.vslam.ImuMeasurement(timestamp_ns=int(rec.ts_ns),
                                                        linear_accelerations=list(accel),
                                                        angular_velocities=list(gyro))
                self.tracker.register_imu_measurement(0, measurement)
                c.imu_registered += 1
            self.last_registered_ts = rec.ts_ns
            n += 1
        return n, arrived


class _Coaster:
    """Pose estimates while the engine gives none (module docstring).

    :meth:`accepted` records every measured pose (velocity window); :meth:`coast` returns the
    estimate for a frame without a usable pose, or ``None`` when there is nothing to coast from or
    the gap exceeds ``cfg.coast_max_s``.  The rotation integrates the gyroscope samples the feeder
    saw between the previous output and this frame (rig-frame rates through *R_rig_imu*), the
    position continues at the velocity fitted over the last accepted poses, decaying exponentially.
    """

    def __init__(self, cfg: TrackConfig, R_rig_imu: Optional[np.ndarray], feeder: "_ImuFeeder") -> None:
        self.cfg = cfg
        self.R_rig_imu = R_rig_imu
        self.feeder = feeder
        self.history: Deque[Tuple[int, np.ndarray]] = collections.deque()
        self.last_out_ts: Optional[int] = None
        self.gap_start_ts: Optional[int] = None
        self.gap_start_p: Optional[np.ndarray] = None
        self.gap_v = np.zeros(3)
        self.gap_rows = 0
        self.gap_first_row = 0
        self.gaps: List[Tuple[int, int]] = []       #: (first row index, coasted rows) per bridged gap

    def accepted(self, ts: int, T: np.ndarray) -> None:
        if self.gap_start_ts is not None:
            if self.gap_rows:
                self.gaps.append((self.gap_first_row, self.gap_rows))
            self.gap_start_ts = None
            self.gap_rows = 0
        self.history.append((int(ts), np.array(T[:3, 3], dtype=np.float64)))
        window_ns = int(round(self.cfg.coast_velocity_window_s * _NS_PER_S))
        while self.history and self.history[0][0] < ts - window_ns:
            self.history.popleft()
        self.last_out_ts = int(ts)

    def velocity(self) -> np.ndarray:
        """Velocity (m/s, world frame) over the window: the component-wise median of the frame-to-frame
        velocities, so the few corrupted poses the guard lets through just before it trips (measured:
        1.4-2.9 m/s for 0.3 s before a jump, at walking pace) do not set the coasting speed.  Zero when
        the window holds too few poses."""
        if len(self.history) < 4:
            return np.zeros(3)
        t = np.array([h[0] for h in self.history], dtype=np.float64) / _NS_PER_S
        if t[-1] - t[0] < 0.1:
            return np.zeros(3)
        p = np.array([h[1] for h in self.history])
        dt = np.diff(t)
        v = np.median(np.diff(p, axis=0) / dt[:, None], axis=0)
        speed = float(np.linalg.norm(v))
        limit = self.cfg.max_speed_mps
        if math.isfinite(limit) and speed > limit > 0.0:
            v *= limit / speed
        return v

    def gyro_rotvec(self, t0: int, t1: int) -> np.ndarray:
        """Rig-frame rotation vector integrated from the gyro samples in ``(t0, t1]`` (zero when none)."""
        if self.R_rig_imu is None:
            return np.zeros(3)
        total = np.zeros(3)
        prev = t0
        omega: Optional[np.ndarray] = None
        for ts, gyro in self.feeder.recent:
            if ts <= t0:
                continue
            if ts > t1:
                break
            omega = self.R_rig_imu @ gyro
            total += omega * ((ts - prev) / _NS_PER_S)
            prev = ts
        if omega is not None and t1 > prev:
            total += omega * ((t1 - prev) / _NS_PER_S)
        return total

    def predict(self, ts: int, last_T: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """The estimate for time *ts* without recording it (the re-anchor target of a returning pose)."""
        if last_T is None or self.last_out_ts is None or ts <= self.last_out_ts:
            return None
        if self.gap_start_ts is None:
            start_ts, start_p, v = self.last_out_ts, np.array(last_T[:3, 3], dtype=np.float64), self.velocity()
        else:
            start_ts, start_p, v = self.gap_start_ts, self.gap_start_p, self.gap_v
        tau = (ts - start_ts) / _NS_PER_S
        if tau > self.cfg.coast_max_s:
            return None
        decay = self.cfg.coast_decay_s
        travelled = decay * (1.0 - math.exp(-tau / decay)) if decay > 0.0 else tau
        R = last_T[:3, :3] @ _exp_so3(self.gyro_rotvec(self.last_out_ts, ts))
        return _rig.se3(R, start_p + v * travelled)

    def coast(self, ts: int, last_T: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """The estimate for a frame without a usable pose; records it as an output."""
        if last_T is None or self.last_out_ts is None or ts <= self.last_out_ts:
            return None
        if self.gap_start_ts is None:
            self.gap_start_ts = self.last_out_ts
            self.gap_start_p = np.array(last_T[:3, 3], dtype=np.float64)
            self.gap_v = self.velocity()
            self.gap_rows = 0
        T_est = self.predict(ts, last_T)
        if T_est is None:
            return None
        self.last_out_ts = int(ts)
        self.gap_rows += 1
        return T_est

    def finish(self) -> List[Tuple[int, int]]:
        gaps = list(self.gaps)
        if self.gap_start_ts is not None and self.gap_rows:
            gaps.append((self.gap_first_row, self.gap_rows))
        return gaps


def _slam_rows_from(poses: Any, anchors: Dict[int, np.ndarray]) -> List[Tuple[int, float, float, float, float, float, float, float]]:
    """TUM rows from ``get_all_slam_poses()`` (``PoseStamped`` list) for the accepted frames, each anchored
    like the odometry row of the same timestamp.  The SLAM layer also holds the frames the guard
    rejected (it sees every frame the odometry tracked); those are left out, as in the odometry rows."""
    rows: List[Tuple[int, float, float, float, float, float, float, float]] = []
    for stamped in list(poses or []):
        try:
            ts = int(stamped.timestamp_ns)
            if ts not in anchors:
                continue
            pose = stamped.pose
            t = np.array(pose.translation, dtype=np.float64).reshape(3)
            q = np.array(pose.rotation, dtype=np.float64).reshape(4)
        except (AttributeError, TypeError, ValueError):
            continue
        norm = float(np.linalg.norm(q))
        if not (np.all(np.isfinite(t)) and math.isfinite(norm) and norm > 0.0):
            continue
        q, t = _matrix_pose(anchors[ts] @ _pose_matrix(q / norm, t))
        rows.append((ts, float(t[0]), float(t[1]), float(t[2]), float(q[0]), float(q[1]), float(q[2]), float(q[3])))
    return rows


def _slam_snapshot(tracker: Any) -> Any:
    """``tracker.get_all_slam_poses()`` or ``[]`` (never raises)."""
    try:
        return list(tracker.get_all_slam_poses() or [])
    except Exception:  # noqa: BLE001 - diagnostics only
        return []


def _pose_matrix(q: np.ndarray, t: np.ndarray) -> np.ndarray:
    return _rig.se3(_rig.quaternion_xyzw_to_matrix(q), t)


def _matrix_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return _rig.matrix_to_quaternion_xyzw(T[:3, :3]), np.array(T[:3, 3], dtype=np.float64)


def _fmt_pos(p: Optional[Sequence[float]]) -> str:
    if p is None:
        return "-"
    return "({:.3f}, {:.3f}, {:.3f})".format(float(p[0]), float(p[1]), float(p[2]))


def _observation_count(tracker: Any) -> Optional[int]:
    """``len(tracker.get_last_observations(0))``, or ``None`` when the engine cannot export observations.

    The real binding copies the list (a few hundred small objects) on every call; that is far
    below ``track()``'s cost, but the guard asks only once per usable pose.  Any exception
    (export disabled, a binding without the method) yields ``None``, which never triggers the
    guard.
    """
    try:
        return len(tracker.get_last_observations(0))
    except Exception:  # noqa: BLE001 - a diagnostic must never break tracking
        return None


def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Rotation angle in degrees between two unit quaternions (``2 * acos(|q1 . q2|)``)."""
    dot = min(1.0, abs(float(np.dot(q1, q2))))
    return math.degrees(2.0 * math.acos(dot))


def _zero_position_covariance(world_from_rig: Any) -> bool:
    """``True`` when the x/y/z variances of ``covariance_xyz_rpy`` (row-major 6x6: indices 0, 7, 14) are all exactly 0.0.

    Measured with the real engine: the block is exactly zero on about half of the blank-wall
    glitch frames and on well under 1 % of normal frames.  Only exact zeros count: the engine
    also returns a placeholder variance of 1.0 per axis (sigma 1.732 m) on ~30 % of perfectly
    good frames of a still camera, so any magnitude rule would reject good data.  A missing or
    short covariance is ``False`` (never triggers).
    """
    try:
        cov = world_from_rig.covariance_xyz_rpy
        return len(cov) >= 15 and float(cov[0]) == 0.0 and float(cov[7]) == 0.0 and float(cov[14]) == 0.0
    except Exception:  # noqa: BLE001 - a diagnostic must never break tracking
        return False


# --------------------------------------------------------------------------------------
# The loop
# --------------------------------------------------------------------------------------

def run_tracking(vslam: Any, rig: Any, mode: str, frame_iter: Iterable[Any],
                 imu_iter: Optional[Any] = None, cfg: Optional[TrackConfig] = None,
                 on_pose: Optional[Callable[[PoseEvent], None]] = None,
                 on_stats: Optional[Callable[[Dict[str, Any]], None]] = None,
                 *, tracker: Any = None, odom_cfg: Any = None) -> Dict[str, Any]:
    """Track every frame of *frame_iter* with cuVSLAM and return a summary dict.

    *vslam*: the cuvslam module (real or fake).  *rig*: a ``vslam.Rig`` from
    :func:`cuvslam_oak.rig.build_rig` (``mode='vio'`` for an IMU).  *mode*: ``'stereo'`` or
    ``'vio'`` (aliases in :data:`MODE_ALIASES`).  *frame_iter*: stereo frames in any
    :func:`as_frame` form, or a merged stream that also carries IMU records (see
    :func:`is_imu_item`, e.g. ``EurocReader.merged()``).  *imu_iter*: IMU samples in any
    :func:`as_imu_sample` form, sorted by time - an iterator (a dataset generator, or a live
    generator blocking on the camera queue) or a ``queue.Queue`` drained without blocking; ``None``
    when the frames carry the IMU records or in ``'stereo'`` mode, where an IMU source is never
    consumed.  *cfg*: :class:`TrackConfig`.  *on_pose* receives a :class:`PoseEvent` per valid
    pose; *on_stats* a dict every ``cfg.stats_interval_s`` seconds (keys: ``frames_seen``,
    ``frames_warmup``, ``frames_attempted``, ``frames_tracked``, ``frames_failed``, ``resets``,
    ``frames_guarded``, ``guard_resets``, ``frames_coasted``,
    ``fps`` (frames handed to ``track()`` per wall second over the interval), ``fps_mean``
    (the same over ``tracking_s``, the wall time since the first ``track()``), ``track_ms``
    (mean engine time over the interval), ``elapsed_s`` (wall time since the first item,
    warm-up included), ``tracking_s``, ``data_s``, ``position``, ``gravity``,
    ``imu_registered``, ``imu_held_back``, ``last_ts_ns``, ``total_frames`` (when
    ``len(frame_iter)`` is known), ``final``).  *tracker*: an already built ``vslam.Tracker`` to
    use instead of :func:`make_tracker` (borders, verbosity and GPU warm-up are then the
    caller's), or the ``(tracker, odometry_config)`` pair :func:`make_tracker` returns;
    *odom_cfg*: its ``Tracker.OdometryConfig``.  The real binding does not expose the config an
    ``Odometry`` was built with, so with a prebuilt tracker and no *odom_cfg* the summary's
    ``async_sba`` and the ``frame_gaps_over_max`` threshold come from *cfg* (``realtime``,
    ``max_frame_delta_s``) rather than from the tracker.

    Ordering rules enforced here so the engine never sees a violation: frame timestamps must be
    strictly increasing (a repeated/older frame is skipped and counted); IMU samples with
    ``ts <= image ts`` are registered before that image, in non-decreasing order, and samples
    older than the last tracker timestamp are dropped; samples newer than the last image are held
    back (``imu_held_back`` in the summary) and never registered.  During warm-up frames the IMU
    samples of the last ``cfg.warmup_imu_window_s`` seconds (default 0) stay buffered and are
    registered before the first ``track()``; older ones are discarded (``imu_skipped_warmup``).  A live *imu_iter*
    must be a ``queue.Queue``, or the IMU records must travel inside a merged *frame_iter* such
    as ``OakStereoImuSource.events()``: the iterator form reads one sample ahead of every image
    with no timeout, so a blocking generator whose IMU stream stalls would block the loop
    forever.

    Stops at the end of *frame_iter*, after ``cfg.max_frames`` tracked frames, when
    ``cfg.should_stop()`` returns ``True`` or on ``KeyboardInterrupt`` (``stopped_by`` says which).
    Raises :class:`TrackingError` for a rig/mode mismatch, a broken CUDA environment, a frame the
    engine rejects (dtype/shape), an unreadable record or a pose that cannot be turned into an
    SE(3) matrix; the summary is then lost, so callers that need partial results catch it around
    their own ``on_pose`` accumulation.  A pose holding a NaN/inf or a zero-norm quaternion does
    not raise: it is counted (``frames_nonfinite``) and its row skipped.

    Summary keys: ``mode``, ``odometry_mode``, ``realtime``, ``async_sba``, ``cuvslam_version``,
    ``fake``, the counters (``frames_seen``, ``frames_warmup``, ``frames_attempted`` (equal to
    ``frames_tracked + frames_failed``), ``frames_tracked``, ``frames_failed`` (``None`` poses
    plus the ``frames_nonfinite`` sub-count: poses holding a NaN/inf or a zero-norm quaternion,
    skipped without a reset), ``frames_out_of_order``, ``frames_without_imu`` (tracked frames
    for which no IMU sample was registered since the previous one, whether none arrived or every
    arrival was dropped; the warning says which), ``frame_gaps``, ``frame_gaps_over_max``,
    ``max_frame_gap_ms``,
    ``resets`` (engine losses followed by a valid pose, plus the guard's engine resets),
    ``imu_registered``, ``imu_dropped_old``, ``imu_out_of_order``,
    ``imu_skipped_warmup``, ``imu_nonfinite``, ``imu_ignored``, ``imu_gaps``, ``max_imu_gap_ms``,
    ``frames_guarded`` (usable poses skipped by the starvation guard, the sum of
    ``guard_starved``, ``guard_degenerate``, ``guard_jump`` and ``guard_spin``; each guarded
    frame carries the first matching reason in that order), ``guard_resets`` (trackers re-created after
    ``cfg.guard_reset_after`` consecutive guarded frames; counted in ``resets`` and opening a
    new segment when a pose had been tracked before)), ``guard_gaps`` (``(start_index,
    n_frames)`` per guarded stretch: the ``track()`` index of its first guarded frame and how
    many frames it guarded; a tracking loss inside a stretch does not end it), ``frames_coasted``
    (rows estimated while the engine gave no usable pose), ``coast_gaps`` (``(first row index,
    rows)`` per bridged gap), ``coast_gyro_used`` (whether the rotation followed the gyroscope),
    ``observations_min``
    / ``observations_median`` / ``observations_max`` (camera-0 observation counts over every
    usable pose, accepted or guarded, while the guard is on and the engine exports them; else
    ``None``),
    ``imu_held_back`` (buffered samples newer than the last image that were never registered: at
    most 1 for an iterator source, which is read only one sample ahead so a live generator never
    blocks the end of a run; all of them for a queue or merged stream), ``imu_source_ignored``,
    ``segments`` (tracked-frame index where each segment starts), ``first_ts_ns``,
    ``last_ts_ns``, ``duration_s`` (data time between the first and last frame handed to
    ``track()``), ``wall_s`` (wall time from the first item of *frame_iter* to the end, warm-up
    included), ``wall_tracking_s`` (wall time from the first ``track()`` call to the end),
    ``mean_fps`` (``frames_attempted / wall_tracking_s``, so warm-up frames and IMU-only items
    before the first ``track()`` do not deflate it), ``engine_fps`` (``frames_attempted /
    sum(track time)``), ``track_ms_mean``,
    ``track_ms_max``, ``path_length_m``, ``endpoint_offset_m``, ``max_from_start_m``,
    ``start_position``, ``end_position``, ``poses``, ``gravity``, ``final_landmarks`` (count, or
    ``None`` when not exported; after a guard reset only the last tracker's), ``stopped_by``,
    ``config`` (``cfg.to_dict()``), ``tum_rows``
    (count) and ``rows`` (the TUM rows, ``(ts_ns, tx, ty, tz, qx, qy, qz, qw)`` tuples for
    :func:`write_tum`).
    """
    mode = normalize_mode(mode)
    cfg = cfg if cfg is not None else TrackConfig()
    log = cfg.log if cfg.log is not None else _stderr_log
    warnings = _Warnings(log if cfg.verbose else None)
    vio = mode == "vio"

    if tracker is None:
        tracker, odom_cfg = make_tracker(vslam, rig, mode, cfg)
    elif isinstance(tracker, (tuple, list)):  # the (tracker, odometry_config) pair make_tracker() returns
        if len(tracker) != 2:
            raise TrackingError("tracker must be a vslam.Tracker or the (tracker, odom_cfg) pair make_tracker() "
                                "returns, got a sequence of {} elements".format(len(tracker)))
        tracker, odom_cfg = tracker[0], (odom_cfg if odom_cfg is not None else tracker[1])
    if not callable(getattr(tracker, "track", None)):
        raise TrackingError("tracker has no track() method: {!r}".format(type(tracker).__name__))
    # The real binding keeps no reference to the config an Odometry was built with; fall back to cfg.
    max_frame_delta_s = float(getattr(odom_cfg, "max_frame_delta_s", cfg.max_frame_delta_s))
    cameras = list(rig.cameras)
    sizes = [(int(cam.size[0]), int(cam.size[1])) for cam in cameras]
    if not sizes:
        raise TrackingError("the rig has no cameras")

    counters = _Counters()
    read_imu = vio or (cfg.coast and cfg.coast_gyro)      # stereo mode reads the IMU for coasting only
    feeder = _ImuFeeder(imu_iter if read_imu else None, vslam, tracker, counters, warnings, cfg.imu_jitter_threshold_ns,
                        warmup_window_ns=int(round(cfg.warmup_imu_window_s * _NS_PER_S)),
                        correction=cfg.imu_correction)
    imu_source_ignored = (not read_imu) and imu_iter is not None
    R_rig_imu: Optional[np.ndarray] = None
    if cfg.coast and cfg.coast_gyro:
        imus = list(getattr(rig, "imus", []) or [])
        try:
            if imus:
                R_rig_imu = _rotation_matrix_of(imus[0].rig_from_imu, "rig.imus[0].rig_from_imu")
            elif cfg.coast_rig_from_imu is not None:
                R_rig_imu = _rotation_matrix_of(cfg.coast_rig_from_imu, "coast_rig_from_imu")
        except (ValueError, TypeError) as exc:
            raise TrackingError("cannot read the IMU rotation for gyro coasting: {}".format(exc)) from exc
        if R_rig_imu is None and cfg.verbose:
            log("note: no IMU rotation known (stereo rig without coast_rig_from_imu): coasting holds the heading")
    coaster = _Coaster(cfg, R_rig_imu, feeder)
    slam_rows: List[Tuple[int, float, float, float, float, float, float, float]] = []
    slam_optimized: List[Tuple[int, float, float, float, float, float, float, float]] = []
    slam_anchors: Dict[int, np.ndarray] = {}        # accepted frame ts -> anchor applied to its rows
    slam_lc_prev = slam_pgo_prev = False
    if imu_source_ignored and cfg.verbose:
        log("note: mode 'stereo' ignores the IMU source; use mode 'vio' to fuse it")
    if vio and imu_iter is None and cfg.verbose:
        log("note: mode 'vio' without imu_iter: IMU records are expected inside frame_iter (merged stream)")

    rows: List[Tuple[int, float, float, float, float, float, float, float]] = []
    segments: List[int] = []
    stopped_by = "end"
    total_frames: Optional[int] = None
    try:
        total_frames = len(frame_iter)  # type: ignore[arg-type]
    except TypeError:
        total_frames = None

    prev_frame_ts: Optional[int] = None
    last_tracker_ts: Optional[int] = None
    first_tracked_ts: Optional[int] = None
    last_attempted_ts: Optional[int] = None
    lost = False
    anchor: Optional[np.ndarray] = None
    last_valid_T: Optional[np.ndarray] = None
    # Starvation guard: the last ACCEPTED raw pose of the current engine segment (reset on a loss
    # and on an engine reset), the guarded streak that arms the engine reset, the current no-row
    # stretch (start index + guarded frames; it also arms the re-anchor), and a flag set by an
    # engine reset so that the next accepted pose is chained onto the last good pose.
    prev_t_raw: Optional[np.ndarray] = None
    prev_q_raw: Optional[np.ndarray] = None
    prev_ts: Optional[int] = None
    consecutive_guarded = 0
    gap_start: Optional[int] = None
    gap_frames = 0
    reanchor = False
    gap_engine_restarted = False      # the engine's world frame restarted (loss / guard reset) during this gap
    guard_gaps: List[Tuple[int, int]] = []
    obs_counts: List[int] = []
    last_position: Optional[List[float]] = None
    last_gravity: Optional[List[float]] = None
    track_time_s = 0.0
    track_ms_max = 0.0
    wall_start: Optional[float] = None        # first item of frame_iter (warm-up included)
    track_wall_start: Optional[float] = None  # just before the first track() call
    stats_t0: Optional[float] = None
    stats_frames = 0
    stats_track_s = 0.0

    def emit_stats(now: float, final: bool = False) -> None:
        nonlocal stats_t0, stats_frames, stats_track_s
        window = now - (stats_t0 if stats_t0 is not None else now)
        elapsed = now - (wall_start if wall_start is not None else now)
        tracking = now - (track_wall_start if track_wall_start is not None else now)
        fps = stats_frames / window if window > 0 else 0.0
        stats = {
            "frames_seen": counters.frames_seen,
            "frames_warmup": counters.frames_warmup,
            "frames_attempted": counters.frames_attempted,
            "frames_tracked": counters.frames_tracked,
            "frames_failed": counters.frames_failed,
            "resets": counters.resets,
            "frames_guarded": counters.frames_guarded,
            "guard_resets": counters.guard_resets,
            "frames_coasted": counters.frames_coasted,
            "loop_closures": counters.loop_closures,
            "slam_rows": len(slam_rows),
            "fps": fps,
            "fps_mean": counters.frames_attempted / tracking if tracking > 0 else 0.0,
            "track_ms": 1e3 * stats_track_s / stats_frames if stats_frames else 0.0,
            "elapsed_s": elapsed,
            "tracking_s": tracking,
            "data_s": ((last_attempted_ts - first_tracked_ts) / _NS_PER_S
                       if first_tracked_ts is not None and last_attempted_ts is not None else 0.0),
            "position": None if last_position is None else list(last_position),
            "gravity": None if last_gravity is None else list(last_gravity),
            "imu_registered": counters.imu_registered,
            "imu_held_back": feeder.held_back,
            "last_ts_ns": last_attempted_ts,
            "total_frames": total_frames,
            "final": final,
        }
        if on_stats is not None:
            on_stats(stats)
        if cfg.verbose:
            log("[track] {:7.1f} s | seen {} tracked {} failed {} guarded {} resets {} | fps {:.1f} (mean {:.1f}) | "
                "track {:.1f} ms | pos {} | imu {}{}".format(
                    elapsed, counters.frames_seen, counters.frames_tracked, counters.frames_failed,
                    counters.frames_guarded, counters.resets, fps, stats["fps_mean"], stats["track_ms"],
                    _fmt_pos(last_position), counters.imu_registered, " | final" if final else ""))
        stats_t0 = now
        stats_frames = 0
        stats_track_s = 0.0

    try:
        for item in frame_iter:
            now = time.monotonic()
            if wall_start is None:
                wall_start = now
                stats_t0 = now
            if cfg.should_stop is not None and cfg.should_stop():
                stopped_by = "should_stop"
                break

            if is_imu_item(item):
                if read_imu:
                    feeder.push(as_imu_sample(item))
                else:
                    counters.imu_ignored += 1
                continue

            ts, raw_images = as_frame(item)
            counters.frames_seen += 1
            frame_no = counters.frames_seen

            if prev_frame_ts is not None:
                gap = ts - prev_frame_ts
                if gap <= 0:
                    counters.frames_out_of_order += 1
                    warnings.emit("frame_order", "frame #{} at {} is not newer than the previous frame {}; skipped".format(
                        frame_no, ts, prev_frame_ts))
                    continue
                if gap > cfg.jitter_threshold_ns:
                    counters.frame_gaps += 1
                    counters.max_frame_gap_ns = max(counters.max_frame_gap_ns, gap)
                    over_max = gap > max_frame_delta_s * _NS_PER_S
                    if over_max:
                        counters.frame_gaps_over_max += 1
                    warnings.emit("frame_gap", "camera stream gap {:.2f} ms exceeds {:.2f} ms before frame #{}{}".format(
                        gap / 1e6, cfg.jitter_threshold_ns / 1e6, frame_no,
                        " (above max_frame_delta_s: tracking will restart)" if over_max else ""))
            prev_frame_ts = ts

            warming = counters.frames_warmup < cfg.warmup_frames
            drops_before = (counters.imu_dropped_old, counters.imu_out_of_order, counters.imu_nonfinite)
            n_imu, n_imu_arrived = (feeder.register_until(ts, last_tracker_ts, register=not warming, engine=vio)
                                    if read_imu else (0, 0))
            if warming:
                counters.frames_warmup += 1
                continue
            if vio and n_imu == 0 and counters.frames_attempted > 0:
                # Nothing registered for this interval: either no sample arrived, or every one that
                # did was dropped (late from a lagging live queue, out of order, non-finite).
                counters.frames_without_imu += 1
                if n_imu_arrived:
                    drops_after = (counters.imu_dropped_old, counters.imu_out_of_order, counters.imu_nonfinite)
                    old, ooo, nonfinite = (a - b for a, b in zip(drops_after, drops_before))
                    warnings.emit("no_imu", "no IMU samples registered between {} and {} (frame #{}): {} arrived "
                                  "but were dropped ({} old, {} out of order, {} non-finite)".format(
                                      last_attempted_ts, ts, frame_no, n_imu_arrived, old, ooo, nonfinite))
                else:
                    warnings.emit("no_imu", "no IMU samples between {} and {} (frame #{})".format(
                        last_attempted_ts, ts, frame_no))

            images = _prepare_images(raw_images, sizes, ts, frame_no)
            if track_wall_start is None:
                track_wall_start = time.monotonic()
            t0 = time.perf_counter()
            try:
                estimate, slam_pose = tracker.track(ts, images)
            except (ValueError, TypeError) as exc:
                raise TrackingError("cuvslam rejected frame #{} (t={}): {}: {}".format(
                    frame_no, ts, type(exc).__name__, exc)) from exc
            except RuntimeError as exc:  # engine / CUDA failure mid-run (not the binding's argument checks)
                raise TrackingError("cuvslam failed on frame #{} (t={}) after {} tracked frame(s): {}: {}; if this "
                                    "is a CUDA error run python methods/cuvslam-vio/tools/check_env.py".format(
                                        frame_no, ts, counters.frames_attempted, type(exc).__name__, exc)) from exc
            dt = time.perf_counter() - t0
            track_time_s += dt
            stats_track_s += dt
            track_ms_max = max(track_ms_max, dt * 1e3)
            last_tracker_ts = ts
            last_attempted_ts = ts
            if first_tracked_ts is None:
                first_tracked_ts = ts
            index = counters.frames_attempted
            counters.frames_attempted += 1
            stats_frames += 1

            def emit_coasted() -> None:
                """A row for this frame from the coaster, when it has something to offer."""
                nonlocal last_valid_T, last_position
                if not cfg.coast:
                    return
                if coaster.gap_rows == 0:
                    coaster.gap_first_row = len(rows)
                T_est = coaster.coast(ts, last_valid_T)
                if T_est is None:
                    return
                q_est, t_est = _matrix_pose(T_est)
                last_valid_T = T_est
                counters.frames_coasted += 1
                rows.append((int(ts), float(t_est[0]), float(t_est[1]), float(t_est[2]),
                             float(q_est[0]), float(q_est[1]), float(q_est[2]), float(q_est[3])))
                last_position = [float(t_est[0]), float(t_est[1]), float(t_est[2])]
                if on_pose is not None:
                    on_pose(PoseEvent(index=index, frame_no=frame_no, ts_ns=int(ts), translation=t_est,
                                      rotation_xyzw=q_est, segment=len(segments) - 1, track_ms=0.0,
                                      images=images, coasted=True))

            world_from_rig = estimate.world_from_rig
            pose = None if world_from_rig is None else world_from_rig.pose
            t_raw: Optional[np.ndarray] = None
            q_raw: Optional[np.ndarray] = None
            usable = False
            if pose is not None:
                t_raw = np.array(pose.translation, dtype=np.float64).reshape(3)
                q_raw = np.array(pose.rotation, dtype=np.float64).reshape(4)
                q_norm = float(np.linalg.norm(q_raw))  # NaN/inf when any component is
                if bool(np.all(np.isfinite(t_raw))) and math.isfinite(q_norm) and q_norm > 0.0:
                    usable = True
                    q_raw = q_raw / q_norm  # unit quaternion for the row; the SE(3) helpers reject a zero norm

            if pose is None:
                # Only a None pose is a loss: the engine restarts its world frame after it
                # ("poses in the same coordinate system until a loss of tracking"), so only
                # here is `lost` set and a later valid pose recorded as a reset / re-anchored.
                counters.frames_failed += 1
                lost = True
                gap_engine_restarted = True
                prev_t_raw = prev_q_raw = prev_ts = None  # the next pose is in a new world frame
                warnings.emit("failed", "tracking failed at frame #{} (t={})".format(frame_no, ts))
                emit_coasted()
            elif not usable or t_raw is None or q_raw is None:
                # A NaN/inf or a zero-norm quaternion inside a returned pose is unusable for the
                # trajectory but not a loss: the next pose continues in the same world frame, so no
                # reset and no re-anchor.
                counters.frames_failed += 1
                counters.frames_nonfinite += 1
                warnings.emit("nonfinite", "frame #{} (t={}) returned a non-finite or zero-norm pose; row skipped, "
                              "no reset".format(frame_no, ts))
                emit_coasted()
            else:
                # ---- starvation guard (module docstring): is this usable pose worth a row? ----
                reason: Optional[str] = None
                detail = ""
                if cfg.guard:
                    n_obs = _observation_count(tracker)
                    if n_obs is not None:
                        obs_counts.append(n_obs)
                    if n_obs is not None and n_obs < cfg.min_observations:
                        reason = "starved"
                        detail = "{} observation(s) < {}".format(n_obs, cfg.min_observations)
                    elif (cfg.guard_zero_covariance and prev_ts is not None
                          and _zero_position_covariance(world_from_rig)):
                        # not on the first accepted pose of a segment: the engine reports an
                        # all-zero covariance at initialisation (measured on a still camera)
                        reason = "degenerate"
                        detail = "position covariance exactly zero"
                    elif prev_t_raw is not None and prev_q_raw is not None and prev_ts is not None:
                        dt_s = (ts - prev_ts) / _NS_PER_S
                        step_m = float(np.linalg.norm(t_raw - prev_t_raw))
                        rot_deg = _quat_angle_deg(q_raw, prev_q_raw)
                        if dt_s > 0.0 and step_m / dt_s > cfg.max_speed_mps:
                            reason = "jump"
                            detail = "step {:.3f} m in {:.1f} ms = {:.1f} m/s > {:g} m/s".format(
                                step_m, dt_s * 1e3, step_m / dt_s, cfg.max_speed_mps)
                        elif dt_s > 0.0 and rot_deg / dt_s > cfg.max_rot_dps:
                            reason = "spin"
                            detail = "rotation {:.1f} deg in {:.1f} ms = {:.0f} deg/s > {:g} deg/s".format(
                                rot_deg, dt_s * 1e3, rot_deg / dt_s, cfg.max_rot_dps)
                if reason is not None:
                    counters.frames_guarded += 1
                    setattr(counters, "guard_" + reason, getattr(counters, "guard_" + reason) + 1)
                    consecutive_guarded += 1
                    if gap_start is None:
                        gap_start = index
                    gap_frames += 1
                    warnings.emit("guard_" + reason, "guard skipped frame #{} (t={}): {} ({}); {} guarded in a "
                                  "row".format(frame_no, ts, reason, detail, consecutive_guarded))
                    if cfg.guard_reset_after > 0 and consecutive_guarded >= cfg.guard_reset_after:
                        # The engine keeps misbehaving: re-create it with the same rig and config.
                        # The old tracker is dropped first (the feeder holds the other reference),
                        # the IMU stream continues into the new one, no warm-up frames are skipped,
                        # and the next accepted pose is chained onto the last good pose whatever
                        # cfg.stitch_resets says (the guard exists to keep the trajectory continuous).
                        warnings.emit("guard_reset", "guard: {} consecutive guarded frame(s) up to frame #{} (t={}); "
                                      "re-creating the cuVSLAM tracker".format(consecutive_guarded, frame_no, ts))
                        if cfg.slam:
                            slam_optimized.extend(_slam_rows_from(_slam_snapshot(tracker), slam_anchors))
                            counters.slam_map_resets += 1
                        feeder.tracker = None
                        tracker = None
                        tracker = _new_tracker(vslam, rig, mode, cfg, odom_cfg)
                        feeder.tracker = tracker
                        counters.guard_resets += 1
                        if counters.frames_tracked > 0:  # as a normal reset: a loss before any pose is none
                            counters.resets += 1
                            segments.append(counters.frames_attempted)  # the next frame handed to track()
                        lost = False           # a pending loss-reset is subsumed by this reset
                        reanchor = True
                        gap_engine_restarted = True
                        consecutive_guarded = 0
                        prev_t_raw = prev_q_raw = prev_ts = None
                    # No measured row; the coaster may add an estimate (else last_valid_T keeps the last pose).
                    emit_coasted()
                    now = time.monotonic()
                    if cfg.stats_interval_s > 0 and stats_t0 is not None and now - stats_t0 >= cfg.stats_interval_s:
                        emit_stats(now)
                    if cfg.max_frames and counters.frames_attempted >= cfg.max_frames:
                        stopped_by = "max_frames"
                        break
                    continue

                if lost and counters.frames_tracked > 0:
                    counters.resets += 1
                    segments.append(index)
                    if cfg.stitch_resets:
                        anchor = last_valid_T
                    elif cfg.coast:
                        # coasting keeps the trajectory continuous: the returning pose is chained onto
                        # the coasted estimate instead of restarting the world frame at identity
                        reanchor = True
                elif counters.frames_tracked == 0:
                    segments.append(index)
                lost = False
                if gap_start is not None:
                    # First accepted pose after a guarded stretch: continue from the last output pose
                    # (the coasted estimate, or the last good pose with zero motion across the gap
                    # when coasting is off).  When a loss/stitch re-anchor applies too, this
                    # one wins (both are computed from last_valid_T; this one also absorbs a non-identity
                    # first pose).  A guard engine reset arms `reanchor` the same way.
                    guard_gaps.append((gap_start, gap_frames))
                    gap_start, gap_frames = None, 0
                    reanchor = True
                consecutive_guarded = 0
                try:
                    T_raw = _pose_matrix(q_raw, t_raw)
                    if reanchor and last_valid_T is not None:
                        # continue from where the coaster says we are *now* (else from the last output)
                        target = coaster.predict(ts, last_valid_T) if cfg.coast else None
                        target = target if target is not None else last_valid_T
                        if cfg.coast and cfg.coast_keep_engine_rotation and not gap_engine_restarted and anchor is not None:
                            # the engine tracked through the gap in one world frame: keep its rotation (the
                            # previous anchor's) and move only the position onto the estimate
                            R_keep = anchor[:3, :3]
                            anchor = _rig.se3(R_keep, target[:3, 3] - R_keep @ T_raw[:3, 3])
                        else:
                            anchor = target @ _rig.invert_se3(T_raw)
                    reanchor = False
                    gap_engine_restarted = False
                    if anchor is not None:
                        q, t = _matrix_pose(anchor @ T_raw)
                    else:
                        q, t = q_raw, t_raw
                    last_valid_T = _pose_matrix(q, t)
                except ValueError as exc:  # rig.CalibrationError: unreachable for a unit quaternion; contract only
                    raise TrackingError("frame #{} (t={}): cannot convert the pose to SE(3): {}".format(
                        frame_no, ts, exc)) from exc
                coaster.accepted(ts, last_valid_T)
                slam_t: Optional[np.ndarray] = None
                slam_q: Optional[np.ndarray] = None
                if cfg.slam:
                    slam_anchors[int(ts)] = np.eye(4) if anchor is None else anchor
                    if slam_pose is not None:
                        try:
                            st = np.array(slam_pose.translation, dtype=np.float64).reshape(3)
                            sq = np.array(slam_pose.rotation, dtype=np.float64).reshape(4)
                            sn = float(np.linalg.norm(sq))
                            if bool(np.all(np.isfinite(st))) and math.isfinite(sn) and sn > 0.0:
                                if anchor is not None:
                                    slam_q, slam_t = _matrix_pose(anchor @ _pose_matrix(sq / sn, st))
                                else:
                                    slam_q, slam_t = sq / sn, st
                                slam_rows.append((int(ts), float(slam_t[0]), float(slam_t[1]), float(slam_t[2]),
                                                  float(slam_q[0]), float(slam_q[1]), float(slam_q[2]), float(slam_q[3])))
                        except (AttributeError, TypeError, ValueError):
                            slam_t = slam_q = None
                    try:
                        metrics = tracker.get_slam_metrics()
                    except Exception:  # noqa: BLE001 - diagnostics only
                        metrics = None
                    if metrics is not None:
                        lc, pgo = bool(getattr(metrics, "lc_status", False)), bool(getattr(metrics, "pgo_status", False))
                        if lc and not slam_lc_prev:
                            counters.loop_closures += 1
                            warnings.emit("loop_closure", "loop closure at frame #{} (t={})".format(frame_no, ts))
                        if pgo and not slam_pgo_prev:
                            counters.pgo_events += 1
                        slam_lc_prev, slam_pgo_prev = lc, pgo
                prev_t_raw, prev_q_raw, prev_ts = t_raw, q_raw, ts
                counters.frames_tracked += 1
                rows.append((int(ts), float(t[0]), float(t[1]), float(t[2]),
                             float(q[0]), float(q[1]), float(q[2]), float(q[3])))
                last_position = [float(t[0]), float(t[1]), float(t[2])]
                gravity: Optional[List[float]] = None
                if vio:
                    try:
                        raw_gravity = tracker.get_last_gravity()
                    except ValueError:
                        raw_gravity = None
                    if raw_gravity is not None:
                        gravity = [float(v) for v in raw_gravity]
                        last_gravity = gravity
                if on_pose is not None:
                    on_pose(PoseEvent(index=index, frame_no=frame_no, ts_ns=int(ts), translation=t,
                                      rotation_xyzw=q, segment=len(segments) - 1, track_ms=dt * 1e3,
                                      pose=pose, covariance=list(world_from_rig.covariance_xyz_rpy),
                                      gravity=gravity, images=images, tracker=tracker,
                                      slam_translation=slam_t, slam_rotation_xyzw=slam_q))

            now = time.monotonic()
            if cfg.stats_interval_s > 0 and stats_t0 is not None and now - stats_t0 >= cfg.stats_interval_s:
                emit_stats(now)
            if cfg.max_frames and counters.frames_attempted >= cfg.max_frames:
                stopped_by = "max_frames"
                break
    except KeyboardInterrupt:
        stopped_by = "keyboard_interrupt"

    end = time.monotonic()
    if wall_start is not None and (on_stats is not None or cfg.verbose):
        emit_stats(end, final=True)
    wall_s = (end - wall_start) if wall_start is not None else 0.0
    wall_tracking_s = (end - track_wall_start) if track_wall_start is not None else 0.0

    metrics = trajectory_metrics(rows)
    slam_summary: Optional[Dict[str, Any]] = None
    if cfg.slam:
        slam_optimized.extend(_slam_rows_from(_slam_snapshot(tracker), slam_anchors))
        graph_nodes = graph_edges = None
        try:
            graph = tracker.get_pose_graph()
            if graph is not None:
                graph_nodes, graph_edges = len(list(graph.nodes)), len(list(graph.edges))
        except Exception:  # noqa: BLE001
            pass
        map_saved: Optional[bool] = None
        if cfg.slam_save_map:
            map_saved = _save_map(tracker, cfg.slam_save_map)
        live = trajectory_metrics(slam_rows)
        optimized = trajectory_metrics(slam_optimized)
        slam_summary = {
            "enabled": True, "sync": (not cfg.realtime) if cfg.slam_sync is None else bool(cfg.slam_sync),
            "max_map_size": cfg.slam_max_map_size, "rows": len(slam_rows), "optimized_rows": len(slam_optimized),
            "loop_closures": counters.loop_closures, "pgo_events": counters.pgo_events,
            "map_resets": counters.slam_map_resets, "pose_graph_nodes": graph_nodes, "pose_graph_edges": graph_edges,
            "path_length_m": live.get("path_length_m"), "endpoint_offset_m": live.get("endpoint_offset_m"),
            "optimized_path_length_m": optimized.get("path_length_m"),
            "optimized_endpoint_offset_m": optimized.get("endpoint_offset_m"),
            "map_saved": map_saved, "map_path": cfg.slam_save_map or None,
        }
    final_landmarks: Optional[int] = None
    if cfg.final_landmarks:
        try:
            final_landmarks = len(tracker.get_final_landmarks())
        except Exception:  # noqa: BLE001 - never let a diagnostic break the summary
            final_landmarks = None
    try:
        version = str(vslam.get_version()[0])
    except Exception:  # noqa: BLE001
        version = "?"

    summary: Dict[str, Any] = {
        "mode": mode,
        "odometry_mode": ODOMETRY_MODE_NAME[mode],
        "realtime": bool(cfg.realtime),
        "async_sba": bool(getattr(odom_cfg, "async_sba", cfg.realtime)),
        "cuvslam_version": version,
        "fake": bool(_is_fake(vslam)),
    }
    for name in _Counters.__dataclass_fields__:  # type: ignore[attr-defined]
        summary[name] = getattr(counters, name)
    summary["max_frame_gap_ms"] = counters.max_frame_gap_ns / 1e6
    summary["max_imu_gap_ms"] = counters.max_imu_gap_ns / 1e6
    del summary["max_frame_gap_ns"], summary["max_imu_gap_ns"]
    summary.update({
        "imu_held_back": feeder.held_back,
        "imu_source_ignored": imu_source_ignored,
        "segments": list(segments),
        "first_ts_ns": first_tracked_ts,
        "last_ts_ns": last_attempted_ts,
        "duration_s": ((last_attempted_ts - first_tracked_ts) / _NS_PER_S
                       if first_tracked_ts is not None and last_attempted_ts is not None else 0.0),
        "wall_s": wall_s,
        "wall_tracking_s": wall_tracking_s,
        "mean_fps": counters.frames_attempted / wall_tracking_s if wall_tracking_s > 0 else 0.0,
        "engine_fps": counters.frames_attempted / track_time_s if track_time_s > 0 else 0.0,
        "track_ms_mean": 1e3 * track_time_s / counters.frames_attempted if counters.frames_attempted else 0.0,
        "track_ms_max": track_ms_max,
        "gravity": last_gravity,
        "final_landmarks": final_landmarks,
        "guard_gaps": list(guard_gaps),
        "coast_gaps": coaster.finish(),
        "coast_gyro_used": R_rig_imu is not None,
        "observations_min": min(obs_counts) if obs_counts else None,
        "observations_median": float(statistics.median(obs_counts)) if obs_counts else None,
        "observations_max": max(obs_counts) if obs_counts else None,
        "stopped_by": stopped_by,
        "config": cfg.to_dict(),
        "tum_rows": len(rows),
        "rows": rows,
        "slam": slam_summary,
        "slam_rows": slam_rows if cfg.slam else [],
        "slam_optimized_rows": slam_optimized if cfg.slam else [],
    })
    summary.update(metrics)
    return summary


def _save_map(tracker: Any, folder: str, timeout_s: float = 120.0) -> bool:
    """``tracker.save_map(folder, cb)`` and wait for the callback (asynchronous unless ``sync_mode``)."""
    import threading

    done = threading.Event()
    result: List[bool] = []

    def callback(ok: Any) -> None:
        result.append(bool(ok))
        done.set()

    try:
        tracker.save_map(str(folder), callback)
    except Exception:  # noqa: BLE001 - reported as not saved
        return False
    done.wait(timeout_s)
    return bool(result and result[0])


# --------------------------------------------------------------------------------------
# Trajectory metrics and TUM I/O
# --------------------------------------------------------------------------------------

def _flatten_row(row: Any) -> Tuple[int, List[float]]:
    if isinstance(row, PoseEvent):
        row = row.tum_row
    if isinstance(row, Mapping):
        ts = row.get("ts_ns", row.get("timestamp_ns"))
        t = row.get("translation")
        q = row.get("rotation_xyzw", row.get("rotation"))
        if ts is None or t is None or q is None:
            raise ValueError("a TUM row dict needs ts_ns, translation and rotation_xyzw")
        row = (ts, t, q)
    values = list(row)
    if len(values) == 3 and not np.isscalar(values[1]):
        ts, t, q = values
        flat = [float(v) for v in np.asarray(t, dtype=np.float64).reshape(-1)] + \
               [float(v) for v in np.asarray(q, dtype=np.float64).reshape(-1)]
    elif len(values) == 8:
        ts, flat = values[0], [float(v) for v in values[1:]]
    else:
        raise ValueError("a TUM row must be (ts_ns, tx, ty, tz, qx, qy, qz, qw) or (ts_ns, translation, rotation), "
                         "got {} elements".format(len(values)))
    if len(flat) != 7:
        raise ValueError("a TUM row needs 3 translation + 4 quaternion values, got {}".format(len(flat)))
    return int(ts), flat


def trajectory_metrics(rows: Sequence[Any]) -> Dict[str, Any]:
    """Path length, end-point offset from the first pose (drift proxy), farthest excursion.

    Keys: ``path_length_m`` (sum of consecutive translation steps), ``endpoint_offset_m``
    (``|t_last - t_first|``; for a loop closed on foot this is the error), ``max_from_start_m``,
    ``start_position``, ``end_position`` (lists or ``None``), ``poses``.  With
    ``stitch_resets=False`` a reset jumps the trajectory back to the origin and both distances
    then include that jump; compare ``resets`` before trusting them.
    """
    n = len(rows)
    if n == 0:
        return {"path_length_m": 0.0, "endpoint_offset_m": 0.0, "max_from_start_m": 0.0,
                "start_position": None, "end_position": None, "poses": 0}
    P = np.array([_flatten_row(r)[1][:3] for r in rows], dtype=np.float64)
    steps = np.linalg.norm(np.diff(P, axis=0), axis=1) if n > 1 else np.zeros(0)
    from_start = np.linalg.norm(P - P[0], axis=1)
    return {
        "path_length_m": float(steps.sum()),
        "endpoint_offset_m": float(from_start[-1]),
        "max_from_start_m": float(from_start.max()),
        "start_position": [float(v) for v in P[0]],
        "end_position": [float(v) for v in P[-1]],
        "poses": n,
    }


def format_tum_timestamp(ns: int) -> str:
    """``ns`` -> ``'<seconds>.<9 digits>'`` exactly (``f"{ns // 10**9}.{ns % 10**9:09d}"``)."""
    ns = int(ns)
    if ns < 0:
        raise ValueError("TUM timestamps must be non-negative nanoseconds, got {}".format(ns))
    return "{}.{:09d}".format(ns // _NS_PER_S, ns % _NS_PER_S)


def parse_tum_timestamp(text: str) -> int:
    """Inverse of :func:`format_tum_timestamp`: decimal seconds -> integer ns (truncating below 1 ns)."""
    token = str(text).strip()
    if not token or token[0] in "+-" or "e" in token.lower():
        raise ValueError("not a plain decimal TUM timestamp: {!r}".format(text))
    whole, _, frac = token.partition(".")
    if not whole.isdigit() or (frac and not frac.isdigit()):
        raise ValueError("not a plain decimal TUM timestamp: {!r}".format(text))
    frac = (frac + "000000000")[:9]
    return int(whole) * _NS_PER_S + int(frac)


def write_tum(path: Union[str, Path], rows: Iterable[Any], header: Optional[Union[str, Sequence[str]]] = None,
              precision: int = 6) -> Path:
    """Write *rows* as a TUM trajectory (``datasets/dataset-format.md`` section 11) and return the path.

    Each row is ``(ts_ns, tx, ty, tz, qx, qy, qz, qw)`` (``world_from_rig``, metres, quaternion in
    cuVSLAM's x y z w order = TUM order), a ``(ts_ns, translation, rotation_xyzw)`` triple, a
    :class:`PoseEvent` or a dict with those keys.  The first line is :data:`TUM_HEADER`;
    *header* adds comment lines (a string with newlines or a sequence, ``#`` prepended as
    needed).  Values use *precision* decimals; timestamps are exact (``ns // 1e9 . ns % 1e9``).
    Parent directories are created; the file is pure ASCII with ``\\n`` line ends so ``evo``
    reads it under any locale, on Linux or Windows (a non-ASCII character in *header* is written
    as ``?`` instead of raising; the rows themselves are always ASCII).
    """
    out = Path(path)
    lines = [TUM_HEADER]
    if header:
        extra = header.splitlines() if isinstance(header, str) else [str(h) for h in header]
        for line in extra:
            for sub in str(line).splitlines() or [""]:
                sub = sub.strip()
                lines.append(sub if sub.startswith("#") else "# " + sub)
    fmt = "{:.%df}" % max(0, int(precision))
    for row in rows:
        ts, flat = _flatten_row(row)
        lines.append(format_tum_timestamp(ts) + " " + " ".join(fmt.format(v) for v in flat))
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="ascii", errors="replace", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    return out


def read_tum(path: Union[str, Path]) -> List[Tuple[int, float, float, float, float, float, float, float]]:
    """Read a TUM trajectory back as ``(ts_ns, tx, ty, tz, qx, qy, qz, qw)`` tuples (comments and blank lines skipped)."""
    rows: List[Tuple[int, float, float, float, float, float, float, float]] = []
    with Path(path).open("r", encoding="ascii", errors="replace") as fh:
        for lineno, line in enumerate(fh, start=1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.replace(",", " ").split()
            if len(parts) != 8:
                raise ValueError("{}:{}: expected 8 columns, got {}".format(path, lineno, len(parts)))
            try:
                ts = parse_tum_timestamp(parts[0])
                vals = [float(v) for v in parts[1:]]
            except ValueError as exc:
                raise ValueError("{}:{}: {}".format(path, lineno, exc)) from None
            rows.append((ts, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]))
    return rows
