#!/usr/bin/env python3
"""08_cuvslam_track.py - run NVIDIA cuVSLAM (PyCuVSLAM 17.0.0) on an OAK-D S2 recording or live.

What this script does
---------------------
It drives ``cuvslam_oak.tracking.run_tracking`` over one of two sources and writes the
trajectory in TUM format (``timestamp_s tx ty tz qx qy qz qw``, what ``evo`` reads):

* ``--dataset PATH`` (or a positional PATH): an EuRoC ASL recording written by
  ``datasets/07_record_dataset.py`` / ``cuvslam_oak.dataset.EurocWriter`` (the folder that holds
  ``mav0/``, or ``mav0`` itself).  Offline, deterministic (``async_sba=False``), no camera and no
  ``depthai`` needed; the rig is built from the dataset's own ``sensor.yaml`` files exactly as
  NVIDIA's ``examples/euroc/dataset_utils.py`` builds it (cam0 = rig, ``inv(T_BS0) @ T_BS1``,
  ``inv(T_BS0) @ T_BS_imu``), so the poses are the left camera (CAM_B) in the frame of the
  first tracked image.
* ``--live``: the connected OAK-D S2 through ``cuvslam_oak.oak_source.OakStereoImuSource``
  (synced GRAY8 pair + BNO086 raw IMU on one device clock).  Real time (``async_sba=True``),
  the first ``--warmup-frames`` (60, as NVIDIA's OAK-D example) are skipped for auto-exposure.
  The rig comes from the camera's EEPROM via ``cuvslam_oak.rig.build_rig`` with raw frames
  (``Distortion.Model.Polynomial``, 8 coefficients, the example's border masks) and
  ``--rig-socket`` (default CAM_B, so live and dataset trajectories share the cam0 frame).
  Everything that needs no camera (``--calibration`` file, ``--mode`` check, rig) is done before
  the device is opened, so a bad file or mode never boots it.

``--mode stereo|vio`` picks ``OdometryMode.Multicamera`` (cameras only) or ``Inertial``
(cameras + IMU); the default is ``vio`` whenever IMU data is available (dataset: IMU rows
present and the IMU extrinsics are not the identity placeholder of a stereo-only recording;
live: the EEPROM carries ``imuExtrinsics``), else ``stereo``.  ``--calibration PATH`` replaces
the calibration (a ``calibration.json`` = ``CalibrationHandler.eepromToJson()`` dump, or a
holistic ``camera_info.json``) for both sources: the rig is then built with
``cuvslam_oak.rig.build_rig`` at the source resolution (Pinhole when the dataset frames are
host-undistorted, Polynomial otherwise).

The loop's *starvation guard* is on by default (``--no-guard`` switches it off).  On a blank wall
from close range cuVSLAM still returns a pose, but a garbage one (measured on hand-held walks:
steps of 0.4-52 m per frame, rotations up to 63 deg, and in ``vio`` mode a run-away of hundreds
of metres) without ever reporting a failure.  A returned pose is skipped, and the trajectory
continues from the last good pose (zero motion across the gap), when it moved faster than
``--max-speed`` (3 m/s) or rotated faster than ``--max-rot-rate`` (720 deg/s) since the last
accepted pose, or when its position covariance is exactly zero (``--no-zero-covariance-guard``
disables that rule; there is no covariance *magnitude* rule because the engine reports a
placeholder variance of 1.0 on ~30 % of good frames of a still camera).  ``--min-observations N``
adds an observation-count rule; it defaults to 0 (off) because on this camera the count did not
drop on the glitch frames (it sat at the engine's cap of 450).  After ``--guard-reset-after``
(30) consecutive guarded frames the tracker is re-created and the trajectory chained onto the
last good pose.  The defaults suit a hand-held walk or an indoor flight; a racing drone needs a
higher ``--max-speed``.  The summary line ``guard:`` and the JSON (``frames_guarded``,
``guard_*``, ``guard_gaps``, ``observations_min/median/max``) report what was skipped.

cuVSLAM is loaded through ``cuvslam_oak.import_cuvslam``: the real wheel on WSL2 / Jetson, or
the built-in test double ``cuvslam_oak.fake_cuvslam`` when ``--fake-cuvslam`` is given (or
``CUVSLAM_FAKE=1`` is set and the real module is missing).  The fake is announced LOUDLY on
stderr, in the TUM header and in the summary: its poses are a synthetic constant-velocity
trajectory, not tracking.  It exists so this whole pipeline (CLI, dataset reader, rig, loop,
TUM writer) runs on the Windows laptop, where no cuVSLAM wheel exists.

How to run (from the repository root)
-------------------------------------
    python methods/cuvslam-vio/08_cuvslam_track.py recordings/cuvslam_20260919_203107              # auto mode
    python methods/cuvslam-vio/08_cuvslam_track.py --dataset ~/datasets/cuvslam_x --mode stereo --out traj.tum
    python methods/cuvslam-vio/08_cuvslam_track.py --dataset D --mode vio --summary --rerun          # + viewer
    python methods/cuvslam-vio/08_cuvslam_track.py --live --duration 60 --show                      # Jetson
    python methods/cuvslam-vio/08_cuvslam_track.py --dataset D --fake-cuvslam --max-frames 100      # dry run
    python methods/cuvslam-vio/08_cuvslam_track.py --dataset D --max-speed 30 --guard-reset-after 0  # racing drone

Outputs
-------
* TUM trajectory: ``--out PATH`` (a directory puts ``trajectory_<mode>.tum`` inside it).
  Default: ``<dataset root>/trajectory_<mode>.tum`` for a dataset,
  ``recordings/trajectory_<mode>_<YYYYmmdd_HHMMSS>.tum`` for a live run.  One row per
  successfully tracked frame, failed frames are skipped (counted in the summary).  Plot with
  ``evo_traj tum trajectory_vio.tum --plot --plot_mode xz``.  Timestamps: a dataset run keeps
  the dataset's own (Unix-epoch ns for our recordings); a live run adds the source's
  ``epoch_offset_ns`` to the device clock exactly as ``07_record_dataset.py`` does
  (datasets/dataset-format.md section 8), so live and recorded trajectories share the
  epoch domain for ``evo`` association - raw device-clock ns only when the offset never
  became known.  The header line ``time:`` says which.
* ``--summary``: a JSON file with the run summary (counters, timing, path length, end-point
  offset, rig, source facts, arguments) next to the TUM file as ``<stem>_summary.json``;
  ``--summary-path PATH`` puts it elsewhere (and implies ``--summary``).
* stdout: the source/rig/mode facts, one statistics line per second (frames seen/tracked/
  failed, resets, fps, engine time, position, IMU count) and the final summary.  Tracking
  warnings (frame gaps, failures, IMU drops; rate-limited) go to stderr, as do the dataset
  validation warnings once the source is accepted (one summarising line, or each one with
  ``--verbose``; all of them are in the summary JSON).
* ``--rerun``: NVIDIA-style live visualisation (trajectory, rig axes, left image with the
  tracked observations, gravity) in a rerun viewer.  ``rerun-sdk`` is imported lazily; when it
  is missing an install hint is printed and the run continues without it.
* ``--show``: an OpenCV window with the left image, the observations and a status line;
  ``q`` / ``Esc`` stop the run cleanly (the trajectory is still written).  Ctrl-C does the same.

Exit codes
----------
0 finished (end of data, ``--duration``, ``--max-frames``, ``q``, Ctrl-C during tracking);
1 tracking / calibration / stream error (one stderr line);
2 no device (or no depthai for ``--live``), dataset missing or unusable, ``--calibration`` file
missing or unreadable, bad source selection or argument (one stderr line, before any ``Source:`` /
``Mode:`` output; the details of a dataset that fails validation go to stdout);
3 cuvslam not importable and no ``--fake-cuvslam`` (one stderr line naming
methods/cuvslam-vio/docs/cuvslam-setup.md).

Conventions followed (methods/cuvslam-vio/docs/cuvslam-api-v17.md, methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md,
datasets/dataset-format.md): ``Tracker(rig, OdometryConfig)`` of v17.0.0,
``register_imu_measurement(0, m)`` for every IMU sample with ``t <= image t`` before
``track(t, [left, right])``, integer-nanosecond timestamps on one clock, metres, quaternions
``x y z w``, OpenCV axes (x right, y down, z forward), no axis flips.  Runs on Windows (fake),
WSL2 and the Jetson Orin Nano (Python 3.10, pathlib, no Windows-only calls).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import platform
import sys
import time
import warnings
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPT_DIR.parents[1]
sys.path.insert(0, str(REPO_ROOT / "camera"))   # camera/common.py
sys.path.insert(0, str(REPO_ROOT))     # cuvslam_oak package

import cuvslam_oak  # noqa: E402
from cuvslam_oak import imu_calib  # noqa: E402
from cuvslam_oak import rig as oakrig  # noqa: E402
from cuvslam_oak import tracking as oaktrack  # noqa: E402

try:  # common.py imports depthai; a tracking-only host (WSL2 with WITH_DEPTHAI=0) may lack it
    import common  # noqa: E402
except ImportError as _exc:  # pragma: no cover - depends on the host
    common = None  # type: ignore[assignment]
    _COMMON_IMPORT_ERROR: Optional[BaseException] = _exc
else:
    _COMMON_IMPORT_ERROR = None

try:  # importable without depthai (constants and pure helpers); the live source itself needs the wheel
    from cuvslam_oak import oak_source  # noqa: E402
except ImportError as _exc:  # pragma: no cover - depends on the host
    oak_source = None  # type: ignore[assignment]
    _OAK_SOURCE_IMPORT_ERROR: Optional[BaseException] = _exc
else:
    _OAK_SOURCE_IMPORT_ERROR = None

#: Exit statuses (see the module docstring).
EXIT_OK = 0
EXIT_ERROR = 1
EXIT_NO_INPUT = 2
EXIT_NO_CUVSLAM = 3

DEFAULT_RECORDINGS_DIR = REPO_ROOT / "recordings"
DEFAULT_LIVE_SIZE = (1280, 800)
DEFAULT_LIVE_FPS = 30.0
DEFAULT_IMU_RATE_HZ = 200
#: Highest ``--imu-rate`` accepted (BNO08X gyroscope jitter above it, see cuvslam_oak.oak_source).
MAX_IMU_RATE_HZ = int(getattr(oak_source, "MAX_IMU_RATE_HZ", 400))
DEFAULT_LIVE_WARMUP_FRAMES = oaktrack.DEFAULT_WARMUP_FRAMES
DEFAULT_RIG_SOCKET = oakrig.CAM_B
STATS_INTERVAL_S = 1.0
RERUN_INSTALL_HINT = ("rerun-sdk is not installed; continuing without the viewer. Install it with "
                      "'pip install rerun-sdk==0.33.1' (methods/cuvslam-vio/tools/setup_wsl2.sh: WITH_RERUN=1; "
                      "methods/cuvslam-vio/docs/cuvslam-setup.md section 2.6)")

_NS_PER_S = 1_000_000_000


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def one_line(text: Any) -> str:
    """Collapse whitespace so a message fits on one stderr line."""
    return " ".join(str(text).split())


def fail(message: str, code: int = EXIT_NO_INPUT) -> int:
    """Print ONE line to stderr and return *code*."""
    print(one_line(message), file=sys.stderr, flush=True)
    return code


def note(message: str) -> None:
    """Informational line on stdout."""
    print(message, flush=True)


def warn(message: str) -> None:
    """Warning line on stderr."""
    print("warning: " + one_line(message), file=sys.stderr, flush=True)


def _safe(fn: Callable[[], Any], default: Any = None) -> Any:
    try:
        return fn()
    except Exception:  # noqa: BLE001 - diagnostics must never abort the run
        return default


def parse_size(text: str) -> Tuple[int, int]:
    """``'640x400'`` -> ``(640, 400)`` for argparse."""
    try:
        w_str, h_str = text.lower().replace(",", "x").split("x")
        width, height = int(w_str), int(h_str)
    except ValueError:
        raise argparse.ArgumentTypeError("expected WxH, e.g. 640x400, got {!r}".format(text)) from None
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("size must be positive, got {!r}".format(text))
    return width, height


def parse_mode(text: str) -> str:
    """``--mode`` value -> ``'stereo'`` / ``'vio'`` (aliases of :data:`cuvslam_oak.tracking.MODE_ALIASES`)."""
    try:
        return oaktrack.normalize_mode(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from None


def timestamp_str() -> str:
    """Local-time stamp ``YYYYmmdd_HHMMSS`` (same as ``common.timestamp_str``)."""
    return time.strftime("%Y%m%d_%H%M%S")


def iso_now() -> str:
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


def json_safe(value: Any) -> Any:
    """Make *value* JSON-serialisable: numpy -> Python, Path -> str, NaN/inf -> None."""
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        f = float(value)
        return f if math.isfinite(f) else None
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, str):
        return value
    return str(value)


class SizedStream:
    """Iterable wrapper that also answers ``len()`` so the tracking loop can report ``N/total``."""

    def __init__(self, iterable: Iterable[Any], total: int) -> None:
        self._iterable = iterable
        self._total = int(total)

    def __iter__(self) -> Iterator[Any]:
        return iter(self._iterable)

    def __len__(self) -> int:
        return self._total


# --------------------------------------------------------------------------------------
# cuvslam loading
# --------------------------------------------------------------------------------------

def load_cuvslam(force_fake: bool) -> Tuple[Any, str]:
    """Return ``(module, chosen_by)``: the cuvslam module and, when it is the fake, what selected it.

    Raises ``ImportError`` when the real module is missing and the fake was not allowed.
    """
    if force_fake:
        return cuvslam_oak.import_cuvslam(prefer_fake=True), "--fake-cuvslam"
    module = cuvslam_oak.import_cuvslam()
    if cuvslam_oak.is_fake(module):
        return module, "{}=1 (the real cuvslam is not importable)".format(cuvslam_oak.FAKE_ENV_VAR)
    return module, ""


def cuvslam_version(vslam: Any) -> str:
    return str(_safe(lambda: vslam.get_version()[0], "?"))


def announce_fake(vslam: Any, chosen_by: str) -> None:
    """Loud stderr banner: the poses of this run are synthetic."""
    bar = "!" * 100
    lines = [
        bar,
        "!!! FAKE cuVSLAM in use: {} ({}) selected by {}.".format(
            getattr(vslam, "__name__", "cuvslam_oak.fake_cuvslam"), cuvslam_version(vslam), chosen_by),
        "!!! Poses are SYNTHETIC (constant-velocity model, no vision) - NOT tracking results.",
        "!!! Dry run only: it exercises the CLI, the dataset reader / live source, the rig, the loop and the "
        "TUM writer.",
        bar,
    ]
    print("\n".join(lines), file=sys.stderr, flush=True)


# --------------------------------------------------------------------------------------
# Rig helpers
# --------------------------------------------------------------------------------------

def rig_from_dataset(vslam: Any, info: Any, mode: str) -> Any:
    """``vslam.Rig`` from a dataset's ``RigInfo`` (the three ``sensor.yaml`` files).

    Built exactly as NVIDIA's ``dataset_utils.get_rig``: cam0 = rig (identity),
    ``cam1.rig_from_camera = inv(T_BS0) @ T_BS1``, ``imu.rig_from_imu = inv(T_BS0) @ T_BS_imu``,
    intrinsics/resolution as written, the distortion model from ``distortion_model`` +
    coefficient count (``CameraInfo.cuvslam_distortion``: zero coefficients -> Pinhole).
    Borders come from ``dataset.json`` for raw (not undistorted) frames.  The IMU is included
    only in ``'vio'`` mode.
    """
    cameras = []
    for index, cam in enumerate(info.cameras):
        model_name, params = cam.cuvslam_distortion()
        T = np.eye(4) if index == 0 else info.rig_from_cam1()
        pose = oakrig.pose_from_matrix(T, vslam)
        borders = tuple(cam.borders) if not cam.undistorted else (0, 0, 0, 0)
        model = getattr(vslam.Distortion.Model, model_name)
        cameras.append(vslam.Camera(
            size=(int(cam.width), int(cam.height)),
            principal=(float(cam.cu), float(cam.cv)),
            focal=(float(cam.fu), float(cam.fv)),
            rig_from_camera=pose,
            distortion=vslam.Distortion(model, [float(p) for p in params]),
            border_top=int(borders[0]), border_bottom=int(borders[1]),
            border_left=int(borders[2]), border_right=int(borders[3]),
        ))
    imus = []
    if mode == "vio":
        imu = info.imu
        imus.append(vslam.ImuCalibration(
            rig_from_imu=oakrig.pose_from_matrix(info.rig_from_imu(), vslam),
            gyroscope_noise_density=float(imu.gyroscope_noise_density),
            gyroscope_random_walk=float(imu.gyroscope_random_walk),
            accelerometer_noise_density=float(imu.accelerometer_noise_density),
            accelerometer_random_walk=float(imu.accelerometer_random_walk),
            frequency=float(imu.rate_hz),
        ))
    return vslam.Rig(cameras=cameras, imus=imus)


def describe_rig(rig_obj: Any) -> Dict[str, Any]:
    """JSON-serialisable description of a ``vslam.Rig`` (works for the real binding and the fake)."""
    cams = []
    for cam in list(rig_obj.cameras):
        dist = cam.distortion
        model = getattr(getattr(dist, "model", None), "name", str(getattr(dist, "model", "?")))
        cams.append({
            "size": [int(cam.size[0]), int(cam.size[1])],
            "focal": [float(cam.focal[0]), float(cam.focal[1])],
            "principal": [float(cam.principal[0]), float(cam.principal[1])],
            "distortion_model": model,
            "distortion": [float(p) for p in list(dist.parameters)],
            "rig_from_camera_translation_m": [float(v) for v in cam.rig_from_camera.translation],
            "rig_from_camera_quaternion_xyzw": [float(v) for v in cam.rig_from_camera.rotation],
            "borders": [int(cam.border_top), int(cam.border_bottom), int(cam.border_left), int(cam.border_right)],
        })
    imus = []
    for imu in list(rig_obj.imus):
        imus.append({
            "rig_from_imu_translation_m": [float(v) for v in imu.rig_from_imu.translation],
            "rig_from_imu_quaternion_xyzw": [float(v) for v in imu.rig_from_imu.rotation],
            "frequency_hz": float(imu.frequency),
            "gyroscope_noise_density": float(imu.gyroscope_noise_density),
            "gyroscope_random_walk": float(imu.gyroscope_random_walk),
            "accelerometer_noise_density": float(imu.accelerometer_noise_density),
            "accelerometer_random_walk": float(imu.accelerometer_random_walk),
        })
    baseline = None
    if len(cams) >= 2:
        t0 = np.array(cams[0]["rig_from_camera_translation_m"])
        t1 = np.array(cams[1]["rig_from_camera_translation_m"])
        baseline = float(np.linalg.norm(t1 - t0))
    return {"cameras": cams, "imus": imus, "baseline_m": baseline}


def print_rig(desc: Dict[str, Any], verbose: bool) -> None:
    cams = desc["cameras"]
    for index, cam in enumerate(cams):
        line = "  cam{}: {}x{}  f=({:.2f}, {:.2f}) c=({:.2f}, {:.2f})  {}[{}]  t=({:+.4f}, {:+.4f}, {:+.4f}) m".format(
            index, cam["size"][0], cam["size"][1], cam["focal"][0], cam["focal"][1],
            cam["principal"][0], cam["principal"][1], cam["distortion_model"], len(cam["distortion"]),
            *cam["rig_from_camera_translation_m"])
        if verbose:
            line += "  q_xyzw=({:+.5f}, {:+.5f}, {:+.5f}, {:+.5f})  borders t/b/l/r={}".format(
                *cam["rig_from_camera_quaternion_xyzw"], "/".join(str(b) for b in cam["borders"]))
            if cam["distortion"]:
                line += "  coeffs=[{}]".format(", ".join("{:.6g}".format(p) for p in cam["distortion"]))
        note(line)
    if desc["baseline_m"] is not None:
        note("  baseline: {:.4f} m".format(desc["baseline_m"]))
    if not desc["imus"]:
        note("  imu: none in the rig (stereo-only tracking)")
    for imu in desc["imus"]:
        line = "  imu: rig_from_imu t=({:+.4f}, {:+.4f}, {:+.4f}) m  {:g} Hz".format(
            *imu["rig_from_imu_translation_m"], imu["frequency_hz"])
        if verbose:
            line += "  q_xyzw=({:+.5f}, {:+.5f}, {:+.5f}, {:+.5f})  noise gyro {:.3g}/{:.3g} accel {:.3g}/{:.3g}".format(
                *imu["rig_from_imu_quaternion_xyzw"], imu["gyroscope_noise_density"], imu["gyroscope_random_walk"],
                imu["accelerometer_noise_density"], imu["accelerometer_random_walk"])
        note(line)


class ModeError(ValueError):
    """``--mode`` cannot be honoured by the source (a selection error: exit 2, not a calibration error)."""


def resolve_mode(requested: Optional[str], imu_available: bool, why_no_imu: str) -> Tuple[str, str]:
    """``(mode, reason)``: the requested mode checked against the IMU availability, or the automatic choice.

    Raises :class:`ModeError` (a ``ValueError``) when ``'vio'`` was requested without IMU data.
    """
    if requested is None:
        if imu_available:
            return "vio", "auto: IMU data available"
        return "stereo", "auto: {}".format(why_no_imu or "no IMU data")
    mode = oaktrack.normalize_mode(requested)
    if mode == "vio" and not imu_available:
        raise ModeError("--mode vio needs IMU data but {}; use --mode stereo".format(why_no_imu or "there is none"))
    return mode, "requested"


# --------------------------------------------------------------------------------------
# Visualisation (rerun viewer and/or an OpenCV window), both optional
# --------------------------------------------------------------------------------------

def color_from_id(track_id: int) -> Tuple[int, int, int]:
    """Deterministic bright colour per observation id (R, G, B)."""
    i = int(track_id)
    return (60 + (i * 97) % 196, 60 + (i * 61) % 196, 60 + (i * 41) % 196)


class Visualizer:
    """``on_pose`` sink that feeds a rerun viewer and/or an OpenCV window; ``stop_requested`` after ``q``.

    Every viewer failure is reported once and disables that viewer; tracking never depends on it.
    """

    WINDOW = "cuvslam: cam0 (q quits)"

    def __init__(self, *, use_rerun: bool, show: bool, mode: str, fake: bool, title: str) -> None:
        self.stop_requested = False
        self.mode = mode
        self.fake = fake
        self.title = title
        self.positions: List[List[float]] = []
        self.rr: Any = None
        self.cv2: Any = None
        self._rr_error_reported = False
        self._show_error_reported = False
        if use_rerun:
            self._init_rerun()
        if show:
            self._init_window()

    # -- setup --------------------------------------------------------------------------
    def _init_rerun(self) -> None:
        try:
            import rerun as rr  # type: ignore[import-not-found]
        except ImportError:
            print(RERUN_INSTALL_HINT, file=sys.stderr, flush=True)
            return
        try:
            rr.init("cuvslam_oak 08_cuvslam_track", spawn=True)
            # cuVSLAM: right-handed, x right, y down, z forward (methods/cuvslam-vio/docs/cuvslam-api-v17.md section 6)
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
        except Exception as exc:  # noqa: BLE001 - a viewer problem must not stop tracking
            warn("rerun viewer could not start ({}: {}); continuing without it".format(type(exc).__name__, exc))
            return
        self.rr = rr
        note("rerun viewer: spawned (version {})".format(getattr(rr, "__version__", "?")))

    def _init_window(self) -> None:
        try:
            import cv2
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
        except Exception as exc:  # noqa: BLE001 - no display, no highgui build, ...
            warn("cannot open an OpenCV window ({}: {}); continuing without --show".format(type(exc).__name__, exc))
            return
        self.cv2 = cv2

    # -- per pose -----------------------------------------------------------------------
    @staticmethod
    def observations(event: Any) -> List[Tuple[float, float, int]]:
        """``(u, v, id)`` of the last observations of camera 0 (empty when the tracker has none)."""
        tracker = event.tracker
        if tracker is None:
            return []
        obs = _safe(lambda: tracker.get_last_observations(0), None) or []
        out: List[Tuple[float, float, int]] = []
        for o in obs:
            try:
                out.append((float(o.u), float(o.v), int(o.id)))
            except (TypeError, ValueError, AttributeError):
                continue
        return out

    def on_pose(self, event: Any) -> None:
        self.positions.append([float(v) for v in event.translation])
        if self.rr is not None or self.cv2 is not None:
            obs = self.observations(event)
            if self.rr is not None:
                self._log_rerun(event, obs)
            if self.cv2 is not None:
                self._draw(event, obs)

    def _log_rerun(self, event: Any, obs: List[Tuple[float, float, int]]) -> None:
        rr = self.rr
        try:
            rr.set_time("frame", sequence=int(event.index))
            rr.log("world/trajectory", rr.LineStrips3D([self.positions]), static=True)
            rr.log("world/rig",
                   rr.Transform3D(translation=[float(v) for v in event.translation],
                                  quaternion=[float(v) for v in event.rotation_xyzw]),
                   rr.Arrows3D(vectors=np.eye(3) * 0.2, colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]))
            if event.images:
                left = event.images[0]
                entities: List[Any] = []
                if obs:
                    entities.append(rr.Points2D(positions=np.array([[u, v] for u, v, _ in obs], dtype=np.float32),
                                                colors=np.array([color_from_id(i) for _, _, i in obs], dtype=np.uint8),
                                                radii=4.0))
                entities.append(rr.Image(left).compress(jpeg_quality=80))
                rr.log("world/rig/cam0", *entities)
            if event.gravity is not None:
                rr.log("world/rig/gravity", rr.Arrows3D(vectors=[[float(v) for v in event.gravity]],
                                                        colors=[[255, 0, 0]], radii=0.015))
        except Exception as exc:  # noqa: BLE001 - API drift between rerun versions must not stop tracking
            if not self._rr_error_reported:
                warn("rerun logging failed ({}: {}); the viewer is disabled for the rest of the run".format(
                    type(exc).__name__, exc))
                self._rr_error_reported = True
            self.rr = None

    def _draw(self, event: Any, obs: List[Tuple[float, float, int]]) -> None:
        cv2 = self.cv2
        try:
            if not event.images:
                return
            left = np.asarray(event.images[0])
            bgr = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR) if left.ndim == 2 else np.array(left, copy=True)
            for u, v, track_id in obs:
                r, g, b = color_from_id(track_id)
                cv2.circle(bgr, (int(round(u)), int(round(v))), 3, (b, g, r), -1, cv2.LINE_AA)
            t = event.translation
            text = "{} #{} {}{} | pos ({:+.2f}, {:+.2f}, {:+.2f}) m | {} obs | track {:.1f} ms | q quits".format(
                self.title, event.index, self.mode, " FAKE" if self.fake else "", float(t[0]), float(t[1]),
                float(t[2]), len(obs), event.track_ms)
            org = (10, 25)
            cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self.WINDOW, bgr)
            key = cv2.waitKey(1)
            if key >= 0 and (key & 0xFF) in (ord("q"), ord("Q"), 27):
                self.stop_requested = True
        except Exception as exc:  # noqa: BLE001 - a display problem must not stop tracking
            if not self._show_error_reported:
                warn("OpenCV window failed ({}: {}); --show disabled for the rest of the run".format(
                    type(exc).__name__, exc))
                self._show_error_reported = True
            self.cv2 = None

    def close(self) -> None:
        if self.cv2 is not None:
            _safe(self.cv2.destroyAllWindows)
            self.cv2 = None


# --------------------------------------------------------------------------------------
# Per-second statistics and the final summary
# --------------------------------------------------------------------------------------

def fmt_pos(p: Optional[Sequence[float]]) -> str:
    if p is None:
        return "-"
    return "({:+.3f}, {:+.3f}, {:+.3f})".format(float(p[0]), float(p[1]), float(p[2]))


def on_stats(stats: Dict[str, Any]) -> None:
    """One stdout line per ``STATS_INTERVAL_S`` from the tracking loop (the final one is the summary's job)."""
    if stats.get("final"):
        return
    attempted = stats["frames_attempted"]
    tracked = stats["frames_tracked"]
    pct = 100.0 * tracked / attempted if attempted else 0.0
    seen = stats["frames_seen"]
    total = stats.get("total_frames")
    progress = "{}/{}".format(seen, total) if total else str(seen)
    note("[{:7.1f} s] frames {} tracked {} ({:.1f} %) failed {} guarded {} resets {} | fps {:.1f} (mean {:.1f}) | "
         "track {:.1f} ms | pos {} | imu {}".format(
             stats["elapsed_s"], progress, tracked, pct, stats["frames_failed"], stats.get("frames_guarded", 0),
             stats["resets"], stats["fps"], stats["fps_mean"], stats["track_ms"], fmt_pos(stats.get("position")),
             stats["imu_registered"]))


def make_tracking_log(verbose: bool) -> Callable[[str], None]:
    """``TrackConfig.log`` sink: the loop's warnings go to stderr, its notes only with ``--verbose``, and its
    own per-interval status line is dropped (``on_stats`` prints ours)."""

    def tracking_log(message: str) -> None:
        if message.startswith("[track]"):
            return
        if message.startswith("note:") and not verbose:
            return
        print(message, file=sys.stderr, flush=True)

    return tracking_log


def fmt_observations(summary: Dict[str, Any]) -> str:
    """``min/median/max`` of the camera-0 observation counts, or ``-`` when the engine exported none."""
    lo, mid, hi = (summary.get(key) for key in ("observations_min", "observations_median", "observations_max"))
    if lo is None or mid is None or hi is None:
        return "-"
    return "{}/{:g}/{}".format(int(lo), float(mid), int(hi))


def fmt_m(value: Any) -> str:
    return "-" if value is None else "{:.3f}".format(float(value))


def print_summary(summary: Dict[str, Any], source_label: str, vslam: Any, tum_path: Path,
                  summary_path: Optional[Path]) -> None:
    attempted = summary["frames_attempted"]
    tracked = summary["frames_tracked"]
    pct = 100.0 * tracked / attempted if attempted else 0.0
    fake = " FAKE" if summary.get("fake") else ""
    note("Summary: mode {} ({}) on {} | cuvslam {}{}".format(
        summary["mode"], summary["odometry_mode"], source_label, cuvslam_version(vslam), fake))
    note("  frames: seen {}, warm-up {}, attempted {}, tracked {} ({:.1f} %), failed {}, resets {}, "
         "out-of-order {}, gaps {} (max {:.1f} ms)".format(
             summary["frames_seen"], summary["frames_warmup"], attempted, tracked, pct, summary["frames_failed"],
             summary["resets"], summary["frames_out_of_order"], summary["frame_gaps"], summary["max_frame_gap_ms"]))
    note("  timing: {:.2f} s of data in {:.2f} s wall, mean {:.1f} fps (engine {:.1f} fps, track {:.2f} ms mean / "
         "{:.2f} ms max)".format(summary["duration_s"], summary["wall_s"], summary["mean_fps"], summary["engine_fps"],
                                 summary["track_ms_mean"], summary["track_ms_max"]))
    if summary["mode"] == "vio":
        gravity = summary.get("gravity")
        note("  imu: registered {}, held back {}, dropped old {}, out of order {}, gaps {} (max {:.1f} ms), "
             "frames without imu {}, gravity {}".format(
                 summary["imu_registered"], summary["imu_held_back"], summary["imu_dropped_old"],
                 summary["imu_out_of_order"], summary["imu_gaps"], summary["max_imu_gap_ms"],
                 summary["frames_without_imu"],
                 "-" if gravity is None else "({:+.2f}, {:+.2f}, {:+.2f}) m/s^2".format(*gravity)))
    note("  trajectory: path length {:.3f} m, end-point offset {:.3f} m, max from start {:.3f} m, end {}".format(
        summary["path_length_m"], summary["endpoint_offset_m"], summary["max_from_start_m"],
        fmt_pos(summary.get("end_position"))))
    slam = summary.get("slam")
    if slam:
        note("  slam: {} pose(s), {} loop closure(s), {} pose-graph optimisation(s), {} map reset(s); end-point offset "
             "{} m as tracked, {} m re-optimised ({} rows); pose graph {} nodes / {} edges{}".format(
                 slam.get("rows"), slam.get("loop_closures"), slam.get("pgo_events"), slam.get("map_resets"),
                 fmt_m(slam.get("endpoint_offset_m")), fmt_m(slam.get("optimized_endpoint_offset_m")),
                 slam.get("optimized_rows"), slam.get("pose_graph_nodes"), slam.get("pose_graph_edges"),
                 "; map {} -> {}".format("saved" if slam.get("map_saved") else "NOT saved", slam.get("map_path"))
                 if slam.get("map_path") else ""))
    if (summary.get("config") or {}).get("coast"):
        note("  coast: {} estimated row(s) in {} gap(s), heading from the {}".format(
            summary.get("frames_coasted", 0), len(summary.get("coast_gaps") or []),
            "gyroscope" if summary.get("coast_gyro_used") else "last pose (no IMU rotation known)"))
    if (summary.get("config") or {}).get("guard"):
        note("  guard: {} frame(s) skipped (starved {}, degenerate {}, jump {}, spin {}), {} engine reset(s); "
             "observations min/median/max {}".format(
                 summary.get("frames_guarded", 0), summary.get("guard_starved", 0),
                 summary.get("guard_degenerate", 0), summary.get("guard_jump", 0), summary.get("guard_spin", 0),
                 summary.get("guard_resets", 0), fmt_observations(summary)))
    note("  stopped by: {}".format(summary["stopped_by"]))
    note("  TUM: {} ({} rows)".format(tum_path, summary["tum_rows"]))
    if summary_path is not None:
        note("  summary JSON: {}".format(summary_path))


# --------------------------------------------------------------------------------------
# Output files
# --------------------------------------------------------------------------------------

def slam_tum_path(tum_path: Path) -> Path:
    """``trajectory_<mode>.tum`` -> ``trajectory_<mode>_slam.tum`` next to it."""
    return tum_path.with_name(tum_path.stem + "_slam" + tum_path.suffix)


def resolve_out_path(out: Optional[Path], default_dir: Path, mode: str, live: bool) -> Path:
    """``--out`` (a file, or an existing directory that receives ``trajectory_<mode>.tum``) or the default."""
    if out is None:
        if live:
            return default_dir / "trajectory_{}_{}.tum".format(mode, timestamp_str())
        return default_dir / "trajectory_{}.tum".format(mode)
    out = Path(out).expanduser()
    if out.is_dir():
        return out / "trajectory_{}.tum".format(mode)
    return out


def resolve_summary_path(want: bool, explicit: Optional[Any], tum_path: Path) -> Optional[Path]:
    """``--summary-path PATH`` -> that path; ``--summary`` -> ``<tum stem>_summary.json``; neither -> ``None``."""
    if explicit:
        return Path(explicit).expanduser()
    if want:
        return tum_path.with_name(tum_path.stem + "_summary.json")
    return None


def shift_summary_timestamps(summary: Dict[str, Any], offset_ns: int) -> None:
    """Rebase the trajectory rows and the first/last stamps of a ``run_tracking`` summary by *offset_ns*.

    Used by a live run to turn device-clock nanoseconds into Unix-epoch nanoseconds, the
    recorder's convention (datasets/dataset-format.md section 8); tracking itself only
    ever saw the device clock, so nothing else changes.
    """
    offset_ns = int(offset_ns)
    summary["rows"] = [(int(row[0]) + offset_ns,) + tuple(row[1:]) for row in summary["rows"]]
    for key in ("first_ts_ns", "last_ts_ns"):
        if summary.get(key) is not None:
            summary[key] = int(summary[key]) + offset_ns


def dataset_time_note(reader: Any) -> str:
    """TUM header line describing the timestamp domain of a dataset run."""
    offset = ((reader.dataset or {}).get("time") or {}).get("epoch_offset_ns")
    if offset is not None:
        return "time: dataset timestamps (device clock + epoch offset {} ns = Unix-epoch ns)".format(offset)
    return "time: dataset timestamps as written in cam0/data.csv (ns)"


def live_time_note(epoch_offset_ns: Optional[int]) -> str:
    """TUM header line describing the timestamp domain of a live run."""
    if epoch_offset_ns:
        return ("time: device clock + epoch offset {} ns (Unix-epoch ns, as datasets/07_record_dataset.py "
                "writes)".format(int(epoch_offset_ns)))
    return "time: raw device-clock ns (the epoch offset never became known)"


def tum_header(source_label: str, summary: Dict[str, Any], vslam: Any, rig_frame: str,
               time_note: Optional[str] = None) -> List[str]:
    lines = [
        "source: {}".format(source_label),
        "mode: {} (OdometryMode.{}), cuvslam {}, async_sba {}".format(
            summary["mode"], summary["odometry_mode"], cuvslam_version(vslam), summary["async_sba"]),
        "frames: world = rig pose at the first tracked frame; rig = {}; x right, y down, z forward; metres; "
        "quaternion x y z w".format(rig_frame),
    ]
    if time_note:
        lines.append(time_note)
    lines.append("written {} by methods/cuvslam-vio/08_cuvslam_track.py".format(iso_now()))
    if summary.get("fake"):
        lines.insert(0, "FAKE cuvslam ({}): SYNTHETIC poses from cuvslam_oak.fake_cuvslam, NOT tracking results".format(
            cuvslam_version(vslam)))
    return lines


def write_outputs(summary: Dict[str, Any], *, tum_path: Path, summary_path: Optional[Path], source_label: str,
                  vslam: Any, rig_frame: str, extra: Dict[str, Any], time_note: Optional[str] = None) -> None:
    oaktrack.write_tum(tum_path, summary["rows"],
                       header=tum_header(source_label, summary, vslam, rig_frame, time_note))
    if summary.get("slam"):
        slam_rows = summary.get("slam_optimized_rows") or summary.get("slam_rows") or []
        oaktrack.write_tum(slam_tum_path(tum_path), slam_rows,
                           header=tum_header(source_label, summary, vslam, rig_frame, time_note) + [
                               "SLAM trajectory ({}): {} loop closure(s), {} map reset(s)".format(
                                   "re-optimised at the end" if summary.get("slam_optimized_rows") else "as tracked",
                                   summary["slam"].get("loop_closures"), summary["slam"].get("map_resets"))])
    if summary_path is None:
        return
    tracking = {k: v for k, v in summary.items() if k not in ("rows", "slam_rows", "slam_optimized_rows")}
    slam_tum = slam_tum_path(tum_path) if summary.get("slam") else None
    doc = {
        "generator": {
            "script": "methods/cuvslam-vio/08_cuvslam_track.py",
            "written": iso_now(),
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "cuvslam_version": cuvslam_version(vslam),
            "fake": bool(summary.get("fake")),
        },
        "mode": summary["mode"],
        "odometry_mode": summary["odometry_mode"],
        "outputs": {"tum": str(tum_path), "tum_rows": summary["tum_rows"], "rig_frame": rig_frame,
                    "slam_tum": None if slam_tum is None else str(slam_tum)},
        "tracking": tracking,
    }
    doc.update(extra)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", encoding="utf-8") as fh:
        json.dump(json_safe(doc), fh, indent=2)
        fh.write("\n")


# --------------------------------------------------------------------------------------
# Runs
# --------------------------------------------------------------------------------------

def effective_warmup(args: argparse.Namespace, *, live: bool) -> int:
    """``--warmup-frames`` or its default: 60 (NVIDIA's OAK-D example) live, 0 for a recording."""
    if args.warmup_frames is None:
        return DEFAULT_LIVE_WARMUP_FRAMES if live else 0
    return max(0, int(args.warmup_frames))


def resolve_imu_correction(args: argparse.Namespace, mode: str, device_id: Optional[str]) -> Any:
    """The ``--imu-calibration`` correction for a ``vio`` run, or ``None`` (raw IMU values).

    ``auto`` looks for ``datasets/calibration/imu_<device id>.json`` in the repo; a path names the file.
    Raises ``imu_calib.ImuCalibError`` (a ``ValueError``) when the file is missing or unusable.
    """
    choice = getattr(args, "imu_calibration", None)
    if mode != "vio" or not choice or str(choice).lower() == "none":
        return None
    if str(choice).lower() == "auto":
        correction = imu_calib.load_for_device(device_id)
        if correction is None:
            raise imu_calib.ImuCalibError("--imu-calibration auto: no {} (fit one with datasets/fit_imu_calibration.py)".format(
                imu_calib.default_path(device_id or "unknown")))
    else:
        correction = imu_calib.ImuCorrection.load(choice)
        if correction.device_id and device_id and correction.device_id != str(device_id):
            raise imu_calib.ImuCalibError("{} is for device {}, this data is from {}".format(
                choice, correction.device_id, device_id))
    note("IMU correction: {} ({})".format(correction.describe(), correction.path or choice))
    return correction


def make_config(args: argparse.Namespace, *, live: bool, should_stop: Callable[[], bool],
                imu_correction: Any = None, coast_rig_from_imu: Any = None) -> Any:
    return oaktrack.TrackConfig(
        use_denoising=bool(getattr(args, "denoising", False)),
        imu_correction=imu_correction,
        coast=bool(args.coast),
        coast_decay_s=args.coast_decay,
        coast_velocity_window_s=args.coast_velocity_window,
        coast_max_s=args.coast_max,
        coast_gyro=bool(args.coast_gyro),
        coast_keep_engine_rotation=bool(args.coast_keep_engine_rotation),
        coast_rig_from_imu=coast_rig_from_imu,
        slam=bool(args.slam),
        slam_sync=None if args.slam_async is None else (not args.slam_async),
        slam_max_map_size=args.slam_map_size,
        slam_save_map=str(args.slam_save) if args.slam_save else "",
        realtime=live,
        warmup_frames=effective_warmup(args, live=live),
        verbose=True,                     # warnings of the loop (rate-limited) always reach stderr
        max_frames=args.max_frames,
        stats_interval_s=STATS_INTERVAL_S,
        stitch_resets=bool(args.stitch_resets),
        multicam_mode=args.multicam_mode,
        should_stop=should_stop,
        log=make_tracking_log(bool(args.verbose)),
        guard=bool(args.guard),
        min_observations=args.min_observations,
        guard_zero_covariance=bool(args.guard_zero_covariance),
        max_speed_mps=args.max_speed,
        max_rot_dps=args.max_rot_rate,
        guard_reset_after=args.guard_reset_after,
    )


def make_should_stop(duration: float, viewer: Visualizer) -> Callable[[], bool]:
    deadline = time.monotonic() + duration if duration and duration > 0 else None

    def should_stop() -> bool:
        if viewer.stop_requested:
            return True
        return deadline is not None and time.monotonic() >= deadline

    return should_stop


def _alnum(text: str) -> str:
    """Lower-case letters and digits only (the yaml writer rewrites punctuation inside comments)."""
    return "".join(ch for ch in str(text).lower() if ch.isalnum())


def dataset_imu_availability(reader: Any) -> Tuple[bool, str]:
    """Whether the dataset can be tracked in ``'vio'`` mode, and why not."""
    if reader.imu_count == 0:
        return False, "imu0/data.csv has no samples"
    comment = str(getattr(reader.imu, "comment", "") or "")
    if _alnum(oakrig.MISSING_IMU_COMMENT)[:40] in _alnum(comment):
        return False, "imu0/sensor.yaml holds the identity placeholder of a stereo-only recording"
    imu_meta = (reader.dataset or {}).get("imu") or {}
    if imu_meta.get("available") is False:
        return False, "dataset.json says imu.available = false"
    return True, ""


def report_dataset_warnings(texts: Sequence[str], verbose: bool) -> None:
    """Dataset validation warnings on stderr: every one with *verbose*, otherwise one summarising line.

    Called only once the dataset, the mode and the rig were accepted, so an early exit-2
    failure is still exactly one stderr line.
    """
    lines = [one_line(t) for t in texts]
    if not lines:
        return
    if verbose or len(lines) == 1:
        for line in lines:
            warn("dataset: " + line)
        return
    warn("dataset: {} validation warnings, e.g. {} (--verbose lists them; all are in the summary JSON)".format(
        len(lines), lines[0]))


def run_dataset(args: argparse.Namespace, vslam: Any, root: Path, fake_chosen_by: Optional[str]) -> int:
    from cuvslam_oak import dataset as oakds

    try:
        reader = oakds.EurocReader(root)
    except oakds.DatasetError as exc:
        return fail("dataset {} is unusable: {}".format(root, exc))
    report = reader.validate(check_images=True, decode_images=3)
    if not report.ok:
        # Every detail on stdout, ONE line on stderr (the exit-2 contract of the docstring).
        for text in report.errors:
            note("dataset error: " + one_line(text))
        for text in report.warnings:
            note("dataset warning: " + one_line(text))
        first = one_line(report.errors[0]) if report.errors else "validation failed"
        more = len(report.errors) - 1
        return fail("dataset {} failed validation: {}{}".format(
            root, first, " (+{} more error(s), listed on stdout)".format(more) if more > 0 else ""))
    if reader.rig is None or reader.frame_count == 0:
        return fail("dataset {} has no usable sensor.yaml files or no stereo pairs".format(root))

    cam0 = reader.cam0
    imu_available, why_no_imu = dataset_imu_availability(reader)
    try:
        mode, reason = resolve_mode(args.mode, imu_available, why_no_imu)
    except ModeError as exc:
        return fail(str(exc))

    # --max-frames counts frames handed to the tracker; the warm-up frames come on top of them.
    warmup = effective_warmup(args, live=False)
    stop = (args.max_frames + warmup) if args.max_frames else None
    total = reader.frame_count if stop is None else min(reader.frame_count, stop)
    note("Source: dataset {}".format(root))
    note("  {} stereo pairs {}x{} @ {:g} fps, {} IMU samples @ {:g} Hz, {:.1f} s, undistorted={}, "
         "distortion_model={}{}".format(
             reader.frame_count, cam0.width, cam0.height, cam0.rate_hz, reader.imu_count, reader.imu.rate_hz,
             reader.duration_s, cam0.undistorted, cam0.distortion_model,
             ", first {} pairs only".format(total) if total < reader.frame_count else ""))
    note("Mode: {} ({})".format(mode, reason))

    try:
        if args.calibration:
            calib = oakrig.load_calibration_json(args.calibration)
            borders: Any = list(cam0.borders) if (not cam0.undistorted and any(cam0.borders)) else None
            rig_obj, _info = oakrig.build_rig(calib, cam0.resolution, mode, imu_rate_hz=reader.imu.rate_hz,
                                              borders=borders, vslam=vslam, rig_socket=args.rig_socket,
                                              undistorted=bool(cam0.undistorted))
            rig_source = "{} (build_rig, rig socket {}, undistorted={})".format(
                Path(args.calibration), args.rig_socket, cam0.undistorted)
            rig_frame = "{} optical frame".format(args.rig_socket)
        else:
            rig_obj = rig_from_dataset(vslam, reader.rig, mode)
            rig_source = "dataset sensor.yaml (cam0 = rig, as NVIDIA's dataset_utils)"
            rig_frame = "cam0 ({}) optical frame".format(cam0.socket or "CAM_B")
    except FileNotFoundError as exc:
        return fail(str(exc))
    except (ValueError, TypeError) as exc:  # CalibrationError / DatasetError are ValueErrors
        return fail("cannot build the cuVSLAM rig: {}".format(exc), EXIT_ERROR)
    rig_desc = describe_rig(rig_obj)
    note("Rig: {}".format(rig_source))
    print_rig(rig_desc, args.verbose)

    tum_path = resolve_out_path(args.out, root, mode, live=False)
    summary_path = resolve_summary_path(args.summary, args.summary_path, tum_path)
    viewer = Visualizer(use_rerun=args.rerun, show=args.show, mode=mode, fake=cuvslam_oak.is_fake(vslam),
                        title=root.name)
    try:
        imu_correction = resolve_imu_correction(args, mode, reader.rig.device_id or None)
    except ValueError as exc:
        return fail(str(exc))
    coast_rot = None
    if mode != "vio":
        try:
            coast_rot = reader.rig.rig_from_imu()          # from imu0/sensor.yaml (identity placeholder if no IMU)
        except (ValueError, AttributeError, TypeError):
            coast_rot = None
    cfg = make_config(args, live=False, should_stop=make_should_stop(args.duration, viewer),
                      imu_correction=imu_correction, coast_rig_from_imu=coast_rot)
    if args.verbose:
        note("TrackConfig: {}".format(json.dumps(json_safe(cfg.to_dict()), sort_keys=True)))
    stream = SizedStream(reader.merged(load_images=True, with_imu=(mode == "vio" or (cfg.coast and cfg.coast_gyro)),
                                       stop=stop), total)
    # The source is accepted: the validation warnings may go to stderr now (never before an exit-2 line).
    report_dataset_warnings(report.warnings, args.verbose)
    if fake_chosen_by is not None:
        announce_fake(vslam, fake_chosen_by)
    note("Tracking {} frame(s){}...".format(total, " ({} warm-up)".format(min(warmup, total)) if warmup else ""))
    try:
        summary = oaktrack.run_tracking(vslam, rig_obj, mode, stream, None, cfg, on_pose=viewer.on_pose,
                                        on_stats=on_stats)
    except oaktrack.TrackingError as exc:
        return fail("tracking failed: {}".format(exc), EXIT_ERROR)
    except oakds.DatasetError as exc:
        return fail("dataset {}: {}".format(root, exc), EXIT_ERROR)
    finally:
        viewer.close()

    source_label = "dataset {}".format(root)
    time_meta = (reader.dataset or {}).get("time") or {}
    extra = {
        "source": {
            "kind": "dataset", "path": str(root), "mav0": str(reader.mav0),
            "stereo_pairs": reader.frame_count, "imu_samples": reader.imu_count,
            "resolution": list(cam0.resolution), "fps": cam0.rate_hz, "imu_rate_hz": reader.imu.rate_hz,
            "duration_s": reader.duration_s, "undistorted": cam0.undistorted,
            "device_id": reader.rig.device_id or None,
            "time": {"domain": "as in the dataset", "epoch_offset_ns": time_meta.get("epoch_offset_ns")},
        },
        "rig": dict(rig_desc, source=rig_source),
        "calibration": str(Path(args.calibration)) if args.calibration else "dataset sensor.yaml",
        "validation_warnings": list(report.warnings),
        "args": {k: json_safe(v) for k, v in vars(args).items()},
    }
    try:
        write_outputs(summary, tum_path=tum_path, summary_path=summary_path, source_label=source_label, vslam=vslam,
                      rig_frame=rig_frame, extra=extra, time_note=dataset_time_note(reader))
    except OSError as exc:
        return fail("cannot write the output files: {}".format(exc), EXIT_ERROR)
    print_summary(summary, source_label, vslam, tum_path, summary_path)
    return EXIT_OK


def coast_rotation(calib: Any, rig_socket: Any) -> Any:
    """``rig_from_imu`` (4x4) for gyro coasting in stereo mode, ``None`` when the EEPROM has no IMU extrinsics."""
    try:
        return oakrig.rig_from_imu(calib, rig_socket)
    except (ValueError, KeyError, TypeError):
        return None


def live_rig(args: argparse.Namespace, vslam: Any, calib: Dict[str, Any]) -> Tuple[str, str, Any]:
    """``(mode, reason, vslam.Rig)`` for a live run from *calib* (an EEPROM dump or the ``--calibration`` file).

    Raises :class:`ModeError` when ``--mode vio`` was asked but the calibration has no IMU
    extrinsics, and ``CalibrationError`` / ``TypeError`` from ``build_rig``.
    """
    imu_available = oakrig.has_imu_extrinsics(calib)
    mode, reason = resolve_mode(args.mode, imu_available, "the calibration has no IMU extrinsics")
    width, height = args.size
    rig_obj, _info = oakrig.build_rig(calib, (width, height), mode, imu_rate_hz=args.imu_rate, vslam=vslam,
                                      rig_socket=args.rig_socket, undistorted=False)
    return mode, reason, rig_obj


def run_live(args: argparse.Namespace, vslam: Any, fake_chosen_by: Optional[str]) -> int:
    if common is None or oak_source is None:
        return fail("--live needs depthai and camera/common.py ({}); install depthai==3.10.0 or track a "
                    "--dataset instead".format(_COMMON_IMPORT_ERROR or _OAK_SOURCE_IMPORT_ERROR))
    width, height = args.size

    # Everything that needs no camera comes first: a bad --calibration file or --mode never boots it.
    calib_source = "device EEPROM (readCalibration)"
    mode = reason = ""
    rig_obj: Any = None
    if args.calibration:
        calib_source = str(Path(args.calibration))
        try:
            live_calib = oakrig.load_calibration_json(args.calibration)
            mode, reason, rig_obj = live_rig(args, vslam, live_calib)
        except FileNotFoundError as exc:
            return fail(str(exc))
        except ModeError as exc:
            return fail(str(exc))
        except (ValueError, TypeError) as exc:  # CalibrationError is a ValueError
            return fail("cannot build the cuVSLAM rig from {}: {}".format(calib_source, exc), EXIT_ERROR)

    device = common.open_device(args.device_id, retries=1 if args.device_id else 3)  # exits 2 on failure
    src: Any = None
    try:
        common.print_device_summary(device)
        device_id = str(_safe(device.getDeviceId, "?"))
        device_meta = _safe(lambda: oak_source.device_info(device), None)  # while the device is still open
        if rig_obj is None:
            try:
                live_calib = oak_source.dump_calibration(device)
                mode, reason, rig_obj = live_rig(args, vslam, live_calib)
            except ModeError as exc:
                return fail(str(exc))
            except (ValueError, TypeError, oak_source.OakSourceError) as exc:
                return fail("cannot build the cuVSLAM rig from the {}: {}".format(calib_source, exc), EXIT_ERROR)
        note("Source: live OAK {} at {}x{} @ {:g} fps, IMU {} Hz (raw streams)".format(
            device_id, width, height, args.fps, args.imu_rate))
        note("Mode: {} ({})".format(mode, reason))
        rig_desc = describe_rig(rig_obj)
        rig_frame = "{} optical frame".format(args.rig_socket)
        note("Rig: {} (build_rig, rig socket {}, raw frames)".format(calib_source, args.rig_socket))
        print_rig(rig_desc, args.verbose)

        tum_path = resolve_out_path(args.out, DEFAULT_RECORDINGS_DIR, mode, live=True)
        summary_path = resolve_summary_path(args.summary, args.summary_path, tum_path)
        viewer = Visualizer(use_rerun=args.rerun, show=args.show, mode=mode, fake=cuvslam_oak.is_fake(vslam),
                            title=device_id)
        try:
            imu_correction = resolve_imu_correction(args, mode, device_id)
        except ValueError as exc:
            return fail(str(exc))
        cfg = make_config(args, live=True, should_stop=make_should_stop(args.duration, viewer),
                          imu_correction=imu_correction, coast_rig_from_imu=coast_rotation(live_calib, args.rig_socket))
        if args.verbose:
            note("TrackConfig: {}".format(json.dumps(json_safe(cfg.to_dict()), sort_keys=True)))
        if fake_chosen_by is not None:
            announce_fake(vslam, fake_chosen_by)
        limits = []
        if args.duration > 0:
            limits.append("{:g} s".format(args.duration))
        if args.max_frames:
            limits.append("{} tracked frames".format(args.max_frames))
        stops = ["Ctrl-C"] + (["q in the window"] if viewer.cv2 is not None else [])
        note("Tracking live ({}; {} stops)...".format(
            "until " + " or ".join(limits) if limits else "no time or frame limit", " / ".join(stops)))
        try:
            src = oak_source.OakStereoImuSource(device, width, height, args.fps, args.imu_rate)
            with src:
                summary = oaktrack.run_tracking(vslam, rig_obj, mode, src.events(timeout_s=1.0), None, cfg,
                                                on_pose=viewer.on_pose, on_stats=on_stats)
        except oak_source.OakStreamError as exc:
            return fail("oak source: {}".format(exc), EXIT_ERROR)
        except oak_source.OakSourceError as exc:
            return fail("oak source: {}".format(exc), EXIT_ERROR)
        except oaktrack.TrackingError as exc:
            return fail("tracking failed: {}".format(exc), EXIT_ERROR)
        finally:
            viewer.close()
        source_stats = _safe(src.stats, {}) or {}
        if source_stats:
            note("Source stats: frames {}, frame gaps {}, sequence gaps {}, sync violations {}, imu samples {}, "
                 "imu gaps {}, stop reason {}".format(
                     source_stats.get("frames"), source_stats.get("frame_gaps"),
                     source_stats.get("stereo_sequence_gaps"), source_stats.get("stereo_sync_violations"),
                     source_stats.get("imu_samples"), source_stats.get("imu_gaps"), source_stats.get("stop_reason")))
        # Live TUM rows: device clock + epoch offset = Unix-epoch ns, the recorder's convention.
        raw_offset = getattr(src, "epoch_offset_ns", None)
        epoch_offset_ns: Optional[int] = _safe(lambda: int(raw_offset), None) if raw_offset is not None else None
        if epoch_offset_ns:
            shift_summary_timestamps(summary, epoch_offset_ns)
        time_note = live_time_note(epoch_offset_ns)
        note("Timestamps: {}".format(time_note[len("time: "):]))

        source_label = "live OAK {} {}x{}".format(device_id, width, height)
        extra = {
            "source": {
                "kind": "live", "device_id": device_id, "resolution": [width, height], "fps": args.fps,
                "imu_rate_hz": args.imu_rate, "device": device_meta,
                "time": {"domain": "device clock + epoch offset (Unix-epoch ns)" if epoch_offset_ns
                         else "raw device clock (ns)", "epoch_offset_ns": epoch_offset_ns},
            },
            "rig": dict(rig_desc, source=calib_source, rig_socket=args.rig_socket),
            "calibration": calib_source,
            "source_stats": source_stats,
            "args": {k: json_safe(v) for k, v in vars(args).items()},
        }
        try:
            write_outputs(summary, tum_path=tum_path, summary_path=summary_path, source_label=source_label,
                          vslam=vslam, rig_frame=rig_frame, extra=extra, time_note=time_note)
        except OSError as exc:
            return fail("cannot write the output files: {}".format(exc), EXIT_ERROR)
        print_summary(summary, source_label, vslam, tum_path, summary_path)
        return EXIT_OK
    finally:
        if src is not None:
            _safe(src.stop)
        _safe(device.close)  # harmless when pipeline.stop() already closed it


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run cuVSLAM (PyCuVSLAM 17.0.0) on an EuRoC recording of the OAK-D S2 or on the live camera and "
                    "write a TUM trajectory. Exit codes: 0 ok, 1 tracking error, 2 no device / dataset missing, "
                    "3 cuvslam not importable (see methods/cuvslam-vio/docs/cuvslam-setup.md).",
        epilog="Modes: stereo = OdometryMode.Multicamera (cameras only), vio = OdometryMode.Inertial (cameras + "
               "BNO086). Aliases: {}. The default is vio when IMU data is available, else stereo.".format(
                   ", ".join(sorted(oaktrack.MODE_ALIASES))),
    )
    src = parser.add_argument_group("source (one of)")
    src.add_argument("dataset_positional", nargs="?", metavar="DATASET", default=None,
                     help="EuRoC recording folder (holds mav0/) or its mav0 directory; same as --dataset")
    src.add_argument("--dataset", metavar="PATH", default=None, help="EuRoC recording to track offline")
    src.add_argument("--live", action="store_true", help="track the connected OAK-D S2 in real time")
    src.add_argument("--device-id", dest="device_id", default=None, metavar="ID",
                     help="OAK device id to open with --live, e.g. OAKDS2EXAMPLEID000 (default: first found)")

    trk = parser.add_argument_group("tracking")
    trk.add_argument("--mode", type=parse_mode, default=None, metavar="{stereo,vio}",
                     help="stereo (cameras only) or vio (cameras + IMU); default: vio if IMU data exists, else stereo")
    trk.add_argument("--calibration", metavar="PATH", default=None,
                     help="calibration.json (eepromToJson dump) or holistic camera_info.json to use instead of the "
                          "dataset's sensor.yaml / the device EEPROM (rig built with cuvslam_oak.rig.build_rig)")
    trk.add_argument("--rig-socket", choices=(oakrig.CAM_A, oakrig.CAM_B), default=DEFAULT_RIG_SOCKET,
                     help="rig frame for build_rig (live and --calibration): CAM_B = left camera, the dataset frame "
                          "(default); CAM_A = colour camera, as NVIDIA's OAK-D example")
    trk.add_argument("--max-frames", type=int, default=0, metavar="N",
                     help="stop after N frames handed to the tracker (0 = all; warm-up frames come on top)")
    trk.add_argument("--duration", type=float, default=0.0, metavar="SECONDS",
                     help="stop after this many wall-clock seconds (0 = no limit)")
    trk.add_argument("--warmup-frames", type=int, default=None, metavar="N",
                     help="frames skipped before the first track() (default: {} live, 0 dataset)".format(
                         DEFAULT_LIVE_WARMUP_FRAMES))
    trk.add_argument("--stitch-resets", action="store_true",
                     help="chain each segment after a tracking loss onto the last good pose (continuous trajectory)")
    trk.add_argument("--multicam-mode", choices=oaktrack.MULTICAM_MODES, default="Precision",
                     help="Tracker.MulticameraMode (default Precision; Performance for a small Jetson power budget)")
    trk.add_argument("--no-guard", dest="guard", action="store_false",
                     help="switch the starvation guard off and keep every pose cuVSLAM returns, including the garbage "
                          "ones it produces on a blank wall from close range (default: guard on)")
    trk.add_argument("--min-observations", type=int, default=oaktrack.DEFAULT_MIN_OBSERVATIONS, metavar="N",
                     help="guard: skip a pose tracked from fewer than N observations in camera 0 (default {}: off, "
                          "because on the OAK-D S2 the count did not drop on the starved frames, it sat at the "
                          "engine's cap of 450; kept for other sensors)".format(oaktrack.DEFAULT_MIN_OBSERVATIONS))
    trk.add_argument("--no-zero-covariance-guard", dest="guard_zero_covariance", action="store_false",
                     help="guard: keep poses whose position covariance is exactly zero (default: skip them; measured "
                          "on about half of the glitch frames and on well under 1 %% of normal ones)")
    trk.add_argument("--max-speed", type=float, default=oaktrack.DEFAULT_MAX_SPEED_MPS, metavar="M_PER_S",
                     help="guard: skip a pose that moved faster than this since the last accepted one (default {:g} "
                          "m/s, right for a hand-held walk or an indoor flight; a racing drone needs a higher "
                          "value; inf = off)".format(oaktrack.DEFAULT_MAX_SPEED_MPS))
    trk.add_argument("--max-rot-rate", type=float, default=oaktrack.DEFAULT_MAX_ROT_DPS, metavar="DEG_PER_S",
                     help="guard: skip a pose that rotated faster than this since the last accepted one (default {:g} "
                          "deg/s = 24 deg per frame at 30 fps, for hand-held / indoor flight; inf = off)".format(
                              oaktrack.DEFAULT_MAX_ROT_DPS))
    trk.add_argument("--guard-reset-after", type=int, default=oaktrack.DEFAULT_GUARD_RESET_AFTER, metavar="N",
                     help="re-create the cuVSLAM tracker after N consecutive guarded frames and chain the trajectory "
                          "onto the last good pose (default {}; 0 = never)".format(oaktrack.DEFAULT_GUARD_RESET_AFTER))
    trk.add_argument("--slam", action="store_true",
                     help="run cuVSLAM's SLAM layer (loop closure, pose graph) next to the odometry; writes "
                          "trajectory_<mode>_slam.tum too (re-optimised at the end) and reports loop closures")
    trk.add_argument("--slam-map-size", type=int, default=0, metavar="N",
                     help="poses kept in the pose graph (default 0 = unlimited; NVIDIA's default 300 forgets the "
                          "start of a run longer than a few minutes)")
    trk.add_argument("--slam-async", dest="slam_async", action="store_true", default=None,
                     help="loop closure in its own thread (default: synchronous, deterministic, unless --live)")
    trk.add_argument("--slam-save", type=Path, default=None, metavar="DIR",
                     help="save the SLAM map into DIR at the end (for localize_in_map later)")
    trk.add_argument("--no-coast", dest="coast", action="store_false",
                     help="no estimate while the engine gives no pose: the trajectory freezes across guard skips "
                          "and losses (default: coast at the last velocity, rotating with the gyro)")
    trk.add_argument("--coast-decay", type=float, default=oaktrack.DEFAULT_COAST_DECAY_S, metavar="SECONDS",
                     help="time constant of the coasted velocity's decay (default %(default)s)")
    trk.add_argument("--coast-max", type=float, default=oaktrack.DEFAULT_COAST_MAX_S, metavar="SECONDS",
                     help="longest gap without vision that is bridged with estimates (default %(default)s)")
    trk.add_argument("--coast-velocity-window", type=float, default=oaktrack.DEFAULT_COAST_VELOCITY_WINDOW_S,
                     metavar="SECONDS", help="accepted poses the coasting velocity is fitted over (default %(default)s; "
                                             "0 = hold the position, rotate only)")
    trk.add_argument("--coast-keep-engine-rotation", dest="coast_keep_engine_rotation", action="store_true",
                     help="when the engine tracked through a gap without restarting, keep its rotation at the "
                          "re-anchor and correct only the position (default: the gyro-coasted rotation wins; "
                          "measured better for the heading)")
    trk.add_argument("--no-coast-gyro", dest="coast_gyro", action="store_false",
                     help="hold the heading while coasting instead of integrating the gyroscope")
    trk.add_argument("--imu-calibration", metavar="PATH|auto|none", default="none",
                     help="vio: correct every IMU sample (accelerometer bias and scale, gyroscope bias) before it is "
                          "registered; 'auto' = datasets/calibration/imu_<device id>.json written by "
                          "datasets/fit_imu_calibration.py, or the path of such a file (default none: on the walks "
                          "of 2026-09 the correction changed the drift by less than the run-to-run spread)")
    trk.add_argument("--denoising", action="store_true",
                     help="enable cuVSLAM's image denoising (use_denoising; off by default as in NVIDIA's examples)")
    trk.add_argument("--fake-cuvslam", action="store_true",
                     help="use cuvslam_oak.fake_cuvslam (synthetic poses) for a dry run; also honoured: {}=1".format(
                         cuvslam_oak.FAKE_ENV_VAR))

    live = parser.add_argument_group("live camera")
    live.add_argument("--size", type=parse_size, default=DEFAULT_LIVE_SIZE, metavar="WxH",
                      help="stereo frame size: 1280x800 (default, native) or 640x400")
    live.add_argument("--fps", type=float, default=DEFAULT_LIVE_FPS, help="camera frame rate (default 30)")
    live.add_argument("--imu-rate", type=int, default=DEFAULT_IMU_RATE_HZ, metavar="HZ",
                      help="BNO086 report rate for ACCELEROMETER_RAW + GYROSCOPE_RAW (default {}, max {}: the "
                           "gyroscope jitters above it)".format(DEFAULT_IMU_RATE_HZ, MAX_IMU_RATE_HZ))

    out = parser.add_argument_group("output")
    out.add_argument("--out", type=Path, default=None, metavar="PATH",
                     help="TUM trajectory file, or an existing directory for trajectory_<mode>.tum (default: next to "
                          "the dataset, or recordings/trajectory_<mode>_<stamp>.tum for --live)")
    out.add_argument("--summary", action="store_true",
                     help="also write a JSON summary next to the TUM file (<tum stem>_summary.json)")
    out.add_argument("--summary-path", dest="summary_path", metavar="PATH", default=None,
                     help="write the JSON summary to PATH instead (implies --summary)")
    out.add_argument("--rerun", action="store_true",
                     help="visualise in a rerun viewer (lazy import; a missing rerun-sdk only prints a hint)")
    out.add_argument("--show", action="store_true",
                     help="OpenCV window with the left image and the tracked observations; q / Esc stop the run")
    out.add_argument("--verbose", action="store_true",
                     help="print the full rig, the TrackConfig and every dataset validation warning (otherwise one "
                          "line summarises them)")
    return parser


def _format_warning(message: Any, category: Any, filename: str, lineno: int, line: Optional[str] = None) -> str:
    """One stderr line per Python warning (CalibrationWarning, RuntimeWarning of the version check, ...)."""
    return "warning: {}: {}\n".format(getattr(category, "__name__", category), one_line(message))


def main(argv: Optional[Sequence[str]] = None) -> int:
    warnings.formatwarning = _format_warning  # type: ignore[assignment]
    parser = build_parser()
    args = parser.parse_args(argv)

    # ---- source selection (before anything expensive) --------------------------------
    dataset_arg = args.dataset if args.dataset is not None else args.dataset_positional
    if args.dataset is not None and args.dataset_positional is not None and \
            Path(args.dataset).expanduser() != Path(args.dataset_positional).expanduser():
        return fail("give the dataset once: positional {!r} and --dataset {!r} differ".format(
            args.dataset_positional, args.dataset))
    if args.live and dataset_arg is not None:
        return fail("--live and a dataset are mutually exclusive")
    if not args.live and dataset_arg is None:
        return fail("nothing to track: give an EuRoC recording (positional or --dataset PATH) or --live "
                    "(see --help)")
    if args.calibration is not None:
        # With the other argument errors: a missing file never opens the dataset (or, for --live,
        # the camera) and no Source/Mode lines precede the one stderr line.
        calib_file = Path(args.calibration)
        if not calib_file.is_file():
            return fail("calibration file not found: {}".format(calib_file))
        try:
            with calib_file.open("rb") as fh:
                fh.read(1)
        except OSError as exc:
            return fail("calibration file {} is not readable: {}".format(calib_file, exc.strerror or exc))
    if args.max_frames < 0:
        return fail("--max-frames must be >= 0")
    if not math.isfinite(args.duration) or args.duration < 0:
        return fail("--duration must be a non-negative number")
    if args.fps <= 0 or not math.isfinite(args.fps):
        return fail("--fps must be positive")
    if args.imu_rate <= 0:
        return fail("--imu-rate must be positive")
    if args.imu_rate > MAX_IMU_RATE_HZ:
        return fail("--imu-rate {} is above the {} Hz maximum (BNO08X gyroscope jitter above it, "
                    "cuvslam_oak.oak_source.MAX_IMU_RATE_HZ)".format(args.imu_rate, MAX_IMU_RATE_HZ))
    if args.warmup_frames is not None and args.warmup_frames < 0:
        return fail("--warmup-frames must be >= 0")
    if args.min_observations < 0:
        return fail("--min-observations must be >= 0")
    if args.guard_reset_after < 0:
        return fail("--guard-reset-after must be >= 0")
    for flag, value in (("--max-speed", args.max_speed), ("--max-rot-rate", args.max_rot_rate)):
        if math.isnan(value) or value < 0:
            return fail("{} must be a non-negative number (inf switches the rule off)".format(flag))
    if args.summary_path:
        args.summary = True
    args.dataset = dataset_arg

    root: Optional[Path] = None
    if dataset_arg is not None:
        from cuvslam_oak import dataset as oakds
        try:
            root, _mav0 = oakds.resolve_mav0(dataset_arg)
        except oakds.DatasetError as exc:
            return fail(str(exc))

    # ---- cuvslam ---------------------------------------------------------------------
    try:
        vslam, chosen_by = load_cuvslam(args.fake_cuvslam)
    except (ImportError, OSError) as exc:
        reason = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
        return fail("{}; install PyCuVSLAM 17.0.0 as described in methods/cuvslam-vio/docs/cuvslam-setup.md (WSL2 / Jetson), "
                    "or pass --fake-cuvslam for a dry run with synthetic poses".format(reason), EXIT_NO_CUVSLAM)
    # The fake is announced (loudly, on stderr) right before tracking starts, once the source is
    # known to be usable, so every early failure still produces exactly one stderr line.
    fake_chosen_by: Optional[str] = chosen_by if cuvslam_oak.is_fake(vslam) else None
    if fake_chosen_by is None:
        note("cuvslam {} ({})".format(cuvslam_version(vslam), getattr(vslam, "__file__", "?")))

    try:
        if root is not None:
            return run_dataset(args, vslam, root, fake_chosen_by)
        return run_live(args, vslam, fake_chosen_by)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl-C)", flush=True)
        return 130


if __name__ == "__main__":
    sys.exit(main())
