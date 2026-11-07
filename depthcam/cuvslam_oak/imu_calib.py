"""Accelerometer / gyroscope correction for the OAK's raw IMU streams.

``ACCELEROMETER_RAW`` and ``GYROSCOPE_RAW`` are uncalibrated by definition.  On this project's unit
(OAK-D-S2-AF ``OAKDS2EXAMPLEID000``, BNO086 firmware 3.9.9) a still camera read 9.05 to 10.44 m/s^2
depending on which way was up (measured 2026-09-29): the y axis carries a bias of about
-0.69 m/s^2.  cuVSLAM's ``ImuCalibration`` has no bias field and its inertial mode is given noise
and random-walk figures that assume a calibrated sensor, so the samples are corrected before
they are registered::

    accel_corrected = (accel_raw - accel_bias) * accel_scale      # per axis, IMU frame
    gyro_corrected  =  gyro_raw  - gyro_bias

The correction is fitted from a *six-pose* recording (``datasets/07_record_dataset.py``, the camera
resting still on each of its six faces for a few seconds, any order, exact levelling not needed):
every still stretch must measure the same gravity magnitude, which fixes three biases and three
scales (:func:`fit_accel`).  Datasets keep the raw values; the correction is a small JSON file
(``datasets/calibration/imu_<device id>.json``) applied at tracking time, so old recordings benefit too.

A bias of this kind drifts with temperature; refit when the still-camera ``|accel|`` of a new
recording (``datasets/fit_imu_calibration.py --check``) is off by more than about 0.1 m/s^2.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

__all__ = [
    "STANDARD_GRAVITY",
    "CALIBRATION_DIR",
    "ImuCalibError",
    "ImuCorrection",
    "StaticSegment",
    "AccelFit",
    "find_static_segments",
    "fit_accel",
    "fit_six_pose",
    "load_imu_csv",
    "default_path",
    "load_for_device",
]

#: Standard gravity (m/s^2); the local value differs by less than 0.05 %.
STANDARD_GRAVITY = 9.80665
#: Folder (under the repo root) that holds ``imu_<device id>.json``.
CALIBRATION_DIR = "datasets/calibration"

_REPO_ROOT = Path(__file__).resolve().parent.parent


class ImuCalibError(ValueError):
    """The data cannot be fitted or the correction file is unusable."""


def _vec3(value: Any, what: str) -> Tuple[float, float, float]:
    try:
        v = [float(x) for x in value]
    except (TypeError, ValueError) as exc:
        raise ImuCalibError("{} must be three numbers, got {!r}".format(what, value)) from exc
    if len(v) != 3 or not all(math.isfinite(x) for x in v):
        raise ImuCalibError("{} must be three finite numbers, got {!r}".format(what, value))
    return (v[0], v[1], v[2])


@dataclass
class ImuCorrection:
    """Per-axis accelerometer bias and scale plus gyroscope bias, in the IMU's own frame.

    ``apply`` returns ``(gyro - gyro_bias, (accel - accel_bias) * accel_scale)``.  The remaining
    fields are provenance: which device, which recording, when, the gravity used, how well the
    still poses agree after the correction (``residual_std``, m/s^2) and how many there were.
    """

    accel_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    gyro_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    device_id: str = ""
    source: str = ""
    fitted: str = ""
    gravity: float = STANDARD_GRAVITY
    residual_std: Optional[float] = None
    poses: int = 0
    notes: str = ""
    path: str = field(default="", compare=False)

    def __post_init__(self) -> None:
        self.accel_bias = _vec3(self.accel_bias, "accel_bias")
        self.accel_scale = _vec3(self.accel_scale, "accel_scale")
        self.gyro_bias = _vec3(self.gyro_bias, "gyro_bias")
        if not all(0.5 < s < 2.0 for s in self.accel_scale):
            raise ImuCalibError("accel_scale must be close to 1 (0.5 .. 2), got {!r}".format(self.accel_scale))
        self.gravity = float(self.gravity)
        self.poses = int(self.poses)

    @property
    def is_identity(self) -> bool:
        return (self.accel_bias == (0.0, 0.0, 0.0) and self.accel_scale == (1.0, 1.0, 1.0)
                and self.gyro_bias == (0.0, 0.0, 0.0))

    def apply(self, gyro: Sequence[float], accel: Sequence[float]) -> Tuple[Tuple[float, float, float],
                                                                          Tuple[float, float, float]]:
        """Corrected ``(gyro, accel)`` of one sample (rad/s, m/s^2)."""
        g = (float(gyro[0]) - self.gyro_bias[0], float(gyro[1]) - self.gyro_bias[1],
             float(gyro[2]) - self.gyro_bias[2])
        a = ((float(accel[0]) - self.accel_bias[0]) * self.accel_scale[0],
             (float(accel[1]) - self.accel_bias[1]) * self.accel_scale[1],
             (float(accel[2]) - self.accel_bias[2]) * self.accel_scale[2])
        return g, a

    def apply_arrays(self, gyro: np.ndarray, accel: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Corrected ``(gyro, accel)`` for ``(N, 3)`` arrays."""
        g = np.asarray(gyro, dtype=np.float64) - np.array(self.gyro_bias)
        a = (np.asarray(accel, dtype=np.float64) - np.array(self.accel_bias)) * np.array(self.accel_scale)
        return g, a

    def to_dict(self) -> Dict[str, Any]:
        return {
            "format": "cuvslam_oak.imu_correction",
            "format_version": 1,
            "device_id": self.device_id,
            "accel_bias": list(self.accel_bias),
            "accel_scale": list(self.accel_scale),
            "gyro_bias": list(self.gyro_bias),
            "model": "accel = (raw - accel_bias) * accel_scale; gyro = raw - gyro_bias; IMU frame, m/s^2, rad/s",
            "gravity": self.gravity,
            "residual_std": self.residual_std,
            "poses": self.poses,
            "source": self.source,
            "fitted": self.fitted,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any], path: str = "") -> "ImuCorrection":
        if not isinstance(data, dict) or "accel_bias" not in data:
            raise ImuCalibError("not an IMU correction (no accel_bias){}".format(" in " + path if path else ""))
        residual = data.get("residual_std")
        return cls(accel_bias=data["accel_bias"], accel_scale=data.get("accel_scale", (1.0, 1.0, 1.0)),
                   gyro_bias=data.get("gyro_bias", (0.0, 0.0, 0.0)), device_id=str(data.get("device_id") or ""),
                   source=str(data.get("source") or ""), fitted=str(data.get("fitted") or ""),
                   gravity=float(data.get("gravity") or STANDARD_GRAVITY),
                   residual_std=None if residual is None else float(residual),
                   poses=int(data.get("poses") or 0), notes=str(data.get("notes") or ""), path=path)

    @classmethod
    def load(cls, path: Union[str, Path]) -> "ImuCorrection":
        p = Path(path)
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except OSError as exc:
            raise ImuCalibError("cannot read {}: {}".format(p, exc)) from exc
        except ValueError as exc:
            raise ImuCalibError("{} is not valid JSON: {}".format(p, exc)) from exc
        return cls.from_dict(data, path=str(p))

    def save(self, path: Union[str, Path]) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8", newline="\n")
        return p

    def describe(self) -> str:
        """One line for logs."""
        return ("accel bias ({:+.3f}, {:+.3f}, {:+.3f}) m/s^2, scale ({:.4f}, {:.4f}, {:.4f}), "
                "gyro bias ({:+.5f}, {:+.5f}, {:+.5f}) rad/s".format(*self.accel_bias, *self.accel_scale,
                                                                    *self.gyro_bias))


@dataclass
class StaticSegment:
    """One still stretch of an IMU stream (times in seconds from the first sample)."""

    start_s: float
    end_s: float
    accel_mean: np.ndarray
    accel_std: np.ndarray
    gyro_mean: np.ndarray
    gyro_norm_mean: float
    samples: int

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    @property
    def norm(self) -> float:
        return float(np.linalg.norm(self.accel_mean))


@dataclass
class AccelFit:
    """Result of :func:`fit_accel`."""

    bias: np.ndarray
    scale: np.ndarray
    gravity: float
    residuals: np.ndarray          #: corrected ``|accel|`` minus gravity, one per observation (m/s^2)
    condition: float
    raw_norm_min: float
    raw_norm_max: float

    @property
    def residual_std(self) -> float:
        return float(np.sqrt(np.mean(self.residuals ** 2)))


def load_imu_csv(path: Union[str, Path]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(t_s, gyro, accel)`` from a EuRoC ``imu0/data.csv`` (or the dataset folder that holds it).

    ``t_s`` starts at 0; columns are ``timestamp_ns, wx, wy, wz, ax, ay, az``.
    """
    p = Path(path)
    if p.is_dir():
        for candidate in (p / "mav0" / "imu0" / "data.csv", p / "imu0" / "data.csv", p / "data.csv"):
            if candidate.is_file():
                p = candidate
                break
        else:
            raise ImuCalibError("no imu0/data.csv under {}".format(path))
    try:
        data = np.loadtxt(str(p), delimiter=",", skiprows=1, ndmin=2)
    except (OSError, ValueError) as exc:
        raise ImuCalibError("cannot read {}: {}".format(p, exc)) from exc
    if data.shape[0] < 2 or data.shape[1] < 7:
        raise ImuCalibError("{} has {} row(s) x {} column(s); need timestamp + 6 values".format(p, *data.shape))
    t = (data[:, 0] - data[0, 0]) / 1e9
    return t, data[:, 1:4], data[:, 4:7]


def find_static_segments(t_s: np.ndarray, gyro: np.ndarray, accel: np.ndarray, *, window_s: float = 0.5,
                         gyro_max: float = 0.15, accel_std_max: float = 0.7, merge_tolerance: float = 0.6,
                         min_duration_s: float = 2.0) -> List[StaticSegment]:
    """Still stretches of an IMU stream.

    A *window_s* window is still when its mean ``|gyro|`` is below *gyro_max* (rad/s) and no
    accelerometer axis has a standard deviation above *accel_std_max* (m/s^2); the defaults accept
    a camera held against a surface by hand.  Consecutive still windows whose mean acceleration
    stays within *merge_tolerance* (m/s^2) of the segment's first window form one segment;
    segments shorter than *min_duration_s* are dropped.  The first and last window of each
    segment are trimmed when the segment is long enough (the hand is still settling).
    """
    t_s = np.asarray(t_s, dtype=np.float64)
    gyro = np.asarray(gyro, dtype=np.float64)
    accel = np.asarray(accel, dtype=np.float64)
    if t_s.ndim != 1 or gyro.shape != (len(t_s), 3) or accel.shape != (len(t_s), 3):
        raise ImuCalibError("expected t (N,), gyro (N, 3), accel (N, 3)")
    if len(t_s) < 2:
        return []
    edges = np.arange(t_s[0], t_s[-1] + window_s, window_s)
    index = np.searchsorted(t_s, edges)
    windows: List[Tuple[int, int, bool, np.ndarray]] = []
    for a, b in zip(index[:-1], index[1:]):
        if b - a < 5:
            continue
        still = (float(np.linalg.norm(gyro[a:b], axis=1).mean()) < gyro_max
                 and float(accel[a:b].std(axis=0).max()) < accel_std_max)
        windows.append((int(a), int(b), still, accel[a:b].mean(axis=0)))

    groups: List[List[Tuple[int, int, bool, np.ndarray]]] = []
    current: List[Tuple[int, int, bool, np.ndarray]] = []
    for w in windows:
        if w[2] and (not current or float(np.linalg.norm(w[3] - current[0][3])) < merge_tolerance):
            current.append(w)
            continue
        if current:
            groups.append(current)
        current = [w] if w[2] else []
    if current:
        groups.append(current)

    segments: List[StaticSegment] = []
    for group in groups:
        if len(group) >= 6:  # 3 s or more: drop the settling windows at both ends
            group = group[1:-1]
        a, b = group[0][0], group[-1][1]
        duration = float(t_s[b - 1] - t_s[a])
        if duration < min_duration_s:
            continue
        segments.append(StaticSegment(start_s=float(t_s[a]), end_s=float(t_s[b - 1]),
                                      accel_mean=accel[a:b].mean(axis=0), accel_std=accel[a:b].std(axis=0),
                                      gyro_mean=gyro[a:b].mean(axis=0),
                                      gyro_norm_mean=float(np.linalg.norm(gyro[a:b], axis=1).mean()),
                                      samples=int(b - a)))
    return segments


def _coverage_problem(observations: np.ndarray) -> Optional[str]:
    """Why the still poses cannot fix three biases and three scales, or ``None`` when they can."""
    unit = observations / np.linalg.norm(observations, axis=1, keepdims=True)
    missing = []
    for axis, name in enumerate("xyz"):
        if not (unit[:, axis] > 0.7).any():
            missing.append("+" + name)
        if not (unit[:, axis] < -0.7).any():
            missing.append("-" + name)
    if missing:
        return "no still pose with gravity along {}".format(", ".join(missing))
    return None


def fit_accel(observations: Any, gravity: float = STANDARD_GRAVITY, *, require_six_poses: bool = True) -> AccelFit:
    """Per-axis bias and scale such that ``|(a - bias) * scale| = gravity`` for every still observation.

    *observations* is ``(N, 3)``: mean accelerometer readings of still stretches in different
    orientations (N >= 6).  The constraint is linear in ``[a^2, a]`` (an axis-aligned ellipsoid),
    solved by least squares.  With *require_six_poses* every axis must have pointed both up and
    down in some pose (within about 45 degrees), otherwise bias and scale cannot be told apart.
    """
    A = np.asarray(observations, dtype=np.float64)
    if A.ndim != 2 or A.shape[1] != 3 or A.shape[0] < 6:
        raise ImuCalibError("need at least 6 still observations of 3 values, got shape {}".format(A.shape))
    if not np.all(np.isfinite(A)):
        raise ImuCalibError("observations contain non-finite values")
    if require_six_poses:
        problem = _coverage_problem(A)
        if problem:
            raise ImuCalibError(problem + " (rest the camera on each of its six faces)")
    design = np.hstack([A ** 2, A])
    solution, _res, rank, singular = np.linalg.lstsq(design, np.ones(len(A)), rcond=None)
    if rank < 6 or np.any(solution[:3] <= 0.0):
        raise ImuCalibError("the still poses do not determine bias and scale (rank {}, quadratic terms {})".format(
            rank, np.round(solution[:3], 6).tolist()))
    bias = -solution[3:] / (2.0 * solution[:3])
    k = gravity ** 2 / (1.0 + float(np.sum(solution[:3] * bias ** 2)))
    scale = np.sqrt(solution[:3] * k)
    norms = np.linalg.norm((A - bias) * scale, axis=1)
    raw = np.linalg.norm(A, axis=1)
    return AccelFit(bias=bias, scale=scale, gravity=float(gravity), residuals=norms - gravity,
                    condition=float(singular[0] / singular[-1]), raw_norm_min=float(raw.min()),
                    raw_norm_max=float(raw.max()))


def fit_six_pose(t_s: np.ndarray, gyro: np.ndarray, accel: np.ndarray, *, gravity: float = STANDARD_GRAVITY,
                 sub_window_s: float = 1.0, gyro_bias_max_motion: float = 0.03,
                 **segment_options: Any) -> Tuple[ImuCorrection, AccelFit, List[StaticSegment]]:
    """Fit an :class:`ImuCorrection` from a six-pose recording.

    The still segments (:func:`find_static_segments`) are cut into *sub_window_s* pieces so a
    hand-held pose contributes its small orientation changes as extra observations.  The
    gyroscope bias is the mean rate over the segments that were really at rest (mean ``|gyro|``
    below *gyro_bias_max_motion*), weighted by their length; zero when there is none.
    """
    segments = find_static_segments(t_s, gyro, accel, **segment_options)
    if len(segments) < 6:
        raise ImuCalibError("found {} still pose(s), need 6 (each face down for a few seconds): {}".format(
            len(segments), ", ".join("{:.0f}-{:.0f} s".format(s.start_s, s.end_s) for s in segments) or "none"))
    problem = _coverage_problem(np.array([s.accel_mean for s in segments]))
    if problem:
        raise ImuCalibError(problem + " (rest the camera on each of its six faces)")
    t_s = np.asarray(t_s, dtype=np.float64)
    accel = np.asarray(accel, dtype=np.float64)
    observations: List[np.ndarray] = []
    for seg in segments:
        start = seg.start_s
        while start + sub_window_s <= seg.end_s + 1e-9:
            mask = (t_s >= start) & (t_s < start + sub_window_s)
            if int(mask.sum()) >= 5:
                observations.append(accel[mask].mean(axis=0))
            start += sub_window_s
    fit = fit_accel(np.array(observations), gravity)
    at_rest = [s for s in segments if s.gyro_norm_mean < gyro_bias_max_motion]
    if at_rest:
        weights = np.array([s.samples for s in at_rest], dtype=np.float64)
        gyro_bias = np.average(np.array([s.gyro_mean for s in at_rest]), axis=0, weights=weights)
    else:
        gyro_bias = np.zeros(3)
    correction = ImuCorrection(accel_bias=tuple(fit.bias), accel_scale=tuple(fit.scale), gyro_bias=tuple(gyro_bias),
                               gravity=gravity, residual_std=fit.residual_std, poses=len(segments))
    return correction, fit, segments


def default_path(device_id: str, root: Optional[Union[str, Path]] = None) -> Path:
    """``<root>/datasets/calibration/imu_<device id>.json`` (root defaults to the repo root)."""
    base = Path(root) if root is not None else _REPO_ROOT
    safe = "".join(c for c in str(device_id) if c.isalnum() or c in "-_") or "unknown"
    return base / CALIBRATION_DIR / "imu_{}.json".format(safe)


def load_for_device(device_id: Optional[str], root: Optional[Union[str, Path]] = None) -> Optional[ImuCorrection]:
    """The stored correction of *device_id*, or ``None`` when there is no file for it.

    A file that exists but cannot be used raises :class:`ImuCalibError`.
    """
    if not device_id:
        return None
    path = default_path(device_id, root)
    if not path.is_file():
        return None
    correction = ImuCorrection.load(path)
    if correction.device_id and correction.device_id != str(device_id):
        raise ImuCalibError("{} is for device {}, not {}".format(path, correction.device_id, device_id))
    return correction
