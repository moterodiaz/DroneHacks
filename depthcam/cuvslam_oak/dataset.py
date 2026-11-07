"""EuRoC ASL dataset writer and reader for OAK-D S2 cuVSLAM recordings.

Implements ``datasets/dataset-format.md``: one recording folder holds ``mav0/`` in the
EuRoC ASL layout (``cam0``/``cam1`` with ``data/<ts_ns>.png`` + ``data.csv`` + ``sensor.yaml``,
``imu0`` with ``data.csv`` + ``sensor.yaml``) plus our own ``dataset.json``, ``README.txt`` and
optional ``calibration.json``.  A recording written by :class:`EurocWriter` loads unmodified in
NVIDIA's ``examples/euroc/track_euroc.py`` (PyCuVSLAM v17.0.0) *and* in :class:`EurocReader`.

Key format rules (all enforced or checked here):

* timestamps are integer nanoseconds on one clock; frame timestamps strictly increasing, IMU
  timestamps non-decreasing; cam0 and cam1 rows share one timestamp per stereo pair;
* images are 8-bit single-channel PNGs (PIL mode ``L``), same resolution on both cameras;
* ``imu0/data.csv`` columns are ``timestamp, gx, gy, gz, ax, ay, az`` (gyro rad/s FIRST, then
  accel m/s^2, specific force, gravity not subtracted);
* ``sensor.yaml`` floats are written so that YAML 1.1 (``yaml.safe_load``) reads them as floats:
  mantissa with a dot, exponent with an explicit sign (``1.0e-05``, never ``1e-05``);
* ``T_BS`` is body_from_sensor (row-major 4x4), body = cam0 = left camera optical frame.

This module has no dependency on ``cuvslam``, ``depthai``, ``pyyaml`` or ``scipy``: it uses only
the standard library, numpy and OpenCV, so it runs on Windows (recording, tests), WSL2 and the
Jetson.  YAML is written by hand in the EuRoC style and parsed with a tiny reader that follows the
YAML 1.1 scalar rules that matter (so a badly formatted float is reported as a string, exactly as
``yaml.safe_load`` would see it).  Calibration is passed in as plain :class:`RigInfo` /
:class:`CameraInfo` / :class:`ImuInfo` dataclasses; building a ``cuvslam.Rig`` from a
:class:`RigInfo` is ``cuvslam_oak.rig``'s job.

Typical use::

    rig = RigInfo(cam0=CameraInfo(...), cam1=CameraInfo(...), imu=ImuInfo(...))
    with EurocWriter(root, rig, cam_rate_hz=30, imu_rate_hz=200) as w:
        w.add_imu(ts_ns, gyro_xyz, accel_xyz)
        w.add_frame(ts_ns, left_gray, right_gray)
    summary = w.summary          # the dict written to dataset.json

    reader = EurocReader(root)   # root or root/mav0
    report = reader.validate()   # ValidationReport(errors, warnings)
    for item in reader.merged():
        if item.kind == "imu":   tracker.register_imu_measurement(0, ...)
        else:                    tracker.track(item.ts_ns, [item.left, item.right])
"""

from __future__ import annotations

import atexit
import collections
import csv
import json
import math
import os
import platform
import queue
import re
import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, Iterator, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np

__all__ = [
    "FORMAT_NAME",
    "FORMAT_VERSION",
    "CAM_CSV_HEADER",
    "IMU_CSV_HEADER",
    "DISTORTION_MODELS",
    "PLACEHOLDER_IMU_NOISE",
    "DatasetError",
    "CameraInfo",
    "ImuInfo",
    "RigInfo",
    "ImuSample",
    "StereoFrame",
    "ValidationReport",
    "EurocWriter",
    "EurocReader",
    "yaml_float",
    "parse_simple_yaml",
    "write_camera_yaml",
    "write_imu_yaml",
    "load_gray_png",
    "matrix_to_quaternion_xyzw",
    "trim_dataset",
    "resolve_mav0",
    "resolve_png_workers",
]

FORMAT_NAME = "euroc-asl"
FORMAT_VERSION = 1

#: First line of ``cam*/data.csv`` (skipped unconditionally by every reader).
CAM_CSV_HEADER = "#timestamp [ns],filename"
#: First line of ``imu0/data.csv``: gyroscope first, accelerometer second (EuRoC order).
IMU_CSV_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                  "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")

#: ``sensor.yaml`` ``distortion_model`` -> (cuvslam ``Distortion.Model`` name, allowed coefficient
#: counts).  ``radial-tangential`` with 4 values is what NVIDIA's loader expects (Brown with k3 = 0
#: inserted); all-zero coefficients of any model mean Pinhole.  ``rational-polynomial`` (8 OpenCV
#: coefficients) is our raw-frame A/B variant which NVIDIA's loader would misread.  ``none`` takes
#: no coefficients (or four zeros, the EuRoC habit); any non-zero value with ``none`` is an error.
DISTORTION_MODELS: Dict[str, Tuple[str, Tuple[int, ...]]] = {
    "radial-tangential": ("Brown", (4, 5)),
    "equidistant": ("Fisheye", (4,)),
    "rational-polynomial": ("Polynomial", (8,)),
    "none": ("Pinhole", (0, 4)),
}

#: IMU noise model NVIDIA ships for a consumer MEMS IMU (examples/realsense/camera_utils.py at
#: v17.0.0).  Used until the BNO086 is measured; ``ImuInfo.noise_source`` says so.
PLACEHOLDER_IMU_NOISE: Dict[str, float] = {
    "gyroscope_noise_density": 6.0673370376614875e-03,      # rad / s / sqrt(Hz)
    "gyroscope_random_walk": 3.6211951458325785e-05,        # rad / s^2 / sqrt(Hz)
    "accelerometer_noise_density": 3.3621979208052800e-02,  # m / s^2 / sqrt(Hz)
    "accelerometer_random_walk": 9.8256589971851467e-04,    # m / s^3 / sqrt(Hz)
}
PLACEHOLDER_NOISE_SOURCE = ("placeholder: NVIDIA examples/realsense/camera_utils.py values; "
                            "not measured on this BNO086")

IMU_NOISE_KEYS = ("gyroscope_noise_density", "gyroscope_random_walk",
                  "accelerometer_noise_density", "accelerometer_random_walk")
#: Every key of ``dataset.json["drops"]`` (always present, always an int).  The first seven are
#: the format doc's; ``frames_out_of_order`` (pair rejected by :meth:`EurocWriter.add_frame` for a
#: non-increasing timestamp) and ``stereo_writer_failed`` (pair rejected after the PNG writer
#: thread hit a fatal ``data.csv`` error) are the writer's own additions.
DROP_COUNTERS = ("stereo_sequence_gaps", "stereo_unpaired", "stereo_sync_violations",
                 "stereo_write_queue_dropped", "imu_gaps", "imu_out_of_order", "imu_queue_dropped",
                 "frames_out_of_order", "stereo_writer_failed")

_NS_PER_S = 1_000_000_000


class DatasetError(ValueError):
    """A dataset on disk (or the data handed to the writer) violates the format."""


# --------------------------------------------------------------------------------------
# Sensor descriptions (plain numpy/float, no cuvslam types)
# --------------------------------------------------------------------------------------

def _as_transform(value: Any) -> np.ndarray:
    """Coerce a 4x4 / 16-element / None transform to a float64 (4, 4) array."""
    if value is None:
        return np.eye(4, dtype=np.float64)
    arr = np.asarray(value, dtype=np.float64)
    if arr.size != 16:
        raise DatasetError("transform must have 16 elements (4x4), got shape {}".format(arr.shape))
    arr = arr.reshape(4, 4)
    if not np.allclose(arr[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise DatasetError("transform last row must be [0, 0, 0, 1], got {}".format(arr[3].tolist()))
    return np.ascontiguousarray(arr)


@dataclass
class CameraInfo:
    """One camera of the rig as it appears in ``cam<i>/sensor.yaml`` (+ a few ``dataset.json`` facts).

    ``T_BS`` is body_from_sensor in metres (body = cam0); ``fu fv cu cv`` at the recorded
    ``width x height``; ``distortion_model`` is the EuRoC spelling (see :data:`DISTORTION_MODELS`).
    """

    width: int
    height: int
    fu: float
    fv: float
    cu: float
    cv: float
    T_BS: Any = None                                   # 4x4 body_from_sensor, metres
    distortion_model: str = "radial-tangential"
    distortion_coefficients: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    rate_hz: float = 30.0
    comment: str = ""
    socket: Optional[str] = None                       # "CAM_B" / "CAM_C"
    role: Optional[str] = None                         # "left" / "right"
    sensor: Optional[str] = None                       # "OV9282"
    image_type: str = "GRAY8"
    undistorted: bool = True
    borders: Tuple[int, int, int, int] = (0, 0, 0, 0)  # top, bottom, left, right (raw variant only)

    def __post_init__(self) -> None:
        self.width = int(self.width)
        self.height = int(self.height)
        if self.width <= 0 or self.height <= 0:
            raise DatasetError("camera resolution must be positive, got {}x{}".format(self.width, self.height))
        self.fu, self.fv, self.cu, self.cv = (float(self.fu), float(self.fv), float(self.cu), float(self.cv))
        self.T_BS = _as_transform(self.T_BS)
        self.distortion_coefficients = [float(c) for c in self.distortion_coefficients]
        self.rate_hz = float(self.rate_hz)
        self.borders = tuple(int(b) for b in self.borders)  # type: ignore[assignment]

    @property
    def resolution(self) -> Tuple[int, int]:
        """``(width, height)`` as written to ``sensor.yaml`` and as ``cuvslam.Camera.size``."""
        return (self.width, self.height)

    @property
    def shape(self) -> Tuple[int, int]:
        """numpy ``(height, width)`` of a frame."""
        return (self.height, self.width)

    @property
    def intrinsics(self) -> List[float]:
        """``[fu, fv, cu, cv]`` in EuRoC order."""
        return [self.fu, self.fv, self.cu, self.cv]

    @property
    def K(self) -> np.ndarray:
        """3x3 camera matrix."""
        return np.array([[self.fu, 0.0, self.cu], [0.0, self.fv, self.cv], [0.0, 0.0, 1.0]])

    def cuvslam_distortion(self) -> Tuple[str, List[float]]:
        """Map ``distortion_model`` + coefficients to ``(cuvslam Distortion.Model name, parameters)``.

        Rules (format doc section 12.3): ``radial-tangential`` 4 -> Brown ``[k1, k2, 0, p1, p2]``
        (identical to NVIDIA's loader), 5 -> Brown as given; all-zero -> Pinhole ``[]``;
        ``equidistant`` 4 -> Fisheye; ``rational-polynomial`` 8 -> Polynomial; ``none`` ->
        Pinhole ``[]`` (its coefficients, if any, must all be zero: cuvslam's Pinhole takes no
        parameters).  Anything else raises :class:`DatasetError`.
        """
        model = str(self.distortion_model).strip().lower()
        coeffs = list(self.distortion_coefficients)
        if model not in DISTORTION_MODELS:
            raise DatasetError("unknown distortion_model {!r} (known: {})".format(
                self.distortion_model, ", ".join(sorted(DISTORTION_MODELS))))
        target, counts = DISTORTION_MODELS[model]
        if len(coeffs) not in counts:
            raise DatasetError("distortion_model {!r} needs {} coefficients, got {}".format(
                model, " or ".join(str(c) for c in counts), len(coeffs)))
        if all(c == 0.0 for c in coeffs):
            return ("Pinhole", [])
        if target == "Pinhole":
            raise DatasetError("distortion_model 'none' with non-zero coefficients {}: Pinhole takes no "
                               "parameters (use radial-tangential/equidistant/rational-polynomial)".format(coeffs))
        if target == "Brown" and len(coeffs) == 4:
            return ("Brown", [coeffs[0], coeffs[1], 0.0, coeffs[2], coeffs[3]])
        return (target, coeffs)

    def nvidia_loader_note(self) -> Optional[str]:
        """Why PyCuVSLAM's ``examples/euroc/dataset_utils.py`` (v17.0.0) would misread this camera, or ``None``.

        That loader ignores ``distortion_model`` and always builds ``Brown`` from
        ``coeffs[:2] + [0] + coeffs[2:]``, which is only right for all-zero coefficients (Pinhole
        in effect) or ``radial-tangential`` with exactly 4 values.  Our writer records the note as
        a ``dataset.json`` warning and :meth:`EurocReader.validate` repeats it.
        """
        coeffs = list(self.distortion_coefficients)
        if all(c == 0.0 for c in coeffs):
            return None
        model = str(self.distortion_model).strip().lower()
        if model == "radial-tangential" and len(coeffs) == 4:
            return None
        if model == "radial-tangential":
            return ("radial-tangential with {} coefficients: NVIDIA's dataset_utils builds Brown from coeffs[:2] "
                    "+ [0] + coeffs[2:] ({} values, not a valid Brown set); only cuvslam_oak's reader loads it "
                    "correctly (Brown as given)".format(len(coeffs), len(coeffs) + 1))
        return ("distortion_model {} with {} coefficients: NVIDIA's dataset_utils ignores the model and would "
                "build a wrong Brown camera; track it with methods/cuvslam-vio/08_cuvslam_track.py".format(model, len(coeffs)))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": self.width, "height": self.height,
            "fu": self.fu, "fv": self.fv, "cu": self.cu, "cv": self.cv,
            "T_BS": self.T_BS.flatten().tolist(),
            "distortion_model": self.distortion_model,
            "distortion_coefficients": list(self.distortion_coefficients),
            "rate_hz": self.rate_hz, "comment": self.comment,
            "socket": self.socket, "role": self.role, "sensor": self.sensor,
            "image_type": self.image_type, "undistorted": self.undistorted,
            "borders": list(self.borders),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "CameraInfo":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class ImuInfo:
    """The IMU as it appears in ``imu0/sensor.yaml``: ``T_BS`` = body_from_imu (metres), rate, noise."""

    T_BS: Any = None
    rate_hz: float = 200.0
    gyroscope_noise_density: float = PLACEHOLDER_IMU_NOISE["gyroscope_noise_density"]
    gyroscope_random_walk: float = PLACEHOLDER_IMU_NOISE["gyroscope_random_walk"]
    accelerometer_noise_density: float = PLACEHOLDER_IMU_NOISE["accelerometer_noise_density"]
    accelerometer_random_walk: float = PLACEHOLDER_IMU_NOISE["accelerometer_random_walk"]
    comment: str = ""
    noise_source: str = PLACEHOLDER_NOISE_SOURCE
    sensors: List[str] = field(default_factory=lambda: ["ACCELEROMETER_RAW", "GYROSCOPE_RAW"])
    timestamp_source: str = "acceleroMeter.tsDevice"
    eeprom_noise: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        self.T_BS = _as_transform(self.T_BS)
        self.rate_hz = float(self.rate_hz)
        for key in IMU_NOISE_KEYS:
            value = float(getattr(self, key))
            if not (value > 0.0) or not math.isfinite(value):
                raise DatasetError("{} must be a positive finite float, got {!r}".format(key, value))
            setattr(self, key, value)

    def noise(self) -> Dict[str, float]:
        return {key: float(getattr(self, key)) for key in IMU_NOISE_KEYS}

    def to_dict(self) -> Dict[str, Any]:
        d = {"T_BS": self.T_BS.flatten().tolist(), "rate_hz": self.rate_hz, "comment": self.comment,
             "noise_source": self.noise_source, "sensors": list(self.sensors),
             "timestamp_source": self.timestamp_source, "eeprom_noise": self.eeprom_noise}
        d.update(self.noise())
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ImuInfo":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class RigInfo:
    """cam0 (left), cam1 (right) and the IMU, each with ``T_BS`` relative to body = cam0.

    ``rig_from_cam1()`` / ``rig_from_imu()`` give what a cuVSLAM rig needs (cam0 = identity),
    exactly as NVIDIA's loader computes them: ``inv(T_BS(cam0)) @ T_BS(x)``.
    """

    cam0: CameraInfo
    cam1: CameraInfo
    imu: ImuInfo
    device_id: str = ""
    body_frame: str = "cam0 (CAM_B optical frame, x right, y down, z forward)"

    def __post_init__(self) -> None:
        if isinstance(self.cam0, dict):
            self.cam0 = CameraInfo.from_dict(self.cam0)
        if isinstance(self.cam1, dict):
            self.cam1 = CameraInfo.from_dict(self.cam1)
        if isinstance(self.imu, dict):
            self.imu = ImuInfo.from_dict(self.imu)
        if self.cam0.resolution != self.cam1.resolution:
            raise DatasetError("cam0 and cam1 must share one resolution, got {} and {}".format(
                self.cam0.resolution, self.cam1.resolution))

    @property
    def cameras(self) -> List[CameraInfo]:
        return [self.cam0, self.cam1]

    def rig_from_cam1(self) -> np.ndarray:
        return np.linalg.inv(self.cam0.T_BS) @ self.cam1.T_BS

    def rig_from_imu(self) -> np.ndarray:
        return np.linalg.inv(self.cam0.T_BS) @ self.imu.T_BS

    def baseline_m(self) -> float:
        """Distance between the two optical centres (metres)."""
        return float(np.linalg.norm(self.rig_from_cam1()[:3, 3]))

    def to_dict(self) -> Dict[str, Any]:
        return {"cam0": self.cam0.to_dict(), "cam1": self.cam1.to_dict(), "imu": self.imu.to_dict(),
                "device_id": self.device_id, "body_frame": self.body_frame}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RigInfo":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


def matrix_to_quaternion_xyzw(R: Any) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion ``(x, y, z, w)`` (scipy ``as_quat()`` order), float64.

    Shepperd's method (numerically safe for every rotation); raises :class:`DatasetError` when
    *R* is not a proper rotation (orthonormal, det +1) to 1e-5.  ``w`` is made non-negative.
    """
    m = np.asarray(R, dtype=np.float64)
    if m.shape == (4, 4):
        m = m[:3, :3]
    if m.shape != (3, 3):
        raise DatasetError("rotation must be 3x3 (or 4x4), got shape {}".format(m.shape))
    if not np.allclose(m @ m.T, np.eye(3), atol=1e-5) or abs(np.linalg.det(m) - 1.0) > 1e-5:
        raise DatasetError("rotation block is not a proper rotation (orthonormal, det +1)")
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    if q[3] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


# --------------------------------------------------------------------------------------
# YAML: hand-written EuRoC style out, tiny YAML-1.1-aware reader in (no pyyaml dependency)
# --------------------------------------------------------------------------------------

def yaml_float(x: Any) -> str:
    """Format *x* so ``yaml.safe_load`` (YAML 1.1) reads it back as a float, never as a string.

    YAML 1.1 needs a dot in the mantissa and a signed exponent: ``1e-05`` and ``1.5e5`` load as
    strings, ``1.0e-05`` and ``1.5e+05`` as floats.  ``0`` becomes ``0.0``.  NaN/inf are refused
    (they are not representable in a sensor.yaml that cuvslam can use).
    """
    value = float(x)
    if not math.isfinite(value):
        raise DatasetError("cannot write non-finite float {!r} to YAML".format(value))
    text = repr(value)
    if "e" in text or "E" in text:
        mantissa, exponent = text.lower().split("e")
        if "." not in mantissa:
            mantissa += ".0"
        if exponent[0] not in "+-":
            exponent = "+" + exponent
        return "{}e{}".format(mantissa, exponent)
    if "." not in text:
        text += ".0"
    return text


def _yaml_float_list(values: Sequence[Any]) -> str:
    return "[" + ", ".join(yaml_float(v) for v in values) + "]"


def _yaml_transform_block(T: np.ndarray, indent: str = "  ") -> str:
    """``T_BS:`` mapping with ``cols``/``rows``/``data`` laid out as 4 rows (EuRoC style)."""
    m = _as_transform(T)
    rows = []
    for r in range(4):
        rows.append(", ".join(yaml_float(v) for v in m[r]))
    joiner = ",\n" + indent + "       "
    return ("T_BS:\n{i}cols: 4\n{i}rows: 4\n{i}data: [{rows}]\n".format(i=indent, rows=joiner.join(rows)))


def write_camera_yaml(path: Union[str, Path], cam: CameraInfo, rate_hz: Optional[float] = None,
                      header: str = "") -> Path:
    """Write ``cam<i>/sensor.yaml`` for *cam* in the exact EuRoC layout (format doc section 5.3)."""
    rate = float(cam.rate_hz if rate_hz is None else rate_hz)
    rate_text = str(int(rate)) if float(rate).is_integer() else yaml_float(rate)
    lines = _yaml_header_lines(header)
    lines += [
        "sensor_type: camera",
        "comment: " + _yaml_plain_string(cam.comment or "camera"),
        "",
        "# Sensor extrinsics wrt. the body frame (body = cam0 = left camera optical frame): body_from_sensor, metres.",
        _yaml_transform_block(cam.T_BS).rstrip("\n"),
        "",
        "# Camera specific definitions.",
        "rate_hz: " + rate_text,
        "resolution: [{}, {}]".format(int(cam.width), int(cam.height)),
        "camera_model: pinhole",
        "intrinsics: {} # fu, fv, cu, cv".format(_yaml_float_list(cam.intrinsics)),
        "distortion_model: " + _yaml_plain_string(str(cam.distortion_model) or "none"),
        "distortion_coefficients: {} # {}".format(
            _yaml_float_list(cam.distortion_coefficients),
            "k1, k2, p1, p2 - frames are pre-undistorted" if cam.undistorted
            else "OpenCV order (see datasets/dataset-format.md section 2.3)"),
    ]
    return _write_ascii_lines(path, lines)


def write_imu_yaml(path: Union[str, Path], imu: ImuInfo, rate_hz: Optional[float] = None,
                   header: str = "") -> Path:
    """Write ``imu0/sensor.yaml`` for *imu* (format doc section 7)."""
    rate = float(imu.rate_hz if rate_hz is None else rate_hz)
    rate_text = str(int(rate)) if float(rate).is_integer() else yaml_float(rate)
    lines = _yaml_header_lines(header)
    lines += [
        "sensor_type: imu",
        "comment: " + _yaml_plain_string(imu.comment or "imu"),
        "",
        "# Sensor extrinsics wrt. the body frame (body = cam0 = left camera optical frame): body_from_imu, metres.",
        _yaml_transform_block(imu.T_BS).rstrip("\n"),
        "rate_hz: " + rate_text,
        "",
        "# Inertial sensor noise model parameters (static). " + _yaml_comment(imu.noise_source),
        "gyroscope_noise_density: {}     # [ rad / s / sqrt(Hz) ]".format(yaml_float(imu.gyroscope_noise_density)),
        "gyroscope_random_walk: {}       # [ rad / s^2 / sqrt(Hz) ]".format(yaml_float(imu.gyroscope_random_walk)),
        "accelerometer_noise_density: {} # [ m / s^2 / sqrt(Hz) ]".format(yaml_float(imu.accelerometer_noise_density)),
        "accelerometer_random_walk: {}   # [ m / s^3 / sqrt(Hz) ]".format(yaml_float(imu.accelerometer_random_walk)),
    ]
    return _write_ascii_lines(path, lines)


def _one_line(text: Any) -> str:
    return " ".join(str(text).split())


def _yaml_comment(text: Any) -> str:
    """Free text made safe for the remainder of a ``# comment`` line: one line, ASCII only."""
    return _one_line(text).encode("ascii", "replace").decode("ascii")


def _yaml_header_lines(header: str) -> List[str]:
    """Turn a free-text *header* (may be several lines, may contain non-ASCII) into ``#`` lines."""
    lines: List[str] = []
    for line in (header or "").splitlines():
        line = _yaml_comment(line)
        lines.append("# " + line if line and not line.startswith("#") else (line or "#"))
    return lines


def _write_ascii_lines(path: Union[str, Path], lines: Sequence[str]) -> Path:
    """Write *lines* as an ASCII text file with ``\\n`` endings.

    Every free-text field has already been sanitised; ``errors="replace"`` is only a last-resort
    guard so a sensor.yaml write can never raise ``UnicodeEncodeError``.
    """
    out = Path(path)
    out.write_text("\n".join(lines) + "\n", encoding="ascii", errors="replace", newline="\n")
    return out


def _yaml_plain_string(text: str) -> str:
    """Make *text* a safe YAML plain scalar: one line, ASCII, no ``#``/``:``/quotes surprises."""
    clean = _yaml_comment(text)
    clean = clean.replace("#", "").replace(": ", " - ").replace('"', "'")
    if not clean or clean[0] in "[]{}&*!|>'%@`,?-":
        clean = "-" if not clean else clean
        return '"' + clean.replace("'", "").replace("\\", "/") + '"'
    return clean


# YAML 1.1 int (pyyaml 6 resolver, same alternatives in the same order): binary, octal (leading 0),
# decimal, hex, sexagesimal (``1:30`` = 90).
_INT_RE = re.compile(r"^(?:[-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(?:0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+"
                     r"|[-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+)$")
# YAML 1.1 float (pyyaml resolver): mantissa with a dot, optional exponent WITH sign; or sexagesimal
# with a fraction (``1:30.5`` = 90.5).  ``.inf``/``.nan`` are in _SPECIAL_FLOATS.
_FLOAT_RE = re.compile(r"^(?:[-+]?[0-9][0-9_]*\.[0-9_]*(?:[eE][-+][0-9]+)?|\.[0-9_]+(?:[eE][-+][0-9]+)?"
                       r"|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\.[0-9_]*)$")
_SPECIAL_FLOATS = {".inf": math.inf, "+.inf": math.inf, "-.inf": -math.inf, ".nan": math.nan,
                   ".Inf": math.inf, "+.Inf": math.inf, "-.Inf": -math.inf, ".NaN": math.nan,
                   ".INF": math.inf, "+.INF": math.inf, "-.INF": -math.inf, ".NAN": math.nan}
_BOOLS = {"true": True, "false": False, "yes": True, "no": False, "on": True, "off": False}


def _sexagesimal(parts: Sequence[str], convert: Any) -> Any:
    """``h:m:s`` digits -> one number, least significant part first (pyyaml's construct_yaml_int/float)."""
    value = 0
    base = 1
    for part in reversed(list(parts)):
        value += convert(part) * base
        base *= 60
    return value


def _yaml_int(text: str) -> int:
    """pyyaml's ``construct_yaml_int`` for a token that matched :data:`_INT_RE`."""
    value = text.replace("_", "")
    sign = -1 if value[0] == "-" else 1
    if value[0] in "+-":
        value = value[1:]
    if value == "0":
        return 0
    if value.startswith("0b"):
        return sign * int(value[2:], 2)
    if value.startswith("0x"):
        return sign * int(value[2:], 16)
    if value[0] == "0":
        return sign * int(value, 8)
    if ":" in value:
        return sign * _sexagesimal(value.split(":"), int)
    return sign * int(value)


def _yaml_float_value(text: str) -> float:
    """pyyaml's ``construct_yaml_float`` for a token that matched :data:`_FLOAT_RE`."""
    value = text.replace("_", "").lower()
    sign = -1.0 if value[0] == "-" else 1.0
    if value[0] in "+-":
        value = value[1:]
    if ":" in value:
        return sign * float(_sexagesimal(value.split(":"), float))
    return sign * float(value)


def _yaml_scalar(token: str) -> Any:
    """Resolve one YAML 1.1 plain scalar the way ``yaml.safe_load`` (pyyaml 6) does.

    Covers null (``~``/``null``/empty), the YAML 1.1 bools (``yes``/``no``/``on``/``off``/
    ``true``/``false`` in their three capitalisations), ints (decimal, ``0b``, leading-``0``
    octal, ``0x``, sexagesimal ``1:30``), floats (dot in the mantissa, signed exponent, ``.inf``,
    ``.nan``, sexagesimal), single/double-quoted strings (no escape processing) and otherwise the
    string itself, so ``1e-05`` stays a *string* exactly as ``safe_load`` sees it.
    """
    t = token.strip()
    if t == "" or t in ("~", "null", "Null", "NULL"):
        return None
    if len(t) >= 2 and t[0] == t[-1] and t[0] in "'\"":
        return t[1:-1]
    if t.lower() in _BOOLS and t in ("true", "false", "True", "False", "TRUE", "FALSE",
                                     "yes", "no", "Yes", "No", "YES", "NO", "on", "off", "On", "Off", "ON", "OFF"):
        return _BOOLS[t.lower()]
    if _INT_RE.match(t):
        return _yaml_int(t)
    if t in _SPECIAL_FLOATS:
        return _SPECIAL_FLOATS[t]
    if _FLOAT_RE.match(t):
        return _yaml_float_value(t)
    return t


def _strip_comment(line: str) -> str:
    """Drop a ``# comment`` (a ``#`` at line start or after whitespace), keep the rest."""
    if line.lstrip().startswith("#"):
        return ""
    pos = 0
    while True:
        idx = line.find("#", pos)
        if idx < 0:
            return line.rstrip()
        if idx == 0 or line[idx - 1] in " \t":
            return line[:idx].rstrip()
        pos = idx + 1


def _parse_flow_list(text: str) -> List[Any]:
    body = text.strip()
    if not (body.startswith("[") and body.endswith("]")):
        raise DatasetError("expected a [flow list], got {!r}".format(text))
    inner = body[1:-1].strip()
    if not inner:
        return []
    return [_yaml_scalar(item) for item in inner.split(",") if item.strip() != ""]


def parse_simple_yaml(text: str) -> Dict[str, Any]:
    """Parse the subset of YAML used by EuRoC ``sensor.yaml`` files into a dict.

    Supports: top-level ``key: scalar``, ``key: [flow, list]`` (the list may continue over
    several lines until the brackets balance), one level of nested block mapping
    (``T_BS:`` with indented ``cols/rows/data``), ``#`` comments, blank lines, ``%YAML``/``%TAG``
    directive lines (skipped), a ``---`` document start (skipped, content after it on the same
    line is parsed) and a ``...`` document end (parsing stops).  Scalars are resolved with YAML
    1.1 rules (see :func:`_yaml_scalar`), so ``1e-05`` becomes the *string* ``'1e-05'`` (the same
    bug ``yaml.safe_load`` would produce) and the caller can flag it.  Anything else raises
    :class:`DatasetError`.
    """
    result: Dict[str, Any] = {}
    current_block: Optional[Dict[str, Any]] = None
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        raw = lines[i]
        i += 1
        line = _strip_comment(raw.replace("\t", "    "))
        if not line.strip():
            continue
        stripped = line.strip()
        if stripped.startswith("%"):        # %YAML 1.1 / %TAG directives
            continue
        if stripped == "...":               # document end
            break
        if stripped == "---" or stripped.startswith("--- "):
            line = stripped[3:].strip()     # document start; anything after it is top level
            if not line:
                continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        if ":" not in stripped:
            raise DatasetError("YAML line {}: expected 'key: value', got {!r}".format(i, raw))
        key, _, value = stripped.partition(":")
        key = key.strip()
        value = value.strip()
        if value.startswith("[") and not value.endswith("]"):
            # flow list continued on following lines
            while i < len(lines):
                nxt = _strip_comment(lines[i])
                i += 1
                value += " " + nxt.strip()
                if value.rstrip().endswith("]"):
                    break
            if not value.rstrip().endswith("]"):
                raise DatasetError("YAML: unterminated flow list for key {!r}".format(key))
        if indent == 0:
            current_block = None
            target = result
        else:
            if current_block is None:
                raise DatasetError("YAML line {}: unexpected indentation".format(i))
            target = current_block
        if value == "":
            if indent != 0:
                raise DatasetError("YAML line {}: nested mappings deeper than one level are not supported".format(i))
            block: Dict[str, Any] = {}
            result[key] = block
            current_block = block
            continue
        if value.startswith("["):
            target[key] = _parse_flow_list(value)
        else:
            target[key] = _yaml_scalar(value)
    return result


# --------------------------------------------------------------------------------------
# Small file helpers
# --------------------------------------------------------------------------------------

def resolve_mav0(root: Union[str, Path]) -> Tuple[Path, Path]:
    """Return ``(recording_root, mav0)`` for a path that is either the root or ``mav0`` itself."""
    p = Path(root).expanduser().resolve()
    if not p.exists():
        raise DatasetError("dataset path does not exist: {}".format(p))
    if p.name == "mav0" and p.is_dir():
        return p.parent, p
    if (p / "mav0").is_dir():
        return p, p / "mav0"
    if (p / "cam0").is_dir():  # a bare EuRoC sequence directory not named mav0
        return p.parent, p
    raise DatasetError("{} contains no mav0/ (nor cam0/): not an EuRoC ASL dataset".format(p))


def load_gray_png(path: Union[str, Path], expected_shape: Optional[Tuple[int, int]] = None) -> np.ndarray:
    """Read an 8-bit single-channel PNG as a C-contiguous ``(height, width)`` uint8 array.

    Uses ``IMREAD_UNCHANGED`` (a colour or 16-bit PNG is an error, not silently converted) and
    decodes from bytes so non-ASCII paths work on every platform.
    """
    p = Path(path)
    try:
        data = np.frombuffer(p.read_bytes(), dtype=np.uint8)
    except OSError as exc:
        raise DatasetError("cannot read image {}: {}".format(p, exc)) from exc
    img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise DatasetError("cannot decode image {}".format(p))
    if img.dtype != np.uint8 or img.ndim != 2:
        raise DatasetError("{}: expected 8-bit single-channel PNG, got dtype {} shape {}".format(
            p.name, img.dtype, img.shape))
    if expected_shape is not None and tuple(img.shape) != tuple(expected_shape):
        raise DatasetError("{}: expected shape {} (height, width), got {}".format(
            p.name, tuple(expected_shape), img.shape))
    return np.ascontiguousarray(img)


def _encode_png(img: np.ndarray, compression: int) -> bytes:
    ok, buf = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, int(compression)])
    if not ok:
        raise DatasetError("cv2.imencode failed for a {} {} image".format(img.dtype, img.shape))
    return buf.tobytes()


def _read_csv_rows(path: Path, min_fields: int) -> List[List[str]]:
    """EuRoC csv: skip line 1 unconditionally, skip blank rows, require ``min_fields`` per row."""
    rows: List[List[str]] = []
    with open(path, "r", encoding="utf-8", newline="") as fh:
        next(fh, None)
        for row in csv.reader(fh):
            if not row or all(cell.strip() == "" for cell in row):
                continue
            if len(row) < min_fields:
                raise DatasetError("{}: row {!r} has {} fields, need {}".format(path, row, len(row), min_fields))
            rows.append([cell.strip() for cell in row])
    return rows


def _fmt_ts(ts_ns: int) -> str:
    return str(int(ts_ns))


def _fmt_val(v: float) -> str:
    return format(float(v), ".10g")


def trim_dataset(mav0: Union[str, Path], remove_dangling_pngs: bool = True) -> Dict[str, int]:
    """Make an interrupted recording loadable: keep only pairs whose two PNGs exist.

    Rewrites ``cam0/data.csv`` and ``cam1/data.csv`` with the rows (paired by index) for which
    both ``cam0/data/<f>`` and ``cam1/data/<f>`` exist, optionally deletes PNGs that no row
    references, and returns ``{"rows_kept", "rows_removed", "pngs_removed"}``.  Rows are
    written in their original order; the two csv files end up with equal length.
    """
    _, m = resolve_mav0(mav0)
    csv0, csv1 = m / "cam0" / "data.csv", m / "cam1" / "data.csv"
    rows0 = _read_csv_rows(csv0, 2) if csv0.exists() else []
    rows1 = _read_csv_rows(csv1, 2) if csv1.exists() else []
    n = min(len(rows0), len(rows1))
    kept0: List[List[str]] = []
    kept1: List[List[str]] = []
    for r0, r1 in zip(rows0[:n], rows1[:n]):
        if (m / "cam0" / "data" / r0[1]).is_file() and (m / "cam1" / "data" / r1[1]).is_file():
            kept0.append(r0)
            kept1.append(r1)
    removed = len(rows0) + len(rows1) - 2 * len(kept0)
    for path, rows in ((csv0, kept0), (csv1, kept1)):
        with open(path, "w", encoding="ascii", newline="\n") as fh:
            fh.write(CAM_CSV_HEADER + "\n")
            for r in rows:
                fh.write("{},{}\n".format(r[0], r[1]))
    pngs_removed = 0
    if remove_dangling_pngs:
        for cam, rows in (("cam0", kept0), ("cam1", kept1)):
            wanted = {r[1] for r in rows}
            data_dir = m / cam / "data"
            if data_dir.is_dir():
                for png in data_dir.glob("*.png"):
                    if png.name not in wanted:
                        png.unlink()
                        pngs_removed += 1
    return {"rows_kept": len(kept0), "rows_removed": removed, "pngs_removed": pngs_removed}


# --------------------------------------------------------------------------------------
# Writer
# --------------------------------------------------------------------------------------

#: Upper bound of the automatic PNG encoder thread count (``png_workers=0``).
MAX_AUTO_PNG_WORKERS = 4


def resolve_png_workers(requested: int = 0) -> int:
    """Effective PNG encoder thread count for :class:`EurocWriter`'s *png_workers*.

    ``0`` (auto) gives ``min(4, max(1, os.cpu_count() - 2))``: up to four encoder threads while
    leaving two cores to the recorder's own threads (undistortion, DepthAI queues); any other
    value is used as given, clamped to at least 1.
    """
    n = int(requested)
    if n > 0:
        return n
    cpus = os.cpu_count() or 1
    return min(MAX_AUTO_PNG_WORKERS, max(1, cpus - 2))


def _deep_merge(base: Dict[str, Any], extra: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Recursively merge *extra* into *base* (dicts merge, other values replace) and return base."""
    if not extra:
        return base
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def _iso_local_now() -> str:
    """Local wall time with UTC offset, ISO 8601 (``2026-09-19T20:31:07-04:00``)."""
    t = time.localtime()
    off = t.tm_gmtoff
    sign = "+" if off >= 0 else "-"
    off = abs(int(off))
    return time.strftime("%Y-%m-%dT%H:%M:%S", t) + "{}{:02d}:{:02d}".format(sign, off // 3600, (off % 3600) // 60)


class EurocWriter:
    """Write one EuRoC ASL recording (``<root>/mav0/...`` + ``dataset.json`` + ``README.txt``).

    Construction creates the directory tree and the three ``sensor.yaml`` files from *rig_info*
    (``rate_hz`` in the yaml comes from *cam_rate_hz* / *imu_rate_hz*).  Then:

    * :meth:`add_frame` queues a stereo pair for the background PNG pipeline: a pool of
      *png_workers* encoder threads (``cv2.imencode`` releases the GIL, so they scale) and one
      *committer* thread that takes the pairs out of the queue in order, hands them to the pool
      and, strictly in queue order, writes the two PNGs and only then the ``data.csv`` rows.
      The queue is bounded (*queue_size* pairs); when it is full the pair is dropped, counted in
      ``drops.stereo_write_queue_dropped`` and ``add_frame`` returns ``False``.  At most
      ``2 * png_workers`` pairs are in flight in the pool on top of the queue, so memory stays
      bounded.  ``png_workers=0`` (default) means :func:`resolve_png_workers`:
      ``min(4, max(1, cpu_count - 2))``; ``1`` is the historical single-thread behaviour.
    * :meth:`add_imu` appends one ``imu0/data.csv`` row (gyro then accel) in the calling thread.
    * :meth:`close` (or leaving the ``with`` block) drains the queue and every pair already in
      the pool, trims dangling rows/PNGs, writes ``dataset.json`` (section 12.1 of the format
      doc, unknown values ``null``), ``README.txt`` and, when *calibration* was given,
      ``calibration.json``; it returns the summary dict, also available as :attr:`summary`.

    Timestamps are integer nanoseconds on one clock; an out-of-order frame or a left/right skew
    above *max_stereo_skew_ns* (default 1 ms) is dropped and counted, an out-of-order IMU sample
    likewise (cuVSLAM rejects both).  Counters the writer cannot know (sequence gaps from
    ``getSequenceNum()``, unpaired frames, IMU queue drops) are reported by the recorder through
    :meth:`count_drop`; free-text notes through :meth:`add_warning`; any ``dataset.json`` field
    through *metadata* (constructor) or ``close(metadata=...)`` (deep-merged, recorder wins).

    ``dataset.json`` records the pipeline: ``record.png_workers`` (effective thread count),
    ``measured.png_write_ms_mean`` (wall time per pair on the committer path: waiting for its
    encode to finish, writing both PNGs and the csv rows), ``measured.png_encode_ms_mean`` (mean
    per-pair encode time measured inside the pool) and ``measured.png_throughput_pairs_per_s``
    (pairs written per second of committer *busy* time, i.e. excluding the time it waited for
    input; the sustainable rate of this pipeline on this disk).  :meth:`stats` gives the same
    numbers live.

    Robustness: the committer thread never dies.  A pair that fails to encode/write is counted
    (``counts.frames_failed``, a ``dataset.json`` warning) and recording continues; a failure
    while appending the ``data.csv`` rows (disk full, locked or closed handle) is fatal for the
    stereo stream: from then on :meth:`add_frame` returns ``False`` and counts the pair in
    ``drops.stereo_writer_failed``, :attr:`writer_error` says why, and :meth:`close` still
    returns normally.  The constructor refuses a rig the reader could not load (unknown
    distortion model / coefficient count) before creating any directory and records a warning
    when NVIDIA's ``dataset_utils`` would misread the coefficients.  An ``atexit`` hook closes a
    writer whose owner forgot to (``stopped_by = "atexit"``); it keeps the writer alive until
    :meth:`close` unregisters it (when the interpreter has already shut the pool down by then,
    the committer encodes the remaining pairs itself).
    """

    def __init__(self, root: Union[str, Path], rig_info: RigInfo, cam_rate_hz: float = 30.0,
                 imu_rate_hz: float = 200.0, *, queue_size: int = 64, png_compression: int = 1,
                 max_stereo_skew_ns: int = 1_000_000, calibration: Optional[Dict[str, Any]] = None,
                 metadata: Optional[Dict[str, Any]] = None, imu_flush_every: int = 200,
                 png_workers: int = 0) -> None:
        if not isinstance(rig_info, RigInfo):
            rig_info = RigInfo.from_dict(rig_info) if isinstance(rig_info, dict) else rig_info
        if not isinstance(rig_info, RigInfo):
            raise TypeError("rig_info must be a RigInfo (or its to_dict() form)")
        self.root = Path(root).expanduser().resolve()
        self.mav0 = self.root / "mav0"
        self.rig = rig_info
        self.cam_rate_hz = float(cam_rate_hz)
        self.imu_rate_hz = float(imu_rate_hz)
        if self.cam_rate_hz <= 0 or self.imu_rate_hz <= 0:
            raise DatasetError("cam_rate_hz and imu_rate_hz must be positive")
        self.queue_size = max(1, int(queue_size))
        self.png_compression = min(9, max(0, int(png_compression)))
        self.png_workers = resolve_png_workers(png_workers)
        self._max_inflight = 2 * self.png_workers
        self.max_stereo_skew_ns = int(max_stereo_skew_ns)
        self.calibration = calibration
        self._metadata: Dict[str, Any] = dict(metadata or {})
        self._imu_flush_every = max(1, int(imu_flush_every))
        self.summary: Optional[Dict[str, Any]] = None
        self._warnings: List[str] = []
        self._stopped_by = "close"
        self._pairs_kept = 0

        # refuse a rig our own reader could not load, before touching the disk
        for name, cam in (("cam0", self.rig.cam0), ("cam1", self.rig.cam1)):
            try:
                cam.cuvslam_distortion()
            except DatasetError as exc:
                raise DatasetError("{}: {}".format(name, exc)) from exc
            note = cam.nvidia_loader_note()
            if note:
                self._warnings.append("{}: {}".format(name, note))

        self._cam0_dir = self.mav0 / "cam0"
        self._cam1_dir = self.mav0 / "cam1"
        self._imu_dir = self.mav0 / "imu0"
        for d in (self._cam0_dir / "data", self._cam1_dir / "data", self._imu_dir):
            d.mkdir(parents=True, exist_ok=True)
        if (self._cam0_dir / "data.csv").exists() or (self._imu_dir / "data.csv").exists():
            raise DatasetError("{} already holds a recording (data.csv exists); refusing to append".format(self.mav0))
        if (self._cam0_dir / "sensor_recalibrated.yaml").exists():
            raise DatasetError("cam0/sensor_recalibrated.yaml exists: NVIDIA's loader would switch to Fisheye; remove it")

        header = "Written by cuvslam_oak.dataset.EurocWriter (datasets/dataset-format.md)."
        write_camera_yaml(self._cam0_dir / "sensor.yaml", self.rig.cam0, self.cam_rate_hz, header)
        write_camera_yaml(self._cam1_dir / "sensor.yaml", self.rig.cam1, self.cam_rate_hz, header)
        write_imu_yaml(self._imu_dir / "sensor.yaml", self.rig.imu, self.imu_rate_hz, header)

        self._shape = self.rig.cam0.shape
        self._frame_period_ns = int(round(_NS_PER_S / self.cam_rate_hz))
        self._imu_period_ns = int(round(_NS_PER_S / self.imu_rate_hz))

        self._lock = threading.Lock()
        self._closed = False
        self._start_wall = _iso_local_now()
        self._start_unix_ns = time.time_ns()
        self._t0 = time.monotonic()

        # stereo bookkeeping (caller thread)
        self._last_frame_ts: Optional[int] = None
        self._first_frame_ts: Optional[int] = None
        self._frames_accepted = 0
        self._max_frame_gap_ns = 0
        self._frame_gap_count = 0
        self._max_lr_skew_ns = 0
        # committer / encoder results (under self._lock)
        self._frames_written = 0
        self._frames_failed = 0
        self._png_bytes = 0
        self._png_time_s = 0.0          # committer wall time per written pair (wait + write + csv)
        self._png_encode_s = 0.0        # encode time per written pair, measured inside the pool
        self._committer_started: Optional[float] = None   # perf_counter() when the committer thread began
        self._committer_stopped: Optional[float] = None   # ... and when it returned
        self._idle_s = 0.0              # committer time spent waiting for input (nothing queued or in flight)
        self._idle_since: Optional[float] = None
        self._worker_errors: List[str] = []
        # IMU bookkeeping (caller thread)
        self._last_imu_ts: Optional[int] = None
        self._first_imu_ts: Optional[int] = None
        self._imu_written = 0
        self._imu_unflushed = 0
        self._max_imu_gap_ns = 0
        self._max_imu_skew_ns = 0
        self._drops: Dict[str, int] = {name: 0 for name in DROP_COUNTERS}
        self._writer_error: Optional[str] = None   # set by the worker on a fatal data.csv failure

        handles = []
        try:
            for path, header_line in ((self._cam0_dir / "data.csv", CAM_CSV_HEADER),
                                      (self._cam1_dir / "data.csv", CAM_CSV_HEADER),
                                      (self._imu_dir / "data.csv", IMU_CSV_HEADER)):
                fh = open(path, "w", encoding="ascii", newline="\n")
                handles.append(fh)
                fh.write(header_line + "\n")
                fh.flush()
        except OSError:
            for fh in handles:
                fh.close()
            raise
        self._cam0_csv, self._cam1_csv, self._imu_csv = handles

        self._queue: "queue.Queue[Optional[Tuple[int, np.ndarray, np.ndarray]]]" = queue.Queue(maxsize=self.queue_size)
        # set by add_frame (new input), by a finished encode job and by close (sentinel); the
        # committer waits on it instead of blocking on one of the two sources at a time
        self._wake = threading.Event()
        self._pool = ThreadPoolExecutor(max_workers=self.png_workers, thread_name_prefix="euroc-png-encode")
        self._worker = threading.Thread(target=self._run_worker, name="euroc-png-writer", daemon=True)
        self._worker.start()
        # a forgotten close() would otherwise kill the daemon worker mid-write and leave no dataset.json
        atexit.register(self._close_at_exit)

    # ---------------------------------------------------------------- context manager
    def __enter__(self) -> "EurocWriter":
        return self

    def __exit__(self, exc_type, _exc, _tb) -> None:
        self.close(stopped_by="exception" if exc_type is not None else "close")

    # ---------------------------------------------------------------- input
    def add_frame(self, ts_ns: int, left: np.ndarray, right: np.ndarray, *,
                  right_ts_ns: Optional[int] = None) -> bool:
        """Queue one stereo pair (uint8 ``(height, width)`` arrays) stamped *ts_ns*.

        Returns ``True`` when queued.  ``False`` (and a counter) when the timestamp does not
        increase, ``|ts_ns - right_ts_ns|`` exceeds the allowed skew, the writer queue is full,
        or the writer thread has reported a fatal ``data.csv`` error (:attr:`writer_error`,
        counted in ``drops.stereo_writer_failed``).  Raises :class:`DatasetError` for a wrong
        dtype/shape and ``RuntimeError`` after :meth:`close`.  The arrays are copied, the caller
        may reuse its buffers.
        """
        if self._closed:
            raise RuntimeError("EurocWriter is closed")
        ts = int(ts_ns)
        for name, img in (("left", left), ("right", right)):
            if not isinstance(img, np.ndarray) or img.dtype != np.uint8 or img.ndim != 2:
                raise DatasetError("{} frame must be a 2-D uint8 array, got {}".format(
                    name, getattr(img, "dtype", type(img)) if isinstance(img, np.ndarray) else type(img)))
            if tuple(img.shape) != self._shape:
                raise DatasetError("{} frame shape {} != (height, width) {} from the rig".format(
                    name, img.shape, self._shape))
        if self._writer_error is not None:
            self._drops["stereo_writer_failed"] += 1
            return False
        if self._last_frame_ts is not None and ts <= self._last_frame_ts:
            self._drops["frames_out_of_order"] += 1
            return False
        if right_ts_ns is not None:
            skew = abs(int(right_ts_ns) - ts)
            self._max_lr_skew_ns = max(self._max_lr_skew_ns, skew)
            if skew > self.max_stereo_skew_ns:
                self._drops["stereo_sync_violations"] += 1
                return False
        item = (ts, np.ascontiguousarray(left).copy(), np.ascontiguousarray(right).copy())
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._drops["stereo_write_queue_dropped"] += 1
            return False
        self._wake.set()
        if self._last_frame_ts is not None:
            gap = ts - self._last_frame_ts
            self._max_frame_gap_ns = max(self._max_frame_gap_ns, gap)
            if gap > 1.5 * self._frame_period_ns:
                self._frame_gap_count += 1
        else:
            self._first_frame_ts = ts
        self._last_frame_ts = ts
        self._frames_accepted += 1
        return True

    def add_imu(self, ts_ns: int, gyro: Sequence[float], accel: Sequence[float], *,
                gyro_ts_ns: Optional[int] = None) -> bool:
        """Append one IMU row: *gyro* (rad/s, x y z) then *accel* (m/s^2, x y z, specific force).

        Returns ``False`` (counted in ``drops.imu_out_of_order``) when *ts_ns* is older than the
        previous sample; raises :class:`DatasetError` for non-finite or mis-sized values.
        """
        if self._closed:
            raise RuntimeError("EurocWriter is closed")
        ts = int(ts_ns)
        g = [float(v) for v in gyro]
        a = [float(v) for v in accel]
        if len(g) != 3 or len(a) != 3 or not all(math.isfinite(v) for v in g + a):
            raise DatasetError("gyro and accel must each be 3 finite floats, got {} / {}".format(gyro, accel))
        if self._last_imu_ts is not None and ts < self._last_imu_ts:
            self._drops["imu_out_of_order"] += 1
            return False
        if gyro_ts_ns is not None:
            self._max_imu_skew_ns = max(self._max_imu_skew_ns, abs(int(gyro_ts_ns) - ts))
        if self._last_imu_ts is not None:
            gap = ts - self._last_imu_ts
            self._max_imu_gap_ns = max(self._max_imu_gap_ns, gap)
            if gap > 2.5 * self._imu_period_ns:
                self._drops["imu_gaps"] += 1
        else:
            self._first_imu_ts = ts
        self._imu_csv.write("{},{},{},{},{},{},{}\n".format(
            _fmt_ts(ts), _fmt_val(g[0]), _fmt_val(g[1]), _fmt_val(g[2]),
            _fmt_val(a[0]), _fmt_val(a[1]), _fmt_val(a[2])))
        self._last_imu_ts = ts
        self._imu_written += 1
        self._imu_unflushed += 1
        if self._imu_unflushed >= self._imu_flush_every:
            self._imu_csv.flush()
            self._imu_unflushed = 0
        return True

    def count_drop(self, name: str, n: int = 1) -> int:
        """Add *n* to the ``dataset.json.drops`` counter *name* (recorder-side counters); return it."""
        self._drops[name] = self._drops.get(name, 0) + int(n)
        return self._drops[name]

    def add_warning(self, text: str) -> None:
        """Append a free-text note to ``dataset.json.warnings``."""
        self._warnings.append(_one_line(text))

    # ---------------------------------------------------------------- PNG pipeline
    def _run_worker(self) -> None:
        """Ordered committer thread.  It never dies: every failure is counted and reported at close.

        Takes pairs out of the input queue in order, submits each to the encoder pool (at most
        ``2 * png_workers`` in flight) and commits the oldest pair as soon as its encode is done:
        cam0 PNG, cam1 PNG, then the two csv rows.  A pair is never committed before the pairs
        queued ahead of it, so ``data.csv`` order equals :meth:`add_frame` order (strictly
        increasing).  When the pool is at its bound (or the sentinel has arrived) the thread
        blocks on the oldest encode; otherwise it waits on :attr:`_wake` for either new input or
        a finished encode, whichever comes first.  Time spent waiting for input with nothing in
        flight is *idle*; everything else is *busy* (see ``png_throughput_pairs_per_s``).
        """
        cam0_data = self._cam0_dir / "data"
        cam1_data = self._cam1_dir / "data"
        pending: Deque[Tuple[int, "Future[Tuple[bytes, bytes, float]]"]] = collections.deque()
        stopping = False
        last_ts = -1
        with self._lock:
            self._committer_started = time.perf_counter()
        while True:
            try:
                progressed = False
                # 1. top up the pool from the queue, never beyond the in-flight bound
                while not stopping and len(pending) < self._max_inflight:
                    try:
                        item = self._queue.get_nowait()
                    except queue.Empty:
                        break
                    if item is None:
                        stopping = True
                        break
                    last_ts = item[0]
                    pending.append((item[0], self._submit_encode(item[0], item[1], item[2])))
                    progressed = True
                # 2. commit the oldest pair when it is ready, or when nothing else can happen
                #    anyway (pool full / draining): then blocking on it is the back-pressure
                if pending and (pending[0][1].done() or len(pending) >= self._max_inflight or stopping):
                    ts, fut = pending.popleft()
                    last_ts = ts
                    self._commit_pair(cam0_data, cam1_data, ts, fut)
                    progressed = True
                if progressed:
                    continue
                if stopping:
                    break                              # sentinel seen and nothing left in flight
                # 3. nothing to do right now: wait for new input or a finished encode
                self._wake.clear()
                if not self._queue.empty() or (pending and pending[0][1].done()):
                    continue                           # arrived between the check and the clear
                idle = not pending
                if idle:
                    with self._lock:
                        self._idle_since = time.perf_counter()
                self._wake.wait(timeout=0.1)           # the timeout is only a safety net
                if idle:
                    with self._lock:
                        self._idle_s += time.perf_counter() - (self._idle_since or 0.0)
                        self._idle_since = None
            except Exception as exc:  # noqa: BLE001 - a dead committer would hang close(); count instead
                self._note_worker_failure(last_ts, exc, fatal=True)
        with self._lock:
            self._committer_stopped = time.perf_counter()

    def _submit_encode(self, ts: int, left: np.ndarray, right: np.ndarray) -> "Future[Tuple[bytes, bytes, float]]":
        """Hand one pair to the encoder pool; encode inline when the pool is already shut down."""
        try:
            fut = self._pool.submit(self._encode_job, ts, left, right)
        except RuntimeError:
            # the interpreter is shutting down (atexit close) or the pool is gone: same result,
            # same ordering, just on this thread
            fut: "Future[Tuple[bytes, bytes, float]]" = Future()
            try:
                fut.set_result(self._encode_job(ts, left, right))
            except Exception as exc:  # noqa: BLE001 - surfaces from fut.result() in _commit_pair
                fut.set_exception(exc)
            return fut
        fut.add_done_callback(self._on_encoded)
        return fut

    def _on_encoded(self, _fut: "Future[Tuple[bytes, bytes, float]]") -> None:
        self._wake.set()

    def _encode_job(self, ts: int, left: np.ndarray, right: np.ndarray) -> Tuple[bytes, bytes, float]:
        """Pool job: encode both images of pair *ts*; returns ``(cam0_png, cam1_png, seconds)``.

        *ts* is not needed for the encode itself; it is part of the signature so that a wrapper
        (tests, diagnostics) can single out one pair.
        """
        t0 = time.perf_counter()
        b0 = _encode_png(left, self.png_compression)
        b1 = _encode_png(right, self.png_compression)
        return b0, b1, time.perf_counter() - t0

    def _commit_pair(self, cam0_data: Path, cam1_data: Path, ts: int,
                     fut: "Future[Tuple[bytes, bytes, float]]") -> None:
        """Write pair *ts* from its finished (or awaited) encode job: PNGs first, csv rows last."""
        name = "{}.png".format(ts)
        p0, p1 = cam0_data / name, cam1_data / name
        t0 = time.perf_counter()
        stage = "png"
        try:
            b0, b1, encode_s = fut.result()          # re-raises the encode exception, if any
            p0.write_bytes(b0)
            p1.write_bytes(b1)
            # rows only after BOTH PNGs exist (an interrupted recording stays loadable)
            stage = "csv"
            self._cam0_csv.write("{},{}\n".format(ts, name))
            self._cam1_csv.write("{},{}\n".format(ts, name))
            self._cam0_csv.flush()
            self._cam1_csv.flush()
        except Exception as exc:  # noqa: BLE001 - keep recording, report at close
            for p in (p0, p1):
                try:
                    if p.exists():
                        p.unlink()
                except OSError:
                    pass
            # a csv row that cannot be appended means no later pair can be indexed either
            self._note_worker_failure(ts, exc, fatal=(stage == "csv"))
            return
        dt = time.perf_counter() - t0
        with self._lock:
            self._frames_written += 1
            self._png_bytes += len(b0) + len(b1)
            self._png_time_s += dt
            self._png_encode_s += encode_s

    def _busy_seconds_locked(self, now: float) -> float:
        """Committer busy time so far (caller holds :attr:`_lock`)."""
        if self._committer_started is None:
            return 0.0
        end = now if self._committer_stopped is None else min(now, self._committer_stopped)
        idle = self._idle_s
        if self._idle_since is not None and self._committer_stopped is None:
            idle += max(0.0, now - self._idle_since)
        return max(0.0, (end - self._committer_started) - idle)

    def _note_worker_failure(self, ts: int, exc: BaseException, fatal: bool) -> None:
        msg = "frame {}: {}: {}".format(ts, type(exc).__name__, _one_line(exc) or "(no message)")
        with self._lock:
            self._frames_failed += 1
            if len(self._worker_errors) < 10:
                self._worker_errors.append(msg)
            if fatal and self._writer_error is None:
                self._writer_error = msg

    # ---------------------------------------------------------------- state
    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def frames_accepted(self) -> int:
        """Pairs accepted into the queue so far (written + still queued)."""
        return self._frames_accepted

    @property
    def frames_written(self) -> int:
        with self._lock:
            return self._frames_written

    @property
    def frames_dropped(self) -> int:
        """Pairs dropped because the writer queue was full."""
        return self._drops["stereo_write_queue_dropped"]

    @property
    def imu_written(self) -> int:
        return self._imu_written

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    @property
    def drops(self) -> Dict[str, int]:
        return dict(self._drops)

    @property
    def writer_error(self) -> Optional[str]:
        """``None`` while the PNG writer thread is healthy; otherwise the fatal ``data.csv`` error."""
        with self._lock:
            return self._writer_error

    def stats(self) -> Dict[str, Any]:
        """Live snapshot: counts, queue depth, drops, measured rates (safe from any thread).

        ``png_write_ms_mean`` is the committer's wall time per written pair, ``png_encode_ms_mean``
        the encode time per pair inside the pool, ``png_throughput_pairs_per_s`` the pairs written
        per second of committer busy time (idle waits for input excluded).
        """
        with self._lock:
            written, failed, png_bytes, png_time, encode_time, writer_error = (
                self._frames_written, self._frames_failed, self._png_bytes, self._png_time_s,
                self._png_encode_s, self._writer_error)
            busy_s = self._busy_seconds_locked(time.perf_counter())
        return {
            "frames_accepted": self._frames_accepted, "frames_written": written, "frames_failed": failed,
            "imu_written": self._imu_written, "queue_depth": self._queue.qsize(), "queue_size": self.queue_size,
            "png_workers": self.png_workers, "drops": dict(self._drops), "png_bytes_total": png_bytes,
            "png_write_ms_mean": (1000.0 * png_time / written) if written else 0.0,
            "png_encode_ms_mean": (1000.0 * encode_time / written) if written else 0.0,
            "png_throughput_pairs_per_s": (written / busy_s) if (written and busy_s > 0.0) else 0.0,
            "png_busy_s": busy_s,
            "elapsed_s": time.monotonic() - self._t0, "writer_error": writer_error,
        }

    # ---------------------------------------------------------------- close
    def close(self, *, stopped_by: str = "close", metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Drain the queue, trim, write ``dataset.json`` / ``README.txt`` / ``calibration.json``.

        Idempotent: a second call returns the same summary; if it passes *metadata*, that is
        deep-merged and ``dataset.json`` / ``README.txt`` are rewritten so the files and the
        returned dict agree.  *stopped_by* (``"duration"``, ``"user"``, ``"exception"``...) and
        *metadata* land in ``dataset.json``.  Never hangs and never raises for a failure inside
        the writer thread (those become ``dataset.json`` warnings); an ``OSError`` writing the
        final ``dataset.json`` / ``README.txt`` themselves does propagate, after the worker has
        been joined and the csv handles closed.
        """
        if self._closed:
            if metadata:
                _deep_merge(self._metadata, metadata)
                self.summary = self._build_summary(self._stopped_by, self._pairs_kept)
                self._write_summary_files()
            return self.summary if self.summary is not None else {}
        self._closed = True
        atexit.unregister(self._close_at_exit)
        if metadata:
            _deep_merge(self._metadata, metadata)
        self._stop_worker()
        for fh, label in ((self._cam0_csv, "cam0/data.csv"), (self._cam1_csv, "cam1/data.csv"),
                          (self._imu_csv, "imu0/data.csv")):
            try:
                fh.flush()
            except OSError as exc:
                self._warnings.append("final flush of {} failed: {}: {}".format(label, type(exc).__name__, _one_line(exc)))
            except ValueError:
                pass  # handle already closed: reported by the worker (or closed by the caller)
            try:
                fh.close()
            except (OSError, ValueError):
                pass
        try:
            trim = trim_dataset(self.mav0)
        except (OSError, DatasetError) as exc:
            with self._lock:
                written = self._frames_written
            trim = {"rows_kept": written, "rows_removed": 0, "pngs_removed": 0}
            self._warnings.append("trim at close failed ({}: {}); csv files left as written".format(
                type(exc).__name__, _one_line(exc)))
        if trim["rows_removed"] or trim["pngs_removed"]:
            self._warnings.append("trim at close removed {} csv rows and {} dangling PNGs".format(
                trim["rows_removed"], trim["pngs_removed"]))
        with self._lock:
            worker_errors, frames_failed, writer_error = (list(self._worker_errors), self._frames_failed,
                                                          self._writer_error)
        for err in worker_errors:
            self._warnings.append("png write failed: " + err)
        if frames_failed:
            self._warnings.append("{} stereo pairs failed to encode/write".format(frames_failed))
        if writer_error is not None:
            self._warnings.append("png writer stopped accepting frames after a data.csv error ({}); {} later "
                                  "pair(s) rejected (drops.stereo_writer_failed)".format(
                                      writer_error, self._drops.get("stereo_writer_failed", 0)))
        if stopped_by == "atexit":
            self._warnings.append("closed by the interpreter-exit hook: the recorder never called close()")
        self._stopped_by = stopped_by
        self._pairs_kept = int(trim["rows_kept"])
        self.summary = self._build_summary(self._stopped_by, self._pairs_kept)
        self._write_summary_files()
        return self.summary

    def _stop_worker(self) -> None:
        """Hand the committer its sentinel, join it (it commits every pair still in flight), then
        shut the encoder pool down; never blocks on a queue nobody drains."""
        while True:
            if not self._worker.is_alive():
                discarded = 0
                while True:
                    try:
                        if self._queue.get_nowait() is not None:
                            discarded += 1
                    except queue.Empty:
                        break
                with self._lock:
                    self._frames_failed += discarded
                    reason = self._writer_error
                self._warnings.append("png writer thread died{}; {} queued pair(s) discarded".format(
                    " (" + reason + ")" if reason else "", discarded))
                self._pool.shutdown(wait=True)
                return
            try:
                self._queue.put(None, timeout=0.5)
                self._wake.set()
                break
            except queue.Full:
                continue  # the (alive) committer is still draining; re-check it and try again
        self._worker.join()
        self._pool.shutdown(wait=True)

    def _write_summary_files(self) -> None:
        assert self.summary is not None
        with open(self.root / "dataset.json", "w", encoding="utf-8", newline="\n") as fh:
            json.dump(self.summary, fh, indent=2, default=str)
            fh.write("\n")
        if self.calibration is not None:
            with open(self.root / "calibration.json", "w", encoding="utf-8", newline="\n") as fh:
                json.dump(self.calibration, fh, indent=2, default=str)
                fh.write("\n")
        (self.root / "README.txt").write_text(self._readme_text(self.summary), encoding="utf-8", newline="\n")

    def _close_at_exit(self) -> None:
        """``atexit`` hook: finish a recording whose owner never called :meth:`close`."""
        if self._closed:
            return
        try:
            self.close(stopped_by="atexit")
        except Exception as exc:  # noqa: BLE001 - one line, never a traceback at interpreter exit
            sys.stderr.write("EurocWriter: closing {} at interpreter exit failed: {}: {}\n".format(
                self.root, type(exc).__name__, exc))

    def _build_summary(self, stopped_by: str, pairs: int) -> Dict[str, Any]:
        rig = self.rig
        first_ts = self._first_frame_ts
        last_ts = self._last_frame_ts
        if self._first_imu_ts is not None:
            first_ts = self._first_imu_ts if first_ts is None else min(first_ts, self._first_imu_ts)
            last_ts = self._last_imu_ts if last_ts is None else max(last_ts, self._last_imu_ts)
        span_s = ((last_ts - first_ts) / _NS_PER_S) if (first_ts is not None and last_ts is not None) else 0.0
        frame_span = ((self._last_frame_ts - self._first_frame_ts) / _NS_PER_S
                      if self._frames_accepted > 1 else 0.0)
        imu_span = ((self._last_imu_ts - self._first_imu_ts) / _NS_PER_S if self._imu_written > 1 else 0.0)
        with self._lock:
            png_bytes, png_time, encode_time, written, failed = (
                self._png_bytes, self._png_time_s, self._png_encode_s, self._frames_written, self._frames_failed)
            busy_s = self._busy_seconds_locked(time.perf_counter())

        def cam_entry(cam: CameraInfo, name: str) -> Dict[str, Any]:
            return {
                "socket": cam.socket, "role": cam.role, "sensor": cam.sensor,
                "resolution": [cam.width, cam.height], "fps": self.cam_rate_hz, "type": cam.image_type,
                "undistorted": bool(cam.undistorted), "distortion_model_in_yaml": cam.distortion_model,
                "undistort": ({"method": "cv2.initUndistortRectifyMap + cv2.remap INTER_LINEAR",
                               "coefficients_used": 14, "new_camera_matrix": "same as intrinsics"}
                              if cam.undistorted else None),
                "borders": list(cam.borders),
                "sensor_yaml": "mav0/{}/sensor.yaml".format(name), "data_csv": "mav0/{}/data.csv".format(name),
            }

        # fixed key set (DROP_COUNTERS, always present) plus any counter the recorder registered
        # itself through count_drop(), kept even when zero so the key set never depends on values
        drops = {name: int(self._drops.get(name, 0)) for name in DROP_COUNTERS}
        for name, value in self._drops.items():
            if name not in drops:
                drops[name] = int(value)
        summary: Dict[str, Any] = {
            "format": FORMAT_NAME,
            "format_version": FORMAT_VERSION,
            "generator": {
                "script": "cuvslam_oak.dataset.EurocWriter",
                "depthai_version": None,
                "opencv_version": cv2.__version__,
                "python_version": platform.python_version(),
                "platform": platform.platform(),
                "hostname": platform.node(),
            },
            "device": {
                "device_id": rig.device_id or None, "name": None, "product": None, "platform": None,
                "usb_speed": None, "board_name": None, "board_rev": None, "imu": None,
                "imu_firmware": None, "eeprom_version": None, "eeprom_batch_time": None,
            },
            "record": {
                "start_wall_time": self._start_wall, "start_unix_ns": self._start_unix_ns,
                "requested_duration_s": None, "duration_s": round(span_s, 3), "headless": None,
                "warmup_frames_discarded": None, "stopped_by": stopped_by,
                "png_workers": int(self.png_workers),
            },
            "time": {"clock": "device", "epoch_offset_ns": None, "epoch_offset_source": None, "unit": "ns"},
            "cameras": {"cam0": cam_entry(rig.cam0, "cam0"), "cam1": cam_entry(rig.cam1, "cam1")},
            "stereo": {
                "pairing": None,
                "max_left_right_skew_ms_allowed": self.max_stereo_skew_ns / 1e6,
                "baseline_m": round(rig.baseline_m(), 6),
                "body_frame": rig.body_frame,
            },
            "imu": {
                "sensors": list(rig.imu.sensors), "rate_hz": self.imu_rate_hz, "batch_report_threshold": None,
                "timestamp_source": rig.imu.timestamp_source, "data_csv": "mav0/imu0/data.csv",
                "sensor_yaml": "mav0/imu0/sensor.yaml", "noise_source": rig.imu.noise_source,
                "noise": rig.imu.noise(), "eeprom_noise": rig.imu.eeprom_noise,
            },
            # counts: the format doc's four keys plus the writer's own integer counters (the doc
            # keeps `measured` for floats, so the counters live here)
            "counts": {
                "stereo_pairs": int(pairs), "imu_samples": int(self._imu_written),
                "first_timestamp_ns": first_ts, "last_timestamp_ns": last_ts,
                "stereo_timestamp_gaps": int(self._frame_gap_count),
                "frames_accepted": int(self._frames_accepted),
                "frames_written": int(written),
                "frames_failed": int(failed),
            },
            "measured": {
                "stereo_fps": round((self._frames_accepted - 1) / frame_span, 3) if frame_span > 0 else 0.0,
                "imu_hz": round((self._imu_written - 1) / imu_span, 3) if imu_span > 0 else 0.0,
                "max_stereo_gap_ms": round(self._max_frame_gap_ns / 1e6, 3),
                "max_imu_gap_ms": round(self._max_imu_gap_ns / 1e6, 3),
                "max_left_right_skew_us": round(self._max_lr_skew_ns / 1e3, 1),
                "imu_max_accel_gyro_skew_ms": round(self._max_imu_skew_ns / 1e6, 3),
                "png_write_ms_mean": round(1000.0 * png_time / written, 3) if written else 0.0,
                "png_encode_ms_mean": round(1000.0 * encode_time / written, 3) if written else 0.0,
                "png_throughput_pairs_per_s": (round(written / busy_s, 2) if (written and busy_s > 0.0) else 0.0),
                "png_bytes_total": int(png_bytes),
            },
            "drops": drops,
            "calibration": {
                "source": None, "dump": "calibration.json" if self.calibration is not None else None,
                "extrinsics_convention": ("getCameraExtrinsics(src, dst) = dst_from_src, cm -> m; "
                                          "T_BS = body_from_sensor, body = cam0"),
                "T_BS_cam1_from": None, "T_BS_imu0_from": None,
            },
            "warnings": list(self._warnings),
        }
        _deep_merge(summary, self._metadata)
        summary["warnings"] = list(summary.get("warnings") or [])
        return summary

    def _readme_text(self, s: Dict[str, Any]) -> str:
        c, m, d = s["counts"], s["measured"], s["drops"]
        cam0, cam1 = s["cameras"]["cam0"], s["cameras"]["cam1"]
        dev = s["device"]
        name = self.root.name
        lines = [
            "cuVSLAM recording  {}   (EuRoC ASL layout, see datasets/dataset-format.md)".format(name),
            "Device   {} {}, depthai {}, USB {}".format(dev.get("product") or "OAK-D", dev.get("device_id") or "?",
                                                        s["generator"].get("depthai_version") or "?",
                                                        dev.get("usb_speed") or "?"),
            "Streams  cam0 = {} left, cam1 = {} right: {}x{} {} @ {:g} fps, {}, {} pairs".format(
                cam0.get("socket") or "?", cam1.get("socket") or "?", cam0["resolution"][0], cam0["resolution"][1],
                cam0.get("type") or "GRAY8", float(cam0.get("fps") or 0.0),
                "undistorted" if cam0.get("undistorted") else "RAW (distorted)", c["stereo_pairs"]),
            "         imu0 = {} {} @ {:g} Hz, {} samples".format(
                dev.get("imu") or "IMU", " + ".join(s["imu"].get("sensors") or []), float(s["imu"].get("rate_hz") or 0.0),
                c["imu_samples"]),
            "Time     {} clock{}; {} s; first {}, last {}".format(
                s["time"].get("clock") or "device",
                " + epoch offset (Unix ns)" if s["time"].get("epoch_offset_ns") is not None else "",
                s["record"].get("duration_s"), c["first_timestamp_ns"], c["last_timestamp_ns"]),
            "Drops    stereo gaps {}, unpaired {}, sync violations {}, write-queue {}; imu gaps {}, out-of-order {}".format(
                d.get("stereo_sequence_gaps", 0), d.get("stereo_unpaired", 0), d.get("stereo_sync_violations", 0),
                d.get("stereo_write_queue_dropped", 0), d.get("imu_gaps", 0), d.get("imu_out_of_order", 0)),
            "Measured stereo {} fps, imu {} Hz, max stereo gap {} ms, max imu gap {} ms".format(
                m.get("stereo_fps"), m.get("imu_hz"), m.get("max_stereo_gap_ms"), m.get("max_imu_gap_ms")),
            "Frames   T_BS = body_from_sensor, body = cam0 (left camera optical frame, x right, y down, z forward)",
            "Caveats  IMU noise densities in imu0/sensor.yaml: {}.".format(s["imu"].get("noise_source") or "see sensor.yaml"),
        ]
        if not cam0.get("undistorted"):
            lines.append("         RAW frames with distortion_model {}: NVIDIA's track_euroc.py would build a WRONG "
                         "camera model from this dataset; use methods/cuvslam-vio/08_cuvslam_track.py.".format(
                             cam0.get("distortion_model_in_yaml")))
        for w in s.get("warnings") or []:
            lines.append("Warning  " + str(w))
        lines += [
            "",
            "Track with our reader (WSL2 / Jetson, cuvslam 17.0.0 installed):",
            "  python methods/cuvslam-vio/08_cuvslam_track.py recordings/{} --mode stereo-inertial --out trajectory.tum".format(name),
            "Track with NVIDIA's example, unmodified:",
            "  ln -s \"$PWD/recordings/{}/mav0\" PyCuVSLAM/examples/euroc/dataset/mav0".format(name),
            "  cd PyCuVSLAM/examples/euroc && python3 track_euroc.py       # edit line 90 for stereo-only",
            "Plot:  evo_traj tum trajectory.tum --plot --plot_mode xz",
            "",
        ]
        return "\n".join(lines)


# --------------------------------------------------------------------------------------
# Reader
# --------------------------------------------------------------------------------------

@dataclass
class ImuSample:
    """One ``imu0/data.csv`` row: ``gyro`` rad/s and ``accel`` m/s^2 (float64 ``(3,)`` arrays)."""

    ts_ns: int
    gyro: np.ndarray
    accel: np.ndarray

    @property
    def kind(self) -> str:
        return "imu"


@dataclass
class StereoFrame:
    """One stereo pair; ``left``/``right`` are ``(height, width)`` uint8 arrays (``None`` until loaded)."""

    ts_ns: int
    index: int
    paths: Tuple[Path, Path]
    left: Optional[np.ndarray] = None
    right: Optional[np.ndarray] = None

    @property
    def kind(self) -> str:
        return "stereo"

    def images(self, expected_shape: Optional[Tuple[int, int]] = None) -> List[np.ndarray]:
        """``[left, right]`` in rig camera order, loading from disk if needed."""
        if self.left is None:
            self.left = load_gray_png(self.paths[0], expected_shape)
        if self.right is None:
            self.right = load_gray_png(self.paths[1], expected_shape)
        return [self.left, self.right]


@dataclass
class ValidationReport:
    """Result of :meth:`EurocReader.validate`: ``errors`` make the dataset unusable, ``warnings`` do not."""

    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def __str__(self) -> str:
        lines = ["dataset OK" if self.ok else "dataset has {} error(s)".format(len(self.errors))]
        lines += ["ERROR: " + e for e in self.errors]
        lines += ["warning: " + w for w in self.warnings]
        return "\n".join(lines)


_CAMERA_YAML_KEYS = ("T_BS", "resolution", "intrinsics", "distortion_coefficients")
_IMU_YAML_KEYS = ("T_BS", "rate_hz") + IMU_NOISE_KEYS


def _numeric_list(value: Any, key: str, n: Optional[int], problems: List[str], where: str) -> List[float]:
    if not isinstance(value, list):
        problems.append("{}: {} must be a [list], got {!r}".format(where, key, value))
        return []
    out: List[float] = []
    for i, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            problems.append("{}: {}[{}] = {!r} is not a number (YAML 1.1 float formatting bug: "
                            "write 1.0e-05 not 1e-05)".format(where, key, i, item))
            return []
        out.append(float(item))
    if n is not None and len(out) != n:
        problems.append("{}: {} must have {} values, got {}".format(where, key, n, len(out)))
        return []
    return out


def _yaml_transform(cfg: Dict[str, Any], problems: List[str], where: str) -> Optional[np.ndarray]:
    keys = [k for k in cfg if str(k).startswith("T_")]
    if not keys:
        problems.append("{}: no T_* transform key".format(where))
        return None
    if keys[0] != "T_BS":
        problems.append("{}: first T_* key is {!r}, NVIDIA's loader takes the first one; expected T_BS".format(
            where, keys[0]))
    block = cfg[keys[0]]
    if not isinstance(block, dict) or "data" not in block:
        problems.append("{}: {} must be a mapping with a data: [16 floats] entry".format(where, keys[0]))
        return None
    data = _numeric_list(block["data"], keys[0] + ".data", 16, problems, where)
    if not data:
        return None
    try:
        return _as_transform(data)
    except DatasetError as exc:
        problems.append("{}: {}".format(where, exc))
        return None


def _camera_from_yaml(cfg: Dict[str, Any], where: str, problems: List[str],
                      name: str) -> Optional[CameraInfo]:
    missing = [k for k in _CAMERA_YAML_KEYS if k not in cfg]
    if missing:
        problems.append("{}: missing mandatory key(s) {}".format(where, ", ".join(missing)))
        return None
    T = _yaml_transform(cfg, problems, where)
    res = _numeric_list(cfg["resolution"], "resolution", 2, problems, where)
    intr = _numeric_list(cfg["intrinsics"], "intrinsics", 4, problems, where)
    coeffs = _numeric_list(cfg["distortion_coefficients"], "distortion_coefficients", None, problems, where)
    if T is None or not res or not intr:
        return None
    if any(not float(v).is_integer() or v <= 0 for v in res):
        problems.append("{}: resolution must be two positive integers, got {}".format(where, res))
        return None
    model = str(cfg.get("distortion_model", "radial-tangential"))
    if model.lower() not in DISTORTION_MODELS:
        problems.append("{}: unknown distortion_model {!r}".format(where, model))
        return None
    cam = CameraInfo(width=int(res[0]), height=int(res[1]), fu=intr[0], fv=intr[1], cu=intr[2], cv=intr[3],
                     T_BS=T, distortion_model=model, distortion_coefficients=coeffs,
                     rate_hz=float(cfg.get("rate_hz") or 0.0) if isinstance(cfg.get("rate_hz"), (int, float)) else 0.0,
                     comment=str(cfg.get("comment") or ""), role="left" if name == "cam0" else "right")
    try:
        cam.cuvslam_distortion()
    except DatasetError as exc:
        problems.append("{}: {}".format(where, exc))
        return None
    cam.undistorted = all(c == 0.0 for c in coeffs)
    return cam


def _imu_from_yaml(cfg: Dict[str, Any], where: str, problems: List[str]) -> Optional[ImuInfo]:
    missing = [k for k in _IMU_YAML_KEYS if k not in cfg]
    if missing:
        problems.append("{}: missing mandatory key(s) {}".format(where, ", ".join(missing)))
        return None
    T = _yaml_transform(cfg, problems, where)
    values: Dict[str, float] = {}
    for key in ("rate_hz",) + IMU_NOISE_KEYS:
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            problems.append("{}: {} = {!r} is not a number (YAML 1.1 float formatting bug: write 1.0e-05 "
                            "not 1e-05)".format(where, key, v))
            return None
        values[key] = float(v)
    if T is None:
        return None
    try:
        return ImuInfo(T_BS=T, comment=str(cfg.get("comment") or ""), **values)
    except DatasetError as exc:
        problems.append("{}: {}".format(where, exc))
        return None


class EurocReader:
    """Read one EuRoC ASL recording written by :class:`EurocWriter` (or any EuRoC sequence).

    *root* is the recording folder (contains ``mav0/``) or the ``mav0`` directory itself.
    Construction parses the three ``sensor.yaml`` files into :attr:`rig` (a :class:`RigInfo`),
    reads ``cam0/data.csv``, ``cam1/data.csv``, ``imu0/data.csv`` and ``dataset.json`` (if
    present, :attr:`dataset`).  With ``strict=True`` (default) a sensor.yaml problem raises
    :class:`DatasetError`; otherwise :attr:`rig` may be ``None`` and :meth:`validate` lists
    the problems.  Images are read lazily by :meth:`frames` / :meth:`merged`.

    Iteration order for a tracker (:meth:`merged`): all IMU samples with ``ts <= frame ts``
    come before that frame, then the frame; this is the RealSense-example order and satisfies
    cuVSLAM's rules (frames strictly increasing, IMU non-decreasing, interleaved non-decreasing).
    """

    def __init__(self, root: Union[str, Path], *, strict: bool = True) -> None:
        self.root, self.mav0 = resolve_mav0(root)
        self.strict = bool(strict)
        self._problems: List[str] = []
        self.rig: Optional[RigInfo] = None
        self.dataset: Optional[Dict[str, Any]] = None

        recal = self.mav0 / "cam0" / "sensor_recalibrated.yaml"
        if recal.exists():
            raise DatasetError("{} exists: NVIDIA's loader would switch every sensor to the Fisheye model; "
                               "delete it (our recorder never writes it)".format(recal))

        self._yaml: Dict[str, Dict[str, Any]] = {}
        cams: Dict[str, Optional[CameraInfo]] = {}
        for name in ("cam0", "cam1"):
            cfg = self._load_yaml(self.mav0 / name / "sensor.yaml")
            cams[name] = _camera_from_yaml(cfg, "{}/sensor.yaml".format(name), self._problems, name) if cfg else None
        imu_cfg = self._load_yaml(self.mav0 / "imu0" / "sensor.yaml")
        imu = _imu_from_yaml(imu_cfg, "imu0/sensor.yaml", self._problems) if imu_cfg else None
        cam0, cam1 = cams["cam0"], cams["cam1"]
        if cam0 is not None and cam1 is not None and imu is not None:
            if cam0.resolution != cam1.resolution:
                self._problems.append("cam0 and cam1 resolutions differ: {} vs {}".format(cam0.resolution, cam1.resolution))
            else:
                self.rig = RigInfo(cam0=cam0, cam1=cam1, imu=imu)
        if self.strict and self._problems:
            raise DatasetError("; ".join(self._problems))

        ds_path = self.root / "dataset.json"
        if ds_path.is_file():
            try:
                with open(ds_path, "r", encoding="utf-8") as fh:
                    self.dataset = json.load(fh)
            except (OSError, ValueError) as exc:
                self._problems.append("dataset.json unreadable: {}".format(_one_line(exc)))
        if self.dataset is not None and self.rig is not None:
            self._apply_dataset_json(self.dataset)

        self._frames: List[Tuple[int, str, str]] = []
        self._pair_problems: List[str] = []
        self._load_camera_csvs()
        self._imu_ts = np.zeros(0, dtype=np.int64)
        self._imu_gyro = np.zeros((0, 3), dtype=np.float64)
        self._imu_accel = np.zeros((0, 3), dtype=np.float64)
        self._load_imu_csv()

    # ---------------------------------------------------------------- parsing
    def _load_yaml(self, path: Path) -> Optional[Dict[str, Any]]:
        rel = path.relative_to(self.mav0).as_posix()
        if not path.is_file():
            self._problems.append("{} is missing (mandatory in every mode)".format(rel))
            return None
        try:
            cfg = parse_simple_yaml(path.read_text(encoding="utf-8"))
        except (OSError, DatasetError) as exc:
            self._problems.append("{}: {}".format(rel, _one_line(exc)))
            return None
        self._yaml[rel] = cfg
        return cfg

    def _apply_dataset_json(self, ds: Dict[str, Any]) -> None:
        """Copy the dataset.json facts the yaml cannot carry (sockets, undistorted flag, borders...)."""
        assert self.rig is not None
        cams = ds.get("cameras") or {}
        for name, cam in (("cam0", self.rig.cam0), ("cam1", self.rig.cam1)):
            entry = cams.get(name) or {}
            cam.socket = entry.get("socket", cam.socket)
            cam.role = entry.get("role", cam.role)
            cam.sensor = entry.get("sensor", cam.sensor)
            cam.image_type = entry.get("type", cam.image_type)
            if "undistorted" in entry:
                cam.undistorted = bool(entry["undistorted"])
            borders = entry.get("borders")
            if isinstance(borders, list) and len(borders) == 4:
                cam.borders = tuple(int(b) for b in borders)  # type: ignore[assignment]
            if entry.get("fps"):
                cam.rate_hz = float(entry["fps"])
        imu = ds.get("imu") or {}
        if imu.get("sensors"):
            self.rig.imu.sensors = list(imu["sensors"])
        if imu.get("noise_source"):
            self.rig.imu.noise_source = str(imu["noise_source"])
        if imu.get("timestamp_source"):
            self.rig.imu.timestamp_source = str(imu["timestamp_source"])
        if imu.get("eeprom_noise") is not None:
            self.rig.imu.eeprom_noise = imu["eeprom_noise"]
        dev = ds.get("device") or {}
        if dev.get("device_id"):
            self.rig.device_id = str(dev["device_id"])
        stereo = ds.get("stereo") or {}
        if stereo.get("body_frame"):
            self.rig.body_frame = str(stereo["body_frame"])

    def _load_camera_csvs(self) -> None:
        rows: Dict[str, List[List[str]]] = {}
        for name in ("cam0", "cam1"):
            path = self.mav0 / name / "data.csv"
            if not path.is_file():
                self._pair_problems.append("{}/data.csv is missing".format(name))
                rows[name] = []
                continue
            try:
                rows[name] = _read_csv_rows(path, 2)
            except (OSError, DatasetError) as exc:
                self._pair_problems.append("{}/data.csv: {}".format(name, _one_line(exc)))
                rows[name] = []
        r0, r1 = rows["cam0"], rows["cam1"]
        if len(r0) != len(r1):
            self._pair_problems.append("cam0 has {} rows, cam1 has {} (NVIDIA's loader raises 'Number of frames "
                                       "differs'); pairing the first {}".format(len(r0), len(r1), min(len(r0), len(r1))))
        frames: List[Tuple[int, str, str]] = []
        for i, (a, b) in enumerate(zip(r0, r1)):
            try:
                ta, tb = int(a[0]), int(b[0])
            except ValueError:
                self._pair_problems.append("row {}: non-integer timestamp {!r} / {!r}".format(i, a[0], b[0]))
                continue
            if ta != tb:
                self._pair_problems.append("row {}: cam0 timestamp {} != cam1 timestamp {}".format(i, ta, tb))
            frames.append((ta, a[1], b[1]))
        self._frames = frames

    def _load_imu_csv(self) -> None:
        path = self.mav0 / "imu0" / "data.csv"
        if not path.is_file():
            self._pair_problems.append("imu0/data.csv is missing (write it even for stereo-only datasets)")
            return
        try:
            rows = _read_csv_rows(path, 7)
            ts = np.array([int(r[0]) for r in rows], dtype=np.int64)
            vals = np.array([[float(x) for x in r[1:7]] for r in rows], dtype=np.float64).reshape(-1, 6)
        except (OSError, ValueError, DatasetError) as exc:
            self._pair_problems.append("imu0/data.csv: {}".format(_one_line(exc)))
            return
        self._imu_ts = ts
        self._imu_gyro = np.ascontiguousarray(vals[:, 0:3])
        self._imu_accel = np.ascontiguousarray(vals[:, 3:6])

    # ---------------------------------------------------------------- facts
    @property
    def cam0(self) -> CameraInfo:
        if self.rig is None:
            raise DatasetError("sensor.yaml files are invalid: " + "; ".join(self._problems))
        return self.rig.cam0

    @property
    def cam1(self) -> CameraInfo:
        if self.rig is None:
            raise DatasetError("sensor.yaml files are invalid: " + "; ".join(self._problems))
        return self.rig.cam1

    @property
    def imu(self) -> ImuInfo:
        if self.rig is None:
            raise DatasetError("sensor.yaml files are invalid: " + "; ".join(self._problems))
        return self.rig.imu

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def imu_count(self) -> int:
        return int(self._imu_ts.shape[0])

    @property
    def frame_timestamps(self) -> np.ndarray:
        """int64 array of stereo-pair timestamps (ns) in csv order."""
        return np.array([f[0] for f in self._frames], dtype=np.int64)

    @property
    def imu_timestamps(self) -> np.ndarray:
        return self._imu_ts.copy()

    @property
    def imu_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(timestamps int64 (N,), gyro float64 (N, 3), accel float64 (N, 3))`` views."""
        return self._imu_ts, self._imu_gyro, self._imu_accel

    @property
    def first_ts_ns(self) -> Optional[int]:
        cands = [self._frames[0][0]] if self._frames else []
        if self.imu_count:
            cands.append(int(self._imu_ts[0]))
        return min(cands) if cands else None

    @property
    def last_ts_ns(self) -> Optional[int]:
        cands = [self._frames[-1][0]] if self._frames else []
        if self.imu_count:
            cands.append(int(self._imu_ts[-1]))
        return max(cands) if cands else None

    @property
    def duration_s(self) -> float:
        if self.first_ts_ns is None or self.last_ts_ns is None:
            return 0.0
        return (self.last_ts_ns - self.first_ts_ns) / _NS_PER_S

    def frame_paths(self, index: int) -> Tuple[Path, Path]:
        """``(cam0/data/<f>, cam1/data/<f>)`` for pair *index*."""
        ts, f0, f1 = self._frames[index]
        return (self.mav0 / "cam0" / "data" / f0, self.mav0 / "cam1" / "data" / f1)

    def load_image(self, path: Union[str, Path]) -> np.ndarray:
        """Read one PNG as a C-contiguous uint8 ``(height, width)`` array checked against the rig."""
        return load_gray_png(path, self.rig.cam0.shape if self.rig is not None else None)

    # ---------------------------------------------------------------- iteration
    def frames(self, start: int = 0, stop: Optional[int] = None, step: int = 1,
               load_images: bool = True) -> Iterator[StereoFrame]:
        """Yield :class:`StereoFrame` in csv order (``left``/``right`` loaded unless *load_images* is False)."""
        shape = self.rig.cam0.shape if self.rig is not None else None
        for i in range(start, len(self._frames) if stop is None else min(stop, len(self._frames)), max(1, step)):
            ts = self._frames[i][0]
            paths = self.frame_paths(i)
            frame = StereoFrame(ts_ns=ts, index=i, paths=paths)
            if load_images:
                frame.images(shape)
            yield frame

    def imu_samples(self, start: int = 0, stop: Optional[int] = None) -> Iterator[ImuSample]:
        """Yield :class:`ImuSample` in csv order."""
        n = self.imu_count if stop is None else min(stop, self.imu_count)
        for i in range(start, n):
            yield ImuSample(int(self._imu_ts[i]), self._imu_gyro[i].copy(), self._imu_accel[i].copy())

    def merged(self, load_images: bool = True, with_imu: bool = True,
               start: int = 0, stop: Optional[int] = None) -> Iterator[Union[ImuSample, StereoFrame]]:
        """Yield IMU samples and stereo frames in tracker order.

        Every IMU sample whose timestamp is ``<=`` a frame's is yielded *before* that frame
        (``register_imu_measurement`` then ``track``).  *start*/*stop* select frames (like a
        slice); IMU samples not newer than the previous frame are skipped so the stream starts
        just after it, and IMU samples after the dataset's last frame are yielded at the end
        only when that frame is part of the selection.
        """
        shape = self.rig.cam0.shape if self.rig is not None else None
        n_total = len(self._frames)
        n_frames = n_total if stop is None else min(stop, n_total)
        j = 0
        n_imu = self.imu_count if with_imu else 0
        if with_imu and start > 0 and n_frames > start:
            prev_ts = self._frames[start - 1][0]
            j = int(np.searchsorted(self._imu_ts, prev_ts, side="right"))
        for i in range(start, n_frames):
            ts = self._frames[i][0]
            while j < n_imu and int(self._imu_ts[j]) <= ts:
                yield ImuSample(int(self._imu_ts[j]), self._imu_gyro[j].copy(), self._imu_accel[j].copy())
                j += 1
            frame = StereoFrame(ts_ns=ts, index=i, paths=self.frame_paths(i))
            if load_images:
                frame.images(shape)
            yield frame
        if n_frames == n_total:
            while j < n_imu:
                yield ImuSample(int(self._imu_ts[j]), self._imu_gyro[j].copy(), self._imu_accel[j].copy())
                j += 1

    # ---------------------------------------------------------------- validation
    def validate(self, check_images: bool = True, decode_images: Optional[int] = 3) -> ValidationReport:
        """Check every rule of the format doc; return a :class:`ValidationReport`.

        *check_images*: verify every referenced PNG exists.  *decode_images*: decode that many
        PNG pairs (spread over the sequence) and check dtype/shape; the default 3 is a spot
        check, pass ``None`` to size-check every PNG (slow on a long recording).  Also warns when
        NVIDIA's ``dataset_utils`` would misread a camera's distortion coefficients (anything
        but all-zero or ``radial-tangential`` with exactly 4 values), since it ignores
        ``distortion_model``.
        """
        rep = ValidationReport()
        rep.errors.extend(self._problems)
        rep.errors.extend(self._pair_problems)
        frames = self._frames
        ts_list = [f[0] for f in frames]
        if self.rig is not None:
            for name, cam in (("cam0", self.rig.cam0), ("cam1", self.rig.cam1)):
                note = cam.nvidia_loader_note()
                if note:
                    rep.warnings.append("{}: {}".format(name, note))

        # frame timestamps strictly increasing; file names match
        bad_order = [i for i in range(1, len(ts_list)) if ts_list[i] <= ts_list[i - 1]]
        if bad_order:
            rep.errors.append("frame timestamps not strictly increasing at row(s) {}{}".format(
                bad_order[:5], "..." if len(bad_order) > 5 else ""))
        mismatched = [i for i, (ts, f0, f1) in enumerate(frames) if f0 != "{}.png".format(ts) or f1 != "{}.png".format(ts)]
        if mismatched:
            rep.warnings.append("{} row(s) whose file name is not <timestamp>.png (first at row {})".format(
                len(mismatched), mismatched[0]))
        if not frames:
            rep.errors.append("no stereo pairs")

        # IMU
        imu_ts = self._imu_ts
        if imu_ts.shape[0] == 0:
            rep.warnings.append("no IMU samples (stereo-only tracking only)")
        else:
            if np.any(np.diff(imu_ts) < 0):
                idx = int(np.argmax(np.diff(imu_ts) < 0)) + 1
                rep.errors.append("IMU timestamps decrease at row {} (register_imu_measurement needs non-decreasing)".format(idx))
            if not np.all(np.isfinite(self._imu_gyro)) or not np.all(np.isfinite(self._imu_accel)):
                rep.errors.append("IMU data contains NaN/inf")
            if frames:
                if int(imu_ts[0]) > ts_list[0]:
                    rep.warnings.append("first IMU sample ({}) is after the first frame ({}): no IMU for the first "
                                        "{:.1f} ms".format(int(imu_ts[0]), ts_list[0], (int(imu_ts[0]) - ts_list[0]) / 1e6))
                if int(imu_ts[-1]) < ts_list[-1]:
                    rep.warnings.append("IMU stream ends {:.1f} ms before the last frame".format(
                        (ts_list[-1] - int(imu_ts[-1])) / 1e6))
            if self.rig is not None and imu_ts.shape[0] > 1:
                span = (int(imu_ts[-1]) - int(imu_ts[0])) / _NS_PER_S
                rate = (imu_ts.shape[0] - 1) / span if span > 0 else 0.0
                if self.rig.imu.rate_hz > 0 and abs(rate - self.rig.imu.rate_hz) > 0.1 * self.rig.imu.rate_hz:
                    rep.warnings.append("measured IMU rate {:.1f} Hz differs >10% from sensor.yaml rate_hz {:g}".format(
                        rate, self.rig.imu.rate_hz))
                period = _NS_PER_S / self.rig.imu.rate_hz if self.rig.imu.rate_hz > 0 else 0.0
                if period > 0:
                    gaps = int(np.count_nonzero(np.diff(imu_ts) > 2.5 * period))
                    if gaps:
                        rep.warnings.append("{} IMU gap(s) longer than 2.5 sample periods".format(gaps))
        if self.rig is not None and len(ts_list) > 1 and self.rig.cam0.rate_hz > 0:
            span = (ts_list[-1] - ts_list[0]) / _NS_PER_S
            fps = (len(ts_list) - 1) / span if span > 0 else 0.0
            if abs(fps - self.rig.cam0.rate_hz) > 0.1 * self.rig.cam0.rate_hz:
                rep.warnings.append("measured frame rate {:.2f} fps differs >10% from rate_hz {:g}".format(
                    fps, self.rig.cam0.rate_hz))
            period = _NS_PER_S / self.rig.cam0.rate_hz
            gaps = int(np.count_nonzero(np.diff(np.array(ts_list, dtype=np.int64)) > 1.5 * period))
            if gaps:
                rep.warnings.append("{} frame gap(s) longer than 1.5 frame periods".format(gaps))

        # files
        if check_images and frames:
            missing: List[str] = []
            for i in range(len(frames)):
                for p in self.frame_paths(i):
                    if not p.is_file():
                        missing.append(p.relative_to(self.mav0).as_posix())
            if missing:
                rep.errors.append("{} PNG file(s) missing, e.g. {}".format(len(missing), ", ".join(missing[:3])))
        if frames and (decode_images is None or decode_images > 0):
            n = len(frames)
            if decode_images is None or decode_images >= n:
                indices = list(range(n))
            else:
                indices = sorted({int(round(k * (n - 1) / max(1, decode_images - 1))) for k in range(decode_images)})
            shape = self.rig.cam0.shape if self.rig is not None else None
            for i in indices:
                for p in self.frame_paths(i):
                    if not p.is_file():
                        continue
                    try:
                        load_gray_png(p, shape)
                    except DatasetError as exc:
                        rep.errors.append(str(exc))
                        break

        # dataset.json cross-checks
        if self.dataset is None:
            rep.warnings.append("dataset.json missing (fine for a foreign EuRoC sequence)")
        else:
            ds = self.dataset
            if ds.get("format") != FORMAT_NAME:
                rep.warnings.append("dataset.json format is {!r}, expected {!r}".format(ds.get("format"), FORMAT_NAME))
            counts = ds.get("counts") or {}
            if counts.get("stereo_pairs") is not None and int(counts["stereo_pairs"]) != len(frames):
                rep.errors.append("dataset.json counts.stereo_pairs = {} but data.csv has {} pairs".format(
                    counts["stereo_pairs"], len(frames)))
            if counts.get("imu_samples") is not None and int(counts["imu_samples"]) != self.imu_count:
                rep.errors.append("dataset.json counts.imu_samples = {} but imu0/data.csv has {}".format(
                    counts["imu_samples"], self.imu_count))
            drops = ds.get("drops") or {}
            nonzero = {k: v for k, v in drops.items() if isinstance(v, (int, float)) and v}
            if nonzero:
                rep.warnings.append("recorder reported drops: {}".format(
                    ", ".join("{} {}".format(k, int(v)) for k, v in sorted(nonzero.items()))))
            for w in ds.get("warnings") or []:
                rep.warnings.append("recorder: {}".format(w))
            cams = ds.get("cameras") or {}
            if self.rig is not None:
                for name, cam in (("cam0", self.rig.cam0), ("cam1", self.rig.cam1)):
                    res = (cams.get(name) or {}).get("resolution")
                    if res is not None and tuple(int(v) for v in res) != cam.resolution:
                        rep.errors.append("dataset.json {} resolution {} != sensor.yaml {}".format(name, res, cam.resolution))
                    if (cams.get(name) or {}).get("undistorted") is False and cam.distortion_model.lower() == "radial-tangential":
                        rep.warnings.append("{}: raw frames with radial-tangential coefficients: NVIDIA's loader "
                                            "would build Brown from them".format(name))
        return rep

    def __repr__(self) -> str:
        res = "{}x{}".format(*self.rig.cam0.resolution) if self.rig is not None else "?"
        return "EurocReader({!r}: {} pairs {}, {} IMU samples, {:.1f} s)".format(
            str(self.root), self.frame_count, res, self.imu_count, self.duration_s)
