"""Build a PyCuVSLAM v17.0.0 ``Rig`` for the OAK-D S2 from its EEPROM calibration.

What this module does
---------------------
It turns the calibration stored in an OAK-D-S2-AF (two OV9282 global-shutter mono cameras
CAM_B = stereo left, CAM_C = stereo right, one BNO086 IMU) into the objects cuVSLAM needs:
two ``cuvslam.Camera`` (intrinsics at the delivered frame size, first 8 OpenCV distortion
coefficients as ``Distortion.Model.Polynomial``, ``rig_from_camera`` in metres) and, for
stereo-inertial tracking, one ``cuvslam.ImuCalibration`` (``rig_from_imu`` plus noise
densities).  It follows ``methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md`` exactly and emits the EuRoC
``sensor.yaml`` dictionaries described in ``datasets/dataset-format.md``.

The calibration can come from three places, all handled by :func:`as_calibration`:

* a ``dict`` as returned by ``dai.CalibrationHandler.eepromToJson()`` (what
  ``camera/05_record.py`` writes to ``recordings/*/calibration.json``),
* the ``camera_info.json`` inside a DepthAI holistic recording tar (the same dump wrapped
  under a ``"calibration"`` key), or a path to either file,
* a live ``dai.CalibrationHandler`` (``device.readCalibration()``); its ``eepromToJson()`` is
  taken and the same pure-numpy code runs, so the live path and the offline path can never
  disagree.  ``depthai`` is never imported by this module; :func:`verify_against_depthai`
  imports it lazily to cross-check the numpy port against DepthAI's own implementation.

DepthAI's ``CalibrationHandler`` algorithms are re-implemented here (``getCameraIntrinsics``
resize/crop rules in float32, ``getCameraExtrinsics`` chaining through the EEPROM links,
``getImuToCameraExtrinsics``) from ``depthai-core v3.10.0 src/device/CalibrationHandler.cpp``.
Cross-checked on this unit's dump with :func:`verify_against_depthai`: intrinsics at eight
sizes in both aspect modes, all nine socket pairs of ``getCameraExtrinsics`` (calibrated and
spec translations) and ``getImuToCameraExtrinsics`` for CAM_A/B/C agree with
``dai.CalibrationHandler`` (depthai 3.10.0) to better than 1e-6.

Conventions (from the docs, verified against the sources)
---------------------------------------------------------
* ``X_from_Y`` is the 4x4 SE(3) matrix with ``p_X = X_from_Y @ p_Y``; cuVSLAM names its
  fields the same way (``rig_from_camera``, ``rig_from_imu``, ``world_from_rig``).
* All frames are OpenCV right-handed: x right, y down, z forward.  DepthAI extrinsics are in
  the same convention, so **no axis flip** is applied anywhere; the only non-identity
  rotation in the rig is the EEPROM ``imuExtrinsics`` rotation (``diag(1, -1, -1)`` on this
  unit) inside ``rig_from_imu``.
* ``DepthAI getCameraExtrinsics(src, dst)`` returns ``dst_from_src`` with the translation in
  centimetres; here everything is converted to **metres** and never inverted or transposed:
  ``rig_from_camera(calib, socket, rig_socket) == getCameraExtrinsics(socket, rig_socket) / 100``.
* The default rig frame is CAM_A (the colour camera), exactly like NVIDIA's
  ``examples/oak-d/run_stereo.py``.  Pass ``rig_socket=CAM_B`` to make the left camera the rig
  (EuRoC style, what ``dataset.py`` / NVIDIA's ``track_euroc.py`` do); the EuRoC helpers use
  ``body_socket=CAM_B`` by default so ``T_BS(cam0)`` is the identity.
* Quaternions are ``(x, y, z, w)`` (``cuvslam.Pose`` order == scipy ``as_quat()``); the
  conversion is a scipy-free Shepperd routine (:func:`matrix_to_quaternion_xyzw`).
* IMU streams: ``imu_frame="raw"`` (``ACCELEROMETER_RAW`` + ``GYROSCOPE_RAW``, sensor-native
  axes) keeps the EEPROM rotation in ``rig_from_imu``; ``imu_frame="aligned"``
  (``*_UNCALIBRATED`` / ``*_CALIBRATED``, already rotated on the device into the CAM_A frame)
  replaces that rotation by the identity.  Never mix the two.
* IMU noise densities: the EEPROM block has undocumented units and is **not** used.  Two
  placeholder presets exist and each helper follows the doc that describes its output:
  :func:`build_rig` / :func:`imu_noise` default to ``DEFAULT_NOISE_PRESET = "conservative"``
  (calibration doc section 5.2 / 8, the live rig), the EuRoC ``sensor.yaml`` helpers default to
  ``EUROC_NOISE_PRESET = "realsense"`` (dataset doc section 7 / 12.1: NVIDIA's RealSense values,
  the same numbers ``dataset.py`` ships as ``PLACEHOLDER_IMU_NOISE``).  Both are recorded with
  their provenance (``rig_info['imu']['noise']['source']``, the yaml ``comment``) so the bench
  measurement can replace either without guessing which one a file holds.
* Calibrations without IMU extrinsics (e.g. an OAK-D Lite whose EEPROM lacks ``imuExtrinsics``):
  mode ``'stereo'`` builds normally and reports ``rig_info['imu']['available'] = False`` with
  the reason in ``rig_info['imu']['error']``; mode ``'vio'`` raises :class:`CalibrationError`.
  The EuRoC helpers raise too, because NVIDIA's loader reads ``imu0/sensor.yaml``
  unconditionally, unless ``allow_missing_imu=True`` asks for an identity ``T_BS`` placeholder
  (with a :class:`CalibrationWarning`) so a stereo-only dataset can still be written.
  :func:`has_imu_extrinsics` tells the two cases apart up front.

Typical use
-----------
>>> from cuvslam_oak import rig as oakrig
>>> calib = oakrig.load_calibration_json("recordings/video_20260918_034017/calibration.json")
>>> rig, info = oakrig.build_rig(calib, (1280, 800), "vio", imu_rate_hz=200, vslam=cuvslam)
>>> yamls = oakrig.to_euroc_sensor_yaml(calib, (1280, 800), imu_rate_hz=200, undistorted=True)
>>> text = oakrig.sensor_yaml_text(yamls["cam0"])

Command line (no camera, no cuvslam needed): ``python -m cuvslam_oak.rig <calibration.json>
[--size 640x400] [--mode vio] [--rig-socket CAM_B] [--euroc] [--verify]`` prints the rig
description as JSON (or the three sensor.yaml texts with ``--euroc``); every failure is one
line on stderr and exit status 1.  :class:`CalibrationWarning` is attributed to the caller's
line (the first frame outside this module), so ``warnings.filterwarnings(module=...)`` works.

Runs on Windows (no cuvslam wheel: pass ``vslam=`` a fake module, or use the pure-numpy
:func:`rig_spec` / EuRoC helpers which need no cuvslam at all), WSL2 and Jetson (Python 3.10,
numpy 2.x).  Importing this module has no side effects and imports only the standard library
and numpy.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import warnings
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

__all__ = [
    "CAM_A",
    "CAM_B",
    "CAM_C",
    "SOCKET_AUTO",
    "DEFAULT_SIZE",
    "SUPPORTED_SIZES",
    "MODES",
    "IMU_FRAMES",
    "DISTORTION_NAMES",
    "NOISE_PRESETS",
    "DEFAULT_NOISE_PRESET",
    "EUROC_NOISE_PRESET",
    "MISSING_IMU_COMMENT",
    "DEFAULT_BORDERS",
    "CalibrationError",
    "CalibrationWarning",
    "socket_id",
    "socket_name",
    "load_calibration_json",
    "normalize_calibration",
    "as_calibration",
    "camera_entry",
    "camera_sockets",
    "stereo_sockets",
    "native_size",
    "intrinsics_for",
    "distortion_coefficients",
    "distortion_model_for",
    "distortion_for",
    "camera_extrinsics",
    "rig_from_camera",
    "imu_extrinsics_raw",
    "has_imu_extrinsics",
    "rig_from_imu",
    "stereo_baseline_m",
    "imu_noise",
    "check_rotation",
    "matrix_to_quaternion_xyzw",
    "quaternion_xyzw_to_matrix",
    "invert_se3",
    "se3",
    "pose_from_matrix",
    "matrix_from_pose",
    "default_borders",
    "normalize_borders",
    "camera_spec",
    "rig_spec",
    "rig_from_spec",
    "build_rig",
    "euroc_camera_sensor",
    "euroc_imu_sensor",
    "to_euroc_sensor_yaml",
    "yaml_float",
    "sensor_yaml_text",
    "undistort_maps",
    "verify_against_depthai",
]

# --------------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------------

#: Socket names as strings so nothing here needs ``depthai``; ``socket_id`` also accepts
#: ``dai.CameraBoardSocket`` members and plain ints (CAM_A = 0, CAM_B = 1, CAM_C = 2).
CAM_A = "CAM_A"
CAM_B = "CAM_B"
CAM_C = "CAM_C"
#: ``dai.CameraBoardSocket.AUTO``: the EEPROM link target of the origin camera.
SOCKET_AUTO = -1

_SOCKET_IDS: Dict[str, int] = {"CAM_{}".format(chr(ord("A") + i)): i for i in range(8)}

#: Native OV9282 calibration size on the S2; ``(640, 400)`` is its exact half.
DEFAULT_SIZE: Tuple[int, int] = (1280, 800)
SUPPORTED_SIZES: Tuple[Tuple[int, int], ...] = ((1280, 800), (640, 400), (320, 200))
DEFAULT_RIG_SOCKET = CAM_A
DEFAULT_BODY_SOCKET = CAM_B  # EuRoC body frame = cam0 = left camera (T_BS(cam0) = identity)

MODES: Tuple[str, ...] = ("stereo", "vio")
_ODOMETRY_MODE_NAME = {"stereo": "Multicamera", "vio": "Inertial"}
IMU_FRAMES: Tuple[str, ...] = ("raw", "aligned")
_IMU_STREAMS = {
    "raw": ["ACCELEROMETER_RAW", "GYROSCOPE_RAW"],
    "aligned": ["ACCELEROMETER_UNCALIBRATED", "GYROSCOPE_UNCALIBRATED"],
}

#: OpenCV order of DepthAI's 14 ``distortionCoeff`` values (rational + thin prism + tilt).
DISTORTION_NAMES: Tuple[str, ...] = ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
                                     "s1", "s2", "s3", "s4", "tx", "ty")
#: cuVSLAM ``Distortion.Model.Polynomial`` takes exactly the first 8 (same order).
POLYNOMIAL_COEFFICIENTS = 8
FISHEYE_COEFFICIENTS = 4
CAMERA_MODEL_PERSPECTIVE = 0  # dai.CameraModel.Perspective
CAMERA_MODEL_FISHEYE = 1      # dai.CameraModel.Fisheye

#: ``dai.LengthUnit`` value -> metres (``lengthUnit`` field of every EEPROM extrinsics block).
LENGTH_UNIT_TO_METRES: Dict[int, float] = {0: 1.0, 1: 0.01, 2: 0.001, 3: 0.0254, 4: 0.3048}
_DEFAULT_LENGTH_UNIT = 1  # CENTIMETER (what every DepthAI EEPROM stores)

#: Feature-exclusion borders of NVIDIA's ``examples/oak-d/run_stereo.py`` (1280 px wide raw
#: frames); scaled with the frame size by :func:`default_borders`.
DEFAULT_BORDERS: Dict[str, int] = {"top": 50, "bottom": 0, "left": 70, "right": 70}
_BORDER_KEYS: Tuple[str, ...] = ("top", "bottom", "left", "right")

#: IMU noise presets (cuVSLAM / Kalibr units: rad/s/sqrt(Hz), rad/s^2/sqrt(Hz),
#: m/s^2/sqrt(Hz), m/s^3/sqrt(Hz)).  ``conservative`` is what the calibration doc ships;
#: ``realsense`` is NVIDIA's ``examples/realsense/camera_utils.py`` set (dataset doc);
#: ``euroc`` is the ADIS16448 tactical-grade set, an optimistic bound, never for a BNO086.
NOISE_PRESETS: Dict[str, Dict[str, Any]] = {
    "conservative": {
        "gyroscope_noise_density": 3.0e-3,
        "gyroscope_random_walk": 3.0e-5,
        "accelerometer_noise_density": 3.0e-2,
        "accelerometer_random_walk": 1.0e-3,
        "source": ("conservative defaults from methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md section 5.2 "
                   "(consumer MEMS, ~20x the industry-unit reading of the EEPROM table); "
                   "the BNO086 has not been Allan-characterised"),
    },
    "realsense": {
        "gyroscope_noise_density": 6.0673370376614875e-03,
        "gyroscope_random_walk": 3.6211951458325785e-05,
        "accelerometer_noise_density": 3.3621979208052800e-02,
        "accelerometer_random_walk": 9.8256589971851467e-04,
        # byte-identical to dataset.py PLACEHOLDER_NOISE_SOURCE and the dataset doc's 12.1 example,
        # so dataset.json "noise_source" reads the same whichever module wrote it
        "source": "placeholder: NVIDIA examples/realsense/camera_utils.py values; not measured on this BNO086",
    },
    "euroc": {
        "gyroscope_noise_density": 1.6968e-04,
        "gyroscope_random_walk": 1.9393e-05,
        "accelerometer_noise_density": 2.0e-3,
        "accelerometer_random_walk": 3.0e-3,
        "source": "EuRoC ADIS16448 (examples/euroc/sensor_imu0.yaml); optimistic bound, not for a BNO086",
    },
}
#: Preset :func:`build_rig` / :func:`rig_spec` / :func:`imu_noise` use for the live rig
#: (calibration doc section 5.2 and the section 8 contract).
DEFAULT_NOISE_PRESET = "conservative"
#: Preset :func:`euroc_imu_sensor` / :func:`to_euroc_sensor_yaml` write into ``imu0/sensor.yaml``
#: (dataset doc section 7 / 12.1; ``dataset.py`` ships the same numbers as ``PLACEHOLDER_IMU_NOISE``).
EUROC_NOISE_PRESET = "realsense"
_NOISE_KEYS: Tuple[str, ...] = ("gyroscope_noise_density", "gyroscope_random_walk",
                                "accelerometer_noise_density", "accelerometer_random_walk")
_NOISE_UNITS: Dict[str, str] = {
    "gyroscope_noise_density": "rad / s / sqrt(Hz)",
    "gyroscope_random_walk": "rad / s^2 / sqrt(Hz)",
    "accelerometer_noise_density": "m / s^2 / sqrt(Hz)",
    "accelerometer_random_walk": "m / s^3 / sqrt(Hz)",
    "frequency": "Hz",
}

_ROTATION_TOL = 1e-4  # EEPROM rotations are float32 and orthonormal to ~1e-7


class CalibrationError(ValueError):
    """The calibration data is missing, malformed or inconsistent."""


class CalibrationWarning(UserWarning):
    """Something about the calibration deserves a look but does not stop the build."""


def _warn(message: str, sink: Optional[List[str]] = None) -> None:
    """Emit a :class:`CalibrationWarning` and record it in *sink* (for ``rig_info['warnings']``).

    The warning is attributed to the first stack frame outside this module (the public entry
    point the caller used: ``build_rig``, ``distortion_for``, ...), however deep inside the
    module it was raised, so the printed location and ``warnings.filterwarnings(module=...)``
    refer to the caller (Python 3.10 has no ``skip_file_prefixes``; this walks the frames).
    """
    if sink is not None:
        sink.append(message)
    level = 2  # stacklevel=2 is the caller of _warn
    frame = sys._getframe(1)
    while frame is not None and frame.f_back is not None and frame.f_globals.get("__name__") == __name__:
        frame = frame.f_back
        level += 1
    warnings.warn(message, CalibrationWarning, stacklevel=level)


# --------------------------------------------------------------------------------------
# Sockets
# --------------------------------------------------------------------------------------

def socket_id(socket: Union[int, str, Any]) -> int:
    """Map ``'CAM_B'`` / ``'B'`` / ``1`` / ``dai.CameraBoardSocket.CAM_B`` to the integer socket id.

    ``-1`` (``AUTO``) is accepted because it is the EEPROM link target of the origin camera.
    """
    if isinstance(socket, bool):
        raise TypeError("socket must be an int, a name or a CameraBoardSocket, not bool")
    if isinstance(socket, (int, np.integer)):
        return int(socket)
    if isinstance(socket, str):
        name = socket.strip().upper()
        if name == "AUTO":
            return SOCKET_AUTO
        if name.lstrip("-").isdigit():  # JSON object keys are strings: {"1": {...}} means CAM_B
            return int(name)
        if not name.startswith("CAM_"):
            name = "CAM_" + name
        if name in _SOCKET_IDS:
            return _SOCKET_IDS[name]
        raise CalibrationError("Unknown camera socket name {!r} (expected CAM_A..CAM_H)".format(socket))
    value = getattr(socket, "value", None)  # dai.CameraBoardSocket (pybind enum)
    if isinstance(value, (int, np.integer)):
        return int(value)
    try:
        return int(socket)
    except (TypeError, ValueError):
        raise TypeError("Cannot interpret {!r} as a camera socket".format(socket)) from None


def socket_name(socket: Union[int, str, Any]) -> str:
    """``1 -> 'CAM_B'`` (inverse of :func:`socket_id`); ``-1 -> 'AUTO'``."""
    sid = socket_id(socket)
    if sid == SOCKET_AUTO:
        return "AUTO"
    if 0 <= sid < 8:
        return "CAM_{}".format(chr(ord("A") + sid))
    return "CAM_{}".format(sid)


# --------------------------------------------------------------------------------------
# Loading / normalising the EEPROM dump
# --------------------------------------------------------------------------------------

def normalize_calibration(obj: Mapping[str, Any], source: str = "<dict>") -> Dict[str, Any]:
    """Return an EEPROM dump dict in ``eepromToJson()`` layout, whatever wrapper it came in.

    Accepts the plain dump (top-level ``cameraData``), the holistic ``camera_info.json``
    (``{"calibration": <dump>, "camera_features": ..., "imu": ...}``) and a ``cameraData``
    given either as the ``[[socket, info], ...]`` list DepthAI writes or as a dict keyed by
    socket id/name.  The result is a shallow copy whose ``cameraData`` is always the list
    form with integer sockets, so ``dai.CalibrationHandler.fromJson(result)`` still works.
    Raises :class:`CalibrationError` for anything that is not an intrinsics-carrying dump.
    """
    if not isinstance(obj, Mapping):
        raise CalibrationError("{}: calibration must be a JSON object, got {}".format(source, type(obj).__name__))
    data: Mapping[str, Any] = obj
    if "cameraData" not in data:
        inner = data.get("calibration")
        if isinstance(inner, Mapping) and "cameraData" in inner:
            data = inner  # holistic recording camera_info.json
        else:
            hint = ""
            if isinstance(inner, Mapping):
                hint = (" (this looks like a session.json: its 'calibration' block only holds derived "
                        "intrinsics; load the calibration.json next to it)")
            raise CalibrationError("{}: not an EEPROM calibration dump, no 'cameraData' key{}".format(source, hint))

    result: Dict[str, Any] = dict(data)
    version = result.get("version")
    if version is not None:
        try:
            version = int(version)
        except (TypeError, ValueError):
            raise CalibrationError("{}: EEPROM version {!r} is not an integer".format(source, version)) from None
        if version < 4:
            raise CalibrationError("{}: EEPROM version {} has no intrinsics (needs >= 4)".format(source, version))

    cams = result["cameraData"]
    pairs: List[List[Any]] = []
    if isinstance(cams, Mapping):
        items = list(cams.items())
    elif isinstance(cams, (list, tuple)):
        items = []
        for entry in cams:
            if not (isinstance(entry, (list, tuple)) and len(entry) == 2):
                raise CalibrationError("{}: cameraData entries must be [socket, info] pairs".format(source))
            items.append((entry[0], entry[1]))
    else:
        raise CalibrationError("{}: cameraData must be a list of [socket, info] pairs or a dict".format(source))
    for key, info in items:
        sid = socket_id(key)
        if not isinstance(info, Mapping):
            raise CalibrationError("{}: camera {} info must be an object".format(source, socket_name(sid)))
        _validate_camera_info(info, socket_name(sid), source)
        pairs.append([sid, dict(info)])
    result["cameraData"] = pairs
    return result


def _validate_camera_info(info: Mapping[str, Any], name: str, source: str) -> None:
    for key in ("intrinsicMatrix", "width", "height", "distortionCoeff", "extrinsics"):
        if key not in info:
            raise CalibrationError("{}: camera {} lacks '{}'".format(source, name, key))
    K = info["intrinsicMatrix"]
    if len(K) != 3 or any(len(row) != 3 for row in K):
        raise CalibrationError("{}: camera {} intrinsicMatrix is not 3x3".format(source, name))
    if int(info["width"]) <= 0 or int(info["height"]) <= 0:
        raise CalibrationError("{}: camera {} has a non-positive calibration size".format(source, name))
    ext = info["extrinsics"]
    for key in ("rotationMatrix", "translation", "toCameraSocket"):
        if key not in ext:
            raise CalibrationError("{}: camera {} extrinsics lack '{}'".format(source, name, key))


def load_calibration_json(path: Union[str, Path]) -> Dict[str, Any]:
    """Load ``calibration.json`` (``eepromToJson()`` dump) or a holistic ``camera_info.json``.

    Given a ``session.json`` (which only carries derived intrinsics) the ``calibration.json``
    next to it is loaded instead.  Returns the normalised dump (see
    :func:`normalize_calibration`).  Raises ``FileNotFoundError`` / :class:`CalibrationError`.
    """
    file = Path(path)
    if not file.is_file():
        raise FileNotFoundError("calibration file not found: {}".format(file))
    with file.open("r", encoding="utf-8") as fh:
        try:
            obj = json.load(fh)
        except ValueError as exc:  # json.JSONDecodeError and UnicodeDecodeError are both ValueErrors
            raise CalibrationError("{}: not valid JSON ({}); a calibration.json truncated by an interrupted "
                                   "recording looks like this, re-dump it from the camera".format(file, exc)) from None
    if isinstance(obj, Mapping) and "cameraData" not in obj:
        inner = obj.get("calibration")
        if not (isinstance(inner, Mapping) and "cameraData" in inner):
            sibling = file.with_name("calibration.json")
            if sibling.is_file() and sibling.resolve() != file.resolve():
                return load_calibration_json(sibling)
    return normalize_calibration(obj, source=str(file))


def as_calibration(calib: Any) -> Dict[str, Any]:
    """Coerce a dict, a JSON path or a ``dai.CalibrationHandler`` into the normalised dump.

    A handler is recognised by its ``eepromToJson()`` method (duck-typed: no depthai import).
    """
    if isinstance(calib, Mapping):
        return normalize_calibration(calib)
    if isinstance(calib, (str, Path)):
        return load_calibration_json(calib)
    to_json = getattr(calib, "eepromToJson", None)
    if callable(to_json):
        return normalize_calibration(to_json(), source=type(calib).__name__)
    raise TypeError("calib must be an EEPROM dict, a path to calibration.json or a dai.CalibrationHandler, "
                    "got {}".format(type(calib).__name__))


def camera_entry(calib: Any, socket: Union[int, str, Any]) -> Dict[str, Any]:
    """The EEPROM ``cameraData`` record of *socket* (raises :class:`CalibrationError` if absent)."""
    data = as_calibration(calib)
    sid = socket_id(socket)
    for key, info in data["cameraData"]:
        if key == sid:
            return info
    have = ", ".join(socket_name(k) for k, _ in data["cameraData"]) or "none"
    raise CalibrationError("No calibration for {} (cameras in EEPROM: {})".format(socket_name(sid), have))


def camera_sockets(calib: Any) -> List[str]:
    """Names of all calibrated cameras, EEPROM order (``['CAM_A', 'CAM_C', 'CAM_B']`` on this unit)."""
    return [socket_name(k) for k, _ in as_calibration(calib)["cameraData"]]


def stereo_sockets(calib: Any) -> Tuple[str, str]:
    """``(left, right)`` stereo sockets from ``stereoRectificationData`` (``('CAM_B', 'CAM_C')``)."""
    data = as_calibration(calib)
    rect = data.get("stereoRectificationData") or {}
    left = rect.get("leftCameraSocket", _SOCKET_IDS[CAM_B])
    right = rect.get("rightCameraSocket", _SOCKET_IDS[CAM_C])
    return socket_name(left), socket_name(right)


def native_size(calib: Any, socket: Union[int, str, Any]) -> Tuple[int, int]:
    """``(width, height)`` the camera was calibrated at (1280x800 for the S2 mono cameras)."""
    info = camera_entry(calib, socket)
    return int(info["width"]), int(info["height"])


# --------------------------------------------------------------------------------------
# Intrinsics and distortion (DepthAI getCameraIntrinsics / getDistortionCoefficients port)
# --------------------------------------------------------------------------------------

def intrinsics_for(calib: Any, socket: Union[int, str, Any], width: Optional[int], height: Optional[int],
                   *, keep_aspect_ratio: bool = True,
                   crop: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None) -> np.ndarray:
    """3x3 camera matrix at ``width x height``, bit-for-bit as ``getCameraIntrinsics`` computes it.

    ``None``/``-1`` for both sizes returns the native matrix; one of them may be ``-1`` and is
    derived from the native aspect ratio.  ``keep_aspect_ratio=True`` (DepthAI default) is a
    uniform scale-to-fill plus centre crop (``ImgResizeMode.CROP``): 1280x720 shifts ``cy`` by
    -40 px; ``False`` stretches x and y independently.  640x400 and 320x200 are exact
    halvings/quarterings of the 1280x800 calibration.  *crop* is
    ``((x0, y0), (x1, y1))`` in resized pixels, applied after the resize.  Arithmetic is done
    in float32 like DepthAI, the result is returned as float64.  Distortion coefficients are
    scale-invariant and unaffected.
    """
    info = camera_entry(calib, socket)
    f32 = np.float32
    K = np.array(info["intrinsicMatrix"], dtype=np.float32)
    W, H = int(info["width"]), int(info["height"])
    w = -1 if width is None else int(width)
    h = -1 if height is None else int(height)
    if (w != -1 and w <= 0) or (h != -1 and h <= 0):
        raise CalibrationError("intrinsics_for: size must be positive or -1, got {}x{}".format(width, height))

    if w != -1 or h != -1:
        if w == -1:
            w = int(f32(W * h) / f32(H))
        if h == -1:
            h = int(f32(H * w) / f32(W))
        if keep_aspect_ratio:
            original_ratio = f32(W) / f32(H)
            resize_ratio = f32(w) / f32(h)
            if resize_ratio <= f32(1.34) and original_ratio <= f32(1.778) and original_ratio > f32(1.5):
                # 4:3-ish request from a 16:9/16:10 sensor: scale by the smaller factor and pad.
                scale = min(f32(w) / f32(W), f32(h) / f32(H))
                K = np.diag([scale, scale, f32(1)]).astype(np.float32) @ K
                if scale * f32(H) < f32(h):
                    K[1, 2] += f32(f32(h) - f32(H) * scale) / f32(2)
                elif scale * f32(W) > f32(w):
                    K[0, 2] += f32(f32(w) - f32(W) * scale) / f32(2)
            else:
                scale = f32(h) / f32(H)
                if scale * f32(W) < f32(w):
                    scale = f32(w) / f32(W)
                K = np.diag([scale, scale, f32(1)]).astype(np.float32) @ K
                if scale * f32(H) > f32(h):
                    K[1, 2] -= (f32(H) * scale - f32(h)) / f32(2)
                elif scale * f32(W) > f32(w):
                    K[0, 2] -= (f32(W) * scale - f32(w)) / f32(2)
        else:
            sx = f32(w) / f32(W)
            sy = f32(h) / f32(H)
            K = np.diag([sx, sy, f32(1)]).astype(np.float32) @ K

    if crop is not None:
        (x0, y0), (x1, y1) = crop
        limit_w, limit_h = (w, h) if (w != -1 or h != -1) else (W, H)
        if y1 > limit_h or x1 > limit_w:
            raise CalibrationError("Invalid crop size: crop exceeds the resized image {}x{}".format(limit_w, limit_h))
        if x0 > x1 or y0 > y1:
            raise CalibrationError("Invalid crop ratio: top-left must not exceed bottom-right")
        K[0, 2] -= f32(x0)
        K[1, 2] -= f32(y0)
    return K.astype(np.float64)


def distortion_coefficients(calib: Any, socket: Union[int, str, Any]) -> List[float]:
    """All stored coefficients like ``getDistortionCoefficients``: 14 OpenCV values for a
    Perspective camera (``[k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,tx,ty]``), 4 for a Fisheye one."""
    info = camera_entry(calib, socket)
    coeffs = [float(c) for c in info["distortionCoeff"]]
    if int(info.get("cameraType", CAMERA_MODEL_PERSPECTIVE)) == CAMERA_MODEL_FISHEYE:
        return coeffs[:FISHEYE_COEFFICIENTS]
    if len(coeffs) < len(DISTORTION_NAMES):
        coeffs = coeffs + [0.0] * (len(DISTORTION_NAMES) - len(coeffs))
    return coeffs[:len(DISTORTION_NAMES)]


def distortion_model_for(calib: Any, socket: Union[int, str, Any]) -> str:
    """cuVSLAM ``Distortion.Model`` name for raw frames of *socket*: ``'Polynomial'`` for the
    Perspective (rational) EEPROM model, ``'Fisheye'`` for a fisheye calibration."""
    info = camera_entry(calib, socket)
    model = int(info.get("cameraType", CAMERA_MODEL_PERSPECTIVE))
    if model == CAMERA_MODEL_PERSPECTIVE:
        return "Polynomial"
    if model == CAMERA_MODEL_FISHEYE:
        return "Fisheye"
    raise CalibrationError("{}: unsupported DepthAI camera model {} (only Perspective/Fisheye)".format(
        socket_name(socket), model))


def distortion_for(calib: Any, socket: Union[int, str, Any], *, count: int = POLYNOMIAL_COEFFICIENTS,
                   warn: bool = True, warnings_sink: Optional[List[str]] = None) -> List[float]:
    """The coefficients cuVSLAM gets for raw frames: first 8 OpenCV values ``(k1 k2 p1 p2 k3 k4 k5 k6)``
    = ``Distortion.Model.Polynomial`` (same order on both sides, no reordering), exactly the
    official OAK-D example's ``[:8]`` slice.

    Warns (:class:`CalibrationWarning`) when any of the dropped coefficients 9-14 (thin prism
    ``s1..s4``, tilt ``tx, ty``) is non-zero, naming them: on this unit the tilt terms are
    non-zero and their omission mis-projects up to ~3.3 px on CAM_B at 1280x800 (calibration
    doc section 2.2), which is why the dataset path undistorts with all 14 instead.  For a
    Fisheye calibration the 4 equidistant coefficients are returned unchanged.
    """
    coeffs = distortion_coefficients(calib, socket)
    if distortion_model_for(calib, socket) == "Fisheye":
        return coeffs
    if not 0 < count <= len(DISTORTION_NAMES):
        raise CalibrationError("distortion_for: count must be 1..14, got {}".format(count))
    kept, dropped = coeffs[:count], coeffs[count:]
    nonzero = [(DISTORTION_NAMES[count + i], v) for i, v in enumerate(dropped) if v != 0.0]
    if nonzero and warn:
        _warn("{}: dropping non-zero distortion coefficients beyond the first {}: {} "
              "(cuVSLAM Polynomial has no thin-prism/tilt terms; expect up to a few px of "
              "mis-projection at the frame edges, or undistort on the host with all 14)".format(
                  socket_name(socket), count,
                  ", ".join("{}={:.6g}".format(n, v) for n, v in nonzero)), warnings_sink)
    return kept


# --------------------------------------------------------------------------------------
# Rotations, quaternions, SE(3)
# --------------------------------------------------------------------------------------

def check_rotation(R: Any, tol: float = _ROTATION_TOL, what: str = "rotation") -> np.ndarray:
    """Return *R* as a float64 3x3 after checking it is a proper rotation (orthonormal, det +1)."""
    M = np.asarray(R, dtype=np.float64)
    if M.shape != (3, 3):
        raise CalibrationError("{}: expected a 3x3 matrix, got shape {}".format(what, M.shape))
    if not np.all(np.isfinite(M)):
        raise CalibrationError("{}: non-finite entries".format(what))
    ortho_err = float(np.abs(M @ M.T - np.eye(3)).max())
    det = float(np.linalg.det(M))
    if ortho_err > tol or abs(det - 1.0) > tol:
        raise CalibrationError("{}: not a proper rotation (|R R^T - I| = {:.2e}, det = {:.6f}); "
                               "the EEPROM block is empty or corrupt".format(what, ortho_err, det))
    return M


def matrix_to_quaternion_xyzw(R: Any) -> np.ndarray:
    """3x3 rotation -> unit quaternion ``[x, y, z, w]`` (cuVSLAM ``Pose.rotation`` order, same as
    scipy ``Rotation.from_matrix(R).as_quat()``).  Shepperd's method, numerically safe for all
    rotations; the sign is chosen so that ``w >= 0``.  ``diag(1, -1, -1)`` gives ``[1, 0, 0, 0]``."""
    M = check_rotation(R)
    m00, m01, m02 = M[0]
    m10, m11, m12 = M[1]
    m20, m21, m22 = M[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0  # 4 w
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0  # 4 x
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0  # 4 y
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0  # 4 z
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    if q[3] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def quaternion_xyzw_to_matrix(q: Any) -> np.ndarray:
    """Unit quaternion ``[x, y, z, w]`` -> 3x3 rotation matrix (inverse of :func:`matrix_to_quaternion_xyzw`)."""
    v = np.asarray(q, dtype=np.float64).reshape(-1)
    if v.shape != (4,):
        raise CalibrationError("quaternion must have 4 elements (x, y, z, w), got {}".format(v.shape))
    n = float(np.linalg.norm(v))
    if n == 0.0 or not math.isfinite(n):
        raise CalibrationError("quaternion has zero or non-finite norm")
    x, y, z, w = v / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def se3(R: Any, t: Any) -> np.ndarray:
    """Assemble a 4x4 SE(3) matrix from a 3x3 rotation and a 3-vector."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_se3(T: Any) -> np.ndarray:
    """Exact inverse of a 4x4 SE(3) matrix (``R^T``, ``-R^T t``), like DepthAI's ``invertSe3Matrix4x4InPlace``."""
    M = np.asarray(T, dtype=np.float64)
    if M.shape != (4, 4):
        raise CalibrationError("invert_se3 expects a 4x4 matrix, got {}".format(M.shape))
    Rt = M[:3, :3].T
    return se3(Rt, -Rt @ M[:3, 3])


def pose_from_matrix(T: Any, vslam: Any = None) -> Any:
    """4x4 metres -> ``vslam.Pose(rotation=[x,y,z,w], translation=[tx,ty,tz])`` (no inversion)."""
    module = _resolve_vslam(vslam)
    M = np.asarray(T, dtype=np.float64)
    if M.shape != (4, 4):
        raise CalibrationError("pose_from_matrix expects a 4x4 matrix, got {}".format(M.shape))
    q = matrix_to_quaternion_xyzw(M[:3, :3])
    return module.Pose(rotation=[float(v) for v in q], translation=[float(v) for v in M[:3, 3]])


def matrix_from_pose(pose: Any) -> np.ndarray:
    """``vslam.Pose`` (or anything with ``.rotation`` xyzw and ``.translation``) -> 4x4 float64."""
    return se3(quaternion_xyzw_to_matrix(np.asarray(pose.rotation, dtype=np.float64)),
               np.asarray(pose.translation, dtype=np.float64))


# --------------------------------------------------------------------------------------
# Extrinsics (DepthAI getCameraExtrinsics / getImuToCameraExtrinsics port, metres)
# --------------------------------------------------------------------------------------

def _translation_m(block: Mapping[str, Any], use_spec: bool, what: str) -> np.ndarray:
    """Translation of an EEPROM extrinsics block converted to metres via its ``lengthUnit``."""
    key = "specTranslation" if use_spec else "translation"
    tr = block.get(key)
    if not isinstance(tr, Mapping):
        raise CalibrationError("{}: extrinsics lack '{}'".format(what, key))
    t = np.array([float(tr.get("x", 0.0)), float(tr.get("y", 0.0)), float(tr.get("z", 0.0))], dtype=np.float64)
    if use_spec and not np.any(t):
        # DepthAI: "Cannot use useSpecTranslation argument since specTranslation has {0, 0, 0}"
        raise CalibrationError("{}: specTranslation is (0, 0, 0); use the calibrated translation".format(what))
    unit = int(block.get("lengthUnit", _DEFAULT_LENGTH_UNIT))
    if unit not in LENGTH_UNIT_TO_METRES:
        raise CalibrationError("{}: unsupported lengthUnit {} (METER=0 .. FOOT=4)".format(what, unit))
    return t * LENGTH_UNIT_TO_METRES[unit]


def _link_matrix(info: Mapping[str, Any], use_spec: bool, what: str) -> np.ndarray:
    """``toCameraSocket_from_camera`` of one EEPROM link, translation in metres."""
    ext = info["extrinsics"]
    R = check_rotation(ext["rotationMatrix"], what="{} extrinsics rotation".format(what))
    return se3(R, _translation_m(ext, use_spec, "{} extrinsics".format(what)))


def _extrinsics_to_origin(data: Dict[str, Any], socket: int, use_spec: bool) -> Tuple[np.ndarray, int]:
    """``origin_from_socket`` (metres) by chaining EEPROM links, and the origin socket id
    (the camera whose link target is AUTO), like DepthAI ``getExtrinsicsToOrigin``."""
    path = [socket]
    current = socket
    while True:
        info = camera_entry(data, current)
        nxt = socket_id(info["extrinsics"]["toCameraSocket"])
        if nxt == SOCKET_AUTO:
            break
        if nxt in path:
            raise CalibrationError("Cyclic extrinsics chain in the EEPROM: {}".format(
                " -> ".join(socket_name(s) for s in path + [nxt])))
        path.append(nxt)
        current = nxt
    origin = current
    T = np.eye(4, dtype=np.float64)
    for s in path[:-1]:  # origin_from_s0 = link(s_{n-1}) @ ... @ link(s0)
        T = _link_matrix(camera_entry(data, s), use_spec, socket_name(s)) @ T
    return T, origin


def camera_extrinsics(calib: Any, src_socket: Union[int, str, Any], dst_socket: Union[int, str, Any],
                      *, use_spec_translation: bool = False) -> np.ndarray:
    """``dst_from_src`` in **metres**: DepthAI ``getCameraExtrinsics(src, dst, useSpecTranslation,
    LengthUnit.METER)`` (maps points expressed in *src* into *dst*; translation = origin of
    *src* expressed in *dst*).  ``camera_extrinsics(calib, CAM_B, CAM_A)`` puts the left
    camera 3.7 cm to the left (``t.x = -0.0374``) of the colour camera on this unit."""
    data = as_calibration(calib)
    src, dst = socket_id(src_socket), socket_id(dst_socket)
    camera_entry(data, src)
    camera_entry(data, dst)
    if src == dst:
        return np.eye(4, dtype=np.float64)
    T_os, origin_s = _extrinsics_to_origin(data, src, use_spec_translation)
    T_od, origin_d = _extrinsics_to_origin(data, dst, use_spec_translation)
    if origin_s != origin_d:
        raise CalibrationError("Missing extrinsic link between {} and {} (different origins {} / {})".format(
            socket_name(src), socket_name(dst), socket_name(origin_s), socket_name(origin_d)))
    return invert_se3(T_od) @ T_os


def rig_from_camera(calib: Any, socket: Union[int, str, Any], rig_socket: Union[int, str, Any] = DEFAULT_RIG_SOCKET,
                    *, use_spec_translation: bool = False) -> np.ndarray:
    """cuVSLAM ``Camera.rig_from_camera`` as a 4x4 in metres: ``getCameraExtrinsics(socket, rig_socket)``
    verbatim (cm -> m), **no inversion, no transpose**.  With the default rig = CAM_A this is what
    NVIDIA's ``run_stereo.py`` feeds; with ``rig_socket=CAM_B`` the left camera is the identity
    and the right one sits at ``+0.075 m`` along x (EuRoC ``T_BS`` with body = cam0)."""
    return camera_extrinsics(calib, socket, rig_socket, use_spec_translation=use_spec_translation)


def imu_extrinsics_raw(calib: Any) -> Dict[str, Any]:
    """The EEPROM ``imuExtrinsics`` block (rotation = ``toCameraSocket_from_imu``, translation in
    cm); raises :class:`CalibrationError` when the device has no IMU calibration."""
    data = as_calibration(calib)
    blk = data.get("imuExtrinsics")
    if not isinstance(blk, Mapping) or not blk.get("rotationMatrix") \
            or socket_id(blk.get("toCameraSocket", SOCKET_AUTO)) == SOCKET_AUTO:
        raise CalibrationError("IMU calibration data is not available in this EEPROM dump")
    return dict(blk)


def has_imu_extrinsics(calib: Any) -> bool:
    """``True`` when the EEPROM carries a usable ``imuExtrinsics`` block (a proper rotation, a
    finite translation and a calibrated ``toCameraSocket``), i.e. what mode ``'vio'`` and the EuRoC
    ``imu0/sensor.yaml`` need.  ``False`` for a dump without one, such as an OAK-D Lite whose
    calibration lacks IMU extrinsics: use mode ``'stereo'`` there.  A *calib* that is not a
    calibration dump at all still raises :class:`CalibrationError`."""
    data = as_calibration(calib)
    try:
        blk = imu_extrinsics_raw(data)
        rig_from_imu(data, blk["toCameraSocket"])
    except CalibrationError:
        return False
    return True


def rig_from_imu(calib: Any, rig_socket: Union[int, str, Any] = DEFAULT_RIG_SOCKET, *,
                 imu_frame: str = "raw", use_spec_translation: bool = False) -> np.ndarray:
    """cuVSLAM ``ImuCalibration.rig_from_imu`` as a 4x4 in metres = DepthAI
    ``getImuToCameraExtrinsics(rig_socket, useSpecTranslation, METER)`` = ``rig_from_imu``.

    ``imu_frame="raw"`` (``*_RAW`` streams, sensor-native axes): the EEPROM rotation
    (``diag(1, -1, -1)`` on this unit, i.e. IMU x = cam x, y = cam -y, z = cam -z) is kept.
    ``imu_frame="aligned"`` (``*_UNCALIBRATED``/``*_CALIBRATED``: the device already rotated
    the samples into the ``toCameraSocket`` (CAM_A) frame): the rotation is replaced by the
    identity and only the lever arm remains; for a rig other than CAM_A the CAM_A -> rig
    rotation is still applied.  Worked value for rig = CAM_A on this unit:
    ``[[1,0,0,-0.037948],[0,-1,0,-0.00079],[0,0,-1,0],[0,0,0,1]]``.
    """
    if imu_frame not in IMU_FRAMES:
        raise CalibrationError("imu_frame must be one of {}, got {!r}".format(IMU_FRAMES, imu_frame))
    data = as_calibration(calib)
    blk = imu_extrinsics_raw(data)
    R = check_rotation(blk["rotationMatrix"], what="imuExtrinsics rotation")
    if imu_frame == "aligned":
        R = np.eye(3, dtype=np.float64)
    T_cam_from_imu = se3(R, _translation_m(blk, use_spec_translation, "imuExtrinsics"))
    to_socket = socket_id(blk["toCameraSocket"])
    rig = socket_id(rig_socket)
    if to_socket == rig:
        return T_cam_from_imu
    T_rig_from_to = camera_extrinsics(data, to_socket, rig, use_spec_translation=use_spec_translation)
    return T_rig_from_to @ T_cam_from_imu


def stereo_baseline_m(calib: Any, left: Union[int, str, Any] = CAM_B, right: Union[int, str, Any] = CAM_C,
                      *, use_spec_translation: bool = False) -> float:
    """Distance between the two optical centres in metres (0.074967 calibrated / 0.075 spec here)."""
    T = camera_extrinsics(calib, right, left, use_spec_translation=use_spec_translation)
    return float(np.linalg.norm(T[:3, 3]))


# --------------------------------------------------------------------------------------
# IMU noise
# --------------------------------------------------------------------------------------

def imu_noise(calib: Any = None, *, frequency_hz: float = 200.0, preset: str = DEFAULT_NOISE_PRESET,
              overrides: Optional[Mapping[str, float]] = None) -> Dict[str, Any]:
    """``ImuCalibration`` noise fields with provenance.

    Returns ``gyroscope_noise_density``, ``gyroscope_random_walk``, ``accelerometer_noise_density``,
    ``accelerometer_random_walk`` (cuVSLAM/Kalibr units), ``frequency`` (Hz, the rate requested
    from ``enableIMUSensor``), ``source`` (where the numbers come from), ``preset``, ``units``,
    and, when *calib* is given, ``eeprom`` = the verbatim ``imuCalibrationParams.noise`` block
    with ``eeprom_units = "undocumented"`` (kept for the record, never fed to cuVSLAM: taken
    literally the gyro values would be ~1000x a MEMS gyro).  *overrides* replaces individual
    noise values (e.g. after an Allan-variance measurement) and is recorded in ``source``.
    """
    if preset not in NOISE_PRESETS:
        raise CalibrationError("Unknown IMU noise preset {!r}; choose one of {}".format(preset, sorted(NOISE_PRESETS)))
    if isinstance(frequency_hz, bool) or not (isinstance(frequency_hz, (int, float, np.integer, np.floating))
                                              and math.isfinite(frequency_hz) and frequency_hz > 0):
        raise CalibrationError("frequency_hz must be a positive number, got {!r}".format(frequency_hz))
    base = NOISE_PRESETS[preset]
    noise: Dict[str, Any] = {key: float(base[key]) for key in _NOISE_KEYS}
    noise["frequency"] = float(frequency_hz)
    source = str(base["source"])
    if overrides:
        applied = []
        for key, value in overrides.items():
            if key not in _NOISE_KEYS:
                raise CalibrationError("imu_noise override {!r} is not one of {}".format(key, _NOISE_KEYS))
            fval = float(value)
            if not (math.isfinite(fval) and fval > 0):
                raise CalibrationError("imu_noise override {} must be a positive finite number".format(key))
            noise[key] = fval
            applied.append("{}={:.4g}".format(key, fval))
        source += "; overrides: " + ", ".join(applied)
    noise["source"] = source
    noise["preset"] = preset
    noise["units"] = dict(_NOISE_UNITS)
    noise["eeprom"] = None
    noise["eeprom_units"] = "undocumented (methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md section 5.1); not used"
    if calib is not None:
        data = as_calibration(calib)
        block = (data.get("imuCalibrationParams") or {}).get("noise")
        if isinstance(block, Mapping):
            noise["eeprom"] = copy.deepcopy(dict(block))
            noise["imu_name"] = block.get("name")
    return noise


# --------------------------------------------------------------------------------------
# Borders
# --------------------------------------------------------------------------------------

def default_borders(size: Sequence[int], *, undistorted: bool = False) -> Dict[str, int]:
    """Pixels to ignore for feature selection: the official example's 50/0/70/70 scaled to
    *size* for raw (distorted) frames (``top = 50*h//800``, ``left = right = 70*w//1280``), all
    zero for host-undistorted frames (no highly distorted edges left to avoid)."""
    w, h = _check_size(size)
    if undistorted:
        return {key: 0 for key in _BORDER_KEYS}
    return {
        "top": DEFAULT_BORDERS["top"] * h // DEFAULT_SIZE[1],
        "bottom": DEFAULT_BORDERS["bottom"] * h // DEFAULT_SIZE[1],
        "left": DEFAULT_BORDERS["left"] * w // DEFAULT_SIZE[0],
        "right": DEFAULT_BORDERS["right"] * w // DEFAULT_SIZE[0],
    }


def normalize_borders(borders: Any, size: Sequence[int], *, undistorted: bool = False) -> Dict[str, int]:
    """Accept ``None`` (defaults), ``0``/``"none"`` (all zero), a mapping with any of
    top/bottom/left/right, or a 4-sequence ``(top, bottom, left, right)``; validate against *size*."""
    w, h = _check_size(size)
    zeros = {key: 0 for key in _BORDER_KEYS}
    bad = CalibrationError("borders must be None, 0, 'none', a dict or a (top, bottom, left, right) sequence, "
                           "got {!r}".format(borders))
    if borders is None:
        result = default_borders(size, undistorted=undistorted)
    elif isinstance(borders, str):
        if borders.strip().lower() in ("none", "0"):
            result = zeros
        else:
            raise bad
    elif isinstance(borders, (int, float, np.integer, np.floating)) and not isinstance(borders, bool):
        if borders == 0:  # a scalar is only meaningful as "no borders"
            result = zeros
        else:
            raise bad
    elif isinstance(borders, Mapping):
        unknown = set(borders) - set(_BORDER_KEYS)
        if unknown:
            raise CalibrationError("Unknown border keys {}; use {}".format(sorted(unknown), _BORDER_KEYS))
        try:
            result = {key: int(borders.get(key, 0)) for key in _BORDER_KEYS}
        except (TypeError, ValueError):
            raise CalibrationError("border values must be integers, got {!r}".format(dict(borders))) from None
    elif isinstance(borders, (list, tuple, np.ndarray)):
        values = list(np.asarray(borders).reshape(-1)) if isinstance(borders, np.ndarray) else list(borders)
        if len(values) != 4:
            raise bad
        try:
            result = {key: int(v) for key, v in zip(_BORDER_KEYS, values)}
        except (TypeError, ValueError):
            raise CalibrationError("border values must be integers, got {!r}".format(values)) from None
    else:
        raise bad
    if any(v < 0 for v in result.values()):
        raise CalibrationError("borders must be non-negative: {}".format(result))
    if result["top"] + result["bottom"] >= h or result["left"] + result["right"] >= w:
        raise CalibrationError("borders {} leave no image at {}x{}".format(result, w, h))
    return result


def _check_size(size: Sequence[int]) -> Tuple[int, int]:
    try:
        if isinstance(size, (str, bytes)) or any(isinstance(v, bool) for v in size[:2]):
            raise TypeError("size must not be a string or contain bools")
        w, h = int(size[0]), int(size[1])
    except (TypeError, IndexError, ValueError):
        raise CalibrationError("size must be (width, height), got {!r}".format(size)) from None
    if len(size) != 2 or w <= 0 or h <= 0:
        raise CalibrationError("size must be two positive ints (width, height), got {!r}".format(size))
    return w, h


# --------------------------------------------------------------------------------------
# Pure-numpy rig description (no cuvslam needed)
# --------------------------------------------------------------------------------------

def camera_spec(calib: Any, socket: Union[int, str, Any], size: Sequence[int], *,
                rig_socket: Union[int, str, Any] = DEFAULT_RIG_SOCKET, undistorted: bool = False,
                borders: Any = None, use_spec_translation: bool = False,
                warnings_sink: Optional[List[str]] = None) -> Dict[str, Any]:
    """JSON-serialisable description of one cuVSLAM ``Camera`` (what :func:`rig_from_spec` builds).

    ``undistorted=True`` means the frames were undistorted on the host with all 14 EEPROM
    coefficients and ``newCameraMatrix = K`` (dataset path): the model becomes ``Pinhole`` with no
    coefficients while the intrinsics stay the EEPROM values; ``False`` (live raw GRAY8 frames)
    gives ``Polynomial`` with the first 8 coefficients and the example's border masks.
    """
    w, h = _check_size(size)
    K = intrinsics_for(calib, socket, w, h)
    T = rig_from_camera(calib, socket, rig_socket, use_spec_translation=use_spec_translation)
    q = matrix_to_quaternion_xyzw(T[:3, :3])
    full = distortion_coefficients(calib, socket)
    if undistorted:
        model, params = "Pinhole", []
    else:
        model = distortion_model_for(calib, socket)
        params = distortion_for(calib, socket, warnings_sink=warnings_sink)
    W, H = native_size(calib, socket)
    return {
        "socket": socket_name(socket),
        "socket_id": socket_id(socket),
        "size": [w, h],
        "native_size": [W, H],
        "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
        "intrinsics": [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],  # EuRoC fu fv cu cv
        "K": K.tolist(),
        "undistorted": bool(undistorted),
        "distortion_model": model,
        "distortion": [float(p) for p in params],
        "distortion_full": full,
        "rig_socket": socket_name(rig_socket),
        "rig_from_camera": T.tolist(),
        "rig_from_camera_quaternion_xyzw": [float(v) for v in q],
        "rig_from_camera_translation_m": [float(v) for v in T[:3, 3]],
        "borders": normalize_borders(borders, (w, h), undistorted=undistorted),
    }


def rig_spec(calib: Any, size: Sequence[int] = DEFAULT_SIZE, mode: str = "stereo", imu_rate_hz: float = 200.0,
             borders: Any = None, *, rig_socket: Union[int, str, Any] = DEFAULT_RIG_SOCKET,
             left: Optional[Union[int, str, Any]] = None, right: Optional[Union[int, str, Any]] = None,
             undistorted: bool = False, imu_frame: str = "raw", noise: Optional[Mapping[str, Any]] = None,
             noise_preset: str = DEFAULT_NOISE_PRESET, use_spec_translation: bool = False) -> Dict[str, Any]:
    """Everything :func:`build_rig` needs, as a JSON-serialisable dict (``rig_info``), computed
    with numpy only.  Camera order is ``[left, right]`` = cuVSLAM camera index 0 and 1.

    ``mode``: ``'stereo'`` (no IMU in the rig, ``OdometryMode.Multicamera``, mirrors the official
    OAK-D example) or ``'vio'`` (one ``ImuCalibration``, ``OdometryMode.Inertial``).
    ``imu_rate_hz`` is the rate requested from ``enableIMUSensor`` (200 or 400).
    ``noise``: a dict from :func:`imu_noise` (or the same keys) to override the preset.
    ``warnings`` collects every :class:`CalibrationWarning` raised while building.

    The ``imu`` block always carries ``included`` (an ``ImuCalibration`` goes into the rig: only
    in ``'vio'``) and ``available`` (the EEPROM has usable IMU extrinsics).  A calibration without
    them (OAK-D Lite style) builds in ``'stereo'`` mode with ``available = False`` and the
    reason in ``imu['error']`` (the extrinsics fields are ``None``); in ``'vio'`` mode it raises
    :class:`CalibrationError`.
    """
    if mode not in MODES:
        raise CalibrationError("mode must be one of {}, got {!r}".format(MODES, mode))
    if imu_frame not in IMU_FRAMES:
        raise CalibrationError("imu_frame must be one of {}, got {!r}".format(IMU_FRAMES, imu_frame))
    data = as_calibration(calib)
    w, h = _check_size(size)
    sink: List[str] = []
    default_left, default_right = stereo_sockets(data)
    left_name = socket_name(default_left if left is None else left)
    right_name = socket_name(default_right if right is None else right)
    if left_name == right_name:
        raise CalibrationError("left and right must be different sockets")
    rig_name = socket_name(rig_socket)
    camera_entry(data, rig_name)  # the rig frame must be a calibrated camera

    cams = [camera_spec(data, s, (w, h), rig_socket=rig_name, undistorted=undistorted, borders=borders,
                        use_spec_translation=use_spec_translation, warnings_sink=sink)
            for s in (left_name, right_name)]
    if cams[0]["native_size"] != cams[1]["native_size"]:
        raise CalibrationError("left/right native calibration sizes differ: {} vs {}".format(
            cams[0]["native_size"], cams[1]["native_size"]))
    W, H = cams[0]["native_size"]
    if (w, h) not in SUPPORTED_SIZES:
        if abs(w / h - W / H) > 1e-3:
            _warn("requested size {}x{} does not keep the native {}x{} aspect: getCameraIntrinsics "
                  "crops (centre) and shifts the principal point (1280x720: cy - 40 px); request "
                  "1280x800 or 640x400 instead".format(w, h, W, H), sink)
        else:
            _warn("requested size {}x{} is not one of the verified sizes {}".format(w, h, SUPPORTED_SIZES), sink)

    t_left = np.array(cams[0]["rig_from_camera_translation_m"])
    t_right = np.array(cams[1]["rig_from_camera_translation_m"])
    delta = t_right - t_left
    baseline = float(np.linalg.norm(delta))
    direction = (delta / baseline).tolist() if baseline > 0 else [0.0, 0.0, 0.0]
    try:
        baseline_spec = stereo_baseline_m(data, left_name, right_name, use_spec_translation=True)
    except CalibrationError:
        baseline_spec = None
    if baseline <= 0 or direction[0] < 0.99:
        _warn("right camera is not at +x of the left camera in the rig frame (right - left = {}); "
              "check the extrinsics direction (must be dst_from_src, never inverted)".format(
                  np.round(delta, 6).tolist()), sink)
    if baseline_spec and abs(baseline - baseline_spec) > 0.05 * baseline_spec:
        _warn("calibrated baseline {:.4f} m differs from the spec baseline {:.4f} m by more than 5 %".format(
            baseline, baseline_spec), sink)

    imu_info: Dict[str, Any] = {
        "included": mode == "vio",
        "available": False,
        "frame": imu_frame,
        "streams": list(_IMU_STREAMS[imu_frame]),
        "to_camera_socket": None,
        "eeprom_rotation": None,
        "rig_from_imu": None,
        "rig_from_imu_quaternion_xyzw": None,
        "rig_from_imu_translation_m": None,
    }
    try:
        imu_block = imu_extrinsics_raw(data)
        T_imu = rig_from_imu(data, rig_name, imu_frame=imu_frame, use_spec_translation=use_spec_translation)
    except CalibrationError as exc:
        if mode == "vio":
            raise CalibrationError("mode 'vio' needs the IMU extrinsics but {}; build mode 'stereo' "
                                   "(cameras only, OdometryMode.Multicamera) for this calibration".format(exc)) from None
        imu_info["error"] = str(exc)  # stereo mode never uses the IMU: record why it is absent and go on
    else:
        q_imu = matrix_to_quaternion_xyzw(T_imu[:3, :3])
        imu_info.update({
            "available": True,
            "to_camera_socket": socket_name(imu_block["toCameraSocket"]),
            "eeprom_rotation": [[float(v) for v in row] for row in imu_block["rotationMatrix"]],
            "rig_from_imu": T_imu.tolist(),
            "rig_from_imu_quaternion_xyzw": [float(v) for v in q_imu],
            "rig_from_imu_translation_m": [float(v) for v in T_imu[:3, 3]],
        })
    if noise is None:
        noise_dict = imu_noise(data, frequency_hz=imu_rate_hz, preset=noise_preset)
    else:
        noise_dict = imu_noise(data, frequency_hz=imu_rate_hz, preset=noise_preset,
                               overrides={k: noise[k] for k in _NOISE_KEYS if k in noise})
        if "source" in noise:
            noise_dict["source"] = str(noise["source"])
        if "preset" in noise:
            noise_dict["preset"] = str(noise["preset"])
    if imu_rate_hz not in (200, 200.0, 400, 400.0):
        _warn("imu_rate_hz {} is not 200 or 400 (the rates verified on this unit)".format(imu_rate_hz), sink)
    imu_info["rate_hz"] = float(imu_rate_hz)
    imu_info["noise"] = noise_dict

    return {
        "format": "cuvslam_oak.rig_info",
        "format_version": 1,
        "mode": mode,
        "odometry_mode": _ODOMETRY_MODE_NAME[mode],
        "size": [w, h],
        "rig_socket": rig_name,
        "rig_frame": "{} optical frame: x right, y down, z forward (OpenCV); metres".format(rig_name),
        "camera_order": [left_name, right_name],
        "cameras": cams,
        "stereo": {
            "left": left_name,
            "right": right_name,
            "baseline_m": baseline,
            "baseline_spec_m": baseline_spec,
            "right_minus_left_m": [float(v) for v in delta],
            "direction": [float(v) for v in direction],
        },
        "imu": imu_info,
        "calibration": {
            "product": data.get("productName"),
            "board": data.get("boardName"),
            "board_rev": data.get("boardRev"),
            "eeprom_version": data.get("version"),
            "batch_time": data.get("batchTime"),
            "cameras_in_eeprom": camera_sockets(data),
            "use_spec_translation": bool(use_spec_translation),
            "extrinsics_convention": "getCameraExtrinsics(src, dst) = dst_from_src, translation cm -> m, no inversion",
        },
        "warnings": sink,
    }


# --------------------------------------------------------------------------------------
# cuVSLAM objects
# --------------------------------------------------------------------------------------

def _resolve_vslam(vslam: Any = None) -> Any:
    """The cuvslam-compatible module: the argument, else ``cuvslam_oak.import_cuvslam()`` (real
    wheel or fake, chosen there), else a plain ``import cuvslam``.  Never imported at module load."""
    if vslam is not None:
        return vslam
    try:
        from . import import_cuvslam  # type: ignore[attr-defined]
    except ImportError:
        import_cuvslam = None
    if import_cuvslam is not None:
        return import_cuvslam()
    try:
        import cuvslam  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError("cuvslam is not importable here ({}); pass vslam=<module> explicitly "
                          "(the real wheel on WSL2/Jetson, cuvslam_oak.fake_cuvslam on Windows) or use "
                          "rig_spec()/to_euroc_sensor_yaml(), which need no cuvslam".format(exc)) from exc
    return cuvslam


def _camera_from_spec(spec: Mapping[str, Any], vslam: Any) -> Any:
    model = getattr(vslam.Distortion.Model, spec["distortion_model"])
    distortion = vslam.Distortion(model, [float(p) for p in spec["distortion"]])
    pose = vslam.Pose(rotation=[float(v) for v in spec["rig_from_camera_quaternion_xyzw"]],
                      translation=[float(v) for v in spec["rig_from_camera_translation_m"]])
    b = spec["borders"]
    return vslam.Camera(size=(int(spec["size"][0]), int(spec["size"][1])),
                        principal=(float(spec["cx"]), float(spec["cy"])),
                        focal=(float(spec["fx"]), float(spec["fy"])),
                        rig_from_camera=pose, distortion=distortion,
                        border_top=int(b["top"]), border_bottom=int(b["bottom"]),
                        border_left=int(b["left"]), border_right=int(b["right"]))


def _imu_from_spec(imu: Mapping[str, Any], vslam: Any) -> Any:
    noise = imu["noise"]
    pose = vslam.Pose(rotation=[float(v) for v in imu["rig_from_imu_quaternion_xyzw"]],
                      translation=[float(v) for v in imu["rig_from_imu_translation_m"]])
    return vslam.ImuCalibration(rig_from_imu=pose,
                                gyroscope_noise_density=float(noise["gyroscope_noise_density"]),
                                gyroscope_random_walk=float(noise["gyroscope_random_walk"]),
                                accelerometer_noise_density=float(noise["accelerometer_noise_density"]),
                                accelerometer_random_walk=float(noise["accelerometer_random_walk"]),
                                frequency=float(noise.get("frequency", imu["rate_hz"])))


def rig_from_spec(info: Mapping[str, Any], vslam: Any = None) -> Any:
    """Construct the ``vslam.Rig`` described by a :func:`rig_spec` dict (cameras in
    ``camera_order``; one IMU when ``info['imu']['included']``)."""
    module = _resolve_vslam(vslam)
    cameras = [_camera_from_spec(c, module) for c in info["cameras"]]
    imu = info["imu"]
    if imu["included"] and not imu.get("available", True):
        raise CalibrationError("rig_info asks for an IMU but has no extrinsics ({})".format(imu.get("error", "?")))
    imus = [_imu_from_spec(imu, module)] if imu["included"] else []
    return module.Rig(cameras=cameras, imus=imus)


def build_rig(calib: Any, size: Sequence[int] = DEFAULT_SIZE, mode: str = "stereo", imu_rate_hz: float = 200.0,
              borders: Any = None, *, vslam: Any = None, rig_socket: Union[int, str, Any] = DEFAULT_RIG_SOCKET,
              left: Optional[Union[int, str, Any]] = None, right: Optional[Union[int, str, Any]] = None,
              undistorted: bool = False, imu_frame: str = "raw", noise: Optional[Mapping[str, Any]] = None,
              noise_preset: str = DEFAULT_NOISE_PRESET, use_spec_translation: bool = False) -> Tuple[Any, Dict[str, Any]]:
    """Build ``(vslam.Rig, rig_info)`` for the OAK-D S2.

    *calib*: EEPROM dict / ``calibration.json`` path / ``dai.CalibrationHandler``.
    *size*: ``(width, height)`` of the frames that will be tracked (1280x800 or 640x400).
    *mode*: ``'stereo'`` (cameras only; also the mode for a calibration without IMU extrinsics,
    see :func:`has_imu_extrinsics`) or ``'vio'`` (cameras + BNO086 ``ImuCalibration``; raises
    :class:`CalibrationError` without IMU extrinsics).
    *imu_rate_hz*: the IMU report rate (200 or 400).  *borders*: see :func:`normalize_borders`.
    *vslam*: the cuvslam module (real or fake); ``None`` resolves it lazily.
    Keyword extras: ``rig_socket`` (default CAM_A like the official example; CAM_B for the
    EuRoC/dataset frame), ``left``/``right`` sockets (default from the EEPROM stereo pair),
    ``undistorted`` (host-undistorted frames -> Pinhole, no borders), ``imu_frame``
    (``'raw'``/``'aligned'`` streams), ``noise``/``noise_preset`` (see :func:`imu_noise`),
    ``use_spec_translation`` (CAD instead of calibrated translations; default False like
    ``getCameraExtrinsics``).  Pair the rig with ``OdometryConfig(rectified_stereo_camera=False,
    odometry_mode=Multicamera|Inertial)``; ``rig_info['odometry_mode']`` names the mode.
    """
    info = rig_spec(calib, size, mode, imu_rate_hz, borders, rig_socket=rig_socket, left=left, right=right,
                    undistorted=undistorted, imu_frame=imu_frame, noise=noise, noise_preset=noise_preset,
                    use_spec_translation=use_spec_translation)
    return rig_from_spec(info, vslam=vslam), info


# --------------------------------------------------------------------------------------
# EuRoC sensor.yaml helpers (datasets/dataset-format.md sections 5-7)
# --------------------------------------------------------------------------------------

def _t_bs(T: np.ndarray) -> Dict[str, Any]:
    return {"cols": 4, "rows": 4, "data": [float(v) for v in np.asarray(T, dtype=np.float64).reshape(16)]}


def _rate(value: float) -> Union[int, float]:
    f = float(value)
    return int(f) if f.is_integer() else f


def euroc_camera_sensor(calib: Any, socket: Union[int, str, Any], size: Sequence[int], *,
                        body_socket: Union[int, str, Any] = DEFAULT_BODY_SOCKET, undistorted: bool = True,
                        fps: float = 30.0, comment: Optional[str] = None, role: Optional[str] = None,
                        use_spec_translation: bool = False) -> Dict[str, Any]:
    """The ``mav0/camN/sensor.yaml`` dict for *socket* (keys in the EuRoC order).

    ``T_BS`` = ``body_from_sensor`` = :func:`rig_from_camera` with ``rig_socket=body_socket``
    (identity for the body camera itself, ``B_from_C`` for cam1 when body = CAM_B).
    ``undistorted=True`` (the default and the only layout NVIDIA's loader reads correctly)
    writes ``radial-tangential`` with four zeros; ``False`` writes ``rational-polynomial`` with
    the 8 OpenCV coefficients, which only our own reader maps to ``Polynomial``.
    """
    w, h = _check_size(size)
    K = intrinsics_for(calib, socket, w, h)
    T = rig_from_camera(calib, socket, body_socket, use_spec_translation=use_spec_translation)
    name = socket_name(socket)
    if comment is None:
        product = as_calibration(calib).get("productName") or "OAK-D"
        comment = "{} {}{} {}x{} GRAY8 {}fps {}".format(
            product, name, " " + role if role else "", w, h, _rate(fps),
            "undistorted (host, 14 EEPROM coefficients, intrinsics unchanged)" if undistorted
            else "raw (rational-polynomial, first 8 OpenCV coefficients)")
    sensor: Dict[str, Any] = {
        "sensor_type": "camera",
        "comment": comment,
        "T_BS": _t_bs(T),
        "rate_hz": _rate(fps),
        "resolution": [w, h],
        "camera_model": "pinhole",
        "intrinsics": [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],
    }
    if undistorted:
        sensor["distortion_model"] = "radial-tangential"
        sensor["distortion_coefficients"] = [0.0, 0.0, 0.0, 0.0]
    else:
        sensor["distortion_model"] = "rational-polynomial"
        sensor["distortion_coefficients"] = distortion_for(calib, socket, warn=False)
    return sensor


#: Appended to the ``imu0/sensor.yaml`` comment when ``allow_missing_imu`` wrote an identity ``T_BS``.
MISSING_IMU_COMMENT = ("no IMU extrinsics in the EEPROM, T_BS is an identity placeholder: "
                       "stereo-only dataset, track it in Multicamera mode, never Inertial")


def euroc_imu_sensor(calib: Any, *, body_socket: Union[int, str, Any] = DEFAULT_BODY_SOCKET,
                     imu_rate_hz: float = 200.0, noise: Optional[Mapping[str, Any]] = None,
                     noise_preset: str = EUROC_NOISE_PRESET, imu_frame: str = "raw",
                     comment: Optional[str] = None, use_spec_translation: bool = False,
                     allow_missing_imu: bool = False) -> Dict[str, Any]:
    """The ``mav0/imu0/sensor.yaml`` dict: ``T_BS`` = :func:`rig_from_imu` with
    ``rig_socket=body_socket`` (``B_from_IMU`` for body = CAM_B), ``rate_hz`` and the four
    noise keys (from *noise*, else :func:`imu_noise` with *noise_preset*, default
    ``EUROC_NOISE_PRESET`` = the RealSense placeholder the dataset doc prescribes).

    NVIDIA's ``dataset_utils.get_rig`` reads this file unconditionally, so a calibration without
    IMU extrinsics raises :class:`CalibrationError` unless ``allow_missing_imu=True``: then
    ``T_BS`` is the identity, the comment ends with :data:`MISSING_IMU_COMMENT` and a
    :class:`CalibrationWarning` is emitted; such a dataset is for stereo-only tracking.
    """
    data = as_calibration(calib)
    placeholder = False
    try:
        T = rig_from_imu(data, body_socket, imu_frame=imu_frame, use_spec_translation=use_spec_translation)
    except CalibrationError as exc:
        if not allow_missing_imu:
            raise CalibrationError("EuRoC layout requires mav0/imu0/sensor.yaml (NVIDIA's dataset_utils.get_rig reads "
                                   "it unconditionally) but {}; pass allow_missing_imu=True to write an identity "
                                   "T_BS placeholder for a stereo-only dataset".format(exc)) from None
        T = np.eye(4, dtype=np.float64)
        placeholder = True
        _warn("imu0/sensor.yaml: {}; writing an identity T_BS placeholder, track this dataset in "
              "stereo-only (Multicamera) mode".format(exc))
    n = dict(noise) if noise is not None else imu_noise(data, frequency_hz=imu_rate_hz, preset=noise_preset)
    if comment is None:
        product = data.get("productName") or "OAK-D"
        block = ((data.get("imuCalibrationParams") or {}).get("noise") or {})
        comment = "{} {} {} {}Hz; noise: {}".format(
            product, block.get("name", "IMU"), "+".join(_IMU_STREAMS[imu_frame]), _rate(imu_rate_hz),
            n.get("source", "unspecified"))
    if placeholder:
        comment = "{}; {}".format(comment, MISSING_IMU_COMMENT)
    sensor: Dict[str, Any] = {
        "sensor_type": "imu",
        "comment": comment,
        "T_BS": _t_bs(T),
        "rate_hz": _rate(imu_rate_hz),
    }
    for key in _NOISE_KEYS:
        if key not in n:
            raise CalibrationError("noise dict lacks '{}'".format(key))
        sensor[key] = float(n[key])
    return sensor


def to_euroc_sensor_yaml(calib: Any, size: Sequence[int] = DEFAULT_SIZE, *, imu_rate_hz: float = 200.0,
                         undistorted: bool = True, noise: Optional[Mapping[str, Any]] = None,
                         noise_preset: str = EUROC_NOISE_PRESET, body_socket: Union[int, str, Any] = DEFAULT_BODY_SOCKET,
                         left: Optional[Union[int, str, Any]] = None, right: Optional[Union[int, str, Any]] = None,
                         fps: float = 30.0, imu_frame: str = "raw", device_id: Optional[str] = None,
                         use_spec_translation: bool = False, allow_missing_imu: bool = False) -> Dict[str, Dict[str, Any]]:
    """The three ``sensor.yaml`` dicts ``{'cam0': ..., 'cam1': ..., 'imu0': ...}`` for one
    recording (cam0 = left, cam1 = right, body = *body_socket* = cam0 by default so
    ``T_BS(cam0)`` is the identity, exactly what NVIDIA's ``dataset_utils.get_rig`` expects).
    Write each with :func:`sensor_yaml_text` (or ``yaml.safe_dump``).  ``imu0`` uses
    ``EUROC_NOISE_PRESET`` (RealSense placeholder, dataset doc section 7) unless *noise* /
    *noise_preset* say otherwise; see :func:`euroc_imu_sensor` for ``allow_missing_imu``."""
    data = as_calibration(calib)
    default_left, default_right = stereo_sockets(data)
    left_name = socket_name(default_left if left is None else left)
    right_name = socket_name(default_right if right is None else right)
    product = data.get("productName") or "OAK-D"
    tag = "{} {}".format(product, device_id) if device_id else product
    w, h = _check_size(size)
    mode_text = "undistorted" if undistorted else "raw rational-polynomial"
    cam0 = euroc_camera_sensor(data, left_name, size, body_socket=body_socket, undistorted=undistorted, fps=fps,
                               comment="{} {} left = cam0 {}x{} GRAY8 {}fps {}".format(
                                   tag, left_name, w, h, _rate(fps), mode_text),
                               use_spec_translation=use_spec_translation)
    cam1 = euroc_camera_sensor(data, right_name, size, body_socket=body_socket, undistorted=undistorted, fps=fps,
                               comment="{} {} right = cam1 {}x{} GRAY8 {}fps {}".format(
                                   tag, right_name, w, h, _rate(fps), mode_text),
                               use_spec_translation=use_spec_translation)
    imu0 = euroc_imu_sensor(data, body_socket=body_socket, imu_rate_hz=imu_rate_hz, noise=noise,
                            noise_preset=noise_preset, imu_frame=imu_frame,
                            use_spec_translation=use_spec_translation, allow_missing_imu=allow_missing_imu)
    if device_id:
        imu0["comment"] = "{} {}".format(tag, imu0["comment"][len(product) + 1:])
    return {"cam0": cam0, "cam1": cam1, "imu0": imu0}


def yaml_float(value: float) -> str:
    """Format a float so ``yaml.safe_load`` (YAML 1.1) parses it as a float: the mantissa always
    has a dot and any exponent an explicit sign (``1e-05`` would load as a *string*)."""
    f = float(value)
    if not math.isfinite(f):
        raise CalibrationError("cannot write a non-finite float to sensor.yaml: {!r}".format(value))
    text = repr(f)
    if "e" in text or "E" in text:
        mantissa, exponent = text.lower().split("e")
        if "." not in mantissa:
            mantissa += ".0"
        if exponent[0] not in "+-":
            exponent = "+" + exponent
        return mantissa + "e" + exponent
    if "." not in text:
        text += ".0"
    return text


_YAML_COMMENTS: Dict[str, str] = {
    "intrinsics": "fu, fv, cu, cv",
    "distortion_coefficients": "k1, k2, p1, p2 (all zero: frames are pre-undistorted)",
    "gyroscope_noise_density": "[ rad / s / sqrt(Hz) ]",
    "gyroscope_random_walk": "[ rad / s^2 / sqrt(Hz) ]",
    "accelerometer_noise_density": "[ m / s^2 / sqrt(Hz) ]",
    "accelerometer_random_walk": "[ m / s^3 / sqrt(Hz) ]",
}


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        return yaml_float(value)
    if value is None:
        return "null"
    text = str(value)
    plain = text and all(ch.isalnum() or ch in " _.,+/()=-" for ch in text) \
        and text[0].isalnum() and text.strip() == text
    return text if plain else json.dumps(text)


def sensor_yaml_text(sensor: Mapping[str, Any], header: Optional[str] = None) -> str:
    """Render a sensor dict as EuRoC-style YAML text (flat keys, ``T_BS`` block with a 4x4 layout,
    safe float formatting, unit comments) without needing pyyaml.  Parses back with
    ``yaml.safe_load`` to the same values; write it with ``newline='\\n'``."""
    lines: List[str] = []
    if header:
        lines.extend("# " + ln if ln else "#" for ln in str(header).splitlines())
    for key, value in sensor.items():
        if key == "T_BS" or (isinstance(value, Mapping) and "data" in value):
            data = [float(v) for v in value["data"]]
            lines.append("{}:".format(key))
            for sub in ("cols", "rows"):
                if sub in value:
                    lines.append("  {}: {}".format(sub, int(value[sub])))
            rows = [data[i:i + 4] for i in range(0, len(data), 4)]
            cells = [[yaml_float(v) for v in row] for row in rows]
            width = max(len(c) for row in cells for c in row)
            rendered = [", ".join(c.rjust(width) for c in row) for row in cells]
            lines.append("  data: [" + rendered[0] + ("," if len(rendered) > 1 else "]"))
            for i, row in enumerate(rendered[1:], start=2):
                lines.append("         " + row + ("]" if i == len(rendered) else ","))
        elif isinstance(value, (list, tuple)):
            body = ", ".join(_yaml_scalar(v) for v in value)
            comment = _YAML_COMMENTS.get(key)
            if key == "distortion_coefficients" and any(float(v) != 0.0 for v in value):
                comment = "k1, k2, p1, p2, k3, k4, k5, k6 (OpenCV rational model: cuVSLAM Polynomial)"
            lines.append("{}: [{}]{}".format(key, body, "  # " + comment if comment else ""))
        else:
            comment = _YAML_COMMENTS.get(key)
            lines.append("{}: {}{}".format(key, _yaml_scalar(value), "  # " + comment if comment else ""))
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------------------
# Host undistortion maps (dataset path) and DepthAI cross-check
# --------------------------------------------------------------------------------------

def undistort_maps(calib: Any, socket: Union[int, str, Any], size: Sequence[int], *,
                   keep_aspect_ratio: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    """``cv2.initUndistortRectifyMap`` maps that remove all 14 EEPROM coefficients at *size*
    with ``newCameraMatrix = K`` (intrinsics unchanged, so ``sensor.yaml`` keeps the EEPROM
    values and zero distortion).  Apply with ``cv2.remap(gray, map1, map2, cv2.INTER_LINEAR)``.
    Imports OpenCV lazily; Perspective cameras only."""
    import cv2  # local: keep the module importable without OpenCV

    if distortion_model_for(calib, socket) != "Polynomial":
        raise NotImplementedError("undistort_maps handles the Perspective (rational) model only")
    w, h = _check_size(size)
    K = intrinsics_for(calib, socket, w, h, keep_aspect_ratio=keep_aspect_ratio)
    D = np.array(distortion_coefficients(calib, socket), dtype=np.float64)
    return cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_32FC1)


def verify_against_depthai(calib: Any, *, sizes: Sequence[Sequence[int]] = ((1280, 800), (640, 400)),
                           rig_sockets: Sequence[Union[int, str, Any]] = (CAM_A, CAM_B),
                           tol: float = 1e-3, raise_on_mismatch: bool = True) -> Dict[str, Any]:
    """Cross-check this module's numpy port against ``dai.CalibrationHandler`` built from the same
    dump (``fromJson``; no device needed).  Compares intrinsics (both aspect modes), distortion,
    ``getCameraExtrinsics`` to every *rig_socket* and ``getImuToCameraExtrinsics`` for the stereo
    pair.  Returns ``{'ok', 'checks', 'max_abs_diff', 'mismatches'}``; raises
    :class:`CalibrationError` on a mismatch unless ``raise_on_mismatch=False``.  Imports depthai
    lazily and raises ``ImportError`` when it is missing."""
    import depthai as dai  # local: the module itself never needs depthai

    data = as_calibration(calib)
    handler = dai.CalibrationHandler.fromJson(data)
    metre = dai.LengthUnit.METER
    mismatches: List[str] = []
    checks = 0
    max_diff = 0.0

    def compare(label: str, ours: Any, theirs: Any) -> None:
        nonlocal checks, max_diff
        a = np.asarray(ours, dtype=np.float64)
        b = np.asarray(theirs, dtype=np.float64)
        checks += 1
        if a.shape != b.shape:
            mismatches.append("{}: shape {} vs depthai {}".format(label, a.shape, b.shape))
            return
        diff = float(np.abs(a - b).max()) if a.size else 0.0
        max_diff = max(max_diff, diff)
        if diff > tol:
            mismatches.append("{}: max abs diff {:.3e} > {:.1e}".format(label, diff, tol))

    def dai_socket(name: str) -> Any:
        return getattr(dai.CameraBoardSocket, socket_name(name))

    left, right = stereo_sockets(data)
    for name in (left, right):
        sock = dai_socket(name)
        compare("{} distortion".format(name), distortion_coefficients(data, name),
                handler.getDistortionCoefficients(sock))
        for (w, h) in sizes:
            for keep in (True, False):
                compare("{} intrinsics {}x{} keepAspectRatio={}".format(name, w, h, keep),
                        intrinsics_for(data, name, w, h, keep_aspect_ratio=keep),
                        handler.getCameraIntrinsics(sock, int(w), int(h), keepAspectRatio=keep))
        compare("{} intrinsics native".format(name), intrinsics_for(data, name, None, None),
                handler.getCameraIntrinsics(sock, -1, -1))
        for rig in rig_sockets:
            rig_name = socket_name(rig)
            compare("{} rig_from_camera (rig {})".format(name, rig_name), rig_from_camera(data, name, rig_name),
                    handler.getCameraExtrinsics(sock, dai_socket(rig_name), False, metre))
    for rig in rig_sockets:
        rig_name = socket_name(rig)
        compare("rig_from_imu (rig {})".format(rig_name), rig_from_imu(data, rig_name),
                handler.getImuToCameraExtrinsics(dai_socket(rig_name), False, metre))
    compare("baseline", [stereo_baseline_m(data, left, right)],
            [handler.getBaselineDistance(dai_socket(right), dai_socket(left), False, metre)])

    report = {"ok": not mismatches, "checks": checks, "max_abs_diff": max_diff, "mismatches": mismatches,
              "depthai_version": getattr(dai, "__version__", "?")}
    if mismatches and raise_on_mismatch:
        raise CalibrationError("numpy port disagrees with depthai {}: {}".format(
            report["depthai_version"], "; ".join(mismatches)))
    return report


# --------------------------------------------------------------------------------------
# Command line: describe the rig a calibration file produces (no camera, no cuvslam)
# --------------------------------------------------------------------------------------

def _parse_size(text: str) -> Tuple[int, int]:
    try:
        w, h = text.lower().replace(",", "x").split("x")
        return int(w), int(h)
    except ValueError:
        raise argparse.ArgumentTypeError("size must look like 1280x800, got {!r}".format(text)) from None


def _main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Print the cuVSLAM rig (JSON) or the EuRoC sensor.yaml files derived from an "
                    "OAK-D S2 calibration.json / holistic camera_info.json. No camera is opened.")
    parser.add_argument("calibration", help="path to calibration.json (CalibrationHandler.eepromToJson dump)")
    parser.add_argument("--size", type=_parse_size, default=DEFAULT_SIZE, metavar="WxH",
                        help="frame size the rig is built for (default 1280x800; 640x400 is the light option)")
    parser.add_argument("--mode", choices=MODES, default="stereo", help="stereo (cameras only) or vio (+ IMU)")
    parser.add_argument("--rig-socket", default=None, metavar="CAM_X",
                        help="rig / body frame camera (default CAM_A for the rig like NVIDIA's example, "
                             "CAM_B = cam0 for --euroc)")
    parser.add_argument("--imu-rate", type=float, default=200.0, metavar="HZ", help="IMU report rate (200 or 400)")
    parser.add_argument("--imu-frame", choices=IMU_FRAMES, default="raw",
                        help="raw = *_RAW streams (EEPROM rotation kept); aligned = *_UNCALIBRATED/_CALIBRATED")
    frames = parser.add_mutually_exclusive_group()
    frames.add_argument("--undistorted", action="store_true",
                        help="frames are undistorted on the host (Pinhole, no borders); the --euroc default")
    frames.add_argument("--raw", action="store_true",
                        help="frames are raw (Polynomial, 8 coefficients, border masks); the rig default")
    parser.add_argument("--noise-preset", choices=sorted(NOISE_PRESETS), default=None,
                        help="IMU noise preset (default: {} for the rig, {} for --euroc)".format(
                            DEFAULT_NOISE_PRESET, EUROC_NOISE_PRESET))
    parser.add_argument("--euroc", action="store_true", help="print the three sensor.yaml texts instead of the rig")
    parser.add_argument("--allow-missing-imu", action="store_true",
                        help="with --euroc: write an identity T_BS placeholder into imu0/sensor.yaml when the "
                             "calibration has no IMU extrinsics (stereo-only dataset)")
    parser.add_argument("--verify", action="store_true",
                        help="cross-check the numpy port against depthai (if installed)")
    args = parser.parse_args(argv)

    try:
        calib = load_calibration_json(args.calibration)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", CalibrationWarning)
            if args.euroc:
                undistorted = not args.raw
                body = DEFAULT_BODY_SOCKET if args.rig_socket is None else args.rig_socket
                preset = EUROC_NOISE_PRESET if args.noise_preset is None else args.noise_preset
                yamls = to_euroc_sensor_yaml(calib, args.size, imu_rate_hz=args.imu_rate, undistorted=undistorted,
                                             noise_preset=preset, body_socket=body, imu_frame=args.imu_frame,
                                             allow_missing_imu=args.allow_missing_imu)
                for name in ("cam0", "cam1", "imu0"):
                    print("# ---- mav0/{}/sensor.yaml ----".format(name))
                    print(sensor_yaml_text(yamls[name]), end="")
                for w in caught:
                    print("warning: {}".format(w.message), file=sys.stderr)
            else:
                undistorted = bool(args.undistorted)
                rig_socket = DEFAULT_RIG_SOCKET if args.rig_socket is None else args.rig_socket
                preset = DEFAULT_NOISE_PRESET if args.noise_preset is None else args.noise_preset
                info = rig_spec(calib, args.size, args.mode, args.imu_rate, rig_socket=rig_socket,
                                undistorted=undistorted, imu_frame=args.imu_frame, noise_preset=preset)
                if args.verify:
                    info["depthai_verification"] = verify_against_depthai(calib, raise_on_mismatch=False)
                print(json.dumps(info, indent=2))
                for line in info["warnings"]:
                    print("warning: {}".format(line), file=sys.stderr)
                if args.verify and not info["depthai_verification"]["ok"]:
                    print("verification FAILED: {}".format(info["depthai_verification"]["mismatches"]), file=sys.stderr)
                    return 1
    except (ValueError, OSError, ImportError) as exc:
        # CalibrationError is a ValueError; a stray ValueError/OSError still gets one line, not a traceback
        print("{}: {}".format(type(exc).__name__, exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(_main())
