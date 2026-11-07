"""Tests for the ``cuvslam_oak`` package and the ``07``/``08`` scripts (no camera, no cuVSLAM wheel).

What is covered, and against which reference:

* Rig math (``cuvslam_oak.rig``) on the vendored EEPROM dump
  ``tests/data/calibration_OAKDS2EXAMPLEID000.json`` of this OAK-D-S2-AF (``recordings/`` is
  git-ignored; any ``recordings/video_*/calibration.json`` made on the same unit is cross-checked
  against it): stereo baseline 0.075 +- 0.001 m, exact half-scale intrinsics at 640x400,
  ``rig_from_imu`` equal to the worked matrix of ``methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md`` (section
  4.2) within 1e-6, 8 Polynomial distortion coefficients, the worked ``rig_from_camera``
  translations and quaternion conventions.
* Dataset round trip (``cuvslam_oak.dataset``): a tiny EuRoC ASL recording (6 stereo pairs at
  640x400, 200 Hz IMU) is written with ``EurocWriter``, read back with ``EurocReader``,
  validated, and checked file by file against ``datasets/dataset-format.md`` (layout,
  ``sensor.yaml`` keys and YAML 1.1 float spelling, csv headers and column order, 8-bit PNGs,
  merged IMU-before-frame order), plus the negative cases the format doc lists.
* Tracking (``cuvslam_oak.tracking``) with the fake cuVSLAM in both modes: TUM row counts,
  IMU hold-back per source type (iterator / queue / merged stream), warm-up, ``max_frames``,
  out-of-order frames, rig/mode mismatch, TUM write/read.
* ``cuvslam_oak.fake_cuvslam`` input validation: wrong image count, dtype or size, timestamp
  order, IMU registration without inertial mode, ``ImuMeasurement`` field length.
* CLI smoke tests through ``subprocess``: ``07 --help``, ``08 --help``,
  ``07 --device-id nonexistent`` (exit 2, one stderr line, the camera is only enumerated),
  ``08 --dataset nonexistent`` (exit 2), ``08 --fake-cuvslam`` on the tiny dataset (exit 0,
  TUM rows) and the exit-3 path without any cuvslam.

Runs on Windows with ``python -m pytest tests -q``; the camera is never opened (see
``tests/conftest.py``).  Python 3.10 syntax, pathlib only.
"""

from __future__ import annotations

import importlib.util
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import types
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np
import pytest

import cuvslam_oak
from cuvslam_oak import dataset, rig, tracking

REPO_ROOT = Path(__file__).resolve().parents[1]
RECORD_SCRIPT = REPO_ROOT / "datasets" / "07_record_dataset.py"
TRACK_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py"
PYTHON = sys.executable
SUBPROCESS_TIMEOUT_S = 60.0

#: Recording geometry of the tiny synthetic dataset (640x400 = exact halving of the calibration).
SIZE = (640, 400)
SHAPE = (SIZE[1], SIZE[0])
FPS = 30.0
IMU_RATE_HZ = 200.0
T0 = 1_758_000_000_000_000_000          # 19-digit Unix-epoch nanoseconds, as the recorder writes
FRAME_PERIOD_NS = 33_333_333
IMU_PERIOD_NS = 5_000_000
N_FRAMES = 6
IMU_TAIL = 3                             # IMU rows after the last pair (the stream ends last)
GYRO = (0.001, -0.002, 0.003)            # rad/s, written first in imu0/data.csv
ACCEL = (0.0, 9.81, 0.0)                 # m/s^2 specific force, upright camera

#: Worked values for this unit from methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md (sections 1.3, 3.3, 4.2, 8).
DOC_RIG_FROM_IMU = np.array([[1.0, 0.0, 0.0, -0.037948],
                             [0.0, -1.0, 0.0, -0.00079],
                             [0.0, 0.0, -1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
DOC_LEFT_PRINCIPAL_1280 = (649.4675, 391.5385)
DOC_LEFT_FOCAL_640 = (399.5633, 399.4518)
DOC_LEFT_PRINCIPAL_640 = (324.7338, 195.7693)
DOC_RIGHT_FOCAL_640 = (398.7681, 398.7350)
DOC_RIG_FROM_LEFT_T = (-0.037373, 0.000334, -0.001100)
DOC_RIG_FROM_RIGHT_T = (0.037591, 0.000806, -0.000594)
DOC_BASELINE_M = 0.074967
DOC_K1 = {rig.CAM_B: 23.206055, rig.CAM_C: 25.588825}

#: The unit every DOC_* constant was worked out for, and its EEPROM dump (``eepromToJson()`` as the
#: recorders save it), vendored so the rig-math tests run on a fresh clone (``recordings/`` is git-ignored).
DEVICE_ID = "OAKDS2EXAMPLEID000"
REFERENCE_CALIBRATION = REPO_ROOT / "tests" / "data" / "calibration_{}.json".format(DEVICE_ID)
#: Fields that identify one calibration of one board; a recalibration changes ``batchTime``.
CALIBRATION_IDENTITY_KEYS = ("productName", "boardName", "boardRev", "batchTime")

#: Literals of datasets/dataset-format.md (sections 3, 4 and 7), asserted as such so that a
#: ``dataset.py`` constant drifting from the doc fails here.
DOC_CAM_CSV_HEADER = "#timestamp [ns],filename"
DOC_IMU_CSV_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                      "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")
DOC_IMU_NOISE = {                                       # NVIDIA's RealSense placeholders, in doc order
    "gyroscope_noise_density": 6.0673370376614875e-03,
    "gyroscope_random_walk": 3.6211951458325785e-05,
    "accelerometer_noise_density": 3.3621979208052800e-02,
    "accelerometer_random_walk": 9.8256589971851467e-04,
}
CAMERA_YAML_MANDATORY = ("T_BS", "resolution", "intrinsics", "distortion_coefficients")
CAMERA_YAML_OPTIONAL = ("distortion_model", "camera_model", "rate_hz", "sensor_type", "comment")
IMU_YAML_MANDATORY = ("T_BS", "rate_hz") + tuple(DOC_IMU_NOISE)


# --------------------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------------------

def recorded_calibration_files() -> List[Path]:
    """``recordings/video_*/calibration.json`` dumps: git-ignored, present only where recordings were made."""
    return sorted(REPO_ROOT.glob("recordings/video_*/calibration.json"))


def recorded_device_id(calibration_file: Path) -> Optional[str]:
    """Device id from the ``session.json`` the recorder wrote next to a dump, ``None`` when unknown."""
    try:
        doc = json.loads(calibration_file.with_name("session.json").read_text(encoding="utf-8"))
        value = doc["device"]["id"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return str(value) if value else None


def calibration_identity(calib: Dict[str, Any]) -> Tuple[Any, ...]:
    return tuple(calib.get(key) for key in CALIBRATION_IDENTITY_KEYS)


def build_fake_rig(vslam: Any, calib: Dict[str, Any], mode: str, *, undistorted: bool = False,
                   rig_socket: str = rig.CAM_A) -> Tuple[Any, Dict[str, Any]]:
    """``rig.build_rig`` at 640x400 with the distortion-truncation warning silenced."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", rig.CalibrationWarning)
        return rig.build_rig(calib, SIZE, mode, imu_rate_hz=IMU_RATE_HZ, vslam=vslam,
                             undistorted=undistorted, rig_socket=rig_socket)


def track_config(**overrides: Any) -> tracking.TrackConfig:
    """Deterministic offline config: no warm-up frames, no GPU warm-up, quiet."""
    base: Dict[str, Any] = {"verbose": False, "warm_up_gpu": False, "warmup_frames": 0}
    base.update(overrides)
    return tracking.TrackConfig(**base)


def synthetic_frames(count: int, t0: int = T0) -> List[Tuple[int, np.ndarray, np.ndarray]]:
    """``(ts_ns, left, right)`` frames at 30 fps, black 640x400 GRAY8 images."""
    return [(t0 + i * FRAME_PERIOD_NS, np.zeros(SHAPE, np.uint8), np.zeros(SHAPE, np.uint8)) for i in range(count)]


def synthetic_imu(count: int, t0: int = T0 - IMU_PERIOD_NS) -> List[Tuple[int, Tuple[float, ...], Tuple[float, ...]]]:
    """``(ts_ns, gyro, accel)`` samples at 200 Hz starting one period before ``T0``."""
    return [(t0 + j * IMU_PERIOD_NS, GYRO, ACCEL) for j in range(count)]


def merged_stream(frames: Sequence[Tuple[int, Any, Any]], imu: Sequence[Tuple[int, Any, Any]]) -> List[Any]:
    """One stream in time order with IMU samples *before* a frame of equal timestamp (RealSense order)."""
    out: List[Any] = []
    j = 0
    for frame in frames:
        while j < len(imu) and imu[j][0] <= frame[0]:
            out.append(("imu",) + tuple(imu[j]))
            j += 1
        out.append(frame)
    out.extend(("imu",) + tuple(s) for s in imu[j:])
    return out


def run_cli(script: Path, *args: Any, timeout: float = SUBPROCESS_TIMEOUT_S) -> subprocess.CompletedProcess:
    """Run a script with the test interpreter from the repo root; ``CUVSLAM_FAKE`` is never inherited."""
    env = dict(os.environ)
    env.pop(cuvslam_oak.FAKE_ENV_VAR, None)
    env.setdefault("PYTHONIOENCODING", "utf-8")
    return subprocess.run([PYTHON, str(script), *(str(a) for a in args)], cwd=str(REPO_ROOT),
                          capture_output=True, text=True, encoding="utf-8", errors="replace",
                          env=env, timeout=timeout, check=False)


def stderr_lines(proc: subprocess.CompletedProcess) -> List[str]:
    return [line for line in proc.stderr.splitlines() if line.strip()]


def float_tokens(text: str) -> Iterator[re.Match]:
    """Every exponent-form number in a yaml file (the only form YAML 1.1 can mis-parse)."""
    return re.finditer(r"(?<![\w.])([-+]?\d+)(\.\d+)?[eE]([-+]?)\d+(?![\w.])", text)


# --------------------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------------------

@pytest.fixture(scope="session")
def calibration_path() -> Path:
    """The vendored dump; a recorded dump of the same unit only when the vendored file is missing."""
    if REFERENCE_CALIBRATION.is_file():
        return REFERENCE_CALIBRATION
    fallback = [p for p in recorded_calibration_files() if recorded_device_id(p) == DEVICE_ID]
    missing = REFERENCE_CALIBRATION.relative_to(REPO_ROOT).as_posix()
    if not fallback:
        pytest.skip("{} is missing and no recordings/video_*/calibration.json of unit {} exists".format(
            missing, DEVICE_ID))
    warnings.warn("{} is missing; the rig-math tests fall back to {}".format(missing, fallback[0]), UserWarning,
                  stacklevel=1)
    return fallback[0]


@pytest.fixture(scope="session")
def calib(calibration_path: Path) -> Dict[str, Any]:
    return rig.load_calibration_json(calibration_path)


@pytest.fixture(scope="session")
def vslam() -> types.ModuleType:
    module = cuvslam_oak.import_cuvslam(prefer_fake=True)
    assert cuvslam_oak.is_fake(module), "the suite must run against cuvslam_oak.fake_cuvslam"
    return module


@pytest.fixture(scope="session")
def rig_info(calib: Dict[str, Any]) -> dataset.RigInfo:
    """The ``RigInfo`` of an undistorted 640x400 recording, built the way ``07_record_dataset.py`` does
    (``rig.to_euroc_sensor_yaml``: body = cam0 = CAM_B, RealSense placeholder IMU noise)."""
    sensors = rig.to_euroc_sensor_yaml(calib, SIZE, imu_rate_hz=IMU_RATE_HZ, undistorted=True,
                                       body_socket=rig.CAM_B, fps=FPS, device_id="TESTUNIT")

    def camera(sensor: Dict[str, Any], socket: str, role: str) -> dataset.CameraInfo:
        fu, fv, cu, cv_ = (float(v) for v in sensor["intrinsics"])
        return dataset.CameraInfo(width=SIZE[0], height=SIZE[1], fu=fu, fv=fv, cu=cu, cv=cv_,
                                  T_BS=sensor["T_BS"]["data"], distortion_model=str(sensor["distortion_model"]),
                                  distortion_coefficients=[float(c) for c in sensor["distortion_coefficients"]],
                                  rate_hz=FPS, comment=str(sensor["comment"]), socket=socket, role=role,
                                  sensor="OV9282", undistorted=True)

    imu0 = sensors["imu0"]
    imu = dataset.ImuInfo(T_BS=imu0["T_BS"]["data"], rate_hz=IMU_RATE_HZ,
                          gyroscope_noise_density=float(imu0["gyroscope_noise_density"]),
                          gyroscope_random_walk=float(imu0["gyroscope_random_walk"]),
                          accelerometer_noise_density=float(imu0["accelerometer_noise_density"]),
                          accelerometer_random_walk=float(imu0["accelerometer_random_walk"]),
                          comment=str(imu0["comment"]))
    return dataset.RigInfo(cam0=camera(sensors["cam0"], rig.CAM_B, "left"),
                           cam1=camera(sensors["cam1"], rig.CAM_C, "right"), imu=imu, device_id="TESTUNIT")


@dataclass
class TinyDataset:
    """A written EuRoC recording and what went into it."""

    root: Path
    frame_ts: List[int]
    imu_ts: List[int]
    summary: Dict[str, Any]

    @property
    def mav0(self) -> Path:
        return self.root / "mav0"

    @property
    def last_frame_ts(self) -> int:
        return self.frame_ts[-1]

    def imu_up_to(self, ts_ns: int) -> int:
        """Number of IMU rows stamped at or before *ts_ns* (what a tracker may register by then)."""
        return sum(1 for t in self.imu_ts if t <= ts_ns)


@pytest.fixture(scope="session")
def tiny_dataset(tmp_path_factory: pytest.TempPathFactory, rig_info: dataset.RigInfo,
                 calib: Dict[str, Any]) -> TinyDataset:
    """6 noisy 640x400 pairs at 30 fps and a 200 Hz IMU that starts one period before the first
    pair and ends three rows after the last, exactly the ordering the format doc asks for."""
    root = tmp_path_factory.mktemp("euroc") / "euroc_test"
    rng = np.random.default_rng(1234)
    frame_ts = [T0 + i * FRAME_PERIOD_NS for i in range(N_FRAMES)]
    imu_ts: List[int] = []
    with dataset.EurocWriter(root, rig_info, cam_rate_hz=FPS, imu_rate_hz=IMU_RATE_HZ, calibration=calib,
                             png_compression=1) as writer:
        next_imu = T0 - IMU_PERIOD_NS
        for ts in frame_ts:
            while next_imu <= ts:
                assert writer.add_imu(next_imu, GYRO, ACCEL)
                imu_ts.append(next_imu)
                next_imu += IMU_PERIOD_NS
            left = rng.integers(0, 256, size=SHAPE, dtype=np.uint8)
            right = rng.integers(0, 256, size=SHAPE, dtype=np.uint8)
            assert writer.add_frame(ts, left, right, right_ts_ns=ts + 12_000)   # 12 us L/R skew as on the unit
        for _ in range(IMU_TAIL):
            assert writer.add_imu(next_imu, GYRO, ACCEL)
            imu_ts.append(next_imu)
            next_imu += IMU_PERIOD_NS
    assert writer.summary is not None
    return TinyDataset(root=root, frame_ts=frame_ts, imu_ts=imu_ts, summary=writer.summary)


@pytest.fixture(scope="session")
def track_script() -> types.ModuleType:
    """``methods/cuvslam-vio/08_cuvslam_track.py`` imported as a module (its ``rig_from_dataset`` mirrors NVIDIA's loader)."""
    spec = importlib.util.spec_from_file_location("cuvslam_track_script", TRACK_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------------------------------
# Rig math on the saved calibration
# --------------------------------------------------------------------------------------

class TestRigMath:
    def test_reference_dump_identity(self, calib: Dict[str, Any]) -> None:
        assert calib["productName"] == "OAK-D-S2-AF" and calib["boardName"] == "DM9098"
        assert calib["version"] == 7 and len(calib["cameraData"]) == 3
        assert rig.stereo_sockets(calib) == (rig.CAM_B, rig.CAM_C)
        assert rig.native_size(calib, rig.CAM_B) == (1280, 800)
        assert rig.has_imu_extrinsics(calib)

    def test_recorded_dumps_match_reference(self, calib: Dict[str, Any]) -> None:
        """Every recordings/video_*/calibration.json made on this unit is the vendored dump (optional:
        the folder is git-ignored; other units are ignored, a recalibration only warns)."""
        files = recorded_calibration_files()
        if not files:
            pytest.skip("no recordings/video_*/calibration.json to cross-check (recordings/ is git-ignored)")
        compared = 0
        for path in files:
            other = rig.load_calibration_json(path)
            device_id = recorded_device_id(path)
            if device_id is not None and device_id != DEVICE_ID:
                continue                                              # another unit's recording
            if calibration_identity(other) != calibration_identity(calib):
                if device_id == DEVICE_ID:
                    warnings.warn("{} is a different calibration of unit {} ({}): refresh {} if the unit was "
                                  "recalibrated".format(path, DEVICE_ID, calibration_identity(other),
                                                        REFERENCE_CALIBRATION.name), UserWarning, stacklevel=1)
                continue                                              # unknown unit or a later recalibration
            assert other == calib, "{} differs from the vendored dump of unit {}".format(path, DEVICE_ID)
            compared += 1
        if compared == 0:
            pytest.skip("no recordings/video_* dump of unit {} to cross-check".format(DEVICE_ID))

    def test_stereo_baseline(self, calib: Dict[str, Any], vslam: types.ModuleType) -> None:
        assert abs(rig.stereo_baseline_m(calib) - 0.075) <= 1e-3
        assert rig.stereo_baseline_m(calib) == pytest.approx(DOC_BASELINE_M, abs=1e-5)
        assert rig.stereo_baseline_m(calib, use_spec_translation=True) == pytest.approx(0.075, abs=1e-6)
        _rig_obj, info = build_fake_rig(vslam, calib, "stereo")
        stereo = info["stereo"]
        assert stereo["left"] == rig.CAM_B and stereo["right"] == rig.CAM_C
        assert abs(stereo["baseline_m"] - 0.075) <= 1e-3
        # the right camera sits along +x of the rig (RDF, image right): no inversion happened
        assert stereo["right_minus_left_m"][0] > 0.074
        assert stereo["direction"][0] > 0.9999

    @pytest.mark.parametrize("socket", [rig.CAM_B, rig.CAM_C])
    def test_half_scale_intrinsics_at_640x400(self, calib: Dict[str, Any], socket: str) -> None:
        K_native = rig.intrinsics_for(calib, socket, 1280, 800)
        K_half = rig.intrinsics_for(calib, socket, 640, 400)
        assert K_native.shape == (3, 3) and K_half.shape == (3, 3)
        np.testing.assert_allclose(K_half[:2, :], K_native[:2, :] / 2.0, rtol=0, atol=1e-6)
        np.testing.assert_allclose(K_half[2], [0.0, 0.0, 1.0], atol=0)
        assert rig.intrinsics_for(calib, socket, 640, 400, keep_aspect_ratio=False) == pytest.approx(K_half)

    def test_worked_intrinsics(self, calib: Dict[str, Any]) -> None:
        K_b = rig.intrinsics_for(calib, rig.CAM_B, 1280, 800)
        assert (K_b[0, 2], K_b[1, 2]) == pytest.approx(DOC_LEFT_PRINCIPAL_1280, abs=1e-4)
        K_b_half = rig.intrinsics_for(calib, rig.CAM_B, 640, 400)
        assert (K_b_half[0, 0], K_b_half[1, 1]) == pytest.approx(DOC_LEFT_FOCAL_640, abs=1e-4)
        assert (K_b_half[0, 2], K_b_half[1, 2]) == pytest.approx(DOC_LEFT_PRINCIPAL_640, abs=1e-4)
        K_c_half = rig.intrinsics_for(calib, rig.CAM_C, 640, 400)
        assert (K_c_half[0, 0], K_c_half[1, 1]) == pytest.approx(DOC_RIGHT_FOCAL_640, abs=1e-4)

    def test_rig_from_imu_matches_worked_matrix(self, calib: Dict[str, Any]) -> None:
        T = rig.rig_from_imu(calib, rig.CAM_A)
        assert T.shape == (4, 4)
        np.testing.assert_allclose(T, DOC_RIG_FROM_IMU, rtol=0, atol=1e-6)
        np.testing.assert_allclose(T[:3, :3], np.diag([1.0, -1.0, -1.0]), atol=1e-9)
        # quaternion x y z w of a 180 deg rotation about x is (1, 0, 0, 0)
        np.testing.assert_allclose(rig.matrix_to_quaternion_xyzw(T[:3, :3]), [1.0, 0.0, 0.0, 0.0], atol=1e-9)
        # 'aligned' streams: same lever arm, identity rotation
        T_aligned = rig.rig_from_imu(calib, rig.CAM_A, imu_frame="aligned")
        np.testing.assert_allclose(T_aligned[:3, :3], np.eye(3), atol=0)
        np.testing.assert_allclose(T_aligned[:3, 3], T[:3, 3], atol=0)
        # rig = CAM_B: the IMU sits within 1.2 mm per axis of the left camera, t = (-0.58, -1.13, +1.10) mm
        # (doc sections 3.2 and 6.2)
        T_b = rig.rig_from_imu(calib, rig.CAM_B)
        assert np.max(np.abs(T_b[:3, 3])) < 1.2e-3
        assert np.linalg.norm(T_b[:3, 3]) < 2.0e-3
        np.testing.assert_allclose(T_b, rig.invert_se3(rig.rig_from_camera(calib, rig.CAM_B)) @ T, atol=1e-7)

    def test_rig_from_camera_translations(self, calib: Dict[str, Any]) -> None:
        T_left = rig.rig_from_camera(calib, rig.CAM_B)      # A_from_B, cm -> m, never inverted
        T_right = rig.rig_from_camera(calib, rig.CAM_C)     # A_from_C
        assert T_left[:3, 3] == pytest.approx(DOC_RIG_FROM_LEFT_T, abs=1e-4)
        assert T_right[:3, 3] == pytest.approx(DOC_RIG_FROM_RIGHT_T, abs=1e-4)
        assert np.linalg.norm(T_right[:3, 3] - T_left[:3, 3]) == pytest.approx(DOC_BASELINE_M, abs=1e-5)
        # rig = CAM_B (the dataset frame): left is the identity, right at +0.075 m along x
        np.testing.assert_allclose(rig.rig_from_camera(calib, rig.CAM_B, rig.CAM_B), np.eye(4), atol=0)
        T_bc = rig.rig_from_camera(calib, rig.CAM_C, rig.CAM_B)
        assert T_bc[0, 3] == pytest.approx(DOC_BASELINE_M, abs=1e-5)
        assert abs(T_bc[1, 3]) < 1e-3 and abs(T_bc[2, 3]) < 1e-3
        # getCameraExtrinsics(B, C) translation x = -0.074966 m (doc section 8)
        assert rig.camera_extrinsics(calib, rig.CAM_B, rig.CAM_C)[0, 3] == pytest.approx(-0.074966, abs=1e-5)
        for T in (T_left, T_right, T_bc):
            rig.check_rotation(T[:3, :3])
            np.testing.assert_allclose(rig.quaternion_xyzw_to_matrix(rig.matrix_to_quaternion_xyzw(T[:3, :3])),
                                       T[:3, :3], atol=1e-9)

    @pytest.mark.parametrize("socket", [rig.CAM_B, rig.CAM_C])
    def test_eight_distortion_coefficients(self, calib: Dict[str, Any], socket: str) -> None:
        full = rig.distortion_coefficients(calib, socket)
        assert len(full) == 14                                   # k1 k2 p1 p2 k3 k4 k5 k6 s1..s4 tx ty
        assert rig.distortion_model_for(calib, socket) == "Polynomial"
        with pytest.warns(rig.CalibrationWarning):              # non-zero tx, ty are dropped on this unit
            eight = rig.distortion_for(calib, socket)
        assert len(eight) == 8
        assert eight == pytest.approx(full[:8])
        assert eight[0] == pytest.approx(DOC_K1[socket], abs=1e-5)
        sink: List[str] = []
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", rig.CalibrationWarning)
            assert rig.distortion_for(calib, socket, warnings_sink=sink) == pytest.approx(eight)
            assert rig.distortion_for(calib, socket, warn=False) == pytest.approx(eight)
        assert len(sink) == 1 and "tx=" in sink[0]

    def test_build_rig_objects(self, calib: Dict[str, Any], vslam: types.ModuleType) -> None:
        stereo_rig, stereo_info = build_fake_rig(vslam, calib, "stereo")
        vio_rig, vio_info = build_fake_rig(vslam, calib, "vio")
        assert stereo_info["odometry_mode"] == "Multicamera" and vio_info["odometry_mode"] == "Inertial"
        assert len(stereo_rig.cameras) == 2 and len(stereo_rig.imus) == 0
        assert len(vio_rig.cameras) == 2 and len(vio_rig.imus) == 1
        for cam, socket in zip(vio_rig.cameras, (rig.CAM_B, rig.CAM_C)):
            assert tuple(int(v) for v in cam.size) == SIZE
            assert cam.distortion.model == vslam.Distortion.Model.Polynomial
            assert len(list(cam.distortion.parameters)) == 8
            K = rig.intrinsics_for(calib, socket, *SIZE)
            assert tuple(float(v) for v in cam.focal) == pytest.approx((K[0, 0], K[1, 1]), abs=1e-3)
            assert tuple(float(v) for v in cam.principal) == pytest.approx((K[0, 2], K[1, 2]), abs=1e-3)
            assert (cam.border_top, cam.border_left, cam.border_right) == (25, 35, 35)   # 50/70/70 scaled to 640x400
        imu = vio_rig.imus[0]
        np.testing.assert_allclose(list(imu.rig_from_imu.rotation), [1.0, 0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(list(imu.rig_from_imu.translation), DOC_RIG_FROM_IMU[:3, 3], atol=1e-6)
        assert float(imu.frequency) == IMU_RATE_HZ
        assert vio_info["imu"]["noise"]["preset"] == rig.DEFAULT_NOISE_PRESET
        assert float(imu.gyroscope_noise_density) == rig.NOISE_PRESETS[rig.DEFAULT_NOISE_PRESET]["gyroscope_noise_density"]
        # host-undistorted frames: Pinhole, no borders
        undist_rig, _info = build_fake_rig(vslam, calib, "stereo", undistorted=True)
        for cam in undist_rig.cameras:
            assert cam.distortion.model == vslam.Distortion.Model.Pinhole
            assert list(cam.distortion.parameters) == []
            assert (cam.border_top, cam.border_bottom, cam.border_left, cam.border_right) == (0, 0, 0, 0)

    def test_load_errors(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            rig.load_calibration_json(tmp_path / "missing.json")
        bad = tmp_path / "bad.json"
        bad.write_text("not json", encoding="utf-8")
        with pytest.raises(rig.CalibrationError):
            rig.load_calibration_json(bad)
        with pytest.raises(rig.CalibrationError):
            rig.build_rig({"version": 7, "cameraData": []}, SIZE, "stereo")


# --------------------------------------------------------------------------------------
# Dataset round trip
# --------------------------------------------------------------------------------------

class TestDatasetRoundTrip:
    def test_layout(self, tiny_dataset: TinyDataset) -> None:
        root = tiny_dataset.root
        for rel in ("dataset.json", "calibration.json", "README.txt", "mav0/cam0/data.csv", "mav0/cam0/sensor.yaml",
                    "mav0/cam1/data.csv", "mav0/cam1/sensor.yaml", "mav0/imu0/data.csv", "mav0/imu0/sensor.yaml"):
            assert (root / rel).is_file(), rel
        assert not (root / "mav0" / "cam0" / "sensor_recalibrated.yaml").exists()
        names0 = sorted(p.name for p in (root / "mav0" / "cam0" / "data").glob("*.png"))
        names1 = sorted(p.name for p in (root / "mav0" / "cam1" / "data").glob("*.png"))
        assert names0 == names1 == ["{}.png".format(ts) for ts in tiny_dataset.frame_ts]
        assert all(len(name) == 19 + 4 for name in names0)      # 19-digit stamps, lexical == numeric order
        assert dataset.resolve_mav0(root) == (root, root / "mav0")
        assert dataset.resolve_mav0(root / "mav0") == (root, root / "mav0")

    def test_dataset_json(self, tiny_dataset: TinyDataset, calib: Dict[str, Any]) -> None:
        doc = json.loads((tiny_dataset.root / "dataset.json").read_text(encoding="utf-8"))
        assert doc["format"] == dataset.FORMAT_NAME == "euroc-asl"
        assert doc["format_version"] == dataset.FORMAT_VERSION == 1
        for key in ("generator", "device", "record", "time", "cameras", "stereo", "imu", "counts", "measured",
                    "drops", "calibration", "warnings"):
            assert key in doc, key
        assert doc["counts"]["stereo_pairs"] == N_FRAMES
        assert doc["counts"]["imu_samples"] == len(tiny_dataset.imu_ts)
        assert doc["counts"]["first_timestamp_ns"] == tiny_dataset.imu_ts[0]
        assert doc["counts"]["last_timestamp_ns"] == tiny_dataset.imu_ts[-1]
        assert doc["cameras"]["cam0"]["undistorted"] is True
        assert doc["cameras"]["cam0"]["resolution"] == list(SIZE)
        assert doc["imu"]["rate_hz"] == IMU_RATE_HZ
        assert all(isinstance(v, int) for v in doc["drops"].values())
        assert all(doc["drops"][name] == 0 for name in dataset.DROP_COUNTERS)
        assert doc["warnings"] == []
        assert json.loads((tiny_dataset.root / "calibration.json").read_text(encoding="utf-8")) == calib
        assert tiny_dataset.summary["counts"] == doc["counts"]

    @pytest.mark.parametrize("cam", ["cam0", "cam1"])
    def test_camera_sensor_yaml(self, tiny_dataset: TinyDataset, cam: str) -> None:
        text = (tiny_dataset.mav0 / cam / "sensor.yaml").read_text(encoding="ascii")
        cfg = dataset.parse_simple_yaml(text)
        for key in CAMERA_YAML_MANDATORY + CAMERA_YAML_OPTIONAL:
            assert key in cfg, key
        assert [k for k in cfg if k.startswith("T_")] == ["T_BS"]          # the loader takes the first T_* key
        t_bs = cfg["T_BS"]
        assert (t_bs["rows"], t_bs["cols"]) == (4, 4)
        assert len(t_bs["data"]) == 16 and all(isinstance(v, float) for v in t_bs["data"])
        T = np.array(t_bs["data"]).reshape(4, 4)
        if cam == "cam0":
            np.testing.assert_allclose(T, np.eye(4), atol=0)               # body = cam0
        else:
            assert T[0, 3] == pytest.approx(DOC_BASELINE_M, abs=1e-5)      # B_from_C, metres, +x
            rig.check_rotation(T[:3, :3])
        assert cfg["resolution"] == list(SIZE) and all(isinstance(v, int) for v in cfg["resolution"])
        assert len(cfg["intrinsics"]) == 4 and all(isinstance(v, float) for v in cfg["intrinsics"])
        assert cfg["distortion_coefficients"] == [0.0, 0.0, 0.0, 0.0]
        assert cfg["distortion_model"] == "radial-tangential"
        assert cfg["camera_model"] == "pinhole"
        assert cfg["sensor_type"] == "camera"
        assert cfg["rate_hz"] == 30
        socket = rig.CAM_B if cam == "cam0" else rig.CAM_C
        assert socket in cfg["comment"]
        assert not any(ord(ch) > 127 for ch in text)

    def test_camera_yaml_intrinsics_are_the_calibration(self, tiny_dataset: TinyDataset, calib: Dict[str, Any]) -> None:
        for cam, socket in (("cam0", rig.CAM_B), ("cam1", rig.CAM_C)):
            cfg = dataset.parse_simple_yaml((tiny_dataset.mav0 / cam / "sensor.yaml").read_text(encoding="ascii"))
            K = rig.intrinsics_for(calib, socket, *SIZE)
            assert cfg["intrinsics"] == pytest.approx([K[0, 0], K[1, 1], K[0, 2], K[1, 2]], abs=1e-9)

    def test_imu_sensor_yaml(self, tiny_dataset: TinyDataset, calib: Dict[str, Any]) -> None:
        text = (tiny_dataset.mav0 / "imu0" / "sensor.yaml").read_text(encoding="ascii")
        cfg = dataset.parse_simple_yaml(text)
        for key in IMU_YAML_MANDATORY + ("sensor_type", "comment"):
            assert key in cfg, key
        assert cfg["sensor_type"] == "imu"
        assert cfg["rate_hz"] == 200
        # the format doc (section 7) ships NVIDIA's RealSense placeholders until a bench measurement
        for key, value in DOC_IMU_NOISE.items():
            assert isinstance(cfg[key], float) and cfg[key] > 0.0
            assert cfg[key] == pytest.approx(value, rel=1e-12)
        assert tuple(dataset.IMU_NOISE_KEYS) == tuple(DOC_IMU_NOISE)
        assert dataset.PLACEHOLDER_IMU_NOISE == DOC_IMU_NOISE
        T = np.array(cfg["T_BS"]["data"], dtype=np.float64).reshape(4, 4)
        np.testing.assert_allclose(T, rig.rig_from_imu(calib, rig.CAM_B), atol=1e-9)   # B_from_IMU, metres
        assert rig.MISSING_IMU_COMMENT[:20] not in cfg["comment"]

    @pytest.mark.parametrize("rel", ["mav0/cam0/sensor.yaml", "mav0/cam1/sensor.yaml", "mav0/imu0/sensor.yaml"])
    def test_yaml_floats_load_as_floats(self, tiny_dataset: TinyDataset, rel: str) -> None:
        """YAML 1.1: a float needs a dot in the mantissa and a signed exponent (``1e-05`` is a string)."""
        text = (tiny_dataset.root / rel).read_text(encoding="ascii")
        tokens = list(float_tokens(text))
        for match in tokens:
            assert match.group(2) is not None, "mantissa without a dot: {}".format(match.group(0))
            assert match.group(3) in ("+", "-"), "unsigned exponent: {}".format(match.group(0))
        if rel.endswith("imu0/sensor.yaml"):
            assert tokens, "expected exponent-form noise values in imu0/sensor.yaml"
        assert dataset.yaml_float(1e-5) == "1.0e-05"
        assert dataset.yaml_float(0.0) == "0.0"
        for value in (3.6211951458325785e-05, 799.1265869140625, -5.06e-05):
            assert float(dataset.yaml_float(value)) == value

    def test_csv_files(self, tiny_dataset: TinyDataset) -> None:
        for cam in ("cam0", "cam1"):
            raw = (tiny_dataset.mav0 / cam / "data.csv").read_bytes()
            assert b"\r" not in raw and raw.endswith(b"\n") and not raw.startswith(b"\xef\xbb\xbf")
            lines = raw.decode("ascii").splitlines()
            assert lines[0] == DOC_CAM_CSV_HEADER == dataset.CAM_CSV_HEADER
            rows = [line.split(",") for line in lines[1:]]
            assert rows == [[str(ts), "{}.png".format(ts)] for ts in tiny_dataset.frame_ts]
        raw = (tiny_dataset.mav0 / "imu0" / "data.csv").read_bytes()
        assert b"\r" not in raw
        lines = raw.decode("ascii").splitlines()
        assert lines[0] == DOC_IMU_CSV_HEADER == dataset.IMU_CSV_HEADER
        rows = [line.split(",") for line in lines[1:]]
        assert [int(r[0]) for r in rows] == tiny_dataset.imu_ts
        assert all(len(r) == 7 for r in rows)
        for r in rows:
            assert [float(v) for v in r[1:4]] == pytest.approx(GYRO)       # gyro first (rad/s)
            assert [float(v) for v in r[4:7]] == pytest.approx(ACCEL)      # then accel (m/s^2)
        assert all(b > a for a, b in zip(tiny_dataset.imu_ts, tiny_dataset.imu_ts[1:]))

    def test_png_format(self, tiny_dataset: TinyDataset) -> None:
        import cv2

        path = tiny_dataset.mav0 / "cam0" / "data" / "{}.png".format(tiny_dataset.frame_ts[0])
        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        assert img is not None and img.dtype == np.uint8 and img.ndim == 2 and img.shape == SHAPE
        loaded = dataset.load_gray_png(path, expected_shape=SHAPE)
        assert loaded.dtype == np.uint8 and loaded.shape == SHAPE and loaded.flags["C_CONTIGUOUS"]
        np.testing.assert_array_equal(loaded, img)
        with pytest.raises(dataset.DatasetError):
            dataset.load_gray_png(path, expected_shape=(800, 1280))

    def test_reader_matches_what_was_written(self, tiny_dataset: TinyDataset) -> None:
        reader = dataset.EurocReader(tiny_dataset.root)
        assert reader.root == tiny_dataset.root and reader.mav0 == tiny_dataset.mav0
        assert reader.frame_count == N_FRAMES
        assert reader.imu_count == len(tiny_dataset.imu_ts)
        assert reader.frame_timestamps.tolist() == tiny_dataset.frame_ts
        assert reader.imu_timestamps.tolist() == tiny_dataset.imu_ts
        ts, gyro, accel = reader.imu_arrays
        assert ts.shape == (len(tiny_dataset.imu_ts),) and gyro.shape == accel.shape == (len(tiny_dataset.imu_ts), 3)
        np.testing.assert_allclose(gyro, np.tile(GYRO, (len(tiny_dataset.imu_ts), 1)))
        np.testing.assert_allclose(accel, np.tile(ACCEL, (len(tiny_dataset.imu_ts), 1)))
        assert reader.first_ts_ns == tiny_dataset.imu_ts[0] and reader.last_ts_ns == tiny_dataset.imu_ts[-1]
        assert reader.duration_s == pytest.approx((tiny_dataset.imu_ts[-1] - tiny_dataset.imu_ts[0]) / 1e9)
        assert reader.dataset is not None and reader.dataset["counts"]["stereo_pairs"] == N_FRAMES
        info = reader.rig
        assert info is not None
        assert info.cam0.resolution == SIZE and info.cam1.resolution == SIZE
        assert info.cam0.undistorted and info.cam0.cuvslam_distortion() == ("Pinhole", [])
        assert info.baseline_m() == pytest.approx(DOC_BASELINE_M, abs=1e-5)
        assert info.rig_from_cam1()[0, 3] == pytest.approx(DOC_BASELINE_M, abs=1e-5)
        assert np.max(np.abs(info.rig_from_imu()[:3, 3])) < 1.2e-3      # IMU next to the left camera
        assert info.imu.rate_hz == IMU_RATE_HZ
        # the reader accepts the mav0 directory too
        assert dataset.EurocReader(tiny_dataset.mav0).frame_count == N_FRAMES

    def test_reader_validate_ok(self, tiny_dataset: TinyDataset) -> None:
        report = dataset.EurocReader(tiny_dataset.root).validate(check_images=True, decode_images=None)
        assert report.ok, str(report)
        assert report.errors == [] and report.warnings == []

    def test_merged_order(self, tiny_dataset: TinyDataset) -> None:
        reader = dataset.EurocReader(tiny_dataset.root)
        items = list(reader.merged(load_images=True, with_imu=True))
        assert len(items) == N_FRAMES + len(tiny_dataset.imu_ts)
        frames_seen = 0
        last_frame_ts: Optional[int] = None
        for item in items:
            if item.kind == "stereo":
                assert item.ts_ns == tiny_dataset.frame_ts[frames_seen] and item.index == frames_seen
                images = item.images(SHAPE)
                assert len(images) == 2 and all(im.dtype == np.uint8 and im.shape == SHAPE for im in images)
                last_frame_ts = item.ts_ns
                frames_seen += 1
            else:
                assert item.kind == "imu"
                if last_frame_ts is not None:
                    assert item.ts_ns > last_frame_ts            # IMU with ts <= frame came before that frame
                if frames_seen < N_FRAMES:
                    assert item.ts_ns <= tiny_dataset.frame_ts[frames_seen]
        assert frames_seen == N_FRAMES
        imu_before_first = [it for it in items[:items.index(next(i for i in items if i.kind == "stereo"))]]
        assert len(imu_before_first) == tiny_dataset.imu_up_to(tiny_dataset.frame_ts[0]) == 2
        assert sum(1 for it in items if it.kind == "imu") == len(tiny_dataset.imu_ts)
        without_imu = list(reader.merged(load_images=False, with_imu=False))
        assert [it.kind for it in without_imu] == ["stereo"] * N_FRAMES

    def test_validate_reports_extra_row(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "extra_row"
        shutil.copytree(tiny_dataset.root, root)
        with open(root / "mav0" / "cam0" / "data.csv", "a", encoding="ascii", newline="\n") as fh:
            fh.write("{0},{0}.png\n".format(T0 + N_FRAMES * FRAME_PERIOD_NS))
        report = dataset.EurocReader(root).validate()
        assert not report.ok
        assert any("rows" in text for text in report.errors)

    def test_validate_reports_missing_png(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "missing_png"
        shutil.copytree(tiny_dataset.root, root)
        (root / "mav0" / "cam1" / "data" / "{}.png".format(tiny_dataset.frame_ts[2])).unlink()
        report = dataset.EurocReader(root).validate(check_images=True)
        assert not report.ok
        assert any("missing" in text.lower() for text in report.errors)

    def test_validate_reports_imu_going_backwards(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "imu_order"
        shutil.copytree(tiny_dataset.root, root)
        path = root / "mav0" / "imu0" / "data.csv"
        lines = path.read_text(encoding="ascii").splitlines()
        lines.insert(5, "{},0,0,0,0,9.81,0".format(tiny_dataset.imu_ts[0] - IMU_PERIOD_NS))
        path.write_text("\n".join(lines) + "\n", encoding="ascii")
        report = dataset.EurocReader(root).validate()
        assert not report.ok
        assert any("decrease" in text.lower() for text in report.errors)

    def test_reader_refuses_recalibrated_yaml(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "recalibrated"
        shutil.copytree(tiny_dataset.root, root)
        (root / "mav0" / "cam0" / "sensor_recalibrated.yaml").write_text("sensor_type: camera\n", encoding="ascii")
        with pytest.raises(dataset.DatasetError, match="sensor_recalibrated"):
            dataset.EurocReader(root)

    def test_reader_refuses_string_floats(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "string_float"
        shutil.copytree(tiny_dataset.root, root)
        path = root / "mav0" / "imu0" / "sensor.yaml"
        text = path.read_text(encoding="ascii")
        patched = re.sub(r"^gyroscope_random_walk: \S+", "gyroscope_random_walk: 1e-05", text, flags=re.M)
        assert patched != text
        path.write_text(patched, encoding="ascii")
        with pytest.raises(dataset.DatasetError, match="not a number"):
            dataset.EurocReader(root)
        lenient = dataset.EurocReader(root, strict=False)
        assert lenient.rig is None and not lenient.validate().ok

    def test_writer_rejects_bad_input(self, tmp_path: Path, rig_info: dataset.RigInfo) -> None:
        img = np.zeros(SHAPE, np.uint8)
        with dataset.EurocWriter(tmp_path / "neg", rig_info, cam_rate_hz=FPS, imu_rate_hz=IMU_RATE_HZ) as writer:
            assert writer.add_frame(T0, img, img)
            assert not writer.add_frame(T0 - 1, img, img)                                  # older frame
            assert writer.drops["frames_out_of_order"] == 1
            assert not writer.add_frame(T0 + FRAME_PERIOD_NS, img, img, right_ts_ns=T0 + FRAME_PERIOD_NS + 2_000_000)
            assert writer.drops["stereo_sync_violations"] == 1                             # > 1 ms L/R skew
            with pytest.raises(dataset.DatasetError):
                writer.add_frame(T0 + 2 * FRAME_PERIOD_NS, np.zeros((200, 320), np.uint8), img)
            with pytest.raises(dataset.DatasetError):
                writer.add_frame(T0 + 2 * FRAME_PERIOD_NS, img.astype(np.uint16), img)
            assert writer.add_imu(T0, GYRO, ACCEL)
            assert not writer.add_imu(T0 - 1, GYRO, ACCEL)
            assert writer.drops["imu_out_of_order"] == 1
            with pytest.raises(dataset.DatasetError):
                writer.add_imu(T0 + 1, (0.0, 0.0), ACCEL)
        summary = writer.summary
        assert summary is not None
        assert summary["counts"]["stereo_pairs"] == 1 and summary["counts"]["imu_samples"] == 1
        assert dataset.EurocReader(tmp_path / "neg").validate().ok
        with pytest.raises(RuntimeError):
            writer.add_frame(T0 + 3 * FRAME_PERIOD_NS, img, img)


# --------------------------------------------------------------------------------------
# Tracking with the fake in both modes
# --------------------------------------------------------------------------------------

class TestTracking:
    def test_stereo_mode_rows(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        frames = synthetic_frames(5)
        events: List[tracking.PoseEvent] = []
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", frames, None, track_config(),
                                        on_pose=events.append)
        assert summary["mode"] == "stereo" and summary["odometry_mode"] == "Multicamera"
        assert summary["fake"] is True and summary["cuvslam_version"] == "17.0.0+fake"
        assert summary["frames_seen"] == summary["frames_attempted"] == summary["frames_tracked"] == 5
        assert summary["frames_failed"] == 0 and summary["resets"] == 0
        assert summary["tum_rows"] == len(summary["rows"]) == len(events) == 5
        assert [row[0] for row in summary["rows"]] == [f[0] for f in frames]
        assert summary["rows"][0][1:] == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))   # identity start
        assert [e.tum_row for e in events] == summary["rows"]
        assert summary["imu_registered"] == 0 and summary["imu_held_back"] == 0
        assert summary["stopped_by"] == "end"
        assert summary["path_length_m"] > 0.0                                                  # constant-velocity fake

    def test_stereo_mode_ignores_imu_source(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", synthetic_frames(4), iter(synthetic_imu(30)),
                                        track_config(coast=False))
        assert summary["imu_source_ignored"] is True
        assert summary["imu_registered"] == 0 and summary["tum_rows"] == 4
        # with coasting on (the default) the samples are read for the gyro but still never registered
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", synthetic_frames(4), iter(synthetic_imu(30)),
                                        track_config())
        assert summary["imu_source_ignored"] is False
        assert summary["imu_registered"] == 0 and summary["tum_rows"] == 4 and summary["frames_coasted"] == 0

    def test_vio_mode_iterator_holds_back_one_sample(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        vio_rig, _info = build_fake_rig(vslam, calib, "vio")
        frames = synthetic_frames(5)
        imu = synthetic_imu(40)                       # 28 samples up to the last frame, 12 after it
        expected = sum(1 for s in imu if s[0] <= frames[-1][0])
        assert expected == 28
        summary = tracking.run_tracking(vslam, vio_rig, "vio", frames, iter(imu), track_config())
        assert summary["odometry_mode"] == "Inertial"
        assert summary["frames_tracked"] == summary["tum_rows"] == 5
        assert summary["imu_registered"] == expected
        assert summary["imu_held_back"] == 1          # an iterator is read only one sample ahead
        assert summary["imu_dropped_old"] == summary["imu_out_of_order"] == 0
        assert summary["frames_without_imu"] == 0
        assert summary["gravity"] == pytest.approx([0.0, 9.81, 0.0])

    def test_vio_mode_queue_and_merged_hold_back_the_tail(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        frames = synthetic_frames(5)
        imu = synthetic_imu(40)
        up_to_last = sum(1 for s in imu if s[0] <= frames[-1][0])
        tail = len(imu) - up_to_last
        assert tail == 12
        imu_queue: "queue.Queue[Any]" = queue.Queue()
        for sample in imu:
            imu_queue.put(sample)
        vio_rig, _info = build_fake_rig(vslam, calib, "vio")
        summary = tracking.run_tracking(vslam, vio_rig, "vio", frames, imu_queue, track_config())
        assert summary["imu_registered"] == up_to_last and summary["imu_held_back"] == tail
        assert summary["tum_rows"] == 5 and imu_queue.empty()
        vio_rig2, _info = build_fake_rig(vslam, calib, "vio")
        summary2 = tracking.run_tracking(vslam, vio_rig2, "vio", merged_stream(frames, imu), None, track_config())
        assert summary2["imu_registered"] == up_to_last and summary2["imu_held_back"] == tail
        assert summary2["tum_rows"] == 5
        assert summary2["rows"] == summary["rows"]

    @pytest.mark.parametrize("mode", ["stereo", "vio"])
    def test_dataset_tracking(self, mode: str, vslam: types.ModuleType, tiny_dataset: TinyDataset,
                              track_script: types.ModuleType) -> None:
        reader = dataset.EurocReader(tiny_dataset.root)
        rig_obj = track_script.rig_from_dataset(vslam, reader.rig, mode)
        assert len(rig_obj.imus) == (1 if mode == "vio" else 0)
        stream = reader.merged(load_images=True, with_imu=(mode == "vio"))
        summary = tracking.run_tracking(vslam, rig_obj, mode, stream, None, track_config())
        assert summary["frames_seen"] == summary["frames_tracked"] == summary["tum_rows"] == N_FRAMES
        assert summary["frames_failed"] == 0
        assert [row[0] for row in summary["rows"]] == tiny_dataset.frame_ts
        if mode == "vio":
            assert summary["imu_registered"] == tiny_dataset.imu_up_to(tiny_dataset.last_frame_ts)
            assert summary["imu_held_back"] == IMU_TAIL
            assert summary["gravity"] == pytest.approx([0.0, 9.81, 0.0])
        else:
            assert summary["imu_registered"] == 0 and summary["imu_held_back"] == 0

    def test_warmup_and_max_frames(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", synthetic_frames(6), None,
                                        track_config(warmup_frames=2))
        assert summary["frames_warmup"] == 2 and summary["frames_tracked"] == summary["tum_rows"] == 4
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", synthetic_frames(6), None,
                                        track_config(max_frames=3))
        assert summary["frames_tracked"] == summary["tum_rows"] == 3 and summary["stopped_by"] == "max_frames"

    def test_out_of_order_frames_are_skipped(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        img = np.zeros(SHAPE, np.uint8)
        frames = [(T0, img, img), (T0, img, img), (T0 - 1, img, img), (T0 + FRAME_PERIOD_NS, img, img)]
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", frames, None, track_config())
        assert summary["frames_seen"] == 4 and summary["frames_out_of_order"] == 2
        assert summary["frames_tracked"] == summary["tum_rows"] == 2

    def test_rig_mode_mismatch(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        with pytest.raises(tracking.TrackingError):
            tracking.run_tracking(vslam, stereo_rig, "vio", synthetic_frames(2), None, track_config())
        with pytest.raises(ValueError):
            tracking.normalize_mode("mono")
        assert tracking.normalize_mode("stereo-inertial") == "vio" and tracking.normalize_mode("VO") == "stereo"

    def test_tum_write_read(self, vslam: types.ModuleType, calib: Dict[str, Any], tmp_path: Path) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        summary = tracking.run_tracking(vslam, stereo_rig, "stereo", synthetic_frames(4), None, track_config())
        path = tracking.write_tum(tmp_path / "traj.tum", summary["rows"], header=["unit test"])
        raw = path.read_bytes()
        assert b"\r" not in raw and raw.endswith(b"\n")
        lines = raw.decode("ascii").splitlines()
        assert lines[0] == tracking.TUM_HEADER == "# timestamp_s tx ty tz qx qy qz qw"
        assert lines[1] == "# unit test"
        data = [line for line in lines if not line.startswith("#")]
        assert len(data) == 4 and all(len(line.split()) == 8 for line in data)
        assert data[0].split()[0] == tracking.format_tum_timestamp(T0) == "1758000000.000000000"
        back = tracking.read_tum(path)
        assert [row[0] for row in back] == [row[0] for row in summary["rows"]]
        for got, want in zip(back, summary["rows"]):
            assert got[1:] == pytest.approx(want[1:], abs=1e-6)
        assert tracking.parse_tum_timestamp("1758000000.033333333") == T0 + 33_333_333
        with pytest.raises(ValueError):
            tracking.format_tum_timestamp(-1)


# --------------------------------------------------------------------------------------
# fake_cuvslam input validation
# --------------------------------------------------------------------------------------

class TestFakeCuvslam:
    @pytest.fixture()
    def tracker(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> Any:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        return vslam.Tracker(stereo_rig, vslam.Tracker.OdometryConfig())

    def test_identity(self, vslam: types.ModuleType) -> None:
        assert vslam.IS_FAKE is True
        assert vslam.get_version() == ("17.0.0+fake", 17, 0, 0)
        assert vslam.Tracker.OdometryMode.Inertial != vslam.Tracker.OdometryMode.Multicamera
        assert cuvslam_oak.is_fake(vslam) and not cuvslam_oak.is_fake(np)

    @pytest.mark.parametrize("count", [0, 1, 3])
    def test_wrong_image_count_raises(self, tracker: Any, count: int) -> None:
        img = np.zeros(SHAPE, np.uint8)
        with pytest.raises(ValueError):          # type only: the real binding's messages are unverified (API doc 3.5)
            tracker.track(T0, [img] * count)

    def test_images_must_be_a_sequence(self, tracker: Any) -> None:
        with pytest.raises(TypeError):
            tracker.track(T0, np.zeros(SHAPE, np.uint8))

    def test_wrong_image_shape_or_dtype_raises(self, tracker: Any) -> None:
        img = np.zeros(SHAPE, np.uint8)
        with pytest.raises(ValueError):
            tracker.track(T0, [img, np.zeros((800, 1280), np.uint8)])
        with pytest.raises(ValueError):
            tracker.track(T0, [img.astype(np.float32), img])

    def test_timestamps_must_increase(self, tracker: Any) -> None:
        img = np.zeros(SHAPE, np.uint8)
        estimate, slam_pose = tracker.track(T0, [img, img])
        assert estimate.timestamp_ns == T0 and estimate.world_from_rig is not None and slam_pose is None
        assert list(estimate.world_from_rig.pose.translation) == pytest.approx([0.0, 0.0, 0.0])
        with pytest.raises(ValueError):
            tracker.track(T0, [img, img])
        with pytest.raises(ValueError):
            tracker.track(T0 - 1, [img, img])
        estimate2, _ = tracker.track(T0 + FRAME_PERIOD_NS, (img, img))          # a tuple is fine too
        assert estimate2.timestamp_ns == T0 + FRAME_PERIOD_NS

    def test_register_imu_requires_inertial_mode(self, vslam: types.ModuleType, tracker: Any) -> None:
        measurement = vslam.ImuMeasurement(timestamp_ns=T0, linear_accelerations=list(ACCEL),
                                           angular_velocities=list(GYRO))
        with pytest.raises(ValueError):
            tracker.register_imu_measurement(0, measurement)

    def test_imu_measurement_needs_three_components(self, vslam: types.ModuleType) -> None:
        with pytest.raises((TypeError, ValueError)):
            vslam.ImuMeasurement(timestamp_ns=T0, linear_accelerations=[0.0, 9.81], angular_velocities=list(GYRO))
        with pytest.raises((TypeError, ValueError)):
            vslam.ImuMeasurement(timestamp_ns=T0, linear_accelerations=list(ACCEL), angular_velocities=[0.0])

    def test_rig_validation(self, vslam: types.ModuleType, calib: Dict[str, Any]) -> None:
        stereo_rig, _info = build_fake_rig(vslam, calib, "stereo")
        cfg = vslam.Tracker.OdometryConfig(odometry_mode=vslam.Tracker.OdometryMode.Inertial)
        with pytest.raises(ValueError):
            vslam.Tracker(stereo_rig, cfg)                          # Inertial without an IMU
        with pytest.raises(ValueError):
            vslam.Tracker(vslam.Rig(cameras=[], imus=[]), vslam.Tracker.OdometryConfig())


# --------------------------------------------------------------------------------------
# CLI smoke tests (subprocess)
# --------------------------------------------------------------------------------------

class TestCli:
    def test_record_help(self) -> None:
        proc = run_cli(RECORD_SCRIPT, "--help")
        assert proc.returncode == 0, proc.stderr
        assert proc.stdout.startswith("usage:")
        for flag in ("--device-id", "--duration", "--headless", "--width", "--imu-rate", "--out", "--raw"):
            assert flag in proc.stdout, flag
        assert stderr_lines(proc) == []

    def test_track_help(self) -> None:
        proc = run_cli(TRACK_SCRIPT, "--help")
        assert proc.returncode == 0, proc.stderr
        assert proc.stdout.startswith("usage:")
        for flag in ("--dataset", "--live", "--mode", "--fake-cuvslam", "--out", "--summary"):
            assert flag in proc.stdout, flag
        assert stderr_lines(proc) == []

    def test_record_device_id_nonexistent_exits_2(self, tmp_path: Path) -> None:
        out_dir = tmp_path / "rec"
        proc = run_cli(RECORD_SCRIPT, "--device-id", "nonexistent", "--duration", "1", "--out", out_dir)
        assert proc.returncode == 2, proc.stderr
        lines = stderr_lines(proc)
        assert len(lines) == 1, proc.stderr
        assert "nonexistent" in lines[0] or "No OAK device found" in lines[0]
        assert "Traceback" not in proc.stderr
        assert "Device:" not in proc.stdout               # print_device_summary never ran: nothing was opened
        assert not out_dir.exists()                       # the empty output folder is removed again

    def test_track_dataset_nonexistent_exits_2(self, tmp_path: Path) -> None:
        missing = tmp_path / "nonexistent"
        proc = run_cli(TRACK_SCRIPT, "--dataset", missing)
        assert proc.returncode == 2, proc.stderr
        lines = stderr_lines(proc)
        assert len(lines) == 1 and "nonexistent" in lines[0]
        assert proc.stdout.strip() == ""

    def test_track_without_a_source_exits_2(self) -> None:
        proc = run_cli(TRACK_SCRIPT, "--fake-cuvslam")
        assert proc.returncode == 2
        assert len(stderr_lines(proc)) == 1

    def test_track_fake_on_dataset(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        tum_path = tmp_path / "trajectory.tum"
        proc = run_cli(TRACK_SCRIPT, "--dataset", tiny_dataset.root, "--fake-cuvslam", "--out", tum_path, "--summary")
        assert proc.returncode == 0, proc.stderr
        assert "FAKE cuVSLAM in use" in proc.stderr
        assert "Traceback" not in proc.stderr
        assert re.search(r"\bvio\b", proc.stdout), proc.stdout           # the automatic mode choice is reported
        assert tum_path.is_file()
        lines = tum_path.read_text(encoding="ascii").splitlines()
        assert lines[0] == tracking.TUM_HEADER
        assert any("FAKE" in line for line in lines if line.startswith("#"))
        assert any("OdometryMode.Inertial" in line for line in lines if line.startswith("#"))
        rows = tracking.read_tum(tum_path)
        assert len(rows) == N_FRAMES
        assert [row[0] for row in rows] == tiny_dataset.frame_ts
        assert rows[0][1:] == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
        summary_path = tmp_path / "trajectory_summary.json"
        assert summary_path.is_file()
        doc = json.loads(summary_path.read_text(encoding="utf-8"))
        assert doc["generator"]["fake"] is True
        assert doc["mode"] == "vio" and doc["odometry_mode"] == "Inertial"     # auto: the dataset has IMU rows
        assert doc["outputs"]["tum_rows"] == N_FRAMES
        assert doc["tracking"]["frames_tracked"] == N_FRAMES
        assert doc["tracking"]["imu_registered"] == tiny_dataset.imu_up_to(tiny_dataset.last_frame_ts)
        assert doc["source"]["stereo_pairs"] == N_FRAMES
        assert doc["source"]["resolution"] == list(SIZE) and doc["source"]["imu_samples"] == len(tiny_dataset.imu_ts)

    def test_track_fake_stereo_positional_max_frames(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        proc = run_cli(TRACK_SCRIPT, tiny_dataset.root, "--fake-cuvslam", "--mode", "stereo", "--max-frames", "4",
                       "--out", tmp_path, "--summary")
        assert proc.returncode == 0, proc.stderr
        assert re.search(r"\bstereo\b", proc.stdout), proc.stdout
        tum_path = tmp_path / "trajectory_stereo.tum"        # a directory --out receives trajectory_<mode>.tum
        assert tum_path.is_file()
        doc = json.loads((tmp_path / "trajectory_stereo_summary.json").read_text(encoding="utf-8"))
        assert doc["mode"] == "stereo" and doc["odometry_mode"] == "Multicamera"
        assert doc["tracking"]["stopped_by"] == "max_frames" and doc["outputs"]["tum_rows"] == 4
        rows = tracking.read_tum(tum_path)
        assert len(rows) == 4
        assert [row[0] for row in rows] == tiny_dataset.frame_ts[:4]
        assert any("OdometryMode.Multicamera" in line for line in tum_path.read_text(encoding="ascii").splitlines())

    def test_track_vio_refused_without_imu_rows(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        root = tmp_path / "no_imu"
        shutil.copytree(tiny_dataset.root, root)
        (root / "mav0" / "imu0" / "data.csv").write_text(dataset.IMU_CSV_HEADER + "\n", encoding="ascii")
        meta_path = root / "dataset.json"
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta["counts"]["imu_samples"] = 0                    # keep dataset.json consistent with the empty csv
        meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        proc = run_cli(TRACK_SCRIPT, "--dataset", root, "--fake-cuvslam", "--mode", "vio", "--out", tmp_path / "x.tum")
        assert proc.returncode == 2, proc.stderr
        assert len(stderr_lines(proc)) == 1 and "--mode stereo" in proc.stderr

    def test_track_without_cuvslam_exits_3(self, tiny_dataset: TinyDataset, tmp_path: Path) -> None:
        if importlib.util.find_spec("cuvslam") is not None:
            pytest.skip("a real cuvslam wheel is installed on this host")
        proc = run_cli(TRACK_SCRIPT, "--dataset", tiny_dataset.root, "--out", tmp_path / "x.tum")
        assert proc.returncode == 3, proc.stderr
        lines = stderr_lines(proc)
        assert len(lines) == 1 and "--fake-cuvslam" in lines[0] and "methods/cuvslam-vio/docs/cuvslam-setup.md" in lines[0]
        assert not (tmp_path / "x.tum").exists()
