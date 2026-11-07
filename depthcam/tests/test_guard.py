"""Starvation guard of ``cuvslam_oak.tracking.run_tracking`` against the fake cuvslam.

Scripted trajectories (``fake_cuvslam.set_default_pose_source``), observation counts
(``set_default_observations_source``) and covariances (``set_default_covariance_source``)
exercise every guard rule (starved / degenerate / jump / spin), the re-anchoring across guarded
gaps (zero motion across the gap), the engine reset after ``guard_reset_after`` consecutive
guarded frames (stereo, and vio where the IMU feeder must follow the new tracker),
``guard=False`` (today's behaviour) and the CLI flags of ``methods/cuvslam-vio/08_cuvslam_track.py``.

No camera is opened (``tests/conftest.py`` guards ``depthai.Device``); the CLI runs in a
subprocess with ``--fake-cuvslam`` on a tiny EuRoC dataset written into ``tmp_path``.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import types
import warnings
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import pytest

import cuvslam_oak
from cuvslam_oak import dataset, rig, tracking

REPO_ROOT = Path(__file__).resolve().parents[1]
TRACK_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py"
CALIBRATION = REPO_ROOT / "tests" / "data" / "calibration_OAKDS2EXAMPLEID000.json"
PYTHON = sys.executable
SUBPROCESS_TIMEOUT_S = 60.0

SIZE = (640, 400)
SHAPE = (SIZE[1], SIZE[0])
FPS = 30.0
IMU_RATE_HZ = 200.0
T0 = 1_758_000_000_000_000_000
FRAME_PERIOD_NS = 33_333_333
IMU_PERIOD_NS = 5_000_000
GYRO = (0.001, -0.002, 0.003)
ACCEL = (0.0, 9.81, 0.0)
STEP_M = 0.01                     # scripted forward motion per frame (0.3 m/s at 30 fps)
STEP_TOL = 1e-6                   # cuvslam.Pose.translation is float32: 0.01 comes back as 0.0100000054
IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)
FAKE_GRID_OBSERVATIONS = 36       # fake_cuvslam.OBSERVATION_GRID ** 2 per camera


# --------------------------------------------------------------------------------------
# Fixtures and helpers
# --------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def vslam() -> types.ModuleType:
    module = cuvslam_oak.import_cuvslam(prefer_fake=True)
    assert cuvslam_oak.is_fake(module), "the suite must run against cuvslam_oak.fake_cuvslam"
    return module


@pytest.fixture(scope="module")
def calib() -> Dict[str, Any]:
    if not CALIBRATION.is_file():
        pytest.skip("{} is missing".format(CALIBRATION.relative_to(REPO_ROOT).as_posix()))
    return rig.load_calibration_json(CALIBRATION)


@pytest.fixture(autouse=True)
def clean_fake_sources(vslam: types.ModuleType) -> Any:
    """The scripted sources are module-wide defaults of the fake: never leak them into other tests."""
    vslam.set_default_pose_source(None)
    vslam.set_default_observations_source(None)
    vslam.set_default_covariance_source(None)
    vslam.reset_call_counts()
    yield
    vslam.set_default_pose_source(None)
    vslam.set_default_observations_source(None)
    vslam.set_default_covariance_source(None)


def build_rig(vslam: Any, calib: Dict[str, Any], mode: str) -> Any:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", rig.CalibrationWarning)
        rig_obj, _info = rig.build_rig(calib, SIZE, mode, imu_rate_hz=IMU_RATE_HZ, vslam=vslam)
    return rig_obj


def config(**overrides: Any) -> tracking.TrackConfig:
    base: Dict[str, Any] = {"verbose": False, "warm_up_gpu": False, "warmup_frames": 0, "coast": False}
    base.update(overrides)
    return tracking.TrackConfig(**base)


def frames(count: int) -> List[Tuple[int, np.ndarray, np.ndarray]]:
    return [(T0 + i * FRAME_PERIOD_NS, np.zeros(SHAPE, np.uint8), np.zeros(SHAPE, np.uint8)) for i in range(count)]


def imu_samples(count: int) -> List[Tuple[int, Tuple[float, ...], Tuple[float, ...]]]:
    return [(T0 - IMU_PERIOD_NS + j * IMU_PERIOD_NS, GYRO, ACCEL) for j in range(count)]


def frame_index(ts_ns: int) -> int:
    return int(round((int(ts_ns) - T0) / FRAME_PERIOD_NS))


def walk(vslam: Any, jumps: Optional[Dict[int, float]] = None, spins: Optional[Dict[int, float]] = None,
         relative: bool = False) -> Callable[[int, int], Any]:
    """Pose source: ``STEP_M`` forward per frame along +z, plus *jumps* (frame index -> extra metres on that
    frame only) and *spins* (frame index -> yaw in degrees on that frame only).  *relative* restarts the
    distance from the origin frame of the engine, as the real cuVSLAM does after a (re)start."""
    jumps = jumps or {}
    spins = spins or {}

    def source(timestamp_ns: int, origin_ns: int) -> Any:
        i = frame_index(timestamp_ns)
        base = i - frame_index(origin_ns) if relative else i
        z = STEP_M * base + jumps.get(i, 0.0)
        yaw = math.radians(spins.get(i, 0.0))
        q = (0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0))
        return vslam.Pose(q, (0.0, 0.0, z))

    return source


def positions(rows: List[Any]) -> np.ndarray:
    return np.array([row[1:4] for row in rows], dtype=np.float64)


def steps(rows: List[Any]) -> np.ndarray:
    return np.linalg.norm(np.diff(positions(rows), axis=0), axis=1)


def row_timestamps(summary: Dict[str, Any]) -> List[int]:
    return [int(row[0]) for row in summary["rows"]]


def run(vslam: Any, rig_obj: Any, mode: str, stream: Any, imu: Any = None, **overrides: Any) -> Dict[str, Any]:
    return tracking.run_tracking(vslam, rig_obj, mode, stream, imu, config(**overrides))


def assert_no_other_reasons(summary: Dict[str, Any], *allowed: str) -> None:
    for reason in ("starved", "degenerate", "jump", "spin"):
        if reason not in allowed:
            assert summary["guard_" + reason] == 0, reason
    assert summary["frames_guarded"] == sum(summary["guard_" + r] for r in ("starved", "degenerate", "jump", "spin"))


# --------------------------------------------------------------------------------------
# Guard rules and re-anchoring
# --------------------------------------------------------------------------------------

class TestGuard:
    def test_jump_frame_is_guarded_and_trajectory_stays_continuous(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, jump_at = 30, 10
        vslam.set_default_pose_source(walk(vslam, jumps={jump_at: 3.0}))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n))
        assert summary["frames_attempted"] == n
        assert summary["frames_tracked"] == summary["tum_rows"] == len(summary["rows"]) == n - 1
        assert summary["frames_guarded"] == summary["guard_jump"] == 1
        assert_no_other_reasons(summary, "jump")
        assert summary["guard_resets"] == 0 and summary["resets"] == 0 and summary["segments"] == [0]
        assert summary["guard_gaps"] == [(jump_at, 1)]
        assert T0 + jump_at * FRAME_PERIOD_NS not in row_timestamps(summary)
        assert steps(summary["rows"]).max() < 0.05
        P = positions(summary["rows"])
        # zero motion across the gap: the frame after the jump sits where the last good frame was ...
        assert P[jump_at] == pytest.approx(P[jump_at - 1], abs=1e-9)
        # ... and the path then continues at STEP_M per frame, two frame steps short of the un-jumped path
        assert P[-1] == pytest.approx([0.0, 0.0, STEP_M * (n - 1) - 2 * STEP_M], abs=1e-6)
        assert abs(P[-1][2] - STEP_M * (n - 1)) < 3 * STEP_M
        assert summary["config"]["guard"] is True and summary["config"]["max_speed_mps"] == tracking.DEFAULT_MAX_SPEED_MPS
        assert summary["config"]["min_observations"] == 0 and summary["config"]["guard_zero_covariance"] is True
        assert summary["observations_min"] == summary["observations_max"] == FAKE_GRID_OBSERVATIONS
        assert summary["observations_median"] == FAKE_GRID_OBSERVATIONS

    def test_spin_frame_is_guarded(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, spin_at = 20, 10
        vslam.set_default_pose_source(walk(vslam, spins={spin_at: 90.0}))   # 90 deg in 33 ms = 2700 deg/s
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n))
        assert summary["frames_guarded"] == summary["guard_spin"] == 1
        assert_no_other_reasons(summary, "spin")
        assert summary["tum_rows"] == n - 1 and summary["guard_gaps"] == [(spin_at, 1)]
        assert steps(summary["rows"]).max() <= STEP_M + STEP_TOL
        assert all(row[4:] == pytest.approx(IDENTITY_Q, abs=1e-9) for row in summary["rows"])

    def test_starved_frames_are_guarded_and_reanchored(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 30
        vslam.set_default_pose_source(walk(vslam))
        vslam.set_default_observations_source(lambda ts: 5 if 10 <= frame_index(ts) <= 14 else None)
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), min_observations=20)
        assert summary["frames_guarded"] == summary["guard_starved"] == 5
        assert_no_other_reasons(summary, "starved")
        assert summary["tum_rows"] == n - 5 and summary["guard_gaps"] == [(10, 5)]
        assert summary["guard_resets"] == 0 and summary["resets"] == 0
        stamps = row_timestamps(summary)
        assert all(T0 + i * FRAME_PERIOD_NS not in stamps for i in range(10, 15))
        assert steps(summary["rows"]).max() <= STEP_M + STEP_TOL
        P = positions(summary["rows"])
        assert P[10] == pytest.approx(P[9], abs=1e-9)                       # frame 15 continues from frame 9
        assert P[-1] == pytest.approx([0.0, 0.0, STEP_M * (n - 1) - 6 * STEP_M], abs=1e-6)
        assert summary["observations_min"] == 5 and summary["observations_max"] == FAKE_GRID_OBSERVATIONS
        assert summary["observations_median"] == FAKE_GRID_OBSERVATIONS
        assert summary["config"]["min_observations"] == 20

    def test_zero_covariance_is_degenerate_and_can_be_switched_off(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 20
        vslam.set_default_pose_source(walk(vslam))
        vslam.set_default_covariance_source(lambda ts: [0.0] * 36 if 5 <= frame_index(ts) <= 7 else None)
        vslam.set_default_observations_source(lambda ts: 3 if frame_index(ts) == 5 else None)
        stereo_rig = build_rig(vslam, calib, "stereo")
        summary = run(vslam, stereo_rig, "stereo", frames(n), min_observations=20)
        # precedence: starved before degenerate (frame 5 has both), then the two zero-covariance frames
        assert summary["guard_starved"] == 1 and summary["guard_degenerate"] == 2
        assert summary["frames_guarded"] == 3 and summary["tum_rows"] == n - 3
        assert_no_other_reasons(summary, "starved", "degenerate")
        assert summary["guard_gaps"] == [(5, 3)]
        assert steps(summary["rows"]).max() <= STEP_M + STEP_TOL
        # the placeholder magnitude (variance 1.0 per axis) is NOT degenerate
        vslam.set_default_covariance_source(lambda ts: [1.0 if i % 7 == 0 else 0.0 for i in range(36)])
        vslam.set_default_observations_source(None)
        summary = run(vslam, stereo_rig, "stereo", frames(n))
        assert summary["frames_guarded"] == 0 and summary["tum_rows"] == n
        # rule switched off: the exact zeros pass
        vslam.set_default_covariance_source(lambda ts: [0.0] * 36)
        summary = run(vslam, stereo_rig, "stereo", frames(n), guard_zero_covariance=False)
        assert summary["frames_guarded"] == 0 and summary["tum_rows"] == n
        assert summary["config"]["guard_zero_covariance"] is False

    def test_engine_reset_after_consecutive_guarded_frames(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 60
        vslam.set_default_pose_source(walk(vslam, relative=True))            # a fresh engine restarts at 0
        vslam.set_default_observations_source(lambda ts: 5 if 10 <= frame_index(ts) < 50 else None)
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n),
                      min_observations=20, guard_reset_after=30)
        assert vslam.call_counts["Odometry.__init__"] == 2                   # one tracker, then the re-created one
        assert summary["guard_resets"] == 1 and summary["resets"] == 1
        assert summary["segments"] == [0, 40]                                # frame 40 was the first handed to the new engine
        assert summary["frames_guarded"] == summary["guard_starved"] == 40
        assert_no_other_reasons(summary, "starved")                          # the restarted raw pose is not a "jump"
        assert summary["tum_rows"] == 20 and summary["guard_gaps"] == [(10, 40)]
        assert summary["frames_failed"] == 0
        assert steps(summary["rows"]).max() <= STEP_M + STEP_TOL
        P = positions(summary["rows"])
        assert P[10] == pytest.approx(P[9], abs=1e-9)                        # chained onto the last good pose
        assert P[-1] == pytest.approx([0.0, 0.0, STEP_M * 18], abs=1e-6)
        assert summary["config"]["guard_reset_after"] == 30

    def test_engine_reset_in_vio_mode_keeps_feeding_the_imu(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 60
        imu = imu_samples(420)                                               # covers the 2 s of frames plus a tail
        last_frame_ts = T0 + (n - 1) * FRAME_PERIOD_NS
        expected_imu = sum(1 for s in imu if s[0] <= last_frame_ts)
        vslam.set_default_pose_source(walk(vslam, relative=True))
        vslam.set_default_observations_source(lambda ts: 5 if 10 <= frame_index(ts) < 50 else None)
        events: List[tracking.PoseEvent] = []                                # keep the trackers alive (event.tracker)
        summary = tracking.run_tracking(vslam, build_rig(vslam, calib, "vio"), "vio", frames(n), iter(imu),
                                        config(min_observations=20, guard_reset_after=30), on_pose=events.append)
        assert summary["guard_resets"] == 1 and summary["resets"] == 1 and summary["tum_rows"] == 20
        assert summary["imu_registered"] == expected_imu and summary["frames_without_imu"] == 0
        assert summary["imu_dropped_old"] == summary["imu_out_of_order"] == 0
        assert len(events) == 20 and events[9].tracker is not events[10].tracker
        assert events[9].segment == 0 and events[10].segment == 1
        new_odom = events[-1].tracker.odom
        reset_ts = T0 + 39 * FRAME_PERIOD_NS                                 # the 30th guarded frame triggered the reset
        assert new_odom.imu_log and all(m.timestamp_ns > reset_ts for m in new_odom.imu_log)
        assert new_odom.imu_log[-1].timestamp_ns <= last_frame_ts
        assert summary["gravity"] == pytest.approx([0.0, 9.81, 0.0])         # the new engine got enough IMU samples
        assert steps(summary["rows"]).max() <= STEP_M + STEP_TOL

    def test_no_guard_reproduces_the_raw_trajectory(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, jump_at = 30, 10
        vslam.set_default_pose_source(walk(vslam, jumps={jump_at: 3.0}))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), guard=False)
        assert summary["tum_rows"] == n and summary["frames_guarded"] == 0 and summary["guard_gaps"] == []
        assert_no_other_reasons(summary)
        assert vslam.call_counts["Odometry.get_last_observations"] == 0      # no extra engine calls
        assert summary["observations_min"] is None and summary["observations_median"] is None
        assert summary["observations_max"] is None
        assert T0 + jump_at * FRAME_PERIOD_NS in row_timestamps(summary)
        assert steps(summary["rows"]).max() > 2.9                            # the jump is in the TUM rows
        assert summary["config"]["guard"] is False

    def test_config_validation_and_to_dict(self) -> None:
        cfg = config()
        for key in ("guard", "min_observations", "guard_zero_covariance", "max_speed_mps", "max_rot_dps",
                    "guard_reset_after"):
            assert key in cfg.to_dict(), key
        assert cfg.min_observations == tracking.DEFAULT_MIN_OBSERVATIONS == 0
        assert cfg.max_speed_mps == tracking.DEFAULT_MAX_SPEED_MPS == 3.0
        assert cfg.max_rot_dps == tracking.DEFAULT_MAX_ROT_DPS == 720.0
        assert cfg.guard_reset_after == tracking.DEFAULT_GUARD_RESET_AFTER == 30
        assert config(max_speed_mps=float("inf"), max_rot_dps="1e9").max_rot_dps == 1e9
        assert config(min_observations="7", guard_reset_after=0.0).min_observations == 7
        for bad in ({"min_observations": -1}, {"guard_reset_after": -5}, {"max_speed_mps": -1.0},
                    {"max_rot_dps": float("nan")}):
            with pytest.raises(ValueError):
                config(**bad)


# --------------------------------------------------------------------------------------
# CLI smoke (subprocess, fake cuvslam, tiny EuRoC dataset)
# --------------------------------------------------------------------------------------

def run_cli(*args: Any) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.pop(cuvslam_oak.FAKE_ENV_VAR, None)
    env.setdefault("PYTHONIOENCODING", "utf-8")
    return subprocess.run([PYTHON, str(TRACK_SCRIPT), *(str(a) for a in args)], cwd=str(REPO_ROOT),
                          capture_output=True, text=True, encoding="utf-8", errors="replace",
                          env=env, timeout=SUBPROCESS_TIMEOUT_S, check=False)


def make_rig_info(calib: Dict[str, Any]) -> dataset.RigInfo:
    """``RigInfo`` of an undistorted 640x400 recording, as ``07_record_dataset.py`` builds it."""
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


def write_tiny_dataset(root: Path, calib: Dict[str, Any], n_frames: int = 4) -> List[int]:
    rng = np.random.default_rng(7)
    frame_ts = [T0 + i * FRAME_PERIOD_NS for i in range(n_frames)]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", rig.CalibrationWarning)
        rig_info = make_rig_info(calib)
    with dataset.EurocWriter(root, rig_info, cam_rate_hz=FPS, imu_rate_hz=IMU_RATE_HZ, calibration=calib,
                             png_compression=1) as writer:
        next_imu = T0 - IMU_PERIOD_NS
        for ts in frame_ts:
            while next_imu <= ts:
                assert writer.add_imu(next_imu, GYRO, ACCEL)
                next_imu += IMU_PERIOD_NS
            left = rng.integers(0, 256, size=SHAPE, dtype=np.uint8)
            right = rng.integers(0, 256, size=SHAPE, dtype=np.uint8)
            assert writer.add_frame(ts, left, right, right_ts_ns=ts + 12_000)
        for _ in range(3):
            assert writer.add_imu(next_imu, GYRO, ACCEL)
            next_imu += IMU_PERIOD_NS
    return frame_ts


class TestCli:
    FLAGS = ("--no-guard", "--min-observations", "--no-zero-covariance-guard", "--max-speed", "--max-rot-rate",
             "--guard-reset-after")

    def test_help_lists_the_guard_flags(self) -> None:
        proc = run_cli("--help")
        assert proc.returncode == 0, proc.stderr
        for flag in self.FLAGS:
            assert flag in proc.stdout, flag
        assert "racing drone" in proc.stdout
        assert proc.stderr.strip() == ""

    def test_fake_run_with_guard_flags(self, calib: Dict[str, Any], tmp_path: Path) -> None:
        root = tmp_path / "euroc_guard"
        frame_ts = write_tiny_dataset(root, calib)
        tum_path = tmp_path / "traj.tum"
        proc = run_cli("--fake-cuvslam", root, "--mode", "stereo", "--min-observations", "0", "--max-speed", "1e9",
                       "--out", tum_path, "--summary")
        assert proc.returncode == 0, proc.stderr
        assert "Traceback" not in proc.stderr
        assert "guard:" in proc.stdout and "observations min/median/max" in proc.stdout, proc.stdout
        rows = tracking.read_tum(tum_path)
        assert [row[0] for row in rows] == frame_ts
        doc = json.loads((tmp_path / "traj_summary.json").read_text(encoding="utf-8"))
        trk = doc["tracking"]
        assert trk["frames_guarded"] == 0 and trk["guard_resets"] == 0 and trk["guard_gaps"] == []
        assert trk["observations_min"] == trk["observations_max"] == FAKE_GRID_OBSERVATIONS
        assert trk["config"]["guard"] is True and trk["config"]["max_speed_mps"] == 1e9
        assert trk["config"]["min_observations"] == 0 and trk["config"]["guard_zero_covariance"] is True
        assert doc["args"]["guard"] is True and doc["args"]["guard_zero_covariance"] is True
        # --no-guard / --no-zero-covariance-guard reach the config; a negative limit is refused before any output
        proc = run_cli("--fake-cuvslam", root, "--mode", "stereo", "--no-guard", "--no-zero-covariance-guard",
                       "--out", tmp_path / "noguard.tum", "--summary")
        assert proc.returncode == 0, proc.stderr
        assert "guard:" not in proc.stdout
        doc = json.loads((tmp_path / "noguard_summary.json").read_text(encoding="utf-8"))
        assert doc["tracking"]["config"]["guard"] is False
        assert doc["tracking"]["config"]["guard_zero_covariance"] is False
        assert doc["tracking"]["observations_min"] is None
        proc = run_cli("--fake-cuvslam", root, "--min-observations", "-1", "--out", tmp_path / "bad.tum")
        assert proc.returncode == 2
        lines = [line for line in proc.stderr.splitlines() if line.strip()]
        assert len(lines) == 1 and "--min-observations" in lines[0]
        assert proc.stdout.strip() == ""
