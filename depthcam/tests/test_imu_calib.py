"""cuvslam_oak.imu_calib: six-pose fit, correction file round trip, and the tracking loop applying it.

No camera, no real cuvslam: synthetic IMU streams and the fake engine.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from cuvslam_oak import fake_cuvslam, imu_calib
from cuvslam_oak import tracking as oaktrack

REPO = Path(__file__).resolve().parent.parent
G = imu_calib.STANDARD_GRAVITY
TRUE_BIAS = np.array([0.05, -0.69, -0.12])
TRUE_SCALE = np.array([1.006, 1.006, 1.002])
TRUE_GYRO_BIAS = np.array([-0.0004, 0.0001, -0.0002])

#: gravity directions of the six poses, deliberately a few degrees off the axes (nobody levels a camera)
POSES = [(0.02, -0.03, 1.0), (-0.01, 0.04, -1.0), (1.0, 0.03, -0.05), (-1.0, -0.02, 0.03), (0.01, -1.0, 0.02),
         (-0.03, 1.0, 0.05)]


def six_pose_stream(poses=POSES, hold_s=6.0, move_s=2.0, rate=200.0, noise=0.02, seed=1):
    """``(t, gyro, accel)`` of a camera resting on each pose in turn, with a rough move in between."""
    rng = np.random.default_rng(seed)
    gyro_rows, accel_rows = [], []
    for direction in poses:
        d = np.array(direction, dtype=np.float64)
        d /= np.linalg.norm(d)
        n = int(hold_s * rate)
        true_accel = np.tile(d * G, (n, 1))
        accel_rows.append(true_accel / TRUE_SCALE + TRUE_BIAS + rng.normal(0, noise, (n, 3)))
        gyro_rows.append(TRUE_GYRO_BIAS + rng.normal(0, 0.002, (n, 3)))
        m = int(move_s * rate)
        accel_rows.append(rng.normal(0, 3.0, (m, 3)) + d * G)
        gyro_rows.append(rng.normal(0, 1.5, (m, 3)))
    gyro = np.vstack(gyro_rows)
    accel = np.vstack(accel_rows)
    t = np.arange(len(gyro)) / rate
    return t, gyro, accel


def test_fit_recovers_bias_and_scale():
    t, gyro, accel = six_pose_stream()
    correction, fit, segments = imu_calib.fit_six_pose(t, gyro, accel)
    assert len(segments) == 6
    assert np.allclose(correction.accel_bias, TRUE_BIAS, atol=0.01)
    assert np.allclose(correction.accel_scale, TRUE_SCALE, atol=0.002)
    assert np.allclose(correction.gyro_bias, TRUE_GYRO_BIAS, atol=2e-4)
    assert fit.residual_std < 0.01
    assert fit.raw_norm_max - fit.raw_norm_min > 1.0  # the raw magnitudes really disagreed
    _g, fixed = correction.apply(TRUE_GYRO_BIAS, np.array([0.0, G, 0.0]) / TRUE_SCALE + TRUE_BIAS)
    assert abs(float(np.linalg.norm(fixed)) - G) < 0.01


def test_fit_refuses_incomplete_data():
    t, gyro, accel = six_pose_stream(poses=POSES[:4])
    with pytest.raises(imu_calib.ImuCalibError, match="need 6"):
        imu_calib.fit_six_pose(t, gyro, accel)
    # six still stretches, but y never points down: bias and scale of y cannot be separated
    repeated = POSES[:4] + [POSES[5], (0.1, 1.0, -0.1)]
    t, gyro, accel = six_pose_stream(poses=repeated)
    with pytest.raises(imu_calib.ImuCalibError, match="-y"):
        imu_calib.fit_six_pose(t, gyro, accel)
    with pytest.raises(imu_calib.ImuCalibError):
        imu_calib.fit_accel(np.zeros((3, 3)))


def test_static_segments_ignore_motion():
    t, gyro, accel = six_pose_stream(hold_s=5.0, move_s=3.0)
    segments = imu_calib.find_static_segments(t, gyro, accel)
    assert len(segments) == 6
    for seg in segments:
        assert 3.0 <= seg.duration_s <= 5.0
        assert seg.gyro_norm_mean < 0.05


def test_correction_round_trip_and_validation(tmp_path):
    correction = imu_calib.ImuCorrection(accel_bias=(0.05, -0.69, -0.12), accel_scale=(1.006, 1.006, 1.002),
                                         gyro_bias=(0.0, 0.001, 0.0), device_id="ABC123", source="euroc_x", poses=6,
                                         residual_std=0.001)
    path = correction.save(imu_calib.default_path("ABC123", tmp_path))
    assert path == tmp_path / "datasets" / "calibration" / "imu_ABC123.json"
    loaded = imu_calib.load_for_device("ABC123", tmp_path)
    assert loaded == correction and loaded.path == str(path)
    assert imu_calib.load_for_device("OTHER", tmp_path) is None
    assert imu_calib.load_for_device(None, tmp_path) is None
    assert not loaded.is_identity and imu_calib.ImuCorrection().is_identity
    gyro, accel = loaded.apply((0.0, 0.001, 0.0), (0.05, -0.69, 9.0))
    assert gyro == (0.0, 0.0, 0.0)
    assert accel == pytest.approx((0.0, 0.0, (9.0 + 0.12) * 1.002))
    g2, a2 = loaded.apply_arrays(np.array([[0.0, 0.001, 0.0]]), np.array([[0.05, -0.69, 9.0]]))
    assert np.allclose(g2, 0.0) and np.allclose(a2[0], accel)

    with pytest.raises(imu_calib.ImuCalibError):
        imu_calib.ImuCorrection(accel_scale=(1.0, 0.0, 1.0))
    with pytest.raises(imu_calib.ImuCalibError):
        imu_calib.ImuCorrection(accel_bias=(0.0, float("nan"), 0.0))
    bad = tmp_path / "bad.json"
    bad.write_text("{\"hello\": 1}", encoding="utf-8")
    with pytest.raises(imu_calib.ImuCalibError):
        imu_calib.ImuCorrection.load(bad)
    wrong = imu_calib.default_path("OTHER", tmp_path)
    correction.save(wrong)  # a file named for one device that says it belongs to another
    with pytest.raises(imu_calib.ImuCalibError, match="not OTHER"):
        imu_calib.load_for_device("OTHER", tmp_path)


def test_stored_correction_of_this_unit_is_sane():
    path = imu_calib.default_path("OAKDS2EXAMPLEID000")
    if not path.is_file():
        pytest.skip("no stored correction in this checkout")
    correction = imu_calib.ImuCorrection.load(path)
    assert correction.device_id == "OAKDS2EXAMPLEID000" and correction.poses == 6
    assert max(abs(b) for b in correction.accel_bias) < 1.5
    assert all(abs(s - 1.0) < 0.05 for s in correction.accel_scale)
    assert correction.residual_std is not None and correction.residual_std < 0.05


def _frames_and_imu(n_frames=20, imu_per_frame=6):
    frame_dt = 33_333_333
    frames = [(1_000_000_000 + i * frame_dt, [np.full((40, 64), 80, np.uint8), np.full((40, 64), 80, np.uint8)])
              for i in range(n_frames)]
    imu = []
    for i in range(n_frames * imu_per_frame):
        ts = 1_000_000_000 - frame_dt + i * (frame_dt // imu_per_frame)
        imu.append({"ts_ns": ts, "gyro": (0.01, 0.02, 0.03), "accel": (0.05, -0.69 + 9.0, -0.12)})
    return frames, imu


def _rig(vslam):
    cams = []
    for x in (0.0, 0.075):
        cam = vslam.Camera()
        cam.size = (64, 40)
        cam.focal = (40.0, 40.0)
        cam.principal = (32.0, 20.0)
        cam.rig_from_camera = vslam.Pose(rotation=[0, 0, 0, 1], translation=[x, 0, 0])
        cams.append(cam)
    imu = vslam.ImuCalibration()
    imu.rig_from_imu = vslam.Pose(rotation=[0, 0, 0, 1], translation=[0, 0, 0])
    imu.frequency = 200
    return vslam.Rig(cameras=cams, imus=[imu])


@pytest.mark.parametrize("use_correction", [False, True])
def test_tracking_loop_applies_the_correction(use_correction, monkeypatch):
    registered = []
    original = fake_cuvslam.Tracker.register_imu_measurement

    def spy(self, sensor_index, measurement):
        registered.append((list(measurement.angular_velocities), list(measurement.linear_accelerations)))
        return original(self, sensor_index, measurement)

    monkeypatch.setattr(fake_cuvslam.Tracker, "register_imu_measurement", spy)
    correction = imu_calib.ImuCorrection(accel_bias=(0.05, -0.69, -0.12), accel_scale=(1.0, 1.01, 1.0),
                                         gyro_bias=(0.01, 0.0, 0.0)) if use_correction else None
    frames, imu = _frames_and_imu()
    cfg = oaktrack.TrackConfig(warmup_frames=0, imu_correction=correction, log=lambda _m: None)
    summary = oaktrack.run_tracking(fake_cuvslam, _rig(fake_cuvslam), "vio", iter(frames), iter(imu), cfg=cfg)
    assert summary["imu_registered"] == len(registered) > 0
    gyro, accel = registered[0]
    if use_correction:
        assert gyro == pytest.approx([0.0, 0.02, 0.03])
        assert accel == pytest.approx([0.0, 9.0 * 1.01, 0.0])
        assert summary["config"]["imu_correction"]["accel_bias"] == [0.05, -0.69, -0.12]
    else:
        assert gyro == pytest.approx([0.01, 0.02, 0.03])
        assert accel == pytest.approx([0.05, 8.31, -0.12])
        assert summary["config"]["imu_correction"] is None
    json.dumps(summary["config"])  # stays serialisable


def test_config_accepts_a_dict_and_rejects_rubbish():
    cfg = oaktrack.TrackConfig(imu_correction={"accel_bias": [0, 0.1, 0], "accel_scale": [1, 1, 1]})
    assert isinstance(cfg.imu_correction, imu_calib.ImuCorrection)
    with pytest.raises(TypeError):
        oaktrack.TrackConfig(imu_correction="calibration.json")
    with pytest.raises(ValueError):
        oaktrack.TrackConfig(imu_correction={"accel_scale": [1, 1, 1]})


def test_fit_tool_cli(tmp_path):
    t, gyro, accel = six_pose_stream()
    recording = tmp_path / "euroc_sixpose"
    (recording / "mav0" / "imu0").mkdir(parents=True)
    (recording / "dataset.json").write_text(json.dumps({"device": {"device_id": "TESTDEVICE"}}), encoding="utf-8")
    with open(recording / "mav0" / "imu0" / "data.csv", "w", encoding="utf-8", newline="\n") as fh:
        fh.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
        for ts, g, a in zip(t, gyro, accel):
            fh.write("{},{},{},{},{},{},{}\n".format(1_700_000_000_000_000_000 + int(round(ts * 1e9)), *g, *a))
    tool = str(REPO / "datasets" / "fit_imu_calibration.py")
    out = tmp_path / "imu.json"
    run = subprocess.run([sys.executable, tool, str(recording), "--file", str(out)], capture_output=True, text=True,
                         timeout=120)
    assert run.returncode == 0, run.stderr
    assert "Still poses:" in run.stdout and out.is_file()
    stored = imu_calib.ImuCorrection.load(out)
    assert stored.device_id == "TESTDEVICE" and stored.source == "euroc_sixpose"
    assert np.allclose(stored.accel_bias, TRUE_BIAS, atol=0.01)
    check = subprocess.run([sys.executable, tool, "--check", str(recording), "--file", str(out)], capture_output=True,
                           text=True, timeout=120)
    assert check.returncode == 0, check.stderr + check.stdout
    assert "corrected |a|" in check.stdout
    missing = subprocess.run([sys.executable, tool, str(tmp_path / "nope")], capture_output=True, text=True, timeout=120)
    assert missing.returncode == 2
