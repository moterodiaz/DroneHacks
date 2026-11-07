"""Coasting in ``cuvslam_oak.tracking.run_tracking``: position at the last velocity, rotation from the
gyroscope, while the engine gives no usable pose (guard skip, loss).  Fake engine, no camera.
"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pytest

from cuvslam_oak import rig, tracking

from test_guard import (ACCEL, FRAME_PERIOD_NS, IMU_PERIOD_NS, STEP_M, T0, build_rig, calib, clean_fake_sources,  # noqa: F401
                        frames, positions, run, vslam, walk, write_tiny_dataset)

REPO_ROOT = Path(__file__).resolve().parents[1]
TRACK_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py"
DT_S = FRAME_PERIOD_NS / 1e9
SPEED = STEP_M / DT_S                     # 0.3 m/s along +z


def lost_between(vslam: Any, first: int, last: int, **walk_kwargs: Any):
    """Pose source: the walk, but ``None`` (a loss) on frames ``first..last``."""
    base = walk(vslam, **walk_kwargs)

    def source(timestamp_ns: int, origin_ns: int) -> Any:
        i = int(round((int(timestamp_ns) - T0) / FRAME_PERIOD_NS))
        return None if first <= i <= last else base(timestamp_ns, origin_ns)

    return source


def expected_coast(steps: int, decay: float = tracking.DEFAULT_COAST_DECAY_S) -> float:
    """Distance coasted after *steps* frame periods at SPEED with the exponential decay."""
    tau = steps * DT_S
    return SPEED * decay * (1.0 - math.exp(-tau / decay))


def imu_stream(n_frames: int, gyro_rig=(0.0, 0.0, 0.0)) -> List[Any]:
    """Merged stream: 200 Hz IMU tuples with a constant rate, each frame after its samples."""
    items: List[Any] = []
    next_imu = T0 - IMU_PERIOD_NS
    for ts, left, right in frames(n_frames):
        while next_imu <= ts:
            items.append(("imu", next_imu, tuple(gyro_rig), ACCEL))
            next_imu += IMU_PERIOD_NS
        items.append((ts, left, right))
    return items


class TestCoasting:
    def test_guarded_frame_is_coasted_at_the_last_velocity(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, jump_at = 30, 10
        vslam.set_default_pose_source(walk(vslam, jumps={jump_at: 3.0}))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), coast=True, coast_gyro=False)
        assert summary["frames_guarded"] == summary["guard_jump"] == 1
        assert summary["frames_coasted"] == 1 and summary["coast_gaps"] == [(jump_at, 1)]
        assert summary["tum_rows"] == n                                     # the guarded frame got a row
        P = positions(summary["rows"])
        # the coasted row continues the walk (0.3 m/s, one frame, decay barely started)
        assert P[jump_at][2] == pytest.approx(P[jump_at - 1][2] + expected_coast(1), abs=1e-6)
        # the returning pose is anchored on the prediction for its own time, then the raw steps resume
        assert P[jump_at + 1][2] == pytest.approx(P[jump_at - 1][2] + expected_coast(2), abs=1e-6)
        assert P[-1][2] == pytest.approx(STEP_M * (n - 1), abs=0.001)      # under 1 mm short over the gap
        steps = np.linalg.norm(np.diff(P, axis=0), axis=1)
        assert steps.max() < 1.5 * STEP_M
        assert summary["coast_gyro_used"] is False

    def test_loss_is_bridged_and_the_gap_recorded(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 40
        vslam.set_default_pose_source(lost_between(vslam, 10, 14, relative=True))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), coast=True, coast_gyro=False)
        assert summary["frames_failed"] == 5 and summary["frames_coasted"] == 5
        assert summary["coast_gaps"] == [(10, 5)] and summary["resets"] == 1
        assert summary["tum_rows"] == n
        P = positions(summary["rows"])
        for k in range(1, 6):
            assert P[9 + k][2] == pytest.approx(P[9][2] + expected_coast(k), abs=1e-6)
        assert P[15][2] == pytest.approx(P[9][2] + expected_coast(6), abs=1e-6)
        # the decay (time constant 1 s) costs 9 % of the distance over this 0.2 s gap, 5.6 mm, nothing else is lost
        assert P[-1][2] == pytest.approx(STEP_M * (n - 1) - (6 * STEP_M - expected_coast(6)), abs=1e-6)
        assert STEP_M * (n - 1) - P[-1][2] < 0.006
        assert np.linalg.norm(np.diff(P, axis=0), axis=1).max() < 1.5 * STEP_M

    def test_coasting_stops_after_coast_max(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n = 60
        vslam.set_default_pose_source(lost_between(vslam, 10, 49, relative=True))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), coast=True, coast_gyro=False,
                      coast_max_s=0.5)
        bridged = sum(1 for k in range(1, 41) if k * DT_S <= 0.5)
        assert summary["frames_coasted"] == bridged == 15
        assert summary["coast_gaps"] == [(10, 15)]
        assert summary["tum_rows"] == n - 40 + 15
        P = positions(summary["rows"])
        # the returning pose (frame 50, 1.33 s after the last measurement) continues from the last coasted row
        assert P[10 + 15][2] == pytest.approx(P[10 + 14][2], abs=1e-9)

    def test_coasting_off_freezes_across_the_gap(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, jump_at = 30, 10
        vslam.set_default_pose_source(walk(vslam, jumps={jump_at: 3.0}))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(n), coast=False)
        assert summary["frames_coasted"] == 0 and summary["coast_gaps"] == [] and summary["tum_rows"] == n - 1
        P = positions(summary["rows"])
        assert P[jump_at] == pytest.approx(P[jump_at - 1], abs=1e-9)        # zero motion across the gap

    def test_still_camera_does_not_drift_while_coasting(self, vslam: Any, calib: Dict[str, Any]) -> None:
        rng = np.random.default_rng(2)
        jitter = {i: rng.normal(0.0, 0.0005) for i in range(30)}

        def source(timestamp_ns: int, origin_ns: int) -> Any:
            i = int(round((int(timestamp_ns) - T0) / FRAME_PERIOD_NS))
            if 10 <= i <= 19:
                return None
            return vslam.Pose((0.0, 0.0, 0.0, 1.0), (jitter[i], 0.0, 0.0))

        vslam.set_default_pose_source(source)
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(30), coast=True, coast_gyro=False)
        assert summary["frames_coasted"] == 10
        P = positions(summary["rows"])
        assert np.abs(P[10:20]).max() < 0.01                               # velocity fit is not fooled by jitter

    def test_rotation_follows_the_gyro(self, vslam: Any, calib: Dict[str, Any]) -> None:
        n, first, last = 30, 10, 19
        rate = 1.0                                                           # rad/s yaw (about rig y)
        vslam.set_default_pose_source(lost_between(vslam, first, last, relative=True))
        stream = imu_stream(n, gyro_rig=(0.0, rate, 0.0))
        summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", stream, coast=True,
                      coast_rig_from_imu=(0.0, 0.0, 0.0, 1.0))
        assert summary["coast_gyro_used"] is True and summary["frames_coasted"] == 10
        assert summary["imu_registered"] == 0 and summary["imu_source_ignored"] is False   # read, never registered
        rows = summary["rows"]

        def yaw_of(row: Any) -> float:
            R = rig.quaternion_xyzw_to_matrix(np.array(row[4:8]))
            f = R @ np.array([0.0, 0.0, 1.0])
            return math.atan2(f[0], f[2])

        assert yaw_of(rows[first - 1]) == pytest.approx(0.0, abs=1e-9)
        assert yaw_of(rows[last]) == pytest.approx(rate * 10 * DT_S, abs=0.01)         # 0.333 rad after 10 frames
        assert yaw_of(rows[last + 1]) == pytest.approx(rate * 11 * DT_S, abs=0.01)     # the returning pose keeps it
        assert yaw_of(rows[-1]) == pytest.approx(rate * 11 * DT_S, abs=0.01)           # and it stays (engine: no yaw)

    def test_vio_rig_supplies_the_imu_rotation(self, vslam: Any, calib: Dict[str, Any]) -> None:
        vslam.set_default_pose_source(lost_between(vslam, 5, 6, relative=True))
        summary = run(vslam, build_rig(vslam, calib, "vio"), "vio", imu_stream(12), coast=True)
        assert summary["coast_gyro_used"] is True and summary["frames_coasted"] == 2
        assert summary["imu_registered"] > 0

    def test_config_validation(self) -> None:
        cfg = tracking.TrackConfig(coast_rig_from_imu=np.diag([1.0, -1.0, -1.0]))
        assert cfg.coast_rig_from_imu == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-9)
        cfg = tracking.TrackConfig(coast_rig_from_imu=np.eye(4))
        assert cfg.coast_rig_from_imu == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-9)
        for bad in ({"coast_decay_s": -1.0}, {"coast_max_s": float("nan")}, {"coast_rig_from_imu": [1, 2, 3]}):
            with pytest.raises(ValueError):
                tracking.TrackConfig(**bad)
        d = tracking.TrackConfig(coast_rig_from_imu=(0, 0, 0, 1)).to_dict()
        assert d["coast"] is True and d["coast_rig_from_imu"] == [0.0, 0.0, 0.0, 1.0]

    def test_pose_event_flag(self, vslam: Any, calib: Dict[str, Any]) -> None:
        events: List[tracking.PoseEvent] = []
        vslam.set_default_pose_source(walk(vslam, jumps={10: 3.0}))
        tracking.run_tracking(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(20), None,
                              tracking.TrackConfig(verbose=False, warm_up_gpu=False, warmup_frames=0,
                                                   coast_gyro=False), on_pose=events.append)
        assert [e.coasted for e in events] == [i == 10 for i in range(20)]
        assert events[10].tracker is None and events[10].pose is None and events[10].track_ms == 0.0
        assert events[9].tracker is not None


def test_cli_prints_the_coast_line(tmp_path: Path, calib: Dict[str, Any]) -> None:
    root = tmp_path / "euroc_tiny"
    write_tiny_dataset(root, calib, n_frames=6)
    proc = subprocess.run([sys.executable, str(TRACK_SCRIPT), str(root), "--fake-cuvslam", "--mode", "stereo",
                           "--out", str(tmp_path / "t.tum"), "--summary"], cwd=str(REPO_ROOT), capture_output=True,
                          text=True, timeout=90)
    assert proc.returncode == 0, proc.stderr
    assert "coast: 0 estimated row(s) in 0 gap(s), heading from the gyroscope" in proc.stdout
    proc = subprocess.run([sys.executable, str(TRACK_SCRIPT), str(root), "--fake-cuvslam", "--no-coast",
                           "--out", str(tmp_path / "t2.tum")], cwd=str(REPO_ROOT), capture_output=True, text=True,
                          timeout=90)
    assert proc.returncode == 0, proc.stderr
    assert "coast:" not in proc.stdout
