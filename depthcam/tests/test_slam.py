"""cuVSLAM's SLAM layer in ``run_tracking`` (``TrackConfig.slam``): rows, counters, map save, the
script's second TUM file, the bridge message.  Fake engine (its SLAM pose follows the odometry).
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest

from cuvslam_oak import bridge, tracking

from test_guard import (T0, build_rig, calib, clean_fake_sources, frames, positions, run, vslam, walk,  # noqa: F401
                        write_tiny_dataset)
from test_bridge import Server, collect, hello, stream

REPO_ROOT = Path(__file__).resolve().parents[1]
TRACK_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py"


def test_slam_rows_follow_the_odometry_and_survive_a_guard_reset(vslam: Any, calib: Dict[str, Any], tmp_path: Path) -> None:
    vslam.set_default_pose_source(walk(vslam, relative=True))
    vslam.set_default_covariance_source(lambda ts: [0.0] * 36 if 10 <= round((ts - T0) / 33_333_333) < 45 else None)
    summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(60), slam=True,
                  slam_save_map=str(tmp_path / "map"), coast=False)
    slam = summary["slam"]
    assert slam["enabled"] is True and slam["sync"] is True and slam["max_map_size"] == 0
    assert summary["guard_resets"] == 1 and slam["map_resets"] == 1              # the map died with the engine
    assert slam["rows"] == summary["frames_tracked"] == len(summary["slam_rows"])
    # the fake's SLAM pose equals its odometry pose: anchored the same way, the rows coincide
    assert np.allclose(positions(summary["slam_rows"]), positions(summary["rows"]), atol=1e-9)
    # the re-optimised trajectory covers both engines (snapshot before the reset + the final one), accepted frames only
    assert slam["optimized_rows"] == len(summary["slam_optimized_rows"]) == summary["frames_tracked"]
    assert [r[0] for r in summary["slam_optimized_rows"]] == [r[0] for r in summary["rows"]]
    assert np.allclose(positions(summary["slam_optimized_rows"]), positions(summary["rows"]), atol=1e-9)
    assert slam["loop_closures"] == 0 and slam["pgo_events"] == 0
    assert slam["map_saved"] is True and (tmp_path / "map").is_dir()
    assert slam["endpoint_offset_m"] == pytest.approx(summary["endpoint_offset_m"], abs=1e-9)
    assert dict(vslam.call_counts)["Slam.track"] == summary["frames_attempted"]   # SLAM sees every tracked frame


def test_slam_off_by_default_and_config_validation(vslam: Any, calib: Dict[str, Any]) -> None:
    summary = run(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(5))
    assert summary["slam"] is None and summary["slam_rows"] == [] and summary["slam_optimized_rows"] == []
    assert dict(vslam.call_counts).get("Slam.track", 0) == 0
    cfg = tracking.TrackConfig(slam=True, realtime=True)
    assert tracking.slam_config(vslam, cfg).sync_mode is False           # live: loop closure in its own thread
    cfg = tracking.TrackConfig(slam=True, slam_sync=True, realtime=True, slam_max_map_size=500, slam_planar=True)
    scfg = tracking.slam_config(vslam, cfg)
    assert scfg.sync_mode is True and scfg.max_map_size == 500 and scfg.planar_constraints is True
    assert scfg.enable_reading_internals is True
    assert tracking.slam_config(vslam, tracking.TrackConfig()) is None
    with pytest.raises(ValueError):
        tracking.TrackConfig(slam_max_map_size=-1)
    assert tracking.TrackConfig(slam=True).to_dict()["slam"] is True


def test_pose_events_carry_the_slam_pose(vslam: Any, calib: Dict[str, Any]) -> None:
    events: List[tracking.PoseEvent] = []
    vslam.set_default_pose_source(walk(vslam))
    tracking.run_tracking(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(6), None,
                          tracking.TrackConfig(verbose=False, warm_up_gpu=False, warmup_frames=0, slam=True),
                          on_pose=events.append)
    assert all(e.slam_translation is not None and e.slam_rotation_xyzw is not None for e in events)
    assert np.allclose(events[-1].slam_translation, events[-1].translation, atol=1e-9)
    events.clear()
    tracking.run_tracking(vslam, build_rig(vslam, calib, "stereo"), "stereo", frames(3), None,
                          tracking.TrackConfig(verbose=False, warm_up_gpu=False, warmup_frames=0),
                          on_pose=events.append)
    assert all(e.slam_translation is None for e in events)


def test_bridge_sends_the_slam_pose(vslam: Any, calib: Dict[str, Any]) -> None:
    server = Server(vslam)
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, "stereo", options={"slam": True})) as client:
        ready = client.connect()
        stream(client, 8)
        events = collect(client)
    server.join()
    assert ready["config"]["slam"] is True
    poses = [value for mtype, value in events if mtype == bridge.MSG_POSE]
    assert len(poses) == 8 and all(len(p["s"]) == 3 and len(p["sq"]) == 4 for p in poses)
    assert events[-1][1]["slam"]["rows"] == 8 and "slam_rows" not in events[-1][1]
    stats = [value for mtype, value in events if mtype == bridge.MSG_STATS]
    assert all("loop_closures" in value for value in stats)


def test_script_writes_the_slam_trajectory(tmp_path: Path, calib: Dict[str, Any]) -> None:
    root = tmp_path / "euroc_tiny"
    write_tiny_dataset(root, calib, n_frames=6)
    out = tmp_path / "trajectory.tum"
    proc = subprocess.run([sys.executable, str(TRACK_SCRIPT), str(root), "--fake-cuvslam", "--mode", "stereo",
                           "--slam", "--slam-save", str(tmp_path / "map"), "--out", str(out), "--summary"],
                          cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=90)
    assert proc.returncode == 0, proc.stderr
    assert "slam: 6 pose(s), 0 loop closure(s)" in proc.stdout and "map saved" in proc.stdout
    slam_tum = tmp_path / "trajectory_slam.tum"
    assert slam_tum.is_file() and len(tracking.read_tum(slam_tum)) == 6
    assert "SLAM trajectory (re-optimised at the end)" in slam_tum.read_text(encoding="ascii")
    summary = __import__("json").loads((tmp_path / "trajectory_summary.json").read_text(encoding="utf-8"))
    assert summary["outputs"]["slam_tum"] == str(slam_tum)
    assert summary["tracking"]["slam"]["map_saved"] is True and "slam_rows" not in summary["tracking"]
    proc = subprocess.run([sys.executable, str(TRACK_SCRIPT), str(root), "--fake-cuvslam", "--out",
                           str(tmp_path / "plain.tum")], cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=90)
    assert proc.returncode == 0 and "slam:" not in proc.stdout and not (tmp_path / "plain_slam.tum").exists()
