"""``cuvslam_oak.obstacles`` (depth -> bearing sectors) and ``methods/obstacle-detection/11_obstacle_detect.py``.

Synthetic depth images with known geometry: flat walls, a thin pole, speckles, holes, a floor
seen by a level and by a pitched camera.  The CLI runs in a subprocess on a tiny
``05_record.py``-style video recording written into ``tmp_path``; the live path only ever
enumerates (``--device-id nonexistent``), so no camera is opened.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

from cuvslam_oak import obstacles as ob

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "methods" / "obstacle-detection" / "11_obstacle_detect.py"
PYTHON = sys.executable
SUBPROCESS_TIMEOUT_S = 60.0

W, H = 640, 400
K = np.array([[400.0, 0.0, 320.0], [0.0, 400.0, 200.0], [0.0, 0.0, 1.0]])


def wall(z_m: float) -> np.ndarray:
    return np.full((H, W), int(round(z_m * 1000)), dtype=np.uint16)


def column_at(bearing_deg: float) -> int:
    return int(round(K[0, 2] + K[0, 0] * math.tan(math.radians(bearing_deg))))


def sector(bearing_deg: float) -> int:
    return int(round(bearing_deg / 5.0)) % 72


def floor(height_m: float, pitch_deg: float = 0.0) -> np.ndarray:
    """Depth of a floor *height_m* below a camera pitched nose-down by *pitch_deg* (ray cast)."""
    p = math.radians(pitch_deg)
    # Rows: the camera axes in level coordinates (x right, y down, z forward).
    cam_from_level = np.array([[1.0, 0.0, 0.0],
                               [0.0, math.cos(p), -math.sin(p)],
                               [0.0, math.sin(p), math.cos(p)]])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, dtype=float)], axis=-1)
    down = rays @ cam_from_level[:, 1]          # level y component of each ray (z_cam = 1)
    with np.errstate(divide="ignore"):
        depth = np.where(down > 1e-6, height_m / down, 0.0)
    depth[depth > 20.0] = 0.0
    return np.rint(depth * 1000).astype(np.uint16)


# --------------------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------------------

def test_flat_wall_gives_its_distance_in_every_sector_in_view() -> None:
    scan = ob.detect(wall(2.0), K)
    assert scan.distances_m[0] == pytest.approx(2.0, abs=0.01)
    assert scan.ahead_m == pytest.approx(2.0, abs=0.01)
    # Off-axis sectors report the range to their nearest edge: 2 / cos(27.5 deg) at 30 deg.
    assert scan.distances_m[sector(30)] == pytest.approx(2.0 / math.cos(math.radians(27.5)), abs=0.02)
    assert scan.distances_m[sector(-30)] == pytest.approx(2.0 / math.cos(math.radians(27.5)), abs=0.02)
    # Outside the ~77 deg field of view nothing is known.
    for bearing in (60, 90, 180, -90):
        assert math.isnan(scan.distances_m[sector(bearing)])
    assert scan.counts()["obstacle"] == 17
    assert scan.valid_fraction == pytest.approx(1.0)


def test_wall_beyond_max_range_is_clear_not_unknown() -> None:
    scan = ob.detect(wall(6.0), K)
    assert np.isposinf(scan.distances_m[0])
    assert np.isposinf(scan.ahead_m)
    assert scan.nearest() is None


def test_holes_are_unknown_never_free() -> None:
    scan = ob.detect(np.zeros((H, W), dtype=np.uint16), K)
    assert np.isnan(scan.distances_m).all()
    assert math.isnan(scan.ahead_m)
    assert scan.valid_fraction == 0.0
    assert scan.counts() == {"obstacle": 0, "clear": 0, "unknown": 72}


def test_depth_beyond_the_evidence_range_does_not_count_as_clear() -> None:
    scan = ob.detect(wall(9.0), K)
    assert np.isnan(scan.distances_m).all()


def test_thin_pole_shows_up_in_its_own_sector() -> None:
    depth = wall(4.0)
    u = column_at(10.0)
    depth[:, u - 2:u + 3] = 2000                  # 2.5 cm wide at 2 m
    scan = ob.detect(depth, K)
    assert scan.distances_m[sector(10)] == pytest.approx(2.0 / math.cos(math.radians(10.0)), abs=0.05)
    assert scan.distances_m[sector(5)] == pytest.approx(4.0, abs=0.05)
    assert scan.distances_m[sector(15)] > 3.9
    distance, bearing = scan.nearest()
    assert distance == pytest.approx(2.03, abs=0.05) and bearing == pytest.approx(10.0)


def test_isolated_speckles_are_ignored() -> None:
    depth = wall(4.0)
    rng = np.random.default_rng(0)
    rows = rng.integers(0, H, 40)
    cols = rng.integers(0, W, 40)
    depth[rows, cols] = 800
    scan = ob.detect(depth, K, ob.ObstacleConfig(stride=1))
    assert np.nanmin(scan.distances_m) > 3.9


def test_bearing_sign_right_is_positive() -> None:
    depth = wall(4.0)
    depth[:, column_at(20.0) - 3:column_at(20.0) + 4] = 1500    # right of centre
    depth[:, column_at(-20.0) - 3:column_at(-20.0) + 4] = 2500  # left of centre
    scan = ob.detect(depth, K)
    assert scan.distances_m[4] < 1.7            # +20 deg -> index 4
    assert 2.5 < scan.distances_m[68] < 2.8     # -20 deg -> index 68
    assert scan.bearings_deg()[68] == pytest.approx(-20.0)


def test_corridor_only_sees_what_is_in_its_width() -> None:
    def pole_at(x_m: float) -> np.ndarray:
        depth = wall(4.0)
        u = int(round(K[0, 2] + K[0, 0] * x_m / 2.0))
        depth[:, u - 3:u + 4] = 2000
        return depth

    assert ob.detect(pole_at(0.2), K).ahead_m == pytest.approx(2.0, abs=0.01)
    assert ob.detect(pole_at(0.6), K).ahead_m == pytest.approx(4.0, abs=0.01)
    wide = ob.ObstacleConfig(corridor_half_width_m=0.7)
    assert ob.detect(pole_at(0.6), K, wide).ahead_m == pytest.approx(2.0, abs=0.01)


def test_height_band_drops_the_floor_of_a_level_camera() -> None:
    depth = floor(1.0)
    assert np.isnan(ob.detect(depth, K).distances_m).all()
    low_band = ob.detect(depth, K, ob.ObstacleConfig(band_below_m=1.2))
    assert np.isfinite(low_band.distances_m).any()


def test_levelling_keeps_a_pitched_camera_from_seeing_the_floor_as_a_wall() -> None:
    depth = floor(1.0, pitch_deg=20.0)
    unlevelled = ob.detect(depth, K)
    assert np.isfinite(unlevelled.distances_m).any()          # the floor ~2.7 m out sits at "camera height"
    levelled = ob.detect(depth, K, down=ob.down_from_tilt(pitch_deg=20.0))
    assert not np.isfinite(levelled.distances_m).any()


def test_points_closer_than_min_range_are_discarded() -> None:
    scan = ob.detect(wall(0.2), K)
    assert np.isnan(scan.distances_m).all()


def test_decimated_depth_works_with_matching_intrinsics() -> None:
    half_K = K * np.array([[0.5], [0.5], [1.0]])
    scan = ob.detect(np.full((H // 2, W // 2), 2000, dtype=np.uint16), half_K, ob.ObstacleConfig(stride=1))
    assert scan.distances_m[0] == pytest.approx(2.0, abs=0.01)


# --------------------------------------------------------------------------------------
# Levelling helpers
# --------------------------------------------------------------------------------------

def test_level_rotation_identity_and_orthonormal() -> None:
    assert np.allclose(ob.level_rotation(None), np.eye(3))
    rng = np.random.default_rng(1)
    for _ in range(20):
        down = np.array([0.0, 1.0, 0.0]) + 0.5 * rng.normal(size=3)
        R = ob.level_rotation(down)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0)
        assert np.allclose(R @ (down / np.linalg.norm(down)), [0.0, 1.0, 0.0])


def test_level_rotation_matches_a_pitched_camera() -> None:
    p = math.radians(20.0)
    level_from_cam = np.array([[1.0, 0.0, 0.0],
                               [0.0, math.cos(p), math.sin(p)],
                               [0.0, -math.sin(p), math.cos(p)]])
    assert np.allclose(ob.level_rotation(ob.down_from_tilt(pitch_deg=20.0)), level_from_cam)
    assert ob.tilt_deg(ob.down_from_tilt(pitch_deg=20.0)) == pytest.approx(20.0)
    assert ob.down_from_tilt(roll_deg=30.0)[0] > 0.0            # right side down: gravity towards +x


def test_level_rotation_refuses_a_camera_looking_straight_down() -> None:
    with pytest.raises(ValueError):
        ob.level_rotation([0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        ob.level_rotation([0.0, 0.0, 0.0])


def test_down_from_raw_imu_at_rest() -> None:
    # This unit's EEPROM rotation: IMU x = cam x, y = cam -y, z = cam -z; at rest the raw IMU reads +g on y.
    R_cam_from_imu = np.diag([1.0, -1.0, -1.0])
    assert np.allclose(ob.down_from_accel(R_cam_from_imu @ [0.0, 9.81, 0.0]), [0.0, 1.0, 0.0])


def test_gravity_estimator_low_passes() -> None:
    est = ob.GravityEstimator(time_constant_s=0.3)
    assert est.down is None
    est.update([0.0, -9.81, 0.0], 0.0)
    assert np.allclose(est.down, [0.0, 1.0, 0.0])
    tilted = -9.81 * ob.down_from_tilt(pitch_deg=30.0)
    est.update(tilted, 0.01)                                   # one 10 ms sample barely moves it
    assert ob.tilt_deg(est.down) < 2.0
    for k in range(2, 300):                                    # 3 s = 10 time constants
        est.update(tilted, k * 0.01)
    assert ob.tilt_deg(est.down) == pytest.approx(30.0, abs=0.1)
    est.update([float("nan"), 0.0, 0.0], 3.1)                  # ignored
    assert ob.tilt_deg(est.down) == pytest.approx(30.0, abs=0.1)


# --------------------------------------------------------------------------------------
# MAVLink layout and config
# --------------------------------------------------------------------------------------

def test_obstacle_distance_fields() -> None:
    depth = wall(6.0)
    depth[:, column_at(0.0) - 10:column_at(0.0) + 11] = 1500
    msg = ob.detect(depth, K).to_obstacle_distance(time_usec=123)
    assert msg["time_usec"] == 123
    assert msg["frame"] == ob.MAV_FRAME_BODY_FRD == 12
    assert msg["increment_f"] == 5.0 and msg["angle_offset"] == 0.0
    assert msg["min_distance"] == 30 and msg["max_distance"] == 500
    assert len(msg["distances"]) == 72
    assert msg["distances"][0] == 150                           # obstacle, cm
    assert msg["distances"][sector(20)] == 501                  # clear = max_distance + 1
    assert msg["distances"][sector(90)] == ob.UINT16_MAX        # out of view = unknown


def test_obstacle_distance_pads_coarse_sectors_and_refuses_fine_ones() -> None:
    coarse = ob.detect(wall(2.0), K, ob.ObstacleConfig(sector_deg=10.0)).to_obstacle_distance()
    assert len(coarse["distances"]) == 72 and coarse["distances"][36:] == [ob.UINT16_MAX] * 36
    with pytest.raises(ValueError):
        ob.detect(wall(2.0), K, ob.ObstacleConfig(sector_deg=2.0)).to_obstacle_distance()


@pytest.mark.parametrize("kwargs", [
    {"sector_deg": 7.0},
    {"min_range_m": 6.0},
    {"max_range_m": 9.0},
    {"stride": 0},
    {"min_points": 0},
    {"cluster_m": 0.0},
    {"band_below_m": -0.1},
])
def test_config_rejects_bad_values(kwargs: dict) -> None:
    with pytest.raises(ValueError):
        ob.ObstacleConfig(**kwargs)


def test_detect_rejects_a_non_2d_image() -> None:
    with pytest.raises(ValueError):
        ob.detect(np.zeros((H, W, 3), dtype=np.uint16), K)


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def write_recording(folder: Path, frames: int = 5) -> Path:
    """A minimal ``05_record.py --mode video`` layout: session.json, depth_index.csv, depth PNGs."""
    import cv2

    (folder / "depth").mkdir(parents=True)
    depth = wall(4.0)
    depth[:, column_at(0.0) - 10:column_at(0.0) + 11] = 1500
    lines = ["index,sequence,timestamp_s,device_timestamp_s,width,height,file"]
    for i in range(frames):
        name = "depth/depth_{:06d}.png".format(i)
        assert cv2.imwrite(str(folder / name), depth)
        lines.append("{0},{0},{1:.6f},{2:.6f},{3},{4},{5}".format(i, 100 + i / 30.0, 2 + i / 30.0, W, H, name))
    (folder / "depth_index.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    session = {"script": "05_record.py", "mode": "video",
               "video": {"depth": {"intrinsics": K.tolist(), "size": [W, H]}}}
    (folder / "session.json").write_text(json.dumps(session), encoding="utf-8")
    return folder


def run_script(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([PYTHON, str(SCRIPT), *args], capture_output=True, text=True,
                          timeout=SUBPROCESS_TIMEOUT_S, cwd=str(REPO_ROOT))


def test_cli_replay_headless(tmp_path: Path) -> None:
    rec = write_recording(tmp_path / "video_test")
    result = run_script("--replay", str(rec), "--headless")
    assert result.returncode == 0, result.stderr
    assert "Done: 5 depth frames" in result.stdout
    assert "nearest 1.50 m @ +0" in result.stdout
    assert "ahead 1.50 m" in result.stdout


def test_cli_replay_missing_folder(tmp_path: Path) -> None:
    result = run_script("--replay", str(tmp_path / "nope"), "--headless")
    assert result.returncode == 2
    assert "not a 05_record.py --mode video recording" in result.stderr
    assert "Traceback" not in result.stderr


def test_cli_rejects_bad_settings(tmp_path: Path) -> None:
    rec = write_recording(tmp_path / "video_test", frames=1)
    result = run_script("--replay", str(rec), "--headless", "--max-range", "0.1")
    assert result.returncode == 2
    assert "Bad detection settings" in result.stderr


def test_cli_live_wrong_device_id_fails_fast() -> None:
    result = run_script("--device-id", "nonexistent", "--headless", "--duration", "1")
    assert result.returncode == 2
    assert "Traceback" not in result.stderr
