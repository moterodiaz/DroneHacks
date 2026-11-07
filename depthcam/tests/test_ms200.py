"""``course.ms200`` (MS200K serial reader) and ``lidar_localize.Tracker``, from bytes to pose.

No sensor needed: ``lidar_sim`` scans are encoded into the sensor's own 47-byte frames
(``ms200.synthetic_stream``), with a wrapping ms clock, late and jittery arrival, and several frames
per read.  The byte layout, CRC table and command frames are checked against the vendor SDK
(MS200_Ros-V1.3.4).
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from course import ms200
from course.course_map import CourseMap
from course.lidar_localize import DistanceField, MatchConfig, Tracker
from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP = REPO_ROOT / "course" / "maps" / "course_v3_map.json"
HOST0 = 1000.0  # the host clock is far from the sensor's


@pytest.fixture(scope="module")
def scene() -> Scene:
    return Scene.from_map(CourseMap.load(MAP))


@pytest.fixture(scope="module")
def field() -> DistanceField:
    return DistanceField(CourseMap.load(MAP), 0.9)


def _walk() -> Trajectory:
    """Still on the launch mark for 1.5 s, then 1 m/s north up the first corridor."""
    return Trajectory(np.array([0.0, 1.5, 3.0]), np.array([[-2.515, 0.914, 0.8], [-2.515, 0.914, 0.8], [-2.515, 2.414, 0.8]]),
                      np.array([[0.0, 0.0, math.pi / 2]] * 3))


def _beams(scans):
    return (np.concatenate([s.t for s in scans]) + HOST0, np.concatenate([s.angle for s in scans]),
            np.concatenate([s.range for s in scans]))


def _truth(rev: ms200.Revolution, t_all: np.ndarray, a_all: np.ndarray) -> np.ndarray:
    """Index of the simulated beam behind every decoded point: the same angle on the 0.8 deg beam
    grid, nearest in time (the same angle comes round only every 0.1 s, so a wrong angle shows up as
    a wrong time)."""
    grid = lambda ang: np.round(np.mod(np.degrees(ang), 360.0) / 0.8).astype(int) % 450  # noqa: E731
    k_all, k = grid(a_all), grid(rev.angle)
    idx = np.empty(len(k), dtype=int)
    for i in range(len(k)):
        cand = np.flatnonzero(k_all == k[i])
        idx[i] = cand[np.argmin(np.abs(t_all[cand] - rev.t[i]))]
    return idx


def test_crc_table_and_frame_layout_match_the_sdk() -> None:
    assert ms200.FRAME_LEN == 47
    assert list(ms200._CRC[:8]) == [0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE]
    assert ms200._CRC[255] == 0xA8
    raw = ms200.encode_frame(3600, 359.5, 8.3, np.arange(12) * 100, np.full(12, 200), 29999)
    assert raw[:2] == b"\x54\x2c" and raw[2:4] == (3600).to_bytes(2, "little") and raw[-3:-1] == (29999).to_bytes(2, "little")
    (frame,) = ms200.FrameParser().feed(raw, 5.0)
    assert frame.speed_dps == 3600 and frame.stamp_ms == 29999 and frame.host_t == 5.0
    assert frame.step_deg == pytest.approx(0.8)
    assert frame.angles_deg()[[0, 1, 11]] == pytest.approx([359.5, 0.3, 8.3])
    assert list(frame.distance_mm) == list(np.arange(12) * 100)


def test_parser_resyncs_after_junk_a_bad_crc_and_a_fake_header() -> None:
    good1 = ms200.encode_frame(3600, 10.0, 18.8, np.full(12, 1500), np.full(12, 200), 100)
    bad = bytearray(ms200.encode_frame(3600, 19.6, 28.4, np.full(12, 1500), np.full(12, 200), 103))
    bad[10] ^= 0xFF
    tricky = ms200.encode_frame(3600, 29.2, 38.0, np.full(12, 0x2C54), np.full(12, 0x54), 105)  # 54 2C inside
    stream = b"\xaa\x55\x01\x03SN!\x54" + good1 + bytes(bad) + tricky
    parser, frames = ms200.FrameParser(), []
    rng = np.random.default_rng(0)
    i = 0
    while i < len(stream):
        n = int(rng.integers(1, 9))
        frames += parser.feed(stream[i:i + n], float(i))
        i += n
    assert [f.stamp_ms for f in frames] == [100, 105]
    assert parser.crc_errors >= 1
    assert list(frames[1].distance_mm) == [0x2C54] * 12


def test_command_frames_match_the_sdk() -> None:
    assert ms200.command(0xA2, b"\x81") == bytes.fromhex("a5f5a2c10181b331f2")      # start
    assert ms200.command(0xA2, b"\x80") == bytes.fromhex("a5f5a2c10180b231f2")      # stop
    assert ms200.command(0xA1, bytes([10, 0])) == bytes.fromhex("a5f5a1c1020a003831f2")  # 10 Hz


@pytest.mark.parametrize("wrap, clock_start", [(30000, 29000), (65536, 65000)])
def test_stream_round_trip(scene: Scene, wrap: int, clock_start: int) -> None:
    traj = Trajectory(np.array([0.0, 3.0]), np.array([[-2.5, -0.5, 0.8], [-2.5, 2.5, 0.8]]),
                      np.array([[0.05, 0.1, math.pi / 2]] * 2))
    scans = simulate(traj, scene, LidarSpec(), 0.0, 2.0, seed=1)
    t, a, r = _beams(scans)
    reads = ms200.synthetic_stream(t, a, r, 10.0, clock_start_ms=clock_start, wrap=wrap, seed=2)
    asm = ms200.Assembler(range_min=0.05, range_max=12.0)
    revs = list(ms200.revolutions(reads, ms200.FrameParser(), asm))
    assert len(revs) == len(scans) - 1 and asm.clock_resets == 0  # the last turn closes on the next wrap
    for rev, s in zip(revs, scans):
        assert len(rev.t) == len(s.t) and rev.rate_hz == pytest.approx(10.0) and rev.missing_frames == 0
        assert np.abs(rev.t - (s.t + HOST0)).max() < 1e-3        # within the sensor's 1 ms stamps
        assert np.abs(np.diff(rev.t) - 1 / 4500).max() < 1e-3     # no clock step inside a turn
        assert np.allclose(np.mod(rev.angle - s.angle + np.pi, 2 * np.pi) - np.pi, 0.0, atol=math.radians(0.01))
        assert np.array_equal(np.isnan(rev.range), np.isnan(s.range))
        assert np.nanmax(np.abs(rev.range - s.range)) <= 0.0005


def test_mount_offset_direction_and_range_limits(scene: Scene) -> None:
    traj = Trajectory(np.array([0.0, 1.0]), np.array([[-2.515, 0.914, 0.8]] * 2), np.array([[0.0, 0.0, math.pi / 2]] * 2))
    # mounted upside down: it sweeps counter-clockwise seen from above, its own angle still growing;
    # its 0 deg points 30 deg right of the nose, so its angle = body angle + 30 deg
    scans = simulate(traj, scene, LidarSpec(clockwise=False), 0.0, 1.0, seed=1)
    t, a, r = _beams(scans)
    reads = ms200.synthetic_stream(t, a + math.radians(30.0), r, 10.0, clockwise=False, seed=4)
    asm = ms200.Assembler(clockwise=False, yaw_offset_deg=-30.0, range_min=0.6)
    revs = list(ms200.revolutions(reads, ms200.FrameParser(), asm))
    assert len(revs) >= 8 and asm.dropped_partial == 1  # the first turn starts at 30 deg: partial
    near_seen = 0
    for rev in revs:
        assert len(rev.t) == 450
        idx = _truth(rev, t, a)
        assert np.abs(rev.t - t[idx]).max() < 1e-3
        near = np.isfinite(r[idx]) & (r[idx] < 0.599)  # 1 mm clear of the limit: ranges are whole mm
        assert np.all(np.isnan(rev.range[near]))  # closer than range_min: dropped
        far = np.isfinite(r[idx]) & (r[idx] > 0.601)
        assert np.abs(rev.range[far] - r[idx][far]).max() <= 0.0005
        near_seen += int(near.sum())
    assert near_seen > 0


def test_sensor_restart_resets_the_clock(scene: Scene) -> None:
    traj = Trajectory(np.array([0.0, 1.0]), np.array([[-2.515, 0.914, 0.8]] * 2), np.array([[0.0, 0.0, math.pi / 2]] * 2))
    scans = simulate(traj, scene, LidarSpec(), 0.0, 1.6, seed=1)
    t, a, r = _beams(scans)
    half = len(t) // 2 // 12 * 12 + 5 * 12
    first = ms200.synthetic_stream(t[:half], a[:half], r[:half], 10.0, clock_start_ms=100, seed=1)
    second = ms200.synthetic_stream(t[half:], a[half:], r[half:], 10.0, clock_start_ms=20000, seed=2)
    asm = ms200.Assembler(range_max=12.0)
    revs = list(ms200.revolutions(first + second, ms200.FrameParser(), asm))
    assert asm.clock_resets == 1 and asm.dropped_partial == 1  # the turn the restart cut
    assert len(revs) == len(scans) - 2
    for rev in revs:  # every turn on one clock, before and after the restart
        assert np.abs(rev.t - t[_truth(rev, t, a)]).max() < 2e-3


def test_recorder_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "x.ms200"
    rec = ms200.Recorder(path)
    rec.write(1.5, b"\x01\x02")
    rec.write(2.25, b"")
    rec.write(3.0, bytes(range(256)))
    rec.close()
    assert list(ms200.read_log(path)) == [(1.5, b"\x01\x02"), (2.25, b""), (3.0, bytes(range(256)))]
    (tmp_path / "bad.ms200").write_bytes(b"nope")
    with pytest.raises(ValueError):
        list(ms200.read_log(tmp_path / "bad.ms200"))


def test_bytes_to_pose(scene: Scene, field: DistanceField) -> None:
    traj = _walk()
    scans = simulate(traj, scene, LidarSpec(), 0.0, 2.95, seed=1)
    t, a, r = _beams(scans)
    revs = list(ms200.revolutions(ms200.synthetic_stream(t, a, r, 10.0, seed=3), ms200.FrameParser(),
                                  ms200.Assembler(range_min=0.25)))
    tracker = Tracker(field, MatchConfig(deskew=True, tilt=False), np.array([-2.49, 0.89, math.pi / 2 + 0.02]))
    errors = []
    for rev in revs:
        pose, stats = tracker.update(rev.t, rev.range, rev.angle, np.zeros((len(rev.t), 2)), 0.8)
        truth = traj.at(np.array([rev.t_end - HOST0]))[0][0]
        errors.append(np.hypot(*(pose[:2] - truth[:2])))
        assert abs(pose[2] - math.pi / 2) < math.radians(1.0)
    e = np.array(errors)
    assert tracker.lost == 0
    assert np.sqrt(np.mean(e ** 2)) < 0.015
    assert e.max() < 0.06            # the worst is the instant the walk starts (an instant 0 -> 1 m/s)
    assert tracker.vel[1] == pytest.approx(1.0, abs=0.1)


def test_tracker_keeps_the_prediction_when_a_scan_fails(field: DistanceField) -> None:
    tracker = Tracker(field, MatchConfig(deskew=True, tilt=False), np.array([-2.515, 0.914, math.pi / 2]))
    tracker.vel, tracker.t_ref = np.array([0.0, 1.0]), 0.0
    n = 450
    pose, stats = tracker.update(np.linspace(0.0, 0.1, n), np.full(n, np.nan), np.linspace(0, 2 * np.pi, n),
                                 np.zeros((n, 2)), 0.8)
    assert stats["lost"] == 1 and tracker.lost == 1
    assert pose[:2] == pytest.approx([-2.515, 1.014])


def test_cli_check_and_localize_on_a_recording(scene: Scene, tmp_path: Path) -> None:
    scans = simulate(_walk(), scene, LidarSpec(), 0.0, 2.0, seed=1)
    t, a, r = _beams(scans)
    rec = ms200.Recorder(tmp_path / "walk.ms200")
    for host_t, data in ms200.synthetic_stream(t, a - math.radians(10.0), r, 10.0, seed=3):
        rec.write(host_t, data)
    rec.close()
    script = str(REPO_ROOT / "course" / "18_lidar_live.py")
    out = subprocess.run([sys.executable, script, "check", "--replay", str(tmp_path / "walk.ms200")],
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    assert "rate 10.00 Hz" in out.stdout and "CRC errors 0" in out.stdout
    out = subprocess.run([sys.executable, script, "localize", "--replay", str(tmp_path / "walk.ms200"),
                          "--yaw-offset", "10", "--csv", str(tmp_path / "walk.csv")],
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    rows = list(csv.DictReader((tmp_path / "walk.csv").open()))
    assert len(rows) >= 18
    last = rows[-1]
    truth_y = float(np.interp(float(last["t"]) - HOST0, [0, 1.5, 3.0], [0.914, 0.914, 2.414]))
    assert float(last["x"]) == pytest.approx(-2.515, abs=0.03) and float(last["y"]) == pytest.approx(truth_y, abs=0.03)
    assert float(last["yaw_deg"]) == pytest.approx(90.0, abs=1.0)
    out = subprocess.run([sys.executable, script, "localize"], capture_output=True, text=True, timeout=60)
    assert out.returncode == 2  # no --port and no --replay
