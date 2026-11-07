"""``course/room_mapper.py`` and ``course/21_room_map.py``: map a room from a recorded walk.

The end-to-end test walks a simulated rig around a 20_bench_map.py room with two boxes, encodes the
scans as the MS200K byte stream (``ms200.synthetic_stream``) next to a PX4 attitude log, runs
``build``, and checks every inside face of the written map against the room it came from.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

from course import ms200
from course import room_mapper as rm
from course.course_map import CourseMap
from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate

REPO_ROOT = Path(__file__).resolve().parents[1]
ROOM, START, HEIGHT, MOUNT_Z = (4.2, 6.1), (2.1, 1.0), 0.95, 0.10
BOXES = [(3.4, 2.0, 4.2, 2.6, 1.2), (0.0, 4.0, 0.6, 4.8, 1.5)]
# (t s, x, y, yaw deg) in the room frame: still 3 s on the mark (the first second of lidar goes to the
# clock map, and the first trusted revolution defines the map frame), out 3 m, across, back along the
# left side, home facing ahead, still 3 s.  Turns of 90 deg take 1.5-3 s (30-60 deg/s), faster than
# the record step advises.
_W = [(1.0, 2.1, 1.0, 90), (8.5, 2.1, 4.0, 90), (10.0, 2.1, 4.0, 0), (13.0, 3.3, 4.0, 0), (16.0, 3.3, 4.0, 180),
      (21.0, 1.2, 4.0, 180), (23.0, 1.2, 4.0, 270), (30.0, 1.2, 1.0, 270), (33.0, 1.2, 1.0, 360), (36.0, 2.1, 1.0, 360),
      (38.0, 2.1, 1.0, 450)]
WAYPOINTS = [(0, 2.1, 1.0, 90)] + [(t + 2.0, x, y, a) for t, x, y, a in _W] + [(43.0, 2.1, 1.0, 450)]


def _load(name: str, file: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "course" / file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _walk() -> Trajectory:
    w = np.array(WAYPOINTS, dtype=float)
    t = np.arange(0.0, w[-1, 0], 0.01)
    yaw = np.radians(np.interp(t, w[:, 0], w[:, 3]))
    pos = np.stack([np.interp(t, w[:, 0], w[:, 1]), np.interp(t, w[:, 0], w[:, 2]), np.full(len(t), HEIGHT)], axis=1)
    return Trajectory(t, pos, np.stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw], axis=1))


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """The walk recorded as 21_room_map.py record would leave it, then built."""
    out = tmp_path_factory.mktemp("room")
    bench = _load("bench_map", "20_bench_map.py")
    cmap = CourseMap(bench.room_map(*ROOM, START, 2.5, BOXES))
    traj = _walk()
    scans = simulate(traj, Scene.from_map(cmap), LidarSpec(mount=(0.0, 0.0, MOUNT_Z)), 0.0, traj.t[-1], seed=3)
    t = np.concatenate([s.t for s in scans])
    reads = ms200.synthetic_stream(t, np.concatenate([s.angle for s in scans]), np.concatenate([s.range for s in scans]),
                                   10.0, seed=1)
    rec = ms200.Recorder(out / "walk.ms200")
    for host_t, data in reads:
        rec.write(host_t, data)
    rec.close()
    yaw_enu = np.unwrap(traj.rpy[:, 2])
    with (out / "walk_att.csv").open("w", newline="") as fh:  # PX4's ATTITUDE: FRD / NED
        w = csv.writer(fh)
        w.writerow(["t", "roll", "pitch", "yaw", "yaw_dot"])
        for ti, y, r in zip(traj.t, yaw_enu, np.gradient(yaw_enu, traj.t)):
            w.writerow([ti, 0.0, 0.0, math.pi / 2 - y, -r])
    tool = _load("room_map", "21_room_map.py")
    code = tool.main(["build", str(out / "walk.ms200"), "--height", str(HEIGHT), "--mount-z", str(MOUNT_Z),
                      "--closed", "--out", str(out / "map")])
    return code, out / "map", scans


def test_scan_normals_on_a_line_and_a_corner() -> None:
    x = np.linspace(-1, 1, 101)
    line = np.stack([x, 0.5 * x + 2.0], axis=1)
    n = rm.scan_normals(line)
    want = np.array([-0.5, 1.0]) / math.hypot(0.5, 1.0)
    assert np.allclose(np.abs(n[5:-5] @ want), 1.0, atol=1e-6)
    gap = line.copy()
    gap[40:60] = np.nan                                      # no return: no normal there
    assert np.isnan(rm.scan_normals(gap)[40:60]).all()


def test_point_map_lookup_is_point_to_plane() -> None:
    x = np.arange(0.0, 3.0, 0.005)
    pts = np.stack([x, np.zeros_like(x)], axis=1)            # a wall along y = 0, seen from y > 0
    pm = rm.PointMap(pts, np.tile([0.0, 1.0], (len(x), 1)), np.tile([1.5, 1.0], (len(x), 1)))
    r, g = pm.lookup(np.array([[1.0, 0.07], [2.0, -0.03], [1.5, 0.5]]))
    assert r[0] * g[0, 1] == pytest.approx(0.07, abs=1e-3) and r[1] * g[1, 1] == pytest.approx(-0.03, abs=1e-3)
    assert r[2] == rm.OUTLIER_R                              # nothing within 0.3 m: dropped by match()


def test_too_short_a_walk_is_refused() -> None:
    rev = (np.linspace(0, 0.1, 450), np.full(450, 2.0), np.linspace(0, 2 * math.pi, 450, endpoint=False), None, None)
    with pytest.raises(rm.MappingError):
        rm.build([rev] * 5)


def test_build_maps_every_inside_face(built) -> None:
    code, out, _ = built
    assert code == 0
    report = json.loads((out / "room_report.json").read_text())
    assert report["check_passed"] and report["check"]["trusted"] >= 0.98
    assert report["mapping"]["slam_trusted"] == report["mapping"]["slam_revolutions"]
    assert report["mapping"]["start_motion_m"] < 0.01        # still on the mark: no "moved at the start" warning
    cmap = CourseMap.load(out / "room_map.json")
    walls = cmap.of_kind("wall")
    # inside faces in the map frame (the start pose is the origin, nose +y as in the room)
    sx, sy = START
    faces = {"left": ("x", -sx), "right": ("x", ROOM[0] - sx), "back": ("y", -sy), "front": ("y", ROOM[1] - sy),
             "box 1 west": ("x", 3.4 - sx), "box 1 south": ("y", 2.0 - sy), "box 1 north": ("y", 2.6 - sy),
             "box 2 east": ("x", 0.6 - sx)}
    for name, (axis, want) in faces.items():
        k = 0 if axis == "x" else 1
        best = None
        for f in walls:
            p0, p1 = np.array(f["p0"]), np.array(f["p1"])
            if abs(p0[k] - p1[k]) > 0.02:                    # not parallel to this face
                continue
            for side in (-1, 1):                             # the faces of the thin wall
                err = abs((p0[k] + p1[k]) / 2 + side * f["thickness"] / 2 - want)
                best = err if best is None else min(best, err)
        assert best is not None and best < 0.015, "{} face: {}".format(name, best)
    rules = json.loads((out / "room_rules.json").read_text())
    summary = json.loads((out / "room_summary.json").read_text())
    assert cmap.feature(rules["start"]["marking"])["centre"] == [0.0, 0.0]
    assert summary["px4_ned_origin_enu"] == [0.0, 0.0, HEIGHT] and Path(summary["rules"]).is_file()
    assert (out / "room_map.png").stat().st_size > 1000


def test_mapper_poses_follow_the_walk(built) -> None:
    _, out, scans = built
    rows = list(csv.DictReader((out / "room_poses.csv").open()))
    assert len(rows) >= 0.95 * len(scans)
    truth_t = np.array([s.t_end for s in scans])
    truth = np.array([[s.true_pos[-1, 0] - START[0], s.true_pos[-1, 1] - START[1]] for s in scans])
    t = np.array([float(r["t"]) for r in rows])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    want = np.stack([np.interp(t, truth_t, truth[:, 0]), np.interp(t, truth_t, truth[:, 1])], axis=1)
    err = np.hypot(*(xy - want).T)
    assert np.sqrt(np.mean(err ** 2)) < 0.02 and err.max() < 0.06


def test_live_mapper_tracks_the_walk_from_the_start(built) -> None:
    """room_mapper.LiveMapper, the live view's no-map mode: one revolution at a time, the map grown
    after each pose (grow()), poses relative to the start within a few cm, nothing lost."""
    _, _, scans = built
    traj_yaw = np.unwrap(_walk().rpy[:, 2])
    rate = np.gradient(traj_yaw, _walk().t)
    lm = rm.LiveMapper(rm.MapConfig(height=HEIGHT, mount_z=MOUNT_Z), tilt=False)
    errs, added = [], 0
    for s in scans:
        g = float(np.interp(s.t_end, _walk().t, rate))
        pose, st = lm.update((s.t, s.range, s.angle, None, g))
        assert "lost" not in st
        added += lm.grow() is not None
        errs.append(math.hypot(pose[0] - (s.true_pos[-1, 0] - START[0]), pose[1] - (s.true_pos[-1, 1] - START[1])))
    assert added >= 20 and len(lm.kfs) == added
    assert np.sqrt(np.mean(np.square(errs))) < 0.02 and max(errs) < 0.08
