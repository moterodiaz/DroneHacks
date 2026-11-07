"""``course/20_bench_map.py``: a room map, so the lidar tools run on the bench.

The map it writes must load in every tool that reads a map: 18_lidar_live.py localize (through
``--rules``) and 19_lidar_px4_bridge.py (through ``--summary``), the latter end to end in --sim
mode against a fake Gazebo walking the room and a fake PX4 link.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from course import ms200
from course.course_map import CourseMap
from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate

REPO_ROOT = Path(__file__).resolve().parents[1]
ROOM, START, HEIGHT = (4.2, 6.1), (2.1, 1.0), 0.95
WALK_T, WALK_Y = [0.0, 0.3, 4.0], [1.0, 1.0, 1.0 + 0.5 * 3.7]  # still on the mark, then 0.5 m/s ahead


def _load(name: str, file: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "course" / file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def bench(tmp_path_factory) -> Path:
    out = tmp_path_factory.mktemp("bench")
    tool = _load("bench_map", "20_bench_map.py")
    code = tool.main(["--room", *map(str, ROOM), "--start", *map(str, START), "--height", str(HEIGHT),
                      "--box", "3.4", "2.0", "4.2", "2.6", "1.2", "--out", str(out)])
    assert code == 0
    return out


def test_map_has_the_inside_faces_where_measured(bench: Path) -> None:
    cmap = CourseMap.load(bench / "bench_map.json")
    rules = json.loads((bench / "bench_rules.json").read_text(encoding="utf-8"))
    summary = json.loads((bench / "bench_summary.json").read_text(encoding="utf-8"))
    assert cmap.feature(rules["start"]["marking"])["centre"] == list(START)
    assert summary["px4_ned_origin_enu"] == [*START, HEIGHT] and Path(summary["rules"]).is_file()
    scene = Scene.from_map(cmap)
    hit = lambda d: scene.cast(np.array([[*START, 1.05]]), np.array([d], dtype=float), np.random.default_rng(0), 0.0)[0][0]  # noqa: E731
    assert hit([-1, 0, 0]) == pytest.approx(START[0]) and hit([1, 0, 0]) == pytest.approx(ROOM[0] - START[0])
    assert hit([0, -1, 0]) == pytest.approx(START[1]) and hit([0, 1, 0]) == pytest.approx(ROOM[1] - START[1])
    assert hit([1.7, 1.0, 0] / np.linalg.norm([1.7, 1.0, 0])) == pytest.approx(math.hypot(1.7, 1.0))  # the box's near face


@pytest.mark.parametrize("extra", [["--start", "0.2", "1.0"], ["--box", "1.9", "0.8", "2.4", "1.3", "1.0"],
                                   ["--height", "2.3"]])
def test_bad_layouts_are_refused(tmp_path: Path, extra) -> None:
    tool = _load("bench_map_bad", "20_bench_map.py")
    args = ["--room", "4.2", "6.1", "--start", "2.1", "1.0", "--height", "0.95", "--out", str(tmp_path)]
    assert tool.main(args + extra) == 2
    assert not (tmp_path / "bench_map.json").exists()


def test_localize_cli_in_the_room(bench: Path, tmp_path: Path) -> None:
    scene = Scene.from_map(CourseMap.load(bench / "bench_map.json"))
    traj = Trajectory(np.array(WALK_T), np.array([[START[0], y, HEIGHT] for y in WALK_Y]),
                      np.array([[0.0, 0.0, math.pi / 2]] * 3))
    scans = simulate(traj, scene, LidarSpec(), 0.0, 2.0, seed=1)
    rec = ms200.Recorder(tmp_path / "room.ms200")
    t, a, r = (np.concatenate([getattr(s, k) for s in scans]) for k in ("t", "angle", "range"))
    for host_t, data in ms200.synthetic_stream(t + 1000.0, a, r, 10.0, seed=3):
        rec.write(host_t, data)
    rec.close()
    out = subprocess.run([sys.executable, str(REPO_ROOT / "course" / "18_lidar_live.py"), "localize",
                          "--replay", str(tmp_path / "room.ms200"), "--rules", str(bench / "bench_rules.json"),
                          "--scan-z", str(HEIGHT + 0.1), "--csv", str(tmp_path / "room.csv")],
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    last = list(csv.DictReader((tmp_path / "room.csv").open(encoding="utf-8")))[-1]
    truth_y = float(np.interp(float(last["t"]) - 1000.0, WALK_T, WALK_Y))
    assert float(last["x"]) == pytest.approx(START[0], abs=0.03) and float(last["y"]) == pytest.approx(truth_y, abs=0.03)
    assert float(last["yaw_deg"]) == pytest.approx(90.0, abs=1.0)


class _FakeTruth:
    """Gazebo stand-in: the rig on the start mark, then carried 0.5 m/s straight ahead."""

    def __init__(self, world, model) -> None:
        self.traj = Trajectory(np.array(WALK_T), np.array([[START[0], y, HEIGHT] for y in WALK_Y]),
                               np.array([[0.0, 0.0, math.pi / 2]] * 3))
        self.t = 0.3

    def wait(self, timeout_s: float = 30.0) -> bool:
        return True

    def latest_t(self) -> float:
        self.t = min(self.t + 0.004, 3.9)
        return self.t

    def now(self) -> float:
        return self.t

    def trajectory(self) -> Trajectory:
        return self.traj


class _FakeLink:
    """PX4 on the bench: disarmed and landed, level, its position the truth; keeps every ODOMETRY."""

    made = []
    landed = True

    def __init__(self, url, baud, clock) -> None:
        self.clock, self.sent, self.timesync_replies = clock, [], 600
        _FakeLink.made.append(self)

    def attitude(self, t):
        return np.zeros((len(t), 3))

    def velocity(self, t, course_yaw):
        return np.array([0.0, 0.5 if t > 0.3 else 0.0])

    def height(self, rest_height):
        return None

    def yaw_rate(self, t):
        return 0.0

    def local_position(self):
        t = self.clock()
        return t, np.array([float(np.interp(t, WALK_T, WALK_Y)) - START[1], 0.0, 0.0])

    def send_odometry(self, fields) -> None:
        self.sent.append(fields)


def test_bridge_runs_on_the_room_map(bench: Path, tmp_path: Path, monkeypatch) -> None:
    bridge = _load("bridge_bench", "19_lidar_px4_bridge.py")
    monkeypatch.setattr(bridge, "SLICE_HEIGHTS", (0.2, 0.9))
    monkeypatch.setattr(bridge, "Px4Link", _FakeLink)
    monkeypatch.setitem(sys.modules, "course.gz_truth", types.SimpleNamespace(GzTruth=_FakeTruth))
    _FakeLink.made.clear()
    code = bridge.main(["--sim", "--summary", str(bench / "bench_summary.json"), "--seconds", "2.0",
                        "--log", str(tmp_path / "log")])
    assert code == 0
    rows = [r for r in csv.DictReader((tmp_path / "log" / "bridge_log.csv").open(encoding="utf-8")) if r["sent"] == "1"]
    sent = _FakeLink.made[0].sent
    assert len(sent) == len(rows) >= 15
    for f, r in zip(sent, rows):
        # PX4's frame starts on the mark: north = ahead of the rig, east = to its right
        assert f["x"] == pytest.approx(float(r["truth_y"]) - START[1], abs=0.02)
        assert f["y"] == pytest.approx(float(r["truth_x"]) - START[0], abs=0.02)
        assert float(r["z_body"]) == HEIGHT  # landed: the rig's height from the summary
