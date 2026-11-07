"""``course.lidar_sim`` / ``course.lidar_localize``: synthetic MS200K scans and scan-to-map matching.

Runs on the course v3 map in ``course/maps``.  The matcher cases are the ones that broke earlier
versions: a start that sits past the middle of a 10 cm wall (the nearest-surface residual settled
one wall thickness off), the tube's 6 mm shell (thinner than the 2 cm distance grid), and a fast,
tilted pass where the scan is smeared over 20 cm and dips into the floor.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from course.course_map import CourseMap
from course.lidar_localize import DistanceField, MatchConfig, _beam_residual, match
from course.lidar_sim import FLOOR, NONE, TUBE, WALL, LidarSpec, Scene, Trajectory, simulate

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP = REPO_ROOT / "course" / "maps" / "course_v3_map.json"
Z = 0.9  # scan plane height used throughout (cruise 0.8 m + 0.1 m mount)


@pytest.fixture(scope="module")
def cmap() -> CourseMap:
    return CourseMap.load(MAP)


@pytest.fixture(scope="module")
def scene(cmap: CourseMap) -> Scene:
    return Scene.from_map(cmap)


@pytest.fixture(scope="module")
def field(cmap: CourseMap) -> DistanceField:
    return DistanceField(cmap, Z)


def _cast(scene: Scene, origin, direction):
    o = np.array([origin], dtype=float)
    d = np.array([direction], dtype=float)
    dist, kind = scene.cast(o, d / np.linalg.norm(d), np.random.default_rng(0), 0.0)
    return float(dist[0]), int(kind[0])


def _still(x: float, y: float, yaw: float, z: float = 0.8) -> Trajectory:
    return Trajectory(np.array([0.0, 10.0]), np.array([[x, y, z], [x, y, z]]), np.array([[0, 0, yaw], [0, 0, yaw]]))


def test_cast_walls_floor_tube_and_open_air(cmap: CourseMap, scene: Scene) -> None:
    a1 = next(o for o in cmap.obstacles_2d(Z - 0.02, Z + 0.02) if o.name == "A1 approach wall")
    face = float(np.min(np.asarray(a1.polygon)[:, 0]))
    dist, kind = _cast(scene, (-2.0, 0.0, Z), (1, 0, 0))
    assert kind == WALL and dist == pytest.approx(face + 2.0, abs=1e-6)
    dist, kind = _cast(scene, (-2.0, 0.0, Z), (0, 0, -1))
    assert kind == FLOOR and dist == pytest.approx(Z, abs=1e-6)
    tube = scene.tubes[0]
    centre = tube["p0"] + tube["u"] * tube["length"] / 2
    dist, kind = _cast(scene, (centre[0], centre[1], tube["p0"][2]), (1, 0, 0))
    assert kind == TUBE and dist == pytest.approx(tube["ellipses"][1][0], abs=1e-6)
    # a level beam above the 1.73 m walls hits nothing (it used to hit every box: slab bug)
    dist, kind = _cast(scene, (-2.0, 0.0, 2.0), (1, 0, 0))
    assert kind == NONE and not np.isfinite(dist)


def test_simulate_timing_freeze_and_level() -> None:
    traj = Trajectory(np.array([0.0, 1.0]), np.array([[-2.5, 0.0, 0.8], [-2.5, 2.0, 0.8]]),
                      np.array([[0.1, 0.2, math.pi / 2], [0.1, 0.2, math.pi / 2]]))
    scene = Scene(np.zeros((0, 3)), np.zeros(0), np.zeros((0, 3)), np.zeros(0, dtype=int))
    spec = LidarSpec()
    moving = simulate(traj, scene, spec, 0.0, 0.1001)[0]
    assert len(moving.t) == spec.points_per_rev == 450
    assert np.ptp(moving.t) == pytest.approx(449 / 4500)
    assert np.ptp(moving.true_pos[:, 1]) == pytest.approx(2.0 * 449 / 4500, rel=1e-6)
    assert np.all(np.diff(np.unwrap(moving.angle)) < 0)  # clockwise sweep
    frozen = simulate(traj, scene, spec, 0.0, 0.1001, freeze=True, level=True)[0]
    assert np.ptp(frozen.true_pos[:, 1]) == 0.0 and np.all(frozen.true_rpy[:, :2] == 0.0)
    # tilted, the half of the sweep that points down reaches the floor, the empty scene's only surface
    assert np.mean(moving.kind == FLOOR) > 0.3


def test_beam_residual_on_thin_shell_and_inside_a_thick_wall(cmap: CourseMap, field: DistanceField) -> None:
    tube = next(f for f in cmap.features if f["kind"] == "tube")
    inner_x = tube["p0"][0] - tube["inner"]["half_width"] * math.sqrt(1 - ((Z - tube["p0"][2]) / tube["inner"]["half_height"]) ** 2)
    shell = next(o for o in cmap.obstacles_2d(Z - 0.02, Z + 0.02) if o.kind == "tube" and np.mean(np.asarray(o.polygon)[:, 0]) < tube["p0"][0])
    face = float(np.max(np.asarray(shell.polygon)[:, 0]))
    assert face - np.min(np.asarray(shell.polygon)[:, 0]) < 0.01  # the shell is thinner than the 2 cm grid
    assert face == pytest.approx(inner_x, abs=0.003)
    origin = np.array([[tube["p0"][0], tube["p0"][1] + 0.7]] * 3)
    pts = origin + np.array([[face - origin[0, 0] + dx, 0.3] for dx in (0.0, 0.02, -0.003)])
    r, g = _beam_residual(field, pts, origin, 0.25)  # oblique beams: on the face, 2 cm short, 3 mm into it
    assert r == pytest.approx([0.0, 0.02, -0.003], abs=1e-6)  # point-to-plane: the perpendicular distance
    assert np.allclose(g, [[1.0, 0.0]] * 3)
    # 3 cm into the 10 cm A1 wall, seen from the west: 3 cm too far, not 7 cm short of its back face
    a1 = next(o for o in cmap.obstacles_2d(Z - 0.02, Z + 0.02) if o.name == "A1 approach wall")
    west = float(np.min(np.asarray(a1.polygon)[:, 0]))
    r, g = _beam_residual(field, np.array([[west + 0.07, 0.0]]), np.array([[-2.0, 0.0]]), 0.25)
    assert r[0] == pytest.approx(-0.07, abs=1e-6) and np.allclose(g, [[-1.0, 0.0]])
    r_near, _ = field.lookup(np.array([[west + 0.07, 0.0]]))
    assert r_near[0] > -0.04  # the nearest-surface field sees it 3 cm from the back face instead


@pytest.mark.parametrize("xy, yaw", [((-2.50, 1.04), 1.574), ((-2.13, -0.82), 0.0), ((2.04, 0.50), 1.571)])
def test_match_converges_from_offset_starts(scene: Scene, field: DistanceField, xy, yaw) -> None:
    traj = _still(xy[0], xy[1], yaw)
    s = simulate(traj, scene, LidarSpec(), 0.0, 0.1001, seed=3, freeze=True, level=True)[0]
    truth = np.array([xy[0], xy[1], yaw])
    for off in ([0.08, 0, 0], [-0.08, 0, 0], [0, 0.08, 0], [0, -0.08, 0], [0.06, 0.06, 0.04], [0.15, -0.1, 0.06]):
        pose, stats = match(field, s.range, s.angle, np.zeros(len(s.t)), np.zeros((len(s.t), 2)), truth + off, 0.8,
                            np.zeros(2), 0.0, MatchConfig(deskew=False, tilt=False))
        assert np.hypot(*(pose[:2] - truth[:2])) < 0.01, (off, pose - truth)
        assert abs(pose[2] - truth[2]) < math.radians(0.5)
        assert "failed" not in stats


def test_deskew_and_tilt_compensation(scene: Scene, field: DistanceField) -> None:
    """2 m/s north up the first corridor, pitched 10 deg and rolled 5 deg: the snapshot / level
    assumption is several cm off, deskew + tilt compensation is sub-centimetre."""
    yaw, v = math.pi / 2, 2.0
    traj = Trajectory(np.array([0.0, 1.0]), np.array([[-2.5, 0.6, 0.8], [-2.5, 0.6 + v, 0.8]]),
                      np.array([[0.09, 0.17, yaw], [0.09, 0.17, yaw]]))
    s = simulate(traj, scene, LidarSpec(), 0.0, 0.1001, seed=5)[0]
    pos, rpy = traj.at(np.array([s.t_end]))
    truth = np.array([pos[0, 0], pos[0, 1], yaw])
    args = (field, s.range, s.angle, s.t - s.t_end, s.true_rpy[:, :2], truth + [0.03, -0.03, 0.01], 0.8,
            np.array([0.0, v]), 0.0)
    naive, _ = match(*args, MatchConfig(deskew=False, tilt=False))
    full, stats = match(*args, MatchConfig(deskew=True, tilt=True), scene.wall_top)
    assert stats["floor_dropped"] > 10
    assert np.hypot(*(naive[:2] - truth[:2])) > 0.04
    assert np.hypot(*(full[:2] - truth[:2])) < 0.01


def test_as_built_moves_pieces_rigidly(cmap: CourseMap) -> None:
    spec = importlib.util.spec_from_file_location("lidar_study", REPO_ROOT / "course" / "17_lidar_study.py")
    study = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(study)
    built = study.as_built(cmap, 0.02, 1.0, seed=1)
    before = {f["name"]: f for f in cmap.features}
    moved = 0
    for f in built.features:
        g = before[f["name"]]
        if f["kind"] in ("floor", "marking", "net"):
            assert f == g
        if "p0" in f:
            length = lambda h: np.hypot(*(np.array(h["p1"][:2]) - np.array(h["p0"][:2])))  # noqa: E731
            assert length(f) == pytest.approx(length(g), abs=1e-9)
            moved += f["p0"] != g["p0"]
    assert moved > 10
    tube_shift = np.array(built.feature("Tube oval 58 x 72 x 60")["p0"][:2]) - before["Tube oval 58 x 72 x 60"]["p0"][:2]
    cradle_shift = np.array(built.feature("Tube cradle 2")["p0"]) - before["Tube cradle 2"]["p0"]
    assert np.linalg.norm(cradle_shift - tube_shift) < 0.02  # cradles ride with the tube
