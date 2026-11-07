"""``course`` package: STEP -> map, geometry, slicing, speed profile, planner, Gazebo world, CLIs.

Runs on the real competition v3 STEP file in ``course/maps`` (no CAD software needed).  The numbers
checked against the published drawing: 0.927 m maze gaps (36.5 in), a 36 x 36 in window with a
12 in sill, a 58 x 72 x 60 in tube centred 37 in up, a 5 ft pad, 18 in boulders.
"""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from course.course_map import CourseMap, extract_map
from course.geometry import (PolygonSet, closest_points_between, convex_hull, min_area_rect, minkowski_sum,
                             points_in_convex, polygon_area, rect_polygon, signed_distance)
from course.planner.cspace import DroneSpec, build_cspace
from course.planner.plan import PlanConfig, plan_course, write_outputs
from course.planner.smooth import _StepSystem
from course.planner.speed import signed_curvature, speed_profile, time_stamps
from course.worldgen import build_model, build_world, tube_obj

REPO_ROOT = Path(__file__).resolve().parents[1]
STEP = REPO_ROOT / "course" / "maps" / "course_v3.step"
RULES = REPO_ROOT / "course" / "maps" / "course_v3_rules.json"
PYTHON = sys.executable
INCH = 0.0254


@pytest.fixture(scope="module")
def cmap() -> CourseMap:
    return CourseMap(extract_map(STEP))


@pytest.fixture(scope="module")
def rules() -> dict:
    return json.loads(RULES.read_text(encoding="utf-8"))


# ------------------------------------------------------------------------------------- the map


def test_feature_kinds(cmap: CourseMap) -> None:
    kinds = {}
    for f in cmap.features:
        kinds[f["kind"]] = kinds.get(f["kind"], 0) + 1
    assert kinds == {"wall": 10, "rock": 5, "net": 5, "marking": 8, "tube": 1, "support": 3, "floor": 1}
    assert cmap.boundary == {"x": [-3.048, 3.048], "y": [-3.048, 3.048], "z": [0.0, 3.048]}


def test_window_opening(cmap: CourseMap) -> None:
    wall = cmap.feature("C2a chicane wall (window)")
    assert len(wall["openings"]) == 1
    o = wall["openings"][0]
    assert o["s"][1] - o["s"][0] == pytest.approx(36 * INCH, abs=1e-4)
    assert o["z"] == pytest.approx([12 * INCH, 48 * INCH], abs=1e-4)
    assert wall["thickness"] == pytest.approx(4 * INCH, abs=1e-4)


def test_tube(cmap: CourseMap) -> None:
    tube = cmap.feature("Tube oval 58 x 72 x 60")
    assert tube["kind"] == "tube"
    assert 2 * tube["outer"]["half_width"] == pytest.approx(58 * INCH, abs=1e-4)
    assert 2 * tube["outer"]["half_height"] == pytest.approx(72 * INCH, abs=1e-4)
    assert math.dist(tube["p0"], tube["p1"]) == pytest.approx(60 * INCH, abs=1e-4)
    assert tube["p0"][2] == pytest.approx(37 * INCH, abs=1e-4)
    assert all(cmap.feature("Tube cradle {}".format(i))["kind"] == "support" for i in (1, 2, 3))


def test_maze_gaps_and_markings(cmap: CourseMap) -> None:
    m1 = cmap.feature("M1 maze divider")
    assert cmap.boundary["y"][1] - m1["p1"][1] == pytest.approx(36.5 * INCH, abs=1e-4)
    pad = cmap.feature("Landing pad 5 ft")
    assert pad["half_size"] == pytest.approx([30 * INCH, 30 * INCH], abs=1e-4)
    rock = cmap.feature("Boulder 1")
    assert 2 * rock["radius"] == pytest.approx(18 * INCH, abs=1e-4)
    launch = cmap.feature("Launch mark")
    assert launch["centre"] == pytest.approx([-2.5146, 0.9144], abs=1e-4)


def test_obstacle_slices(cmap: CourseMap) -> None:
    at_cruise = [o for o in cmap.obstacles_2d(0.65, 0.95) if o.name.startswith("C2a")]
    assert len(at_cruise) == 2  # the window is open at this height
    below_sill = [o for o in cmap.obstacles_2d(0.10, 0.25) if o.name.startswith("C2a")]
    assert len(below_sill) == 1
    tube = [o for o in cmap.obstacles_2d(0.65, 0.95) if o.kind == "tube"]
    assert len(tube) == 2 and all(polygon_area(o.polygon) < 0.1 for o in tube)
    too_tall = [o for o in cmap.obstacles_2d(0.0, 1.9) if o.kind == "tube"]
    assert len(too_tall) == 1  # the band does not fit inside: the whole tube blocks
    assert not any(o.kind == "rock" for o in cmap.obstacles_2d(0.65, 0.95, rocks_full_height=False))


def test_map_round_trip(cmap: CourseMap, tmp_path: Path) -> None:
    path = tmp_path / "map.json"
    cmap.save(path)
    again = CourseMap.load(path)
    assert again.data == json.loads(json.dumps(cmap.data))


# ------------------------------------------------------------------------------------- geometry


def test_hull_rect_minkowski() -> None:
    pts = np.random.default_rng(1).uniform(-1, 1, (200, 2))
    hull = convex_hull(pts)
    assert polygon_area(hull) > 0 and points_in_convex(pts, hull, tol=1e-9).all()
    rect = rect_polygon((1.0, 2.0), 0.7, 3.0, 0.5)
    c, angle, length, width = min_area_rect(rect)
    assert c == pytest.approx([1.0, 2.0]) and angle == pytest.approx(0.7)
    assert (length, width) == pytest.approx((3.0, 0.5))
    unit = rect_polygon((0, 0), 0.0, 1.0, 1.0)
    assert polygon_area(minkowski_sum(unit, unit)) == pytest.approx(4.0)


def test_signed_distance_and_packed() -> None:
    square = rect_polygon((0, 0), 0.0, 2.0, 2.0)
    d, g = signed_distance(np.array([[3.0, 0.0], [0.5, 0.0], [2.0, 2.0]]), [square])
    assert d == pytest.approx([2.0, -0.5, math.sqrt(2)])
    assert g[0] == pytest.approx([1, 0]) and g[1] == pytest.approx([1, 0])
    polys = [rect_polygon((x, y), a, 1.0, 0.2) for x, y, a in ((0, 0, 0.3), (1.5, 0.5, 1.2), (-1, 1, 2.0))]
    pts = np.random.default_rng(2).uniform(-2.5, 2.5, (500, 2))
    d1, g1 = signed_distance(pts, polys)
    d2, g2 = PolygonSet(polys).distance(pts)
    assert np.allclose(d1, d2) and np.allclose(g1, g2)


def test_closest_points_parallel_walls() -> None:
    a = rect_polygon((0.3048, 1.3), np.pi / 2, 1.6, 0.1)  # vertical wall, top end at y = 2.1
    b = rect_polygon((0.0, 3.1), 0.0, 6.0, 0.02)          # horizontal net, bottom face at y = 3.09
    pa, pb, d = closest_points_between(a, b)
    assert d == pytest.approx(0.99)
    assert pa == pytest.approx([0.3048, 2.1]) and pb == pytest.approx([0.3048, 3.09])


# ---------------------------------------------------------------------------- speed and solver


def test_speed_profile_matches_physics() -> None:
    r, a_max = 0.5, 2.0
    t = np.linspace(0, 6 * math.pi, 2000)
    circle = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
    v = speed_profile(circle, a_max, np.full(len(circle), 10.0), signed_curvature(circle))
    assert np.median(v[500:1500]) == pytest.approx(math.sqrt(a_max * r), rel=0.02)
    line = np.stack([np.linspace(0, 4.0, 4001), np.zeros(4001)], axis=1)
    v = speed_profile(line, a_max, np.full(len(line), 100.0))
    assert time_stamps(line, v)[-1] == pytest.approx(2 * math.sqrt(4.0 / a_max), rel=0.01)  # bang-bang
    v = speed_profile(line, a_max, np.full(len(line), 1.0))
    assert time_stamps(line, v)[-1] == pytest.approx(4.0 / 1.0 + 1.0 / a_max, rel=0.01)  # trapezoid


def test_step_system_solves_like_dense() -> None:
    n, h, lam = 40, 0.04, 0.1
    d = np.diff(np.eye(n), axis=0)
    k = np.diff(np.eye(n), 2, axis=0)
    two_a = 2 * (d.T @ d / h + lam * k.T @ k / h ** 3)
    system = _StepSystem(n, h, lam, np.array([0, n - 1]))
    m = two_a + system.shift * np.eye(n)
    rng = np.random.default_rng(3)
    rhs, fixed_values = rng.random((n, 2)), rng.random((n, 2))
    free = np.arange(1, n - 1)
    expected = fixed_values.copy()
    expected[free] = np.linalg.solve(m[np.ix_(free, free)], rhs[free] - m[np.ix_(free, [0, n - 1])] @ fixed_values[[0, n - 1]])
    assert np.allclose(system.solve(rhs, fixed_values, fixed_values), expected, atol=1e-9)


# ------------------------------------------------------------------------------------- planner


@pytest.fixture(scope="module")
def window_plan(cmap: CourseMap, rules: dict):
    return plan_course(cmap, rules, "window", PlanConfig(smoothness=(0.1,), iterations=250))


def test_plan_is_clear_and_in_order(window_plan) -> None:
    plan = window_plan
    t = [cp["time_s"] for cp in plan.checkpoints]
    assert t == sorted(t) and len(t) == 5
    dense = np.concatenate([plan.path, 0.5 * (plan.path[:-1] + plan.path[1:])])
    d, _ = plan.cspace.distance(dense)
    assert d.min() >= -2e-3  # the whole drone plus margin stays clear (2 mm numerical tolerance)
    for gate, idx in zip(plan.gates, plan.gate_index):
        assert np.linalg.norm(gate.project(plan.path[idx]) - plan.path[idx]) < 1e-6
    assert 8.0 < plan.finish_s < 30.0
    assert plan.return_s is not None and plan.finish_s + plan.return_s < 180.0


def test_plan_gate_room_matches_hand_numbers(window_plan) -> None:
    widths = {g.name.split(" ")[0]: g.free_width for g in window_plan.gates}
    # Gate A: 0.927 m gap - 0.40 m drone length (nose north) - 2 x 0.05 m margin = 0.427 m
    assert widths["CP2"] == pytest.approx(0.927 - 0.40 - 0.10, abs=0.005)


def test_plan_outputs(window_plan, tmp_path: Path) -> None:
    paths = write_outputs(window_plan, tmp_path, "w")
    with paths["ned"].open() as fh:
        rows = list(csv.DictReader(fh))
    assert rows[0]["phase"] == "takeoff"
    assert [float(rows[0][k]) for k in ("north", "east", "down")] == pytest.approx([0.0, 0.0, 0.0], abs=1e-3)
    assert float(rows[0]["yaw"]) == pytest.approx(0.0, abs=1e-4)  # nose north is NED yaw 0
    phases = [r["phase"] for r in rows]
    assert phases.index("course") < phases.index("landing") < phases.index("return_climb") \
        < phases.index("return_cross") < phases.index("return_descent")
    summary = json.loads(paths["summary"].read_text())
    assert summary["route"] == "window" and summary["checkpoints"][3]["bonus_points"] == 5


def test_narrow_heading_still_plans(cmap: CourseMap, rules: dict) -> None:
    plan = plan_course(cmap, rules, "gate_b", PlanConfig(yaw_deg=0.0, smoothness=(0.1,), iterations=150))
    assert len(plan.checkpoints) == 5


# ----------------------------------------------------------------------------------- worldgen


def test_world_files(cmap: CourseMap) -> None:
    model_sdf, meshes, notes = build_model(cmap, "course_test")
    root = ET.fromstring(model_sdf)
    window_boxes = [v for v in root.iter("visual") if v.get("name", "").startswith("C2a")]
    assert len(window_boxes) == 4  # left, below, above, right of the window
    assert len([n for n in notes if "support" in n]) == 3
    assert "meshes/tube_0.obj" in meshes
    world = ET.fromstring(build_world("w", "course_test"))
    assert world.find(".//include/uri").text == "model://course_test"
    assert not world.findall(".//plugin")  # PX4's server.config supplies the systems
    assert len(ET.fromstring(build_world("w", "m", embed_systems=True)).findall(".//plugin")) >= 9


def test_tube_mesh_faces_outward(cmap: CourseMap) -> None:
    tube = cmap.feature("Tube oval 58 x 72 x 60")
    lines = tube_obj(tube, segments=16).splitlines()
    verts = np.array([[float(x) for x in ln.split()[1:]] for ln in lines if ln.startswith("v ")])
    faces = [[int(tok.split("/")[0]) - 1 for tok in ln.split()[1:]] for ln in lines if ln.startswith("f ")]
    assert len(faces) == 16 * 4 * 2
    centre = (np.array(tube["p0"]) + np.array(tube["p1"])) / 2
    volume = sum(np.dot(verts[a] - centre, np.cross(verts[b] - centre, verts[c] - centre)) for a, b, c in faces) / 6
    hw_o, hh_o = tube["outer"]["half_width"], tube["outer"]["half_height"]
    hw_i, hh_i = tube["inner"]["half_width"], tube["inner"]["half_height"]
    expected = math.pi * (hw_o * hh_o - hw_i * hh_i) * math.dist(tube["p0"], tube["p1"])
    assert volume == pytest.approx(expected, rel=0.05)  # positive: closed and wound outwards


# ------------------------------------------------------------------------------------------ CLIs


def test_cli_pipeline(tmp_path: Path) -> None:
    out_map = tmp_path / "course_v3_map.json"
    r = subprocess.run([PYTHON, str(REPO_ROOT / "course" / "12_extract_course_map.py"), str(STEP), "--out", str(out_map)],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert "0.927 m  M3 maze divider <-> Net side north" in r.stdout
    r = subprocess.run([PYTHON, str(REPO_ROOT / "course" / "13_make_gazebo_world.py"), "--map", str(out_map),
                        "--out", str(tmp_path / "gz")], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "gz" / "worlds" / "course_v3.sdf").is_file()
    assert (tmp_path / "gz" / "models" / "course_v3_model" / "meshes" / "floor.png").is_file()
    rules = json.loads(RULES.read_text())
    rules["map"] = out_map.name
    (tmp_path / "rules.json").write_text(json.dumps(rules))
    r = subprocess.run([PYTHON, str(REPO_ROOT / "course" / "14_plan_course.py"), "--rules", str(tmp_path / "rules.json"),
                        "--route", "window", "--iterations", "120", "--out", str(tmp_path / "out")],
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "+5 bonus" in r.stdout
    assert (tmp_path / "out" / "window_yaw90_px4_ned.csv").is_file()
    assert (tmp_path / "out" / "window_yaw90.png").is_file()
    r = subprocess.run([PYTHON, str(REPO_ROOT / "course" / "14_plan_course.py"), "--rules", str(tmp_path / "nope.json")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 2
