"""``course.step_reader`` on both CAD exporters: Onshape (``course/maps/course_v3.step``) and
SolidWorks (the repo-root ``dronehacks-course-v3.step``, the 2026-11-07 layout).

SolidWorks names parts ``'<part>.step'``, gives solids generic names, keeps a virtual component's
bodies in one part (``'Part67^<assembly>'``) and writes circles and ellipses as B-splines.  The map
the extractor builds from it must still have the boulders as rocks and the tube as a tube, with
the same numbers as the Onshape export of the parts that did not change.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from course.course_map import extract_map
from course.step_reader import _clean_product_name, _fit_conic

REPO_ROOT = Path(__file__).resolve().parents[1]
ONSHAPE_STEP = REPO_ROOT / "course" / "maps" / "course_v3.step"
ONSHAPE_MAP = REPO_ROOT / "course" / "maps" / "course_v3_map.json"
SOLIDWORKS_STEP = REPO_ROOT.parent / "dronehacks-course-v3.step"


def _features(data: dict) -> dict:
    return {f["name"]: f for f in data["features"]}


@pytest.fixture(scope="module")
def solidworks() -> dict:
    if not SOLIDWORKS_STEP.is_file():
        pytest.skip("the repo-root STEP is not present (installed copy)")
    data = extract_map(SOLIDWORKS_STEP)
    if data["source"]["sha256"] != "2194664e852e341f86fb63a19ecc4f8f0ef34da6a51867e49beb3157cb90b876":
        pytest.skip("the repo-root STEP is not the 2026-11-07 SolidWorks export")
    return _features(data)


@pytest.fixture(scope="module")
def onshape_committed() -> dict:
    return _features(json.loads(ONSHAPE_MAP.read_text()))


def test_onshape_export_extracts_unchanged(onshape_committed) -> None:
    fresh = _features(extract_map(ONSHAPE_STEP))
    assert list(fresh) == list(onshape_committed)
    for name, f in fresh.items():
        assert f == onshape_committed[name], name


def test_solidworks_names_come_from_the_parts(solidworks) -> None:
    assert not [n for n in solidworks if n.lower().endswith(".step") or "<" in n or "^" in n]
    assert {"Part67 / Boss-Extrude1[1]", "Part67 / Boss-Extrude1[2]", "Barrier 1",
            "C2a chicane wall (window)", "Tube oval 58 x 72 x 60", "Launch mark"} <= set(solidworks)
    assert not [n for n in solidworks if "maze divider" in n or n == "Net roof"]


def test_solidworks_bspline_rocks_and_tube_match_the_onshape_parts(solidworks, onshape_committed) -> None:
    for name, old in onshape_committed.items():
        if old["kind"] not in ("rock", "tube", "wall", "support", "net"):
            continue
        if name not in solidworks:
            continue  # removed in the 2026-11-07 layout
        new = solidworks[name]
        assert new["kind"] == old["kind"], name
        for key in ("centre", "radius", "p0", "p1", "thickness"):
            if key in old:
                assert np.allclose(new[key], old[key], atol=1e-3), (name, key, new[key], old[key])
        for part in ("outer", "inner"):
            if part in old:
                for k in old[part]:
                    assert math.isclose(new[part][k], old[part][k], abs_tol=2e-3), (name, part, k)


def test_solidworks_new_walls(solidworks) -> None:
    block = solidworks["Part67 / Boss-Extrude1[1]"]
    assert block["kind"] == "block" and block["z"] == [0.0, 1.7272]
    xs, ys = zip(*block["footprint"])
    assert math.isclose(max(xs) - min(xs), 1.8, abs_tol=1e-4) and math.isclose(max(ys) - min(ys), 0.35, abs_tol=1e-4)
    wall = solidworks["Part67 / Boss-Extrude1[2]"]
    assert wall["kind"] == "wall" and math.isclose(wall["thickness"], 0.100, abs_tol=1e-4)
    assert wall["openings"] == []


def test_fit_conic_circle_ellipse_and_not_a_square() -> None:
    t = np.linspace(0.0, 2.0 * math.pi, 50, endpoint=False)
    up = np.array([0.0, 0.0, 1.0])
    circle = np.stack([1.0 + 0.3 * np.cos(t), -2.0 + 0.3 * np.sin(t), np.full_like(t, 0.5)], axis=1)
    c = _fit_conic(circle, up)
    assert c is not None and c.kind == "circle" and c.closed
    assert np.allclose(c.centre, [1.0, -2.0, 0.5], atol=1e-9) and math.isclose(c.a, 0.3, abs_tol=1e-9)
    side = np.array([1.0, 0.0, 0.0])
    ellipse = np.stack([np.full_like(t, 2.0), 0.7 * np.cos(t), 0.9 + 0.9 * np.sin(t)], axis=1)
    e = _fit_conic(ellipse, side)
    assert e is not None and e.kind == "ellipse"
    assert math.isclose(e.a, 0.9, abs_tol=1e-9) and math.isclose(e.b, 0.7, abs_tol=1e-9)
    assert abs(float(np.dot(e.x_dir, [0.0, 0.0, 1.0]))) > 0.999  # major axis vertical
    square = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]] * 3, dtype=float)
    assert _fit_conic(square, up) is None


@pytest.mark.parametrize("raw, clean", [
    ("Barrier 1.step", "Barrier 1"),
    ("Tube oval 58 x 72 x 60.STEP", "Tube oval 58 x 72 x 60"),
    ("Part67^dronehacks-course-v3.step", "Part67"),
    ("Bracket.SLDPRT", "Bracket"),
    ("M3 maze divider", "M3 maze divider"),
])
def test_clean_product_name(raw: str, clean: str) -> None:
    assert _clean_product_name(raw) == clean
