"""Focused regression tests for course/step_to_map.py.

Run from the repository root: python3 -m pytest course/tests
"""

from __future__ import annotations

import copy
import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

COURSE_DIR = Path(__file__).resolve().parents[1]
ROOT = COURSE_DIR.parent
SCRIPT = COURSE_DIR / "step_to_map.py"
REAL_STEP = ROOT / "dronehacks-course-v3.step"

sys.path.insert(0, str(COURSE_DIR))
import step_to_map as stm  # noqa: E402

# Whitespace-tolerant: SolidWorks writes "#1 = CARTESIAN_POINT ( 'NONE',  ( x, y, z ) ) ;".
POINT_RE = re.compile(r"#(\d+)\s*=\s*(CARTESIAN_POINT|DIRECTION)\s*\(\s*'([^']*)'\s*,\s*\(([^)]*)\)\s*\)\s*;")


def step_text() -> str:
    return REAL_STEP.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def entities():
    return stm.parse_entities(REAL_STEP)


@pytest.fixture(scope="module")
def base():
    return stm.generate_map(REAL_STEP, 0.0, None)


def transform(text: str, *, rotate: bool = False, dx: float = 0.0, dy: float = 0.0,
              replacements: tuple[tuple[str, str], ...] = (), only: set[int] | None = None, change=None) -> str:
    """Rewrite a STEP model by textual coordinate surgery (fail-loud helper).

    Rotates (90 degrees about z) points and directions alike, translates
    points only, or applies ``change(x, y, z)`` to the points numbered in
    ``only``.
    """
    for old, new in replacements:
        assert old in text, old
        text = text.replace(old, new)
    rewritten = 0

    def rewrite(match: re.Match) -> str:
        nonlocal rewritten
        number, kind, name, raw = match.groups()
        if only is not None and int(number) not in only:
            return match.group(0)
        values = [float(value) for value in raw.split(",")]
        if len(values) != 3:
            return match.group(0)
        x, y, z = values
        if rotate:
            x, y = -y, x  # rotation applies to points and directions alike
        if kind == "CARTESIAN_POINT":
            x, y = x + dx, y + dy  # translation moves geometry, not orientations
            if change is not None:
                x, y, z = change(x, y, z)
        rewritten += 1
        return f"#{number}={kind}('{name}',({x!r},{y!r},{z!r}));"

    text = POINT_RE.sub(rewrite, text)
    assert rewritten > 0 or not (rotate or dx or dy or change)
    return text


def part_points(entities: dict, prefix: str) -> set[int]:
    """Numbers of every Cartesian point of the solids named ``prefix...``."""
    solids = stm.part_solids(entities)
    found = {
        number
        for name, brep in solids.items() if name.startswith(prefix)
        for number in stm.reachable_entities(brep, entities) if entities[number].type == "CARTESIAN_POINT"
    }
    assert found, prefix
    return found


def write_model(directory: Path, text: str) -> Path:
    path = directory / "model.step"
    path.write_text(text, encoding="utf-8")
    return path


def generate(step: Path, heading: float = 0.0, clearance: float | None = None) -> dict:
    return stm.generate_map(step, heading, clearance)


def goal(data: dict, name: str) -> dict:
    return {item["name"]: item for item in data["goals"]}[name]


def test_real_model_generates_all_contract_goals(base):
    data = base
    assert [item["name"] for item in data["goals"]] == ["tunnel", "window", "pad", "launch"]
    assert len(data["boulders"]) == 5

    pad = goal(data, "pad")
    assert pad["centre"] == pytest.approx([-2.04216, -0.79248], abs=1e-4)
    assert pad["params"]["radius"] == pytest.approx(0.762, abs=1e-6)

    launch = goal(data, "launch")
    assert launch["centre"] == pytest.approx([-2.5146, 0.9144], abs=1e-4)
    assert launch["params"]["heading_deg"] == 0.0

    tunnel = goal(data, "tunnel")
    assert tunnel["params"]["width"] == pytest.approx(1.4732, abs=1e-6)
    assert tunnel["params"]["height"] == pytest.approx(1.8288, abs=1e-6)

    window = goal(data, "window")
    assert window["params"]["sill_height"] == pytest.approx(0.3048, abs=1e-6)
    assert window["params"]["head_height"] == pytest.approx(1.2192, abs=1e-6)
    assert window["params"]["side"] == pytest.approx(0.9144, abs=1e-6)
    assert window["params"]["clearance_radius"] is None
    assert data["_segment_names"][window["params"]["wall_segment_index"]] == "C2a chicane wall (window)"


def test_traversal_axes_depart_toward_the_pad(base):
    pad = goal(base, "pad")["centre"]
    window = goal(base, "window")
    tunnel = goal(base, "tunnel")

    # window: approach side away from the pad, departure side toward it
    start, end, centre = window["params"]["axis_start"], window["params"]["axis_end"], window["centre"]
    to_pad = (pad[0] - centre[0], pad[1] - centre[1])
    assert (end[0] - start[0]) * to_pad[0] + (end[1] - start[1]) * to_pad[1] > 0.0

    # tunnel: the exit is the end nearer the pad, the entrance the farther one
    start, end, centre = tunnel["params"]["axis_start"], tunnel["params"]["axis_end"], tunnel["centre"]
    entry_reach = (start[0] - centre[0]) * (pad[0] - centre[0]) + (start[1] - centre[1]) * (pad[1] - centre[1])
    exit_reach = (end[0] - centre[0]) * (pad[0] - centre[0]) + (end[1] - centre[1]) * (pad[1] - centre[1])
    assert exit_reach > entry_reach


def test_self_checks_pass_on_the_real_model(base):
    assert stm.structural_check(base) == []
    assert stm.nominal_check(base) == []


def test_nominal_wall_thickness_is_one_of_the_contract_values(base):
    walls = [segment for segment, kind in zip(base["segments"], base["_segment_kinds"]) if kind == "wall"]
    for thickness, ok in ((0.102, True), (0.1, True), (0.35, True), (0.1016, True), (0.2, False), (0.104, False)):
        data = copy.deepcopy(base)
        index = base["segments"].index(walls[0])
        data["segments"][index]["thickness"] = thickness
        failures = [failure for failure in stm.nominal_check(data) if "wall thickness" in failure]
        assert (failures == []) is ok, (thickness, failures)


def test_render_marks_unreviewed_clearance_and_records_the_command(base):
    text = stm.render_yaml(base, "python3 course/step_to_map.py --launch-heading 0")
    assert "clearance_radius: null" in text
    assert "UNREVIEWED" in text
    assert "# Regenerate: python3 course/step_to_map.py --launch-heading 0" in text

    reviewed = stm.render_yaml(generate(REAL_STEP, clearance=0.31), "cmd")
    assert "clearance_radius: 0.31" in reviewed


def test_translation_invariance(tmp_path):
    before_map = generate(REAL_STEP, heading=7.5)
    shifted = generate(write_model(tmp_path, transform(step_text(), dx=1.25, dy=-0.5)), heading=7.5)

    assert shifted["_segment_names"] == before_map["_segment_names"]
    for before, after in zip(before_map["segments"], shifted["segments"]):
        for key in ("start", "end"):
            assert after[key][0] == pytest.approx(before[key][0] + 1.25, abs=1e-5)
            assert after[key][1] == pytest.approx(before[key][1] - 0.5, abs=1e-5)
        assert after["thickness"] == pytest.approx(before["thickness"], abs=1e-6)
    for before, after in zip(before_map["boulders"], shifted["boulders"]):
        assert after["centre"] == pytest.approx([before["centre"][0] + 1.25, before["centre"][1] - 0.5], abs=1e-5)
        assert after["radius"] == before["radius"]
    for before, after in zip(before_map["goals"], shifted["goals"]):
        assert after["centre"][0] == pytest.approx(before["centre"][0] + 1.25, abs=1e-5)
        assert after["centre"][1] == pytest.approx(before["centre"][1] - 0.5, abs=1e-5)
        for key, value in before["params"].items():
            if isinstance(value, list):
                assert after["params"][key][0] == pytest.approx(value[0] + 1.25, abs=1e-5)
                assert after["params"][key][1] == pytest.approx(value[1] - 0.5, abs=1e-5)
            else:
                assert after["params"][key] == value  # dimensions and heading are layout-invariant


def test_rotation_invariance(tmp_path, base):
    rotated = generate(write_model(tmp_path, transform(step_text(), rotate=True)))

    def rotate2(p):
        return [-p[1], p[0]]

    tunnel_before = goal(base, "tunnel")
    tunnel_after = goal(rotated, "tunnel")
    assert tunnel_after["params"]["axis_start"] == pytest.approx(rotate2(tunnel_before["params"]["axis_start"]), abs=1e-5)
    assert tunnel_after["params"]["axis_end"] == pytest.approx(rotate2(tunnel_before["params"]["axis_end"]), abs=1e-5)
    assert tunnel_after["params"]["width"] == tunnel_before["params"]["width"]
    assert tunnel_after["params"]["height"] == tunnel_before["params"]["height"]

    window_before = goal(base, "window")
    window_after = goal(rotated, "window")
    for key in ("side", "sill_height", "head_height"):
        assert window_after["params"][key] == window_before["params"][key]
    pad = goal(rotated, "pad")["centre"]
    centre = window_after["centre"]
    to_pad = (pad[0] - centre[0], pad[1] - centre[1])
    axis = (window_after["params"]["axis_end"][0] - window_after["params"]["axis_start"][0],
            window_after["params"]["axis_end"][1] - window_after["params"]["axis_start"][1])
    assert axis[0] * to_pad[0] + axis[1] * to_pad[1] > 0.0  # still departs toward the pad

    # every wall and net footprint turns with the model (its ends may swap)
    assert rotated["_segment_names"] == base["_segment_names"]
    for before, after in zip(base["segments"], rotated["segments"]):
        start, end = rotate2(before["start"]), rotate2(before["end"])
        direct = math.dist(after["start"], start) + math.dist(after["end"], end)
        swapped = math.dist(after["start"], end) + math.dist(after["end"], start)
        assert min(direct, swapped) < 2e-5, (before, after)
        assert after["thickness"] == pytest.approx(before["thickness"], abs=1e-6)
    for before, after in zip(base["boulders"], rotated["boulders"]):
        assert after["centre"] == pytest.approx(rotate2(before["centre"]), abs=1e-5)


def test_missing_window_part_fails_closed(tmp_path):
    # the window wall's shape representation is gone: a dangling reference
    text = re.sub(r"#\d+\s*=\s*SHAPE_REPRESENTATION\s*\(\s*'C2a[^']*'[^;]*;", "", step_text())
    assert text != step_text()
    with pytest.raises(stm.StepError, match="referenced but undefined"):
        generate(write_model(tmp_path, text))

    # no part is the window wall any more
    text = transform(step_text(), replacements=(("C2a chicane wall (window)", "C2a chicane wall"),))
    with pytest.raises(stm.StepError, match="missing the part that provides the window wall"):
        generate(write_model(tmp_path, text))


def test_window_wall_must_be_at_the_slice(tmp_path, entities):
    text = transform(step_text(), only=part_points(entities, "C2a"), change=lambda x, y, z: (x, y, z * 0.2))
    with pytest.raises(stm.StepError, match="is not a wall at the map slice height"):
        generate(write_model(tmp_path, text))


def test_parts_below_the_slice_are_not_segments(tmp_path, entities, base):
    # The map is the model sliced at the flight height: a part entirely below
    # it is no obstacle there (and only floor-standing vertical extrusions
    # that reach it become segments).
    text = transform(step_text(), only=part_points(entities, "A1 approach wall"), change=lambda x, y, z: (x, y, z * 0.2))
    data = generate(write_model(tmp_path, text))
    assert "A1 approach wall" in base["_segment_names"]
    assert "A1 approach wall" not in data["_segment_names"]
    assert len(data["segments"]) == len(base["segments"]) - 1


def test_goal_parts_are_found_by_role_not_by_label(tmp_path):
    # a relabelled tube still provides the tunnel; nothing is keyed to the label
    text = transform(step_text(), replacements=(("Tube oval 58 x 72 x 60", "Tube oval 61 x 73 x 63"),))
    data = generate(write_model(tmp_path, text))
    assert goal(data, "tunnel")["params"]["width"] == pytest.approx(1.4732, abs=1e-6)
    assert goal(data, "tunnel")["params"]["height"] == pytest.approx(1.8288, abs=1e-6)


def test_tube_without_end_faces_fails_closed(tmp_path, entities):
    solids = stm.part_solids(entities)
    tube = entities[solids[next(name for name in solids if name.startswith("Tube oval"))]]
    shell = entities[int(tube.args[1])]
    faces = [int(face) for face in shell.args[1]]
    planar = [face for face in faces if entities[int(entities[face].args[2])].type == "PLANE"]
    assert len(planar) == 2
    for removed, message in ((planar, "no planar end faces"), (planar[:1], "exactly two planar end faces")):
        kept = ",".join(f"#{face}" for face in faces if face not in removed)
        text, count = re.subn(rf"#{shell.id}\s*=\s*CLOSED_SHELL\s*\([^;]*;", f"#{shell.id}=CLOSED_SHELL('',({kept}));", step_text())
        assert count == 1
        with pytest.raises(stm.StepError, match=message):
            generate(write_model(tmp_path, text))


def test_tube_cross_section_must_be_an_upright_ellipse(tmp_path, entities):
    # a sheared tube's end faces are tilted ellipses: unsupported, not guessed
    text = transform(step_text(), only=part_points(entities, "Tube oval"), change=lambda x, y, z: (x + 0.2 * z, y, z))
    with pytest.raises(stm.StepError, match="not an ellipse with horizontal and vertical axes"):
        generate(write_model(tmp_path, text))


def test_bspline_boulder_outline_must_be_a_circle(tmp_path, entities, base):
    centre = base["boulders"][2]["centre"]  # Boulder 3, in natural name order
    text = transform(step_text(), only=part_points(entities, "Boulder 3"),
                     change=lambda x, y, z: (centre[0] + 1.1 * (x - centre[0]), y, z))
    with pytest.raises(stm.StepError, match="Boulder 3 planar B-spline outline is not a circle"):
        generate(write_model(tmp_path, text))


def test_ambiguous_window_departure_fails_closed(tmp_path, entities, base):
    # move the pad onto the window centre: neither side is the landing area
    pad, window = goal(base, "pad")["centre"], goal(base, "window")["centre"]
    shift = (window[0] - pad[0], window[1] - pad[1])
    text = transform(step_text(), only=part_points(entities, "Landing pad 5 ft"),
                     change=lambda x, y, z: (x + shift[0], y + shift[1], z))
    with pytest.raises(stm.StepError, match="window traversal direction is ambiguous"):
        generate(write_model(tmp_path, text))


def test_part_placements_must_be_the_identity(tmp_path, entities):
    operator = next(entity for entity in entities.values() if entity.type == "ITEM_DEFINED_TRANSFORMATION")
    location = int(entities[int(operator.args[2])].args[1])
    text = transform(step_text(), only={location}, change=lambda x, y, z: (x + 1.0, y, z))
    with pytest.raises(stm.StepError, match="not the identity"):
        generate(write_model(tmp_path, text))


def test_launch_heading_is_required(tmp_path):
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--step", str(REAL_STEP), "--output", str(tmp_path / "map.yaml")],
        capture_output=True, text=True,
    )
    assert result.returncode == 2
    assert not (tmp_path / "map.yaml").exists()


def test_failing_self_check_preserves_the_previous_map(tmp_path, entities, base):
    # fat boulders: every boulder scaled about its own centre to a 0.5 m radius
    text = step_text()
    for index, name in enumerate(sorted((n for n in stm.part_solids(entities) if n.startswith("Boulder ")), key=stm.natural_key)):
        cx, cy = base["boulders"][index]["centre"]
        scale = 0.5 / base["boulders"][index]["radius"]
        text = transform(text, only=part_points(entities, name),
                         change=lambda x, y, z, cx=cx, cy=cy, scale=scale: (cx + scale * (x - cx), cy + scale * (y - cy), z))
    model = write_model(tmp_path, text)
    output = tmp_path / "map.yaml"
    output.write_text("sentinel", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--step", str(model), "--output", str(output),
         "--launch-heading", "0", "--self-check"],
        capture_output=True, text=True,
    )
    assert result.returncode == 1
    assert "output not written" in result.stderr
    assert "boulder radius is 0.5" in result.stderr
    assert output.read_text(encoding="utf-8") == "sentinel"


def test_passing_self_check_reports_measured_values(tmp_path):
    output = tmp_path / "map.yaml"
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--step", str(REAL_STEP), "--output", str(output),
         "--launch-heading", "0", "--self-check"],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "SELF-CHECK: PASS" in result.stdout
    assert "6.098540" in result.stdout  # measured net breadth, not the nominal constant
    assert "clearance_radius null" in result.stdout
    assert "wall thicknesses: [0.1, 0.1016, 0.35]" in result.stdout  # measured wall thicknesses
    assert "0.2286" in result.stdout  # measured boulder radius
    assert output.exists()


def test_window_opening_rejects_ambiguous_levels():
    wall = [(0, 0, 0), (10, 0, 0), (10, 1, 0), (0, 1, 0),  # footprint boundary
            (5, 0.5, 0), (5, 0.5, 2.0),                    # interior floor and top
            (4, 0.5, 0.5), (6, 0.5, 0.5), (5, 0.5, 1.5), (4, 0.5, 1.5), (6, 0.5, 1.5)]
    sill, head, edges = stm.window_opening("wall", wall)
    assert (sill, head) == (0.5, 1.5)
    assert len(edges) == 5

    with pytest.raises(stm.StepError):
        stm.window_opening("wall", wall + [(5, 0.5, 1.0)])  # a third interior level is ambiguous


def test_disc_requires_consistent_circle_geometry():
    with pytest.raises(stm.StepError):
        stm.disc_geometry("pad", [])
    with pytest.raises(stm.StepError):
        stm.disc_geometry("pad", [{"centre": (0.0, 0.0, 0.0), "radius": 0.5}, {"centre": (1.0, 0.0, 0.0), "radius": 0.5}])
    with pytest.raises(stm.StepError):
        stm.disc_geometry("pad", [{"centre": (0.0, 0.0, 0.0), "radius": 0.5}, {"centre": (0.0, 0.0, 0.0), "radius": 0.6}])
    centre, radius = stm.disc_geometry("pad", [{"centre": (1.0, 2.0, 0.0), "radius": 0.5}, {"centre": (1.0, 2.0, 0.01), "radius": 0.5}])
    assert centre == (1.0, 2.0)
    assert radius == 0.5


def test_structural_check_rejects_a_degenerate_window_axis(base):
    broken = copy.deepcopy(base)
    params = broken["goals"][1]["params"]
    params["axis_start"] = list(params["axis_end"])
    failures = stm.structural_check(broken)
    assert any("degenerate" in failure for failure in failures)


def test_heading_flows_from_the_argument():
    assert generate(REAL_STEP, heading=-12.5)["goals"][3]["params"]["heading_deg"] == -12.5


# ---------------------------------------------------------------- Part 21 and B-splines

W = math.sqrt(0.5)
CIRCLE_SNIPPET = """ISO-10303-21;
HEADER;
FILE_NAME('snippet','2026-11-07',(''),(''),'','','');
ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));
/* a rational quadratic B-spline circle, centre (1, 2), radius 0.5 */
#2=CARTESIAN_POINT('',(1.5,2.,0.));
#3=CARTESIAN_POINT('',(1.5,2.5,0.));
#4=CARTESIAN_POINT('',(1.,2.5,0.));
#5=CARTESIAN_POINT('',(0.5,2.5,0.));
#6=CARTESIAN_POINT('',(0.5,2.,0.));
#7=CARTESIAN_POINT('',(0.5,1.5,0.));
#8=CARTESIAN_POINT('',(1.,1.5,0.));
#9=CARTESIAN_POINT('',(1.5,1.5,0.));
#10=CARTESIAN_POINT('',(1.5,2.,0.));
#11=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#2,#3,#4,#5,#6,#7,#8,#9,#10),.CIRCULAR_ARC.,.T.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,{w},1.,{w},1.,{w},1.,{w},1.))REPRESENTATION_ITEM(''));
#12=B_SPLINE_CURVE_WITH_KNOTS('',1,(#2,#4),.POLYLINE_FORM.,.F.,.F.,(2,2),(0.,1.),.UNSPECIFIED.);
#13=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#2,#3,#4),.UNSPECIFIED.,.F.,.F.)BEZIER_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM(''));
#14=PRODUCT('it''s /* not a comment */; really','x','',());
ENDSEC;
END-ISO-10303-21;
""".replace("{w}", repr(W))


def test_onshape_style_snippet_and_rational_bspline(tmp_path):
    path = tmp_path / "snippet.step"
    path.write_text(CIRCLE_SNIPPET, encoding="utf-8")
    entities = stm.parse_entities(path)
    assert entities[14].args[0] == "it's /* not a comment */; really"
    assert entities[11].type is None and "RATIONAL_B_SPLINE_CURVE" in entities[11].parts

    samples = stm.sample_bspline(*stm.bspline_curve(entities, entities[11]))
    assert len(samples) > 8
    for x, y, z in samples:
        assert math.hypot(x - 1.0, y - 2.0) == pytest.approx(0.5, abs=1e-12)
        assert z == 0.0
    centre, radius, deviation = stm.fit_circle([(x, y) for x, y, _ in samples], "circle")
    assert centre == pytest.approx((1.0, 2.0), abs=1e-12)
    assert radius == pytest.approx(0.5, abs=1e-12)
    assert deviation < 1e-12

    degree, control, knots = stm.bspline_curve(entities, entities[12])
    assert stm.de_boor(degree, control, knots, 0.5) == pytest.approx((1.25, 2.25, 0.0), abs=1e-15)

    with pytest.raises(stm.StepError, match="unsupported B-spline curve"):
        stm.bspline_curve(entities, entities[13])


def test_spaced_snippet_parses_like_the_compact_one(tmp_path):
    def space(text: str) -> str:
        pieces = text.split("'")
        pieces[::2] = [re.sub(r"([(),=;])", r" \1 ", piece) for piece in pieces[::2]]
        return "'".join(pieces).replace("\n", "\r\n")

    compact, spaced = tmp_path / "compact.step", tmp_path / "spaced.step"
    compact.write_text(CIRCLE_SNIPPET, encoding="utf-8")
    spaced.write_bytes(space(CIRCLE_SNIPPET).encode("utf-8"))
    first, second = stm.parse_entities(compact), stm.parse_entities(spaced)
    assert {n: (e.type, e.args, e.parts) for n, e in first.items()} == {n: (e.type, e.args, e.parts) for n, e in second.items()}


def test_malformed_step_text_fails_closed(tmp_path):
    for broken, message in (
        (CIRCLE_SNIPPET.replace("(1.5,2.5,0.)", "(1.5,2.5,nan)"), "cannot read STEP data"),
        (CIRCLE_SNIPPET.replace("#3=CARTESIAN_POINT", "#2=CARTESIAN_POINT"), "defined twice"),
        (CIRCLE_SNIPPET.replace("SI_UNIT($,.METRE.)", "SI_UNIT(.MILLI.,.METRE.)"), "not metres"),
        (CIRCLE_SNIPPET.replace("#14=PRODUCT('it''s /* not a comment */; really','x','',());", "#14=PRODUCT('x',"), "STEP"),
    ):
        path = tmp_path / "broken.step"
        path.write_text(broken, encoding="utf-8")
        with pytest.raises(stm.StepError, match=message):
            stm.parse_entities(path)


def test_ellipse_fit_is_exact_for_upright_ellipses_only():
    upright = [(2.0 + 0.7 * math.cos(t), -1.0 + 0.9 * math.sin(t)) for t in (k * math.pi / 16 for k in range(32))]
    centre, semi_u, semi_v, deviation = stm.fit_axis_aligned_ellipse(upright, "ellipse")
    assert centre == pytest.approx((2.0, -1.0), abs=1e-12)
    assert (semi_u, semi_v) == pytest.approx((0.7, 0.9), abs=1e-12)
    assert deviation < 1e-12

    tilt = math.radians(10.0)
    tilted = [(u * math.cos(tilt) - v * math.sin(tilt), u * math.sin(tilt) + v * math.cos(tilt))
              for u, v in ((0.7 * math.cos(t), 0.9 * math.sin(t)) for t in (k * math.pi / 16 for k in range(32)))]
    assert stm.fit_axis_aligned_ellipse(tilted, "ellipse")[3] > stm.FIT_TOLERANCE_M
