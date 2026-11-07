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


def step_text() -> str:
    return REAL_STEP.read_text(encoding="utf-8")


def transform(text: str, *, rotate: bool = False, dx: float = 0.0, dy: float = 0.0, replacements: tuple[tuple[str, str], ...] = ()) -> str:
    """Rewrite a STEP model by textual coordinate surgery (fail-loud helper)."""
    for old, new in replacements:
        text = text.replace(old, new)

    def rewrite(match: re.Match) -> str:
        kind, name, raw = match.group(1), match.group(2), match.group(3)
        values = [float(value) for value in raw.split(",")]
        if len(values) < 2:
            return match.group(0)
        x, y = values[0], values[1]
        if rotate:
            x, y = -y, x
        rendered = [repr(x + dx), repr(y + dy)] + [repr(value) for value in values[2:]]
        return f"{kind}('{name}',({','.join(rendered)}))"

    return re.sub(r"(CARTESIAN_POINT|DIRECTION)\('([^']*)',\(([^)]*)\)\)", rewrite, text)


def write_model(directory: Path, text: str) -> Path:
    path = directory / "model.step"
    path.write_text(text, encoding="utf-8")
    return path


def generate(step: Path, heading: float = 0.0, clearance: float | None = None) -> dict:
    return stm.generate_map(step, heading, clearance)


def goal(data: dict, name: str) -> dict:
    return {item["name"]: item for item in data["goals"]}[name]


def test_real_model_generates_all_contract_goals():
    data = generate(REAL_STEP)
    assert [item["name"] for item in data["goals"]] == ["tunnel", "window", "pad", "launch"]
    assert len(data["boulders"]) == 5

    pad = goal(data, "pad")
    assert pad["centre"] == pytest.approx([-2.04216, -0.79248], abs=1e-4)
    assert pad["params"]["radius"] == pytest.approx(0.762, abs=1e-6)

    launch = goal(data, "launch")
    assert launch["centre"] == pytest.approx([-2.5146, 0.9144], abs=1e-4)
    assert launch["params"]["heading_deg"] == 0.0

    tunnel = goal(data, "tunnel")
    assert tunnel["params"]["width"] == pytest.approx(1.4732, abs=1e-3)
    assert tunnel["params"]["height"] == pytest.approx(1.8288, abs=1e-3)

    window = goal(data, "window")
    assert window["params"]["sill_height"] == pytest.approx(0.3048, abs=1e-6)
    assert window["params"]["head_height"] == pytest.approx(1.2192, abs=1e-6)
    assert window["params"]["side"] == pytest.approx(0.920, abs=0.01)
    assert window["params"]["clearance_radius"] is None


def test_traversal_axes_depart_toward_the_pad():
    data = generate(REAL_STEP)
    pad = goal(data, "pad")["centre"]
    window = goal(data, "window")
    tunnel = goal(data, "tunnel")

    # window: approach side away from the pad, departure side toward it
    start, end, centre = window["params"]["axis_start"], window["params"]["axis_end"], window["centre"]
    to_pad = (pad[0] - centre[0], pad[1] - centre[1])
    assert (end[0] - start[0]) * to_pad[0] + (end[1] - start[1]) * to_pad[1] > 0.0

    # tunnel: the exit is the end nearer the pad, the entrance the farther one
    start, end, centre = tunnel["params"]["axis_start"], tunnel["params"]["axis_end"], tunnel["centre"]
    entry_reach = (start[0] - centre[0]) * (pad[0] - centre[0]) + (start[1] - centre[1]) * (pad[1] - centre[1])
    exit_reach = (end[0] - centre[0]) * (pad[0] - centre[0]) + (end[1] - centre[1]) * (pad[1] - centre[1])
    assert exit_reach > entry_reach


def test_self_checks_pass_on_the_real_model():
    data = generate(REAL_STEP)
    assert stm.structural_check(data) == []
    assert stm.nominal_check(data) == []


def test_render_marks_unreviewed_clearance_and_records_the_command():
    data = generate(REAL_STEP, heading=0.0)
    text = stm.render_yaml(data, "python3 course/step_to_map.py --launch-heading 0")
    assert "clearance_radius: null" in text
    assert "UNREVIEWED" in text
    assert "# Regenerate: python3 course/step_to_map.py --launch-heading 0" in text

    reviewed = stm.render_yaml(generate(REAL_STEP, clearance=0.31), "cmd")
    assert "clearance_radius: 0.31" in reviewed


def test_translation_invariance(tmp_path):
    base = generate(REAL_STEP, heading=7.5)
    shifted = generate(write_model(tmp_path, transform(step_text(), dx=1.25, dy=-0.5)), heading=7.5)

    for before, after in zip(base["segments"], shifted["segments"]):
        for key in ("start", "end"):
            assert after[key][0] == pytest.approx(before[key][0] + 1.25, abs=1e-5)
            assert after[key][1] == pytest.approx(before[key][1] - 0.5, abs=1e-5)
    for before, after in zip(base["goals"], shifted["goals"]):
        assert after["centre"][0] == pytest.approx(before["centre"][0] + 1.25, abs=1e-5)
        assert after["centre"][1] == pytest.approx(before["centre"][1] - 0.5, abs=1e-5)
        for key, value in before["params"].items():
            if isinstance(value, list):
                assert after["params"][key][0] == pytest.approx(value[0] + 1.25, abs=1e-5)
                assert after["params"][key][1] == pytest.approx(value[1] - 0.5, abs=1e-5)
            else:
                assert after["params"][key] == value  # dimensions and heading are layout-invariant


def test_rotation_invariance(tmp_path):
    base = generate(REAL_STEP)
    rotated = generate(write_model(tmp_path, transform(step_text(), rotate=True)))

    def rotate2(point):
        return [-point[1], point[0]]

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


def test_missing_window_part_fails_closed(tmp_path):
    text = re.sub(r"#\d+\s*=\s*SHAPE_REPRESENTATION\('C2a[^']*'[^;]*;", "", step_text())
    with pytest.raises(stm.StepError):
        generate(write_model(tmp_path, text))


def test_goal_parts_are_found_by_role_not_by_label(tmp_path):
    # a relabelled tube still provides the tunnel; nothing is keyed to the label
    text = transform(step_text(), replacements=(("Tube oval 58 x 72 x 60", "Tube oval 61 x 73 x 63"),))
    data = generate(write_model(tmp_path, text))
    assert goal(data, "tunnel")["params"]["width"] == pytest.approx(1.4732, abs=1e-3)
    assert goal(data, "tunnel")["params"]["height"] == pytest.approx(1.8288, abs=1e-3)


def test_tube_without_ellipses_fails_closed(tmp_path):
    text = re.sub(r"#\d+\s*=\s*ELLIPSE\([^;]*;", "", step_text())
    with pytest.raises(stm.StepError):
        generate(write_model(tmp_path, text))


def test_ambiguous_window_departure_fails_closed(tmp_path):
    # move the pad onto the window centre: neither side is the landing area
    text = transform(step_text(), replacements=(("-2.04216,-0.79248", "-0.470938,-2.271897"),))
    with pytest.raises(stm.StepError):
        generate(write_model(tmp_path, text))


def test_launch_heading_is_required(tmp_path):
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--step", str(REAL_STEP), "--output", str(tmp_path / "map.yaml")],
        capture_output=True, text=True,
    )
    assert result.returncode == 2
    assert not (tmp_path / "map.yaml").exists()


def test_failing_self_check_preserves_the_previous_map(tmp_path):
    fat_boulders = transform(step_text(), replacements=((",0.2286)", ",0.5)"),))
    model = write_model(tmp_path, fat_boulders)
    output = tmp_path / "map.yaml"
    output.write_text("sentinel", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--step", str(model), "--output", str(output),
         "--launch-heading", "0", "--self-check"],
        capture_output=True, text=True,
    )
    assert result.returncode == 1
    assert "output not written" in result.stderr
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
    assert "0.1016" in result.stdout  # measured wall thickness
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
    centre, radius = stm.disc_geometry("pad", [{"centre": (1.0, 2.0, 0.0), "radius": 0.5}, {"centre": (1.0, 2.0, 0.01), "radius": 0.5}])
    assert centre == (1.0, 2.0)
    assert radius == 0.5


def test_structural_check_rejects_a_degenerate_window_axis():
    data = generate(REAL_STEP)
    broken = copy.deepcopy(data)
    params = broken["goals"][1]["params"]
    params["axis_start"] = list(params["axis_end"])
    failures = stm.structural_check(broken)
    assert any("degenerate" in failure for failure in failures)


def test_heading_flows_from_the_argument():
    assert generate(REAL_STEP, heading=-12.5)["goals"][3]["params"]["heading_deg"] == -12.5
