#!/usr/bin/env python3
"""Generate the planner course map from the organizer AP214 STEP model.

This intentionally uses only the Python standard library so it can run on a
bare laptop.  It supports the small, explicit subset of STEP used by the
course model: named shape representations, their BREP relationships,
Cartesian points, and circles.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STEP = ROOT / "dronehacks-course-v3.step"
DEFAULT_OUTPUT = ROOT / "contracts/course_v3_layout.yaml"
SLICE_HEIGHT = 0.8
WINDOW_NAME = "C2a chicane wall (window)"


class StepError(ValueError):
    """The STEP file does not contain the expected usable geometry."""


def parse_entities(path: Path) -> dict[int, str]:
    text = path.read_text(encoding="utf-8")
    entities = {
        int(number): body.strip()
        for number, body in re.findall(r"#(\d+)\s*=\s*(.*?);", text, re.DOTALL)
    }
    if not entities:
        raise StepError(f"no STEP entities found in {path}")
    if "SI_UNIT($,.METRE.)" not in text:
        raise StepError("STEP length unit is not metres")
    return entities


def refs(body: str) -> list[int]:
    return [int(value) for value in re.findall(r"#(\d+)", body)]


def named_brep_roots(entities: dict[int, str]) -> dict[str, int]:
    relationships: dict[int, int] = {}
    for body in entities.values():
        if body.startswith("SHAPE_REPRESENTATION_RELATIONSHIP"):
            linked = refs(body)
            if len(linked) >= 2:
                relationships[linked[0]] = linked[1]

    result: dict[str, int] = {}
    for number, body in entities.items():
        match = re.match(r"SHAPE_REPRESENTATION\('([^']*)'", body)
        if match and number in relationships:
            result[match.group(1)] = relationships[number]
    return result


def reachable_entities(root: int, entities: dict[int, str]) -> set[int]:
    seen: set[int] = set()
    pending = [root]
    while pending:
        number = pending.pop()
        if number in seen:
            continue
        body = entities.get(number)
        if body is None:
            raise StepError(f"entity #{number} is referenced but undefined")
        seen.add(number)
        pending.extend(reference for reference in refs(body) if reference not in seen)
    return seen


def part_geometry(root: int, entities: dict[int, str]) -> tuple[list[tuple[float, float, float]], list[float]]:
    points: list[tuple[float, float, float]] = []
    radii: list[float] = []
    for number in reachable_entities(root, entities):
        body = entities[number]
        point = re.match(r"CARTESIAN_POINT\('[^']*',\(([^)]*)\)\)", body)
        if point:
            values = tuple(float(value) for value in point.group(1).split(","))
            if len(values) == 3:
                points.append(values)
        circle = re.match(r"CIRCLE\('[^']*',#\d+,([^)]*)\)", body)
        if circle:
            radii.append(float(circle.group(1)))
    if not points:
        raise StepError(f"BREP #{root} contains no Cartesian points")
    return points, radii


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) < 2:
        raise StepError("footprint has fewer than two distinct points")

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-10:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-10:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def segment_from_points(points: list[tuple[float, float, float]]) -> dict[str, object]:
    hull = convex_hull([(point[0], point[1]) for point in points])
    edges = [
        (math.dist(hull[index], hull[(index + 1) % len(hull)]), hull[index], hull[(index + 1) % len(hull)])
        for index in range(len(hull))
    ]
    length, edge_start, edge_end = max(edges)
    if length <= 0.0:
        raise StepError("segment footprint has zero length")
    along = ((edge_end[0] - edge_start[0]) / length, (edge_end[1] - edge_start[1]) / length)
    normal = (-along[1], along[0])
    along_values = [point[0] * along[0] + point[1] * along[1] for point in hull]
    normal_values = [point[0] * normal[0] + point[1] * normal[1] for point in hull]
    centre_normal = (min(normal_values) + max(normal_values)) / 2.0
    start = (min(along_values) * along[0] + centre_normal * normal[0], min(along_values) * along[1] + centre_normal * normal[1])
    end = (max(along_values) * along[0] + centre_normal * normal[0], max(along_values) * along[1] + centre_normal * normal[1])
    return {"start": start, "end": end, "thickness": max(normal_values) - min(normal_values)}


def mean_xy(points: list[tuple[float, float, float]]) -> tuple[float, float]:
    return (sum(point[0] for point in points) / len(points), sum(point[1] for point in points) / len(points))


def round_value(value: float) -> float:
    rounded = round(value, 6)
    return 0.0 if abs(rounded) < 5e-7 else rounded


def rounded_point(point: tuple[float, float]) -> list[float]:
    return [round_value(point[0]), round_value(point[1])]


def is_wall_name(name: str) -> bool:
    return name.startswith("Barrier") or bool(re.match(r"[MCA]\d", name))


def is_segment_name(name: str) -> bool:
    return is_wall_name(name) or name.startswith("Net side")


def generate_map(step_path: Path, clearance: float) -> dict[str, object]:
    if not math.isfinite(clearance) or clearance <= 0.0:
        raise StepError("--window-clearance must be a positive finite value from a reviewed human budget")

    entities = parse_entities(step_path)
    roots = named_brep_roots(entities)
    part_data = {name: part_geometry(root, entities) for name, root in roots.items()}

    selected_names = [name for name in roots if is_segment_name(name)]
    if WINDOW_NAME not in selected_names:
        raise StepError(f"missing required part {WINDOW_NAME!r}")

    segments: list[dict[str, object]] = []
    segment_names: list[str] = []
    raw_segments: dict[str, dict[str, object]] = {}
    for name in selected_names:
        segment = segment_from_points(part_data[name][0])
        raw_segments[name] = segment
        segments.append({
            "start": rounded_point(segment["start"]),
            "end": rounded_point(segment["end"]),
            "thickness": round_value(segment["thickness"]),
        })
        segment_names.append(name)

    boulders: list[dict[str, object]] = []
    for name in sorted((name for name in roots if name.startswith("Boulder ")), key=lambda item: int(item.split()[-1])):
        points, circles = part_data[name]
        if not circles:
            raise StepError(f"{name} has no circle radius")
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        centre = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)
        boulders.append({"centre": rounded_point(centre), "radius": round_value(max(circles))})

    tube_points = part_data["Tube oval 58 x 72 x 60"][0]
    tube_xs = [point[0] for point in tube_points]
    tube_ys = [point[1] for point in tube_points]
    tube_centre = ((min(tube_xs) + max(tube_xs)) / 2.0, (min(tube_ys) + max(tube_ys)) / 2.0)
    # The course direction enters the north end and exits the south end.
    tube_axis_start = (tube_centre[0], max(tube_ys))
    tube_axis_end = (tube_centre[0], min(tube_ys))

    window_points = part_data[WINDOW_NAME][0]
    opening_edges = [point for point in window_points if math.isclose(point[2], 0.3048, abs_tol=1e-6) or math.isclose(point[2], 1.2192, abs_tol=1e-6)]
    if not opening_edges:
        raise StepError("window opening edge geometry was not found")
    window_centre = mean_xy(opening_edges)
    wall = raw_segments[WINDOW_NAME]
    wall_start = wall["start"]
    wall_end = wall["end"]
    wall_length = math.dist(wall_start, wall_end)
    wall_normal = (-(wall_end[1] - wall_start[1]) / wall_length, (wall_end[0] - wall_start[0]) / wall_length)
    half_axis = max(0.5, float(wall["thickness"]))
    window_axis_start = (window_centre[0] - wall_normal[0] * half_axis, window_centre[1] - wall_normal[1] * half_axis)
    window_axis_end = (window_centre[0] + wall_normal[0] * half_axis, window_centre[1] + wall_normal[1] * half_axis)

    pad_points = part_data["Landing pad 5 ft"][0]
    pad_centre = min(pad_points, key=lambda point: point[0])[:2]
    launch_points = part_data["Launch mark"][0]
    launch_centre = min(launch_points, key=lambda point: point[0])[:2]

    goals = [
        {"name": "tunnel", "shape": "oval", "centre": rounded_point(tube_centre), "params": {
            "axis_start": rounded_point(tube_axis_start), "axis_end": rounded_point(tube_axis_end), "width": 1.473, "height": 1.524,
        }},
        {"name": "window", "shape": "rect", "centre": rounded_point(window_centre), "params": {
            "wall_segment_index": segment_names.index(WINDOW_NAME), "axis_start": rounded_point(window_axis_start),
            "axis_end": rounded_point(window_axis_end), "side": 0.914, "sill_height": 0.305,
            "head_height": 1.219, "clearance_radius": round_value(clearance),
        }},
        {"name": "pad", "shape": "disc", "centre": rounded_point(pad_centre), "params": {"radius": 0.762}},
        {"name": "launch", "shape": "point", "centre": rounded_point(launch_centre), "params": {"heading_deg": 0}},
    ]
    return {"origin": [0.0, 0.0], "slice_height": SLICE_HEIGHT, "segments": segments, "boulders": boulders, "goals": goals, "_segment_names": segment_names}


def scalar(value: object) -> str:
    if isinstance(value, float):
        text = f"{value:.6f}".rstrip("0").rstrip(".")
        return text + ".0" if "." not in text else text
    return str(value)


def point_yaml(point: list[float]) -> str:
    return f"[{scalar(point[0])}, {scalar(point[1])}]"


def render_yaml(data: dict[str, object]) -> str:
    lines = ["# Generated by course/step_to_map.py from dronehacks-course-v3.step.", f"origin: {point_yaml(data['origin'])}", f"slice_height: {scalar(data['slice_height'])}", "segments:"]
    for segment in data["segments"]:
        lines.extend([f"  - start: {point_yaml(segment['start'])}", f"    end: {point_yaml(segment['end'])}", f"    thickness: {scalar(segment['thickness'])}"])
    lines.append("boulders:")
    for boulder in data["boulders"]:
        lines.extend([f"  - centre: {point_yaml(boulder['centre'])}", f"    radius: {scalar(boulder['radius'])}"])
    lines.append("goals:")
    for goal in data["goals"]:
        lines.extend([f"  - name: {goal['name']}", f"    shape: {goal['shape']}", f"    centre: {point_yaml(goal['centre'])}", "    params:"])
        for key, value in goal["params"].items():
            rendered = point_yaml(value) if isinstance(value, list) else scalar(value)
            lines.append(f"      {key}: {rendered}")
    return "\n".join(lines) + "\n"


def self_check(data: dict[str, object]) -> list[str]:
    failures: list[str] = []
    segment_names = data["_segment_names"]
    named_segments = dict(zip(segment_names, data["segments"]))

    net_points = [point for name, segment in named_segments.items() if name.startswith("Net side") for point in (segment["start"], segment["end"])]
    net_width = max(point[0] for point in net_points) - min(point[0] for point in net_points)
    net_height = max(point[1] for point in net_points) - min(point[1] for point in net_points)
    if not (math.isclose(net_width, 6.096, abs_tol=0.005) and math.isclose(net_height, 6.096, abs_tol=0.005)):
        failures.append(f"net bbox is {net_width:.6f} x {net_height:.6f} m, expected 6.096 x 6.096 m ± 0.005 m")

    goals = {goal["name"]: goal for goal in data["goals"]}
    for name, expected, tolerance in (("pad", (-2.042, -0.792), 0.005), ("launch", (-2.515, 0.914), 0.0006)):
        if name not in goals or any(not math.isclose(goals[name]["centre"][index], expected[index], abs_tol=tolerance) for index in range(2)):
            failures.append(f"{name} centre is {goals.get(name, {}).get('centre')}, expected {expected}")
    if len(data["boulders"]) != 5 or any(not math.isclose(item["radius"], 0.2285, abs_tol=0.0002) for item in data["boulders"]):
        failures.append("expected exactly 5 boulders with radius approximately 0.2285 m")
    wall_thicknesses = [segment["thickness"] for name, segment in named_segments.items() if is_wall_name(name)]
    if not wall_thicknesses or any(not math.isclose(value, 0.102, abs_tol=0.001) for value in wall_thicknesses):
        failures.append(f"wall thicknesses are {wall_thicknesses}, expected approximately 0.102 m")
    if "window" not in goals or WINDOW_NAME not in named_segments or goals["window"]["params"]["wall_segment_index"] != segment_names.index(WINDOW_NAME):
        failures.append(f"window goal is not linked to {WINDOW_NAME!r}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, default=DEFAULT_STEP, help="input AP214 STEP file")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="output course YAML")
    parser.add_argument("--window-clearance", type=float, help="reviewed window robot-centre clearance radius in metres")
    parser.add_argument("--self-check", action="store_true", help="validate known CAD/contract invariants after generation")
    args = parser.parse_args()
    if args.window_clearance is None:
        parser.error("--window-clearance is required; supply a reviewed human safety budget (no geometric default is permitted)")

    try:
        data = generate_map(args.step, args.window_clearance)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(render_yaml(data), encoding="utf-8")
        print(f"generated {args.output}")
        if args.self_check:
            failures = self_check(data)
            if failures:
                print("SELF-CHECK: FAIL", file=sys.stderr)
                for failure in failures:
                    print(f"  - {failure}", file=sys.stderr)
                return 1
            named_segments = dict(zip(data["_segment_names"], data["segments"]))
            net_points = [point for name, segment in named_segments.items() if name.startswith("Net side") for point in (segment["start"], segment["end"])]
            net_width = max(point[0] for point in net_points) - min(point[0] for point in net_points)
            net_height = max(point[1] for point in net_points) - min(point[1] for point in net_points)
            print("SELF-CHECK: PASS")
            print(f"  net bbox: {net_width:.6f} m x {net_height:.6f} m (target 6.096 m ± 0.005 m)")
            print(f"  pad centre: {tuple(data['goals'][2]['centre'])} m")
            print(f"  launch mark: {tuple(data['goals'][3]['centre'])} m, heading 0 deg")
            print("  boulders: 5, radius 0.2286 m")
            print("  wall thickness: 0.1016 m")
            print(f"  window: linked to {WINDOW_NAME!r}")
        return 0
    except (OSError, StepError, KeyError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
