#!/usr/bin/env python3
"""Generate the planner course map from the organizer AP214 STEP model.

This intentionally uses only the Python standard library so it can run on a
bare laptop.  It supports the small, explicit subset of STEP used by the
course model: named shape representations, their BREP relationships,
Cartesian points, directions, circles and ellipses.

Every goal parameter is derived from the model's own geometry.  Anything the
model does not define unambiguously is rejected with an error, never
substituted with a typed course-specific value (AGENTS.md §3.9).  The two
values the CAD cannot supply are explicit CLI arguments: the launch heading
(the mark is an undirected disc) and the window clearance radius, which has
no geometric default per ``contracts/course.md`` — without a reviewed budget
it is emitted as ``null`` so a conforming consumer refuses window traversal.

The self-check compares the model's *measured* geometry against the nominal
values in ``contracts/course.md``'s geometry table, and always runs the
schema's structural validation before the output file is written: a failing
check leaves any previous map on disk untouched.
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

# contracts/course.md's geometry table (organisers rows), course v3. The
# self-check compares the model's measured geometry against these nominal
# contract values with these tolerances. A layout change that moves a
# measured value outside a tolerance means the model or the contract row is
# stale — both are human decisions, so the check fails closed until one is
# updated (course.md: a surveyed row supersedes and gets a CHANGELOG entry).
NOMINAL = {
    "net_side_m": (6.096, 0.005),
    "wall_thickness_m": (0.102, 0.001),
    "boulder_count": 5,
    "boulder_radius_m": (0.2286, 0.001),  # geometry table: 0.457 m across
    "pad_centre_m": ((-2.042, -0.792), 0.005),
    "pad_radius_m": (0.762, 0.001),  # geometry table: 1.524 m across
    "launch_centre_m": ((-2.515, 0.914), 0.005),
    "launch_heading_deg": (0.0, 1e-9),  # geometry table: facing north
    "tunnel_width_m": (1.473, 0.01),  # 58 in
    # The geometry table's tunnel row is 58 x 72 x 60 in with 72 in vertical;
    # the schema example's "height: 1.524 # 60 in" is the tube's length, not
    # its height. Measured height is checked against the geometry table here.
    "tunnel_height_m": (1.829, 0.01),  # 72 in
    "window_side_m": (0.914, 0.01),
    "window_sill_m": (0.305, 0.01),
    "window_head_m": (1.219, 0.01),
}


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


def _point(body: str) -> tuple[float, float, float] | None:
    match = re.match(r"CARTESIAN_POINT\('[^']*',\(([^)]*)\)\)", body)
    if not match:
        return None
    values = [float(value) for value in match.group(1).split(",")]
    return (values[0], values[1], values[2]) if len(values) == 3 else None


def _direction(entities: dict[int, str], number: int) -> tuple[float, float, float] | None:
    body = entities.get(number)
    if body is None:
        return None
    match = re.match(r"DIRECTION\('[^']*',\(([^)]*)\)\)", body)
    if not match:
        return None
    values = [float(value) for value in match.group(1).split(",")]
    return (values[0], values[1], values[2]) if len(values) == 3 else None


def _placement(entities: dict[int, str], number: int) -> dict[str, object]:
    body = entities.get(number)
    if body is None:
        raise StepError(f"placement #{number} is referenced but undefined")
    match = re.match(r"AXIS2_PLACEMENT_3D\('[^']*',#(\d+)(?:,#(\d+))?(?:,#(\d+))?\)", body)
    if not match:
        raise StepError(f"placement #{number} is not an AXIS2_PLACEMENT_3D")
    location = _point(entities.get(int(match.group(1)), ""))
    if location is None:
        raise StepError(f"placement #{number} has no Cartesian location")
    axis = _direction(entities, int(match.group(2))) if match.group(2) else None
    reference = _direction(entities, int(match.group(3))) if match.group(3) else None
    return {"location": location, "axis": axis, "ref": reference}


def part_geometry(root: int, entities: dict[int, str]) -> tuple[list[tuple[float, float, float]], list[dict[str, object]], list[dict[str, object]]]:
    points: list[tuple[float, float, float]] = []
    circles: list[dict[str, object]] = []
    ellipses: list[dict[str, object]] = []
    for number in reachable_entities(root, entities):
        body = entities[number]
        if body.startswith("CARTESIAN_POINT"):
            point = _point(body)
            if point is not None:
                points.append(point)
        elif body.startswith("CIRCLE"):
            match = re.match(r"CIRCLE\('[^']*',#(\d+),([^)]*)\)", body)
            if match:
                placement = _placement(entities, int(match.group(1)))
                circles.append({"centre": placement["location"], "radius": float(match.group(2))})
        elif body.startswith("ELLIPSE"):
            match = re.match(r"ELLIPSE\('[^']*',#(\d+),([^,]+),([^,)]+)\)", body)
            if match:
                placement = _placement(entities, int(match.group(1)))
                ellipses.append({
                    "centre": placement["location"],
                    "semi1": float(match.group(2)),
                    "semi2": float(match.group(3)),
                    "axis": placement["axis"],
                    "ref": placement["ref"],
                })
    if not points:
        raise StepError(f"BREP #{root} contains no Cartesian points")
    return points, circles, ellipses


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


def _point_segment_distance(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    length = math.dist(start, end)
    if length <= 0.0:
        return math.dist(point, start)
    along = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
    offset = max(0.0, min(length, (point[0] - start[0]) * along[0] + (point[1] - start[1]) * along[1]))
    return math.dist(point, (start[0] + along[0] * offset, start[1] + along[1] * offset))


def hull_and_interior(points: list[tuple[float, float, float]]) -> tuple[list[tuple[float, float]], list[tuple[float, float, float]]]:
    """Split a part's points into footprint-hull boundary and interior points."""
    hull = convex_hull([(point[0], point[1]) for point in points])
    boundary: list[tuple[float, float, float]] = []
    interior: list[tuple[float, float, float]] = []
    for point in points:
        distance = min(
            _point_segment_distance((point[0], point[1]), hull[index], hull[(index + 1) % len(hull)])
            for index in range(len(hull))
        )
        (boundary if distance <= 1e-9 else interior).append(point)
    return hull, interior


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


def disc_geometry(name: str, circles: list[dict[str, object]]) -> tuple[tuple[float, float], float]:
    """Centre and radius of a disc part, from its CAD circles (fail closed)."""
    if not circles:
        raise StepError(f"{name} has no circle geometry to derive a centre and radius")
    radii = [float(circle["radius"]) for circle in circles]
    if max(radii) - min(radii) > 1e-9:
        raise StepError(f"{name} has inconsistent circle radii: {sorted(set(radii))}")
    centres = [circle["centre"] for circle in circles]
    for index in range(2):
        span = [float(centre[index]) for centre in centres]
        if max(span) - min(span) > 1e-6:
            raise StepError(f"{name} has inconsistent circle centres: {centres}")
    mean = (sum(float(centre[0]) for centre in centres) / len(centres), sum(float(centre[1]) for centre in centres) / len(centres))
    return mean, radii[0]


def tube_geometry(name: str, ellipses: list[dict[str, object]], pad_centre: tuple[float, float]) -> dict[str, object]:
    """Axis, width and height of the tube, from its CAD end-face ellipses.

    The two end faces carry the tube's outer cross-section ellipses; their
    placements give the entrance and exit face centres and the tube axis.
    The traversal direction is derived, not typed: the drone exits toward the
    pad (course.md checkpoint 3, "out of the south end" for this layout). Any
    other orientation, or an ambiguous relation to the pad, is rejected.
    """
    if not ellipses:
        raise StepError(f"{name} has no ellipse geometry to derive an axis and cross-section")
    peak = max(max(float(ellipse["semi1"]), float(ellipse["semi2"])) for ellipse in ellipses)
    outer = [ellipse for ellipse in ellipses if math.isclose(max(float(ellipse["semi1"]), float(ellipse["semi2"])), peak, abs_tol=1e-9)]
    if len(outer) != 2:
        raise StepError(f"{name} does not have exactly two outer end-face ellipses")
    first, second = outer
    if first["axis"] is None or second["axis"] is None:
        raise StepError(f"{name} end faces have no axis direction")
    first_axis, second_axis = first["axis"], second["axis"]
    if abs(first_axis[0] * second_axis[0] + first_axis[1] * second_axis[1] + first_axis[2] * second_axis[2]) < 1.0 - 1e-6:
        raise StepError(f"{name} end-face normals are not parallel; unsupported tube geometry")
    if abs(first_axis[2]) > 1e-6:
        raise StepError(f"{name} end-face normal is not horizontal; unsupported tube orientation")
    first_centre, second_centre = first["centre"], second["centre"]
    if math.isclose(first_centre[0], second_centre[0], abs_tol=1e-6) and math.isclose(first_centre[1], second_centre[1], abs_tol=1e-6):
        raise StepError(f"{name} end-face ellipses share one centre; no tube axis")
    if not math.isclose(first_centre[2], second_centre[2], abs_tol=1e-6):
        raise StepError(f"{name} end-face centres differ in height; unsupported tube orientation")
    axis_length = math.dist((first_centre[0], first_centre[1]), (second_centre[0], second_centre[1]))
    if axis_length <= 0.0:
        raise StepError(f"{name} has a degenerate axis")
    along = ((second_centre[0] - first_centre[0]) / axis_length, (second_centre[1] - first_centre[1]) / axis_length)
    mid = ((first_centre[0] + second_centre[0]) / 2.0, (first_centre[1] + second_centre[1]) / 2.0)
    reach = (pad_centre[0] - mid[0]) * along[0] + (pad_centre[1] - mid[1]) * along[1]
    if abs(reach) < 0.05:
        raise StepError(f"{name} traversal direction is ambiguous: the pad is not clearly on one side of the tube")
    # reach > 0 puts the pad beyond the second face: exit there, enter at the first
    entry, exit_ = (first_centre, second_centre) if reach > 0.0 else (second_centre, first_centre)

    reference = first["ref"]
    if reference is None:
        raise StepError(f"{name} end-face ellipses have no reference direction")
    in_plane = (
        first_axis[1] * reference[2] - first_axis[2] * reference[1],
        first_axis[2] * reference[0] - first_axis[0] * reference[2],
        first_axis[0] * reference[1] - first_axis[1] * reference[0],
    )
    if abs(reference[2]) > 0.9:
        horizontal_semi, vertical_semi = float(first["semi2"]), float(first["semi1"])
    elif abs(in_plane[2]) > 0.9:
        horizontal_semi, vertical_semi = float(first["semi1"]), float(first["semi2"])
    else:
        raise StepError(f"{name} cross-section has no vertical axis; unsupported tube orientation")
    return {
        "centre": mid,
        "axis_start": (entry[0], entry[1]),
        "axis_end": (exit_[0], exit_[1]),
        "width": 2.0 * horizontal_semi,
        "height": 2.0 * vertical_semi,
    }


def distinct_levels(values, digits: int = 6) -> list[float]:
    return sorted({round(value, digits) for value in values})


def window_opening(name: str, points: list[tuple[float, float, float]]) -> tuple[float, float, list[tuple[float, float, float]]]:
    """Sill/head heights and edge samples of a wall opening, from CAD.

    The opening's horizontal edges are the only geometry strictly inside the
    wall's footprint, at the only z levels that are neither the wall's floor
    nor its top. More or fewer than exactly two such levels is ambiguous and
    rejected (course.md: a flight slice must lie between floor-relative
    sill/head heights; nothing about the centre height alone is traversable).
    """
    hull, interior = hull_and_interior(points)
    if not interior:
        raise StepError(f"{name} has no interior opening geometry")
    levels = distinct_levels(point[2] for point in points)
    if len(levels) < 3:
        raise StepError(f"{name} has no distinct floor/opening/top z levels")
    floor, top = levels[0], levels[-1]
    opening_levels = [
        level
        for level in distinct_levels(point[2] for point in interior)
        if not (math.isclose(level, floor, abs_tol=1e-6) or math.isclose(level, top, abs_tol=1e-6))
    ]
    if len(opening_levels) != 2:
        raise StepError(f"{name} opening z levels are ambiguous: {opening_levels}")
    sill, head = opening_levels
    edges = [point for point in interior if math.isclose(point[2], sill, abs_tol=1e-6) or math.isclose(point[2], head, abs_tol=1e-6)]
    if not edges:
        raise StepError(f"{name} has no opening edge points at z {sill}/{head}")
    return sill, head, edges


def generate_map(step_path: Path, heading_deg: float, clearance: float | None) -> dict[str, object]:
    if not math.isfinite(heading_deg):
        raise StepError("--launch-heading must be a finite value from course.md's launch row")
    if clearance is not None and (not math.isfinite(clearance) or clearance <= 0.0):
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
        centre, radius = disc_geometry(name, part_data[name][1])
        boulders.append({"centre": rounded_point(centre), "radius": round_value(radius)})

    pad_centre, pad_radius = disc_geometry("Landing pad 5 ft", part_data["Landing pad 5 ft"][1])
    launch_centre, _launch_radius = disc_geometry("Launch mark", part_data["Launch mark"][1])

    tube = tube_geometry("Tube oval 58 x 72 x 60", part_data["Tube oval 58 x 72 x 60"][2], pad_centre)

    window_points = part_data[WINDOW_NAME][0]
    sill, head, opening_edges = window_opening(WINDOW_NAME, window_points)
    window_centre = mean_xy(opening_edges)
    wall = raw_segments[WINDOW_NAME]
    wall_start, wall_end = wall["start"], wall["end"]
    wall_length = math.dist(wall_start, wall_end)
    along = ((wall_end[0] - wall_start[0]) / wall_length, (wall_end[1] - wall_start[1]) / wall_length)
    normal = (-along[1], along[0])
    projections = [point[0] * along[0] + point[1] * along[1] for point in opening_edges]
    side = max(projections) - min(projections)
    if side <= 0.0:
        raise StepError(f"{WINDOW_NAME!r} opening has no measurable width")
    departure = (pad_centre[0] - window_centre[0]) * normal[0] + (pad_centre[1] - window_centre[1]) * normal[1]
    if abs(departure) < 0.05:
        raise StepError("window traversal direction is ambiguous: the pad is not clearly on one side of the wall")
    sign = 1.0 if departure > 0.0 else -1.0
    half_thickness = float(wall["thickness"]) / 2.0
    window_axis_start = (window_centre[0] - normal[0] * half_thickness * sign, window_centre[1] - normal[1] * half_thickness * sign)
    window_axis_end = (window_centre[0] + normal[0] * half_thickness * sign, window_centre[1] + normal[1] * half_thickness * sign)

    goals = [
        {"name": "tunnel", "shape": "oval", "centre": rounded_point(tube["centre"]), "params": {
            "axis_start": rounded_point(tube["axis_start"]), "axis_end": rounded_point(tube["axis_end"]),
            "width": round_value(tube["width"]), "height": round_value(tube["height"]),
        }},
        {"name": "window", "shape": "rect", "centre": rounded_point(window_centre), "params": {
            "wall_segment_index": segment_names.index(WINDOW_NAME),
            "axis_start": rounded_point(window_axis_start), "axis_end": rounded_point(window_axis_end),
            "side": round_value(side), "sill_height": round_value(sill), "head_height": round_value(head),
            "clearance_radius": None if clearance is None else round_value(clearance),
        }},
        {"name": "pad", "shape": "disc", "centre": rounded_point(pad_centre), "params": {"radius": round_value(pad_radius)}},
        {"name": "launch", "shape": "point", "centre": rounded_point(launch_centre), "params": {"heading_deg": round_value(heading_deg)}},
    ]
    return {"origin": [0.0, 0.0], "slice_height": SLICE_HEIGHT, "segments": segments, "boulders": boulders, "goals": goals, "_segment_names": segment_names}


def scalar(value: object) -> str:
    if isinstance(value, float):
        text = f"{value:.6f}".rstrip("0").rstrip(".")
        return text + ".0" if "." not in text else text
    return str(value)


def point_yaml(point: list[float]) -> str:
    return f"[{scalar(point[0])}, {scalar(point[1])}]"


def render_yaml(data: dict[str, object], command: str) -> str:
    lines = [
        "# Generated by course/step_to_map.py from dronehacks-course-v3.step.",
        f"# Regenerate: {command}",
        f"origin: {point_yaml(data['origin'])}",
        f"slice_height: {scalar(data['slice_height'])}",
        "segments:",
    ]
    for segment in data["segments"]:
        lines.extend([f"  - start: {point_yaml(segment['start'])}", f"    end: {point_yaml(segment['end'])}", f"    thickness: {scalar(segment['thickness'])}"])
    lines.append("boulders:")
    for boulder in data["boulders"]:
        lines.extend([f"  - centre: {point_yaml(boulder['centre'])}", f"    radius: {scalar(boulder['radius'])}"])
    lines.append("goals:")
    for goal in data["goals"]:
        lines.extend([f"  - name: {goal['name']}", f"    shape: {goal['shape']}", f"    centre: {point_yaml(goal['centre'])}", "    params:"])
        for key, value in goal["params"].items():
            if value is None:
                lines.append(f"      {key}: null  # UNREVIEWED — window traversal is blocked until a reviewed budget is supplied via --window-clearance")
            else:
                rendered = point_yaml(value) if isinstance(value, list) else scalar(value)
                lines.append(f"      {key}: {rendered}")
    return "\n".join(lines) + "\n"


def structural_check(data: dict[str, object]) -> list[str]:
    """course.md schema validation, independent of any one layout."""
    failures: list[str] = []
    goals = {goal["name"]: goal for goal in data["goals"]}
    for name in ("tunnel", "window", "pad", "launch"):
        if name not in goals:
            failures.append(f"missing required goal {name!r} (course.md: hard parse error)")
    segments = data["segments"]
    for segment in segments:
        values = [segment["start"][0], segment["start"][1], segment["end"][0], segment["end"][1], segment["thickness"]]
        if any(not math.isfinite(value) for value in values):
            failures.append("segment geometry is not finite")
            break
        if segment["thickness"] <= 0.0:
            failures.append("segment thickness is not positive")
            break
    for boulder in data["boulders"]:
        if not math.isfinite(boulder["radius"]) or boulder["radius"] <= 0.0:
            failures.append("boulder radius is not positive")
            break
    if "window" in goals:
        params = goals["window"]["params"]
        index = params.get("wall_segment_index")
        if not isinstance(index, int) or not 0 <= index < len(segments):
            failures.append("window wall_segment_index does not reference a segment")
        else:
            segment = segments[index]
            start, end = params["axis_start"], params["axis_end"]
            centre = goals["window"]["centre"]
            if start == end:
                failures.append("window traversal axis is degenerate")
            if any(not math.isclose((start[i] + end[i]) / 2.0, centre[i], abs_tol=2e-6) for i in range(2)):
                failures.append("window traversal axis does not cross the opening centre")
            segment_vector = (segment["end"][0] - segment["start"][0], segment["end"][1] - segment["start"][1])
            axis_vector = (end[0] - start[0], end[1] - start[1])
            if abs(segment_vector[0] * axis_vector[0] + segment_vector[1] * axis_vector[1]) > 1e-5:
                failures.append("window traversal axis is not normal to the referenced wall")
        if not params["sill_height"] < params["head_height"]:
            failures.append("window sill/head heights do not bound a flight slice")
        if not params["side"] > 0.0:
            failures.append("window side is not positive")
        clearance = params.get("clearance_radius")
        if clearance is not None and (not math.isfinite(clearance) or clearance <= 0.0):
            failures.append("window clearance_radius must be null (unreviewed) or a positive reviewed budget")
    if "tunnel" in goals:
        params = goals["tunnel"]["params"]
        if params["axis_start"] == params["axis_end"]:
            failures.append("tunnel traversal axis is degenerate")
        for key in ("width", "height"):
            if not math.isfinite(params[key]) or params[key] <= 0.0:
                failures.append(f"tunnel {key} is not positive")
    if "pad" in goals:
        radius = goals["pad"]["params"]["radius"]
        if not math.isfinite(radius) or radius <= 0.0:
            failures.append("pad radius is not positive")
    return failures


def nominal_check(data: dict[str, object]) -> list[str]:
    """Compare measured geometry against course.md's course-v3 nominal table."""
    failures: list[str] = []
    named_segments = dict(zip(data["_segment_names"], data["segments"]))

    def expect(measured: float, key: str, what: str) -> None:
        expected, tolerance = NOMINAL[key]
        if not math.isclose(measured, expected, abs_tol=tolerance):
            failures.append(f"{what} is {measured}, nominal is {expected} ± {tolerance}")

    net_points = [point for name, segment in named_segments.items() if name.startswith("Net side") for point in (segment["start"], segment["end"])]
    net_width = max(point[0] for point in net_points) - min(point[0] for point in net_points)
    net_height = max(point[1] for point in net_points) - min(point[1] for point in net_points)
    expect(net_width, "net_side_m", "net breadth")
    expect(net_height, "net_side_m", "net depth")

    goals = {goal["name"]: goal for goal in data["goals"]}
    for name in ("pad", "launch"):
        if name in goals:
            expected, tolerance = NOMINAL[f"{name}_centre_m"]
            if any(not math.isclose(goals[name]["centre"][index], expected[index], abs_tol=tolerance) for index in range(2)):
                failures.append(f"{name} centre is {goals[name]['centre']}, nominal is {list(expected)}")
    if "pad" in goals:
        expect(goals["pad"]["params"]["radius"], "pad_radius_m", "pad radius")
    if "launch" in goals:
        expect(goals["launch"]["params"]["heading_deg"], "launch_heading_deg", "launch heading (deg)")
    if len(data["boulders"]) != NOMINAL["boulder_count"]:
        failures.append(f"expected exactly {NOMINAL['boulder_count']} boulders, found {len(data['boulders'])}")
    for boulder in data["boulders"]:
        expect(boulder["radius"], "boulder_radius_m", "boulder radius")
    wall_thicknesses = [segment["thickness"] for name, segment in named_segments.items() if is_wall_name(name)]
    if not wall_thicknesses:
        failures.append("no wall segments found")
    for thickness in wall_thicknesses:
        expect(thickness, "wall_thickness_m", "wall thickness")
    if "tunnel" in goals:
        expect(goals["tunnel"]["params"]["width"], "tunnel_width_m", "tunnel width")
        expect(goals["tunnel"]["params"]["height"], "tunnel_height_m", "tunnel height")
    if "window" in goals:
        expect(goals["window"]["params"]["side"], "window_side_m", "window side")
        expect(goals["window"]["params"]["sill_height"], "window_sill_m", "window sill height")
        expect(goals["window"]["params"]["head_height"], "window_head_m", "window head height")
    return failures


def report_pass(data: dict[str, object]) -> None:
    """Print the map's measured values (never typed constants)."""
    named_segments = dict(zip(data["_segment_names"], data["segments"]))
    net_points = [point for name, segment in named_segments.items() if name.startswith("Net side") for point in (segment["start"], segment["end"])]
    net_width = max(point[0] for point in net_points) - min(point[0] for point in net_points)
    net_height = max(point[1] for point in net_points) - min(point[1] for point in net_points)
    goals = {goal["name"]: goal for goal in data["goals"]}
    boulder_radii = sorted({boulder["radius"] for boulder in data["boulders"]})
    thicknesses = sorted({segment["thickness"] for name, segment in named_segments.items() if is_wall_name(name)})
    print("SELF-CHECK: PASS")
    print(f"  net bbox: {net_width:.6f} m x {net_height:.6f} m (nominal {NOMINAL['net_side_m'][0]} m ± {NOMINAL['net_side_m'][1]} m)")
    if "tunnel" in goals:
        params = goals["tunnel"]["params"]
        print(f"  tunnel: width {params['width']} m, height {params['height']} m, axis {tuple(params['axis_start'])} -> {tuple(params['axis_end'])}")
    if "window" in goals:
        params = goals["window"]["params"]
        clearance = "null (unreviewed; traversal blocked)" if params["clearance_radius"] is None else f"{params['clearance_radius']} m"
        print(f"  window: linked to {WINDOW_NAME!r}, side {params['side']} m, sill {params['sill_height']} m, head {params['head_height']} m")
        print(f"  window: clearance_radius {clearance}")
    if "pad" in goals:
        print(f"  pad: centre {tuple(goals['pad']['centre'])} m, radius {goals['pad']['params']['radius']} m")
    if "launch" in goals:
        print(f"  launch: {tuple(goals['launch']['centre'])} m, heading {goals['launch']['params']['heading_deg']} deg")
    print(f"  boulders: {len(data['boulders'])}, radii {boulder_radii} m")
    print(f"  wall thicknesses: {thicknesses} m")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, default=DEFAULT_STEP, help="input AP214 STEP file")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="output course YAML")
    parser.add_argument("--launch-heading", type=float, required=True,
                        help="launch heading in degrees clockwise from course north, from course.md's launch row (the CAD mark is an undirected disc)")
    parser.add_argument("--window-clearance", type=float, default=None,
                        help="reviewed window robot-centre clearance radius in metres; omit to emit an explicit unreviewed null")
    parser.add_argument("--self-check", action="store_true",
                        help="validate the model's measured geometry against contracts/course.md's nominal table before writing")
    args = parser.parse_args(argv)

    parts = [f"python3 course/step_to_map.py --launch-heading {args.launch_heading}"]
    if args.window_clearance is not None:
        parts.append(f"--window-clearance {args.window_clearance}")
    if args.self_check:
        parts.append("--self-check")
    command = " ".join(parts)

    try:
        data = generate_map(args.step, args.launch_heading, args.window_clearance)
        failures = structural_check(data)
        if args.self_check:
            failures += nominal_check(data)
        if failures:
            print("SELF-CHECK: FAIL — output not written", file=sys.stderr)
            for failure in failures:
                print(f"  - {failure}", file=sys.stderr)
            return 1
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(render_yaml(data, command), encoding="utf-8")
        print(f"generated {args.output}")
        if args.self_check:
            report_pass(data)
        return 0
    except (OSError, StepError, KeyError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
