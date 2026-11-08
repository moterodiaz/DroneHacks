"""Read a ``contracts/course.md`` map file and hand the planner a ``CourseMap``.

This is the one place that knows the contract schema. A new course is a new map
file: nothing downstream knows about mazes, barriers or chicanes, it only sees
walls and goals, and A* finds the way between them.

What the conversion does, all of it derived from the file:

* **tunnel** becomes a pass-through goal along ``axis_start -> axis_end``, plus
  two thin side walls either side of the axis. The contract's segments do not
  include the tube, so without them the planner would see an open gap and could
  cut across it. ``width`` is the tube's widest, so the walls go where the oval
  really is over the drone's height at flight height, measured up from the
  floor. A roof the drone cannot fit under is refused.
* **window** is traversal-blocked unless ``use_window`` is set and the map
  carries a reviewed ``clearance_radius`` (contracts/course.md: never substitute
  a default). While blocked its wall stays solid, so the route takes whatever
  other way the walls leave open. On the published layout that is Gate B,
  without Gate B being named anywhere in code or map.
* **launch** gives the planner's start and the yaw reference. It is not routed.

No course coordinate is written here (rule 9.5, AGENTS.md section 3.9).
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple

import yaml

from .planning.course_map import CourseMap, MapError, parse_map
from .planning.route import PlannerConfig

REQUIRED_GOALS = ("tunnel", "window", "pad", "launch")
#: Tolerance for "this point is on that line", in metres. Generated maps round to
#: about a tenth of a millimetre; a hand-surveyed one may be a little looser.
ON_LINE_TOL_M = 0.02
#: Largest allowed |cos| between the window's axis and its wall (about 2 degrees off normal).
NORMAL_TOL_COS = 0.035


def load_contract_map(path: "str | Path", use_window: bool = False) -> CourseMap:
    """Read and validate a contract map file. Raises ``MapError``: never substitutes a default."""
    path = Path(path)
    raw = path.read_bytes()
    try:
        doc = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise MapError(f"{path}: not valid YAML: {exc}") from exc
    return from_contract(doc, source=str(path), sha256=hashlib.sha256(raw).hexdigest(), use_window=use_window)


def from_contract(doc: Any, source: str = "<dict>", sha256: str = "", use_window: bool = False) -> CourseMap:
    """Convert an already-parsed contract map document."""
    if not isinstance(doc, Mapping):
        raise MapError(f"{source}: top level must be a mapping")
    goals = doc.get("goals")
    if not isinstance(goals, list):
        raise MapError(f"{source}: goals must be a list")
    by_name: Dict[str, Mapping] = {}
    for g in goals:
        if not isinstance(g, Mapping) or not isinstance(g.get("name"), str):
            raise MapError(f"{source}: every goal needs a name")
        if g["name"] in by_name:
            raise MapError(f"{source}: goal {g['name']!r} is listed twice")
        by_name[g["name"]] = g
    missing = [n for n in REQUIRED_GOALS if n not in by_name]
    if missing:
        raise MapError(f"{source}: missing required goal(s) {', '.join(missing)} (contracts/course.md)")

    segments = _segments(doc, source)
    window = _window(by_name["window"], segments, source)
    converted: List[dict] = []
    for g in goals:
        name = g["name"]
        if name == "tunnel":
            converted.append(_tunnel(g, segments, doc.get("slice_height"), source))
        elif name == "window":
            if use_window:
                converted.append(_open_window(g, window, segments, source))
        elif name == "pad":
            converted.append(_pad(g, source))
        elif name != "launch":
            raise MapError(f"{source}: goal {name!r} is not in the contract (tunnel, window, pad, launch)")

    return parse_map({
        "origin": doc.get("origin"),
        "slice_height": doc.get("slice_height"),
        "segments": segments,
        "boulders": doc.get("boulders") or [],
        "goals": converted,
        "launch": _launch(by_name["launch"], source),
    }, source=source, sha256=sha256)


# -- goals -------------------------------------------------------------------------------


def _segments(doc: Mapping, source: str) -> List[dict]:
    raw = doc.get("segments")
    if not isinstance(raw, list) or not raw:
        raise MapError(f"{source}: segments must be a non-empty list")
    out = []
    for i, s in enumerate(raw):
        if not isinstance(s, Mapping):
            raise MapError(f"{source}: segments[{i}] must be a mapping")
        out.append(dict(s))
    return out


def _params(goal: Mapping, keys: Tuple[str, ...], source: str) -> Mapping:
    params = goal.get("params")
    where = f"{source}: goal {goal['name']}"
    if not isinstance(params, Mapping):
        raise MapError(f"{where}: params must be a mapping")
    for key in keys:
        if key not in params:
            raise MapError(f"{where}: params.{key} is required (contracts/course.md)")
    return params


def _shape(goal: Mapping, shape: str, source: str) -> None:
    if goal.get("shape") != shape:
        raise MapError(f"{source}: goal {goal['name']} must have shape {shape}, not {goal.get('shape')!r}")


def _pt(v: Any, where: str) -> Tuple[float, float]:
    if (not isinstance(v, (list, tuple)) or len(v) != 2
            or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)):
        raise MapError(f"{where}: must be [east, north], got {v!r}")
    return float(v[0]), float(v[1])


def _pos(v: Any, where: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
        raise MapError(f"{where}: must be a finite number > 0, got {v!r}")
    return float(v)


def _axis(params: Mapping, where: str) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    """``(start, unit direction, length)`` of a goal's directed axis."""
    a = _pt(params["axis_start"], f"{where}.axis_start")
    b = _pt(params["axis_end"], f"{where}.axis_end")
    length = math.dist(a, b)
    if length < 1e-6:
        raise MapError(f"{where}: the axis has zero length")
    return a, ((b[0] - a[0]) / length, (b[1] - a[1]) / length), length


def _off_axis(p: Tuple[float, float], a: Tuple[float, float], u: Tuple[float, float]) -> float:
    return abs((p[0] - a[0]) * u[1] - (p[1] - a[1]) * u[0])


def _oval_width(width: float, height: float, z: float) -> float:
    """The width of an oval standing on the floor, ``z`` above the floor."""
    return width * math.sqrt(max(0.0, 1.0 - ((z - height / 2.0) / (height / 2.0)) ** 2))


def _tunnel(goal: Mapping, segments: List[dict], slice_height: Any, source: str) -> dict:
    where = f"{source}: goal tunnel"
    _shape(goal, "oval", source)
    params = _params(goal, ("axis_start", "axis_end", "width", "height"), source)
    a, u, length = _axis(params, f"{where}.params")
    centre = _pt(goal.get("centre"), f"{where}.centre")
    if _off_axis(centre, a, u) > ON_LINE_TOL_M:
        raise MapError(f"{where}: its centre is not on its axis")
    width = _pos(params["width"], f"{where}.params.width")
    height = _pos(params["height"], f"{where}.params.height")
    if isinstance(slice_height, (int, float)) and not isinstance(slice_height, bool):   # else parse_map refuses it
        half = PlannerConfig().drone_half_height_m
        if not half <= slice_height <= height - half:
            raise MapError(f"{where}: flight height {slice_height} m does not fit a {height} m tube "
                           f"with {half} m of drone above and below")
        width = min(_oval_width(width, height, slice_height - half), _oval_width(width, height, slice_height + half))
    nx, ny = -u[1], u[0]
    for side in (-1.0, 1.0):
        off = side * width / 2.0
        segments.append({
            "start": [a[0] + nx * off, a[1] + ny * off],
            "end": [a[0] + u[0] * length + nx * off, a[1] + u[1] * length + ny * off],
            "thickness": 0.0,
        })
    return {"name": "tunnel", "shape": "oval", "centre": list(centre),
            "params": {"axis": list(u), "length": length, "width": width, "height": height}}


def _window(goal: Mapping, segments: List[dict], source: str) -> dict:
    """Validate the window whether or not it is flown: a bad map is refused either way."""
    where = f"{source}: goal window"
    _shape(goal, "rect", source)
    params = _params(goal, ("wall_segment_index", "axis_start", "axis_end", "side", "sill_height",
                            "head_height", "clearance_radius"), source)
    index = params["wall_segment_index"]
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(segments):
        raise MapError(f"{where}: wall_segment_index {index!r} is not a segment")
    wall = segments[index]
    w0 = _pt(wall.get("start"), f"{where}: its wall's start")
    w1 = _pt(wall.get("end"), f"{where}: its wall's end")
    wall_len = math.dist(w0, w1)
    if wall_len < 1e-6:
        raise MapError(f"{where}: its wall has zero length")
    wu = ((w1[0] - w0[0]) / wall_len, (w1[1] - w0[1]) / wall_len)
    a, u, _ = _axis(params, f"{where}.params")
    centre = _pt(goal.get("centre"), f"{where}.centre")
    if _off_axis(centre, a, u) > ON_LINE_TOL_M:
        raise MapError(f"{where}: its axis does not cross the opening's centre")
    if abs(u[0] * wu[0] + u[1] * wu[1]) > NORMAL_TOL_COS:
        raise MapError(f"{where}: its axis is not normal to its wall")
    if _off_axis(centre, w0, wu) > ON_LINE_TOL_M:
        raise MapError(f"{where}: its centre is not on its wall")
    side = _pos(params["side"], f"{where}.params.side")
    along = (centre[0] - w0[0]) * wu[0] + (centre[1] - w0[1]) * wu[1]
    if along - side / 2.0 < -ON_LINE_TOL_M or along + side / 2.0 > wall_len + ON_LINE_TOL_M:
        raise MapError(f"{where}: the opening does not fit inside its wall")
    sill = float(params["sill_height"])
    head = _pos(params["head_height"], f"{where}.params.head_height")
    if not 0.0 <= sill < head:
        raise MapError(f"{where}: sill_height {sill} must be below head_height {head}")
    return {"index": index, "w0": w0, "w1": w1, "wu": wu, "u": u, "along": along, "side": side,
            "sill": sill, "head": head, "centre": centre}


def _open_window(goal: Mapping, window: dict, segments: List[dict], source: str) -> dict:
    """Cut the opening out of its wall and make it a pass-through goal. Fails closed without
    a reviewed ``clearance_radius`` (contracts/course.md)."""
    where = f"{source}: goal window"
    clearance = goal["params"]["clearance_radius"]
    if clearance is None:
        raise MapError(f"{where}: clearance_radius is null, so the window is traversal-blocked "
                       "(contracts/course.md). Supply a reviewed budget in the map, or leave use_window off")
    clearance = _pos(clearance, f"{where}.params.clearance_radius")
    side, along, wu, w0, w1 = window["side"], window["along"], window["wu"], window["w0"], window["w1"]
    wall = segments[window["index"]]
    near = max(along - side / 2.0, 0.0)
    far = min(along + side / 2.0, math.dist(w0, w1))
    pieces = [{**wall, "start": list(w0), "end": [w0[0] + wu[0] * near, w0[1] + wu[1] * near]},
              {**wall, "start": [w0[0] + wu[0] * far, w0[1] + wu[1] * far], "end": list(w1)}]
    # Replace in place, so any later index still means the same wall. A sliver left by an
    # opening reaching the wall's end is dropped.
    segments[window["index"]] = pieces[0]
    if math.dist(pieces[1]["start"], pieces[1]["end"]) > 1e-6:
        segments.append(pieces[1])
    if math.dist(pieces[0]["start"], pieces[0]["end"]) <= 1e-6:
        last = segments.pop()
        if window["index"] < len(segments):     # else the sliver was itself the last segment
            segments[window["index"]] = last
    return {"name": "window", "shape": "rect", "centre": list(window["centre"]),
            "params": {"axis": list(window["u"]), "width": side, "depth": float(wall.get("thickness", 0.0)),
                       "bottom": window["sill"], "top": window["head"], "clearance_radius": clearance}}


def _pad(goal: Mapping, source: str) -> dict:
    where = f"{source}: goal pad"
    _shape(goal, "disc", source)
    params = _params(goal, ("radius",), source)
    return {"name": "pad", "shape": "disc", "centre": list(_pt(goal.get("centre"), f"{where}.centre")),
            "params": {"radius": _pos(params["radius"], f"{where}.params.radius")}}


def _launch(goal: Mapping, source: str) -> dict:
    where = f"{source}: goal launch"
    _shape(goal, "point", source)
    params = _params(goal, ("heading_deg",), source)
    heading = params["heading_deg"]
    if isinstance(heading, bool) or not isinstance(heading, (int, float)) or not math.isfinite(heading):
        raise MapError(f"{where}: heading_deg must be a finite number, got {heading!r}")
    h = math.radians(float(heading))   # clockwise from course north
    return {"position": list(_pt(goal.get("centre"), f"{where}.centre")), "heading": [math.sin(h), math.cos(h)]}
