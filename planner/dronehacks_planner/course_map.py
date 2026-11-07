"""Course map: load and validate the planner's only input.

The format is the YAML map file in ``contracts/course.md`` ("Map-file format
(planner input)"): wall and net footprints sliced at the flight height, boulder
discs, and the goals the route must visit, in order. Everything the planner
knows about a course comes from this file. No course coordinate is written in
code (rule 9.5; AGENTS.md section 3.7 on the contracts branch).

Coordinates are course-frame ``(east, north)`` metres. Turning them into PX4
setpoints is ``contracts/frames.md``'s transform and is not done here.

Fields marked PROPOSED are not in ``contracts/course.md`` yet. They are
implemented here and listed in ``planner/README.md`` for a contracts/ PR.
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

import yaml

Point = Tuple[float, float]

#: Shapes the route flies straight through, along the goal's ``axis``.
PASS_SHAPES = ("oval", "rect", "gate")
#: Shapes the route flies to the centre of.
POINT_SHAPES = ("disc",)
SHAPES = PASS_SHAPES + POINT_SHAPES


class MapError(ValueError):
    """The map file is malformed or inconsistent."""


@dataclasses.dataclass(frozen=True)
class Segment:
    """A wall or net footprint: a line with a thickness (0 for an outline edge)."""

    start: Point
    end: Point
    thickness: float


@dataclasses.dataclass(frozen=True)
class Boulder:
    centre: Point
    radius: float
    #: PROPOSED. Height above the floor. Unknown (None) means the planner
    #: treats the boulder as tall as a wall.
    height: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class Goal:
    """A place the route must visit.

    ``oval`` (a tunnel), ``rect`` (a window) and ``gate`` (a gap between walls,
    PROPOSED) are flown straight through along ``params.axis``. ``disc`` (the
    pad) is flown to its centre. Goals sharing a ``group`` (PROPOSED) are
    alternatives: the route visits exactly one of them.
    """

    name: str
    shape: str
    centre: Point
    params: Mapping[str, Any]
    group: Optional[str] = None

    @property
    def passes_through(self) -> bool:
        return self.shape in PASS_SHAPES

    @property
    def axis(self) -> Point:
        """Unit direction of travel through a pass-through goal. A checkpoint
        counts in one direction only ("out of the south end"), so the sign
        matters, unless ``params.bidirectional`` (PROPOSED) is true."""
        return self.params["axis"]

    @property
    def bidirectional(self) -> bool:
        return bool(self.params.get("bidirectional", False))

    @property
    def depth(self) -> float:
        """Length of the opening along its axis: a tunnel's ``length``, a window's or
        gate's ``depth`` (the wall thickness, PROPOSED; 0 if not given)."""
        if self.shape == "oval":
            return float(self.params["length"])
        return float(self.params.get("depth", 0.0))


@dataclasses.dataclass(frozen=True)
class Launch:
    """PROPOSED: where the drone starts, and which way it faces."""

    position: Point
    heading: Optional[Point] = None


@dataclasses.dataclass(frozen=True)
class CourseMap:
    origin: Point
    slice_height: float
    segments: Tuple[Segment, ...]
    boulders: Tuple[Boulder, ...]
    goals: Tuple[Goal, ...]
    launch: Optional[Launch]
    sha256: str
    source: str

    def stages(self) -> List[Tuple[Goal, ...]]:
        """The goals in visiting order, with each group of alternatives as one stage."""
        out: List[Tuple[Goal, ...]] = []
        for goal in self.goals:
            if goal.group is not None and out and out[-1][0].group == goal.group:
                out[-1] = out[-1] + (goal,)
            else:
                out.append((goal,))
        return out

    def bounds(self) -> Tuple[float, float, float, float]:
        """``(min_east, min_north, max_east, max_north)`` over every element."""
        xs: List[float] = []
        ys: List[float] = []
        for s in self.segments:
            pad = s.thickness / 2.0
            for p in (s.start, s.end):
                xs += [p[0] - pad, p[0] + pad]
                ys += [p[1] - pad, p[1] + pad]
        for b in self.boulders:
            xs += [b.centre[0] - b.radius, b.centre[0] + b.radius]
            ys += [b.centre[1] - b.radius, b.centre[1] + b.radius]
        for g in self.goals:
            xs.append(g.centre[0])
            ys.append(g.centre[1])
        if self.launch is not None:
            xs.append(self.launch.position[0])
            ys.append(self.launch.position[1])
        return min(xs), min(ys), max(xs), max(ys)


def load_map(path: "str | Path") -> CourseMap:
    """Read and validate a map file."""
    path = Path(path)
    raw = path.read_bytes()
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise MapError(f"{path}: not valid YAML: {exc}") from exc
    return parse_map(data, source=str(path), sha256=hashlib.sha256(raw).hexdigest())


def parse_map(data: Any, source: str = "<dict>", sha256: str = "") -> CourseMap:
    """Validate an already-parsed map document."""
    if not isinstance(data, Mapping):
        raise MapError(f"{source}: top level must be a mapping")
    where = source

    origin = _point(_require(data, "origin", where), f"{where}: origin")
    if origin != (0.0, 0.0):
        # contracts/course.md does not say what a non-zero origin would mean
        # (issue #12). Refuse rather than guess at an offset.
        raise MapError(f"{where}: origin {origin} is not [0, 0]; a non-zero origin is undefined")
    slice_height = _positive(_require(data, "slice_height", where), f"{where}: slice_height")

    default_thickness = data.get("default_thickness")  # PROPOSED
    if default_thickness is not None:
        default_thickness = _non_negative(default_thickness, f"{where}: default_thickness")

    segments = []
    for i, s in enumerate(_list(data.get("segments", []), f"{where}: segments")):
        w = f"{where}: segments[{i}]"
        if not isinstance(s, Mapping):
            raise MapError(f"{w}: must be a mapping")
        thickness = s.get("thickness", default_thickness)
        if thickness is None:
            raise MapError(f"{w}: no thickness, and the map has no default_thickness")
        segments.append(Segment(
            start=_point(_require(s, "start", w), f"{w}.start"),
            end=_point(_require(s, "end", w), f"{w}.end"),
            thickness=_non_negative(thickness, f"{w}.thickness"),
        ))

    boulders = []
    for i, b in enumerate(_list(data.get("boulders", []), f"{where}: boulders")):
        w = f"{where}: boulders[{i}]"
        if not isinstance(b, Mapping):
            raise MapError(f"{w}: must be a mapping")
        boulders.append(Boulder(
            centre=_point(_require(b, "centre", w), f"{w}.centre"),
            radius=_positive(_require(b, "radius", w), f"{w}.radius"),
            height=_positive(b["height"], f"{w}.height") if b.get("height") is not None else None,
        ))

    if not segments and not boulders:
        raise MapError(f"{where}: no segments or boulders; nothing to plan around")

    goals = []
    seen = set()
    for i, g in enumerate(_list(_require(data, "goals", where), f"{where}: goals")):
        goal = _goal(g, f"{where}: goals[{i}]")
        if goal.name in seen:
            raise MapError(f"{where}: goal name {goal.name!r} is used twice")
        seen.add(goal.name)
        goals.append(goal)
    if not goals:
        raise MapError(f"{where}: goals is empty")

    launch = None
    if data.get("launch") is not None:
        lw = f"{where}: launch"
        ld = data["launch"]
        if not isinstance(ld, Mapping):
            raise MapError(f"{lw}: must be a mapping")
        heading = ld.get("heading")
        launch = Launch(
            position=_point(_require(ld, "position", lw), f"{lw}.position"),
            heading=_unit(heading, f"{lw}.heading") if heading is not None else None,
        )

    cmap = CourseMap(
        origin=origin,
        slice_height=slice_height,
        segments=tuple(segments),
        boulders=tuple(boulders),
        goals=tuple(goals),
        launch=launch,
        sha256=sha256,
        source=source,
    )
    _check_groups(cmap, where)
    return cmap


def _goal(g: Any, w: str) -> Goal:
    if not isinstance(g, Mapping):
        raise MapError(f"{w}: must be a mapping")
    name = _require(g, "name", w)
    if not isinstance(name, str) or not name:
        raise MapError(f"{w}.name: must be a non-empty string")
    w = f"{w} ({name})"
    shape = _require(g, "shape", w)
    if shape not in SHAPES:
        raise MapError(f"{w}.shape: {shape!r} is not one of {', '.join(SHAPES)}")
    centre = _point(_require(g, "centre", w), f"{w}.centre")
    raw = g.get("params") or {}
    if not isinstance(raw, Mapping):
        raise MapError(f"{w}.params: must be a mapping")
    params: Dict[str, Any] = dict(raw)
    pw = f"{w}.params"
    if shape == "disc":
        params["radius"] = _positive(_require(raw, "radius", pw), f"{pw}.radius")
    else:
        params["axis"] = _unit(_require(raw, "axis", pw), f"{pw}.axis")
        params["width"] = _positive(_require(raw, "width", pw), f"{pw}.width")
        if not isinstance(raw.get("bidirectional", False), bool):
            raise MapError(f"{pw}.bidirectional: must be true or false")
        if shape == "oval":
            params["length"] = _positive(_require(raw, "length", pw), f"{pw}.length")
        elif "depth" in raw:
            params["depth"] = _non_negative(raw["depth"], f"{pw}.depth")
        if shape == "rect" and ("bottom" in raw or "top" in raw):
            bottom = _non_negative(_require(raw, "bottom", pw), f"{pw}.bottom")
            top = _positive(_require(raw, "top", pw), f"{pw}.top")
            if not bottom < top:
                raise MapError(f"{pw}: bottom {bottom} must be below top {top}")
            params["bottom"], params["top"] = bottom, top
    group = g.get("group")
    if group is not None and (not isinstance(group, str) or not group):
        raise MapError(f"{w}.group: must be a non-empty string")
    return Goal(name=name, shape=shape, centre=centre, params=params, group=group)


def _check_groups(cmap: CourseMap, where: str) -> None:
    seen_groups = set()
    for stage in cmap.stages():
        group = stage[0].group
        if group is None:
            continue
        if group in seen_groups:
            raise MapError(f"{where}: goals in group {group!r} must be listed together")
        seen_groups.add(group)


def _require(m: Mapping, key: str, where: str) -> Any:
    if key not in m or m[key] is None:
        raise MapError(f"{where}: missing {key!r}")
    return m[key]


def _list(v: Any, where: str) -> list:
    if not isinstance(v, list):
        raise MapError(f"{where}: must be a list")
    return v


def _number(v: Any, where: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise MapError(f"{where}: {v!r} is not a finite number")
    return float(v)


def _positive(v: Any, where: str) -> float:
    x = _number(v, where)
    if x <= 0:
        raise MapError(f"{where}: must be > 0, got {x}")
    return x


def _non_negative(v: Any, where: str) -> float:
    x = _number(v, where)
    if x < 0:
        raise MapError(f"{where}: must be >= 0, got {x}")
    return x


def _point(v: Any, where: str) -> Point:
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise MapError(f"{where}: must be [east, north], got {v!r}")
    return (_number(v[0], where), _number(v[1], where))


def _unit(v: Any, where: str) -> Point:
    x, y = _point(v, where)
    n = math.hypot(x, y)
    if n < 1e-9:
        raise MapError(f"{where}: direction has zero length")
    return (x / n, y / n)
