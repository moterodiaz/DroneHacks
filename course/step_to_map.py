#!/usr/bin/env python3
"""Generate the planner course map from the organizer AP214 STEP model.

This intentionally uses only the Python standard library so it can run on a
bare laptop (CI runs it under ``python3 -I`` with no third-party packages).
It reads ISO 10303-21 exchange files whatever their spacing: Onshape writes
them compact (``#1=CARTESIAN_POINT('',(0.,0.,0.));``), SolidWorks spaced and
with CRLF line ends (``#1 = CARTESIAN_POINT ( 'NONE', ( 0.0, 0.0, 0.0 ) ) ;``),
and both write complex instances (``#2 =( LENGTH_UNIT ( ) SI_UNIT ( ... ) );``).
It supports the explicit subset of AP214 the course model uses: the product
structure that names each solid, manifold solid B-reps, planar faces, and
line, circle, ellipse and (rational) B-spline edge curves.  Anything else is
rejected with an error rather than guessed.

Each MANIFOLD_SOLID_BREP is one solid, named after its product (see
``part_label``).  A product holding several solids (a SolidWorks multi-body
part) gives one solid per body, ``'<product> / <body name>'``; its bodies are
never merged into one footprint.

Walls are found by geometry, not by name: every solid that reaches the slice
height must be a floor-standing vertical extrusion (a wall, or a net side) or
a tube or boulder, whose footprints are their own goals; anything else at the
slice height is rejected.  The window wall is the unique part whose name ends
``(window)``, and the tube, boulders, pad and launch mark are found by role.

Every goal parameter is derived from the model's own geometry.  Circles and
ellipses come from CIRCLE/ELLIPSE edges or, where the exporter wrote them as
B-spline curves (SolidWorks does for the boulders and the tube), from a
least-squares fit to the boundary loops of the part's planar faces: through
each B-spline edge's points at its knots, where CAD spline forms of a conic
interpolate it, after which every evaluated sample (de Boor, rational weights
honoured) must lie within ``FIT_TOLERANCE_M`` of the fitted conic or the map
fails closed.  Anything the model does not define unambiguously is rejected
with an error, never substituted with a typed course-specific value
(AGENTS.md §3.9).  The two values the CAD cannot supply are explicit CLI
arguments: the launch heading (the mark is an undirected disc) and the window
clearance radius, which has no geometric default per ``contracts/course.md``
— without a reviewed budget it is emitted as ``null`` so a conforming
consumer refuses window traversal.

The self-check compares the model's *measured* geometry against the nominal
values in ``contracts/course.md``'s geometry table, and always runs the
schema's structural validation before the output file is written: a failing
check leaves any previous map on disk untouched.
"""

from __future__ import annotations

import argparse
import bisect
import math
import re
import shlex
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STEP = ROOT / "dronehacks-course-v3.step"
DEFAULT_OUTPUT = ROOT / "contracts/course_v3_layout.yaml"
SLICE_HEIGHT = 0.8  # contracts/course.md map-file format: sliced at 0.8 m
# contracts/course.md: the course frame's origin is at the floor centre, so
# the floor is z = 0. A solid at the slice height must stand on it.
FLOOR_HEIGHT = 0.0
WINDOW_SUFFIX = "(window)"

# Modelling tolerances (CAD precision, not course dimensions).
POINT_TOLERANCE_M = 1e-6  # coincident vertices, curve ends, heights
# A circle or ellipse fitted to sampled B-spline edges must pass within this
# distance of every sample; a worse fit means the curve is not that conic.
FIT_TOLERANCE_M = 1e-3
CONIC_SAMPLES_PER_TURN = 64
BSPLINE_SAMPLES_PER_SPAN = 4

# contracts/course.md's geometry table (organisers rows), course v3. The
# self-check compares the model's measured geometry against these nominal
# contract values with these tolerances. A layout change that moves a
# measured value outside a tolerance means the model or the contract row is
# stale — both are human decisions, so the check fails closed until one is
# updated (course.md: a surveyed row supersedes and gets a CHANGELOG entry).
NOMINAL = {
    "net_side_m": (6.096, 0.005),
    # Geometry table, Walls row: "1.727 m (68 in) tall, 0.102 m thick; the two
    # walls added in the 2026-11-07 layout are 0.100 m and 0.350 m thick".
    # Every wall must match one of the listed thicknesses.
    "wall_thickness_m": ((0.102, 0.100, 0.350), 0.001),
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


# --------------------------------------------------------------------------
# ISO 10303-21 (STEP Part 21) reading

class Ref(int):
    """An entity instance reference, ``#123``."""

    __slots__ = ()

    def __repr__(self) -> str:
        return f"#{int(self)}"


class Enum(str):
    """An enumeration value without its dots: ``.T.`` is ``Enum('T')``."""

    __slots__ = ()


class Typed(tuple):
    """A typed parameter ``(keyword, args)``, e.g. ``LENGTH_MEASURE(1.E-05)``."""

    __slots__ = ()


class _Derived:
    __slots__ = ()

    def __repr__(self) -> str:
        return "*"


DERIVED = _Derived()  # the ``*`` (derived attribute) marker


class Entity:
    """One DATA-section instance.

    ``type`` is the keyword of a simple instance (``None`` for a complex
    instance) and ``args`` its parameters; ``parts`` maps every keyword of the
    instance to its parameters, so a complex instance is read part by part.
    """

    __slots__ = ("id", "type", "args", "parts")

    def __init__(self, number: int, kind: str | None, args: list | None, parts: dict[str, list]):
        self.id, self.type, self.args, self.parts = number, kind, args, parts

    @property
    def kind(self) -> str:
        return self.type or "(" + " ".join(self.parts) + ")"

    def __repr__(self) -> str:
        return f"#{self.id}={self.kind}"


_TOKEN = re.compile(
    r"""\s*(?:
        '((?:[^']|'')*)'                                # 1 string
      | \#(\d+)                                         # 2 entity reference
      | \.([A-Z_][A-Z0-9_]*)\.                          # 3 enumeration
      | ([+-]?(?:\d+\.?\d*|\.\d+)(?:[Ee][+-]?\d+)?)     # 4 real or integer
      | (!?[A-Z_][A-Z0-9_]*)                            # 5 keyword
      | ([()=,;$*])                                     # 6 punctuation
      | /\*.*?\*/                                       # comment (skipped)
    )""",
    re.VERBOSE | re.DOTALL,
)
_DATA_SECTION = re.compile(r"(?:^|;)(?:\s|/\*.*?\*/)*DATA\s*;", re.DOTALL)


def _tokens(text: str, start: int) -> list[tuple[int, str]]:
    """Tokens of the DATA section from ``start`` up to its ENDSEC keyword."""
    tokens: list[tuple[int, str]] = []
    match = _TOKEN.match
    position = start
    while True:
        found = match(text, position)
        if found is None:
            raise StepError(f"cannot read STEP data near {text[position:position + 60].strip()!r}")
        position = found.end()
        kind = found.lastindex
        if kind is None:
            continue  # a comment
        value = found.group(kind)
        if kind == 5 and value == "ENDSEC":
            return tokens
        tokens.append((kind, value))


def _parse_instances(tokens: list[tuple[int, str]]) -> dict[int, Entity]:
    index = 0

    def take(expected: str) -> None:
        nonlocal index
        if tokens[index] != (6, expected):  # punctuation, never a string's text
            raise StepError(f"malformed STEP data: expected {expected!r}, found {tokens[index][1]!r}")
        index += 1

    def parameters() -> list:
        """Values up to the matching ')' (the '(' has been consumed)."""
        nonlocal index
        values: list = []
        if tokens[index] == (6, ")"):
            index += 1
            return values
        while True:
            values.append(value())
            kind, text = tokens[index]
            index += 1
            if text == ")" and kind == 6:
                return values
            if text != "," or kind != 6:
                raise StepError(f"malformed STEP data: expected ',' or ')', found {text!r}")

    def value() -> object:
        nonlocal index
        kind, text = tokens[index]
        index += 1
        if kind == 2:
            return Ref(text)
        if kind == 4:
            return float(text) if any(mark in text for mark in ".Ee") else int(text)
        if kind == 1:
            return text.replace("''", "'")
        if kind == 3:
            return Enum(text)
        if kind == 5:
            take("(")
            return Typed((text, parameters()))
        if text == "(":
            return parameters()
        if text == "$":
            return None
        if text == "*":
            return DERIVED
        raise StepError(f"malformed STEP data: unexpected {text!r}")

    entities: dict[int, Entity] = {}
    try:
        while index < len(tokens):
            kind, text = tokens[index]
            index += 1
            if kind != 2:
                raise StepError(f"malformed STEP data: expected an entity number, found {text!r}")
            number = int(text)
            if number in entities:
                raise StepError(f"entity #{number} is defined twice")
            take("=")
            if tokens[index] == (6, "("):  # complex instance: (A(...) B(...) ...)
                index += 1
                parts: dict[str, list] = {}
                while tokens[index] != (6, ")"):
                    keyword_kind, keyword = tokens[index]
                    index += 1
                    if keyword_kind != 5:
                        raise StepError(f"malformed STEP data: #{number} has a complex part {keyword!r}")
                    take("(")
                    parts[keyword] = parameters()
                index += 1
                take(";")
                entities[number] = Entity(number, None, None, parts)
            else:
                keyword_kind, keyword = tokens[index]
                index += 1
                if keyword_kind != 5:
                    raise StepError(f"malformed STEP data: #{number} has no entity type")
                take("(")
                args = parameters()
                take(";")
                entities[number] = Entity(number, keyword, args, {keyword: args})
    except IndexError:
        raise StepError("STEP data section ends inside an entity") from None
    return entities


def check_length_unit(entities: dict[int, Entity]) -> None:
    """Every length unit must be the plain SI metre (no prefix, no conversion)."""
    units = [entity for entity in entities.values() if "LENGTH_UNIT" in entity.parts]
    if not units:
        raise StepError("STEP length unit is not metres: the model declares no length unit")
    for unit in units:
        si = unit.parts.get("SI_UNIT")
        if si is None or len(si) != 2 or si[0] is not None or si[1] != "METRE":
            raise StepError(f"STEP length unit is not metres (#{unit.id} is {unit.kind})")


def parse_entities(path: Path) -> dict[int, Entity]:
    text = Path(path).read_text(encoding="utf-8")
    data = _DATA_SECTION.search(text)
    if data is None:
        raise StepError(f"no STEP DATA section found in {path}")
    entities = _parse_instances(_tokens(text, data.end()))
    if not entities:
        raise StepError(f"no STEP entities found in {path}")
    check_length_unit(entities)
    return entities


def refs(value: object) -> list[int]:
    """Every entity reference inside a parameter value, in order."""
    if isinstance(value, Ref):
        return [int(value)]
    if isinstance(value, list):
        return [reference for item in value for reference in refs(item)]
    if isinstance(value, Typed):
        return refs(value[1])
    return []


def reachable_entities(root: int, entities: dict[int, Entity]) -> set[int]:
    seen: set[int] = set()
    pending = [root]
    while pending:
        number = pending.pop()
        if number in seen:
            continue
        entity = entities.get(number)
        if entity is None:
            raise StepError(f"entity #{number} is referenced but undefined")
        seen.add(number)
        for args in entity.parts.values():
            pending.extend(reference for reference in refs(args) if reference not in seen)
    return seen


def _entity(entities: dict[int, Entity], reference: object, *kinds: str) -> Entity:
    if not isinstance(reference, Ref):
        raise StepError(f"expected an entity reference, found {reference!r}")
    entity = entities.get(int(reference))
    if entity is None:
        raise StepError(f"entity #{int(reference)} is referenced but undefined")
    if kinds and entity.type not in kinds:
        raise StepError(f"#{entity.id} is {entity.kind}, expected {' or '.join(kinds)}")
    return entity


def _flag(value: object, where: str) -> bool:
    if value == "T" and isinstance(value, Enum):
        return True
    if value == "F" and isinstance(value, Enum):
        return False
    raise StepError(f"{where}: expected .T. or .F., found {value!r}")


# --------------------------------------------------------------------------
# Vectors and placements

Vec = tuple  # (x, y, z)


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add_scaled(a: Vec, b: Vec, scale: float) -> Vec:
    return (a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale)


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(vector: Vec, where: str) -> Vec:
    length = math.sqrt(_dot(vector, vector))
    if not math.isfinite(length) or length <= 1e-12:
        raise StepError(f"{where} has no usable direction")
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _coordinates(entity: Entity, where: str) -> Vec:
    values = entity.args[1] if entity.args and len(entity.args) > 1 else None
    if (not isinstance(values, list) or len(values) != 3
            or any(type(value) not in (int, float) for value in values)):
        raise StepError(f"{where} #{entity.id} is not a 3-D coordinate")
    if not all(math.isfinite(value) for value in values):
        raise StepError(f"{where} #{entity.id} is not finite")
    return (float(values[0]), float(values[1]), float(values[2]))


def point(entities: dict[int, Entity], reference: object) -> Vec:
    return _coordinates(_entity(entities, reference, "CARTESIAN_POINT"), "CARTESIAN_POINT")


def direction(entities: dict[int, Entity], reference: object) -> Vec:
    entity = _entity(entities, reference, "DIRECTION")
    return _unit(_coordinates(entity, "DIRECTION"), f"DIRECTION #{entity.id}")


def placement(entities: dict[int, Entity], reference: object) -> tuple[Vec, Vec, Vec, Vec]:
    """``(origin, x, y, z)`` of an AXIS2_PLACEMENT_3D, with STEP's defaults."""
    entity = _entity(entities, reference, "AXIS2_PLACEMENT_3D")
    args = entity.args
    origin = point(entities, args[1])
    z_axis = direction(entities, args[2]) if len(args) > 2 and args[2] is not None else (0.0, 0.0, 1.0)
    x_hint = direction(entities, args[3]) if len(args) > 3 and args[3] is not None else (1.0, 0.0, 0.0)
    if abs(_dot(x_hint, z_axis)) > 1.0 - 1e-9:
        if len(args) > 3 and args[3] is not None:
            raise StepError(f"placement #{entity.id} has a reference direction along its axis")
        x_hint = (0.0, 1.0, 0.0) if abs(z_axis[0]) > 0.5 else (1.0, 0.0, 0.0)
    x_axis = _unit(_add_scaled(x_hint, z_axis, -_dot(x_hint, z_axis)), f"placement #{entity.id}")
    return origin, x_axis, _cross(z_axis, x_axis), z_axis


# --------------------------------------------------------------------------
# Product structure: which solid is which part

_CAD_EXTENSION = re.compile(r"\.(?:step|stp|sldprt|sldasm)$", re.IGNORECASE)
_VIRTUAL_COMPONENT = re.compile(r"(.+)\^[^^]+")


def part_label(raw: object) -> str:
    """The part name of a PRODUCT (or shape representation) name.

    CAD exporters decorate part names: SolidWorks writes the source file name
    (``'Barrier 1.step'``) and names a virtual component, a part saved inside
    its assembly, ``'<part>^<assembly name>'``. A trailing CAD file extension
    (``.step``, ``.stp``, ``.sldprt``, ``.sldasm``, any case) and a
    ``^<assembly>`` suffix are removed; anything else is kept as written, so
    Onshape names (``'Barrier 1'``) come out unchanged. The SITL uses the
    same rule, so both tools name every part alike.
    """
    name = _CAD_EXTENSION.sub("", str(raw or "").strip()).strip()
    virtual = _VIRTUAL_COMPONENT.fullmatch(name)
    if virtual:
        name = _CAD_EXTENSION.sub("", virtual.group(1).strip()).strip()
    return name


def body_labels(part: str, bodies: list[str]) -> list[str]:
    """Solid names of one part: the part name for a single body, else
    ``'<part> / <body name>'`` per body (1-based indices when a body name is
    empty or repeated). A multi-body part is never merged into one solid."""
    if len(bodies) == 1:
        return [part]
    names = [str(body or "").strip() for body in bodies]
    if not all(names) or len(set(names)) != len(names):
        names = [str(number + 1) for number in range(len(bodies))]
    return [f"{part} / {name}" for name in names]


def _check_identity_placements(entities: dict[int, Entity]) -> None:
    """Assembly transforms are not applied, so each must be the identity: the
    exporters write every part already placed in the course frame."""
    for entity in entities.values():
        relation = entity.parts.get("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        if relation is None:
            continue
        operator = _entity(entities, relation[0] if relation else None)
        if operator.type != "ITEM_DEFINED_TRANSFORMATION":
            raise StepError(f"#{entity.id} uses an unsupported part transformation {operator.kind}")
        first = placement(entities, operator.args[2])
        second = placement(entities, operator.args[3])
        if any(math.dist(a, b) > 1e-9 for a, b in zip(first, second)):
            raise StepError(f"part transformation #{operator.id} is not the identity; "
                            "only models with every part placed in the course frame are supported")


def part_solids(entities: dict[int, Entity]) -> dict[str, int]:
    """Solid name -> MANIFOLD_SOLID_BREP entity number, in natural name order.

    The name comes from the product that owns the solid (PRODUCT ->
    PRODUCT_DEFINITION -> PRODUCT_DEFINITION_SHAPE ->
    SHAPE_DEFINITION_REPRESENTATION -> shape representation -> its B-rep
    representation), via ``part_label`` and ``body_labels``. Every solid in
    the file must belong to exactly one named part.
    """
    _check_identity_placements(entities)
    links: dict[int, list[int]] = {}
    for entity in entities.values():
        if entity.type == "SHAPE_REPRESENTATION_RELATIONSHIP":  # simple: no transformation
            linked = refs(entity.args[2:4])
            if len(linked) != 2:
                raise StepError(f"#{entity.id} does not relate two representations")
            first, second = linked
            links.setdefault(first, []).append(second)
            links.setdefault(second, []).append(first)

    parts: list[tuple[int, str, list[int]]] = []
    for entity in entities.values():
        if entity.type != "SHAPE_DEFINITION_REPRESENTATION":
            continue
        definition = _entity(entities, entity.args[0])
        if definition.type != "PRODUCT_DEFINITION_SHAPE":
            continue
        product_definition = _entity(entities, definition.args[2], "PRODUCT_DEFINITION")
        formation = _entity(entities, product_definition.args[2])
        product = _entity(entities, formation.args[2], "PRODUCT")
        representation = _entity(entities, entity.args[1])
        candidates = [product.args[1], product.args[0], representation.args[0]]
        label = next((part_label(name) for name in candidates if isinstance(name, str) and part_label(name)),
                     f"#{product_definition.id}")
        breps: list[int] = []
        brep_reps = sorted({representation.id, *links.get(representation.id, [])})
        for number in brep_reps:
            rep = entities.get(number)
            if rep is None:
                raise StepError(f"entity #{number} is referenced but undefined")
            if rep.type != "ADVANCED_BREP_SHAPE_REPRESENTATION":
                continue
            for item in rep.args[1]:
                solid = _entity(entities, item)
                if solid.type == "MANIFOLD_SOLID_BREP":
                    breps.append(solid.id)
                elif solid.type != "AXIS2_PLACEMENT_3D":
                    raise StepError(f"{label!r} contains an unsupported solid {solid.kind} (#{solid.id})")
        if breps:
            parts.append((representation.id, label, breps))

    result: dict[str, int] = {}
    for _, label, breps in sorted(parts):
        bodies = [entities[number].args[0] for number in breps]
        for name, number in zip(body_labels(label, bodies), breps):
            if name in result:
                raise StepError(f"two solids are both named {name!r}")
            result[name] = number
    claimed = set(result.values())
    for entity in entities.values():
        if entity.type == "MANIFOLD_SOLID_BREP" and entity.id not in claimed:
            raise StepError(f"solid #{entity.id} ({entity.args[0]!r}) does not belong to a named part")
    # Name order, not entity order: exporters number entities arbitrarily, so a
    # re-export of the same model must not reorder the map's segments.
    return dict(sorted(result.items(), key=lambda item: natural_key(item[0])))


# --------------------------------------------------------------------------
# Solid geometry: vertices, sampled edges, planar faces

class Conic:
    """A circle or ellipse: ``centre + a cos(t) x + b sin(t) y``."""

    __slots__ = ("kind", "centre", "x", "y", "axis", "a", "b")

    def __init__(self, kind: str, frame: tuple[Vec, Vec, Vec, Vec], a: float, b: float):
        self.kind = kind
        self.centre, self.x, self.y, self.axis = frame
        self.a, self.b = a, b

    def at(self, t: float) -> Vec:
        return _add_scaled(_add_scaled(self.centre, self.x, self.a * math.cos(t)), self.y, self.b * math.sin(t))

    def parameter(self, p: Vec) -> float:
        offset = _sub(p, self.centre)
        return math.atan2(_dot(offset, self.y) / self.b, _dot(offset, self.x) / self.a)


class Face:
    """A face's surface type, its placement if planar, and its loops.

    ``loops`` holds each loop's dense edge samples; ``loop_nodes`` the points a
    conic is fitted through (a B-spline edge's points at its knots, where CAD
    spline forms of conics interpolate the true curve; every sample of a
    conic edge); ``loop_kinds`` the curve kinds of each loop's edges.
    """

    __slots__ = ("surface", "frame", "loops", "loop_nodes", "loop_kinds")

    def __init__(self, surface: str, frame, loops: list[list[Vec]], loop_nodes: list[list[Vec]], loop_kinds: list[set[str]]):
        self.surface, self.frame, self.loops, self.loop_nodes, self.loop_kinds = surface, frame, loops, loop_nodes, loop_kinds


class Solid:
    """One manifold solid: its boundary points, conic edges and faces.

    ``points`` are the vertices plus samples along every edge (the solid's
    true boundary curves, never construction points such as a plane's
    origin); ``z_min``/``z_max`` also include B-spline surface control points,
    which bound those surfaces, so a curved top above its edges still counts.
    """

    __slots__ = ("name", "points", "conics", "faces", "z_min", "z_max")

    def __init__(self, name: str, points: list[Vec], conics: list[Conic], faces: list[Face], z_values: list[float]):
        self.name, self.points, self.conics, self.faces = name, points, conics, faces
        self.z_min, self.z_max = min(z_values), max(z_values)


def bspline_curve(entities: dict[int, Entity], curve: Entity) -> tuple[int, list[tuple[float, ...]], list[float]]:
    """``(degree, homogeneous control points, expanded knots)`` of a B-spline
    curve with knots, plain or complex/rational (weights become the fourth
    homogeneous coordinate). Other B-spline forms are rejected."""
    weights = None
    if curve.type == "B_SPLINE_CURVE_WITH_KNOTS":
        args = curve.args
        degree, control, multiplicities, knots = args[1], args[2], args[6], args[7]
    elif curve.type is None and "B_SPLINE_CURVE" in curve.parts and "B_SPLINE_CURVE_WITH_KNOTS" in curve.parts:
        degree, control = curve.parts["B_SPLINE_CURVE"][0], curve.parts["B_SPLINE_CURVE"][1]
        multiplicities, knots = curve.parts["B_SPLINE_CURVE_WITH_KNOTS"][0], curve.parts["B_SPLINE_CURVE_WITH_KNOTS"][1]
        if "RATIONAL_B_SPLINE_CURVE" in curve.parts:
            weights = curve.parts["RATIONAL_B_SPLINE_CURVE"][0]
        elif any("RATIONAL" in part for part in curve.parts):
            raise StepError(f"#{curve.id}: unsupported rational B-spline form {curve.kind}")
    else:
        raise StepError(f"#{curve.id}: unsupported B-spline curve {curve.kind}")
    where = f"B-spline curve #{curve.id}"
    if type(degree) is not int or degree < 1 or not isinstance(control, list) or not isinstance(knots, list):
        raise StepError(f"{where} is malformed")
    if (not isinstance(multiplicities, list) or len(multiplicities) != len(knots)
            or any(type(count) is not int or count < 1 for count in multiplicities)):
        raise StepError(f"{where} has malformed knot multiplicities")
    expanded = [float(knot) for knot, count in zip(knots, multiplicities) for _ in range(count)]
    if len(expanded) != len(control) + degree + 1 or any(b < a for a, b in zip(expanded, expanded[1:])):
        raise StepError(f"{where} has an inconsistent knot vector")
    if not all(math.isfinite(knot) for knot in expanded) or expanded[len(control)] <= expanded[degree]:
        raise StepError(f"{where} has an empty or non-finite parameter range")
    points = [point(entities, reference) for reference in control]
    if weights is None:
        homogeneous = [(p[0], p[1], p[2], 1.0) for p in points]
    else:
        if (not isinstance(weights, list) or len(weights) != len(points)
                or any(type(w) not in (int, float) or not math.isfinite(w) or w <= 0.0 for w in weights)):
            raise StepError(f"{where} has invalid weights")
        homogeneous = [(p[0] * w, p[1] * w, p[2] * w, float(w)) for p, w in zip(points, weights)]
    return degree, homogeneous, expanded


def de_boor(degree: int, control: list[tuple[float, ...]], knots: list[float], t: float) -> Vec:
    """One point of a (homogeneous) B-spline curve by de Boor's algorithm."""
    span = bisect.bisect_right(knots, t) - 1
    span = min(max(span, degree), len(control) - 1)
    d = [list(control[j + span - degree]) for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            low = knots[j + span - degree]
            high = knots[j + 1 + span - r]
            alpha = 0.0 if high == low else (t - low) / (high - low)
            d[j] = [(1.0 - alpha) * a + alpha * b for a, b in zip(d[j - 1], d[j])]
    x, y, z, w = d[degree]
    return (x / w, y / w, z / w)


def sample_bspline(degree: int, control: list[tuple[float, ...]], knots: list[float]) -> list[Vec]:
    start, end = knots[degree], knots[len(control)]
    breaks = sorted({knot for knot in knots if start <= knot <= end})
    samples: list[Vec] = []
    for low, high in zip(breaks, breaks[1:]):
        for step in range(BSPLINE_SAMPLES_PER_SPAN):
            samples.append(de_boor(degree, control, knots, low + (high - low) * step / BSPLINE_SAMPLES_PER_SPAN))
    samples.append(de_boor(degree, control, knots, end))
    return samples


def _edge_geometry(entities: dict[int, Entity], reference: object, name: str) -> tuple[list[Vec], list[Vec], str, Conic | None]:
    """``(samples, fit nodes, curve kind, conic or None)`` of an EDGE_CURVE.

    Samples run from the start vertex to the end vertex. The fit nodes are the
    points a conic is fitted through: a B-spline edge's points at its distinct
    knots (a subset of its samples), otherwise the samples themselves.
    """
    edge = _entity(entities, reference, "EDGE_CURVE")
    start = point(entities, _entity(entities, edge.args[1], "VERTEX_POINT").args[1])
    end = point(entities, _entity(entities, edge.args[2], "VERTEX_POINT").args[1])
    same_sense = _flag(edge.args[4], f"edge #{edge.id}")
    curve = _entity(entities, edge.args[3])
    while curve.type in ("SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"):
        curve = _entity(entities, curve.args[1])  # its 3-D curve
    closed = math.dist(start, end) <= POINT_TOLERANCE_M
    if curve.type == "LINE":
        if closed:
            raise StepError(f"{name}: line edge #{edge.id} has zero length")
        # The midpoint matters: an edge crossing a wall's thickness (a window
        # opening's sill or head) has both vertices on the wall's faces and
        # only its interior inside the footprint.
        middle = tuple((a + b) / 2.0 for a, b in zip(start, end))
        return [start, middle, end], [start, end], "line", None
    if curve.type in ("CIRCLE", "ELLIPSE"):
        frame = placement(entities, curve.args[1])
        sizes = curve.args[2:4] if curve.type == "ELLIPSE" else curve.args[2:3] * 2
        if any(type(size) not in (int, float) or not math.isfinite(size) or size <= 0.0 for size in sizes):
            raise StepError(f"{name}: {curve.type} #{curve.id} has no positive finite size")
        conic = Conic(curve.type.lower(), frame, float(sizes[0]), float(sizes[1]))
        first, last = (start, end) if same_sense else (end, start)
        t0 = conic.parameter(first)
        sweep = 2.0 * math.pi if closed else (conic.parameter(last) - t0) % (2.0 * math.pi)
        count = max(2, math.ceil(CONIC_SAMPLES_PER_TURN * sweep / (2.0 * math.pi)))
        samples = [conic.at(t0 + sweep * index / count) for index in range(count + 1)]
        samples = samples if same_sense else samples[::-1]
        return samples, samples, conic.kind, conic
    if "B_SPLINE_CURVE_WITH_KNOTS" in curve.parts:
        samples = sample_bspline(*bspline_curve(entities, curve))
        if not same_sense:
            samples = samples[::-1]
        if not closed and math.dist(samples[0], end) <= POINT_TOLERANCE_M and math.dist(samples[-1], start) <= POINT_TOLERANCE_M:
            samples = samples[::-1]
        # The whole curve must be the edge: trimmed B-spline edges are unsupported.
        if math.dist(samples[0], start) > POINT_TOLERANCE_M or math.dist(samples[-1], end) > POINT_TOLERANCE_M:
            raise StepError(f"{name}: B-spline edge #{edge.id} does not run between its vertices; "
                            "trimmed B-spline edges are unsupported")
        # sample_bspline puts a sample on every distinct knot, one every
        # BSPLINE_SAMPLES_PER_SPAN samples (so in either direction).
        return samples, samples[::BSPLINE_SAMPLES_PER_SPAN], "bspline", None
    raise StepError(f"{name}: edge #{edge.id} uses an unsupported curve {curve.kind}")


def solid_geometry(entities: dict[int, Entity], brep: int, name: str) -> Solid:
    solid = _entity(entities, Ref(brep), "MANIFOLD_SOLID_BREP")
    shell = _entity(entities, solid.args[1], "CLOSED_SHELL")
    edges: dict[int, tuple[list[Vec], list[Vec], str, Conic | None]] = {}
    faces: list[Face] = []
    points: list[Vec] = []
    z_values: list[float] = []
    for face_reference in shell.args[1]:
        face = _entity(entities, face_reference, "ADVANCED_FACE", "FACE_SURFACE")
        surface = _entity(entities, face.args[2])
        frame = placement(entities, surface.args[1]) if surface.type == "PLANE" else None
        if any(part.startswith("B_SPLINE_SURFACE") for part in surface.parts):
            # The control net bounds the surface (convex-hull property).
            z_values.extend(point(entities, Ref(number))[2] for number in reachable_entities(surface.id, entities)
                            if entities[number].type == "CARTESIAN_POINT")
        loops: list[list[Vec]] = []
        loop_nodes: list[list[Vec]] = []
        loop_kinds: list[set[str]] = []
        for bound_reference in face.args[1]:
            bound = _entity(entities, bound_reference, "FACE_OUTER_BOUND", "FACE_BOUND")
            loop = _entity(entities, bound.args[1])
            samples: list[Vec] = []
            nodes: list[Vec] = []
            kinds: set[str] = set()
            if loop.type == "VERTEX_LOOP":
                samples.append(point(entities, _entity(entities, loop.args[1], "VERTEX_POINT").args[1]))
                nodes.extend(samples)
                kinds.add("vertex")
            elif loop.type == "EDGE_LOOP":
                for oriented_reference in loop.args[1]:
                    oriented = _entity(entities, oriented_reference, "ORIENTED_EDGE")
                    number = int(oriented.args[3])
                    if number not in edges:
                        edges[number] = _edge_geometry(entities, oriented.args[3], name)
                    edge_samples, edge_nodes, kind, _ = edges[number]
                    forward = _flag(oriented.args[4], f"oriented edge #{oriented.id}")
                    samples.extend(edge_samples if forward else edge_samples[::-1])
                    nodes.extend(edge_nodes if forward else edge_nodes[::-1])
                    kinds.add(kind)
            else:
                raise StepError(f"{name}: face #{face.id} has an unsupported loop {loop.kind}")
            points.extend(samples)
            loops.append(samples)
            loop_nodes.append(nodes)
            loop_kinds.append(kinds)
        faces.append(Face(surface.kind, frame, loops, loop_nodes, loop_kinds))
    if not points:
        raise StepError(f"{name} (solid #{brep}) has no boundary geometry")
    conics = [conic for _, _, _, conic in edges.values() if conic is not None]
    return Solid(name, points, conics, faces, z_values + [p[2] for p in points])


# --------------------------------------------------------------------------
# Conic fits for curves exported as B-splines

def _solve(matrix: list[list[float]], rhs: list[float], what: str) -> list[float]:
    """Gaussian elimination with partial pivoting for a small dense system."""
    size = len(rhs)
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    if not all(math.isfinite(value) for row in rows for value in row):
        raise StepError(f"{what}: fit input is not finite")
    scale = max(abs(value) for row in rows for value in row[:-1]) or 1.0
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) <= 1e-13 * scale:
            raise StepError(f"{what}: degenerate geometry, no unique fit")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for row in range(column + 1, size):
            factor = rows[row][column] / rows[column][column]
            for index in range(column, size + 1):
                rows[row][index] -= factor * rows[column][index]
    solution = [0.0] * size
    for row in range(size - 1, -1, -1):
        solution[row] = (rows[row][size] - sum(rows[row][index] * solution[index] for index in range(row + 1, size))) / rows[row][row]
    return solution


def _least_squares(features: list[list[float]], targets: list[float], what: str) -> list[float]:
    size = len(features[0])
    normal = [[sum(f[i] * f[j] for f in features) for j in range(size)] for i in range(size)]
    rhs = [sum(f[i] * y for f, y in zip(features, targets)) for i in range(size)]
    return _solve(normal, rhs, what)


def _centred(points: list[tuple[float, float]]) -> tuple[tuple[float, float], list[tuple[float, float]]]:
    """Points relative to their centroid (computed without overflow)."""
    base = points[0]
    offsets = [(u - base[0], v - base[1]) for u, v in points]
    mean = (sum(u for u, _ in offsets) / len(offsets), sum(v for _, v in offsets) / len(offsets))
    centre = (base[0] + mean[0], base[1] + mean[1])
    return centre, [(u - mean[0], v - mean[1]) for u, v in offsets]


def fit_circle(points: list[tuple[float, float]], what: str,
               check: list[tuple[float, float]] | None = None) -> tuple[tuple[float, float], float, float]:
    """Least-squares circle through 2-D ``points``: ``(centre, radius, worst
    deviation)``, the deviation measured over ``check`` (default ``points``)."""
    if len(set(points)) < 3:
        raise StepError(f"{what}: too few points to fit a circle")
    origin, local = _centred(points)
    d, e, f = _least_squares([[u, v, 1.0] for u, v in local], [-(u * u + v * v) for u, v in local], what)
    cu, cv = -d / 2.0, -e / 2.0
    squared = cu * cu + cv * cv - f
    if not math.isfinite(squared) or squared <= 0.0:
        raise StepError(f"{what}: points do not lie on a circle")
    radius = math.sqrt(squared)
    centre = (origin[0] + cu, origin[1] + cv)
    deviation = 0.0
    for u, v in (check or points):
        distance = abs(math.hypot(u - centre[0], v - centre[1]) - radius)
        deviation = distance if not distance <= deviation else deviation  # a NaN stays NaN
    return centre, radius, deviation


def fit_axis_aligned_ellipse(points: list[tuple[float, float]], what: str,
                             check: list[tuple[float, float]] | None = None) -> tuple[tuple[float, float], float, float, float]:
    """Least-squares ellipse with axes along u and v through 2-D ``points``:
    ``(centre, semi_u, semi_v, worst deviation)``, the deviation measured over
    ``check`` (default ``points``).

    The deviation is the first-order (Sampson) distance of each point from the
    fitted ellipse; a rotated or non-elliptical loop fits badly and is caught
    by the caller's tolerance.
    """
    if len(set(points)) < 4:
        raise StepError(f"{what}: too few points to fit an ellipse")
    origin, local = _centred(points)
    a, b, c, d = _least_squares([[u * u, v * v, u, v] for u, v in local], [1.0] * len(local), what)
    if not (a > 0.0 and b > 0.0):
        raise StepError(f"{what}: points do not lie on an ellipse")
    cu, cv = -c / (2.0 * a), -d / (2.0 * b)
    gain = 1.0 + c * c / (4.0 * a) + d * d / (4.0 * b)
    if not math.isfinite(gain) or gain <= 0.0:
        raise StepError(f"{what}: points do not lie on an ellipse")
    semi_u, semi_v = math.sqrt(gain / a), math.sqrt(gain / b)
    centre = (origin[0] + cu, origin[1] + cv)
    deviation = 0.0
    for u, v in (check or points):
        du, dv = u - centre[0], v - centre[1]
        nu, nv = du / semi_u, dv / semi_v  # products, not **: never an OverflowError
        value = nu * nu + nv * nv - 1.0
        gradient = 2.0 * math.hypot(nu / semi_u, nv / semi_v)
        distance = abs(value) / gradient if gradient > 0.0 else min(semi_u, semi_v)
        deviation = distance if not distance <= deviation else deviation  # a NaN stays NaN
    return centre, semi_u, semi_v, deviation


def _in_plane(points: list[Vec], origin: Vec, u_axis: Vec, v_axis: Vec) -> list[tuple[float, float]]:
    return [(_dot(_sub(p, origin), u_axis), _dot(_sub(p, origin), v_axis)) for p in points]


# --------------------------------------------------------------------------
# Footprints and goal geometry

def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) < 2:
        raise StepError("footprint has fewer than two distinct points")

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point_ in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point_) <= 1e-10:
            lower.pop()
        lower.append(point_)
    upper: list[tuple[float, float]] = []
    for point_ in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point_) <= 1e-10:
            upper.pop()
        upper.append(point_)
    return lower[:-1] + upper[:-1]


def _point_segment_distance(point_: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    length = math.dist(start, end)
    if length <= 0.0:
        return math.dist(point_, start)
    along = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
    offset = max(0.0, min(length, (point_[0] - start[0]) * along[0] + (point_[1] - start[1]) * along[1]))
    return math.dist(point_, (start[0] + along[0] * offset, start[1] + along[1] * offset))


def hull_and_interior(points: list[tuple[float, float, float]]) -> tuple[list[tuple[float, float]], list[tuple[float, float, float]]]:
    """Split a part's points into footprint-hull boundary and interior points."""
    hull = convex_hull([(p[0], p[1]) for p in points])
    boundary: list[tuple[float, float, float]] = []
    interior: list[tuple[float, float, float]] = []
    for p in points:
        distance = min(
            _point_segment_distance((p[0], p[1]), hull[index], hull[(index + 1) % len(hull)])
            for index in range(len(hull))
        )
        (boundary if distance <= 1e-9 else interior).append(p)
    return hull, interior


def segment_from_points(points: list[tuple[float, float, float]], name: str = "segment") -> dict[str, object]:
    """The wall segment (rectangle) of a floor-standing vertical extrusion.

    The segment is the footprint's bounding rectangle in the frame of its
    longest hull edge: start/end on the long axis, thickness across it.
    """
    hull = convex_hull([(p[0], p[1]) for p in points])
    bottom = min(p[2] for p in points)
    top = max(p[2] for p in points)
    if not bottom <= SLICE_HEIGHT <= top:
        raise StepError(f"{name} does not span the map slice height")
    if not math.isclose(bottom, FLOOR_HEIGHT, abs_tol=POINT_TOLERANCE_M):
        raise StepError(f"{name} reaches the slice height but does not stand on the floor; unsupported geometry")
    # Only vertical extrusions are supported; never project a sloping solid
    # into a footprint that claims to describe one horizontal flight slice.
    for level in (bottom, top):
        face = convex_hull([(p[0], p[1]) for p in points if math.isclose(p[2], level, abs_tol=1e-9)])
        if len(face) != len(hull) or any(math.dist(a, b) > 1e-8 for a, b in zip(face, hull)):
            raise StepError(f"{name}: unsupported nonvertical wall geometry")
    edges = [
        (math.dist(hull[index], hull[(index + 1) % len(hull)]), hull[index], hull[(index + 1) % len(hull)])
        for index in range(len(hull))
    ]
    length, edge_start, edge_end = max(edges)
    if length <= 0.0:
        raise StepError(f"{name}: segment footprint has zero length")
    along = ((edge_end[0] - edge_start[0]) / length, (edge_end[1] - edge_start[1]) / length)
    normal = (-along[1], along[0])
    along_values = [p[0] * along[0] + p[1] * along[1] for p in hull]
    normal_values = [p[0] * normal[0] + p[1] * normal[1] for p in hull]
    centre_normal = (min(normal_values) + max(normal_values)) / 2.0
    start = (min(along_values) * along[0] + centre_normal * normal[0], min(along_values) * along[1] + centre_normal * normal[1])
    end = (max(along_values) * along[0] + centre_normal * normal[0], max(along_values) * along[1] + centre_normal * normal[1])
    return {"start": start, "end": end, "thickness": max(normal_values) - min(normal_values)}


def round_value(value: float) -> float:
    rounded = round(value, 6)
    return 0.0 if abs(rounded) < 5e-7 else rounded


def rounded_point(p: tuple[float, float]) -> list[float]:
    return [round_value(p[0]), round_value(p[1])]


def unique_role(names: list[str], predicate, purpose: str) -> str:
    """The one part playing a given role, by predicate — never a typed label."""
    matches = [name for name in names if predicate(name)]
    if not matches:
        raise StepError(f"missing the part that provides the {purpose}")
    if len(matches) > 1:
        raise StepError(f"ambiguous parts provide the {purpose}: {matches}")
    return matches[0]


def natural_key(name: str) -> list:
    return [int(token) if token.isdigit() else token for token in re.split(r"(\d+)", name)]


def disc_circles(solid: Solid) -> list[dict[str, object]]:
    """Horizontal circles of a part: its CIRCLE edges, plus a fitted circle for
    every loop of a horizontal planar face made only of B-spline edges (an
    exporter's spline form of a circular outline). A spline loop that is not a
    circle within ``FIT_TOLERANCE_M`` fails closed."""
    circles = [{"centre": conic.centre, "radius": conic.a} for conic in solid.conics if conic.kind == "circle"]
    for face in solid.faces:
        if face.frame is None or abs(face.frame[3][2]) < 1.0 - 1e-9:
            continue
        origin, x_axis, y_axis, _ = face.frame
        for loop, nodes, kinds in zip(face.loops, face.loop_nodes, face.loop_kinds):
            if kinds != {"bspline"}:
                continue
            what = f"{solid.name} planar B-spline outline"
            samples = _in_plane(loop, origin, x_axis, y_axis)
            fit_points = _in_plane(nodes, origin, x_axis, y_axis)
            if len(set(fit_points)) < 6:
                fit_points = samples  # too few knots: fit the samples themselves
            (cu, cv), radius, deviation = fit_circle(fit_points, what, samples)
            if not deviation <= FIT_TOLERANCE_M:
                raise StepError(f"{what} is not a circle: worst deviation {deviation * 1000:.3f} mm")
            centre = _add_scaled(_add_scaled(origin, x_axis, cu), y_axis, cv)
            circles.append({"centre": centre, "radius": radius})
    return circles


def disc_geometry(name: str, circles: list[dict[str, object]]) -> tuple[tuple[float, float], float]:
    """Centre and radius of a disc part, from its circles (fail closed)."""
    if not circles:
        raise StepError(f"{name} has no circle geometry to derive a centre and radius")
    radii = [float(circle["radius"]) for circle in circles]
    if max(radii) - min(radii) > POINT_TOLERANCE_M:
        raise StepError(f"{name} has inconsistent circle radii: {sorted(set(radii))}")
    centres = [circle["centre"] for circle in circles]
    for index in range(2):
        span = [float(centre[index]) for centre in centres]
        if max(span) - min(span) > POINT_TOLERANCE_M:
            raise StepError(f"{name} has inconsistent circle centres: {centres}")
    mean = (sum(float(centre[0]) for centre in centres) / len(centres), sum(float(centre[1]) for centre in centres) / len(centres))
    return mean, sum(radii) / len(radii)


def is_disc(solid: Solid) -> bool:
    """A pure disc: some circles, all of a single radius."""
    radii = [float(circle["radius"]) for circle in disc_circles(solid)]
    return bool(radii) and max(radii) - min(radii) <= POINT_TOLERANCE_M


def tube_sections(name: str, solid: Solid) -> list[dict[str, object]]:
    """Outer cross-section of each planar end face of the tube.

    Each end face is a vertical plane; its outermost loop (ELLIPSE edges, or
    B-spline edges as SolidWorks writes them) is fitted with an ellipse whose
    axes are horizontal and vertical in that plane.
    """
    planar = [face for face in solid.faces if face.frame is not None]
    if not planar:
        raise StepError(f"{name} has no planar end faces to derive an axis and cross-section")
    if len(planar) != 2:
        raise StepError(f"{name} does not have exactly two planar end faces (found {len(planar)})")
    sections = []
    for face in planar:
        origin, _, _, normal = face.frame
        if abs(normal[2]) > 1e-6:
            raise StepError(f"{name} end-face normal is not horizontal; unsupported tube orientation")
        horizontal = _unit(_cross((0.0, 0.0, 1.0), normal), f"{name} end face")
        vertical = (0.0, 0.0, 1.0)
        loops = [(_in_plane(loop, origin, horizontal, vertical), _in_plane(nodes, origin, horizontal, vertical))
                 for loop, nodes in zip(face.loops, face.loop_nodes) if len(loop) > 1]
        if not loops:
            raise StepError(f"{name} end face has no boundary loop")

        def extent(loop: tuple[list[tuple[float, float]], list[tuple[float, float]]]) -> float:
            samples = loop[0]
            return max(u for u, _ in samples) - min(u for u, _ in samples) + max(v for _, v in samples) - min(v for _, v in samples)

        samples, fit_points = max(loops, key=extent)
        if len(set(fit_points)) < 6:
            fit_points = samples  # too few knots: fit the samples themselves
        what = f"{name} end-face outline"
        (cu, cv), semi_h, semi_v, deviation = fit_axis_aligned_ellipse(fit_points, what, samples)
        if not deviation <= FIT_TOLERANCE_M:
            raise StepError(f"{what} is not an ellipse with horizontal and vertical axes "
                            f"(worst deviation {deviation * 1000:.3f} mm); unsupported tube geometry")
        centre = _add_scaled(_add_scaled(origin, horizontal, cu), vertical, cv)
        sections.append({"centre": centre, "normal": normal, "width": 2.0 * semi_h, "height": 2.0 * semi_v})
    return sections


def tube_geometry(name: str, solid: Solid, pad_centre: tuple[float, float]) -> dict[str, object]:
    """Axis, width and height of the tube, from its CAD end faces.

    The two end faces carry the tube's outer cross-section; their centres
    give the entrance and exit and the tube axis. The traversal direction is
    derived, not typed: the drone exits toward the pad (course.md checkpoint
    3, "out of the south end" for this layout). Any other orientation, or an
    ambiguous relation to the pad, is rejected.
    """
    first, second = tube_sections(name, solid)
    if abs(_dot(first["normal"], second["normal"])) < 1.0 - 1e-6:
        raise StepError(f"{name} end-face normals are not parallel; unsupported tube geometry")
    for key in ("width", "height"):
        if abs(first[key] - second[key]) > FIT_TOLERANCE_M:
            raise StepError(f"{name} end faces have different {key}s; unsupported tube geometry")
    first_centre, second_centre = first["centre"], second["centre"]
    if math.isclose(first_centre[0], second_centre[0], abs_tol=1e-6) and math.isclose(first_centre[1], second_centre[1], abs_tol=1e-6):
        raise StepError(f"{name} end faces share one centre; no tube axis")
    if not math.isclose(first_centre[2], second_centre[2], abs_tol=POINT_TOLERANCE_M):
        raise StepError(f"{name} end-face centres differ in height; unsupported tube orientation")
    axis_length = math.dist((first_centre[0], first_centre[1]), (second_centre[0], second_centre[1]))
    if not axis_length > 0.0:
        raise StepError(f"{name} has a degenerate axis")
    along = ((second_centre[0] - first_centre[0]) / axis_length, (second_centre[1] - first_centre[1]) / axis_length)
    mid = ((first_centre[0] + second_centre[0]) / 2.0, (first_centre[1] + second_centre[1]) / 2.0)
    reach = (pad_centre[0] - mid[0]) * along[0] + (pad_centre[1] - mid[1]) * along[1]
    if abs(reach) < 0.05:
        raise StepError(f"{name} traversal direction is ambiguous: the pad is not clearly on one side of the tube")
    # reach > 0 puts the pad beyond the second face: exit there, enter at the first
    entry, exit_ = (first_centre, second_centre) if reach > 0.0 else (second_centre, first_centre)
    return {
        "centre": mid,
        "axis_start": (entry[0], entry[1]),
        "axis_end": (exit_[0], exit_[1]),
        "width": (first["width"] + second["width"]) / 2.0,
        "height": (first["height"] + second["height"]) / 2.0,
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
    levels = distinct_levels(p[2] for p in points)
    if len(levels) < 3:
        raise StepError(f"{name} has no distinct floor/opening/top z levels")
    floor, top = levels[0], levels[-1]
    opening_levels = [
        level
        for level in distinct_levels(p[2] for p in interior)
        if not (math.isclose(level, floor, abs_tol=1e-6) or math.isclose(level, top, abs_tol=1e-6))
    ]
    if len(opening_levels) != 2:
        raise StepError(f"{name} opening z levels are ambiguous: {opening_levels}")
    sill, head = opening_levels
    edges = [p for p in interior if math.isclose(p[2], sill, abs_tol=1e-6) or math.isclose(p[2], head, abs_tol=1e-6)]
    if not edges:
        raise StepError(f"{name} has no opening edge points at z {sill}/{head}")
    return sill, head, edges


def generate_map(step_path: Path, heading_deg: float, clearance: float | None) -> dict[str, object]:
    if not math.isfinite(heading_deg):
        raise StepError("--launch-heading must be a finite value from course.md's launch row")
    if clearance is not None and (not math.isfinite(clearance) or clearance <= 0.0):
        raise StepError("--window-clearance must be a positive finite value from a reviewed human budget")
    entities = parse_entities(step_path)
    try:
        return map_from_entities(entities, heading_deg, clearance)
    except (IndexError, TypeError, AttributeError, ArithmeticError) as error:
        # An entity with the wrong parameters for its type: malformed, so
        # fail closed like any other unusable model.
        raise StepError(f"malformed STEP model ({type(error).__name__}: {error})") from error


def map_from_entities(entities: dict[int, Entity], heading_deg: float, clearance: float | None) -> dict[str, object]:
    solids = {name: solid_geometry(entities, brep, name) for name, brep in part_solids(entities).items()}
    names = list(solids)

    tube_name = unique_role(names, lambda name: name.startswith("Tube oval"), "tunnel goal")
    launch_name = unique_role(names, lambda name: name.startswith("Launch mark"), "launch goal")
    boulder_names = sorted((name for name in names if name.startswith("Boulder ")), key=natural_key)
    # The pad part is the pure disc: a single distinct circle radius. The pad's
    # decorative rings are bands (inner and outer radii) and its cross has no
    # circles, so exactly one disc part must remain (checkpoint 5 lands inside
    # the outer ring, whose radius is the disc's).
    pad_discs = [name for name in names if name.startswith("Landing pad") and is_disc(solids[name])]
    if not pad_discs:
        raise StepError("missing the disc part that provides the pad goal")
    if len(pad_discs) > 1:
        raise StepError(f"ambiguous disc parts provide the pad goal: {sorted(pad_discs)}")
    pad_name = pad_discs[0]

    # The slice: every solid that reaches the slice height is an obstacle
    # there. The tube and boulders have their own footprints (goal and
    # obstacle discs); everything else must be a floor-standing vertical
    # extrusion — a net side by role, otherwise a wall — or the map fails.
    segments: list[dict[str, object]] = []
    segment_names: list[str] = []
    segment_kinds: list[str] = []
    raw_segments: dict[str, dict[str, object]] = {}
    footprint_roles = {tube_name, *boulder_names}
    for name, solid in solids.items():
        if name in footprint_roles or not solid.z_min <= SLICE_HEIGHT <= solid.z_max:
            continue
        if not min(p[2] for p in solid.points) <= SLICE_HEIGHT <= max(p[2] for p in solid.points):
            raise StepError(f"{name}: a curved surface may cross the slice height between its edges; unsupported geometry")
        segment = segment_from_points(solid.points, name)
        raw_segments[name] = segment
        segments.append({
            "start": rounded_point(segment["start"]),
            "end": rounded_point(segment["end"]),
            "thickness": round_value(segment["thickness"]),
        })
        segment_names.append(name)
        segment_kinds.append("net" if name.startswith("Net side") else "wall")

    window_name = unique_role(names, lambda name: name.endswith(WINDOW_SUFFIX), "window wall")
    if window_name not in raw_segments:
        raise StepError(f"{window_name!r} is not a wall at the map slice height")

    boulders: list[dict[str, object]] = []
    for name in boulder_names:
        centre, radius = disc_geometry(name, disc_circles(solids[name]))
        boulders.append({"centre": rounded_point(centre), "radius": round_value(radius)})

    pad_centre, pad_radius = disc_geometry(pad_name, disc_circles(solids[pad_name]))
    launch_centre, _launch_radius = disc_geometry(launch_name, disc_circles(solids[launch_name]))

    tube = tube_geometry(tube_name, solids[tube_name], pad_centre)

    sill, head, opening_edges = window_opening(window_name, solids[window_name].points)
    wall = raw_segments[window_name]
    wall_start, wall_end = wall["start"], wall["end"]
    wall_length = math.dist(wall_start, wall_end)
    along = ((wall_end[0] - wall_start[0]) / wall_length, (wall_end[1] - wall_start[1]) / wall_length)
    normal = (-along[1], along[0])
    projections = [p[0] * along[0] + p[1] * along[1] for p in opening_edges]
    normal_projections = [p[0] * normal[0] + p[1] * normal[1] for p in opening_edges]
    centre_along = (min(projections) + max(projections)) / 2.0
    centre_normal = (min(normal_projections) + max(normal_projections)) / 2.0
    window_centre = (
        centre_along * along[0] + centre_normal * normal[0],
        centre_along * along[1] + centre_normal * normal[1],
    )
    side = max(projections) - min(projections)
    if side <= 0.0:
        raise StepError(f"{window_name!r} opening has no measurable width")
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
            "wall_segment_index": segment_names.index(window_name),
            "axis_start": rounded_point(window_axis_start), "axis_end": rounded_point(window_axis_end),
            "side": round_value(side), "sill_height": round_value(sill), "head_height": round_value(head),
            "clearance_radius": None if clearance is None else round_value(clearance),
        }},
        {"name": "pad", "shape": "disc", "centre": rounded_point(pad_centre), "params": {"radius": round_value(pad_radius)}},
        {"name": "launch", "shape": "point", "centre": rounded_point(launch_centre), "params": {"heading_deg": round_value(heading_deg)}},
    ]
    return {
        "origin": [0.0, 0.0], "slice_height": SLICE_HEIGHT, "segments": segments, "boulders": boulders, "goals": goals,
        "_segment_names": segment_names, "_segment_kinds": segment_kinds, "_window_name": window_name,
    }


def scalar(value: object) -> str:
    if isinstance(value, float):
        text = f"{value:.6f}".rstrip("0").rstrip(".")
        return text + ".0" if "." not in text else text
    return str(value)


def point_yaml(p: list[float]) -> str:
    return f"[{scalar(p[0])}, {scalar(p[1])}]"


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
    def finite(value: object) -> bool:
        if isinstance(value, dict):
            return all(finite(item) for item in value.values())
        if isinstance(value, (list, tuple)):
            return all(finite(item) for item in value)
        return not isinstance(value, (int, float)) or math.isfinite(value)

    # Derived centres can overflow even when the source coordinates are finite.
    if not finite(data):
        return ["map geometry is not finite"]

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


def _segments_of_kind(data: dict[str, object], kind: str) -> list[dict[str, object]]:
    return [segment for segment, segment_kind in zip(data["segments"], data["_segment_kinds"]) if segment_kind == kind]


def _net_bbox(data: dict[str, object]) -> tuple[float, float] | None:
    net_points = [p for segment in _segments_of_kind(data, "net") for p in (segment["start"], segment["end"])]
    if not net_points:
        return None
    return (max(p[0] for p in net_points) - min(p[0] for p in net_points),
            max(p[1] for p in net_points) - min(p[1] for p in net_points))


def nominal_check(data: dict[str, object]) -> list[str]:
    """Compare measured geometry against course.md's course-v3 nominal table."""
    failures: list[str] = []

    def expect(measured: float, key: str, what: str) -> None:
        expected, tolerance = NOMINAL[key]
        if not math.isclose(measured, expected, abs_tol=tolerance):
            failures.append(f"{what} is {measured}, nominal is {expected} ± {tolerance}")

    net = _net_bbox(data)
    if net is None:
        failures.append("no net segments found")
    else:
        expect(net[0], "net_side_m", "net breadth")
        expect(net[1], "net_side_m", "net depth")

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
    walls = _segments_of_kind(data, "wall")
    if not walls:
        failures.append("no wall segments found")
    nominal_thicknesses, tolerance = NOMINAL["wall_thickness_m"]
    for segment in walls:
        thickness = segment["thickness"]
        if not any(math.isclose(thickness, expected, abs_tol=tolerance) for expected in nominal_thicknesses):
            failures.append(f"wall thickness is {thickness}, nominal is one of {list(nominal_thicknesses)} ± {tolerance}")
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
    net = _net_bbox(data) or (float("nan"), float("nan"))
    goals = {goal["name"]: goal for goal in data["goals"]}
    boulder_radii = sorted({boulder["radius"] for boulder in data["boulders"]})
    thicknesses = sorted({segment["thickness"] for segment in _segments_of_kind(data, "wall")})
    print("SELF-CHECK: PASS")
    print(f"  net bbox: {net[0]:.6f} m x {net[1]:.6f} m (nominal {NOMINAL['net_side_m'][0]} m ± {NOMINAL['net_side_m'][1]} m)")
    print(f"  segments: {data['_segment_kinds'].count('net')} net sides, {len(_segments_of_kind(data, 'wall'))} walls")
    if "tunnel" in goals:
        params = goals["tunnel"]["params"]
        print(f"  tunnel: width {params['width']} m, height {params['height']} m, axis {tuple(params['axis_start'])} -> {tuple(params['axis_end'])}")
    if "window" in goals:
        params = goals["window"]["params"]
        clearance = "null (unreviewed; traversal blocked)" if params["clearance_radius"] is None else f"{params['clearance_radius']} m"
        print(f"  window: linked to {data.get('_window_name', '?')!r}, side {params['side']} m, sill {params['sill_height']} m, head {params['head_height']} m")
        print(f"  window: clearance_radius {clearance}")
    if "pad" in goals:
        print(f"  pad: centre {tuple(goals['pad']['centre'])} m, radius {goals['pad']['params']['radius']} m")
    if "launch" in goals:
        print(f"  launch: {tuple(goals['launch']['centre'])} m, heading {goals['launch']['params']['heading_deg']} deg")
    print(f"  boulders: {len(data['boulders'])}, radii {boulder_radii} m")
    print(f"  wall thicknesses: {thicknesses} m")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--step", type=Path, default=DEFAULT_STEP, help="input AP214 STEP file")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="output course YAML")
    parser.add_argument("--launch-heading", type=float, required=True,
                        help="launch heading in degrees clockwise from course north, from course.md's launch row (the CAD mark is an undirected disc)")
    parser.add_argument("--window-clearance", type=float, default=None,
                        help="reviewed window robot-centre clearance radius in metres; omit to emit an explicit unreviewed null")
    parser.add_argument("--self-check", action="store_true",
                        help="validate the model's measured geometry against contracts/course.md's nominal table before writing")
    args = parser.parse_args(argv)

    def display(path: Path) -> str:
        """Repo-relative when possible, so the recorded command is portable."""
        try:
            return path.resolve().relative_to(ROOT).as_posix()
        except ValueError:
            return str(path)

    parts = [
        "python3 course/step_to_map.py",
        f"--step {shlex.quote(display(args.step))}",
        f"--output {shlex.quote(display(args.output))}",
        f"--launch-heading {args.launch_heading}",
    ]
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
        args.output.write_text(render_yaml(data, command), encoding="utf-8", newline="\n")
        print(f"generated {args.output}")
        if args.self_check:
            report_pass(data)
        return 0
    except (OSError, StepError, KeyError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
