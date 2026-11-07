"""Minimal reader for STEP (ISO-10303-21) assemblies as Onshape exports them, numpy only.

What it gives you
-----------------
:func:`read_solids` returns one :class:`Solid` per ``MANIFOLD_SOLID_BREP`` with

* its name (the Onshape part name) and colour (``STYLED_ITEM`` -> ``COLOUR_RGB``);
* every face as sampled boundary loops in world coordinates, the plane normal for planar faces,
  and the loops sorted largest first so ``loops[0]`` is the outer boundary and the rest are holes;
* the conic edges (circles and ellipses) with their centre, axes and semi-axes, which is how
  tubes and round rocks are recognised later;
* the assembly placement already applied (``ITEM_DEFINED_TRANSFORMATION`` chains), and the file's
  length unit converted to metres.

Edges on lines are exact; circle and ellipse arcs are sampled every 2*pi/64 rad.  Edges on any
other curve type (B-splines) fall back to their end points, which is enough for the axis-aligned
courses this is meant for; :attr:`Solid.unsupported_curves` counts them so a caller can warn.

What it is not
--------------
Not a general CAD kernel: no surface tessellation, no Booleans, no trimming of B-spline surfaces.
The point is to recover walls, openings, tubes and floor markings from a course model, not to
render arbitrary parts.
"""

from __future__ import annotations

import math
import re
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

ARC_STEP_RAD = 2.0 * math.pi / 64.0


# ------------------------------------------------------------------------------------- parsing


class Ref(int):
    """An entity reference ``#123``."""


@dataclass(frozen=True)
class Enum:
    """An enumeration value such as ``.T.`` or ``.METRE.``."""

    value: str


@dataclass(frozen=True)
class Typed:
    """A typed parameter such as ``LENGTH_MEASURE(1.E-8)``."""

    name: str
    args: Tuple[Any, ...]


DERIVED = "*"


@dataclass
class Entity:
    """One data-section instance.  Complex instances keep every part in ``parts``."""

    id: int
    type: str
    args: List[Any]
    parts: Dict[str, List[Any]] = field(default_factory=dict)


_TOKEN = re.compile(
    r"""
    (?P<ref>\#\d+)
  | (?P<str>'(?:[^']|'')*')
  | (?P<enum>\.[A-Za-z_][A-Za-z0-9_]*\.)
  | (?P<num>[+-]?(?:\d+\.?\d*|\.\d+)(?:[Ee][+-]?\d+)?)
  | (?P<kw>[A-Za-z_][A-Za-z0-9_]*)
  | (?P<punct>[(),$*=;])
  | (?P<ws>\s+)
    """,
    re.VERBOSE,
)
_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)


def _tokenize(text: str) -> List[Tuple[str, str]]:
    tokens: List[Tuple[str, str]] = []
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if m is None:
            raise ValueError("STEP: cannot tokenize near {!r}".format(text[pos:pos + 40]))
        pos = m.end()
        kind = m.lastgroup
        if kind != "ws":
            tokens.append((kind, m.group()))
    return tokens


class _Parser:
    def __init__(self, tokens: List[Tuple[str, str]]) -> None:
        self.tokens = tokens
        self.i = 0

    def peek(self) -> Tuple[str, str]:
        return self.tokens[self.i]

    def take(self, value: Optional[str] = None) -> Tuple[str, str]:
        tok = self.tokens[self.i]
        if value is not None and tok[1] != value:
            raise ValueError("STEP: expected {!r}, got {!r} (token {})".format(value, tok[1], self.i))
        self.i += 1
        return tok

    def value(self) -> Any:
        kind, text = self.take()
        if kind == "ref":
            return Ref(int(text[1:]))
        if kind == "num":
            return float(text) if any(c in text for c in ".Ee") else int(text)
        if kind == "str":
            return text[1:-1].replace("''", "'")
        if kind == "enum":
            return Enum(text[1:-1])
        if text == "$":
            return None
        if text == "*":
            return DERIVED
        if text == "(":
            return self.list_rest()
        if kind == "kw":
            self.take("(")
            return Typed(text, tuple(self.list_rest()))
        raise ValueError("STEP: unexpected token {!r}".format(text))

    def list_rest(self) -> List[Any]:
        """Values up to the matching ')' (the '(' has been consumed)."""
        out: List[Any] = []
        if self.peek()[1] == ")":
            self.take()
            return out
        while True:
            out.append(self.value())
            if self.take()[1] == ")":
                return out

    def entity(self) -> Entity:
        ref = self.take()
        if ref[0] != "ref":
            raise ValueError("STEP: expected an entity id, got {!r}".format(ref[1]))
        self.take("=")
        if self.peek()[1] == "(":  # complex instance: (A(...) B(...) ...)
            self.take("(")
            parts: Dict[str, List[Any]] = {}
            while self.peek()[1] != ")":
                name = self.take()[1]
                self.take("(")
                parts[name] = self.list_rest()
            self.take(")")
            self.take(";")
            return Entity(int(ref[1][1:]), "COMPLEX", [], parts)
        name = self.take()[1]
        self.take("(")
        args = self.list_rest()
        self.take(";")
        return Entity(int(ref[1][1:]), name, args)


class StepFile:
    """All entities of one STEP file, indexed by id and by type."""

    def __init__(self, entities: Dict[int, Entity]) -> None:
        self.entities = entities
        self.by_type: Dict[str, List[Entity]] = defaultdict(list)
        for e in entities.values():
            if e.type == "COMPLEX":
                for part in e.parts:
                    self.by_type[part].append(e)
            else:
                self.by_type[e.type].append(e)

    @classmethod
    def load(cls, path: Path) -> "StepFile":
        text = Path(path).read_text(encoding="utf-8", errors="replace")
        start = text.find("DATA;")
        end = text.find("ENDSEC;", start)
        if start < 0 or end < 0:
            raise ValueError("{}: no DATA section; not an ISO-10303-21 file?".format(path))
        body = _COMMENT.sub(" ", text[start + len("DATA;"):end])
        parser = _Parser(_tokenize(body))
        entities: Dict[int, Entity] = {}
        while parser.i < len(parser.tokens):
            e = parser.entity()
            entities[e.id] = e
        return cls(entities)

    def __getitem__(self, ref: int) -> Entity:
        return self.entities[int(ref)]

    def of_type(self, name: str) -> List[Entity]:
        return self.by_type.get(name, [])


# ------------------------------------------------------------------------------------ geometry


@dataclass
class Conic:
    """A circle or ellipse edge: ``p(t) = centre + a cos(t) x_dir + b sin(t) y_dir``."""

    kind: str
    centre: np.ndarray
    axis: np.ndarray
    x_dir: np.ndarray
    y_dir: np.ndarray
    a: float
    b: float
    closed: bool


@dataclass
class Face:
    surface: str
    normal: Optional[np.ndarray]
    loops: List[np.ndarray]


@dataclass
class Solid:
    name: str
    color: Optional[Tuple[float, float, float]]
    faces: List[Face]
    vertices: np.ndarray
    conics: List[Conic]
    unsupported_curves: int = 0

    def points(self) -> np.ndarray:
        """Every vertex and sampled loop point, ``(N, 3)``."""
        chunks = [self.vertices] + [loop for f in self.faces for loop in f.loops]
        return np.concatenate([c for c in chunks if len(c)], axis=0)


def _vec(step: StepFile, ref: Optional[int]) -> Optional[np.ndarray]:
    if ref is None:
        return None
    e = step[ref]
    if e.type == "CARTESIAN_POINT":
        return np.array(e.args[1], dtype=float)
    if e.type == "DIRECTION":
        v = np.array(e.args[1], dtype=float)
        return v / np.linalg.norm(v)
    if e.type == "VECTOR":
        return _vec(step, e.args[1]) * float(e.args[2])
    raise ValueError("STEP: #{} is {}, expected a point or direction".format(ref, e.type))


def _placement(step: StepFile, ref: int) -> np.ndarray:
    """4x4 matrix of an ``AXIS2_PLACEMENT_3D`` (local -> parent)."""
    e = step[ref]
    if e.type != "AXIS2_PLACEMENT_3D":
        raise ValueError("STEP: #{} is {}, expected AXIS2_PLACEMENT_3D".format(ref, e.type))
    origin = _vec(step, e.args[1])
    z = _vec(step, e.args[2]) if len(e.args) > 2 and e.args[2] is not None else np.array([0.0, 0.0, 1.0])
    x = _vec(step, e.args[3]) if len(e.args) > 3 and e.args[3] is not None else np.array([1.0, 0.0, 0.0])
    x = x - np.dot(x, z) * z
    if np.linalg.norm(x) < 1e-9:
        x = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x, y, z, origin
    return m


def length_unit_to_metres(step: StepFile) -> float:
    """Scale from the file's length unit to metres (SI prefixes and inches are handled)."""
    prefixes = {None: 1.0, "MILLI": 1e-3, "CENTI": 1e-2, "DECI": 1e-1, "KILO": 1e3, "MICRO": 1e-6}
    for e in step.of_type("LENGTH_UNIT"):
        if "SI_UNIT" in e.parts:
            prefix = e.parts["SI_UNIT"][0]
            return prefixes[prefix.value if isinstance(prefix, Enum) else None]
        if "CONVERSION_BASED_UNIT" in e.parts:
            name = str(e.parts["CONVERSION_BASED_UNIT"][0]).upper()
            if name in ("INCH", "IN"):
                return 0.0254
            if name in ("FOOT", "FT"):
                return 0.3048
            raise ValueError("STEP: unsupported length unit {!r}".format(name))
    return 1.0


class _BrepSampler:
    def __init__(self, step: StepFile) -> None:
        self.step = step
        self.edge_cache: Dict[int, np.ndarray] = {}
        self.unsupported = 0

    def vertex(self, ref: int) -> np.ndarray:
        return _vec(self.step, self.step[ref].args[1])

    def conic(self, curve: Entity, closed: bool) -> Conic:
        m = _placement(self.step, curve.args[1])
        a = float(curve.args[2])
        b = float(curve.args[3]) if curve.type == "ELLIPSE" else a
        return Conic(curve.type.lower(), m[:3, 3].copy(), m[:3, 2].copy(), m[:3, 0].copy(), m[:3, 1].copy(), a, b, closed)

    def edge(self, ref: int) -> np.ndarray:
        """Sampled points of an ``EDGE_CURVE`` from its start vertex to its end vertex."""
        if ref in self.edge_cache:
            return self.edge_cache[ref]
        e = self.step[ref]
        p0, p1 = self.vertex(e.args[1]), self.vertex(e.args[2])
        curve = self.step[e.args[3]]
        same_sense = not (isinstance(e.args[4], Enum) and e.args[4].value == "F")
        if curve.type in ("CIRCLE", "ELLIPSE"):
            c = self.conic(curve, closed=int(e.args[1]) == int(e.args[2]))

            def angle(p: np.ndarray) -> float:
                d = p - c.centre
                return math.atan2(np.dot(d, c.y_dir) / c.b, np.dot(d, c.x_dir) / c.a)

            t0, t1 = angle(p0), angle(p1)
            if c.closed:
                t1 = t0 + (2.0 * math.pi if same_sense else -2.0 * math.pi)
            elif same_sense:
                while t1 <= t0:
                    t1 += 2.0 * math.pi
            else:
                while t1 >= t0:
                    t1 -= 2.0 * math.pi
            n = max(2, int(math.ceil(abs(t1 - t0) / ARC_STEP_RAD)) + 1)
            t = np.linspace(t0, t1, n)
            pts = c.centre + c.a * np.cos(t)[:, None] * c.x_dir + c.b * np.sin(t)[:, None] * c.y_dir
            pts[0], pts[-1] = p0, p1
        elif curve.type == "LINE":
            pts = np.stack([p0, p1])
        else:
            self.unsupported += 1
            pts = np.stack([p0, p1])
        self.edge_cache[ref] = pts
        return pts

    def loop(self, ref: int) -> np.ndarray:
        e = self.step[ref]
        if e.type == "VERTEX_LOOP":
            return self.vertex(e.args[1])[None]
        chunks: List[np.ndarray] = []
        for oe_ref in e.args[1]:
            oe = self.step[oe_ref]
            pts = self.edge(oe.args[3])
            if isinstance(oe.args[4], Enum) and oe.args[4].value == "F":
                pts = pts[::-1]
            chunks.append(pts if not chunks else pts[1:])
        pts = np.concatenate(chunks, axis=0)
        if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
            pts = pts[:-1]
        return pts


def _loop_area(loop: np.ndarray, normal: Optional[np.ndarray]) -> float:
    if len(loop) < 3:
        return 0.0
    if normal is None:
        centred = loop - loop.mean(axis=0)
        normal = np.linalg.svd(centred, full_matrices=False)[2][-1]
    total = np.zeros(3)
    for p, q in zip(loop, np.roll(loop, -1, axis=0)):
        total += np.cross(p, q)
    return abs(0.5 * float(np.dot(total, normal)))


def _colour(step: StepFile, start: Iterable[Any], depth: int = 0) -> Optional[Tuple[float, float, float]]:
    if depth > 10:
        return None
    for item in start:
        if isinstance(item, list):
            found = _colour(step, item, depth + 1)
        elif isinstance(item, Ref):
            e = step[item]
            if e.type == "COLOUR_RGB":
                return (float(e.args[1]), float(e.args[2]), float(e.args[3]))
            found = _colour(step, e.args, depth + 1)
        else:
            found = None
        if found is not None:
            return found
    return None


def _solid_transforms(step: StepFile) -> Dict[int, np.ndarray]:
    """World transform of every B-rep solid, following the assembly's representation links."""
    rep_of_item: Dict[int, int] = {}
    for kind in ("ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
                 "SHAPE_REPRESENTATION"):
        for e in step.of_type(kind):
            if e.type == "COMPLEX":
                continue
            for item in e.args[1]:
                if isinstance(item, Ref) and step[item].type == "MANIFOLD_SOLID_BREP":
                    rep_of_item[int(item)] = e.id
    linked: Dict[int, List[int]] = defaultdict(list)
    for e in step.of_type("SHAPE_REPRESENTATION_RELATIONSHIP"):
        if e.type != "COMPLEX":
            linked[int(e.args[2])].append(int(e.args[3]))
            linked[int(e.args[3])].append(int(e.args[2]))
    parent_of: Dict[int, Tuple[int, np.ndarray]] = {}
    for e in step.of_type("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"):
        rr = e.parts.get("REPRESENTATION_RELATIONSHIP")
        idt = step[e.parts["REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"][0]]
        if rr is None or idt.type != "ITEM_DEFINED_TRANSFORMATION":
            continue
        child, parent = int(rr[2]), int(rr[3])
        m = _placement(step, idt.args[3]) @ np.linalg.inv(_placement(step, idt.args[2]))
        parent_of[child] = (parent, m)

    def world(rep: int, seen: frozenset) -> np.ndarray:
        group, queue = {rep}, deque([rep])
        while queue:
            r = queue.popleft()
            for other in linked.get(r, []):
                if other not in group:
                    group.add(other)
                    queue.append(other)
        for r in group:
            if r in parent_of and r not in seen:
                parent, m = parent_of[r]
                return world(parent, seen | group) @ m
        return np.eye(4)

    return {solid: world(rep, frozenset()) for solid, rep in rep_of_item.items()}


def read_solids(path: Path) -> Tuple[List[Solid], Dict[str, Any]]:
    """Every solid of a STEP file in world metres, plus file info (product names, unit scale)."""
    step = StepFile.load(path)
    scale = length_unit_to_metres(step)
    transforms = _solid_transforms(step)
    colours: Dict[int, Tuple[float, float, float]] = {}
    for e in step.of_type("STYLED_ITEM"):
        c = _colour(step, e.args[1])
        if c is not None and isinstance(e.args[2], Ref):
            colours[int(e.args[2])] = c

    solids: List[Solid] = []
    for brep in step.of_type("MANIFOLD_SOLID_BREP"):
        sampler = _BrepSampler(step)
        m = transforms.get(brep.id, np.eye(4))
        rot, shift = m[:3, :3], m[:3, 3]

        def to_world(p: np.ndarray) -> np.ndarray:
            return (p * scale) @ rot.T + shift * scale

        faces: List[Face] = []
        vertex_refs = set()
        conics: Dict[int, Conic] = {}
        for face_ref in step[brep.args[1]].args[1]:
            face = step[face_ref]
            surface = step[face.args[2]]
            normal = None
            if surface.type == "PLANE":
                normal = _placement(step, surface.args[1])[:3, 2] @ rot.T
                if isinstance(face.args[3], Enum) and face.args[3].value == "F":
                    normal = -normal
            loops = []
            for bound_ref in face.args[1]:
                bound = step[bound_ref]
                pts = sampler.loop(bound.args[1])
                if isinstance(bound.args[2], Enum) and bound.args[2].value == "F":
                    pts = pts[::-1]
                loops.append(to_world(pts))
                loop_entity = step[bound.args[1]]
                if loop_entity.type == "EDGE_LOOP":
                    for oe_ref in loop_entity.args[1]:
                        edge = step[step[oe_ref].args[3]]
                        vertex_refs.update((int(edge.args[1]), int(edge.args[2])))
                        curve = step[edge.args[3]]
                        if curve.type in ("CIRCLE", "ELLIPSE") and curve.id not in conics:
                            c = sampler.conic(curve, closed=int(edge.args[1]) == int(edge.args[2]))
                            c.centre = to_world(c.centre)
                            c.axis, c.x_dir, c.y_dir = c.axis @ rot.T, c.x_dir @ rot.T, c.y_dir @ rot.T
                            c.a, c.b = c.a * scale, c.b * scale
                            conics[curve.id] = c
                else:
                    vertex_refs.add(int(loop_entity.args[1]))
            loops.sort(key=lambda lp: -_loop_area(lp, normal))
            faces.append(Face(surface.type, normal, loops))
        vertices = np.array([to_world(sampler.vertex(v)) for v in sorted(vertex_refs)]) if vertex_refs \
            else np.zeros((0, 3))
        solids.append(Solid(str(brep.args[0]), colours.get(brep.id), faces, vertices,
                            list(conics.values()), sampler.unsupported))

    products = [str(e.args[0]) for e in step.of_type("PRODUCT")]
    return solids, {"products": products, "unit_to_metres": scale, "entities": len(step.entities)}
