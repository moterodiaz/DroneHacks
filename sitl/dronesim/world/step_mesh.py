"""Tessellate the solids of a STEP B-rep model into world-coordinate triangles.

Pipeline
--------
1. :class:`~dronesim.world.step_parser.StepFile` indexes the Part 21 entities.
2. The product structure is walked from the root product definition(s) to find
   every MANIFOLD_SOLID_BREP instance and its placement in the root (world) frame.
3. Each B-rep is tessellated face by face.  Edges are discretised once per
   solid and shared by both adjacent faces, so the mesh is watertight by
   construction (no T-junctions).  Planar faces are triangulated exactly from
   their boundary polygon (ear clipping with hole bridging, then Delaunay edge
   flips).  Curved faces are meshed in the surface's (u, v) parameter domain,
   trimmed by the boundary loops mapped into (u, v), with interior lattice
   points and refinement until the chordal error is <= ``chord_tol`` and no
   edge is longer than ``max_edge``.
4. Every triangle is oriented so its normal points out of the solid; each
   solid's signed volume and closedness are then checked.

STEP conventions relied on
--------------------------
Assembly placement (AP214; CAx-IF recommended practices, "assembly structure"):

* ``NEXT_ASSEMBLY_USAGE_OCCURRENCE(id, name, description, relating, related, ref)``:
  ``relating`` is the parent PRODUCT_DEFINITION, ``related`` the child.
* ``CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(representation_relation,
  represented_product_relation)``: the second attribute is a
  PRODUCT_DEFINITION_SHAPE whose ``definition`` is the NAUO; the first is the
  complex instance ``(REPRESENTATION_RELATIONSHIP(name, description, rep_1, rep_2)
  REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(operator)
  SHAPE_REPRESENTATION_RELATIONSHIP())``.  ``rep_1`` is the child's shape
  representation and ``rep_2`` the parent's.  Some exporters swap them; the
  swap is detected by checking which side is a representation of the child
  product (through SHAPE_DEFINITION_REPRESENTATION) and the transform is
  inverted in that case, as OpenCascade does.
* ``ITEM_DEFINED_TRANSFORMATION(name, description, transform_item_1,
  transform_item_2)``: item 1 is an AXIS2_PLACEMENT_3D in ``rep_1``, item 2 one
  in ``rep_2``, and the transformation carries item 1's frame onto item 2's.
  A point with coordinates ``p`` in the child representation therefore has
  coordinates ``T(item_2) @ inv(T(item_1)) @ p`` in the parent, where ``T(a)``
  maps placement ``a``'s local coordinates into its representation.
  In ``dronehacks-course-v3.step`` every one of the 33 transformations maps the
  identity placement onto the identity placement, so both conceivable
  directions give the identity: the parts are modelled in world coordinates
  (confirmed against the known boulder and landing-pad positions).  The
  direction is exercised by a synthetic rotated + translated assembly in
  ``tests/test_step_mesh.py``.
* A child SHAPE_REPRESENTATION (placements only) is linked to the
  ADVANCED_BREP_SHAPE_REPRESENTATION holding the MANIFOLD_SOLID_BREP by a plain
  SHAPE_REPRESENTATION_RELATIONSHIP (no transformation).
* Units come from the GLOBAL_UNIT_ASSIGNED_CONTEXT of the representation's
  context (SI prefixes and CONVERSION_BASED_UNIT are honoured); all output is in
  metres.

Topology and orientation (ISO 10303-42):

* ``ADVANCED_FACE(name, bounds, face_geometry, same_sense)``: the face normal is
  the surface's natural normal when ``same_sense`` is .T., else its reverse.  In
  the CLOSED_SHELL of a MANIFOLD_SOLID_BREP face normals point out of the
  material.  Natural normals: PLANE -> placement axis; every parametric
  surface -> S_u x S_v (outward from the axis for cylinders, cones, spheres and
  tori with the parametrisations below).
* ``FACE_BOUND/FACE_OUTER_BOUND(name, bound, orientation)``: the loop is used in
  its own direction when ``orientation`` is .T., else reversed.  The region of a
  face is derived from loop *geometry* (outer = FACE_OUTER_BOUND, else the loop
  enclosing the largest area; Onshape writes only FACE_BOUND), and the loop
  direction is cross-checked against the face normal.
* ``EDGE_LOOP(name, edges)`` of ``ORIENTED_EDGE(name, *, *, edge_element,
  orientation)``: .F. traverses the EDGE_CURVE from ``edge_end`` to ``edge_start``.
* ``EDGE_CURVE(name, edge_start, edge_end, edge_geometry, same_sense)``: .T.
  means the curve parameter increases from ``edge_start`` to ``edge_end``.  On a
  closed curve (circle, ellipse, closed B-spline) this selects which of the two
  arcs is meant; an edge whose two vertices coincide is the full curve.
* ``VERTEX_LOOP(name, vertex)``: a degenerate loop at a surface singularity
  (cone apex, pole of a revolved dome).  In (u, v) it becomes a segment of the
  pole iso-line.
* SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE are tessellated from their
  ``curve_3d``.

Surface parametrisations (P = location, x/y/z = placement axes):

* CYLINDRICAL_SURFACE  S = P + r(cos u x + sin u y) + v z
* CONICAL_SURFACE      S = P + (r + v tan a)(cos u x + sin u y) + v z
* SPHERICAL_SURFACE    S = P + r cos v (cos u x + sin u y) + r sin v z
* TOROIDAL_SURFACE     S = P + (R + r cos v)(cos u x + sin u y) + r sin v z
* SURFACE_OF_LINEAR_EXTRUSION  S = C(u) + v V
* SURFACE_OF_REVOLUTION        S = A + Rot(axis, u)(C(v) - A)
* B_SPLINE_SURFACE_WITH_KNOTS (and its rational complex form) evaluated with the
  Cox-de Boor recursion; closed/periodic directions are detected numerically.
"""

from __future__ import annotations

import hashlib
import math
import os
import tempfile
import zipfile
from collections import defaultdict
from dataclasses import dataclass, field
from fnmatch import fnmatch
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np

from .step_parser import Entity, Ref, StepFile, Typed

__all__ = [
    "MESH_VERSION",
    "Solid",
    "StepMeshError",
    "WatertightReport",
    "load_step",
    "load_step_cached",
]

#: Bump whenever tessellation output changes, to invalidate caches.
MESH_VERSION = 1

_TWO_PI = 2.0 * math.pi


class StepMeshError(ValueError):
    """The STEP data uses an unsupported construct or is geometrically inconsistent."""


# ==========================================================================
# Output types
# ==========================================================================

@dataclass(frozen=True)
class WatertightReport:
    """Closedness of a triangle mesh after welding coincident vertices.

    ``boundary_edges`` are used by one triangle, ``nonmanifold_edges`` by more
    than two, and ``misoriented_edges`` by two triangles traversing them in the
    same direction (inconsistent orientation).
    """

    triangles: int
    vertices: int
    boundary_edges: int
    nonmanifold_edges: int
    misoriented_edges: int
    degenerate_triangles: int

    @property
    def watertight(self) -> bool:
        return self.boundary_edges == 0 and self.nonmanifold_edges == 0 and self.misoriented_edges == 0


@dataclass(eq=False)
class Solid:
    """One tessellated solid in world coordinates (metres).

    ``triangles[i]`` holds the three vertices of triangle ``i``, wound
    counter-clockwise seen from outside, and ``normals[i]`` is its unit outward
    normal.  ``notes`` records anything noteworthy found while meshing it.
    """

    name: str
    triangles: np.ndarray
    normals: np.ndarray
    color: Optional[Tuple[float, float, float]]
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    notes: Tuple[str, ...] = field(default=())

    @classmethod
    def from_triangles(cls, name: str, triangles: np.ndarray,
                       color: Optional[Tuple[float, float, float]] = None,
                       notes: Sequence[str] = ()) -> "Solid":
        tris = np.ascontiguousarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        length = np.linalg.norm(n, axis=1)
        normals = n / np.where(length > 0, length, 1.0)[:, None]
        if len(tris):
            lo, hi = tris.min(axis=(0, 1)), tris.max(axis=(0, 1))
        else:
            lo = hi = np.zeros(3)
        return cls(name, tris, normals, color, lo, hi, tuple(notes))

    def volume(self) -> float:
        """Signed volume by the divergence theorem; positive for an outward-oriented closed mesh."""
        a, b, c = self.triangles[:, 0], self.triangles[:, 1], self.triangles[:, 2]
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

    def area(self) -> float:
        return float(0.5 * np.linalg.norm(np.cross(self.triangles[:, 1] - self.triangles[:, 0],
                                                   self.triangles[:, 2] - self.triangles[:, 0]), axis=1).sum())

    def watertight_report(self, weld_tol: float = 1e-7) -> WatertightReport:
        """Weld vertices closer than ``weld_tol`` and count open / non-manifold / misoriented edges."""
        return watertight_report(self.triangles, weld_tol)

    @property
    def is_watertight(self) -> bool:
        return self.watertight_report().watertight


def weld_vertices(triangles: np.ndarray, weld_tol: float = 1e-7) -> Tuple[np.ndarray, np.ndarray]:
    """Merge coincident vertices.  Returns ``(vertices (V,3), faces (T,3) int)``."""
    pts = triangles.reshape(-1, 3)
    keys = np.round(pts / weld_tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    return pts[first], inverse.reshape(-1, 3)


def watertight_report(triangles: np.ndarray, weld_tol: float = 1e-7) -> WatertightReport:
    verts, faces = weld_vertices(triangles, weld_tol)
    degenerate = (faces[:, 0] == faces[:, 1]) | (faces[:, 1] == faces[:, 2]) | (faces[:, 2] == faces[:, 0])
    f = faces[~degenerate]
    directed = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    nv = max(len(verts), 1)
    lo = directed.min(axis=1)
    hi = directed.max(axis=1)
    undirected = lo * nv + hi
    forward = (directed[:, 0] == lo).astype(np.int64)
    uniq, inv, counts = np.unique(undirected, return_inverse=True, return_counts=True)
    fwd = np.bincount(inv, weights=forward, minlength=len(uniq))
    boundary = int((counts == 1).sum())
    nonmanifold = int((counts > 2).sum())
    misoriented = int(((counts == 2) & (fwd != 1)).sum())
    return WatertightReport(len(triangles), len(verts), boundary, nonmanifold, misoriented, int(degenerate.sum()))


# ==========================================================================
# Small geometry helpers
# ==========================================================================

def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n == 0.0:
        raise StepMeshError("zero-length direction")
    return v / n


def _wrap(d: np.ndarray, period: float) -> np.ndarray:
    """Map differences into [-period/2, period/2]."""
    return d - period * np.round(d / period)


def _safe_div(a, b) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a, b = np.broadcast_arrays(a, b)
    out = np.zeros(a.shape)
    np.divide(a, b, out=out, where=b != 0)
    return out


def _dot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.einsum("ij,ij->i", a, b)


class _Frame:
    """A right-handed placement: origin and orthonormal axes x, y, z."""

    __slots__ = ("origin", "x", "y", "z")

    def __init__(self, origin: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> None:
        self.origin, self.x, self.y, self.z = origin, x, y, z

    def local(self, pts: np.ndarray) -> np.ndarray:
        d = np.asarray(pts, float) - self.origin
        return np.stack([d @ self.x, d @ self.y, d @ self.z], axis=-1)

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = self.x, self.y, self.z, self.origin
        return m


# ==========================================================================
# Units
# ==========================================================================

_SI_PREFIX = {
    None: 1.0, "EXA": 1e18, "PETA": 1e15, "TERA": 1e12, "GIGA": 1e9, "MEGA": 1e6,
    "KILO": 1e3, "HECTO": 1e2, "DECA": 1e1, "DECI": 1e-1, "CENTI": 1e-2,
    "MILLI": 1e-3, "MICRO": 1e-6, "NANO": 1e-9, "PICO": 1e-12, "FEMTO": 1e-15, "ATTO": 1e-18,
}


def _unit_factor(sf: StepFile, ref: Ref, depth: int = 0) -> float:
    """Factor from a NAMED_UNIT to metres / radians."""
    if depth > 8:
        raise StepMeshError(f"unit chain too deep at {ref}")
    ent = sf.get(ref)
    if ent.isa("SI_UNIT"):
        prefix, name = ent.part("SI_UNIT")
        if name not in ("METRE", "RADIAN", "STERADIAN"):
            raise StepMeshError(f"#{ent.id}: unexpected SI unit {name}")
        return _SI_PREFIX[None if prefix is None else str(prefix)]
    if ent.isa("CONVERSION_BASED_UNIT"):
        _, measure_ref = ent.part("CONVERSION_BASED_UNIT")
        measure = sf.get(measure_ref)
        args = measure.parts.get("MEASURE_WITH_UNIT", measure.args)
        value = args[0].value if isinstance(args[0], Typed) else args[0]
        return float(value) * _unit_factor(sf, args[1], depth + 1)
    raise StepMeshError(f"#{ent.id}: unsupported unit {ent.type}")


def _context_units(sf: StepFile, ctx_ref: Optional[Ref]) -> Tuple[float, float]:
    """(metres per length unit, radians per angle unit) of a representation context."""
    length, angle = 1.0, 1.0
    if ctx_ref is None:
        return length, angle
    ctx = sf.get(ctx_ref)
    units = ctx.parts.get("GLOBAL_UNIT_ASSIGNED_CONTEXT")
    if units:
        for uref in units[0]:
            u = sf.get(uref)
            if u.isa("LENGTH_UNIT"):
                length = _unit_factor(sf, uref)
            elif u.isa("PLANE_ANGLE_UNIT"):
                angle = _unit_factor(sf, uref)
    return length, angle


# ==========================================================================
# B-spline basis
# ==========================================================================

def _bspline_basis(U: np.ndarray, p: int, n: int, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Non-zero B-spline basis functions of degree ``p`` and their first derivatives.

    ``U`` is the full knot vector (length ``n + p + 1``) for ``n`` control points.
    Returns ``(span, N, dN)`` with ``N[:, a]`` the basis function of control point
    ``span - p + a``.  Cox-de Boor triangular scheme (Piegl & Tiller, *The NURBS
    Book*, A2.2 / A2.3), vectorised over ``t``.
    """
    t = np.atleast_1d(np.asarray(t, dtype=float))
    span = np.clip(np.searchsorted(U, t, side="right") - 1, p, n - 1)
    m = t.shape[0]
    N = np.ones((m, 1))
    prev = N
    for j in range(1, p + 1):
        prev = N
        N = np.zeros((m, j + 1))
        for a in range(j + 1):
            i = span - j + a
            if a > 0:
                N[:, a] += _safe_div(t - U[i], U[i + j] - U[i]) * prev[:, a - 1]
            if a < j:
                N[:, a] += _safe_div(U[i + j + 1] - t, U[i + j + 1] - U[i + 1]) * prev[:, a]
    dN = np.zeros((m, p + 1))
    if p > 0:
        for a in range(p + 1):
            i = span - p + a
            if a > 0:
                dN[:, a] += p * _safe_div(1.0, U[i + p] - U[i]) * prev[:, a - 1]
            if a < p:
                dN[:, a] -= p * _safe_div(1.0, U[i + p + 1] - U[i + 1]) * prev[:, a]
    return span, N, dN


def _knot_vector(mults: Sequence[int], knots: Sequence[float]) -> np.ndarray:
    return np.repeat(np.asarray(knots, dtype=float), np.asarray(mults, dtype=int))


def _distinct_spans(U: np.ndarray, lo: float, hi: float) -> int:
    k = np.unique(U[(U >= lo) & (U <= hi)])
    return max(1, len(k) - 1)


# ==========================================================================
# Curves
# ==========================================================================

class _Curve:
    """A 3-D parametric curve.  ``period`` is set for closed (periodic) curves."""

    period: Optional[float] = None

    def derivs(self, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def eval(self, t: np.ndarray) -> np.ndarray:
        return self.derivs(t)[0]

    def param(self, pts: np.ndarray) -> np.ndarray:
        """Parameters of points lying on the curve."""
        raise NotImplementedError

    def domain(self) -> Optional[Tuple[float, float]]:
        return None

    def initial_segments(self, t0: float, t1: float) -> int:
        """Segments to start adaptive sampling with (enough to see the curve's shape)."""
        return 1


class _Line(_Curve):
    def __init__(self, p0: np.ndarray, d: np.ndarray) -> None:
        self.p0, self.d = p0, d

    def derivs(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        return self.p0 + t[:, None] * self.d, np.broadcast_to(self.d, (t.size, 3)).copy()

    def param(self, pts):
        return (np.asarray(pts, float) - self.p0) @ self.d / float(self.d @ self.d)


class _Conic(_Curve):
    """CIRCLE (a == b) or ELLIPSE: C(t) = P + a cos t x + b sin t y."""

    period = _TWO_PI

    def __init__(self, frame: _Frame, a: float, b: float) -> None:
        self.f, self.a, self.b = frame, a, b

    def derivs(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        c, s = np.cos(t)[:, None], np.sin(t)[:, None]
        f = self.f
        return (f.origin + self.a * c * f.x + self.b * s * f.y,
                -self.a * s * f.x + self.b * c * f.y)

    def param(self, pts):
        loc = self.f.local(pts)
        return np.arctan2(loc[:, 1] / self.b, loc[:, 0] / self.a)

    def domain(self):
        return (0.0, _TWO_PI)

    def initial_segments(self, t0, t1):
        return max(2, int(math.ceil(abs(t1 - t0) / (math.pi / 2) - 1e-9)))


class _BSplineCurve(_Curve):
    """Non-rational or rational B-spline curve."""

    def __init__(self, degree: int, P: np.ndarray, U: np.ndarray, W: Optional[np.ndarray]) -> None:
        self.p = degree
        self.n = len(P)
        self.U = U
        self.Pw = P if W is None else np.hstack([P * W[:, None], W[:, None]])
        self.rational = W is not None
        self.lo, self.hi = float(U[degree]), float(U[self.n])
        a, b = self.eval(np.array([self.lo, self.hi]))
        size = float(np.ptp(P, axis=0).max()) or 1.0
        if np.linalg.norm(a - b) <= 1e-9 * size:
            self.period = self.hi - self.lo

    def _norm_t(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        if self.period:
            return self.lo + np.mod(t - self.lo, self.period)
        return np.clip(t, self.lo, self.hi)

    def derivs(self, t):
        t = self._norm_t(t)
        span, N, dN = _bspline_basis(self.U, self.p, self.n, t)
        idx = span[:, None] - self.p + np.arange(self.p + 1)
        G = self.Pw[idx]
        A = np.einsum("ma,mak->mk", N, G)
        dA = np.einsum("ma,mak->mk", dN, G)
        if not self.rational:
            return A, dA
        w, dw = A[:, 3:], dA[:, 3:]
        C = A[:, :3] / w
        return C, (dA[:, :3] - dw * C) / w

    def domain(self):
        return (self.lo, self.hi)

    def param(self, pts):
        pts = np.atleast_2d(np.asarray(pts, float))
        ns = max(16, 8 * _distinct_spans(self.U, self.lo, self.hi))
        ts = np.linspace(self.lo, self.hi, ns + 1)
        C = self.eval(ts)
        d2 = ((pts[:, None, :] - C[None, :, :]) ** 2).sum(-1)
        t = ts[np.argmin(d2, axis=1)]
        for _ in range(20):
            Ct, dC = self.derivs(t)
            num = _dot(pts - Ct, dC)
            den = _dot(dC, dC)
            step = _safe_div(num, den)
            t = t + step
            if not self.period:
                t = np.clip(t, self.lo, self.hi)
            if np.abs(step).max() < 1e-13 * (self.hi - self.lo):
                break
        return t

    def initial_segments(self, t0, t1):
        return max(2, _distinct_spans(self.U, min(t0, t1), max(t0, t1)) * max(1, self.p))


class _Polyline(_Curve):
    def __init__(self, pts: np.ndarray) -> None:
        self.pts = pts

    def derivs(self, t):
        t = np.clip(np.atleast_1d(np.asarray(t, float)), 0, len(self.pts) - 1)
        i = np.minimum(np.floor(t).astype(int), len(self.pts) - 2)
        f = (t - i)[:, None]
        d = self.pts[i + 1] - self.pts[i]
        return self.pts[i] + f * d, d

    def param(self, pts):
        pts = np.atleast_2d(np.asarray(pts, float))
        a, b = self.pts[:-1], self.pts[1:]
        d = b - a
        dd = np.maximum((d * d).sum(1), 1e-300)
        f = np.clip(((pts[:, None, :] - a[None]) * d[None]).sum(-1) / dd, 0, 1)
        q = a[None] + f[..., None] * d[None]
        k = np.argmin(((pts[:, None, :] - q) ** 2).sum(-1), axis=1)
        return k + f[np.arange(len(pts)), k]

    def initial_segments(self, t0, t1):
        return max(1, int(math.ceil(abs(t1 - t0))))


# ==========================================================================
# Surfaces
# ==========================================================================

class _Surface:
    """A parametric surface S(u, v) with natural normal S_u x S_v.

    ``periods[k]`` is the period of direction k (0 = u, 1 = v) or None, and
    ``bounds[k]`` the parameter interval (None for unbounded directions).
    """

    kind = "surface"
    periods: Tuple[Optional[float], Optional[float]] = (None, None)
    bounds: Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]]] = (None, None)

    def derivs(self, u, v) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        raise NotImplementedError

    def eval(self, u, v) -> np.ndarray:
        return self.derivs(u, v)[0]

    def initial_uv(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def invert(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(u, v) of points lying on the surface (initial guess + Gauss-Newton)."""
        pts = np.atleast_2d(np.asarray(pts, float))
        u, v = self.initial_uv(pts)
        return self._refine(pts, u, v)

    def poles(self) -> List[Tuple[int, float]]:
        """Iso-lines that collapse to a point: (fixed direction j, parameter value)."""
        return []

    def _clamp(self, u, v):
        for k, arr in ((0, u), (1, v)):
            b = self.bounds[k]
            if b is not None and self.periods[k] is None:
                np.clip(arr, b[0], b[1], out=arr)
        return u, v

    def _refine(self, pts, u, v, iters: int = 12):
        u = np.array(u, dtype=float)
        v = np.array(v, dtype=float)
        for _ in range(iters):
            P, Su, Sv = self.derivs(u, v)
            r = pts - P
            a, b, c = _dot(Su, Su), _dot(Su, Sv), _dot(Sv, Sv)
            g1, g2 = _dot(Su, r), _dot(Sv, r)
            det = a * c - b * b
            good = det > 1e-12 * (a * c + 1e-300)
            du = np.where(good, _safe_div(c * g1 - b * g2, det), _safe_div(g1, a))
            dv = np.where(good, _safe_div(a * g2 - b * g1, det), _safe_div(g2, c))
            u += du
            v += dv
            self._clamp(u, v)
            if max(np.abs(du).max(initial=0), np.abs(dv).max(initial=0)) < 1e-14:
                break
        return u, v


class _Plane(_Surface):
    kind = "plane"

    def __init__(self, frame: _Frame) -> None:
        self.f = frame

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        v = np.atleast_1d(np.asarray(v, float))
        f = self.f
        P = f.origin + u[:, None] * f.x + v[:, None] * f.y
        return P, np.broadcast_to(f.x, P.shape).copy(), np.broadcast_to(f.y, P.shape).copy()

    def initial_uv(self, pts):
        loc = self.f.local(pts)
        return loc[:, 0], loc[:, 1]


class _Cylinder(_Surface):
    kind = "cylinder"
    periods = (_TWO_PI, None)

    def __init__(self, frame: _Frame, r: float) -> None:
        self.f, self.r = frame, r

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        v = np.atleast_1d(np.asarray(v, float))
        c, s = np.cos(u)[:, None], np.sin(u)[:, None]
        f = self.f
        radial = c * f.x + s * f.y
        P = f.origin + self.r * radial + v[:, None] * f.z
        return P, self.r * (-s * f.x + c * f.y), np.broadcast_to(f.z, P.shape).copy()

    def initial_uv(self, pts):
        loc = self.f.local(pts)
        return np.arctan2(loc[:, 1], loc[:, 0]), loc[:, 2]

    invert = initial_uv  # exact


class _Cone(_Surface):
    kind = "cone"
    periods = (_TWO_PI, None)

    def __init__(self, frame: _Frame, r: float, semi_angle: float) -> None:
        self.f, self.r, self.t = frame, r, math.tan(semi_angle)

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        v = np.atleast_1d(np.asarray(v, float))
        c, s = np.cos(u)[:, None], np.sin(u)[:, None]
        f = self.f
        radial = c * f.x + s * f.y
        rho = (self.r + v * self.t)[:, None]
        P = f.origin + rho * radial + v[:, None] * f.z
        return P, rho * (-s * f.x + c * f.y), self.t * radial + f.z

    def initial_uv(self, pts):
        loc = self.f.local(pts)
        return np.arctan2(loc[:, 1], loc[:, 0]), loc[:, 2]

    invert = initial_uv

    def poles(self):
        return [(1, -self.r / self.t)] if self.t != 0 else []


class _Sphere(_Surface):
    kind = "sphere"
    periods = (_TWO_PI, None)
    bounds = (None, (-math.pi / 2, math.pi / 2))

    def __init__(self, frame: _Frame, r: float) -> None:
        self.f, self.r = frame, r

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        v = np.atleast_1d(np.asarray(v, float))
        cu, su = np.cos(u)[:, None], np.sin(u)[:, None]
        cv, sv = np.cos(v)[:, None], np.sin(v)[:, None]
        f = self.f
        radial = cu * f.x + su * f.y
        P = f.origin + self.r * cv * radial + self.r * sv * f.z
        return P, self.r * cv * (-su * f.x + cu * f.y), self.r * (-sv * radial + cv * f.z)

    def initial_uv(self, pts):
        loc = self.f.local(pts)
        return np.arctan2(loc[:, 1], loc[:, 0]), np.arctan2(loc[:, 2], np.hypot(loc[:, 0], loc[:, 1]))

    invert = initial_uv

    def poles(self):
        return [(1, -math.pi / 2), (1, math.pi / 2)]


class _Torus(_Surface):
    kind = "torus"
    periods = (_TWO_PI, _TWO_PI)

    def __init__(self, frame: _Frame, R: float, r: float) -> None:
        self.f, self.R, self.r = frame, R, r

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        v = np.atleast_1d(np.asarray(v, float))
        cu, su = np.cos(u)[:, None], np.sin(u)[:, None]
        cv, sv = np.cos(v)[:, None], np.sin(v)[:, None]
        f = self.f
        radial = cu * f.x + su * f.y
        rho = self.R + self.r * cv
        P = f.origin + rho * radial + self.r * sv * f.z
        return P, rho * (-su * f.x + cu * f.y), self.r * (-sv * radial + cv * f.z)

    def initial_uv(self, pts):
        loc = self.f.local(pts)
        return np.arctan2(loc[:, 1], loc[:, 0]), np.arctan2(loc[:, 2], np.hypot(loc[:, 0], loc[:, 1]) - self.R)

    invert = initial_uv


class _BSplineSurface(_Surface):
    kind = "bspline"

    def __init__(self, pu: int, pv: int, P: np.ndarray, U: np.ndarray, V: np.ndarray,
                 W: Optional[np.ndarray]) -> None:
        self.pu, self.pv = pu, pv
        self.nu, self.nv = P.shape[0], P.shape[1]
        self.U, self.V = U, V
        self.rational = W is not None
        self.Pw = P if W is None else np.concatenate([P * W[..., None], W[..., None]], axis=-1)
        ub = (float(U[pu]), float(U[self.nu]))
        vb = (float(V[pv]), float(V[self.nv]))
        self.bounds = (ub, vb)
        self.periods = (None, None)
        size = float(np.ptp(P.reshape(-1, 3), axis=0).max()) or 1.0
        self._tol = 1e-9 * size
        s = np.linspace(0.0, 1.0, 17)
        us = ub[0] + s * (ub[1] - ub[0])
        vs = vb[0] + s * (vb[1] - vb[0])
        per = [None, None]
        if np.abs(self.eval(np.full(17, ub[0]), vs) - self.eval(np.full(17, ub[1]), vs)).max() <= self._tol:
            per[0] = ub[1] - ub[0]
        if np.abs(self.eval(us, np.full(17, vb[0])) - self.eval(us, np.full(17, vb[1]))).max() <= self._tol:
            per[1] = vb[1] - vb[0]
        self.periods = (per[0], per[1])
        self._poles = []
        for j, (b, other) in ((0, (ub, vs)), (1, (vb, us))):
            for val in b:
                fixed = np.full(17, val)
                pts = self.eval(fixed, other) if j == 0 else self.eval(other, fixed)
                if np.ptp(pts, axis=0).max() <= self._tol:
                    self._poles.append((j, val))

    def _norm(self, t, k):
        lo, hi = self.bounds[k]
        if self.periods[k]:
            return lo + np.mod(t - lo, self.periods[k])
        return np.clip(t, lo, hi)

    def derivs(self, u, v):
        u = self._norm(np.atleast_1d(np.asarray(u, float)), 0)
        v = self._norm(np.atleast_1d(np.asarray(v, float)), 1)
        ku, Nu, dNu = _bspline_basis(self.U, self.pu, self.nu, u)
        kv, Nv, dNv = _bspline_basis(self.V, self.pv, self.nv, v)
        iu = ku[:, None] - self.pu + np.arange(self.pu + 1)
        iv = kv[:, None] - self.pv + np.arange(self.pv + 1)
        G = self.Pw[iu[:, :, None], iv[:, None, :]]
        A = np.einsum("ma,mb,mabk->mk", Nu, Nv, G)
        Au = np.einsum("ma,mb,mabk->mk", dNu, Nv, G)
        Av = np.einsum("ma,mb,mabk->mk", Nu, dNv, G)
        if not self.rational:
            return A, Au, Av
        w, wu, wv = A[:, 3:], Au[:, 3:], Av[:, 3:]
        S = A[:, :3] / w
        return S, (Au[:, :3] - wu * S) / w, (Av[:, :3] - wv * S) / w

    def initial_uv(self, pts):
        (u0, u1), (v0, v1) = self.bounds
        nu = int(min(200, max(16, 4 * _distinct_spans(self.U, u0, u1) * self.pu)))
        nv = int(min(200, max(16, 4 * _distinct_spans(self.V, v0, v1) * self.pv)))
        gu, gv = np.meshgrid(np.linspace(u0, u1, nu + 1), np.linspace(v0, v1, nv + 1), indexing="ij")
        gu, gv = gu.ravel(), gv.ravel()
        G = self.eval(gu, gv)
        best = np.empty(len(pts), dtype=int)
        for s in range(0, len(pts), 256):  # bound the (points x grid) distance matrix
            chunk = pts[s:s + 256]
            d2 = (chunk ** 2).sum(1)[:, None] - 2.0 * chunk @ G.T + (G ** 2).sum(1)[None, :]
            best[s:s + 256] = np.argmin(d2, axis=1)
        return gu[best].copy(), gv[best].copy()

    def poles(self):
        return list(self._poles)


class _Extrusion(_Surface):
    """SURFACE_OF_LINEAR_EXTRUSION: S(u, v) = C(u) + v V."""

    kind = "extrusion"

    def __init__(self, curve: _Curve, vec: np.ndarray) -> None:
        self.c, self.V = curve, vec
        self.periods = (curve.period, None)
        self.bounds = (curve.domain(), None)

    def derivs(self, u, v):
        v = np.atleast_1d(np.asarray(v, float))
        C, dC = self.c.derivs(u)
        P = C + v[:, None] * self.V
        return P, dC, np.broadcast_to(self.V, P.shape).copy()

    def initial_uv(self, pts):
        if isinstance(self.c, _Line):
            A = np.stack([self.c.d, self.V], axis=1)
            sol, *_ = np.linalg.lstsq(A, (pts - self.c.p0).T, rcond=None)
            return sol[0], sol[1]
        lo, hi = self.c.domain() or (0.0, 1.0)
        ts = np.linspace(lo, hi, 257)
        C = self.c.eval(ts)
        vv = float(self.V @ self.V)
        rel = pts[:, None, :] - C[None]
        v = (rel @ self.V) / vv
        res = (rel ** 2).sum(-1) - v ** 2 * vv
        k = np.argmin(res, axis=1)
        return ts[k], v[np.arange(len(pts)), k]


class _Revolution(_Surface):
    """SURFACE_OF_REVOLUTION: S(u, v) = A + Rot(a, u) (C(v) - A), u = angle, v = curve parameter."""

    kind = "revolution"

    def __init__(self, curve: _Curve, origin: np.ndarray, axis: np.ndarray) -> None:
        self.c, self.A, self.a = curve, origin, axis
        self.periods = (_TWO_PI, curve.period)
        self.bounds = (None, curve.domain())
        e1 = np.cross(axis, [1.0, 0.0, 0.0])
        if np.linalg.norm(e1) < 1e-6:
            e1 = np.cross(axis, [0.0, 1.0, 0.0])
        self.e1 = _unit(e1)
        self.e2 = np.cross(axis, self.e1)

    def derivs(self, u, v):
        u = np.atleast_1d(np.asarray(u, float))
        C, dC = self.c.derivs(v)
        w = C - self.A
        cu, su = np.cos(u)[:, None], np.sin(u)[:, None]
        a = self.a
        aw = (w @ a)[:, None]
        axw = np.cross(a, w)
        P = self.A + w * cu + axw * su + a * aw * (1 - cu)
        Su = -w * su + axw * cu + a * aw * su
        adc = (dC @ a)[:, None]
        Sv = dC * cu + np.cross(a, dC) * su + a * adc * (1 - cu)
        return P, Su, Sv

    def _cyl(self, pts):
        w = pts - self.A
        z = w @ self.a
        x, y = w @ self.e1, w @ self.e2
        return np.hypot(x, y), z, np.arctan2(y, x)

    def initial_uv(self, pts):
        lo, hi = self.c.domain() or self._line_range(pts)
        ts = np.linspace(lo, hi, 257)
        rc, zc, thc = self._cyl(self.c.eval(ts))
        rp, zp, thp = self._cyl(pts)
        d2 = (rp[:, None] - rc[None]) ** 2 + (zp[:, None] - zc[None]) ** 2
        k = np.argmin(d2, axis=1)
        return thp - thc[k], ts[k]

    def _line_range(self, pts):
        t = self.c.param(pts)
        return float(t.min()) - 1.0, float(t.max()) + 1.0

    def poles(self):
        """Profile parameters where the swept curve touches the axis (the surface collapses
        to a point there): closed form for a line, else minima of the distance to the axis."""
        if isinstance(self.c, _Line):
            d = self.c.d - (self.c.d @ self.a) * self.a
            w0 = (self.c.p0 - self.A) - ((self.c.p0 - self.A) @ self.a) * self.a
            if d @ d > 0:
                t = -float(w0 @ d) / float(d @ d)
                if np.linalg.norm(w0 + t * d) < 1e-9:
                    return [(1, t)]
            return []
        dom = self.c.domain()
        if dom is None:
            return []
        lo, hi = dom
        per = self.c.period
        n = 512
        ts = np.linspace(lo, hi, n, endpoint=per is None)
        rho = self._cyl(self.c.eval(ts))[0]
        if per:
            left, right = np.roll(rho, 1), np.roll(rho, -1)
        else:
            left = np.r_[np.inf, rho[:-1]]
            right = np.r_[rho[1:], np.inf]
        step = ts[1] - ts[0]
        out: List[Tuple[int, float]] = []
        size = float(np.abs(self.c.eval(ts) - self.A).max()) or 1.0
        for i in np.flatnonzero((rho <= left) & (rho <= right)):
            a = max(ts[i] - step, lo) if per is None else ts[i] - step
            b = min(ts[i] + step, hi) if per is None else ts[i] + step
            for _ in range(60):  # golden-section search for the minimum of rho
                m1, m2 = b - 0.618034 * (b - a), a + 0.618034 * (b - a)
                r1, r2 = self._cyl(self.c.eval(np.array([m1, m2])))[0]
                if r1 < r2:
                    b = m2
                else:
                    a = m1
            t = 0.5 * (a + b)
            if self._cyl(self.c.eval(np.array([t])))[0][0] < 1e-9 * size:
                if per:
                    t = lo + (t - lo) % per
                if not any(abs(t - q) < 1e-6 * (hi - lo) for _, q in out):
                    out.append((1, t))
        return out


# ==========================================================================
# 2-D triangulation: ear clipping with holes + Delaunay flips + point insertion
# ==========================================================================

def _ring_area(x: Sequence[float], y: Sequence[float], idx: Sequence[int]) -> float:
    s = 0.0
    m = len(idx)
    for k in range(m):
        i, j = idx[k], idx[(k + 1) % m]
        s += x[i] * y[j] - x[j] * y[i]
    return 0.5 * s


class _EarCut:
    """Ear-clipping triangulation of a polygon with holes.

    * Collinear vertices are kept (they are shared with neighbouring faces) and
      never become the tip of a zero-area ear.
    * Holes are bridged to the outer ring from their leftmost vertex to the
      nearest mutually visible vertex, duplicating the two bridge vertices.
    * Output triangles are counter-clockwise and refer to the original indices
      of the concatenated input rings.
    """

    def __init__(self, rings: Sequence[np.ndarray]) -> None:
        xs: List[float] = []
        ys: List[float] = []
        self.ranges = []
        for r in rings:
            start = len(xs)
            xs.extend(float(a) for a in r[:, 0])
            ys.extend(float(b) for b in r[:, 1])
            self.ranges.append((start, len(xs)))
        self.x, self.y = xs, ys
        self.orig = list(range(len(xs)))
        span = max(max(xs) - min(xs), max(ys) - min(ys)) if xs else 1.0
        self.eps = 1e-13 * (span or 1.0) ** 2
        self.prv = [0] * len(xs)
        self.nxt = [0] * len(xs)
        self.fallbacks = 0

    # -- ring construction -------------------------------------------------
    def _link(self, start: int, end: int, ccw: bool) -> List[int]:
        idx = list(range(start, end))
        if (_ring_area(self.x, self.y, idx) > 0) != ccw:
            idx.reverse()
        for a, b in zip(idx, idx[1:] + idx[:1]):
            self.nxt[a] = b
            self.prv[b] = a
        return idx

    def _dup(self, i: int) -> int:
        self.x.append(self.x[i])
        self.y.append(self.y[i])
        self.orig.append(self.orig[i])
        self.prv.append(0)
        self.nxt.append(0)
        return len(self.x) - 1

    def _ring_nodes(self, start: int) -> List[int]:
        out = [start]
        i = self.nxt[start]
        while i != start:
            out.append(i)
            i = self.nxt[i]
        return out

    def _cross(self, a: int, b: int, c: int) -> float:
        x, y = self.x, self.y
        return (x[b] - x[a]) * (y[c] - y[a]) - (y[b] - y[a]) * (x[c] - x[a])

    def _locally_inside(self, a: int, px: float, py: float) -> bool:
        """Is direction a -> (px, py) inside the polygon's interior angle at vertex a?"""
        x, y, eps = self.x, self.y, self.eps
        ax, ay = x[a], y[a]
        nx, ny = x[self.nxt[a]] - ax, y[self.nxt[a]] - ay
        qx, qy = x[self.prv[a]] - ax, y[self.prv[a]] - ay
        dx, dy = px - ax, py - ay
        if nx * qy - ny * qx > eps:  # convex corner: sector from next to prev
            return nx * dy - ny * dx > eps and dx * qy - dy * qx > eps
        return not (qx * dy - qy * dx >= -eps and dx * ny - dy * nx >= -eps)

    def _blocked(self, a: int, b: int, segs: np.ndarray, verts: np.ndarray) -> bool:
        """Does segment a-b cross a ring edge or pass through a ring vertex?"""
        eps = self.eps
        ax, ay, bx, by = self.x[a], self.y[a], self.x[b], self.y[b]
        x0, y0, x1, y1 = segs.T
        o1 = (bx - ax) * (y0 - ay) - (by - ay) * (x0 - ax)
        o2 = (bx - ax) * (y1 - ay) - (by - ay) * (x1 - ax)
        o3 = (x1 - x0) * (ay - y0) - (y1 - y0) * (ax - x0)
        o4 = (x1 - x0) * (by - y0) - (y1 - y0) * (bx - x0)
        proper = (((o1 > eps) & (o2 < -eps)) | ((o1 < -eps) & (o2 > eps))) & \
                 (((o3 > eps) & (o4 < -eps)) | ((o3 < -eps) & (o4 > eps)))
        if proper.any():
            return True
        vx, vy = verts.T
        not_end = ~(((vx == ax) & (vy == ay)) | ((vx == bx) & (vy == by)))
        cr = (bx - ax) * (vy - ay) - (by - ay) * (vx - ax)
        t = (vx - ax) * (bx - ax) + (vy - ay) * (by - ay)
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        on = not_end & (np.abs(cr) <= eps) & (t > 0) & (t < L2)
        return bool(on.any())

    def _all_segments(self, ring_starts: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
        segs, verts = [], []
        for s in ring_starts:
            for i in self._ring_nodes(s):
                j = self.nxt[i]
                segs.append((self.x[i], self.y[i], self.x[j], self.y[j]))
                verts.append((self.x[i], self.y[i]))
        return np.array(segs, float).reshape(-1, 4), np.array(verts, float).reshape(-1, 2)

    def _bridge(self, outer: int, hole: int, others: Sequence[int]) -> None:
        """Connect the hole ring (starting node ``hole`` = its leftmost vertex) into the outer ring."""
        segs, verts = self._all_segments([outer, hole, *others])
        nodes = self._ring_nodes(outer)
        hole_nodes = sorted(self._ring_nodes(hole), key=lambda i: (self.x[i], self.y[i]))
        for h in hole_nodes:
            hx, hy = self.x[h], self.y[h]
            order = sorted(nodes, key=lambda i: (self.x[i] - hx) ** 2 + (self.y[i] - hy) ** 2)
            for c in order:
                if not self._locally_inside(c, hx, hy) or not self._locally_inside(h, self.x[c], self.y[c]):
                    continue
                if self._blocked(c, h, segs, verts):
                    continue
                c2, h2 = self._dup(c), self._dup(h)
                cn, hp = self.nxt[c], self.prv[h]
                self.nxt[c], self.prv[h] = h, c
                self.nxt[hp], self.prv[h2] = h2, hp
                self.nxt[h2], self.prv[c2] = c2, h2
                self.nxt[c2], self.prv[cn] = cn, c2
                return
        raise StepMeshError("could not bridge a hole to the outer boundary (self-intersecting loops?)")

    # -- clipping ------------------------------------------------------------
    def triangulate(self) -> List[Tuple[int, int, int]]:
        if not self.ranges or self.ranges[0][1] - self.ranges[0][0] < 3:
            return []
        outer = self._link(*self.ranges[0], ccw=True)[0]
        holes = []
        for start, end in self.ranges[1:]:
            if end - start >= 3:
                idx = self._link(start, end, ccw=False)
                holes.append(min(idx, key=lambda i: (self.x[i], self.y[i])))
        holes.sort(key=lambda i: self.x[i])
        for k, h in enumerate(holes):
            self._bridge(outer, h, holes[k + 1:])
        return self._clip(outer)

    def _ear_ok(self, a: int, b: int, c: int, X: np.ndarray, Y: np.ndarray, alive: np.ndarray,
                strict: bool) -> bool:
        x, y, eps = self.x, self.y, self.eps
        ax, ay, bx, by, cx, cy = x[a], y[a], x[b], y[b], x[c], y[c]
        m = alive & (X >= min(ax, bx, cx) - eps) & (X <= max(ax, bx, cx) + eps) & \
            (Y >= min(ay, by, cy) - eps) & (Y <= max(ay, by, cy) + eps)
        m[a] = m[b] = m[c] = False
        idx = np.flatnonzero(m)
        if idx.size == 0:
            return True
        px, py = X[idx], Y[idx]
        coincident = ((px == ax) & (py == ay)) | ((px == bx) & (py == by)) | ((px == cx) & (py == cy))
        lim = eps if strict else -eps
        d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
        d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
        inside = (d1 >= lim) & (d2 >= lim) & (d3 >= lim) & ~coincident
        return not inside.any()

    def _clip(self, start: int) -> List[Tuple[int, int, int]]:
        X, Y = np.array(self.x), np.array(self.y)
        alive = np.zeros(len(self.x), dtype=bool)
        nodes = self._ring_nodes(start)
        alive[nodes] = True
        count = len(nodes)
        prv, nxt, orig, eps = self.prv, self.nxt, self.orig, self.eps
        out: List[Tuple[int, int, int]] = []

        def clip(b: int) -> int:
            a, c = prv[b], nxt[b]
            if self._cross(a, b, c) > eps:
                out.append((orig[a], orig[b], orig[c]))
            nxt[a], prv[c] = c, a
            alive[b] = False
            return c

        ear = stop = start
        while count > 3:
            a, c = prv[ear], nxt[ear]
            if self._cross(a, ear, c) > eps and self._ear_ok(a, ear, c, X, Y, alive, strict=False):
                ear = stop = clip(ear)
                count -= 1
                continue
            ear = c
            if ear != stop:
                continue
            # A full pass found no clean ear (numerical trouble): clip the convex corner
            # with no vertex strictly inside, else the largest convex corner.
            self.fallbacks += 1
            ring = self._ring_nodes(ear)
            convex = [b for b in ring if self._cross(prv[b], b, nxt[b]) > eps]
            if not convex:
                return out  # remaining ring is degenerate (zero area)
            relaxed = [b for b in convex if self._ear_ok(prv[b], b, nxt[b], X, Y, alive, strict=True)]
            pick = max(relaxed or convex, key=lambda b: self._cross(prv[b], b, nxt[b]))
            ear = stop = clip(pick)
            count -= 1
        if count == 3:
            a = ear
            b = nxt[a]
            c = nxt[b]
            if self._cross(a, b, c) > eps:
                out.append((orig[a], orig[b], orig[c]))
        return out


class _Mesh2D:
    """A 2-D triangulation with adjacency, Lawson flips and point insertion.

    ``tri[t]`` is CCW; ``nbr[t][i]`` is the triangle across the edge opposite
    vertex ``i`` (``-1`` on the domain boundary, which is never flipped).
    """

    def __init__(self, x: Sequence[float], y: Sequence[float], tris: Iterable[Sequence[int]]) -> None:
        self.x = [float(a) for a in x]
        self.y = [float(b) for b in y]
        self.tri = [list(t) for t in tris]
        self.nbr = [[-1, -1, -1] for _ in self.tri]
        edges: Dict[Tuple[int, int], Tuple[int, int]] = {}
        for t, (a, b, c) in enumerate(self.tri):
            edges[(b, c)] = (t, 0)
            edges[(c, a)] = (t, 1)
            edges[(a, b)] = (t, 2)
        for (p, q), (t, i) in edges.items():
            o = edges.get((q, p))
            if o is not None:
                self.nbr[t][i] = o[0]
        xs = self.x or [0.0]
        ys = self.y or [0.0]
        self.scale = max(max(xs) - min(xs), max(ys) - min(ys)) or 1.0
        self.eps = 1e-13 * self.scale ** 2

    def orient(self, a: int, b: int, c: int) -> float:
        x, y = self.x, self.y
        return (x[b] - x[a]) * (y[c] - y[a]) - (y[b] - y[a]) * (x[c] - x[a])

    def _incircle(self, a: int, b: int, c: int, d: int) -> bool:
        x, y = self.x, self.y
        adx, ady = x[a] - x[d], y[a] - y[d]
        bdx, bdy = x[b] - x[d], y[b] - y[d]
        cdx, cdy = x[c] - x[d], y[c] - y[d]
        ad, bd, cd = adx * adx + ady * ady, bdx * bdx + bdy * bdy, cdx * cdx + cdy * cdy
        det = adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx)
        perm = (abs(adx) * (abs(bdy) * cd + bd * abs(cdy)) + abs(ady) * (abs(bdx) * cd + bd * abs(cdx))
                + ad * (abs(bdx * cdy) + abs(bdy * cdx)))
        return det > 1e-10 * perm

    def _relink(self, t: int, old: int, new: int) -> None:
        if t >= 0:
            n = self.nbr[t]
            n[n.index(old)] = new

    def legalize(self, stack: List[Tuple[int, int]], limit: int = 1_000_000) -> None:
        """Flip non-Delaunay interior edges; stack items are (triangle, index of apex opposite the edge)."""
        tri, nbr = self.tri, self.nbr
        while stack and limit > 0:
            limit -= 1
            t, i = stack.pop()
            s = nbr[t][i]
            if s < 0:
                continue
            p, u, v = tri[t][i], tri[t][(i + 1) % 3], tri[t][(i + 2) % 3]
            j = nbr[s].index(t)
            d = tri[s][j]
            if not self._incircle(p, u, v, d):
                continue
            if self.orient(p, u, d) <= self.eps or self.orient(p, d, v) <= self.eps:
                continue  # quadrilateral not strictly convex
            t_pu, t_vp = nbr[t][(i + 2) % 3], nbr[t][(i + 1) % 3]
            s_ud, s_dv = nbr[s][(j + 1) % 3], nbr[s][(j + 2) % 3]
            tri[t], nbr[t] = [p, u, d], [s_ud, s, t_pu]
            tri[s], nbr[s] = [p, d, v], [s_dv, t_vp, t]
            self._relink(s_ud, s, t)
            self._relink(t_vp, t, s)
            stack.extend(((t, 0), (t, 2), (s, 0), (s, 1)))

    def legalize_all(self) -> None:
        self.legalize([(t, i) for t in range(len(self.tri)) for i in range(3)])

    def locate(self, px: float, py: float, start: int) -> int:
        """Triangle containing (px, py): visibility walk, brute force if the walk leaves the domain."""
        x, y, tri, nbr, eps = self.x, self.y, self.tri, self.nbr, self.eps
        t = start if 0 <= start < len(tri) else 0
        for step in range(4 * len(tri) + 16):
            a, b, c = tri[t]
            r = step % 3  # rotate the test order so the walk cannot cycle
            moved = False
            for k in range(3):
                i = (k + r) % 3
                p, q = (b, c) if i == 0 else ((c, a) if i == 1 else (a, b))
                if (x[q] - x[p]) * (py - y[p]) - (y[q] - y[p]) * (px - x[p]) < -eps:
                    t = nbr[t][i]
                    moved = True
                    break
            if not moved:
                return t
            if t < 0:
                break
        for t, (a, b, c) in enumerate(tri):
            if all((x[q] - x[p]) * (py - y[p]) - (y[q] - y[p]) * (px - x[p]) >= -eps
                   for p, q in ((b, c), (c, a), (a, b))):
                return t
        return -1

    def insert(self, px: float, py: float, t: int) -> int:
        """Insert a point strictly inside triangle ``t``; returns -1 if it is too close to an edge."""
        a, b, c = self.tri[t]
        x, y = self.x, self.y
        area = self.orient(a, b, c)
        o = ((x[c] - x[b]) * (py - y[b]) - (y[c] - y[b]) * (px - x[b]),
             (x[a] - x[c]) * (py - y[c]) - (y[a] - y[c]) * (px - x[c]),
             (x[b] - x[a]) * (py - y[a]) - (y[b] - y[a]) * (px - x[a]))
        if area <= 0 or min(o) <= 1e-6 * area:
            return -1
        p = len(x)
        x.append(px)
        y.append(py)
        na, nb, nc = self.nbr[t]
        t1, t2 = len(self.tri), len(self.tri) + 1
        self.tri[t], self.nbr[t] = [a, b, p], [t1, t2, nc]
        self.tri.append([b, c, p])
        self.nbr.append([t2, t, na])
        self.tri.append([c, a, p])
        self.nbr.append([t, t1, nb])
        self._relink(na, t, t1)
        self._relink(nb, t, t2)
        self.legalize([(t, 2), (t1, 2), (t2, 2)])
        return t


def _point_in_polygon(px: np.ndarray, py: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Even-odd test of points against a set of closed-ring edges (S, 4)."""
    inside = np.zeros(px.shape, dtype=bool)
    for x0, y0, x1, y1 in segs:
        if y0 == y1:
            continue
        crosses = (y0 > py) != (y1 > py)
        xint = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (px < xint)
    return inside


def _segment_distance(px: np.ndarray, py: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest of the segments (S, 4)."""
    x0, y0, x1, y1 = (segs[:, k][None, :] for k in range(4))
    dx, dy = x1 - x0, y1 - y0
    L2 = np.maximum(dx * dx + dy * dy, 1e-300)
    t = np.clip(((px[:, None] - x0) * dx + (py[:, None] - y0) * dy) / L2, 0.0, 1.0)
    ex = px[:, None] - (x0 + t * dx)
    ey = py[:, None] - (y0 + t * dy)
    return np.sqrt((ex * ex + ey * ey).min(axis=1))


# ==========================================================================
# Curve sampling
# ==========================================================================

def _sample_params(f: Callable[[np.ndarray], np.ndarray], t0: float, t1: float, n0: int,
                   tol: float, max_edge: Optional[float]) -> np.ndarray:
    """Parameters from t0 to t1 such that each chord deviates <= tol from the curve and,
    when ``max_edge`` is given, is no longer than it."""
    ts = np.linspace(t0, t1, max(1, n0) + 1)
    for _ in range(40):
        P = f(ts)
        mid = 0.5 * (ts[:-1] + ts[1:])
        Pm = f(mid)
        sag = np.linalg.norm(Pm - 0.5 * (P[:-1] + P[1:]), axis=1)
        k = np.ones(len(sag), dtype=np.int64)
        bad = sag > tol
        k[bad] = np.maximum(2, np.ceil(np.sqrt(sag[bad] / tol))).astype(np.int64)
        if max_edge:
            arc = np.linalg.norm(Pm - P[:-1], axis=1) + np.linalg.norm(P[1:] - Pm, axis=1)
            k = np.maximum(k, np.ceil(arc / max_edge - 1e-9).astype(np.int64))
        if (k <= 1).all():
            break
        seg = np.repeat(np.arange(len(k)), k)
        first = np.repeat(np.cumsum(k) - k, k)
        j = np.arange(int(k.sum())) - first + 1
        frac = j / k[seg]
        ts = np.concatenate([ts[:1], ts[seg] + (ts[seg + 1] - ts[seg]) * frac])
    return ts


# ==========================================================================
# B-rep tessellation
# ==========================================================================

@dataclass
class _Loop:
    points: np.ndarray        # (n, 3) boundary polyline (closed implicitly), or (1, 3) for a vertex loop
    vertex: bool              # VERTEX_LOOP
    outer: bool               # declared FACE_OUTER_BOUND


class _Tessellator:
    """Tessellates the MANIFOLD_SOLID_BREPs of one representation (one unit context)."""

    def __init__(self, sf: StepFile, length_scale: float, angle_scale: float,
                 chord_tol: float, max_edge: Optional[float]) -> None:
        if chord_tol <= 0:
            raise ValueError("chord_tol must be positive")
        self.sf = sf
        self.ls = length_scale
        self.angle = angle_scale
        self.tol = chord_tol
        self.max_edge = max_edge if max_edge else None
        self._curves: Dict[int, Tuple[_Curve, bool]] = {}
        self._surfaces: Dict[int, _Surface] = {}
        self._edges: Dict[int, np.ndarray] = {}   # per solid: EDGE_CURVE id -> polyline
        self._fine: set = set()                   # per solid: edges bordering a curved face
        self.notes: List[str] = []

    # -- primitive entities -------------------------------------------------
    def point(self, ref: Ref) -> np.ndarray:
        e = self.sf.get(ref)
        if e.type != "CARTESIAN_POINT":
            raise StepMeshError(f"#{e.id}: expected CARTESIAN_POINT, got {e.type}")
        c = list(e.args[1]) + [0.0] * (3 - len(e.args[1]))
        return np.array(c[:3], dtype=float) * self.ls

    def direction(self, ref: Ref) -> np.ndarray:
        e = self.sf.get(ref)
        if e.type != "DIRECTION":
            raise StepMeshError(f"#{e.id}: expected DIRECTION, got {e.type}")
        c = list(e.args[1]) + [0.0] * (3 - len(e.args[1]))
        return _unit(np.array(c[:3], dtype=float))

    def vector(self, ref: Ref) -> np.ndarray:
        e = self.sf.get(ref)
        if e.type == "DIRECTION":
            return self.direction(ref)
        if e.type != "VECTOR":
            raise StepMeshError(f"#{e.id}: expected VECTOR, got {e.type}")
        return self.direction(e.args[1]) * float(e.args[2]) * self.ls

    def placement(self, ref: Ref) -> _Frame:
        return _placement(self.sf, ref, self.ls)

    def vertex_point(self, ref: Ref) -> np.ndarray:
        e = self.sf.get(ref)
        if e.type != "VERTEX_POINT":
            raise StepMeshError(f"#{e.id}: expected VERTEX_POINT, got {e.type}")
        return self.point(e.args[1])

    # -- curves ----------------------------------------------------------------
    def curve(self, ref: Ref) -> Tuple[_Curve, bool]:
        """(curve, reversed) for an edge geometry; ``reversed`` is set by a TRIMMED_CURVE
        whose sense disagrees with its basis curve."""
        hit = self._curves.get(ref.id)
        if hit is None:
            hit = self._curves[ref.id] = self._make_curve(self.sf.get(ref))
        return hit

    def _make_curve(self, e: Entity) -> Tuple[_Curve, bool]:
        t = e.type
        if t == "LINE":
            return _Line(self.point(e.args[1]), self.vector(e.args[2])), False
        if t == "CIRCLE":
            r = float(e.args[2]) * self.ls
            return _Conic(self.placement(e.args[1]), r, r), False
        if t == "ELLIPSE":
            return _Conic(self.placement(e.args[1]), float(e.args[2]) * self.ls,
                          float(e.args[3]) * self.ls), False
        if t in ("SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"):
            return self.curve(e.args[1])
        if t == "TRIMMED_CURVE":
            basis, rev = self.curve(e.args[1])
            return basis, rev != (not bool(e.args[4]))
        if t == "POLYLINE":
            return _Polyline(np.array([self.point(p) for p in e.args[1]])), False
        if e.isa("B_SPLINE_CURVE_WITH_KNOTS"):
            if e.is_complex:
                degree, cps = e.part("B_SPLINE_CURVE")[0:2]
                mults, knots = e.part("B_SPLINE_CURVE_WITH_KNOTS")[0:2]
                W = np.array(e.part("RATIONAL_B_SPLINE_CURVE")[0], float) if e.isa("RATIONAL_B_SPLINE_CURVE") else None
            else:
                degree, cps = e.args[1], e.args[2]
                mults, knots = e.args[6], e.args[7]
                W = None
            P = np.array([self.point(p) for p in cps])
            return _BSplineCurve(int(degree), P, _knot_vector(mults, knots), W), False
        raise StepMeshError(f"#{e.id}: unsupported curve type {t}")

    # -- surfaces --------------------------------------------------------------
    def surface(self, ref: Ref) -> _Surface:
        s = self._surfaces.get(ref.id)
        if s is None:
            s = self._surfaces[ref.id] = self._make_surface(self.sf.get(ref))
        return s

    def _make_surface(self, e: Entity) -> _Surface:
        t = e.type
        if t == "PLANE":
            return _Plane(self.placement(e.args[1]))
        if t == "CYLINDRICAL_SURFACE":
            return _Cylinder(self.placement(e.args[1]), float(e.args[2]) * self.ls)
        if t == "CONICAL_SURFACE":
            return _Cone(self.placement(e.args[1]), float(e.args[2]) * self.ls, float(e.args[3]) * self.angle)
        if t == "SPHERICAL_SURFACE":
            return _Sphere(self.placement(e.args[1]), float(e.args[2]) * self.ls)
        if t == "TOROIDAL_SURFACE":
            return _Torus(self.placement(e.args[1]), float(e.args[2]) * self.ls, float(e.args[3]) * self.ls)
        if t == "SURFACE_OF_LINEAR_EXTRUSION":
            curve, _ = self.curve(e.args[1])
            return _Extrusion(curve, self.vector(e.args[2]))
        if t == "SURFACE_OF_REVOLUTION":
            curve, _ = self.curve(e.args[1])
            ax = self.sf.get(e.args[2])  # AXIS1_PLACEMENT(name, location, axis)
            axis = self.direction(ax.args[2]) if ax.args[2] is not None else np.array([0.0, 0.0, 1.0])
            return _Revolution(curve, self.point(ax.args[1]), axis)
        if e.isa("B_SPLINE_SURFACE_WITH_KNOTS"):
            if e.is_complex:
                pu, pv, cps = e.part("B_SPLINE_SURFACE")[0:3]
                um, vm, uk, vk = e.part("B_SPLINE_SURFACE_WITH_KNOTS")[0:4]
                W = (np.array(e.part("RATIONAL_B_SPLINE_SURFACE")[0], float)
                     if e.isa("RATIONAL_B_SPLINE_SURFACE") else None)
            else:
                pu, pv, cps = e.args[1:4]
                um, vm, uk, vk = e.args[8:12]
                W = None
            P = np.array([[self.point(p) for p in row] for row in cps])
            return _BSplineSurface(int(pu), int(pv), P, _knot_vector(um, uk), _knot_vector(vm, vk), W)
        raise StepMeshError(f"#{e.id}: unsupported surface type {t}")

    # -- edges -------------------------------------------------------------------
    def edge_points(self, ref: Ref) -> np.ndarray:
        """Polyline of an EDGE_CURVE from edge_start to edge_end, cached per solid so that
        both faces sharing the edge use identical points.

        Edges bordering a curved face also obey ``max_edge``; edges between two
        planar faces only obey the chordal tolerance (straight ones stay one segment)."""
        hit = self._edges.get(ref.id)
        if hit is not None:
            return hit
        ec = self.sf.get(ref)
        if ec.type != "EDGE_CURVE":
            raise StepMeshError(f"#{ec.id}: unsupported edge type {ec.type}")
        ps, pe = self.vertex_point(ec.args[1]), self.vertex_point(ec.args[2])
        curve, rev = self.curve(ec.args[3])
        sense = bool(ec.args[4]) != rev
        max_edge = self.max_edge if ref.id in self._fine else None
        closed = ec.args[1] == ec.args[2] or bool(np.linalg.norm(pe - ps) <= 1e-12 * max(1.0, float(np.abs(ps).max())))
        if isinstance(curve, _Line):
            n = int(math.ceil(np.linalg.norm(pe - ps) / max_edge - 1e-9)) if max_edge else 1
            pts = ps + (pe - ps) * np.linspace(0.0, 1.0, max(n, 1) + 1)[:, None]
        else:
            t0, t1 = (float(a) for a in curve.param(np.stack([ps, pe])))
            if curve.period:
                per = curve.period
                d = (t1 - t0) % per if sense else -((t0 - t1) % per)
                if closed or abs(d) < 1e-12 * per:
                    d = per if sense else -per
                t1 = t0 + d
            elif closed:
                lo, hi = curve.domain() or (t0, t1)
                t0, t1 = (lo, hi) if sense else (hi, lo)
            ts = _sample_params(curve.eval, t0, t1, curve.initial_segments(t0, t1), self.tol, max_edge)
            pts = curve.eval(ts)
            dev = max(np.linalg.norm(pts[0] - ps), np.linalg.norm(pts[-1] - pe))
            if dev > max(1e-4, 10 * self.tol):
                raise StepMeshError(f"#{ec.id}: edge vertices are {dev:.3g} m off their curve")
            pts[0], pts[-1] = ps, pe
        self._edges[ref.id] = pts
        return pts

    def _edge_refs(self, face: Entity) -> Iterable[Ref]:
        for bref in face.args[1]:
            lp = self.sf.get(self.sf.get(bref).args[1])
            if lp.type == "EDGE_LOOP":
                for oref in lp.args[1]:
                    yield self.sf.get(oref).args[3]

    def face_loops(self, face: Entity) -> List[_Loop]:
        loops = []
        for bref in face.args[1]:
            b = self.sf.get(bref)
            if b.type not in ("FACE_BOUND", "FACE_OUTER_BOUND"):
                raise StepMeshError(f"#{b.id}: unsupported face bound {b.type}")
            lp = self.sf.get(b.args[1])
            forward = bool(b.args[2])
            if lp.type == "VERTEX_LOOP":
                loops.append(_Loop(self.vertex_point(lp.args[1])[None, :], True, b.type == "FACE_OUTER_BOUND"))
                continue
            if lp.type == "POLY_LOOP":
                pts = np.array([self.point(p) for p in lp.args[1]])
            elif lp.type == "EDGE_LOOP":
                chunks = []
                for oref in lp.args[1]:
                    oe = self.sf.get(oref)
                    if oe.type != "ORIENTED_EDGE":
                        raise StepMeshError(f"#{oe.id}: expected ORIENTED_EDGE, got {oe.type}")
                    pts = self.edge_points(oe.args[3])
                    chunks.append(pts if oe.args[4] else pts[::-1])
                for k, ch in enumerate(chunks):
                    nxt = chunks[(k + 1) % len(chunks)]
                    gap = np.linalg.norm(ch[-1] - nxt[0])
                    if gap > 1e-9 * max(1.0, float(np.abs(ch[-1]).max())):
                        raise StepMeshError(f"#{lp.id}: edge loop is not connected (gap {gap:.3g} m)")
                pts = np.concatenate([ch[:-1] for ch in chunks])
            else:
                raise StepMeshError(f"#{lp.id}: unsupported loop type {lp.type}")
            if not forward:
                pts = pts[::-1]
            loops.append(_Loop(pts, False, b.type == "FACE_OUTER_BOUND"))
        return loops

    # -- solids ------------------------------------------------------------------
    def solid(self, msb: Entity) -> np.ndarray:
        """World-independent triangles (N, 3, 3) of a MANIFOLD_SOLID_BREP, oriented outward."""
        self._edges = {}
        self._fine = set()
        self.notes = []
        if msb.type != "MANIFOLD_SOLID_BREP":
            raise StepMeshError(f"#{msb.id}: unsupported solid type {msb.type}")
        shell = self.sf.get(msb.args[1])
        if shell.type != "CLOSED_SHELL":
            raise StepMeshError(f"#{shell.id}: unsupported shell type {shell.type}")
        faces: List[Tuple[Entity, bool]] = []
        for fref in shell.args[1]:
            f = self.sf.get(fref)
            if f.type == "ORIENTED_FACE":  # (name, *, face_element, orientation)
                inner = self.sf.get(f.args[2])
                faces.append((inner, bool(f.args[3]) == bool(inner.args[3])))
            elif f.type in ("ADVANCED_FACE", "FACE_SURFACE"):
                faces.append((f, bool(f.args[3])))
            else:
                raise StepMeshError(f"#{f.id}: unsupported face type {f.type}")
        # Edges of curved faces get the max_edge limit; decide before any edge is sampled
        # so that the planar neighbour sees the same points.
        for f, _ in faces:
            if self.surface(f.args[2]).kind != "plane":
                self._fine.update(r.id for r in self._edge_refs(f))
        out = []
        for f, sense in faces:
            surf = self.surface(f.args[2])
            loops = self.face_loops(f)
            try:
                if surf.kind == "plane":
                    tris = self._planar_face(surf, loops, sense, f.id)
                else:
                    tris = _ParametricFace(self, surf, loops, sense, f.id).run()
            except StepMeshError:
                raise
            except Exception as exc:  # add the face to the message, keep the traceback
                raise StepMeshError(f"face #{f.id} ({surf.kind}): {exc!r}") from exc
            out.append(tris)
        return np.concatenate(out) if out else np.zeros((0, 3, 3))

    def _planar_face(self, plane: _Plane, loops: List[_Loop], sense: bool, fid: int) -> np.ndarray:
        rings2d, rings3d = [], []
        outer_flags = []
        for lp in loops:
            if lp.vertex:
                self.notes.append(f"face #{fid}: VERTEX_LOOP on a plane ignored")
                continue
            loc = plane.f.local(lp.points)
            off = np.abs(loc[:, 2]).max(initial=0.0)
            if off > max(1e-6, self.tol):
                raise StepMeshError(f"face #{fid}: boundary is {off:.3g} m off its plane")
            rings2d.append(loc[:, :2])
            rings3d.append(lp.points)
            outer_flags.append(lp.outer)
        if not rings2d:
            return np.zeros((0, 3, 3))
        areas = [abs(_ring_area(r[:, 0], r[:, 1], range(len(r)))) for r in rings2d]
        outer = outer_flags.index(True) if any(outer_flags) else int(np.argmax(areas))
        # Cross-check: the outer loop should run counter-clockwise about the face normal.
        signed = _ring_area(rings2d[outer][:, 0], rings2d[outer][:, 1], range(len(rings2d[outer])))
        if (signed > 0) != sense:
            self.notes.append(f"face #{fid}: outer loop direction disagrees with face normal")
        order = [outer] + [k for k in range(len(rings2d)) if k != outer]
        rings2d = [rings2d[k] for k in order]
        pts3 = np.concatenate([rings3d[k] for k in order])
        ec = _EarCut(rings2d)
        tri = ec.triangulate()
        if ec.fallbacks:
            self.notes.append(f"face #{fid}: ear clipping needed {ec.fallbacks} fallback(s)")
        xy = np.concatenate(rings2d)
        mesh = _Mesh2D(xy[:, 0], xy[:, 1], tri)
        mesh.legalize_all()
        idx = np.array(mesh.tri, dtype=int).reshape(-1, 3)
        if not sense:
            idx = idx[:, ::-1]
        return pts3[idx]


class _ParametricFace:
    """Mesh one curved face in its (u, v) parameter domain.

    The boundary loops are inverted onto the surface and unwrapped across the
    seam of periodic directions.  The trimmed domain then becomes one polygon
    (with holes) in "metric" coordinates (s_u u, s_v v), where s_u, s_v are the
    largest surface speeds |S_u|, |S_v| over the face, so that distances in the
    polygon bound distances on the surface:

    * no loop wraps around a periodic direction: the loops are an outer boundary
      and holes (holes are shifted by whole periods into the outer's range);
    * two loops wrap (a full band, e.g. a cylinder between two circles): they are
      joined by a cut along the periodic direction, the cut appearing twice;
    * one loop wraps and the other side is a pole (VERTEX_LOOP, or a natural
      pole such as a sphere's): the polygon is closed along the pole iso-line.

    The polygon is ear-clipped, made Delaunay, filled with a hexagonal lattice of
    spacing ~``max_edge`` and refined (centroid insertion) until every interior
    edge is shorter than ``max_edge`` and every edge midpoint / triangle centroid
    is within ``chord_tol`` of the surface.  Boundary vertices keep the exact 3-D
    points of the shared edge discretisation.
    """

    MAX_REFINE_ROUNDS = 12

    def __init__(self, owner: _Tessellator, surf: _Surface, loops: List[_Loop], sense: bool, fid: int) -> None:
        self.o, self.s, self.loops, self.sense, self.fid = owner, surf, loops, sense, fid
        self.tol = owner.tol
        self.max_edge = owner.max_edge
        self.err = f"face #{fid} ({surf.kind})"

    # -- loop mapping ----------------------------------------------------------
    def _uv_loop(self, pts: np.ndarray) -> Tuple[np.ndarray, List[int]]:
        u, v = self.s.invert(pts)
        uv = np.stack([u, v], axis=1)
        resid = np.linalg.norm(self.s.eval(uv[:, 0], uv[:, 1]) - pts, axis=1).max()
        if resid > max(1e-4, 2 * self.tol):
            raise StepMeshError(f"{self.err}: boundary is {resid:.3g} m off its surface")
        winding = [0, 0]
        for k, per in enumerate(self.s.periods):
            if per:
                d = _wrap(np.diff(uv[:, k], append=uv[0, k]), per)
                uv[1:, k] = uv[0, k] + np.cumsum(d[:-1])
                winding[k] = int(round(d.sum() / per))
        return uv, winding

    def _check_poles(self, pts: np.ndarray) -> None:
        for j, val in self.s.poles():
            q = self._pole_point(j, val)
            if (np.linalg.norm(pts - q, axis=1) < 1e-7).any():
                raise StepMeshError(f"{self.err}: an edge loop passes through a surface pole "
                                    "(only VERTEX_LOOP poles are supported)")

    def _pole_point(self, j: int, val: float) -> np.ndarray:
        other = 0.0
        b = self.s.bounds[1 - j]
        if b is not None:
            other = 0.5 * (b[0] + b[1])
        uv = (val, other) if j == 0 else (other, val)
        return self.s.eval(np.array([uv[0]]), np.array([uv[1]]))[0]

    # -- polygon construction ----------------------------------------------------
    def run(self) -> np.ndarray:
        edge_loops = [lp for lp in self.loops if not lp.vertex]
        vloops = [lp for lp in self.loops if lp.vertex]
        uvs, winds = [], []
        for lp in edge_loops:
            self._check_poles(lp.points)
            uv, w = self._uv_loop(lp.points)
            uvs.append(uv)
            winds.append(w)
        dirs = {k for w in winds for k in (0, 1) if w[k] != 0}
        if len(dirs) > 1 or any(abs(w[k]) > 1 for w in winds for k in (0, 1)):
            raise StepMeshError(f"{self.err}: loops wrap around both parameter directions (unsupported)")
        k = dirs.pop() if dirs else None
        wrap = [i for i, w in enumerate(winds) if k is not None and w[k] != 0]
        plain = [i for i in range(len(uvs)) if i not in wrap]

        # Each ring: (uv (n,2), xyz (n,3) with NaN where the surface must be evaluated).
        rings: List[Tuple[np.ndarray, np.ndarray]] = []
        pole_val: Optional[Tuple[int, float, np.ndarray]] = None
        if k is None:
            if vloops:
                raise StepMeshError(f"{self.err}: VERTEX_LOOP without a loop around the pole (unsupported)")
            if not plain:
                return np.zeros((0, 3, 3))
            areas = [abs(_ring_area(uv[:, 0], uv[:, 1], range(len(uv)))) for uv in uvs]
            flags = [edge_loops[i].outer for i in plain]
            outer = plain[flags.index(True)] if any(flags) else plain[int(np.argmax([areas[i] for i in plain]))]
            centre = 0.5 * (uvs[outer].min(0) + uvs[outer].max(0))
            rings.append((uvs[outer], edge_loops[outer].points))
            for i in plain:
                if i != outer:
                    rings.append((self._shift_into(uvs[i], centre), edge_loops[i].points))
            self._setup_metric([r[0] for r in rings])
        else:
            P = self.s.periods[k]
            j = 1 - k
            if len(wrap) == 2 and not vloops:
                a, b = wrap
                self._setup_metric([uvs[a], uvs[b]] + [uvs[i] for i in plain])
                ring = self._band(k, P, uvs[a], winds[a][k], edge_loops[a].points,
                                  uvs[b], winds[b][k], edge_loops[b].points)
            elif len(wrap) == 1 and len(vloops) <= 1:
                a = wrap[0]
                if vloops:
                    pole_val = self._vertex_pole(vloops[0].points[0], j)
                else:
                    pole_val = self._natural_pole(uvs[a], winds[a][k], k, j)
                jv = pole_val[1]
                extra = np.array([[uvs[a][0, k], jv]]) if k == 0 else np.array([[jv, uvs[a][0, k]]])
                self._setup_metric([uvs[a], extra] + [uvs[i] for i in plain])
                ring = self._cap(k, P, uvs[a], winds[a][k], edge_loops[a].points, jv, pole_val[2])
            else:
                raise StepMeshError(f"{self.err}: unsupported boundary configuration "
                                    f"({len(wrap)} wrapping loop(s), {len(vloops)} vertex loop(s))")
            rings.append(ring)
            centre = 0.5 * (ring[0].min(0) + ring[0].max(0))
            for i in plain:
                rings.append((self._shift_into(uvs[i], centre), edge_loops[i].points))
        return self._mesh(rings)

    def _shift_into(self, uv: np.ndarray, centre: np.ndarray) -> np.ndarray:
        uv = uv.copy()
        c = 0.5 * (uv.min(0) + uv.max(0))
        for k, per in enumerate(self.s.periods):
            if per:
                uv[:, k] -= per * np.round((c[k] - centre[k]) / per)
        return uv

    def _vertex_pole(self, q: np.ndarray, j: int) -> Tuple[int, float, np.ndarray]:
        for jj, val in self.s.poles():
            if jj == j and np.linalg.norm(self._pole_point(jj, val) - q) < max(1e-6, self.tol):
                return jj, val, q
        raise StepMeshError(f"{self.err}: VERTEX_LOOP is not at a pole of the surface")

    def _natural_pole(self, uv: np.ndarray, w: int, k: int, j: int) -> Tuple[int, float, np.ndarray]:
        # The face lies left of its boundary in (u, v) when same_sense, right otherwise.
        # Moving +k with the face on the left means the face is at +j when k == u, -j when k == v.
        side = (1 if k == 0 else -1) * (1 if w > 0 else -1) * (1 if self.sense else -1)
        cands = [(jj, val) for jj, val in self.s.poles() if jj == j and (val - uv[:, j].mean()) * side > 0]
        if not cands:
            raise StepMeshError(f"{self.err}: single wrapping loop with no pole on the face side")
        jj, val = min(cands, key=lambda c: abs(c[1] - uv[:, j].mean()))
        return jj, val, self._pole_point(jj, val)

    def _setup_metric(self, uv_sets: Sequence[np.ndarray]) -> None:
        allp = np.concatenate(uv_sets)
        lo, hi = allp.min(0), allp.max(0)
        g = np.linspace(0.0, 1.0, 11)
        gu, gv = np.meshgrid(lo[0] + g * (hi[0] - lo[0]), lo[1] + g * (hi[1] - lo[1]), indexing="ij")
        _, Su, Sv = self.s.derivs(gu.ravel(), gv.ravel())
        su = float(np.linalg.norm(Su, axis=1).max())
        sv = float(np.linalg.norm(Sv, axis=1).max())
        self.scale = np.array([su if su > 1e-12 else 1.0, sv if sv > 1e-12 else 1.0])
        self.h = 0.85 * self.max_edge if self.max_edge else None
        if self.h is None:
            size = float(np.max((hi - lo) * self.scale)) or 1.0
            self.h = size / 8.0

    def _iso_points(self, uv0: np.ndarray, uv1: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Interior samples (uv, xyz) of the straight (u, v) segment uv0 -> uv1."""
        d = uv1 - uv0

        def f(s):
            q = uv0 + np.outer(s, d)
            return self.s.eval(q[:, 0], q[:, 1])

        metric = float(np.linalg.norm(d * self.scale))
        n0 = max(1, int(math.ceil(metric / self.h - 1e-9)))
        s = _sample_params(f, 0.0, 1.0, n0, self.tol, self.max_edge)[1:-1]
        q = uv0 + np.outer(s, d)
        return q, f(s) if len(s) else np.zeros((0, 3))

    def _oriented(self, uv, w, pts, want: int):
        if w != want:
            return uv[::-1].copy(), pts[::-1].copy()
        return uv, pts

    def _band(self, k, P, uva, wa, pa, uvb, wb, pb):
        ek = np.zeros(2)
        ek[k] = P
        uva, pa = self._oriented(uva, wa, pa, +1)
        uvb, pb = self._oriented(uvb, wb, pb, -1)
        a_end = uva[0] + ek
        diff = _wrap(uvb[:, k] - a_end[k], P)
        js = int(np.argmin(np.abs(diff)))
        # Start B at vertex js, re-unwrap (rotation breaks continuity at the old start),
        # and shift it by whole periods next to A's end.
        uvb = np.roll(uvb, -js, axis=0)
        pb = np.roll(pb, -js, axis=0)
        uvb[1:, k] = uvb[0, k] + np.cumsum(_wrap(np.diff(uvb[:, k]), P))
        uvb[:, k] -= P * np.round((uvb[0, k] - (a_end[k] + diff[js])) / P)
        b_end = uvb[0] - ek
        cut_uv, cut_xyz = self._iso_points(a_end, uvb[0])
        uv = np.concatenate([uva, a_end[None], cut_uv, uvb, b_end[None], cut_uv[::-1] - ek])
        xyz = np.concatenate([pa, pa[:1], cut_xyz, pb, pb[:1], cut_xyz[::-1]])
        return uv, xyz

    def _cap(self, k, P, uva, wa, pa, jv, q):
        ek = np.zeros(2)
        ek[k] = P
        j = 1 - k
        uva, pa = self._oriented(uva, wa, pa, +1)
        a_end = uva[0] + ek
        q1 = a_end.copy()
        q1[j] = jv
        q0 = uva[0].copy()
        q0[j] = jv
        cut_uv, cut_xyz = self._iso_points(a_end, q1)
        n_pole = max(1, int(math.ceil(P * self.scale[k] / self.h - 1e-9)))
        s = np.linspace(0.0, 1.0, n_pole + 1)[1:-1]
        pole_uv = q1 + np.outer(s, q0 - q1)
        pole_xyz = np.repeat(q[None], len(s) + 2, axis=0)
        uv = np.concatenate([uva, a_end[None], cut_uv, q1[None], pole_uv, q0[None], cut_uv[::-1] - ek])
        xyz = np.concatenate([pa, pa[:1], cut_xyz, pole_xyz, cut_xyz[::-1]])
        return uv, xyz

    # -- meshing --------------------------------------------------------------------
    def _mesh(self, rings: List[Tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
        sc = self.scale
        metric = [r[0] * sc for r in rings]
        ec = _EarCut(metric)
        tri = ec.triangulate()
        if ec.fallbacks:
            self.o.notes.append(f"{self.err}: ear clipping needed {ec.fallbacks} fallback(s)")
        xy = np.concatenate(metric)
        mesh = _Mesh2D(xy[:, 0], xy[:, 1], tri)
        mesh.legalize_all()
        xyz_fixed = np.concatenate([r[1] for r in rings])

        # Boundary segments for the lattice keep-out test.
        segs = []
        for r in metric:
            segs.append(np.hstack([r, np.roll(r, -1, axis=0)]))
        segs = np.concatenate(segs)
        self._fill_lattice(mesh, segs)

        xyz = self._vertex_xyz(mesh, xyz_fixed)
        for _ in range(self.MAX_REFINE_ROUNDS):
            bad = self._bad_triangles(mesh, xyz)
            if not bad:
                break
            # Centroids of this round's bad triangles (computed before any insertion).
            x, y = mesh.x, mesh.y
            cents = [((x[a] + x[b] + x[c]) / 3.0, (y[a] + y[b] + y[c]) / 3.0, t)
                     for t in bad for a, b, c in (mesh.tri[t],)]
            for cx, cy, t in cents:
                loc = mesh.locate(cx, cy, t)
                if loc >= 0:
                    mesh.insert(cx, cy, loc)
            xyz = self._vertex_xyz(mesh, xyz)
        else:
            if self._bad_triangles(mesh, xyz):
                self.o.notes.append(f"{self.err}: refinement stopped with tolerance violations")

        idx = np.array(mesh.tri, dtype=int).reshape(-1, 3)
        tris = xyz[idx]
        same = (np.all(tris[:, 0] == tris[:, 1], axis=1) | np.all(tris[:, 1] == tris[:, 2], axis=1)
                | np.all(tris[:, 2] == tris[:, 0], axis=1))
        tris = tris[~same]
        if not self.sense:
            tris = tris[:, ::-1]
        return tris

    def _fill_lattice(self, mesh: _Mesh2D, segs: np.ndarray) -> None:
        h = self.h
        xy = np.stack([mesh.x, mesh.y], axis=1)
        lo, hi = xy.min(0), xy.max(0)
        dy = h * math.sqrt(3.0) / 2.0
        rows = int(math.floor((hi[1] - lo[1]) / dy))
        cols = int(math.floor((hi[0] - lo[0]) / h)) + 1
        if rows < 1 or cols < 1:
            return
        ry = lo[1] + dy * np.arange(1, rows + 1)
        pts = []
        for r, yv in enumerate(ry):
            xs = lo[0] + h * (np.arange(cols) + (0.5 if r % 2 else 0.0))
            if r % 2:
                xs = xs[::-1]  # snake order keeps the point-location walk short
            pts.append(np.stack([xs, np.full(len(xs), yv)], axis=1))
        pts = np.concatenate(pts)
        rng = np.random.default_rng(12345)
        pts = pts + rng.uniform(-1e-3 * h, 1e-3 * h, size=pts.shape)
        keep = _point_in_polygon(pts[:, 0], pts[:, 1], segs)
        pts = pts[keep]
        if len(pts):
            pts = pts[_segment_distance(pts[:, 0], pts[:, 1], segs) > 0.45 * h]
        t = 0
        for px, py in pts:
            loc = mesh.locate(float(px), float(py), t)
            if loc >= 0:
                r = mesh.insert(float(px), float(py), loc)
                if r >= 0:
                    t = r

    def _vertex_xyz(self, mesh: _Mesh2D, known: np.ndarray) -> np.ndarray:
        """3-D positions of all mesh vertices.  ``known`` covers the first vertices (rows that
        are NaN, and every vertex beyond it, are evaluated on the surface)."""
        n = len(mesh.x)
        xyz = np.full((n, 3), np.nan)
        xyz[:len(known)] = known
        todo = np.flatnonzero(np.isnan(xyz[:, 0]))
        if len(todo):
            u = np.asarray(mesh.x)[todo] / self.scale[0]
            v = np.asarray(mesh.y)[todo] / self.scale[1]
            xyz[todo] = self.s.eval(u, v)
        return xyz

    def _bad_triangles(self, mesh: _Mesh2D, xyz: np.ndarray) -> List[int]:
        tri = np.array(mesh.tri, dtype=int).reshape(-1, 3)
        nbr = np.array(mesh.nbr, dtype=int).reshape(-1, 3)
        if not len(tri):
            return []
        X = np.asarray(mesh.x) / self.scale[0]
        Y = np.asarray(mesh.y) / self.scale[1]
        bad = np.zeros(len(tri), dtype=bool)
        mids_u, mids_v, mids_p, owners = [], [], [], []
        for i in range(3):
            a, b = tri[:, (i + 1) % 3], tri[:, (i + 2) % 3]
            interior = nbr[:, i] >= 0
            L = np.linalg.norm(xyz[b] - xyz[a], axis=1)
            if self.max_edge:
                bad |= interior & (L > self.max_edge * (1 + 1e-9))
            sel = np.flatnonzero(interior & (L > 0))
            mids_u.append(0.5 * (X[a[sel]] + X[b[sel]]))
            mids_v.append(0.5 * (Y[a[sel]] + Y[b[sel]]))
            mids_p.append(0.5 * (xyz[a[sel]] + xyz[b[sel]]))
            owners.append(sel)
        cu = X[tri].mean(1)
        cv = Y[tri].mean(1)
        mids_u.append(cu)
        mids_v.append(cv)
        mids_p.append(xyz[tri].mean(1))
        owners.append(np.arange(len(tri)))
        u = np.concatenate(mids_u)
        v = np.concatenate(mids_v)
        p = np.concatenate(mids_p)
        own = np.concatenate(owners)
        if len(u):
            sag = np.linalg.norm(self.s.eval(u, v) - p, axis=1)
            bad[own[sag > self.tol]] = True
        return [int(t) for t in np.flatnonzero(bad)]


def _placement(sf: StepFile, ref: Ref, length_scale: float) -> _Frame:
    """AXIS2_PLACEMENT_3D(name, location, axis, ref_direction) (or _2D) as a frame in metres."""
    e = sf.get(ref)
    if e.type not in ("AXIS2_PLACEMENT_3D", "AXIS2_PLACEMENT_2D"):
        raise StepMeshError(f"#{e.id}: unsupported placement {e.type}")
    loc_e = sf.get(e.args[1])
    c = list(loc_e.args[1]) + [0.0] * (3 - len(loc_e.args[1]))
    origin = np.array(c[:3], float) * length_scale

    def direction(r):
        d = sf.get(r)
        cc = list(d.args[1]) + [0.0] * (3 - len(d.args[1]))
        return _unit(np.array(cc[:3], float))

    if e.type == "AXIS2_PLACEMENT_2D":
        z = np.array([0.0, 0.0, 1.0])
        xr = direction(e.args[2]) if e.args[2] is not None else np.array([1.0, 0.0, 0.0])
    else:
        z = direction(e.args[2]) if e.args[2] is not None else np.array([0.0, 0.0, 1.0])
        xr = direction(e.args[3]) if e.args[3] is not None else None
    if xr is None or abs(float(xr @ z)) > 1 - 1e-12:
        xr = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    x = _unit(xr - (xr @ z) * z)
    return _Frame(origin, x, np.cross(z, x), z)


# ==========================================================================
# Assembly structure, colours
# ==========================================================================

@dataclass
class _Instance:
    name: str
    msb: Entity
    matrix: np.ndarray        # rep coordinates (metres) -> world (metres)
    rep: int                  # geometry representation id (for its unit context)


def _rel_reps(e: Entity) -> Tuple[Ref, Ref]:
    args = e.parts["REPRESENTATION_RELATIONSHIP"] if e.isa("REPRESENTATION_RELATIONSHIP") else e.args
    return args[2], args[3]


def _rep_scale(sf: StepFile, rep_id: int) -> Tuple[float, float]:
    rep = sf.get(rep_id)
    return _context_units(sf, rep.args[2] if len(rep.args) > 2 else None)


def _relationship_matrix(sf: StepFile, rr: Entity, child_reps: Sequence[int]) -> np.ndarray:
    """Matrix mapping child representation coordinates to parent coordinates (metres)."""
    rep1, rep2 = _rel_reps(rr)
    op = sf.get(rr.part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")[0])
    if op.type != "ITEM_DEFINED_TRANSFORMATION":
        raise StepMeshError(f"#{op.id}: unsupported transformation operator {op.type}")
    t1 = _placement(sf, op.args[2], _rep_scale(sf, rep1.id)[0]).matrix()
    t2 = _placement(sf, op.args[3], _rep_scale(sf, rep2.id)[0]).matrix()
    m12 = t2 @ np.linalg.inv(t1)  # rep_1 coordinates -> rep_2 coordinates
    child_is_rep1 = rep1.id in child_reps or rep2.id not in child_reps
    return m12 if child_is_rep1 else np.linalg.inv(m12)


def _assembly_instances(sf: StepFile) -> List[_Instance]:
    pd_name: Dict[int, str] = {}
    for pd in sf.by_type("PRODUCT_DEFINITION"):
        try:
            prod = sf.get(sf.get(pd.args[2]).args[2])
            pd_name[pd.id] = prod.args[1] or prod.args[0] or f"#{pd.id}"
        except (KeyError, IndexError):
            pd_name[pd.id] = f"#{pd.id}"
    pds_def = {p.id: p.args[2].id for p in sf.by_type("PRODUCT_DEFINITION_SHAPE") if isinstance(p.args[2], Ref)}
    pd_reps: Dict[int, List[int]] = defaultdict(list)
    for sdr in sf.by_type("SHAPE_DEFINITION_REPRESENTATION"):
        d = pds_def.get(sdr.args[0].id)
        if d is not None and d in pd_name:
            pd_reps[d].append(sdr.args[1].id)
    sdr_reps = {r for reps in pd_reps.values() for r in reps}

    children: Dict[int, List[Entity]] = defaultdict(list)
    child_pds = set()
    for nauo in sf.by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
        children[nauo.args[3].id].append(nauo)
        child_pds.add(nauo.args[4].id)
    cdsr_of: Dict[int, Entity] = {}
    for c in sf.by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
        nauo_id = pds_def.get(c.args[1].id)
        if nauo_id is not None:
            cdsr_of[nauo_id] = sf.get(c.args[0])

    links: Dict[int, List[int]] = defaultdict(list)
    seen = set()
    for rel in sf.by_type("SHAPE_REPRESENTATION_RELATIONSHIP") + sf.by_type("REPRESENTATION_RELATIONSHIP"):
        if rel.id in seen or rel.isa("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"):
            continue
        seen.add(rel.id)
        r1, r2 = _rel_reps(rel)
        if r1.id in sdr_reps and r2.id not in sdr_reps:
            links[r1.id].append(r2.id)
        elif r2.id in sdr_reps and r1.id not in sdr_reps:
            links[r2.id].append(r1.id)

    def solids_of(pd: int) -> List[Tuple[Entity, int]]:
        out: List[Tuple[Entity, int]] = []
        stack = list(pd_reps.get(pd, ()))
        visited = set()
        while stack:
            rid = stack.pop()
            if rid in visited:
                continue
            visited.add(rid)
            rep = sf.get(rid)
            for item in rep.args[1]:
                t = sf.type_of(item)
                if t == "MANIFOLD_SOLID_BREP":
                    out.append((sf.get(item), rid))
                elif t in ("BREP_WITH_VOIDS", "MAPPED_ITEM", "FACETED_BREP"):
                    raise StepMeshError(f"#{item.id}: {t} is not supported")
            stack.extend(links.get(rid, ()))
        out.sort(key=lambda s: s[0].id)
        return out

    instances: List[_Instance] = []

    def walk(pd: int, matrix: np.ndarray, name: str, depth: int) -> None:
        if depth > 32:
            raise StepMeshError("assembly nesting deeper than 32 levels (cycle?)")
        solids = solids_of(pd)
        for k, (msb, rep) in enumerate(solids):
            label = name if len(solids) == 1 else f"{name}/{msb.args[0] or k + 1}"
            instances.append(_Instance(label, msb, matrix, rep))
        for nauo in children.get(pd, ()):
            child = nauo.args[4].id
            rr = cdsr_of.get(nauo.id)
            if rr is None:
                raise StepMeshError(f"#{nauo.id}: occurrence has no CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
            m = _relationship_matrix(sf, rr, pd_reps.get(child, ()))
            walk(child, matrix @ m, nauo.args[1] or pd_name.get(child, f"#{child}"), depth + 1)

    for pd in sorted(pd_name):
        if pd not in child_pds:
            walk(pd, np.eye(4), pd_name[pd], 0)

    if not instances:  # bare geometry without product structure
        for rep in sf.by_type("ADVANCED_BREP_SHAPE_REPRESENTATION") + sf.by_type("SHAPE_REPRESENTATION"):
            for item in rep.args[1]:
                if sf.type_of(item) == "MANIFOLD_SOLID_BREP":
                    msb = sf.get(item)
                    instances.append(_Instance(msb.args[0] or f"#{msb.id}", msb, np.eye(4), rep.id))

    # Unique names (a product instanced twice gets " (2)", " (3)", ...).
    count: Dict[str, int] = defaultdict(int)
    for inst in instances:
        count[inst.name] += 1
        if count[inst.name] > 1:
            inst.name = f"{inst.name} ({count[inst.name]})"
    return instances


def _find_colour(sf: StepFile, value, depth: int = 0) -> Optional[Tuple[float, float, float]]:
    """First COLOUR_RGB reachable from a style (STYLED_ITEM -> PRESENTATION_STYLE_ASSIGNMENT ->
    SURFACE_STYLE_USAGE -> SURFACE_SIDE_STYLE -> SURFACE_STYLE_FILL_AREA -> FILL_AREA_STYLE ->
    FILL_AREA_STYLE_COLOUR -> COLOUR_RGB); ``depth`` counts entity hops."""
    if depth > 12:
        return None
    if isinstance(value, list):
        for v in value:
            c = _find_colour(sf, v, depth)
            if c is not None:
                return c
        return None
    if not isinstance(value, Ref):
        return None
    e = sf.get(value)
    if e.type == "COLOUR_RGB":
        return (float(e.args[1]), float(e.args[2]), float(e.args[3]))
    if e.type == "DRAUGHTING_PRE_DEFINED_COLOUR":
        return _PREDEFINED_COLOURS.get(str(e.args[0]).lower())
    return _find_colour(sf, e.args, depth + 1)


_PREDEFINED_COLOURS = {
    "red": (1.0, 0.0, 0.0), "green": (0.0, 1.0, 0.0), "blue": (0.0, 0.0, 1.0), "yellow": (1.0, 1.0, 0.0),
    "magenta": (1.0, 0.0, 1.0), "cyan": (0.0, 1.0, 1.0), "black": (0.0, 0.0, 0.0), "white": (1.0, 1.0, 1.0),
}


def _solid_colours(sf: StepFile) -> Dict[int, Tuple[float, float, float]]:
    out: Dict[int, Tuple[float, float, float]] = {}
    for si in sf.by_type("STYLED_ITEM"):
        item = si.args[2]
        if isinstance(item, Ref) and item.id not in out:
            c = _find_colour(sf, si.args[1])
            if c is not None:
                out[item.id] = c
    return out


# ==========================================================================
# Public API
# ==========================================================================

def load_step(path: Union[str, os.PathLike], chord_tol: float = 0.005, max_edge: Optional[float] = 0.10,
              include: Optional[Iterable[str]] = None) -> List[Solid]:
    """Tessellate every solid of a STEP file into world-coordinate triangles.

    ``chord_tol`` bounds the distance between the mesh and the exact surface on
    curved faces and edges (metres).  ``max_edge`` bounds triangle edge length on
    curved faces (planar faces are triangulated exactly from their boundary).
    ``include`` optionally restricts meshing to solids whose name matches one of
    the given ``fnmatch`` patterns.  Solids are returned in assembly order.
    """
    sf = StepFile.parse(path)
    patterns = list(include) if include is not None else None
    colours = _solid_colours(sf)
    meshed: Dict[int, Tuple[np.ndarray, Tuple[str, ...]]] = {}
    tessellators: Dict[Tuple[float, float], _Tessellator] = {}
    solids: List[Solid] = []
    for inst in _assembly_instances(sf):
        if patterns is not None and not any(fnmatch(inst.name, p) for p in patterns):
            continue
        if inst.msb.id not in meshed:
            units = _rep_scale(sf, inst.rep)
            tess = tessellators.get(units)
            if tess is None:
                tess = tessellators[units] = _Tessellator(sf, units[0], units[1], chord_tol, max_edge)
            tris = tess.solid(inst.msb)
            notes = list(tess.notes)
            vol = _signed_volume(tris)
            if vol < 0:
                tris = tris[:, ::-1]
                notes.append(f"signed volume was negative ({vol:.6g} m^3); orientation flipped")
            meshed[inst.msb.id] = (tris, tuple(notes))
        tris, notes = meshed[inst.msb.id]
        m = inst.matrix
        world = tris @ m[:3, :3].T + m[:3, 3]
        if np.linalg.det(m[:3, :3]) < 0:
            world = world[:, ::-1]
        solids.append(Solid.from_triangles(inst.name, world, colours.get(inst.msb.id), notes))
    return solids


def _signed_volume(tris: np.ndarray) -> float:
    if not len(tris):
        return 0.0
    return float(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0)


def _file_sha256(path: Union[str, os.PathLike]) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_step_cached(path: Union[str, os.PathLike], cache_dir: Union[str, os.PathLike],
                     chord_tol: float = 0.005, max_edge: Optional[float] = 0.10) -> List[Solid]:
    """:func:`load_step` with an ``.npz`` cache keyed by the file's SHA-256, the
    tessellation parameters and :data:`MESH_VERSION`."""
    digest = _file_sha256(path)
    key_src = f"{digest}|chord_tol={chord_tol!r}|max_edge={max_edge!r}|v{MESH_VERSION}"
    key = hashlib.sha256(key_src.encode()).hexdigest()[:24]
    cache_dir = Path(cache_dir)
    target = cache_dir / f"{Path(path).stem}-{key}.npz"
    if target.exists():
        try:
            return _read_cache(target, key_src)
        except (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile):
            pass  # corrupt or foreign file: rebuild
    solids = load_step(path, chord_tol, max_edge)
    cache_dir.mkdir(parents=True, exist_ok=True)
    _write_cache(target, key_src, solids)
    return solids


def _write_cache(target: Path, key_src: str, solids: List[Solid]) -> None:
    counts = np.array([len(s.triangles) for s in solids], dtype=np.int64)
    tris = np.concatenate([s.triangles for s in solids]) if solids else np.zeros((0, 3, 3))
    colours = np.array([s.color if s.color is not None else (np.nan,) * 3 for s in solids], float).reshape(-1, 3)
    names = np.array([s.name for s in solids], dtype=str)
    notes = np.array(["\n".join(s.notes) for s in solids], dtype=str)
    fd, tmp = tempfile.mkstemp(dir=target.parent, suffix=".npz.tmp")
    try:
        with os.fdopen(fd, "wb") as fh:
            np.savez(fh, key=np.array(key_src), names=names, counts=counts, triangles=tris,
                     colours=colours, notes=notes)
        os.replace(tmp, target)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def _read_cache(target: Path, key_src: str) -> List[Solid]:
    with np.load(target, allow_pickle=False) as z:
        if str(z["key"]) != key_src:
            raise ValueError("cache key mismatch")
        names, counts, tris = z["names"], z["counts"], z["triangles"]
        colours, notes = z["colours"], z["notes"]
    out = []
    start = 0
    for name, n, col, note in zip(names, counts, colours, notes):
        t = tris[start:start + int(n)]
        start += int(n)
        colour = None if np.isnan(col).any() else (float(col[0]), float(col[1]), float(col[2]))
        out.append(Solid.from_triangles(str(name), t, colour, tuple(x for x in str(note).split("\n") if x)))
    return out
