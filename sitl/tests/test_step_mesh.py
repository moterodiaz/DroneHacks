"""Tests for dronesim.world: STEP parser, B-rep tessellation and mesh queries.

Run from the ``sitl`` directory::

    py -B -m unittest tests.test_step_mesh -v

The course tests read ``dronehacks-course-v3.step`` from the repository root
(override with the ``DRONESIM_COURSE_STEP`` environment variable) and are
skipped if it is missing.  They expect the 2026-11-07 layout (SolidWorks 2023
export): 31 solids, the maze dividers M1-M3 and the net roof gone, two new
walls in one multi-body part.  Reference numbers for the unchanged parts come
from two independent extractions of the earlier (Onshape) export, whose parts
match this one to 0.1 mm (the 2.5-D ``course-map/1`` JSON and the task brief);
those of the new walls from the SolidWorks model.  Tolerance is 2 mm unless
stated.
"""

from __future__ import annotations

import math
import os
import tempfile
import time
import unittest
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from dronesim.world.mesh_query import MeshIndex
from dronesim.world.step_mesh import Solid, StepMeshError, load_step, load_step_cached, product_label
from dronesim.world.step_parser import DERIVED, Enum, Ref, StepFile, StepSyntaxError, Typed

STEP_PATH = Path(os.environ.get("DRONESIM_COURSE_STEP",
                                Path(__file__).resolve().parents[2] / "dronehacks-course-v3.step"))
TOL = 0.002

WALLS: Dict[str, Tuple[Tuple[float, float], Tuple[float, float]]] = {
    "A1 approach wall": ((-0.97536, -0.6223), (-0.97536, 0.4064)),
    "Barrier 1": ((-3.048, 0.4572), (-1.4351, 0.4572)),
    "Barrier 2": ((-1.4351, 0.4572), (0.1778, 0.4572)),
    "Barrier 3 (cut to 55 in)": ((0.1778, 0.4572), (1.5748, 0.4572)),
    "C1 chicane wall": ((0.97338, -1.12852), (1.52502, 0.38711)),
    "C2a chicane wall (window)": ((-0.74676, -3.02971), (-0.19512, -1.51408)),
    "C2b chicane wall": ((-0.19503, -1.51387), (0.15681, -0.54721)),
}
#: the two walls added on 2026-11-07, one SolidWorks part with two bodies: footprint corners
#: (counter-clockwise), from 0 to the wall top
NEW_WALLS: Dict[str, List[Tuple[float, float]]] = {
    "Part67 / Boss-Extrude1[1]": [(-3.04927, 2.698), (-1.24927, 2.698), (-1.24927, 3.048), (-3.04927, 3.048)],
    "Part67 / Boss-Extrude1[2]": [(-2.16664, 0.508), (-2.01985, 0.508), (-0.65859, 1.77479), (-0.72672, 1.848)],
}
COURSE_NAMES = {
    "A1 approach wall", "Approach ring (stage 4 line)", "Barrier 1", "Barrier 2", "Barrier 3 (cut to 55 in)",
    "Boulder 1", "Boulder 2", "Boulder 3", "Boulder 4", "Boulder 5", "C1 chicane wall",
    "C2a chicane wall (window)", "C2b chicane wall", "Floor 20 x 20 ft", "Guideline (zone 1)",
    "Landing pad 5 ft", "Landing pad cross", "Landing pad ring r20", "Landing pad ring r30",
    "Landing pad ring r9", "Launch mark", "Net side east", "Net side north", "Net side south", "Net side west",
    "Tube cradle 1", "Tube cradle 2", "Tube cradle 3", "Tube oval 58 x 72 x 60", *NEW_WALLS,
}
#: one boulder's volume in the earlier (Onshape) export at the default tolerances, m^3
OLD_BOULDER_VOLUME = 0.05718
BOULDER_CENTRES = [(2.01168, -2.46888), (2.52984, -1.55448), (0.79248, -2.34696),
                   (-2.56032, -2.49936), (-1.34112, -2.37744)]
TUBE = "Tube oval 58 x 72 x 60"
TUBE_X, TUBE_Z = 2.3114, 0.9398  # axis

_COURSE = None


def course() -> Tuple[List[Solid], Dict[str, Solid], float]:
    """Tessellate the course once for the whole module: (solids, by name, seconds)."""
    global _COURSE
    if _COURSE is None:
        if not STEP_PATH.exists():
            raise unittest.SkipTest(f"{STEP_PATH} not found")
        t0 = time.perf_counter()
        solids = load_step(STEP_PATH)
        _COURSE = (solids, {s.name: s for s in solids}, time.perf_counter() - t0)
    return _COURSE


def wall_frame(s: Solid) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
    """(direction d, normal n, thickness, p0, p1) of a box wall: n from its largest vertical faces."""
    areas = 0.5 * np.linalg.norm(np.cross(s.triangles[:, 1] - s.triangles[:, 0],
                                          s.triangles[:, 2] - s.triangles[:, 0]), axis=1)
    horiz = np.abs(s.normals[:, 2]) < 1e-6
    k = np.flatnonzero(horiz)[np.argmax(areas[horiz])]
    n = s.normals[k, :2] / np.linalg.norm(s.normals[k, :2])
    d = np.array([-n[1], n[0]])
    xy = s.triangles.reshape(-1, 3)[:, :2]
    off = xy @ n
    along = xy @ d
    mid = 0.5 * (off.max() + off.min())
    p0 = d * along.min() + n * mid
    p1 = d * along.max() + n * mid
    return d, n, float(off.max() - off.min()), p0, p1


# --------------------------------------------------------------------------
# Synthetic STEP writer (tests only)
# --------------------------------------------------------------------------

def _real(x: float) -> str:
    s = repr(float(x))
    if "e" in s and "." not in s:
        s = s.replace("e", ".e")
    elif "." not in s and "e" not in s and "inf" not in s:
        s += "."
    return s.upper()


class StepWriter:
    """Builds a minimal AP214-style Part 21 file entity by entity."""

    def __init__(self, length_unit: str = "$") -> None:
        self.records: List[str] = []
        self.n = 0
        prefix = length_unit
        self.len_unit = self.add(f"(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT({prefix},.METRE.))")
        self.ang_unit = self.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
        self.sr_unit = self.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
        unc = self.add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),{self.len_unit},"
                       "'distance_accuracy_value','')")
        self.ctx = self.add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({unc}))"
                            f"GLOBAL_UNIT_ASSIGNED_CONTEXT(({self.len_unit},{self.ang_unit},{self.sr_unit}))"
                            "REPRESENTATION_CONTEXT('ctx','3D'))")

    def add(self, body: str) -> str:
        self.n += 1
        self.records.append(f"#{self.n}={body};")
        return f"#{self.n}"

    def point(self, p: Sequence[float]) -> str:
        return self.add(f"CARTESIAN_POINT('',({','.join(_real(c) for c in p)}))")

    def direction(self, d: Sequence[float]) -> str:
        return self.add(f"DIRECTION('',({','.join(_real(c) for c in d)}))")

    def axis(self, o, z=(0.0, 0.0, 1.0), x=(1.0, 0.0, 0.0)) -> str:
        return self.add(f"AXIS2_PLACEMENT_3D('',{self.point(o)},{self.direction(z)},{self.direction(x)})")

    def vertex(self, p) -> str:
        return self.add(f"VERTEX_POINT('',{self.point(p)})")

    def line_edge(self, va: str, vb: str, pa, pb) -> str:
        d = np.asarray(pb, float) - np.asarray(pa, float)
        L = float(np.linalg.norm(d))
        vec = self.add(f"VECTOR('',{self.direction(d / L)},{_real(L)})")
        line = self.add(f"LINE('',{self.point(pa)},{vec})")
        return self.add(f"EDGE_CURVE('',{va},{vb},{line},.T.)")

    def circle_edge(self, v: str, centre, r: float, z=(0.0, 0.0, 1.0), x=(1.0, 0.0, 0.0)) -> str:
        c = self.add(f"CIRCLE('',{self.axis(centre, z, x)},{_real(r)})")
        return self.add(f"EDGE_CURVE('',{v},{v},{c},.T.)")

    def loop(self, edges: Sequence[Tuple[str, bool]]) -> str:
        oes = [self.add(f"ORIENTED_EDGE('',*,*,{e},{'.T.' if fwd else '.F.'})") for e, fwd in edges]
        return self.add(f"EDGE_LOOP('',({','.join(oes)}))")

    def face(self, bounds: Sequence[Tuple[str, bool, str]], surface: str, same_sense: bool) -> str:
        refs = [self.add(f"{kind}('',{lp},{'.T.' if ori else '.F.'})") for lp, ori, kind in bounds]
        return self.add(f"ADVANCED_FACE('',({','.join(refs)}),{surface},{'.T.' if same_sense else '.F.'})")

    def solid(self, name: str, faces: Sequence[str]) -> str:
        shell = self.add(f"CLOSED_SHELL('',({','.join(faces)}))")
        return self.add(f"MANIFOLD_SOLID_BREP('{name}',{shell})")

    def text(self) -> str:
        return ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('synthetic test model'),'2;1');\n"
                "FILE_NAME('t.step','2026-11-07T00:00:00',(''),(''),'','','');\n"
                "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n"
                + "\n".join(self.records) + "\nENDSEC;\nEND-ISO-10303-21;\n")


def box_brep(w: StepWriter, lo, hi, name: str) -> str:
    """Axis-aligned box.  One face uses an inward plane with same_sense .F. and one a
    reversed loop under FACE_BOUND(.F.), to exercise both orientation flags."""
    corners = [np.array([(lo[0], hi[0])[i & 1], (lo[1], hi[1])[(i >> 1) & 1], (lo[2], hi[2])[(i >> 2) & 1]], float)
               for i in range(8)]
    V = [w.vertex(c) for c in corners]
    edges: Dict[Tuple[int, int], str] = {}

    def edge(i: int, j: int) -> Tuple[str, bool]:
        key = (min(i, j), max(i, j))
        if key not in edges:
            edges[key] = w.line_edge(V[key[0]], V[key[1]], corners[key[0]], corners[key[1]])
        return edges[key], i < j

    quads = [((0, 0, -1), [0, 1, 3, 2]), ((0, 0, 1), [4, 5, 7, 6]), ((0, -1, 0), [0, 1, 5, 4]),
             ((0, 1, 0), [2, 3, 7, 6]), ((-1, 0, 0), [0, 2, 6, 4]), ((1, 0, 0), [1, 3, 7, 5])]
    faces = []
    for k, (normal, quad) in enumerate(quads):
        nrm = np.array(normal, float)
        p = [corners[i] for i in quad]
        if np.cross(p[1] - p[0], p[2] - p[0]) @ nrm < 0:
            quad = quad[::-1]
        ring = [edge(quad[m], quad[(m + 1) % 4]) for m in range(4)]
        inplane = corners[quad[1]] - corners[quad[0]]
        inplane = inplane / np.linalg.norm(inplane)
        if k == 1:   # top: plane axis points inward, face reverses it
            surf = w.add(f"PLANE('',{w.axis(corners[quad[0]], -nrm, inplane)})")
            faces.append(w.face([(w.loop(ring), True, "FACE_OUTER_BOUND")], surf, False))
        elif k == 2:  # -y side: loop written backwards, bound orientation .F.
            back = [(e, not fwd) for e, fwd in reversed(ring)]
            surf = w.add(f"PLANE('',{w.axis(corners[quad[0]], nrm, inplane)})")
            faces.append(w.face([(w.loop(back), False, "FACE_BOUND")], surf, True))
        else:
            surf = w.add(f"PLANE('',{w.axis(corners[quad[0]], nrm, inplane)})")
            faces.append(w.face([(w.loop(ring), True, "FACE_OUTER_BOUND")], surf, True))
    return w.solid(name, faces)


def assembly_step(swapped: bool) -> str:
    """Root assembly (mm) with one box part 1000 x 2000 x 3000 mm placed by a rotation of
    +90 deg about z and a translation of (1000, 2000, 3000) mm.  ``swapped`` writes the
    relationship with rep_1 = parent (the reversed convention some exporters use)."""
    w = StepWriter(length_unit=".MILLI.")
    app = w.add("APPLICATION_CONTEXT('automotive design')")
    pctx = w.add(f"PRODUCT_CONTEXT('',{app},'mechanical')")
    pdctx = w.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',{app},'design')")

    def product(name: str) -> Tuple[str, str]:
        p = w.add(f"PRODUCT('{name}','{name}','',({pctx}))")
        f = w.add(f"PRODUCT_DEFINITION_FORMATION('','',{p})")
        pd = w.add(f"PRODUCT_DEFINITION('design','',{f},{pdctx})")
        return pd, w.add(f"PRODUCT_DEFINITION_SHAPE('','',{pd})")

    root_pd, root_pds = product("Assembly")
    box_pd, box_pds = product("Box")
    root_origin = w.axis((0, 0, 0))
    place = w.axis((1000, 2000, 3000), (0, 0, 1), (0, 1, 0))
    root_sr = w.add(f"SHAPE_REPRESENTATION('Assembly',({root_origin},{place}),{w.ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({root_pds},{root_sr})")
    box_origin = w.axis((0, 0, 0))
    box_sr = w.add(f"SHAPE_REPRESENTATION('Box',({box_origin}),{w.ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({box_pds},{box_sr})")
    msb = box_brep(w, (0, 0, 0), (1000, 2000, 3000), "box")
    absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({msb}),{w.ctx})")
    w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{box_sr},{absr})")
    colour = w.add("COLOUR_RGB('',0.2,0.4,0.6)")
    fasc = w.add(f"FILL_AREA_STYLE_COLOUR('',{colour})")
    fas = w.add(f"FILL_AREA_STYLE('',({fasc}))")
    ssfa = w.add(f"SURFACE_STYLE_FILL_AREA({fas})")
    sss = w.add(f"SURFACE_SIDE_STYLE('',({ssfa}))")
    ssu = w.add(f"SURFACE_STYLE_USAGE(.BOTH.,{sss})")
    psa = w.add(f"PRESENTATION_STYLE_ASSIGNMENT(({ssu}))")
    w.add(f"STYLED_ITEM('',({psa}),{msb})")
    nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','Box instance','',{root_pd},{box_pd},$)")
    nauo_pds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
    if swapped:
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{place},{box_origin})")
        reps = f"{root_sr},{box_sr}"
    else:
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{box_origin},{place})")
        reps = f"{box_sr},{root_sr}"
    rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{reps})"
               f"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})SHAPE_REPRESENTATION_RELATIONSHIP())")
    w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{nauo_pds})")
    return w.text()


def parts_step(parts: Sequence[Tuple[str, Sequence[str]]], root: str = "Assy",
               nauo_name: Optional[str] = " ") -> str:
    """A root assembly of parts named the way a CAD exporter writes them.  Each part is a
    PRODUCT ``name`` whose ADVANCED_BREP_SHAPE_REPRESENTATION holds one unit box
    MANIFOLD_SOLID_BREP per body name (side by side along x), placed by an identity
    transformation.  Occurrence (NAUO) names are ``nauo_name``: SolidWorks writes them
    blank, Onshape repeats the product name (``None`` here)."""
    w = StepWriter()
    app = w.add("APPLICATION_CONTEXT('automotive design')")
    pctx = w.add(f"PRODUCT_CONTEXT('',{app},'mechanical')")
    pdctx = w.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',{app},'design')")

    def product(name: str) -> Tuple[str, str]:
        p = w.add(f"PRODUCT('{name}','{name}','',({pctx}))")
        f = w.add(f"PRODUCT_DEFINITION_FORMATION('','',{p})")
        pd = w.add(f"PRODUCT_DEFINITION('design','',{f},{pdctx})")
        return pd, w.add(f"PRODUCT_DEFINITION_SHAPE('','',{pd})")

    root_pd, root_pds = product(root)
    root_origin = w.axis((0, 0, 0))
    root_sr = w.add(f"SHAPE_REPRESENTATION('{root}',({root_origin}),{w.ctx})")
    w.add(f"SHAPE_DEFINITION_REPRESENTATION({root_pds},{root_sr})")
    x = 0.0
    for k, (name, bodies) in enumerate(parts):
        pd, pds = product(name)
        origin = w.axis((0, 0, 0))
        sr = w.add(f"SHAPE_REPRESENTATION('{name}',({origin}),{w.ctx})")
        w.add(f"SHAPE_DEFINITION_REPRESENTATION({pds},{sr})")
        msbs = []
        for body in bodies:
            msbs.append(box_brep(w, (x, 0.0, 0.0), (x + 1.0, 1.0, 1.0), body))
            x += 2.0
        absr = w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({','.join(msbs)}),{w.ctx})")
        w.add(f"SHAPE_REPRESENTATION_RELATIONSHIP('','',{sr},{absr})")
        occ = name if nauo_name is None else nauo_name
        nauo = w.add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO{k + 1}','{occ}',' ',{root_pd},{pd},$)")
        nauo_pds = w.add(f"PRODUCT_DEFINITION_SHAPE('','',{nauo})")
        idt = w.add(f"ITEM_DEFINED_TRANSFORMATION('','',{origin},{root_origin})")
        rr = w.add(f"(REPRESENTATION_RELATIONSHIP('','',{sr},{root_sr})"
                   f"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt})SHAPE_REPRESENTATION_RELATIONSHIP())")
        w.add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{nauo_pds})")
    return w.text()


def pole_loops_step(r: float = 0.5) -> str:
    """Bare geometry, metres: spheres whose faces have edge loops *through* a pole, the way
    SolidWorks writes the course boulders (no VERTEX_LOOP).

    * "split hemisphere" (centre at the origin): two dome faces, each bounded by half the
      equator and the two meridians that meet at the pole, and a planar base;
    * "two-lune sphere" (centre (3, 0, 0)): two faces, each bounded by two meridians running
      from pole to pole (each loop visits both poles)."""
    w = StepWriter()
    items = []

    def arc(va: str, vb: str, centre, z, x) -> str:
        c = w.add(f"CIRCLE('',{w.axis(centre, z, x)},{_real(r)})")
        return w.add(f"EDGE_CURVE('',{va},{vb},{c},.T.)")

    c = np.zeros(3)
    e0, e1, top = w.vertex(c + (r, 0, 0)), w.vertex(c + (-r, 0, 0)), w.vertex(c + (0, 0, r))
    eq_n = arc(e0, e1, c, (0, 0, 1), (1, 0, 0))       # equator through +y
    eq_s = arc(e1, e0, c, (0, 0, 1), (1, 0, 0))       # equator through -y
    m0 = arc(e0, top, c, (0, -1, 0), (1, 0, 0))       # meridian in the xz-plane, +x side
    m1 = arc(top, e1, c, (0, -1, 0), (1, 0, 0))       # meridian in the xz-plane, -x side
    sph = w.add(f"SPHERICAL_SURFACE('',{w.axis(c)},{_real(r)})")
    north = w.face([(w.loop([(eq_n, True), (m1, False), (m0, False)]), True, "FACE_OUTER_BOUND")], sph, True)
    south = w.face([(w.loop([(eq_s, True), (m0, True), (m1, True)]), True, "FACE_OUTER_BOUND")], sph, True)
    base = w.face([(w.loop([(eq_s, False), (eq_n, False)]), True, "FACE_OUTER_BOUND")],
                  w.add(f"PLANE('',{w.axis(c)})"), False)
    items.append(w.solid("split hemisphere", [north, south, base]))

    c = np.array([3.0, 0.0, 0.0])
    vn, vs = w.vertex(c + (0, 0, r)), w.vertex(c + (0, 0, -r))
    ma = arc(vs, vn, c, (0, -1, 0), (1, 0, 0))        # south to north through +x
    mb = arc(vn, vs, c, (0, -1, 0), (1, 0, 0))        # north to south through -x
    sph = w.add(f"SPHERICAL_SURFACE('',{w.axis(c)},{_real(r)})")
    f1 = w.face([(w.loop([(ma, True), (mb, True)]), True, "FACE_BOUND")], sph, True)
    f2 = w.face([(w.loop([(mb, False), (ma, False)]), True, "FACE_BOUND")], sph, True)
    items.append(w.solid("two-lune sphere", [f1, f2]))
    w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({','.join(items)}),{w.ctx})")
    return w.text()


def curved_solids_step(r: float = 0.5, h: float = 0.8, R: float = 1.0, rt: float = 0.25) -> str:
    """Bare geometry (no product structure), metres: a cone with an apex VERTEX_LOOP, a
    hemisphere with a pole VERTEX_LOOP, a cylinder whose side face has a seam edge, and a
    torus bounded by its two seam circles."""
    w = StepWriter()
    items = []

    # Cone: base circle r at z = 0, apex at z = h.  Placement axis -z so tan(semi_angle) = r / h > 0.
    v0, va = w.vertex((r, 0, 0)), w.vertex((0, 0, h))
    circ = w.circle_edge(v0, (0, 0, 0), r)
    base = w.face([(w.loop([(circ, False)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis((0, 0, 0))})"), False)
    cone = w.add(f"CONICAL_SURFACE('',{w.axis((0, 0, 0), (0, 0, -1), (1, 0, 0))},{_real(r)},{_real(math.atan(r / h))})")
    apex = w.add(f"VERTEX_LOOP('',{va})")
    side = w.face([(w.loop([(circ, True)]), True, "FACE_BOUND"), (apex, True, "FACE_BOUND")], cone, True)
    items.append(w.solid("cone", [base, side]))

    # Hemisphere of radius r centred at (3, 0, 0).
    c = (3.0, 0.0, 0.0)
    v0, vp = w.vertex((3.0 + r, 0, 0)), w.vertex((3.0, 0, r))
    circ = w.circle_edge(v0, c, r)
    base = w.face([(w.loop([(circ, False)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c)})"), False)
    sph = w.add(f"SPHERICAL_SURFACE('',{w.axis(c)},{_real(r)})")
    pole = w.add(f"VERTEX_LOOP('',{vp})")
    dome = w.face([(w.loop([(circ, True)]), True, "FACE_BOUND"), (pole, True, "FACE_BOUND")], sph, True)
    items.append(w.solid("hemisphere", [base, dome]))

    # Cylinder r x h centred at (6, 0, 0) with a seam edge on its side face.
    c0, c1 = (6.0, 0.0, 0.0), (6.0, 0.0, h)
    vb, vt = w.vertex((6.0 + r, 0, 0)), w.vertex((6.0 + r, 0, h))
    bot, top = w.circle_edge(vb, c0, r), w.circle_edge(vt, c1, r)
    seam = w.line_edge(vb, vt, (6.0 + r, 0, 0), (6.0 + r, 0, h))
    f_bot = w.face([(w.loop([(bot, False)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c0)})"), False)
    f_top = w.face([(w.loop([(top, True)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c1)})"), True)
    cyl = w.add(f"CYLINDRICAL_SURFACE('',{w.axis(c0)},{_real(r)})")
    f_side = w.face([(w.loop([(bot, True), (seam, True), (top, False), (seam, False)]), True, "FACE_BOUND")],
                    cyl, True)
    items.append(w.solid("seam cylinder", [f_bot, f_top, f_side]))

    # Torus (major R, minor rt) centred at (0, 4, 0): one face bounded by its two seam circles.
    c = (0.0, 4.0, 0.0)
    v = w.vertex((R + rt, 4.0, 0))
    e_u = w.circle_edge(v, c, R + rt)
    e_v = w.circle_edge(v, (R, 4.0, 0), rt, (0, -1, 0), (1, 0, 0))
    tor = w.add(f"TOROIDAL_SURFACE('',{w.axis(c)},{_real(R)},{_real(rt)})")
    f = w.face([(w.loop([(e_u, True), (e_v, True), (e_u, False), (e_v, False)]), True, "FACE_BOUND")], tor, True)
    items.append(w.solid("torus", [f]))

    # Elliptic prism (a = 0.6, b = 0.3, height h) at (0, -4, 0): SURFACE_OF_LINEAR_EXTRUSION side
    # bounded by two full ellipses (the band case), planar elliptic caps.
    c0, c1 = (0.0, -4.0, 0.0), (0.0, -4.0, h)
    vb, vt = w.vertex((0.6, -4.0, 0.0)), w.vertex((0.6, -4.0, h))
    ell_b = w.add(f"ELLIPSE('',{w.axis(c0)},0.6,0.3)")
    ell_t = w.add(f"ELLIPSE('',{w.axis(c1)},0.6,0.3)")
    e_b = w.add(f"EDGE_CURVE('',{vb},{vb},{ell_b},.T.)")
    e_t = w.add(f"EDGE_CURVE('',{vt},{vt},{ell_t},.T.)")
    vec = w.add(f"VECTOR('',{w.direction((0, 0, 1))},{_real(h)})")
    ext = w.add(f"SURFACE_OF_LINEAR_EXTRUSION('',{ell_b},{vec})")
    f_side = w.face([(w.loop([(e_b, True)]), True, "FACE_BOUND"), (w.loop([(e_t, False)]), True, "FACE_BOUND")],
                    ext, True)
    f_bot = w.face([(w.loop([(e_b, False)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c0)})"), False)
    f_top = w.face([(w.loop([(e_t, True)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c1)})"), True)
    items.append(w.solid("extruded ellipse", [f_side, f_bot, f_top]))

    # Hemisphere as a SURFACE_OF_REVOLUTION of a full circle (profile in the xz-plane) about z,
    # centred at (3, 4, 0); its pole is found where the profile touches the axis.
    c = (3.0, 4.0, 0.0)
    v0, vp = w.vertex((3.0 + r, 4.0, 0)), w.vertex((3.0, 4.0, r))
    circ = w.circle_edge(v0, c, r)
    prof = w.add(f"CIRCLE('',{w.axis(c, (0, -1, 0), (1, 0, 0))},{_real(r)})")
    ax1 = w.add(f"AXIS1_PLACEMENT('',{w.point(c)},{w.direction((0, 0, 1))})")
    rev = w.add(f"SURFACE_OF_REVOLUTION('',{prof},{ax1})")
    pole = w.add(f"VERTEX_LOOP('',{vp})")
    dome = w.face([(w.loop([(circ, True)]), True, "FACE_BOUND"), (pole, True, "FACE_BOUND")], rev, True)
    base = w.face([(w.loop([(circ, False)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c)})"), False)
    items.append(w.solid("revolved hemisphere", [dome, base]))

    # Hemisphere bounded only by its equator (no VERTEX_LOOP): the pole is chosen from the
    # loop direction and face sense.  Centred at (6, 4, 0), dome facing -z this time.
    c = (6.0, 4.0, 0.0)
    v0 = w.vertex((6.0 + r, 4.0, 0))
    circ = w.circle_edge(v0, c, r)
    sph = w.add(f"SPHERICAL_SURFACE('',{w.axis(c)},{_real(r)})")
    dome = w.face([(w.loop([(circ, False)]), True, "FACE_BOUND")], sph, True)
    base = w.face([(w.loop([(circ, True)]), True, "FACE_BOUND")], w.add(f"PLANE('',{w.axis(c)})"), True)
    items.append(w.solid("lower hemisphere", [dome, base]))

    w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({','.join(items)}),{w.ctx})")
    return w.text()


def cube(name: str = "cube", lo=(0.0, 0.0, 0.0), hi=(1.0, 1.0, 1.0)) -> Solid:
    """A hand-built box as 12 outward triangles."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = np.array([[lo[0] if not i & 1 else hi[0], lo[1] if not i & 2 else hi[1], lo[2] if not i & 4 else hi[2]]
                  for i in range(8)])
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    tris = []
    for a, b, cc, d in quads:
        tris += [(c[a], c[b], c[cc]), (c[a], c[cc], c[d])]
    return Solid.from_triangles(name, np.array(tris))


def write_temp(text: str) -> str:
    fd, path = tempfile.mkstemp(suffix=".step")
    with os.fdopen(fd, "w") as fh:
        fh.write(text)
    return path


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------

class ParserTests(unittest.TestCase):
    TEXT = """ISO-10303-21;
HEADER;
/* a comment; with a semicolon */
FILE_DESCRIPTION(('demo'),'2;1');
FILE_NAME('it''s.step','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('it''s; a name',(0.,-1.5,1.E-05));
#2 = VALUE_REPRESENTATION_ITEM('n',COUNT_MEASURE(0.));
#3=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#4=B_SPLINE_SURFACE_WITH_KNOTS('',3,1,((#1,#1),(#1,#1)),.SURF_OF_LINEAR_EXTRUSION.,.F.,.T.,$,(4,4),(2,2),
 (0.,1.),(0.,1.),.UNSPECIFIED.);
/* between records */ #5=PRODUCT('caf\\X2\\00E9\\X0\\','\\\\x','',());
ENDSEC;
END-ISO-10303-21;
"""

    def test_synthetic_records(self):
        sf = StepFile.from_text(self.TEXT)
        self.assertEqual(sf.header["FILE_NAME"][0], "it's.step")
        p = sf.get(1)
        self.assertEqual(p.type, "CARTESIAN_POINT")
        self.assertEqual(p.args[0], "it's; a name")
        self.assertEqual(p.args[1], [0.0, -1.5, 1e-5])
        v = sf.get(Ref(2))
        self.assertIsInstance(v.args[1], Typed)
        self.assertEqual((v.args[1].type, v.args[1].value), ("COUNT_MEASURE", 0.0))
        u = sf.get(3)
        self.assertTrue(u.is_complex)
        self.assertEqual(set(u.parts), {"LENGTH_UNIT", "NAMED_UNIT", "SI_UNIT"})
        self.assertIs(u.part("NAMED_UNIT")[0], DERIVED)
        self.assertEqual(u.part("SI_UNIT"), ["MILLI", "METRE"])
        self.assertIsInstance(u.part("SI_UNIT")[0], Enum)
        b = sf.get(4)
        self.assertEqual(b.args[1:3], [3, 1])
        self.assertIsInstance(b.args[1], int)
        self.assertEqual(b.args[3][0][1], Ref(1))
        self.assertEqual(b.args[4], "SURF_OF_LINEAR_EXTRUSION")
        self.assertEqual(b.args[5:8], [False, True, None])
        self.assertEqual(sf.get(5).args[:2], ["café", "\\x"])
        self.assertEqual([e.id for e in sf.by_type("SI_UNIT")], [3])
        self.assertEqual(sf.type_of(3), "(LENGTH_UNIT NAMED_UNIT SI_UNIT)")
        with self.assertRaises(KeyError):
            sf.get(99)

    def test_syntax_errors(self):
        with self.assertRaises(StepSyntaxError):
            StepFile.from_text("not a step file;")
        with self.assertRaises(StepSyntaxError):
            StepFile.from_text("ISO-10303-21;\nDATA;\nFOO(1);\nENDSEC;\n")  # record without '#n='
        lazy = StepFile.from_text("ISO-10303-21;\nDATA;\n#1=FOO(1,;\nENDSEC;\n")  # parameters parse on demand
        self.assertEqual(lazy.type_of(1), "FOO")
        with self.assertRaises(StepSyntaxError):
            lazy.get(1)

    def test_course_entity_counts(self):
        if not STEP_PATH.exists():
            self.skipTest(f"{STEP_PATH} not found")
        sf = StepFile.parse(STEP_PATH)
        # SolidWorks writes the tube, its cradles and the boulders as B-splines (no ELLIPSE, no
        # VERTEX_LOOP); 30 parts, one of them (Part67) with two bodies
        expected = {"PLANE": 152, "CYLINDRICAL_SURFACE": 20, "B_SPLINE_SURFACE_WITH_KNOTS": 17, "LINE": 308,
                    "CIRCLE": 40, "ELLIPSE": 0, "VERTEX_LOOP": 0, "B_SPLINE_CURVE_WITH_KNOTS": 38,
                    "STYLED_ITEM": 31, "MANIFOLD_SOLID_BREP": 31, "PRODUCT": 31,
                    "NEXT_ASSEMBLY_USAGE_OCCURRENCE": 30, "ITEM_DEFINED_TRANSFORMATION": 30,
                    "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION": 30}
        for name, n in expected.items():
            self.assertEqual(sf.count(name), n, name)
        self.assertEqual({str(e.args[4]) for e in sf.by_type("B_SPLINE_SURFACE_WITH_KNOTS")}, {"UNSPECIFIED"})
        self.assertEqual(sorted({(e.args[1], e.args[2]) for e in sf.by_type("B_SPLINE_SURFACE_WITH_KNOTS")}),
                         [(3, 1), (3, 3)])
        rr = sf.get(sf.by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")[0].args[0])
        self.assertEqual(set(rr.parts), {"REPRESENTATION_RELATIONSHIP",
                                         "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
                                         "SHAPE_REPRESENTATION_RELATIONSHIP"})


# --------------------------------------------------------------------------
# Course tessellation
# --------------------------------------------------------------------------

class CourseSolidTests(unittest.TestCase):
    def test_count_watertight_positive(self):
        solids, by_name, _ = course()
        self.assertEqual(len(solids), 31)  # 29 single-body parts + Part67's two bodies; the root has none
        self.assertEqual(len(by_name), 31)
        for s in solids:
            rep = s.watertight_report()
            self.assertTrue(rep.watertight, f"{s.name}: {rep}")
            self.assertGreater(s.volume(), 0.0, s.name)
            self.assertEqual(s.notes, (), s.name)
            self.assertIsNotNone(s.color, s.name)
            np.testing.assert_allclose(np.linalg.norm(s.normals, axis=1), 1.0, atol=1e-9)

    def test_names_come_from_the_products(self):
        # SolidWorks names products after their files ('Barrier 1.step') and the multi-body virtual
        # component 'Part67^dronehacks-course-v3.step'; its bodies are 'Boss-Extrude1[1]' and '[2]'
        _, by_name, _ = course()
        self.assertEqual(set(by_name), COURSE_NAMES)
        for gone in ("M1 maze divider", "M2 maze divider", "M3 maze divider", "Net roof"):
            self.assertNotIn(gone, by_name)

    def test_performance(self):
        # The 2026-11-07 SolidWorks export tessellates in ~26 s on the dev laptop, idle (the
        # Onshape file took under 15 s): ~26 % more triangles and B-spline domes and tube faces.
        # The bound still catches a pathological slowdown; the result is cached after one run.
        _, _, seconds = course()
        self.assertLess(seconds, 45.0)

    def test_curved_face_tolerances(self):
        _, by_name, _ = course()
        tube = by_name[TUBE]
        curved = np.abs(tube.normals[:, 1]) < 0.999
        t = tube.triangles[curved]
        edges = np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2)
        self.assertLessEqual(edges.max(), 0.10 + 1e-9)
        # Deviation of edge midpoints / centroids from the exact elliptic cylinders.
        probe = np.concatenate([t.mean(axis=1), (0.5 * (t + np.roll(t, 1, axis=1))).reshape(-1, 3)])
        x, z = probe[:, 0] - TUBE_X, probe[:, 2] - TUBE_Z
        dev = np.minimum(_ellipse_distance(0.7366, 0.9144, x, z), _ellipse_distance(0.73025, 0.90805, x, z))
        self.assertLessEqual(dev.max(), 0.005)
        for name in ("Boulder 1", "Approach ring (stage 4 line)"):
            s = by_name[name]
            curved = np.abs(s.normals[:, 2]) < 0.999
            t = s.triangles[curved]
            self.assertLessEqual(np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2).max(), 0.10 + 1e-9, name)


def _ellipse_distance(a: float, b: float, px: np.ndarray, pz: np.ndarray) -> np.ndarray:
    th = np.arctan2(pz / b, px / a)
    for _ in range(30):
        ex, ez, dx, dz = a * np.cos(th), b * np.sin(th), -a * np.sin(th), b * np.cos(th)
        th = th + ((px - ex) * dx + (pz - ez) * dz) / (dx * dx + dz * dz)
    return np.hypot(px - a * np.cos(th), pz - b * np.sin(th))


class CourseGeometryTests(unittest.TestCase):
    def solid(self, name: str) -> Solid:
        return course()[1][name]

    def footprint(self, name: str) -> Tuple[np.ndarray, float]:
        s = self.solid(name)
        centre = 0.5 * (s.bbox_min[:2] + s.bbox_max[:2])
        r = np.hypot(*(s.triangles.reshape(-1, 3)[:, :2] - centre).T).max()
        return centre, float(r)

    def test_launch_mark_and_landing_pad(self):
        c, _ = self.footprint("Launch mark")
        np.testing.assert_allclose(c, (-2.5146, 0.9144), atol=TOL)
        c, r = self.footprint("Landing pad 5 ft")
        np.testing.assert_allclose(c, (-2.04216, -0.79248), atol=TOL)
        self.assertAlmostEqual(r, 0.762, delta=TOL)
        self.assertAlmostEqual(2 * r, 1.524, delta=TOL)

    def test_boulders(self):
        # The B-rep boulders are domes, not cylinders: a planar disc of radius 0.2286 at
        # z = 0 capped by two B-spline halves (SolidWorks; Onshape wrote one revolved
        # B-spline with a VERTEX_LOOP) whose meridian edges meet at the pole, z = 0.5334,
        # approximating a half-ellipsoid.  The reference volume pi r^2 h (0.0876 m^3) is a
        # 2.5-D footprint x height model; the solid's volume converges to
        # (2/3) pi r^2 h = 0.05838 m^3 (checked here at default and fine tolerances).
        r, h = 0.2286, 0.5334
        dome = 2.0 / 3.0 * math.pi * r * r * h
        centres = []
        for k in range(1, 6):
            s = self.solid(f"Boulder {k}")
            c, rad = self.footprint(f"Boulder {k}")
            centres.append(c)
            self.assertAlmostEqual(rad, r, delta=TOL)
            self.assertAlmostEqual(s.bbox_min[2], 0.0, delta=TOL)
            self.assertAlmostEqual(s.bbox_max[2], h, delta=TOL)
            self.assertAlmostEqual(s.volume() / dome, 1.0, delta=0.03)  # inscribed mesh is ~2 % low
            self.assertAlmostEqual(s.volume() / OLD_BOULDER_VOLUME, 1.0, delta=0.01)
            v = s.triangles.reshape(-1, 3)
            top = v[np.abs(v[:, 2] - 0.4) < 0.02]
            radius_04 = np.hypot(top[:, 0] - c[0], top[:, 1] - c[1]).max()
            self.assertAlmostEqual(radius_04, r * math.sqrt(1 - (0.4 / h) ** 2), delta=0.01)
        for ref in BOULDER_CENTRES:
            self.assertLess(min(np.linalg.norm(c - ref) for c in centres), TOL, ref)
        fine = load_step(STEP_PATH, chord_tol=0.001, max_edge=0.03, include=["Boulder 1"])[0]
        self.assertTrue(fine.is_watertight)
        self.assertAlmostEqual(fine.volume() / dome, 1.0, delta=0.005)

    def test_walls(self):
        for name, (r0, r1) in WALLS.items():
            s = self.solid(name)
            _, _, thick, p0, p1 = wall_frame(s)
            self.assertAlmostEqual(thick, 0.1016, delta=TOL, msg=name)
            self.assertAlmostEqual(s.bbox_min[2], 0.0, delta=TOL, msg=name)
            self.assertAlmostEqual(s.bbox_max[2], 1.7272, delta=TOL, msg=name)
            ends = sorted([tuple(p0), tuple(p1)])
            np.testing.assert_allclose(ends, sorted([r0, r1]), atol=TOL, err_msg=name)

    def test_new_walls(self):
        # 2026-11-07: a 1.8 x 0.35 m block against the west and north nets, and a 0.10 m diagonal
        # wall at 42.9 deg from east whose south end is cut along Barrier 1's north face
        for name, corners in NEW_WALLS.items():
            s = self.solid(name)
            self.assertTrue(s.is_watertight, name)
            self.assertAlmostEqual(s.bbox_min[2], 0.0, delta=TOL, msg=name)
            self.assertAlmostEqual(s.bbox_max[2], 1.7272, delta=TOL, msg=name)
            v = s.triangles.reshape(-1, 3)
            xy = np.unique(np.round(v[:, :2], 5), axis=0)
            self.assertEqual(len(xy), 4, name)        # a prism over a quadrilateral
            for c in corners:
                self.assertLess(np.linalg.norm(xy - c, axis=1).min(), 1e-4, (name, c))
        diag = np.array(NEW_WALLS["Part67 / Boss-Extrude1[2]"])
        along = diag[2] - diag[1]
        self.assertAlmostEqual(math.degrees(math.atan2(along[1], along[0])), 42.94, delta=0.05)
        normal = np.array([-along[1], along[0]]) / np.linalg.norm(along)
        self.assertAlmostEqual(abs(float((diag[3] - diag[2]) @ normal)), 0.100, delta=0.001)
        b1 = self.solid("Barrier 1")
        self.assertAlmostEqual(float(diag[:2, 1].max()), b1.bbox_max[1], delta=1e-4)  # cut along its north face

    def test_barrier_3_is_cut_in_length_not_height(self):
        # "cut to 55 in" is the length (1.397 m = 55 in); its height is the full 68 in.
        s = self.solid("Barrier 3 (cut to 55 in)")
        _, _, _, p0, p1 = wall_frame(s)
        self.assertAlmostEqual(float(np.linalg.norm(p1 - p0)), 55 * 0.0254, delta=TOL)
        self.assertAlmostEqual(s.bbox_max[2] - s.bbox_min[2], 1.7272, delta=TOL)

    def test_c2a_window(self):
        name = "C2a chicane wall (window)"
        s = self.solid(name)
        index = MeshIndex([s])
        d, n, _, _, _ = wall_frame(s)
        p0 = np.array(WALLS[name][0])
        d = d if d @ (np.array(WALLS[name][1]) - p0) > 0 else -d
        s_lo, s_hi = 0.34925, 1.26365
        centre = p0 + d * 0.5 * (s_lo + s_hi)
        np.testing.assert_allclose(centre, (-0.470938, -2.271897), atol=TOL)

        def ray(s_along: float, z: float) -> float:
            o = np.r_[p0 + d * s_along - 0.3 * n, z]
            return float(index.raycast(o[None], np.r_[n, 0.0][None], 0.6)[0][0])

        self.assertEqual(ray(0.5 * (s_lo + s_hi), 0.76), np.inf)
        self.assertAlmostEqual(ray(0.5 * (s_lo + s_hi), 0.2), 0.3 - 0.0508, delta=TOL)
        self.assertAlmostEqual(ray(0.5 * (s_lo + s_hi), 1.4), 0.3 - 0.0508, delta=TOL)
        for s_along, z, open_ in ((s_lo + 0.005, 0.76, True), (s_lo - 0.005, 0.76, False),
                                  (s_hi - 0.005, 0.76, True), (s_hi + 0.005, 0.76, False),
                                  (0.8, 0.3048 + 0.005, True), (0.8, 0.3048 - 0.005, False),
                                  (0.8, 1.2192 - 0.005, True), (0.8, 1.2192 + 0.005, False)):
            self.assertEqual(np.isinf(ray(s_along, z)), open_, (s_along, z))
        # Opening corners straight from the mesh vertices.
        v = s.triangles.reshape(-1, 3)
        along = (v[:, :2] - p0) @ d
        inner = (along > 0.01) & (along < 1.6) & (v[:, 2] > 0.01) & (v[:, 2] < 1.7)
        self.assertAlmostEqual(along[inner].min(), s_lo, delta=TOL)
        self.assertAlmostEqual(along[inner].max(), s_hi, delta=TOL)
        self.assertAlmostEqual(v[inner, 2].min(), 0.3048, delta=TOL)
        self.assertAlmostEqual(v[inner, 2].max(), 1.2192, delta=TOL)

    def test_tube(self):
        s = self.solid(TUBE)
        np.testing.assert_allclose(s.bbox_min, (TUBE_X - 0.7366, -0.3048, 0.0254), atol=TOL)
        np.testing.assert_allclose(s.bbox_max, (TUBE_X + 0.7366, 1.2192, 1.8542), atol=TOL)
        index = MeshIndex([s])
        mid = np.array([TUBE_X, 0.4572, TUBE_Z])
        # Open at both ends: a ray down the axis passes straight through.
        self.assertEqual(index.raycast(np.array([[TUBE_X, 2.0, TUBE_Z]]), np.array([[0, -1.0, 0]]), 5.0)[0][0],
                         np.inf)
        dirs = np.array([[1.0, 0, 0], [-1.0, 0, 0], [0, 0, 1.0], [0, 0, -1.0]])
        dist, sid, normal = index.raycast(np.repeat(mid[None], 4, 0), dirs, 2.0)
        np.testing.assert_allclose(dist, (0.73025, 0.73025, 0.90805, 0.90805), atol=TOL)
        np.testing.assert_allclose(normal, -dirs, atol=0.1)  # inner wall faces the axis (facet tilt ~L/2R)
        self.assertAlmostEqual(mid[0] + dist[0], TUBE_X + 0.73025, delta=TOL)
        # Outer half-axes from outside.
        far = mid + 3.0 * np.vstack([dirs[0], dirs[2]])
        dist, _, _ = index.raycast(far, -np.vstack([dirs[0], dirs[2]]), 5.0)
        np.testing.assert_allclose(3.0 - dist, (0.7366, 0.9144), atol=TOL)
        wall = math.pi * (0.7366 * 0.9144 - 0.73025 * 0.90805) * 1.524
        self.assertAlmostEqual(s.volume() / wall, 1.0, delta=0.01)

    def test_net_and_floor(self):
        for name, axis, sign in (("Net side north", 1, 1), ("Net side south", 1, -1),
                                 ("Net side east", 0, 1), ("Net side west", 0, -1)):
            s = self.solid(name)
            inner = s.bbox_min[axis] if sign > 0 else s.bbox_max[axis]
            self.assertAlmostEqual(inner, sign * 3.048, delta=TOL, msg=name)
            self.assertAlmostEqual(s.bbox_max[axis] - s.bbox_min[axis], 0.00127, delta=0.0005, msg=name)
            self.assertAlmostEqual(s.bbox_max[2], 3.048, delta=TOL, msg=name)
        # the net roof is hidden in the 2026-11-07 assembly, so not exported: nothing is above the
        # panels' top, which is the course's height limit (world.course.Course.net_hi)
        self.assertNotIn("Net roof", course()[1])
        self.assertAlmostEqual(max(s.bbox_max[2] for s in course()[0]), 3.048, delta=TOL)
        floor = self.solid("Floor 20 x 20 ft")
        np.testing.assert_allclose(floor.bbox_max[:2] - floor.bbox_min[:2], (6.096, 6.096), atol=TOL)
        np.testing.assert_allclose(floor.bbox_min[:2], (-3.048, -3.048), atol=TOL)
        self.assertLessEqual(floor.bbox_max[2], 0.0 + 1e-9)


class CacheTests(unittest.TestCase):
    def test_cache_roundtrip_and_speed(self):
        solids, _, _ = course()
        with tempfile.TemporaryDirectory() as tmp:
            first = load_step_cached(STEP_PATH, tmp)
            files = list(Path(tmp).glob("*.npz"))
            self.assertEqual(len(files), 1)
            t0 = time.perf_counter()
            second = load_step_cached(STEP_PATH, tmp)
            self.assertLess(time.perf_counter() - t0, 0.5)
            self.assertEqual([s.name for s in second], [s.name for s in solids])
            for a, b in zip(solids, second):
                np.testing.assert_array_equal(a.triangles, b.triangles)
                self.assertEqual(a.color, b.color)
            self.assertEqual(len(first), len(second))
            load_step_cached(STEP_PATH, tmp, chord_tol=0.004)  # other parameters -> another entry
            self.assertEqual(len(list(Path(tmp).glob("*.npz"))), 2)


# --------------------------------------------------------------------------
# Synthetic models
# --------------------------------------------------------------------------

class SyntheticStepTests(unittest.TestCase):
    def load(self, text: str, **kw) -> List[Solid]:
        path = write_temp(text)
        try:
            return load_step(path, **kw)
        finally:
            os.unlink(path)

    def test_assembly_transform_direction(self):
        # child -> parent = T(item_2) @ inv(T(item_1)): local x -> world y, local y -> world -x,
        # then + (1, 2, 3) m.  The inverse interpretation would put the box at x in [-2, 0].
        for swapped in (False, True):
            (s,) = self.load(assembly_step(swapped))
            self.assertEqual(s.name, "Box")     # the product's name, not the occurrence's
            np.testing.assert_allclose(s.bbox_min, (-1.0, 2.0, 3.0), atol=1e-9)
            np.testing.assert_allclose(s.bbox_max, (1.0, 3.0, 6.0), atol=1e-9)
            self.assertAlmostEqual(s.volume(), 6.0, places=9)
            self.assertTrue(s.is_watertight)
            self.assertEqual(s.color, (0.2, 0.4, 0.6))
            self.assertEqual(len(s.triangles), 12)

    def test_curved_surfaces(self):
        r, h, R, rt = 0.5, 0.8, 1.0, 0.25
        tol = 0.001
        solids = {s.name: s for s in self.load(curved_solids_step(r, h, R, rt), chord_tol=tol, max_edge=0.05)}
        half_ball = 2 * math.pi * r ** 3 / 3
        expected = {"cone": math.pi * r * r * h / 3, "hemisphere": half_ball,
                    "seam cylinder": math.pi * r * r * h, "torus": 2 * math.pi ** 2 * R * rt * rt,
                    "extruded ellipse": math.pi * 0.6 * 0.3 * h, "revolved hemisphere": half_ball,
                    "lower hemisphere": half_ball}
        for name, vol in expected.items():
            s = solids[name]
            self.assertTrue(s.is_watertight, f"{name}: {s.watertight_report()}")
            self.assertEqual(s.notes, (), name)
            self.assertAlmostEqual(s.volume() / vol, 1.0, delta=0.01, msg=name)
        # All hemisphere vertices on the sphere, centroids within the chordal tolerance.
        t = solids["hemisphere"].triangles - np.array([3.0, 0.0, 0.0])
        dome = t[solids["hemisphere"].normals[:, 2] > -0.999999]  # everything but the base disc
        rad = np.linalg.norm(dome.reshape(-1, 3), axis=1)
        np.testing.assert_allclose(rad, r, atol=1e-9)
        self.assertLessEqual(r - np.linalg.norm(dome.mean(axis=1), axis=1).min(), tol + 1e-9)
        cone = solids["cone"]
        self.assertAlmostEqual(cone.bbox_max[2], h, places=9)
        self.assertAlmostEqual(solids["torus"].bbox_max[0], R + rt, delta=tol)
        self.assertAlmostEqual(solids["revolved hemisphere"].bbox_max[2], r, places=9)
        self.assertAlmostEqual(solids["lower hemisphere"].bbox_min[2], -r, delta=tol)
        self.assertAlmostEqual(solids["lower hemisphere"].bbox_max[2], 0.0, places=9)

    def test_product_label(self):
        cases = {"Barrier 1.step": "Barrier 1", "Tube oval 58 x 72 x 60.step": "Tube oval 58 x 72 x 60",
                 "Part67^dronehacks-course-v3.step": "Part67", "Part1^Assem1": "Part1",
                 "Bracket.SLDPRT": "Bracket", "Frame.Stp": "Frame", "Course v3.SLDASM": "Course v3",
                 " Launch mark.STEP ": "Launch mark", "Barrier 3 (cut to 55 in)": "Barrier 3 (cut to 55 in)",
                 "M3 maze divider": "M3 maze divider", "v1.5 step": "v1.5 step", "notes.txt": "notes.txt",
                 "": "", None: ""}
        for raw, want in cases.items():
            self.assertEqual(product_label(raw), want, raw)

    def test_solidworks_product_names_and_multibody_suffix(self):
        solids = self.load(parts_step([
            ("Barrier 1.step", ["Assy.step<1>"]),
            ("Part67^Assy.step", ["Boss-Extrude1[1]", "Boss-Extrude1[2]"]),
            ("Bracket.SLDPRT", ["", ""]),                  # blank body names: numbered
            ("Plate.stp", ["Body", "Body"]),               # repeated body names: numbered
        ]))
        self.assertEqual([s.name for s in solids],
                         ["Barrier 1", "Part67 / Boss-Extrude1[1]", "Part67 / Boss-Extrude1[2]",
                          "Bracket / 1", "Bracket / 2", "Plate / 1", "Plate / 2"])
        for s in solids:
            self.assertTrue(s.is_watertight, s.name)
            self.assertAlmostEqual(s.volume(), 1.0, places=9)

    def test_onshape_product_names_are_unchanged(self):
        # Onshape names the occurrence like its product, no extension; a repeated name gets " (2)"
        solids = self.load(parts_step([("Barrier 1", ["Barrier 1"]), ("M3 maze divider", ["M3 maze divider"]),
                                       ("Barrier 1", ["Barrier 1"])], root="FINAL Course v3", nauo_name=None))
        self.assertEqual([s.name for s in solids], ["Barrier 1", "M3 maze divider", "Barrier 1 (2)"])

    def test_edge_loops_through_a_pole(self):
        r = 0.5
        solids = {s.name: s for s in self.load(pole_loops_step(r), chord_tol=0.001, max_edge=0.05)}
        expected = {"split hemisphere": 2 * math.pi * r ** 3 / 3, "two-lune sphere": 4 * math.pi * r ** 3 / 3}
        for name, vol in expected.items():
            s = solids[name]
            self.assertTrue(s.is_watertight, f"{name}: {s.watertight_report()}")
            self.assertEqual(s.notes, (), name)              # no orientation flip needed
            self.assertAlmostEqual(s.volume() / vol, 1.0, delta=0.005, msg=name)
        dome = solids["split hemisphere"]
        np.testing.assert_allclose(dome.bbox_min, (-r, -r, 0.0), atol=0.001)
        np.testing.assert_allclose(dome.bbox_max, (r, r, r), atol=1e-9)
        curved = dome.triangles[dome.normals[:, 2] > -0.999999]
        np.testing.assert_allclose(np.linalg.norm(curved.reshape(-1, 3), axis=1), r, atol=1e-9)
        apex = np.all(np.isclose(curved, (0.0, 0.0, r), atol=1e-12), axis=2).any(axis=1)
        self.assertGreaterEqual(int(apex.sum()), 8)      # a fan of triangles at the pole

    def test_unsupported_entity_is_named(self):
        w = StepWriter()
        v = w.vertex((1, 0, 0))
        circ = w.circle_edge(v, (0, 0, 0), 1.0)
        surf = w.add("OFFSET_SURFACE('',#1,0.1,.F.)")
        face = w.face([(w.loop([(circ, True)]), True, "FACE_BOUND")], surf, True)
        msb = w.solid("odd", [face])
        w.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({msb}),{w.ctx})")
        with self.assertRaisesRegex(StepMeshError, "OFFSET_SURFACE"):
            self.load(w.text())


# --------------------------------------------------------------------------
# Mesh queries
# --------------------------------------------------------------------------

class MeshIndexCubeTests(unittest.TestCase):
    def setUp(self):
        self.index = MeshIndex([cube()], cell=0.3)

    def test_raycast(self):
        O = np.array([[-1, 0.5, 0.5], [0.5, 0.5, 0.5], [-1, 2, 0.5], [0.5, 0.5, 2.0], [0.25, 0.75, -3.0]])
        D = np.array([[1.0, 0, 0], [0, 0, 1.0], [1.0, 0, 0], [0, 0, -1.0], [0, 0, 1.0]])
        dist, sid, normal = self.index.raycast(O, D, 10.0)
        np.testing.assert_allclose(dist, [1.0, 0.5, np.inf, 1.0, 3.0])
        np.testing.assert_array_equal(sid, [0, 0, -1, 0, 0])
        np.testing.assert_allclose(normal, [[-1, 0, 0], [0, 0, 1], [0, 0, 0], [0, 0, 1], [0, 0, -1]], atol=1e-12)
        dist, _, _ = self.index.raycast(O[:1], D[:1], 0.9)
        self.assertEqual(dist[0], np.inf)
        dist, _, _ = self.index.raycast(O[:2], D[:2], np.array([2.0, 0.4]))
        np.testing.assert_allclose(dist, [1.0, np.inf])

    def test_raycast_random_against_slabs(self):
        rng = np.random.default_rng(7)
        O = rng.uniform(-2, 3, size=(400, 3))
        D = rng.normal(size=(400, 3))
        D /= np.linalg.norm(D, axis=1)[:, None]
        dist, _, _ = self.index.raycast(O, D, 20.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (0.0 - O) / D, (1.0 - O) / D
        tn = np.nanmax(np.minimum(t1, t2), axis=1)
        tf = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tf >= np.maximum(tn, 0)) & (tf > 0)
        expect = np.where(hit, np.where(tn > 0, tn, tf), np.inf)
        np.testing.assert_allclose(dist, expect, atol=1e-9)

    def test_closest(self):
        P = np.array([[0.5, 0.5, 1.2], [0.5, 0.5, 0.9], [1.3, 1.4, 0.5], [1.1, 1.1, 1.1],
                      [0.95, 0.95, 0.95], [0.99, 0.99, 0.5], [0.5, 0.5, 0.5]])
        dist, cp, sid, normal, inside = self.index.closest(P, 0.6)
        np.testing.assert_allclose(dist, [0.2, 0.1, 0.5, math.sqrt(0.03), 0.05, 0.01, 0.5], atol=1e-12)
        np.testing.assert_allclose(cp[:4], [[0.5, 0.5, 1.0], [0.5, 0.5, 1.0], [1.0, 1.0, 0.5], [1, 1, 1]],
                                   atol=1e-12)
        np.testing.assert_array_equal(inside, [False, True, False, False, True, True, True])
        np.testing.assert_allclose(normal[0], [0, 0, 1])
        self.assertTrue((sid == 0).all())
        dist, cp, sid, normal, inside = self.index.closest(P[-1:], 0.3)  # nothing within 0.3
        self.assertEqual((dist[0], sid[0], bool(inside[0])), (np.inf, -1, False))

    def test_closest_random_against_box_distance(self):
        rng = np.random.default_rng(3)
        P = rng.uniform(-0.5, 1.5, size=(2000, 3))
        dist, _, _, _, inside = self.index.closest(P, 2.0)
        outside_d = np.linalg.norm(np.maximum(0, np.maximum(-P, P - 1)), axis=1)
        inside_d = np.minimum(P, 1 - P).min(axis=1)
        is_in = ((P > 0) & (P < 1)).all(axis=1)
        np.testing.assert_allclose(dist, np.where(is_in, inside_d, outside_d), atol=1e-12)
        np.testing.assert_array_equal(inside, is_in)

    def test_support_height_and_exclude(self):
        self.assertEqual(self.index.support_height(0.5, 0.5, 2.0), (1.0, 0))
        self.assertEqual(self.index.support_height(0.5, 0.5, 1.0), (0.0, 0))  # strictly below z_max
        self.assertEqual(self.index.support_height(2.0, 2.0, 2.0), (-np.inf, -1))
        z, s = self.index.support_height(np.array([0.5, 2.0]), np.array([0.5, 2.0]), 2.0)
        np.testing.assert_allclose(z, [1.0, -np.inf])
        both = MeshIndex([cube("A"), cube("B", (2, 0, 0), (3, 1, 1))], exclude=["B"])
        self.assertEqual(both.solid_names, ["A"])
        self.assertEqual(both.raycast(np.array([[1.5, 0.5, 0.5]]), np.array([[1.0, 0, 0]]), 5.0)[1][0], -1)


class MeshIndexCourseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        solids, by_name, _ = course()
        cls.solids = solids
        cls.index = MeshIndex(solids)

    def test_raycast_timing(self):
        rng = np.random.default_rng(0)
        O = np.column_stack([rng.uniform(-2.8, 2.8, 500), rng.uniform(-2.8, 2.8, 500), rng.uniform(0.2, 2.5, 500)])
        D = rng.normal(size=(500, 3))
        D /= np.linalg.norm(D, axis=1)[:, None]
        self.index.raycast(O, D, 12.0)
        times = []
        for _ in range(7):
            t0 = time.perf_counter()
            dist, sid, _ = self.index.raycast(O, D, 12.0)
            times.append(time.perf_counter() - t0)
        self.assertLess(float(np.median(times)), 0.050)
        # the arena is enclosed by the nets and the floor but has no roof (2026-11-07 export)
        level_or_down = D[:, 2] <= 0
        self.assertTrue(np.isfinite(dist[level_or_down]).all())
        self.assertTrue((sid[level_or_down] >= 0).all())
        self.assertTrue((D[~np.isfinite(dist), 2] > 0).all())

    def test_closest_near_wall(self):
        name = "Barrier 2"
        sid = self.index.solid_id(name)
        dist, cp, s, normal, inside = self.index.closest(np.array([[-0.6, 0.608, 1.0], [-0.6, 0.4572, 1.0]]), 0.3)
        np.testing.assert_allclose(dist, [0.1, 0.0508], atol=1e-9)
        np.testing.assert_array_equal(s, [sid, sid])
        np.testing.assert_allclose(cp[0], [-0.6, 0.508, 1.0], atol=1e-9)
        np.testing.assert_allclose(normal[0], [0, 1, 0], atol=1e-9)
        np.testing.assert_array_equal(inside, [False, True])

    def test_support_height(self):
        idx = self.index
        z, s = idx.support_height(2.01168, -2.46888, 2.0)
        self.assertAlmostEqual(z, 0.5334, delta=TOL)
        self.assertEqual(idx.solid_names[s], "Boulder 1")
        z, s = idx.support_height(2.0, 2.5, 1.0)
        self.assertAlmostEqual(z, 0.0, delta=1e-9)
        self.assertEqual(idx.solid_names[s], "Floor 20 x 20 ft")
        z, s = idx.support_height(2.0, 2.5, 0.0)  # strictly below: the floor's underside
        self.assertAlmostEqual(z, -0.00635, delta=1e-6)
        z, s = idx.support_height(TUBE_X, 0.4572, 1.5)  # inside the tube: its inner bottom
        self.assertAlmostEqual(z, TUBE_Z - 0.90805, delta=TOL)
        self.assertEqual(idx.solid_names[s], TUBE)

    def test_exclude_nets_and_open_roof(self):
        no_nets = MeshIndex(self.solids, exclude=["Net *"])
        self.assertEqual(len(no_nets.solid_names), 27)
        self.assertFalse(any(n.startswith("Net") for n in no_nets.solid_names))
        north = np.array([[0.0, -1.0, 2.5]]), np.array([[0.0, 1.0, 0.0]])   # above every wall
        self.assertEqual(self.index.solid_names[self.index.raycast(*north, 8.0)[1][0]], "Net side north")
        self.assertEqual(no_nets.raycast(*north, 8.0)[0][0], np.inf)
        up = np.array([[0.0, -1.0, 2.0]]), np.array([[0.0, 0.0, 1.0]])
        self.assertEqual(self.index.raycast(*up, 5.0)[0][0], np.inf)        # no roof


if __name__ == "__main__":
    unittest.main()
