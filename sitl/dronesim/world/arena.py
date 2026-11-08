"""Arenas: the course as measured, written as edits on top of the STEP model.

The STEP export is the CAD. Where the team has taped the real course and found it different, a
``course-edits/1`` file (``sitl/course/*_edits.json``) records what changed:

* ``remove``  -- solids that are not on the real course, by name;
* ``move``    -- solids translated in xy: each named solid's bounding-box centre must be within
  ``MOVE_TOLERANCE_M`` of ``from``, and it moves by ``to - from``;
* ``replace`` -- a solid rebuilt as a wall (``p0``/``p1`` centreline, ``thickness``, ``z`` range,
  ``openings`` with ``s`` measured from ``p0`` along ``p0 -> p1``), keeping its name so the
  referee's gates still find it;
* ``add``     -- new walls in the same form;
* ``rules.checkpoints.<name>.gate`` -- a checkpoint's gate for this arena (scoring only; keys the
  planner reads, such as ``planner_goal``, are ignored here).

Edits apply in that order. ``ARENAS`` names the arenas a scenario can pick; ``step`` is the STEP as
exported, with no edits. The same edits file drives the team's map converter
(``nav_traj/maps/source``), so the simulated world and the flown map stay one source.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os

import numpy as np

from .step_mesh import Solid

FORMAT = 'course-edits/1'
SITL_COURSE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                               'course')
#: arena name -> (label, edits file in sitl/course or None)
ARENAS = {
    'step': ('STEP as exported (dronehacks-course-v3.step)', None),
    'taped-2026-11-08': ("taped 2026-11-08 (Michael Bar On's course map: north wall, short diagonal, "
                         "high window, moved pad and rocks)", 'arena_2026-11-08_edits.json'),
}
DEFAULT_ARENA = 'step'
#: a moved solid's bounding-box centre must be this close to the edit's ``from``: the STEP's
#: tessellated pad rings and boulder domes are symmetric, so their bbox centres sit within 1e-5 m
MOVE_TOLERANCE_M = 0.01
#: bump when the meshes built here change: it is part of an edited course's cache key
ARENA_MESH_VERSION = 1


class ArenaError(ValueError):
    pass


def arena_names():
    return list(ARENAS)


def arena_label(name):
    return ARENAS[check_arena(name)][0]


def check_arena(name):
    if name not in ARENAS:
        raise ArenaError(f'unknown arena {name!r}; known: {", ".join(ARENAS)}')
    return name


def edits_path(name):
    """The edits file of an arena (None for the STEP as exported)."""
    fname = ARENAS[check_arena(name)][1]
    return os.path.join(SITL_COURSE_DIR, fname) if fname else None


def load_edits(path):
    """``(edits dict, sha256)`` of a course-edits/1 file."""
    with open(path, 'rb') as fh:
        raw = fh.read()
    edits = json.loads(raw.decode('utf-8'))
    if edits.get('format') != FORMAT:
        raise ArenaError(f'{path}: expected format {FORMAT!r}, got {edits.get("format")!r}')
    return edits, hashlib.sha256(raw).hexdigest()


# -- geometry ---------------------------------------------------------------------------------------
def _quad(out, a, b, c, d, outward):
    """Two triangles for the planar quad a-b-c-d, wound counter-clockwise seen from ``outward``."""
    a, b, c, d = (np.asarray(p, float) for p in (a, b, c, d))
    if np.dot(np.cross(b - a, c - a), outward) < 0:
        a, b, c, d = a, d, c, b
    out.append((a, b, c))
    out.append((a, c, d))


def wall_triangles(p0, p1, thickness, z, openings=()):
    """A closed, outward-wound mesh of a straight wall with rectangular openings: every edge is
    shared by exactly two triangles, with no inner faces.

    A wall with one opening inside it (a window) is meshed as the STEP meshes one: each face is
    four trapezoids from the outer corners to the opening's corners, so the only vertices
    between the wall's foot and top are the opening's (the referee's ``opening`` gate reads the
    opening from them). Anything else (several openings, a doorway) is cut on a grid of every
    opening edge instead."""
    p0 = np.array([float(p0[0]), float(p0[1]), 0.0])
    p1 = np.array([float(p1[0]), float(p1[1]), 0.0])
    length = float(np.linalg.norm(p1 - p0))
    if length <= 1e-9 or thickness <= 0:
        raise ArenaError(f'wall {p0[:2]}-{p1[:2]}: zero length or thickness')
    z0, z1 = float(z[0]), float(z[1])
    holes = [h for h in (_clip_opening(o, length, z0, z1) for o in openings or ()) if h]
    if len(holes) == 1:
        sa, sb, za, zb = holes[0]
        if sa > 1e-6 and sb < length - 1e-6 and za > z0 + 1e-6 and zb < z1 - 1e-6:
            return _window_wall(p0, p1, length, thickness, z0, z1, holes[0])
    return _grid_wall(p0, p1, thickness, z, openings)


def _clip_opening(o, length, z0, z1):
    sa, sb = max(0.0, float(o['s'][0])), min(length, float(o['s'][1]))
    za, zb = max(z0, float(o['z'][0])), min(z1, float(o['z'][1]))
    return (sa, sb, za, zb) if sb - sa > 1e-6 and zb - za > 1e-6 else None


def _window_wall(p0, p1, length, thickness, z0, z1, hole):
    u = (p1 - p0) / length
    n = np.array([-u[1], u[0], 0.0])
    up = np.array([0.0, 0.0, 1.0])
    h = thickness / 2.0
    sa, sb, za, zb = hole

    def pt(s, w, zz):
        return p0 + u * s + n * w + up * zz

    tris = []
    for w, nrm in ((-h, -n), (h, n)):        # each face: four trapezoids around the opening
        o00, o10, o11, o01 = pt(0, w, z0), pt(length, w, z0), pt(length, w, z1), pt(0, w, z1)
        i00, i10, i11, i01 = pt(sa, w, za), pt(sb, w, za), pt(sb, w, zb), pt(sa, w, zb)
        _quad(tris, o00, o10, i10, i00, nrm)
        _quad(tris, o10, o11, i11, i10, nrm)
        _quad(tris, o11, o01, i01, i11, nrm)
        _quad(tris, o01, o00, i00, i01, nrm)
    _quad(tris, pt(0, -h, z0), pt(0, h, z0), pt(0, h, z1), pt(0, -h, z1), -u)                      # ends
    _quad(tris, pt(length, -h, z0), pt(length, h, z0), pt(length, h, z1), pt(length, -h, z1), u)
    _quad(tris, pt(0, -h, z0), pt(length, -h, z0), pt(length, h, z0), pt(0, h, z0), -up)            # foot
    _quad(tris, pt(0, -h, z1), pt(length, -h, z1), pt(length, h, z1), pt(0, h, z1), up)             # top
    _quad(tris, pt(sa, -h, za), pt(sb, -h, za), pt(sb, h, za), pt(sa, h, za), up)                   # sill
    _quad(tris, pt(sa, -h, zb), pt(sb, -h, zb), pt(sb, h, zb), pt(sa, h, zb), -up)                  # head
    _quad(tris, pt(sa, -h, za), pt(sa, h, za), pt(sa, h, zb), pt(sa, -h, zb), u)                    # jambs
    _quad(tris, pt(sb, -h, za), pt(sb, h, za), pt(sb, h, zb), pt(sb, -h, zb), -u)
    return np.array(tris, dtype=float).reshape(-1, 3, 3)


def _grid_wall(p0, p1, thickness, z, openings=()):
    """The general case: both faces cut on one grid of every opening's ``s`` and ``z`` edges, so
    all faces meet edge to edge."""
    p0 = np.array([float(p0[0]), float(p0[1]), 0.0])
    p1 = np.array([float(p1[0]), float(p1[1]), 0.0])
    length = float(np.linalg.norm(p1 - p0))
    if length <= 1e-9 or thickness <= 0:
        raise ArenaError(f'wall {p0[:2]}-{p1[:2]}: zero length or thickness')
    u = (p1 - p0) / length                       # along the wall
    n = np.array([-u[1], u[0], 0.0])             # across it (left of p0 -> p1)
    up = np.array([0.0, 0.0, 1.0])
    z0, z1 = float(z[0]), float(z[1])
    holes = []
    for o in openings or ():
        sa, sb = max(0.0, float(o['s'][0])), min(length, float(o['s'][1]))
        za, zb = max(z0, float(o['z'][0])), min(z1, float(o['z'][1]))
        if sb - sa > 1e-6 and zb - za > 1e-6:
            holes.append((sa, sb, za, zb))
    ss = sorted({0.0, length, *[h[0] for h in holes], *[h[1] for h in holes]})
    zs = sorted({z0, z1, *[h[2] for h in holes], *[h[3] for h in holes]})
    ni, nk = len(ss) - 1, len(zs) - 1

    def solid(i, k):
        if i < 0 or k < 0 or i >= ni or k >= nk:
            return False
        sm, zm = (ss[i] + ss[i + 1]) / 2, (zs[k] + zs[k + 1]) / 2
        return not any(sa <= sm <= sb and za <= zm <= zb for sa, sb, za, zb in holes)

    h = thickness / 2.0

    def pt(s, w, zz):
        return p0 + u * s + n * w + up * zz

    tris = []
    for i in range(ni):
        for k in range(nk):
            if not solid(i, k):
                continue
            sa, sb, za, zb = ss[i], ss[i + 1], zs[k], zs[k + 1]
            for w, nrm in ((-h, -n), (h, n)):                       # the two faces
                _quad(tris, pt(sa, w, za), pt(sb, w, za), pt(sb, w, zb), pt(sa, w, zb), nrm)
            if not solid(i - 1, k):                                 # towards p0
                _quad(tris, pt(sa, -h, za), pt(sa, h, za), pt(sa, h, zb), pt(sa, -h, zb), -u)
            if not solid(i + 1, k):                                 # towards p1
                _quad(tris, pt(sb, -h, za), pt(sb, h, za), pt(sb, h, zb), pt(sb, -h, zb), u)
            if not solid(i, k - 1):                                 # below
                _quad(tris, pt(sa, -h, za), pt(sb, -h, za), pt(sb, h, za), pt(sa, h, za), -up)
            if not solid(i, k + 1):                                 # above
                _quad(tris, pt(sa, -h, zb), pt(sb, -h, zb), pt(sb, h, zb), pt(sa, h, zb), up)
    return np.array(tris, dtype=float).reshape(-1, 3, 3)


def wall_solid(spec, color=None):
    tris = wall_triangles(spec['p0'], spec['p1'], float(spec['thickness']), spec['z'], spec.get('openings') or ())
    return Solid.from_triangles(spec['name'], tris, color, notes=('arena edit: wall built from its centreline',))


def _translated(solid, dx, dy, note):
    tris = solid.triangles + np.array([dx, dy, 0.0])
    return Solid(solid.name, tris, solid.normals.copy(), solid.color, solid.bbox_min + np.array([dx, dy, 0.0]),
                 solid.bbox_max + np.array([dx, dy, 0.0]), tuple(solid.notes) + (note,))


def _centre(solid):
    return ((float(solid.bbox_min[0]) + float(solid.bbox_max[0])) / 2.0,
            (float(solid.bbox_min[1]) + float(solid.bbox_max[1])) / 2.0)


def apply_edits(solids, edits, tolerance_m=MOVE_TOLERANCE_M):
    """``(solids, kinds, notes)``: the STEP solids with the arena's edits applied, in order.

    ``kinds`` maps each solid an edit built to the edit's ``kind`` (the caller's name-based
    classification stays for the rest)."""
    out = list(solids)
    kinds = {}
    notes = []

    def index(name):
        for i, s in enumerate(out):
            if s.name == name:
                return i
        raise ArenaError(f'arena edit names a solid the course does not have: {name!r}')

    for r in edits.get('remove') or ():
        out.pop(index(r['name']))
        notes.append(f"removed {r['name']}")
    for m in edits.get('move') or ():
        fx, fy = (float(v) for v in m['from'])
        tx, ty = (float(v) for v in m['to'])
        for name in m['names']:
            i = index(name)
            cx, cy = _centre(out[i])
            off = math.hypot(cx - fx, cy - fy)
            if off > tolerance_m:
                raise ArenaError(f'{name}: its centre ({cx:.4f}, {cy:.4f}) is {off:.3f} m from the edit\'s '
                                 f'from ({fx}, {fy}); the edits were written for another course')
            out[i] = _translated(out[i], tx - fx, ty - fy, f'arena edit: moved by ({tx - fx:+.4f}, {ty - fy:+.4f})')
        notes.append(f"moved {', '.join(m['names'])} to ({tx}, {ty})")
    for r in edits.get('replace') or ():
        i = index(r['name'])
        out[i] = wall_solid(r, out[i].color)
        kinds[r['name']] = r.get('kind', 'wall')
        notes.append(f"rebuilt {r['name']}")
    for a in edits.get('add') or ():
        if any(s.name == a['name'] for s in out):
            raise ArenaError(f"arena edit adds {a['name']!r}, which the course already has")
        out.append(wall_solid(a))
        kinds[a['name']] = a.get('kind', 'wall')
        notes.append(f"added {a['name']}")
    return out, kinds, notes


def apply_rule_edits(rules, edits):
    """The rules (course-rules/1) with the arena's checkpoint gates in place of the STEP's."""
    if not edits:
        return rules
    over = ((edits.get('rules') or {}).get('checkpoints')) or {}
    if not over:
        return rules
    rules = copy.deepcopy(rules)
    names = {cp['name'] for cp in rules['checkpoints']}
    for name in over:
        if name not in names:
            raise ArenaError(f'arena rules name a checkpoint the rules file does not have: {name!r}')
    for cp in rules['checkpoints']:
        o = over.get(cp['name'])
        if o and 'gate' in o:
            cp['gate'] = copy.deepcopy(o['gate'])
    return rules
