"""The course: the STEP model as the simulator's ground truth.

Frames
------
* **course** -- the STEP file's frame: x east, y north, z up, metres, origin
  at the floor centre (``contracts/course.md``).
* **world** (physics) -- PX4 convention NED with the same origin: x = course
  north, y = course east, z = down.  ``ned_to_course`` / ``course_to_ned``.

Every solid keeps its STEP product name and gets a ``kind`` (wall, net, tunnel,
tunnel_support, boulder, floor, pad, marking) used for contact rules, sensor
models and scoring.  The kind comes from the name first (``KINDS``); a solid
whose name says nothing (``other``, e.g. a CAD default name such as ``Part67``)
that stands on the floor and reaches the wall top is a wall, so a newly added,
unnamed wall still counts as a touch (rule 7.8).

The net box (``net_lo`` / ``net_hi``, the bounding box of the net solids) is the
course volume: leaving it ends a run.  Its top is the net panels' top, so it is
also the height limit when the export has no roof solid.

Two caches speed up the hot loop and are stored next to the tessellation
cache, keyed by the STEP file's SHA-256 and the tessellation version: a layered
heightmap of upward-facing surfaces (landing gear, ground effect) and a coarse
distance field (collision broadphase).
"""
from __future__ import annotations

import hashlib
import math
import os
import tempfile

import numpy as np

from .mesh_query import MeshIndex
from .step_mesh import MESH_VERSION, load_step_cached

KINDS = (
    ('Net', 'net'), ('Floor', 'floor'), ('Tube cradle', 'tunnel_support'), ('Tube', 'tunnel'),
    ('Boulder', 'boulder'), ('Landing pad', 'pad'), ('Launch mark', 'marking'), ('Guideline', 'marking'),
    ('Approach ring', 'marking'), ('Barrier', 'wall'), ('maze divider', 'wall'), ('chicane wall', 'wall'),
    ('approach wall', 'wall'),
)
#: contacts with these count as "touches" (+5 s each, rule 7.8)
TOUCH_KINDS = {'wall', 'tunnel', 'tunnel_support', 'boulder'}
#: surfaces a landing gear may rest on
SUPPORT_KINDS = {'floor', 'pad', 'marking', 'boulder', 'tunnel', 'tunnel_support', 'wall'}

#: the rulebook's wall height (68 in); the floor top is z = 0 (contracts/course.md)
WALL_TOP_M = 1.7272
#: geometry fallback (``classify_solid``): how close to the floor a wall's base and to
#: WALL_TOP_M its top must be, and the least horizontal extent (thinner is a sheet, e.g. a net)
FLOOR_TOL_M = 0.02
WALL_TOP_TOL_M = 0.03
SHEET_MAX_M = 0.01

HEIGHTMAP_RES = 0.02
HEIGHTMAP_LAYERS = 4
DF_RES = 0.10
DF_CLAMP = 0.6
CACHE_VERSION = 3


def classify(name):
    """Kind of a solid from its name alone (``'other'`` if no keyword matches)."""
    for key, kind in KINDS:
        if key.lower() in name.lower():
            return kind
    return 'other'


def classify_solid(solid):
    """Kind of a solid: by name, else by shape.  A solid the name leaves as ``'other'``
    that stands on the floor and reaches the wall top is a ``'wall'`` (its touches
    count, rule 7.8), unless it is a thin sheet.  Nets are named, so they are
    classified before the fallback."""
    kind = classify(solid.name)
    if kind != 'other' or not len(solid.triangles):
        return kind
    lo, hi = solid.bbox_min, solid.bbox_max
    on_floor = abs(float(lo[2])) <= FLOOR_TOL_M
    tall = float(hi[2]) >= WALL_TOP_M - WALL_TOP_TOL_M
    sheet = float(min(hi[0] - lo[0], hi[1] - lo[1])) < SHEET_MAX_M
    return 'wall' if on_floor and tall and not sheet else kind


def ned_to_course(p):
    return (p[1], p[0], -p[2])


course_to_ned = ned_to_course


class Course:
    def __init__(self, step_path, cache_dir, edits_path=None):
        """``edits_path``: a course-edits/1 file (``world.arena``) applied on top of the STEP
        solids, e.g. the course as the team taped it; None is the STEP as exported."""
        self.step_path = os.path.abspath(step_path)
        os.makedirs(cache_dir, exist_ok=True)
        self.cache_dir = cache_dir
        with open(self.step_path, 'rb') as fh:
            self.sha256 = hashlib.sha256(fh.read()).hexdigest()
        self.solids = load_step_cached(self.step_path, cache_dir)
        self.edits = None
        self.edits_path = None
        self.edits_sha256 = None
        self.edit_notes = []
        self.arena = 'STEP as exported'
        edit_kinds = {}
        if edits_path:
            from .arena import ArenaError, apply_edits, load_edits
            self.edits_path = os.path.abspath(edits_path)
            self.edits, self.edits_sha256 = load_edits(self.edits_path)
            want = (self.edits.get('applies_to') or {}).get('step_sha256')
            if want and want != self.sha256:
                raise ArenaError(f'{os.path.basename(edits_path)} was written for STEP sha256 {want[:12]}..., '
                                 f'not {os.path.basename(self.step_path)} ({self.sha256[:12]}...)')
            self.solids, edit_kinds, self.edit_notes = apply_edits(self.solids, self.edits)
            self.arena = self.edits.get('name') or os.path.basename(edits_path)
        self.kinds = [edit_kinds.get(s.name) or classify_solid(s) for s in self.solids]
        self.by_name = {s.name: s for s in self.solids}
        self.kind_by_name = {s.name: k for s, k in zip(self.solids, self.kinds)}
        self.index = MeshIndex(self.solids)
        # per-solid kind, in the index's solid order
        self.solid_kind = [self.kind_by_name[n] for n in self.index.solid_names]
        tris = self.index.triangles
        self.tri_lo = tris.min(axis=1)
        self.tri_hi = tris.max(axis=1)
        lo = np.min([s.bbox_min for s in self.solids], axis=0)
        hi = np.max([s.bbox_max for s in self.solids], axis=0)
        self.bounds = (lo, hi)
        # the course volume: the nets' bounding box (its top is the panels' top: no roof needed)
        net = [s for s, k in zip(self.solids, self.kinds) if k == 'net']
        self.net_lo = np.min([s.bbox_min for s in net], axis=0) if net else lo
        self.net_hi = np.max([s.bbox_max for s in net], axis=0) if net else hi
        # heightmap rays start just below the net box top (a roof, if the export has one, and
        # the panels' top edges are no landing surfaces), or above everything without nets
        self._hm_top = float(self.net_hi[2]) - 0.01 if net else float(hi[2]) + 0.01
        self._load_or_build_caches()

    # -- geometry facts used elsewhere ---------------------------------------------------------
    def solid(self, name):
        return self.by_name[name]

    def centre_xy(self, name):
        s = self.by_name[name]
        return ((s.bbox_min[0] + s.bbox_max[0]) / 2.0, (s.bbox_min[1] + s.bbox_max[1]) / 2.0)

    def footprint(self, name, z_lo=None, z_hi=None):
        """Vertices of a solid (course frame), optionally within a z band."""
        v = self.by_name[name].triangles.reshape(-1, 3)
        if z_lo is not None:
            v = v[(v[:, 2] >= z_lo - 1e-6) & (v[:, 2] <= z_hi + 1e-6)]
        return v

    # -- caches -----------------------------------------------------------------------------------
    def _cache_path(self):
        from .arena import ARENA_MESH_VERSION
        arena = f'_e{self.edits_sha256[:16]}a{ARENA_MESH_VERSION}' if self.edits_sha256 else ''
        return os.path.join(self.cache_dir, f'course_{self.sha256[:16]}{arena}_v{CACHE_VERSION}m{MESH_VERSION}.npz')

    def _load_or_build_caches(self):
        path = self._cache_path()
        if os.path.isfile(path):
            # read everything and close the file, so another process may replace it (Windows)
            with np.load(path) as data:
                self.hm = data['hm']
                self.hm_kind = data['hm_kind']
                self.df = data['df']
                self.hm_origin = data['hm_origin']
                self.df_origin = data['df_origin']
            return
        self._build_heightmap()
        self._build_distance_field()
        # several simulator processes may build the cache at once: each writes its own temp file
        fd, tmp = tempfile.mkstemp(dir=self.cache_dir, prefix='course_', suffix='.tmp.npz')
        try:
            with os.fdopen(fd, 'wb') as fh:
                np.savez_compressed(fh, hm=self.hm, hm_kind=self.hm_kind, df=self.df, hm_origin=self.hm_origin,
                                    df_origin=self.df_origin)
            try:
                os.replace(tmp, path)
            except PermissionError:
                if not os.path.isfile(path):     # the target is busy but missing: a real failure
                    raise
                # another process wrote the same cache first and has it open (Windows): keep theirs
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def _build_heightmap(self):
        lo, hi = self.bounds
        x0, y0 = float(lo[0]) - 0.1, float(lo[1]) - 0.1
        nx = int(math.ceil((float(hi[0]) - x0 + 0.1) / HEIGHTMAP_RES)) + 1
        ny = int(math.ceil((float(hi[1]) - y0 + 0.1) / HEIGHTMAP_RES)) + 1
        xs = x0 + np.arange(nx) * HEIGHTMAP_RES
        ys = y0 + np.arange(ny) * HEIGHTMAP_RES
        gx, gy = np.meshgrid(xs, ys, indexing='ij')
        gx, gy = gx.ravel(), gy.ravel()
        hm = np.full((HEIGHTMAP_LAYERS, gx.size), -np.inf, dtype=np.float32)
        kind = np.full((HEIGHTMAP_LAYERS, gx.size), -1, dtype=np.int16)
        zmax = np.full(gx.size, self._hm_top, dtype=float)
        chunk = 4096
        for layer in range(HEIGHTMAP_LAYERS):
            for s in range(0, gx.size, chunk):
                z, sid = self.index.support_height(gx[s:s + chunk], gy[s:s + chunk], zmax[s:s + chunk])
                hm[layer, s:s + chunk] = z
                kind[layer, s:s + chunk] = sid
            zmax = np.where(np.isfinite(hm[layer]), hm[layer] - 0.005, -np.inf)
            if not np.isfinite(zmax).any():
                break
        self.hm = hm.reshape(HEIGHTMAP_LAYERS, nx, ny)
        self.hm_kind = kind.reshape(HEIGHTMAP_LAYERS, nx, ny)
        self.hm_origin = np.array([x0, y0, HEIGHTMAP_RES])

    def _build_distance_field(self):
        lo, hi = self.bounds
        x0, y0, z0 = float(lo[0]) - 0.2, float(lo[1]) - 0.2, -0.2
        nx = int(math.ceil((float(hi[0]) + 0.2 - x0) / DF_RES)) + 1
        ny = int(math.ceil((float(hi[1]) + 0.2 - y0) / DF_RES)) + 1
        nz = int(math.ceil((float(self.net_hi[2]) + 0.25 - z0) / DF_RES)) + 1
        xs = x0 + np.arange(nx) * DF_RES
        ys = y0 + np.arange(ny) * DF_RES
        zs = z0 + np.arange(nz) * DF_RES
        g = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1).reshape(-1, 3)
        out = np.empty(g.shape[0], dtype=np.float32)
        chunk = 2048
        for s in range(0, g.shape[0], chunk):
            d, *_ = self.index.closest(g[s:s + chunk], DF_CLAMP)
            out[s:s + chunk] = np.minimum(d, DF_CLAMP)
        self.df = out.reshape(nx, ny, nz)
        self.df_origin = np.array([x0, y0, z0, DF_RES])

    # -- fast queries (course frame) ------------------------------------------------------------------
    def support_below(self, x, y, z):
        """(height, solid index) of the highest upward surface at or below z (+2 cm), else (-inf, -1)."""
        x0, y0, res = self.hm_origin
        i = int(round((x - x0) / res))
        j = int(round((y - y0) / res))
        hm = self.hm
        if i < 0 or j < 0 or i >= hm.shape[1] or j >= hm.shape[2]:
            return -math.inf, -1
        for layer in range(hm.shape[0]):
            h = float(hm[layer, i, j])
            if h <= z + 0.02:
                return h, int(self.hm_kind[layer, i, j])
        return -math.inf, -1

    def clearance(self, x, y, z):
        """Distance to the nearest surface, clamped to DF_CLAMP (coarse, conservative +-0.09 m)."""
        x0, y0, z0, res = self.df_origin
        i = int(round((x - x0) / res))
        j = int(round((y - y0) / res))
        k = int(round((z - z0) / res))
        df = self.df
        if i < 0 or j < 0 or k < 0 or i >= df.shape[0] or j >= df.shape[1] or k >= df.shape[2]:
            return DF_CLAMP
        return float(df[i, j, k])

    def raycast_course(self, origins, dirs, max_range):
        return self.index.raycast(np.asarray(origins, float), np.asarray(dirs, float), max_range)

    def raycast_ned(self, origin_ned, dir_ned, max_range):
        o = np.array([ned_to_course(origin_ned)], dtype=float)
        d = np.array([ned_to_course(dir_ned)], dtype=float)
        dist, _sid, _n = self.index.raycast(o, d, max_range)
        return float(dist[0])

    def triangles_near(self, centre, radius):
        """Indices of triangles whose bounding box comes within ``radius`` of ``centre`` (course)."""
        c = np.asarray(centre, float)
        m = np.all(self.tri_lo <= c + radius, axis=1) & np.all(self.tri_hi >= c - radius, axis=1)
        return np.nonzero(m)[0]

    def inside_net(self, p, margin=0.0):
        return bool(np.all(np.asarray(p) >= self.net_lo - margin) and np.all(np.asarray(p) <= self.net_hi + margin))

    def kind(self, name):
        return self.kind_by_name[name]

    def describe(self):
        counts = {}
        for k in self.solid_kind:
            counts[k] = counts.get(k, 0) + 1
        return {'step': os.path.basename(self.step_path), 'sha256': self.sha256, 'solids': len(self.solids),
                'triangles': int(self.index.triangles.shape[0]), 'kinds': counts,
                'net_top_m': round(float(self.net_hi[2]), 4), 'arena': self.arena,
                'edits': os.path.basename(self.edits_path) if self.edits_path else None,
                'edits_sha256': self.edits_sha256}


def closest_points_on_triangles(p, a, b, c):
    """Vectorised closest point (Ericson) of points p (P,3) to triangles (T,3) each -> (P,T,3)."""
    p = p[:, None, :]
    ab, ac, ap = b - a, c - a, p - a
    d1 = np.einsum('ijk,jk->ij', ap, ab)
    d2 = np.einsum('ijk,jk->ij', ap, ac)
    bp = p - b
    d3 = np.einsum('ijk,jk->ij', bp, ab)
    d4 = np.einsum('ijk,jk->ij', bp, ac)
    cp = p - c
    d5 = np.einsum('ijk,jk->ij', cp, ab)
    d6 = np.einsum('ijk,jk->ij', cp, ac)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = va + vb + vc
    denom = np.where(np.abs(denom) < 1e-18, 1e-18, denom)
    v = vb / denom
    w = vc / denom
    res = a + ab * v[..., None] + ac * w[..., None]
    # vertex / edge regions
    m = (d1 <= 0) & (d2 <= 0)
    res = np.where(m[..., None], a, res)
    m = (d3 >= 0) & (d4 <= d3)
    res = np.where(m[..., None], b, res)
    m = (d6 >= 0) & (d5 <= d6)
    res = np.where(m[..., None], c, res)
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    t = d1 / np.where(np.abs(d1 - d3) < 1e-18, 1e-18, d1 - d3)
    res = np.where(m[..., None], a + ab * t[..., None], res)
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    t = d2 / np.where(np.abs(d2 - d6) < 1e-18, 1e-18, d2 - d6)
    res = np.where(m[..., None], a + ac * t[..., None], res)
    m = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    t = (d4 - d3) / np.where(np.abs((d4 - d3) + (d5 - d6)) < 1e-18, 1e-18, (d4 - d3) + (d5 - d6))
    res = np.where(m[..., None], b + (c - b) * t[..., None], res)
    return res
