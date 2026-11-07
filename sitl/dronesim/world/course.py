"""The course: the STEP model as the simulator's ground truth.

Frames
------
* **course** -- the STEP file's frame: x east, y north, z up, metres, origin
  at the floor centre (``contracts/course.md``).
* **world** (physics) -- PX4 convention NED with the same origin: x = course
  north, y = course east, z = down.  ``ned_to_course`` / ``course_to_ned``.

Every solid keeps its STEP product name and gets a ``kind`` (wall, net, tunnel,
tunnel_support, boulder, floor, pad, marking) used for contact rules, sensor
models and scoring.  Two caches speed up the hot loop and are stored next to
the tessellation cache, keyed by the STEP file's SHA-256: a layered heightmap
of upward-facing surfaces (landing gear, ground effect) and a coarse distance
field (collision broadphase).
"""
from __future__ import annotations

import hashlib
import math
import os

import numpy as np

from .mesh_query import MeshIndex
from .step_mesh import load_step_cached

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

HEIGHTMAP_RES = 0.02
HEIGHTMAP_LAYERS = 4
DF_RES = 0.10
DF_CLAMP = 0.6
CACHE_VERSION = 2


def classify(name):
    for key, kind in KINDS:
        if key.lower() in name.lower():
            return kind
    return 'other'


def ned_to_course(p):
    return (p[1], p[0], -p[2])


course_to_ned = ned_to_course


class Course:
    def __init__(self, step_path, cache_dir):
        self.step_path = os.path.abspath(step_path)
        os.makedirs(cache_dir, exist_ok=True)
        self.cache_dir = cache_dir
        with open(self.step_path, 'rb') as fh:
            self.sha256 = hashlib.sha256(fh.read()).hexdigest()
        self.solids = load_step_cached(self.step_path, cache_dir)
        self.kinds = [classify(s.name) for s in self.solids]
        self.by_name = {s.name: s for s in self.solids}
        self.index = MeshIndex(self.solids)
        # per-solid kind, in the index's solid order
        self.solid_kind = [classify(n) for n in self.index.solid_names]
        tris = self.index.triangles
        self.tri_lo = tris.min(axis=1)
        self.tri_hi = tris.max(axis=1)
        lo = np.min([s.bbox_min for s in self.solids], axis=0)
        hi = np.max([s.bbox_max for s in self.solids], axis=0)
        self.bounds = (lo, hi)
        net = [s for s in self.solids if classify(s.name) == 'net']
        self.net_lo = np.min([s.bbox_min for s in net], axis=0) if net else lo
        self.net_hi = np.max([s.bbox_max for s in net], axis=0) if net else hi
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
        return os.path.join(self.cache_dir, f'course_{self.sha256[:16]}_v{CACHE_VERSION}.npz')

    def _load_or_build_caches(self):
        path = self._cache_path()
        if os.path.isfile(path):
            data = np.load(path)
            self.hm = data['hm']
            self.hm_kind = data['hm_kind']
            self.df = data['df']
            self.hm_origin = data['hm_origin']
            self.df_origin = data['df_origin']
            return
        self._build_heightmap()
        self._build_distance_field()
        tmp = path + '.tmp.npz'
        np.savez_compressed(tmp, hm=self.hm, hm_kind=self.hm_kind, df=self.df, hm_origin=self.hm_origin,
                            df_origin=self.df_origin)
        os.replace(tmp, path)

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
        zmax = np.full(gx.size, 3.0, dtype=float)          # below the net roof
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
        nz = int(math.ceil((3.3 - z0) / DF_RES)) + 1
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

    def describe(self):
        counts = {}
        for k in self.solid_kind:
            counts[k] = counts.get(k, 0) + 1
        return {'step': os.path.basename(self.step_path), 'sha256': self.sha256, 'solids': len(self.solids),
                'triangles': int(self.index.triangles.shape[0]), 'kinds': counts}


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
