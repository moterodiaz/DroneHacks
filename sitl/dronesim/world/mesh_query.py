"""Fast numpy spatial queries over a set of tessellated solids.

:class:`MeshIndex` concatenates the triangles of the selected solids and bins
them into a uniform 3-D grid of cubic cells (a triangle is stored in every
cell its bounding box touches whose box also straddles the triangle's plane).
All queries are vectorised over many rays / points at once:

* :meth:`MeshIndex.raycast` walks every ray through the grid in lock-step
  (Amanatides-Woo 3-D DDA), a few cells per batch, and tests the triangles of
  the visited cells with the Moller-Trumbore algorithm (two-sided).  A ray is
  retired as soon as its best hit lies before the next cell it would enter.
* :meth:`MeshIndex.closest` gathers the triangles in the cells overlapping
  each point's search sphere and computes exact point-triangle closest points
  (Ericson, *Real-Time Collision Detection* 5.1.5).  Whether the point is
  inside the closed solid owning the closest triangle comes from the sign
  against the angle-weighted pseudo-normal of the closest feature (face, edge
  or vertex; Baerentzen & Aanaes 2005), which is exact for closed,
  consistently oriented meshes even when the closest point is on an edge or
  vertex.
* :meth:`MeshIndex.support_height` is a downward ray cast.
"""

from __future__ import annotations

from fnmatch import fnmatch
from typing import Iterable, List, Sequence, Tuple, Union

import numpy as np

from .step_mesh import Solid

__all__ = ["MeshIndex"]

ArrayLike = Union[float, Sequence[float], np.ndarray]


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.stack([a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
                     a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
                     a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]], axis=1)


def _dot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1] + a[:, 2] * b[:, 2]


def _expand_boxes(lo: np.ndarray, hi: np.ndarray, dims: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """All integer cells in the inclusive boxes [lo, hi] (K, 3).  Returns (box index, flat cell id)."""
    ext = hi - lo + 1
    count = ext.prod(axis=1)
    box = np.repeat(np.arange(len(lo)), count)
    local = np.arange(int(count.sum())) - np.repeat(np.cumsum(count) - count, count)
    ex, ey = ext[box, 0], ext[box, 1]
    ix = lo[box, 0] + local % ex
    iy = lo[box, 1] + (local // ex) % ey
    iz = lo[box, 2] + local // (ex * ey)
    return box, (iz * dims[1] + iy) * dims[0] + ix


class MeshIndex:
    """Uniform-grid acceleration structure over the triangles of several solids.

    ``exclude`` drops solids whose name matches any of the given ``fnmatch``
    patterns (e.g. ``exclude=["Net side north"]`` or ``["Net *"]``).
    ``solid_names[i]`` is the name of solid id ``i``; ``tri_solid[t]`` is the
    solid id of triangle ``t``.
    """

    #: Hits closer than this along the ray are ignored (avoids self-hits at t = 0).
    T_MIN = 1e-9

    def __init__(self, solids: Iterable[Solid], cell: float = 0.25, exclude: Iterable[str] = ()) -> None:
        patterns = list(exclude)
        chosen: List[Solid] = [s for s in solids if not any(fnmatch(s.name, p) for p in patterns)]
        chosen = [s for s in chosen if len(s.triangles)]
        if not chosen:
            raise ValueError("MeshIndex needs at least one non-empty solid")
        if cell <= 0:
            raise ValueError("cell must be positive")
        self.solids = chosen
        self.solid_names: List[str] = [s.name for s in chosen]
        tris = np.concatenate([s.triangles for s in chosen]).astype(np.float64)
        self.triangles = tris
        self.normals = np.concatenate([s.normals for s in chosen]).astype(np.float64)
        self.tri_solid = np.repeat(np.arange(len(chosen)), [len(s.triangles) for s in chosen])
        self.v0 = np.ascontiguousarray(tris[:, 0])
        self.e1 = np.ascontiguousarray(tris[:, 1] - tris[:, 0])
        self.e2 = np.ascontiguousarray(tris[:, 2] - tris[:, 0])
        self._build_grid(cell)
        self._build_pseudo_normals()

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------
    def _build_grid(self, cell: float) -> None:
        tris = self.triangles
        tmin, tmax = tris.min(axis=1), tris.max(axis=1)
        pad = 1e-6 + 1e-3 * cell
        lo = tmin.min(axis=0) - pad
        dims = np.maximum(1, np.ceil((tmax.max(axis=0) + pad - lo) / cell).astype(np.int64))
        self.cell = float(cell)
        self.lo = lo
        self.dims = dims
        self.hi = lo + dims * cell
        clo = np.clip(np.floor((tmin - lo) / cell).astype(np.int64), 0, dims - 1)
        chi = np.clip(np.floor((tmax - lo) / cell).astype(np.int64), 0, dims - 1)
        tri, cid = _expand_boxes(clo, chi, dims)
        # Drop cells whose box does not straddle the triangle's plane.
        ix = cid % dims[0]
        iy = (cid // dims[0]) % dims[1]
        iz = cid // (dims[0] * dims[1])
        centre = lo + (np.stack([ix, iy, iz], axis=1) + 0.5) * cell
        n = self.normals[tri]
        reach = 0.5 * cell * np.abs(n).sum(axis=1) + 1e-9
        keep = np.abs(_dot(n, centre - self.v0[tri])) <= reach
        tri, cid = tri[keep], cid[keep]
        order = np.argsort(cid, kind="stable")
        self._cell_tris = tri[order]
        ncell = int(dims.prod())
        self._cell_count = np.bincount(cid, minlength=ncell).astype(np.int64)
        self._cell_start = np.cumsum(self._cell_count) - self._cell_count

    def _build_pseudo_normals(self) -> None:
        tris, normals = self.triangles, self.normals
        T = len(tris)
        keys = np.round(tris.reshape(-1, 3) / 1e-7).astype(np.int64)
        keys = np.hstack([np.repeat(self.tri_solid, 3)[:, None], keys])  # never weld across solids
        _, vid = np.unique(keys, axis=0, return_inverse=True)
        vid = vid.reshape(T, 3)
        nv = int(vid.max()) + 1
        vnorm = np.zeros((nv, 3))
        for k in range(3):
            a = tris[:, (k + 1) % 3] - tris[:, k]
            b = tris[:, (k + 2) % 3] - tris[:, k]
            cosang = _dot(a, b) / np.maximum(np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1), 1e-300)
            ang = np.arccos(np.clip(cosang, -1.0, 1.0))
            np.add.at(vnorm, vid[:, k], ang[:, None] * normals)
        ea, eb = vid, np.roll(vid, -1, axis=1)  # edge k joins corners k and k+1
        ekey = (np.minimum(ea, eb) * nv + np.maximum(ea, eb)).ravel()
        _, eid = np.unique(ekey, return_inverse=True)
        eid = eid.reshape(T, 3)
        enorm = np.zeros((int(eid.max()) + 1, 3))
        for k in range(3):
            np.add.at(enorm, eid[:, k], normals)
        self._vid, self._vnorm = vid, vnorm
        self._eid, self._enorm = eid, enorm

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def solid_id(self, name: str) -> int:
        """Index of the solid called ``name`` (``ValueError`` if absent)."""
        return self.solid_names.index(name)

    def _candidates(self, ids: np.ndarray, cells: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Expand (query id, cell) pairs to (query id, triangle) pairs."""
        cnt = self._cell_count[cells]
        nz = cnt > 0
        ids, cells, cnt = ids[nz], cells[nz], cnt[nz]
        if not len(ids):
            return ids, ids
        total = int(cnt.sum())
        q = np.repeat(ids, cnt)
        offs = np.repeat(self._cell_start[cells] - (np.cumsum(cnt) - cnt), cnt) + np.arange(total)
        return q, self._cell_tris[offs]

    def _moller_trumbore(self, O: np.ndarray, D: np.ndarray, tri: np.ndarray, R: np.ndarray) -> np.ndarray:
        """Ray parameter of each (ray, triangle) pair, inf when it misses."""
        e1, e2 = self.e1[tri], self.e2[tri]
        p = _cross(D, e2)
        det = _dot(e1, p)
        ok = np.abs(det) > 1e-300
        inv = np.divide(1.0, det, out=np.zeros_like(det), where=ok)
        s = O - self.v0[tri]
        u = _dot(s, p) * inv
        q = _cross(s, e1)
        v = _dot(D, q) * inv
        t = _dot(e2, q) * inv
        eps = 1e-10
        hit = ok & (u >= -eps) & (v >= -eps) & (u + v <= 1.0 + eps) & (t > self.T_MIN) & (t <= R)
        return np.where(hit, t, np.inf)

    # ------------------------------------------------------------------
    # queries
    # ------------------------------------------------------------------
    def raycast(self, origins: np.ndarray, dirs: np.ndarray, max_range: ArrayLike = np.inf,
                batch_cells: int = 6) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """First intersection of each ray with any triangle (two-sided).

        ``origins`` and ``dirs`` are (M, 3) (or (3,)); ``dirs`` should be unit
        length so that distances are metres.  ``max_range`` is a scalar or (M,).
        Returns ``(dist (M,), solid_id (M,), normal (M, 3))``: ``dist`` is
        ``inf``, ``solid_id`` -1 and ``normal`` zero for rays that hit nothing
        within range; ``normal`` is the outward normal of the triangle hit.
        """
        O = np.atleast_2d(np.asarray(origins, dtype=np.float64))
        D = np.atleast_2d(np.asarray(dirs, dtype=np.float64))
        M = max(len(O), len(D))
        O = np.broadcast_to(O, (M, 3))
        D = np.broadcast_to(D, (M, 3))
        R = np.broadcast_to(np.asarray(max_range, dtype=np.float64), (M,))
        best_t = np.full(M, np.inf)
        best_tri = np.full(M, -1, dtype=np.int64)

        with np.errstate(divide="ignore", invalid="ignore"):
            inv = np.where(D != 0, 1.0 / np.where(D != 0, D, 1.0), np.inf)
            ta = (self.lo - O) * inv
            tb = (self.hi - O) * inv
            # Axis-parallel rays: the slab is all of t if the origin is inside it, else empty.
            par = D == 0
            inside = (O >= self.lo) & (O <= self.hi)
            ta = np.where(par, np.where(inside, -np.inf, np.inf), ta)
            tb = np.where(par, np.where(inside, np.inf, -np.inf), tb)
        t_near = np.minimum(ta, tb).max(axis=1)
        t_far = np.maximum(ta, tb).min(axis=1)
        t_start = np.maximum(t_near, 0.0)
        t_end = np.minimum(t_far, R)
        idx = np.flatnonzero(t_start <= t_end)
        if idx.size:
            cs, dims, lo = self.cell, self.dims, self.lo
            Oi, Di = O[idx], D[idx]
            P = Oi + Di * t_start[idx, None]
            c = np.clip(np.floor((P - lo) / cs).astype(np.int64), 0, dims - 1)
            st = np.sign(Di).astype(np.int64)
            with np.errstate(invalid="ignore"):
                bound = lo + (c + (st > 0)) * cs
                tm = np.where(st != 0, (bound - Oi) * inv[idx], np.inf)
                td = np.where(st != 0, cs * np.abs(inv[idx]), np.inf)
            te = t_end[idx]
            while idx.size:
                pr, pc = [], []
                t_next = None
                for _ in range(batch_cells):
                    pr.append(idx)
                    pc.append((c[:, 2] * dims[1] + c[:, 1]) * dims[0] + c[:, 0])
                    ax = np.argmin(tm, axis=1)
                    rows = np.arange(len(idx))
                    t_exit = tm[rows, ax]
                    c[rows, ax] += st[rows, ax]
                    tm[rows, ax] += td[rows, ax]
                    cax = c[rows, ax]
                    alive = (t_exit <= te) & (cax >= 0) & (cax < dims[ax])
                    idx, c, tm, td, st, te = idx[alive], c[alive], tm[alive], td[alive], st[alive], te[alive]
                    t_next = t_exit[alive]
                    if not idx.size:
                        break
                rays, tri = self._candidates(np.concatenate(pr), np.concatenate(pc))
                if rays.size:
                    th = self._moller_trumbore(O[rays], D[rays], tri, R[rays])
                    m = np.isfinite(th)
                    if m.any():
                        rays, tri, th = rays[m], tri[m], th[m]
                        np.minimum.at(best_t, rays, th)
                        win = th == best_t[rays]
                        best_tri[rays[win]] = tri[win]
                if idx.size:
                    keep = best_t[idx] > t_next
                    idx, c, tm, td, st, te = idx[keep], c[keep], tm[keep], td[keep], st[keep], te[keep]
        hit = best_tri >= 0
        sid = np.where(hit, self.tri_solid[np.maximum(best_tri, 0)], -1)
        normal = np.where(hit[:, None], self.normals[np.maximum(best_tri, 0)], 0.0)
        return best_t, sid, normal

    def closest(self, points: np.ndarray, radius: ArrayLike) -> Tuple[np.ndarray, np.ndarray, np.ndarray,
                                                                      np.ndarray, np.ndarray]:
        """Closest surface point within ``radius`` of each point.

        Returns ``(dist (P,), closest_point (P,3), solid_id (P,), normal (P,3),
        inside (P,))``.  Points with no triangle within ``radius`` get
        ``dist = inf``, NaN closest point, solid id -1, zero normal and
        ``inside = False``.  ``normal`` is the outward normal of the closest
        triangle; ``inside`` tells whether the point lies inside the solid that
        owns it (points on the surface count as outside).
        """
        Pts = np.atleast_2d(np.asarray(points, dtype=np.float64))
        n = len(Pts)
        rad = np.broadcast_to(np.asarray(radius, dtype=np.float64), (n,))
        dist = np.full(n, np.inf)
        cp = np.full((n, 3), np.nan)
        sid = np.full(n, -1, dtype=np.int64)
        nrm = np.zeros((n, 3))
        inside = np.zeros(n, dtype=bool)
        cs, dims, lo = self.cell, self.dims, self.lo
        reach = (Pts + rad[:, None] >= self.lo).all(1) & (Pts - rad[:, None] <= self.hi).all(1)
        q = np.flatnonzero(reach & (rad >= 0))
        if not q.size:
            return dist, cp, sid, nrm, inside
        clo = np.clip(np.floor((Pts[q] - rad[q, None] - lo) / cs).astype(np.int64), 0, dims - 1)
        chi = np.clip(np.floor((Pts[q] + rad[q, None] - lo) / cs).astype(np.int64), 0, dims - 1)
        box, cid = _expand_boxes(clo, chi, dims)
        pid, tri = self._candidates(q[box], cid)
        if not pid.size:
            return dist, cp, sid, nrm, inside
        key = np.unique(pid * len(self.triangles) + tri)
        pid, tri = key // len(self.triangles), key % len(self.triangles)
        c, feat = _closest_on_triangles(Pts[pid], self.triangles[tri])
        d = np.linalg.norm(Pts[pid] - c, axis=1)
        ok = d <= rad[pid]
        pid, tri, c, feat, d = pid[ok], tri[ok], c[ok], feat[ok], d[ok]
        if not pid.size:
            return dist, cp, sid, nrm, inside
        order = np.lexsort((d, pid))
        first = order[np.r_[True, pid[order][1:] != pid[order][:-1]]]
        p, t, f = pid[first], tri[first], feat[first]
        dist[p] = d[first]
        cp[p] = c[first]
        sid[p] = self.tri_solid[t]
        nrm[p] = self.normals[t]
        pseudo = self.normals[t].copy()
        for k in range(3):
            m = f == 1 + k  # vertex k
            pseudo[m] = self._vnorm[self._vid[t[m], k]]
            m = f == 4 + k  # edge k (corners k, k+1)
            pseudo[m] = self._enorm[self._eid[t[m], k]]
        inside[p] = (_dot(Pts[p] - c[first], pseudo) < 0) & (d[first] > 1e-12)
        return dist, cp, sid, nrm, inside

    def support_height(self, x: ArrayLike, y: ArrayLike, z_max: ArrayLike):
        """Highest surface strictly below ``z_max`` at (x, y): ``(z, solid_id)``.

        Scalars in give scalars out; ``z = -inf`` and ``solid_id = -1`` where
        there is nothing below.
        """
        scalar = np.ndim(x) == 0 and np.ndim(y) == 0 and np.ndim(z_max) == 0
        x, y, z = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float), np.asarray(z_max, float))
        shape = x.shape
        O = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)
        D = np.broadcast_to(np.array([0.0, 0.0, -1.0]), O.shape)
        rng = np.maximum(O[:, 2] - self.lo[2], 0.0) + 1.0
        t, s, _ = self.raycast(O, D, rng)
        zz = np.where(np.isfinite(t), O[:, 2] - t, -np.inf)
        if scalar:
            return float(zz[0]), int(s[0])
        return zz.reshape(shape), s.reshape(shape)


def _closest_on_triangles(p: np.ndarray, tri: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Closest point on each triangle to each point (paired), with the feature reached:
    0 = face interior, 1..3 = vertex 0..2, 4..6 = edge (0,1), (1,2), (2,0)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = _dot(ab, ap), _dot(ac, ap)
    bp = p - b
    d3, d4 = _dot(ab, bp), _dot(ac, bp)
    cp = p - c
    d5, d6 = _dot(ab, cp), _dot(ac, cp)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    out = np.empty_like(p)
    feat = np.full(len(p), -1, dtype=np.int64)

    def assign(mask, value, code):
        m = mask & (feat < 0)
        out[m] = value[m] if value.ndim == 2 else value
        feat[m] = code

    assign((d1 <= 0) & (d2 <= 0), a, 1)
    assign((d3 >= 0) & (d4 <= d3), b, 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        v_ab = d1 / (d1 - d3)
        assign((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + v_ab[:, None] * ab, 4)
        assign((d6 >= 0) & (d5 <= d6), c, 3)
        w_ac = d2 / (d2 - d6)
        assign((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + w_ac[:, None] * ac, 6)
        w_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        assign((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0), b + w_bc[:, None] * (c - b), 5)
        denom = va + vb + vc
        v = vb / denom
        w = vc / denom
        assign(np.ones(len(p), dtype=bool), a + v[:, None] * ab + w[:, None] * ac, 0)
    return out, feat
