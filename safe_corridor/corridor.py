"""Safest smooth line through a CAD course, and the tube it flies in.

Inputs
  * a triangle mesh of the course (.npz from export_step.py, or example_course.py);
  * a route file: a start, an ordered list of openings (tunnel, window, ...), an end.
    Openings are named entries of a course map (contracts/course.md format) or given inline.

Method
  1. Rasterise the mesh into a 3 cm occupancy grid; Euclidean distance transform (EDT).
     The drone is an ellipsoid r_xy x r_xy x r_z, so z distances are scaled by r_xy / r_z:
     the EDT is then the room around the drone in "horizontal metres".
  2. Each opening's height band is measured from the mesh, straight up and down from its
     centre; the line crosses each opening at the middle of that band. A rough route
     around the walls comes from Dijkstra on a 6 cm grid (cost grows as room shrinks),
     with the band blending between openings.
  3. Each opening's frame is its outline across the route, measured from the mesh.
  4. The line is pulled taut and smooth between fixed points (start, each opening's
     lead-in, centre and lead-out, end). Walls push it away only where the drone's
     margin is below target_margin_m, so it never wanders into open space.
  5. The tube: along the line, each frame's outline blends linearly into the next, so its
     edges run straight from frame to frame; walls only trim it. Halfway between the
     tunnel and the window its bottom and top are the averages of the two frames'.

    py corridor.py routes/taped_2026-11-08.yaml --out out/taped
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import yaml
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

DEFAULTS = {
    'res_m': 0.03,
    'search_stride': 2,
    'drone_r_xy_m': 0.32,      # half the 0.50 x 0.40 m footprint's diagonal: any heading
    'drone_r_z_m': 0.12,       # half the drone's height, props included, plus a little
    'open_band_m': [0.30, 1.60],
    'c_ref_m': 0.5,
    'target_margin_m': 0.25,   # walls push the line only where the margin is below this
    'gate_lead_m': 0.35,       # straight run through each opening's centre, either side
    'open_frame_m': 0.40,      # radius of the round frame at the start and the end
    'pad_z_m': 0.12,           # the line's height on a pad: the drone resting (its half-height)
    'ring_rays': 72,
    'ring_max_m': 1.5,
}


# ---------------------------------------------------------------- inputs
def load_mesh(path):
    d = np.load(path)
    return d['triangles'].astype(np.float64), d['kinds'], d['names'], str(d['source'])


def load_route(path):
    with open(path, encoding='utf-8') as f:
        route = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(path))
    cfg = dict(DEFAULTS, **(route.get('config') or {}))
    goals, launch = {}, None
    if route.get('map'):
        with open(os.path.join(base, route['map']), encoding='utf-8') as f:
            m = yaml.safe_load(f)
        launch = m.get('launch', {}).get('position')
        for g in m.get('goals', []):
            goals[g['name']] = g

    def point(spec):
        if spec == 'launch':
            return [float(launch[0]), float(launch[1])]
        if isinstance(spec, str):
            return [float(c) for c in goals[spec]['centre']]
        return [float(c) for c in spec]

    gates = []
    for g in route['gates']:
        if isinstance(g, str):
            m = goals[g]
            p = m.get('params', {})
            if 'bottom' in p:
                seed = 0.5 * (p['bottom'] + p['top'])
            elif 'height' in p:
                seed = 0.5 * p['height']
            else:
                raise ValueError(f'gate {g}: no height in the map')
            gates.append({'name': g, 'centre': [*map(float, m['centre']), float(seed)]})
        else:
            gates.append({'name': g['name'], 'centre': [float(c) for c in g['centre']]})
    def radius(spec, key):
        """A pad's radius: the map's disc, else the route's <key>, else the default frame."""
        if isinstance(spec, str) and spec in goals and 'radius' in goals[spec].get('params', {}):
            return float(goals[spec]['params']['radius'])
        return float(route.get(key, cfg['open_frame_m']))

    return {'start': point(route['start']), 'end': point(route['end']), 'gates': gates,
            'start_r': radius(route['start'], 'start_radius'), 'end_r': radius(route['end'], 'end_radius'),
            'mesh': os.path.join(base, route['mesh']), 'name': route.get('name', ''),
            'map': route.get('map')}, cfg


# ---------------------------------------------------------------- grid
class Grid:
    def __init__(self, tris, kinds, cfg, z_top):
        self.res = h = cfg['res_m']
        v = tris.reshape(-1, 3)
        lo = v.min(0) - 0.1
        hi = v.max(0) + 0.1
        lo[2], hi[2] = -0.1, z_top
        self.lo = lo
        self.shape = tuple(int(math.ceil((hi[i] - lo[i]) / h)) + 1 for i in range(3))
        occ = np.zeros(self.shape, bool)
        keep = np.array([k not in ('marking',) for k in kinds])
        for t in tris[keep]:
            n = max(1, int(math.ceil(max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[1]),
                                         np.linalg.norm(t[0] - t[2])) / (0.5 * h))))
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing='ij')
            m = i + j <= n
            a, b = i[m] / n, j[m] / n
            p = t[0] + a[:, None] * (t[1] - t[0]) + b[:, None] * (t[2] - t[0])
            idx = np.round((p - lo) / h).astype(int)
            ok = np.all((idx >= 0) & (idx < self.shape), axis=1)
            occ[tuple(idx[ok].T)] = True
        occ[:, :, 0] = True                      # below the floor
        occ[0, :, :] = occ[-1, :, :] = occ[:, 0, :] = occ[:, -1, :] = True
        self.occ = occ
        self.k = cfg['drone_r_xy_m'] / cfg['drone_r_z_m']
        self.edt = ndimage.distance_transform_edt(~occ, sampling=(h, h, h)).astype(np.float32)
        self.edt_s = ndimage.distance_transform_edt(~occ, sampling=(h, h, h * self.k)).astype(np.float32)
        ax = [lo[i] + h * np.arange(self.shape[i]) for i in range(3)]
        self.X, self.Y, self.Z = np.meshgrid(*ax, indexing='ij', sparse=True)

    def idx(self, p):
        return (np.asarray(p, float) - self.lo) / self.res

    def sample(self, field, pts):
        return ndimage.map_coordinates(field, self.idx(pts).T, order=1, mode='nearest')

    def opening_band(self, c):
        """Free height at an opening's centre: march up and down from the seed height."""
        i, j, k = np.round(self.idx(c)).astype(int)
        col = self.occ[i, j]
        if col[k]:
            raise ValueError(f'opening seed {c} is inside the mesh')
        up = k + int(np.argmax(col[k:])) if col[k:].any() else len(col) - 1
        dn = k - int(np.argmax(col[k::-1])) if col[:k + 1].any() else 0
        return [float(self.lo[2] + (dn + 0.5) * self.res), float(self.lo[2] + (up - 0.5) * self.res)]


# ---------------------------------------------------------------- legs
class Leg:
    """Between two nodes: the band blends from a's to b's with progress d_a / (d_a + d_b)."""

    def __init__(self, grid, a, b, band_a, band_b):
        self.g, self.a, self.b = grid, np.array(a), np.array(b)
        self.band_a, self.band_b = np.array(band_a), np.array(band_b)
        g = grid
        da = np.sqrt((g.X - a[0]) ** 2 + (g.Y - a[1]) ** 2 + (g.Z - a[2]) ** 2)
        db = np.sqrt((g.X - b[0]) ** 2 + (g.Y - b[1]) ** 2 + (g.Z - b[2]) ** 2)
        f = da / np.maximum(da + db, 1e-9)
        self.lo = (band_a[0] + f * (band_b[0] - band_a[0])).astype(np.float32)
        self.hi = (band_a[1] + f * (band_b[1] - band_a[1])).astype(np.float32)
        self.c = np.minimum(g.edt_s, g.k * np.minimum(g.Z - self.lo, self.hi - g.Z)).astype(np.float32)

    def band_at(self, p):
        da = np.linalg.norm(p - self.a, axis=-1)
        db = np.linalg.norm(p - self.b, axis=-1)
        f = (da / np.maximum(da + db, 1e-9))[..., None]
        return self.band_a + f * (self.band_b - self.band_a)

    def search(self, cfg):
        g, s = self.g, cfg['search_stride']
        c = self.c[::s, ::s, ::s]
        h = g.res * s
        for need in np.arange(cfg['drone_r_xy_m'], 0.0, -0.02):
            ok = c > need
            ia = self._snap(self.a, ok, s)
            ib = self._snap(self.b, ok, s)
            if ia is None or ib is None:
                continue
            lab, _ = ndimage.label(ok, structure=np.ones((3, 3, 3)))
            if lab[ia] and lab[ia] == lab[ib]:
                break
        else:
            raise RuntimeError('no free path between the nodes')
        self.relaxed = float(need)
        ids = -np.ones(c.shape, np.int64)
        ids[ok] = np.arange(ok.sum())
        rows, cols, w = [], [], []
        offs = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                if (dx, dy, dz) > (0, 0, 0)]
        n = c.shape
        for dx, dy, dz in offs:
            sa = tuple(slice(max(0, -d), n[i] - max(0, d)) for i, d in enumerate((dx, dy, dz)))
            sb = tuple(slice(max(0, d), n[i] - max(0, -d)) for i, d in enumerate((dx, dy, dz)))
            m = ok[sa] & ok[sb]
            cm = 0.5 * (c[sa][m] + c[sb][m])
            ln = h * math.sqrt(dx * dx + dy * dy + dz * dz)
            rows.append(ids[sa][m]); cols.append(ids[sb][m])
            w.append(ln * (1.0 + (cfg['c_ref_m'] / cm) ** 2))
        N = int(ok.sum())
        G = coo_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(N, N)).tocsr()
        _, pred = dijkstra(G, directed=False, indices=ids[ia], return_predecessors=True)
        coords = np.argwhere(ok)
        path, j = [], ids[ib]
        while j >= 0:
            path.append(coords[j])
            j = pred[j]
        pts = g.lo + np.array(path[::-1], float) * h
        return np.vstack([self.a, pts[1:-1], self.b])

    def _snap(self, p, ok, s):
        i = np.round(self.g.idx(p) / s).astype(int)
        r = 4
        sl = tuple(slice(max(0, i[d] - r), i[d] + r + 1) for d in range(3))
        cand = np.argwhere(ok[sl])
        if len(cand) == 0:
            return None
        cand = cand + [x.start for x in sl]
        return tuple(cand[np.argmin(np.linalg.norm(cand - i, axis=1))])


def resample(P, step):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    s = np.linspace(0, d[-1], max(2, int(math.ceil(d[-1] / step)) + 1))
    return np.stack([np.interp(s, d, P[:, i]) for i in range(3)], axis=1)


def plane_basis(t):
    """Section plane at a path tangent: u horizontal (to the right), v up-ish."""
    t = t / np.linalg.norm(t)
    u = np.cross(t, [0, 0, 1.0])
    u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0, 0])
    return u, np.cross(u, t)


def rays(g, c, t, cfg, limit=None):
    """Free length of each ray of the section plane at c (tangent t), up to limit (per ray)."""
    th = np.linspace(0, 2 * math.pi, cfg['ring_rays'], endpoint=False)
    u, v = plane_basis(t)
    dirs = np.cos(th)[:, None] * u + np.sin(th)[:, None] * v
    r = np.arange(0.0, cfg['ring_max_m'], 0.01)
    free = g.sample(g.edt, (c + r[None, :, None] * dirs[:, None, :]).reshape(-1, 3)) > 0.6 * g.res
    free = free.reshape(len(th), len(r))
    out = r[np.where(free.all(1), len(r) - 1, np.argmin(free, axis=1))]
    return out if limit is None else np.minimum(out, limit), dirs


def snake(g, P, cfg, iters=1500):
    """Pull the line taut and smooth between fixed ends; walls push it away only where the
    drone's room drops below the target, so it never wanders into open space."""
    target = cfg['drone_r_xy_m'] + cfg['target_margin_m']
    gx, gy, gz = np.gradient(g.edt_s, g.res)
    for it in range(iters):
        if it % 100 == 0:
            P = resample(P, 0.05)
        t = np.gradient(P, axis=0)
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        c = g.sample(g.edt_s, P)
        gr = np.stack([g.sample(gx, P), g.sample(gy, P), g.sample(gz, P)], axis=1)
        gr -= (gr * t).sum(1, keepdims=True) * t
        n = np.linalg.norm(gr, axis=1, keepdims=True)
        push = np.clip(target - c, 0, None)[:, None] * gr / np.maximum(n, 1e-9)
        lap = np.zeros_like(P)
        lap[1:-1] = P[:-2] + P[2:] - 2 * P[1:-1]
        upd = 0.3 * lap + np.clip(0.5 * push, -0.01, 0.01)
        upd[0] = upd[-1] = 0
        P = P + upd
    return resample(P, 0.05)


# ---------------------------------------------------------------- main
def plan(route, cfg, tris, kinds):
    z_top = max(cfg['open_band_m'][1], *(gt['centre'][2] for gt in route['gates'])) + 1.0
    g = Grid(tris, kinds, cfg, z_top)
    open_band = list(cfg['open_band_m'])
    nodes = [{'name': 'start', 'p': [*route['start'], 0.5 * sum(open_band)], 'band': open_band}]
    for gt in route['gates']:
        band = g.opening_band(gt['centre'])
        nodes.append({'name': gt['name'], 'p': [gt['centre'][0], gt['centre'][1], 0.5 * sum(band)],
                      'band': band})
    nodes.append({'name': 'end', 'p': [*route['end'], 0.5 * sum(open_band)], 'band': open_band})

    # 1. a route around the walls: the max-clearance search, leg by leg
    rough = []
    for a, b in zip(nodes, nodes[1:]):
        leg = Leg(g, a['p'], b['p'], a['band'], b['band'])
        P = resample(leg.search(cfg), 0.05)
        rough.append(P)

    # 2. each opening's axis (the route's direction through it) and its frame: the opening's
    #    outline in the plane across that axis, measured from the mesh
    th = np.linspace(0, 2 * math.pi, cfg['ring_rays'], endpoint=False)
    frames = [None] * len(nodes)
    fixed = [np.array(nodes[0]['p'])]
    lead = cfg['gate_lead_m']
    for n in range(1, len(nodes) - 1):
        c = np.array(nodes[n]['p'])
        ax = rough[n][min(8, len(rough[n]) - 1)] - rough[n - 1][-min(9, len(rough[n - 1]))]
        ax /= np.linalg.norm(ax)
        frames[n], _ = rays(g, c, ax, cfg)
        fixed += [c - lead * ax, c, c + lead * ax]
    fixed.append(np.array(nodes[-1]['p']))

    # 3. the line: taut and smooth between the fixed points (start, each opening's lead-in,
    #    centre and lead-out, end), starting from the rough route (searched in order along it)
    allp = np.vstack(rough)
    segs, ia = [], 0
    for b in fixed[1:]:
        ib = ia + int(np.argmin(np.linalg.norm(allp[ia:] - b, axis=1)))
        init = np.vstack([allp[ia] if not segs else segs[-1][-1], allp[ia + 1:ib], b])
        segs.append(snake(g, init, cfg))
        ia = ib
    P = np.vstack([segs[0]] + [q[1:] for q in segs[1:]])

    # 3b. pads (when the route gives them): the line leaves the launch pad straight up and
    #     lands straight down, so the end sections lie flat on the pads; a cubic curve bends
    #     it smoothly into the line's own direction about one climb-height along
    pads = 'end_r' in route
    if pads:
        def lift(P):
            rise = P[0, 2] - cfg['pad_z_m']
            d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
            j = min(int(np.searchsorted(d, rise)), len(P) - 2)
            tc = (P[j + 1] - P[j - 1]) / np.linalg.norm(P[j + 1] - P[j - 1])
            a = np.array([P[0, 0], P[0, 1], cfg['pad_z_m']])
            ctl = [a, a + [0, 0, 0.6 * rise], P[j] - 0.6 * rise * tc, P[j]]
            u = np.linspace(0, 1, 30)[:, None]
            curve = sum(w * c for w, c in zip(((1 - u) ** 3, 3 * u * (1 - u) ** 2, 3 * u ** 2 * (1 - u), u ** 3), ctl))
            return np.vstack([resample(curve, 0.05), P[j + 1:]])
        P = lift(lift(P)[::-1])[::-1]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    node_s = [float(s[np.argmin(np.linalg.norm(P - np.array(nd['p']), axis=1))]) for nd in nodes]

    # 4. sections: each frame's outline blends linearly into the next along the line, so every
    #    edge (top, bottom, sides) runs straight from frame to frame; halfway, the top is the
    #    average of the two frames' tops. Walls only trim the sections. The end frames are the
    #    pads' discs (or the default round frame without pads).
    gates = range(1, len(nodes) - 1)
    fs = [0.0] + [node_s[n] for n in gates] + [float(s[-1])]
    fr = ([np.full(len(th), route['start_r'] if pads else cfg['open_frame_m'])] + [frames[n] for n in gates]
          + [np.full(len(th), route['end_r'] if pads else cfg['open_frame_m'])])
    T = np.gradient(P, axis=0)
    rings, path, legs_out = [], [], []
    c_all = g.sample(g.edt_s, P)
    d_all = g.sample(g.edt, P)
    for i, p in enumerate(P):
        k = min(int(np.searchsorted(fs, s[i], side='right')) - 1, len(fs) - 2)
        f = (s[i] - fs[k]) / max(fs[k + 1] - fs[k], 1e-9)
        want = (1 - f) * fr[k] + f * fr[k + 1]
        radii, dirs = rays(g, p, T[i], cfg, want)
        rings.append(p + radii[:, None] * dirs)
        lo, hi = (p + radii[:, None] * dirs)[:, 2].min(), (p + radii[:, None] * dirs)[:, 2].max()
        leg = int(np.clip(np.searchsorted(node_s, s[i], side='right') - 1, 0, len(nodes) - 2))
        path.append({'p': p.tolist(), 'margin': float(c_all[i] - cfg['drone_r_xy_m']),
                     'dist': float(d_all[i]), 'band': [float(lo), float(hi)], 'leg': leg})
    for k, (a, b) in enumerate(zip(nodes, nodes[1:])):
        m = (s >= node_s[k]) & (s <= node_s[k + 1])
        j = np.flatnonzero(m)[np.argmin(c_all[m])]
        legs_out.append({'from': a['name'], 'to': b['name'], 'length_m': node_s[k + 1] - node_s[k],
                         'min_margin_m': float(c_all[j] - cfg['drone_r_xy_m']), 'at': P[j].tolist()})
    frames_out = [{'name': nodes[n]['name'], 'pts': rings[int(np.argmin(np.abs(s - node_s[n])))]}
                  for n in gates]
    if pads:  # the pads are frames too; their markers move down onto them
        frames_out = ([{'name': 'start', 'pts': rings[0], 'pad': True}] + frames_out
                      + [{'name': 'end', 'pts': rings[-1], 'pad': True}])
        for nd, q, r in ((nodes[0], P[0], route['start_r']), (nodes[-1], P[-1], route['end_r'])):
            nd['p'], nd['pad_r'] = q.tolist(), r
    return g, nodes, path, rings, legs_out, frames_out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('route')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    route, cfg = load_route(a.route)
    tris, kinds, names, source = load_mesh(route['mesh'])
    g, nodes, path, rings, legs, frames = plan(route, cfg, tris, kinds)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out + '_path.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['idx', 'x_m', 'y_m', 'z_m', 'margin_m', 'dist_to_surface_m', 'band_lo_m', 'band_hi_m'])
        for i, q in enumerate(path):
            w.writerow([i, *(f'{v:.3f}' for v in q['p']), f"{q['margin']:.3f}", f"{q['dist']:.3f}",
                        *(f'{v:.3f}' for v in q['band'])])
    show = [i for i, k in enumerate(kinds) if k not in ('floor', 'net')]
    view = {
        'name': route['name'], 'source': source, 'config': cfg,
        'nodes': [{'name': n['name'], 'p': [round(v, 3) for v in n['p']], 'band': [round(v, 3) for v in n['band']],
                   **({'pad_r': round(n['pad_r'], 3)} if 'pad_r' in n else {})} for n in nodes],
        'legs': legs,
        'path': [[*(round(v, 3) for v in q['p']), round(q['margin'], 3), round(q['dist'], 3)] for q in path],
        'bands': [[round(v, 3) for v in q['band']] for q in path],
        'rings': [np.round(r, 3).tolist() for r in rings],
        'frames': [{'name': f['name'], 'pts': np.round(f['pts'], 3).tolist()} for f in frames],
        'tris': np.round(tris[show], 3).reshape(len(show), 9).tolist(),
        'tri_kind': [str(kinds[i]) for i in show],
        'bounds': [np.round(g.lo, 3).tolist(), np.round(g.lo + g.res * (np.array(g.shape) - 1), 3).tolist()],
    }
    with open(a.out + '_view.json', 'w') as f:
        json.dump(view, f, separators=(',', ':'))
    print(f"{route['name']}: {len(path)} points, {sum(l['length_m'] for l in legs):.2f} m")
    for n in nodes:
        print(f"  node {n['name']:<8} at ({n['p'][0]:+.2f}, {n['p'][1]:+.2f}, {n['p'][2]:.2f})  band {n['band'][0]:.2f}-{n['band'][1]:.2f} m")
    for l in legs:
        print(f"  {l['from']:>7} -> {l['to']:<7} {l['length_m']:5.2f} m  min margin {l['min_margin_m']*100:5.1f} cm "
              f"at ({l['at'][0]:+.2f}, {l['at'][1]:+.2f}, {l['at'][2]:.2f})")


if __name__ == '__main__':
    main()
