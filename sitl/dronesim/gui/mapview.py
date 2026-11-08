"""Top-down course map on a Tk canvas: the STEP geometry plus live overlays."""
from __future__ import annotations

import math
import tkinter as tk

COLORS = {
    'floor': '#f4f1ea', 'wall': '#8d6e63', 'net': '#263238', 'tunnel': '#455a64',
    'tunnel_support': '#b0bec5', 'boulder': '#9e9e9e', 'pad': '#c8e6c9', 'marking': '#ffcc80',
    'other': '#cfd8dc',
}
DRAW_ORDER = ['floor', 'marking', 'pad', 'tunnel_support', 'boulder', 'wall', 'tunnel', 'net', 'other']


def hull(points):
    pts = sorted(set((round(p[0], 4), round(p[1], 4)) for p in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def course_shapes(course):
    """[(name, kind, polygon xy)] for drawing, from the STEP solids."""
    from ..world.course import classify
    shapes = []
    for s in course.solids:
        kind = classify(s.name)
        v = s.triangles.reshape(-1, 3)
        poly = hull(v[:, :2].tolist())
        shapes.append((s.name, kind, poly, float(s.bbox_max[2])))
    shapes.sort(key=lambda x: DRAW_ORDER.index(x[1]) if x[1] in DRAW_ORDER else 99)
    return shapes


class MapView:
    def __init__(self, parent, shapes, gates, rotors, prop_radius):
        self.canvas = tk.Canvas(parent, background='#fafafa', highlightthickness=0)
        self.shapes = shapes
        self.gates = gates
        self.rotors = rotors
        self.prop_r = prop_radius
        self.scale = 80.0
        self.cx, self.cy = 0.0, 0.0           # course point at the canvas centre
        self.trail = []
        self.items = {}
        self.show_lidar = tk.BooleanVar(value=True)
        self.show_est = tk.BooleanVar(value=True)
        self._drag = None
        self.canvas.bind('<Configure>', lambda e: self.redraw_static())
        self.canvas.bind('<MouseWheel>', self._wheel)
        self.canvas.bind('<ButtonPress-3>', self._press)
        self.canvas.bind('<B3-Motion>', self._motion)
        self.canvas.bind('<ButtonPress-2>', self._press)
        self.canvas.bind('<B2-Motion>', self._motion)
        self.canvas.bind('<Double-Button-1>', lambda e: self.fit())
        self._last = None

    # -- transforms -----------------------------------------------------------------------
    def w2c(self, x, y):
        w = self.canvas.winfo_width() or 600
        h = self.canvas.winfo_height() or 600
        return (w / 2 + (x - self.cx) * self.scale, h / 2 - (y - self.cy) * self.scale)

    def fit(self):
        w = self.canvas.winfo_width() or 600
        h = self.canvas.winfo_height() or 600
        self.scale = min(w, h) / 6.5
        self.cx = self.cy = 0.0
        self.redraw_static()

    def _wheel(self, e):
        f = 1.15 if e.delta > 0 else 1 / 1.15
        self.scale = max(20.0, min(800.0, self.scale * f))
        self.redraw_static()

    def _press(self, e):
        self._drag = (e.x, e.y, self.cx, self.cy)

    def _motion(self, e):
        if self._drag:
            x0, y0, cx, cy = self._drag
            self.cx = cx - (e.x - x0) / self.scale
            self.cy = cy + (e.y - y0) / self.scale
            self.redraw_static()

    # -- drawing ------------------------------------------------------------------------------------
    def redraw_static(self):
        c = self.canvas
        c.delete('all')
        self.items = {}
        for name, kind, poly, top in self.shapes:
            if len(poly) < 3:
                continue
            pts = [v for p in poly for v in self.w2c(*p)]
            fill = COLORS.get(kind, '#cfd8dc')
            if kind == 'net':
                c.create_polygon(*pts, fill='', outline=fill, width=2)
            elif kind == 'tunnel':
                c.create_polygon(*pts, fill=fill, outline='#263238', stipple='gray50', width=1)
                x, y = poly[0]
                cxp = sum(p[0] for p in poly) / len(poly)
                cyp = sum(p[1] for p in poly) / len(poly)
                c.create_text(*self.w2c(cxp, cyp), text='TUNNEL (dark)', fill='white', font=('Segoe UI', 8, 'bold'))
            else:
                c.create_polygon(*pts, fill=fill, outline='#5d4037' if kind == 'wall' else '', width=1)
        for g in self.gates:
            a, b = self.w2c(*g['a']), self.w2c(*g['b'])
            c.create_line(*a, *b, fill='#d81b60', width=2, dash=(6, 3))
            lab = f"CP{g['cp']}" + ('' if g['route'] == 'main' else f" {g['route']}")
            c.create_text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 - 9, text=lab, fill='#ad1457',
                          font=('Segoe UI', 8, 'bold'))
        # scale bar
        x0, y0 = 14, (self.canvas.winfo_height() or 600) - 14
        c.create_line(x0, y0, x0 + self.scale, y0, width=3, fill='#37474f')
        c.create_text(x0 + self.scale / 2, y0 - 9, text='1 m', fill='#37474f', font=('Segoe UI', 8))
        self.items['trail'] = c.create_line(0, 0, 0, 0, fill='#1e88e5', width=2)
        self.items['lidar'] = []
        self.items['drone'] = []
        self.items['est'] = []
        self._last = None
        if self.trail:
            self._draw_trail()

    def reset(self):
        self.trail = []
        self.redraw_static()

    def _draw_trail(self):
        if len(self.trail) >= 2:
            pts = [v for p in self.trail[-4000:] for v in self.w2c(*p)]
            self.canvas.coords(self.items['trail'], *pts)

    def update(self, snap):
        c = self.canvas
        tr = snap['true']
        x, y, yaw = tr['x'], tr['y'], tr['yaw']
        if not self.trail or math.dist(self.trail[-1], (x, y)) > 0.01:
            self.trail.append((x, y))
            self._draw_trail()
        for it in self.items.get('drone', []) + self.items.get('est', []) + self.items.get('lidar', []):
            c.delete(it)
        items = []
        cs, sn = math.cos(yaw), math.sin(yaw)

        def body(bx, by):              # FRD body xy -> course
            fx, fy = bx, -by
            return (x + cs * fx - sn * fy, y + sn * fx + cs * fy)
        r = self.prop_r * self.scale
        armed = snap['px4']['armed']
        for i, (px, py, _s) in enumerate(self.rotors):
            p = self.w2c(*body(px, py))
            col = '#43a047' if armed else '#9e9e9e'
            items.append(c.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r, outline=col, width=2))
        a, b = self.w2c(*body(self.rotors[0][0], self.rotors[0][1])), self.w2c(*body(self.rotors[1][0], self.rotors[1][1]))
        items.append(c.create_line(*a, *b, fill='#212121', width=3))
        a, b = self.w2c(*body(self.rotors[2][0], self.rotors[2][1])), self.w2c(*body(self.rotors[3][0], self.rotors[3][1]))
        items.append(c.create_line(*a, *b, fill='#212121', width=3))
        nose = self.w2c(*body(0.25, 0.0))
        ctr = self.w2c(x, y)
        items.append(c.create_line(*ctr, *nose, fill='#e53935', width=3, arrow='last'))
        self.items['drone'] = items
        est_items = []
        est = snap.get('est') or {}
        if self.show_est.get():
            def mark(p, col, label, shape='x'):
                if not p:
                    return
                q = self.w2c(*p)
                s = 6
                if shape == 'x':
                    est_items.append(c.create_line(q[0] - s, q[1] - s, q[0] + s, q[1] + s, fill=col, width=2))
                    est_items.append(c.create_line(q[0] - s, q[1] + s, q[0] + s, q[1] - s, fill=col, width=2))
                else:
                    est_items.append(c.create_oval(q[0] - s, q[1] - s, q[0] + s, q[1] + s, outline=col, width=2))
                est_items.append(c.create_text(q[0] + 9, q[1] - 9, text=label, fill=col, anchor='w',
                                               font=('Segoe UI', 8)))
            mark(est.get('ekf'), '#1565c0' if est.get('ekf_valid') else '#ef6c00',
                 'EKF2' if est.get('ekf_valid') else 'EKF2 (invalid)', 'o')
            mark(est.get('vo'), '#2e7d32', 'cuVSLAM', 'o')
            mark(est.get('setpoint'), '#c62828', 'setpoint')
        self.items['est'] = est_items
        lid_items = []
        lid = snap.get('lidar')
        if lid and self.show_lidar.get() and lid.get('scan'):
            for ang, rng in lid['scan']:
                if rng is None:
                    continue
                gx = x + rng * math.cos(yaw + ang)
                gy = y + rng * math.sin(yaw + ang)
                q = self.w2c(gx, gy)
                lid_items.append(c.create_rectangle(q[0] - 1, q[1] - 1, q[0] + 1, q[1] + 1, outline='#7b1fa2'))
        self.items['lidar'] = lid_items


class StripChart:
    """A few time series on a canvas (last ``window`` seconds)."""

    def __init__(self, parent, title, series, ylim=None, window=40.0, height=150):
        self.canvas = tk.Canvas(parent, height=height, background='white', highlightthickness=0)
        self.title = title
        self.series = series          # [(key, color, label, dash)]
        self.data = {k: [] for k, *_ in series}
        self.window = window
        self.ylim = ylim

    def clear(self):
        for k in self.data:
            self.data[k] = []

    def add(self, t, values):
        for k, v in values.items():
            if k in self.data:
                self.data[k].append((t, v))
                if len(self.data[k]) > 3000:
                    del self.data[k][:1000]

    def draw(self, t_now):
        c = self.canvas
        c.delete('all')
        w = c.winfo_width() or 400
        h = c.winfo_height() or 150
        left, right, top, bottom = 40, 8, 16, 18
        t0 = t_now - self.window
        vals = [v for k in self.data for (t, v) in self.data[k] if t >= t0 and v is not None and math.isfinite(v)]
        if self.ylim:
            lo, hi = self.ylim
        elif vals:
            lo, hi = min(vals), max(vals)
            if hi - lo < 0.2:
                mid = (hi + lo) / 2
                lo, hi = mid - 0.1, mid + 0.1
            pad = 0.1 * (hi - lo)
            lo, hi = lo - pad, hi + pad
        else:
            lo, hi = 0.0, 1.0
        c.create_text(left, 2, text=self.title, anchor='nw', font=('Segoe UI', 8, 'bold'), fill='#37474f')

        def tx(t):
            return left + (t - t0) / self.window * (w - left - right)

        def ty(v):
            return top + (hi - v) / (hi - lo) * (h - top - bottom)
        for k in range(5):
            v = lo + (hi - lo) * k / 4
            yy = ty(v)
            c.create_line(left, yy, w - right, yy, fill='#eceff1')
            c.create_text(left - 4, yy, text=f'{v:.2f}', anchor='e', font=('Segoe UI', 7), fill='#607d8b')
        c.create_text(w - right, h - 2, text=f't = {t_now:.1f} s', anchor='se', font=('Segoe UI', 7), fill='#607d8b')
        lx = left + 4
        for key, color, label, dash in self.series:
            pts = [(tx(t), ty(v)) for (t, v) in self.data[key] if t >= t0 and v is not None and math.isfinite(v)]
            if len(pts) >= 2:
                c.create_line(*[q for p in pts for q in p], fill=color, width=2 if not dash else 1,
                              dash=dash or None)
            c.create_text(lx + 90, 2, text=label, anchor='nw', fill=color, font=('Segoe UI', 8))
            lx += 8 + 7 * len(label)
