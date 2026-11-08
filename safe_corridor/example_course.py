"""An alternative course example: a different layout from ours, to show the planner is general.

6 x 6 m netted room, 2.2 m walls. A low oval tunnel (0.15-0.95 m) through a dividing wall,
then a high window (1.30-1.90 m) in a second wall, then the pad. A pillar near the start.
Writes courses/example.npz (same fields as export_step.py) and routes/example.yaml.

    py example_course.py
"""
import math
import os

import numpy as np

H = 2.2          # wall height
T = 0.10         # wall thickness


def box(x0, y0, z0, x1, y1, z1):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [v[list(t)] for t in f]


def tube(cx, cy, zc, length, half_w, half_h, axis='y', n=48):
    """Oval shell along y: inner half-axes (half_w, half_h), wall T, open ends."""
    th = np.linspace(0, 2 * math.pi, n + 1)
    tris = []
    for r_w, r_h in ((half_w, half_h), (half_w + T, half_h + T)):
        ring = [(cx + r_w * math.cos(a), zc + r_h * math.sin(a)) for a in th]
        for (x0, z0), (x1, z1) in zip(ring, ring[1:]):
            a = [x0, cy - length / 2, z0]; b = [x1, cy - length / 2, z1]
            c = [x1, cy + length / 2, z1]; d = [x0, cy + length / 2, z0]
            tris += [np.array([a, b, c]), np.array([a, c, d])]
    for y in (cy - length / 2, cy + length / 2):          # end annuli
        for a0, a1 in zip(th, th[1:]):
            p = [[cx + half_w * math.cos(a), y, zc + half_h * math.sin(a)] for a in (a0, a1)]
            q = [[cx + (half_w + T) * math.cos(a), y, zc + (half_h + T) * math.sin(a)] for a in (a0, a1)]
            tris += [np.array([p[0], p[1], q[1]]), np.array([p[0], q[1], q[0]])]
    return tris


def main():
    parts = []                                            # (name, kind, triangles)
    parts.append(('Floor', 'floor', box(-3.0, -3.0, -0.02, 3.0, 3.0, 0.0)))
    for name, b in (('Net west', (-3.05, -3.0, 0, -3.0, 3.0, 3.0)), ('Net east', (3.0, -3.0, 0, 3.05, 3.0, 3.0)),
                    ('Net south', (-3.0, -3.05, 0, 3.0, -3.0, 3.0)), ('Net north', (-3.0, 3.0, 0, 3.0, 3.05, 3.0))):
        parts.append((name, 'net', box(*b)))
    # tunnel through divider D1 (y = 0.5): inner 1.0 wide x 0.8 tall, bottom 0.15 m
    tx, tz, hw, hh, tl = 1.8, 0.55, 0.5, 0.4, 1.6
    parts.append(('Tunnel', 'tunnel', tube(tx, 0.5, tz, tl, hw, hh)))
    ow = hw + T
    parts.append(('D1 west', 'wall', box(-3.0, 0.45, 0, tx - ow, 0.55, H)))
    parts.append(('D1 east', 'wall', box(tx + ow, 0.45, 0, 3.0, 0.55, H)))
    parts.append(('D1 over tunnel', 'wall', box(tx - ow, 0.45, tz + hh + T, tx + ow, 0.55, H)))
    # divider D2 (x = -0.5) from D1 to the south net, window 0.9 wide at 1.30-1.90 m, centre y = -1.5
    wx, wy, wlo, whi, ww = -0.5, -1.5, 1.30, 1.90, 0.9
    parts.append(('D2 north', 'wall', box(wx - T / 2, wy + ww / 2, 0, wx + T / 2, 0.45, H)))
    parts.append(('D2 south', 'wall', box(wx - T / 2, -3.0, 0, wx + T / 2, wy - ww / 2, H)))
    parts.append(('D2 below window', 'wall', box(wx - T / 2, wy - ww / 2, 0, wx + T / 2, wy + ww / 2, wlo)))
    parts.append(('D2 above window', 'wall', box(wx - T / 2, wy - ww / 2, whi, wx + T / 2, wy + ww / 2, H)))
    # a pillar the path must go round, and a short wall making a corridor in the start room
    parts.append(('Pillar', 'wall', box(-0.6, 1.4, 0, -0.2, 1.8, H)))
    parts.append(('Start wall', 'wall', box(-1.6, 2.2, 0, -1.5, 3.0, H)))
    parts.append(('Pad', 'pad', box(-2.6, -2.6, 0, -1.6, -1.6, 0.005)))

    tris, names, kinds = [], [], []
    for name, kind, t in parts:
        tris += t
        names += [name] * len(t)
        kinds += [kind] * len(t)
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, 'courses', 'example.npz')
    np.savez_compressed(out, triangles=np.array(tris, np.float32), names=np.array(names), kinds=np.array(kinds),
                        source=np.array('example_course.py (alternative course example)'))
    with open(os.path.join(here, 'routes', 'example.yaml'), 'w') as f:
        f.write(f"""# Alternative course example (example_course.py): start -> low tunnel -> high window -> pad.
name: Alternative course example
mesh: ../courses/example.npz
start: [-2.4, 2.4]
gates:
  - {{name: tunnel, centre: [{tx}, 0.5, {tz}]}}
  - {{name: window, centre: [{wx}, {wy}, {(wlo + whi) / 2}]}}
end: [-2.1, -2.1]
end_radius: 0.5
config:
  open_band_m: [0.30, 1.90]
""")
    print(f'{out}: {len(tris)} triangles')


if __name__ == '__main__':
    main()
