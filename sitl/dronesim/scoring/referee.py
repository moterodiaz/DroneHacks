"""Referee: DRONEHACKS.md section 4 scored against ground truth.

Gates come from ``sitl/course/course_v3_rules.json`` (``course-rules/1``, the
team's ground-truth checkpoint file) and are built from the STEP solids by name:

* ``segment``  -- an explicit line;
* ``between``  -- the clear gap between two solids (closest points of their
  footprints);
* ``opening``  -- the window of a wall (its jambs, sill and head from the mesh);
* ``tube_end`` -- the end face of the tube (``p0`` = the low-y end).

Rules applied (numbers from the rulebook digest):

* a checkpoint counts when the **whole drone** (prop-tip outline) has crossed its
  line, in order; CP4 by Gate B or by the window (the window adds 5 points to a
  finished run);
* a checkpoint reached after any part of the frame was above the 68 in wall tops
  since the previous one earns 15 % (rule 7.7);
* every touch of a wall, the tunnel or a boulder adds 5 s (rule 7.8);
* the clock starts when the drone leaves the launch mark and stops when it
  disarms with the whole drone inside the pad's outer ring (``inside_radius_m``);
* the run ends on a crash, ground contact outside the pad, leaving the net, or
  3 minutes (rule 7.9).
"""
from __future__ import annotations

import json
import math

WALL_TOP_M = 1.7272


def _hull(points):
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in points))
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


def _seg_closest(p1, q1, p2, q2):
    """Closest points between segments p1q1 and p2q2 (2D)."""
    d1 = (q1[0] - p1[0], q1[1] - p1[1])
    d2 = (q2[0] - p2[0], q2[1] - p2[1])
    r = (p1[0] - p2[0], p1[1] - p2[1])
    a = d1[0] ** 2 + d1[1] ** 2
    e = d2[0] ** 2 + d2[1] ** 2
    f = d2[0] * r[0] + d2[1] * r[1]
    if a < 1e-12 and e < 1e-12:
        return p1, p2
    if a < 1e-12:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = d1[0] * r[0] + d1[1] * r[1]
        if e < 1e-12:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = d1[0] * d2[0] + d1[1] * d2[1]
            den = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    c1 = (p1[0] + d1[0] * s, p1[1] + d1[1] * s)
    c2 = (p2[0] + d2[0] * t, p2[1] + d2[1] * t)
    return c1, c2


def _poly_gap(h1, h2):
    best = None
    for i in range(len(h1)):
        a, b = h1[i], h1[(i + 1) % len(h1)]
        for j in range(len(h2)):
            c, d = h2[j], h2[(j + 1) % len(h2)]
            p, q = _seg_closest(a, b, c, d)
            dd = math.dist(p, q)
            if best is None or dd < best[0]:
                best = (dd, p, q)
    return best[1], best[2]


def _seg_intersect(p, q, a, b):
    """Parameter t on p->q where it crosses segment a-b, or None."""
    r = (q[0] - p[0], q[1] - p[1])
    s = (b[0] - a[0], b[1] - a[1])
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-12:
        return None
    t = ((a[0] - p[0]) * s[1] - (a[1] - p[1]) * s[0]) / den
    u = ((a[0] - p[0]) * r[1] - (a[1] - p[1]) * r[0]) / den
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return t
    return None


class Gate:
    def __init__(self, a, b, z_lo=-math.inf, z_hi=math.inf, label=''):
        self.a, self.b = a, b
        self.z_lo, self.z_hi = z_lo, z_hi
        self.label = label
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        self.n = (-dy / n, dx / n)            # left normal of a->b

    def side(self, p):
        return (p[0] - self.a[0]) * self.n[0] + (p[1] - self.a[1]) * self.n[1]

    def as_dict(self):
        return {'a': list(self.a), 'b': list(self.b), 'z': [self.z_lo, self.z_hi], 'label': self.label}


class Referee:
    def __init__(self, course, rules_path, airframe_cfg, time_limit_s=180.0):
        self.course = course
        with open(rules_path, 'r', encoding='utf-8') as fh:
            self.rules = json.load(fh)
        self.cfg = airframe_cfg
        self.time_limit_s = time_limit_s
        self.outline = [(p[0], p[1]) for p in airframe_cfg.footprint_points(16)]   # body FRD xy
        self.checkpoints = []
        for cp in self.rules['checkpoints']:
            entry = {'id': cp['id'], 'name': cp['name'], 'points': cp['points'], 'gates': {}, 'land': cp.get('land', False),
                     'done_t': None, 'route': None, 'over_wall': False, 'state': None,
                     'centre_t': None, 'whole_t': None}
            if 'gate' in cp:
                entry['gates']['main'] = (self._gate(cp['gate']), 0)
            for rname, r in cp.get('routes', {}).items():
                entry['gates'][rname] = (self._gate(r['gate']), r.get('bonus_points', 0))
            self.checkpoints.append(entry)
        fin = self.rules['finish']
        self.pad_name = fin['marking']
        self.pad_centre = self.course.centre_xy(self.pad_name)
        self.pad_radius = float(fin['inside_radius_m'])
        self.launch_centre = self.course.centre_xy(self.rules['start']['marking'])
        ret = self.rules.get('return', {})
        self.return_within = float(ret.get('within_m', 0.6096))
        # run state
        self.t_start = None
        self.t_finish = None
        self.ended = None
        self.touches = []           # (t, solid name)
        self.net_touches = []
        self._last_touch_t = {}
        self._prev_c = None
        self._start_pos = None
        self.max_frame_low = 0.0
        self._over_since_cp = False
        self.next_cp = 0
        self.chain_broken = None
        self._was_armed = False
        self.violations = []
        self.return_state = None
        self.events = []

    # -- gates --------------------------------------------------------------------------------------
    def _gate(self, g):
        if 'segment' in g:
            (a, b) = g['segment']
            return Gate(tuple(a), tuple(b), label='segment')
        if 'between' in g:
            n1, n2 = g['between']
            h1 = _hull(self.course.footprint(n1)[:, :2].tolist())
            h2 = _hull(self.course.footprint(n2)[:, :2].tolist())
            p, q = _poly_gap(h1, h2)
            return Gate(p, q, label=f'gap {n1} / {n2}')
        if 'opening' in g:
            name = g['opening']
            v = self.course.footprint(name)
            inner = v[(v[:, 2] > 0.05) & (v[:, 2] < WALL_TOP_M - 0.05)]
            z_lo, z_hi = float(inner[:, 2].min()), float(inner[:, 2].max())
            # jambs: project the opening's vertices on the wall's long axis
            allxy = v[:, :2]
            h = _hull(allxy.tolist())
            # long axis = longest hull edge direction
            best = max(((h[i], h[(i + 1) % len(h)]) for i in range(len(h))), key=lambda e: math.dist(e[0], e[1]))
            ux, uy = best[1][0] - best[0][0], best[1][1] - best[0][1]
            ln = math.hypot(ux, uy)
            ux, uy = ux / ln, uy / ln
            cx, cy = float(allxy[:, 0].mean()), float(allxy[:, 1].mean())
            s = (inner[:, 0] - cx) * ux + (inner[:, 1] - cy) * uy
            a = (cx + ux * float(s.min()), cy + uy * float(s.min()))
            b = (cx + ux * float(s.max()), cy + uy * float(s.max()))
            return Gate(a, b, z_lo, z_hi, label=f'window of {name}')
        if 'tube_end' in g:
            s = self.course.solid(g['tube_end'])
            y = float(s.bbox_min[1]) if g.get('end', 'p0') == 'p0' else float(s.bbox_max[1])
            return Gate((float(s.bbox_min[0]), y), (float(s.bbox_max[0]), y), float(s.bbox_min[2]),
                        float(s.bbox_max[2]), label=f'{g["tube_end"]} end {g.get("end", "p0")}')
        raise ValueError(f'unknown gate kind {g}')

    def gates_for_display(self):
        out = []
        for cp in self.checkpoints:
            for rname, (gate, _b) in cp['gates'].items():
                d = gate.as_dict()
                d['cp'] = cp['id']
                d['route'] = rname
                out.append(d)
        return out

    # -- per-tick update ---------------------------------------------------------------------------------
    def update(self, t, pos_c, yaw_c, frame_low_z, armed, landed, contacts, feet, tilt_deg):
        """pos_c: drone centre in course frame; yaw_c: CCW from east; frame_low_z: lowest frame point."""
        if self.ended:
            return
        if self._start_pos is None:
            self._start_pos = pos_c
        # clock start: leave the launch mark
        if self.t_start is None:
            lifted = frame_low_z > 0.04
            moved = math.hypot(pos_c[0] - self._start_pos[0], pos_c[1] - self._start_pos[1]) > 0.15
            if armed and (lifted or moved):
                self.t_start = t
                self.events.append((t, 'clock started: left the launch mark'))
        # touches
        for c in contacts:
            if c.kind in ('wall', 'tunnel', 'tunnel_support', 'boulder'):
                last = self._last_touch_t.get(c.name)
                if last is None or t - last > 0.5:
                    self.touches.append((t, c.name))
                    self.events.append((t, f'touch: {c.name} (+5 s)'))
                self._last_touch_t[c.name] = t
                if c.speed_into > 2.0 or (c.part.startswith('prop') and c.speed_into > 1.2):
                    self._end(t, f'crash into {c.name} at {c.speed_into:.1f} m/s')
            elif c.kind == 'net':
                last = self._last_touch_t.get(c.name)
                if last is None or t - last > 0.5:
                    self.net_touches.append((t, c.name))
                    self.events.append((t, f'net contact: {c.name} (no rule penalty)'))
                self._last_touch_t[c.name] = t
            elif c.kind in ('floor', 'marking', 'pad') and self.t_start is not None and c.part != 'gear':
                self._end(t, f'crash: {c.part} hit the {c.kind}')
        if self.t_start is not None and armed is False and self._was_armed and self.t_finish is None and                 not self._inside_pad((pos_c[0], pos_c[1]), yaw_c):
            self._end(t, 'disarmed outside the pad (the run is over)')
        if armed:
            self._was_armed = True
        if self.t_start is not None and tilt_deg > 75:
            self._end(t, f'crash: vehicle tipped over ({tilt_deg:.0f} deg)')
        if self.t_start is not None and not self.ended:
            for f in feet:
                if f.kind in ('floor', 'marking') and t - self.t_start > 1.0 and not self._near_start(pos_c):
                    self._end(t, f'ground contact outside the pad ({f.name or f.kind})')
                    break
            if t - self.t_start > self.time_limit_s:
                self._end(t, '3-minute limit reached')
            if not self.course.inside_net(pos_c, margin=0.05):
                self._end(t, 'left the course')
        if frame_low_z > WALL_TOP_M:
            self._over_since_cp = True
        self.max_frame_low = max(self.max_frame_low, frame_low_z)
        # checkpoints: every gate is watched (progress), officially they count in order
        c_xy = (pos_c[0], pos_c[1])
        if self._prev_c is not None and not self.ended and self.t_start is not None:
            for cp in self.checkpoints:
                if cp['whole_t'] is not None:
                    continue
                if cp['land']:
                    if armed is False and landed and self._inside_pad(c_xy, yaw_c):
                        cp['whole_t'] = cp['centre_t'] = t
                        if self.t_finish is None:
                            self.t_finish = t
                            self.events.append((t, 'disarmed with the whole drone inside the pad: clock stopped'))
                    continue
                for rname, (gate, _bonus) in cp['gates'].items():
                    self._gate_step(t, cp, rname, gate, self._prev_c, c_xy, pos_c[2], yaw_c)
                    if cp['whole_t'] is not None:
                        break
            self._advance_official(t)
        self._prev_c = c_xy

    def _advance_official(self, t):
        cps = self.checkpoints
        while self.chain_broken is None and self.next_cp < len(cps):
            cp = cps[self.next_cp]
            later = [c for c in cps[self.next_cp + 1:] if c['whole_t'] is not None]
            if cp['whole_t'] is None:
                if later:
                    self.chain_broken = (t, cp['name'])
                    self.events.append((t, f"{cp['name']} skipped: scoring stops at the checkpoint before it "
                                           '(rule 7.6)'))
                return
            self._complete(cp['whole_t'], cp, cp['route'])

    def _near_start(self, pos_c):
        return math.hypot(pos_c[0] - self.launch_centre[0], pos_c[1] - self.launch_centre[1]) < 0.3

    def _outline_world(self, c_xy, yaw_c):
        # body FRD xy -> course: FRD x fwd, y right; course heading yaw_c CCW from east
        c, s = math.cos(yaw_c), math.sin(yaw_c)
        out = []
        for bx, by in self.outline:
            fx, fy = bx, -by                       # FLU
            out.append((c_xy[0] + c * fx - s * fy, c_xy[1] + s * fx + c * fy))
        return out

    def _inside_pad(self, c_xy, yaw_c):
        return all(math.dist(p, self.pad_centre) <= self.pad_radius for p in self._outline_world(c_xy, yaw_c))

    def _gate_step(self, t, cp, rname, gate, prev, cur, z, yaw_c):
        st = cp['state'] or {}
        cp['state'] = st
        key = rname
        tt = _seg_intersect(prev, cur, gate.a, gate.b)
        if tt is not None and gate.z_lo - 0.3 <= z <= gate.z_hi + 0.3:
            sign = 1.0 if gate.side(cur) > 0 else -1.0
            st[key] = sign
            if cp['centre_t'] is None:
                cp['centre_t'] = t
                self.events.append((t, f"{cp['name']}: centre crossed the line"
                                       + (f' ({rname})' if rname != 'main' else '')))
        sign = st.get(key)
        if sign is None:
            return
        pts = self._outline_world(cur, yaw_c)
        sides = [gate.side(p) * sign for p in pts]
        if min(sides) > 0:
            cp['whole_t'] = t
            cp['route'] = rname
        elif gate.side(cur) * sign < -0.05:
            st.pop(key, None)                      # backed out before the whole drone passed

    def _complete(self, t, cp, route):
        cp['done_t'] = t
        cp['route'] = route
        cp['over_wall'] = self._over_since_cp
        self._over_since_cp = False
        self.next_cp += 1
        self.events.append((t, f"{cp['name']} reached" + (f' via {route}' if route and route != 'main' else '')
                            + (' (over a wall: 15 %)' if cp['over_wall'] else '')))

    def _end(self, t, reason):
        if not self.ended:
            self.ended = (t, reason)
            self.events.append((t, 'RUN ENDED: ' + reason))

    def add_violation(self, t, text):
        if all(v[1] != text for v in self.violations):
            self.violations.append((t, text))
            self.events.append((t, 'RULE VIOLATION: ' + text))

    # -- results --------------------------------------------------------------------------------------------
    def summary(self):
        pts = 0.0
        rows = []
        for cp in self.checkpoints:
            got = cp['done_t'] is not None
            p = cp['points'] * (0.15 if cp['over_wall'] else 1.0) if got else 0.0
            pts += p
            rel = (lambda x: round(x - self.t_start, 2) if x is not None and self.t_start is not None else None)
            rows.append({'id': cp['id'], 'name': cp['name'], 'points': cp['points'], 'earned': round(p, 2),
                         'time_s': rel(cp['done_t']), 'route': cp['route'], 'over_wall': cp['over_wall'],
                         'centre_crossed_s': rel(cp['centre_t']), 'whole_drone_s': rel(cp['whole_t'])})
        finished = self.t_finish is not None
        window = any(cp['route'] == 'window' for cp in self.checkpoints)
        bonus = 5 if finished and window and not any(cp['over_wall'] for cp in self.checkpoints) else 0
        raw_time = None
        if self.t_start is not None:
            end_t = self.t_finish or (self.ended[0] if self.ended else None)
            last_cp = max((cp['done_t'] for cp in self.checkpoints if cp['done_t'] is not None), default=None)
            ref = end_t if finished else last_cp
            raw_time = (ref - self.t_start) if ref is not None else None
        penalty = 5.0 * len(self.touches)
        return {
            'checkpoints': rows, 'checkpoint_points': round(pts, 2), 'finished': finished,
            'window_route': window, 'window_bonus': bonus,
            'time_s': round(raw_time, 2) if raw_time is not None else None,
            'touches': len(self.touches), 'touch_penalty_s': penalty,
            'penalised_time_s': round(raw_time + penalty, 2) if raw_time is not None else None,
            'net_contacts': len(self.net_touches),
            'ended': {'t': round(self.ended[0], 2), 'reason': self.ended[1]} if self.ended else None,
            'violations': [{'t': round(t, 2), 'text': x} for t, x in self.violations],
            'max_frame_low_z': round(self.max_frame_low, 3),
            'chain_broken': ({'t': round(self.chain_broken[0], 2), 'at': self.chain_broken[1]}
                             if self.chain_broken else None),
            'note': 'run_points = C + 20*(t_best/t) + window_bonus for a finished run, C + 7*(t_best/t) '
                    'otherwise; t_best is the fastest team, unknown here.',
        }
