"""Turn the planner's polyline into a smooth path with continuous curvature.

1. **Fillet** every corner with the widest circular arc that fits. An arc's
   tangent length may use at most half of a segment shared with another
   corner. Next to an opening it is limited further: up to the run-in before
   the entry, and up to the room past the far face after the exit. An arc may
   come no closer to an obstacle than ``min(comfort, what the polyline did
   there)``, the planner's own shortcut rule.
2. **Sample** that path (lines and arcs) at uniform arc length.
3. **Smooth** the samples with a triangular kernel ``smoothing_half_width_m``
   wide. Curvature then ramps instead of jumping, and straight runs stay
   exactly straight (a symmetric average of collinear points is on the line).
   The endpoints keep their place: fillets stay that far from them.
4. **Verify** the result conservatively: margin is 1-Lipschitz, so the
   smallest sampled margin minus half the largest sample gap bounds the
   margin everywhere on the curve. The bound must be at least 0, and at
   least ``min(comfort, the polyline's closest approach)`` less 1 cm. Any
   shortfall, or an opening the whole
   drone no longer clears, shrinks the nearest fillet (or the smoothing) and
   the path is rebuilt. The last resort is the planner's polyline, which is
   already verified exactly.

Corner sharpness is left to the speed profile: a tight curve just gets a
low speed limit.
"""

from __future__ import annotations

import dataclasses
import math
from typing import List, Optional, Tuple

import numpy as np

from .clearance import ObstacleField, conservative_min_margin
from .course_map import CourseMap
from .route import PlannerConfig, Route


class SmoothingError(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
class SmoothingConfig:
    #: No corner needs to be wider than this. At 1 m/s and 2 m/s^2 lateral, 0.5 m already suffices.
    max_radius_m: float = 1.0
    #: Arcs tighter than this become sharp corners (the speed profile slows right down there).
    min_radius_m: float = 0.03
    #: Factor a failing fillet's radius is multiplied by on each repair.
    radius_shrink: float = 0.7
    #: Half-width of the triangular smoothing kernel. Wider ramps curvature
    #: more gently (lower lateral jerk) but cuts corners slightly more.
    smoothing_half_width_m: float = 0.35
    #: Arc-length spacing of the output samples.
    sample_step_m: float = 0.01
    #: Enough to shrink several corners near one tight spot and then the kernel (the 2026-11-08 taped
    #: map needs 22: at 20 every corner fell back to sharp). A path that verifies stops the loop early.
    repair_iterations: int = 60
    #: A "via" waypoint with a neighbour closer than this is dropped when the
    #: straight line replacing it is clear and no riskier. Such stubs, left
    #: where a planner leg meets an opening's entry, would otherwise leave a
    #: near-sharp corner, since a fillet may use only half a segment.
    merge_short_segment_m: float = 0.25
    #: The smoothed path may come at most this much closer to an obstacle than
    #: ``min(comfort, the polyline's own closest approach)``; repairs enforce it.
    no_riskier_tolerance_m: float = 0.01


@dataclasses.dataclass
class SmoothPath:
    s: np.ndarray            # (N,) arc length, metres
    xy: np.ndarray           # (N, 2) course-frame (east, north)
    tangent: np.ndarray      # (N, 2) unit
    curvature: np.ndarray    # (N,) 1/m, left turns positive
    dcurvature: np.ndarray   # (N,) d(curvature)/ds, 1/m^2
    pass_name: List[str]     # per sample: the opening it is in, or ""
    pass_dir: np.ndarray     # (N, 2) direction of travel inside an opening, else 0
    pass_width: np.ndarray   # (N,) width of that opening, inf elsewhere
    radii: List[float]       # fillet radius at each route waypoint (0 at ends and sharp corners)
    smoothing_half_width_m: float
    min_margin_m: float      # proven lower bound over the whole curve
    min_clearance_m: float   # smallest sampled distance to an obstacle surface
    notes: List[str]
    worst_s: float = 0.0     # where the margin is smallest

    @property
    def length_m(self) -> float:
        return float(self.s[-1])


@dataclasses.dataclass(frozen=True)
class _Opening:
    name: str
    centre: np.ndarray
    axis: np.ndarray      # nominal axis, signed for the direction flown
    perp: np.ndarray
    half_depth: float
    half_width: float


def smooth_route(route: Route, cmap: CourseMap, field: ObstacleField,
                 planner: PlannerConfig = PlannerConfig(),
                 cfg: SmoothingConfig = SmoothingConfig()) -> SmoothPath:
    """A verified smooth path through the route's waypoints."""
    if len(route.waypoints) < 2:
        raise SmoothingError("a route needs at least two waypoints")
    route = _merge_stubs(route, field, planner, cfg.merge_short_segment_m)
    V = np.array([w.position for w in route.waypoints], dtype=float)
    openings = _openings(route, cmap)
    budgets = _budgets(route, planner, 0.0)
    radii = _initial_radii(V, budgets, field, planner, cfg)
    comfort = planner.comfort_clearance_m - planner.inscribed_radius_m
    polyline = min(field.segment_query(tuple(V[i]), tuple(V[i + 1]))[1] for i in range(len(V) - 1))
    floor = max(0.0, min(comfort, polyline) - cfg.no_riskier_tolerance_m)
    width = cfg.smoothing_half_width_m
    notes: List[str] = []
    for _ in range(cfg.repair_iterations):
        path, pieces = _build(V, radii, width, cfg, openings, field)
        problem = _problem(path, openings, planner, V, floor)
        if problem is None:
            path.notes = notes
            return path
        kind, where = problem
        k = None if kind.startswith("endpoint") else _nearest_corner(pieces, where, radii)
        if k is not None:
            radii[k] = radii[k] * cfg.radius_shrink
            if radii[k] < cfg.min_radius_m:
                radii[k] = 0.0
            notes.append(f"{kind} near s={where:.2f} m: corner {k} radius -> {radii[k]:.3f} m")
        elif width > 0:
            width = width / 2.0 if width > 0.02 else 0.0
            notes.append(f"{kind} near s={where:.2f} m: smoothing half-width -> {width:.3f} m")
        else:
            break
    radii = [0.0] * len(V)
    path, _ = _build(V, radii, 0.0, cfg, openings, field)
    problem = _problem(path, openings, planner, V, 0.0)
    if problem is not None:
        raise SmoothingError(f"even the planner's polyline fails the dense check: {problem[0]} "
                             f"near s={problem[1]:.2f} m")
    path.notes = notes + ["fell back to the planner's polyline"]
    return path


# -- route clean-up -------------------------------------------------------------------


def _merge_stubs(route: Route, field: ObstacleField, planner: PlannerConfig, min_len: float) -> Route:
    """Drop "via" waypoints next to a segment shorter than ``min_len`` when the
    straight replacement is clear (exact check) and keeps
    ``margin >= min(comfort, the margin of the two segments it replaces)``."""
    if min_len <= 0:
        return route
    comfort = planner.comfort_clearance_m - planner.inscribed_radius_m
    wps = list(route.waypoints)
    changed = True
    while changed:
        changed = False
        for i in range(1, len(wps) - 1):
            w = wps[i]
            if w.kind != "via" or wps[i - 1].straight_to_next:
                continue
            short = min(np.hypot(*np.subtract(w.position, wps[i - 1].position)),
                        np.hypot(*np.subtract(wps[i + 1].position, w.position)))
            if short >= min_len:
                continue
            floor = min(comfort, field.segment_query(wps[i - 1].position, w.position)[1],
                        field.segment_query(w.position, wps[i + 1].position)[1])
            if field.segment_query(wps[i - 1].position, wps[i + 1].position)[1] >= max(floor - 1e-9, 0.0):
                del wps[i]
                changed = True
                break
    return dataclasses.replace(route, waypoints=wps) if len(wps) != len(route.waypoints) else route


# -- corner budgets and radii ----------------------------------------------------


def _budgets(route: Route, planner: PlannerConfig, reserve: float) -> List[Tuple[float, float]]:
    """Per waypoint: the most tangent length a fillet may use (before, after)."""
    wps = route.waypoints
    V = np.array([w.position for w in wps], dtype=float)
    n = len(V) - 1
    L = np.hypot(*np.diff(V, axis=0).T)
    turns = [0.0] + [_turn(V[k - 1], V[k], V[k + 1]) for k in range(1, n)] + [0.0]
    out = [(0.0, 0.0)] * (n + 1)
    for k in range(1, n):
        before = _side(k - 1, k, k - 1, wps, L, turns, route, planner, reserve, n)
        after = _side(k, k, k + 1, wps, L, turns, route, planner, reserve, n)
        out[k] = (before, after)
    return out


def _side(seg: int, k: int, other: int, wps, L, turns, route: Route, planner: PlannerConfig,
          reserve: float, n: int) -> float:
    if wps[seg].straight_to_next:
        p = route.passes[wps[k].goal]
        cos_t = math.cos(math.radians(p.tilt_deg))
        if seg == k:   # k is an entry: keep the run-in straight
            return p.approach_m / cos_t
        return (planner.drone_extent_m + planner.crossing_margin_m) / cos_t  # k is an exit
    if other in (0, n):
        return max(0.0, L[seg] - reserve)
    if abs(turns[other]) > 1e-9:
        return L[seg] / 2.0
    return L[seg]


def _initial_radii(V: np.ndarray, budgets, field: ObstacleField, planner: PlannerConfig,
                   cfg: SmoothingConfig) -> List[float]:
    comfort = planner.comfort_clearance_m - planner.inscribed_radius_m
    radii = [0.0] * len(V)
    for k in range(1, len(V) - 1):
        theta = _turn(V[k - 1], V[k], V[k + 1])
        if abs(theta) < 1e-9:
            continue
        t_max = min(budgets[k])
        half = math.tan(abs(theta) / 2.0)
        if half <= 0 or t_max <= 0:
            continue
        r = min(cfg.max_radius_m, t_max / half)
        while r >= cfg.min_radius_m:
            arc, a, b = _arc(V[k - 1], V[k], V[k + 1], r, cfg.sample_step_m)
            # "No riskier": compare the arc with the polyline it replaces
            # (a -> corner -> b), both sampled the same way. The hard bound
            # (margin >= 0 everywhere) is the final conservative check's job.
            replaced = np.vstack([_samples(a, V[k], cfg.sample_step_m), _samples(V[k], b, cfg.sample_step_m)])
            floor = min(comfort, float(field.margin(replaced).min()))
            if float(field.margin(arc).min()) >= floor - 1e-3:
                break
            r *= cfg.radius_shrink
        radii[k] = r if r >= cfg.min_radius_m else 0.0
    return radii


def _samples(a, b, step: float) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return a * (1 - t) + b * t


def _turn(p0, p1, p2) -> float:
    a = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float)
    b = np.asarray(p2, dtype=float) - np.asarray(p1, dtype=float)
    return math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])


def _arc(p0, p1, p2, r: float, step: float):
    """Samples of the fillet of radius r at corner p1, and its tangent points."""
    p0, p1, p2 = (np.asarray(p, dtype=float) for p in (p0, p1, p2))
    a = (p1 - p0) / np.linalg.norm(p1 - p0)
    b = (p2 - p1) / np.linalg.norm(p2 - p1)
    theta = math.atan2(a[0] * b[1] - a[1] * b[0], a @ b)
    t = r * math.tan(abs(theta) / 2.0)
    start, end = p1 - a * t, p1 + b * t
    sign = 1.0 if theta > 0 else -1.0
    centre = start + sign * np.array([-a[1], a[0]]) * r
    phi0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    n = max(2, int(math.ceil(r * abs(theta) / step)) + 1)
    phi = phi0 + sign * np.linspace(0.0, abs(theta), n)
    return np.column_stack([centre[0] + r * np.cos(phi), centre[1] + r * np.sin(phi)]), start, end


# -- building the path -------------------------------------------------------------


def _build(V: np.ndarray, radii: List[float], width: float, cfg: SmoothingConfig,
           openings: List[_Opening], field: ObstacleField):
    pieces = []  # (s0, s1, kind, data, corner index or None)
    cur = V[0]
    s = 0.0
    for k in range(1, len(V) - 1):
        if radii[k] > 0:
            p0, p1, p2 = V[k - 1], V[k], V[k + 1]
            a = (p1 - p0) / np.linalg.norm(p1 - p0)
            b = (p2 - p1) / np.linalg.norm(p2 - p1)
            theta = math.atan2(a[0] * b[1] - a[1] * b[0], a @ b)
            r = radii[k]
            t = r * math.tan(abs(theta) / 2.0)
            start = p1 - a * t
            s = _line(pieces, s, cur, start)
            sign = 1.0 if theta > 0 else -1.0
            centre = start + sign * np.array([-a[1], a[0]]) * r
            phi0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
            length = r * abs(theta)
            pieces.append((s, s + length, "arc", (centre, r, phi0, sign), k))
            s += length
            cur = p1 + b * t
        else:
            s = _line(pieces, s, cur, V[k], corner=k)
            cur = V[k]
    s = _line(pieces, s, cur, V[-1])

    n = max(2, int(math.ceil(s / cfg.sample_step_m)))
    su = np.linspace(0.0, s, n + 1)
    pts = _evaluate(pieces, su)
    h = s / n
    if width > 0:
        pts = _triangular_smooth(pts, max(1, int(round(width / h))))
    d1 = np.gradient(pts, axis=0)
    d2 = np.gradient(d1, axis=0)
    speed = np.hypot(*d1.T)
    tangent = d1 / speed[:, None]
    curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / speed ** 3
    sarr = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    dcurv = np.gradient(curvature) / speed
    names, dirs, widths = _pass_mask(pts, openings)
    bound, worst = conservative_min_margin(field, pts)
    surface, _ = field.query(pts)
    path = SmoothPath(s=sarr, xy=pts, tangent=tangent, curvature=curvature, dcurvature=dcurv,
                      pass_name=names, pass_dir=dirs, pass_width=widths, radii=list(radii), smoothing_half_width_m=width,
                      min_margin_m=bound, min_clearance_m=float(surface.min()), notes=[],
                      worst_s=float(sarr[worst]))
    return path, pieces


def _line(pieces, s: float, a, b, corner: Optional[int] = None) -> float:
    length = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
    if length > 1e-12:
        pieces.append((s, s + length, "line", (np.asarray(a, dtype=float), np.asarray(b, dtype=float)), corner))
    return s + length


def _evaluate(pieces, su: np.ndarray) -> np.ndarray:
    out = np.empty((len(su), 2))
    idx = np.searchsorted([p[1] for p in pieces], su, side="left")
    idx = np.clip(idx, 0, len(pieces) - 1)
    for i, (s0, s1, kind, data, _) in enumerate(pieces):
        mask = idx == i
        if not mask.any():
            continue
        local = su[mask] - s0
        if kind == "line":
            a, b = data
            frac = np.clip(local / max(s1 - s0, 1e-12), 0.0, 1.0)[:, None]
            out[mask] = a * (1 - frac) + b * frac
        else:
            centre, r, phi0, sign = data
            phi = phi0 + sign * local / r
            out[mask] = np.column_stack([centre[0] + r * np.cos(phi), centre[1] + r * np.sin(phi)])
    return out


def _triangular_smooth(pts: np.ndarray, m: int) -> np.ndarray:
    """Symmetric triangular moving average, half-width m samples. Within m of
    either end the half-width tapers to the distance from that end, so the
    endpoints stay exactly where they are."""
    n = len(pts)
    m = min(m, (n - 1) // 2)
    if m < 1:
        return pts.copy()
    kern = (m + 1 - np.abs(np.arange(-m, m + 1))).astype(float)
    kern /= kern.sum()
    out = pts.copy()
    out[m:n - m] = np.column_stack([np.convolve(pts[:, i], kern, mode="valid") for i in range(2)])
    for i in list(range(1, m)) + list(range(n - m, n - 1)):
        q = min(i, n - 1 - i)
        k = (q + 1 - np.abs(np.arange(-q, q + 1))).astype(float)
        out[i] = (k / k.sum()) @ pts[i - q:i + q + 1]
    return out


# -- openings and checks -------------------------------------------------------------


def _openings(route: Route, cmap: CourseMap) -> List[_Opening]:
    goals = {g.name: g for g in cmap.goals}
    out = []
    for name, p in route.passes.items():
        g = goals[name]
        axis = np.asarray(g.axis, dtype=float)
        if float(np.dot(axis, p.direction)) < 0:
            axis = -axis
        out.append(_Opening(name, np.asarray(g.centre, dtype=float), axis, np.array([-axis[1], axis[0]]),
                            g.depth / 2.0, float(g.params["width"]) / 2.0))
    return out


def _pass_mask(pts: np.ndarray, openings: List[_Opening], reach: float = 0.7):
    names = [""] * len(pts)
    dirs = np.zeros_like(pts)
    widths = np.full(len(pts), np.inf)
    for o in openings:
        rel = pts - o.centre
        along = rel @ o.axis
        lateral = np.abs(rel @ o.perp)
        # The whole straight pass: run-in, opening, and the room past it.
        inside = (np.abs(along) <= o.half_depth + reach) & (lateral <= o.half_width)
        for i in np.flatnonzero(inside):
            names[i] = o.name
        dirs[inside] = o.axis
        widths[inside] = 2.0 * o.half_width
    return names, dirs, widths


def _problem(path: SmoothPath, openings: List[_Opening], planner: PlannerConfig,
             V: np.ndarray, floor: float) -> Optional[Tuple[str, float]]:
    """``(what, where along the path)`` for the first thing wrong, or None.
    ``floor`` is the least margin allowed anywhere (0 = only the hard bound)."""
    if np.linalg.norm(path.xy[0] - V[0]) > 1e-6 or np.linalg.norm(path.xy[-1] - V[-1]) > 1e-6:
        return "endpoint moved by smoothing", 0.0
    if path.min_margin_m < floor:
        return f"clearance (bound {path.min_margin_m:+.3f} m, floor {floor:+.3f} m)", path.worst_s
    for o in openings:
        rel = path.xy - o.centre
        along = rel @ o.axis
        lateral = np.abs(rel @ o.perp)
        crossings = np.flatnonzero((along[:-1] < 0) & (along[1:] >= 0))
        ok = False
        for i in crossings:
            if lateral[i] <= o.half_width - planner.drone_extent_m + 1e-9 \
                    and along[i:].max() >= o.half_depth + planner.drone_extent_m - 1e-9:
                ok = True
                break
        if not ok:
            where = float(path.s[int(np.argmin(np.abs(along)))])
            return f"whole drone no longer clears {o.name}", where
    return None


def _nearest_corner(pieces, where: float, radii: List[float]) -> Optional[int]:
    best, best_d = None, math.inf
    for s0, s1, kind, _, k in pieces:
        if k is None or radii[k] <= 0:
            continue
        d = 0.0 if s0 <= where <= s1 else min(abs(where - s0), abs(where - s1))
        if d < best_d:
            best, best_d = k, d
    return best if best_d < 1.0 else None
