"""Plan a full run: takeoff, the course through every checkpoint, landing, disarm, and the return.

:func:`plan_course` takes a :class:`~course.course_map.CourseMap`, a rules dict (checkpoints as
gates, start and landing markings) and a :class:`PlanConfig`, and returns a :class:`Plan` with the
timed setpoints, checkpoint times, and the room the drone has at every gate and along the path.
:func:`write_outputs` saves it as CSV setpoints (course ENU and PX4 local NED), a JSON summary
and a picture.

Times follow the organisers' clock: zero when the drone leaves the launch mark (taken as
liftoff), the finish at disarm on the pad.  The return is timed separately; it only has to fit in
the 3 minute run limit.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from course.course_map import CourseMap
from course.geometry import minkowski_sum, signed_distance
from course.planner.cspace import CSpace, DroneSpec, build_cspace
from course.planner.route import Gate, PlanError, free_interval, resolve_gate, route_through, shortcut
from course.planner.smooth import resample_sections, smooth_path
from course.planner.speed import Samples, profile_1d, sample, signed_curvature, speed_profile, time_stamps


@dataclass
class PlanConfig:
    drone: DroneSpec = field(default_factory=DroneSpec)
    cruise_z_m: float = 0.80          # body centre height for the whole course
    yaw_deg: float = 90.0             # fixed heading, ENU (0 = nose east, 90 = nose north)
    grid_m: float = 0.02
    spacing_m: float = 0.04
    smoothness: Tuple[float, ...] = (0.003, 0.01, 0.03, 0.1, 0.3, 1.0)
    clearance_m: float = 0.01         # room the smoother keeps on top of the drone's margin
    extra_margin_m: Dict[str, float] = field(default_factory=dict)  # per feature, on top of the rules file's
    iterations: int = 400
    v_max: float = 2.5                # m/s, horizontal
    a_max: float = 3.0                # m/s^2, total horizontal acceleration the drone can track
    climb_v: float = 1.0
    vertical_a: float = 2.0
    descend_v: float = 1.0
    touchdown_v: float = 0.3
    touchdown_last_m: float = 0.25
    disarm_delay_s: float = 0.5       # touchdown -> disarm (land detector or a forced disarm)
    rocks_full_height: bool = True    # keep out of the boulders' footprint even when above them
    return_z_m: float = 2.3
    return_v: float = 1.0
    return_a: float = 1.5
    rearm_delay_s: float = 2.0        # disarm on the pad -> liftoff for the return
    dt: float = 0.02                  # setpoint period (50 Hz)

    @property
    def rest_z_m(self) -> float:
        """Body centre height when sitting on the floor (z in the setpoints is the body centre)."""
        return self.drone.height_m / 2.0


@dataclass
class Plan:
    route: str
    config: PlanConfig
    cspace: CSpace
    gates: List[Gate]
    grid_route: np.ndarray
    path: np.ndarray            # (N, 2) course path at cruise height
    speed: np.ndarray           # (N,)
    path_times: np.ndarray      # (N,), course-relative
    curvature: np.ndarray       # (N,)
    gate_index: List[int]
    smoothness: float
    candidates: List[Dict[str, Any]]
    samples: Samples            # the timed run, liftoff to touchdown
    phases: List[Tuple[str, float, float]]
    checkpoints: List[Dict[str, Any]]
    finish_s: float
    return_samples: Optional[Samples]
    return_s: Optional[float]
    clearance: List[Dict[str, Any]]
    start: np.ndarray
    goal: np.ndarray
    notes: List[str]
    return_phases: List[Tuple[str, float, float]] = field(default_factory=list)


def _marking_centre(cmap: CourseMap, name: str) -> np.ndarray:
    f = cmap.feature(name)
    if f["kind"] != "marking":
        raise PlanError("{!r} is a {}, not a floor marking".format(name, f["kind"]))
    return np.array(f["centre"], dtype=float)


def column_clear(cmap: CourseMap, drone: DroneSpec, yaw: float, xy: np.ndarray, z_top: float) -> Optional[str]:
    """Name of the first obstacle a vertical climb from the floor to ``z_top`` at ``xy`` would hit."""
    lo, hi = drone.band(z_top)
    footprint = drone.footprint(yaw)
    for o in cmap.obstacles_2d(0.0, hi, rocks_full_height=False):
        d, _ = signed_distance(xy[None], [minkowski_sum(o.polygon, footprint)])
        if d[0] < 0.0:
            return o.name
    return None


def segment_clear(cmap: CourseMap, drone: DroneSpec, yaw: float, a: np.ndarray, b: np.ndarray, z: float) -> Optional[str]:
    """Name of the first obstacle a level straight flight a -> b at height ``z`` would hit."""
    footprint = drone.footprint(yaw)
    pts = a[None] + np.linspace(0.0, 1.0, 200)[:, None] * (b - a)[None]
    for o in cmap.obstacles_2d(*drone.band(z), rocks_full_height=False):
        d, _ = signed_distance(pts, [minkowski_sum(o.polygon, footprint)])
        if (d < 0.0).any():
            return o.name
    return None


def _densify(p: np.ndarray, k: int = 4) -> np.ndarray:
    t = np.linspace(0.0, 1.0, k, endpoint=False)
    mids = p[:-1, None, :] + t[None, :, None] * (p[1:] - p[:-1])[:, None, :]
    return np.concatenate([mids.reshape(-1, 2), p[-1:]], axis=0)


def nearest_obstacle(cs: CSpace, point: np.ndarray) -> str:
    best, name = math.inf, "boundary"
    for poly, o in zip(cs.polygons, cs.obstacles):
        d, _ = signed_distance(point[None], [poly])
        if d[0] < best:
            best, name = d[0], o.name
    return name


def plan_course(cmap: CourseMap, rules: Dict[str, Any], route: str, config: PlanConfig) -> Plan:
    yaw = math.radians(config.yaw_deg)
    drone = config.drone
    notes: List[str] = []
    extra = dict(rules.get("extra_margin_m", {}))
    extra.update(config.extra_margin_m)
    config = replace(config, extra_margin_m=extra)
    cs = build_cspace(cmap, drone, yaw, config.cruise_z_m, config.grid_m, config.rocks_full_height, extra)
    band = drone.band(config.cruise_z_m)

    start = _marking_centre(cmap, rules["start"]["marking"])
    goal = _marking_centre(cmap, rules["finish"]["marking"])
    for label, xy in (("launch mark", start), ("landing pad", goal)):
        d, _ = cs.distance(xy)
        if d[0] < 0.0:
            raise PlanError("the {} at ({:.2f}, {:.2f}) is too close to {} for this drone".format(
                label, *xy, nearest_obstacle(cs, xy)))
        hit = column_clear(cmap, drone, yaw, xy, config.cruise_z_m)
        if hit:
            raise PlanError("climbing at the {} would hit {}".format(label, hit))

    gates: List[Gate] = []
    gate_meta: List[Dict[str, Any]] = []
    for cp in rules["checkpoints"]:
        if cp.get("land"):
            continue
        if "routes" in cp:
            if route not in cp["routes"]:
                raise PlanError("checkpoint {} has no route {!r} (have {})".format(cp["id"], route, list(cp["routes"])))
            spec = cp["routes"][route]
        else:
            spec = cp
        gate = free_interval(resolve_gate(spec["gate"], cp["name"], cmap, band), cs)
        gates.append(gate)
        gate_meta.append({"id": cp["id"], "name": cp["name"], "points": cp.get("points", 0),
                          "bonus_points": spec.get("bonus_points", 0)})

    grid_route, anchors = route_through(cs, start, gates, goal)
    short, anchors = shortcut(cs, grid_route, anchors)
    p0, anchors = resample_sections(short, anchors, config.spacing_m)

    candidates: List[Dict[str, Any]] = []
    best: Optional[Tuple[float, np.ndarray, float]] = None
    for lam in config.smoothness:
        p = smooth_path(p0, anchors, gates, cs, config.spacing_m, lam, config.clearance_m, config.iterations)
        d, _ = cs.distance(_densify(p))
        kappa = signed_curvature(p)
        v = speed_profile(p, config.a_max, np.full(len(p), config.v_max), kappa)
        t = time_stamps(p, v)
        ok = bool(d.min() > -2e-3)
        candidates.append({"smoothness": lam, "course_s": round(float(t[-1]), 3),
                           "length_m": round(float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()), 3),
                           "min_room_m": round(float(d.min()), 4), "feasible": ok})
        if ok and (best is None or t[-1] < best[0]):
            best = (float(t[-1]), p, lam)
    if best is None:
        raise PlanError("every smoothed path touches an obstacle; try --grid 0.01 or more --iterations")
    _, path, lam = best
    kappa = signed_curvature(path)
    v = speed_profile(path, config.a_max, np.full(len(path), config.v_max), kappa)
    t = time_stamps(path, v)

    rest, cruise, dt = config.rest_z_m, config.cruise_z_m, config.dt
    up, t_up = profile_1d(np.r_[start, rest], np.r_[start, cruise], config.climb_v, config.vertical_a, dt)
    path3 = np.column_stack([path, np.full(len(path), cruise)])
    course = sample(path3, v, t, dt, kappa, t0=t_up)
    t_course_end = t_up + float(t[-1])
    down, t_down = profile_1d(np.r_[goal, cruise], np.r_[goal, rest], config.descend_v, config.vertical_a, dt,
                              v_slow=config.touchdown_v, slow_last_m=config.touchdown_last_m, t0=t_course_end)
    touchdown = t_course_end + t_down
    finish = touchdown + config.disarm_delay_s

    samples = _concat([up, course, down])
    phases = [("takeoff", 0.0, t_up), ("course", t_up, t_course_end), ("landing", t_course_end, touchdown)]

    checkpoints = []
    prev = 0
    for meta, gate, idx in zip(gate_meta, gates, anchors[1:-1]):
        checkpoints.append(dict(meta, time_s=round(t_up + _first_crossing(path, t, prev, idx, gate), 3)))
        prev = idx
    finish_cp = [cp for cp in rules["checkpoints"] if cp.get("land")]
    if finish_cp:
        checkpoints.append({"id": finish_cp[0]["id"], "name": finish_cp[0]["name"],
                            "points": finish_cp[0].get("points", 0), "bonus_points": 0, "time_s": round(finish, 3)})

    clearance = []
    names = ["launch"] + [g.name for g in gates] + ["pad"]
    for (s0, s1), a_name, b_name in zip(zip(anchors[:-1], anchors[1:]), names[:-1], names[1:]):
        section = path[s0:s1 + 1]
        d, _ = cs.distance(_densify(section))
        i = int(np.argmin(d))
        where = _densify(section)[i]
        clearance.append({"section": "{} -> {}".format(a_name, b_name), "min_room_m": round(float(d[i]), 4),
                          "at": [round(float(where[0]), 3), round(float(where[1]), 3)],
                          "nearest": nearest_obstacle(cs, where),
                          "max_speed_mps": round(float(v[s0:s1 + 1].max()), 2)})

    return_samples, return_s = None, None
    ret = rules.get("return")
    return_phases: List[Tuple[str, float, float]] = []
    if ret:
        home = _marking_centre(cmap, ret["marking"])
        z_ret = config.return_z_m
        roof = cmap.boundary.get("z", [0.0, None])[1]
        if roof is not None and drone.band(z_ret)[1] > roof:
            raise PlanError("return height {:.2f} m puts the drone into the roof net at {:.2f} m".format(z_ret, roof))
        hits = [column_clear(cmap, drone, yaw, goal, z_ret), column_clear(cmap, drone, yaw, home, z_ret),
                segment_clear(cmap, drone, yaw, goal, home, z_ret)]
        if any(hits):
            notes.append("return not planned: it would hit {}".format(", ".join(h for h in hits if h)))
        else:
            t0 = finish + config.rearm_delay_s
            legs, tt = [], t0
            for name, a3, b3, vmax, slow in (
                    ("return_climb", np.r_[goal, rest], np.r_[goal, z_ret], config.climb_v, None),
                    ("return_cross", np.r_[goal, z_ret], np.r_[home, z_ret], config.return_v, None),
                    ("return_descent", np.r_[home, z_ret], np.r_[home, rest], config.descend_v, config.touchdown_v)):
                acc = config.vertical_a if a3[0] == b3[0] and a3[1] == b3[1] else config.return_a
                s, dur = profile_1d(a3, b3, vmax, acc, dt, v_slow=slow,
                                    slow_last_m=config.touchdown_last_m if slow else 0.0, t0=tt)
                legs.append(s)
                return_phases.append((name, tt, tt + dur))
                tt += dur
            return_samples = _concat(legs)
            return_s = tt + config.disarm_delay_s - finish
            limit = ret.get("time_limit_s")
            if limit and finish + return_s > limit:
                notes.append("finish + return = {:.1f} s exceeds the {:.0f} s limit".format(finish + return_s, limit))

    return Plan(route, config, cs, gates, grid_route, path, v, t, kappa, list(anchors[1:-1]), lam, candidates,
                samples, phases, checkpoints, finish, return_samples, return_s, clearance, start, goal, notes,
                return_phases)


def _first_crossing(path: np.ndarray, t: np.ndarray, i0: int, i1: int, gate: Gate) -> float:
    """Time the path first reaches the gate segment between indices i0 and i1 (the gate point at i1
    is on it; a smoothed path can cross the line a little earlier and come back to it)."""
    a, e = gate.a, gate.b - gate.a
    for i in range(i0, i1):
        p, q = path[i], path[i + 1]
        d = q - p
        den = d[0] * e[1] - d[1] * e[0]
        if abs(den) < 1e-12:
            continue
        w = a - p
        s = (w[0] * e[1] - w[1] * e[0]) / den   # along the path segment
        u = (w[0] * d[1] - w[1] * d[0]) / den   # along the gate
        if 0.0 <= s <= 1.0 and 0.0 <= u <= 1.0:
            return float(t[i] + s * (t[i + 1] - t[i]))
    return float(t[i1])


def _concat(parts: Sequence[Samples]) -> Samples:
    keep = [parts[0]] + [Samples(p.t[1:], p.pos[1:], p.vel[1:], p.acc[1:]) for p in parts[1:]]
    return Samples(np.concatenate([p.t for p in keep]), np.concatenate([p.pos for p in keep]),
                   np.concatenate([p.vel for p in keep]), np.concatenate([p.acc for p in keep]))


def _phase_of(t: float, phases: Sequence[Tuple[str, float, float]]) -> str:
    for name, t0, t1 in phases:
        if t <= t1 + 1e-9:
            return name
    return phases[-1][0]


def summary(plan: Plan) -> Dict[str, Any]:
    cfg = asdict(plan.config)
    return {
        "route": plan.route,
        "finish_s": round(plan.finish_s, 3),
        "checkpoints": plan.checkpoints,
        "return_s": None if plan.return_s is None else round(plan.return_s, 3),
        "smoothness_chosen": plan.smoothness,
        "candidates": plan.candidates,
        "gates": [{"name": g.name, "free_width_m": round(g.free_width, 4),
                   "free_from": g.point(g.lo).round(4).tolist(), "free_to": g.point(g.hi).round(4).tolist()}
                  for g in plan.gates],
        "room_along_path": plan.clearance,
        "course_length_m": round(float(np.linalg.norm(np.diff(plan.path, axis=0), axis=1).sum()), 3),
        "config": cfg,
        "notes": plan.notes,
        "frames": {
            "enu": "course map frame: metres, x east, y north, z up; z is the body centre height",
            "px4_ned": "PX4 local NED with its origin at the launch mark, z = 0 at the resting body height",
        },
    }


def write_outputs(plan: Plan, out_dir: Path, stem: str, rules_path: Optional[Path] = None) -> Dict[str, Path]:
    """CSV setpoints (course ENU and PX4 NED) and the JSON summary.  ``rules_path`` is recorded in
    the summary so 16_analyze_flight.py can find the rules and map again."""
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {"enu": out_dir / "{}_enu.csv".format(stem), "ned": out_dir / "{}_px4_ned.csv".format(stem),
             "summary": out_dir / "{}_summary.json".format(stem)}
    yaw = math.radians(plan.config.yaw_deg)
    yaw_ned = math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))
    rows: List[Tuple[str, Samples]] = [("run", plan.samples)]
    if plan.return_samples is not None:
        rows.append(("return", plan.return_samples))
    origin = np.r_[plan.start, plan.config.rest_z_m]
    with paths["enu"].open("w", newline="") as fe, paths["ned"].open("w", newline="") as fn:
        we, wn = csv.writer(fe), csv.writer(fn)
        we.writerow(["t", "phase", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw"])
        wn.writerow(["t", "phase", "north", "east", "down", "v_north", "v_east", "v_down",
                     "a_north", "a_east", "a_down", "yaw"])
        for label, s in rows:
            for k in range(len(s.t)):
                phase = _phase_of(s.t[k], plan.phases if label == "run" else plan.return_phases)
                p, v, a = s.pos[k], s.vel[k], s.acc[k]
                we.writerow(["{:.3f}".format(s.t[k]), phase] + ["{:.4f}".format(x) for x in (*p, *v, *a)]
                            + ["{:.4f}".format(yaw)])
                rel = p - origin
                ned = (rel[1], rel[0], -rel[2], v[1], v[0], -v[2], a[1], a[0], -a[2])
                wn.writerow(["{:.3f}".format(s.t[k]), phase] + ["{:.4f}".format(x) for x in ned]
                            + ["{:.4f}".format(yaw_ned)])
    info = summary(plan)
    info["px4_ned_origin_enu"] = [round(float(v), 5) for v in origin]
    info["home_enu"] = [round(float(v), 5) for v in plan.start]
    info["pad_enu"] = [round(float(v), 5) for v in plan.goal]
    if rules_path is not None:
        info["rules"] = str(Path(rules_path).resolve())
    paths["summary"].write_text(json.dumps(info, indent=1) + "\n", encoding="utf-8")
    return paths
