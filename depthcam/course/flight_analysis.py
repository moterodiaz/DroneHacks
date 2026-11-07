"""Flight log (from course/player.py) + plan summary -> what the judges would see, and how well it tracked.

:func:`analyze` returns a JSON-ready report:

* ``checkpoints``: when the flown path crossed each gate (seconds from liftoff, the organisers'
  clock), next to the planned time; CP5 is the disarm on the pad.
* ``tracking``: horizontal and vertical distance between setpoint and drone, per phase (max, RMS,
  95th percentile).  This is the number to size ``--margin`` and ``--a-max`` from.
* ``clearance``: the closest the real drone outline (length x width from the plan, no margin)
  came to any obstacle at cruise height, and every stretch where it overlapped one ("touch":
  +5 s each under the rules, or a crash).
* ``landing``: where it disarmed relative to the pad centre and whether the whole drone was inside
  the outer ring; the same for the return relative to the launch mark.

Frames: the log is PX4 local NED; the plan summary's ``px4_ned_origin_enu`` maps it back to the
course frame (x = east + x0, y = north + y0, z = -down + z0).
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from course.course_map import CourseMap
from course.planner.cspace import DroneSpec, build_cspace
from course.planner.plan import nearest_obstacle
from course.planner.route import Gate, resolve_gate

REPO_ROOT = Path(__file__).resolve().parents[1]


def _find(path_text: str, fallback_dir: Path) -> Path:
    """A path recorded on another machine (WSL vs Windows): try it, then its name in ``fallback_dir``."""
    p = Path(path_text)
    if p.is_file():
        return p
    q = fallback_dir / Path(path_text.replace("\\", "/")).name
    if q.is_file():
        return q
    raise FileNotFoundError("cannot find {} (also tried {})".format(path_text, q))


def load_log(path: Path) -> Dict[str, np.ndarray]:
    with Path(path).open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError("{}: empty log".format(path))
    out: Dict[str, Any] = {"stage": np.array([r["stage"] for r in rows]), "phase": np.array([r["phase"] for r in rows])}
    for key in ("time_s", "heading"):
        out[key] = np.array([float(r[key]) for r in rows])
    out["sp"] = np.array([[float(r[k]) for k in ("sp_n", "sp_e", "sp_d")] for r in rows])
    out["pos"] = np.array([[float(r[k]) for k in ("n", "e", "d")] for r in rows])
    for key in ("armed", "offboard", "landed"):
        out[key] = np.array([int(r[key]) for r in rows], dtype=bool)
    return out


def _segments_cross(p0: np.ndarray, p1: np.ndarray, a: np.ndarray, b: np.ndarray) -> bool:
    def side(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return float((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))
    d1, d2 = side(a, b, p0), side(a, b, p1)
    d3, d4 = side(p0, p1, a), side(p0, p1, b)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _distance_to_polyline(points: np.ndarray, poly: np.ndarray, chunk: int = 256) -> np.ndarray:
    a, b = poly[:-1], poly[1:]
    ab = b - a
    ab2 = np.maximum((ab ** 2).sum(axis=1), 1e-18)
    out = []
    for start in range(0, len(points), chunk):
        p = points[start:start + chunk]
        ap = p[:, None, :] - a[None]
        u = np.clip((ap * ab[None]).sum(axis=2) / ab2[None], 0.0, 1.0)
        out.append(np.sqrt(((ap - u[..., None] * ab[None]) ** 2).sum(axis=2)).min(axis=1))
    return np.concatenate(out) if out else np.zeros(0)


def _stats(x: np.ndarray) -> Dict[str, float]:
    x = x[np.isfinite(x)]
    if not len(x):
        return {}
    return {"max_m": round(float(x.max()), 3), "rms_m": round(float(np.sqrt(np.mean(x ** 2))), 3),
            "p95_m": round(float(np.percentile(x, 95)), 3)}


def analyze(log_dir: Path, summary_path: Optional[Path] = None,
            gate_tolerance_m: float = 0.10) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Report dict, and the arrays a picture needs (course-frame flown/planned paths, errors, gates).

    A gate counts when the flown path crosses it, or (for a gate the plan only touches, like CP1 at
    the floor line's corner) comes within ``gate_tolerance_m`` of it."""
    log_dir = Path(log_dir)
    events = json.loads((log_dir / "flight_events.json").read_text(encoding="utf-8"))
    log = load_log(log_dir / "flight_log.csv")
    if summary_path is None:
        plan_name = Path(str(events["plan"]).replace("\\", "/")).name.replace("_px4_ned.csv", "_summary.json")
        summary_path = _find(str(Path(str(events["plan"]).replace("\\", "/")).with_name(plan_name)),
                             REPO_ROOT / "course" / "out")
    summary = json.loads(Path(summary_path).read_text(encoding="utf-8"))
    rules_path = _find(summary.get("rules", "course_v3_rules.json"), REPO_ROOT / "course" / "maps")
    rules = json.loads(rules_path.read_text(encoding="utf-8"))
    cmap = CourseMap.load(rules_path.parent / rules["map"])
    cfg = summary["config"]
    drone_cfg = cfg["drone"]
    origin = np.array(summary["px4_ned_origin_enu"])

    enu = np.column_stack([log["pos"][:, 1] + origin[0], log["pos"][:, 0] + origin[1], -log["pos"][:, 2] + origin[2]])
    sp_enu = np.column_stack([log["sp"][:, 1] + origin[0], log["sp"][:, 0] + origin[1], -log["sp"][:, 2] + origin[2]])
    t = log["time_s"]
    in_run = np.isin(log["stage"], ["run", "run_touchdown"])
    # The height estimate on the floor is off by several cm and differs run to run (barometer), so
    # liftoff is measured from what the drone reported while waiting to arm, not from zero.
    first_run = int(np.argmax(in_run))
    waiting = np.where(np.isin(log["stage"][:first_run], ["prestream", "arming"]))[0]
    ground = float(np.median(enu[waiting, 2])) if len(waiting) else float(enu[first_run, 2])
    lifted = np.where(in_run & log["armed"] & (enu[:, 2] - ground > 0.05))[0]
    report: Dict[str, Any] = {"result": events.get("result"), "final_stage": events.get("final_stage"),
                              "abort": events.get("abort"), "route": summary["route"]}
    if not len(lifted):
        report["error"] = "the drone never left the ground"
        return report, {}
    # Liftoff = start of the climb (the planner's time zero, and when the drone leaves the mark):
    # find it 5 cm up, then step back to the last sample within 1 cm of the resting height.
    i0 = int(lifted[0])
    while i0 > 0 and in_run[i0 - 1] and enu[i0 - 1, 2] - ground > 0.01:
        i0 -= 1
    t0 = float(t[i0])
    run_idx = np.where(in_run)[0]
    run_end = int(run_idx[-1])

    band = DroneSpec(drone_cfg["length_m"], drone_cfg["width_m"], drone_cfg["height_m"], 0.0).band(cfg["cruise_z_m"])
    gates: List[Gate] = []
    meta: List[Dict[str, Any]] = []
    for cp in rules["checkpoints"]:
        if cp.get("land"):
            continue
        spec = cp["routes"][summary["route"]] if "routes" in cp else cp
        gates.append(resolve_gate(spec["gate"], cp["name"], cmap, band))
        meta.append(cp)
    planned = {cp["id"]: cp["time_s"] for cp in summary["checkpoints"]}
    checkpoints = []
    k = i0
    for gate, cp in zip(gates, meta):
        hit, how = None, None
        for j in range(k, run_end):
            if _segments_cross(enu[j, :2], enu[j + 1, :2], gate.a, gate.b):
                hit, how = j + 1, "crossed"
                break
        if hit is None:  # a gate the plan only touches (CP1's corner): reached if it came within tolerance
            near = np.array([float(np.linalg.norm(gate.project_unclamped(p) - p)) for p in enu[k:run_end, :2]])
            close = np.where(near <= gate_tolerance_m)[0]
            if len(close):
                run_end_close = close[0]  # the first stretch within tolerance; its closest point is the touch
                while run_end_close + 1 < len(near) and near[run_end_close + 1] <= gate_tolerance_m:
                    run_end_close += 1
                best = close[0] + int(np.argmin(near[close[0]:run_end_close + 1]))
                hit, how = k + int(best), "within {:.2f} m".format(gate_tolerance_m)
        checkpoints.append({"id": cp["id"], "name": cp["name"], "planned_s": planned.get(cp["id"]),
                            "flown_s": None if hit is None else round(float(t[hit]) - t0, 3), "how": how})
        if hit is not None:
            k = hit
    disarm = events.get("run_disarmed_s")
    finish_cp = [cp for cp in rules["checkpoints"] if cp.get("land")]
    if finish_cp:
        checkpoints.append({"id": finish_cp[0]["id"], "name": finish_cp[0]["name"],
                            "planned_s": planned.get(finish_cp[0]["id"]),
                            "flown_s": None if disarm is None else round(float(disarm) - t0, 3)})
    report["checkpoints"] = checkpoints
    report["finish_s"] = None if disarm is None else round(float(disarm) - t0, 3)
    report["planned_finish_s"] = summary["finish_s"]

    # Two error measures.  "horizontal": distance to the setpoint of the same instant; includes lag
    # along the path, which costs time but cannot hit a wall.  "off_path": distance to the nearest
    # point of the planned path, which is what eats into the margin: size --margin from this one.
    tracking = {}
    horiz = np.linalg.norm(enu[:, :2] - sp_enu[:, :2], axis=1)
    vert = np.abs(enu[:, 2] - sp_enu[:, 2])
    path_pts = sp_enu[(log["stage"] == "run") & np.isfinite(sp_enu[:, 0]), :2]
    off_path = np.full(len(t), np.nan)
    for phase in ("takeoff", "course", "landing"):
        mask = (log["stage"] == "run") & (log["phase"] == phase)
        if mask.any():
            idx = np.where(mask)[0]
            off_path[idx] = _distance_to_polyline(enu[idx, :2], path_pts)
            tracking[phase] = {"horizontal": _stats(horiz[mask]), "off_path": _stats(off_path[mask]),
                               "vertical": _stats(vert[mask])}
    report["tracking"] = tracking

    yaw = math.radians(cfg["yaw_deg"])
    body = DroneSpec(drone_cfg["length_m"], drone_cfg["width_m"], drone_cfg["height_m"], 0.0)
    cs = build_cspace(cmap, body, yaw, cfg["cruise_z_m"], res=0.05, rocks_full_height=False)
    flying = np.where((log["stage"] == "run") & (log["phase"] == "course"))[0]
    clearance: Dict[str, Any] = {}
    touches: List[Dict[str, Any]] = []
    room = np.full(len(t), np.nan)
    if len(flying):
        d, _ = cs.distance(enu[flying, :2])
        room[flying] = d
        i = int(np.argmin(d))
        where = enu[flying[i], :2]
        clearance = {"min_m": round(float(d[i]), 3), "at": [round(float(v), 3) for v in where],
                     "nearest": nearest_obstacle(cs, where), "planned_margin_m": drone_cfg["margin_m"]}
        inside = d < 0.0
        j = 0
        while j < len(inside):
            if inside[j]:
                k2 = j
                while k2 + 1 < len(inside) and inside[k2 + 1]:
                    k2 += 1
                seg = slice(j, k2 + 1)
                worst = j + int(np.argmin(d[seg]))
                p = enu[flying[worst], :2]
                touches.append({"from_s": round(float(t[flying[j]]) - t0, 2), "to_s": round(float(t[flying[k2]]) - t0, 2),
                                "depth_m": round(float(-d[worst]), 3), "at": [round(float(v), 3) for v in p],
                                "obstacle": nearest_obstacle(cs, p)})
                j = k2 + 1
            else:
                j += 1
    clearance["touches"] = touches
    report["clearance"] = clearance

    half_diag = 0.5 * math.hypot(drone_cfg["length_m"], drone_cfg["width_m"])
    pad = np.array(summary["pad_enu"])
    landing = {}
    if disarm is not None:
        at = enu[int(np.searchsorted(t, disarm)) - 1, :2]
        off = float(np.linalg.norm(at - pad))
        limit = rules.get("finish", {}).get("inside_radius_m")
        landing["run"] = {"offset_m": round(off, 3), "inside_ring": None if limit is None else off + half_diag <= limit}
    ret_disarm = events.get("return_disarmed_s")
    if ret_disarm is not None:
        at = enu[int(np.searchsorted(t, ret_disarm)) - 1, :2]
        off = float(np.linalg.norm(at - np.array(summary["home_enu"])))
        limit = rules.get("return", {}).get("within_m")
        landing["return"] = {"offset_m": round(off, 3), "within_limit": None if limit is None else off <= limit,
                             "run_plus_return_s": round(float(ret_disarm) - t0, 2)}
    report["landing"] = landing

    picture = {"cmap": cmap, "flown": enu[i0:run_end + 1, :2], "error": np.nan_to_num(off_path[i0:run_end + 1]),
               "obstacles": cs.obstacles, "gates": gates, "touches": touches,
               "planned": sp_enu[(log["stage"] == "run"), :2]}
    return report, picture


def report_lines(report: Dict[str, Any]) -> List[str]:
    lines = ["route {}: {}".format(report["route"], report.get("result"))]
    if report.get("abort"):
        lines.append("ABORT: {}".format(report["abort"].get("reason")))
    if report.get("error"):
        return lines + [report["error"]]
    lines.append("checkpoint times from liftoff (flown / planned):")
    for cp in report["checkpoints"]:
        flown = "-" if cp["flown_s"] is None else "{:.2f}".format(cp["flown_s"])
        plan = "-" if cp["planned_s"] is None else "{:.2f}".format(cp["planned_s"])
        lines.append("  {:>6} / {:>6} s  {}".format(flown, plan, cp["name"]))
    for phase, tr in report["tracking"].items():
        h, o, v = tr["horizontal"], tr["off_path"], tr["vertical"]
        if h:
            lines.append("{:8s} off path max {:.2f} p95 {:.2f} m | behind/ahead of setpoint p95 {:.2f} m | "
                         "vertical max {:.2f} m".format(phase, o["max_m"], o["p95_m"], h["p95_m"], v.get("max_m", math.nan)))
    c = report["clearance"]
    if "min_m" in c:
        lines.append("closest approach of the drone outline: {:+.3f} m ({}) at ({:.2f}, {:.2f})".format(
            c["min_m"], c["nearest"], *c["at"]))
    for touch in c.get("touches", []):
        lines.append("  TOUCH {} at {:.1f}-{:.1f} s, {:.3f} m deep at ({:.2f}, {:.2f})".format(
            touch["obstacle"], touch["from_s"], touch["to_s"], touch["depth_m"], *touch["at"]))
    for key, land in report.get("landing", {}).items():
        extra = ""
        if "inside_ring" in land:
            extra = ", whole drone inside the ring: {}".format(land["inside_ring"])
        if "within_limit" in land:
            extra = ", within 2 ft: {}, run + return {:.1f} s".format(land["within_limit"], land["run_plus_return_s"])
        lines.append("{} landing {:.3f} m from target{}".format(key, land["offset_m"], extra))
    return lines
