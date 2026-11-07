#!/usr/bin/env python3
"""How much do scan smear and tilt hurt MS200K localisation on the course?  (synthetic scans)

WHAT THIS DOES
    Takes a flight log from 15_fly_plan.py (positions, velocities and PX4 roll/pitch/heading), turns
    it into the "true" trajectory in the course frame, simulates MS200K revolutions along it with
    ``course.lidar_sim`` (every beam cast in 3D from the pose at its own time), and localises every
    scan against the map with ``course.lidar_localize.match``, the way it would run on the Jetson:
    each scan starts from the previous estimate moved by the EKF velocity (truth + 5 cm/s noise),
    roll/pitch come from PX4 (truth + 0.5 deg noise), height from the rangefinder (truth + 2 cm).
    Experiments:
      smear   level attitude so only motion matters: frozen pose (no smear) / moving, no deskew /
              moving, deskew with true beam times / moving, deskew with the stock ROS driver's
              timing (index order, which runs backwards for a clockwise sweep)
      tilt    frozen pose so only attitude matters: level / tilted, matcher assumes level /
              tilted, roll-pitch compensated and floor / over-wall beams dropped
      both    real motion and tilt at 5, 10, 15 Hz rotation: naive vs deskew + tilt compensation
      build   10 Hz, both corrections, but the simulated course is built off the CAD (each wall,
              tube and boulder moved by 1, 2 or 4 cm and turned by 0.5, 1 or 2 deg) while the
              matcher keeps the CAD map: the error the real course will add
    Prints a table per experiment and writes study.json and study.png into the log folder.

HOW TO RUN
    python course/17_lidar_study.py course/out/flights/<log folder>
    python course/17_lidar_study.py <log folder> --net-hit 0.0 --seed 1

EXIT CODES
    0  done    2  missing files or a log without the course phase
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.course_map import CourseMap  # noqa: E402
from course.flight_analysis import _find, load_log  # noqa: E402
from course.lidar_localize import DistanceField, MatchConfig, match  # noqa: E402
from course.lidar_sim import (FLOOR, KIND_NAMES, NET, NONE, LidarSpec, Scene, Trajectory, frd_to_flu,  # noqa: E402
                              simulate)


def trajectory_from_log(log_dir: Path, summary: dict) -> tuple:
    log = load_log(log_dir / "flight_log.csv")
    import csv
    with (log_dir / "flight_log.csv").open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    has_att = "roll" in rows[0]
    origin = np.array(summary["px4_ned_origin_enu"])
    keep = np.isin(log["stage"], ["run", "run_touchdown"])
    t = log["time_s"][keep]
    t, first = np.unique(t, return_index=True)
    pos_ned = log["pos"][keep][first]
    pos = np.column_stack([pos_ned[:, 1] + origin[0], pos_ned[:, 0] + origin[1], -pos_ned[:, 2] + origin[2]])
    heading = log["heading"][keep][first]
    if has_att:
        roll = np.array([float(r["roll"]) for r in rows])[keep][first]
        pitch = np.array([float(r["pitch"]) for r in rows])[keep][first]
    else:  # older logs: the thrust direction from the acceleration (a multirotor tilts into it)
        vel = np.gradient(pos, t, axis=0)
        acc = np.gradient(vel, t, axis=0)
        yaw = np.pi / 2 - heading
        fwd = acc[:, 0] * np.cos(yaw) + acc[:, 1] * np.sin(yaw)
        left = -acc[:, 0] * np.sin(yaw) + acc[:, 1] * np.cos(yaw)
        # FRD convention: accelerating forward pitches the nose down (negative), to the right rolls
        # the right side down (positive).
        pitch, roll = -np.arctan2(fwd, 9.81), np.arctan2(-left, 9.81)
    rpy = frd_to_flu(roll, pitch, heading)
    course = log["phase"][keep][first] == "course"
    if not course.any():
        raise ValueError("no course phase in this log")
    return Trajectory(t, pos, rpy), float(t[course][0]), float(t[course][-1]), has_att


def as_built(cmap: CourseMap, sigma_m: float, sigma_deg: float, seed: int) -> CourseMap:
    """The course as it might really be set up: every wall, block, tube and boulder shifted by its
    own random offset (sigma_m per axis) and turned about its centre (sigma_deg).  Tube cradles move
    with the tube; the floor, markings and perimeter nets stay put.  Simulate scans in this one and
    match them against the CAD map to see what an imperfect build costs."""
    rng = np.random.default_rng(seed)
    data = json.loads(json.dumps(cmap.data))
    moves = {}
    for f in data["features"]:
        if f["kind"] in ("wall", "block", "tube", "rock") or (f["kind"] == "support" and "under" not in f):
            moves[f["name"]] = (rng.normal(0, sigma_m, 2), math.radians(rng.normal(0, sigma_deg)))
    for f in data["features"]:
        move = moves.get(f["name"]) or moves.get(f.get("under"))
        if move is None:
            continue
        shift, angle = move
        owner = f if f["name"] in moves else next(g for g in data["features"] if g["name"] == f["under"])
        pivot = (np.array(owner["centre"]) if "centre" in owner else
                 (np.array(owner["p0"][:2]) + np.array(owner["p1"][:2])) / 2 if "p0" in owner else
                 np.mean(np.array(owner["footprint"]), axis=0))
        c, s = math.cos(angle), math.sin(angle)

        def place(p, pivot=pivot, shift=shift, c=c, s=s):
            d = np.asarray(p[:2], dtype=float) - pivot
            return (pivot + shift + [c * d[0] - s * d[1], s * d[0] + c * d[1]]).tolist() + list(p[2:])

        for key in ("p0", "p1", "centre"):
            if key in f:
                f[key] = place(f[key])
        if "footprint" in f:
            f["footprint"] = [place(p) for p in f["footprint"]]
    return CourseMap(data)


def run_config(traj: Trajectory, scene: Scene, field: DistanceField, spec: LidarSpec, t0: float, t1: float,
               mcfg: MatchConfig, freeze: bool, level: bool, seed: int, driver_timing: bool = False,
               tube: dict = None) -> Dict[str, object]:
    rng = np.random.default_rng(1000 + seed)
    scans = simulate(traj, scene, spec, t0, t1, seed=seed, freeze=freeze, level=level)
    est = None
    errs, yaw_errs, inside_tube, lost = [], [], [], 0
    kinds = np.concatenate([s.kind for s in scans])
    period = 1.0 / spec.rate_hz
    for s in scans:
        t_ref = s.t_end
        pos_ref, rpy_ref = traj.at(np.array([t_ref]))
        truth = np.array([pos_ref[0, 0], pos_ref[0, 1], rpy_ref[0, 2]])
        vel = (traj.at(np.array([t_ref + 0.01]))[0][0, :2] - traj.at(np.array([t_ref - 0.01]))[0][0, :2]) / 0.02
        yaw_rate = (traj.at(np.array([t_ref + 0.01]))[1][0, 2] - traj.at(np.array([t_ref - 0.01]))[1][0, 2]) / 0.02
        vel_meas = vel + rng.normal(0, 0.05, 2)
        if est is None:
            prior = truth + np.r_[rng.normal(0, 0.05, 2), rng.normal(0, 0.02)]
        else:
            prior = est + np.r_[vel_meas * period, 0.0]
        if freeze:  # the scan was taken from one pose: nothing to deskew (the prior still moves)
            vel_meas, yaw_rate = np.zeros(2), 0.0
        rp = (np.zeros((len(s.t), 2)) if level else s.true_rpy[:, :2]) + rng.normal(0, math.radians(0.5), (len(s.t), 2))
        if driver_timing:
            rank = np.argsort(np.argsort(s.angle))
            dt = rank / len(s.t) * period - period
        else:
            dt = s.t - t_ref
        pose, stats = match(field, s.range, s.angle, dt, rp, prior, float(pos_ref[0, 2]) + rng.normal(0, 0.02),
                            vel_meas, yaw_rate, mcfg, scene.wall_top)
        err = float(np.hypot(*(pose[:2] - truth[:2])))
        if not np.isfinite(err) or err > 0.5:
            lost += 1
            pose = truth.copy()
            err = math.nan
        errs.append(err)
        yaw_errs.append(abs(math.degrees((pose[2] - truth[2] + math.pi) % (2 * math.pi) - math.pi)))
        if tube is not None:
            rel = pos_ref[0, :2] - tube["p0"][:2]
            inside_tube.append(0 <= rel @ tube["u"][:2] <= tube["length"] and abs(rel @ tube["w"][:2]) < tube["ellipses"][0][0])
        else:
            inside_tube.append(False)
        est = pose
    e = np.array(errs)
    finite = e[np.isfinite(e)]
    tube_e = e[np.array(inside_tube) & np.isfinite(e)]
    return {"scans": len(scans), "lost": lost,
            "rms_cm": round(100 * float(np.sqrt(np.mean(finite ** 2))), 1) if len(finite) else None,
            "p95_cm": round(100 * float(np.percentile(finite, 95)), 1) if len(finite) else None,
            "max_cm": round(100 * float(finite.max()), 1) if len(finite) else None,
            "yaw_rms_deg": round(float(np.sqrt(np.mean(np.array(yaw_errs) ** 2))), 2),
            "tube_rms_cm": round(100 * float(np.sqrt(np.mean(tube_e ** 2))), 1) if len(tube_e) else None,
            "returns": {KIND_NAMES[k]: round(float(np.mean(kinds == k)), 3) for k in (1, 2, 3, FLOOR, NET, NONE)},
            "errors": e.tolist()}


def table(title: str, rows: List[tuple]) -> List[str]:
    out = ["", title, "  {:44s} {:>6s} {:>6s} {:>6s} {:>7s} {:>6s} {:>6s} {:>6s}".format(
        "configuration", "rms", "p95", "max", "yaw rms", "lost", "floor", "none")]
    for name, r in rows:
        out.append("  {:44s} {:>6} {:>6} {:>6} {:>7} {:>6} {:>5.0f}% {:>5.0f}%".format(
            name, r["rms_cm"], r["p95_cm"], r["max_cm"], r["yaw_rms_deg"], "{}/{}".format(r["lost"], r["scans"]),
            100 * r["returns"]["floor"], 100 * r["returns"]["none"]))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log_dir", type=Path)
    ap.add_argument("--summary", type=Path, default=None)
    ap.add_argument("--net-hit", type=float, default=0.2, help="probability a net returns a beam")
    ap.add_argument("--mount-z", type=float, default=0.10, help="scan plane above the body centre, m")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    if not (args.log_dir / "flight_log.csv").is_file():
        print("no flight_log.csv in {}".format(args.log_dir), file=sys.stderr)
        return 2
    events = json.loads((args.log_dir / "flight_events.json").read_text())
    summary_path = args.summary or _find(str(Path(str(events["plan"]).replace("\\", "/")).with_name(
        Path(str(events["plan"]).replace("\\", "/")).name.replace("_px4_ned.csv", "_summary.json"))),
        REPO_ROOT / "course" / "out")
    summary = json.loads(Path(summary_path).read_text())
    rules_path = _find(summary.get("rules", "course_v3_rules.json"), REPO_ROOT / "course" / "maps")
    cmap = CourseMap.load(rules_path.parent / json.loads(rules_path.read_text())["map"])
    try:
        traj, t0, t1, has_att = trajectory_from_log(args.log_dir, summary)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    scene = Scene.from_map(cmap)
    z_scan = summary["config"]["cruise_z_m"] + args.mount_z
    field = DistanceField(cmap, z_scan)
    tube = scene.tubes[0] if scene.tubes else None
    speed = np.linalg.norm(np.diff(traj.pos[:, :2], axis=0), axis=1) / np.maximum(np.diff(traj.t), 1e-6)
    tilt = np.degrees(np.hypot(traj.rpy[:, 0], traj.rpy[:, 1]))
    in_course = (traj.t >= t0) & (traj.t <= t1)
    lines = ["flight {}: course {:.1f} s, speed max {:.2f} m/s, tilt p95 {:.1f} max {:.1f} deg ({}), scan plane {:.2f} m".format(
        args.log_dir.name, t1 - t0, float(speed[in_course[1:]].max()), float(np.percentile(tilt[in_course], 95)),
        float(tilt[in_course].max()), "PX4 attitude" if has_att else "attitude from acceleration", z_scan)]
    results: Dict[str, Dict[str, object]] = {}

    def run(key: str, spec: LidarSpec, mcfg: MatchConfig, freeze: bool, level: bool, driver: bool = False,
            world: Scene = None):
        results[key] = run_config(traj, world or scene, field, spec, t0, t1, mcfg, freeze, level, args.seed, driver, tube)
        return results[key]

    base = LidarSpec(net_hit_prob=args.net_hit, mount=(0.0, 0.0, args.mount_z))
    m = lambda deskew, tilt_: MatchConfig(deskew=deskew, tilt=tilt_, mount=(0.0, 0.0, args.mount_z))  # noqa: E731
    smear = [("frozen pose (no smear)", run("smear_frozen", base, m(False, False), True, True)),
             ("moving, matched as a snapshot", run("smear_naive", base, m(False, False), False, True)),
             ("moving, deskewed with true beam times", run("smear_deskew", base, m(True, False), False, True)),
             ("moving, deskewed with stock-driver timing", run("smear_driver", base, m(True, False), False, True, True))]
    lines += table("SMEAR (10 Hz, level attitude): position error in cm", smear)
    tilt_rows = [("level", results["smear_frozen"]),
                 ("tilted, matcher assumes level", run("tilt_naive", base, m(False, False), True, False)),
                 ("tilted, roll/pitch compensated + filtered", run("tilt_comp", base, m(False, True), True, False))]
    lines += table("TILT (frozen pose, real roll/pitch): position error in cm", tilt_rows)
    both = []
    for hz in (5.0, 10.0, 15.0):
        spec = LidarSpec(rate_hz=hz, net_hit_prob=args.net_hit, mount=(0.0, 0.0, args.mount_z))
        both.append(("{:.0f} Hz naive".format(hz), run("both_{:.0f}_naive".format(hz), spec, m(False, False), False, False)))
        both.append(("{:.0f} Hz deskew + tilt compensation".format(hz),
                     run("both_{:.0f}_full".format(hz), spec, m(True, True), False, False)))
    lines += table("BOTH (real motion and tilt): position error in cm", both)
    build = [("as designed", results["both_10_full"])]
    for cm, deg in ((1, 0.5), (2, 1.0), (4, 2.0)):
        world = Scene.from_map(as_built(cmap, cm / 100, deg, 100 + args.seed))
        build.append(("pieces off by {} cm / {} deg".format(cm, deg),
                      run("build_{}cm".format(cm), base, m(True, True), False, False, world=world)))
    lines += table("BUILD (10 Hz, both corrections; course built off the CAD map): position error in cm", build)
    lines.append("")
    lines.append("tube only (rms cm): naive {}  full {}  (10 Hz)".format(results["both_10_naive"]["tube_rms_cm"],
                                                                        results["both_10_full"]["tube_rms_cm"]))
    print("\n".join(lines))
    out = {k: {kk: vv for kk, vv in v.items() if kk != "errors"} for k, v in results.items()}
    (args.log_dir / "study.json").write_text(json.dumps({"lines": lines, "results": out}, indent=1) + "\n")
    _plot(args.log_dir / "study.png", traj, t0, t1, results, cmap, scene, base, args.mount_z)
    print("wrote {} and study.png".format(args.log_dir / "study.json"))
    return 0


def _plot(path: Path, traj: Trajectory, t0: float, t1: float, results: dict, cmap: CourseMap, scene: Scene,
          spec: LidarSpec, mount_z: float) -> None:
    """Left: the scan at the moment of greatest tilt x speed, true hit points vs where a level,
    snapshot assumption puts them.  Right: position error per scan, naive vs full, 10 Hz."""
    import cv2
    from course.lidar_sim import rot
    from course.planner.render import _View
    view = _View(cmap)
    img = np.full((view.h, view.w + 560, 3), 255, dtype=np.uint8)
    for o in cmap.obstacles_2d(0.85, 0.95, rocks_full_height=False):
        cv2.fillPoly(img, [view.px(o.polygon)], (90, 90, 90), lineType=cv2.LINE_AA)
    mask = (traj.t >= t0) & (traj.t <= t1)
    cv2.polylines(img, [view.px(traj.pos[mask, :2])], False, (190, 190, 190), 2, cv2.LINE_AA)
    speed = np.linalg.norm(np.gradient(traj.pos[:, :2], traj.t, axis=0), axis=1)
    score = np.where(mask, np.hypot(traj.rpy[:, 0], traj.rpy[:, 1]) * speed, -1)
    tk = float(traj.t[int(np.argmax(score))])
    scan = simulate(traj, scene, spec, tk - 0.1, tk + 0.001, seed=7)[0]
    pos, rpy = scan.true_pos, scan.true_rpy
    r = rot(rpy[:, 0], rpy[:, 1], rpy[:, 2])
    local = np.stack([scan.range * np.cos(scan.angle), scan.range * np.sin(scan.angle), np.zeros(len(scan.t))], 1)
    true_pts = pos + np.einsum("nij,nj->ni", r, local + np.array([0, 0, mount_z]))
    end_pos, end_rpy = pos[-1], rpy[-1]
    c, s = math.cos(end_rpy[2]), math.sin(end_rpy[2])
    naive = end_pos[:2] + np.stack([c * local[:, 0] - s * local[:, 1], s * local[:, 0] + c * local[:, 1]], 1)
    colours = {FLOOR: (40, 40, 230), NET: (200, 120, 0)}
    for k in range(len(scan.t)):
        if not np.isfinite(scan.range[k]):
            continue
        a, b = view.px(naive[k])[0], view.px(true_pts[k, :2])[0]
        if max(a[0], b[0]) >= view.w:  # far floor hits: keep them off the graph panel
            continue
        cv2.circle(img, tuple(a), 2, (0, 170, 255), -1)
        cv2.circle(img, tuple(b), 2, colours.get(int(scan.kind[k]), (40, 160, 40)), -1)
    cv2.circle(img, tuple(view.px(end_pos[:2])[0]), 6, (0, 0, 0), 2)
    x = view.w + 20
    notes = ["scan at t = {:.2f} s: speed {:.2f} m/s, roll {:.1f} pitch {:.1f} deg".format(
        tk - t0, float(np.interp(tk, traj.t, speed)), math.degrees(end_rpy[0]), math.degrees(end_rpy[1])),
        "green: true wall hits   red: floor hits   blue: net hits",
        "orange: the same beams placed as a level snapshot at the scan end",
        "", "position error per scan, 10 Hz (cm):", "  grey = naive, green = deskew + tilt comp."]
    for i, line in enumerate(notes):
        cv2.putText(img, line, (x, 30 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (30, 30, 30), 1, cv2.LINE_AA)
    gx0, gy0, gw, gh = x, 180, 520, 260
    cv2.rectangle(img, (gx0, gy0), (gx0 + gw, gy0 + gh), (200, 200, 200), 1)
    top = 30.0
    for key, colour in (("both_10_naive", (150, 150, 150)), ("both_10_full", (40, 160, 40))):
        e = np.array(results[key]["errors"], dtype=float) * 100
        pts = [(gx0 + int(gw * i / max(len(e) - 1, 1)), gy0 + gh - int(gh * min(v, top) / top))
               for i, v in enumerate(e) if np.isfinite(v)]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, dtype=np.int32)], False, colour, 2, cv2.LINE_AA)
        for i, v in enumerate(e):
            if not np.isfinite(v):
                cv2.circle(img, (gx0 + int(gw * i / max(len(e) - 1, 1)), gy0 + 6), 4, (0, 0, 230), -1)
    for v in (10, 20, 30):
        yy = gy0 + gh - int(gh * v / top)
        cv2.putText(img, str(v), (gx0 - 22, yy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 80), 1, cv2.LINE_AA)
    cv2.putText(img, "red dot at top = lost (>50 cm)", (gx0, gy0 + gh + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (80, 80, 80), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


if __name__ == "__main__":
    sys.exit(main())
