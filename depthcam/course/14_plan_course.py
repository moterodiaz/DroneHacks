#!/usr/bin/env python3
"""Course map + rules -> fastest safe trajectory through every checkpoint, as PX4 setpoints.

WHAT THIS DOES
    For each route option (the v3 course has two ways into the landing area: the window or
    Gate B) and each heading you ask for, ``course.planner.plan.plan_course``:
      1. grows every obstacle by the drone's footprint plus margin at that fixed heading;
      2. finds the shortest grid route launch -> CP1 -> ... -> pad and straightens it;
      3. smooths it with several length/curvature trade-offs, keeping the whole drone clear;
      4. computes the time-optimal speed along each for --v-max / --a-max and keeps the fastest;
      5. adds the vertical takeoff, the landing, the disarm delay, and the return over the barrier.
    It writes, per option, into --out:
      <route>_yaw<deg>_enu.csv       50 Hz setpoints in the course frame (x east, y north, z up)
      <route>_yaw<deg>_px4_ned.csv   the same in PX4 local NED, origin at the launch mark
      <route>_yaw<deg>_summary.json  checkpoint times, room at every gate and along the path
      <route>_yaw<deg>.png           top-down picture

HOW TO RUN (from the repository root)
    python course/14_plan_course.py                                  # both routes, nose north
    python course/14_plan_course.py --route window --yaw-deg 90 0    # compare two headings
    python course/14_plan_course.py --a-max 2 --v-max 1.5            # a gentler first flight
    python course/14_plan_course.py --extra-margin "Tube oval 58 x 72 x 60=0.15"

WHAT YOU SHOULD SEE
    One block per option, e.g.
        window, heading 90 deg: finish 15.4 s (CP1 2.7  CP2 6.8  CP3 9.3  CP4 11.6), +5 window bonus
          tightest room beyond the margin: +0.003 m (CP1 end of first corridor -> CP2 Gate A, M1 maze divider)
    and a final comparison line.  "Room beyond the margin" is how far the drone outline plus
    --margin stays from the nearest obstacle; 0 means the margin is all you have there.

LIMITS
    Point-mass model: no jerk limit and no tracking lag, so the times are what a perfect tracker
    would do with --a-max.  Set --a-max from what your drone tracks in flight logs.  The heading is
    fixed for the whole run.

EXIT CODES
    0  planned (every option asked for)
    1  at least one option could not be planned (the reason is printed)
    2  missing map or rules file
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.course_map import CourseMap  # noqa: E402
from course.planner.cspace import DroneSpec  # noqa: E402
from course.planner.plan import PlanConfig, plan_course, write_outputs  # noqa: E402
from course.planner.route import PlanError  # noqa: E402

DEFAULT_RULES = REPO_ROOT / "course" / "maps" / "course_v3_rules.json"


def parse_extra(items: List[str]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for item in items:
        name, _, value = item.rpartition("=")
        if not name:
            raise argparse.ArgumentTypeError("--extra-margin wants NAME=METRES, got {!r}".format(item))
        out[name] = float(value)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rules", type=Path, default=DEFAULT_RULES, help="course rules JSON (names its map file)")
    ap.add_argument("--route", default="all", help="route option at a checkpoint with 'routes' (default: all)")
    ap.add_argument("--yaw-deg", type=float, nargs="+", default=[90.0],
                    help="fixed heading(s) to try, ENU degrees (0 = nose east, 90 = nose north)")
    ap.add_argument("--length", type=float, default=0.40, help="drone length nose to tail, m")
    ap.add_argument("--width", type=float, default=0.50, help="drone width side to side, m")
    ap.add_argument("--height", type=float, default=0.20, help="drone height, m (an assumption: measure it)")
    ap.add_argument("--margin", type=float, default=0.05, help="safety margin on every side, m")
    ap.add_argument("--extra-margin", action="append", default=[], metavar="NAME=M",
                    help="more margin around one feature (adds to the rules file's extra_margin_m)")
    ap.add_argument("--cruise-z", type=float, default=0.80, help="body centre height for the course, m")
    ap.add_argument("--v-max", type=float, default=2.5, help="horizontal speed limit, m/s")
    ap.add_argument("--a-max", type=float, default=3.0, help="total horizontal acceleration limit, m/s^2")
    ap.add_argument("--disarm-delay", type=float, default=0.5, help="touchdown to disarm, s")
    ap.add_argument("--fly-over-rocks", action="store_true",
                    help="allow flying over boulders lower than the drone (default: keep out of their footprint)")
    ap.add_argument("--grid", type=float, default=0.02, help="route grid resolution, m")
    ap.add_argument("--iterations", type=int, default=400, help="smoothing iterations per candidate")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "course" / "out", help="output folder")
    ap.add_argument("--no-image", action="store_true", help="skip the PNG")
    args = ap.parse_args(argv)

    if not args.rules.is_file():
        print("no rules file {}".format(args.rules), file=sys.stderr)
        return 2
    rules = json.loads(args.rules.read_text(encoding="utf-8"))
    map_path = args.rules.parent / rules["map"]
    if not map_path.is_file():
        print("no map file {} (run course/12_extract_course_map.py first)".format(map_path), file=sys.stderr)
        return 2
    cmap = CourseMap.load(map_path)

    options = sorted({r for cp in rules["checkpoints"] for r in cp.get("routes", {})}) or ["direct"]
    routes = options if args.route == "all" else [args.route]
    drone = DroneSpec(args.length, args.width, args.height, args.margin)

    results = []
    failed = False
    for route in routes:
        for yaw in args.yaw_deg:
            cfg = PlanConfig(drone=drone, cruise_z_m=args.cruise_z, yaw_deg=yaw, grid_m=args.grid,
                             iterations=args.iterations, v_max=args.v_max, a_max=args.a_max,
                             disarm_delay_s=args.disarm_delay, rocks_full_height=not args.fly_over_rocks,
                             extra_margin_m=parse_extra(args.extra_margin))
            label = "{}, heading {:g} deg".format(route, yaw)
            try:
                plan = plan_course(cmap, rules, route, cfg)
            except (PlanError, KeyError) as exc:
                print("{}: cannot plan: {}".format(label, exc))
                failed = True
                continue
            stem = "{}_yaw{:g}".format(route, yaw)
            paths = write_outputs(plan, args.out, stem, args.rules)
            if not args.no_image:
                import cv2
                from course.planner.render import render_plan
                cv2.imwrite(str(args.out / "{}.png".format(stem)), render_plan(plan, cmap, label))
            bonus = sum(cp.get("bonus_points", 0) for cp in plan.checkpoints)
            cps = "  ".join("CP{} {:.1f}".format(cp["id"], cp["time_s"]) for cp in plan.checkpoints[:-1])
            print("{}: finish {:.2f} s ({}){}".format(label, plan.finish_s, cps,
                                                     ", +{} bonus".format(bonus) if bonus else ""))
            tight = min(plan.clearance, key=lambda c: c["min_room_m"])
            print("  tightest room beyond the margin: {:+.3f} m ({}, {})".format(
                tight["min_room_m"], tight["section"], tight["nearest"]))
            print("  gate free width for the drone centre: " + ", ".join(
                "{} {:.2f} m".format(g.name.split(" ")[0], g.free_width) for g in plan.gates))
            if plan.return_s is not None:
                print("  return over the barrier: {:.1f} s (run total {:.1f} s of the 180 s limit)".format(
                    plan.return_s, plan.finish_s + plan.return_s))
            for note in plan.notes:
                print("  note: " + note)
            print("  wrote {} (+ _px4_ned.csv, _summary.json{})".format(paths["enu"], "" if args.no_image else ", .png"))
            results.append((plan.finish_s - 0.0, bonus, label, plan))

    if len(results) > 1:
        print("\nfastest finish: {} ({:.2f} s)".format(*min(((r[2], r[0]) for r in results), key=lambda x: x[1])))
    if results:
        best = results[0][3]
        print("PX4 SITL spawn at the launch mark: PX4_GZ_MODEL_POSE=\"{:.4f},{:.4f},0.05,0,0,{:.4f}\"".format(
            best.start[0], best.start[1], math.radians(best.config.yaw_deg)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
