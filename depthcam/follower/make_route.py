"""Plan a route from the STEP-derived map and the ground-truth checkpoints, for the follower.

    python -m follower.make_route                       # both routes into routes/
    python -m follower.make_route --route window --out routes/course_v3_window.json

Runs ``course.planner.plan.plan_course`` (ported from DroneDepthCamTesting ``course-planner-lidar``)
on ``course/maps/course_v3_map.json`` and ``course/maps/course_v3_rules.json``. It plans with the
planner's defaults: the drone 0.40 x 0.50 m plus 0.05 m margin, nose north (``yaw_deg`` 90), and
0.80 m cruise height. It writes ``dronehacks-lidar-route/1`` (``follower.route``). Takes about
8 s per route. Run from ``depthcam/``.

Exit status: 0 planned, 1 a route could not be planned, 2 a missing file.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROUTES = ("window", "gate_b")


def sha256(path: Path) -> str:
    # Hash with LF line ends, as git stores the file, so a Windows checkout gives the same value
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--map", type=Path, default=ROOT / "course" / "maps" / "course_v3_map.json")
    ap.add_argument("--rules", type=Path, default=ROOT / "course" / "maps" / "course_v3_rules.json")
    ap.add_argument("--route", choices=ROUTES, action="append", help="window and/or gate_b (default both)")
    ap.add_argument("--out", type=Path, help="output file (one --route) or folder (default routes/)")
    args = ap.parse_args(argv)
    for f in (args.map, args.rules):
        if not f.exists():
            print("missing {}".format(f), file=sys.stderr)
            return 2

    from course.course_map import CourseMap
    from course.planner.plan import PlanConfig, plan_course
    from course.planner.route import PlanError

    from .route import plan_to_dict

    cmap = CourseMap.load(args.map)
    rules = json.loads(args.rules.read_text(encoding="utf-8"))
    routes = args.route or list(ROUTES)
    generated = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    status = 0
    for route in routes:
        try:
            plan = plan_course(cmap, rules, route, PlanConfig())
        except PlanError as exc:
            print("{}: could not be planned: {}".format(route, exc), file=sys.stderr)
            status = 1
            continue
        data = plan_to_dict(plan, route, sha256(args.map), sha256(args.rules), generated)
        if args.out is not None and len(routes) == 1 and args.out.suffix == ".json":
            out = args.out
        else:
            out = (args.out or ROOT / "routes") / "course_v3_{}.json".format(route)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")
        print("{}: {} points, {:.2f} m, gates {}, tightest room beyond the margin {:+.3f} m -> {}".format(
            route, len(data["path"]), plan.path.shape[0] and sum(
                ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 for a, b in zip(data["path"], data["path"][1:])),
            [g["index"] for g in data["gates"]], data["min_room_m"] or 0.0, out))
    return status


if __name__ == "__main__":
    sys.exit(main())
