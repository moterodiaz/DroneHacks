#!/usr/bin/env python3
"""Course STEP file -> course map JSON (walls, openings, tube, rocks, nets, floor markings).

WHAT THIS DOES
    Reads the organisers' STEP assembly with ``course.step_reader`` (no CAD kernel needed), sorts
    every solid into a feature kind with ``course.course_map.classify_solid`` and writes the map
    JSON that ``13_make_gazebo_world.py`` and ``14_plan_course.py`` read.  It then prints every
    feature and the narrow gaps between obstacles so you can check them against the published
    drawing before trusting a plan.

HOW TO RUN (from the repository root)
    python course/12_extract_course_map.py course/maps/course_v3.step
    python course/12_extract_course_map.py new_course.step --out course/maps/new_course_map.json

WHAT YOU SHOULD SEE
    One line per feature, e.g.
        wall    C2a chicane wall (window)       1.613 m x 0.102 m, z 0.00-1.73, opening s 0.35-1.26 z 0.30-1.22
        tube    Tube oval 58 x 72 x 60          1.524 m long, inside 1.461 m wide x 1.816 m tall, centre z 0.94
    then the clear gaps at --gap-height-m (default 0.8 m) narrower than --gap-report-m (default
    1.2 m), e.g.
        0.927 m  M3 maze divider <-> Net side north
    A gap counts only if the segment across it does not run through a third obstacle.  For the
    v3 course the three maze gaps and Gate A are 0.927 m and Gate B 0.936 m.

EXIT CODES
    0  map written
    2  the STEP file is missing or could not be parsed
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.course_map import CourseMap, extract_map  # noqa: E402
from course.geometry import closest_points_between, points_in_convex  # noqa: E402


def describe(f: dict) -> str:
    kind, z = f["kind"], "z {:.2f}-{:.2f}".format(*f["z"])
    if kind == "wall":
        length = math.dist(f["p0"], f["p1"])
        text = "{:.3f} m x {:.3f} m, {}".format(length, f["thickness"], z)
        for o in f["openings"]:
            text += ", opening s {:.2f}-{:.2f} z {:.2f}-{:.2f}".format(*o["s"], *o["z"])
        return text
    if kind == "tube":
        return "{:.3f} m long, inside {:.3f} m wide x {:.3f} m tall, centre z {:.2f}".format(
            math.dist(f["p0"], f["p1"]), 2 * f["inner"]["half_width"], 2 * f["inner"]["half_height"], f["p0"][2])
    if kind == "rock":
        return "centre ({:.3f}, {:.3f}), radius {:.3f}, {}".format(*f["centre"], f["radius"], z)
    if kind == "marking":
        return "centre ({:.3f}, {:.3f}), half size {:.3f} x {:.3f}".format(*f["centre"], *f["half_size"])
    return z


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step", type=Path, help="course STEP file")
    ap.add_argument("--out", type=Path, default=None,
                    help="map JSON to write (default: <step stem>_map.json next to the STEP file)")
    ap.add_argument("--gap-report-m", type=float, default=1.2, help="print gaps narrower than this")
    ap.add_argument("--gap-height-m", type=float, default=0.8, help="height at which gaps are measured")
    args = ap.parse_args(argv)

    if not args.step.is_file():
        print("no such file: {}".format(args.step), file=sys.stderr)
        return 2
    try:
        data = extract_map(args.step)
    except (ValueError, KeyError, IndexError) as exc:
        print("could not read {}: {}".format(args.step, exc), file=sys.stderr)
        return 2
    out = args.out or args.step.with_name(args.step.stem + "_map.json")
    cmap = CourseMap(data)
    cmap.save(out)

    b = data["boundary"]
    print("{}  ({} features)".format(", ".join(data["source"]["products"]), len(data["features"])))
    print("boundary x {:.3f}..{:.3f}  y {:.3f}..{:.3f}  roof {}".format(
        *b["x"], *b["y"], "{:.3f} m".format(b["z"][1]) if b["z"][1] is not None else "none"))
    for f in data["features"]:
        print("{:8s} {:32s} {}".format(f["kind"], f["name"][:32], describe(f)))
    if data["source"]["unsupported_curves"]:
        print("warning: {} edges on unsupported curve types were reduced to their end points".format(
            data["source"]["unsupported_curves"]))

    obstacles = cmap.obstacles_2d(args.gap_height_m, args.gap_height_m, rocks_full_height=True)
    best = {}
    for i, a in enumerate(obstacles):
        for b_ in obstacles[i + 1:]:
            if a.name == b_.name:
                continue
            pa, pb, d = closest_points_between(a.polygon, b_.polygon)
            key = tuple(sorted((a.name, b_.name)))
            if d < args.gap_report_m and (key not in best or d < best[key][0]):
                best[key] = (d, pa, pb)
    gaps = []
    for (na, nb), (d, pa, pb) in best.items():
        inner = pa + np.linspace(0.05, 0.95, 19)[:, None] * (pb - pa)
        blocked = any(points_in_convex(inner, o.polygon).any() for o in obstacles if o.name not in (na, nb))
        if d > 0.02 and not blocked:
            gaps.append((d, na, nb, (pa + pb) / 2.0))
    print("\nclear gaps narrower than {:.2f} m at z = {:.2f} m (rocks counted at any height):".format(
        args.gap_report_m, args.gap_height_m))
    for d, na, nb, mid in sorted(gaps):
        print("  {:.3f} m  {} <-> {}   at ({:.2f}, {:.2f})".format(d, na, nb, *mid))
    print("\nwrote {}".format(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
