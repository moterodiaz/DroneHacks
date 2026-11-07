#!/usr/bin/env python3
"""A map of a rectangular room, so the lidar tools can run on the bench instead of only on the course.

WHAT THIS DOES
    18_lidar_live.py localize and 19_lidar_px4_bridge.py match every lidar revolution to a map.
    Away from the course, this writes a map of the room you test in: its four walls, any boxes you
    add, and a start mark.

    Coordinates are in metres:
    * Stand behind the rig on the start mark, facing where its nose points. That direction is +y,
      and +x is to your right.
    * The origin is the room's back-left inside corner: the left wall is x = 0, the back wall y = 0.
    * The rig therefore starts nose "north" (yaw 90), which is what both tools assume.

    Writes to --out:
      bench_map.json      the map (course-map/1)
      bench_rules.json    names the map and the start mark (18_lidar_live.py localize --rules)
      bench_summary.json  PX4's origin: the start mark at the rig's height (19_lidar_px4_bridge.py --summary)

    Then it checks the map. Synthetic MS200K revolutions are cast from the start mark and matched
    from a guess 14 cm and 4 deg off; the match must come back to the mark.

HOW TO RUN
    python3 course/20_bench_map.py --room 4.20 6.10 --start 2.10 1.00 --height 0.95
    python3 course/20_bench_map.py --room 4.20 6.10 --start 2.10 1.00 --height 0.95 --box 3.40 2.00 4.20 2.60 1.20

    --room W L               inside width (left wall to right wall) and length (back wall to front
                             wall), measured at the lidar's height
    --start X Y              the start mark: X from the left wall, Y from the back wall
    --height H               the body centre (the Pixhawk) above the floor during the test
    --box X0 Y0 X1 Y1 TOP    a box-shaped thing in the room (a cabinet, a stack of boxes): two
                             opposite corners and its height. Repeat for more.

    Only what the map holds can be matched. Keep the scan plane clear of furniture that isn't in it.

EXIT CODES
    0  written and checked    1  written, but the check did not come back to the mark    2  bad arguments
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.course_map import MAP_FORMAT, CourseMap  # noqa: E402
from course.lidar_localize import DistanceField, MatchConfig, Tracker  # noqa: E402
from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate  # noqa: E402

WALL_T = 0.10           # wall thickness in the map; only the inside faces matter
MARK_HALF = 0.0762      # the start mark is drawn 6 in square, like the course's launch mark
WALL_COLOR = [0.92157, 0.8, 0.6549]


def _square(cx: float, cy: float, h: float) -> List[List[float]]:
    return [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]


def room_map(width: float, length: float, start: Sequence[float], ceiling: float,
             boxes: Sequence[Sequence[float]]) -> dict:
    """course-map/1 data: four walls with their inside faces on x = 0, x = width, y = 0, y = length."""
    t, w, l = WALL_T, width, length

    def wall(name, p0, p1):
        return {"name": name, "color": WALL_COLOR, "z": [0.0, ceiling], "kind": "wall",
                "p0": p0, "p1": p1, "thickness": t, "openings": []}

    features = [
        {"name": "Floor", "color": [0.91765, 0.91765, 0.91765], "z": [-0.00635, 0.0], "kind": "floor",
         "footprint": [[0.0, 0.0], [w, 0.0], [w, l], [0.0, l]]},
        wall("Left wall", [-t / 2, -t], [-t / 2, l + t]),
        wall("Right wall", [w + t / 2, -t], [w + t / 2, l + t]),
        wall("Back wall", [-t, -t / 2], [w + t, -t / 2]),
        wall("Front wall", [-t, l + t / 2], [w + t, l + t / 2]),
        {"name": "Launch mark", "color": [0.58824, 0.15686, 0.12549], "z": [0.0, 0.00127], "kind": "marking",
         "centre": [start[0], start[1]], "half_size": [MARK_HALF, MARK_HALF],
         "polygons": [{"outer": _square(start[0], start[1], MARK_HALF), "holes": []}]},
    ]
    for k, (x0, y0, x1, y1, top) in enumerate(boxes, 1):
        features.append({"name": "Box {}".format(k), "color": [0.6, 0.45, 0.3], "z": [0.0, top], "kind": "block",
                         "footprint": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]})
    return {"format": MAP_FORMAT,
            "frame": "metres; +x to the right and +y ahead of the rig on the start mark; origin the back-left inside corner",
            "source": {"bench": "course/20_bench_map.py", "room_m": [w, l]},
            "boundary": {"x": [-t, w + t], "y": [-t, l + t], "z": [0.0, ceiling]},
            "features": features}


def check(cmap: CourseMap, start: Sequence[float], height: float, mount_z: float) -> np.ndarray:
    """(dx, dy, dyaw) left after matching half a second of synthetic scans from a guess 14 cm and 4
    deg off the start mark."""
    scene = Scene.from_map(cmap)
    pos = [start[0], start[1], height]
    traj = Trajectory(np.array([0.0, 1.0]), np.array([pos, pos]), np.array([[0.0, 0.0, math.pi / 2]] * 2))
    scans = simulate(traj, scene, LidarSpec(mount=(0.0, 0.0, mount_z)), 0.0, 0.5, seed=1)
    guess = np.array([start[0] + 0.1, start[1] - 0.1, math.radians(94.0)])
    tracker = Tracker(DistanceField(cmap, height + mount_z), MatchConfig(mount=(0.0, 0.0, mount_z)), guess,
                      wall_top=scene.wall_top)
    for s in scans:
        pose, _ = tracker.update(s.t, s.range, s.angle, np.zeros((len(s.t), 2)), height)
    return np.array([pose[0] - start[0], pose[1] - start[1], pose[2] - math.pi / 2])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Write a map of a rectangular room for the lidar tools (bench tests).")
    ap.add_argument("--room", type=float, nargs=2, metavar=("W", "L"), required=True,
                    help="inside width (left to right wall) and length (back to front wall), m")
    ap.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"), required=True,
                    help="start mark: from the left wall, from the back wall, m")
    ap.add_argument("--height", type=float, required=True, help="body centre (Pixhawk) above the floor, m")
    ap.add_argument("--mount-z", type=float, default=0.10, help="scan plane above the body centre, m")
    ap.add_argument("--ceiling", type=float, default=2.5, help="wall height in the map, m")
    ap.add_argument("--box", type=float, nargs=5, action="append", default=[], metavar=("X0", "Y0", "X1", "Y1", "TOP"),
                    help="a box in the room: two opposite corners and its height, m (repeatable)")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "course" / "out" / "bench")
    args = ap.parse_args(argv)

    (w, l), (sx, sy) = args.room, args.start
    scan_z = args.height + args.mount_z
    boxes = [(min(b[0], b[2]), min(b[1], b[3]), max(b[0], b[2]), max(b[1], b[3]), b[4]) for b in args.box]
    problems = []
    if w < 1.0 or l < 1.0:
        problems.append("the room must be at least 1 m each way")
    if not (0.3 <= sx <= w - 0.3 and 0.3 <= sy <= l - 0.3):
        problems.append("the start mark must be inside the room, at least 0.3 m from every wall (the lidar ignores "
                        "returns closer than 0.25 m)")
    if not 0.2 <= scan_z <= args.ceiling - 0.3:
        problems.append("the scan plane ({:.2f} m) must be between 0.2 m and 0.3 m under --ceiling".format(scan_z))
    for x0, y0, x1, y1, top in boxes:
        if x0 < 0 or y0 < 0 or x1 > w or y1 > l or top <= 0:
            problems.append("box {} is not inside the room".format([x0, y0, x1, y1, top]))
        elif x0 - 0.3 < sx < x1 + 0.3 and y0 - 0.3 < sy < y1 + 0.3:
            problems.append("box {} is within 0.3 m of the start mark".format([x0, y0, x1, y1, top]))
    if problems:
        for p in problems:
            print(p, file=sys.stderr)
        return 2

    args.out.mkdir(parents=True, exist_ok=True)
    cmap = CourseMap(room_map(w, l, (sx, sy), args.ceiling, boxes))
    cmap.save(args.out / "bench_map.json")
    rules_path = (args.out / "bench_rules.json").resolve()
    rules_path.write_text(json.dumps({"format": "course-rules/1", "name": "Bench room {:g} x {:g} m".format(w, l),
                                      "map": "bench_map.json", "start": {"marking": "Launch mark"}}, indent=1) + "\n")
    summary_path = args.out / "bench_summary.json"
    summary_path.write_text(json.dumps({"rules": str(rules_path), "px4_ned_origin_enu": [sx, sy, args.height],
                                        "bench": {"room_m": [w, l], "start_m": [sx, sy], "height_m": args.height,
                                                  "mount_z_m": args.mount_z, "boxes": boxes}}, indent=1) + "\n")
    print("wrote {}, {} and {}".format(args.out / "bench_map.json", rules_path, summary_path))

    err = check(cmap, (sx, sy), args.height, args.mount_z)
    off_cm, off_deg = 100 * math.hypot(err[0], err[1]), math.degrees(err[2])
    print("check: synthetic scans from the mark match to {:.1f} cm, {:.2f} deg".format(off_cm, off_deg))
    print("\nnext (add --yaw-offset / --counter-clockwise from 18_lidar_live.py check):")
    print("  python3 course/18_lidar_live.py localize --port /dev/ms200 --rules {} --scan-z {:.2f} --mount-z {:.2f}".format(
        rules_path, scan_z, args.mount_z))
    print("  python3 course/19_lidar_px4_bridge.py --port /dev/ms200 --mavlink udpin:0.0.0.0:14541 --summary {} "
          "--mount-z {:.2f}".format(summary_path.resolve(), args.mount_z))
    return 0 if off_cm < 1.0 and abs(off_deg) < 0.5 else 1


if __name__ == "__main__":
    sys.exit(main())
