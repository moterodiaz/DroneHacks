#!/usr/bin/env python3
"""Course map JSON -> Gazebo Harmonic world + model for PX4 SITL (walls, window, tube, boulders, floor).

WHAT THIS DOES
    Writes, under --out (default course/gazebo):
        models/<world>_course/   model.config, model.sdf, meshes/ (tube OBJ, floor OBJ + texture)
        worlds/<world>.sdf       the systems PX4 SITL needs, a sun, a ground plane and the course
    See ``course/worldgen.py`` for how each feature kind is modelled.  The world frame is the map
    frame (x east, y north, z up), so a plan's ENU setpoints are Gazebo world coordinates.

HOW TO RUN (from the repository root, any OS; Gazebo itself runs in WSL2 / Ubuntu 22.04)
    python course/13_make_gazebo_world.py
    python course/13_make_gazebo_world.py --map course/maps/other_map.json --world other
    python course/13_make_gazebo_world.py --visible-nets      # the GPU lidar will then see the nets

THEN, IN WSL2 (Gazebo Harmonic, PX4 v1.17 checkout in ~/PX4-Autopilot)
    bash course/gazebo/run_sitl.sh               # Gazebo on the course, PX4 SITL spawned at the launch mark
    HEADLESS=1 bash course/gazebo/run_sitl.sh     # no GUI
    The script starts Gazebo on this world and PX4 attaches to it, so nothing is copied into the PX4
    tree.  (Alternative: link the model and world into PX4's Tools/simulation/gz/ and run
    PX4_GZ_WORLD=course_v3 make px4_sitl gz_x500_lidar_2d.)

WHAT YOU SHOULD SEE
    The console lists what was written and anything simplified (the tube cradles are left out).  In
    Gazebo: the red floor line starting at the launch mark north of the barrier and the pad rings
    south-west.  If the floor markings look mirrored north-south, re-run with --flip-floor-v.

EXIT CODES
    0  written
    2  map file missing
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.course_map import CourseMap  # noqa: E402
from course.worldgen import write_world  # noqa: E402

DEFAULT_MAP = REPO_ROOT / "course" / "maps" / "course_v3_map.json"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP, help="course map JSON")
    ap.add_argument("--world", default=None, help="world name (default: the map file name without _map)")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "course" / "gazebo", help="output folder")
    ap.add_argument("--visible-nets", action="store_true", help="give the nets a faint visual (sensors see them)")
    ap.add_argument("--flip-floor-v", action="store_true", help="flip the floor texture north-south")
    ap.add_argument("--embed-systems", action="store_true",
                    help="list gz-sim's sensor systems in the world (for use without PX4's server.config)")
    args = ap.parse_args(argv)

    if not args.map.is_file():
        print("no map file {} (run course/12_extract_course_map.py first)".format(args.map), file=sys.stderr)
        return 2
    world = args.world or args.map.stem.replace("_map", "")
    result = write_world(CourseMap.load(args.map), args.out, world, args.visible_nets, args.flip_floor_v,
                         args.embed_systems)
    print("model: {}".format(result["model_dir"]))
    print("world: {}".format(result["world"]))
    for note in result["notes"]:
        print("note: " + note)
    return 0


if __name__ == "__main__":
    sys.exit(main())
