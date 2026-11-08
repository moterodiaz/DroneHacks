"""Fly a whole mission offline, from arming to disarm, before trusting it with the drone.

    python -m dronehacks_traj.dry_run MAP.yaml --rest-height 0.07 [--v-max 1.0] [--px4-yaw-offset-deg 0]
    ros2 run dronehacks_traj dry_run MAP.yaml --rest-height 0.07

The mission is built exactly as the ROS node builds it (``factory.prepare``)
and flown in ``vehicle_sim``: the kit's interface and Offboard rule, and a
stand-in PX4 with drone 1's parameters. One difference from the node: the
dry run flies the *model's* odom frame, which starts at the launch pose
(``frames.model_odom_frame``); the node's frame is measured
(``course_to_odom_*`` parameters, contracts/frames.md). It prints the state
timeline and the checks: wall touches, openings crossed whole-drone, and the
landing on the pad. Exit status 0 only if the mission reached STOPPED after
landing, untouched, through every opening and on the pad. Model output, not a
flight.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys

from dronehacks_planner.course_map import load_map
from dronehacks_planner.limits import VehicleLimits

from .factory import flight_checks, prepare
from .frames import model_odom_frame
from .mission import MissionConfig
from .vehicle_sim import VehicleSimConfig, fly, summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="dry_run", description=__doc__.splitlines()[0])
    ap.add_argument("map")
    ap.add_argument("--rest-height", type=float, required=True,
                    help="body centre above the floor when landed, m (frames.md: 0.07; measure it)")
    ap.add_argument("--v-max", type=float, default=VehicleLimits().v_max_mps,
                    help="MPC_XY_VEL_MAX on the drone (read it off the drone)")
    ap.add_argument("--px4-yaw-offset-deg", type=float, default=0.0,
                    help="PX4's heading minus vision's at the start, for the heading gate")
    args = ap.parse_args(argv)

    limits = dataclasses.replace(VehicleLimits(), v_max_mps=args.v_max)
    try:
        cmap = load_map(args.map)
        if cmap.launch is None or cmap.launch.heading is None:
            raise ValueError("the map has no launch position and heading, so the model's odom frame "
                             "(which starts at the launch pose) is undefined")
        frame = model_odom_frame(cmap.launch.position, cmap.launch.heading)
        # land: the default holds over the pad for the pilot (px4.md); the dry run exercises
        # the landing too, so it opts in explicitly.
        prepared = prepare(cmap, MissionConfig(rest_height_m=args.rest_height, land=True), limits, frame=frame)
    except Exception as exc:  # noqa: BLE001 - report any planning failure and fail the run
        print(f"dry_run: could not build the mission: {exc}", file=sys.stderr)
        return 2
    plan = prepared.plan
    print(f"route {' > '.join(plan.route.goals)}: {plan.path.length_m:.2f} m, planned {plan.duration:.2f} s "
          f"(built in {prepared.seconds:.2f} s); alternatives: "
          + "; ".join(f"{c}: {o}" for c, o in prepared.alternatives))
    log = fly(prepared.mission, limits,
              VehicleSimConfig(px4_yaw_offset_rad=math.radians(args.px4_yaw_offset_deg)))
    result = summary(log)
    result.update(flight_checks(prepared, log.pos))
    print(json.dumps(result, indent=2))
    ok = (log.final_state == "STOPPED" and "disarm" in log.events and result["wall_touches"] == 0
          and all(result["openings"].values()) and result["on_pad"])
    print("dry run: PASS" if ok else "dry run: FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
