"""Plans the course map's route and flies the mission against an ideal drone, without ROS: prints
the route, the first setpoints as the node would publish them in the odom frame, and how long the
run takes to land. The ideal drone sits on the un-led setpoint and turns as commanded, so the times
are the follower's own, not a flight's.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from .follower import Drone, HeadingMode
from .frames import CourseToOdom
from .mission import (DEFAULT_RETURN_HEIGHT_M, DEFAULT_RETURN_REST_S, DEFAULT_RETURN_SPEED_MPS, Mission, Phase,
                      PoseCommand, ReturnPlan, TwistCommand)
from .planner import load_course_map, plan_flight

# This package's name, under which the course map is installed, and the map's file in the source tree
PACKAGE = "dronehacks_course_follower"
SOURCE_MAP = Path(__file__).resolve().parents[1] / "maps" / "course_map.json"

# The ideal drone gives up after this long, so a stuck follower ends the run
_RUN_LIMIT_S = 180.0

# With the return, PX4 disarms this long after it reports landed: COM_DISARM_LAND as raised on skybeta-11
_COM_DISARM_LAND_S = 5.0


def packaged_map() -> Path:
    """The course map installed with the package, or the source tree's where it is not installed."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return SOURCE_MAP

    try:
        return Path(get_package_share_directory(PACKAGE)) / "maps" / "course_map.json"
    except PackageNotFoundError:
        return SOURCE_MAP


def build_parser() -> argparse.ArgumentParser:
    """The command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=None, help="course map JSON (default: the packaged one)")
    parser.add_argument("--speed", type=float, default=1.0, metavar="MPS", help="top speed (default 1.0)")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="setpoint rate (default 50)")
    parser.add_argument("--steps", type=int, default=20, help="setpoints to print (default 20)")
    parser.add_argument("--no-window", action="store_true", help="never fly through the window")
    parser.add_argument("--no-lead", action="store_true", help="send the plain setpoint, not the led pose")
    parser.add_argument("--heading-mode", choices=[mode.value for mode in HeadingMode], default=HeadingMode.AHEAD.value,
                        help="face the point ahead on the route, or hold the heading at Offboard (default ahead)")
    parser.add_argument("--return-home", action="store_true",
                        help="after the pad landing, return to the launch mark (PX4 disarms 5 s after landing)")
    parser.add_argument("--tunnel-lead-m", type=float, default=0.0, metavar="M",
                        help="fly each two-ended goal straight, from this far outside its mouths")
    parser.add_argument("--course-to-odom", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("YAW", "X", "Y"),
                        help="course-to-odom yaw (rad) and odom x, y of the launch mark (default: aligned, at 0, 0)")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Plans, prints the route, then flies the mission against the ideal drone."""
    args = build_parser().parse_args(argv)
    if args.speed <= 0 or args.rate_hz <= 0:
        raise SystemExit(f"speed and rate must be positive, not {args.speed} and {args.rate_hz}")

    course_map = load_course_map(args.map or packaged_map())
    planned = plan_flight(course_map, allow_window=not args.no_window, tunnel_lead_m=args.tunnel_lead_m)
    launch = (float(course_map["launch"][0]), float(course_map["launch"][1]))
    yaw, odom_x, odom_y = args.course_to_odom
    transform = CourseToOdom(yaw_rad=yaw, odom_origin=(odom_x, odom_y), course_origin=launch, origin_height_m=0.0)

    print_route(planned.route, planned.window_flown, course_map["order"])
    return_plan = None
    if args.return_home:
        return_plan = ReturnPlan(height_m=DEFAULT_RETURN_HEIGHT_M, rest_s=DEFAULT_RETURN_REST_S,
                                 speed_mps=DEFAULT_RETURN_SPEED_MPS)
    mission = Mission(planned.route, planned.walls, float(course_map["height_m"]), launch, args.speed,
                      lead=not args.no_lead, heading_mode=HeadingMode(args.heading_mode), return_plan=return_plan)
    disarm_after_s = _COM_DISARM_LAND_S if args.return_home else 0.0
    drone = fly_ideal(mission, launch, math.radians(float(course_map["launch"][2])), transform, args.rate_hz,
                      args.steps, disarm_after_s)
    if args.return_home:
        print(f"landed {math.dist((drone.east, drone.north), launch):.3f} m from the launch mark")
    return 0 if mission.phase is Phase.DONE else 1


def print_route(route: list[tuple[float, float]], window_flown: bool, order: list[str]) -> None:
    """The planned waypoints in the course frame."""
    print(f"goal order {order}; window {'flown' if window_flown else 'closed'}; {len(route)} waypoints (east, north):")
    for east, north in route:
        print(f"  {east:+.2f} {north:+.2f}")


def fly_ideal(mission: Mission, launch: tuple[float, float], yaw_rad: float, transform: CourseToOdom,
              rate_hz: float, steps: int, disarm_after_s: float = 0.0) -> Drone:
    """Runs the mission at `rate_hz` against a drone that is wherever the reference is, printing
    the first `steps` commands as the node sends them and each phase change; PX4 disarms the drone
    `disarm_after_s` after it lands. Returns where the drone ends."""
    period_s = 1.0 / rate_hz
    drone = Drone(launch[0], launch[1], 0.0, yaw_rad)
    seen_events, printed, now_s = 0, 0, 0.0
    flown, landed_since_s = False, math.nan

    while now_s < _RUN_LIMIT_S and mission.phase is not Phase.DONE:
        # PX4's land detector and its disarm COM_DISARM_LAND after it, once the drone has flown
        landed = flown and drone.height_m <= 0.0
        if not landed:
            landed_since_s = math.nan
        elif math.isnan(landed_since_s):
            landed_since_s = now_s
        armed = not landed or now_s - landed_since_s < disarm_after_s

        command = mission.tick(now_s, offboard=True, armed=armed, drone=drone, landed=landed)
        for event in mission.events[seen_events:]:
            print(f"{now_s:6.2f} s  {event}")
        seen_events = len(mission.events)

        if command is not None and printed < steps:
            print(f"{now_s:6.2f} s  {mission.phase.value:8s} {_odom_text(command, transform)}")
            printed += 1

        drone = _ideal_drone(mission, command, drone, period_s)
        flown = flown or drone.height_m > 0.0
        now_s += period_s

    if mission.phase is not Phase.DONE:
        print(f"no landing within {_RUN_LIMIT_S:g} s: the mission is in {mission.phase.value}")
    return drone


def _odom_text(command: PoseCommand | TwistCommand, transform: CourseToOdom) -> str:
    """A command as the node publishes it, in the odom frame."""
    if isinstance(command, TwistCommand):
        x, y = transform.vector_to_odom(command.east_mps, command.north_mps)
        return f"twist ({x:+.2f}, {y:+.2f}, {command.up_mps:+.2f}) m/s"

    x, y = transform.point_to_odom(command.east, command.north)
    heading = "hold" if math.isnan(command.yaw_rad) else f"{math.degrees(transform.yaw_to_odom(command.yaw_rad)):+.1f} deg"
    return f"pose ({x:+.3f}, {y:+.3f}, {transform.height_to_odom(command.height_m):+.2f}) heading {heading}"


def _ideal_drone(mission: Mission, command: PoseCommand | TwistCommand | None, drone: Drone, period_s: float) -> Drone:
    """Where the ideal drone is one period on: up at the commanded height, on the follower's
    reference facing its heading, on the return's pose, or sinking with the twist, landed at the floor."""
    if command is None:
        return drone

    if isinstance(command, TwistCommand):
        return Drone(drone.east + command.east_mps * period_s, drone.north + command.north_mps * period_s,
                     max(drone.height_m + command.up_mps * period_s, 0.0), drone.yaw_rad)

    if mission.phase in (Phase.RETURN_CLIMB, Phase.RETURN_FLY):
        yaw_rad = drone.yaw_rad if math.isnan(command.yaw_rad) else command.yaw_rad
        return Drone(command.east, command.north, command.height_m, yaw_rad)

    if mission.follower is not None:
        east, north = mission.follower.reference()
        return Drone(east, north, mission.height_m, mission.follower.yaw_rad)

    if command is not None:
        return Drone(drone.east, drone.north, command.height_m, drone.yaw_rad)

    return drone


if __name__ == "__main__":
    raise SystemExit(main())
