"""``traj_nav_competitor.launch.py`` flying the "window at 0.5 m/s" (slow test) flight line.

For ``COMPETITOR_LAUNCH`` in ``/etc/skybeta/drone.env`` (the competitor window launches with no
arguments). Everything is as in ``traj_nav_competitor.launch.py`` -- the same map precedence, raw
cuVSLAM, the installed ``config/traj_nav.example.yaml`` with its measured values -- plus
``config/flight_line_window_0p5mps.yaml`` applied over the mission's parameters: the window route,
track speed 0.5 m/s, the plan's speed limit 0.5 m/s, and the twist cap 0.5 m/s. The kit's flight
interface gets its documented ``max_speed_mps`` launch argument from the same file, so its twist
clamp matches; the interface is not modified. The line flies the course as the team taped it
on 2026-11-08 (``maps/course_arena_2026-11-08.yaml``, LINE_MAP) at the window's taped middle,
over the boulders (``avoid_boulders`` false), unless ``$DRONEHACKS_MAP`` or the team's map is given.

This is ``traj_nav_competitor_window_2mps.launch.py`` with every speed at 0.5 m/s instead of 2.0:
a slow test of the same route, leaving the pilot time to take over before a touch. PX4's
MPC_XY_VEL_MAX (contracts/px4.md: the organisers' 1.0 m/s, not ours to write) is above 0.5, so it
doesn't bind. See the YAML file's header for the dark tunnel at this speed.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

TEAM_MAP = "/root/team_ws/course_map.yaml"
#: The course as the team taped it on 2026-11-08 (maps/README.md), not the default launch's STEP map.
LINE_MAP = "course_arena_2026-11-08.yaml"
LINE = "flight_line_window_0p5mps.yaml"


def choose_map(share: str) -> str:
    """$DRONEHACKS_MAP, else the team's map if present, else the packaged taped map (LINE_MAP)."""
    if os.environ.get("DRONEHACKS_MAP"):
        return os.environ["DRONEHACKS_MAP"]
    if os.path.isfile(TEAM_MAP):
        return TEAM_MAP
    return os.path.join(share, "maps", LINE_MAP)


def generate_launch_description():
    share = get_package_share_directory("dronehacks_traj")
    line = os.path.join(share, "config", LINE)
    with open(line, encoding="utf-8") as fh:
        interface = yaml.safe_load(fh)["flight_interface"]["ros__parameters"]
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "traj_nav.launch.py")),
        launch_arguments={
            "map_file": choose_map(share),
            "guard_package": os.environ.get("DRONEHACKS_GUARD", "none"),
            "mission_overrides": line,
            "max_speed_mps": str(float(interface["max_speed_mps"])),
        }.items())])
