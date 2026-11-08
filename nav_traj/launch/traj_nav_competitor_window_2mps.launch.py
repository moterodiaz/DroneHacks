"""``traj_nav_competitor.launch.py`` flying the "window at 2 m/s" flight line.

For ``COMPETITOR_LAUNCH`` in ``/etc/skybeta/drone.env`` (the competitor window launches with no
arguments). Everything is as in ``traj_nav_competitor.launch.py`` -- the same map choice, raw
cuVSLAM, the installed ``config/traj_nav.example.yaml`` with its measured values -- plus
``config/flight_line_window_2mps.yaml`` applied over the mission's parameters: the window route,
track speed 2 m/s, the plan's speed limit 2 m/s, and the twist cap 2 m/s. The kit's flight
interface gets its documented ``max_speed_mps`` launch argument from the same file, so its twist
clamp matches; the interface is not modified.

PX4's own MPC_XY_VEL_MAX still caps horizontal speed (contracts/px4.md: the organisers' 1.0 m/s,
not ours to write). Read the live value before flying this line; see the YAML file's header.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

TEAM_MAP = "/root/team_ws/course_map.yaml"
LINE = "flight_line_window_2mps.yaml"


def choose_map(share: str) -> str:
    """$DRONEHACKS_MAP, else the team's map if present, else the packaged STEP map."""
    if os.environ.get("DRONEHACKS_MAP"):
        return os.environ["DRONEHACKS_MAP"]
    if os.path.isfile(TEAM_MAP):
        return TEAM_MAP
    return os.path.join(share, "maps", "course_v3_step.yaml")


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
