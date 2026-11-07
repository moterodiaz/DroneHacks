"""``traj_nav.launch.py`` with its arguments filled in, for the stack's ``competitor`` window.

For ``COMPETITOR_LAUNCH`` in ``/etc/skybeta/drone.env``, whose window launches with no arguments.
The kit reads only its own keys from drone.env and passes only those into the container
(``host/drone_env.sh``, ``host/run.sh``). A ``DRONEHACKS_*`` key there never arrives. So the competitor
window takes:

* the map: ``/root/team_ws/course_map.yaml`` (``~/team_ws/course_map.yaml`` on the drone) if that file
  exists, else the map generated from the STEP model and installed with the package. The launch
  log says which;
* the Guard: ``auto``, meaning ``dronehacks_vo``'s ``vo_guard`` if that package is installed, else
  raw cuVSLAM (with a warning). ``traj_nav_competitor_raw.launch.py`` always flies on raw cuVSLAM.

``DRONEHACKS_MAP`` and ``DRONEHACKS_GUARD`` override both when launching by hand in the container.
Nothing here reads the lidar.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

TEAM_MAP = "/root/team_ws/course_map.yaml"


def choose_map(share: str) -> str:
    """$DRONEHACKS_MAP, else the team's map if present, else the packaged STEP map."""
    if os.environ.get("DRONEHACKS_MAP"):
        return os.environ["DRONEHACKS_MAP"]
    if os.path.isfile(TEAM_MAP):
        return TEAM_MAP
    return os.path.join(share, "maps", "course_v3_step.yaml")


def generate_launch_description(guard_default: str = "auto"):
    share = get_package_share_directory("dronehacks_traj")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "traj_nav.launch.py")),
        launch_arguments={
            "map_file": choose_map(share),
            "guard_package": os.environ.get("DRONEHACKS_GUARD", guard_default),
        }.items())])
