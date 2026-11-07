"""``traj_nav_competitor.launch.py`` without the Guard: the mission flies on raw cuVSLAM odometry.

For ``COMPETITOR_LAUNCH``. The map is chosen as in ``traj_nav_competitor.launch.py``: the team's
``/root/team_ws/course_map.yaml`` if present, else the packaged STEP map. Nothing here reads the lidar.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

TEAM_MAP = "/root/team_ws/course_map.yaml"


def generate_launch_description():
    share = get_package_share_directory("dronehacks_traj")
    map_file = os.environ.get("DRONEHACKS_MAP") or (
        TEAM_MAP if os.path.isfile(TEAM_MAP) else os.path.join(share, "maps", "course_v3_step.yaml"))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "traj_nav.launch.py")),
        launch_arguments={"map_file": map_file, "guard_package": "none"}.items())])
