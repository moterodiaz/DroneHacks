"""``traj_nav_competitor.launch.py`` without the Guard: the mission flies on raw cuVSLAM odometry.

For ``COMPETITOR_LAUNCH`` when ``dronehacks_vo`` (which provides ``vo_guard``) is not installed in the
team workspace. Map: ``/root/team_ws/course_map.yaml``. Nothing here reads the lidar.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    main = os.path.join(get_package_share_directory("dronehacks_traj"), "launch", "traj_nav.launch.py")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main),
        launch_arguments={
            "map_file": os.environ.get("DRONEHACKS_MAP", "/root/team_ws/course_map.yaml"),
            "guard_package": "none",
        }.items())])
