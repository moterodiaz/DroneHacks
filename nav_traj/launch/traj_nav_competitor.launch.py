"""``traj_nav.launch.py`` with its arguments filled in, for the stack's ``competitor`` window.

For ``COMPETITOR_LAUNCH`` in ``/etc/skybeta/drone.env``, whose window launches with no arguments.
The kit reads only its own keys from drone.env and passes only those into the container
(``host/drone_env.sh``, ``host/run.sh``). A ``DRONEHACKS_*`` key there never arrives, so these defaults
are what the competitor window flies:

* the map: ``/root/team_ws/course_map.yaml`` (``~/team_ws/course_map.yaml`` on the drone). If the
  file is missing, the mission node exits with a configuration error and publishes nothing; the kit
  then refuses Offboard and the pilot keeps the vehicle;
* the Guard: ``dronehacks_vo``'s ``vo_guard``. Use ``traj_nav_competitor_raw.launch.py`` to fly on
  raw cuVSLAM instead.

``DRONEHACKS_MAP`` and ``DRONEHACKS_GUARD`` still override both when launching by hand in the container.
Nothing here reads the lidar.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

DEFAULT_MAP = "/root/team_ws/course_map.yaml"


def generate_launch_description(guard_default: str = "dronehacks_vo"):
    main = os.path.join(get_package_share_directory("dronehacks_traj"), "launch", "traj_nav.launch.py")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main),
        launch_arguments={
            "map_file": os.environ.get("DRONEHACKS_MAP", DEFAULT_MAP),
            "guard_package": os.environ.get("DRONEHACKS_GUARD", guard_default),
        }.items())])
