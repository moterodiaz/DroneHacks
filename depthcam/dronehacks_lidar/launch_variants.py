"""The launch variants for the stack's ``competitor`` window, which launches with no arguments.

The kit passes only ``COMPETITOR_PKG`` and ``COMPETITOR_LAUNCH`` from ``/etc/skybeta/drone.env``
into the container (``host/run.sh``, ``host/drone_env.sh``), so a run's lidar policy and route are
chosen by the launch file's *name*:

    lidar_nav_allowed.launch.py           lidar allowed,   window route
    lidar_nav_allowed_gate_b.launch.py    lidar allowed,   Gate B route
    lidar_nav_forbidden.launch.py         lidar forbidden, window route
    lidar_nav_forbidden_gate_b.launch.py  lidar forbidden, Gate B route

Each includes ``lidar_nav.launch.py`` with those two arguments set.
"""

from __future__ import annotations

import os

POLICIES = ("allowed", "forbidden")
ROUTES = ("window", "gate_b")


def include(policy: str, route: str):
    """``lidar_nav.launch.py`` with ``lidar_policy`` and ``route`` set."""
    if policy not in POLICIES or route not in ROUTES:
        raise ValueError("policy must be one of {} and route one of {}".format(POLICIES, ROUTES))
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    main = os.path.join(get_package_share_directory("dronehacks_lidar"), "launch", "lidar_nav.launch.py")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main), launch_arguments={"lidar_policy": policy, "route": route}.items())])
