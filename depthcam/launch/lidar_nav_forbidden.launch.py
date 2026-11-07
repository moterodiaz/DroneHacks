"""Lidar forbidden, window route. For COMPETITOR_LAUNCH (see dronehacks_lidar/launch_variants.py).

The lidar localiser is not started; pose_fusion passes the Guard's poses and the follower's
setpoints through unchanged.
"""

from dronehacks_lidar.launch_variants import include


def generate_launch_description():
    return include("forbidden", "window")
