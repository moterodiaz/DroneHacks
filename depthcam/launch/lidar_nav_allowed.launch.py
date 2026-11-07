"""Lidar allowed, window route. For COMPETITOR_LAUNCH (see dronehacks_lidar/launch_variants.py)."""

from dronehacks_lidar.launch_variants import include


def generate_launch_description():
    return include("allowed", "window")
