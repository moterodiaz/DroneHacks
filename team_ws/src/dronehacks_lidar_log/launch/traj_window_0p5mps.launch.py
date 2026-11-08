"""dronehacks_traj's traj_nav_competitor_window_0p5mps.launch.py, unchanged and with no arguments (as the stack starts it), plus the lidar logger.

The logger only reads /skybeta/state and the lidar's port; it publishes nothing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    team = os.path.join(get_package_share_directory("dronehacks_traj"), "launch", "traj_nav_competitor_window_0p5mps.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(team)),
        Node(package="dronehacks_lidar_log", executable="lidar_logger", name="lidar_logger", output="screen"),
    ])
