"""Staged bring-up test 2 (see BRINGUP.md): pathfly.launch.py stage:=2.
For COMPETITOR_LAUNCH, which cannot pass launch arguments."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    main = os.path.join(get_package_share_directory('dronehacks_pathfly'),
                        'launch', 'pathfly.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main),
        launch_arguments={'stage': '2'}.items())])
