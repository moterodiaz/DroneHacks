"""The kit's flight interface plus the map-driven mission, for the stack's ``competitor`` window.

    COMPETITOR_PKG=dronehacks_autonomy  COMPETITOR_LAUNCH=competitor.launch.py      # /etc/skybeta/drone.env
    ros2 launch dronehacks_autonomy competitor.launch.py                            # by hand
    ros2 launch dronehacks_autonomy competitor.launch.py competitor:=false          # interface only

The window launches with no arguments, so two optional files in the team workspace configure it:

* ``/root/team_ws/course_map.yaml``: the surveyed map. Without it, the packaged published map.
* ``/root/team_ws/autonomy_params.yaml``: the measured values, written by ``measure_frame`` and by
  hand. Without them the mission refuses to start: it publishes nothing, the kit refuses Offboard,
  and the pilot keeps the vehicle.

``flight_interface`` is started exactly as the kit's competitor.launch.py starts it, with the same
arguments and defaults (competitor_guide.md section 5).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TEAM_MAP = "/root/team_ws/course_map.yaml"
TEAM_PARAMS = "/root/team_ws/autonomy_params.yaml"
CUVSLAM = "/visual_slam/tracking/odometry"

# The kit's arguments for flight_interface, with its defaults (competitor_example/launch/competitor.launch.py).
FLIGHT_INTERFACE_ARGUMENTS = [
    ("max_altitude_m", "3.0", "ceiling above the odometry origin, metres"),
    ("min_altitude_m", "0.3", "floor above the odometry origin, metres"),
    ("max_speed_mps", "1.5", "cap on the norm of a twist setpoint, m/s"),
    ("setpoint_rate_hz", "10.0", "OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz"),
    ("allow_mode_request", "false", "enable /skybeta/request_offboard. SIM ONLY"),
    ("auto_offboard_on_arm", "true", "enter Offboard once when the pilot arms, then switches to Position"),
    ("odometry_topic", CUVSLAM, "source of the ENU pose the altitude band is measured against"),
]


def choose_map(share: str) -> str:
    """The surveyed map if the team has placed one, else the packaged published map."""
    return TEAM_MAP if os.path.isfile(TEAM_MAP) else os.path.join(share, "maps", "course_v3_layout.yaml")


def mission_params(share: str, map_file) -> list:
    """Later entries win: packaged defaults, then the team's measured values, then the map."""
    files = [os.path.join(share, "config", "autonomy.example.yaml")]
    if os.path.isfile(TEAM_PARAMS):
        files.append(TEAM_PARAMS)
    return files + [{"map_file": map_file}]


def generate_launch_description():
    share = get_package_share_directory("dronehacks_autonomy")
    declared = [DeclareLaunchArgument(name, default_value=default, description=help_)
                for name, default, help_ in FLIGHT_INTERFACE_ARGUMENTS]
    declared += [
        DeclareLaunchArgument("map_file", default_value=choose_map(share),
                              description="course map, contracts/course.md format"),
        DeclareLaunchArgument("competitor", default_value="true", description="also start the mission node"),
    ]
    map_file = LaunchConfiguration("map_file")
    return LaunchDescription(declared + [
        LogInfo(msg=["dronehacks_autonomy: map ", map_file]),
        Node(
            package="skybeta_flight_interface",
            executable="flight_interface",
            name="flight_interface",
            output="screen",
            emulate_tty=True,
            parameters=[{name: LaunchConfiguration(name) for name, _, _ in FLIGHT_INTERFACE_ARGUMENTS}],
        ),
        Node(
            package="dronehacks_autonomy",
            executable="mission",
            name="mission",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("competitor")),
            parameters=mission_params(share, map_file),
        ),
    ])
