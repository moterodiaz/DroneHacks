"""Launch the kit's flight interface and the DroneHacks lidar nodes.

    ros2 launch dronehacks_lidar lidar_nav.launch.py lidar_policy:=allowed
    ros2 launch dronehacks_lidar lidar_nav.launch.py lidar_policy:=forbidden route:=gate_b

``lidar_policy`` has no default: without it the launch fails and nothing starts, not even the
flight interface, so the vehicle stays in Position under the pilot. For the stack's
``competitor`` window, which launches with no arguments, use one of the four variants
(``lidar_nav_{allowed,forbidden}[_gate_b].launch.py``, ``dronehacks_lidar/launch_variants.py``)
as ``COMPETITOR_LAUNCH`` (docs/deploy.md).

The route: ``route`` picks a packaged one, planned from the STEP-model map through the
ground-truth checkpoints (``share/dronehacks_lidar/routes/course_v3_<route>.json``). ``window``
(the default: CP4 through C2a's window, +5 bonus) or ``gate_b``. ``route_file:=`` overrides it with
any ``dronehacks-lidar-route/1`` file.

Started:

* ``flight_interface``: the kit's node, **exactly as** ``competitor_example/launch/competitor.launch.py``
  starts it. Same arguments and defaults; ``auto_offboard_on_arm`` true; ``allow_mode_request``
  false (SIM ONLY). It owns the Offboard heartbeat and the safety envelope.
* ``vo_guard``: always.
* ``lidar_localizer``: only when ``lidar_policy`` is ``allowed``.
* ``pose_fusion``: always. The follower's setpoints reach the flight interface only through it.
* ``follower``: always.

Parameters come from ``config/pose_fusion.example.yaml`` (installed with the package; override
with ``params_file:=``). Its lidar mount is the one from past tests. The map, rules and route
paths are set here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

SHARE = get_package_share_directory("dronehacks_lidar")

# The kit's arguments for flight_interface, with its defaults (competitor.launch.py)
FLIGHT_INTERFACE_ARGUMENTS = [
    ("max_altitude_m", "3.0", "ceiling above the odometry origin, metres"),
    ("min_altitude_m", "0.3", "floor above the odometry origin, metres"),
    ("max_speed_mps", "1.5", "cap on the norm of a twist setpoint, m/s"),
    ("setpoint_rate_hz", "10.0", "OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz"),
    ("allow_mode_request", "false", "enable /skybeta/request_offboard. SIM ONLY"),
    ("auto_offboard_on_arm", "true", "enter Offboard once when the pilot arms, then switches to Position"),
    ("odometry_topic", "/visual_slam/tracking/odometry", "source of the ENU pose the altitude band is measured against"),
]


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=help_)
                for name, default, help_ in FLIGHT_INTERFACE_ARGUMENTS]
    declared += [
        DeclareLaunchArgument("lidar_policy", description="'allowed' or 'forbidden' for this run (no default)"),
        DeclareLaunchArgument("route", default_value="window", description="packaged route: window or gate_b"),
        DeclareLaunchArgument("route_file", default_value=PathJoinSubstitution(
            [SHARE, "routes", ["course_v3_", LaunchConfiguration("route"), ".json"]]),
            description="a dronehacks-lidar-route/1 file (default: the packaged route)"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(SHARE, "config", "pose_fusion.example.yaml"),
                              description="parameters for the four nodes"),
    ]
    params = LaunchConfiguration("params_file")
    policy = LaunchConfiguration("lidar_policy")
    course_files = {"map_file": os.path.join(SHARE, "maps", "course_v3_map.json"),
                    "rules_file": os.path.join(SHARE, "maps", "course_v3_rules.json")}

    flight_interface = Node(
        package="skybeta_flight_interface",
        executable="flight_interface",
        name="flight_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "max_altitude_m": LaunchConfiguration("max_altitude_m"),
            "min_altitude_m": LaunchConfiguration("min_altitude_m"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
            "setpoint_rate_hz": LaunchConfiguration("setpoint_rate_hz"),
            "allow_mode_request": LaunchConfiguration("allow_mode_request"),
            "auto_offboard_on_arm": LaunchConfiguration("auto_offboard_on_arm"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
        }],
    )
    vo_guard = Node(package="dronehacks_lidar", executable="vo_guard", name="vo_guard", output="screen",
                    parameters=[params])
    lidar_localizer = Node(
        package="dronehacks_lidar", executable="lidar_localizer", name="lidar_localizer", output="screen",
        parameters=[params, course_files, {"lidar_policy": policy}],
        condition=IfCondition(PythonExpression(["'", policy, "' == 'allowed'"])),
    )
    pose_fusion = Node(package="dronehacks_lidar", executable="pose_fusion", name="pose_fusion", output="screen",
                       parameters=[params, course_files, {"lidar_policy": policy}])
    follower = Node(package="dronehacks_lidar", executable="follower", name="follower", output="screen",
                    parameters=[params, course_files, {"route_file": LaunchConfiguration("route_file")}])
    return LaunchDescription(declared + [flight_interface, vo_guard, lidar_localizer, pose_fusion, follower])
