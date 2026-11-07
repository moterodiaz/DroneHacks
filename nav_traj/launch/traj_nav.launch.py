"""Launch the kit's flight interface, the Guard and the mission, without the 360 lidar.

    ros2 launch dronehacks_traj traj_nav.launch.py map_file:=/root/team_ws/course_map.yaml
    ros2 launch dronehacks_traj traj_nav.launch.py map_file:=... guard_package:=none   # raw cuVSLAM

``map_file`` has no default. Without it the launch fails and nothing starts,
not even the flight interface, so the vehicle stays in Position under the
pilot. For the stack's ``competitor`` window, which launches with no
arguments, use ``traj_nav_competitor.launch.py`` as ``COMPETITOR_LAUNCH``
(docs/deploy.md).

Started:

* ``flight_interface``: the kit's node, **exactly as**
  ``competitor_example/launch/competitor.launch.py`` starts it, with the same
  arguments and defaults. ``auto_offboard_on_arm`` is true;
  ``allow_mode_request`` is false (SIM ONLY). It owns the Offboard heartbeat
  and the safety envelope.
* ``vo_guard``: from ``guard_package`` (default ``dronehacks_vo``, the
  lidar-free build on branch ``core/vo-nav-no-lidar``). It withholds cuVSLAM
  poses that can't be trusted. Both no-lidar variants then run the same
  Guard and differ only in the follower. ``guard_package:=none`` skips it,
  and the mission reads cuVSLAM directly. If the named package isn't
  installed, the launch fails before anything starts.
* ``mission``: plans from the map, then flies takeoff, the route with this
  repo's Follower, the hold and the landing. Its setpoints go straight to the
  flight interface.

No lidar node is started and nothing here reads a lidar.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("dronehacks_traj")
CUVSLAM = "/visual_slam/tracking/odometry"
GUARDED = "/vo_guard/odometry"

# The kit's arguments for flight_interface, with its defaults (competitor.launch.py)
FLIGHT_INTERFACE_ARGUMENTS = [
    ("max_altitude_m", "3.0", "ceiling above the odometry origin, metres"),
    ("min_altitude_m", "0.3", "floor above the odometry origin, metres"),
    ("max_speed_mps", "1.5", "cap on the norm of a twist setpoint, m/s"),
    ("setpoint_rate_hz", "10.0", "OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz"),
    ("allow_mode_request", "false", "enable /skybeta/request_offboard. SIM ONLY"),
    ("auto_offboard_on_arm", "true", "enter Offboard once when the pilot arms, then switches to Position"),
    ("odometry_topic", CUVSLAM, "source of the ENU pose the altitude band is measured against"),
]


def _nodes(context):
    guard_package = LaunchConfiguration("guard_package").perform(context).strip()
    params = LaunchConfiguration("params_file").perform(context)
    nodes = []
    if guard_package and guard_package != "none":
        get_package_prefix(guard_package)   # raises, and so aborts the launch, if it isn't installed
        nodes.append(Node(package=guard_package, executable="vo_guard", name="vo_guard", output="screen",
                          parameters=[{"odom_in": CUVSLAM, "output_topic": GUARDED}]))
        pose_topic = GUARDED
    else:
        pose_topic = CUVSLAM
    nodes.append(Node(package="dronehacks_traj", executable="mission", name="mission", output="screen",
                      emulate_tty=True,
                      parameters=[params, {"map_file": LaunchConfiguration("map_file").perform(context),
                                           "odometry_topic": pose_topic}]))
    return nodes


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=help_)
                for name, default, help_ in FLIGHT_INTERFACE_ARGUMENTS]
    declared += [
        DeclareLaunchArgument("map_file", description="the course map, contracts/course.md format (no default)"),
        DeclareLaunchArgument("guard_package", default_value="dronehacks_vo",
                              description="package providing vo_guard, or 'none' to fly on raw cuVSLAM"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(SHARE, "config", "traj_nav.example.yaml"),
                              description="parameters for the mission node"),
    ]
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
    return LaunchDescription(declared + [flight_interface, OpaqueFunction(function=_nodes)])
