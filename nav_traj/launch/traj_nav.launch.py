"""Launch the kit's flight interface, the Guard and the mission, without the 360 lidar.

    ros2 launch dronehacks_traj traj_nav.launch.py                                   # the packaged STEP map
    ros2 launch dronehacks_traj traj_nav.launch.py map_file:=/root/team_ws/course_map.yaml
    ros2 launch dronehacks_traj traj_nav.launch.py guard_package:=auto               # the Guard, if installed

``map_file`` defaults to the map generated from the organisers' STEP model and
installed with the package (``maps/course_v3_step.yaml``; it is not
surveyed). The launch logs which map it uses. If the mission node can't plan
on it, it exits and publishes nothing: the kit then refuses Offboard and the
pilot keeps the vehicle. For the stack's ``competitor`` window use
``traj_nav_competitor.launch.py`` as ``COMPETITOR_LAUNCH`` (docs/deploy.md).

Started:

* ``flight_interface``: the kit's node, **exactly as**
  ``competitor_example/launch/competitor.launch.py`` starts it, with the same
  arguments and defaults. ``auto_offboard_on_arm`` is true;
  ``allow_mode_request`` is false (SIM ONLY). It owns the Offboard heartbeat
  and the safety envelope.
* ``vo_guard``: from ``guard_package``. It withholds cuVSLAM poses that
  can't be trusted. Both no-lidar variants then run the same Guard and
  differ only in the follower.
  * ``none`` (the default): no Guard; the mission reads cuVSLAM directly
    (contracts/ros-graph.md: cuVSLAM direct is the contracted path).
  * ``auto``: ``dronehacks_vo``'s Guard (the lidar-free build on branch
    ``core/vo-nav-no-lidar``) if that package is installed; otherwise none,
    with a warning in the log.
  * A package name: that package's ``vo_guard``. If it isn't installed, the
    launch fails before anything starts.
* ``mission``: plans from the map, then flies takeoff, the route with this
  repo's Follower, the hold and the landing. Its setpoints go straight to the
  flight interface.

No lidar node is started and nothing here reads a lidar.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("dronehacks_traj")
CUVSLAM = "/visual_slam/tracking/odometry"
GUARDED = "/vo_guard/odometry"
STEP_MAP = os.path.join(SHARE, "maps", "course_v3_step.yaml")
AUTO_GUARD = "dronehacks_vo"

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


def _installed(package: str) -> bool:
    try:
        get_package_prefix(package)
        return True
    except Exception:  # noqa: BLE001 - PackageNotFoundError, whatever its module
        return False


def _nodes(context):
    guard_package = LaunchConfiguration("guard_package").perform(context).strip()
    params = LaunchConfiguration("params_file").perform(context)
    overrides = LaunchConfiguration("mission_overrides").perform(context).strip()
    map_file = LaunchConfiguration("map_file").perform(context)
    actions = [LogInfo(msg=f"dronehacks_traj: map {map_file}")]
    if overrides:
        actions.append(LogInfo(msg=f"dronehacks_traj: mission parameters overridden by {overrides}"))
    if guard_package == "auto":
        if _installed(AUTO_GUARD):
            guard_package = AUTO_GUARD
        else:
            guard_package = "none"
            actions.append(LogInfo(msg=f"dronehacks_traj: WARNING {AUTO_GUARD} is not installed, so there is no "
                                       "Guard: the mission flies on raw cuVSLAM odometry"))
    if guard_package and guard_package != "none":
        get_package_prefix(guard_package)   # raises, and so aborts the launch, if a named package isn't installed
        actions.append(Node(package=guard_package, executable="vo_guard", name="vo_guard", output="screen",
                            parameters=[{"odom_in": CUVSLAM, "output_topic": GUARDED}]))
        actions.append(LogInfo(msg=f"dronehacks_traj: Guard {guard_package}/vo_guard -> {GUARDED}"))
        pose_topic = GUARDED
    else:
        pose_topic = CUVSLAM
    actions.append(Node(package="dronehacks_traj", executable="mission", name="mission", output="screen",
                        emulate_tty=True,
                        parameters=[params] + ([overrides] if overrides else [])
                        + [{"map_file": map_file, "odometry_topic": pose_topic}]))
    return actions


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=help_)
                for name, default, help_ in FLIGHT_INTERFACE_ARGUMENTS]
    declared += [
        DeclareLaunchArgument("map_file", default_value=STEP_MAP,
                              description="the course map, contracts/course.md format (default: from the STEP model)"),
        DeclareLaunchArgument("guard_package", default_value="none",
                              description="'none' (raw cuVSLAM, the default), 'auto' (dronehacks_vo if "
                                          "installed) or a package providing vo_guard"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(SHARE, "config", "traj_nav.example.yaml"),
                              description="parameters for the mission node"),
        DeclareLaunchArgument("mission_overrides", default_value="",
                              description="optional second parameter file for the mission node, applied over "
                                          "params_file (a flight line, e.g. config/flight_line_window_2mps.yaml)"),
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
