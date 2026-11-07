"""The kit's flight interface, started as the team's manual navigator starts it, and the course follower.

    ros2 launch dronehacks_course_follower competitor.launch.py \
        course_to_odom_yaw:=<measured> course_to_odom_x:=<measured> course_to_odom_y:=<measured>
    ros2 launch dronehacks_course_follower competitor.launch.py competitor:=false   # interface only

The follower refuses to start until the course-to-odom transform is passed (contracts/frames.md).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The flight interface's arguments, with the kit's defaults
INTERFACE_ARGUMENTS = [
    ("max_altitude_m", "3.0", "kit ceiling above the odometry origin, metres"),
    ("max_speed_mps", "1.0", "cap on the norm of a twist setpoint, m/s"),
    ("setpoint_rate_hz", "10.0", "the interface's TrajectorySetpoint and heartbeat rate"),
    ("allow_mode_request", "false", "enable /skybeta/request_offboard: SIM ONLY"),
    ("auto_offboard_on_arm", "true", "enter Offboard once when the pilot arms, then switches to Position"),
]

# The follower's arguments: name, default, type, description
FOLLOWER_ARGUMENTS = [
    ("course_map", "", str, "course map JSON; empty = the packaged one"),
    ("course_to_odom_yaw", "nan", float, "measured course-to-odom yaw, radians"),
    ("course_to_odom_x", "nan", float, "measured odom x of the course map's launch mark"),
    ("course_to_odom_y", "nan", float, "measured odom y of the course map's launch mark"),
    ("odom_origin_height_m", "0.0", float, "height of the odom origin above the floor, metres"),
    ("odometry_topic", "/visual_slam/tracking/odometry", str, "drone pose source, odom frame"),
    ("rate_hz", "50.0", float, "follower setpoint rate"),
    ("speed_mps", "1.0", float, "top speed along the route, m/s"),
    ("pose_lead", "true", bool, "lead the pose by the setpoint's velocity and acceleration"),
    ("allow_window", "true", bool, "fly the window when the map says it is passable"),
    ("land", "true", bool, "descend onto the pad by twist until PX4 disarms; false = hold over it"),
    ("tunnel_lead_m", "0.0", float, "fly the tunnel straight from this far outside its mouths; 0 = off"),
]


def generate_launch_description() -> LaunchDescription:
    """The interface, and the follower unless `competitor:=false`."""
    declared = [DeclareLaunchArgument(name, default_value=default, description=description)
                for name, default, description in INTERFACE_ARGUMENTS]
    declared += [DeclareLaunchArgument(name, default_value=default, description=description)
                 for name, default, _, description in FOLLOWER_ARGUMENTS]
    declared.append(DeclareLaunchArgument("competitor", default_value="true", description="also start the follower"))

    # min_altitude_m and odometry_topic stay at the kit's defaults, as in the manual navigator's launch
    flight_interface = Node(
        package="skybeta_flight_interface", executable="flight_interface", name="flight_interface",
        output="screen", emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in INTERFACE_ARGUMENTS}])

    follower = Node(
        package="dronehacks_course_follower", executable="follower", name="course_follower",
        output="screen", emulate_tty=True, condition=IfCondition(LaunchConfiguration("competitor")),
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                     for name, _, value_type, _ in FOLLOWER_ARGUMENTS}])

    return LaunchDescription([*declared, flight_interface, follower])
