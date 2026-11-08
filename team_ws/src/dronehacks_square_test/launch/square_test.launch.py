"""The kit's flight interface (unchanged settings) plus the cage square test.

For COMPETITOR_LAUNCH=square_test.launch.py with COMPETITOR_PKG=dronehacks_square_test.
Tune from the command line, e.g. ``ros2 launch dronehacks_square_test square_test.launch.py side_m:=0.5``.
``record:=false`` turns off the node's recording (it records from arming; see square_test_node).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INTERFACE = [  # the kit example's own defaults
    ('max_altitude_m', '3.0'), ('min_altitude_m', '0.3'), ('max_speed_mps', '1.5'),
    ('setpoint_rate_hz', '10.0'), ('allow_mode_request', 'false'), ('auto_offboard_on_arm', 'true'),
    ('odometry_topic', '/visual_slam/tracking/odometry'),
]
MISSION = [('side_m', '1.0'), ('height_m', '0.73'), ('speed_mps', '0.3'), ('hold_s', '2.0'), ('ground_s', '1.0'),
           ('yaw_rate_dps', '20.0'), ('record', 'true')]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in INTERFACE + MISSION]
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in INTERFACE}])
    mission = Node(
        package='dronehacks_square_test', executable='square_test', name='square_test',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in MISSION}])
    return LaunchDescription(args + [interface, mission])
