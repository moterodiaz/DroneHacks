"""Full-course launch: kit flight_interface + map-derived mission node.

Maze → Tunnel → Chicane (Gate B; the window stays refused while its
clearance budget is unreviewed) → hold over the pad; the pilot lands.

    ros2 launch dronehacks_manual_maze_tunnel_chicane_gateb_landing competitor.launch.py \
        course_to_odom_yaw:=<measured> course_to_odom_x:=<measured> course_to_odom_y:=<measured>
    ros2 launch dronehacks_manual_maze_tunnel_chicane_gateb_landing competitor.launch.py competitor:=false

The kit's flight-interface defaults are used except where a mission value is
needed: min_altitude_m stays at the kit's 0.3 m floor (contracts/px4.md:
"the default altitude floor is 0.3 m"; lowering it is needs-human issue #34),
and odometry stays on the kit's own default topic — the Guard is not in the
shipped data path (contracts/ros-graph.md; decision #25 point 5).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('max_altitude_m',       '3.0',   'kit ceiling above the odometry origin, metres'),
    ('max_speed_mps',        '1.0',   'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz',     '10.0',  'OffboardControlMode heartbeat rate'),
    ('allow_mode_request',   'false', 'enable /skybeta/request_offboard — SIM ONLY'),
    ('auto_offboard_on_arm', 'true',  'enter Offboard once when pilot arms then switches to Position'),
    # course_map_yaml empty → the packaged copy of the STEP map; pass the
    # surveyed map per contracts/course.md layout tolerance when available.
    ('course_map_yaml',      '',      'course map (contracts/course.md schema); empty = packaged copy'),
    # Required measured transform (contracts/frames.md); the mission node
    # fails closed until real measured values are passed.
    ('course_to_odom_yaw',   'nan',   'measured course→odom yaw, radians'),
    ('course_to_odom_x',     'nan',   'measured odom easting of the launch reference'),
    ('course_to_odom_y',     'nan',   'measured odom northing of the launch reference'),
    ('odometry_topic',       '/visual_slam/tracking/odometry', 'mission odometry source (kit default)'),
    ('hover_altitude_m',     '0.80',  'flight height in the odom frame, metres'),
    ('arrival_radius_m',     '0.12',  'XY arrival radius; must fit the course ±0.18 m error budget'),
    ('waypoint_timeout_s',   '45.0',  'per-waypoint timeout before hold+abort, seconds'),
    ('competitor',           'true',  'also start the mission node'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m':       LaunchConfiguration('max_altitude_m'),
            'max_speed_mps':        LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz':     LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request':   LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            # min_altitude_m is NOT overridden: the kit's 0.3 m floor stands
            # (contracts/px4.md; issue #34). odometry_topic is NOT overridden:
            # the kit consumes cuVSLAM directly (contracts/ros-graph.md).
        }],
    )

    mission = Node(
        package='dronehacks_manual_maze_tunnel_chicane_gateb_landing',
        executable='mission',
        name='dronehacks_full_mission',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'course_map_yaml':          LaunchConfiguration('course_map_yaml'),
            'course_to_odom_yaw':       LaunchConfiguration('course_to_odom_yaw'),
            'course_to_odom_x':         LaunchConfiguration('course_to_odom_x'),
            'course_to_odom_y':         LaunchConfiguration('course_to_odom_y'),
            'odometry_topic':           LaunchConfiguration('odometry_topic'),
            'hover_altitude_m':         LaunchConfiguration('hover_altitude_m'),
            'arrival_radius_m':        LaunchConfiguration('arrival_radius_m'),
            'waypoint_timeout_s':       LaunchConfiguration('waypoint_timeout_s'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, mission])
