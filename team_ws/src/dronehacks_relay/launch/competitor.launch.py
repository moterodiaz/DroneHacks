"""Launch the flight interface, the odometry guard relay, and the relay mission node.

Data flow:
  skybeta cuVSLAM → /visual_slam/tracking/odometry
                          ↓
                    odometry_guard (relay node)
                          ↓
              /visual_slam/tracking/odometry_mapped
                          ↓
                    dronehacks_relay_mission → /skybeta/setpoint/pose

    ros2 launch dronehacks_relay competitor.launch.py
    ros2 launch dronehacks_relay competitor.launch.py competitor:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('max_altitude_m',      '3.0',   'ceiling above the odometry origin, metres'),
    ('min_altitude_m',      '0.3',   'floor above the odometry origin, metres'),
    ('max_speed_mps',       '1.5',   'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz',    '10.0',  'OffboardControlMode heartbeat rate'),
    ('allow_mode_request',  'false', 'enable /skybeta/request_offboard — SIM ONLY'),
    ('auto_offboard_on_arm','true',  'enter Offboard once when pilot arms then switches to Position'),
    ('odometry_topic',      '/visual_slam/tracking/odometry',
     'raw ENU pose source for the altitude band (flight_interface always uses the raw topic)'),
    ('hover_altitude_m',    '1.5',   'hover height, metres'),
    ('hover_seconds',       '5.0',   'seconds to hold the hover'),
    ('square_side_m',       '5.0',   'side of the square, metres'),
    ('arrival_radius_m',    '0.3',   'how close counts as reaching a waypoint, metres'),
    ('min_brightness_dn',   '15.0',  'guard: withhold poses below this mean image brightness'),
    ('competitor',          'true',  'also start the relay and mission nodes'),
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
            'min_altitude_m':       LaunchConfiguration('min_altitude_m'),
            'max_speed_mps':        LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz':     LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request':   LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic':       LaunchConfiguration('odometry_topic'),
        }],
    )

    relay = Node(
        package='dronehacks_relay',
        executable='relay',
        name='odometry_guard',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'odom_in':          '/visual_slam/tracking/odometry',
            'odom_out':         '/visual_slam/tracking/odometry_mapped',
            'min_brightness_dn': LaunchConfiguration('min_brightness_dn'),
        }],
    )

    mission = Node(
        package='dronehacks_relay',
        executable='mission',
        name='dronehacks_relay_mission',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'odometry_topic':   '/visual_slam/tracking/odometry_mapped',
            'hover_altitude_m': LaunchConfiguration('hover_altitude_m'),
            'hover_seconds':    LaunchConfiguration('hover_seconds'),
            'square_side_m':    LaunchConfiguration('square_side_m'),
            'arrival_radius_m': LaunchConfiguration('arrival_radius_m'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, relay, mission])
