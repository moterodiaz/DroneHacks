"""Full-course launch: flight_interface + full-course mission node.

Maze → Tunnel → Chicane (C2a window, +5 bonus) → Land on pad.

    ros2 launch dronehacks_manual_maze_tunnel_chicane_gateb_landing competitor.launch.py
    ros2 launch dronehacks_manual_maze_tunnel_chicane_gateb_landing competitor.launch.py competitor:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('max_altitude_m',      '3.0',   'ceiling above the odometry origin, metres'),
    # min_altitude_m=0.0 so the LAND state can descend to the floor
    ('min_altitude_m',      '0.0',   'floor — set to 0 to allow landing descent'),
    ('max_speed_mps',       '1.0',   'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz',    '10.0',  'OffboardControlMode heartbeat rate'),
    ('allow_mode_request',  'false', 'enable /skybeta/request_offboard — SIM ONLY'),
    ('auto_offboard_on_arm','true',  'enter Offboard once when pilot arms then switches to Position'),
    ('odometry_topic',      '/visual_slam/tracking/odometry', 'raw ENU pose source'),
    ('hover_altitude_m',    '0.80',  'flight altitude above start, metres'),
    ('arrival_radius_m',    '0.25',  'XY distance that counts as waypoint reached, metres'),
    ('land_speed_mps',      '0.10',  'descent rate during LAND state, m/s'),
    ('competitor',          'true',  'also start the mission node'),
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

    mission = Node(
        package='dronehacks_manual_maze_tunnel_chicane_gateb_landing',
        executable='mission',
        name='dronehacks_full_mission',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'odometry_topic':   LaunchConfiguration('odometry_topic'),
            'hover_altitude_m': LaunchConfiguration('hover_altitude_m'),
            'arrival_radius_m': LaunchConfiguration('arrival_radius_m'),
            'land_speed_mps':   LaunchConfiguration('land_speed_mps'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, mission])
