"""flight_interface (exactly as competitor_example's launch has it) plus the
path-flight node.

    ros2 launch dronehacks_pathfly pathfly.launch.py              # full mission
    ros2 launch dronehacks_pathfly pathfly.launch.py stage:=1     # staged test
    ros2 launch dronehacks_pathfly pathfly.launch.py speed:=fast  # same path, faster
    ros2 launch dronehacks_pathfly pathfly.launch.py params_file:=/root/team_ws/my.yaml

Mission settings are in config/pathfly.yaml (or params_file). The FAST speed
profile adds config/fast.yaml on top of it; a stage adds config/stage<N>.yaml
last (see BRINGUP.md). With no speed argument the profile comes from
~/team_ws/pathfly_speed on the drone host ("safe" or "fast"; missing = safe),
which scripts/speed.sh sets. The stack's autostart
cannot pass arguments, so stage<N>.launch.py exist for COMPETITOR_LAUNCH.

odom_to_ned_yaw_offset_rad is read from the params file's dronehacks_pathfly
section and given to both nodes, so it is set in one place.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('dronehacks_pathfly')

ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3',
     'floor above the odometry origin for POSE setpoints. This mission only '
     'sends twists, which this floor does not affect'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0',
     'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose'),
    ('params_file', os.path.join(SHARE, 'config', 'pathfly.yaml'),
     'mission parameters'),
    ('stage', '', 'staged bring-up test 1-3. Empty: read stage_file; '
                  '"full" = the full mission'),
    ('stage_file', '/root/team_ws/pathfly_stage',
     'when stage is empty, a file holding 1, 2, 3 or full (missing = full). '
     'This is ~/team_ws/pathfly_stage on the drone host: switching stages '
     'needs no sudo and no drone.env edit'),
    ('speed', '', 'speed profile: "safe" (config/pathfly.yaml only) or '
                  '"fast" (adds config/fast.yaml). Empty: read speed_file'),
    ('speed_file', '/root/team_ws/pathfly_speed',
     'when speed is empty, a file holding safe or fast (missing = safe). '
     'This is ~/team_ws/pathfly_speed on the drone host; scripts/speed.sh '
     'sets it'),
    ('competitor', 'true', 'also start the path-flight node'),
]


def _speed_profile(context):
    """'safe' or 'fast', from the speed argument or else the speed file.
    Anything unrecognised fails the launch rather than guessing."""
    speed = LaunchConfiguration('speed').perform(context).strip().lower()
    source = 'speed:='
    if not speed:
        speed_file = LaunchConfiguration('speed_file').perform(context)
        source = speed_file
        try:
            with open(speed_file, encoding='utf-8') as f:
                speed = f.read().strip().lower()
        except OSError:
            speed, source = 'safe', f'{speed_file} missing'
    if speed in ('', 'safe', 'slow', 'default'):
        speed = 'safe'
    if speed not in ('safe', 'fast'):
        raise RuntimeError(f'speed profile "{speed}" from {source}: '
                           f'use safe or fast')
    return speed, source


def _nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    stage = LaunchConfiguration('stage').perform(context).strip()
    if not stage:
        stage_file = LaunchConfiguration('stage_file').perform(context)
        try:
            with open(stage_file, encoding='utf-8') as f:
                stage = f.read().strip()
            print(f'[pathfly] stage "{stage}" from {stage_file}', flush=True)
        except OSError:
            stage = ''
    if stage.lower() in ('', 'full', 'mission', '0'):
        stage = ''
        print('[pathfly] FULL MISSION (no stage)', flush=True)
    else:
        print(f'[pathfly] STAGED TEST {stage}', flush=True)
    with open(params_file, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    mine = (doc.get('dronehacks_pathfly') or {}).get('ros__parameters') or {}
    yaw_offset = float(mine.get('odom_to_ned_yaw_offset_rad', 0.0))

    speed, speed_source = _speed_profile(context)
    print(f'[pathfly] SPEED {speed.upper()} ({speed_source})', flush=True)

    pathfly_params = [params_file]
    if speed == 'fast':
        fast_file = os.path.join(SHARE, 'config', 'fast.yaml')
        if not os.path.exists(fast_file):
            raise RuntimeError(f'speed fast, but no {fast_file}')
        pathfly_params.append(fast_file)
    if stage:
        stage_file = os.path.join(SHARE, 'config', f'stage{stage}.yaml')
        if not os.path.exists(stage_file):
            raise RuntimeError(f'no such stage: {stage_file}')
        pathfly_params.append(stage_file)
    pathfly_params.append(
        {'odometry_topic': LaunchConfiguration('odometry_topic')})

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'odom_to_ned_yaw_offset_rad': yaw_offset,
        }],
    )
    pathfly = Node(
        package='dronehacks_pathfly',
        executable='pathfly',
        name='dronehacks_pathfly',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=pathfly_params,
    )
    return [flight_interface, pathfly]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=_nodes)])
