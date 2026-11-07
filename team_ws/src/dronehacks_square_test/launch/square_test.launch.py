"""The kit's flight interface (unchanged settings) plus the cage square test, recorded.

For COMPETITOR_LAUNCH=square_test.launch.py with COMPETITOR_PKG=dronehacks_square_test.
Tune from the command line, e.g. ``ros2 launch dronehacks_square_test square_test.launch.py side_m:=0.5``.

Recording (``record:=true``, the default): a rosbag2 recording with dronehacks_telemetry's
``light`` profile (every topic but images, point clouds and visualisation) into
``/root/team_ws/flights/<UTC time>-light-square/bag`` (on the host, ``~/team_ws/flights``), the
folder layout ``python3 -m dronehacks_telemetry.analyze <folder>`` reads. It runs from stack
start; when the mission is over and PX4 has disarmed, the mission node exits 5 s later and the
bag is closed with SIGINT. Cut power before that and the bag is left open: run
``ros2 bag reindex <folder>/bag -s mcap`` in the container before analysing it.
"""
import os
import signal
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INTERFACE = [  # the kit example's own defaults
    ('max_altitude_m', '3.0'), ('min_altitude_m', '0.3'), ('max_speed_mps', '1.5'),
    ('setpoint_rate_hz', '10.0'), ('allow_mode_request', 'false'), ('auto_offboard_on_arm', 'true'),
    ('odometry_topic', '/visual_slam/tracking/odometry'),
]
MISSION = [('side_m', '0.5'), ('height_m', '0.73'), ('speed_mps', '0.3'), ('hold_s', '3.0'), ('ground_s', '1.0'),
           ('yaw_rate_dps', '20.0')]
FLIGHTS = '/root/team_ws/flights'
LIGHT_EXCLUDE = 'image|compressed|/depth|points|cloud|/visual_slam/vis/|_path$'  # = record_flight.sh's


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in INTERFACE + MISSION]
    args.append(DeclareLaunchArgument('record', default_value='true', description='record a light bag'))
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in INTERFACE}])
    mission = Node(
        package='dronehacks_square_test', executable='square_test', name='square_test',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in MISSION}])

    started = time.gmtime()
    run = os.path.join(FLIGHTS, time.strftime('%Y%m%d-%H%M%S', started) + '-light-square')
    record = IfCondition(LaunchConfiguration('record'))
    prepare = ExecuteProcess(condition=record, cmd=['bash', '-c', (
        'mkdir -p "$1/meta" "$1/px4" && printf "%s\n" "run_id: $(basename "$1")" "profile: light" '
        '"started_utc: $2" "recorded_by: dronehacks_square_test launch" "exclude_regex: $3" '
        '> "$1/meta/manifest.txt"'), '_', run, time.strftime('%Y-%m-%dT%H:%M:%SZ', started), LIGHT_EXCLUDE])
    recorder = ExecuteProcess(condition=record, output='log', cmd=[
        'ros2', 'bag', 'record', '-s', 'mcap', '-o', run + '/bag', '-a', '-x', LIGHT_EXCLUDE])
    start_recorder = RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=[recorder]))
    close_bag = RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[
        EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(recorder)))]))
    return LaunchDescription(args + [interface, mission, prepare, start_recorder, close_bag])
