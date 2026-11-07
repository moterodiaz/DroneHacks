"""The kit's flight interface, started as the team's manual navigator starts it, the course follower,
and a recording of the run.

    ros2 launch dronehacks_course_follower competitor.launch.py \
        course_to_odom_yaw:=<measured> course_to_odom_x:=<measured> course_to_odom_y:=<measured>
    ros2 launch dronehacks_course_follower competitor.launch.py competitor:=false   # interface only

The follower refuses to start until the course-to-odom transform is passed (contracts/frames.md).
With `record:=true`, the default, a light rosbag2 recording runs from launch into
`/root/team_ws/flights/<UTC time>-light-follower/` (the host's `~/team_ws/flights`), in the folder
layout `python3 -m dronehacks_telemetry.analyze <folder>` reads. The follower exits a few seconds
after the flight's disarm and the bag is then closed with SIGINT, as the square test's launch does.
"""
import signal
import time

from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    ("record", "true", bool, "record a light bag of the run; the follower exits after the flight's disarm "
                             "to close it, so one flight per launch"),
]

# Where the stack's container keeps recorded flights: the host's ~/team_ws/flights
FLIGHTS_DIR = "/root/team_ws/flights"

# dronehacks_telemetry's light profile, as its record_flight.sh has it: every topic but images,
# depth, point clouds and the tracker's visualisation, so a few MB per flight
LIGHT_EXCLUDE = "image|compressed|/depth|points|cloud|/visual_slam/vis/|_path$"

# The run's manifest, and the follower's transform where the analyzer looks for a parameter dump
_PREPARE_SCRIPT = (
    'mkdir -p "$1/meta/params" "$1/px4" '
    '&& printf "%s\\n" "run_id: $(basename "$1")" "profile: light" "started_utc: $2" '
    '"recorded_by: dronehacks_course_follower launch" "exclude_regex: $3" > "$1/meta/manifest.txt" '
    '&& printf "%s\\n" "/course_follower:" "  ros__parameters:" "    course_to_odom_yaw: $4" '
    '"    course_to_odom_x: $5" "    course_to_odom_y: $6" > "$1/meta/params/course_follower.yaml"')


def generate_launch_description() -> LaunchDescription:
    """The interface, the follower unless `competitor:=false`, and the recording while both run."""
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

    return LaunchDescription([*declared, flight_interface, follower, *_recording(follower)])


def _recording(follower: Node) -> list[Action]:
    """The run's folder and manifest, then the bag, closed by SIGINT when the follower exits.
    Only with the follower running: without it nothing would ever close the bag."""
    started = time.gmtime()
    run = f"{FLIGHTS_DIR}/{time.strftime('%Y%m%d-%H%M%S', started)}-light-follower"
    recording = IfCondition(PythonExpression(["'", LaunchConfiguration("record"), "'.lower() == 'true' and '",
                                              LaunchConfiguration("competitor"), "'.lower() == 'true'"]))

    prepare = ExecuteProcess(condition=recording, cmd=[
        "bash", "-c", _PREPARE_SCRIPT, "_", run, time.strftime("%Y-%m-%dT%H:%M:%SZ", started), LIGHT_EXCLUDE,
        LaunchConfiguration("course_to_odom_yaw"), LaunchConfiguration("course_to_odom_x"),
        LaunchConfiguration("course_to_odom_y")])
    recorder = ExecuteProcess(condition=recording, output="log", cmd=[
        "ros2", "bag", "record", "-s", "mcap", "-o", f"{run}/bag", "-a", "-x", LIGHT_EXCLUDE])

    # The bag starts once its folder exists, and SIGINT lets rosbag2 write its metadata on the way out
    start_recorder = RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=[recorder]))
    close_bag = RegisterEventHandler(OnProcessExit(target_action=follower, on_exit=[
        EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(recorder)))]))
    return [prepare, start_recorder, close_bag]
