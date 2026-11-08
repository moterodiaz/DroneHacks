"""The kit's flight interface, started as the team's manual navigator starts it, the course follower,
and a recording of the run.

    ros2 launch dronehacks_course_follower competitor.launch.py     # as the stack starts it
    ros2 launch dronehacks_course_follower competitor.launch.py \
        course_to_odom_yaw:=<measured> course_to_odom_x:=<measured> course_to_odom_y:=<measured>
    ros2 launch dronehacks_course_follower competitor.launch.py competitor:=false   # interface only

The stack starts team code with no arguments, so without the course-to-odom transform
(contracts/frames.md) the follower reads it from the drone resting on the launch mark facing the
launch heading, frozen at arming.
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
    ("speed_mps", "nan", float, "top speed along the route, m/s; NaN flies the speed ladder's current step"),
    ("speed_ladder", "0.5,0.7,0.85,1.0", str, "the ladder's rising top speeds, m/s: a recorded run that lands "
                                              "on the pad moves the next run up a step"),
    ("ladder_file", "/root/team_ws/course_follower_ladder.json", str, "where the ladder's step is kept across runs"),
    ("heading_mode", "ahead", str, "ahead = face the point 0.8 m ahead on the route; fixed = hold the heading at "
                                   "Offboard from the climb to the landing, never yawing"),
    ("turn_rate_dps", "45.0", float, "cap on the commanded heading's turn rate, deg/s"),
    ("yaw_comp", "false", bool, "offset the pose against the airframe's push that PX4 relearns after each yaw; "
                                "measured from PX4's accelerometer in the hover over the start"),
    ("yaw_comp_tau_s", "4.5", float, "PX4's velocity integrator lag, MPC_XY_VEL_P_ACC / MPC_XY_VEL_I_ACC, s"),
    ("yaw_comp_gain", "1.71", float, "PX4's pose-to-acceleration gain, MPC_XY_P x MPC_XY_VEL_P_ACC, 1/s²"),
    ("yaw_comp_max_m", "0.20", float, "cap on the compensator's setpoint offset, metres"),
    ("yaw_comp_push_gain", "1.25", float, "the push per unit of hover accelerometer reading"),
    ("pose_lead", "false", bool, "lead the pose by the setpoint's velocity and acceleration; only with "
                                 "the interface at 50 Hz: at its default 10 Hz led poses cut the first maze corner"),
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

# The run's folders and manifest; the follower writes the transform it flew into meta/params
_PREPARE_SCRIPT = (
    'mkdir -p "$1/meta/params" "$1/px4" '
    '&& printf "%s\\n" "run_id: $(basename "$1")" "profile: light" "started_utc: $2" '
    '"recorded_by: dronehacks_course_follower launch" "exclude_regex: $3" > "$1/meta/manifest.txt"')


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

    started = time.gmtime()
    run = f"{FLIGHTS_DIR}/{time.strftime('%Y%m%d-%H%M%S', started)}-light-follower"
    follower = Node(
        package="dronehacks_course_follower", executable="follower", name="course_follower",
        output="screen", emulate_tty=True, condition=IfCondition(LaunchConfiguration("competitor")),
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                     for name, _, value_type, _ in FOLLOWER_ARGUMENTS}, {"run_dir": run}])

    return LaunchDescription([*declared, flight_interface, follower, *_recording(follower, run, started)])


def _recording(follower: Node, run: str, started: time.struct_time) -> list[Action]:
    """The run's folder and manifest, then the bag, closed by SIGINT when the follower exits.
    Only with the follower running: without it nothing would ever close the bag."""
    recording = IfCondition(PythonExpression(["'", LaunchConfiguration("record"), "'.lower() == 'true' and '",
                                              LaunchConfiguration("competitor"), "'.lower() == 'true'"]))

    prepare = ExecuteProcess(condition=recording, cmd=[
        "bash", "-c", _PREPARE_SCRIPT, "_", run, time.strftime("%Y-%m-%dT%H:%M:%SZ", started), LIGHT_EXCLUDE])
    recorder = ExecuteProcess(condition=recording, output="log", cmd=[
        "ros2", "bag", "record", "-s", "mcap", "-o", f"{run}/bag", "-a", "-x", LIGHT_EXCLUDE])

    # The bag starts once its folder exists, and SIGINT lets rosbag2 write its metadata on the way out
    start_recorder = RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=[recorder]))
    close_bag = RegisterEventHandler(OnProcessExit(target_action=follower, on_exit=[
        EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=matches_action(recorder)))]))
    return [prepare, start_recorder, close_bag]
