"""The ROS 2 node flying the course map's route through the organisers' flight interface.

It reads cuVSLAM's odometry (odom frame, ENU) and `/skybeta/state`, and publishes odom-frame ENU
poses to `/skybeta/setpoint/pose` and, for the landing, twists to `/skybeta/setpoint/twist`. The
mission itself works in the course frame; the course-to-odom transform (contracts/frames.md) is
applied here, at the boundary. It is either given as parameters or, since the stack launches team
code with no arguments, read from the odometry of the drone resting on the launch mark facing the
launch heading, and frozen at arming.
"""
from __future__ import annotations

import json
import math
import time
from enum import Enum
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .dry_run import packaged_map
from .follower import Drone
from .frames import (OFFSET_LIMIT_M, CourseToOdom, NotAtRest, Px4Offset, RestOnMark, quaternion_yaw,
                     yaw_quaternion)
from .ladder import parse_steps, read_step, record_run
from .mission import Mission, Phase, PoseCommand, TwistCommand
from .planner import load_course_map, plan_flight

# An odometry message older than this counts as no estimate: the setpoint waits for it
ODOMETRY_STALE_S = 0.5

# The speed ladder's top speeds, m/s: a recorded run that lands on the pad moves the next up a step.
# It stops at PX4's MPC_XY_VEL_MAX on the kit, 1.0 m/s
DEFAULT_LADDER = "0.5,0.7,0.85,1.0"

# Where the ladder's step lives across stack restarts: the team workspace, the host's ~/team_ws
LADDER_FILE = "/root/team_ws/course_follower_ladder.json"

# The frame the flight interface reads setpoints in
ODOM_FRAME = "odom"

# With `record` on, the node exits this long after the flight's disarm, so the recording the launch
# closes on its exit keeps the touchdown and PX4 settling
RECORDING_TAIL_S = 5.0


class Placement(str, Enum):
    """Where the course-to-odom transform stands."""

    READING = "reading"       # read afresh from the rest on the mark, until arming freezes it
    PLACED = "placed"         # given, or frozen at arming: the course is placed for this flight
    REFUSED = "refused"       # armed while not resting on the mark: nothing is sent until a disarm


class CourseFollower(Node):
    """Plans once at start-up, then ticks the mission at `rate_hz`."""

    def __init__(self) -> None:
        """Reads the parameters, plans, places the course or starts reading it from the rest on the
        mark, and starts the timer."""
        super().__init__("course_follower")
        transform_values = [self._parameter(name, math.nan) for name in
                            ("course_to_odom_yaw", "course_to_odom_x", "course_to_odom_y")]
        given = [math.isfinite(value) for value in transform_values]
        if any(given) and not all(given):
            raise RuntimeError(f"course_to_odom_yaw/x/y are given together or not at all, not {transform_values}")

        # The plan, from the course map alone
        map_path = self._parameter("course_map", "") or str(packaged_map())
        course_map = load_course_map(Path(map_path))
        planned = plan_flight(course_map, allow_window=self._parameter("allow_window", True),
                              tunnel_lead_m=self._parameter("tunnel_lead_m", 0.0))
        launch = (float(course_map["launch"][0]), float(course_map["launch"][1]))
        origin_height_m = self._parameter("odom_origin_height_m", 0.0)

        # One flight per launch while recording: the run ends a little after its disarm, and the
        # transform flown is written into the run's folder for the flight report
        self.record = bool(self._parameter("record", False))
        self.run_dir = self._parameter("run_dir", "")
        self.over_at_s: float | None = None

        # The top speed: given, or, for a recorded run, the speed ladder's current step
        self.ladder_steps = parse_steps(self._parameter("speed_ladder", DEFAULT_LADDER))
        self.ladder_path = Path(self._parameter("ladder_file", LADDER_FILE))
        self.ladder_step: int | None = None
        speed_mps = self._parameter("speed_mps", math.nan)
        if not math.isfinite(speed_mps):
            self.ladder_step = read_step(self.ladder_path, len(self.ladder_steps)) if self.record else 0
            speed_mps = self.ladder_steps[self.ladder_step]

        self.mission = Mission(planned.route, planned.walls, float(course_map["height_m"]), launch, speed_mps,
                               lead=self._parameter("pose_lead", False), land=self._parameter("land", True))

        # The transform as given, or read from the rest on the mark: the map's launch heading is
        # counter-clockwise from east
        self.transform: CourseToOdom | None = None
        self.rest = RestOnMark(launch, math.radians(float(course_map["launch"][2])), origin_height_m)
        self.placement = Placement.READING
        if all(given):
            yaw, odom_x, odom_y = transform_values
            self.transform = CourseToOdom(yaw_rad=yaw, odom_origin=(odom_x, odom_y), course_origin=launch,
                                          origin_height_m=origin_height_m)
            self.placement = Placement.PLACED
            self._record_transform()

        # The kit's topics; the drone's estimate and state as they last arrived
        self.drone: Drone | None = None
        self.drone_at_s = -math.inf
        self.offboard, self.armed = False, False
        self.seen_events = 0

        # PX4's local position as the kit reports it and the raw odometry: their offset rides on
        # every pose setpoint, measured at rest and followed in flight
        self.odom_enu: tuple[float, float, float] | None = None
        self.px4_enu: tuple[float, float, float] | None = None
        self.px4_offset = Px4Offset()
        self.offset_problem: str | None = None
        self.previous_tick_s = time.monotonic()
        self.pose_publisher = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(Odometry, self._parameter("odometry_topic", "/visual_slam/tracking/odometry"),
                                 self.on_odometry, 10)
        self.create_subscription(String, "/skybeta/state", self.on_state, 10)
        self.create_timer(1.0 / self._parameter("rate_hz", 50.0), self.on_tick)

        self.get_logger().info(f"{map_path}: {len(planned.route)} waypoints, window "
                               f"{'flown' if planned.window_flown else 'closed'}, flight height "
                               f"{self.mission.height_m:.2f} m (odom z {self.mission.height_m - origin_height_m:.2f}); "
                               + ("course placed by the given transform" if all(given) else
                                  "the course is placed from the rest on the launch mark at arming")
                               + f"; top speed {speed_mps:.2f} m/s"
                               + ("" if self.ladder_step is None else
                                  f" (ladder step {self.ladder_step + 1} of {len(self.ladder_steps)})"))

    def _parameter(self, name: str, default: float | bool | str) -> float | bool | str:
        """Declares a parameter with its default and returns its value."""
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return float(value) if isinstance(default, float) else value

    def on_odometry(self, message: Odometry) -> None:
        """Keeps the drone's pose, moved from odom into the course frame; while the course is read
        from the rest on the mark, the pose also feeds that reading, and is None while not at rest."""
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        odom_yaw_rad = quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        self.odom_enu = (position.x, position.y, position.z)
        if self.placement is Placement.READING:
            self.rest.add(time.monotonic(), position.x, position.y, odom_yaw_rad)
            try:
                self.transform = self.rest.transform()
            except NotAtRest:
                self.transform = None

        if self.transform is None or self.placement is Placement.REFUSED:
            self.drone = None
            return

        east, north = self.transform.point_to_course(position.x, position.y)
        yaw_rad = self.transform.yaw_to_course(odom_yaw_rad)
        self.drone = Drone(east, north, self.transform.height_to_course(position.z), yaw_rad)
        self.drone_at_s = time.monotonic()

    def on_state(self, message: String) -> None:
        """Keeps whether PX4 is armed and in Offboard, as the flight interface reports it; arming
        freezes a transform read from the rest, and a disarm starts a fresh reading."""
        try:
            state = json.loads(message.data)
        except ValueError:
            return

        armed = bool(state.get("armed", False))
        if armed and not self.armed and self.placement is Placement.READING:
            self._place_from_rest()
        elif not armed and self.armed and self.placement is Placement.REFUSED:
            self.placement = Placement.READING
            self.rest.samples.clear()

        self.offboard = bool(state.get("offboard_active", False))
        self.armed = armed
        local = state.get("local_enu")
        self.px4_enu = tuple(float(value) for value in local) if isinstance(local, list) and len(local) == 3 else None

    def _place_from_rest(self) -> None:
        """Freezes the transform read from the rest on the mark; if the drone was not resting, sends
        nothing until a disarm, so the interface never enters Offboard on a guessed course."""
        try:
            self.transform = self.rest.transform()
        except NotAtRest as failure:
            self.transform, self.drone, self.placement = None, None, Placement.REFUSED
            self.get_logger().error(f"armed, but the course cannot be placed: {failure}. Nothing is sent, so no "
                                    "Offboard: disarm, rest the drone on the mark facing the launch heading, arm again")
            return

        self.placement = Placement.PLACED
        self.get_logger().info(f"course placed from the rest on the mark: course_to_odom_yaw "
                               f"{self.transform.yaw_rad:.4f} x {self.transform.odom_origin[0]:.3f} "
                               f"y {self.transform.odom_origin[1]:.3f}")
        self._record_transform()

    def _record_transform(self) -> None:
        """Writes the transform flown into the run's folder, as the parameter dump the report reads."""
        if not (self.record and self.run_dir):
            return

        path = Path(self.run_dir) / "meta" / "params" / "course_follower.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("/course_follower:\n  ros__parameters:\n"
                        f"    course_to_odom_yaw: {self.transform.yaw_rad!r}\n"
                        f"    course_to_odom_x: {self.transform.odom_origin[0]!r}\n"
                        f"    course_to_odom_y: {self.transform.odom_origin[1]!r}\n", encoding="utf-8")

    def _offset_problem(self, elapsed_s: float) -> str | None:
        """Measures PX4's offset before Offboard and follows it after; before Offboard, why nothing may
        be streamed yet, or None. A change of answer is logged once."""
        problem = None
        if self.odom_enu is not None and self.px4_enu is None:
            problem = "no PX4 local position in /skybeta/state yet"
        elif self.odom_enu is not None and self.mission.phase is Phase.WAIT:
            self.px4_offset.measure(self.px4_enu, self.odom_enu)
            problem = self.px4_offset.too_far(OFFSET_LIMIT_M)
        elif self.odom_enu is not None:
            self.px4_offset.follow(self.px4_enu, self.odom_enu, elapsed_s)

        if self.mission.phase is Phase.WAIT and problem != self.offset_problem:
            if problem:
                self.get_logger().error(f"{problem}: not streaming, so no Offboard. Power-cycle the drone (FC and "
                                        "Jetson) sitting on the launch mark")
            else:
                self.get_logger().info(f"PX4 offset from cuVSLAM {math.hypot(*self.px4_offset.enu):.2f} m: streaming")
            self.offset_problem = problem

        return problem if self.mission.phase is Phase.WAIT else None

    def on_tick(self) -> None:
        """One mission step: its command published, its phase changes logged. Before Offboard, nothing
        is streamed while PX4's frame sits too far from the tracker's."""
        now_s = time.monotonic()
        elapsed_s, self.previous_tick_s = now_s - self.previous_tick_s, now_s
        drone = self.drone if now_s - self.drone_at_s < ODOMETRY_STALE_S else None
        if self._offset_problem(elapsed_s):
            drone = None
        command = self.mission.tick(now_s, self.offboard, self.armed, drone)
        for event in self.mission.events[self.seen_events:]:
            self.get_logger().info(event)
        self.seen_events = len(self.mission.events)

        if isinstance(command, PoseCommand):
            self.pose_publisher.publish(self._pose(command))
        elif isinstance(command, TwistCommand):
            self.twist_publisher.publish(self._twist(command))

        if self.record and self.over_at_s is None and self.mission.flight_over(self.armed):
            self.over_at_s = now_s
            self.get_logger().info(f"flight over and disarmed: exiting in {RECORDING_TAIL_S:.0f} s to close the "
                                   "recording; restart the stack on the mark to fly again")
            self._climb_ladder()

    def _climb_ladder(self) -> None:
        """Records the run on the speed ladder: a landing on the pad moves the next run up a step."""
        if self.ladder_step is None:
            return

        landed = self.mission.phase is Phase.DONE
        try:
            next_step = record_run(self.ladder_path, self.ladder_step, self.ladder_steps, landed)
        except OSError as failure:
            self.get_logger().error(f"speed ladder not written to {self.ladder_path}: {failure}")
            return

        self.get_logger().info(f"speed ladder: {'landed on the pad' if landed else 'did not land on the pad'}, "
                               f"next run {self.ladder_steps[next_step]:.2f} m/s")

    def finished(self) -> bool:
        """Whether the recorded flight is over and its tail kept, so the node can exit."""
        return self.over_at_s is not None and time.monotonic() - self.over_at_s >= RECORDING_TAIL_S

    def _pose(self, command: PoseCommand) -> PoseStamped:
        """A course-frame pose command as the message the interface reads: odom-frame, carrying PX4's
        offset, since the interface hands it to PX4 unanchored and PX4 flies it in its own frame."""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ODOM_FRAME
        x, y = self.transform.point_to_odom(command.east, command.north)
        offset_x, offset_y, offset_z = self.px4_offset.enu
        orientation = yaw_quaternion(self.transform.yaw_to_odom(command.yaw_rad))
        message.pose.position.x, message.pose.position.y = x + offset_x, y + offset_y
        message.pose.position.z = self.transform.height_to_odom(command.height_m) + offset_z
        (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z,
         message.pose.orientation.w) = orientation
        return message

    def _twist(self, command: TwistCommand) -> TwistStamped:
        """A course-frame twist command as the odom-frame message the interface reads."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ODOM_FRAME
        x, y = self.transform.vector_to_odom(command.east_mps, command.north_mps)
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = x, y, command.up_mps
        message.twist.angular.z = command.yaw_rate_rad_s
        return message


def main(args: list[str] | None = None) -> None:
    """Runs the node until interrupted, or until a recorded flight is over."""
    rclpy.init(args=args)
    node = CourseFollower()
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
