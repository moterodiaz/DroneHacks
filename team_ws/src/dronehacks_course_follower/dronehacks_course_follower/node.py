"""The ROS 2 node flying the course map's route through the organisers' flight interface.

It reads cuVSLAM's odometry (odom frame, ENU) and `/skybeta/state`, and publishes odom-frame ENU
poses to `/skybeta/setpoint/pose` and, for the landing, twists to `/skybeta/setpoint/twist`. The
mission itself works in the course frame; the measured course-to-odom transform
(contracts/frames.md) is applied here, at the boundary, and the node refuses to start without it.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .dry_run import packaged_map
from .follower import Drone
from .frames import CourseToOdom, quaternion_yaw, yaw_quaternion
from .mission import Mission, PoseCommand, TwistCommand
from .planner import load_course_map, plan_flight

# An odometry message older than this counts as no estimate: the setpoint waits for it
ODOMETRY_STALE_S = 0.5

# The frame the flight interface reads setpoints in
ODOM_FRAME = "odom"


class CourseFollower(Node):
    """Plans once at start-up, then ticks the mission at `rate_hz`."""

    def __init__(self) -> None:
        """Reads the parameters, fails closed on an unmeasured transform, plans, and starts the timer."""
        super().__init__("course_follower")
        transform_values = [self._parameter(name, math.nan) for name in
                            ("course_to_odom_yaw", "course_to_odom_x", "course_to_odom_y")]
        if not all(math.isfinite(value) for value in transform_values):
            raise RuntimeError(f"course_to_odom_yaw/x/y are required measured parameters (contracts/frames.md), "
                               f"not {transform_values}: refusing to fly an assumed transform")

        # The plan, from the course map alone
        map_path = self._parameter("course_map", "") or str(packaged_map())
        course_map = load_course_map(Path(map_path))
        planned = plan_flight(course_map, allow_window=self._parameter("allow_window", True),
                              tunnel_lead_m=self._parameter("tunnel_lead_m", 0.0))
        launch = (float(course_map["launch"][0]), float(course_map["launch"][1]))
        yaw, odom_x, odom_y = transform_values
        self.transform = CourseToOdom(yaw_rad=yaw, odom_origin=(odom_x, odom_y), course_origin=launch,
                                      origin_height_m=self._parameter("odom_origin_height_m", 0.0))
        self.mission = Mission(planned.route, planned.walls, float(course_map["height_m"]), launch,
                               self._parameter("speed_mps", 1.0), lead=self._parameter("pose_lead", True),
                               land=self._parameter("land", True))

        # The kit's topics; the drone's estimate and state as they last arrived
        self.drone: Drone | None = None
        self.drone_at_s = -math.inf
        self.offboard, self.armed = False, False
        self.seen_events = 0
        self.pose_publisher = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(Odometry, self._parameter("odometry_topic", "/visual_slam/tracking/odometry"),
                                 self.on_odometry, 10)
        self.create_subscription(String, "/skybeta/state", self.on_state, 10)
        self.create_timer(1.0 / self._parameter("rate_hz", 50.0), self.on_tick)

        self.get_logger().info(f"{map_path}: {len(planned.route)} waypoints, window "
                               f"{'flown' if planned.window_flown else 'closed'}, flight height "
                               f"{self.mission.height_m:.2f} m (odom z {self.transform.height_to_odom(self.mission.height_m):.2f})")

    def _parameter(self, name: str, default: float | bool | str) -> float | bool | str:
        """Declares a parameter with its default and returns its value."""
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return float(value) if isinstance(default, float) else value

    def on_odometry(self, message: Odometry) -> None:
        """Keeps the drone's pose, moved from odom into the course frame."""
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        east, north = self.transform.point_to_course(position.x, position.y)
        yaw_rad = self.transform.yaw_to_course(quaternion_yaw(orientation.x, orientation.y, orientation.z,
                                                              orientation.w))
        self.drone = Drone(east, north, self.transform.height_to_course(position.z), yaw_rad)
        self.drone_at_s = time.monotonic()

    def on_state(self, message: String) -> None:
        """Keeps whether PX4 is armed and in Offboard, as the flight interface reports it."""
        try:
            state = json.loads(message.data)
        except ValueError:
            return

        self.offboard = bool(state.get("offboard_active", False))
        self.armed = bool(state.get("armed", False))

    def on_tick(self) -> None:
        """One mission step: its command published, its phase changes logged."""
        now_s = time.monotonic()
        drone = self.drone if now_s - self.drone_at_s < ODOMETRY_STALE_S else None
        command = self.mission.tick(now_s, self.offboard, self.armed, drone)
        for event in self.mission.events[self.seen_events:]:
            self.get_logger().info(event)
        self.seen_events = len(self.mission.events)

        if isinstance(command, PoseCommand):
            self.pose_publisher.publish(self._pose(command))
        elif isinstance(command, TwistCommand):
            self.twist_publisher.publish(self._twist(command))

    def _pose(self, command: PoseCommand) -> PoseStamped:
        """A course-frame pose command as the odom-frame message the interface reads."""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ODOM_FRAME
        x, y = self.transform.point_to_odom(command.east, command.north)
        orientation = yaw_quaternion(self.transform.yaw_to_odom(command.yaw_rad))
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.position.z = self.transform.height_to_odom(command.height_m)
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
    """Runs the node until interrupted."""
    rclpy.init(args=args)
    node = CourseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
