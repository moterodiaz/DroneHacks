"""ROS 2 Humble node ``follower``: the route through the checkpoints -> setpoints in the true frame.

    /skybeta/state (JSON) ──────────────┐
    /pose_fusion/odometry (true pose) ──┤
    /pose_fusion/ready (Bool) ──────────┤ follower ─► /follower/setpoint/{pose,twist} ─► pose_fusion ─► kit
    /fmu/out/vehicle_land_detected ─────┘   (read only)

The route is a ``dronehacks-lidar-route/1`` file (``follower.route``), planned from the STEP-model
map and the ground-truth checkpoints by ``follower.make_route``. The package ships
``routes/course_v3_window.json`` and ``routes/course_v3_gate_b.json``. ``follower.mission.Follower``
does the work:

* **Waiting:** until ``offboard_active``, it streams the takeoff pose, but only while the drone is
  on the route start at the planned heading and pose_fusion is ready. The kit enters Offboard at
  the pilot's switch to Position only on a team setpoint under 1 s old, so a drone that fails these
  checks is refused Offboard and stays on the ground.
* **Pre-takeoff checks:** it checks again once Offboard is active. If they don't pass in time, it
  blocks.
* **Taking off:** it takes off itself.
* **Following:** it follows the route without leaving it by more than ``max_cut_m``, holding the
  planned heading.
* **Landing:** it holds over the pad, then lands with a velocity setpoint. PX4 disarms itself
  after ``COM_DISARM_LAND``.
* **Stopping:** it stops for good when Offboard ends.

It never arms, disarms or changes mode, and it never publishes to ``/skybeta/setpoint/*``
directly. Its setpoints go through ``pose_fusion``, which corrects them into PX4's frame (or
passes them unchanged when the lidar is forbidden).

Parameters:

    route_file          a dronehacks-lidar-route/1 file                       required
    map_file, rules_file  the launch mark (the rules' start marking, in the map)   required
    rest_height_m       body centre above the floor when landed               required
    pose_topic          the true pose                                         /pose_fusion/odometry
    ready_topic         pose_fusion's readiness                               /pose_fusion/ready
    state_topic         the kit's state JSON                                  /skybeta/state
    land_detected_topic PX4's land detector ("" = off; "auto" = resolved)    auto
    setpoint_pose_out   where pose setpoints go (pose_fusion's input)         /follower/setpoint/pose
    setpoint_twist_out  where twist setpoints go                              /follower/setpoint/twist
    frame_id            header.frame_id of the setpoints                      odom
    rate_hz             setpoint rate                                         50.0
    state_timeout_s     /skybeta/state older than this counts as no Offboard  1.0
    <FollowerConfig field>  any other field of follower.mission.FollowerConfig
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import fields
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from follower.mission import Command, Follower, FollowerConfig, Pose
from follower.route import load_route
from pose_fusion.course_files import launch_mark_from_files
from pose_fusion.ros_io import stamp_to_s
from pose_fusion.se2 import quat_yaw

from .px4_topics import fmu_out

QUEUE_DEPTH = 10


class ConfigError(ValueError):
    pass


class FollowerNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("follower", **kwargs)
        for name in ("route_file", "map_file", "rules_file"):
            self.declare_parameter(name, "")
        self.declare_parameter("pose_topic", "/pose_fusion/odometry")
        self.declare_parameter("ready_topic", "/pose_fusion/ready")
        self.declare_parameter("state_topic", "/skybeta/state")
        self.declare_parameter("land_detected_topic", "auto")
        self.declare_parameter("setpoint_pose_out", "/follower/setpoint/pose")
        self.declare_parameter("setpoint_twist_out", "/follower/setpoint/twist")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("state_timeout_s", 1.0)
        self.declare_parameter("rest_height_m", float("nan"))
        cfg_fields = [f for f in fields(FollowerConfig) if f.name != "rest_height_m"]
        defaults = FollowerConfig(rest_height_m=0.0)
        for f in cfg_fields:
            self.declare_parameter(f.name, getattr(defaults, f.name))

        try:
            route_file = self._required_str("route_file")
            mark = launch_mark_from_files(self._required_str("map_file"), self._required_str("rules_file"))
            route = load_route(route_file, mark)
            rest = float(self._param("rest_height_m"))
            if not math.isfinite(rest):
                raise ValueError("parameter rest_height_m is required: measure it (it has no default)")
            config = FollowerConfig(rest_height_m=rest, **{f.name: self._param(f.name) for f in cfg_fields})
            self.mission = Follower(route, config)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise ConfigError(str(exc)) from exc
        rate = float(self._param("rate_hz"))
        if not (math.isfinite(rate) and rate > 0.0):
            raise ConfigError("rate_hz must be > 0")
        self._state_timeout = float(self._param("state_timeout_s"))
        self._frame_id = str(self._param("frame_id"))

        self._pose: Optional[Pose] = None
        self._offboard = False
        self._state_t: Optional[float] = None
        self._landed: Optional[bool] = None
        self._ready = False
        self._ready_t: Optional[float] = None
        self._sent = {"pose": 0, "twist": 0}

        self._pose_pub = self.create_publisher(PoseStamped, str(self._param("setpoint_pose_out")), QUEUE_DEPTH)
        self._twist_pub = self.create_publisher(TwistStamped, str(self._param("setpoint_twist_out")), QUEUE_DEPTH)
        self._status_pub = self.create_publisher(String, "~/status", QUEUE_DEPTH)
        self.create_subscription(Odometry, str(self._param("pose_topic")), self._on_pose, QUEUE_DEPTH)
        self.create_subscription(String, str(self._param("state_topic")), self._on_state, QUEUE_DEPTH)
        self.create_subscription(Bool, str(self._param("ready_topic")), self._on_ready, QUEUE_DEPTH)
        land_topic = str(self._param("land_detected_topic"))
        if land_topic:
            from px4_msgs.msg import VehicleLandDetected
            if land_topic == "auto":
                land_topic = fmu_out("vehicle_land_detected", "VehicleLandDetected")
            px4_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(VehicleLandDetected, land_topic, self._on_land, px4_qos)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info("route {} ({}: {} points, {:.1f} m, {} gates, heading {:.0f} deg in the tracker frame, "
                               "cruise {:.2f} m -> fly z {:.2f} m)".format(
                                   route_file, route.name, len(route.path), route.length_m, len(route.gates),
                                   math.degrees(route.yaw), route.cruise_z_m, self.mission.z_fly))

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _required_str(self, name: str) -> str:
        value = self._param(name)
        if not isinstance(value, str) or not value:
            raise ConfigError("parameter {} is required (it has no default)".format(name))
        return value

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs

    def _on_pose(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = Pose(stamp_to_s(msg.header.stamp), float(p.x), float(p.y), float(p.z),
                          quat_yaw((float(q.x), float(q.y), float(q.z), float(q.w))))

    def _on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self._offboard = bool(state.get("offboard_active", False))
        self._state_t = self._now_s()

    def _on_land(self, msg: Any) -> None:
        self._landed = bool(msg.landed)

    def _on_ready(self, msg: Bool) -> None:
        self._ready = bool(msg.data)
        self._ready_t = self._now_s()

    # ------------------------------------------------------------------ the tick

    def _tick(self) -> None:
        now = self._now_s()
        fresh_state = self._state_t is not None and now - self._state_t <= self._state_timeout
        ready = self._ready and self._ready_t is not None and now - self._ready_t <= self._state_timeout
        cmd = self.mission.update(now, self._offboard and fresh_state, self._pose, self._landed, ready)
        if cmd is not None:
            self._publish(cmd)

    def _publish(self, cmd: Command) -> None:
        stamp = self.get_clock().now().to_msg()
        if cmd.kind == "pose":
            msg = PoseStamped()
            msg.header.frame_id, msg.header.stamp = self._frame_id, stamp
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = cmd.x, cmd.y, cmd.z
            if cmd.yaw is None:                     # all-zero: the kit's "yaw not commanded"
                msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = msg.pose.orientation.w = 0.0
            else:
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(cmd.yaw / 2), math.cos(cmd.yaw / 2)
            self._pose_pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.frame_id, msg.header.stamp = self._frame_id, stamp
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = cmd.vx, cmd.vy, cmd.vz
            msg.twist.angular.z = cmd.yaw_rate
            self._twist_pub.publish(msg)
        self._sent[cmd.kind] += 1

    def _publish_status(self) -> None:
        m = self.mission
        status = {"state": m.state, "route": m.route.name, "progress_m": round(float(m.s[m.index]), 3),
                  "length_m": round(m.route.length_m, 3), "crossed": [name for name, _ in m.crossed],
                  "reason": m.reason, "offboard": self._offboard, "ready": self._ready, "landed": self._landed,
                  "sent": dict(self._sent)}
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args: Any = None) -> int:
    rclpy.init(args=args)
    try:
        node = FollowerNode()
    except ConfigError as exc:
        print("follower: configuration error, nothing published: {}".format(exc), file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
