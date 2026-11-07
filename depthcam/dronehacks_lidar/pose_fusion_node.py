"""ROS 2 Humble node ``pose_fusion``: the lidar's drift correction, applied to the flight commands.

    cuVSLAM ─┬─────────────────────────────────────► kit px4_odom_bridge.py ─► EKF2   (unchanged)
             └► vo_guard ─► input_topic ─┐
    lidar_localizer ─► lidar_topic ──────┤  this node: C, with truth ≈ C ∘ odom
                                         ├─► output_topic: the drone's true pose ─► follower
    follower ─► setpoint_{pose,twist}_in ┴─► C⁻¹ ∘ setpoint ─► /skybeta/setpoint/{pose,twist} ─► flight_interface

PX4 flies on raw cuVSLAM through the organisers' bridge, which team code can't sit in front of.
So the correction goes into the flight commands (``pose_fusion.setpoints``):

* **The follower's frame:** it works in the true tracker frame.
* **Its pose:** it reads the drone's true pose from ``output_topic``.
* **Its setpoints:** it sends them to ``setpoint_pose_in`` / ``setpoint_twist_in``. This node
  turns them into PX4's odom frame for the kit's flight interface.

Both uses take the same slew-limited ``C``, so the commands never jump. The follower's last pose
setpoint is re-sent at ``refresh_hz`` while ``C`` moves, but only while the follower is still
streaming (its last pose under ``refresh_max_age_s`` old, ``pose_fusion.setpoints.refresh_due``).
When the follower falls silent on purpose, for example before Offboard with its start checks
failing, a re-sent setpoint would look live to the kit, which enters Offboard only on a team
setpoint under 1 s old. ``~/ready`` (std_msgs/Bool, 10 Hz) says whether
the correction has settled; it is always true when the lidar is forbidden. The follower waits
for it before taking off.

It never writes to ``/fmu/in/*``, never sets a PX4 parameter, and never arms or changes mode
(contracts/px4.md). The follower's setpoints reach the kit only through this node, so run it in
every run.

The lidar policy is a required parameter with no default:

    lidar_policy:=forbidden   the lidar is never subscribed. Guard poses and setpoints are
                              republished unchanged (the same message objects).
    lidar_policy:=allowed     lidar fixes estimate C; poses and setpoints are corrected.

Anything else stops the node before it publishes anything (exit status 2). The policy is
published once on ``~/lidar_policy`` (transient-local); ``~/status`` carries JSON once a second.

Parameters:

    lidar_policy          "allowed" | "forbidden"                             required
    input_topic           the Guard's guarded nav_msgs/Odometry               required
    output_topic          the drone's true pose, for the follower             required
    setpoint_pose_in      the follower's geometry_msgs/PoseStamped (true frame)    required
    setpoint_twist_in     the follower's geometry_msgs/TwistStamped (true frame)   required
    setpoint_pose_out     the kit interface's pose input                      /skybeta/setpoint/pose
    setpoint_twist_out    the kit interface's twist input                     /skybeta/setpoint/twist
    refresh_hz            re-send rate of a held pose setpoint while C moves  10.0
    refresh_max_age_s     re-send only while the follower's last pose is younger   0.5
    lidar_topic, lidar_frame_id                                               required when allowed
    map_file, rules_file  the launch mark (the rules' start marking, in the map)   required when allowed,
    launch_mark_east/_north  or else the two numbers from contracts/course.md      one or the other
    <FusionConfig field>  any field of pose_fusion.fusion.FusionConfig
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import fields
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header, String

from pose_fusion.course_files import launch_mark_from_files
from pose_fusion.frames import LaunchMark
from pose_fusion.fusion import FusionConfig, PolicyViolation, PoseFusion
from pose_fusion.policy import LidarPolicy, parse_lidar_policy
from pose_fusion.ros_io import (LidarInputError, fill_pose, fill_pose_setpoint, fill_twist_setpoint,
                                lidar_pose_to_fix, odometry_to_vo_sample, pose_setpoint_from_msg,
                                twist_setpoint_from_msg)
from pose_fusion.se2 import IDENTITY, Pose2
from pose_fusion.setpoints import refresh_due, to_odom_pose, to_odom_twist

QUEUE_DEPTH = 10


class ConfigError(ValueError):
    """The node was started with a missing or invalid parameter; it publishes nothing."""


def strict_json(value: Any) -> Any:
    """*value* with every non-finite float replaced by None, so the status is strict JSON."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: strict_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [strict_json(v) for v in value]
    return value


class PoseFusionNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("pose_fusion", **kwargs)
        for name in ("lidar_policy", "input_topic", "output_topic", "setpoint_pose_in", "setpoint_twist_in",
                     "lidar_topic", "lidar_frame_id", "map_file", "rules_file"):
            self.declare_parameter(name, "")
        self.declare_parameter("setpoint_pose_out", "/skybeta/setpoint/pose")
        self.declare_parameter("setpoint_twist_out", "/skybeta/setpoint/twist")
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("refresh_max_age_s", 0.5)
        self.declare_parameter("launch_mark_east", float("nan"))
        self.declare_parameter("launch_mark_north", float("nan"))
        defaults = FusionConfig()
        for f in fields(FusionConfig):
            self.declare_parameter(f.name, getattr(defaults, f.name))

        # Validate everything before creating any publisher or subscription.
        try:
            self.policy = parse_lidar_policy(self._param("lidar_policy"))
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
        topics = {name: self._required_str(name) for name in
                  ("input_topic", "output_topic", "setpoint_pose_in", "setpoint_twist_in",
                   "setpoint_pose_out", "setpoint_twist_out")}
        if len(set(topics.values())) != len(topics):
            raise ConfigError("every topic must be different: {}".format(topics))
        refresh_hz = float(self._param("refresh_hz"))
        if not (math.isfinite(refresh_hz) and refresh_hz > 0.0):
            raise ConfigError("refresh_hz must be > 0")
        self._refresh_max_age = float(self._param("refresh_max_age_s"))
        if not (math.isfinite(self._refresh_max_age) and self._refresh_max_age > 0.0):
            raise ConfigError("refresh_max_age_s must be > 0")

        self.counts = {"relayed_odom": 0, "bad_vo": 0, "lidar_rejected": 0, "pose_setpoints": 0,
                       "twist_setpoints": 0, "refreshed": 0}
        lidar_topic = ""
        self._lidar_frame_id = ""
        if self.policy is LidarPolicy.ALLOWED:
            lidar_topic = self._required_str("lidar_topic")
            self._lidar_frame_id = self._required_str("lidar_frame_id")
            try:
                self._mark = self._launch_mark()
                config = FusionConfig(**{f.name: self._param(f.name) for f in fields(FusionConfig)})
            except (OSError, KeyError, TypeError, ValueError) as exc:
                raise ConfigError(str(exc)) from exc
            self.fusion = PoseFusion(self.policy, config)
        else:
            self.fusion = PoseFusion(self.policy)
            if self._param("lidar_topic"):
                self.get_logger().warning("lidar_policy is 'forbidden': lidar_topic {!r} is ignored and not "
                                          "subscribed".format(self._param("lidar_topic")))

        self._last_pose_msg: Optional[PoseStamped] = None    # the follower's last pose setpoint
        self._last_pose_rx: Optional[float] = None           # when it arrived (this node's clock)
        self._last_kind = ""                                  # "pose" or "twist": what the follower sent last
        self._sent_with = IDENTITY                            # the correction the last pose setpoint used

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._policy_pub = self.create_publisher(String, "~/lidar_policy", latched)
        self._status_pub = self.create_publisher(String, "~/status", QUEUE_DEPTH)
        self._pub = self.create_publisher(Odometry, topics["output_topic"], QUEUE_DEPTH)
        self._pose_pub = self.create_publisher(PoseStamped, topics["setpoint_pose_out"], QUEUE_DEPTH)
        self._twist_pub = self.create_publisher(TwistStamped, topics["setpoint_twist_out"], QUEUE_DEPTH)
        if self.policy is LidarPolicy.ALLOWED:
            self.create_subscription(PoseWithCovarianceStamped, lidar_topic, self._on_lidar, QUEUE_DEPTH)
            self.create_timer(1.0 / refresh_hz, self._refresh)
        self.create_subscription(Odometry, topics["input_topic"], self._on_odom, QUEUE_DEPTH)
        self.create_subscription(PoseStamped, topics["setpoint_pose_in"], self._on_pose_setpoint, QUEUE_DEPTH)
        self.create_subscription(TwistStamped, topics["setpoint_twist_in"], self._on_twist_setpoint, QUEUE_DEPTH)
        self._ready_pub = self.create_publisher(Bool, "~/ready", QUEUE_DEPTH)
        self._policy_pub.publish(String(data=self.policy.value))
        self.create_timer(1.0, self._publish_status)
        self.create_timer(0.1, lambda: self._ready_pub.publish(Bool(data=bool(self.fusion.settled()))))
        self.get_logger().info("lidar_policy={} {}{}".format(
            self.policy.value, topics,
            " lidar={} frame={}".format(lidar_topic, self._lidar_frame_id) if lidar_topic else ""))

    # ------------------------------------------------------------------ parameters

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _launch_mark(self) -> LaunchMark:
        """From map_file + rules_file (preferred: it traces to the map, AGENTS.md §3.7), else the two numbers."""
        map_file, rules_file = self._param("map_file"), self._param("rules_file")
        if map_file or rules_file:
            if not (map_file and rules_file):
                raise ConfigError("map_file and rules_file go together")
            return launch_mark_from_files(map_file, rules_file)
        return LaunchMark(self._param("launch_mark_east"), self._param("launch_mark_north"))

    def _required_str(self, name: str) -> str:
        value = self._param(name)
        if not isinstance(value, str) or not value:
            raise ConfigError("parameter {} is required (it has no default)".format(name))
        return value

    # ------------------------------------------------------------------ the drone's pose

    def _on_odom(self, msg: Odometry) -> None:
        if self.policy is LidarPolicy.FORBIDDEN:
            self.counts["relayed_odom"] += 1
            self._pub.publish(msg)
            return
        sample = odometry_to_vo_sample(msg)
        if not sample.is_finite():
            self.counts["bad_vo"] += 1
            self.get_logger().warning("non-finite Guard pose dropped", throttle_duration_sec=5.0)
            return
        fused = self.fusion.add_vo(sample)
        if fused is None:
            return
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist                        # body-frame twist: the correction doesn't change it
        fill_pose(out.pose.pose, fused)
        self._pub.publish(out)

    def _on_lidar(self, msg: PoseWithCovarianceStamped) -> None:
        try:
            fix = lidar_pose_to_fix(msg, self._lidar_frame_id, self._mark)
        except LidarInputError as exc:
            self.counts["lidar_rejected"] += 1
            self.get_logger().warning("lidar pose refused: {}".format(exc), throttle_duration_sec=5.0)
            return
        try:
            self.fusion.add_lidar(fix)
        except PolicyViolation:  # unreachable: forbidden runs never subscribe
            self.get_logger().error("lidar fix reached a forbidden-policy fusion; ignored")

    # ------------------------------------------------------------------ the flight commands

    def _on_pose_setpoint(self, msg: PoseStamped) -> None:
        self.counts["pose_setpoints"] += 1
        self._last_kind = "pose"
        if self.policy is LidarPolicy.FORBIDDEN:
            self._pose_pub.publish(msg)
            return
        self._last_pose_msg = msg
        self._last_pose_rx = self._now_s()
        self._send_pose(msg, self.fusion.correction_applied)

    def _send_pose(self, msg: PoseStamped, correction: Pose2, restamp: bool = False) -> None:
        out = PoseStamped()
        if restamp:                                  # a fresh header: the follower's message stays as it came
            out.header = Header(frame_id=msg.header.frame_id, stamp=self.get_clock().now().to_msg())
        else:
            out.header = msg.header
        fill_pose_setpoint(out, to_odom_pose(pose_setpoint_from_msg(msg), correction))
        self._pose_pub.publish(out)
        self._sent_with = correction

    def _on_twist_setpoint(self, msg: TwistStamped) -> None:
        self.counts["twist_setpoints"] += 1
        self._last_kind = "twist"
        if self.policy is LidarPolicy.FORBIDDEN:
            self._twist_pub.publish(msg)
            return
        out = TwistStamped()
        out.header = msg.header
        fill_twist_setpoint(out, to_odom_twist(twist_setpoint_from_msg(msg), self.fusion.correction_applied), msg)
        self._twist_pub.publish(out)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _refresh(self) -> None:
        """Re-send the follower's pose setpoint while the correction moves and the follower still streams.

        Never while the follower uses twists, and never once it has fallen silent (``refresh_due``).
        """
        if self._last_kind != "pose" or self._last_pose_msg is None or self._last_pose_rx is None:
            return
        correction = self.fusion.correction_applied
        if refresh_due(correction, self._sent_with, self._now_s() - self._last_pose_rx, self._refresh_max_age):
            self.counts["refreshed"] += 1
            self._send_pose(self._last_pose_msg, correction, restamp=True)

    def _publish_status(self) -> None:
        status = self.fusion.status()
        status.update(self.counts)
        self._status_pub.publish(String(data=json.dumps(strict_json(status), allow_nan=False)))


def main(args: Any = None) -> int:
    rclpy.init(args=args)
    try:
        node = PoseFusionNode()
    except ConfigError as exc:
        print("pose_fusion: configuration error, nothing published: {}".format(exc), file=sys.stderr)
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
