"""ROS 2 Humble node ``vo_guard``: withholds cuVSLAM poses that can't be trusted.

    /visual_slam/tracking/odometry ─► vo_guard ─► output_topic (nav_msgs/Odometry) ─► pose_fusion
    /visual_slam/status ────────────┤   (raw pose, fixed variances, odom frame)
    /fmu/out/vehicle_odometry ──────┘   (read only: PX4's gyro)

It republishes the organisers' cuVSLAM poses that pass every rule in ``pose_fusion.guard``:
tracker failed, jump, spin, gyro disagreement, and settling after a break. Each passed pose
gets fixed variances. The pose stays in cuVSLAM's ``odom`` frame, which is also PX4's.

It reads no camera images: the 655 KB frames exceed Fast DDS's default shared-memory segment,
so the contract's texture rule is not applied (see ``pose_fusion.guard``). It guards only *our*
nodes, because PX4 still fuses raw cuVSLAM through the organisers' bridge. It never restarts
their tracker, never writes to ``/fmu/in/*``, and needs no lidar, so it runs in every run.

Parameters:

    odom_in             cuVSLAM's odometry                         /visual_slam/tracking/odometry
    output_topic        the guarded odometry                       ~/odometry
    status_topic        isaac_ros_visual_slam_interfaces/VisualSlamStatus ("" = off)   /visual_slam/status
    px4_odometry_topic  PX4's odometry for its gyro ("" = off; "auto" = resolved)      auto
    gyro_latency_s      PX4 -> this node transport, subtracted from receive times      0.005
    <GuardConfig field> any field of pose_fusion.guard.GuardConfig
"""

from __future__ import annotations

import json
import sys
from dataclasses import fields
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from pose_fusion.guard import Guard, GuardConfig, fixed_pose_covariance
from pose_fusion.ros_io import stamp_to_s

from .px4_topics import fmu_out

QUEUE_DEPTH = 10


class ConfigError(ValueError):
    pass


class VoGuardNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("vo_guard", **kwargs)
        self.declare_parameter("odom_in", "/visual_slam/tracking/odometry")
        self.declare_parameter("output_topic", "~/odometry")
        self.declare_parameter("status_topic", "/visual_slam/status")
        self.declare_parameter("px4_odometry_topic", "auto")
        self.declare_parameter("gyro_latency_s", 0.005)
        defaults = GuardConfig()
        for f in fields(GuardConfig):
            self.declare_parameter(f.name, float(getattr(defaults, f.name)))
        try:
            config = GuardConfig(**{f.name: float(self.get_parameter(f.name).value) for f in fields(GuardConfig)})
        except (TypeError, ValueError) as exc:
            raise ConfigError(str(exc)) from exc
        odom_in = str(self.get_parameter("odom_in").value)
        if not odom_in:
            raise ConfigError("odom_in is required")
        self.guard = Guard(config)
        self._gyro_latency = float(self.get_parameter("gyro_latency_s").value)
        self._covariance = fixed_pose_covariance()
        self._status_rule = "on"

        self._pub = self.create_publisher(Odometry, str(self.get_parameter("output_topic").value), QUEUE_DEPTH)
        self._status_pub = self.create_publisher(String, "~/status", QUEUE_DEPTH)
        self.create_subscription(Odometry, odom_in, self._on_odom, qos_profile_sensor_data)

        status_topic = str(self.get_parameter("status_topic").value)
        if status_topic:
            try:
                from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            except ImportError:
                self._status_rule = "off: isaac_ros_visual_slam_interfaces is not installed"
                self.get_logger().warning("no isaac_ros_visual_slam_interfaces: the tracker's own failures go unseen")
            else:
                self.create_subscription(VisualSlamStatus, status_topic, self._on_status, qos_profile_sensor_data)
        else:
            self._status_rule = "off"

        px4_topic = str(self.get_parameter("px4_odometry_topic").value)
        if px4_topic:
            from px4_msgs.msg import VehicleOdometry
            if px4_topic == "auto":
                px4_topic = fmu_out("vehicle_odometry", "VehicleOdometry")
            px4_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(VehicleOdometry, px4_topic, self._on_px4, px4_qos)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info("guarding {} -> {} (gyro: {})".format(odom_in, self._pub.topic_name, px4_topic or "off"))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg: Any) -> None:
        self.guard.on_status(int(msg.vo_state))

    def _on_px4(self, msg: Any) -> None:
        w = msg.angular_velocity                     # body FRD, rad/s
        self.guard.on_gyro(self._now_s() - self._gyro_latency, (float(w[0]), -float(w[1]), -float(w[2])))

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        verdict = self.guard.judge(stamp_to_s(msg.header.stamp), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if not verdict.publish:
            return
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.pose.covariance = self._covariance
        out.twist = msg.twist
        self._pub.publish(out)

    def _publish_status(self) -> None:
        g = self.guard
        status = {"published": g.published, "withheld": dict(g.withheld), "status_rule": self._status_rule}
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args: Any = None) -> int:
    rclpy.init(args=args)
    try:
        node = VoGuardNode()
    except ConfigError as exc:
        print("vo_guard: configuration error: {}".format(exc), file=sys.stderr)
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
