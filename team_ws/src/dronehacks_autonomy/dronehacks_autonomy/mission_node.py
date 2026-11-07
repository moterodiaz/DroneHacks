"""ROS 2 Humble node ``mission``: the course map in, setpoints for the kit's flight interface out.

    /visual_slam/tracking/odometry ─────┐
    /skybeta/state (JSON) ──────────────┤ mission ─► /skybeta/setpoint/pose, /skybeta/setpoint/twist
    /fmu/out/vehicle_land_detected ─────┤             (the kit's flight interface)
    /fmu/out/vehicle_attitude ──────────┘   (read only)

At start-up it plans from the map file:
* ``planning.pipeline.fastest`` plans, smooths and profiles the speed;
* it places the odom frame by the **measured** course->odom transform
  (contracts/frames.md): three required parameters, no defaults. It never
  derives the transform from the map's launch heading.

Any failure is a configuration error: the node publishes nothing and exits.
With no team setpoint, the kit refuses Offboard, so the pilot keeps the
vehicle. ``dronehacks_autonomy.mission.Mission`` does the flying (its docstring
has the states and rules).

It never arms, disarms or changes mode, never requests Offboard, never
publishes to ``/fmu/in/*``, and reads no lidar.

Parameters:

    map_file              the course map (contracts/course.md format)          required
    rest_height_m         body centre above the floor when landed, m           required (measure it)
    course_to_odom_yaw    rotation of the measured R_odom_course, rad          required (measure it)
    course_to_odom_x      o_odom x: the course origin's odom reading, m         required (measure it)
    course_to_odom_y      o_odom y                                          required (measure it)
    v_max_mps             MPC_XY_VEL_MAX as set on the drone                   1.0
    odometry_topic        the pose                                             /visual_slam/tracking/odometry
    state_topic           the kit's state JSON                                 /skybeta/state
    land_detected_topic   PX4's land detector ("" = off, "auto" = resolved)    auto
    attitude_topic        PX4's attitude, for the heading gate ("" = off)      auto
    setpoint_pose_out     the kit's pose setpoint topic                        /skybeta/setpoint/pose
    setpoint_twist_out    the kit's twist setpoint topic                       /skybeta/setpoint/twist
    frame_id              header.frame_id of the setpoints                     odom
    rate_hz               setpoint rate                                        20.0
    state_timeout_s       /skybeta/state older than this counts as no Offboard 1.0
    use_window            fly the window (the map needs a reviewed budget too)  false
    <MissionConfig field> any other field of dronehacks_autonomy.mission.MissionConfig
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from dronehacks_autonomy.planning.limits import VehicleLimits

from .factory import prepare
from .frames import OdomFrame, body_to_world, ned_yaw_to_enu, quat_to_yaw, yaw_to_quat
from .mission import Command, MissionConfig, OdomPose

QUEUE_DEPTH = 10
BEST_EFFORT = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)


class ConfigError(ValueError):
    pass


class MissionNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("mission", **kwargs)
        self.declare_parameter("map_file", "")
        self.declare_parameter("rest_height_m", float("nan"))
        self.declare_parameter("course_to_odom_yaw", float("nan"))
        self.declare_parameter("course_to_odom_x", float("nan"))
        self.declare_parameter("course_to_odom_y", float("nan"))
        self.declare_parameter("v_max_mps", VehicleLimits().v_max_mps)
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("state_topic", "/skybeta/state")
        self.declare_parameter("land_detected_topic", "auto")
        self.declare_parameter("attitude_topic", "auto")
        self.declare_parameter("setpoint_pose_out", "/skybeta/setpoint/pose")
        self.declare_parameter("setpoint_twist_out", "/skybeta/setpoint/twist")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("state_timeout_s", 1.0)
        self.declare_parameter("use_window", False)
        cfg_fields = [f for f in dataclasses.fields(MissionConfig) if f.name != "rest_height_m"]
        defaults = MissionConfig(rest_height_m=0.0)
        for f in cfg_fields:
            self.declare_parameter(f.name, getattr(defaults, f.name))

        try:
            map_file = self._param("map_file")
            if not isinstance(map_file, str) or not map_file:
                raise ValueError("parameter map_file is required (it has no default)")
            rest = float(self._param("rest_height_m"))
            if not math.isfinite(rest):
                raise ValueError("parameter rest_height_m is required: measure it (it has no default)")
            transform = tuple(float(self._param(n)) for n in
                              ("course_to_odom_yaw", "course_to_odom_x", "course_to_odom_y"))
            if not all(math.isfinite(v) for v in transform):
                raise ValueError("parameters course_to_odom_yaw, course_to_odom_x and course_to_odom_y "
                                 "are required: the measured course->odom transform, contracts/frames.md "
                                 "(the rotation of R_odom_course, then the translation o_odom). They have "
                                 "no defaults and are not derived from the launch heading: measure them "
                                 "at each tracking initialisation")
            frame = OdomFrame.from_measured(transform[0], transform[1:])
            config = MissionConfig(rest_height_m=rest, **{f.name: self._param(f.name) for f in cfg_fields})
            limits = dataclasses.replace(VehicleLimits(), v_max_mps=float(self._param("v_max_mps")))
            self.prepared = prepare(map_file, config, limits, frame=frame,
                                    use_window=bool(self._param("use_window")))
        except Exception as exc:  # noqa: BLE001 - every start-up failure must stop the node before it publishes
            raise ConfigError(str(exc)) from exc
        self.mission = self.prepared.mission
        rate = float(self._param("rate_hz"))
        if not (math.isfinite(rate) and rate > 0.0):
            raise ConfigError("rate_hz must be > 0")
        self._state_timeout = float(self._param("state_timeout_s"))
        self._frame_id = str(self._param("frame_id"))

        self._pose: Optional[OdomPose] = None
        self._offboard = False
        self._state_t: Optional[float] = None
        self._landed: Optional[bool] = None
        self._px4_yaw: Optional[float] = None
        self._px4_yaw_t: Optional[float] = None
        self._sent = {"pose": 0, "twist": 0}

        self._pose_pub = self.create_publisher(PoseStamped, str(self._param("setpoint_pose_out")), QUEUE_DEPTH)
        self._twist_pub = self.create_publisher(TwistStamped, str(self._param("setpoint_twist_out")), QUEUE_DEPTH)
        self._status_pub = self.create_publisher(String, "~/status", QUEUE_DEPTH)
        # Best effort receives from reliable and best-effort publishers alike.
        self.create_subscription(Odometry, str(self._param("odometry_topic")), self._on_odom, BEST_EFFORT)
        self.create_subscription(String, str(self._param("state_topic")), self._on_state, QUEUE_DEPTH)
        land_topic = self._fmu_topic("land_detected_topic", "vehicle_land_detected", "VehicleLandDetected")
        if self.mission.config.return_leg and not land_topic:
            raise ConfigError("return_leg needs land_detected_topic: the touchdown on the pad starts the return")
        if land_topic:
            from px4_msgs.msg import VehicleLandDetected
            self.create_subscription(VehicleLandDetected, land_topic, self._on_land, BEST_EFFORT)
        att_topic = self._fmu_topic("attitude_topic", "vehicle_attitude", "VehicleAttitude")
        if att_topic:
            from px4_msgs.msg import VehicleAttitude
            self.create_subscription(VehicleAttitude, att_topic, self._on_attitude, BEST_EFFORT)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._publish_status)
        plan = self.prepared.plan
        self.get_logger().info(
            f"map {map_file}: route {' > '.join(plan.route.goals)}, {plan.path.length_m:.2f} m, planned "
            f"{plan.duration:.2f} s at v_max {limits.v_max_mps} m/s (built in {self.prepared.seconds:.2f} s); "
            f"fly z {self.mission.z_fly:.2f} m; alternatives: "
            + "; ".join(f"{c}: {o}" for c, o in self.prepared.alternatives))

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _fmu_topic(self, param: str, base: str, msg: str) -> str:
        value = str(self._param(param))
        if value != "auto":
            return value
        from skybeta_flight_interface.topics import fmu_topic   # the kit's own resolver (_vN suffixes)
        return fmu_topic("out", base, msg)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ------------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = odom_to_pose(msg)

    def _on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(state, dict) or not isinstance(state.get("offboard_active"), bool):
            return
        self._offboard = state["offboard_active"]
        self._state_t = self._now_s()

    def _on_land(self, msg: Any) -> None:
        self._landed = bool(msg.landed)

    def _on_attitude(self, msg: Any) -> None:
        w, x, y, z = (float(v) for v in msg.q)        # px4_msgs: q is w, x, y, z (FRD to NED)
        self._px4_yaw = ned_yaw_to_enu(quat_to_yaw(x, y, z, w))
        self._px4_yaw_t = self._now_s()

    # -- the tick ------------------------------------------------------------------------

    def _tick(self) -> None:
        now = self._now_s()
        fresh_state = self._state_t is not None and now - self._state_t <= self._state_timeout
        px4_yaw = self._px4_yaw if (self._px4_yaw_t is not None and now - self._px4_yaw_t <= 1.0) else None
        # Before Offboard, silence means no: nothing is offered on an unknown state. Once flying it
        # means unknown, and the last state stands. Stopping would be unsafe: a descent's twist expires
        # into the stored pose above, so the drone would climb back up
        # (flight_interface._expire_stale_velocity).
        offboard = self._offboard and (fresh_state or self.mission.started)
        cmd = self.mission.update(now, offboard, self._pose, self._landed, px4_yaw)
        if cmd is not None:
            self._publish(cmd)

    def _publish(self, cmd: Command) -> None:
        stamp = self.get_clock().now().to_msg()
        if cmd.kind == "pose":
            self._pose_pub.publish(pose_msg(cmd, self._frame_id, stamp))
        else:
            self._twist_pub.publish(twist_msg(cmd, self._frame_id, stamp))
        self._sent[cmd.kind] += 1

    def _publish_status(self) -> None:
        status = self.mission.status()
        status.update({"offboard": self._offboard, "landed": self._landed, "sent": dict(self._sent)})
        self._status_pub.publish(String(data=json.dumps(status)))


def odom_to_pose(msg: Odometry) -> OdomPose:
    """nav_msgs/Odometry (pose in odom, twist in the body's FLU frame) as an OdomPose."""
    p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
    vx, vy = body_to_world(float(tw.x), float(tw.y), yaw)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return OdomPose(t=stamp, x=float(p.x), y=float(p.y), z=float(p.z), yaw=yaw, vx=vx, vy=vy)


def pose_msg(cmd: Command, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = cmd.x, cmd.y, cmd.z
    if cmd.yaw is None:   # all-zero quaternion: the kit's "yaw not commanded"
        msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = msg.pose.orientation.w = 0.0
    else:
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = yaw_to_quat(cmd.yaw)
    return msg


def twist_msg(cmd: Command, frame_id: str, stamp) -> TwistStamped:
    msg = TwistStamped()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = cmd.vx, cmd.vy, cmd.vz
    msg.twist.angular.z = cmd.yaw_rate
    return msg


def main(args: Any = None) -> int:
    rclpy.init(args=args)
    try:
        node = MissionNode()
    except ConfigError as exc:
        print(f"mission: configuration error, nothing published: {exc}", file=sys.stderr)
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
