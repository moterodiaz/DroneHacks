"""ROS 2 Humble node ``mission``: the course map in, setpoints for the kit's flight interface out.

    /vo_guard/odometry (or cuVSLAM's) ──┐
    /skybeta/state (JSON) ──────────────┤ mission ─► /skybeta/setpoint/pose, /skybeta/setpoint/twist
    /fmu/out/vehicle_land_detected ─────┤             (the kit's flight interface)
    /fmu/out/vehicle_attitude ──────────┘   (read only)

At start-up it plans from the map file:
* ``dronehacks_planner.pipeline.fastest`` picks the quickest verified
  alternative, then smooths it and profiles its speed;
* it places the odom frame by the **measured** course->odom transform
  (contracts/frames.md): three required parameters, no defaults. It never
  derives the transform from the map's launch heading.

Any failure is a configuration error: the node publishes nothing and exits.
With no team setpoint, the kit refuses Offboard, so the pilot keeps the
vehicle. ``dronehacks_traj.mission.Mission`` does the flying (its docstring
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
    odometry_topic        the pose (the Guard's, or cuVSLAM's)                 /vo_guard/odometry
    state_topic           the kit's state JSON                                 /skybeta/state
    land_detected_topic   PX4's land detector ("" = off, "auto" = resolved)    auto
    attitude_topic        PX4's attitude, for the heading gate ("" = off)      auto
    px4_odometry_topic    PX4's odometry: the velocity a coast dead-reckons on   auto
                          ("" = off: the commanded velocities are integrated)
    setpoint_pose_out     the kit's pose setpoint topic                        /skybeta/setpoint/pose
    setpoint_twist_out    the kit's twist setpoint topic                       /skybeta/setpoint/twist
    frame_id              header.frame_id of the setpoints                     odom
    rate_hz               setpoint rate                                        50.0
    state_timeout_s       /skybeta/state older than this counts as no Offboard 1.0
    prefer                goal names to force among alternatives ([] = fastest) []
    track_speed_mps       fly the course at this speed: the plan is capped at it,   0.0
                          and FOLLOW flies it by velocity, with a hold pose
                          refreshed alongside (factory.for_track_speed);
                          0 = the plan at v_max_mps on reference poses
    avoid_boulders        plan around the boulders (False: drop them from the     true
                          map, so the route may fly over them: they are 0.53 m
                          tall, under the flight slice; PX4's rangefinder height
                          then reads their tops)
    <MissionConfig field> any other field of dronehacks_traj.mission.MissionConfig
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

from dronehacks_planner.course_map import load_map
from dronehacks_planner.limits import VehicleLimits
from dronehacks_planner.route import PlannerConfig

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
        self.declare_parameter("odometry_topic", "/vo_guard/odometry")
        self.declare_parameter("state_topic", "/skybeta/state")
        self.declare_parameter("land_detected_topic", "auto")
        self.declare_parameter("attitude_topic", "auto")
        self.declare_parameter("px4_odometry_topic", "auto")
        self.declare_parameter("setpoint_pose_out", "/skybeta/setpoint/pose")
        self.declare_parameter("setpoint_twist_out", "/skybeta/setpoint/twist")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("state_timeout_s", 1.0)
        self.declare_parameter("prefer", [""])
        self.declare_parameter("avoid_boulders", True)
        self.declare_parameter("track_speed_mps", 0.0)
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
            prefer = tuple(p for p in self._param("prefer") if p)
            cmap = load_map(map_file)
            boulders = cmap.boulders        # the height kept over them, even when the plan ignores them
            if not self._param("avoid_boulders"):
                cmap = dataclasses.replace(cmap, boulders=())
            self.prepared = prepare(cmap, config, limits,
                                   dataclasses.replace(PlannerConfig(), prefer=prefer), frame=frame,
                                   track_speed_mps=float(self._param("track_speed_mps")), low_obstacles=boulders)
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
        if land_topic:
            from px4_msgs.msg import VehicleLandDetected
            self.create_subscription(VehicleLandDetected, land_topic, self._on_land, BEST_EFFORT)
        att_topic = self._fmu_topic("attitude_topic", "vehicle_attitude", "VehicleAttitude")
        if att_topic:
            from px4_msgs.msg import VehicleAttitude
            self.create_subscription(VehicleAttitude, att_topic, self._on_attitude, BEST_EFFORT)
        # PX4's velocity estimate, read only: EKF2 dead-reckons on the IMU through a dropout
        self._px4_velocity = None
        odom_topic = self._fmu_topic("px4_odometry_topic", "vehicle_odometry", "VehicleOdometry")
        if odom_topic:
            from px4_msgs.msg import VehicleOdometry
            self._velocity_frame_ned = VehicleOdometry.VELOCITY_FRAME_NED
            self.create_subscription(VehicleOdometry, odom_topic, self._on_px4_odometry, BEST_EFFORT)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._publish_status)
        plan = self.prepared.plan
        if self.prepared.speed_note:
            self.get_logger().warn(self.prepared.speed_note)
        self.get_logger().info(
            f"map {map_file}: route {' > '.join(plan.route.goals)}, {plan.path.length_m:.2f} m, planned "
            f"{plan.duration:.2f} s at v_max {self.prepared.v_max_mps:g} m/s (built in {self.prepared.seconds:.2f} s); "
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
        # the arrival time lets the mission see the stream itself stop (pose_gap_s)
        self._pose = dataclasses.replace(odom_to_pose(msg), received=self._now_s())

    def _on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self._offboard = bool(state.get("offboard_active", False))
        self._state_t = self._now_s()

    def _on_land(self, msg: Any) -> None:
        self._landed = bool(msg.landed)

    def _on_px4_odometry(self, msg: Any) -> None:
        if msg.velocity_frame != self._velocity_frame_ned:
            return
        v = body_velocity(msg.q, msg.velocity)
        if v is not None:
            self._px4_velocity = (self._now_s(), v[0], v[1])

    def _on_attitude(self, msg: Any) -> None:
        w, x, y, z = (float(v) for v in msg.q)        # px4_msgs: q is w, x, y, z (FRD to NED)
        self._px4_yaw = ned_yaw_to_enu(quat_to_yaw(x, y, z, w))
        self._px4_yaw_t = self._now_s()

    # -- the tick ------------------------------------------------------------------------

    def _tick(self) -> None:
        now = self._now_s()
        fresh_state = self._state_t is not None and now - self._state_t <= self._state_timeout
        px4_yaw = self._px4_yaw if (self._px4_yaw_t is not None and now - self._px4_yaw_t <= 1.0) else None
        cmd = self.mission.update(now, self._offboard and fresh_state, self._pose, self._landed, px4_yaw,
                                  px4_velocity=self._px4_velocity)
        if cmd is not None:
            self._publish(cmd)

    def _publish(self, cmd: Command) -> None:
        stamp = self.get_clock().now().to_msg()
        if cmd.kind == "pose":
            self._pose_pub.publish(pose_msg(cmd, self._frame_id, stamp))
        else:
            if cmd.fallback is not None:
                # the place to hold if the twists stop: the interface keeps the last pose for that.
                # Sent first, so the twist right after it is what the interface flies.
                fx, fy, fz, fyaw = cmd.fallback
                self._pose_pub.publish(pose_msg(Command("pose", fx, fy, fz, fyaw), self._frame_id, stamp))
                self._sent["pose"] += 1
            self._twist_pub.publish(twist_msg(cmd, self._frame_id, stamp))
        self._sent[cmd.kind] += 1

    def _publish_status(self) -> None:
        status = self.mission.status()
        status.update({"offboard": self._offboard, "landed": self._landed, "sent": dict(self._sent)})
        self._status_pub.publish(String(data=json.dumps(status)))


def body_velocity(q_wxyz: Any, velocity_ned: Any) -> Optional[tuple]:
    """PX4's horizontal velocity in the body's heading frame, ``(forward, left)`` m/s, from a
    ``VehicleOdometry``'s quaternion (``[w, x, y, z]``, FRD to NED) and NED velocity; None if
    unusable. Only PX4's own heading enters, so PX4's heading reference doesn't matter."""
    w, x, y, z = (float(v) for v in q_wxyz)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    vn, ve = float(velocity_ned[0]), float(velocity_ned[1])
    if not (math.isfinite(vn) and math.isfinite(ve) and math.isfinite(yaw)):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vn + s * ve, -(-s * vn + c * ve)


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
