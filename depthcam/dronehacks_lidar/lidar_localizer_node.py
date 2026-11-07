"""ROS 2 Humble node ``lidar_localizer``: MS200K lidar -> body pose on the course map.

    MS200K (USB serial) ─► lidar_localizer ─► ~/pose (geometry_msgs/PoseWithCovarianceStamped, course frame)
                              ▲                        │
    /fmu/out/vehicle_odometry ┘ (read only)            └─► pose_fusion (lidar_topic)

For each revolution it:

* reads the serial stream itself (``course/ms200.py``), to keep every point's own time; the
  vendor ROS node drops those, and the deskew needs them;
* matches the scan to the course map (``pose_fusion.localizer.LidarLocalizer``, built on the
  bench-tested ``course/lidar_localize.py``), tilt-corrected and deskewed with PX4's attitude,
  velocity and yaw rate;
* publishes the body pose with a covariance once 3 matches in a row are trusted. It never
  publishes a match the tracker calls lost.

It only *reads* PX4 (``/fmu/out/*``, best-effort). It never writes to ``/fmu/in/*``, never sets a
parameter, and never arms or changes mode. It holds no state across runs and saves no map
(rule 6.4); the map is the published course. Start it with the drone resting on the launch mark,
facing north. That is its starting guess, and PX4's first height is taken as the floor.

``lidar_policy`` is required and has no default. Under ``forbidden`` the node exits (status 0)
before it opens the serial port.

Parameters (``required`` = no default):

    lidar_policy        "allowed" | "forbidden"                                   required
    port                the MS200K's serial device, e.g. /dev/ms200               required
    map_file, rules_file  course map and rules (the rules name the start mark)    required
    yaw_offset_deg      where the sensor's 0 deg points, CCW from the nose        required
    mount_x, mount_y, mount_z   scan centre in the body frame (FLU), m            required
    rest_height_m       body centre above the floor when landed, m                required
    clockwise           true if mounted right way up                              true
    range_min_m         ignore returns closer than this (the drone's own arms)    0.25
    latency_s           serial latency no frame avoids (course/ms200.py)          0.005
    baud                serial baud rate                                          230400
    frame_id            header.frame_id of the output (course frame)              course
    output_topic        where the pose goes                                       ~/pose
    px4_odometry_topic  PX4's odometry, read only ("auto" = resolved)            auto
    slice_heights       scan-plane heights with a map slice (fewer = faster start-up)   8 heights
    cov_scale, max_variance, max_px4_age_s, startup_good, iterations   LocalizerConfig tuning
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from course import ms200
from pose_fusion.course_files import load_course_map, load_rules, start_marking_centre
from pose_fusion.localizer import SLICE_HEIGHTS, LidarLocalizer, LocalizerConfig, Px4Buffer, px4_sample_from_odometry
from pose_fusion.policy import LidarPolicy, parse_lidar_policy
from pose_fusion.ros_io import fill_pose_with_covariance, s_to_stamp

from .params import declare, number, numbers
from .px4_topics import fmu_out

QUEUE_DEPTH = 10


class ConfigError(ValueError):
    pass


class LidarLocalizerNode(Node):
    def __init__(self, policy: LidarPolicy, **kwargs: Any) -> None:
        super().__init__("lidar_localizer", **kwargs)
        from px4_msgs.msg import VehicleOdometry

        self._vehicle_odometry = VehicleOdometry
        self.policy = policy
        self.declare_parameter("lidar_policy", policy.value)
        for name in ("port", "map_file", "rules_file"):
            self.declare_parameter(name, "")
        for name in ("yaw_offset_deg", "mount_x", "mount_y", "mount_z", "rest_height_m"):
            declare(self, name, float("nan"))
        self.declare_parameter("clockwise", True)
        declare(self, "range_min_m", 0.25)
        declare(self, "latency_s", 0.005)
        declare(self, "baud", int(ms200.BAUD))
        self.declare_parameter("frame_id", "course")
        self.declare_parameter("output_topic", "~/pose")
        self.declare_parameter("px4_odometry_topic", "auto")
        declare(self, "cov_scale", 100.0)
        declare(self, "max_variance", 1.0e4)
        declare(self, "max_px4_age_s", 0.3)
        declare(self, "startup_good", 3)
        declare(self, "iterations", 8)
        declare(self, "slice_heights", [float(z) for z in SLICE_HEIGHTS])

        # Validate everything, and build the map slices, before touching the port or the graph.
        port = self._required_str("port")
        map_file, rules_file = self._required_str("map_file"), self._required_str("rules_file")
        try:
            cmap = load_course_map(map_file)
            start = start_marking_centre(cmap, load_rules(rules_file))
            config = LocalizerConfig(
                mount_xyz=(self._required_float("mount_x"), self._required_float("mount_y"),
                           self._required_float("mount_z")),
                rest_height_m=self._required_float("rest_height_m"),
                cov_scale=number(self, "cov_scale"), max_variance=number(self, "max_variance"),
                max_px4_age_s=number(self, "max_px4_age_s"), startup_good=self._whole("startup_good"),
                iterations=self._whole("iterations"),
                slice_heights=tuple(numbers(self, "slice_heights")))
            range_min, latency, baud = number(self, "range_min_m"), number(self, "latency_s"), self._whole("baud")
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise ConfigError(str(exc)) from exc
        yaw_offset = self._required_float("yaw_offset_deg")
        self.get_logger().info("building map slices from {} (start mark {:.4f}, {:.4f})".format(map_file, *start))
        self.localizer = LidarLocalizer(cmap, start, config)
        self._asm = ms200.Assembler(clockwise=bool(self._param("clockwise")), yaw_offset_deg=yaw_offset,
                                    range_min=range_min, latency_s=latency)
        try:
            self._ser = ms200.open_port(port, baud)
        except (OSError, RuntimeError) as exc:
            raise ConfigError("cannot open the lidar on {}: {}".format(port, exc)) from exc

        self._frame_id = str(self._param("frame_id"))
        self._px4 = Px4Buffer()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_reason: Optional[str] = None
        self._px4_rejected = 0

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        px4_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        px4_topic = str(self._param("px4_odometry_topic"))
        if px4_topic == "auto":
            px4_topic = fmu_out("vehicle_odometry", "VehicleOdometry")
        self._policy_pub = self.create_publisher(String, "~/lidar_policy", latched)
        self._status_pub = self.create_publisher(String, "~/status", QUEUE_DEPTH)
        self._pub = self.create_publisher(PoseWithCovarianceStamped, str(self._param("output_topic")), QUEUE_DEPTH)
        self.create_subscription(VehicleOdometry, px4_topic, self._on_px4, px4_qos)
        self._policy_pub.publish(String(data=self.policy.value))
        self.create_timer(1.0, self._publish_status)
        self._thread = threading.Thread(target=self._read_lidar, name="ms200", daemon=True)
        self._thread.start()
        self.get_logger().info("lidar on {}, PX4 from {}, publishing {} in frame {!r}".format(
            port, px4_topic, self._pub.topic_name, self._frame_id))

    # ------------------------------------------------------------------ parameters

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _required_str(self, name: str) -> str:
        value = self._param(name)
        if not isinstance(value, str) or not value:
            raise ConfigError("parameter {} is required (it has no default)".format(name))
        return value

    def _required_float(self, name: str) -> float:
        value = self._param(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ConfigError("parameter {} is required: measure it (it has no default)".format(name))
        return float(value)

    def _whole(self, name: str) -> int:
        value = number(self, name)
        if not value.is_integer():
            raise ValueError("parameter {} must be a whole number (got {!r})".format(name, value))
        return int(value)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs

    def _on_px4(self, msg: Any) -> None:
        if msg.velocity_frame == self._vehicle_odometry.VELOCITY_FRAME_NED:
            vel = msg.velocity
        else:
            vel = None                              # the deskew then uses the tracker's own velocity
            self._px4_rejected += 1
        sample = px4_sample_from_odometry(self._now_s(), msg.q, msg.position, vel, msg.angular_velocity)
        if not all(math.isfinite(v) for v in (sample.roll_flu, sample.pitch_flu, sample.z_ned)):
            self._px4_rejected += 1
            return
        with self._lock:
            self._px4.add(sample)

    def _read_lidar(self) -> None:
        try:
            for rev in ms200.revolutions(ms200.read_port(self._ser), ms200.FrameParser(), self._asm):
                if self._stop.is_set():
                    break
                if not rev.clock_ok:
                    continue
                offset = self._now_s() - time.monotonic()      # host monotonic -> ROS clock
                with self._lock:
                    result = self.localizer.update(rev.t + offset, rev.range, rev.angle, self._px4)
                self._last_reason = result.reason
                if result.publish:
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = self._frame_id
                    msg.header.stamp.sec, msg.header.stamp.nanosec = s_to_stamp(result.t)
                    fill_pose_with_covariance(msg, result.x, result.y, result.z_body, result.yaw, result.cov)
                    self._pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 - report and stop publishing; the fusion falls back to VO
            if not self._stop.is_set():
                self.get_logger().error("lidar reader stopped: {}: {}".format(type(exc).__name__, exc))

    def _publish_status(self) -> None:
        with self._lock:
            status = dict(self.localizer.counts)
        status.update(lidar_policy=self.policy.value, reader_alive=self._thread.is_alive(),
                      last_reason=self._last_reason, px4_rejected=self._px4_rejected,
                      clock_resets=self._asm.clock_resets, dropped_partial=self._asm.dropped_partial)
        self._status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def read_policy() -> LidarPolicy:
    """The ``lidar_policy`` parameter, read by a short-lived node before anything else is opened."""
    probe = Node("lidar_localizer")
    try:
        probe.declare_parameter("lidar_policy", "")
        return parse_lidar_policy(probe.get_parameter("lidar_policy").value)
    finally:
        probe.destroy_node()


def main(args: Any = None) -> int:
    rclpy.init(args=args)
    try:
        policy = read_policy()
    except ValueError as exc:
        print("lidar_localizer: configuration error, nothing published: {}".format(exc), file=sys.stderr)
        rclpy.shutdown()
        return 2
    if policy is LidarPolicy.FORBIDDEN:
        print("lidar_localizer: lidar_policy is 'forbidden'; the lidar is not opened. Exiting.", file=sys.stderr)
        rclpy.shutdown()
        return 0
    try:
        node = LidarLocalizerNode(policy)
    except ConfigError as exc:
        print("lidar_localizer: configuration error, nothing published: {}".format(exc), file=sys.stderr)
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
