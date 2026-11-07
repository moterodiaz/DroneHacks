"""Measure the course->odom transform with the drone sitting on the launch mark. Read-only.

    ros2 run dronehacks_autonomy measure_frame [--map MAP.yaml] [--seconds 3] [--out FILE]

contracts/frames.md requires a measured transform at each tracking initialisation.
This takes that measurement: a person places the drone on the mark facing the launch
heading, and this averages the odometry for a few seconds, then writes the three
``course_to_odom_*`` parameters into the team's parameter file (other keys are kept).
Restart the ``competitor`` window afterwards so the mission reads them.

The reading is only as good as the placement. It refuses a drone that moves during the
measurement. It does not replace the props-off carry test in contracts/frames.md.

It subscribes to odometry and publishes nothing, so it cannot move the vehicle.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, Sequence, Tuple

import yaml

from .contract_map import load_contract_map
from .frames import frame_from_mark, quat_to_yaw, wrap
from .planning.course_map import CourseMap

DEFAULT_TOPIC = "/visual_slam/tracking/odometry"
DEFAULT_OUT = "/root/team_ws/autonomy_params.yaml"
DEFAULT_TEAM_MAP = "/root/team_ws/course_map.yaml"
#: More movement than this during the measurement means the drone was not resting.
MAX_SPREAD_M = 0.02
MAX_SPREAD_RAD = math.radians(1.0)
MIN_SAMPLES = 10


class MeasureError(RuntimeError):
    pass


def average(samples: Sequence[Tuple[float, float, float]]) -> Tuple[float, float, float]:
    """Mean ``(x, y, yaw)`` of still odometry samples. Raises if they are too few or moving."""
    if len(samples) < MIN_SAMPLES:
        raise MeasureError(f"only {len(samples)} odometry samples (need {MIN_SAMPLES}): is the stack running?")
    xs, ys, yaws = zip(*samples)
    if max(xs) - min(xs) > MAX_SPREAD_M or max(ys) - min(ys) > MAX_SPREAD_M:
        raise MeasureError(f"the drone moved during the measurement (more than {MAX_SPREAD_M} m)")
    yaw = math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))
    if max(abs(wrap(y - yaw)) for y in yaws) > MAX_SPREAD_RAD:
        raise MeasureError("the heading changed during the measurement (more than 1 degree)")
    return sum(xs) / len(xs), sum(ys) / len(ys), yaw


def parameters(cmap: CourseMap, x: float, y: float, yaw: float) -> Dict[str, float]:
    """The ``course_to_odom_*`` parameters for a drone on this map's launch mark at this odom reading."""
    if cmap.launch is None or cmap.launch.heading is None:
        raise MeasureError("the map has no launch position and heading")
    frame = frame_from_mark(cmap.launch.position, cmap.launch.heading, x, y, yaw)
    return {"course_to_odom_yaw": frame.yaw, "course_to_odom_x": frame.x, "course_to_odom_y": frame.y}


def merge_into_file(path: str, values: Dict[str, float]) -> None:
    """Write ``values`` under the mission node's parameters, keeping everything else in the file."""
    doc = {}
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
    doc.setdefault("mission", {}).setdefault("ros__parameters", {}).update(values)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    os.replace(tmp, path)


def _collect(topic: str, seconds: float):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    samples = []

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        samples.append((float(p.x), float(p.y), quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))))

    rclpy.init()
    node = rclpy.create_node("measure_frame")
    try:
        node.create_subscription(Odometry, topic, on_odom,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return samples


def _default_map() -> str:
    if os.path.isfile(DEFAULT_TEAM_MAP):
        return DEFAULT_TEAM_MAP
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("dronehacks_autonomy"), "maps", "course_v3_layout.yaml")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="measure_frame", description=__doc__.splitlines()[0])
    ap.add_argument("--map", help="course map (default: the team's map if present, else the packaged one)")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args(argv)
    try:
        map_file = args.map or _default_map()
        cmap = load_contract_map(map_file)
        x, y, yaw = average(_collect(args.topic, args.seconds))
        values = parameters(cmap, x, y, yaw)
        merge_into_file(args.out, values)
    except Exception as exc:  # noqa: BLE001 - report it; the file was not changed
        print(f"measure_frame: {exc}", file=sys.stderr)
        return 2
    print(f"map {map_file}\nodom on the mark: x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg")
    print(f"wrote {args.out}: {values}\nRestart the competitor window so the mission reads them.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
