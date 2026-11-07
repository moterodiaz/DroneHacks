"""Reads a rosbag2 mcap folder into the time series the flight report measures.

Every series is timed by the recorder's receive time (mcap `log_time`), in seconds from the
bag's first message: one clock for every topic, PX4's included. Header stamps are kept beside it
as `stamp_s`, absolute seconds on the publisher's ROS clock, for rates and latency. A topic is
found by role, and PX4 v1.16's versioned names (`vehicle_status_v1`) match their role too.
Image topics are never decoded; `camera_info` carries the same header stamps.
"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# Each role's topic, as a pattern; `_v<n>` is PX4 v1.16's message version suffix
ROLE_PATTERNS = {
    "local_position": r"^/fmu/out/vehicle_local_position(_v\d+)?$",
    "status": r"^/fmu/out/vehicle_status(_v\d+)?$",
    "land_detected": r"^/fmu/out/vehicle_land_detected(_v\d+)?$",
    "fusion_flags": r"^/fmu/out/estimator_status_flags(_v\d+)?$",
    "failsafe_flags": r"^/fmu/out/failsafe_flags(_v\d+)?$",
    "distance": r"^/fmu/out/distance_sensor(_v\d+)?$",
    "trajectory_setpoint": r"^/fmu/in/trajectory_setpoint(_v\d+)?$",
    "bridge": r"^/fmu/in/vehicle_visual_odometry(_v\d+)?$",
    "pose_setpoint": r"^/skybeta/setpoint/pose$",
    "kit_state": r"^/skybeta/state$",
    "vision": r"^/visual_slam/tracking/odometry$",
    "vision_status": r"^/visual_slam/status$",
    "camera": r"^/oak/left/camera_info$",
    "imu": r"^/oak/imu/data$",
    "clock": r"^/clock$",
    "rosout": r"^/rosout$",
}

# rcl_interfaces/Log levels; WARN and worse go on the timeline
_ROSOUT_WARN = 30


@dataclass(frozen=True, slots=True)
class Series:
    """One topic's messages as arrays."""

    times_s: np.ndarray                                   # receive time, from the bag's start
    fields: dict[str, np.ndarray] = field(default_factory=dict)   # each extracted field, as floats


@dataclass(frozen=True, slots=True, kw_only=True)
class LogLine:
    """One /rosout line."""

    time_s: float      # receive time, from the bag's start
    level: int         # rcl_interfaces/Log level: 10 debug to 50 fatal
    node: str          # the logging node's name
    text: str          # the message


@dataclass(frozen=True, slots=True, kw_only=True)
class BagFlight:
    """What the report reads from one bag."""

    start_ns: int                        # the first message's receive time, ns since the epoch
    duration_s: float                    # from the first message to the last
    topics: dict[str, int]               # every topic in the bag and its message count
    series: dict[str, Series]            # each role found, by role name
    log: list[LogLine]                   # /rosout warnings and worse, and the team's own lines
    resolved: dict[str, str]             # each role's topic name


def find_bag_files(folder: Path) -> list[Path]:
    """The mcap files of a bag folder, or of its `bag/` subfolder; a rosbag2 file-compressed
    `.mcap.zstd` is decompressed beside itself first. Raises `FileNotFoundError` with none."""
    for candidate in (folder, folder / "bag"):
        files = sorted(candidate.glob("*.mcap"))
        if files:
            return files

        compressed = sorted(candidate.glob("*.mcap.zstd"))
        if compressed:
            return [_decompressed(path) for path in compressed]

    raise FileNotFoundError(f"no .mcap file in {folder} or {folder / 'bag'}")


def _decompressed(path: Path) -> Path:
    """A zstd-compressed mcap written out decompressed beside it, as `zstd -d` would."""
    try:
        import zstandard
    except ImportError as missing:
        raise ImportError(f"{path.name} is zstd-compressed: pip install zstandard, or run "
                          f"`zstd -d {path}`") from missing

    target = path.with_suffix("")
    with path.open("rb") as source, target.open("wb") as sink:
        zstandard.ZstdDecompressor().copy_stream(source, sink)

    return target


def resolve_roles(topics: Iterable[str]) -> dict[str, str]:
    """Each role's topic among `topics`; the shortest name wins where several match."""
    resolved = {}
    for role, pattern in ROLE_PATTERNS.items():
        matches = sorted((topic for topic in topics if re.match(pattern, topic)), key=len)
        if matches:
            resolved[role] = matches[0]

    return resolved


def read_bag(folder: Path) -> BagFlight:
    """Reads the roles' topics from a bag folder's mcap files."""
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as missing:
        raise ImportError("reading a bag needs mcap: pip install mcap mcap-ros2-support") from missing

    files = find_bag_files(folder)

    # The topic list and counts come from the summaries, before anything is decoded
    topics: dict[str, int] = {}
    for path in files:
        with path.open("rb") as stream:
            summary = make_reader(stream).get_summary()
            if summary is None or summary.statistics is None:
                continue

            for channel_id, channel in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(channel_id, 0)
                topics[channel.topic] = topics.get(channel.topic, 0) + count

    resolved = resolve_roles(topics)
    by_topic = {topic: role for role, topic in resolved.items()}
    rows: dict[str, list[tuple[int, Any]]] = {role: [] for role in resolved}
    for path in files:
        with path.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _, channel, record, message in reader.iter_decoded_messages(topics=list(by_topic)):
                rows[by_topic[channel.topic]].append((record.log_time, message))

    return bag_flight_from_rows(rows, topics, resolved)


def bag_flight_from_rows(rows: dict[str, list[tuple[int, Any]]], topics: dict[str, int],
                         resolved: dict[str, str]) -> BagFlight:
    """The flight from each role's (log time ns, decoded message) rows."""
    every_ns = [log_ns for role_rows in rows.values() for log_ns, _ in role_rows]
    if not every_ns:
        raise ValueError("the bag holds none of the topics the report reads")

    start_ns = min(every_ns)
    series = {}
    log: list[LogLine] = []
    for role, role_rows in rows.items():
        role_rows = sorted(role_rows, key=lambda row: row[0])
        if role == "rosout":
            log = _log_lines(role_rows, start_ns)
        elif role_rows:
            series[role] = _series(role, role_rows, start_ns)

    return BagFlight(start_ns=start_ns, duration_s=(max(every_ns) - start_ns) * 1e-9, topics=topics,
                     series=series, log=log, resolved=resolved)


def stamp_s(header: Any) -> float:
    """A ROS header's stamp in seconds."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    """Yaw about +z of an XYZW quaternion, radians."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _series(role: str, rows: list[tuple[int, Any]], start_ns: int) -> Series:
    """One role's rows as arrays, through that role's field extractor."""
    extract = _EXTRACTORS.get(role, _no_fields)
    records = [extract(message) for _, message in rows]
    times_s = np.array([(log_ns - start_ns) * 1e-9 for log_ns, _ in rows])
    names = {name for record in records for name in record}
    fields = {name: np.array([record.get(name, np.nan) for record in records], dtype=float)
              for name in sorted(names)}
    return Series(times_s=times_s, fields=fields)


def _log_lines(rows: list[tuple[int, Any]], start_ns: int) -> list[LogLine]:
    """The /rosout lines worth a timeline entry: warnings and worse, and the team's nodes."""
    lines = []
    for log_ns, message in rows:
        name = str(message.name)
        if message.level >= _ROSOUT_WARN or "dronehacks" in name or "mission" in name:
            lines.append(LogLine(time_s=(log_ns - start_ns) * 1e-9, level=int(message.level),
                                 node=name, text=str(message.msg)))

    return lines


def _no_fields(_: Any) -> dict[str, float]:
    """No fields: the series is its receive times alone."""
    return {}


def _header_only(message: Any) -> dict[str, float]:
    """The header stamp alone."""
    return {"stamp_s": stamp_s(message.header)}


def _local_position(message: Any) -> dict[str, float]:
    """PX4's local position estimate, NED metres and heading radians from its x axis."""
    return {"x": message.x, "y": message.y, "z": message.z, "heading": message.heading,
            "dist_bottom": getattr(message, "dist_bottom", np.nan),
            "dist_bottom_valid": float(getattr(message, "dist_bottom_valid", False)),
            "xy_reset_counter": getattr(message, "xy_reset_counter", 0)}


def _status(message: Any) -> dict[str, float]:
    """Arming, the navigation state and the failsafe flag."""
    return {"nav_state": message.nav_state, "arming_state": message.arming_state,
            "failsafe": float(message.failsafe)}


def _land_detected(message: Any) -> dict[str, float]:
    """Whether PX4's land detector reports landed."""
    return {"landed": float(message.landed)}


def _fusion_flags(message: Any) -> dict[str, float]:
    """EKF2's vision and range fusion flags."""
    return {name: float(getattr(message, name, np.nan))
            for name in ("cs_ev_pos", "cs_ev_yaw", "cs_ev_hgt", "cs_rng_hgt", "cs_in_air")}


def _failsafe_flags(message: Any) -> dict[str, float]:
    """Every failsafe condition; `mode_req_*` are bitmasks of modes, not conditions."""
    return {name: float(getattr(message, name)) for name in type(message).__slots__
            if name != "timestamp" and not name.startswith("mode_req_")
            and isinstance(getattr(message, name), (bool, int))}


def _distance(message: Any) -> dict[str, float]:
    """A rangefinder reading and its valid range."""
    return {"current_distance": message.current_distance, "min_distance": message.min_distance,
            "max_distance": message.max_distance, "signal_quality": message.signal_quality}


def _trajectory_setpoint(message: Any) -> dict[str, float]:
    """The kit's PX4 setpoint: NED position, NaN where it commands velocity, and yaw."""
    x, y, z = (float(value) for value in message.position)
    return {"x": x, "y": y, "z": z, "yaw": float(message.yaw)}


def _bridge(message: Any) -> dict[str, float]:
    """What the bridge sends PX4: its two stamps, microseconds, and the reset counter."""
    return {"timestamp_us": float(message.timestamp), "sample_us": float(message.timestamp_sample),
            "reset_counter": float(message.reset_counter), "pose_frame": float(message.pose_frame)}


def _pose(message: Any) -> dict[str, float]:
    """A stamped pose's position and yaw."""
    position, orientation = message.pose.position, message.pose.orientation
    return {"stamp_s": stamp_s(message.header), "x": position.x, "y": position.y, "z": position.z,
            "yaw": yaw_from_xyzw(orientation.x, orientation.y, orientation.z, orientation.w)}


def _odometry(message: Any) -> dict[str, float]:
    """cuVSLAM's odometry: ENU position and yaw in its odom frame, and the image stamp."""
    position, orientation = message.pose.pose.position, message.pose.pose.orientation
    return {"stamp_s": stamp_s(message.header), "x": position.x, "y": position.y, "z": position.z,
            "yaw": yaw_from_xyzw(orientation.x, orientation.y, orientation.z, orientation.w)}


def _vision_status(message: Any) -> dict[str, float]:
    """cuVSLAM's tracking state, 1 tracking, and its track time where the message carries it."""
    values = {"vo_state": float(message.vo_state)}
    for name in ("track_execution_time_mean", "track_execution_time_max"):
        if hasattr(message, name):
            values[name] = float(getattr(message, name))

    return values


def _kit_state(message: Any) -> dict[str, float]:
    """The flight interface's JSON state: link, arming, Offboard, and the envelope's flags."""
    try:
        state = json.loads(message.data)
    except ValueError:
        return {}

    return {"px4_link": float(bool(state.get("px4_link"))), "armed": float(bool(state.get("armed"))),
            "offboard_active": float(bool(state.get("offboard_active"))),
            "clamped": float(bool(state.get("clamped"))), "frozen": float(bool(state.get("frozen"))),
            "envelope_ok": float(bool(state.get("envelope_ok", True)))}


def _clock(message: Any) -> dict[str, float]:
    """The simulator's clock, seconds."""
    return {"sim_s": message.clock.sec + message.clock.nanosec * 1e-9}


_EXTRACTORS: dict[str, Callable[[Any], dict[str, float]]] = {
    "local_position": _local_position, "status": _status, "land_detected": _land_detected,
    "fusion_flags": _fusion_flags, "failsafe_flags": _failsafe_flags, "distance": _distance,
    "trajectory_setpoint": _trajectory_setpoint, "bridge": _bridge, "pose_setpoint": _pose,
    "kit_state": _kit_state, "vision": _odometry, "vision_status": _vision_status,
    "camera": _header_only, "imu": _header_only, "clock": _clock,
}
