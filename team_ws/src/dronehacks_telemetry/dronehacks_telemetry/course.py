"""The course map (contracts/course.md's YAML schema) and the frames a flight is placed on it in.

Course coordinates are metres, x east, y north, origin at the floor centre. The vision `odom`
frame is ENU and startup-relative; contracts/frames.md defines the course-to-odom transform as
p_odom = o_odom + R(yaw) (p_course - launch). PX4's local frame is NED. Every coordinate here
comes from the map file: this module holds no course geometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np

# The event drone's prop tips reach this far from its centre (contracts/course.md, derived budget)
PROP_REACH_M = 0.28

# A wall segment written without a thickness is the geometry table's 0.102 m (contracts/course.md)
DEFAULT_WALL_THICKNESS_M = 0.102


@dataclass(frozen=True, slots=True, kw_only=True)
class CourseMap:
    """The map's walls, boulders and goals."""

    segments: np.ndarray                     # (n, 2, 2) wall centre lines, start and end east, north
    thickness_m: np.ndarray                  # (n,) each wall's thickness
    boulders: np.ndarray                     # (m, 3) east, north, radius
    goals: dict[str, tuple[float, float]]    # each goal's centre, east and north
    launch: tuple[float, float]              # the launch mark, east and north
    launch_heading_deg: float                # the launch heading, clockwise from course north


class CourseToOdom(NamedTuple):
    """contracts/frames.md's course-to-odom transform, horizontal part."""

    yaw_rad: float     # rotates course vectors into odom vectors, about +z
    odom_x_m: float    # odom reading of the launch reference
    odom_y_m: float


class Px4Frame(str, Enum):
    """How PX4's local NED frame sits against the vision odom frame."""

    # EKF2 fuses vision yaw in a FRD frame: its local NED is the odom ENU frame converted
    ODOM = "odom"
    # PX4 keeps its own heading: local x along the launch heading, y to its right, from the mark
    LAUNCH = "launch"


def load_course_map(path: Path) -> CourseMap:
    """Reads a course map YAML file. Raises `ValueError` for a map without a launch goal."""
    try:
        import yaml
    except ImportError as missing:
        raise ImportError("reading the course map needs PyYAML: pip install pyyaml") from missing

    return course_map_from_dict(yaml.safe_load(path.read_text(encoding="utf-8")))


def course_map_from_dict(data: dict[str, Any]) -> CourseMap:
    """The map from its parsed YAML. Raises `ValueError` for a map without a launch goal."""
    segments = np.array([[segment["start"], segment["end"]] for segment in data.get("segments", [])],
                        dtype=float).reshape(-1, 2, 2)
    thickness_m = np.array([float(segment.get("thickness") or DEFAULT_WALL_THICKNESS_M)
                            for segment in data.get("segments", [])])
    boulders = np.array([[*boulder["centre"], boulder["radius"]] for boulder in data.get("boulders", [])],
                        dtype=float).reshape(-1, 3)
    goals = {goal["name"]: (float(goal["centre"][0]), float(goal["centre"][1]))
             for goal in data.get("goals", [])}
    launch = next((goal for goal in data.get("goals", []) if goal["name"] == "launch"), None)
    if launch is None:
        raise ValueError("the course map has no launch goal: the flight cannot be placed on it")

    return CourseMap(segments=segments, thickness_m=thickness_m, boulders=boulders, goals=goals,
                     launch=goals["launch"], launch_heading_deg=float(launch["params"]["heading_deg"]))


def launch_transform(course: CourseMap) -> CourseToOdom:
    """The transform of a tracker started on the launch mark facing the launch heading: odom x
    forward, origin at the mark. Assumed, not measured; contracts/frames.md wants it measured."""
    return CourseToOdom(yaw_rad=math.radians(course.launch_heading_deg) - math.pi / 2,
                        odom_x_m=0.0, odom_y_m=0.0)


def odom_from_px4(local_ned: np.ndarray, heading_rad: np.ndarray,
                  frame: Px4Frame) -> tuple[np.ndarray, np.ndarray]:
    """PX4's (n, 3) local NED positions and headings as odom ENU positions and yaws."""
    north, east, down = local_ned.T
    if frame is Px4Frame.ODOM:
        return np.column_stack([east, north, -down]), np.pi / 2 - heading_rad

    return np.column_stack([north, -east, -down]), -heading_rad


def course_from_odom(odom_xyz: np.ndarray, yaw_rad: np.ndarray, course: CourseMap,
                     transform: CourseToOdom) -> tuple[np.ndarray, np.ndarray]:
    """(n, 3) odom ENU positions and yaws as course east, north, up-from-origin and course yaws,
    inverting p_odom = o_odom + R(yaw) (p_course - launch)."""
    cos_yaw, sin_yaw = math.cos(transform.yaw_rad), math.sin(transform.yaw_rad)
    dx = odom_xyz[:, 0] - transform.odom_x_m
    dy = odom_xyz[:, 1] - transform.odom_y_m

    # R(-yaw) undoes the course-to-odom rotation
    east = course.launch[0] + cos_yaw * dx + sin_yaw * dy
    north = course.launch[1] - sin_yaw * dx + cos_yaw * dy
    return np.column_stack([east, north, odom_xyz[:, 2]]), yaw_rad - transform.yaw_rad


def wall_clearance_m(points_en: np.ndarray, course: CourseMap) -> np.ndarray:
    """Each (n, 2) point's horizontal distance to the nearest wall face; walls are taken to span
    the flight height, as the map is sliced there."""
    if course.segments.size == 0:
        return np.full(len(points_en), np.inf)

    starts, ends = course.segments[:, 0], course.segments[:, 1]
    along = ends - starts
    length_sq = np.maximum(np.sum(along**2, axis=1), 1e-12)

    # Project every point onto every segment, clamped to its ends
    offsets = points_en[:, None, :] - starts[None, :, :]
    fraction = np.clip(np.sum(offsets * along[None], axis=2) / length_sq[None], 0.0, 1.0)
    nearest = starts[None] + fraction[..., None] * along[None]
    centre_line_m = np.linalg.norm(points_en[:, None, :] - nearest, axis=2)
    return np.min(centre_line_m - course.thickness_m[None] / 2, axis=1)


def boulder_clearance_m(points_en: np.ndarray, course: CourseMap) -> np.ndarray:
    """Each (n, 2) point's horizontal distance to the nearest boulder's edge; a boulder below the
    flight height is passed over, so this is for reading, not for touches."""
    if course.boulders.size == 0:
        return np.full(len(points_en), np.inf)

    centres, radii = course.boulders[:, :2], course.boulders[:, 2]
    return np.min(np.linalg.norm(points_en[:, None, :] - centres[None], axis=2) - radii[None], axis=1)
