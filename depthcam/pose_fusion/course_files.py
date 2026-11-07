"""Course facts read from the course files, so no code types a course coordinate (AGENTS.md §3.7).

The map (``course/maps/course_v3_map.json``, extracted from the organisers' STEP model by
``course/12_extract_course_map.py``) and its rules file (``course_v3_rules.json``) are the course
data. This module reads the few facts the localiser and the fusion need from them:

* the launch mark: the centre of the floor marking that the rules file names as the start;
* the wall tops and net tops: which map slice a scan plane sees, and which beams passed over
  everything.

The map's launch-mark centre agrees with contracts/course.md's "Launch mark" row to within
0.6 mm (the contract rounds to millimetres), checked 2026-11-06.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, Tuple, Union

from .frames import LaunchMark

#: The heading the drone faces on the launch mark: north, counter-clockwise from east in the course
#: frame (contracts/frames.md, "Origin": "facing north up the first corridor").
START_HEADING_NORTH = 0.5 * math.pi

PathLike = Union[str, Path]


def load_course_map(map_path: PathLike):
    """The :class:`course.course_map.CourseMap` in *map_path*."""
    from course.course_map import CourseMap  # lazy: keeps ``import pose_fusion`` light

    return CourseMap.load(Path(map_path))


def load_rules(rules_path: PathLike) -> Dict[str, Any]:
    with Path(rules_path).open("r", encoding="utf-8") as fh:
        rules = json.load(fh)
    if not isinstance(rules, dict) or "start" not in rules or "marking" not in rules["start"]:
        raise ValueError("{} has no start marking (expected {{\"start\": {{\"marking\": ...}}}})".format(rules_path))
    return rules


def start_marking_centre(cmap: Any, rules: Dict[str, Any]) -> Tuple[float, float]:
    """Course-frame (east, north) centre of the rules' start marking."""
    name = rules["start"]["marking"]
    feature = cmap.feature(name)
    if feature.get("kind") != "marking":
        raise ValueError("start {!r} is a {}, not a floor marking".format(name, feature.get("kind")))
    east, north = (float(v) for v in feature["centre"][:2])
    return east, north


def launch_mark_from_files(map_path: PathLike, rules_path: PathLike) -> LaunchMark:
    """The :class:`LaunchMark` named by the rules file, located in the map."""
    east, north = start_marking_centre(load_course_map(map_path), load_rules(rules_path))
    return LaunchMark(east, north)


def wall_and_net_tops(cmap: Any) -> Tuple[float, float]:
    """Highest wall top and highest net top in the map, in metres above the floor."""
    walls = cmap.of_kind("wall")
    nets = cmap.of_kind("net")
    if not walls or not nets:
        raise ValueError("the map needs walls and nets (found {} walls, {} nets)".format(len(walls), len(nets)))
    return max(float(f["z"][1]) for f in walls), max(float(f["z"][1]) for f in nets)
