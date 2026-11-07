"""The route: the course planner's path through the checkpoints, in the true (tracker) frame.

Where a route comes from
------------------------
* **The map** is the organisers' STEP model (``dronehacks-course-v3.step``, copied as
  ``course/maps/course_v3.step``). ``course/12_extract_course_map.py`` turns it into
  ``course/maps/course_v3_map.json``, and re-extracting gives the same map.
* **The checkpoints** are the ground truth's: ``DRONEHACKS.md`` §4 and
  ``course/maps/course_v3_rules.json`` (CP1 end of the first corridor, CP2 Gate A, CP3 the
  tunnel's south end, CP4 the landing area through the window or Gate B, CP5 the pad).
* ``follower.make_route`` runs the ported planner (``course.planner.plan.plan_course``) on them
  and writes the result in this module's format, ``dronehacks-lidar-route/1``. The package
  ships ``routes/course_v3_window.json`` and ``routes/course_v3_gate_b.json``.

The format (course frame, ``(east, north)`` metres, as the planner works):

* ``path``: dense points (every 4 cm), launch mark to the pad centre, at ``cruise_z_m``;
* ``yaw_deg``: the one heading the whole route was planned for (course frame, counter-clockwise
  from east; 90 = nose north). The planner grows every obstacle by the drone's footprint *at
  this heading*, so the follower holds it all the way;
* ``gates``: the checkpoints crossed, as path indices;
* ``start``, ``goal``: the launch mark and the pad centre; ``drone``, ``min_room_m``: the
  footprint and margin it was planned with, and the tightest room left beyond that margin;
* ``map_sha256``, ``rules_sha256``: the files it came from (:func:`file_sha256`). Given the map
  and rules it will fly with, :func:`load_route` refuses a route planned from different ones.

The follower converts it to the tracker frame of contracts/frames.md (launch-mark origin,
x = north, y = west) with ``pose_fusion.frames.course_pose_to_tracker``, the same transform the
fusion uses. The tracker yaw is ``yaw_deg - 90``.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

import numpy as np

from pose_fusion.frames import LaunchMark, course_pose_to_tracker

ROUTE_FORMAT = "dronehacks-lidar-route/1"


@dataclass(frozen=True)
class Gate:
    index: int
    name: str


@dataclass(frozen=True)
class Route:
    path: np.ndarray                        # (N, 2) tracker frame, m
    yaw: float                              # the planned heading, tracker frame (rad)
    cruise_z_m: float                       # body centre above the floor
    gates: Tuple[Gate, ...]
    name: str = ""
    min_room_m: float = float("nan")
    margin_m: float = float("nan")
    meta: Dict[str, Any] = field(default_factory=dict, compare=False)

    @property
    def start(self) -> Tuple[float, float]:
        return float(self.path[0, 0]), float(self.path[0, 1])

    @property
    def goal(self) -> Tuple[float, float]:
        return float(self.path[-1, 0]), float(self.path[-1, 1])

    @property
    def length_m(self) -> float:
        return float(np.hypot(*np.diff(self.path, axis=0).T).sum())


def route_from_dict(data: Dict[str, Any], mark: LaunchMark) -> Route:
    if not isinstance(data, dict) or data.get("format") != ROUTE_FORMAT:
        fmt = data.get("format") if isinstance(data, dict) else None
        raise ValueError("not a {} route (format {!r}); make one with follower.make_route".format(ROUTE_FORMAT, fmt))
    path = np.asarray(data["path"], dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.all(np.isfinite(path)):
        raise ValueError("route path must be at least 2 finite [east, north] points")
    cruise = float(data["cruise_z_m"])
    yaw_deg = float(data["yaw_deg"])
    if not (math.isfinite(cruise) and cruise > 0.0 and math.isfinite(yaw_deg)):
        raise ValueError("route needs a finite cruise_z_m > 0 and yaw_deg")
    tracker = np.array([course_pose_to_tracker(e, n, 0.0, mark).as_tuple()[:2] for e, n in path])
    gates = tuple(Gate(int(g["index"]), str(g.get("name", ""))) for g in data.get("gates", []))
    for g in gates:
        if not 0 <= g.index < len(path):
            raise ValueError("gate {!r} index {} is outside the path".format(g.name, g.index))
    drone = data.get("drone") or {}
    return Route(tracker, math.radians(yaw_deg) - 0.5 * math.pi, cruise, gates, str(data.get("route", "")),
                 float(data.get("min_room_m", float("nan"))), float(drone.get("margin_m", float("nan"))),
                 {k: data[k] for k in ("map_sha256", "rules_sha256", "generated", "planner_finish_s") if k in data})


def file_sha256(path: Union[str, Path]) -> str:
    """SHA-256 of the file with LF line ends, as git stores it, so a Windows checkout gives the same value."""
    return hashlib.sha256(Path(path).read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def check_sources(data: Dict[str, Any], map_file: Union[str, Path, None] = None,
                  rules_file: Union[str, Path, None] = None) -> None:
    """Raise ValueError unless the route *data* records the hashes of *map_file* and *rules_file*.

    The planner checked the route's clearances against one map and one set of checkpoints. A route
    from other files may cross an obstacle or miss a gate.
    """
    for key, source in (("map_sha256", map_file), ("rules_sha256", rules_file)):
        if source is None:
            continue
        recorded = data.get(key) if isinstance(data, dict) else None
        actual = file_sha256(source)
        if recorded != actual:
            raise ValueError("the route was planned from a different {} file: it records {} {}, but {} is {}. "
                             "Re-plan it with follower.make_route".format(key[:-len("_sha256")], key, recorded,
                                                                        source, actual))


def load_route(path: Union[str, Path], mark: LaunchMark, map_file: Union[str, Path, None] = None,
               rules_file: Union[str, Path, None] = None) -> Route:
    """Read a route file. With *map_file* / *rules_file*, refuse it unless it was planned from them."""
    with Path(path).open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    check_sources(data, map_file, rules_file)
    return route_from_dict(data, mark)


def plan_to_dict(plan: Any, route: str, map_sha256: str, rules_sha256: str, generated: str) -> Dict[str, Any]:
    """A ``course.planner.plan.Plan`` in this module's format (course frame)."""
    r = lambda v: round(float(v), 5)  # noqa: E731
    gate_names: List[str] = [c.get("name", "") for c in plan.checkpoints]
    rooms = [c["min_room_m"] for c in plan.clearance if "min_room_m" in c]
    drone = plan.config.drone
    return {
        "format": ROUTE_FORMAT,
        "frame": "course (east, north), metres; the follower converts it to the tracker frame",
        "route": route,
        "yaw_deg": r(plan.config.yaw_deg),
        "cruise_z_m": r(plan.config.cruise_z_m),
        "path": [[r(e), r(n)] for e, n in plan.path],
        "gates": [{"index": int(i), "name": gate_names[k] if k < len(gate_names) else ""}
                  for k, i in enumerate(plan.gate_index)],
        "start": [r(v) for v in plan.start[:2]],
        "goal": [r(v) for v in plan.goal[:2]],
        "drone": {"length_m": drone.length_m, "width_m": drone.width_m, "height_m": drone.height_m,
                  "margin_m": drone.margin_m},
        "min_room_m": r(min(rooms)) if rooms else None,
        "planner_finish_s": r(plan.finish_s),
        "map_sha256": map_sha256,
        "rules_sha256": rules_sha256,
        "generated": generated,
    }
