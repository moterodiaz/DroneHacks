"""Map file in, a ready Mission out: plan (fastest verified alternative), smooth, profile, frame."""

from __future__ import annotations

import dataclasses
import time
from typing import Optional

from dronehacks_planner.course_map import CourseMap, load_map
from dronehacks_planner.follower import FollowerConfig
from dronehacks_planner.limits import VehicleLimits
from dronehacks_planner.pipeline import FlightPlan, fastest
from dronehacks_planner.route import PlannerConfig, make_field

from .frames import OdomFrame
from .mission import Mission, MissionConfig


@dataclasses.dataclass
class Prepared:
    cmap: CourseMap
    plan: FlightPlan
    frame: OdomFrame
    mission: Mission
    alternatives: list
    seconds: float


def prepare(map_source, mission_config: MissionConfig, limits: VehicleLimits = VehicleLimits(),
            planner: PlannerConfig = PlannerConfig(), follower: FollowerConfig = FollowerConfig(),
            frame: Optional[OdomFrame] = None) -> Prepared:
    """``map_source``: a path or a CourseMap. Raises on anything that can't be planned or verified."""
    t0 = time.perf_counter()
    cmap = map_source if isinstance(map_source, CourseMap) else load_map(map_source)
    if frame is None:
        if cmap.launch is None or cmap.launch.heading is None:
            raise ValueError("the map has no launch position and heading; the odom frame is placed by them")
        frame = OdomFrame.from_launch(cmap.launch.position, cmap.launch.heading)
    plan, alternatives = fastest(cmap, planner, limits)
    mission = Mission(plan, frame, make_field(cmap, planner), cmap.slice_height, limits, mission_config, follower)
    return Prepared(cmap, plan, frame, mission, alternatives, time.perf_counter() - t0)


def flight_checks(prepared: Prepared, positions_odom, planner: PlannerConfig = PlannerConfig()) -> dict:
    """Wall touches, openings and pad landing for a flown path given in the odom frame."""
    import numpy as np

    from dronehacks_planner.sim import landed_on, openings_crossed

    frame, cmap, route = prepared.frame, prepared.cmap, prepared.plan.route
    xy = np.array([frame.to_course(float(p[0]), float(p[1])) for p in positions_odom])
    airborne = np.asarray(positions_odom)[:, 2] > 0.05
    field = make_field(cmap, planner)
    wall = field.wall_distance(xy[airborne]) if airborne.any() else np.array([np.inf])
    touching = wall < planner.drone_extent_m
    pad = next((g.name for g in cmap.goals if g.shape == "disc"), None)
    return {
        "wall_touches": int(np.sum(touching[1:] & ~touching[:-1]) + (1 if touching[:1].any() else 0)),
        "min_wall_clearance_m": round(float(wall.min()), 3),
        "openings": openings_crossed(xy, route, cmap, planner.drone_extent_m),
        "on_pad": landed_on(xy[-1], cmap, pad, planner.drone_extent_m) if pad else None,
    }
