"""Map file in, a ready Mission out: plan (fastest verified alternative), smooth, profile, frame."""

from __future__ import annotations

import dataclasses
import math
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


def for_track_speed(track_speed_mps: float, limits: VehicleLimits, config: MissionConfig):
    """The plan's limits and the mission config for a set track speed: ``(limits, config)``.

    * The trajectory's speed profile is capped at the track speed, never above ``v_max_mps``
      (PX4's own ``MPC_XY_VEL_MAX``).
    * FOLLOW flies it by velocity (``MissionConfig.velocity_follow``): the reference pose alone
      can't go faster here. PX4 flies a pose setpoint at ``MPC_XY_P`` x its distance, so it trails a
      reference moving at v by about v/``MPC_XY_P``: the governor then holds the flight near
      0.3 m/s, and relaxing it only lets PX4 cut the corners by that lag until the cross-track
      abort stops the drone (SITL, 2026-11-07). The kit accepts twists (contracts/ros-graph.md),
      and the reference velocity plus a pull onto the reference keeps the drone on the path at
      the planned speed: the law the dropout coast already flies.
    """
    if not (math.isfinite(track_speed_mps) and track_speed_mps > 0.0):
        raise ValueError(f"track_speed_mps must be > 0 (got {track_speed_mps!r})")
    limits = dataclasses.replace(limits, v_max_mps=min(track_speed_mps, limits.v_max_mps))
    return limits, dataclasses.replace(config, velocity_follow=True)


def prepare(map_source, mission_config: MissionConfig, limits: VehicleLimits = VehicleLimits(),
            planner: PlannerConfig = PlannerConfig(), follower: FollowerConfig = FollowerConfig(),
            frame: Optional[OdomFrame] = None, track_speed_mps: float = 0.0) -> Prepared:
    """``map_source``: a path or a CourseMap. Raises on anything that can't be planned or verified.

    ``track_speed_mps`` > 0 flies the course at that speed (:func:`for_track_speed`); 0 keeps
    the plan at ``limits.v_max_mps`` and FOLLOW on reference poses (the shipped behaviour).

    ``frame`` is the **measured** course->odom transform (contracts/frames.md), and it is
    required: it is never derived from the map's launch heading. The node passes it from its
    required parameters; the offline model builds it with ``frames.model_odom_frame``."""
    t0 = time.perf_counter()
    if track_speed_mps:
        limits, mission_config = for_track_speed(track_speed_mps, limits, mission_config)
    cmap = map_source if isinstance(map_source, CourseMap) else load_map(map_source)
    if frame is None:
        raise ValueError("the measured course->odom transform is required (contracts/frames.md): "
                          "pass OdomFrame.from_measured(yaw, o_odom); it has no default and is not "
                          "derived from the map's launch heading")
    launch = cmap.launch
    launch_heading = None if launch is None or launch.heading is None else math.atan2(
        float(launch.heading[1]), float(launch.heading[0]))
    plan, alternatives = fastest(cmap, planner, limits)
    mission = Mission(plan, frame, make_field(cmap, planner), cmap.slice_height, limits, mission_config,
                      follower, launch_heading=launch_heading)
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
