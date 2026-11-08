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
from dronehacks_planner.route import PlannerConfig, PlanningError, make_field

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
    #: The speed the trajectory was planned at, and why it is lower than asked (None: as asked).
    v_max_mps: float = 0.0
    speed_note: Optional[str] = None


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
    # by twist, the interface clamps every command to its max_speed_mps: a plan above it would run
    # away from the drone
    v = min(track_speed_mps, limits.v_max_mps, config.twist_speed_max_mps)
    limits = dataclasses.replace(limits, v_max_mps=v)
    return limits, dataclasses.replace(config, velocity_follow=True)


#: The slowest a plan is tried at when a faster one fails its own checks.
MIN_PLAN_SPEED_MPS = 0.3


def plan_fastest_verified(cmap, planner: PlannerConfig, limits: VehicleLimits):
    """``fastest`` at ``limits.v_max_mps``, or, if no plan verifies at that speed, 10 % slower each
    time down to MIN_PLAN_SPEED_MPS: ``(plan, alternatives, limits, note)``. A faster speed
    profile can break a limit of its own (along-path jerk at a sharp turn: the window route over
    the boulders at 1.5 m/s and up) that a slower one keeps; without this the node stops before
    it publishes anything, so the kit refuses Offboard and the drone never takes off."""
    asked = limits.v_max_mps
    v = asked
    first_error = None
    while True:
        try:
            plan, alternatives = fastest(cmap, planner, dataclasses.replace(limits, v_max_mps=v))
        except PlanningError as exc:
            slower = round(v * 0.9, 3)
            if slower < MIN_PLAN_SPEED_MPS:
                raise
            first_error = first_error or str(exc)
            v = slower
            continue
        note = None if v == asked else (f"no plan verified at {asked:g} m/s ({first_error}); "
                                        f"planned at {v:g} m/s")
        return plan, alternatives, dataclasses.replace(limits, v_max_mps=v), note


def prepare(map_source, mission_config: MissionConfig, limits: VehicleLimits = VehicleLimits(),
            planner: PlannerConfig = PlannerConfig(), follower: FollowerConfig = FollowerConfig(),
            frame: Optional[OdomFrame] = None, track_speed_mps: float = 0.0,
            low_obstacles=None) -> Prepared:
    """``map_source``: a path or a CourseMap. Raises on anything that can't be planned or verified.

    ``track_speed_mps`` > 0 flies the course at that speed (:func:`for_track_speed`); 0 keeps
    the plan at ``limits.v_max_mps`` and FOLLOW on reference poses (the shipped behaviour).

    ``frame`` is the **measured** course->odom transform (contracts/frames.md), and it is
    required: it is never derived from the map's launch heading. The node passes it from its
    required parameters; the offline model builds it with ``frames.model_odom_frame``.

    ``low_obstacles``: the boulders the mission keeps its height over on a rolling take-off or an
    early descent. None: the map's own (pass the full map's when the plan dropped them)."""
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
    plan, alternatives, limits, note = plan_fastest_verified(cmap, planner, limits)
    mission = Mission(plan, frame, make_field(cmap, planner), cmap.slice_height, limits, mission_config,
                      follower, launch_heading=launch_heading,
                      low_obstacles=cmap.boulders if low_obstacles is None else tuple(low_obstacles))
    return Prepared(cmap, plan, frame, mission, alternatives, time.perf_counter() - t0,
                    limits.v_max_mps, note)


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
