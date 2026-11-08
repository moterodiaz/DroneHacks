"""Map to flyable trajectory: plan, smooth, profile, verify.

``build`` runs one route through every stage. ``fastest`` does that for every
combination of alternatives (Gate B by default; the window only when
``PlannerConfig.allow_windows`` is set, since ``contracts/course.md`` leaves
its ``clearance_radius`` unreviewed) and keeps the one with the shortest
flight time, which accounts for turns and speed limits, not just length.
Anything that fails a check is reported, not used.
"""

from __future__ import annotations

import dataclasses
import itertools
import time
from typing import Dict, List, Optional, Tuple

from .course_map import CourseMap
from .limits import VehicleLimits
from .route import PlannerConfig, PlanningError, Route, make_field, plan
from .smoothing import SmoothingConfig, SmoothingError, SmoothPath, smooth_route
from .speed import ProfileConfig, ProfileError, SpeedProfile, check_profile, speed_profile
from .trajectory import Trajectory


#: Jerk targets to try, as fractions of the vehicle's j_max, in order.
JERK_TARGET_SCALES = (1.0, 0.95, 0.9, 0.8)


@dataclasses.dataclass
class FlightPlan:
    route: Route
    path: SmoothPath
    profile: SpeedProfile
    trajectory: Trajectory
    report: Dict[str, object]
    compute_s: float

    @property
    def duration(self) -> float:
        return self.trajectory.duration


def build(cmap: CourseMap, planner: PlannerConfig = PlannerConfig(), limits: VehicleLimits = VehicleLimits(),
          smoothing: SmoothingConfig = SmoothingConfig(), profile: ProfileConfig = ProfileConfig(),
          route: Optional[Route] = None) -> FlightPlan:
    t0 = time.perf_counter()
    field = make_field(cmap, planner)
    route = route if route is not None else plan(cmap, planner)
    path = smooth_route(route, cmap, field, planner, smoothing)
    # The profile is built to j_max but only approximately hits it, so a route can miss the check
    # by a rounding-sized amount. Build to a tighter target and re-check against the real limits:
    # slower, never looser. Anything still violated is reported as before.
    prof, report, error = None, None, None
    for scale in JERK_TARGET_SCALES:
        target = dataclasses.replace(limits, j_max_mps3=limits.j_max_mps3 * scale)
        try:
            candidate = speed_profile(path, target, field, planner, profile)
        except ProfileError as exc:
            error = exc
            continue
        prof, report = candidate, check_profile(path, candidate, limits)
        if not report["violations"]:
            break
    if prof is None:
        raise error
    report["min_margin_bound_m"] = path.min_margin_m
    report["min_clearance_m"] = path.min_clearance_m
    return FlightPlan(route=route, path=path, profile=prof, trajectory=Trajectory(path, prof),
                      report=report, compute_s=time.perf_counter() - t0)


def fastest(cmap: CourseMap, planner: PlannerConfig = PlannerConfig(), limits: VehicleLimits = VehicleLimits(),
            smoothing: SmoothingConfig = SmoothingConfig(), profile: ProfileConfig = ProfileConfig(),
            ) -> Tuple[FlightPlan, List[Tuple[Dict[str, str], str]]]:
    """The quickest verified plan over every combination of alternatives.

    Returns it and, per combination, ``(choices, outcome)``: the flight time,
    or why that combination was dropped. A combination is dropped if it
    fails to plan, or if its profile reports any violation. If the planner
    config already prefers specific goals, only those combinations are tried.
    """
    stages = cmap.stages()
    if not planner.allow_windows:
        # The window is opt-in (unreviewed clearance_radius), but one the caller
        # explicitly names in prefer is a choice, not a default — route.plan
        # keeps it, so fastest must too.
        stages = [tuple(g for g in stage if g.shape != "rect" or g.name in planner.prefer) or stage
                  for stage in stages]
    groups = [stage for stage in stages if len(stage) > 1]
    options = [[g.name for g in stage if not planner.prefer or g.name in planner.prefer] or
               [g.name for g in stage] for stage in groups]
    best: Optional[FlightPlan] = None
    table: List[Tuple[Dict[str, str], str]] = []
    for combo in itertools.product(*options) if options else [()]:
        choices = {stage[0].group: name for stage, name in zip(groups, combo)}
        cfg = dataclasses.replace(planner, prefer=tuple(combo) + tuple(planner.prefer))
        try:
            fp = build(cmap, cfg, limits, smoothing, profile)
        except (PlanningError, SmoothingError, ProfileError) as exc:
            table.append((choices, f"failed: {exc}"))
            continue
        if fp.report["violations"]:
            table.append((choices, f"rejected: {'; '.join(fp.report['violations'])}"))
            continue
        table.append((choices, f"{fp.duration:.2f} s"))
        if best is None or fp.duration < best.duration:
            best = fp
    if best is None:
        raise PlanningError("no combination of alternatives gives a verified plan: "
                            + "; ".join(f"{c}: {o}" for c, o in table))
    return best, table
