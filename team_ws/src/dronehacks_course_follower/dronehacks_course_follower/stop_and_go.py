"""Stop-and-go: the planner's waypoints flown as straight legs with a hold at each, as the square
test flew its box on the real drone.

At each waypoint the drone first yaws in place to face the next leg, then waits until it is over
the waypoint, facing the leg and steady, so the drift a yaw leaves (PX4 relearning an airframe
push in world axes) dies out before the leg; a hold that never settles goes on after a timeout.
Between holds a pose slides along the leg at a constant speed, heading held, and never runs more
than `LEAD_M` ahead of the drone. Course frame: (east, north) metres, headings counter-clockwise
from east in radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .follower import Drone, HeadingMode, LedPose, path_lengths
from .frames import wrap_angle

# Stop-and-go's defaults: the square test's slide speed, a slow in-place turn (90° in 3 s), and
# the hold's arrival: within 15 cm and 10° of the leg's heading, steady for 1 s, or 15 s at most
DEFAULT_STOP_SPEED_MPS = 0.3
DEFAULT_STOP_TURN_RATE_DPS = 30.0
DEFAULT_STOP_ARRIVE_M = 0.15
DEFAULT_STOP_HEADING_DEG = 10.0
DEFAULT_STOP_SETTLE_S = 1.0
DEFAULT_STOP_HOLD_TIMEOUT_S = 15.0

# The sliding pose waits while the drone is this far behind it, as the square test's lead: PX4
# never sees a far setpoint
LEAD_M = 0.3


@dataclass(frozen=True, slots=True, kw_only=True)
class StopPlan:
    """How stop-and-go flies its legs and holds."""

    speed_mps: float = DEFAULT_STOP_SPEED_MPS               # along each straight leg
    turn_rate_dps: float = DEFAULT_STOP_TURN_RATE_DPS       # the in-place yaw at a hold
    arrive_m: float = DEFAULT_STOP_ARRIVE_M                 # horizontal, from the waypoint
    heading_deg: float = DEFAULT_STOP_HEADING_DEG           # the drone's heading from the leg's
    settle_s: float = DEFAULT_STOP_SETTLE_S                 # arrived and facing this long
    hold_timeout_s: float = DEFAULT_STOP_HOLD_TIMEOUT_S     # then goes on anyway


class Stage(str, Enum):
    """What stop-and-go is doing."""

    HOLD = "hold"             # over a waypoint: yawing to the next leg, then settling
    MOVE = "move"             # sliding along a leg, heading held


class StopAndGo:
    """The route from `start` flown leg by leg with a hold at every waypoint, stepped once per tick;
    the same face as the smooth `Follower`, so the mission flies either."""

    def __init__(self, start: tuple[float, float], route: list[tuple[float, float]], yaw_rad: float,
                 plan: StopPlan, heading_mode: HeadingMode = HeadingMode.AHEAD,
                 events: list[str] | None = None) -> None:
        """Starts holding over `start` facing `yaw_rad`; timed-out holds are noted in `events`."""
        if plan.speed_mps <= 0.0 or plan.turn_rate_dps <= 0.0:
            raise ValueError(f"stop-and-go speed and turn rate must be positive, not {plan.speed_mps} m/s and "
                             f"{plan.turn_rate_dps} deg/s")

        self.points = np.array([start, *route], dtype=float)
        self.lengths = path_lengths(self.points)
        self.plan = plan
        self.heading_mode = heading_mode
        self.events = [] if events is None else events
        self.yaw_rad = yaw_rad
        self.index = 0
        self.stage = Stage.HOLD
        self.setpoint = self.points[0].copy()
        self.along_m = 0.0
        self.held_s = 0.0
        self.settled_s = 0.0
        self.finished = False

    @property
    def end(self) -> tuple[float, float]:
        """The route's last waypoint."""
        return float(self.points[-1, 0]), float(self.points[-1, 1])

    def reference(self) -> tuple[float, float]:
        """The pose's position now, (east, north)."""
        return float(self.setpoint[0]), float(self.setpoint[1])

    def step(self, elapsed_s: float, drone: Drone | None) -> LedPose:
        """The pose to command now, after `elapsed_s` more of the hold or the leg."""
        if not self.finished and self.stage is Stage.HOLD:
            self._hold(elapsed_s, drone)
        elif not self.finished:
            self._move(elapsed_s, drone)

        return LedPose(float(self.setpoint[0]), float(self.setpoint[1]), self.yaw_rad)

    def _leg_heading(self) -> float:
        """The heading the hold turns to: along the next leg, or as it is at the last waypoint or
        with the heading fixed."""
        if self.heading_mode is HeadingMode.FIXED or self.index >= len(self.points) - 1:
            return self.yaw_rad

        leg = self.points[self.index + 1] - self.points[self.index]
        return math.atan2(leg[1], leg[0]) if float(np.hypot(*leg)) > 1e-9 else self.yaw_rad

    def _hold(self, elapsed_s: float, drone: Drone | None) -> None:
        """Turns toward the next leg at the turn rate, then counts the time the drone is over the
        waypoint and facing the leg; settled long enough, or out of time, the leg begins."""
        self.held_s += elapsed_s
        target_rad = self._leg_heading()
        largest_rad = math.radians(self.plan.turn_rate_dps) * elapsed_s
        self.yaw_rad = wrap_angle(self.yaw_rad + min(max(wrap_angle(target_rad - self.yaw_rad), -largest_rad),
                                                     largest_rad))

        # Settled: the commanded turn done, the drone over the waypoint and facing the leg
        turned = abs(wrap_angle(target_rad - self.yaw_rad)) < 1e-9
        steady = (turned and drone is not None
                  and math.dist((drone.east, drone.north), self.points[self.index]) <= self.plan.arrive_m
                  and abs(wrap_angle(drone.yaw_rad - target_rad)) <= math.radians(self.plan.heading_deg))
        self.settled_s = self.settled_s + elapsed_s if steady else 0.0
        if self.settled_s < self.plan.settle_s and self.held_s < self.plan.hold_timeout_s:
            return

        if self.settled_s < self.plan.settle_s:
            self.events.append(f"hold at waypoint {self.index} of {len(self.points) - 1} not settled after "
                               f"{self.plan.hold_timeout_s:.0f} s: going on")
            self.yaw_rad = target_rad

        if self.index >= len(self.points) - 1:
            self.finished = True
        else:
            self.stage, self.along_m = Stage.MOVE, 0.0

    def _move(self, elapsed_s: float, drone: Drone | None) -> None:
        """Slides the pose along the leg at the plan's speed while the drone keeps within `LEAD_M`
        of it; at the leg's end, the hold over its waypoint."""
        start, end = self.points[self.index], self.points[self.index + 1]
        length_m = float(np.hypot(*(end - start)))
        if drone is not None and math.dist((drone.east, drone.north), self.setpoint) < LEAD_M:
            self.along_m = min(length_m, self.along_m + self.plan.speed_mps * elapsed_s)

        self.setpoint = start + (end - start) * (self.along_m / length_m if length_m > 1e-9 else 1.0)
        if self.along_m >= length_m:
            self.index += 1
            self.stage, self.held_s, self.settled_s = Stage.HOLD, 0.0, 0.0
