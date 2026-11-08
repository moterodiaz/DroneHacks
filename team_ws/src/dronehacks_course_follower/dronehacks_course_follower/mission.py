"""The run from the launch mark to a landing on the pad, as a state machine with no ROS in it.

The safety pilot arms in Altitude and switches to Position; the flight interface then requests
Offboard once, and from there this mission flies: it climbs to the course map's flight height,
follows the planned route, and descends onto the route's end with a downward twist fast enough
for PX4's land detector, so PX4 lands and disarms itself. Commands are in the course frame:
(east, north) metres, height above the floor, headings counter-clockwise from east.
"""
from __future__ import annotations

import math
from enum import Enum
from typing import NamedTuple

import numpy as np

from .follower import TURN_RATE_DPS, Drone, Follower, HeadingMode

# The climb is done within this much of the flight height
CLIMBED_M = 0.1

# Or once it has stopped rising: PX4 holds the height by its own estimate (the rangefinder's), which
# can sit well off the tracker's z, so a climb can settle short of the tracker's flight height. It
# counts once the tracker's height has held within CLIMB_STEADY_M for CLIMB_SETTLED_S, above
# CLIMB_MIN_FRACTION of the flight height so a drone still spooling up on the floor does not
CLIMB_STEADY_M = 0.03
CLIMB_SETTLED_S = 1.5
CLIMB_MIN_FRACTION = 0.5

# The route is only flown from within this distance of the map's launch mark: from anywhere else
# its first leg could cross a wall
START_RADIUS_M = 0.5

# The descent starts once the drone is this near the route's end, or this long after the setpoint got there
LANDING_ALIGNED_M = 0.15
LANDING_SETTLE_S = 2.0

# The descent's speed. PX4 counts ground contact only while the setpoint descends at 1.1 x
# LNDMC_Z_VEL_MAX or more, 0.275 m/s at the organisers' 0.25: a pose, or a slower twist, never lands
LANDING_DESCENT_MPS = 0.5

# While descending, the twist steers back over the landing point at this gain, no faster than the
# cap: a twist alone has no position hold
LANDING_GAIN_PER_S = 1.0
LANDING_CORRECTION_MPS = 0.3


class Phase(str, Enum):
    """Where the mission is."""

    WAIT = "wait"             # on the ground, waiting for Offboard
    CLIMB = "climb"           # up to the flight height over the start
    FLY = "fly"               # along the route
    DESCEND = "descend"       # twisting down onto the pad until PX4 disarms
    DONE = "done"             # landed and disarmed on the pad
    ABORTED = "aborted"       # the pilot has it; nothing is sent until a disarm


class PoseCommand(NamedTuple):
    """A pose setpoint in the course frame."""

    east: float               # metres
    north: float              # metres
    height_m: float           # above the floor
    yaw_rad: float            # NaN holds the heading


class TwistCommand(NamedTuple):
    """A velocity setpoint in the course frame."""

    east_mps: float
    north_mps: float
    up_mps: float
    yaw_rate_rad_s: float


class Mission:
    """Flies `route` among `walls` at `height_m`, from the launch mark `launch` to a landing."""

    def __init__(self, route: list[tuple[float, float]], walls: np.ndarray, height_m: float,
                 launch: tuple[float, float], top_speed_mps: float, lead: bool = True, land: bool = True,
                 heading_mode: HeadingMode = HeadingMode.AHEAD,
                 turn_rate_rad_s: float = math.radians(TURN_RATE_DPS)) -> None:
        """Keeps the plan; the follower is built at the start of the route, once the drone is up.
        Without `land`, the mission holds over the route's end for the pilot to land. With
        `heading_mode` fixed, the heading at Offboard is commanded from the climb to the landing."""
        if turn_rate_rad_s <= 0.0:
            raise ValueError(f"turn rate must be positive, not {turn_rate_rad_s} rad/s")

        self.route = route
        self.walls = walls
        self.height_m = height_m
        self.launch = launch
        self.top_speed_mps = top_speed_mps
        self.lead = lead
        self.land = land
        self.heading_mode = heading_mode
        self.turn_rate_rad_s = turn_rate_rad_s
        self.held_yaw_rad = math.nan
        self.phase = Phase.WAIT
        self.follower: Follower | None = None
        self.start: tuple[float, float] | None = None
        self.finished_at_s: float | None = None
        self.previous_s: float | None = None
        self.events: list[str] = []
        self.flown = False
        self.offboard_at_s = math.nan
        self.steady_height_m = math.nan
        self.steady_since_s = math.nan
        self.hovering = False

    def tick(self, now_s: float, offboard: bool, armed: bool, drone: Drone | None,
             hold_climb: bool = False) -> PoseCommand | TwistCommand | None:
        """The command to send now, or None to send nothing; `drone` is None while its estimate is stale.
        With `hold_climb`, a drone at height keeps hovering over the start instead of starting the route."""
        elapsed_s = 0.0 if self.previous_s is None else now_s - self.previous_s
        self.previous_s = now_s
        self.hovering = False

        # PX4's own disarm ends the descent; the pilot taking the drone back ends anything else,
        # and a fresh attempt starts only after a disarm
        if self.phase is Phase.DESCEND and not armed:
            self._enter(Phase.DONE, f"disarmed on the pad {now_s - self.offboard_at_s:.1f} s after Offboard")
        if self.phase in (Phase.CLIMB, Phase.FLY, Phase.DESCEND) and not offboard:
            self._enter(Phase.ABORTED, "Offboard ended mid-run: the pilot has the drone")
        if self.phase is Phase.ABORTED and not armed:
            self._enter(Phase.WAIT, "disarmed: waiting for the next Offboard")

        match self.phase:
            case Phase.WAIT:
                return self._wait(now_s, offboard, drone)
            case Phase.CLIMB:
                return self._climb(now_s, drone, hold_climb)
            case Phase.FLY:
                return self._fly(now_s, elapsed_s, drone)
            case Phase.DESCEND:
                return landing_twist(self.follower.end if self.follower else self.route[-1], drone)
            case _:
                return None

    def flight_over(self, armed: bool) -> bool:
        """Whether a climb has started since start-up and PX4 is now disarmed: the run's recording
        can close. Arming on the ground, or a route refused away from the mark, is no flight."""
        return self.flown and not armed

    def _wait(self, now_s: float, offboard: bool, drone: Drone | None) -> PoseCommand | None:
        """A pose over the drone at the flight height, the stream the interface needs before it
        requests Offboard; on Offboard, the climb, if the drone stands on the launch mark."""
        if drone is None:
            return None

        if offboard:
            if math.dist((drone.east, drone.north), self.launch) > START_RADIUS_M:
                self._enter(Phase.ABORTED, f"Offboard at ({drone.east:.2f}, {drone.north:.2f}), over "
                                           f"{START_RADIUS_M} m from the launch mark: refusing the route")
                return None

            self.start = (drone.east, drone.north)
            self.held_yaw_rad = drone.yaw_rad if self.heading_mode is HeadingMode.FIXED else math.nan
            self.flown = True
            self.offboard_at_s = now_s
            self._enter(Phase.CLIMB, f"Offboard: climbing to {self.height_m:.2f} m")

        return PoseCommand(drone.east, drone.north, self.height_m, math.nan)

    def _climb(self, now_s: float, drone: Drone | None, hold_climb: bool) -> PoseCommand:
        """The pose over the start at the flight height, its heading not commanded, or Offboard's with
        a fixed heading; once there, or settled short of it, the route begins unless `hold_climb`."""
        east, north = self.start
        self.hovering = drone is not None and (abs(drone.height_m - self.height_m) < CLIMBED_M
                                               or self._settled(now_s, drone))
        if self.hovering and not hold_climb:
            yaw_rad = drone.yaw_rad if math.isnan(self.held_yaw_rad) else self.held_yaw_rad
            self.follower = Follower(self.start, self.route, self.walls, yaw_rad, self.top_speed_mps,
                                     turn_rate_rad_s=self.turn_rate_rad_s, lead=self.lead,
                                     heading_mode=self.heading_mode)
            self._enter(Phase.FLY, f"at height: flying {float(self.follower.lengths[-1]):.1f} m of route")

        return PoseCommand(east, north, self.height_m, self.held_yaw_rad)

    def _settled(self, now_s: float, drone: Drone) -> bool:
        """Whether the climb has stopped rising, high enough to be off the floor."""
        if not abs(drone.height_m - self.steady_height_m) <= CLIMB_STEADY_M:
            self.steady_height_m, self.steady_since_s = drone.height_m, now_s

        return (drone.height_m >= CLIMB_MIN_FRACTION * self.height_m
                and now_s - self.steady_since_s >= CLIMB_SETTLED_S)

    def _fly(self, now_s: float, elapsed_s: float, drone: Drone | None) -> PoseCommand | TwistCommand:
        """The follower's led pose; at the route's end, the descent once the drone is over it."""
        pose = self.follower.step(elapsed_s, drone)
        if not self.follower.finished:
            return PoseCommand(pose.east, pose.north, self.height_m, pose.yaw_rad)

        if self.finished_at_s is None:
            self.finished_at_s = now_s

        aligned = drone is not None and math.dist((drone.east, drone.north), self.follower.end) < LANDING_ALIGNED_M
        if self.land and (aligned or now_s - self.finished_at_s > LANDING_SETTLE_S):
            self._enter(Phase.DESCEND, "over the pad: descending")
            return landing_twist(self.follower.end, drone)

        return PoseCommand(pose.east, pose.north, self.height_m, pose.yaw_rad)

    def _enter(self, phase: Phase, event: str) -> None:
        """Moves to `phase`, noting why; a new attempt starts with no follower."""
        self.phase = phase
        self.events.append(event)
        if phase is Phase.WAIT:
            self.follower, self.start, self.finished_at_s = None, None, None


def landing_twist(aim: tuple[float, float], drone: Drone | None) -> TwistCommand:
    """The descent: `LANDING_DESCENT_MPS` down, steered back over `aim` at the capped correction,
    heading held; straight down while the drone's estimate is stale."""
    correction = np.zeros(2)
    if drone is not None:
        correction = LANDING_GAIN_PER_S * (np.asarray(aim) - (drone.east, drone.north))
        size = float(np.hypot(*correction))
        if size > LANDING_CORRECTION_MPS:
            correction *= LANDING_CORRECTION_MPS / size

    return TwistCommand(float(correction[0]), float(correction[1]), -LANDING_DESCENT_MPS, 0.0)
