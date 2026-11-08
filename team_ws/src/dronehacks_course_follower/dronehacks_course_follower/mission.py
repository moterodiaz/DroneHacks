"""The run from the launch mark to a landing on the pad, as a state machine with no ROS in it.

The safety pilot arms in Altitude and switches to Position; the flight interface then requests
Offboard once, and from there this mission flies: it climbs to the course map's flight height,
follows the planned route, and descends onto the route's end with a downward twist fast enough
for PX4's land detector, so PX4 lands and disarms itself. With the return (rule 7.10) it instead
rests on the pad while PX4 reports landed, climbs straight up above the walls (rule 7.7), slides
straight to over the launch mark and lands there. Commands are in the course frame: (east, north)
metres, height above the floor, headings counter-clockwise from east.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import NamedTuple

import numpy as np

from .follower import PATH_STEP_M, TURN_RATE_DPS, Drone, EasedSpeed, Follower, HeadingMode, eased_speed
from .stop_and_go import StopAndGo, StopPlan

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

# The drone is on the floor once the tracker's height has held within CLIMB_STEADY_M for this long,
# no higher than TOUCHDOWN_NEAR_FLOOR_M above where it stood at Offboard
TOUCHDOWN_DWELL_S = 0.3
TOUCHDOWN_NEAR_FLOOR_M = 0.3

# On the floor the descent presses at this speed until PX4 lands or disarms. PX4 counts ground contact
# only below its low-throttle threshold; its vertical velocity loop (MPC_Z_VEL_P_ACC 4) gets there at
# once from a 1 m/s error, where the descent's smaller error waits seconds on the integrator to wind
TOUCHDOWN_PRESS_MPS = 1.0

# While descending, the twist steers back over the landing point at this gain, no faster than the
# cap: a twist alone has no position hold
LANDING_GAIN_PER_S = 1.0
LANDING_CORRECTION_MPS = 0.3

# The return's defaults: above the 1.727 m walls with room for the frame and the height estimate's
# error, a rest on the pad well inside COM_DISARM_LAND's 5 s, and an unhurried slide home
DEFAULT_RETURN_HEIGHT_M = 2.3
DEFAULT_RETURN_REST_S = 2.0
DEFAULT_RETURN_SPEED_MPS = 0.5

# The return's rest on the pad is refused longer than this: PX4 disarms COM_DISARM_LAND (5 s on
# skybeta-11) after it reports landed, and the motors need about a second to lift the drone
RETURN_REST_MAX_S = 3.0

# During the rest the twist presses down gently, so PX4 stays landed without dragging the drone
RETURN_PRESS_MPS = 0.15

# The return's climb is done within this much of its height, or once it has stopped rising above
# this share of it: PX4 holds height by its rangefinder, a few percent off the tracker's z
RETURN_CLIMBED_M = 0.15
RETURN_MIN_FRACTION = 0.9

# The slide to the launch mark waits while the drone trails its setpoint by more than this, and
# brakes into the mark at this deceleration, which reaches it in finite time unlike PX4's jerk-limited curve
RETURN_MAX_LAG_M = 1.0
RETURN_BRAKE_M_S2 = 1.0

# With no odometry the return holds its pose this long, then descends straight down where it is
RETURN_STALE_LAND_S = 3.0


class Phase(str, Enum):
    """Where the mission is."""

    WAIT = "wait"                         # on the ground, waiting for Offboard
    CLIMB = "climb"                       # up to the flight height over the start
    FLY = "fly"                           # along the route
    DESCEND = "descend"                   # twisting down onto the pad until PX4 disarms or, returning, lands
    RETURN_REST = "return_rest"           # landed on the pad, pressing down for the rest
    RETURN_CLIMB = "return_climb"         # straight up over the pad to the return height
    RETURN_FLY = "return_fly"             # sliding straight to over the launch mark
    RETURN_DESCEND = "return_descend"     # twisting down onto the launch mark until PX4 disarms
    DONE = "done"                         # landed and disarmed, on the pad or back on the launch mark
    ABORTED = "aborted"                   # the pilot has it; nothing is sent until a disarm


# Phases that end in DONE when PX4 disarms: the drone is on the ground or coming down onto it
_LANDING_PHASES = (Phase.DESCEND, Phase.RETURN_REST, Phase.RETURN_DESCEND)

# Phases flown under Offboard: Offboard ending in any of them is the pilot taking over
_OFFBOARD_PHASES = (Phase.CLIMB, Phase.FLY, Phase.DESCEND, Phase.RETURN_REST, Phase.RETURN_CLIMB,
                    Phase.RETURN_FLY, Phase.RETURN_DESCEND)

# Phases from the pad descent on: PX4's offset stays frozen, so the rangefinder's floor error on
# the pad and its steps over the wall tops never reach a setpoint
_FROZEN_OFFSET_PHASES = (Phase.DESCEND, Phase.RETURN_REST, Phase.RETURN_CLIMB, Phase.RETURN_FLY,
                         Phase.RETURN_DESCEND, Phase.DONE)


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


@dataclass(frozen=True, slots=True, kw_only=True)
class ReturnPlan:
    """The return to the launch mark after the pad landing (rule 7.10)."""

    height_m: float           # flown above the course floor, over the walls (rule 7.7)
    rest_s: float             # on the pad after PX4 reports landed, before the climb
    speed_mps: float          # the straight slide to over the launch mark


class Mission:
    """Flies `route` among `walls` at `height_m`, from the launch mark `launch` to a landing."""

    def __init__(self, route: list[tuple[float, float]], walls: np.ndarray, height_m: float,
                 launch: tuple[float, float], top_speed_mps: float, lead: bool = True, land: bool = True,
                 heading_mode: HeadingMode = HeadingMode.AHEAD,
                 turn_rate_rad_s: float = math.radians(TURN_RATE_DPS),
                 return_plan: ReturnPlan | None = None, stop_plan: StopPlan | None = None) -> None:
        """Keeps the plan; the follower is built at the start of the route, once the drone is up.
        Without `land`, the mission holds over the route's end for the pilot to land. With
        `heading_mode` fixed, the heading at Offboard is commanded from the climb to the landing.
        With `return_plan`, the pad landing is followed by the return to the launch mark. With
        `stop_plan`, the route is flown stop-and-go: straight legs with a hold at every waypoint."""
        if turn_rate_rad_s <= 0.0:
            raise ValueError(f"turn rate must be positive, not {turn_rate_rad_s} rad/s")

        if return_plan is not None:
            _check_return(return_plan)

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
        self.stop_plan = stop_plan
        self.phase = Phase.WAIT
        self.follower: Follower | StopAndGo | None = None
        self.start: tuple[float, float] | None = None
        self.finished_at_s: float | None = None
        self.previous_s: float | None = None
        self.events: list[str] = []
        self.flown = False
        self.offboard_at_s = math.nan
        self.floor_height_m = math.nan
        self.steady_height_m = math.nan
        self.steady_since_s = math.nan
        self.on_floor = False
        self.hovering = False

        # The return: its plan, and where it stands
        self.return_plan = return_plan
        self.landed_on_pad = False
        self.rest_since_s = math.nan
        self.return_yaw_rad = math.nan
        self.return_along_m = 0.0
        self.return_motion = EasedSpeed(0.0, 0.0)
        self.return_finished_at_s: float | None = None
        self.straight_down = False
        self.stale_since_s: float | None = None
        self.last_drone: Drone | None = None

    @property
    def pad(self) -> tuple[float, float]:
        """Where the route ends: the pad landing's aim."""
        return self.follower.end if self.follower else self.route[-1]

    @property
    def offset_frozen(self) -> bool:
        """Whether PX4's offset must stay as it is: from the pad descent on."""
        return self.phase in _FROZEN_OFFSET_PHASES

    def tick(self, now_s: float, offboard: bool, armed: bool, drone: Drone | None,
             hold_climb: bool = False, landed: bool = False) -> PoseCommand | TwistCommand | None:
        """The command to send now, or None to send nothing; `drone` is None while its estimate is stale.
        With `hold_climb`, a drone at height keeps hovering over the start instead of starting the route;
        `landed` is PX4's land detector, which starts the return's rest on the pad."""
        elapsed_s = 0.0 if self.previous_s is None else now_s - self.previous_s
        self.previous_s = now_s
        self.hovering = False
        if drone is not None:
            self.last_drone = drone

        # PX4's own disarm ends a landing; the pilot taking the drone back ends anything else,
        # and a fresh attempt starts only after a disarm
        if self.phase in _LANDING_PHASES and not armed:
            self._enter(Phase.DONE, self._disarmed_event(now_s))
        if self.phase in (Phase.RETURN_CLIMB, Phase.RETURN_FLY) and not armed:
            self._enter(Phase.ABORTED, "disarmed mid-return")
        if self.phase in _OFFBOARD_PHASES and not offboard:
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
                return self._descend(now_s, drone, landed)
            case Phase.RETURN_REST:
                return self._return_rest(now_s, drone)
            case Phase.RETURN_CLIMB:
                return self._return_climb(now_s, drone)
            case Phase.RETURN_FLY:
                return self._return_fly(now_s, elapsed_s, drone)
            case Phase.RETURN_DESCEND:
                return self._land(now_s, self.launch, drone)
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
            self.floor_height_m = drone.height_m
            self._enter(Phase.CLIMB, f"Offboard: climbing to {self.height_m:.2f} m")

        return PoseCommand(drone.east, drone.north, self.height_m, math.nan)

    def _climb(self, now_s: float, drone: Drone | None, hold_climb: bool) -> PoseCommand:
        """The pose over the start at the flight height, its heading not commanded, or Offboard's with
        a fixed heading; once there, or settled short of it, the route begins unless `hold_climb`."""
        east, north = self.start
        self.hovering = drone is not None and (abs(drone.height_m - self.height_m) < CLIMBED_M
                                               or self._settled(now_s, drone, CLIMB_MIN_FRACTION * self.height_m))
        if self.hovering and not hold_climb:
            yaw_rad = drone.yaw_rad if math.isnan(self.held_yaw_rad) else self.held_yaw_rad
            if self.stop_plan is not None:
                self.follower = StopAndGo(self.start, self.route, yaw_rad, self.stop_plan, self.heading_mode,
                                          events=self.events)
            else:
                self.follower = Follower(self.start, self.route, self.walls, yaw_rad, self.top_speed_mps,
                                         turn_rate_rad_s=self.turn_rate_rad_s, lead=self.lead,
                                         heading_mode=self.heading_mode)
            self._enter(Phase.FLY, f"at height: flying {float(self.follower.lengths[-1]):.1f} m of route")

        return PoseCommand(east, north, self.height_m, self.held_yaw_rad)

    def _settled(self, now_s: float, drone: Drone, min_height_m: float) -> bool:
        """Whether a climb has stopped rising, at least `min_height_m` up."""
        return drone.height_m >= min_height_m and self._steady_for_s(now_s, drone) >= CLIMB_SETTLED_S

    def _steady_for_s(self, now_s: float, drone: Drone) -> float:
        """How long the tracker's height has held within `CLIMB_STEADY_M`, counted from this phase's start."""
        if not abs(drone.height_m - self.steady_height_m) <= CLIMB_STEADY_M:
            self.steady_height_m, self.steady_since_s = drone.height_m, now_s

        return now_s - self.steady_since_s

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

    def _descend(self, now_s: float, drone: Drone | None, landed: bool) -> TwistCommand:
        """The twist down onto the pad; returning, once PX4 reports landed, the rest begins."""
        if self.return_plan is None or not landed:
            return self._land(now_s, self.pad, drone)

        self.landed_on_pad = True
        self.rest_since_s = now_s
        self.return_yaw_rad = drone.yaw_rad if drone is not None else math.nan
        self._enter(Phase.RETURN_REST, f"landed on the pad {now_s - self.offboard_at_s:.1f} s after Offboard: "
                                       f"resting {self.return_plan.rest_s:.1f} s")
        return TwistCommand(0.0, 0.0, -RETURN_PRESS_MPS, 0.0)

    def _return_rest(self, now_s: float, drone: Drone | None) -> PoseCommand | TwistCommand:
        """A gentle press down on the pad for the rest; after it, the climb over the walls."""
        if now_s - self.rest_since_s < self.return_plan.rest_s:
            return TwistCommand(0.0, 0.0, -RETURN_PRESS_MPS, 0.0)

        self._enter(Phase.RETURN_CLIMB, f"returning: climbing to {self.return_plan.height_m:.2f} m over the pad")
        return self._return_climb(now_s, drone)

    def _return_climb(self, now_s: float, drone: Drone | None) -> PoseCommand | TwistCommand:
        """The pose straight over the pad at the return height, heading held; once there, or settled
        just short of it, the slide begins."""
        east, north = self.pad
        command = PoseCommand(east, north, self.return_plan.height_m, self.return_yaw_rad)
        if self._stale_landing(now_s, drone):
            return landing_twist(self.launch, None)

        height_m = self.return_plan.height_m
        if drone is not None and (abs(drone.height_m - height_m) < RETURN_CLIMBED_M
                                  or self._settled(now_s, drone, RETURN_MIN_FRACTION * height_m)):
            self._enter(Phase.RETURN_FLY, f"over the walls at {drone.height_m:.2f} m: sliding "
                                          f"{math.dist(self.pad, self.launch):.1f} m to the launch mark")

        return command

    def _return_fly(self, now_s: float, elapsed_s: float, drone: Drone | None) -> PoseCommand | TwistCommand:
        """The pose sliding straight from over the pad to over the launch mark, eased within PX4's
        limits and waiting while the drone trails; over the mark, the descent onto it."""
        if self._stale_landing(now_s, drone):
            return landing_twist(self.launch, None)

        # Where the slide stands, and the pace the drone allows
        start, end = np.asarray(self.pad), np.asarray(self.launch)
        length_m = float(np.hypot(*(end - start)))
        share = 1.0 if length_m < 1e-9 else min(1.0, self.return_along_m / length_m)
        setpoint = start + share * (end - start)
        remaining_m = length_m - self.return_along_m
        keeping_up = drone is not None and math.dist((drone.east, drone.north), setpoint) <= RETURN_MAX_LAG_M
        braking_mps = math.sqrt(2.0 * RETURN_BRAKE_M_S2 * max(remaining_m, 0.0))
        target_mps = min(self.return_plan.speed_mps, braking_mps) if keeping_up else 0.0
        self.return_motion = eased_speed(self.return_motion.speed_mps, self.return_motion.accel_m_s2, target_mps,
                                         elapsed_s)
        self.return_along_m = min(length_m, self.return_along_m + self.return_motion.speed_mps * elapsed_s)

        # The end of the slide: its braking creeps the last centimetres, so it ends a path step short
        if remaining_m > PATH_STEP_M:
            return PoseCommand(float(setpoint[0]), float(setpoint[1]), self.return_plan.height_m, self.return_yaw_rad)

        if self.return_finished_at_s is None:
            self.return_finished_at_s = now_s
        aligned = drone is not None and math.dist((drone.east, drone.north), self.launch) < LANDING_ALIGNED_M
        if aligned or now_s - self.return_finished_at_s > LANDING_SETTLE_S:
            self._enter(Phase.RETURN_DESCEND, "over the launch mark: descending")
            return landing_twist(self.launch, drone)

        return PoseCommand(*self.launch, self.return_plan.height_m, self.return_yaw_rad)

    def _stale_landing(self, now_s: float, drone: Drone | None) -> bool:
        """Whether the return has gone `RETURN_STALE_LAND_S` without odometry: it then descends
        straight down where it is, since steering blind could cross a wall."""
        if drone is not None:
            self.stale_since_s = None
            return False

        if self.stale_since_s is None:
            self.stale_since_s = now_s
        if now_s - self.stale_since_s < RETURN_STALE_LAND_S:
            return False

        self.straight_down = True
        self._enter(Phase.RETURN_DESCEND, f"no odometry for {RETURN_STALE_LAND_S:.0f} s: straight down")
        return True

    def _land(self, now_s: float, aim: tuple[float, float], drone: Drone | None) -> TwistCommand:
        """The descent's twist onto `aim`, pressed at `TOUCHDOWN_PRESS_MPS` from the moment the drone
        is on the floor until the phase ends; straight down after the return lost its odometry."""
        if not self.on_floor and self._touched_down(now_s, drone):
            self.on_floor = True
            self.events.append(f"on the floor {now_s - self.offboard_at_s:.1f} s after Offboard: pressing "
                               f"{TOUCHDOWN_PRESS_MPS:.1f} m/s down for PX4's land detector")

        down_mps = TOUCHDOWN_PRESS_MPS if self.on_floor else LANDING_DESCENT_MPS
        return landing_twist(aim, None if self.straight_down else drone, down_mps)

    def _touched_down(self, now_s: float, drone: Drone | None) -> bool:
        """Whether the tracker's height has stopped falling for `TOUCHDOWN_DWELL_S` near the floor."""
        if drone is None:
            return False

        near_floor = drone.height_m - self.floor_height_m <= TOUCHDOWN_NEAR_FLOOR_M
        return self._steady_for_s(now_s, drone) >= TOUCHDOWN_DWELL_S and near_floor

    def _disarmed_event(self, now_s: float) -> str:
        """What a disarm in a landing phase means; a disarm on the pad counts it as landed there."""
        after = f"{now_s - self.offboard_at_s:.1f} s after Offboard"
        if self.phase is Phase.DESCEND:
            self.landed_on_pad = True
            return f"disarmed on the pad {after}"

        if self.phase is Phase.RETURN_REST:
            return f"disarmed on the pad during the return's rest {after}: is COM_DISARM_LAND under 5 s?"

        if self.last_drone is None:
            return f"disarmed after the return {after}"

        miss_m = math.dist((self.last_drone.east, self.last_drone.north), self.launch)
        return f"disarmed {miss_m:.2f} m from the launch mark {after}"

    def _enter(self, phase: Phase, event: str) -> None:
        """Moves to `phase`, noting why, its height steadiness and floor press counted afresh; a new
        attempt starts with no follower and no return."""
        self.phase = phase
        self.events.append(event)
        self.steady_height_m, self.on_floor = math.nan, False
        if phase is Phase.WAIT:
            self.follower, self.start, self.finished_at_s = None, None, None
            self.return_along_m, self.return_motion, self.return_finished_at_s = 0.0, EasedSpeed(0.0, 0.0), None
            self.straight_down, self.stale_since_s = False, None


def _check_return(plan: ReturnPlan) -> None:
    """Raises if the return cannot be flown: no height or speed, or a rest PX4 would disarm in."""
    if plan.height_m <= 0.0 or plan.speed_mps <= 0.0:
        raise ValueError(f"return height and speed must be positive, not {plan.height_m} m and {plan.speed_mps} m/s")

    if not 0.0 <= plan.rest_s <= RETURN_REST_MAX_S:
        raise ValueError(f"return rest must be 0 to {RETURN_REST_MAX_S} s, not {plan.rest_s}: PX4 disarms "
                         "COM_DISARM_LAND (5 s) after landing and the motors need about 1 s to lift")


def landing_twist(aim: tuple[float, float], drone: Drone | None,
                  down_mps: float = LANDING_DESCENT_MPS) -> TwistCommand:
    """The descent: `down_mps` down, steered back over `aim` at the capped correction, heading held;
    straight down while the drone's estimate is stale."""
    correction = np.zeros(2)
    if drone is not None:
        correction = LANDING_GAIN_PER_S * (np.asarray(aim) - (drone.east, drone.north))
        size = float(np.hypot(*correction))
        if size > LANDING_CORRECTION_MPS:
            correction *= LANDING_CORRECTION_MPS / size

    return TwistCommand(float(correction[0]), float(correction[1]), -down_mps, 0.0)
