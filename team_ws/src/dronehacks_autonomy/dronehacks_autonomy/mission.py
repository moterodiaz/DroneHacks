"""The flight, arming to disarm: wait, take off, follow, hold over the pad, land, optionally return.

**FOLLOW flies the Follower** (``planning.follower``): the smoothed trajectory with its speed
profile and progress governor. The pose is cuVSLAM's odometry in the ``odom`` frame, and setpoints
go straight to the kit's flight interface.

**It publishes the takeoff pose while waiting, but only while the prechecks pass.** The kit grants
Offboard only if a team setpoint is under 1 s old when the pilot switches to Position
(``auto_offboard.py``). The prechecks:

* a fresh pose;
* the drone still where the measured frame says the route starts (the launch mark);
* its heading still the launch heading;
* optionally, PX4's heading agreeing with vision's.

While they fail before any takeoff pose was streamed it publishes nothing, so Offboard is refused
and the pilot keeps the vehicle. After a takeoff pose was streamed it replaces it with a
zero-velocity twist: the kit's pose setpoint does not expire, and Offboard is still granted up to
1 s after the last team setpoint, so merely stopping would leave the drone able to climb on the
stored pose.

Rules from competitor_guide.md §3-§4:
* Nothing before Offboard except the takeoff pose. If Offboard ends, for any
  reason, stop for good: never re-request it, never arm, disarm or change mode.
* Landing uses a velocity setpoint, and only with ``land: true`` (off by
  default: the mission then holds over the pad for the pilot, px4.md). The
  kit floors poses at ``min_altitude_m``, but only caps twists. PX4 disarms
  itself ``COM_DISARM_LAND`` after landing.

**The return leg** (``return_leg: true``, needs ``land: true``). The organisers said the drone does
not disarm on the pad, and the kit could not re-arm it anyway. So the pad is a touch-and-go: on PX4's
land-detected flag the mission commands a climb at once, inside ``COM_DISARM_LAND``. Only a flag
that follows a "not landed" since takeoff counts, so a stale reading from the launch cannot start it. It climbs straight
up to ``return_height_m``, flies a straight line to the launch mark at that height with a reference
that waits for the drone, holds, descends, and PX4 disarms there. The line needs no route planning:
the net is convex and the height clears what is below, which ``return_height_m`` has to guarantee.
If PX4 disarms on the pad anyway (the window was missed), the mission stops there.

Added for safety:
* **GROUNDED.** Offboard starts but the pose is stale or a precheck now
  fails. The interface already holds the takeoff pose, so stopping alone
  would still take off. Instead, zero-velocity twists keep the drone on the
  ground until PX4's ``COM_DISARM_PRFLT`` disarms it or the pilot takes over.
* **HELD.** Cross-track error over ``abort_cross_track_m``: hold where it is
  and wait for the pilot.
* **A stale pose in the air** sends nothing. The interface then holds the
  last pose: a led pose the Follower has checked is reachable in a straight
  line.

**The coast through a dropout** (``coast_max_s`` > 0; 0 and off by default:
contracts/ros-graph.md blocks tunnel traversal during vision loss until a measured drift bound and
a pilot-approved abort procedure exist. It is for simulation until then.) PX4's only horizontal
position source is cuVSLAM. When it stops (the dark tube), EKF2 dead-reckons on the IMU for
``EKF2_NOAID_TOUT``, then declares the position invalid: Offboard ends and PX4 falls back to Altitude,
which sinks with the throttle low (competitor_guide.md section 4). Stopping in the dark guarantees
that. So in FOLLOW the mission flies on along the trajectory for up to ``coast_max_s``:

* A dead reckoning stands in for the pose: the last odometry, then PX4's own velocity estimate, or
  without a fresh one the velocities commanded since, each reached with a ``coast_response_s`` lag.
* The Follower keeps running on it, and the command is a **velocity**, which needs no position frame:
  the reference's, times the Follower's clock rate, plus ``coast_gain`` times the dead reckoning's
  offset from the reference, capped at ``v_max_mps``.
* The Follower's safety stops still apply. ``no_safe_target`` ends the coast (zero velocity until the
  pose is back); over ``abort_cross_track_m`` it is HELD for the pilot, by zero velocity.
* PX4's velocity is turned into the odom frame by PX4's current heading, plus the vision-PX4 heading
  gap measured at the last good pose: the frozen odometry heading would lag a turn.
* After ``coast_max_s``, or at the trajectory's end, it holds still by zero velocity. Sending nothing
  would let the interface expire the twist into the last pose setpoint, behind the drone.
* **The frame after a coast.** cuVSLAM resumes from the pose where it lost tracking, so its frame jumps
  back by the motion it missed, and PX4 follows it through the bridge's ``reset_counter``. With
  ``stitch_after_coast``, a resumed pose at least ``stitch_fraction`` of the coast (and ``stitch_min_m``)
  from the dead reckoning is taken as that jump: it is added to every later pose and subtracted from
  every later pose setpoint, so the trajectory stays where it is on the course. A jump larger than the
  coast (+ ``stitch_margin_m``) can't be explained: it holds still by velocity for the pilot.

ROS-free; ``mission_node.py`` wraps it.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

from dronehacks_autonomy.planning.clearance import ObstacleField
from dronehacks_autonomy.planning.follower import Follower, FollowerConfig, Setpoint
from dronehacks_autonomy.planning.limits import VehicleLimits
from dronehacks_autonomy.planning.pipeline import FlightPlan

from .frames import OdomFrame, wrap

WAIT = "WAIT"
TAKEOFF = "TAKEOFF"
FOLLOW = "FOLLOW"
HOLD = "HOLD"
LAND = "LAND"
LANDED = "LANDED"
STOPPED = "STOPPED"
GROUNDED = "GROUNDED"
HELD = "HELD"
RETURN_CLIMB = "RETURN_CLIMB"
RETURN_TRANSIT = "RETURN_TRANSIT"


@dataclasses.dataclass(frozen=True)
class MissionConfig:
    #: Body centre above the floor when landed. contracts/frames.md: 0.07 m
    #: (tape-measured in the FlyX lab). Confirm on drone 1. No default.
    rest_height_m: float
    #: Prechecks before the takeoff pose is offered.
    start_tolerance_m: float = 0.15
    heading_tolerance_deg: float = 5.0
    #: PX4's heading vs vision's, before takeoff (field guide: 2 degrees). 0 turns it off.
    px4_heading_gate_deg: float = 2.0
    takeoff_tolerance_m: float = 0.10
    hold_before_land_s: float = 2.0
    land_tolerance_m: float = 0.10
    descent_mps: float = 0.3
    land_xy_gain: float = 1.0          # 1/s, horizontal correction over the pad while descending
    land_xy_max_mps: float = 0.2
    pose_timeout_s: float = 0.5
    #: Most odometry age to predict across with the odometry's velocity.
    max_prediction_s: float = 0.2
    #: Off the path by more than this: hold and wait for the pilot. The 0.93 m passage leaves +-0.18 m.
    abort_cross_track_m: float = 0.35
    #: Land on the pad after the hold. False by default: the drone holds over the pad
    #: for the pilot, because contracts/px4.md blocks the twist descent until a landing
    #: API is contracted. Set true only per that contract.
    land: bool = False
    #: After touching the pad, take off again without disarming and land at the launch mark.
    return_leg: bool = False
    #: Height above the floor to fly the return at. It must clear everything under the straight
    #: line from pad to launch. The map is sliced at flight height and has no wall heights, so
    #: this is set per course. Required with return_leg.
    return_height_m: float = 0.0
    return_speed_mps: float = 0.5
    #: The return's reference waits while the drone is further behind it than this.
    return_lag_m: float = 0.5
    #: FOLLOW: fly on through a dropout this long, by velocity along the trajectory. 0 (the default)
    #: turns it off: contracts/ros-graph.md blocks tunnel traversal during vision loss.
    coast_max_s: float = 0.0
    #: FOLLOW, while coasting is on: no odometry arriving for this long counts as a dropout.
    pose_gap_s: float = 0.15
    #: PX4's velocity estimate older than this isn't used by the dead reckoning.
    px4_velocity_timeout_s: float = 0.2
    #: The dead reckoning's lag to a commanded velocity, without PX4's estimate:
    #: 1/MPC_XY_VEL_P_ACC (1.8 on drone 1).
    coast_response_s: float = 0.55
    #: 1/s: how hard the coast pulls the dead reckoning back onto the reference.
    coast_gain: float = 1.0
    #: Re-anchor the odom frame on the dead reckoning when the pose is back after a coast.
    stitch_after_coast: bool = True
    #: A resumed pose closer than this to the dead reckoning is kept as it is,
    stitch_min_m: float = 0.10
    #: and so is one closer than this share of the distance coasted (a frozen frame resumes
    #: about the whole coast behind; the dead reckoning's own error is a few per cent of it).
    stitch_fraction: float = 0.5
    #: A jump larger than the coast plus this can't be explained: hold for the pilot.
    stitch_margin_m: float = 0.5

    def __post_init__(self) -> None:
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool):
                continue
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value >= 0.0):
                raise ValueError(f"MissionConfig.{f.name} must be a finite number >= 0 (got {value!r})")


@dataclasses.dataclass(frozen=True)
class OdomPose:
    """One odometry sample in the odom frame: position, yaw, and world-frame horizontal velocity."""

    t: float
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    #: When it arrived, on the mission's clock (None: unknown, only ``t`` counts).
    received: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class Command:
    """A pose (``yaw`` None = not commanded) or a twist setpoint, in the odom frame."""

    kind: str  # "pose" | "twist"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: Optional[float] = None
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class _DeadReckoning:
    """Where the drone is during a dropout (odom frame): its last odometry, then the velocities
    commanded since, each approached with a first-order lag (PX4's velocity loop)."""

    def __init__(self, pose: OdomPose, since: float) -> None:
        self.x, self.y, self.t = pose.x, pose.y, pose.t
        self.vx, self.vy = pose.vx, pose.vy
        self.cmd = (pose.vx, pose.vy)          # until the first command: carrying on as it was
        self.since = since                     # when the coast started (the mission's clock)
        self.travelled = 0.0
        self.halted: Optional[str] = None      # a safety stop ended this coast: hold still until the pose is back

    def step(self, t: float, response_s: float, measured: Optional[Tuple[float, float]] = None) -> None:
        """Advance to *t*: on *measured* (PX4's velocity estimate) when given, else on the
        commanded velocity reached with a ``response_s`` lag."""
        dt = t - self.t
        if dt <= 0.0:
            return
        vx0, vy0 = self.vx, self.vy
        if measured is not None:
            self.vx, self.vy = measured
        else:
            a = 1.0 if response_s <= 0.0 else 1.0 - math.exp(-dt / response_s)
            self.vx += (self.cmd[0] - self.vx) * a
            self.vy += (self.cmd[1] - self.vy) * a
        dx, dy = 0.5 * (vx0 + self.vx) * dt, 0.5 * (vy0 + self.vy) * dt
        self.x += dx
        self.y += dy
        self.travelled += math.hypot(dx, dy)
        self.t = t


class Mission:
    """One run. Call :meth:`update` on a timer and publish what it returns (None = send nothing)."""

    def __init__(self, plan: FlightPlan, frame: OdomFrame, field: ObstacleField, slice_height_m: float,
                 limits: VehicleLimits, config: MissionConfig, follower: FollowerConfig = FollowerConfig(),
                 launch_heading: Optional[float] = None):
        """``frame``: the measured course->odom transform (frames.py). ``launch_heading``: the
        map's launch heading in course radians, the precheck's heading reference (None: no
        heading precheck -- the frame itself is still measured, never derived from it)."""
        self.plan = plan
        self.frame = frame
        self.field = field
        self.limits = limits
        self.config = config
        self._launch_yaw_odom = None if launch_heading is None else frame.yaw_to_odom(launch_heading)
        # The mission predicts across the odometry's real age itself, so the Follower must not again.
        self.follower_config = dataclasses.replace(follower, odom_latency_s=0.0)
        self.z_fly = slice_height_m - config.rest_height_m
        if self.z_fly <= 0.0:
            raise ValueError(f"flight height {slice_height_m:.2f} m is not above the rest height "
                             f"{config.rest_height_m:.2f} m")
        traj = plan.trajectory
        self.start_odom = frame.to_odom(*traj.xy[0])
        self.end_odom = frame.to_odom(*traj.xy[-1])
        self.z_return = config.return_height_m - config.rest_height_m
        if config.return_leg:
            if not config.land:
                raise ValueError("return_leg needs land: true (the return starts from a touchdown)")
            if config.return_height_m <= slice_height_m:
                raise ValueError(f"return_height_m {config.return_height_m:.2f} m must be above the flight "
                                 f"height {slice_height_m:.2f} m; set it for this course (it has no default)")
            if config.return_speed_mps <= 0.0:
                raise ValueError("return_speed_mps must be > 0")
            if config.return_lag_m <= 0.0:
                # The reference starts on the drone, over the pad: with no lag it can never move ahead.
                raise ValueError("return_lag_m must be > 0 (with 0 the return never leaves the pad)")
        self._land_xy: Tuple[float, float] = self.end_odom
        self._hold_z = self.z_fly
        self._returning = False
        #: PX4 has said "not landed" since Offboard began, or since the last touchdown. Only then is
        #: a "landed" a touchdown: a reading left over from the launch (or the pad) is not.
        self._airborne_seen = False
        self._return_s = 0.0
        self._return_t: Optional[float] = None
        self.state = WAIT
        self.reason = "waiting for the prechecks"
        self.follower: Optional[Follower] = None
        self.last_setpoint: Optional[Setpoint] = None
        self._was_offboard = False
        self._takeoff_xy: Tuple[float, float] = self.start_odom
        self._takeoff_yaw = 0.0
        self._takeoff_offered = False
        self._hold_since: Optional[float] = None
        self._hold_cmd: Optional[Command] = None
        self._yaw_cmd = 0.0
        self.coasts = 0                                  # dropouts coasted through, this run
        self.stitches = []                               # (t, dx, dy): frame jumps re-anchored
        self.offset = (0.0, 0.0)                         # internal odom = input odom + offset
        self._still_since: Optional[float] = None
        self._last_good: Optional[OdomPose] = None       # newest odometry flown on (internal frame)
        self._dr: Optional[_DeadReckoning] = None        # from a coast's start to the resume
        self._frame_lost: Optional[float] = None         # an unexplained jump (m): never fly on
        self._px4_velocity: Optional[Tuple[float, float, float]] = None   # (t, forward, left)
        self._px4_yaw: Optional[float] = None            # PX4's heading this tick (ENU), if known
        self._yaw_gap: Optional[float] = None            # odometry yaw - PX4 yaw, at the last good pose

    @property
    def started(self) -> bool:
        """Offboard has begun, so the vehicle may be in the air and this mission owns its setpoints."""
        return self._was_offboard

    # -- the tick ------------------------------------------------------------------

    def update(self, t: float, offboard_active: bool, pose: Optional[OdomPose],
               landed: Optional[bool] = None, px4_yaw: Optional[float] = None,
               px4_velocity: Optional[Tuple[float, float, float]] = None) -> Optional[Command]:
        """``px4_yaw``: PX4's heading as an ENU yaw (None if unknown), for the heading gate.
        ``px4_velocity``: PX4's horizontal velocity estimate as ``(t, forward, left)`` in the body's
        heading frame (None if unknown), for a coast's dead reckoning.

        Poses are shifted into the internal frame by :attr:`offset` (zero until a coast's stitch),
        and pose setpoints shifted back out of it, so callers see only their own frame."""
        self._px4_velocity = px4_velocity
        self._px4_yaw = px4_yaw
        if pose is not None and self.offset != (0.0, 0.0):
            pose = dataclasses.replace(pose, x=pose.x + self.offset[0], y=pose.y + self.offset[1])
        cmd = self._update(t, offboard_active, pose, landed, px4_yaw)
        if cmd is not None and cmd.kind == "pose" and self.offset != (0.0, 0.0):
            cmd = dataclasses.replace(cmd, x=cmd.x - self.offset[0], y=cmd.y - self.offset[1])
        return cmd

    def _update(self, t: float, offboard_active: bool, pose: Optional[OdomPose],
                landed: Optional[bool], px4_yaw: Optional[float]) -> Optional[Command]:
        if self.state == STOPPED:
            return None
        if self._was_offboard and landed is False:
            self._airborne_seen = True
        if self._was_offboard and not offboard_active:
            if self.state == LANDED:
                return self._stop("done: landed, and PX4 disarmed (COM_DISARM_LAND)")
            return self._stop("Offboard ended: the pilot has the vehicle")
        fresh = pose is not None and 0.0 <= t - pose.t <= self.config.pose_timeout_s
        if not offboard_active:
            return self._wait(t, pose if fresh else None, px4_yaw)
        if not self._was_offboard:
            self._was_offboard = True
            why = self._precheck(pose if fresh else None, px4_yaw)
            if why:
                self.state = GROUNDED
                self.reason = f"Offboard started but {why}: keeping it on the ground"
            else:
                self.state = TAKEOFF
                self.reason = "taking off"
        if self.state == GROUNDED:
            return Command("twist")
        if self.state == HELD:
            return self._hold_cmd
        if self.state == FOLLOW and self.config.coast_max_s > 0.0:
            flowing = fresh and (pose.received is None or t - pose.received <= self.config.pose_gap_s)
            if not flowing:
                return self._dropout(t)
            if self._dr is not None:
                pose = self._end_coast(t, pose)
            if self._frame_lost is not None:
                return self._hold_still(t, f"odometry resumed {self._frame_lost:.2f} m from the dead reckoning, "
                                           "more than the coast explains: holding for the pilot")
            self._last_good = pose
            self._yaw_gap = None if px4_yaw is None else wrap(pose.yaw - px4_yaw)
            self._still_since = None
        if not fresh:
            if self.follower is not None:
                self.follower.pause(t)
            if self.state in (LAND, LANDED):
                return self._descend_without_pose(t, landed)
            self.reason = "no fresh pose: sending nothing, so the interface holds the last checked pose"
            return None
        handler = {TAKEOFF: self._takeoff, FOLLOW: self._follow, HOLD: self._hold, LAND: self._land,
                   LANDED: self._landed, RETURN_CLIMB: self._return_climb,
                   RETURN_TRANSIT: self._return_transit}[self.state]
        return handler(t, pose, landed)

    # -- states --------------------------------------------------------------------

    def _wait(self, t: float, pose: Optional[OdomPose], px4_yaw: Optional[float]) -> Optional[Command]:
        why = self._precheck(pose, px4_yaw)
        if why:
            if not self._takeoff_offered:
                # Nothing was ever streamed: Offboard is refused and the pilot keeps the vehicle.
                self.reason = f"not offering takeoff: {why}"
                return None
            # The kit's pose setpoint does not expire: it keeps streaming the offered
            # takeoff pose (flight_interface.py: "A position setpoint IS a hold"), and
            # a switch to Position is still granted Offboard up to 1 s after our last
            # setpoint (auto_offboard.py). Stopping alone would leave that climb armed,
            # so a zero-velocity twist must replace the stored pose -- the same fail-closed
            # command GROUNDED uses.
            self.reason = f"precheck failed: {why}: zero-velocity setpoint replaces the takeoff pose"
            return Command("twist")
        self.reason = "ready: offering the takeoff pose for the switch to Position"
        self._takeoff_xy = (pose.x, pose.y)
        self._takeoff_yaw = pose.yaw
        self._yaw_cmd = pose.yaw
        self._takeoff_offered = True
        return Command("pose", pose.x, pose.y, self.z_fly, pose.yaw)

    def _precheck(self, pose: Optional[OdomPose], px4_yaw: Optional[float]) -> Optional[str]:
        cfg = self.config
        if pose is None:
            return "no fresh pose"
        d = math.hypot(pose.x - self.start_odom[0], pose.y - self.start_odom[1])
        if d > cfg.start_tolerance_m:
            return f"{d:.2f} m from the route start (allowed {cfg.start_tolerance_m:.2f} m)"
        if self._launch_yaw_odom is not None:
            heading_err = abs(wrap(pose.yaw - self._launch_yaw_odom))
            if math.degrees(heading_err) > cfg.heading_tolerance_deg:
                return f"heading {math.degrees(heading_err):.1f} deg off the launch heading"
        if cfg.px4_heading_gate_deg > 0:
            if px4_yaw is None:
                return "no heading from PX4 for the heading gate"
            gap = abs(wrap(px4_yaw - pose.yaw))
            if math.degrees(gap) > cfg.px4_heading_gate_deg:
                return f"PX4's heading is {math.degrees(gap):.1f} deg from vision's (gate {cfg.px4_heading_gate_deg:g})"
        return None

    def _takeoff(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        if abs(pose.z - self.z_fly) < self.config.takeoff_tolerance_m:
            self.state = FOLLOW
            self.reason = "following"
            self.follower = Follower(self.plan.trajectory, self.field, self.limits, self.follower_config)
            return self._follow(t, pose, landed)
        return Command("pose", self._takeoff_xy[0], self._takeoff_xy[1], self.z_fly, self._takeoff_yaw)

    def _follow(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        age = min(max(t - pose.t, 0.0), self.config.max_prediction_s)
        x, y = pose.x + pose.vx * age, pose.y + pose.vy * age
        sp = self.follower.update(t, self.frame.to_course(x, y),
                                  self.frame.rotate_to_course(pose.vx, pose.vy))
        self.last_setpoint = sp
        if sp.cross_track_m > self.config.abort_cross_track_m:
            self.state = HELD
            self.reason = (f"{sp.cross_track_m:.2f} m off the path (limit {self.config.abort_cross_track_m:.2f} m):"
                           " holding here for the pilot")
            self._hold_cmd = Command("pose", pose.x, pose.y, self.z_fly, pose.yaw)
            return self._hold_cmd
        if sp.done:
            self.state = HOLD
            self.reason = "over the pad"
            self._hold_since = None
            return self._hold(t, pose, landed)
        sx, sy = self.frame.to_odom(*sp.position)
        self._yaw_cmd = self.frame.yaw_to_odom(sp.yaw)
        return Command("pose", sx, sy, self.z_fly, self._yaw_cmd)

    def _hold(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        cmd = Command("pose", self._land_xy[0], self._land_xy[1], self._hold_z, self._yaw_cmd)
        if not self.config.land:
            return cmd
        near = math.hypot(self._land_xy[0] - pose.x, self._land_xy[1] - pose.y) < self.config.land_tolerance_m
        if not near:
            self._hold_since = None
            return cmd
        if self._hold_since is None:
            self._hold_since = t
        if t - self._hold_since >= self.config.hold_before_land_s:
            self.state = LAND
            self.reason = "landing"
            return self._land(t, pose, landed)
        return cmd

    def _land(self, t: float, pose: Optional[OdomPose], landed: Optional[bool]) -> Command:
        if landed and self._airborne_seen:
            if self.config.return_leg and not self._returning:
                return self._begin_return()
            self.state = LANDED
            return self._landed(t, pose, landed)
        self.reason = "landing"
        return self._descend(pose, self.config.descent_mps)

    def _landed(self, t: float, pose: Optional[OdomPose], landed: Optional[bool]) -> Command:
        self.reason = "landed: pressing down until PX4 disarms (COM_DISARM_LAND)"
        return self._descend(pose, 0.3 * self.config.descent_mps)

    def _begin_return(self) -> Command:
        """Touched the pad and PX4 says landed: climb now, before COM_DISARM_LAND disarms it."""
        self._returning = True
        self._airborne_seen = False          # the landing at the launch mark needs its own liftoff first
        self.state = RETURN_CLIMB
        self.reason = "touched the pad: climbing for the return without disarming"
        return Command("pose", self.end_odom[0], self.end_odom[1], self.z_return, self._yaw_cmd)

    def _return_climb(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        if abs(pose.z - self.z_return) < self.config.takeoff_tolerance_m:
            self.state = RETURN_TRANSIT
            self.reason = "returning to the launch mark"
            self._return_t = t
            return self._return_transit(t, pose, landed)
        return Command("pose", self.end_odom[0], self.end_odom[1], self.z_return, self._yaw_cmd)

    def _return_transit(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        ax, ay = self.end_odom
        bx, by = self.start_odom
        length = max(math.hypot(bx - ax, by - ay), 1e-9)
        ux, uy = (bx - ax) / length, (by - ay) / length
        rx, ry = pose.x - ax, pose.y - ay
        along, across = rx * ux + ry * uy, abs(rx * uy - ry * ux)
        if across > self.config.abort_cross_track_m:
            self.state = HELD
            self.reason = (f"{across:.2f} m off the return line (limit {self.config.abort_cross_track_m:.2f} m):"
                           " holding here for the pilot")
            self._hold_cmd = Command("pose", pose.x, pose.y, self.z_return, self._yaw_cmd)
            return self._hold_cmd
        dt = max(t - self._return_t, 0.0)
        self._return_t = t
        # Capped by the drone's own position, so a vision gap (a large dt) cannot throw the reference
        # far ahead: a far pose setpoint is a fast flight, the kit does not limit its speed.
        advanced = min(self._return_s + self.config.return_speed_mps * dt, along + self.config.return_lag_m)
        self._return_s = min(max(self._return_s, advanced), length)
        if self._return_s >= length and math.hypot(bx - pose.x, by - pose.y) < self.config.land_tolerance_m:
            self._land_xy = self.start_odom
            self._hold_z = self.z_return
            self._hold_since = None
            self.state = HOLD
            self.reason = "over the launch mark"
            return self._hold(t, pose, landed)
        return Command("pose", ax + ux * self._return_s, ay + uy * self._return_s, self.z_return, self._yaw_cmd)

    def _descend_without_pose(self, t: float, landed: Optional[bool]) -> Command:
        """No fresh pose while landing: keep descending, straight down. Sending
        nothing is not a hold here. The interface expires a twist after 1 s into
        its last pose, which is the pad at flight height, so the drone would climb
        back up (``flight_interface._expire_stale_velocity``)."""
        cmd = (self._land if self.state == LAND else self._landed)(t, None, landed)
        self.reason += ": no fresh pose, so straight down"
        return cmd

    def _descend(self, pose: Optional[OdomPose], rate: float) -> Command:
        if pose is None:
            return Command("twist", vz=-rate)
        cfg = self.config
        vx = cfg.land_xy_gain * (self._land_xy[0] - pose.x)
        vy = cfg.land_xy_gain * (self._land_xy[1] - pose.y)
        speed = math.hypot(vx, vy)
        if speed > cfg.land_xy_max_mps:
            vx, vy = vx * cfg.land_xy_max_mps / speed, vy * cfg.land_xy_max_mps / speed
        return Command("twist", vx=vx, vy=vy, vz=-rate)

    # -- dropouts ----------------------------------------------------------------------

    def _dropout(self, t: float) -> Command:
        """No odometry to fly on in FOLLOW: coast on along the trajectory, then hold still."""
        cfg = self.config
        if self._dr is None:
            if self._last_good is None or self._frame_lost is not None:
                return self._hold_still(t, "no fresh pose")
            self._dr = _DeadReckoning(self._last_good, t)
            self.coasts += 1
        dr = self._dr
        dr.step(t, cfg.coast_response_s, self._measured_velocity(t))
        coasted = t - dr.since
        if coasted < cfg.coast_max_s and dr.halted is None:
            here = self.frame.to_course(dr.x, dr.y)
            sp = self.follower.update(t, here, self.frame.rotate_to_course(dr.vx, dr.vy))
            self.last_setpoint = sp
            # The same safety stops as _follow. Either one ends the coast: the correction would
            # otherwise keep pulling toward a reference behind a wall, or further off the path.
            if sp.cross_track_m > cfg.abort_cross_track_m:
                dr.cmd = (0.0, 0.0)
                self.state = HELD
                self.reason = (f"{sp.cross_track_m:.2f} m off the path while coasting (limit "
                               f"{cfg.abort_cross_track_m:.2f} m): holding still for the pilot")
                self._hold_cmd = Command("twist")    # refreshed every tick: it never expires into a pose
                return self._hold_cmd
            if "no_safe_target" in sp.flags:
                dr.halted = "no safe target ahead"
            elif not sp.done:
                ref = self.plan.trajectory.at(sp.ref_time)
                ve = sp.clock_rate * ref.velocity[0] + cfg.coast_gain * (ref.position[0] - here[0])
                vn = sp.clock_rate * ref.velocity[1] + cfg.coast_gain * (ref.position[1] - here[1])
                speed = math.hypot(ve, vn)
                if speed > self.limits.v_max_mps:
                    ve, vn = ve * self.limits.v_max_mps / speed, vn * self.limits.v_max_mps / speed
                vx, vy = self.frame.rotate_to_odom(ve, vn)
                dr.cmd = (vx, vy)
                self._still_since = None
                self.reason = (f"no fresh pose: coasting on the trajectory by dead reckoning "
                               f"({coasted:.1f} s, {dr.travelled:.2f} m)")
                return Command("twist", vx=vx, vy=vy)
        dr.cmd = (0.0, 0.0)
        why = f"; coast stopped: {dr.halted}" if dr.halted else ""
        return self._hold_still(t, f"no fresh pose; coasted {dr.travelled:.2f} m{why}")

    def _measured_velocity(self, t: float) -> Optional[Tuple[float, float]]:
        """PX4's velocity estimate in the odom frame, if fresh: its (forward, left) turned by the
        drone's heading now. That is PX4's current heading plus the gap between vision's and PX4's
        measured at the last good pose; without PX4's heading, the last odometry's (frozen, so it
        lags a turn)."""
        sample = self._px4_velocity
        if (sample is None or self._last_good is None
                or not 0.0 <= t - sample[0] <= self.config.px4_velocity_timeout_s):
            return None
        _t, forward, left = sample
        if not (math.isfinite(forward) and math.isfinite(left)):
            return None
        yaw = self._last_good.yaw
        if self._px4_yaw is not None and self._yaw_gap is not None:
            yaw = self._px4_yaw + self._yaw_gap
        c, s = math.cos(yaw), math.sin(yaw)
        return c * forward - s * left, s * forward + c * left

    def _hold_still(self, t: float, why: str) -> Command:
        """Zero velocity, refreshed: it needs no frame, and it never expires into an old pose."""
        if self._still_since is None:
            self._still_since = t
        if self.follower is not None:
            self.follower.pause(t)                   # holding still is not flight time
        self.reason = f"holding still: {why} ({t - self._still_since:.1f} s)"
        return Command("twist")

    def _end_coast(self, t: float, pose: OdomPose) -> OdomPose:
        """The odometry is back after a coast: compare it with the dead reckoning, stitch the jump."""
        cfg = self.config
        dr, self._dr = self._dr, None
        dr.step(t, cfg.coast_response_s, self._measured_velocity(t))
        back = max(0.0, t - pose.t)                  # the dead reckoning at the pose's capture time
        jx, jy = dr.x - dr.vx * back - pose.x, dr.y - dr.vy * back - pose.y
        jump = math.hypot(jx, jy)
        if not cfg.stitch_after_coast or jump < max(cfg.stitch_min_m, cfg.stitch_fraction * dr.travelled):
            return pose                              # tracking carried on: the dead reckoning's own error
        if jump > dr.travelled + cfg.stitch_margin_m:
            self._frame_lost = jump
            return pose
        self.offset = (self.offset[0] + jx, self.offset[1] + jy)
        self.stitches.append((t, jx, jy))
        return dataclasses.replace(pose, x=pose.x + jx, y=pose.y + jy)

    def _stop(self, why: str) -> None:
        self.state = STOPPED
        self.reason = why
        return None

    # -- status --------------------------------------------------------------------

    def status(self) -> dict:
        sp = self.last_setpoint
        traj = self.plan.trajectory
        return {
            "state": self.state,
            "reason": self.reason,
            "route": self.plan.route.goals,
            "planned_s": round(traj.duration, 2),
            "progress": round(traj.at(sp.ref_time).progress, 3) if sp is not None else 0.0,
            "lag_m": round(sp.lag_m, 3) if sp is not None else None,
            "cross_track_m": round(sp.cross_track_m, 3) if sp is not None else None,
            "lead_scale": round(sp.lead_scale, 2) if sp is not None else None,
            "flags": list(sp.flags) if sp is not None else [],
            "coasts": self.coasts,
            "offset_m": [round(self.offset[0], 3), round(self.offset[1], 3)],
        }
