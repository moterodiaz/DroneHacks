"""The flight, arming to disarm: wait, take off, follow, hold over the pad, land.

The state machine is ported from the lidar stream's ``follower/mission.py``,
at commit ``6dc1263`` on ``core/port-depth-camera-testing``. Three things differ:

1. **FOLLOW flies this repo's Follower** (``dronehacks_planner.follower``):
   the smoothed trajectory with its speed profile, its reference poses (the
   lead formula is opt-in and off by default), and the progress governor.
   The lidar stream flies a carrot on the polyline.
2. **It publishes the takeoff pose while waiting, but only while the
   prechecks pass.** The kit grants Offboard only if a team setpoint is under
   1 s old when the pilot switches to Position (``auto_offboard.py``). The
   prechecks:
   * a fresh pose;
   * the drone still where the tracker started (the launch mark);
   * its heading still the launch heading;
   * optionally, PX4's heading agreeing with vision's (field guide, build step 2).

   While they fail before any takeoff pose was streamed it publishes
   nothing, so Offboard is refused and the pilot keeps the vehicle. After a
   takeoff pose was streamed it replaces it with a zero-velocity twist: the
   kit's pose setpoint does not expire, and Offboard is still granted up to
   1 s after the last team setpoint, so merely stopping would leave the
   drone able to climb on the stored pose. The committed lidar mission
   published nothing while waiting at all, which the kit would always
   refuse.
3. **No lidar, no pose_fusion.** The pose is cuVSLAM's (or the Guard's)
   odometry, in the ``odom`` frame. The setpoints go straight to the kit's
   flight interface.

Rules kept from the lidar stream (competitor_guide.md §3-§4):
* Nothing before Offboard except the takeoff pose. If Offboard ends, for any
  reason, stop for good: never re-request it, never arm, disarm or change mode.
* Landing uses a velocity setpoint, and only with ``land: true`` (off by
  default: the mission then holds over the pad for the pilot, px4.md). The
  kit floors poses at ``min_altitude_m``, but only caps twists. PX4 disarms
  itself ``COM_DISARM_LAND`` after landing.

Added for safety:
* **GROUNDED.** Offboard starts but the pose is stale or a precheck now
  fails. The interface already holds the takeoff pose, so stopping alone
  would still take off. Instead, zero-velocity twists keep the drone on the
  ground until PX4's ``COM_DISARM_PRFLT`` disarms it or the pilot takes over.
* **HELD.** Cross-track error over ``abort_cross_track_m``: hold where it is
  and wait for the pilot.
* **A stale pose in the air**, outside FOLLOW, sends nothing. The interface
  then holds the last pose: a led pose the Follower has checked is reachable
  in a straight line.
* **A pose dropout in FOLLOW coasts on.** PX4's only horizontal position
  source is cuVSLAM. When it stops (the dark tube), EKF2 dead-reckons on the
  IMU for ``EKF2_NOAID_TOUT`` (5 s on drone 1), then declares the position
  invalid: Offboard ends and PX4 falls back to Altitude, which sinks with the
  throttle low (competitor_guide.md section 4). Stopping in the dark
  guarantees that. So the mission flies on along the trajectory for up to
  ``coast_max_s``: a dead reckoning (the last odometry, then PX4's own
  velocity estimate, which EKF2 dead-reckons on the IMU; the node passes it
  in the body's heading frame, so no frame relation is needed. Without a
  fresh one, the velocities commanded since, each reached with a
  ``coast_response_s`` lag) stands in for the pose, the Follower keeps running on it (its reference clock,
  progress and checks), and the command is a **velocity**: the reference's,
  times the Follower's clock rate, plus ``coast_gain`` times the dead
  reckoning's offset from the reference, capped at ``v_max_mps``. A velocity
  needs no position frame. After ``coast_max_s``, or at the trajectory's
  end, it holds still by zero velocity (sending nothing would let the
  interface expire the twist into the last pose setpoint, behind the drone).
* **The frame after a coast.** cuVSLAM (stereo VO, ``enable_imu_fusion`` off
  in the kit) resumes from the pose where it lost tracking, so its frame
  jumps back by the motion it missed; the organisers' bridge bumps
  ``reset_counter`` and PX4 follows it. With ``stitch_after_coast``, a resumed
  pose at least ``stitch_fraction`` of the coast (and ``stitch_min_m``) from
  the dead reckoning is taken as that jump:
  it is added to every later pose and subtracted from every later pose
  setpoint, so the trajectory stays where it is on the course. A jump larger
  than the coast (+ ``stitch_margin_m``) can't be explained: it holds still
  by velocity for the pilot.

Opt-in shortcuts (both off by default; the window flight lines turn them on):
* **Rolling take-off** (``rolling_takeoff_m`` > 0). TAKEOFF hands over to FOLLOW as soon as the
  drone is that far above its rest height, and FOLLOW finishes the climb while it flies the
  first leg. The route is checked in plan only, at the flight height, so FOLLOW holds its
  horizontal progress, still climbing, wherever the path ahead needs more height than the drone
  has: the flight height from ``full_height_lead_m`` before the first opening (tunnel, window,
  gate) to the end of the last; a boulder's top plus ``low_obstacle_clearance_m`` within
  ``low_obstacle_reach_m`` of its edge (every boulder in the map, including any the plan ignored).
  Walls stand on the floor, so a wall clear at the flight height is clear lower down.
* **Early descent** (``early_descent``). From the end of the last opening's pass (the whole
  drone ``crossing_margin_m`` past its far face), the height target drops to
  ``descent_floor_m`` above the floor, never below what the path ahead needs (as above), so the
  last leg is flown descending. Over the pad the mission holds at that height (``land`` false:
  the pilot lands), or goes into LAND after ``hold_before_land_s``.

ROS-free; ``mission_node.py`` wraps it.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from dronehacks_planner.clearance import ObstacleField
from dronehacks_planner.course_map import Boulder
from dronehacks_planner.follower import Follower, FollowerConfig, Setpoint
from dronehacks_planner.limits import VehicleLimits
from dronehacks_planner.pipeline import FlightPlan

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
    #: FOLLOW: no odometry arriving for this long counts as a dropout (when arrivals are known).
    pose_gap_s: float = 0.15
    #: FOLLOW: fly on through a dropout this long, by velocity along the trajectory. 0 turns it off.
    coast_max_s: float = 4.0
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
    #: FOLLOW by velocity (factory.for_track_speed sets it with a track speed): the reference
    #: velocity times the Follower's clock rate, plus ``follow_gain`` times the offset from the
    #: reference, capped at ``v_max_mps``. False: the reference pose itself (the shipped behaviour).
    velocity_follow: bool = False
    #: 1/s: the pull onto the reference while following by velocity.
    follow_gain: float = 1.0
    #: The velocity commanded is the reference's this much later: PX4's velocity loop takes about
    #: 1/MPC_XY_VEL_P_ACC (1.8 on drone 1) to reach a new one, so without it the drone runs on
    #: past a turn by that much (twists only: no led pose, contracts/ros-graph.md).
    follow_anticipation_s: float = 0.55
    #: 1/s and m/s: holding the flight height by velocity.
    follow_z_gain: float = 1.0
    follow_vz_max_mps: float = 0.5
    #: How often a velocity FOLLOW also sends a hold pose at the drone's position, so that if the
    #: twists stop the interface falls back to holding there, not to the takeoff pose.
    fallback_pose_period_s: float = 0.5
    #: The interface's max_speed_mps: it clamps every twist to this (competitor.launch.py: 1.5).
    twist_speed_max_mps: float = 1.5
    #: HELD: brake by zero velocity while faster than this (and at most held_brake_s), then hold
    #: where the drone stopped. A pose hold set while moving fast would make PX4 fly back to it.
    held_brake_speed_mps: float = 0.15
    held_brake_s: float = 2.0
    #: Rolling take-off: FOLLOW starts once the drone is this far above its rest height and climbs
    #: the rest of the way while flying (module docstring). 0: climb to the flight height first,
    #: within takeoff_tolerance_m (the shipped behaviour).
    rolling_takeoff_m: float = 0.0
    #: The flight height is needed from this far along the path before the first opening.
    full_height_lead_m: float = 0.5
    #: Early descent after the last opening (module docstring). False: the last leg is flown at
    #: the flight height (the shipped behaviour).
    early_descent: bool = False
    #: The lowest the early descent goes before the pad: body centre above the floor. The kit
    #: floors pose setpoints at 0.3 m (contracts/px4.md); this less the rest height stays above it.
    descent_floor_m: float = 0.45
    #: A boulder within its radius plus this of the path needs the drone above its top there:
    #: the 0.28 m prop reach (contracts/course.md) + 0.10 m.
    low_obstacle_reach_m: float = 0.38
    #: ...by this much, body centre over the boulder's top: half the drone's height (0.10 m) plus
    #: the planner's boulder_overhead_m (0.15 m).
    low_obstacle_clearance_m: float = 0.25
    #: How far ahead along the trajectory (its time) the height needed is looked up.
    height_lookahead_s: float = 1.0

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
    #: With a twist: a pose to send just before it, (x, y, z, yaw) in the odom frame. The
    #: interface keeps the last pose as the place to hold once a twist expires
    #: (flight_interface._expire_stale_velocity), so this keeps that place under the drone.
    fallback: Optional[Tuple[float, float, float, float]] = None


class _DeadReckoning:
    """Where the drone is during a dropout (odom frame): its last odometry, then the velocities
    commanded since, each approached with a first-order lag (PX4's velocity loop)."""

    def __init__(self, pose: OdomPose) -> None:
        self.x, self.y, self.t = pose.x, pose.y, pose.t
        self.vx, self.vy = pose.vx, pose.vy
        self.cmd = (pose.vx, pose.vy)          # until the first command: carrying on as it was
        self.since = 0.0                       # when the coast started (the mission's clock)
        self.travelled = 0.0

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
                 launch_heading: Optional[float] = None, low_obstacles: Sequence[Boulder] = ()):
        """``frame``: the measured course->odom transform (frames.py). ``launch_heading``: the
        map's launch heading in course radians, the precheck's heading reference (None: no
        heading precheck -- the frame itself is still measured, never derived from it).
        ``low_obstacles``: the map's boulders, including any the plan ignored, for the height the
        rolling take-off and the early descent keep over them."""
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
        self.floor_odom = config.descent_floor_m - config.rest_height_m
        if config.early_descent and not 0.0 < self.floor_odom <= self.z_fly:
            raise ValueError(f"descent_floor_m {config.descent_floor_m:.2f} m must be above the rest height "
                             f"{config.rest_height_m:.2f} m and at most the flight height {slice_height_m:.2f} m")
        self._need, self.descent_from_t = self._height_profile(plan, low_obstacles)
        self._climbing = False
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
        self.holds = 0                                   # zero-velocity holds in FOLLOW, this run
        self.coasts = 0                                  # dropouts coasted through, this run
        self.stitches = []                               # (t, dx, dy): frame jumps re-anchored
        self.offset = (0.0, 0.0)                         # internal odom = input odom + offset
        self._still_since: Optional[float] = None
        self._last_good: Optional[OdomPose] = None       # newest odometry flown on (internal)
        self._dr: Optional[_DeadReckoning] = None        # from a coast's start to the resume
        self._frame_lost: Optional[float] = None         # an unexplained jump (m): never fly on
        self._px4_velocity: Optional[Tuple[float, float, float]] = None   # (t, forward, left)
        self._fallback_t = -math.inf                     # when a velocity FOLLOW last sent its hold pose
        self._held_since = 0.0                           # when HELD began (it brakes first)

    # -- height along the path -----------------------------------------------------

    def _height_profile(self, plan: FlightPlan, low_obstacles: Sequence[Boulder]):
        """``(need, descent_from_t)``: the least odom height each trajectory sample needs (-inf:
        none), and the trajectory time the early descent starts (inf: never)."""
        cfg = self.config
        traj = plan.trajectory
        full = self.z_fly - cfg.takeoff_tolerance_m
        need = np.full(len(traj.t), -np.inf)
        inside = [i for i, name in enumerate(plan.path.pass_name) if name]
        if inside:
            # up to the last opening's end, where the whole drone is past it and the descent may start
            span = (traj.s >= traj.s[inside[0]] - cfg.full_height_lead_m) & (traj.s < traj.s[inside[-1]])
            need[span] = full
        for b in low_obstacles:
            reach = b.radius + cfg.low_obstacle_reach_m
            near = np.hypot(traj.xy[:, 0] - b.centre[0], traj.xy[:, 1] - b.centre[1]) < reach
            top = full if b.height is None else min(full, b.height + cfg.low_obstacle_clearance_m - cfg.rest_height_m)
            need[near] = np.maximum(need[near], top)
        descent_from_t = float(traj.t[inside[-1]]) if cfg.early_descent and inside else math.inf
        return need, descent_from_t

    def need_between(self, t0: float, t1: float) -> float:
        """The most height any trajectory sample from time t0 to t1 needs (odom z; -inf: none)."""
        t = self.plan.trajectory.t
        i0 = max(int(np.searchsorted(t, t0, side="right")) - 1, 0)
        i1 = max(int(np.searchsorted(t, t1, side="right")), i0 + 1)
        return float(self._need[i0:i1].max())

    def z_target(self, ref_time: float) -> float:
        """The height FOLLOW flies at a reference time (odom z): the flight height, and once the
        early descent has started the descent floor, kept above what the path ahead needs."""
        if ref_time < self.descent_from_t:
            return self.z_fly
        need = self.need_between(ref_time, ref_time + self.config.height_lookahead_s)
        return min(self.z_fly, max(self.floor_odom, need))

    # -- the tick ------------------------------------------------------------------

    def update(self, t: float, offboard_active: bool, pose: Optional[OdomPose],
               landed: Optional[bool] = None, px4_yaw: Optional[float] = None,
               px4_velocity: Optional[Tuple[float, float, float]] = None) -> Optional[Command]:
        """``px4_yaw``: PX4's heading as an ENU yaw (None if unknown), for the heading gate.
        ``px4_velocity``: PX4's horizontal velocity estimate as ``(t, forward, left)`` in the
        body's heading frame (None if unknown), for a coast's dead reckoning.

        Poses are shifted into the internal frame by :attr:`offset` (zero until a coast's
        stitch), and pose setpoints shifted back out of it, so callers see only their own frame."""
        self._px4_velocity = px4_velocity
        if pose is not None and self.offset != (0.0, 0.0):
            pose = dataclasses.replace(pose, x=pose.x + self.offset[0], y=pose.y + self.offset[1])
        cmd = self._update(t, offboard_active, pose, landed, px4_yaw)
        if cmd is not None and self.offset != (0.0, 0.0):
            if cmd.kind == "pose":
                cmd = dataclasses.replace(cmd, x=cmd.x - self.offset[0], y=cmd.y - self.offset[1])
            elif cmd.fallback is not None:
                fx, fy, fz, fyaw = cmd.fallback
                cmd = dataclasses.replace(cmd, fallback=(fx - self.offset[0], fy - self.offset[1], fz, fyaw))
        return cmd

    def _update(self, t: float, offboard_active: bool, pose: Optional[OdomPose],
                landed: Optional[bool], px4_yaw: Optional[float]) -> Optional[Command]:
        if self.state == STOPPED:
            return None
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
            return self._held(t, pose if fresh else None)
        if self.state == FOLLOW:
            flowing = fresh and (pose.received is None or t - pose.received <= self.config.pose_gap_s)
            if not flowing:
                return self._dropout(t)
            if self._dr is not None:
                pose = self._end_coast(t, pose)
            if self._frame_lost is not None:
                return self._hold_still(t, f"odometry resumed {self._frame_lost:.2f} m from the dead reckoning, "
                                           "more than the coast explains: holding for the pilot")
            self._last_good = pose
            self._still_since = None
        if not fresh:
            if self.state in (LAND, LANDED):
                return self._descend_without_pose(t, landed)
            self.reason = "no fresh pose: sending nothing, so the interface holds the last checked pose"
            return None
        handler = {TAKEOFF: self._takeoff, FOLLOW: self._follow, HOLD: self._hold, LAND: self._land,
                   LANDED: self._landed}[self.state]
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
        cfg = self.config
        at_height = abs(pose.z - self.z_fly) < cfg.takeoff_tolerance_m
        rolling = cfg.rolling_takeoff_m > 0.0 and pose.z >= min(cfg.rolling_takeoff_m, self.z_fly)
        if at_height or rolling:
            self.state = FOLLOW
            self.reason = "following"
            self._climbing = not at_height
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
            self._hold_cmd = None
            self._held_since = t
            return self._held(t, pose)
        if sp.done:
            self.state = HOLD
            self.reason = "over the pad"
            self._hold_since = None
            return self._hold(t, pose, landed)
        self._yaw_cmd = self.frame.yaw_to_odom(sp.yaw)
        z_ref = self.z_target(sp.ref_time)
        wait = False
        if self._climbing:
            if pose.z >= self.z_fly - self.config.takeoff_tolerance_m:
                self._climbing = False
            else:
                wait = pose.z < self.need_between(sp.ref_time, sp.ref_time + self.config.height_lookahead_s)
        if self.config.velocity_follow and isinstance(self.follower, Follower):
            return self._follow_by_velocity(t, sp, (x, y), pose, z_ref, wait)
        if wait:
            self.reason = "climbing in place: the path ahead needs more height"
            return Command("pose", x, y, z_ref, self._yaw_cmd)
        sx, sy = self.frame.to_odom(*sp.position)
        return Command("pose", sx, sy, z_ref, self._yaw_cmd)

    def _follow_by_velocity(self, t: float, sp: Setpoint, xy: Tuple[float, float], pose: OdomPose,
                            z_ref: float, wait: bool = False) -> Command:
        """The trajectory by velocity: the reference's ``follow_anticipation_s`` ahead (times the
        governor's clock rate), plus a pull onto the reference, capped at ``v_max_mps``; height
        (``z_ref``) and heading held by rate. ``wait``: no horizontal velocity (climbing in place)."""
        cfg = self.config
        ref = self.plan.trajectory.at(sp.ref_time)
        ahead = self.plan.trajectory.at(sp.ref_time + sp.clock_rate * cfg.follow_anticipation_s)
        here = self.frame.to_course(*xy)
        ff = 0.0 if "no_safe_target" in sp.flags else sp.clock_rate
        ve = ff * ahead.velocity[0] + cfg.follow_gain * (ref.position[0] - here[0])
        vn = ff * ahead.velocity[1] + cfg.follow_gain * (ref.position[1] - here[1])
        speed = math.hypot(ve, vn)
        cap = min(self.limits.v_max_mps, cfg.twist_speed_max_mps)
        if speed > cap:
            ve, vn = ve * cap / speed, vn * cap / speed
        vx, vy = (0.0, 0.0) if wait else self.frame.rotate_to_odom(ve, vn)
        vz = max(-cfg.follow_vz_max_mps, min(cfg.follow_vz_max_mps, cfg.follow_z_gain * (z_ref - pose.z)))
        yaw_rate = max(-self.follower_config.yaw_rate_max_rps,
                       min(self.follower_config.yaw_rate_max_rps, 2.0 * wrap(self._yaw_cmd - pose.yaw)))
        fallback = None
        if t - self._fallback_t >= cfg.fallback_pose_period_s:
            self._fallback_t = t
            fallback = (xy[0], xy[1], z_ref, self._yaw_cmd)
        self.reason = ("climbing in place by velocity: the path ahead needs more height" if wait
                       else "following by velocity" + (", descending" if z_ref < self.z_fly else ""))
        return Command("twist", vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, fallback=fallback)

    def _held(self, t: float, pose: Optional[OdomPose]) -> Command:
        """HELD for the pilot: brake by zero velocity, then hold the pose where the drone stopped.

        Holding the pose where the abort fired, at speed, makes PX4 overshoot it and fly back."""
        cfg = self.config
        if self._hold_cmd is not None:
            return self._hold_cmd
        moving = pose is None or math.hypot(pose.vx, pose.vy) > cfg.held_brake_speed_mps
        if moving and t - self._held_since < cfg.held_brake_s:
            return Command("twist")
        last = pose or self._last_good
        if last is None:
            return Command("twist")
        age = min(max(t - last.t, 0.0), cfg.max_prediction_s)
        self._hold_cmd = Command("pose", last.x + last.vx * age, last.y + last.vy * age, self.z_fly, last.yaw)
        return self._hold_cmd

    def _hold(self, t: float, pose: OdomPose, landed: Optional[bool]) -> Command:
        # at the end of the path's height: the flight height, or the early descent's floor
        z_end = self.z_target(self.plan.trajectory.duration)
        cmd = Command("pose", self.end_odom[0], self.end_odom[1], z_end, self._yaw_cmd)
        if not self.config.land:
            return cmd
        near = math.hypot(self.end_odom[0] - pose.x, self.end_odom[1] - pose.y) < self.config.land_tolerance_m
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
        if landed:
            self.state = LANDED
            return self._landed(t, pose, landed)
        self.reason = "landing"
        return self._descend(pose, self.config.descent_mps)

    def _landed(self, t: float, pose: Optional[OdomPose], landed: Optional[bool]) -> Command:
        self.reason = "landed: pressing down until PX4 disarms (COM_DISARM_LAND)"
        return self._descend(pose, 0.3 * self.config.descent_mps)

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
        vx = cfg.land_xy_gain * (self.end_odom[0] - pose.x)
        vy = cfg.land_xy_gain * (self.end_odom[1] - pose.y)
        speed = math.hypot(vx, vy)
        if speed > cfg.land_xy_max_mps:
            vx, vy = vx * cfg.land_xy_max_mps / speed, vy * cfg.land_xy_max_mps / speed
        return Command("twist", vx=vx, vy=vy, vz=-rate)

    # -- dropouts ----------------------------------------------------------------------

    def _dropout(self, t: float) -> Command:
        """No odometry to fly on in FOLLOW: coast on along the trajectory, then hold still."""
        cfg = self.config
        if self._dr is None:
            if cfg.coast_max_s <= 0.0 or self._last_good is None or self._frame_lost is not None:
                return self._hold_still(t, "no fresh pose")
            self._dr = _DeadReckoning(self._last_good)
            self._dr.since = t
            self.coasts += 1
        dr = self._dr
        dr.step(t, cfg.coast_response_s, self._measured_velocity(t))
        coasted = t - dr.since
        if coasted < cfg.coast_max_s:
            here = self.frame.to_course(dr.x, dr.y)
            sp = self.follower.update(t, here, self.frame.rotate_to_course(dr.vx, dr.vy))
            self.last_setpoint = sp
            if not sp.done:
                ref = self.plan.trajectory.at(sp.ref_time)
                ahead = self.plan.trajectory.at(sp.ref_time + sp.clock_rate * cfg.follow_anticipation_s)
                ff = 0.0 if "no_safe_target" in sp.flags else sp.clock_rate
                ve = ff * ahead.velocity[0] + cfg.coast_gain * (ref.position[0] - here[0])
                vn = ff * ahead.velocity[1] + cfg.coast_gain * (ref.position[1] - here[1])
                speed = math.hypot(ve, vn)
                cap = min(self.limits.v_max_mps, cfg.twist_speed_max_mps)
                if speed > cap:
                    ve, vn = ve * cap / speed, vn * cap / speed
                vx, vy = self.frame.rotate_to_odom(ve, vn)
                dr.cmd = (vx, vy)
                self._still_since = None
                self.reason = (f"no fresh pose: coasting on the trajectory by dead reckoning "
                               f"({coasted:.1f} s, {dr.travelled:.2f} m)")
                return Command("twist", vx=vx, vy=vy)
        dr.cmd = (0.0, 0.0)
        return self._hold_still(t, f"no fresh pose; coasted {dr.travelled:.2f} m")

    def _measured_velocity(self, t: float) -> Optional[Tuple[float, float]]:
        """PX4's velocity estimate in the odom frame, if fresh: its (forward, left) turned by the
        heading of the last odometry flown on (the yaw changes little in a few seconds)."""
        sample = self._px4_velocity
        if sample is None or self._last_good is None or not (0.0 <= t - sample[0] <= self.config.px4_velocity_timeout_s):
            return None
        _t, forward, left = sample
        if not (math.isfinite(forward) and math.isfinite(left)):
            return None
        c, s = math.cos(self._last_good.yaw), math.sin(self._last_good.yaw)
        return c * forward - s * left, s * forward + c * left

    def _hold_still(self, t: float, why: str) -> Command:
        """Zero velocity, refreshed: it needs no frame, and it never expires into an old pose."""
        if self._still_since is None:
            self._still_since = t
            self.holds += 1
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
            "holds": self.holds,
            "coasts": self.coasts,
            "offset_m": [round(self.offset[0], 3), round(self.offset[1], 3)],
        }
