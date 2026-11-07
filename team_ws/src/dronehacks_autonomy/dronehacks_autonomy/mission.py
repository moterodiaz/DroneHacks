"""The flight, arming to disarm: wait, take off, follow, hold over the pad, land.

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

    # -- the tick ------------------------------------------------------------------

    def update(self, t: float, offboard_active: bool, pose: Optional[OdomPose],
               landed: Optional[bool] = None, px4_yaw: Optional[float] = None) -> Optional[Command]:
        """``px4_yaw``: PX4's heading as an ENU yaw (None if unknown), for the heading gate."""
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
            return self._hold_cmd
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
        cmd = Command("pose", self.end_odom[0], self.end_odom[1], self.z_fly, self._yaw_cmd)
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
        }
