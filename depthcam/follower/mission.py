"""The follower's mission: wait for Offboard, check the start, take off, follow, hold over the pad, land.

It flies the true pose (``/pose_fusion/odometry``) in the true (tracker) frame. Its setpoints go to
``pose_fusion``, which corrects them into PX4's frame. It obeys the kit's procedure
(``drone-snapshot/.../docs/competitor_guide.md`` §3-§4):

* the pilot's arm-then-Position is the start signal, and **it takes off itself**. The kit asks
  PX4 for Offboard once per arming, at that switch, and only if a team setpoint is under 1 s old
  (``auto_offboard.py``; the kit's own example streams one while it waits for the same reason).
  So before Offboard it sends the takeoff pose, but only while the pre-takeoff checks below pass.
  A drone that fails them gets no stream, the kit refuses Offboard, and the vehicle stays on the
  ground under the pilot (PX4 disarms it after ``COM_DISARM_PRFLT``). Nothing else is sent before
  ``/skybeta/state`` says ``offboard_active``;
* if Offboard ends, for any reason, it stops for good. It never tries to get Offboard back, and
  it never arms, disarms or changes mode;
* landing uses a velocity setpoint. The kit floors pose setpoints at ``min_altitude_m`` but only
  speed-caps velocity setpoints ("NOT a landing inhibit", its ``envelope.py``). Once landed, PX4
  disarms itself after ``COM_DISARM_LAND`` (2 s in the organisers' parameters).

Pre-takeoff checks (``PRECHECK``)
---------------------------------
cuVSLAM's origin is wherever the organisers' stack started, at power-on unless it was restarted.
The route assumes that origin is the launch mark, facing north. So these checks gate the
takeoff-pose stream before Offboard (above), and run again once Offboard is active, for at most
``start_timeout_s``:

* the true pose must be within ``start_tolerance_m`` of the route's start, at the planned heading
  within ``start_yaw_tolerance_rad``. If the drone was carried after the stack started, this fails
  without the lidar;
* when ``require_ready``, ``pose_fusion`` must report ready (the lidar correction has settled; it
  is always ready when the lidar is forbidden).

Offboard can start only on a takeoff pose streamed while the checks passed, so the second run
normally passes on its first tick. It can fail only if the checks stopped passing in the second
before the switch. If it still fails at the timeout, it goes to ``BLOCKED`` and sends nothing: the
interface holds the last takeoff pose it was sent, and the pilot takes over.

Holding still (tracking breaks, frame breaks)
---------------------------------------------
In ``TAKEOFF``, ``FOLLOW``, ``HOLD`` and ``LAND`` the follower **holds still with a zero
velocity setpoint**, and makes no route progress, while the pose can't be flown on:

* **No fresh pose:** none has arrived for ``pose_gap_s`` (0.15 s, about 4 frames), or the
  newest was captured more than ``pose_timeout_s`` ago (it waited somewhere on the way).
* **The Guard is withholding** (``tracking`` false): from the first withheld frame, before the
  pose stream even goes quiet.
* **The frames disagree** (``aligned`` false, when ``require_ready``): cuVSLAM re-initialised,
  its frame jumped, and pose_fusion hasn't re-learnt its correction yet
  (``pose_fusion.fusion.PoseFusion.aligned``).

A zero velocity doesn't depend on any frame. That matters because PX4 gets cuVSLAM's poses from
the organisers' bridge, without our Guard's settling time. So when cuVSLAM resumes in a jumped
frame, PX4's estimate jumps before our chain sees the new pose. A held pose setpoint would then
point PX4 at the wrong place, but a zero velocity keeps the drone where it is.

When the pose can be flown on again, it carries on from the corrected pose. A carrot taken off
the path steers straight back to it. If that takes longer than ``realign_timeout_s``, it gives up
the route and goes to ``LAND_HERE``. That is a straight-down velocity descent, ignoring the pose.
Under the forbidden policy ``aligned`` is always true: without the lidar, a frame jump can't be
measured, only held through.

Following
---------
* **The heading is held:** the route's one planned heading, all the way (``Route.yaw``). The
  planner grew every obstacle by the drone's footprint at that heading, so turning the nose
  would break its clearances.
* **The carrot:** it aims at the furthest path point, up to ``lookahead_max_m`` ahead of the
  drone's projection, such that every path point in between lies within ``max_cut_m`` of the
  straight line from the drone to it. Flying straight at the carrot therefore never leaves the
  path by more than ``max_cut_m``. At a sharp corner the carrot comes close and the drone slows;
  on a straight it runs ahead.
* **Progress only goes forward:** the projection is searched only a short way ahead of the
  current point.
* **Checkpoints:** each gate index the projection passes is recorded as crossed.

Height: ``cruise_z_m`` is above the floor, and the tracker frame's z starts at the body resting on
the floor, so the setpoint z is ``cruise_z_m - rest_height_m``.

Not done: the optional return flight, and the planner's speed profile (PX4 sets the pace here).

numpy only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import List, Optional, Tuple

import numpy as np

from .route import Route

WAIT_FOR_OFFBOARD = "WAIT_FOR_OFFBOARD"
PRECHECK = "PRECHECK"
BLOCKED = "BLOCKED"
TAKEOFF = "TAKEOFF"
FOLLOW = "FOLLOW"
HOLD = "HOLD"
LAND = "LAND"
LAND_HERE = "LAND_HERE"
LANDED = "LANDED"
STOPPED = "STOPPED"

#: Flight states: the follower holds still in them while the pose can't be flown on.
FLIGHT_STATES = (TAKEOFF, FOLLOW, HOLD, LAND)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class FollowerConfig:
    """``rest_height_m`` is measured on the drone and has no default (contracts/frames.md: 0.07 m)."""

    rest_height_m: float
    lookahead_max_m: float = 0.5
    max_cut_m: float = 0.015
    search_ahead_m: float = 0.6
    capture_radius_m: float = 0.10
    takeoff_tolerance_m: float = 0.10
    start_tolerance_m: float = 0.25
    start_yaw_tolerance_rad: float = math.radians(10.0)
    start_timeout_s: float = 20.0
    require_ready: bool = True
    max_yaw_rate_rps: float = math.radians(45.0)
    hold_before_land_s: float = 2.0
    land_tolerance_m: float = 0.10
    descent_mps: float = 0.3
    land_xy_gain: float = 1.0                 # 1/s: horizontal correction over the pad while descending
    land_xy_max_mps: float = 0.2
    pose_timeout_s: float = 0.5               # a pose captured longer ago than this is stale
    pose_gap_s: float = 0.15                  # no pose arriving for this long: the stream stopped
    realign_timeout_s: float = 5.0
    land: bool = True

    def __post_init__(self) -> None:
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool):
                continue
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value >= 0.0):
                raise ValueError("FollowerConfig.{} must be a finite number >= 0 (got {!r})".format(f.name, value))
        for name in ("lookahead_max_m", "max_cut_m", "search_ahead_m", "capture_radius_m"):
            if getattr(self, name) <= 0.0:
                raise ValueError("FollowerConfig.{} must be > 0".format(name))


@dataclass(frozen=True)
class Pose:
    """``t``: when it was captured (its header stamp, never later than its arrival).
    ``received``: when it arrived, on the follower's clock (None: take ``t``)."""

    t: float
    x: float
    y: float
    z: float
    yaw: float
    received: Optional[float] = None


@dataclass(frozen=True)
class Command:
    """A pose (``yaw`` None = not commanded) or a twist setpoint, in the true frame."""

    kind: str                                 # "pose" | "twist"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: Optional[float] = None
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class Follower:
    """One run. Call :meth:`update` on a timer; publish what it returns (``None`` = send nothing)."""

    def __init__(self, route: Route, config: FollowerConfig) -> None:
        if len(route.path) < 2:
            raise ValueError("the route needs at least 2 points")
        self.route = route
        self.config = config
        self.path = np.asarray(route.path, dtype=float)
        seg = np.hypot(*np.diff(self.path, axis=0).T)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])          # arc length at each point
        self.z_fly = route.cruise_z_m - config.rest_height_m
        if self.z_fly <= 0.0:
            raise ValueError("cruise height {:.2f} m is not above the rest height {:.2f} m".format(
                route.cruise_z_m, config.rest_height_m))
        self.state = WAIT_FOR_OFFBOARD
        self.index = 0                                            # the projection's path point
        self.reason = ""
        self.crossed: List[Tuple[str, float]] = []                # (gate name, time) in order
        self._was_offboard = False
        self._precheck_since: Optional[float] = None
        self._takeoff_xy = (0.0, 0.0)
        self._yaw_cmd: Optional[float] = None
        self._last_t: Optional[float] = None
        self._hold_since: Optional[float] = None
        self._still_since: Optional[float] = None
        self.holds = 0                                            # times it held still in flight, this run

    # ------------------------------------------------------------------ the tick

    def update(self, t: float, offboard_active: bool, pose: Optional[Pose], landed: Optional[bool] = None,
               ready: bool = True, aligned: bool = True, tracking: bool = True) -> Optional[Command]:
        """One tick. *ready*: pose_fusion has settled (before takeoff). *aligned*: its correction
        can be flown on (in flight). *tracking*: the Guard is passing poses. *landed*: PX4's land
        detector, None if unknown or stale."""
        if self.state in (STOPPED, BLOCKED):
            return None
        if self._was_offboard and not offboard_active:
            self.state = STOPPED
            self.reason = "Offboard ended: the pilot has the vehicle"
            return None
        if not offboard_active:
            return self._before_offboard(t, pose, ready)
        self._was_offboard = True
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        if self.state == WAIT_FOR_OFFBOARD:
            self.state = PRECHECK
            self._precheck_since = t
        if self.state == LAND_HERE:                     # straight down: it needs no pose
            return self._land_here(t, landed)
        fresh = self.fresh(t, pose)
        if self.state in FLIGHT_STATES:
            if not fresh:
                return self._hold_still(t, "no fresh pose")
            if not tracking:
                return self._hold_still(t, "the Guard is withholding cuVSLAM's poses")
            if self.config.require_ready and not aligned:
                return self._hold_still(t, "pose_fusion is realigning its correction")
        if not fresh:
            self.reason = "no fresh pose: sending nothing"
            if self.state == PRECHECK:
                return self._precheck_timeout(t, "no fresh pose")
            return None                                 # LANDED: PX4 disarms on its own
        if self._still_since is not None:
            self._still_since = None
            self.reason = ""
        handler = {PRECHECK: self._precheck, TAKEOFF: self._takeoff, FOLLOW: self._follow, HOLD: self._hold,
                   LAND: self._land, LANDED: self._landed}[self.state]
        return handler(t, dt, pose, landed, ready)

    # ------------------------------------------------------------------ states

    def _before_offboard(self, t: float, pose: Optional[Pose], ready: bool) -> Optional[Command]:
        """The takeoff pose while the start checks pass; nothing otherwise.

        The kit asks PX4 for Offboard once per arming, at the pilot's switch to Position, and only if
        a team setpoint is under its ``setpoint_timeout_s`` (1 s) old (``auto_offboard.py``). So this
        stream is what lets Offboard start at all, and gating it on the checks means a drone that
        fails them is refused Offboard and stays on the ground under the pilot. The pose is the one
        ``_takeoff`` sends on its first tick (here, at the planned height, nose as it is), so the
        vehicle sees no jump when Offboard starts.
        """
        if not self.fresh(t, pose):
            self.reason = "waiting for Offboard, sending nothing: no fresh pose"
            return None
        problems = self._start_problems(pose, ready)
        if problems:
            self.reason = "waiting for Offboard, sending nothing, so the kit will refuse it: " + "; ".join(problems)
            return None
        self.reason = "waiting for Offboard: on the start, sending the takeoff pose"
        return Command("pose", pose.x, pose.y, self.z_fly, pose.yaw)

    def _start_problems(self, pose: Pose, ready: bool) -> List[str]:
        """Why the drone may not take off from here (empty: it may)."""
        cfg = self.config
        sx, sy = self.route.start
        off = math.hypot(pose.x - sx, pose.y - sy)
        yaw_off = abs(_wrap(pose.yaw - self.route.yaw))
        problems = []
        if off > cfg.start_tolerance_m:
            problems.append("{:.2f} m from the route start (> {:.2f}): restart the stack on the launch mark".format(
                off, cfg.start_tolerance_m))
        if yaw_off > cfg.start_yaw_tolerance_rad:
            problems.append("heading {:.0f} deg off the planned {:.0f} deg".format(
                math.degrees(yaw_off), math.degrees(self.route.yaw)))
        if cfg.require_ready and not ready:
            problems.append("pose_fusion is not ready (the lidar correction has not settled)")
        return problems

    def _precheck(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Optional[Command]:
        problems = self._start_problems(pose, ready)
        if problems:
            return self._precheck_timeout(t, "; ".join(problems))
        self.state = TAKEOFF
        self.reason = ""
        self._takeoff_xy = (pose.x, pose.y)
        self._yaw_cmd = pose.yaw
        return self._takeoff(t, dt, pose, landed, ready)

    def _precheck_timeout(self, t: float, why: str) -> None:
        self.reason = "pre-takeoff check: " + why
        if self._precheck_since is not None and t - self._precheck_since >= self.config.start_timeout_s:
            self.state = BLOCKED
            self.reason = "blocked: " + why
        return None

    def _takeoff(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Command:
        self._turn_toward(self.route.yaw, dt)
        if abs(pose.z - self.z_fly) < self.config.takeoff_tolerance_m:
            self.state = FOLLOW
            self.index = 0
            return self._follow(t, dt, pose, landed, ready)
        return Command("pose", self._takeoff_xy[0], self._takeoff_xy[1], self.z_fly, self._yaw_cmd)

    def _follow(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Command:
        cfg = self.config
        self._advance(pose, t)
        gx, gy = self.route.goal
        if self.index >= len(self.path) - 1 or (self.s[-1] - self.s[self.index] < cfg.capture_radius_m
                                                 and math.hypot(gx - pose.x, gy - pose.y) < cfg.capture_radius_m):
            self.state = HOLD
            self._hold_since = None
            return self._hold(t, dt, pose, landed, ready)
        cx, cy = self.carrot(pose)
        self._turn_toward(self.route.yaw, dt)
        return Command("pose", cx, cy, self.z_fly, self._yaw_cmd)

    def _hold(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Command:
        gx, gy = self.route.goal
        self._turn_toward(self.route.yaw, dt)
        cmd = Command("pose", gx, gy, self.z_fly, self._yaw_cmd)
        if not self.config.land:
            return cmd
        if math.hypot(gx - pose.x, gy - pose.y) >= self.config.land_tolerance_m:
            self._hold_since = None
            return cmd
        if self._hold_since is None:
            self._hold_since = t
        if t - self._hold_since >= self.config.hold_before_land_s:
            self.state = LAND
            return self._land(t, dt, pose, landed, ready)
        return cmd

    def _land(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Command:
        if landed:
            self.state = LANDED
            return self._landed(t, dt, pose, landed, ready)
        return self._descend(pose, self.config.descent_mps)

    def _landed(self, t: float, dt: float, pose: Pose, landed: Optional[bool], ready: bool) -> Command:
        # Keep pressing down gently: PX4 disarms itself (COM_DISARM_LAND), which ends Offboard -> STOPPED
        return self._descend(pose, 0.3 * self.config.descent_mps)

    def fresh(self, t: float, pose: Optional[Pose]) -> bool:
        """Whether *pose* can be flown on at *t*: captured under ``pose_timeout_s`` ago, and the
        stream still arriving (the newest under ``pose_gap_s`` old)."""
        if pose is None or t - pose.t > self.config.pose_timeout_s:
            return False
        return pose.received is None or t - pose.received <= self.config.pose_gap_s

    def _hold_still(self, t: float, why: str) -> Command:
        """Hold still by velocity, which needs no frame; land here if it lasts too long."""
        cfg = self.config
        if self._still_since is None:
            self._still_since = t
            self.holds += 1
        self._hold_since = None                         # the hold over the pad starts again afterwards
        waited = t - self._still_since
        if waited >= cfg.realign_timeout_s:
            self.state = LAND_HERE
            self.reason = "held still {:g} s ({}): landing here, straight down".format(cfg.realign_timeout_s, why)
            return self._land_here(t, None)
        self.reason = "holding still: {} ({:.1f} s)".format(why, waited)
        return Command("twist")

    def _land_here(self, t: float, landed: Optional[bool]) -> Command:
        """Descend straight down, with no horizontal command: the pose can't be trusted."""
        if landed:
            return Command("twist", vz=-0.3 * self.config.descent_mps)
        return Command("twist", vz=-self.config.descent_mps)

    # ------------------------------------------------------------------ helpers

    def _advance(self, pose: Pose, t: float) -> None:
        """Move the projection forward to the nearest path point within ``search_ahead_m``; record gates."""
        i0 = self.index
        i1 = int(np.searchsorted(self.s, self.s[i0] + self.config.search_ahead_m, side="right"))
        window = self.path[i0:max(i1, i0 + 1)]
        d = np.hypot(window[:, 0] - pose.x, window[:, 1] - pose.y)
        new = i0 + int(np.argmin(d))
        if new > self.index:
            for g in self.route.gates:
                if self.index < g.index <= new:
                    self.crossed.append((g.name, t))
            self.index = new

    def carrot(self, pose: Pose) -> Tuple[float, float]:
        """The furthest path point whose straight line from the drone keeps the path within ``max_cut_m``."""
        cfg = self.config
        i = self.index
        last = int(np.searchsorted(self.s, self.s[i] + cfg.lookahead_max_m, side="right")) - 1
        last = min(max(last, i + 1), len(self.path) - 1)
        p0 = np.array([pose.x, pose.y])
        best = min(i + 1, len(self.path) - 1)
        for j in range(i + 1, last + 1):
            chord = self.path[j] - p0
            length = float(np.hypot(*chord))
            between = self.path[i:j + 1]
            if length < 1e-9:
                best = j
                continue
            off = np.abs((between[:, 0] - p0[0]) * chord[1] - (between[:, 1] - p0[1]) * chord[0]) / length
            if float(off.max()) > cfg.max_cut_m:
                break
            best = j
        return float(self.path[best, 0]), float(self.path[best, 1])

    def _descend(self, pose: Pose, rate: float) -> Command:
        gx, gy = self.route.goal
        cfg = self.config
        vx, vy = cfg.land_xy_gain * (gx - pose.x), cfg.land_xy_gain * (gy - pose.y)
        speed = math.hypot(vx, vy)
        if speed > cfg.land_xy_max_mps:
            vx, vy = vx * cfg.land_xy_max_mps / speed, vy * cfg.land_xy_max_mps / speed
        return Command("twist", vx=vx, vy=vy, vz=-rate, yaw_rate=0.0)

    def _turn_toward(self, target: float, dt: float) -> None:
        if self._yaw_cmd is None:
            self._yaw_cmd = target
            return
        step = _wrap(target - self._yaw_cmd)
        limit = self.config.max_yaw_rate_rps * dt
        self._yaw_cmd = _wrap(self._yaw_cmd + max(-limit, min(limit, step)))
