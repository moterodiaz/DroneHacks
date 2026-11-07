"""Fly a trajectory through the kit's position-only interface (the Follower).

``skybeta_flight_interface`` sends PX4 a position *or* a velocity, never
both, so there is no way to pass the trajectory's velocity and acceleration
as feedforward. PX4's controller is cascaded P (``MPC_XY_P`` = 0.95) into a
velocity loop (``MPC_XY_VEL_P_ACC`` = 1.8).

By default the Follower publishes the trajectory's plain poses.
``contracts/ros-graph.md`` marks the lead formula below as withdrawn ("not an
alternative implementation path"), so it is off by default and stays
available only as an explicit opt-in (``lead=True``) for offline research:

    led_pose = pose + v / 0.95 + a / (0.95 * 1.8)

Guards that always apply, with or without the lead:

* **No unsafe target.** The interface keeps resending the last pose if this
  node dies, and PX4 then flies straight to it. So the straight line from
  the drone to the pose actually published must be clear. If it is not -
  the drone has drifted round an inside corner and every approach is
  blocked - the reference is not commanded: the Follower freezes the
  reference clock, holds where the drone is, and flags ``no_safe_target``
  so the caller can react.
  With the lead on, the offset is also cut back until the lines from the
  reference, and from the estimated position, to the led pose are clear,
  and capped at ``lead_max_m``.
* **Progress governor.** The reference clock slows when the drone falls
  behind by more than ``lag_ok_m``, and stops at ``lag_stop_m``. The
  reference never runs away from the vehicle.
* **Yaw.** The trajectory's look-ahead heading, turned at no more than
  ``yaw_rate_max_rps``.

Everything is in course-frame ``(east, north)``. Converting to the ``odom``
frame the interface expects is ``contracts/frames.md``'s transform, done in
the ROS node, which is not written yet.
"""

from __future__ import annotations

import dataclasses
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .clearance import ObstacleField
from .limits import VehicleLimits
from .trajectory import Trajectory, wrap


@dataclasses.dataclass(frozen=True)
class FollowerConfig:
    #: Offset poses ahead of the reference. False (the default): publish the
    #: reference pose itself. True: the lead formula, marked withdrawn in
    #: contracts/ros-graph.md - an explicit opt-in for offline research only.
    lead: bool = False
    #: "formula": contracts/ros-graph.md, pose + v/kp + a/(kp*kv), off along
    #: the tangent and the curve's inside. "path": the trajectory's own
    #: position 1/kp seconds ahead, which Taylor-expands to pose + v/kp +
    #: a/(2 kp^2). That matches the formula to second order (acceleration
    #: coefficient 0.554 vs 0.585 s^2), but stays on the path, so the line to
    #: it is a chord of a verified path instead of a line into the wall. A
    #: contract change is needed before "path" becomes the default.
    lead_mode: str = "formula"
    #: Include the acceleration term of the formula lead.
    accel_feedforward: bool = True
    #: Longest lead. 2.5 m covers 2 m/s / 0.95 plus an acceleration term. Unvalidated.
    lead_max_m: float = 2.5
    #: Governor: full speed while the drone is at most this far behind the reference...
    lag_ok_m: float = 0.15
    #: ...and the reference stops when it is this far behind.
    lag_stop_m: float = 0.50
    #: Flag (not act on) cross-track error above this. The 0.93 m passage leaves +-0.18 m.
    cross_track_warn_m: float = 0.10
    yaw_rate_max_rps: float = 1.0
    #: Done when the reference has finished and the drone is this close to the end.
    done_radius_m: float = 0.10
    #: How far back and ahead of the last match to look when projecting the drone onto the path.
    projection_back_m: float = 0.3
    projection_ahead_m: float = 1.5
    #: Odometry age. When velocity is given, the position is predicted forward by this
    #: much. The organisers measured a 68.7 ms median vision latency.
    odom_latency_s: float = 0.0687
    #: The interface holds each pose for one 10 Hz tick, half a tick late on
    #: average, so the lead looks this much further ahead.
    hold_compensation_s: float = 0.05


@dataclasses.dataclass(frozen=True)
class Setpoint:
    position: np.ndarray     # the pose to publish (course frame)
    yaw: float
    reference: np.ndarray    # where the trajectory says the drone should be now
    ref_time: float
    lead_m: float            # length of the offset actually applied
    lead_scale: float        # fraction of the wanted lead allowed by the clearance check
    lag_m: float             # how far behind the reference the drone is, along the path
    cross_track_m: float
    clock_rate: float        # governor: 1 = real time, 0 = reference stopped
    done: bool
    flags: Tuple[str, ...] = ()


class Follower:
    def __init__(self, traj: Trajectory, field: ObstacleField, limits: VehicleLimits = VehicleLimits(),
                 cfg: FollowerConfig = FollowerConfig()):
        self.traj = traj
        self.field = field
        self.limits = limits
        self.cfg = cfg
        self._now: Optional[float] = None
        self._tau = 0.0
        self._idx = 0
        self._yaw = float(traj.yaw[0])
        ds = float(np.mean(np.diff(traj.s))) if len(traj.s) > 1 else 0.01
        self._back = max(1, int(cfg.projection_back_m / ds))
        self._ahead = max(2, int(cfg.projection_ahead_m / ds))

    def pause(self, now: float) -> None:
        """Tracking is lost, so the drone is holding: this time must not count as flight. Without
        this the next update advances the reference by the whole gap and commands a far pose."""
        self._now = now

    def update(self, now: float, est_pos: Sequence[float], est_vel: Optional[Sequence[float]] = None) -> Setpoint:
        """``est_pos`` / ``est_vel``: the latest odometry, course frame. With a
        velocity, the position is moved forward by ``odom_latency_s``."""
        cfg, traj = self.cfg, self.traj
        est = np.asarray(est_pos, dtype=float)
        if est_vel is not None:
            est = est + np.asarray(est_vel, dtype=float) * cfg.odom_latency_s
        dt = 0.0 if self._now is None else max(0.0, now - self._now)
        self._now = now

        s_proj, cross = self._project(est)
        lag = traj.at(self._tau).s - s_proj
        if lag <= cfg.lag_ok_m:
            rate = 1.0
        else:
            rate = max(0.0, (cfg.lag_stop_m - lag) / (cfg.lag_stop_m - cfg.lag_ok_m))
        tau_prev = self._tau
        self._tau = min(self._tau + rate * dt, traj.duration)
        ref = traj.at(self._tau)

        flags: List[str] = []
        offset = np.zeros(2)
        if cfg.lead and self._tau < traj.duration:
            if cfg.lead_mode == "path":
                ahead = rate * (1.0 / self.limits.kp_pos + cfg.hold_compensation_s)
                offset = traj.at(self._tau + ahead).position - ref.position
            else:
                offset = rate * ref.velocity * (1.0 / self.limits.kp_pos + cfg.hold_compensation_s)
                if cfg.accel_feedforward:
                    offset = offset + rate * rate * ref.acceleration / (self.limits.kp_pos * self.limits.kp_vel)
            n = float(np.hypot(*offset))
            if n > cfg.lead_max_m:
                offset *= cfg.lead_max_m / n
                flags.append("lead_capped")
        scale = self._clear_fraction(ref.position, offset, est)
        if scale < 1.0:
            flags.append("lead_clipped")
        position = ref.position + scale * offset

        # The drone, not just the reference, must be able to reach the pose we
        # publish: the interface keeps resending it and PX4 flies straight to
        # it. _clear_fraction already checks est -> led pose when it can, but
        # not when the offset is zero or every lead it tested was blocked - and
        # then position collapses to the reference, which may be straight
        # through a wall from where the drone actually is. If that line is
        # blocked, do not command the reference: freeze the reference clock,
        # hold where the drone is (the only pose trivially reachable from it),
        # and flag it so the caller can react.
        if (self.field.query(est)[1][0] >= 0.0
                and self.field.segment_query(tuple(est), tuple(position))[1] < 0.0):
            self._tau = tau_prev
            ref = traj.at(self._tau)
            position = est.copy()
            flags.append("no_safe_target")

        step = cfg.yaw_rate_max_rps * dt if dt > 0 else math.pi
        self._yaw = wrap(self._yaw + max(-step, min(step, wrap(ref.yaw - self._yaw))))
        if cross > cfg.cross_track_warn_m:
            flags.append("cross_track")
        if rate < 1e-3:
            flags.append("held")
        done = self._tau >= traj.duration and float(np.hypot(*(est - traj.xy[-1]))) <= cfg.done_radius_m
        return Setpoint(position=position, yaw=self._yaw, reference=ref.position, ref_time=self._tau,
                        lead_m=float(np.hypot(*(scale * offset))), lead_scale=scale, lag_m=lag,
                        cross_track_m=cross, clock_rate=rate, done=done, flags=tuple(flags))

    def _project(self, est: np.ndarray) -> Tuple[float, float]:
        lo = max(0, self._idx - self._back)
        hi = min(len(self.traj.s), self._idx + self._ahead)
        d = np.hypot(*(self.traj.xy[lo:hi] - est).T)
        k = int(np.argmin(d))
        self._idx = lo + k
        return float(self.traj.s[self._idx]), float(d[k])

    def _clear_fraction(self, ref: np.ndarray, offset: np.ndarray, est: np.ndarray) -> float:
        """Largest fraction of ``offset`` whose straight-line approaches are clear."""
        if not np.any(offset):
            return 1.0
        check_est = self.field.query(est)[1][0] >= 0.0

        def ok(f: float) -> bool:
            target = tuple(ref + f * offset)
            if self.field.segment_query(tuple(ref), target)[1] < 0.0:
                return False
            return not check_est or self.field.segment_query(tuple(est), target)[1] >= 0.0

        if ok(1.0):
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(8):
            mid = (lo + hi) / 2.0
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo


class WaypointFollower:
    """Baseline A: the kit example's approach (``competitor_example/square_mission.py``).
    Publish one waypoint as a pose, advance when within ``arrival_radius_m``
    (the example's 0.3 m), finish within ``done_radius_m`` of the last."""

    def __init__(self, waypoints: Sequence[Sequence[float]], arrival_radius_m: float = 0.3,
                 done_radius_m: float = 0.10):
        self.wps = np.asarray(waypoints, dtype=float)
        self.arrival = arrival_radius_m
        self.done_radius = done_radius_m
        self.i = 1 if len(self.wps) > 1 else 0
        self._yaw = 0.0

    def update(self, now: float, est_pos: Sequence[float], est_vel: Optional[Sequence[float]] = None) -> Setpoint:
        est = np.asarray(est_pos, dtype=float)
        last = len(self.wps) - 1
        while self.i < last and float(np.hypot(*(est - self.wps[self.i]))) <= self.arrival:
            self.i += 1
        target = self.wps[self.i]
        d = target - est
        if np.hypot(*d) > 0.05:
            self._yaw = math.atan2(d[1], d[0])
        done = self.i == last and float(np.hypot(*d)) <= self.done_radius
        return Setpoint(position=target.copy(), yaw=self._yaw, reference=target.copy(), ref_time=now,
                        lead_m=0.0, lead_scale=1.0, lag_m=0.0, cross_track_m=0.0, clock_rate=1.0, done=done)
