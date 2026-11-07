"""A whole flight, offline: the mission, the kit's interface and a stand-in PX4, from arming to disarm.

It extends ``dronehacks_planner.sim``, which models horizontal flight only,
with what a full mission needs. Every number is a model output, not a
measurement; the parameter values are drone 1's
(``fleet/SN0001-2026-10-15.params``) unless marked GUESSED.

* **The kit's interface** (``skybeta_flight_interface``):
  * Pose setpoints persist and are floored at ``min_altitude_m``, 0.3 m.
  * Twists expire after ``setpoint_timeout_s`` (1 s); after that it holds
    the last pose.
  * It forwards at 10 Hz.
* **Offboard**, by its ``auto_offboard.py`` rule: at the pilot's switch to
  Position (``switch_s`` after arming), Offboard starts only on the ground,
  and only if a team setpoint is under 1 s old. Otherwise the vehicle stays
  with the pilot. Offboard ends at disarm.
* **PX4**:
  * Horizontal: P position, then PID velocity (``MPC_XY_P``,
    ``MPC_XY_VEL_*_ACC``), capped at ``MPC_XY_VEL_MAX`` and the tilt limit.
  * Vertical: P (``MPC_Z_P`` = 1) into P (``MPC_Z_VEL_P_ACC`` = 4), between
    ``MPC_Z_VEL_MAX_DN`` (0.7) and ``MPC_Z_VEL_MAX_UP`` (1.0).
  * Yaw: P (``MC_YAW_P`` = 2.8), capped at 45 deg/s.
  * Acceleration reaches its setpoint with a 0.1 s lag (GUESSED).
* **Landing:**
  * Landed means within 2 cm of the floor and slower than 0.1 m/s for
    0.5 s (GUESSED).
  * PX4 disarms ``COM_DISARM_LAND`` (2 s) after landing, or
    ``COM_DISARM_PRFLT`` (10 s) after arming if it never took off.
* **Odometry** reaches the mission ``odom_latency_s`` (68.7 ms) late, with
  its capture stamp, and with world-frame velocity.
"""

from __future__ import annotations

import collections
import dataclasses
import math
from typing import Dict, List, Optional

import numpy as np

from dronehacks_planner.limits import VehicleLimits, tilt_accel_limit

from .frames import wrap
from .mission import Mission, OdomPose

_G = 9.80665


@dataclasses.dataclass(frozen=True)
class VehicleSimConfig:
    dt_s: float = 0.002
    px4_rate_hz: float = 100.0
    interface_rate_hz: float = 10.0
    mission_rate_hz: float = 50.0
    odom_latency_s: float = 0.0687
    accel_lag_s: float = 0.10            # GUESSED
    ki_vel: float = 0.4                  # MPC_XY_VEL_I_ACC
    kd_vel: float = 0.2                  # MPC_XY_VEL_D_ACC
    kp_z: float = 1.0                    # MPC_Z_P
    kv_z: float = 4.0                    # MPC_Z_VEL_P_ACC
    vz_max_up: float = 1.0               # MPC_Z_VEL_MAX_UP
    vz_max_dn: float = 0.7               # MPC_Z_VEL_MAX_DN
    yaw_p: float = 2.8                   # MC_YAW_P
    yaw_rate_max: float = math.radians(45.0)
    min_altitude_m: float = 0.3          # the interface's envelope floor
    setpoint_timeout_s: float = 1.0      # the interface's twist expiry, and the auto-Offboard freshness rule
    twist_speed_cap_mps: float = 1.5     # the interface's max_speed_mps
    switch_s: float = 2.0                # pilot arms at 0, switches to Position at this time
    disarm_land_s: float = 2.0           # COM_DISARM_LAND
    disarm_preflight_s: float = 10.0     # COM_DISARM_PRFLT
    timeout_s: float = 180.0
    record_rate_hz: float = 25.0
    #: Heading error PX4's EKF2 reports against vision's at the start (for the heading gate).
    px4_yaw_offset_rad: float = 0.0
    #: Odometry stops arriving between these times (None: never).
    dropout_s: Optional[tuple] = None
    #: After the dropout, cuVSLAM resumes from where it froze (stereo VO, no IMU fusion): its
    #: frame has jumped back by the motion it missed. PX4 follows the jump (the bridge bumps
    #: reset_counter), so its position control runs in the jumped frame too.
    freeze_on_dropout: bool = False
    #: The pilot takes over (Offboard ends) at this time (None: never).
    pilot_takeover_s: Optional[float] = None


@dataclasses.dataclass
class FlightLog:
    t: np.ndarray
    pos: np.ndarray            # (N, 3) odom frame
    yaw: np.ndarray
    state: List[str]
    events: Dict[str, float]   # name -> time
    offboard_refused: Optional[str]
    final_state: str
    final_reason: str
    reasons: List[str]         # every distinct reason the mission gave, in order


def fly(mission: Mission, limits: VehicleLimits = VehicleLimits(),
        cfg: VehicleSimConfig = VehicleSimConfig(), start=(0.0, 0.0, 0.0), start_yaw: float = 0.0) -> FlightLog:
    p = np.array(start, dtype=float)
    v = np.zeros(3)
    a = np.zeros(3)
    yaw = float(start_yaw)
    integ = np.zeros(2)
    a_max = tilt_accel_limit(limits)
    every = lambda hz: max(1, int(round(1.0 / (hz * cfg.dt_s))))  # noqa: E731
    n_mission, n_iface, n_px4, n_rec = (every(cfg.mission_rate_hz), every(cfg.interface_rate_hz),
                                        every(cfg.px4_rate_hz), every(cfg.record_rate_hz))
    history = collections.deque(maxlen=int(cfg.odom_latency_s / (n_mission * cfg.dt_s)) + 3)

    armed, offboard, landed = True, False, True
    landed_since: Optional[float] = None
    ever_airborne = False
    pose_target: Optional[np.ndarray] = None     # what the interface holds (x, y, z, yaw)
    twist_target: Optional[np.ndarray] = None
    twist_t = -1e9
    last_setpoint_t: Optional[float] = None
    fwd_mode, fwd_pos, fwd_vel = "none", None, None
    acc_sp = np.zeros(3)
    yaw_sp = yaw
    events: Dict[str, float] = {}
    refused: Optional[str] = None
    rec = {"t": [], "pos": [], "yaw": [], "state": []}
    reasons: List[str] = []

    frame_jump = np.zeros(2)                     # odom = physical - frame_jump (freeze_on_dropout)
    frozen_at: Optional[np.ndarray] = None
    steps = int(cfg.timeout_s / cfg.dt_s)
    for k in range(steps + 1):
        now = k * cfg.dt_s
        # -- the pilot ---------------------------------------------------------
        if armed and not offboard and "switch" not in events and now >= cfg.switch_s:
            events["switch"] = now
            fresh = last_setpoint_t is not None and now - last_setpoint_t <= cfg.setpoint_timeout_s
            if landed and fresh:
                offboard = True
                events["offboard"] = now
            else:
                refused = "in the air" if not landed else "no team setpoint under 1 s old"
        if offboard and cfg.pilot_takeover_s is not None and now >= cfg.pilot_takeover_s:
            offboard = False
            events["pilot_takeover"] = now

        # -- the mission ----------------------------------------------------------
        if k % n_mission == 0:
            history.append((now, p.copy(), v.copy(), yaw))
            dropped = cfg.dropout_s is not None and cfg.dropout_s[0] <= now <= cfg.dropout_s[1]
            if cfg.freeze_on_dropout and dropped and frozen_at is None:
                frozen_at = p[:2].copy()
            if cfg.freeze_on_dropout and not dropped and frozen_at is not None and not frame_jump.any():
                frame_jump = p[:2] - frozen_at
            pose = None if dropped else _delayed_pose(history, now - cfg.odom_latency_s)
            if pose is not None and frame_jump.any():
                pose = dataclasses.replace(pose, x=pose.x - float(frame_jump[0]), y=pose.y - float(frame_jump[1]))
            px4_yaw = wrap(yaw + cfg.px4_yaw_offset_rad)
            cmd = mission.update(now, offboard, pose, landed, px4_yaw)
            if not reasons or reasons[-1] != mission.reason:
                reasons.append(mission.reason)
            if cmd is not None:
                last_setpoint_t = now
                if cmd.kind == "pose":
                    z = max(cmd.z, cfg.min_altitude_m)
                    pose_target = np.array([cmd.x, cmd.y, z, yaw if cmd.yaw is None else cmd.yaw])
                    twist_target = None
                else:
                    vel = np.array([cmd.vx, cmd.vy, cmd.vz])
                    n = float(np.linalg.norm(vel))
                    if n > cfg.twist_speed_cap_mps:
                        vel *= cfg.twist_speed_cap_mps / n
                    twist_target, twist_t = vel, now

        # -- the interface forwards ----------------------------------------------
        if k % n_iface == 0:
            if twist_target is not None and now - twist_t <= cfg.setpoint_timeout_s:
                fwd_mode, fwd_vel = "velocity", twist_target.copy()
            elif pose_target is not None:
                fwd_mode, fwd_pos = "position", pose_target.copy()
            else:
                fwd_mode = "none"

        # -- PX4 ----------------------------------------------------------------------
        if k % n_px4 == 0:
            dt_c = n_px4 * cfg.dt_s
            if armed and offboard and fwd_mode != "none":
                if fwd_mode == "position":
                    vxy = limits.kp_pos * (fwd_pos[:2] - (p[:2] - frame_jump))
                    s = float(np.hypot(*vxy))
                    if s > limits.v_max_mps:
                        vxy *= limits.v_max_mps / s
                    vz = float(np.clip(cfg.kp_z * (fwd_pos[2] - p[2]), -cfg.vz_max_dn, cfg.vz_max_up))
                    yaw_sp = fwd_pos[3]
                else:
                    vxy = fwd_vel[:2].copy()
                    vz = float(np.clip(fwd_vel[2], -cfg.vz_max_dn, cfg.vz_max_up))
                err = vxy - v[:2]
                integ = np.clip(integ + cfg.ki_vel * err * dt_c, -a_max, a_max)
                axy = limits.kp_vel * err + integ - cfg.kd_vel * a[:2]
                m = float(np.hypot(*axy))
                if m > a_max:
                    axy *= a_max / m
                acc_sp = np.array([axy[0], axy[1], cfg.kv_z * (vz - v[2])])
            else:
                acc_sp = np.zeros(3)
                integ = np.zeros(2)
        # -- the airframe -----------------------------------------------------------
        if armed:
            a = a + (acc_sp - a) * (cfg.dt_s / cfg.accel_lag_s)
            v = v + a * cfg.dt_s
            p = p + v * cfg.dt_s
            if offboard:
                yaw_rate = float(np.clip(cfg.yaw_p * wrap(yaw_sp - yaw), -cfg.yaw_rate_max, cfg.yaw_rate_max))
                yaw = wrap(yaw + yaw_rate * cfg.dt_s)
        if p[2] <= 0.0:
            p[2] = 0.0
            if v[2] < 0:
                v[:] = 0.0
                a[:] = 0.0
        if p[2] > 0.05:
            ever_airborne = True
            if "takeoff" not in events:
                events["takeoff"] = now
        # -- land detector and disarms -----------------------------------------------
        on_ground = p[2] < 0.02 and abs(v[2]) < 0.1
        if on_ground:
            landed_since = now if landed_since is None else landed_since
            landed = now - landed_since >= 0.5 or not ever_airborne
        else:
            landed_since, landed = None, False
        if armed and landed and ever_airborne:
            events.setdefault("touchdown", now)
            if now - events["touchdown"] >= cfg.disarm_land_s:
                armed, offboard = False, False
                events["disarm"] = now
        if armed and not ever_airborne and now >= cfg.disarm_preflight_s:
            armed, offboard = False, False
            events["disarm_preflight"] = now

        if k % n_rec == 0:
            rec["t"].append(now)
            rec["pos"].append(p.copy())
            rec["yaw"].append(yaw)
            rec["state"].append(mission.state)
        if not armed or "pilot_takeover" in events:
            mission.update(now, False, None)   # the mission sees Offboard end
            if reasons[-1] != mission.reason:
                reasons.append(mission.reason)
            break
    return FlightLog(t=np.asarray(rec["t"]), pos=np.asarray(rec["pos"]), yaw=np.asarray(rec["yaw"]),
                     state=rec["state"], events=events, offboard_refused=refused,
                     final_state=mission.state, final_reason=mission.reason, reasons=reasons)


def _delayed_pose(history, when: float) -> Optional[OdomPose]:
    items = list(history)
    if not items:
        return None
    chosen = items[0]
    for item in items:
        if item[0] <= when:
            chosen = item
    t0, p, v, yaw = chosen
    return OdomPose(t=t0, x=float(p[0]), y=float(p[1]), z=float(p[2]), yaw=float(yaw),
                    vx=float(v[0]), vy=float(v[1]))


def summary(log: FlightLog) -> Dict[str, object]:
    ev = log.events
    out: Dict[str, object] = {"final_state": log.final_state, "final_reason": log.final_reason,
                              "offboard_refused": log.offboard_refused, "reasons": log.reasons}
    for k in ("switch", "offboard", "takeoff", "touchdown", "disarm", "disarm_preflight", "pilot_takeover"):
        if k in ev:
            out[f"{k}_s"] = round(ev[k], 2)
    if "offboard" in ev and "touchdown" in ev:
        out["offboard_to_touchdown_s"] = round(ev["touchdown"] - ev["offboard"], 2)
    states = []
    for s in log.state:
        if not states or states[-1] != s:
            states.append(s)
    out["states"] = states
    return out
