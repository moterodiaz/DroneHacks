"""The mission, as a pure state machine (no ROS): odometry in, twist out.

    WAIT        zero twists while odometry is fresh (lets the flight interface
                grant Offboard); starts when Offboard goes active
    TAKEOFF     climb straight up to liftoff_height_m
    PATH        follow the planned path; climb from liftoff_height_m to
                cruise_height_m over the first climb_distance_m of it
    PATH_END    settle over the path's last point
    LAND_1      descend; touchdown at the end of the path
    GROUND_1    sit for touchdown_dwell_s
    CLIMB_2     straight up to hop_height_m above the touchdown point
    YAW         only with hold_heading false: turn to face the take-off point
                (95.6 deg counter-clockwise with the nose east; 66.8 deg
                clockwise with the nose along the final approach)
    FORWARD     fly straight back over the take-off point at hop_height_m
                (by default without yawing: the heading is held throughout)
    FORWARD_END settle over the end of that leg
    LAND_2      descend; touchdown
    DONE        keep pressing down until PX4 disarms (COM_DISARM_LAND)

    HOLD        something went wrong: hover where we are, the pilot takes over
    ABORTED     Offboard was lost: publish nothing, never resume

Every command is a TwistStamped-style velocity (ENU odom, m/s) plus a yaw rate
(rad/s, + = counter-clockwise). Twists are used throughout, never poses: the
flight interface floors every pose at min_altitude_m (0.3 m by default), so a
0.2 m lift-off and a touchdown can only be flown with velocities. Twists are
capped at max_speed_mps by the interface and expire after 1 s if this node
stops, after which the interface holds position.
"""
import math
from collections import deque
from dataclasses import dataclass, fields

from .path import (ArenaToOdom, Polyline, profile_at, profile_duration,
                   speed_profile, truncate_path, wrap_pi)


@dataclass
class Config:
    # --- where the drone starts, in the path's arena frame ---------------
    start_x_m: float = float('nan')     # NaN: the path's first point
    start_y_m: float = float('nan')
    # Nose direction at lift-off, degrees CCW from arena +x. Any value works
    # as long as it matches how the drone is placed: the path is rotated by
    # it and the last leg ends over the take-off point regardless.
    # 0.0 (default) = arena +x ("east" on the map). NaN = parallel to the
    # path's final approach into the goal (162.5 deg for this path), which
    # makes the leg back to START 66.8 deg clockwise of the nose.
    start_yaw_deg: float = 0.0
    approach_chord_m: float = 1.0
    # The nose direction the path's wall margins were computed for. The
    # path-planning repo (11db65f) plans for the 0.50 x 0.40 m drone held
    # facing east, so its margins only hold at this heading.
    planned_nose_deg: float = 0.0

    # --- the mission -----------------------------------------------------
    liftoff_height_m: float = 0.2
    cruise_height_m: float = 1.25
    climb_distance_m: float = 1.5       # path length over which 0.2 -> 1.25
    hop_height_m: float = 2.0
    # The last leg: after touching down at the goal and climbing to
    # hop_height_m, fly straight back over the take-off point and land there.
    # True: never yaw; the camera keeps facing the start heading for the
    # whole flight. False: first turn to face the take-off point (95.6 deg
    # counter-clockwise with the nose east), then fly nose-first.
    # The touchdown point is the same either way.
    hold_heading: bool = True
    # Staged bring-up: fly only the first path_limit_m metres of the path
    # (0 = all of it), and/or stop after the first touchdown (no hop).
    path_limit_m: float = 0.0
    stop_after_landing_1: bool = False

    # --- speeds ----------------------------------------------------------
    takeoff_speed_mps: float = 0.3
    # Slow, for margin: the narrow corridors leave only 8-11 cm, and PX4
    # lags the velocity command (~0.7 s measured), so tracking error grows
    # with speed. Sim with 0.7 s lag, 2 cm noise, 5 cm/s push: 0.6/0.25/0.30
    # touched the north wall (-25 mm); these keep +10 mm there.
    path_speed_mps: float = 0.4
    path_min_speed_mps: float = 0.10
    full_speed_margin_m: float = 0.40   # planner margin at which full speed is allowed
    accel_mps2: float = 0.5
    lateral_accel_mps2: float = 0.6
    climb_speed_mps: float = 0.5
    forward_speed_mps: float = 0.5
    descent_speed_mps: float = 0.4
    final_descent_speed_mps: float = 0.2
    final_descent_height_m: float = 0.4
    max_cmd_speed_mps: float = 1.0      # below the interface's 1.5 cap
    max_yaw_rate_dps: float = 35.0

    # --- control ---------------------------------------------------------
    kp_xy: float = 1.2
    kp_z: float = 1.2
    kp_yaw: float = 1.5
    ff_lead_s: float = 0.3              # ~PX4's velocity-tracking lag
    ki_xy: float = 0.5
    i_xy_limit_mps: float = 0.15

    # --- tolerances, timeouts, safety ------------------------------------
    track_slow_error_m: float = 0.08    # carrot starts slowing beyond this
    track_stop_error_m: float = 0.18    # carrot stops beyond this
    abort_error_m: float = 0.5          # -> HOLD
    settle_xy_m: float = 0.12
    settle_z_m: float = 0.08
    settle_yaw_deg: float = 3.0
    settle_time_s: float = 0.5
    settle_timeout_s: float = 8.0
    liftoff_timeout_s: float = 8.0
    touchdown_dwell_s: float = 0.5      # must stay well under COM_DISARM_LAND
    touchdown_max_height_m: float = 0.3
    stationary_window_s: float = 1.0
    stationary_tol_m: float = 0.02
    descent_timeout_extra_s: float = 8.0
    odom_timeout_s: float = 0.3
    odom_abort_s: float = 2.0
    state_timeout_s: float = 1.0
    max_height_m: float = 2.5
    # Camera (cuVSLAM odom) vs PX4 (EKF2 local position), compared inside
    # one /skybeta/state message so there is no timing skew between them.
    # Before take-off: they must agree within this (3-D), else this node
    # publishes nothing and the interface refuses Offboard.
    preflight_divergence_m: float = 0.3
    # In flight: if the gap grows by more than this since take-off -> HOLD
    # with zero velocity (PX4 holds on its own estimate; the pilot lands).
    # Height is compared only over the floor (path and first landing): over
    # the walls the rangefinder legitimately reads the wall tops.
    divergence_xy_m: float = 0.5
    divergence_z_m: float = 0.4

    @classmethod
    def field_names(cls):
        return [f.name for f in fields(cls)]

    def validate(self):
        problems = []
        if not 0.0 < self.liftoff_height_m < self.cruise_height_m:
            problems.append('need 0 < liftoff_height_m < cruise_height_m')
        if not 0.0 < self.hop_height_m < self.max_height_m:
            problems.append('need 0 < hop_height_m < max_height_m')
        if self.cruise_height_m >= self.max_height_m:
            problems.append('need cruise_height_m < max_height_m')
        if self.max_height_m > 3.0:
            problems.append('max_height_m above the interface ceiling (3.0 m)')
        if self.max_cmd_speed_mps > 1.5:
            problems.append('max_cmd_speed_mps above the interface cap (1.5)')
        if self.touchdown_dwell_s >= 3.0:
            problems.append('touchdown_dwell_s must stay well under '
                            'COM_DISARM_LAND (5 s) or PX4 disarms on the ground')
        for name in ('takeoff_speed_mps', 'path_speed_mps', 'path_min_speed_mps',
                     'accel_mps2', 'lateral_accel_mps2', 'climb_speed_mps',
                     'forward_speed_mps', 'descent_speed_mps',
                     'final_descent_speed_mps', 'climb_distance_m',
                     'approach_chord_m'):
            if not getattr(self, name) > 0.0:
                problems.append(f'{name} must be > 0')
        if self.path_min_speed_mps > self.path_speed_mps:
            problems.append('path_min_speed_mps > path_speed_mps')
        if problems:
            raise ValueError('; '.join(problems))


@dataclass
class Observation:
    position: tuple = None      # ENU odom (x, y, z), or None
    yaw: float = None           # ENU odom yaw, rad
    odom_age_s: float = math.inf
    offboard: bool = None       # /skybeta/state offboard_active
    landed: bool = None         # /skybeta/state landed (PX4 land detector)
    state_age_s: float = math.inf
    divergence: tuple = None    # /skybeta/state odom_enu - local_enu (x, y, z)


@dataclass
class Command:
    phase: str
    twist: tuple = None         # (vx, vy, vz, yaw_rate) or None = publish nothing


FLYING = ('TAKEOFF', 'PATH', 'PATH_END', 'LAND_1', 'GROUND_1', 'CLIMB_2',
          'YAW', 'FORWARD', 'FORWARD_END', 'LAND_2', 'DONE', 'HOLD')


class Mission:
    def __init__(self, arena_points, margins, cfg):
        cfg.validate()
        self.cfg = cfg
        self.full_path_m = Polyline(list(arena_points)).length
        arena_points, margins = truncate_path(list(arena_points), margins,
                                              cfg.path_limit_m)
        self.arena_points = list(arena_points)
        self.arena_margins = margins
        self.phase = 'WAIT'
        self.events = []            # log lines for the node to print
        self.seen_offboard_inactive = False
        self._warned = set()
        self.t_prev = None

        # Path geometry in the arena frame. It is rigidly moved into odom at
        # start, which changes neither lengths nor curvature.
        self.arena_line = Polyline(self.arena_points, margins)
        self.profile = speed_profile(
            self.arena_line, cfg.path_speed_mps, cfg.path_min_speed_mps,
            cfg.full_speed_margin_m, cfg.accel_mps2, cfg.lateral_accel_mps2)
        sx = cfg.start_x_m if math.isfinite(cfg.start_x_m) else self.arena_points[0][0]
        sy = cfg.start_y_m if math.isfinite(cfg.start_y_m) else self.arena_points[0][1]
        self.arena_start = (sx, sy)
        offset = math.dist(self.arena_start, self.arena_points[0])
        if offset > 0.3:
            raise ValueError(
                f'start ({sx:.2f}, {sy:.2f}) is {offset:.2f} m from the path\'s '
                f'first point - the path would begin with a jump')

        end = self.arena_points[-1]
        if math.isfinite(cfg.start_yaw_deg):
            self.start_yaw_deg = cfg.start_yaw_deg
            self.start_yaw_auto = False
        else:
            a = self.arena_line.at(self.arena_line.length - cfg.approach_chord_m)
            self.start_yaw_deg = math.degrees(math.atan2(end[1] - a[1], end[0] - a[0]))
            self.start_yaw_auto = True
        off = abs(math.degrees(wrap_pi(math.radians(self.start_yaw_deg
                                                    - cfg.planned_nose_deg))))
        self.heading_warning = None
        if off > 2.0 or not cfg.hold_heading:
            self.heading_warning = (
                f'WARNING: the path was planned for the drone held facing east '
                f'({cfg.planned_nose_deg:.1f} deg), but this config flies it at '
                f'{self.start_yaw_deg:.1f} deg'
                + ('' if cfg.hold_heading else ' and yaws before the last leg')
                + '. The planner\'s wall margins (as low as 8 cm) do not hold.')

        # The last leg in the arena frame: goal -> take-off point.
        self.arena_leg = (self.arena_start[0] - end[0], self.arena_start[1] - end[1])
        self.arena_leg_yaw_deg = math.degrees(math.atan2(self.arena_leg[1],
                                                         self.arena_leg[0]))

        self.tf = None
        self.div0 = None
        self.hold_zero = False
        self.ground_z = None
        self.yaw_target = None
        self._reset_phase_state()

    # ------------------------------------------------------------------
    def summary(self):
        cfg = self.cfg
        end = self.arena_points[-1]
        cw = wrap_pi(math.radians(self.start_yaw_deg - self.arena_leg_yaw_deg))
        nose_src = ('parallel to the final approach' if self.start_yaw_auto
                    else 'east, along arena +x' if self.start_yaw_deg == 0.0
                    else 'set')
        return (
            f'path {self.arena_line.length:.2f} m, {len(self.arena_points)} points, '
            f'start ({self.arena_start[0]:.3f}, {self.arena_start[1]:.3f}), '
            f'end ({end[0]:.3f}, {end[1]:.3f}); planned path time '
            f'{profile_duration(self.arena_line, self.profile):.1f} s. '
            f'NOSE AT LIFT-OFF: {self.start_yaw_deg:.1f} deg CCW from arena +x '
            f'({nose_src}). Last leg: goal -> take-off point, '
            f'{math.hypot(*self.arena_leg):.2f} m at {self.arena_leg_yaw_deg:.1f} deg '
            f'= {_turn_words(cw)} of the nose; '
            f'{"heading held throughout" if cfg.hold_heading else "turns to face it first"}.'
            + (f' STAGED TEST: only the first {self.arena_line.length:.2f} m of '
               f'{self.full_path_m:.2f} m.' if self.arena_line.length
               < self.full_path_m - 1e-6 else '')
            + (' STAGED TEST: stops after the first touchdown (no hop).'
               if cfg.stop_after_landing_1 else '')
            + (f' {self.heading_warning}' if self.heading_warning else ''))

    def _log(self, msg):
        self.events.append(msg)

    def _warn_once(self, key, msg):
        if key not in self._warned:
            self._warned.add(key)
            self._log(msg)

    def _reset_phase_state(self):
        self.t_phase = None
        self.settled_since = None
        self.z_hist = deque()
        self.airborne_seen = False

    def _enter(self, phase, t, why=''):
        self._log(f'{self.phase} -> {phase}' + (f': {why}' if why else ''))
        self.phase = phase
        self.t_phase = t
        self.settled_since = None
        self.z_hist.clear()

    # ------------------------------------------------------------------
    def update(self, t, obs):
        dt = 0.0 if self.t_prev is None else min(max(t - self.t_prev, 0.0), 0.2)
        self.t_prev = t
        self.dt = dt
        cfg = self.cfg
        odom_ok = obs.position is not None and obs.yaw is not None \
            and obs.odom_age_s <= cfg.odom_timeout_s
        state_ok = obs.state_age_s <= cfg.state_timeout_s

        if self.phase == 'WAIT':
            return self._wait(t, obs, odom_ok, state_ok)
        if self.phase in ('ABORTED', 'COMPLETE'):
            return Command(self.phase, None)

        # The pilot (or a failsafe) took Offboard away: never fight it.
        if state_ok and obs.offboard is False:
            if self.phase == 'DONE':
                self._log('DONE -> COMPLETE: Offboard ended after the final '
                          'touchdown (PX4 disarmed).')
                self.phase = 'COMPLETE'
            else:
                self._log(f'{self.phase} -> ABORTED: Offboard lost - mission '
                          'stopped. It does not resume until the vehicle is '
                          'disarmed and armed again (and this node restarted).')
                self.phase = 'ABORTED'
            return Command(self.phase, None)

        if state_ok and obs.landed is False:
            self.airborne_seen = True

        if not odom_ok:
            if obs.odom_age_s > cfg.odom_abort_s and self.phase not in ('HOLD', 'DONE'):
                self.hold_point = None
                self._enter('HOLD', t, f'odometry lost for {obs.odom_age_s:.1f} s')
            # Freeze the mission and ask PX4 to stop where it is.
            self._warn_once(f'odom_stale_{self.phase}',
                            f'odometry stale in {self.phase} - zero velocity')
            return Command(self.phase, (0.0, 0.0, 0.0, 0.0))

        if (self.phase not in ('HOLD', 'DONE') and state_ok
                and obs.divergence is not None and self.div0 is not None):
            d = [obs.divergence[i] - self.div0[i] for i in range(3)]
            dxy = math.hypot(d[0], d[1])
            z_checked = self.phase in ('TAKEOFF', 'PATH', 'PATH_END', 'LAND_1',
                                       'GROUND_1')
            if dxy > cfg.divergence_xy_m or (z_checked
                                             and abs(d[2]) > cfg.divergence_z_m):
                self.hold_zero = True
                self._enter('HOLD', t, f'camera and PX4 positions drifted apart '
                            f'by {dxy:.2f} m horizontally, {d[2]:+.2f} m vertically '
                            f'since take-off - zero velocity, PX4 holds. Pilot: '
                            f'take over and land.')
                return self._hold(t, dt, obs)

        height = obs.position[2] - self.ground_z
        if height > cfg.max_height_m and self.phase != 'HOLD':
            self.hold_point = (obs.position[0], obs.position[1],
                               self.ground_z + cfg.max_height_m - 0.3)
            self._enter('HOLD', t, f'height {height:.2f} m above max_height_m')

        handler = getattr(self, '_' + self.phase.lower())
        return handler(t, dt, obs)

    # ------------------------------------------------------------------
    # helpers
    def _track(self, obs, ref, ff=(0.0, 0.0, 0.0), xy_gain=None,
               integrate=True):
        """Velocity = feed-forward + PI on the x/y error + P on height.

        The small, clamped x/y integral removes a steady offset (e.g. a
        constant mismatch between the vision frame and PX4's), which matters
        where the planner left only a few centimetres of wall margin. It
        learns only while the reference stands still (integrate=True) and
        is applied, frozen, while moving: integrating the ordinary lag on a
        moving reference winds it up and overshoots bends."""
        cfg = self.cfg
        p = obs.position
        kxy = cfg.kp_xy if xy_gain is None else xy_gain
        ex, ey = ref[0] - p[0], ref[1] - p[1]
        if not integrate or kxy <= 0.0:
            ix, iy = self.i_xy if kxy > 0.0 else (0.0, 0.0)
        elif cfg.ki_xy > 0.0:
            lim = cfg.i_xy_limit_mps
            self.i_xy = [min(max(self.i_xy[0] + cfg.ki_xy * ex * self.dt, -lim), lim),
                         min(max(self.i_xy[1] + cfg.ki_xy * ey * self.dt, -lim), lim)]
            ix, iy = self.i_xy
        else:
            ix = iy = 0.0
        v = [ff[0] + kxy * ex + ix,
             ff[1] + kxy * ey + iy,
             ff[2] + cfg.kp_z * (ref[2] - p[2])]
        return self._cap(v) + (self._yaw_rate(obs),)

    def _cap(self, v):
        n = math.sqrt(sum(c * c for c in v))
        if n > self.cfg.max_cmd_speed_mps:
            v = [c * self.cfg.max_cmd_speed_mps / n for c in v]
        return tuple(v)

    def _yaw_rate(self, obs):
        cfg = self.cfg
        err = wrap_pi(self.yaw_target - obs.yaw)
        lim = math.radians(cfg.max_yaw_rate_dps)
        return min(max(cfg.kp_yaw * err, -lim), lim)

    def _settled(self, t, obs, ref, check_yaw=False):
        cfg = self.cfg
        p = obs.position
        ok = (math.hypot(ref[0] - p[0], ref[1] - p[1]) <= cfg.settle_xy_m
              and abs(ref[2] - p[2]) <= cfg.settle_z_m)
        if check_yaw:
            ok = ok and abs(math.degrees(wrap_pi(self.yaw_target - obs.yaw))) \
                <= cfg.settle_yaw_deg
        if not ok:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = t
        return t - self.settled_since >= cfg.settle_time_s

    def _timed_out(self, t, limit):
        return t - self.t_phase > limit

    # ------------------------------------------------------------------
    # phases
    def _wait(self, t, obs, odom_ok, state_ok):
        cfg = self.cfg
        if state_ok and obs.offboard is False:
            self.seen_offboard_inactive = True
        if state_ok and obs.offboard is True and not self.seen_offboard_inactive:
            self._warn_once('offboard_at_start',
                            'Offboard is already active but this node never saw '
                            'it inactive (restarted mid-flight?) - not flying.')
            return Command('WAIT', None)
        if not odom_ok:
            self._warn_once('wait_no_odom', 'waiting for fresh odometry - '
                            'publishing nothing, so Offboard will be refused')
            return Command('WAIT', None)
        if cfg.preflight_divergence_m > 0.0:
            d = obs.divergence
            if d is None or not state_ok:
                self._warn_once('wait_no_ekf', 'waiting for the PX4 position in '
                                '/skybeta/state - publishing nothing, so Offboard '
                                'will be refused')
                return Command('WAIT', None)
            gap = math.sqrt(sum(c * c for c in d))
            if gap > cfg.preflight_divergence_m:
                self._warn_once(f'diverged_{int(gap)}',
                                f'camera and PX4 positions disagree by {gap:.2f} m '
                                f'(limit {cfg.preflight_divergence_m:.2f}) - '
                                f'publishing nothing, so Offboard will be refused. '
                                f'Power-cycle the drone sitting on START.')
                self.diverged_reported = True
                return Command('WAIT', None)
            if getattr(self, 'diverged_reported', False):
                self.diverged_reported = False
                self._log(f'camera and PX4 positions agree again ({gap:.2f} m)')
        if state_ok and obs.offboard is True:
            self._start(t, obs)
            return self._takeoff(t, 0.0, obs)
        # Zero twists keep the interface's "fresh team setpoint" condition
        # true without asking for any motion when Offboard begins.
        return Command('WAIT', (0.0, 0.0, 0.0, 0.0))

    def _start(self, t, obs):
        cfg = self.cfg
        p = obs.position
        self.tf = ArenaToOdom(self.arena_start, math.radians(self.start_yaw_deg),
                              (p[0], p[1]), obs.yaw)
        self.odom_points = [self.tf.point(a) for a in self.arena_points]
        self.line = Polyline(self.odom_points, self.arena_margins)
        self.ground_z = p[2]
        self.start_xy = (p[0], p[1])
        self.yaw_target = obs.yaw
        self.z_ref = p[2]
        self.s = 0.0
        self.v_c = 0.0
        self.hold_point = None
        self.i_xy = [0.0, 0.0]
        self.div0 = tuple(obs.divergence) if obs.divergence is not None else None
        self._enter('TAKEOFF', t,
                    f'Offboard active; anchored arena start to odom '
                    f'({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) yaw '
                    f'{math.degrees(obs.yaw):.1f} deg')

    def _takeoff(self, t, dt, obs):
        cfg = self.cfg
        target = self.ground_z + cfg.liftoff_height_m
        ff = 0.0
        if self.z_ref < target:
            self.z_ref = min(target, self.z_ref + cfg.takeoff_speed_mps * dt)
            ff = cfg.takeoff_speed_mps if self.z_ref < target else 0.0
        ref = (self.start_xy[0], self.start_xy[1], self.z_ref)
        cmd = self._track(obs, ref, (0.0, 0.0, ff))
        if self.z_ref >= target and self._settled(t, obs, ref):
            self._enter('PATH', t, f'at {cfg.liftoff_height_m:.2f} m - following '
                        f'the path, climbing to {cfg.cruise_height_m:.2f} m')
        elif self._timed_out(t, cfg.liftoff_timeout_s + cfg.liftoff_height_m
                             / cfg.takeoff_speed_mps):
            if obs.position[2] - self.ground_z > 0.5 * cfg.liftoff_height_m:
                self._enter('PATH', t, 'lift-off settle timed out, airborne - '
                            'following the path anyway')
            else:
                self.hold_point = None
                self._enter('HOLD', t, 'did not leave the ground')
        return Command('TAKEOFF', cmd)

    def _height_ref(self, s):
        """Height above ground along the path, and its slope dh/ds.

        A smoothstep from liftoff to cruise height over climb_distance_m:
        the climb starts with the motion and fades out without a kink, so
        PX4 does not overshoot the cruise height."""
        cfg = self.cfg
        u = min(max(s / cfg.climb_distance_m, 0.0), 1.0)
        dz = cfg.cruise_height_m - cfg.liftoff_height_m
        h = cfg.liftoff_height_m + dz * u * u * (3.0 - 2.0 * u)
        slope = dz * 6.0 * u * (1.0 - u) / cfg.climb_distance_m
        return h, slope

    def _path(self, t, dt, obs):
        cfg = self.cfg
        L = self.line.length
        p = obs.position
        ref_xy = self.line.at(self.s)
        err = math.hypot(ref_xy[0] - p[0], ref_xy[1] - p[1])
        if err > cfg.abort_error_m:
            self.hold_point = None
            self._enter('HOLD', t, f'{err:.2f} m off the path at s={self.s:.2f}')
            return self._hold(t, dt, obs)

        # Carrot speed: the profile, slowed (then stopped) when the vehicle
        # lags, so the reference never runs away from it.
        slow, stop = cfg.track_slow_error_m, cfg.track_stop_error_m
        gov = min(max((stop - err) / max(stop - slow, 1e-6), 0.0), 1.0)
        v_target = max(profile_at(self.arena_line, self.profile, self.s), 0.05) * gov
        if v_target > self.v_c:
            self.v_c = min(v_target, self.v_c + cfg.accel_mps2 * dt)
        else:
            self.v_c = v_target
        self.s = min(L, self.s + self.v_c * dt)

        ref_xy = self.line.at(self.s)
        # PX4 follows a velocity command with a lag, so feed forward the
        # speed and direction the reference will have ff_lead_s from now:
        # the vehicle starts braking and turning in time instead of
        # overshooting the end of the path or the outside of a bend.
        s_lead = min(L, self.s + self.v_c * cfg.ff_lead_s)
        ff_speed = min(self.v_c, profile_at(self.arena_line, self.profile, s_lead))
        tan = self.line.tangent(s_lead)
        if self.s >= L:
            ff_speed, self.v_c = 0.0, 0.0
        h, _ = self._height_ref(self.s)
        _, dh = self._height_ref(s_lead)
        ref = (ref_xy[0], ref_xy[1], self.ground_z + h)
        cmd = self._track(obs, ref, (ff_speed * tan[0], ff_speed * tan[1],
                                     ff_speed * dh), integrate=False)
        if self.s >= L:
            self.end_ref = ref
            self._enter('PATH_END', t, 'reached the end of the path')
        return Command('PATH', cmd)

    def _path_end(self, t, dt, obs):
        cmd = self._track(obs, self.end_ref)
        if self._settled(t, obs, self.end_ref):
            self._begin_landing('LAND_1', t, obs, self.end_ref)
        elif self._timed_out(t, self.cfg.settle_timeout_s):
            self._begin_landing('LAND_1', t, obs, self.end_ref,
                                'settle timed out - landing anyway')
        return Command('PATH_END', cmd)

    def _begin_landing(self, phase, t, obs, over, why=''):
        self.land_xy = (over[0], over[1])
        self.land_ground_z = self.ground_z
        self.land_timeout = (max(obs.position[2] - self.ground_z, 0.0)
                             / self.cfg.final_descent_speed_mps
                             + self.cfg.descent_timeout_extra_s)
        self._enter(phase, t, why or 'descending to touch down')

    def _descend(self, t, obs):
        """One tick of a landing descent. Returns (twist, touched_down, why)."""
        cfg = self.cfg
        p = obs.position
        height = p[2] - self.land_ground_z
        vz = -(cfg.descent_speed_mps if height > cfg.final_descent_height_m
               else cfg.final_descent_speed_mps)
        # Hold x/y over the touchdown point (PI, as in a hover) until just
        # above the floor, then stop steering so the legs are not dragged
        # along the ground.
        kxy = cfg.kp_xy if height > 0.05 else 0.0
        ref = (self.land_xy[0], self.land_xy[1], p[2])
        v = self._track(obs, ref, xy_gain=kxy, integrate=height > 0.15)
        cmd = self._cap([v[0], v[1], vz]) + (v[3],)

        self.z_hist.append((t, p[2]))
        while self.z_hist and t - self.z_hist[0][0] > cfg.stationary_window_s:
            self.z_hist.popleft()
        span = self.z_hist[-1][0] - self.z_hist[0][0] if self.z_hist else 0.0
        zs = [z for _, z in self.z_hist]
        stationary = (span >= 0.9 * cfg.stationary_window_s
                      and max(zs) - min(zs) <= cfg.stationary_tol_m)

        if self.airborne_seen and obs.landed is True \
                and obs.state_age_s <= cfg.state_timeout_s:
            return cmd, True, 'PX4 land detector: landed'
        if stationary and height <= cfg.touchdown_max_height_m:
            return cmd, True, (f'not descending at {height:.2f} m '
                               f'(commanded {vz:.2f} m/s)')
        if self._timed_out(t, self.land_timeout):
            return cmd, True, 'descent timeout'
        return cmd, False, ''

    def _land_1(self, t, dt, obs):
        cmd, down, why = self._descend(t, obs)
        if down:
            self.touchdown_xyz = obs.position
            self.airborne_seen = False
            self._enter('GROUND_1', t, f'touchdown 1 ({why}) at odom '
                        f'({obs.position[0]:.2f}, {obs.position[1]:.2f}, '
                        f'{obs.position[2]:.2f}); arena '
                        f'{_fmt(self.tf.inverse_point(obs.position))}')
        return Command('LAND_1', cmd)

    def _ground_1(self, t, dt, obs):
        cfg = self.cfg
        if cfg.stop_after_landing_1:
            self._enter('DONE', t, 'staged test: stopping after the first '
                        'touchdown. Holding down until PX4 disarms.')
            return self._done(t, dt, obs)
        if self._timed_out(t, cfg.touchdown_dwell_s):
            # Heights for the hop are measured from where we actually touched
            # down, which absorbs any vertical drift in the vision estimate.
            self.ground_z = self.touchdown_xyz[2]
            self.z_ref = self.touchdown_xyz[2]
            self._enter('CLIMB_2', t, f'climbing to {cfg.hop_height_m:.2f} m')
            return self._climb_2(t, dt, obs)
        return Command('GROUND_1', (0.0, 0.0, -cfg.final_descent_speed_mps,
                                    0.0))

    def _climb_2(self, t, dt, obs):
        cfg = self.cfg
        target = self.ground_z + cfg.hop_height_m
        ff = 0.0
        if self.z_ref < target:
            self.z_ref = min(target, self.z_ref + cfg.climb_speed_mps * dt)
            ff = cfg.climb_speed_mps if self.z_ref < target else 0.0
        ref = (self.touchdown_xyz[0], self.touchdown_xyz[1], self.z_ref)
        # Little x/y authority until clear of the ground.
        kxy = self.cfg.kp_xy if obs.position[2] - self.ground_z > 0.15 else 0.0
        cmd = self._track(obs, ref, (0.0, 0.0, ff), xy_gain=kxy)
        if self.z_ref >= target and (
                self._settled(t, obs, ref)
                or self._timed_out(t, cfg.hop_height_m / cfg.climb_speed_mps
                                   + cfg.settle_timeout_s)):
            self.hop_ref = ref
            # Last leg: from where we are (over touchdown 1) straight back
            # over the measured take-off point. Measured odom points, so the
            # leg ends over the take-off spot even if start_yaw_deg is off.
            dx = self.start_xy[0] - ref[0]
            dy = self.start_xy[1] - ref[1]
            self.leg_len = math.hypot(dx, dy)
            self.leg_yaw = math.atan2(dy, dx)
            rel = _turn_words(wrap_pi(self.yaw_target - self.leg_yaw))
            if cfg.hold_heading:
                self._begin_forward(t, f'heading held, the take-off point is '
                                    f'{rel} of the nose')
            else:
                self.yaw_target = self.leg_yaw
                self._enter('YAW', t, f'turning {rel} to face the take-off point')
        return Command('CLIMB_2', cmd)

    def _yaw(self, t, dt, obs):
        cmd = self._track(obs, self.hop_ref)
        settled = self._settled(t, obs, self.hop_ref, check_yaw=True)
        if settled or self._timed_out(t, self.cfg.settle_timeout_s + 6.0):
            if not settled:
                self._log('yaw did not settle in time - continuing on the '
                          'commanded heading')
            self._begin_forward(t, 'nose first')
        return Command('YAW', cmd)

    def _begin_forward(self, t, how):
        self.fwd_origin = self.hop_ref
        self.fwd_dir = (math.cos(self.leg_yaw), math.sin(self.leg_yaw))
        self.d = 0.0
        self.v_c = 0.0
        self._enter('FORWARD', t, f'{self.leg_len:.2f} m back over the take-off '
                    f'point at {self.cfg.hop_height_m:.2f} m ({how})')

    def _forward(self, t, dt, obs):
        cfg = self.cfg
        D = self.leg_len
        p = obs.position
        ref = self._fwd_point(self.d)
        err = math.hypot(ref[0] - p[0], ref[1] - p[1])
        if err > cfg.abort_error_m:
            self.hold_point = None
            self._enter('HOLD', t, f'{err:.2f} m off the forward leg')
            return self._hold(t, dt, obs)
        def v_at(d):
            return min(cfg.forward_speed_mps,
                       math.sqrt(max(2.0 * cfg.accel_mps2 * (D - d), 0.0)))
        self.v_c = min(max(v_at(self.d), 0.05), self.v_c + cfg.accel_mps2 * dt)
        self.d = min(D, self.d + self.v_c * dt)
        ref = self._fwd_point(self.d)
        ff_speed = min(self.v_c, v_at(self.d + self.v_c * cfg.ff_lead_s))
        ff = (ff_speed * self.fwd_dir[0], ff_speed * self.fwd_dir[1], 0.0)
        if self.d >= D:
            ff = (0.0, 0.0, 0.0)
        cmd = self._track(obs, ref, ff, integrate=False)
        if self.d >= D:
            self.end_ref = ref
            self._enter('FORWARD_END', t, 'end of the forward leg')
        return Command('FORWARD', cmd)

    def _fwd_point(self, d):
        o = self.fwd_origin
        return (o[0] + d * self.fwd_dir[0], o[1] + d * self.fwd_dir[1], o[2])

    def _forward_end(self, t, dt, obs):
        cmd = self._track(obs, self.end_ref)
        if self._settled(t, obs, self.end_ref):
            self._begin_landing('LAND_2', t, obs, self.end_ref)
        elif self._timed_out(t, self.cfg.settle_timeout_s):
            self._begin_landing('LAND_2', t, obs, self.end_ref,
                                'settle timed out - landing anyway')
        return Command('FORWARD_END', cmd)

    def _land_2(self, t, dt, obs):
        cmd, down, why = self._descend(t, obs)
        if down:
            self._enter('DONE', t, f'final touchdown ({why}) at odom '
                        f'({obs.position[0]:.2f}, {obs.position[1]:.2f}, '
                        f'{obs.position[2]:.2f}); arena '
                        f'{_fmt(self.tf.inverse_point(obs.position))}. Holding '
                        f'down until PX4 disarms.')
        return Command('LAND_2', cmd)

    def _done(self, t, dt, obs):
        # Keep a gentle downward command: never an upward component, so PX4
        # cannot take off again, and its land detector disarms the vehicle.
        return Command('DONE', (0.0, 0.0, -self.cfg.final_descent_speed_mps, 0.0))

    def _hold(self, t, dt, obs):
        if self.hold_zero:
            # Position estimates disagree: do not steer on either; ask PX4
            # for zero velocity and let the pilot land.
            self._warn_once('hold_zero', 'HOLD (zero velocity): the safety '
                            'pilot should take over and land.')
            return Command('HOLD', (0.0, 0.0, 0.0, 0.0))
        if self.hold_point is None:
            self.hold_point = tuple(obs.position)
        self._warn_once('hold', 'HOLD: hovering in place. The safety pilot '
                        'should take over and land.')
        return Command('HOLD', self._track(obs, self.hold_point))


def _turn_words(cw_rad):
    """A clockwise angle (rad, may be negative) as words, seen from above."""
    deg = math.degrees(cw_rad)
    return (f'{deg:.1f} deg clockwise' if deg >= 0
            else f'{-deg:.1f} deg counter-clockwise')


def _fmt(p):
    return f'({p[0]:.2f}, {p[1]:.2f})'
