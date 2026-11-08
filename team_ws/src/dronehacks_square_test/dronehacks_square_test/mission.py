"""Cage test: climb, a box with turns, a low-hover rest, a circle inside the box, land and disarm.

Pure logic, no ROS, so it can be unit-tested. ``square_test_node`` feeds it the cuVSLAM pose and
``/skybeta/state`` and publishes what it returns. Frame: ENU ``odom`` (cuVSLAM's), whose origin
and axes are the body's at power-on: x forward, y left, z up. Heights are above the take-off point.

The sequence (``plan``), every hold 3 s once within ``arrive_m`` of the target:

    up to height_m (0.73 m: the real follower's setpoint, cruise 0.80 m minus 0.07 m rest height)
    forward side_m, left side_m, back side_m, right side_m (over the start again), holding at each;
      during each hold it first yaws slowly (``yaw_rate_dps``) to face the next leg: +90 deg
      (a left turn) at each corner, and +90 more over the start, so it faces its take-off heading
      again before landing. Moves never yaw; a hold's 3 s start once position and heading arrive.
      ``heading_mode``: 'turn' flies that box (a "lap turn"); 'fixed' flies it never yawing, the
      take-off heading held through the box and the circle (a "lap fixed"); 'both' flies a lap
      fixed, then a lap turn from over the start, then the rest: an A/B of whether yawing drifts
      the drone at the corners, on one battery and one bias. The status names the lap
    rest: down (velocity, at most ``descent_mps``) to ``rest_height_m`` above the start by camera,
      pulling back over the start, hover there ``rest_s``; if PX4 reports landed meanwhile, climb at once
    take off again to height_m, hold
    a circle of diameter ``circle_d_m`` inside the box (centre r forward, r left): forward r to the
      middle of the box's right side, round counter-clockwise from there (forward first) back to
      it, heading held; then back over the start and hold
    land as the course follower does: velocity straight down at ``descent_mps`` (0.5 m/s), pulling
      back over the start, then pressing down until PX4 disarms (COM_DISARM_LAND)

The rest hovers instead of landing: PX4 disarms 2 s after it reports landed (COM_DISARM_LAND, the
organisers' and frozen), a disarm ends Offboard and only the pilot can arm again, and in flight
20261108-013018 the motors had not lifted it 0.8 s into a climb started 1.1 s after landed.

States: WAIT (stream the current position so the kit can enter Offboard on arm-then-Position),
FLY (working through the plan), DONE (disarmed), STOPPED (Offboard ended early: the pilot has it;
never tries again). Moves slide a target at ``speed_mps``, never more than ``lead_m`` ahead of the
drone, so PX4 never sees a far setpoint. A stale pose holds still (zero velocity), and after
``stale_land_s`` descends straight down. A step not finished within ``step_timeout_s`` lands where
the drone is and ends.
"""
import math
from dataclasses import dataclass

WAIT, FLY, DONE, STOPPED = 'WAIT', 'FLY', 'DONE', 'STOPPED'

# The box laps each heading mode flies, in order: 'turn' yaws +90 deg at each corner, 'fixed' never yaws
HEADING_MODES = {'turn': ('turn',), 'fixed': ('fixed',), 'both': ('fixed', 'turn')}


@dataclass
class Config:
    side_m: float = 0.5
    height_m: float = 0.73          # the real follower's setpoint z (cruise_z_m 0.80 - rest_height_m 0.07)
    speed_mps: float = 0.3
    lead_m: float = 0.3
    arrive_m: float = 0.10          # horizontal: the real follower's land_tolerance_m
    arrive_z_m: float = 0.15        # vertical: PX4 holds height by rangefinder, a few cm off cuVSLAM's z
    hold_s: float = 3.0
    rest_s: float = 3.0             # the low hover between the box and the circle
    rest_height_m: float = 0.08     # above the start by camera; never landed, so COM_DISARM_LAND never starts
    rest_band_m: float = 0.03       # the rest's seconds count from within this of rest_height_m
    rest_gain: float = 1.0          # m/s per m of height error
    step_timeout_s: float = 20.0
    descent_mps: float = 0.5        # PX4 counts ground contact only while the setpoint descends at 1.1 x
                                    # LNDMC_Z_VEL_MAX = 0.275 m/s; at 0.3 skybeta-11 showed none 1.3 s
                                    # after touchdown (2026-11-07). 0.5, as the course follower lands
    land_xy_gain: float = 1.0       # the real follower's land_xy_gain
    land_xy_max_mps: float = 0.2    # the real follower's land_xy_max_mps
    pose_timeout_s: float = 0.5
    stale_land_s: float = 3.0
    yaw_rate_dps: float = 20.0      # slow turn: 90 deg in 4.5 s
    yaw_tol_deg: float = 5.0
    max_offset_m: float = 0.3       # PX4 vs camera position on the ground; more than this: refuse to fly
    offset_tau_s: float = 1.0       # smoothing of the PX4-vs-camera offset in flight
    circle_d_m: float = 1.0
    heading_mode: str = 'turn'      # 'turn', 'fixed' or 'both': see HEADING_MODES


@dataclass
class Command:
    kind: str                        # 'none', 'pose' or 'twist'
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = None                # ENU rad for a pose; None: do not command yaw


NONE = Command('none')


def plan(c):
    """Steps as (kind, (forward, left, up) offsets from the start, seconds, heading in degrees
    counter-clockwise from the take-off heading, unwrapped, box lap name or None)."""
    if c.heading_mode not in HEADING_MODES:
        raise ValueError('heading_mode must be one of {}, not {!r}'.format(sorted(HEADING_MODES), c.heading_mode))
    s, h, r = c.side_m, c.height_m, c.circle_d_m / 2.0
    box = [(0, 0, h), (s, 0, h), (s, s, h), (0, s, h), (0, 0, h)]
    steps = []
    for lap in HEADING_MODES[c.heading_mode]:
        # a second lap starts over the start, where the first one ended
        first = 1 if steps else 0
        for k, p in enumerate(box[first:], first):
            steps += [('move', p, 0, None, lap), ('hold', p, c.hold_s, 90.0 * k if lap == 'turn' else 0.0, lap)]
    # after a lap turn it faces its take-off heading again a full turn on, unwrapped
    end = steps[-1][3]
    steps += [('rest', None, c.rest_s, None, None),
              ('move', (0, 0, h), 0, None, None), ('hold', (0, 0, h), c.hold_s, end, None),
              ('move', (r, 0, h), 0, None, None), ('circle', (r, 0, h), 0, None, None),
              ('move', (0, 0, h), 0, None, None), ('hold', (0, 0, h), c.hold_s, end, None),
              ('land', None, 0, None, None), ('disarm', None, 0, None, None)]
    return steps


class Mission:
    def __init__(self, cfg: Config = Config()):
        self.cfg = cfg
        self.steps = plan(cfg)
        self.state = WAIT
        self.reason = 'waiting for arm, then Position'
        self.i = 0
        self.start = None
        self.target = None           # the sliding pose target, ENU
        self.start_yaw = None        # ENU rad at Offboard; None if the pose had no heading
        self.heading = 0.0           # the sliding heading target, degrees from start_yaw, unwrapped
        self.since = None            # when the current step started
        self.dwell_since = None
        self.stale_since = None
        self.last_t = None
        self.offset = (0.0, 0.0, 0.0)  # PX4 local minus camera, smoothed
        self.arc = 0.0               # radians of the circle covered

    @property
    def step_name(self):
        return self.steps[self.i][0] if self.state == FLY else self.state

    def step(self, t, pose, pose_t, offboard, landed, armed, yaw=None, px4=None):
        """One tick. pose: (x, y, z) or None; pose_t: when it was captured (s); yaw: its ENU
        heading in radians, or None (then the drone holds whatever heading it has); px4: PX4's
        own local position, ENU (``/skybeta/state`` local_enu), or None.

        The kit passes pose setpoints to PX4 unanchored (``anchor_setpoints`` false), so PX4 flies
        them in its own frame. Its height can sit off the camera's (0.73 m in one flight, 0.15 m
        after a clean power cycle), so every setpoint carries the PX4-minus-camera offset taken at
        Offboard, then followed (smoothed) through the flight, and all checks stay in the camera frame."""
        c = self.cfg
        fresh = pose is not None and pose_t is not None and t - pose_t <= c.pose_timeout_s
        if self.state in (DONE, STOPPED):
            return NONE
        if self.state == WAIT:
            if not fresh:
                return NONE          # no stream: the kit will not enter Offboard
            self.offset = (0.0, 0.0, 0.0) if px4 is None else tuple(p - o for p, o in zip(px4, pose))
            if math.hypot(*self.offset) > c.max_offset_m:
                self.reason = 'PX4 and camera disagree by {:.2f} m: power-cycle on the spot'.format(
                    math.hypot(*self.offset))
                return NONE          # no stream: no Offboard, the pilot keeps Position
            self.reason = 'waiting for arm, then Position'
            if not offboard:
                return Command('pose', *self._px4(pose))
            self.start, self.target, self.start_yaw, self.heading = pose, pose, yaw, 0.0
            self.state, self.i, self.since, self.last_t = FLY, 0, t, t
        elif armed is False:
            # checked first: PX4 disarming also ends Offboard, and the kit can still report it active
            kind = self.steps[self.i][0]
            self.state = DONE
            self.reason = 'disarmed on the ground' if kind == 'disarm' else \
                'PX4 disarmed during step {} ({}): landed longer than COM_DISARM_LAND?'.format(self.i + 1, kind)
            return NONE
        elif not offboard:
            self.state, self.reason = STOPPED, 'Offboard ended: the pilot has it'
            return NONE

        dt, self.last_t = max(0.0, t - self.last_t), t
        if not fresh:
            if self.stale_since is None:
                self.stale_since = t
            if t - self.stale_since >= c.stale_land_s:
                self._abort('pose lost: straight down')
                return Command('twist', 0.0, 0.0, -c.descent_mps)
            self.reason = 'pose stale: holding still'
            return Command('twist')
        self.stale_since = None
        if px4 is not None:
            # follow PX4's frame as it changes (its rangefinder height is wrong on the ground, right in
            # the air); 1 s smoothing; both inputs measure the same drone, so this is not a control loop
            a = min(1.0, dt / c.offset_tau_s)
            self.offset = tuple(o + a * (p - q - o) for o, p, q in zip(self.offset, px4, pose))

        kind, offset, secs, heading, lap = self.steps[self.i]
        self.reason = 'step {}/{}: {}'.format(self.i + 1, len(self.steps), kind)
        if lap is not None:
            self.reason += ', lap ' + lap
        if kind not in ('land', 'disarm') and t - self.since > c.step_timeout_s:
            self._abort('step {} ({}) took over {:.0f} s: landing here'.format(self.i + 1, kind, c.step_timeout_s))
            self.start = pose
            kind = 'disarm'

        if kind == 'move':
            goal = self._at(offset)
            gap = math.dist(self.target, goal)
            if math.dist(pose, self.target) < c.lead_m and gap > 0:
                f = min(1.0, c.speed_mps * dt / gap)
                self.target = tuple(a + (b - a) * f for a, b in zip(self.target, goal))
            if math.dist(self.target, goal) < 1e-6:
                self._next(t)
            return self._pose()
        if kind == 'circle':
            r = c.circle_d_m / 2.0
            if math.dist(pose, self.target) < c.lead_m:
                self.arc = min(2.0 * math.pi, self.arc + c.speed_mps * dt / r)
            a = -math.pi / 2 + self.arc             # round the centre (r, r); the entry (r, 0) is at -pi/2
            self.target = self._at((r + r * math.cos(a), r + r * math.sin(a), offset[2]))
            if self.arc >= 2.0 * math.pi:
                self.target = self._at(offset)
                self._next(t)
            return self._pose()
        if kind == 'hold':
            turn = heading - self.heading
            self.heading += max(-c.yaw_rate_dps * dt, min(c.yaw_rate_dps * dt, turn))
            facing = self.start_yaw is None or (
                abs(turn) < 1e-9 and (yaw is None or abs(_wrap(yaw - self._yaw_cmd())) <= math.radians(c.yaw_tol_deg)))
            arrived = (math.dist(pose[:2], self.target[:2]) <= c.arrive_m
                       and abs(pose[2] - self.target[2]) <= c.arrive_z_m)
            if not arrived or not facing:
                self.dwell_since = None
            elif self.dwell_since is None:
                self.dwell_since = t
            elif t - self.dwell_since >= secs:
                self._next(t)
            return self._pose()
        if kind == 'rest':
            # velocity, not pose: the kit floors pose setpoints at 0.3 m
            zt = self.start[2] + c.rest_height_m
            if abs(zt - pose[2]) <= c.rest_band_m and self.dwell_since is None:
                self.dwell_since = t
            if landed or (self.dwell_since is not None and t - self.dwell_since >= secs):
                self.target = (self.start[0], self.start[1], pose[2])   # touched down: climb now
                self._next(t)
                return Command('twist')
            return Command('twist', _clip(c.land_xy_gain * (self.start[0] - pose[0]), c.land_xy_max_mps),
                           _clip(c.land_xy_gain * (self.start[1] - pose[1]), c.land_xy_max_mps),
                           _clip(c.rest_gain * (zt - pose[2]), c.descent_mps))
        # land / disarm: the real follower's descent
        if landed:
            if kind == 'land':
                self._next(t)
            return Command('twist', 0.0, 0.0, -0.3 * c.descent_mps)
        vx = _clip(c.land_xy_gain * (self.start[0] - pose[0]), c.land_xy_max_mps)
        vy = _clip(c.land_xy_gain * (self.start[1] - pose[1]), c.land_xy_max_mps)
        return Command('twist', vx, vy, -c.descent_mps)

    # ------------------------------------------------------------------
    def _yaw_cmd(self):
        return _wrap(self.start_yaw + math.radians(self.heading))

    def _pose(self):
        return Command('pose', *self._px4(self.target), yaw=None if self.start_yaw is None else self._yaw_cmd())

    def _px4(self, p):
        return tuple(a + o for a, o in zip(p, self.offset))

    def _at(self, offset):
        return tuple(s + o for s, o in zip(self.start, offset))

    def _next(self, t):
        self.i, self.since, self.dwell_since, self.arc = self.i + 1, t, None, 0.0

    def _abort(self, why):
        self.i = len(self.steps) - 1            # the final 'disarm' landing
        self.reason = why


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(v, m):
    return max(-m, min(m, v))
