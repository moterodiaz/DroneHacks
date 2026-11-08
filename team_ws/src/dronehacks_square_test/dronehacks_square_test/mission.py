"""Cage test: climb, a 0.5 m box with holds, land, touch-and-go, land and disarm.

Pure logic, no ROS, so it can be unit-tested. ``square_test_node`` feeds it the cuVSLAM pose and
``/skybeta/state`` and publishes what it returns. Frame: ENU ``odom`` (cuVSLAM's), whose origin
and axes are the body's at power-on: x forward, y left, z up. Heights are above the take-off point.

The sequence (``plan``), every hold 3 s once within ``arrive_m`` of the target:

    up to height_m (0.73 m: the real follower's setpoint, cruise 0.80 m minus 0.07 m rest height)
    forward side_m, left side_m, back side_m, right side_m (over the start again), holding at each;
      during each hold it first yaws slowly (``yaw_rate_dps``) to face the next leg: +90 deg
      (a left turn) at each corner, and +90 more over the start, so it faces its take-off heading
      again before landing. Moves never yaw; a hold's 3 s start once position and heading arrive
    land, as the course follower does: velocity straight down at 0.5 m/s, pulling back over
      the start, then pressing down gently once PX4 reports landed
    wait on the ground ``ground_s``, still pressing down (motors at idle, not off: see below)
    take off again to height_m, hold
    land the same way, and keep pressing down until PX4 disarms (COM_DISARM_LAND)

Motors cannot actually stop between the two landings: stopping them is a disarm, a disarm ends
Offboard, and only the pilot can arm again. PX4 also disarms on its own once it has been landed
for COM_DISARM_LAND (2 s in the organisers' parameters), so ``ground_s`` must stay under that.

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


@dataclass
class Config:
    side_m: float = 0.5
    height_m: float = 0.73          # the real follower's setpoint z (cruise_z_m 0.80 - rest_height_m 0.07)
    speed_mps: float = 0.3
    lead_m: float = 0.3
    arrive_m: float = 0.10          # horizontal: the real follower's land_tolerance_m
    arrive_z_m: float = 0.15        # vertical: PX4 holds height by rangefinder, a few cm off cuVSLAM's z
    hold_s: float = 3.0
    ground_s: float = 1.0           # after PX4 says landed; must stay under COM_DISARM_LAND (2 s)
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
    counter-clockwise from the take-off heading, unwrapped)."""
    s, h = c.side_m, c.height_m
    box = [(0, 0, h), (s, 0, h), (s, s, h), (0, s, h), (0, 0, h)]
    steps = []
    for k, p in enumerate(box):
        steps += [('move', p, 0, None), ('hold', p, c.hold_s, 90.0 * k)]
    steps += [('land', None, 0, None), ('ground', None, c.ground_s, None),
              ('move', (0, 0, h), 0, None), ('hold', (0, 0, h), c.hold_s, 360.0),
              ('land', None, 0, None), ('disarm', None, 0, None)]
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

    @property
    def step_name(self):
        return self.steps[self.i][0] if self.state == FLY else self.state

    def step(self, t, pose, pose_t, offboard, landed, armed, yaw=None):
        """One tick. pose: (x, y, z) or None; pose_t: when it was captured (s); yaw: its ENU
        heading in radians, or None (then the drone holds whatever heading it has)."""
        c = self.cfg
        fresh = pose is not None and pose_t is not None and t - pose_t <= c.pose_timeout_s
        if self.state in (DONE, STOPPED):
            return NONE
        if self.state == WAIT:
            if not fresh:
                return NONE          # no stream: the kit will not enter Offboard
            if not offboard:
                return Command('pose', *pose)
            self.start, self.target, self.start_yaw, self.heading = pose, pose, yaw, 0.0
            self.state, self.i, self.since, self.last_t = FLY, 0, t, t
        elif armed is False and self.steps[self.i][0] in ('land', 'ground', 'disarm'):
            # checked first: PX4 disarming after touchdown also ends Offboard
            self.state = DONE
            self.reason = 'disarmed on the ground' if self.steps[self.i][0] == 'disarm' else \
                'PX4 disarmed after the first landing (ground wait longer than COM_DISARM_LAND?)'
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

        kind, offset, secs, heading = self.steps[self.i]
        self.reason = 'step {}/{}: {}'.format(self.i + 1, len(self.steps), kind)
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
        if kind == 'ground':
            if self.dwell_since is None:
                self.dwell_since = t
            if t - self.dwell_since >= secs:
                self.target = (self.start[0], self.start[1], pose[2])
                self._next(t)
            return Command('twist', 0.0, 0.0, -0.3 * c.descent_mps)
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
        return Command('pose', *self.target, yaw=None if self.start_yaw is None else self._yaw_cmd())

    def _at(self, offset):
        return tuple(s + o for s, o in zip(self.start, offset))

    def _next(self, t):
        self.i, self.since, self.dwell_since = self.i + 1, t, None

    def _abort(self, why):
        self.i = len(self.steps) - 1            # the final 'disarm' landing
        self.reason = why


def frames_disagree(odom_enu, px4_ned, limit_m=0.15):
    """Why PX4's local position and cuVSLAM's odometry disagree by more than ``limit_m``, or None.

    The kit's interface turns an odom ENU setpoint into PX4 NED by swapping x/y and negating z, so
    it is only right when both frames started in the same place. PX4 fuses vision for x, y and yaw
    only (EKF2_EV_CTRL 9) and takes height from the rangefinder (EKF2_HGT_REF 2): its height origin
    is where the flight controller powered up, the tracker's where the stack started. On
    skybeta-11 (2026-11-07) PX4 read -0.72 m on the floor while odom read 0.01 m, and the drone
    flew to 1.33 m for a 0.74 m setpoint. Power-cycling the drone on its take-off spot fixes it."""
    north, east, down = px4_ned
    x, y, z = odom_enu
    horizontal = math.hypot(east - x, north - y)
    vertical = abs(-down - z)
    if horizontal > limit_m:
        return 'PX4 position and cuVSLAM disagree by {:.2f} m'.format(horizontal)
    if vertical > limit_m:
        return 'PX4 height and cuVSLAM disagree by {:.2f} m'.format(vertical)
    return None


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(v, m):
    return max(-m, min(m, v))
