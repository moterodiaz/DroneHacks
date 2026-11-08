"""Offline dry run: the real Mission against a simple vehicle model. No ROS.

    python3 -m dronehacks_pathfly.sim                       # defaults
    python3 -m dronehacks_pathfly.sim --start-yaw-deg 90 --noise 0.01 --csv run.csv

The model is deliberately plain: PX4 tracks the commanded velocity with a
first-order lag and an acceleration limit, twists expire after 1 s (the
interface then holds), the land detector needs 1 s of ground contact with no
upward command, PX4 disarms 10 s after arming if it never took off and 5 s
after landing (COM_DISARM_LAND on drone 11). It checks the mission logic and
timing, not the airframe.
"""
import argparse
import csv
import math
import os
import random

from .mission import Config, Mission, Observation
from .path import load_path_csv, rotate_xy, twist_for_interface, wrap_pi

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(os.path.dirname(HERE), 'data', 'path_dense.csv')
if not os.path.exists(DEFAULT_PATH):        # installed: use the share copy
    try:
        from ament_index_python.packages import get_package_share_directory
        DEFAULT_PATH = os.path.join(
            get_package_share_directory('dronehacks_pathfly'), 'data',
            'path_dense.csv')
    except Exception:  # noqa: BLE001 - no ROS: keep the source-tree path
        pass


class Vehicle:
    def __init__(self, tau_xy=0.35, tau_z=0.3, accel_max=3.0, tau_yaw=0.3,
                 disarm_land_s=5.0, disarm_preflight_s=10.0, spool_s=0.5,
                 cap=1.5, wind=(0.0, 0.0), frame_yaw_deg=0.0):
        self.p = [0.0, 0.0, 0.0]          # odom ENU; origin = power-on pose
        self.v = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.tau_xy, self.tau_z, self.tau_yaw = tau_xy, tau_z, tau_yaw
        self.accel_max = accel_max
        self.cap = cap
        self.wind = wind
        # EKF2's heading minus the vision frame's: the interface's plain
        # ENU->NED swap then flies every twist rotated by this, in ENU.
        self.frame_yaw = math.radians(frame_yaw_deg)
        self.armed = False
        self.offboard = False
        self.landed = True
        self.ground_time = 0.0
        self.arm_time = None
        self.ever_flew = False
        self.disarm_land_s = disarm_land_s
        self.disarm_preflight_s = disarm_preflight_s
        self.spool_s = spool_s
        self.up_cmd_time = 0.0
        self.cmd = None
        self.cmd_time = -1e9

    def set_twist(self, t, twist):
        vx, vy, vz, wz = twist
        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n > self.cap:
            vx, vy, vz = (c * self.cap / n for c in (vx, vy, vz))
        vx, vy = rotate_xy(vx, vy, self.frame_yaw)
        self.cmd = (vx, vy, vz, wz)
        self.cmd_time = t

    def step(self, t, dt):
        if self.offboard and self.cmd is not None and t - self.cmd_time <= 1.0:
            vx, vy, vz, wz = self.cmd
        else:
            vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0   # interface hold / not flying
        if not self.armed:
            vx = vy = vz = wz = 0.0

        on_ground = self.p[2] <= 1e-6
        if on_ground and vz > 0.0:
            self.up_cmd_time += dt                 # spool-up before lift
            if self.up_cmd_time < self.spool_s:
                vz = 0.0
        elif not on_ground:
            self.up_cmd_time = 0.0
        if on_ground and vz <= 0.0:
            vx = vy = vz = 0.0
            self.v = [0.0, 0.0, 0.0]

        for i, (c, tau) in enumerate(((vx, self.tau_xy), (vy, self.tau_xy),
                                      (vz, self.tau_z))):
            a = (c - self.v[i]) / tau
            a = max(-self.accel_max, min(self.accel_max, a))
            self.v[i] += a * dt
        self.yaw_rate += (wz - self.yaw_rate) * dt / self.tau_yaw
        self.yaw = wrap_pi(self.yaw + self.yaw_rate * dt)
        airborne = self.p[2] > 0.0 or self.v[2] > 0.0
        drift = self.wind if airborne else (0.0, 0.0)
        self.p[0] += (self.v[0] + drift[0]) * dt
        self.p[1] += (self.v[1] + drift[1]) * dt
        self.p[2] = max(0.0, self.p[2] + self.v[2] * dt)
        if self.p[2] <= 0.0 and self.v[2] < 0.0:
            self.v[2] = 0.0

        # Land detector and auto-disarm.
        if self.p[2] <= 0.01 and vz <= 0.0:
            self.ground_time += dt
        else:
            self.ground_time = 0.0
        if self.p[2] > 0.05:
            self.landed = False
            self.ever_flew = True
        elif self.ground_time >= 1.0:
            self.landed = True
        if self.armed and self.landed and self.ever_flew \
                and self.ground_time >= 1.0 + self.disarm_land_s:
            self.disarm('COM_DISARM_LAND')
        if self.armed and not self.ever_flew and self.arm_time is not None \
                and t - self.arm_time > self.disarm_preflight_s:
            self.disarm('COM_DISARM_PRFLT')

    def disarm(self, why):
        self.armed = False
        self.offboard = False
        self.disarm_reason = why


def run(cfg, path_csv=DEFAULT_PATH, noise=0.0, wind=(0.0, 0.0), seed=1,
        arm_at=3.0, position_at=4.0, t_end=180.0, mission_hz=20.0,
        odom_hz=30.0, state_hz=5.0, quiet=False, drop_offboard_at=None,
        odom_gap=None, offboard_at_boot=False, frame_yaw_deg=0.0,
        yaw_offset_param_deg=0.0, divergence=None):
    # divergence: function t -> (dx, dy, dz) camera-minus-PX4 position gap
    # as /skybeta/state would report it; None = they agree exactly.
    """Fault hooks: drop_offboard_at (pilot takes over at that time),
    odom_gap=(t0, t1) (no odometry in between), offboard_at_boot (the node
    starts while Offboard is already active, e.g. restarted mid-flight)."""
    rng = random.Random(seed)
    pts, margins = load_path_csv(path_csv)
    mission = Mission(pts, margins, cfg)
    veh = Vehicle(wind=wind, frame_yaw_deg=frame_yaw_deg)
    offset_param = math.radians(yaw_offset_param_deg)
    if offboard_at_boot:
        veh.armed = veh.offboard = True
        veh.arm_time = 0.0
        veh.asked = True

    # Where the vehicle really is in the arena: on the start point, nose at
    # start_yaw_deg; odom is its power-on pose.
    a0 = mission.arena_start
    yaw_a = math.radians(mission.start_yaw_deg)   # resolved (NaN = auto)

    def to_arena(p):
        c, s = math.cos(yaw_a), math.sin(yaw_a)
        return (a0[0] + c * p[0] - s * p[1], a0[1] + s * p[0] + c * p[1])

    dt = 0.005
    t = 0.0
    last_odom = None
    odom_t = -1e9
    state = None
    state_t = -1e9
    next_mission = next_odom = next_state = 0.0
    phases = []
    track = []
    last_setpoint_t = -1e9
    commands = []
    while t < t_end:
        if drop_offboard_at is not None and t >= drop_offboard_at:
            veh.offboard = False
        gap = odom_gap is not None and odom_gap[0] <= t < odom_gap[1]
        if t >= next_odom:
            next_odom += 1.0 / odom_hz
            if not gap:
                last_odom = ([veh.p[i] + rng.gauss(0.0, noise) for i in range(3)],
                             wrap_pi(veh.yaw + rng.gauss(0.0, noise * 0.5)))
                odom_t = t
        if t >= next_state:
            next_state += 1.0 / state_hz
            state = {'offboard_active': veh.offboard, 'landed': veh.landed,
                     'armed': veh.armed,
                     'divergence': divergence(t) if divergence else (0.0, 0.0, 0.0)}
            state_t = t
        # Pilot: arm in Altitude, then switch to Position -> auto Offboard,
        # granted only on a fresh team setpoint while landed.
        if veh.arm_time is None and t >= arm_at:
            veh.armed = True
            veh.arm_time = t
        if veh.armed and not veh.offboard and not getattr(veh, 'asked', False) \
                and t >= position_at:
            veh.asked = True
            if t - last_setpoint_t < 1.0 and veh.landed:
                veh.offboard = True
        if t >= next_mission:
            next_mission += 1.0 / mission_hz
            obs = Observation(
                position=tuple(last_odom[0]) if last_odom else None,
                yaw=last_odom[1] if last_odom else None,
                odom_age_s=t - odom_t,
                offboard=state['offboard_active'] if state else None,
                landed=state['landed'] if state else None,
                state_age_s=t - state_t,
                divergence=state['divergence'] if state else None)
            cmd = mission.update(t, obs)
            for e in mission.events:
                if not quiet:
                    print(f'{t:7.2f}  {e}')
            mission.events.clear()
            if not phases or phases[-1][1] != mission.phase:
                phases.append((t, mission.phase))
            commands.append((t, cmd.phase, cmd.twist))
            if cmd.twist is not None:
                # what node.py publishes: the twist rotated by the parameter
                veh.set_twist(t, twist_for_interface(cmd.twist, offset_param))
                last_setpoint_t = t
            ax, ay = to_arena(veh.p)
            track.append((round(t, 3), mission.phase, ax, ay, veh.p[2],
                          math.degrees(wrap_pi(veh.yaw + yaw_a)), veh.armed))
        veh.step(t, dt)
        t += dt
        if mission.phase in ('ABORTED', 'COMPLETE') and not veh.armed:
            break
    run.commands = commands
    return mission, veh, phases, track


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--path', default=DEFAULT_PATH)
    ap.add_argument('--start-yaw-deg', type=float, default=Config.start_yaw_deg)
    ap.add_argument('--noise', type=float, default=0.0,
                    help='odometry position noise sigma, metres')
    ap.add_argument('--wind', type=float, nargs=2, default=(0.0, 0.0),
                    help='constant drift (odom vx vy) not seen by the command')
    ap.add_argument('--csv', help='write the flown track (arena frame) here')
    ap.add_argument('--frame-yaw-deg', type=float, default=0.0,
                    help='simulated vision-vs-EKF2 heading offset (deg)')
    ap.add_argument('--yaw-offset-param-deg', type=float, default=0.0,
                    help='odom_to_ned_yaw_offset_rad as set on the node, in deg')
    ap.add_argument('--set', nargs='*', default=[], metavar='NAME=VALUE',
                    help='override any Config field')
    args = ap.parse_args(argv)

    cfg = Config(start_yaw_deg=args.start_yaw_deg)
    for kv in args.set:
        k, v = kv.split('=', 1)
        if k not in Config.field_names():
            raise SystemExit(f'unknown field {k}')
        if isinstance(getattr(cfg, k), bool):
            setattr(cfg, k, v.strip().lower() in ('1', 'true', 'yes', 'on'))
        else:
            setattr(cfg, k, float(v))

    mission, veh, phases, track = run(cfg, args.path, args.noise,
                                      tuple(args.wind),
                                      frame_yaw_deg=args.frame_yaw_deg,
                                      yaw_offset_param_deg=args.yaw_offset_param_deg)
    print()
    print(mission.summary())
    print('phase timeline:')
    for i, (t, ph) in enumerate(phases):
        t1 = phases[i + 1][0] if i + 1 < len(phases) else track[-1][0]
        print(f'  {t:7.2f} - {t1:7.2f}  {ph}')
    print(f'final: phase {mission.phase}, armed {veh.armed} '
          f'({getattr(veh, "disarm_reason", "-")}), arena '
          f'({track[-1][2]:.2f}, {track[-1][3]:.2f}) z {track[-1][4]:.2f}, '
          f'nose {track[-1][5]:.1f} deg')
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'phase', 'x_arena', 'y_arena', 'z', 'yaw_arena_deg',
                        'armed'])
            w.writerows(track)
        print(f'wrote {args.csv}')


if __name__ == '__main__':
    main()
