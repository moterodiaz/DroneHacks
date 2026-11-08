"""The simulation engine: one drone-1 test session, from power-up to the end of a run.

Timeline (simulated seconds, ROS time = epoch0 + t), after ``host/run.sh stack``:

    0.0   PX4 boots (EKF2 on mag heading, rangefinder height); stack container starts
    1.5   MicroXRCEAgent session up: /fmu/* appear              (``agent`` window)
    2.0   px4_odom_bridge starts (kit launch file, unmodified)  (``bridge`` window)
    2.5   cuVSLAM starts tracking; odom = base_link here         (``slam`` window)
    4.0   ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH         (``competitor`` window)
   15.0   pilot arms on Altitude; 17.0 switches to Position      (the organizers' test)

Everything runs on one event heap (``dronesim.rosfake.runtime``): the 500 Hz
physics / PX4 loop, sensor frames, DDS deliveries, node timers and threads.
"""
from __future__ import annotations

import bisect
import collections
import copy
import json
import math
import os
import tempfile
import time as _time
from dataclasses import asdict, dataclass, field

from . import rosfake
from .mathutil import q_from_euler, q_to_euler, q_yaw
from .world.course import ned_to_course

SITL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(SITL_ROOT)
EPOCH0 = 1794157200.0          # 2026-11-08 17:00 UTC (scored-run Sunday, 09:00 PST)

#: drone.env keys host/drone_env.sh passes into the container (anything else, e.g. DRONEHACKS_*, never arrives)
CONTAINER_ENV = {'DRONE_ID': '1', 'ROS_DOMAIN_ID': '1', 'SKYBETA_DDS_SCOPE': 'local', 'TEAM': 'unassigned',
                 'CAMERA_X': '0.1053', 'CAMERA_Y': '0', 'CAMERA_Z': '0.0304', 'CAMERA_ROLL': '0',
                 'CAMERA_PITCH': '0', 'CAMERA_YAW': '0', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                 'ROS_DISTRO': 'humble', 'ROS_VERSION': '2', 'ROS_PYTHON_VERSION': '3'}


@dataclass
class Scenario:
    version: str = 'traj'
    follower: str = None
    options: dict = field(default_factory=dict)
    param_profile: str = 'drone1-fleet'
    param_overrides: dict = field(default_factory=dict)
    # PX4's horizontal speed cap, MPC_XY_VEL_MAX (m/s); None = the profile's value. The install
    # config's $v_max (a version's own speed limit, e.g. dronehacks_traj's v_max_mps) follows it.
    speed_cap_mps: float = None
    # the autonomy's own speeds, where a version has them (versions.yaml ``tunables``); None = its default
    track_speed_mps: float = None      # how fast it flies the route
    descent_mps: float = None          # how fast it descends onto the pad when landing
    seed: int = 1
    duration_s: float = 240.0
    stop_after_end_s: float = 8.0
    # start pose: on the launch mark, nose along heading (clockwise from course north)
    start_offset_m: tuple = (0.0, 0.0)
    start_heading_deg: float = 0.0
    # what the "operator" measured for the install config: truth + these errors
    calib_pos_error_m: float = 0.0
    calib_yaw_error_deg: float = 0.0
    calib_rest_height_error_m: float = 0.0
    vo: dict = field(default_factory=dict)
    lidar: dict = field(default_factory=dict)
    vehicle: dict = field(default_factory=dict)
    sensors: dict = field(default_factory=dict)
    pilot_arm_at_s: float = 15.0
    pilot_position_after_s: float = 2.0
    pilot_script: bool = True
    # the optional return (a version whose options enable it): the pilot re-arms this long after
    # the outbound pad disarm; None: the pilot never does
    pilot_rearm_for_return_s: float = 3.0
    faults: list = field(default_factory=list)
    env: dict = field(default_factory=dict)
    agent_start_s: float = 1.5
    bridge_start_s: float = 2.0
    competitor_start_s: float = 4.0
    wind_mps: tuple = (0.0, 0.0)
    # the course flown in (world.arena.ARENAS): 'step' (the STEP as exported), 'taped-2026-11-08'
    # (the course as the team taped it), or None = auto: the arena a chosen option is planned on
    # (versions.yaml choice key ``arena``), else 'step'
    arena: str = None

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        s = cls()
        for k, v in (d or {}).items():
            if hasattr(s, k):
                setattr(s, k, v)
        return s


class TruthHistory:
    def __init__(self, keep_s=1.5):
        self.t = collections.deque()
        self.items = collections.deque()
        self.keep = keep_s

    def add(self, t, pos, q):
        self.t.append(t)
        self.items.append((pos, q))
        while self.t and t - self.t[0] > self.keep:
            self.t.popleft()
            self.items.popleft()

    def at(self, t):
        if not self.t:
            return None
        ts = self.t
        if t <= ts[0]:
            return self.items[0]
        if t >= ts[-1]:
            return self.items[-1]
        i = bisect.bisect_left(ts, t)
        a, b = self.items[i - 1], self.items[i]
        f = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0.0
        pos = tuple(a[0][k] + f * (b[0][k] - a[0][k]) for k in range(3))
        return pos, (a[1] if f < 0.5 else b[1])


class Simulation:
    DT = 1.0 / 500.0

    def __init__(self, scenario, repo_root=REPO_ROOT, cache_dir=None, log_sink=None, catalog_path=None):
        self.sc = scenario
        self.repo_root = os.path.abspath(repo_root)
        self.cache_dir = cache_dir or os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'),
                                                   'dronesim', 'cache')
        os.makedirs(self.cache_dir, exist_ok=True)
        self.log_sink = log_sink
        self.logs = collections.deque(maxlen=20000)
        self.run_notes = []
        self._log_buffer = []
        from .autonomy import catalog as cat
        self.catalog = cat.load_catalog(catalog_path or os.path.join(SITL_ROOT, 'versions.yaml'))
        if scenario.version not in self.catalog:
            raise cat.VersionError(f'unknown version {scenario.version!r}; known: {sorted(self.catalog)}')
        self.version = self.catalog[scenario.version]
        # sources first: git runs in real time, before the simulated clock exists
        self.package_dirs, self.provenance = cat.materialize(self.repo_root, self.version, self.cache_dir)
        for key in ('track_speed_mps', 'descent_mps'):
            value = getattr(scenario, key)
            if value is not None and not (math.isfinite(float(value)) and float(value) > 0.0):
                raise ValueError(f'{key} must be a positive number of m/s (got {value!r})')
        self.launch, self.version_params, self.hook, notes = cat.resolve_selection(
            self.version, scenario.options, scenario.follower,
            {'track_speed_mps': scenario.track_speed_mps, 'descent_mps': scenario.descent_mps})
        self.run_notes += notes
        self.arena, why = cat.resolve_arena(self.version, scenario.options, scenario.arena)
        from .world.arena import arena_label
        self.run_notes.append(f'Arena: {arena_label(self.arena)} ({why})')
        #: what the version planned, captured by an observer (autonomy/observers.py); None if none
        self.plan = None
        self._build()

    # ------------------------------------------------------------------------------------------
    def _log(self, record):
        self.logs.append(record)
        if self.log_sink is not None:
            self.log_sink(record)

    def _build(self):
        sc = self.sc
        rosfake.activate()
        rosfake.patch_globals()
        from .px4.params import load_profile
        from .px4.px4 import PX4
        from .px4.uxrce import UxrceBridge
        from .rosfake import devices, launch_impl, packages, runtime
        from .scoring.referee import Referee
        from .sensors.ms200 import MS200Device
        from .sensors.oak import OakSide
        from .sensors.vo import CuVslamModel
        from .vehicle.contacts import ContactModel
        from .vehicle.physics import AirframeConfig, QuadPhysics
        from .world.course import Course, course_to_ned
        from .pilot import PilotScript
        from .autonomy import catalog as cat
        from .autonomy import hooks as hooks_mod

        self.rt = rt = runtime.SimRuntime(epoch0=EPOCH0, seed=sc.seed)
        rt.log_sinks.append(self._log)
        rng = rt.rng
        # container environment (drone.env whitelist)
        for k in list(os.environ):
            if k.startswith('DRONEHACKS_'):
                del os.environ[k]
        os.environ.update(CONTAINER_ENV)
        os.environ.update({k: str(v) for k, v in (sc.env or {}).items()})
        # world, PX4 parameters, airframe
        from .world.arena import edits_path
        self.course = Course(os.path.join(self.repo_root, 'dronehacks-course-v3.step'), self.cache_dir,
                             edits_path(self.arena))
        overrides = scenario_param_overrides(sc)
        if sc.speed_cap_mps is not None:
            self.run_notes.append(f"speed cap MPC_XY_VEL_MAX = {overrides['MPC_XY_VEL_MAX']:g} m/s (scenario)")
        self.params = load_profile(self.repo_root, sc.param_profile, overrides)
        self.airframe = AirframeConfig.from_params(self.params, sc.vehicle.get('airframe'))
        # start pose
        launch_xy = self.course.centre_xy('Launch mark')
        sx, sy = launch_xy[0] + sc.start_offset_m[0], launch_xy[1] + sc.start_offset_m[1]
        support, _ = self.course.support_below(sx, sy, 0.5)
        z_c = (support if math.isfinite(support) else 0.0) + self.airframe.rest_height_m
        pos_ned = course_to_ned((sx, sy, z_c))
        yaw_ned = math.radians(sc.start_heading_deg)
        q0 = q_from_euler(0.0, 0.0, yaw_ned)
        self.contacts = ContactModel(self.course, self.airframe)
        self.physics = QuadPhysics(self.airframe, pos_ned, q0, contact_model=self.contacts)
        self.wind = tuple(sc.wind_mps)
        self.px4 = PX4(self.params, self.physics, self.course, rng, log_fn=self._px4_log, sensor_cfg=sc.sensors)
        msg_dir = os.path.join(os.path.dirname(rosfake.__file__), 'interfaces', 'px4_msgs', 'msg')
        self.uxrce = UxrceBridge(rt, self.px4, msg_dir, session_start_s=sc.agent_start_s)
        self.history = TruthHistory()
        self.vo = CuVslamModel(rt, self.physics, self.course, rng, sc.vo,
                               camera_flu=self.airframe.camera_xyz_flu)
        # EKF2 drops a vision sample older than its delay buffer (EKF2_DELAY_MAX) once EKF2_EV_DELAY
        # is taken off its capture time: past that latency PX4 has no position at all
        window = (self.params.f('EKF2_DELAY_MAX') - self.params.f('EKF2_EV_DELAY')) / 1000.0 + 0.05
        lat = self.vo.cfg['latency_mean_s']
        if lat + 2.0 * self.vo.cfg['latency_std_s'] > window:
            self.run_notes.append(
                f'WARNING cuVSLAM latency {lat:.3f} s is past EKF2\'s delay window ({window:.3f} s = '
                f'EKF2_DELAY_MAX - EKF2_EV_DELAY + 50 ms): PX4 drops vision samples as too old'
                + (' -- every one, so it has no position' if lat > window else ''))
        self.oak = OakSide(rt, self.physics, rng, camera_pose=(self.airframe.camera_xyz_flu[0],
                                                               self.airframe.camera_xyz_flu[1],
                                                               self.airframe.camera_xyz_flu[2], 0.0, 0.0, 0.0))
        devices.clear()
        self.lidar = None
        if 'ms200' in (self.version.get('devices') or []) or sc.lidar.get('always'):
            self.lidar = MS200Device(rt, self.physics, self.course, rng, sc.lidar, history=self.history.at)
            devices.register(sc.lidar.get('port', '/dev/ms200'), self.lidar)
        self.pilot = PilotScript(self.px4.rc, arm_at_s=sc.pilot_arm_at_s, position_after_s=sc.pilot_position_after_s,
                                 enabled=sc.pilot_script)
        self.referee = Referee(self.course, os.path.join(SITL_ROOT, 'course', 'course_v3_rules.json'), self.airframe)
        # a version's options can enable the optional return (its follower's return_flight)
        self.return_enabled = any(bool(v.get('return_flight')) for v in (self.version_params or {}).values())
        self._rearm_scheduled = False
        if self.return_enabled:
            self.run_notes.append('optional return enabled: the scripted pilot re-arms '
                                  f'{self.sc.pilot_rearm_for_return_s} s after the pad disarm')
        # workspace: kit underlay + team overlay, built like colcon.  One fresh directory per
        # simulation (the pid alone is reused by Windows and by GUI runs), removed when the run
        # closes; trees left by crashed runs are removed once they are a few hours old.
        ws_root = os.path.join(self.cache_dir, 'ws')
        _clean_old_workspaces(ws_root)
        os.makedirs(ws_root, exist_ok=True)
        self.ws = packages.Workspace(tempfile.mkdtemp(dir=ws_root, prefix=f'run{os.getpid()}-'))
        packages.set_active(self.ws)
        self.ws.register_interfaces()
        self.ws.register_native('tf2_ros', {'static_transform_publisher': _static_tf_main})
        for d in cat.kit_package_dirs(self.repo_root):
            self.ws.build_python_package(d, origin='organizer kit (drone-snapshot)')
        self.team_packages = []
        for d in self.package_dirs:
            pkg = self.ws.build_python_package(d, origin=self.provenance)
            self.team_packages.append(pkg.name)
        # measured values for the install config
        self.subs = self._measured_values(sx, sy, yaw_ned, rng)
        shares = {p: self.ws.get(p).share for p in self.ws.packages}
        self.config_written = cat.apply_install_config(self.version, shares, self.subs, self.version_params)
        # launch sessions
        self.bridge_session = launch_impl.LaunchSession(self.ws)
        self.session = launch_impl.LaunchSession(self.ws)
        if self.hook:
            name, args = self.hook
            fn = hooks_mod.HOOKS[name]
            exe = {'traj_follower_config': 'mission', 'traj_waypoint_follower': 'mission'}[name]
            self.session.pre_main[(self.launch['package'], exe)] = [lambda fn=fn, args=args: fn(**args)]
        # observers (simulator-only, change nothing): record what a version planned, for the map
        from .autonomy import observers
        observers.set_sink(self._on_plan)
        for (pkg, exe), obs in observers.OBSERVERS.items():
            if pkg in self.team_packages:
                self.session.pre_main.setdefault((pkg, exe), []).append(obs)
        rt.at(int(sc.bridge_start_s * 1e9), self._start_bridge)
        rt.at(int(sc.competitor_start_s * 1e9), self._start_competitor)
        self._n = 0
        self._fault_events = sorted(sc.faults or [], key=lambda f: f.get('t', 0))
        for f in self._fault_events:
            rt.at(int(float(f.get('t', 0)) * 1e9), self._fault, f)
        rt.at(int(self.DT * 1e9), self._tick)
        self.trajectory = []        # sampled at 20 Hz for plots / reports
        self.finished_at = None
        self.started_wall = _time.perf_counter
        self._ext_force = (0.0, 0.0, 0.0)
        self._ext_force_until = -1.0
        self.status_text = 'booting'
        self._log({'t': 0.0, 'process': None, 'logger': 'dronesim', 'level': 'INFO', 'source': 'sim',
                   'msg': f"version {self.version['id']}: {self.provenance}; " + '; '.join(self.run_notes)})

    def _on_plan(self, record):
        """An observer's record of the plan the version made (autonomy/observers.py)."""
        record = dict(record)
        record['t'] = round(self.rt.now_s, 3) if getattr(self, 'rt', None) else None
        self.plan = record
        self.plan_rev = getattr(self, 'plan_rev', 0) + 1
        if 'error' in record:
            text = f"plan observer: {record['error']}"
        else:
            text = (f"plan observed: {' > '.join(record['goals'])}, {record['length_m']:.2f} m, "
                    f"{record['duration_s']:.2f} s at v_max {record['v_max_mps']:g} m/s, "
                    f"height {record['slice_height_m']:g} m (drawn on the map; plan.json when saved)")
        self._log({'t': self.rt.now_s, 'process': None, 'logger': 'dronesim', 'level': 'INFO', 'source': 'sim',
                   'msg': text})

    def plan_for_display(self):
        """The observed plan, thinned for the GUI map (None before one is made)."""
        if not self.plan or 'xy' not in self.plan:
            return None
        from .autonomy.observers import display_points
        p = self.plan
        return {'rev': getattr(self, 'plan_rev', 0), 'xy': display_points(p), 'goals': p.get('goals'),
                'length_m': p.get('length_m'), 'duration_s': p.get('duration_s'), 'v_max_mps': p.get('v_max_mps'),
                'waypoints': [w['xy'] for w in p.get('waypoints') or []]}

    def _measured_values(self, sx, sy, yaw_ned, rng):
        sc = self.sc
        e_pos = sc.calib_pos_error_m
        e_yaw = math.radians(sc.calib_yaw_error_deg)
        # the odom frame: base_link at startup; its +x = the nose (course heading CCW from east)
        psi = math.pi / 2 - yaw_ned
        psi_m = psi + (rng.gauss(0, e_yaw) if e_yaw > 0 else 0.0)
        ox = sx + (rng.gauss(0, e_pos) if e_pos > 0 else 0.0)
        oy = sy + (rng.gauss(0, e_pos) if e_pos > 0 else 0.0)
        # contracts/frames.md: p_odom = o + R p_course, with R = rotation by -psi
        yaw_r = -psi_m
        c, s = math.cos(yaw_r), math.sin(yaw_r)
        o_x = -(c * ox - s * oy)
        o_y = -(s * ox + c * oy)
        launch = self.course.centre_xy('Launch mark')
        dx, dy = launch[0] - ox, launch[1] - oy
        lx, ly = (c * dx - s * dy), (s * dx + c * dy)
        rest = self.airframe.rest_height_m + (rng.gauss(0, sc.calib_rest_height_error_m)
                                              if sc.calib_rest_height_error_m > 0 else 0.0)
        v = {
            'rest_height_m': round(float(rest), 4), 'v_max': float(self.params.f('MPC_XY_VEL_MAX')),
            'traj_yaw': round(float(yaw_r), 6), 'traj_x': round(float(o_x), 4), 'traj_y': round(float(o_y), 4),
            'odom_yaw_deg': round(float(math.degrees(math.pi / 2 - psi_m)) % 360.0, 4),
            'odom_origin_x': round(float(lx), 4), 'odom_origin_y': round(float(ly), 4),
        }
        if v['odom_yaw_deg'] > 180:
            v['odom_yaw_deg'] -= 360.0
        return v

    # -- stack ----------------------------------------------------------------------------------------
    def _start_bridge(self):
        self._log({'t': self.rt.now_s, 'process': None, 'logger': 'bridge', 'level': 'INFO', 'source': 'sim',
                   'msg': '[bridge] ros2 launch oakd_vslam_bringup px4_bridge.launch.py'})
        self.bridge_session.run('oakd_vslam_bringup', 'px4_bridge.launch.py', {})

    def _start_competitor(self):
        pkg, f = self.launch['package'], self.launch['file']
        self._log({'t': self.rt.now_s, 'process': None, 'logger': 'competitor', 'level': 'INFO', 'source': 'sim',
                   'msg': f'[competitor] ros2 launch {pkg} {f}'})
        self.session.run(pkg, f, dict(self.launch.get('args') or {}))

    def _px4_log(self, severity, text):
        self._log({'t': self.rt.now_s, 'process': None, 'logger': 'px4', 'level': severity, 'source': 'px4',
                   'msg': text})

    # -- the 500 Hz loop ---------------------------------------------------------------------------------
    def _tick(self):
        rt = self.rt
        dt = self.DT
        rt.after(dt, self._tick)
        t = rt.now_s
        px4 = self.px4
        n = self._n = self._n + 1
        if n % 10 == 0:
            self.pilot.update(t)
        px4.step_fast(t, dt)
        self.physics.ext_force_world = self._external_force(t)
        self.physics.step(dt, px4.motor_cmds, px4.commander.armed)
        s = self.physics.state
        self.history.add(t, s.pos, s.q)
        if n % 5 == 0:
            px4.step_100(t, 0.01)
        if n % 10 == 0:
            px4.step_50(t, 0.02)
            px4.step_baro(t, 0.02)
            self._referee(t)
        if n % 100 == 0:
            px4.step_range(t)
        if n % 25 == 0:
            self.trajectory.append(self._sample(t))

    def _external_force(self, t):
        """Wind and pushes as a force on the airframe (N, NED). The physics adds it to the force
        sum, so the motion it causes is in the IMU's specific force, as on a real drone."""
        s = self.physics.state
        fx, fy, fz = 0.0, 0.0, 0.0
        if self.wind != (0.0, 0.0):
            # wind as a drag force on the relative air velocity (course east/north -> NED)
            wn, we = self.wind[1], self.wind[0]
            fx += 0.3 * (wn - s.vel[0])
            fy += 0.3 * (we - s.vel[1])
        if t < self._ext_force_until:
            fx += self._ext_force[0]
            fy += self._ext_force[1]
            fz += self._ext_force[2]
        return (fx, fy, fz)

    def _referee(self, t):
        s = self.physics.state
        c = ned_to_course(s.pos)
        roll, pitch, yaw = q_to_euler(s.q)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        low = c[2] - self.airframe.rest_height_m
        cm = self.px4.commander
        info = self.physics.last_contact or {'feet': [], 'body': []}
        self.referee.update(t, c, math.pi / 2 - yaw, low, cm.armed, self.px4.land_detector.landed,
                            info['body'], info['feet'], tilt)
        for vt, text in self.uxrce.violations:
            self.referee.add_violation(vt, text)
        if cm.acro_violation:
            self.referee.add_violation(cm.acro_violation[0], 'Acro mode requested (AGENTS.md section 3.6)')
        if self.referee.ended and self.finished_at is None:
            self.finished_at = t
        if self.referee.t_finish and self.return_enabled:
            # the run goes on for the return (rule 7.10: the run ends when the return ends)
            if not self._rearm_scheduled and self.sc.pilot_rearm_for_return_s is not None:
                self._rearm_scheduled = True
                self.pilot.rearm_for_return(self.referee.t_finish + float(self.sc.pilot_rearm_for_return_s))
            if self.referee.return_over and self.finished_at is None:
                self.finished_at = t
        elif self.referee.t_finish and self.finished_at is None:
            self.finished_at = t

    def _sample(self, t):
        s = self.physics.state
        c = ned_to_course(s.pos)
        lp = self.px4.lpos_out or {}
        vo = self.vo.last_odom
        return (round(t, 3), round(c[0], 4), round(c[1], 4), round(c[2], 4),
                round(math.pi / 2 - q_yaw(s.q), 4),
                round(math.sqrt(s.vel[0] ** 2 + s.vel[1] ** 2), 3), self.px4.commander.nav_state,
                int(self.px4.commander.armed), round(lp.get('x', float('nan')), 4), round(lp.get('y', float('nan')), 4),
                round(-lp.get('z', float('nan')), 4), (round(vo[1], 4), round(vo[2], 4), round(vo[3], 4)) if vo else None)

    # -- faults / commands ------------------------------------------------------------------------------
    def _fault(self, f):
        self.command(dict(f))

    def command(self, cmd):
        """Pilot inputs and fault injection (GUI, scenario ``faults``)."""
        rt = self.rt
        t = rt.now_s
        what = cmd.get('action')
        if what == 'pilot':
            self.pilot.command(t, cmd['what'], cmd.get('value'))
        elif what == 'vo_lost':
            self.vo.cfg['force_lost'] = True
            dur = cmd.get('duration')
            if dur:
                rt.after(float(dur), lambda: self.vo.cfg.__setitem__('force_lost', False))
        elif what == 'vo_restore':
            self.vo.cfg['force_lost'] = False
        elif what == 'lidar':
            if self.lidar is not None:
                self.lidar.set_enabled(cmd.get('value', True), t)
        elif what == 'push':
            fx, fy, fz = cmd.get('force_n', (3.0, 0.0, 0.0))
            self._ext_force = course_to_ned_vec((fx, fy, fz))
            self._ext_force_until = t + float(cmd.get('duration', 0.5))
        elif what == 'wind':
            self.wind = tuple(cmd.get('value', (0.0, 0.0)))
        elif what == 'kill_node':
            pattern = cmd.get('name', '')
            for p in list(self.rt.processes):
                if p.alive and pattern and pattern in p.name:
                    self.rt.kill_process(p)
                    self._log({'t': t, 'process': p.name, 'logger': 'dronesim', 'level': 'WARN', 'source': 'sim',
                               'msg': f'fault: killed process {p.name} (SIGKILL)'})
        else:
            raise ValueError(f'unknown command {cmd}')
        self._log({'t': t, 'process': None, 'logger': 'dronesim', 'level': 'INFO', 'source': 'sim',
                   'msg': f'command: {json.dumps(cmd)}'})

    # -- running ------------------------------------------------------------------------------------------
    @property
    def t(self):
        return self.rt.now_s

    def step(self, dt_s):
        self.rt.run_until(self.rt.now_ns + int(dt_s * 1e9))

    def done(self):
        if self.t >= self.sc.duration_s:
            return True
        return self.finished_at is not None and self.t - self.finished_at >= self.sc.stop_after_end_s

    def run(self, until_s=None, realtime=None, progress=None):
        """Run to ``until_s`` (default: scenario end). ``realtime`` = speed factor, None = as fast as possible."""
        from .rosfake.runtime import REAL_PERF, REAL_SLEEP
        end = until_s if until_s is not None else self.sc.duration_s
        wall0, sim0 = REAL_PERF(), self.t
        while self.t < end and not self.done():
            self.step(0.02)
            if realtime:
                ahead = (self.t - sim0) / realtime - (REAL_PERF() - wall0)
                if ahead > 0:
                    REAL_SLEEP(ahead)
            if progress is not None and self._n % 500 == 0:
                progress(self)
        return self.result()

    def snapshot(self):
        """Telemetry for the GUI (plain data, picklable)."""
        s = self.physics.state
        c = ned_to_course(s.pos)
        roll, pitch, yaw = q_to_euler(s.q)
        px4, cm = self.px4, self.px4.commander
        lp = px4.lpos_out or {}
        from .px4.commander import NAV_NAMES
        vo = self.vo
        vf = vo.odom_frame()
        traj = px4.traj_sp
        st = {
            't': round(self.t, 3),
            'true': {'x': c[0], 'y': c[1], 'z': c[2], 'yaw': math.pi / 2 - yaw, 'roll': roll, 'pitch': pitch,
                     'speed': math.hypot(s.vel[0], s.vel[1]), 'vz': -s.vel[2]},
            'motors': list(s.motors), 'battery_v': self.physics.battery.voltage(), 'battery_soc': self.physics.battery.soc,
            'px4': {'armed': cm.armed, 'nav_state': cm.nav_state, 'nav_name': NAV_NAMES.get(cm.nav_state, str(cm.nav_state)),
                    'intended': NAV_NAMES.get(cm.user_intended, str(cm.user_intended)), 'failsafe': cm.failsafe,
                    'failsafe_reasons': list(cm.failsafe_reasons), 'landed': px4.land_detector.landed,
                    'takeoff_state': px4.takeoff.state, 'rc_lost': cm.rc_lost, 'kill': cm.kill_since is not None,
                    'hover_thrust': px4.hover_thrust(), 'thrust': -px4.att_sp_thrust,
                    'ekf': {'xy_valid': lp.get('xy_valid'), 'z_valid': lp.get('z_valid'), 'eph': lp.get('eph'),
                            'x': lp.get('x'), 'y': lp.get('y'), 'z': lp.get('z'), 'heading': lp.get('heading'),
                            'ev': px4.ekf.ev_active, 'ev_status': px4.ev_status, 'ev_counts': dict(px4.ev_counts),
                            'rng_ref': px4.ekf.rng_is_ref,
                            'dist_bottom': lp.get('dist_bottom'), 'dead_reckoning': lp.get('dead_reckoning')},
                    'setpoint': ({'pos': traj['position'], 'vel': traj['velocity'], 'yaw': traj['yaw'],
                                  'age': round(px4.t - traj['t'], 2)} if traj else None),
                    'offboard_age': round(px4.t - px4.offboard_mode['t'], 2) if px4.offboard_mode else None,
                    'messages': [(round(a, 2), b, c_) for a, b, c_ in cm.messages[-12:]]},
            'vo': {'state': vo.vo_state, 'tracking': vo.tracking, 'score': round(vo.feature_score, 2),
                   'odom': vo.last_odom, 'frame': ({'origin': vf[0], 'yaw': vf[1]} if vf else None),
                   'lost_events': vo.lost_events},
            'lidar': ({'scan': [(round(a, 3), (round(r, 3) if r is not None else None)) for a, r in
                                self.lidar.last_scan[::3]], 'frames': self.lidar.frames_sent,
                       'open': self.lidar.is_open} if self.lidar else None),
            'contacts': [(ct.kind, ct.name, ct.part) for ct in self.contacts.contacts][:8],
            'referee': self.referee.summary(),
            'events': [(round(a, 2), b) for a, b in self.referee.events[-20:]],
            'processes': [{'name': p.name, 'alive': p.alive, 'exit': p.exit_code} for p in self.rt.processes],
            'pilot': {'arm_switch': self.px4.rc.arm_switch, 'mode_slot': self.px4.rc.mode_slot,
                      'throttle': self.px4.rc.throttle, 'link': self.px4.rc.link, 'kill': self.px4.rc.kill_switch},
        }
        return st

    def odom_to_course(self, x, y):
        """A point in cuVSLAM's odom frame (ENU labels) -> course frame, using the true odom origin."""
        vf = self.vo.odom_frame()
        if vf is None or not (math.isfinite(x) and math.isfinite(y)):
            return None
        (ox, oy, _oz), yaw = vf
        c, s = math.cos(yaw), math.sin(yaw)
        return (ox + c * x - s * y, oy + s * x + c * y)

    def estimates_in_course(self):
        """Where PX4 (EKF2), cuVSLAM and the active setpoint put the drone, drawn on the course.

        PX4's local frame is the bridge's NED view of odom: (x_ned, y_ned) = (y_odom, x_odom).
        """
        out = {}
        lp = self.px4.lpos_out
        if lp and lp.get('xy_valid') is not None:
            out['ekf'] = self.odom_to_course(lp['y'], lp['x'])
            out['ekf_valid'] = bool(lp['xy_valid'])
        if self.vo.last_odom:
            out['vo'] = self.odom_to_course(self.vo.last_odom[1], self.vo.last_odom[2])
        sp = self.px4.traj_sp
        if sp and math.isfinite(sp['position'][0]):
            out['setpoint'] = self.odom_to_course(sp['position'][1], sp['position'][0])
            out['setpoint_z'] = -sp['position'][2]
        return out

    def topics(self):
        g = self.rt.graph
        out = []
        for name, types in g.topic_names_and_types():
            pubs = len([p for p in g.pubs.get(name, ()) if p.alive])
            subs = len([s for s in g.subs.get(name, ()) if s.alive])
            out.append({'topic': name, 'type': types[0] if types else '', 'pubs': pubs, 'subs': subs,
                        'hz': round(g.rate_hz(name), 1)})
        return out

    def echo(self, topic):
        st = self.rt.graph.stats.get(topic)
        if st is None or st.last_msg is None:
            return None
        try:
            from rosidl_runtime_py import message_to_yaml
            return message_to_yaml(st.last_msg)
        except Exception:  # noqa: BLE001
            return repr(st.last_msg)

    def result(self):
        err = sorted(self.vo.errors)
        vision = {'fusion': dict(self.px4.ev_counts),
                  'cuvslam_error_median_m': round(err[len(err) // 2], 3) if err else None,
                  'cuvslam_error_max_m': round(err[-1], 3) if err else None,
                  'cuvslam_losses': self.vo.lost_events}
        return {
            'version': self.version['id'], 'label': self.version['label'], 'provenance': self.provenance,
            'notes': list(self.run_notes), 'vision': vision, 'scenario': self.sc.to_dict(), 'param_profile': self.params.profile,
            'sim_time_s': round(self.t, 2), 'referee': self.referee.summary(),
            'events': [(round(a, 2), b) for a, b in self.referee.events],
            'install_config': [{'file': f, 'node': n, 'values': v} for f, n, v in self.config_written],
            'measured': dict(self.subs),
            'processes': [{'name': p.name, 'alive': p.alive, 'exit_code': p.exit_code} for p in self.rt.processes],
            'px4_messages': [(round(a, 2), b, c) for a, b, c in self.px4.commander.messages],
            'violations': [{'t': round(t, 2), 'text': x} for t, x in self.referee.violations],
            'arena': {'name': self.arena, **{k: v for k, v in self.course.describe().items()
                                             if k in ('arena', 'edits', 'edits_sha256')}},
            'plan': ({k: v for k, v in self.plan.items() if k not in ('xy', 't', 'speed_mps')}
                     if self.plan else None),
        }

    def save(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, 'result.json'), 'w', encoding='utf-8') as fh:
            json.dump(self.result(), fh, indent=1, default=str)
        with open(os.path.join(out_dir, 'log.txt'), 'w', encoding='utf-8') as fh:
            for r in self.logs:
                fh.write(format_log(r) + '\n')
        with open(os.path.join(out_dir, 'trajectory.csv'), 'w', encoding='utf-8') as fh:
            fh.write('t,x,y,z,yaw,speed,nav_state,armed,ekf_x,ekf_y,ekf_up,vo\n')
            for row in self.trajectory:
                fh.write(','.join(str(v) for v in row[:-1]) + ',' + (' '.join(map(str, row[-1])) if row[-1] else '') + '\n')
        if self.plan:
            # the version's own plan in the course frame, as the observer recorded it
            with open(os.path.join(out_dir, 'plan.json'), 'w', encoding='utf-8') as fh:
                json.dump(self.plan, fh, indent=1, default=str)
        return out_dir

    def close(self):
        try:
            self.rt.close()
        finally:
            from .rosfake import devices, packages
            devices.clear()
            self.ws.close()
            packages.set_active(None)
            import shutil
            shutil.rmtree(self.ws.root, ignore_errors=True)


def scenario_param_overrides(sc):
    """The PX4 parameter overrides a scenario asks for: its own, then the speed cap on top."""
    overrides = dict(sc.param_overrides or {})
    if sc.speed_cap_mps is not None:
        cap = float(sc.speed_cap_mps)
        if not (math.isfinite(cap) and cap > 0.0):
            raise ValueError(f'speed cap must be a positive number of m/s (got {sc.speed_cap_mps!r})')
        overrides['MPC_XY_VEL_MAX'] = cap
    return overrides


def course_to_ned_vec(v):
    return (v[1], v[0], -v[2])


def _clean_old_workspaces(root, max_age_s=6 * 3600):
    """Remove build trees that crashed runs left behind, once they are ``max_age_s`` old.

    Ages are measured on the real clock.  ``time.time`` is the simulated clock once a
    runtime exists (the epoch is a day ahead), which made every other run's tree look
    old, including the live trees of simulations running in parallel.
    """
    import shutil
    from .rosfake.runtime import REAL_TIME
    if not os.path.isdir(root):
        return
    now = REAL_TIME()
    for name in os.listdir(root):
        path = os.path.join(root, name)
        if name.startswith('run') and os.path.isdir(path):
            try:
                if now - os.path.getmtime(path) > max_age_s:
                    shutil.rmtree(path, ignore_errors=True)
            except OSError:
                pass


def format_log(r):
    proc = r.get('process')
    stamp = r.get('stamp_ns')
    t = f"{r['t']:8.3f}"
    if r.get('source') in ('ros',):
        return f"{t} [{proc or '-'}] [{r['level']}] [{(stamp or 0) / 1e9:.6f}] [{r['logger']}]: {r['msg']}"
    if r.get('source') in ('stdout', 'stderr'):
        return f"{t} [{proc or '-'}] {r['msg']}"
    return f"{t} [{r['logger']}] [{r['level']}] {r['msg']}"


def _static_tf_main():
    """tf2_ros static_transform_publisher (Humble argument forms)."""
    import sys

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    argv = [a for a in sys.argv[1:]]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    vals = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'frame-id': 'world',
            'child-frame-id': 'child'}
    if argv and argv[0].startswith('--'):
        for i in range(0, len(argv) - 1, 2):
            vals[argv[i][2:]] = argv[i + 1]
    elif len(argv) >= 8:
        vals.update(dict(zip(['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'frame-id', 'child-frame-id'], argv[:8])))
    rclpy.init()
    node = rclpy.create_node('static_transform_publisher')
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = vals['frame-id']
    t.child_frame_id = vals['child-frame-id']
    t.transform.translation.x = float(vals['x'])
    t.transform.translation.y = float(vals['y'])
    t.transform.translation.z = float(vals['z'])
    q = q_from_euler(float(vals['roll']), float(vals['pitch']), float(vals['yaw']))
    t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q
    br = StaticTransformBroadcaster(node)
    br.sendTransform(t)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


_unused = (copy,)
