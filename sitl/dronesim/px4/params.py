"""PX4 parameter sets: load QGC ``.params`` / MAVProxy ``.parm`` files in layers.

The simulated PX4 reads every tunable it models from a :class:`ParamSet`, built
from the organizer's real parameter files in ``drone-snapshot/.../px4/params``.
A *profile* is an ordered list of files (later files override earlier ones),
optionally followed by explicit overrides.  The default profile is what drone
1 (SN0001) should be running: its own full dump, the delta file written for it,
and the fleet-wide staged files that came later.
"""
from __future__ import annotations

import os

# PX4 v1.16 defaults for the parameters the model reads, used only when no file
# in the profile defines them (every full dump does).
DEFAULTS = {
    'MAV_SYS_ID': 1, 'COM_ARMABLE': 1, 'COM_DISARM_LAND': 2.0, 'COM_DISARM_PRFLT': 10.0,
    'COM_FAIL_ACT_T': 5.0, 'COM_KILL_DISARM': 5.0, 'COM_LKDOWN_TKO': 3.0, 'COM_OBL_RC_ACT': 0,
    'COM_OF_LOSS_T': 1.0, 'COM_POSCTL_NAVL': 0, 'COM_POS_FS_EPH': 5.0, 'COM_RCL_EXCEPT': 0,
    'COM_RC_LOSS_T': 0.5, 'COM_RC_OVERRIDE': 3, 'COM_RC_STICK_OV': 30.0, 'COM_SPOOLUP_TIME': 1.0,
    'COM_FLTMODE1': 0, 'COM_FLTMODE2': -1, 'COM_FLTMODE3': -1, 'COM_FLTMODE4': 1, 'COM_FLTMODE5': -1,
    'COM_FLTMODE6': 2, 'COM_LOW_BAT_ACT': 0, 'COM_ARM_WO_GPS': 1, 'COM_RC_IN_MODE': 0,
    'NAV_RCL_ACT': 6, 'NAV_DLL_ACT': 0, 'GF_ACTION': 2, 'GF_MAX_HOR_DIST': 0.0, 'GF_MAX_VER_DIST': 0.0,
    'RC_MAP_ARM_SW': 7, 'RC_MAP_KILL_SW': 5, 'RC_MAP_FLTMODE': 6, 'RC_MAP_THROTTLE': 3,
    'RC_MAP_ROLL': 1, 'RC_MAP_PITCH': 2, 'RC_MAP_YAW': 4, 'RC_ARMSWITCH_TH': 0.75,
    'RC_KILLSWITCH_TH': 0.75,
    'MPC_XY_P': 0.95, 'MPC_Z_P': 1.0, 'MPC_XY_VEL_P_ACC': 1.8, 'MPC_XY_VEL_I_ACC': 0.4,
    'MPC_XY_VEL_D_ACC': 0.2, 'MPC_Z_VEL_P_ACC': 4.0, 'MPC_Z_VEL_I_ACC': 2.0, 'MPC_Z_VEL_D_ACC': 0.0,
    'MPC_XY_VEL_MAX': 12.0, 'MPC_Z_VEL_MAX_UP': 3.0, 'MPC_Z_VEL_MAX_DN': 1.5, 'MPC_TILTMAX_AIR': 45.0,
    'MPC_TILTMAX_LND': 12.0, 'MPC_THR_HOVER': 0.5, 'MPC_THR_MIN': 0.12, 'MPC_THR_MAX': 1.0,
    'MPC_THR_XY_MARG': 0.3, 'MPC_USE_HTE': 1, 'MPC_TKO_RAMP_T': 3.0, 'MPC_TKO_SPEED': 1.5,
    'MPC_LAND_SPEED': 0.7, 'MPC_LAND_CRWL': 0.3, 'MPC_LAND_ALT1': 10.0, 'MPC_LAND_ALT2': 5.0,
    'MPC_LAND_ALT3': 1.0, 'MPC_VELD_LP': 5.0, 'MPC_ACC_HOR': 3.0, 'MPC_ACC_HOR_MAX': 5.0,
    'MPC_ACC_UP_MAX': 4.0, 'MPC_ACC_DOWN_MAX': 3.0, 'MPC_JERK_MAX': 8.0, 'MPC_JERK_AUTO': 4.0,
    'MPC_VEL_MANUAL': 10.0, 'MPC_MAN_TILT_MAX': 35.0, 'MPC_MAN_Y_MAX': 150.0, 'MPC_MAN_Y_TAU': 0.08,
    'MPC_HOLD_DZ': 0.1, 'MPC_Z_MAN_EXPO': 0.6, 'MPC_XY_MAN_EXPO': 0.6, 'MPC_YAW_EXPO': 0.6,
    'MPC_MANTHR_MIN': 0.08, 'MPC_THR_CURVE': 0, 'MPC_POS_MODE': 4, 'MPC_ACC_DECOUPLE': 1,
    'MPC_YAWRAUTO_MAX': 45.0, 'MPC_Z_V_AUTO_DN': 1.5, 'MPC_Z_V_AUTO_UP': 3.0,
    'MIS_TAKEOFF_ALT': 2.5,
    'MC_ROLL_P': 6.5, 'MC_PITCH_P': 6.5, 'MC_YAW_P': 2.8, 'MC_YAW_WEIGHT': 0.4,
    'MC_ROLLRATE_P': 0.15, 'MC_ROLLRATE_I': 0.2, 'MC_ROLLRATE_D': 0.003, 'MC_ROLLRATE_FF': 0.0,
    'MC_ROLLRATE_K': 1.0, 'MC_PITCHRATE_P': 0.15, 'MC_PITCHRATE_I': 0.2, 'MC_PITCHRATE_D': 0.003,
    'MC_PITCHRATE_FF': 0.0, 'MC_PITCHRATE_K': 1.0, 'MC_YAWRATE_P': 0.2, 'MC_YAWRATE_I': 0.1,
    'MC_YAWRATE_D': 0.0, 'MC_YAWRATE_FF': 0.0, 'MC_YAWRATE_K': 1.0, 'MC_RR_INT_LIM': 0.3,
    'MC_PR_INT_LIM': 0.3, 'MC_YR_INT_LIM': 0.3, 'MC_ROLLRATE_MAX': 220.0, 'MC_PITCHRATE_MAX': 220.0,
    'MC_YAWRATE_MAX': 200.0, 'MC_YAW_TQ_CUTOFF': 2.0, 'MC_AIRMODE': 0,
    'IMU_GYRO_CUTOFF': 40.0, 'IMU_DGYRO_CUTOFF': 30.0, 'IMU_GYRO_RATEMAX': 800,
    'CA_ROTOR_COUNT': 4,
    'CA_ROTOR0_PX': 0.08, 'CA_ROTOR0_PY': 0.16, 'CA_ROTOR0_KM': 0.05, 'CA_ROTOR0_CT': 6.5,
    'CA_ROTOR1_PX': -0.15, 'CA_ROTOR1_PY': -0.14, 'CA_ROTOR1_KM': 0.05, 'CA_ROTOR1_CT': 6.5,
    'CA_ROTOR2_PX': 0.08, 'CA_ROTOR2_PY': -0.16, 'CA_ROTOR2_KM': -0.05, 'CA_ROTOR2_CT': 6.5,
    'CA_ROTOR3_PX': -0.15, 'CA_ROTOR3_PY': 0.14, 'CA_ROTOR3_KM': -0.05, 'CA_ROTOR3_CT': 6.5,
    'LNDMC_TRIG_TIME': 1.0, 'LNDMC_Z_VEL_MAX': 0.25, 'LNDMC_XY_VEL_MAX': 1.5, 'LNDMC_ROT_MAX': 20.0,
    'LNDMC_ALT_GND': 2.0,
    'EKF2_EV_CTRL': 9, 'EKF2_EV_DELAY': 70.0, 'EKF2_EV_NOISE_MD': 0, 'EKF2_EVP_NOISE': 0.1,
    'EKF2_EVA_NOISE': 0.05, 'EKF2_EVP_GATE': 5.0, 'EKF2_EV_QMIN': 0, 'EKF2_HGT_REF': 2,
    'EKF2_BARO_CTRL': 1, 'EKF2_RNG_CTRL': 1, 'EKF2_RNG_A_HMAX': 3.0, 'EKF2_RNG_A_VMAX': 1.0,
    'EKF2_RNG_A_IGATE': 1.0, 'EKF2_RNG_QLTY_T': 1.0, 'EKF2_RNG_POS_Z': 0.03445, 'EKF2_MIN_RNG': 0.1,
    'EKF2_RNG_NOISE': 0.1, 'EKF2_RNG_SFE': 0.05, 'EKF2_NOAID_TOUT': 5000000, 'EKF2_ACC_NOISE': 0.35,
    'EKF2_GYR_NOISE': 0.015, 'EKF2_DELAY_MAX': 200.0, 'EKF2_GPS_CTRL': 0,
    'FD_FAIL_P': 60, 'FD_FAIL_R': 0, 'FD_FAIL_P_TTRI': 0.3, 'CBRK_FLIGHTTERM': 121212,
    'BAT1_N_CELLS': 4, 'BAT1_V_CHARGED': 4.05, 'BAT1_V_EMPTY': 3.6, 'BAT_LOW_THR': 0.15,
    'BAT_CRIT_THR': 0.07, 'BAT_EMERGEN_THR': 0.05,
    'UXRCE_DDS_DOM_ID': 1, 'UXRCE_DDS_SYNCT': 1,
}

PARAMS_DIR_PARTS = ('drone-snapshot', 'home', 'dronehacks', 'skybeta', 'px4', 'params')

#: The fleet guide's load order for a drone, plus drone 1's own delta file.
FLEET_FILES = ['00_baseline.params', '02_telem2_uxrce_dds.params', '03a_height_reference.params',
               '03b_ekf2_vision.params', '04_rc_only_fd_roll_off.params', '05_rc_switches_board_rot.params',
               '06_rc_radiomaster.params', '07_airframe_geometry.params', '08_no_optical_flow.params',
               '09_autonomy_takeover.params', 'fleet/SN0001-fixes-2026-10-15.params']
DRONE1_IDENTITY = {'MAV_SYS_ID': 1, 'UXRCE_DDS_DOM_ID': 1}
#: contracts/px4.md "Failsafe behaviour" table: the organisers' values (contracts beat code)
CONTRACT_PX4 = {'COM_OF_LOSS_T': 1.0, 'COM_POSCTL_NAVL': 0, 'NAV_RCL_ACT': 6, 'COM_FAIL_ACT_T': 5.0,
                'MIS_TAKEOFF_ALT': 2.5, 'COM_KILL_DISARM': 5.0, 'MPC_XY_VEL_MAX': 1.0, 'EKF2_EV_CTRL': 9,
                'EKF2_HGT_REF': 2}

#: name -> (description, [files relative to px4/params], overrides)
PROFILES = {
    'drone1-fleet': (
        "Drone 1 set up by the fleet guide (00_baseline, 02, 03a, 03b, 04-09, SN0001-fixes), with the "
        "organisers' failsafe values from contracts/px4.md on top (MPC_XY_VEL_MAX 1.0, NAV_RCL_ACT 6, "
        "COM_OF_LOSS_T 1 s, COM_FAIL_ACT_T 5 s ...).",
        FLEET_FILES, dict(DRONE1_IDENTITY, **CONTRACT_PX4)),
    'drone1-fleet-files-only': (
        "The fleet files alone, without the contract overlay: MPC_XY_VEL_MAX 12, NAV_RCL_ACT 2 "
        "(Return, unavailable without GPS).",
        FLEET_FILES, dict(DRONE1_IDENTITY)),
    'drone1-as-dumped': (
        "Drone 1 exactly as dumped on 2026-10-15 (v1.16.1). NOTE: COM_FLTMODE6 = 0, so the switch "
        "position the pilot calls 'Position' is Manual -- no auto-Offboard on this dump.",
        ['fleet/SN0001-2026-10-15.params'], {}),
    'drone1-dump-plus-fixes': (
        "Drone 1's dump + its fixes file + staged 03b, 04-09 (still COM_FLTMODE6 = 0 from the dump).",
        ['fleet/SN0001-2026-10-15.params', 'fleet/SN0001-fixes-2026-10-15.params',
         '03b_ekf2_vision.params', '04_rc_only_fd_roll_off.params', '05_rc_switches_board_rot.params',
         '06_rc_radiomaster.params', '07_airframe_geometry.params', '08_no_optical_flow.params',
         '09_autonomy_takeover.params'], {}),
    'drone1-team-allowlist': (
        "drone1-fleet plus the only values contracts/px4.md lets the team write: "
        "EKF2_EVA_NOISE=0.01, EKF2_EV_DELAY=0 (simulator-only candidate), EKF2_EV_QMIN=0.",
        FLEET_FILES, dict(DRONE1_IDENTITY, **CONTRACT_PX4,
                          **{'EKF2_EVA_NOISE': 0.01, 'EKF2_EV_DELAY': 0.0, 'EKF2_EV_QMIN': 0})),
    'drone16-vio': (
        "Drone 16's 2026-11-03 dump: the organizers' VIO position-hold reference (EV delay 60, "
        "MPC_XY_VEL_MAX 12, NAV_RCL_ACT 2), with drone 1's system id.",
        ['fleet/drone16-vio-2026-11-03.params'], dict(DRONE1_IDENTITY)),
}
DEFAULT_PROFILE = 'drone1-fleet'

#: Parameters contracts/px4.md allows team code to write (AGENTS.md section 3.8).
TEAM_WRITABLE = {'EKF2_EVA_NOISE', 'EKF2_EV_DELAY', 'EKF2_EV_QMIN'}


def parse_param_file(path):
    """{NAME: value} from a QGC tab-separated file or a MAVProxy ``NAME VALUE`` dump."""
    out = {}
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split('\t') if '\t' in line else line.split()
            if len(parts) >= 5:
                name, value, ptype = parts[2], parts[3], parts[4]
            elif len(parts) >= 2:
                name, value, ptype = parts[0], parts[1], None
            else:
                continue
            try:
                if ptype is not None:
                    num = float(value)
                    out[name] = int(num) if ptype.strip() in ('1', '2', '3', '4', '5', '6', '7', '8') else num
                else:
                    out[name] = int(value) if value.lstrip('-').isdigit() else float(value)
            except ValueError:
                continue
    return out


class ParamSet:
    """Resolved parameter values plus where each one came from."""

    def __init__(self, values=None, sources=None, profile=None):
        self.values = dict(DEFAULTS)
        self.sources = {k: 'PX4 v1.16 default' for k in DEFAULTS}
        if values:
            self.values.update(values)
        if sources:
            self.sources.update(sources)
        self.profile = profile

    def __getitem__(self, name):
        return self.values[name]

    def get(self, name, default=None):
        return self.values.get(name, default)

    def f(self, name):
        return float(self.values[name])

    def i(self, name):
        return int(round(float(self.values[name])))

    def set(self, name, value, source='override'):
        self.values[name] = value
        self.sources[name] = source

    def as_dict(self):
        return dict(self.values)


def params_dir(repo_root):
    return os.path.join(repo_root, *PARAMS_DIR_PARTS)


def load_profile(repo_root, profile=DEFAULT_PROFILE, overrides=None):
    if profile not in PROFILES:
        raise KeyError(f'unknown PX4 parameter profile {profile!r}; known: {sorted(PROFILES)}')
    _desc, files, prof_over = PROFILES[profile]
    base = params_dir(repo_root)
    ps = ParamSet(profile=profile)
    for rel in files:
        path = os.path.join(base, rel)
        if not os.path.isfile(path):
            raise FileNotFoundError(f'PX4 parameter file missing: {path}')
        for k, v in parse_param_file(path).items():
            ps.set(k, v, rel)
    for k, v in prof_over.items():
        ps.set(k, v, f'profile {profile}')
    for k, v in (overrides or {}).items():
        ps.set(k, v, 'scenario override')
    return ps


def conflicts(repo_root, names):
    """Where the organizer files disagree on ``names``: {name: {file: value}}."""
    base = params_dir(repo_root)
    found = {}
    for dirpath, _dirs, files in os.walk(base):
        for fn in files:
            if not fn.endswith(('.params', '.parm')):
                continue
            path = os.path.join(dirpath, fn)
            vals = parse_param_file(path)
            for n in names:
                if n in vals:
                    found.setdefault(n, {})[os.path.relpath(path, base)] = vals[n]
    return {n: v for n, v in found.items() if len(set(v.values())) > 1}
