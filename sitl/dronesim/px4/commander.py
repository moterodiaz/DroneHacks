"""PX4 v1.16 commander: arming, modes, failsafes, RC, vehicle commands.

What it reproduces, with the parameter that controls each:

* **RC** (``COM_RC_IN_MODE`` 0): arm switch on ``RC_MAP_ARM_SW`` (arms on the
  off->on edge only, so a switch already on at boot does nothing), kill switch on
  ``RC_MAP_KILL_SW`` (motors stop at once, disarm after ``COM_KILL_DISARM``),
  mode switch on ``RC_MAP_FLTMODE`` mapped through ``COM_FLTMODE1..6`` -- PX4
  acts only when the switch *moves*.  Stick override of Offboard/Auto per
  ``COM_RC_OVERRIDE`` and ``COM_RC_STICK_OV``.  RC loss after ``COM_RC_LOSS_T``.
* **Arming checks**: the current mode's requirements (Altitude needs a height
  estimate, Position a horizontal position), throttle position, landed.
* **Mode requirements and fallbacks** (ModeUtil / failsafe.cpp): Position with
  no position falls to Altitude (``COM_POSCTL_NAVL`` 0); Offboard needs the
  ``offboard_control_mode`` stream within ``COM_OF_LOSS_T`` (and a valid
  position for position control); losing it applies ``COM_OBL_RC_ACT`` and
  changes the *user's* intended mode, so Offboard never comes back by itself.
* **Failsafe actions**: RC loss -> ``NAV_RCL_ACT``; geofence -> ``GF_ACTION``
  with ``GF_MAX_VER_DIST``; actions from Hold upwards wait ``COM_FAIL_ACT_T`` in
  Hold first.
* **Auto disarm**: not taken off within ``COM_DISARM_PRFLT``; landed for
  ``COM_DISARM_LAND``.  Failure detector lockdown within ``COM_LKDOWN_TKO`` of
  takeoff on ``FD_FAIL_P``.
* **vehicle_command**: DO_SET_MODE, COMPONENT_ARM_DISARM, NAV_TAKEOFF/LAND/RTL,
  DO_FLIGHTTERMINATION, with ``vehicle_command_ack``.  Acro is refused and
  reported (AGENTS.md section 3.6 bans it on this project).
"""
from __future__ import annotations

import math

# vehicle_status nav_state values (px4_msgs/msg/VehicleStatus.msg, v1.16)
MANUAL, ALTCTL, POSCTL, AUTO_MISSION, AUTO_LOITER, AUTO_RTL = 0, 1, 2, 3, 4, 5
POSITION_SLOW, ACRO, DESCEND, TERMINATION, OFFBOARD, STAB = 6, 10, 12, 13, 14, 15
AUTO_TAKEOFF, AUTO_LAND, AUTO_FOLLOW_TARGET, AUTO_PRECLAND, ORBIT = 17, 18, 19, 20, 21

NAV_NAMES = {MANUAL: 'MANUAL', ALTCTL: 'ALTCTL', POSCTL: 'POSCTL', AUTO_MISSION: 'AUTO_MISSION',
             AUTO_LOITER: 'AUTO_LOITER', AUTO_RTL: 'AUTO_RTL', ACRO: 'ACRO', DESCEND: 'DESCEND',
             TERMINATION: 'TERMINATION', OFFBOARD: 'OFFBOARD', STAB: 'STAB', AUTO_TAKEOFF: 'AUTO_TAKEOFF',
             AUTO_LAND: 'AUTO_LAND', POSITION_SLOW: 'POSITION_SLOW'}

ARMING_STATE_DISARMED, ARMING_STATE_ARMED = 1, 2

# COM_FLTMODE values -> nav_state
FLTMODE_TO_NAV = {0: MANUAL, 1: ALTCTL, 2: POSCTL, 3: AUTO_MISSION, 4: AUTO_LOITER, 5: AUTO_RTL, 6: ACRO,
                  7: OFFBOARD, 8: STAB, 10: AUTO_TAKEOFF, 11: AUTO_LAND, 12: AUTO_FOLLOW_TARGET,
                  13: AUTO_PRECLAND, 14: ORBIT}

# PX4 custom main modes (DO_SET_MODE param2) and AUTO sub modes (param3)
CUSTOM_MAIN = {1: MANUAL, 2: ALTCTL, 3: POSCTL, 5: ACRO, 6: OFFBOARD, 7: STAB}
AUTO_SUB = {2: AUTO_TAKEOFF, 3: AUTO_LOITER, 4: AUTO_MISSION, 5: AUTO_RTL, 6: AUTO_LAND}

# vehicle_command ids
CMD_NAV_RETURN_TO_LAUNCH, CMD_NAV_LAND, CMD_NAV_TAKEOFF = 20, 21, 22
CMD_DO_SET_MODE, CMD_DO_FLIGHTTERMINATION, CMD_COMPONENT_ARM_DISARM = 176, 185, 400
CMD_NAV_LOITER_UNLIM = 17
ACK_ACCEPTED, ACK_TEMP_REJECTED, ACK_DENIED, ACK_UNSUPPORTED, ACK_FAILED = 0, 1, 2, 3, 4

# arm / disarm reasons (VehicleStatus ARM_DISARM_REASON_*)
REASON_TRANSITION_TO_STANDBY, REASON_STICK_GESTURE, REASON_RC_SWITCH, REASON_COMMAND_INTERNAL = 0, 1, 2, 3
REASON_COMMAND_EXTERNAL, REASON_MISSION_START, REASON_SAFETY_BUTTON, REASON_AUTO_DISARM_LAND = 4, 5, 6, 7
REASON_AUTO_DISARM_PREFLIGHT, REASON_KILL_SWITCH, REASON_LOCKDOWN, REASON_FAILURE_DETECTOR = 8, 9, 10, 11
REASON_SHUTDOWN, REASON_UNIT_TEST, REASON_RC_BUTTON, REASON_FAILSAFE = 12, 13, 14, 15

# failsafe action ordering (failsafe_base.h)
A_NONE, A_WARN, A_FALLBACK_POSCTRL, A_FALLBACK_ALTCTRL, A_FALLBACK_STAB = 0, 1, 2, 3, 4
A_HOLD, A_RTL, A_LAND, A_DESCEND, A_DISARM, A_TERMINATE = 5, 6, 7, 8, 9, 10
ACTION_NAMES = {A_NONE: 'none', A_WARN: 'warn', A_FALLBACK_POSCTRL: 'fallback to Position',
                A_FALLBACK_ALTCTRL: 'fallback to Altitude', A_FALLBACK_STAB: 'fallback to Stabilized',
                A_HOLD: 'Hold', A_RTL: 'Return', A_LAND: 'Land', A_DESCEND: 'Descend', A_DISARM: 'Disarm',
                A_TERMINATE: 'Terminate'}


class RcInput:
    """What the pilot's transmitter is doing (normalised sticks, switch positions)."""

    def __init__(self):
        self.roll = 0.0          # [-1, 1]
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = -1.0     # [-1, 1]; -1 = stick fully down (v1.16 manual_control_setpoint)
        self.mode_slot = 4       # 1..6 from the 3-position switch: 1 Manual, 4 Altitude, 6 Position
        self.arm_switch = False
        self.kill_switch = False
        self.link = True


class Commander:
    def __init__(self, px4):
        self.px4 = px4
        self.p = px4.params
        self.armed = False
        self.armed_time = None
        self.takeoff_time = None
        self.have_taken_off = False
        self.user_intended = ALTCTL
        self.nav_state = ALTCTL
        self.nav_state_timestamp = 0.0
        self.failsafe = False
        self.failsafe_action = A_NONE
        self.failsafe_reasons = []
        self._fs_delay_until = None
        self._fs_delayed_action = A_NONE
        self.latest_arming_reason = 0
        self.latest_disarming_reason = 0
        self.rc_prev = None
        self.rc_last_seen = 0.0
        self.rc_lost = False
        self.kill_since = None
        self.lockdown = False
        self.terminated = False
        self._landed_since = None
        self._last_lpos_ok_since = None
        self._prev_slot = None
        self._stick_hist = []
        self.pre_flight_checks_pass = False
        self.messages = []                       # (t, severity, text) like mavlink_log
        self.flags = {}
        self.home = None                         # local NED home (z) for the geofence
        self.acro_violation = None
        self.failsafe_and_user_took_over = False
        self._fd_pitch_since = None

    # -- logging -------------------------------------------------------------------------
    def log(self, severity, text):
        self.messages.append((self.px4.t, severity, text))
        self.px4.emit_log(severity, text)

    # -- health ---------------------------------------------------------------------------
    def update_flags(self, t):
        px4 = self.px4
        lpos = px4.lpos_out
        eph_thr = self.p.f('COM_POS_FS_EPH')
        eph_thr = math.inf if eph_thr < 0 else eph_thr
        xy_ok = bool(lpos and lpos['xy_valid'] and lpos['eph'] < eph_thr)
        xy_ok_relaxed = bool(lpos and lpos['xy_valid'] and lpos['eph'] < 2 * eph_thr)
        z_ok = bool(lpos and lpos['z_valid'])
        ocm = px4.offboard_mode
        ocm_recent = ocm is not None and (t - ocm['t']) < self.p.f('COM_OF_LOSS_T')
        ocm_available = False
        if ocm is not None:
            if ocm['position'] and xy_ok and z_ok:
                ocm_available = True
            elif ocm['velocity'] and xy_ok and z_ok:
                ocm_available = True
            elif ocm['acceleration'] and xy_ok:
                ocm_available = True
            elif ocm['attitude'] or ocm['body_rate'] or ocm.get('thrust_and_torque') or ocm.get('direct_actuator'):
                ocm_available = True
        self.flags = {
            'angular_velocity_invalid': False,
            'attitude_invalid': not px4.ekf.tilt_aligned,
            'local_altitude_invalid': not z_ok,
            'local_position_invalid': not xy_ok,
            'local_position_invalid_relaxed': not xy_ok_relaxed,
            'local_velocity_invalid': not xy_ok,
            'global_position_invalid': True,
            'home_position_invalid': self.home is None,
            'offboard_control_signal_lost': not (ocm_recent and ocm_available),
            'manual_control_signal_lost': self.rc_lost,
            'gcs_connection_lost': False,
            'geofence_breached': self._geofence_breached(),
            'battery_warning': px4.battery_warning,
            'fd_critical_failure': self.lockdown,
            'auto_mission_missing': True,
        }

    def _geofence_breached(self):
        vmax = self.p.f('GF_MAX_VER_DIST')
        lpos = self.px4.lpos_out
        if vmax <= 0 or self.home is None or not lpos or not lpos['z_valid']:
            return False
        return (self.home - lpos['z']) > vmax

    def mode_can_run(self, nav):
        f = self.flags
        if nav in (MANUAL, STAB):
            return not f['attitude_invalid']
        if nav == ACRO:
            return True
        if nav == ALTCTL:
            return not (f['attitude_invalid'] or f['local_altitude_invalid'])
        if nav == POSCTL:
            return not (f['attitude_invalid'] or f['local_altitude_invalid'] or f['local_position_invalid_relaxed'])
        if nav == OFFBOARD:
            return not (f['attitude_invalid'] or f['offboard_control_signal_lost'])
        if nav in (AUTO_LOITER, AUTO_LAND, AUTO_TAKEOFF):
            return not (f['attitude_invalid'] or f['local_altitude_invalid'] or f['local_position_invalid'])
        if nav == AUTO_RTL:
            return not (f['global_position_invalid'] or f['home_position_invalid'])
        if nav == AUTO_MISSION:
            return not (f['global_position_invalid'] or f['auto_mission_missing'])
        if nav == DESCEND:
            return not f['attitude_invalid']
        if nav == TERMINATION:
            return True
        return False

    def mode_unavailable_reason(self, nav):
        f = self.flags
        if nav == POSCTL and f['local_position_invalid_relaxed']:
            return 'No valid local position'
        if nav == ALTCTL and f['local_altitude_invalid']:
            return 'No valid altitude estimate'
        if nav == OFFBOARD and f['offboard_control_signal_lost']:
            return 'No offboard signal'
        if nav in (AUTO_LOITER, AUTO_LAND, AUTO_TAKEOFF) and f['local_position_invalid']:
            return 'No valid local position'
        if nav == AUTO_RTL:
            return 'No valid global position / home'
        if nav == AUTO_MISSION:
            return 'No valid mission'
        return 'Mode not available'

    # -- RC --------------------------------------------------------------------------------
    def _process_rc(self, t, rc):
        p = self.p
        if rc.link:
            self.rc_last_seen = t
        lost = (t - self.rc_last_seen) > p.f('COM_RC_LOSS_T')
        if lost and not self.rc_lost:
            self.log('CRITICAL', 'Manual control lost')
        elif not lost and self.rc_lost:
            self.log('INFO', 'Manual control regained')
        self.rc_lost = lost
        if lost:
            self.rc_prev = None
            return
        prev = self.rc_prev
        # arm switch: edges only
        if prev is not None and rc.arm_switch != prev.arm_switch:
            if rc.arm_switch:
                self.try_arm(t, REASON_RC_SWITCH)
            else:
                self.try_disarm(t, REASON_RC_SWITCH)
        # kill switch
        if rc.kill_switch:
            if self.kill_since is None:
                self.kill_since = t
                if self.armed:
                    self.log('CRITICAL', 'Kill-switch engaged')
        elif self.kill_since is not None:
            self.kill_since = None
            if self.armed:
                self.log('INFO', 'Kill-switch disengaged')
        # mode switch: the position at the first RC frame sets the boot mode; afterwards only a
        # change of slot does anything
        if self._prev_slot is None:
            fm = p.i(f'COM_FLTMODE{rc.mode_slot}') if 1 <= rc.mode_slot <= 6 else -1
            if fm >= 0 and fm in FLTMODE_TO_NAV and FLTMODE_TO_NAV[fm] != ACRO:
                self.user_intended = FLTMODE_TO_NAV[fm]
        elif rc.mode_slot != self._prev_slot:
            fm = p.i(f'COM_FLTMODE{rc.mode_slot}') if 1 <= rc.mode_slot <= 6 else -1
            if fm >= 0 and fm in FLTMODE_TO_NAV:
                self.request_mode(t, FLTMODE_TO_NAV[fm], source='rc')
        self._prev_slot = rc.mode_slot
        # stick override (COM_RC_OVERRIDE bit 0 auto, bit 1 offboard; bit 2 ignore throttle)
        self._stick_hist.append((t, rc.roll, rc.pitch, rc.yaw, rc.throttle))
        while self._stick_hist and t - self._stick_hist[0][0] > 0.1:
            self._stick_hist.pop(0)
        ov = p.i('COM_RC_OVERRIDE')
        auto = self.nav_state in (AUTO_LOITER, AUTO_LAND, AUTO_RTL, AUTO_TAKEOFF, AUTO_MISSION, DESCEND)
        offb = self.nav_state == OFFBOARD
        if self.armed and ((ov & 1 and auto) or (ov & 2 and offb)) and len(self._stick_hist) > 1 \
                and self._fs_delay_until is None:
            old = self._stick_hist[0]
            thr = p.f('COM_RC_STICK_OV') / 100.0
            moved = (abs(rc.roll - old[1]) > thr or abs(rc.pitch - old[2]) > thr or abs(rc.yaw - old[3]) > thr)
            if not (ov & 4):
                moved = moved or abs(rc.throttle - old[4]) > thr
            if moved:
                target = POSCTL if self.mode_can_run(POSCTL) else ALTCTL
                self.user_intended = target
                self.failsafe_and_user_took_over = self.failsafe
                self.log('INFO', 'Pilot took over using sticks')
        self.rc_prev = _copy_rc(rc)

    # -- arming ------------------------------------------------------------------------------
    def try_arm(self, t, reason):
        if self.armed:
            return True
        px4 = self.px4
        problem = None
        if self.terminated:
            problem = 'flight termination active'
        elif not self.p.i('COM_ARMABLE'):
            problem = 'arming disabled (COM_ARMABLE)'
        elif self.kill_since is not None:
            problem = 'Kill-switch engaged'
        elif not self.mode_can_run(self.nav_state):
            problem = f'{NAV_NAMES.get(self.nav_state, self.nav_state)} mode not available: ' \
                      f'{self.mode_unavailable_reason(self.nav_state)}'
        elif self.nav_state in (AUTO_LAND, AUTO_RTL, DESCEND, TERMINATION):
            problem = 'mode prevents arming'
        elif not px4.land_detector.landed:
            problem = 'vehicle not landed'
        elif reason in (REASON_RC_SWITCH, REASON_STICK_GESTURE) and not self.rc_lost:
            rc = px4.rc
            climb = self.nav_state in (ALTCTL, POSCTL, POSITION_SLOW)
            if climb and rc.throttle > 0.2:
                problem = 'throttle above center'
            elif not climb and self.nav_state in (MANUAL, STAB, ACRO) and rc.throttle > -0.8:
                problem = 'high throttle'
        if problem:
            self.log('CRITICAL', f'Arming denied: {problem}')
            return False
        self.armed = True
        self.armed_time = t
        self.have_taken_off = False
        self.takeoff_time = None
        self.latest_arming_reason = reason
        self.lockdown = False
        lpos = px4.lpos_out
        self.home = lpos['z'] if lpos and lpos['z_valid'] else None
        self.log('INFO', 'Armed by ' + {REASON_RC_SWITCH: 'RC switch', REASON_COMMAND_EXTERNAL: 'external command'}
                 .get(reason, 'internal command'))
        return True

    def try_disarm(self, t, reason, force=False):
        if not self.armed:
            return True
        landed = self.px4.land_detector.landed
        manual_like = self.nav_state in (MANUAL, STAB, ACRO)
        if not force and not landed and reason in (REASON_RC_SWITCH, REASON_COMMAND_EXTERNAL):
            if not (manual_like and self.p.i('COM_DISARM_MAN')):
                self.log('CRITICAL', 'Disarming denied: not landed')
                return False
        self.armed = False
        self.latest_disarming_reason = reason
        self.armed_time = None
        self._landed_since = None
        self.log('INFO', 'Disarmed by ' + {REASON_RC_SWITCH: 'RC switch', REASON_AUTO_DISARM_LAND: 'auto disarm on land',
                                           REASON_AUTO_DISARM_PREFLIGHT: 'auto preflight disarming',
                                           REASON_KILL_SWITCH: 'kill-switch', REASON_FAILSAFE: 'failsafe',
                                           REASON_LOCKDOWN: 'lockdown', REASON_COMMAND_EXTERNAL: 'external command'}
                 .get(reason, 'internal command'))
        return True

    # -- mode requests -----------------------------------------------------------------------
    def request_mode(self, t, nav, source='rc'):
        if nav == ACRO:
            self.acro_violation = (t, source)
            self.log('CRITICAL', 'SIM: Acro mode requested -- refused. Acro is banned on this project '
                                 '(AGENTS.md section 3.6); this run is flagged as a failure.')
            return False
        if not self.mode_can_run(nav):
            self.log('CRITICAL', f'Switching to {NAV_NAMES.get(nav, nav)} is currently not possible: '
                                 f'{self.mode_unavailable_reason(nav)}')
            return False
        self.user_intended = nav
        if self.failsafe:
            self.failsafe_and_user_took_over = True
        return True

    def handle_command(self, t, cmd):
        """cmd: dict(command, param1..7, target_system, from_external). Returns ack result."""
        sys_id = self.p.i('MAV_SYS_ID')
        if cmd.get('target_system', 0) not in (0, sys_id):
            return None                                  # not for us: no ack
        c = cmd['command']
        if c == CMD_DO_SET_MODE:
            base, main, sub = cmd['param1'], int(round(cmd['param2'])), int(round(cmd['param3']))
            if not (int(base) & 1):
                return ACK_UNSUPPORTED
            if main == 4:
                nav = AUTO_SUB.get(sub)
            else:
                nav = CUSTOM_MAIN.get(main)
            if nav is None:
                return ACK_UNSUPPORTED
            return ACK_ACCEPTED if self.request_mode(t, nav, source='command') else ACK_TEMP_REJECTED
        if c == CMD_COMPONENT_ARM_DISARM:
            arm = cmd['param1'] > 0.5
            force = abs(cmd['param2'] - 21196.0) < 0.5
            if arm:
                ok = self.try_arm(t, REASON_COMMAND_EXTERNAL if cmd.get('from_external') else REASON_COMMAND_INTERNAL)
            else:
                ok = self.try_disarm(t, REASON_COMMAND_EXTERNAL, force=force)
            return ACK_ACCEPTED if ok else ACK_TEMP_REJECTED
        if c == CMD_NAV_TAKEOFF:
            return ACK_ACCEPTED if self.request_mode(t, AUTO_TAKEOFF, 'command') else ACK_TEMP_REJECTED
        if c == CMD_NAV_LAND:
            return ACK_ACCEPTED if self.request_mode(t, AUTO_LAND, 'command') else ACK_TEMP_REJECTED
        if c == CMD_NAV_RETURN_TO_LAUNCH:
            return ACK_ACCEPTED if self.request_mode(t, AUTO_RTL, 'command') else ACK_TEMP_REJECTED
        if c == CMD_NAV_LOITER_UNLIM:
            return ACK_ACCEPTED if self.request_mode(t, AUTO_LOITER, 'command') else ACK_TEMP_REJECTED
        if c == CMD_DO_FLIGHTTERMINATION:
            if cmd['param1'] > 0.5:
                if self.p.i('CBRK_FLIGHTTERM') == 121212:
                    self.log('CRITICAL', 'Flight termination disabled (CBRK_FLIGHTTERM)')
                    return ACK_DENIED
                self.terminated = True
            return ACK_ACCEPTED
        return ACK_UNSUPPORTED

    # -- failsafe ----------------------------------------------------------------------------
    def _rcl_action(self):
        v = self.p.i('NAV_RCL_ACT')
        return {0: A_NONE, 1: A_HOLD, 2: A_RTL, 3: A_LAND, 4: A_DISARM, 5: A_TERMINATE, 6: A_DISARM}.get(v, A_RTL)

    def _obl_action(self):
        v = self.p.i('COM_OBL_RC_ACT')
        return {0: A_FALLBACK_POSCTRL, 1: A_FALLBACK_ALTCTRL, 2: A_FALLBACK_STAB, 3: A_RTL, 4: A_LAND,
                5: A_HOLD, 6: A_TERMINATE, 7: A_DISARM}.get(v, A_FALLBACK_POSCTRL)

    def _gf_action(self):
        v = self.p.i('GF_ACTION')
        return {0: A_NONE, 1: A_WARN, 2: A_HOLD, 3: A_RTL, 4: A_TERMINATE, 5: A_LAND}.get(v, A_WARN)

    def _evaluate_failsafe(self, t):
        f = self.flags
        actions = []
        if not self.armed:
            return A_NONE, []
        if f['manual_control_signal_lost']:
            exc = self.p.i('COM_RCL_EXCEPT')
            excepted = (self.user_intended in (AUTO_MISSION,) and exc & 1) or \
                       (self.user_intended in (AUTO_LOITER,) and exc & 2) or \
                       (self.user_intended == OFFBOARD and exc & 4)
            if not excepted:
                actions.append((self._rcl_action(), 'RC loss'))
        if self.user_intended == OFFBOARD and f['offboard_control_signal_lost']:
            actions.append((self._obl_action(), 'no offboard'))
        if f['geofence_breached']:
            actions.append((self._gf_action(), 'geofence breach'))
        if f['fd_critical_failure']:
            actions.append((A_DISARM, 'failure detector'))
        if not actions:
            return A_NONE, []
        actions.sort(key=lambda a: a[0])
        return actions[-1][0], [a[1] for a in actions]

    def _resolve_nav(self, t, intended):
        """Apply mode fallbacks for an intended nav state (ModeUtil / failsafe.cpp)."""
        nav = intended
        if nav == POSCTL and not self.mode_can_run(POSCTL):
            if self.p.i('COM_POSCTL_NAVL') == 0:
                nav = ALTCTL if self.mode_can_run(ALTCTL) else STAB
            else:
                nav = AUTO_LAND if self.mode_can_run(AUTO_LAND) else DESCEND
        if nav == ALTCTL and not self.mode_can_run(ALTCTL):
            nav = STAB
        if nav in (AUTO_LOITER, AUTO_TAKEOFF, AUTO_LAND) and not self.mode_can_run(nav):
            nav = AUTO_LAND if nav != AUTO_LAND and self.mode_can_run(AUTO_LAND) else DESCEND
        if nav == AUTO_RTL and not self.mode_can_run(AUTO_RTL):
            nav = AUTO_LAND if self.mode_can_run(AUTO_LAND) else DESCEND
        if nav == OFFBOARD and not self.mode_can_run(OFFBOARD):
            nav = POSCTL if self.mode_can_run(POSCTL) else (ALTCTL if self.mode_can_run(ALTCTL) else STAB)
        return nav

    def _apply_action(self, t, action, reasons):
        """Turn a failsafe action into a nav state (and side effects)."""
        delayable = A_HOLD < action < A_TERMINATE
        delay = self.p.f('COM_FAIL_ACT_T')
        if action != self._fs_delayed_action:
            self._fs_delayed_action = action
            self._fs_delay_until = (t + delay) if (delayable and delay > 0.01 and
                                                  self.mode_can_run(AUTO_LOITER)) else None
            if action > A_WARN:
                self.log('CRITICAL', f'Failsafe activated ({", ".join(reasons)}): '
                         + (f'Hold, then {ACTION_NAMES[action]} in {delay:.0f} s' if self._fs_delay_until
                            else ACTION_NAMES[action]))
        if self._fs_delay_until is not None and t < self._fs_delay_until:
            return AUTO_LOITER
        self._fs_delay_until = None
        if action == A_FALLBACK_POSCTRL:
            self.user_intended = POSCTL
            return self._resolve_nav(t, POSCTL)
        if action == A_FALLBACK_ALTCTRL:
            self.user_intended = ALTCTL
            return self._resolve_nav(t, ALTCTL)
        if action == A_FALLBACK_STAB:
            self.user_intended = STAB
            return STAB
        if action == A_HOLD:
            return self._resolve_nav(t, AUTO_LOITER)
        if action == A_RTL:
            return self._resolve_nav(t, AUTO_RTL)
        if action == A_LAND:
            return self._resolve_nav(t, AUTO_LAND)
        if action == A_DESCEND:
            return DESCEND
        if action == A_DISARM:
            self.try_disarm(t, REASON_FAILSAFE, force=True)
            return self.nav_state
        if action == A_TERMINATE:
            if self.p.i('CBRK_FLIGHTTERM') != 121212:
                self.terminated = True
                return TERMINATION
            self.try_disarm(t, REASON_FAILSAFE, force=True)
            return self.nav_state
        return self._resolve_nav(t, self.user_intended)

    # -- the cycle -----------------------------------------------------------------------------
    def update(self, t, dt):
        px4 = self.px4
        self.update_flags(t)
        self._process_rc(t, px4.rc)
        for cmd in px4.pop_commands():
            result = self.handle_command(t, cmd)
            if result is not None:
                px4.ack_command(cmd, result)
        self.update_flags(t)
        # failsafe
        action, reasons = self._evaluate_failsafe(t)
        if action > A_WARN and not (self.failsafe_and_user_took_over and action < A_HOLD):
            nav = self._apply_action(t, action, reasons)
            self.failsafe = True
            self.failsafe_action = action
            self.failsafe_reasons = reasons
        else:
            if self.failsafe and action <= A_WARN:
                self.log('INFO', 'Failsafe mode deactivated')
            self.failsafe = False
            self.failsafe_action = action
            self.failsafe_reasons = reasons
            self._fs_delayed_action = A_NONE
            self._fs_delay_until = None
            self.failsafe_and_user_took_over = False
            nav = self._resolve_nav(t, self.user_intended)
        if self.terminated:
            nav = TERMINATION
        if nav != self.nav_state:
            self.nav_state = nav
            self.nav_state_timestamp = t
        # auto disarm and failure detection
        self._auto_disarm(t)
        self._failure_detector(t)
        self.pre_flight_checks_pass = self.mode_can_run(self.nav_state) and not self.lockdown

    def _auto_disarm(self, t):
        if not self.armed:
            return
        p = self.p
        ld = self.px4.land_detector
        if not ld.landed and not self.have_taken_off:
            self.have_taken_off = True
            self.takeoff_time = t
        prflt = p.f('COM_DISARM_PRFLT')
        if prflt > 0 and not self.have_taken_off and t - self.armed_time > prflt:
            self.log('INFO', f'Disarming: not taken off within {prflt:.0f} s')
            self.try_disarm(t, REASON_AUTO_DISARM_PREFLIGHT, force=True)
            return
        if self.have_taken_off and ld.landed:
            if self._landed_since is None:
                self._landed_since = t
            land_t = p.f('COM_DISARM_LAND')
            if land_t > 0 and t - self._landed_since > land_t:
                self.try_disarm(t, REASON_AUTO_DISARM_LAND, force=True)
                return
        else:
            self._landed_since = None
        if self.kill_since is not None and t - self.kill_since > p.f('COM_KILL_DISARM'):
            self.try_disarm(t, REASON_KILL_SWITCH, force=True)

    def _failure_detector(self, t):
        p = self.p
        if not self.armed:
            self._fd_pitch_since = None
            return
        roll, pitch = self.px4.att_roll_pitch()
        lim_p = p.f('FD_FAIL_P')
        lim_r = p.f('FD_FAIL_R')
        bad = (lim_p > 0 and abs(math.degrees(pitch)) > lim_p) or (lim_r > 0 and abs(math.degrees(roll)) > lim_r)
        if bad:
            if self._fd_pitch_since is None:
                self._fd_pitch_since = t
            if t - self._fd_pitch_since > p.f('FD_FAIL_P_TTRI'):
                if self.takeoff_time is not None and t - self.takeoff_time < p.f('COM_LKDOWN_TKO') and not self.lockdown:
                    self.lockdown = True
                    self.log('CRITICAL', 'Critical failure detected: lockdown')
                elif not self.lockdown and p.i('CBRK_FLIGHTTERM') != 121212:
                    self.terminated = True
                    self.log('CRITICAL', 'Critical failure detected: terminate flight')
        else:
            self._fd_pitch_since = None

    @property
    def motors_enabled(self):
        return self.armed and self.kill_since is None and not self.lockdown and not self.terminated


def _copy_rc(rc):
    c = RcInput()
    c.__dict__.update(rc.__dict__)
    return c
