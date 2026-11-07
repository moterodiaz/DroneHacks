"""The safety pilot: the organizers' test procedure, scripted, plus manual inputs.

README "How your drone is tested":

  1. drone in the cage, stack running, team code publishing setpoints;
  2. the pilot **arms** on the ground with the mode switch on **Altitude**;
  3. within a few seconds the pilot switches to **Position**;
  4. hands off: sticks centred, throttle low, until the run ends or the pilot
     must take over.

``PilotScript`` produces that timeline on the RC channels.  The GUI (and
scenario ``faults``) can override any channel at any time: takeover with the
throttle, mode switch to Altitude/Manual, kill switch, RC link loss.
"""
from __future__ import annotations

SLOT = {'manual': 1, 'altitude': 4, 'position': 6}


class PilotScript:
    def __init__(self, rc, arm_at_s=12.0, position_after_s=2.0, enabled=True, initial_mode='altitude'):
        self.rc = rc
        self.arm_at_s = arm_at_s
        self.position_after_s = position_after_s
        self.enabled = enabled
        self.rc.mode_slot = SLOT[initial_mode]
        self.rc.throttle = -1.0
        self.rc.arm_switch = False
        self.done = set()
        self.overrides = {}          # channel -> value, set by the GUI / faults
        self.log = []

    def update(self, t):
        rc = self.rc
        if self.enabled:
            if 'arm' not in self.done and t >= self.arm_at_s:
                self.done.add('arm')
                rc.mode_slot = SLOT['altitude']
                rc.throttle = -1.0
                rc.arm_switch = True
                self.log.append((t, 'pilot: arm switch ON (mode Altitude, throttle low)'))
            if 'position' not in self.done and 'arm' in self.done and t >= self.arm_at_s + self.position_after_s:
                self.done.add('position')
                rc.mode_slot = SLOT['position']
                self.log.append((t, 'pilot: mode switch -> Position (hands off)'))
        for ch, v in self.overrides.items():
            setattr(rc, ch, v)

    def command(self, t, action, value=None):
        """Manual pilot inputs from the GUI."""
        rc = self.rc
        if action == 'arm':
            self.overrides['arm_switch'] = True
        elif action == 'disarm':
            self.overrides['arm_switch'] = False
        elif action == 'mode':
            self.overrides['mode_slot'] = SLOT[value]
        elif action == 'throttle':
            self.overrides['throttle'] = float(value)
        elif action in ('roll', 'pitch', 'yaw'):
            self.overrides[action] = float(value)
        elif action == 'kill':
            self.overrides['kill_switch'] = bool(value)
        elif action == 'rc_link':
            self.overrides['link'] = bool(value)
        elif action == 'takeover':
            # the normal takeover: raise the throttle towards centre (a stick move PX4 sees)
            self.overrides['throttle'] = 0.0
        elif action == 'release':
            for ch in ('roll', 'pitch', 'yaw', 'throttle'):
                self.overrides.pop(ch, None)
        elif action == 'script':
            self.enabled = bool(value)
        self.log.append((t, f'pilot (manual): {action} {"" if value is None else value}'.strip()))
        rc_changes = {k: getattr(rc, k) for k in ('arm_switch', 'mode_slot', 'throttle')}
        return rc_changes
