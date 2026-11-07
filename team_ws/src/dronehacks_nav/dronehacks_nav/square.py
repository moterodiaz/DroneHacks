"""Pure square-mission state machine for dronehacks_nav.

This module holds all mission logic with no rclpy dependency so it is
testable in the repository's credential-free CI and on the workstation
(``tests/test_square.py``). The ROS node in ``mission.py`` is a thin
adapter: it feeds the engine time, the Offboard flag and the odometry
position, publishes the returned setpoint and logs the returned events.

Safety behaviour (matching the reviewed design at PR #33):

* **Pause, not pretend.** With Offboard inactive the pilot has control:
  the engine returns the current position as the setpoint and advances
  nothing. The node's "mission paused" log is therefore true.
* **Hold where you are.** The HOLD state (square complete or corner
  timeout) commands the position where HOLD was entered — never a corner
  that may not have been reached.
* **Disarm ends the attempt.** ``reset()`` (the kit /skybeta/state
  ``armed`` flag dropping) clears all state, so a re-arm always begins a
  fresh square from wherever the drone then sits.
"""

import math


class SquareEngine:
    """Square-mission state machine (pure, clock injected by the caller)."""

    def __init__(self, side_m=2.0, hover_altitude_m=0.80, hover_seconds=5.0,
                 corner_timeout_s=30.0, arrival_radius_m=0.15):
        self.side = float(side_m)
        self.hover_altitude = float(hover_altitude_m)
        self.hover_seconds = float(hover_seconds)
        self.corner_timeout = float(corner_timeout_s)
        self.arrival_radius = float(arrival_radius_m)

        self.state = 'WAIT_FOR_OFFBOARD'
        self.start = None
        self.corners = []
        self.corner_index = 0
        self.hold_target = None
        self.hover_started_at = None
        self.corner_started_at = None
        self._offboard_was_active = False
        self.log_once = set()

    # -- events ---------------------------------------------------------------
    def _once(self, events, key, message):
        if key in self.log_once:
            return
        self.log_once.add(key)
        events.append(message)

    # -- geometry -------------------------------------------------------------
    def _arrived(self, position, target):
        return math.dist(position, target) <= self.arrival_radius

    def _build_square(self):
        x0, y0, z0 = self.start
        z = z0 + self.hover_altitude
        s = self.side
        return [
            (x0 + s, y0,     z),
            (x0 + s, y0 + s, z),
            (x0,     y0 + s, z),
            (x0,     y0,     z),
        ]

    # -- transitions ------------------------------------------------------------
    def reset(self):
        """Disarm: end the flight attempt; a re-arm starts a fresh square."""
        self.state = 'WAIT_FOR_OFFBOARD'
        self.start = None
        self.corners = []
        self.corner_index = 0
        self.hold_target = None
        self.hover_started_at = None
        self.corner_started_at = None
        self._offboard_was_active = False
        self.log_once.clear()

    def tick(self, now, offboard_active, position):
        """Advance the mission one step.

        Returns ``(target, events)`` where ``target`` is the setpoint to
        publish (None if no odometry has arrived yet) and ``events`` is a
        list of human-readable log messages new this tick.
        """
        events = []
        if position is None:
            self._once(events, 'no_odom',
                       'no /visual_slam/tracking/odometry yet — is cuVSLAM '
                       'up and tracking? Nothing will be commanded until it is.')
            return None, events

        if self._offboard_was_active and not offboard_active:
            events.append('Offboard lost — pilot has taken control. Mission '
                          'paused: holding position until Offboard returns. '
                          'Offboard does not return until disarmed and '
                          're-armed.')
        self._offboard_was_active = offboard_active

        # Pause: with Offboard inactive the pilot has control — hold the
        # current position and advance nothing.
        if not offboard_active and self.state != 'WAIT_FOR_OFFBOARD':
            return tuple(position), events

        handler = getattr(self, f'_state_{self.state.lower()}')
        return handler(now, tuple(position), events)

    # -- states ---------------------------------------------------------------
    def _state_wait_for_offboard(self, now, position, events):
        if not self._offboard_was_active:
            self._once(events, 'waiting',
                       'holding at current position until the safety pilot '
                       'arms and switches to Position.')
            return position, events
        self.start = position
        self.corners = self._build_square()
        self.hold_target = None
        self.hover_started_at = now
        self.state = 'HOVER'
        events.append(
            f'Offboard active. Start ENU ({self.start[0]:.2f}, '
            f'{self.start[1]:.2f}, {self.start[2]:.2f}); climbing to '
            f'{self.start[2] + self.hover_altitude:.2f} m.')
        return position, events

    def _state_hover(self, now, position, events):
        target = (self.start[0], self.start[1],
                  self.start[2] + self.hover_altitude)
        if not self._arrived(position, target):
            self.hover_started_at = now  # hover timer starts on arrival
            return target, events
        if now - self.hover_started_at < self.hover_seconds:
            return target, events
        self.corner_index = 0
        self.corner_started_at = now
        self.state = 'SQUARE'
        events.append(f'hover complete — flying the {self.side:.1f} m '
                      f'square, corner 1/{len(self.corners)}')
        return target, events

    def _state_square(self, now, position, events):
        target = self.corners[self.corner_index]
        if self._arrived(position, target):
            self.corner_index += 1
            self.corner_started_at = now
            if self.corner_index >= len(self.corners):
                self.hold_target = position
                self.state = 'HOLD'
                events.append('square complete — holding. Safety pilot can '
                              'land.')
                return tuple(position), events
            events.append(f'corner reached — next: {self.corner_index + 1}/'
                           f'{len(self.corners)} at ENU '
                           f'({self.corners[self.corner_index][0]:.2f}, '
                           f'{self.corners[self.corner_index][1]:.2f}, '
                           f'{self.corners[self.corner_index][2]:.2f})')
            return self.corners[self.corner_index], events
        if now - self.corner_started_at > self.corner_timeout:
            events.append(f'corner {self.corner_index + 1} not reached '
                          f'within {self.corner_timeout:.0f} s '
                          f'({math.dist(position, target):.2f} m away) — '
                          'holding. Check /skybeta/state for a clamp or '
                          'cuVSLAM for tracking loss.')
            self.hold_target = position
            self.state = 'HOLD'
            return tuple(position), events
        return target, events

    def _state_hold(self, now, position, events):
        # Hold where the drone actually was when HOLD was entered — never
        # a corner that may not have been reached.
        return tuple(self.hold_target or position), events
