"""Tests for the tracking monitor and the pure mission engine.

These cover the reviewed findings: the per-waypoint timeout must hold+abort
(never force-advance), tracking loss must hold+abort with clear logs, and the
final state holds over the pad with no descent (no odometry-derived landing
floor exists at all).
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronehacks_manual_maze_tunnel_chicane_gateb_landing import waypoints as W  # noqa: E402


def make_engine(waypoints=None, **kwargs):
    wps = waypoints or [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    params = dict(waypoint_timeout_s=10.0, max_tracking_losses=3)
    params.update(kwargs)
    return W.MissionEngine(wps, hover_z=0.8, pad_xy=(2.0, 0.0),
                           arrival_radius_m=0.12, **params)


def drive_to_navigate(engine, now=0.0):
    """offboard on, tracking ok, drone already at hover height."""
    pos = (engine.waypoints[0][0], engine.waypoints[0][1], 0.8)
    engine.tick(now, False, pos, 'ok')
    engine.tick(now + 0.1, True, pos, 'ok')
    assert engine.state == 'CLIMB'
    engine.tick(now + 0.2, True, pos, 'ok')
    assert engine.state == 'NAVIGATE'
    return pos


class TestTimeouts:
    def test_per_waypoint_timeout_holds_and_never_advances(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        # drone never arrives; deadline passes
        target, events = eng.tick(11.0, True, pos, 'ok')
        assert eng.state == 'HOLD_FAULT'
        assert any('waypoint timeout' in e for e in events)
        assert any('take over' in e for e in events)
        # holding the last target, waypoint index unchanged
        assert eng.idx == 1
        again, _ = eng.tick(50.0, True, pos, 'ok')
        assert again == target
        assert eng.idx == 1  # never force-advanced
        # a timeout fault does NOT auto-resume on the next tick
        still, _ = eng.tick(51.0, True, pos, 'ok')
        assert eng.state == 'HOLD_FAULT'
        assert still == target

    def test_arrival_resets_the_deadline(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        wp1 = eng.waypoints[1]
        at_wp1 = (wp1[0], wp1[1], 0.8)
        _, events = eng.tick(5.0, True, at_wp1, 'ok')
        assert any('reached waypoint 1' in e for e in events)
        # not timed out yet even after another long stretch? deadline resets:
        assert eng.deadline == 15.0

    def test_slow_healthy_drone_is_not_force_advanced(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        # creeping toward the waypoint past the timeout window
        for i in range(1, 13):
            eng.tick(float(i), True, (pos[0] + i * 0.02, pos[1], 0.8), 'ok')
        assert eng.state == 'HOLD_FAULT'  # held, not advanced past the target
        assert eng.idx == 1


class TestTrackingLoss:
    def test_frozen_pose_holds_instead_of_navigating_blind(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        target, events = eng.tick(1.0, True, pos, 'frozen')
        assert eng.state == 'HOLD_FAULT'
        assert any('frozen' in e for e in events)
        assert any('take over' in e for e in events)
        assert target == eng.last_target

    def test_stale_odometry_holds(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        eng.tick(1.0, True, pos, 'stale')
        assert eng.state == 'HOLD_FAULT'

    def test_recovery_resumes_then_repeated_losses_abort(self):
        eng = make_engine(max_tracking_losses=2)
        pos = drive_to_navigate(eng)
        eng.tick(1.0, True, pos, 'frozen')
        _, events = eng.tick(2.0, True, pos, 'ok')
        assert eng.state == 'NAVIGATE'
        assert any('resuming' in e for e in events)
        eng.tick(3.0, True, pos, 'frozen')
        eng.tick(4.0, True, pos, 'ok')
        eng.tick(5.0, True, pos, 'frozen')
        _, events = eng.tick(6.0, True, pos, 'ok')
        assert eng.state == 'HOLD_FAULT'  # too many losses: stays aborted
        assert any('Too many' in e for e in events)

    def test_recovery_does_not_skip_the_current_waypoint(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        eng.tick(1.0, True, pos, 'frozen')
        eng.tick(2.0, True, pos, 'ok')
        assert eng.state == 'NAVIGATE'
        assert eng.idx == 1


class TestNoLandingDescent:
    def test_final_state_holds_over_pad_at_hover_height(self):
        eng = make_engine()
        pos = (eng.waypoints[0][0], eng.waypoints[0][1], 0.8)
        eng.tick(0.0, True, pos, 'ok')
        eng.tick(0.1, True, pos, 'ok')  # CLIMB done (already at height)
        for i in range(1, len(eng.waypoints)):
            wp = eng.waypoints[i]
            eng.tick(float(i), True, (wp[0], wp[1], 0.8), 'ok')
        assert eng.state == 'HOLD_OVER_PAD'
        for t in (10.0, 100.0, 1000.0):
            target, _ = eng.tick(t, True, (2.0, 0.0, 0.8), 'ok')
            assert target == (2.0, 0.0, 0.8)  # no descent, ever

    def test_heights_are_absolute_not_relative_to_offboard_start(self):
        eng = make_engine()
        # pilot took off in Position mode first: z = 1.0 when offboard begins
        pos = (eng.waypoints[0][0], eng.waypoints[0][1], 1.0)
        target, _ = eng.tick(0.0, True, pos, 'ok')
        assert target[2] == pytest.approx(0.8)  # commanded down to the mission
        # height, not start_z + hover (the reviewed "airborne floor" bug)


class TestOffboardAndWaiting:
    def test_holds_position_without_offboard(self):
        eng = make_engine()
        pos = (0.0, 0.0, 0.8)
        target, events = eng.tick(0.0, False, pos, 'ok')
        assert target == pos
        assert any('waiting for safety pilot' in e for e in events)

    def test_offboard_loss_mid_mission_holds_position(self):
        eng = make_engine()
        pos = drive_to_navigate(eng)
        somewhere = (0.5, 0.1, 0.8)
        target, events = eng.tick(1.0, False, somewhere, 'ok')
        assert target == somewhere
        assert eng.state == 'NAVIGATE'  # no state change, just hold
        assert any('Offboard inactive' in e for e in events)

    def test_no_odometry_publishes_nothing(self):
        eng = make_engine()
        target, _ = eng.tick(0.0, True, None, 'stale')
        assert target is None


class TestTrackingMonitor:
    def test_stale_when_no_messages(self):
        m = W.TrackingMonitor(stale_after_s=0.5)
        assert m.status(0.0) == 'stale'
        m.update(0.0, (0.0, 0.0, 0.0))
        assert m.status(0.2) == 'ok'
        assert m.status(0.6) == 'stale'

    def test_frozen_detection(self):
        m = W.TrackingMonitor(stale_after_s=0.5, frozen_window_s=1.0)
        for i in range(11):
            m.update(i * 0.1, (1.0, 2.0, 0.8))
        assert m.status(1.0) == 'frozen'

    def test_moving_odometry_is_ok(self):
        m = W.TrackingMonitor(stale_after_s=0.5, frozen_window_s=1.0)
        for i in range(11):
            m.update(i * 0.1, (i * 0.01, 2.0, 0.8))
        assert m.status(1.0) == 'ok'

    def test_recovery_after_freeze(self):
        m = W.TrackingMonitor(stale_after_s=0.5, frozen_window_s=1.0)
        for i in range(11):
            m.update(i * 0.1, (1.0, 2.0, 0.8))
        assert m.status(1.0) == 'frozen'
        for i in range(11, 21):
            m.update(i * 0.1, (1.0 + (i - 10) * 0.01, 2.0, 0.8))
        assert m.status(2.0) == 'ok'
