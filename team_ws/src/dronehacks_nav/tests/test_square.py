"""Tests for the pure square-mission engine (dronehacks_nav.square).

These cover the review findings on the square mission: Offboard loss must
actually pause the state machine (hold position, advance nothing), HOLD must
hold the position where it was entered (never an unreached corner), and a
disarm must reset everything so a re-arm starts a fresh square.

Runs under both pytest and the standard unittest runner; no ROS required
(the ROS node in mission.py is a thin adapter over this engine).
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronehacks_nav.square import SquareEngine  # noqa: E402

HOVER_Z = 1.6  # 0.8 m start height + 0.8 m hover altitude


def make_engine(**kwargs):
    params = dict(side_m=1.0, hover_altitude_m=0.8, hover_seconds=5.0,
                  corner_timeout_s=10.0, arrival_radius_m=0.12)
    params.update(kwargs)
    return SquareEngine(**params)


def drive_to_hover(engine, now=0.0, pos=(0.0, 0.0, 0.8)):
    """offboard on; climb to the hover height."""
    engine.tick(now, False, pos)            # waiting, holding
    engine.tick(now + 0.1, True, pos)       # WAIT -> HOVER
    return engine.tick(now + 0.2, True, (0.0, 0.0, HOVER_Z))  # at hover


def drive_to_square(engine, now=0.0):
    """through the hover dwell into the square's first corner."""
    drive_to_hover(engine, now)
    engine.tick(now + 5.5, True, (0.0, 0.0, HOVER_Z))   # dwell completes
    # fly to corner 1; arrival advances the square to corner 2
    return engine.tick(now + 5.8, True, (1.0, 0.0, HOVER_Z))


class TestOffboardPause(unittest.TestCase):
    def test_pause_holds_position_and_advances_nothing(self):
        eng = make_engine()
        drive_to_square(eng)
        self.assertEqual(eng.state, 'SQUARE')
        idx_before = eng.corner_index
        pause_pos = (1.0, 0.2, 1.5)
        all_events = []
        for now in range(100, 120):
            target, events = eng.tick(float(now), False, pause_pos)
            all_events += events
            self.assertEqual(target, pause_pos)      # holds where it is
            self.assertEqual(eng.state, 'SQUARE')    # no progression
            self.assertEqual(eng.corner_index, idx_before)
        # the pause was announced, exactly once
        lost = [e for e in all_events if e.startswith('Offboard lost')]
        self.assertEqual(len(lost), 1)

    def test_pause_during_hover_holds_position(self):
        eng = make_engine()
        drive_to_hover(eng)
        self.assertEqual(eng.state, 'HOVER')
        pause_pos = (0.1, 0.0, 1.55)
        target, _ = eng.tick(50.0, False, pause_pos)
        self.assertEqual(target, pause_pos)
        self.assertEqual(eng.state, 'HOVER')         # hover dwell not consumed


class TestDisarmReset(unittest.TestCase):
    def test_disarm_rearm_cycle_resets_mission_state(self):
        """A disarm clears everything; the re-armed attempt starts a fresh
        square from wherever the drone then sits."""
        eng = make_engine()
        drive_to_square(eng)
        self.assertEqual(eng.state, 'SQUARE')
        eng.reset()                                  # the kit 'armed' drop
        self.assertEqual(eng.state, 'WAIT_FOR_OFFBOARD')
        self.assertEqual(eng.corners, [])
        self.assertIsNone(eng.start)
        self.assertIsNone(eng.hold_target)
        self.assertIsNone(eng.hover_started_at)
        self.assertIsNone(eng.corner_started_at)
        # re-arm somewhere else entirely
        new_pos = (5.0, 5.0, 0.3)
        target, events = eng.tick(0.0, True, new_pos)
        self.assertEqual(eng.state, 'HOVER')          # fresh attempt
        self.assertEqual(eng.start, new_pos)         # fresh start point
        self.assertEqual(len(eng.corners), 4)
        # the new square is built around the new start, not the old one
        self.assertEqual(eng.corners[0], (6.0, 5.0, 1.1))

    def test_reset_clears_a_hold_latch(self):
        eng = make_engine()
        drive_to_square(eng)
        # corner 2 never reached -> timeout -> HOLD where the drone is
        eng.tick(20.0, True, (1.0, 0.1, 1.6))         # 15 s > 10 s timeout
        self.assertEqual(eng.state, 'HOLD')
        self.assertIsNotNone(eng.hold_target)
        eng.reset()
        self.assertIsNone(eng.hold_target)
        self.assertEqual(eng.state, 'WAIT_FOR_OFFBOARD')


class TestHold(unittest.TestCase):
    def test_hold_captures_current_position_not_the_last_corner(self):
        """On a corner timeout the drone holds where it actually is —
        never a corner it may never have reached."""
        eng = make_engine()
        drive_to_square(eng)                          # corner 2 next
        actual = (1.0, 0.1, 1.4)                       # 0.9 m from corner 2
        target, events = eng.tick(20.0, True, actual)  # past the 10 s timeout
        self.assertEqual(eng.state, 'HOLD')
        self.assertEqual(eng.hold_target, actual)
        self.assertNotEqual(eng.hold_target, eng.corners[-1])
        target, _ = eng.tick(21.0, True, actual)
        self.assertEqual(target, actual)

    def test_square_complete_holds_the_final_corner(self):
        eng = make_engine()
        drive_to_square(eng)
        corners = eng.corners
        for corner in corners[1:]:
            eng.tick(30.0, True, corner)
        self.assertEqual(eng.state, 'HOLD')
        self.assertEqual(eng.hold_target, corners[-1])


class TestProgression(unittest.TestCase):
    def test_full_mission_reaches_hold(self):
        eng = make_engine()
        pos = (0.0, 0.0, 0.8)
        eng.tick(0.0, False, pos)
        eng.tick(0.1, True, pos)
        # climb
        eng.tick(0.2, True, (0.0, 0.0, HOVER_Z))
        # hover dwell (hover timer starts on arrival)
        eng.tick(5.0, True, (0.0, 0.0, HOVER_Z))
        target, _ = eng.tick(5.5, True, (0.0, 0.0, HOVER_Z))
        self.assertEqual(eng.state, 'SQUARE')
        self.assertEqual(eng.corner_index, 0)
        # the next tick commands corner 1
        target, _ = eng.tick(5.6, True, (0.0, 0.0, HOVER_Z))
        self.assertEqual(target, (1.0, 0.0, HOVER_Z))
        # fly the four corners
        for corner in eng.corners:
            target, _ = eng.tick(6.0, True, corner)
        self.assertEqual(eng.state, 'HOLD')

    def test_no_odometry_commands_nothing(self):
        eng = make_engine()
        target, events = eng.tick(0.0, True, None)
        self.assertIsNone(target)
        self.assertTrue(any('no_odom' in e or 'cuVSLAM' in e for e in events))
        self.assertEqual(eng.state, 'WAIT_FOR_OFFBOARD')


if __name__ == '__main__':
    unittest.main()
