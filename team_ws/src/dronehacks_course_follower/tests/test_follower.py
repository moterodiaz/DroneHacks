"""The follower's kinematics: the led pose, the pace within PX4's limits, and the look-ahead heading."""
from __future__ import annotations

import math

import numpy as np

from dronehacks_course_follower.follower import (ACCEL_M_S2, JERK_M_S3, LOOKAHEAD_M, PX4_XY_P, PX4_XY_VEL_P_ACC,
                                                 PX4_YAW_P, TURN_ACCEL_SCALE, TURN_RATE_DPS, Drone, EasedSpeed,
                                                 Follower, LedPose, PathMotion, _curvature, eased_speed, led_pose,
                                                 speed_profile)

# The follower's tick, as the node runs it at 50 Hz
_PERIOD_S = 0.02

# An L-shaped route in open space: 3 m east, then 3 m north
_L_ROUTE = [(3.0, 0.0), (3.0, 3.0)]

# The first leg's heading is judged this far before the corner: the corner's arc starts 0.4 m
# before it, the look-ahead reaches `LOOKAHEAD_M` past the drone, and the heading lags its bearing
CORNER_LEEWAY_M = 0.4 + LOOKAHEAD_M + 0.3


def _follow_ideally(follower: Follower, seconds: float) -> list[tuple[LedPose, Drone]]:
    """Steps the follower with a drone that sits on its reference, facing its heading; each led
    pose with the drone it was sent to."""
    drone = Drone(0.0, 0.0, 1.0, follower.yaw_rad)
    steps = []
    for _ in range(int(seconds / _PERIOD_S)):
        pose = follower.step(_PERIOD_S, drone)
        steps.append((pose, drone))
        east, north = follower.reference()
        drone = Drone(east, north, 1.0, follower.yaw_rad)

    return steps


def test_the_led_pose_stands_ahead_by_velocity_over_p_and_acceleration_over_p_times_p_acc() -> None:
    pose = led_pose(LedPose(1.0, 2.0, 0.5), PathMotion((0.6, -0.3), (1.2, 0.4)), yaw_rate=0.7)
    assert math.isclose(pose.east, 1.0 + 0.6 / PX4_XY_P + 1.2 / (PX4_XY_P * PX4_XY_VEL_P_ACC))
    assert math.isclose(pose.north, 2.0 - 0.3 / PX4_XY_P + 0.4 / (PX4_XY_P * PX4_XY_VEL_P_ACC))
    assert math.isclose(pose.yaw_rad, 0.5 + 0.7 / PX4_YAW_P)


def test_a_pose_with_no_heading_keeps_none_when_led() -> None:
    assert math.isnan(led_pose(LedPose(0.0, 0.0, math.nan), PathMotion((1.0, 0.0), (0.0, 0.0)), 0.5).yaw_rad)


def test_the_speed_profile_rests_at_both_ends_and_keeps_turns_within_px4s_acceleration() -> None:
    follower = Follower((0.0, 0.0), _L_ROUTE, np.empty((0, 4)), 0.0, top_speed_mps=1.0)
    speeds = speed_profile(follower.path, 1.0)
    assert speeds[0] == 0.0 and speeds[-1] == 0.0
    assert speeds.max() <= 1.0

    # Lateral acceleration v^2 k on the corner's arc within the scaled limit, to the sampling's rounding
    lateral = [speeds[index] ** 2 * _curvature(*follower.path[index - 1:index + 2])
               for index in range(1, len(speeds) - 1)]
    assert max(lateral) <= ACCEL_M_S2 * TURN_ACCEL_SCALE * 1.01


def test_eased_speed_climbs_to_its_target_within_the_acceleration_and_jerk_limits() -> None:
    motion, previous_accel, speeds = EasedSpeed(0.0, 0.0), 0.0, []
    for _ in range(200):
        motion = eased_speed(motion.speed_mps, motion.accel_m_s2, 1.0, _PERIOD_S)
        assert motion.accel_m_s2 <= ACCEL_M_S2 + 1e-9

        # The last step lands on the target; every other one keeps within the jerk limit
        if motion.speed_mps < 1.0:
            assert abs(motion.accel_m_s2 - previous_accel) <= JERK_M_S3 * _PERIOD_S + 1e-9
        previous_accel = motion.accel_m_s2
        speeds.append(motion.speed_mps)

    assert speeds[-1] == 1.0
    assert all(later >= earlier for earlier, later in zip(speeds, speeds[1:], strict=False))


def test_the_heading_turns_no_faster_than_the_turn_rate_and_faces_ahead_on_a_straight() -> None:
    follower = Follower((0.0, 0.0), _L_ROUTE, np.empty((0, 4)), math.pi / 2, top_speed_mps=1.0, lead=False)
    steps = _follow_ideally(follower, 12.0)
    headings = np.unwrap([pose.yaw_rad for pose, _ in steps])
    assert np.abs(np.diff(headings)).max() <= math.radians(TURN_RATE_DPS) * _PERIOD_S + 1e-9

    # Halfway up the second leg the look-ahead point is straight ahead, north
    index = next(index for index, (_, drone) in enumerate(steps) if drone.north > 1.5)
    assert abs(headings[index] - math.pi / 2) < math.radians(1.0)

    # On the first leg, before the corner comes within the look-ahead, it faced east
    index = next(index for index, (_, drone) in enumerate(steps) if drone.east > 3.0 - CORNER_LEEWAY_M)
    assert abs(headings[index]) < math.radians(1.0)


def test_the_follower_reaches_the_end_of_its_route_and_stops() -> None:
    follower = Follower((0.0, 0.0), _L_ROUTE, np.empty((0, 4)), 0.0, top_speed_mps=1.0)
    _follow_ideally(follower, 15.0)
    assert follower.finished
    assert math.dist(follower.reference(), _L_ROUTE[-1]) < 0.05


def test_the_setpoint_waits_while_the_drones_estimate_is_missing() -> None:
    follower = Follower((0.0, 0.0), _L_ROUTE, np.empty((0, 4)), 0.0, top_speed_mps=1.0)
    for _ in range(100):
        follower.step(_PERIOD_S, None)

    assert follower.along_m == 0.0
