"""The push compensator: its frames and signs, the offset after a yaw and its decay, the cap, and
staying off without an accelerometer."""
from __future__ import annotations

import math

import pytest

from dronehacks_course_follower.yaw_comp import (DEFAULT_GAIN_PER_S2, DEFAULT_MAX_M, DEFAULT_TAU_S, MEASURE_S,
                                                 HorizontalSpeed, YawCompensator, rotated)

# The node's tick
_PERIOD_S = 0.02

# A push forward and to the left of the nose, m/s², as the accelerometer reads it in FRD (left = -y)
_PUSH_FORWARD, _PUSH_LEFT = 0.12, 0.07
_ACCEL_FRD = (_PUSH_FORWARD, -_PUSH_LEFT)


def _estimated(yaw_rad: float = 0.0, accel_frd: tuple[float, float] = _ACCEL_FRD, push_gain: float = 1.0,
               max_m: float = DEFAULT_MAX_M) -> tuple[YawCompensator, float]:
    """A compensator that has hovered steady at `yaw_rad` until its first estimate, and the time it got there."""
    compensator = YawCompensator(max_m=max_m, push_gain=push_gain)
    now_s = 0.0
    while not compensator.ready:
        compensator.add_accel(*accel_frd)
        compensator.step(now_s, yaw_rad, speed_mps=0.0)
        now_s += _PERIOD_S

    return compensator, now_s


def test_the_push_reads_forward_from_ax_and_left_from_minus_ay_times_the_push_gain() -> None:
    compensator, _ = _estimated(push_gain=1.25)
    assert compensator.push_body.x == pytest.approx(1.25 * _PUSH_FORWARD)
    assert compensator.push_body.y == pytest.approx(1.25 * _PUSH_LEFT)


def test_facing_north_the_nose_points_along_odom_y() -> None:
    east, north = rotated(math.pi / 2, 1.0, 0.0)
    assert east == pytest.approx(0.0, abs=1e-12) and north == pytest.approx(1.0)


def test_the_estimate_waits_for_its_hover_and_the_offset_is_zero_without_a_yaw() -> None:
    compensator, now_s = _estimated(yaw_rad=0.7)
    assert now_s >= MEASURE_S
    for step in range(500):
        assert compensator.step(now_s + step * _PERIOD_S, 0.7, speed_mps=0.0) == (0.0, 0.0)


def test_after_a_quarter_turn_the_offset_is_the_unlearned_push_and_decays_with_tau() -> None:
    compensator, now_s = _estimated()
    learned = rotated(0.0, _PUSH_FORWARD, _PUSH_LEFT)
    push = rotated(math.pi / 2, _PUSH_FORWARD, _PUSH_LEFT)

    # The tick the heading arrives: PX4's integrator has moved one tick's share towards the new push
    share = _PERIOD_S / DEFAULT_TAU_S
    first = compensator.step(now_s, math.pi / 2, speed_mps=0.0)
    for got, new, old in zip(first, push, learned, strict=True):
        assert got == pytest.approx(-(1.0 - share) * (new - old) / DEFAULT_GAIN_PER_S2)

    # One time constant on, the first-order lag has left e^-1 of it
    ticks = round(DEFAULT_TAU_S / _PERIOD_S)
    for tick in range(1, ticks + 1):
        later = compensator.step(now_s + tick * _PERIOD_S, math.pi / 2, speed_mps=0.0)
    assert math.hypot(*later) / math.hypot(*first) == pytest.approx(math.exp(-1.0), rel=0.01)


def test_a_push_straight_ahead_turned_left_offsets_the_setpoint_east_and_south() -> None:
    # Forward 0.1 m/s² facing east, learned east; facing north it pushes north while PX4 still leans
    # west against the old push, so the drone would drift north-west: the offset points east and south
    compensator, now_s = _estimated(accel_frd=(0.1, 0.0))
    east, north = compensator.step(now_s, math.pi / 2, speed_mps=0.0)
    share = _PERIOD_S / DEFAULT_TAU_S
    assert east == pytest.approx((1.0 - share) * 0.1 / DEFAULT_GAIN_PER_S2)
    assert north == pytest.approx(-(1.0 - share) * 0.1 / DEFAULT_GAIN_PER_S2)


def test_the_offset_is_capped() -> None:
    compensator, now_s = _estimated(accel_frd=(2.0, 0.0), max_m=0.05)
    offset = compensator.step(now_s, math.pi, speed_mps=0.0)
    assert math.hypot(*offset) == pytest.approx(0.05)


def test_without_accelerometer_data_it_never_estimates_and_never_offsets() -> None:
    compensator = YawCompensator()
    for step in range(500):
        assert compensator.step(step * _PERIOD_S, step * 0.01, speed_mps=0.0) == (0.0, 0.0)

    assert not compensator.ready and not compensator.imu_fresh(10.0)


def test_moving_turning_or_on_the_ground_never_counts_as_steady() -> None:
    for speed_mps, yaw_rate_rad_s, airborne in [(0.3, 0.0, True), (0.0, math.radians(10.0), True),
                                                (0.0, 0.0, False)]:
        compensator = YawCompensator()
        for step in range(500):
            compensator.add_accel(*_ACCEL_FRD)
            compensator.step(step * _PERIOD_S, yaw_rate_rad_s * step * _PERIOD_S, speed_mps, airborne)
        assert not compensator.ready, (speed_mps, yaw_rate_rad_s, airborne)


def test_settings_that_cannot_work_are_refused() -> None:
    with pytest.raises(ValueError, match="yaw_comp"):
        YawCompensator(tau_s=0.0)


def test_the_horizontal_speed_settles_on_the_drones() -> None:
    speed = HorizontalSpeed()
    for step in range(100):
        speed.update(step * 0.1, 0.1 * step, 0.0)

    assert speed.speed_mps == pytest.approx(1.0)


def _peak_drift_m(compensate: bool) -> float:
    """Peak drift of PX4's stock position and velocity loops holding a point through a +90 deg yaw
    at 20 deg/s, with a push fixed to the airframe that its integrator had learned at the start."""
    position_gain, velocity_gain, integral_gain, period_s = 0.95, 1.8, 0.4, 0.05
    push_body = (0.2, 0.1)
    compensator = YawCompensator()
    compensator.push_gain = 1.0
    position, velocity = [0.0, 0.0], [0.0, 0.0]
    integral = [-value for value in rotated(0.0, *push_body)]
    yaw_rad, peak_m = 0.0, 0.0
    for tick in range(int(30.0 / period_s)):
        now_s = tick * period_s
        if now_s > 5.0:
            yaw_rad = min(math.pi / 2, yaw_rad + math.radians(20.0) * period_s)

        # The plant's push is the accelerometer's reading, so the push gain is 1
        compensator.add_accel(push_body[0], -push_body[1])
        offset = compensator.step(now_s, yaw_rad, math.hypot(*velocity)) if compensate else (0.0, 0.0)
        push = rotated(yaw_rad, *push_body)
        for axis in range(2):
            velocity_error = position_gain * (offset[axis] - position[axis]) - velocity[axis]
            integral[axis] += integral_gain * velocity_error * period_s
            velocity[axis] += (velocity_gain * velocity_error + integral[axis] + push[axis]) * period_s
            position[axis] += velocity[axis] * period_s
        if now_s > 5.0:
            peak_m = max(peak_m, math.hypot(*position))

    return peak_m


def test_in_px4s_stock_loops_the_offset_cancels_most_of_the_drift_a_yaw_causes() -> None:
    assert _peak_drift_m(compensate=True) < 0.2 * _peak_drift_m(compensate=False)
