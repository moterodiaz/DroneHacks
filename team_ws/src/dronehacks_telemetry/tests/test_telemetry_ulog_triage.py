"""The PX4 log triage on synthetic pyulog shapes and signals: no log file, no drone."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dronehacks_telemetry import ulog_triage  # noqa: E402


def _smooth_steps(times_s: np.ndarray, starts_s: list[float], step: float,
                  duration_s: float) -> np.ndarray:
    """A signal climbing by `step` over `duration_s` from each of `starts_s`, smoothly."""
    phases = np.clip((times_s[:, None] - np.array(starts_s)) / duration_s, 0.0, 1.0)
    return step * (3 * phases**2 - 2 * phases**3).sum(axis=1)


def test_a_ulog_reads_into_topics_timed_from_the_logs_start() -> None:
    # pyulog's shapes: microsecond stamps from boot, one data set per topic instance
    status = SimpleNamespace(name="vehicle_status", multi_id=0,
                             data={"timestamp": np.array([2_000_000, 3_000_000], dtype=np.uint64),
                                   "nav_state": np.array([4, 14], dtype=np.uint8)})
    warning = SimpleNamespace(timestamp=2_500_000, message="Failsafe activated",
                              log_level_str=lambda: "WARNING")
    ulog = SimpleNamespace(start_timestamp=1_000_000, last_timestamp=11_000_000, data_list=[status],
                           initial_parameters={"EKF2_EV_DELAY": 0.0}, logged_messages=[warning],
                           get_version_info_str=lambda: "v1.16.1")

    flight = ulog_triage.flight_from_ulog(ulog, "px4_1.ulg")

    assert flight.version == "v1.16.1"
    assert flight.duration_s == pytest.approx(10.0)
    assert flight.topics["vehicle_status"][0].times_s == pytest.approx([1.0, 2.0])
    assert flight.topics["vehicle_status"][0].fields["nav_state"] == pytest.approx([4, 14])
    assert flight.parameters["EKF2_EV_DELAY"] == 0.0
    assert flight.messages == [(pytest.approx(1.5), "WARNING", "Failsafe activated")]


def test_a_flag_logged_on_change_is_held_between_changes_and_its_drops_listed() -> None:
    # Off until 10 s, on, off 20-22 s, on again; flying 5-30 s, so 18 of 25 s are on
    flag = ulog_triage.flag_time(np.array([0.0, 10.0, 20.0, 22.0]), np.array([0.0, 1.0, 0.0, 1.0]),
                                 [(5.0, 30.0)])

    assert flag.true_fraction == pytest.approx(0.72, abs=0.01)
    assert [start_s for start_s, _ in flag.dropouts] == pytest.approx([5.0, 20.0], abs=0.02)
    assert [duration_s for _, duration_s in flag.dropouts] == pytest.approx([5.0, 2.0], abs=0.02)


def test_innovation_statistics_count_samples_past_the_gate_and_skip_a_source_never_fused() -> None:
    stats = ulog_triage.innovation_stats(np.array([0.1, -0.1, 0.3, 0.0]),
                                         np.array([0.2, 0.2, 1.5, 0.1]))

    # The rms of 0.1, 0.1, 0.3 and 0; one sample of four has a test ratio past 1, the gate
    assert stats is not None
    assert stats.rms == pytest.approx(math.sqrt(0.11 / 4))
    assert stats.max_abs == pytest.approx(0.3)
    assert stats.max_test_ratio == pytest.approx(1.5)
    assert stats.over_gate_fraction == pytest.approx(0.25)
    assert ulog_triage.innovation_stats(np.zeros(4), np.zeros(4)) is None


def test_a_vision_yaw_stamped_late_is_found_lagging_the_ekf_by_that_delay_in_every_turn() -> None:
    # Three quarter turns; vision at 30 Hz shows each 60 ms after its stamp, in a frame 0.5 rad off
    ekf_times_s = np.arange(0.0, 35.0, 1 / 250)
    vision_times_s = np.arange(0.0, 35.0, 1 / 30)
    turns_s, delay_s = [5.0, 15.0, 25.0], 0.06
    ekf_yaw = ulog_triage.wrap(_smooth_steps(ekf_times_s, turns_s, math.pi / 2, 2.0))
    vision_yaw = ulog_triage.wrap(
        _smooth_steps(vision_times_s - delay_s, turns_s, math.pi / 2, 2.0) + 0.5)

    ekf_rate = ulog_triage.yaw_rate(ekf_times_s, ekf_yaw, np.zeros_like(ekf_times_s))
    vision_rate = ulog_triage.yaw_rate(vision_times_s, vision_yaw, np.zeros_like(vision_times_s))
    lag = ulog_triage.estimate_lag(ekf_rate, vision_rate, [(1.0, 34.0)], ulog_triage.TURN_RATE_RAD_S)

    # The lag grid's step is 5 ms
    assert lag.overall_s == pytest.approx(delay_s, abs=0.006)
    assert lag.per_segment_s == pytest.approx([delay_s] * 3, abs=0.006)
    assert lag.left_out == 0


def test_a_turn_across_a_vision_gap_is_left_out_of_the_lag() -> None:
    # The same turns timed exactly, but vision is silent for a second through the middle one
    ekf_times_s = np.arange(0.0, 35.0, 1 / 250)
    vision_times_s = np.arange(0.0, 35.0, 1 / 30)
    vision_times_s = vision_times_s[(vision_times_s < 15.5) | (vision_times_s > 16.5)]
    turns_s = [5.0, 15.0, 25.0]
    ekf_yaw = ulog_triage.wrap(_smooth_steps(ekf_times_s, turns_s, math.pi / 2, 2.0))
    vision_yaw = ulog_triage.wrap(_smooth_steps(vision_times_s, turns_s, math.pi / 2, 2.0))

    ekf_rate = ulog_triage.yaw_rate(ekf_times_s, ekf_yaw, np.zeros_like(ekf_times_s))
    vision_rate = ulog_triage.yaw_rate(vision_times_s, vision_yaw, np.zeros_like(vision_times_s))
    lag = ulog_triage.estimate_lag(ekf_rate, vision_rate, [(1.0, 34.0)], ulog_triage.TURN_RATE_RAD_S)

    assert lag.per_segment_s == pytest.approx([0.0, 0.0])
    assert lag.left_out == 1


def test_a_vision_reset_does_not_count_as_motion_when_finding_the_position_lag() -> None:
    # Two legs north and back; vision 40 ms late, and its frame jumps 5 m east at a reset at 12 s
    ekf_times_s = np.arange(0.0, 30.0, 1 / 100)
    vision_times_s = np.arange(0.0, 30.0, 1 / 30)
    legs_s, delay_s = [5.0, 18.0], 0.04
    ekf_north = (_smooth_steps(ekf_times_s, legs_s[:1], 3.0, 3.0)
                 - _smooth_steps(ekf_times_s, legs_s[1:], 3.0, 3.0))
    vision_north = (_smooth_steps(vision_times_s - delay_s, legs_s[:1], 3.0, 3.0)
                    - _smooth_steps(vision_times_s - delay_s, legs_s[1:], 3.0, 3.0))
    resets = (vision_times_s >= 12.0).astype(float)
    ekf_xy = np.column_stack([ekf_north, np.zeros_like(ekf_north)])
    vision_xy = np.column_stack([vision_north, 5.0 * resets])

    ekf_speed = ulog_triage.horizontal_speed(ekf_times_s, ekf_xy, np.zeros_like(ekf_times_s))
    vision_speed = ulog_triage.horizontal_speed(vision_times_s, vision_xy, resets)
    lag = ulog_triage.estimate_lag(ekf_speed, vision_speed, [(1.0, 29.0)], ulog_triage.MOVE_SPEED_M_S)

    assert lag.overall_s == pytest.approx(delay_s, abs=0.006)
    assert len(lag.per_segment_s) == 2


def test_the_delay_whose_logs_have_the_lowest_mean_ev_innovation_rms_is_suggested() -> None:
    # Two logs at 0 ms average 0.06 m; the single logs at 30 and 70 ms read 0.045 and 0.04 m
    rows = [(0.0, 0.05), (0.0, 0.07), (70.0, 0.04), (30.0, 0.045)]

    assert ulog_triage.best_delay_ms(rows) == 70.0


@pytest.mark.parametrize(("p95_m_s2", "grade"), [(3.0, "good"), (6.0, "high"), (12.0, "too high")])
def test_the_accel_vibration_is_graded_in_flight_reviews_bands(p95_m_s2: float, grade: str) -> None:
    # Flight Review shades its vibration plot green below 0.5 g, orange to 1 g and red above
    assert ulog_triage.vibration_grade(p95_m_s2) == grade


def test_a_failsafe_flag_reports_each_time_it_rose_and_fell() -> None:
    rises = ulog_triage.rises(np.arange(5.0), np.array([1.0, 0.0, 1.0, 1.0, 1.0]))

    # True at the log's first sample counts as a rise there; still true at the end, it never fell
    assert rises == [(0.0, 1.0), (2.0, None)]


def test_a_rangefinder_jump_is_counted_only_between_valid_readings() -> None:
    # The 4th reading has quality 0, the 5th is past the sensor's 4 m maximum
    times_s = np.arange(6.0)
    distance_m = np.array([1.0, 1.02, 0.2, 1.0, 5.0, 0.21])
    valid = ulog_triage.range_valid(distance_m, np.array([100, 100, 100, 0, 100, -1]), 0.05, 4.0)

    jumps = ulog_triage.range_jumps(times_s, distance_m, valid, ulog_triage.RANGE_JUMP_M)

    assert valid.tolist() == [True, True, True, False, False, True]
    assert jumps == [(2.0, 1.02, 0.2)]


def test_the_vision_rate_and_age_come_from_its_two_stamps() -> None:
    # 30 Hz with one dropped sample; each arrives 4 ms after its capture, the last tenth 10 ms
    times_s = np.delete(np.arange(0.0, 3.0, 1 / 30), 45)
    sample_times_s = times_s - np.where(np.arange(times_s.size) < 80, 0.004, 0.010)

    timing = ulog_triage.vision_timing(times_s, sample_times_s)

    assert timing.rate_hz == pytest.approx(30.0)
    assert timing.longest_gap_s == pytest.approx(2 / 30)
    assert timing.age_p50_s == pytest.approx(0.004)
    assert timing.age_p95_s == pytest.approx(0.010)


def test_a_log_missing_topics_still_reports_what_it_has() -> None:
    # Only the vehicle's status: armed in position mode at 2 s, Offboard at 3 s
    status = ulog_triage.Topic(times_s=np.array([1.0, 2.0, 3.0]),
                               fields={"nav_state": np.array([2.0, 2.0, 14.0]),
                                       "arming_state": np.array([1.0, 2.0, 2.0]),
                                       "failsafe": np.zeros(3)})
    flight = ulog_triage.Flight(name="px4_1.ulg", version="v1.16.1", duration_s=10.0, parameters={},
                                topics={"vehicle_status": [status]}, messages=[])

    text = "\n".join(ulog_triage.report(flight))

    assert "vehicle_visual_odometry not logged" in text
    assert "OFFBOARD" in text
    assert "armed" in text
