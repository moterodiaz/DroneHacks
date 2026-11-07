"""The flight report's measures on small synthetic flights: no ROS, no bag, no drone."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dronehacks_telemetry import analyze, course, metrics  # noqa: E402
from dronehacks_telemetry.bag import bag_flight_from_rows, resolve_roles  # noqa: E402


def _square_course() -> course.CourseMap:
    """A 4 m square of walls around the origin, a launch mark facing north and one boulder."""
    corners = [(-2, -2), (2, -2), (2, 2), (-2, 2)]
    segments = [{"start": list(corners[index]), "end": list(corners[(index + 1) % 4]), "thickness": 0.1}
                for index in range(4)]
    return course.course_map_from_dict({
        "segments": segments, "boulders": [{"centre": [1.0, -1.0], "radius": 0.2}],
        "goals": [{"name": "launch", "centre": [-1.0, 0.0], "params": {"heading_deg": 0.0}},
                  {"name": "pad", "centre": [1.0, 1.0], "params": {"radius": 0.5}}]})


def test_a_waypoint_mission_splits_into_one_leg_per_setpoint() -> None:
    # Two waypoints 1 m north then 1 m east, sent at 10 Hz in Offboard from 0 to 10 s
    setpoint_times_s = np.arange(0.0, 10.0, 0.1)
    setpoints = np.where(setpoint_times_s[:, None] < 5.0, [1.0, 0.0, -0.8], [1.0, 1.0, -0.8])
    estimate_times_s = np.arange(0.0, 10.0, 0.02)
    estimate = np.where(estimate_times_s[:, None] < 5.0, [1.0, 0.0, -0.8], [1.0, 1.0, -0.8])

    legs = metrics.legs(setpoint_times_s, setpoints, estimate_times_s, estimate,
                        np.zeros_like(estimate_times_s), [metrics.Span(0.0, 10.0)], 0.12)

    assert [leg.target_ned for leg in legs] == [(1.0, 0.0, -0.8), (1.0, 1.0, -0.8)]
    assert legs[1].start_s == pytest.approx(5.0)
    assert legs[1].end_s == 10.0


def test_a_leg_measures_its_overshoot_cross_track_reach_and_heading_swing() -> None:
    # 2 m north; the estimate drifts 0.1 m east on the way, passes the target by 0.3 m, comes back
    times_s = np.linspace(0.0, 10.0, 501)
    north = np.interp(times_s, [0.0, 4.0, 6.0, 10.0], [0.0, 2.3, 2.0, 2.0])
    east = np.interp(times_s, [0.0, 2.0, 4.0, 10.0], [0.0, 0.1, 0.0, 0.0])
    estimate = np.column_stack([north, east, np.full_like(times_s, -0.8)])
    heading = np.interp(times_s, [0.0, 5.0, 10.0], [0.0, 0.2, -0.1])

    leg, = metrics.legs(np.array([0.0]), np.array([[2.0, 0.0, -0.8]]), times_s, estimate, heading,
                        [metrics.Span(0.0, 10.0)], 0.12)

    # North 2.3 m at 4 s is 0.3 m past; inside 0.12 m of the target first at north 1.88 m
    assert leg.overshoot_m == pytest.approx(0.3, abs=1e-6)
    assert leg.cross_track_m == pytest.approx(0.1, abs=1e-6)
    assert leg.reach_s == pytest.approx(4.0 * 1.88 / 2.3, abs=0.03)
    assert leg.final_error_m == pytest.approx(0.0, abs=1e-9)
    assert leg.heading_swing_deg == pytest.approx(math.degrees(0.3))


def test_a_leg_never_reached_has_no_reach_time_and_a_velocity_setpoint_is_skipped() -> None:
    times_s = np.arange(0.0, 4.0, 0.1)
    estimate = np.zeros((times_s.size, 3))
    setpoints = np.array([[np.nan, np.nan, np.nan], [3.0, 0.0, 0.0]])

    leg, = metrics.legs(np.array([0.0, 1.0]), setpoints, times_s, estimate, np.zeros_like(times_s),
                        [metrics.Span(0.0, 4.0)], 0.12)

    assert leg.start_s == 1.0
    assert leg.reach_s is None
    assert leg.final_error_m == pytest.approx(3.0)


def test_a_stream_reports_its_rate_from_stamps_and_each_gap_on_the_receive_clock() -> None:
    # 30 Hz stamps with half a second missing after 2 s; received 0.1 s after each stamp, 100 s on
    stamps_s = np.arange(0.0, 5.0, 1 / 30)
    stamps_s = stamps_s[(stamps_s < 2.0) | (stamps_s > 2.5)]

    timing = metrics.stream_timing(stamps_s + 100.1, stamps_s)

    assert timing is not None
    assert timing.rate_hz == pytest.approx(30.0)
    assert len(timing.gaps) == 1
    # The last stamp before the hole is 59/30 s and the first after it 76/30 s
    assert timing.gaps[0].start_s == pytest.approx(100.1 + 59 / 30)
    assert timing.longest_gap_s == pytest.approx(17 / 30)


def test_flying_is_armed_and_not_landed() -> None:
    armed = [metrics.Span(1.0, 9.0)]

    flying = metrics.flying_spans(armed, np.array([0.0, 2.0, 8.0]), np.array([1.0, 0.0, 1.0]), 10.0)

    assert flying == [pytest.approx(metrics.Span(2.0, 8.0), abs=0.03)]


def test_the_rangefinder_offset_is_taken_out_before_its_disagreement_is_measured() -> None:
    # The sensor sits 0.05 m above the estimate's origin; one reading 0.2 m off, one invalid
    times_s = np.arange(10.0)
    height_m = np.full(10, 0.8)
    range_m = height_m + 0.05
    range_m[4] += 0.2
    valid = np.ones(10, dtype=bool)
    valid[7] = False

    agreement = metrics.height_agreement(times_s, height_m, range_m, valid, [metrics.Span(0.0, 9.0)])

    assert agreement is not None
    assert agreement.offset_m == pytest.approx(0.05)
    assert agreement.worst_m == pytest.approx(0.2)
    assert agreement.valid_fraction == pytest.approx(0.9)


def test_a_point_is_as_far_from_a_wall_as_from_its_face() -> None:
    walls = _square_course()

    # Inside the square, 0.5 m from the east wall's centre line, 0.45 m from its face
    clearance = course.wall_clearance_m(np.array([[1.5, 0.0], [0.0, 0.0]]), walls)

    assert clearance == pytest.approx([0.45, 1.95])


def test_a_track_passing_within_the_prop_reach_of_a_wall_counts_one_touch() -> None:
    walls = _square_course()
    times_s = np.arange(6.0)
    track = np.array([[0.0, 0.0], [1.0, 0.0], [1.8, 0.0], [1.8, 0.5], [1.0, 0.5], [0.0, 0.0]])

    result = metrics.clearance(times_s, track, course.wall_clearance_m(track, walls),
                               [metrics.Span(0.0, 5.0)], walls.goals, course.PROP_REACH_M)

    # 1.8 m east is 0.15 m from the east wall's face: inside the 0.28 m reach for two samples
    assert result is not None
    assert result.nearest_m == pytest.approx(0.15)
    assert result.touches == [metrics.Span(2.0, 4.0)]
    assert result.goals_m["pad"] == pytest.approx(math.hypot(0.0, 0.5))


def test_odom_placed_with_the_launch_transform_lands_ahead_of_the_mark_along_its_heading() -> None:
    walls = _square_course()

    # The tracker starts on the mark facing north: 1 m forward (odom +x) is 1 m north of the mark
    placed, yaw = course.course_from_odom(np.array([[1.0, 0.0, 0.5]]), np.array([0.0]), walls,
                                          course.launch_transform(walls))

    assert placed[0] == pytest.approx([-1.0, 1.0, 0.5])
    assert yaw[0] == pytest.approx(math.pi / 2)


def test_the_course_to_odom_transform_inverts_the_missions() -> None:
    # The mission's p_odom = o_odom + R(yaw) (p_course - launch), for a course point (0.5, 1.5)
    walls = _square_course()
    transform = course.CourseToOdom(yaw_rad=0.4, odom_x_m=0.2, odom_y_m=-0.1)
    dx, dy = 0.5 - walls.launch[0], 1.5 - walls.launch[1]
    odom = np.array([[0.2 + math.cos(0.4) * dx - math.sin(0.4) * dy,
                      -0.1 + math.sin(0.4) * dx + math.cos(0.4) * dy, 0.0]])

    placed, _ = course.course_from_odom(odom, np.zeros(1), walls, transform)

    assert placed[0, :2] == pytest.approx([0.5, 1.5])


@pytest.mark.parametrize("frame", list(course.Px4Frame))
def test_each_px4_frame_mapping_round_trips_a_forward_step(frame: course.Px4Frame) -> None:
    # PX4's NED for one metre forward at heading 0 in the launch frame, or due east in the odom one
    ned = np.array([[1.0, 0.0, -0.5]]) if frame is course.Px4Frame.LAUNCH else np.array([[0.0, 1.0, -0.5]])
    heading = np.array([0.0]) if frame is course.Px4Frame.LAUNCH else np.array([math.pi / 2])

    odom, yaw = course.odom_from_px4(ned, heading, frame)

    assert odom[0] == pytest.approx([1.0, 0.0, 0.5])
    assert yaw[0] == pytest.approx(0.0)
    assert course.px4_from_odom(odom, frame) == pytest.approx(ned)


def test_the_missions_transform_is_read_from_its_parameter_dump_and_a_nan_is_refused(tmp_path: Path) -> None:
    params = tmp_path / "meta" / "params"
    params.mkdir(parents=True)
    (params / "flight_interface.yaml").write_text("/flight_interface:\n  ros__parameters:\n"
                                                   "    max_speed_mps: 1.0\n", encoding="utf-8")
    (params / "dronehacks_full_mission.yaml").write_text(
        "/dronehacks_full_mission:\n  ros__parameters:\n    course_to_odom_x: 0.25\n"
        "    course_to_odom_y: -0.5\n    course_to_odom_yaw: -1.5708\n", encoding="utf-8")

    assert analyze.mission_transform(tmp_path) == course.CourseToOdom(-1.5708, 0.25, -0.5)

    (params / "dronehacks_full_mission.yaml").write_text(
        "/m:\n  ros__parameters:\n    course_to_odom_x: .nan\n    course_to_odom_y: .nan\n"
        "    course_to_odom_yaw: .nan\n", encoding="utf-8")
    assert analyze.mission_transform(tmp_path) is None


def test_versioned_px4_topic_names_resolve_to_their_role() -> None:
    resolved = resolve_roles(["/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_local_position",
                              "/fmu/out/vehicle_local_position_v1", "/visual_slam/tracking/odometry"])

    # The unversioned name is the shorter, so it wins where both are published
    assert resolved["status"] == "/fmu/out/vehicle_status_v1"
    assert resolved["local_position"] == "/fmu/out/vehicle_local_position"
    assert "bridge" not in resolved


def _stamp(time_s: float) -> SimpleNamespace:
    """A ROS header stamped at `time_s`."""
    return SimpleNamespace(stamp=SimpleNamespace(sec=int(time_s), nanosec=round((time_s % 1) * 1e9)))


def _synthetic_flight() -> dict[str, list[tuple[int, object]]]:
    """A 12 s flight as decoded rows: armed at 1 s, Offboard 2-10 s to one waypoint 1 m north,
    cuVSLAM at 30 Hz with a 0.5 s hole at 6 s and lost tracking 6-6.5 s, disarmed at 11 s."""
    start_ns = 1_700_000_000 * 10**9
    rows: dict[str, list[tuple[int, object]]] = {role: [] for role in
                                                 ("status", "local_position", "trajectory_setpoint",
                                                  "vision", "vision_status", "bridge")}
    for time_s in np.arange(0.0, 12.0, 0.1):
        log_ns = start_ns + round(time_s * 1e9)
        armed = 1.0 <= time_s < 11.0
        nav = 14 if 2.0 <= time_s < 10.0 else 2
        rows["status"].append((log_ns, SimpleNamespace(nav_state=nav, arming_state=2 if armed else 1,
                                                       failsafe=False)))
        north = float(np.clip((time_s - 2.0) / 3.0, 0.0, 1.0))
        rows["local_position"].append((log_ns, SimpleNamespace(
            x=north, y=0.0, z=-0.8 if armed else 0.0, heading=0.0, dist_bottom=0.85 if armed else 0.05,
            dist_bottom_valid=True, xy_reset_counter=0)))
        rows["trajectory_setpoint"].append((log_ns, SimpleNamespace(position=[1.0, 0.0, -0.8], yaw=0.0)))
        rows["vision_status"].append((log_ns, SimpleNamespace(vo_state=2 if 6.0 <= time_s < 6.5 else 1)))

    for time_s in np.arange(0.0, 12.0, 1 / 30):
        if 6.0 < time_s < 6.5:
            continue

        # Vision arrives 40 ms after its image stamp; the bridge 5 ms later still
        log_ns = start_ns + round((time_s + 0.04) * 1e9)
        stamp_s = 1_700_000_000 + time_s
        north = float(np.clip((time_s - 2.0) / 3.0, 0.0, 1.0))
        rows["vision"].append((log_ns, SimpleNamespace(header=_stamp(stamp_s), pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=north, z=0.8),
                                 orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))))
        rows["bridge"].append((log_ns + 5_000_000, SimpleNamespace(
            timestamp=round((stamp_s + 0.042) * 1e6), timestamp_sample=round(stamp_s * 1e6),
            reset_counter=1 if time_s > 6.5 else 0, pose_frame=2)))

    return rows


def test_a_synthetic_flight_reports_its_leg_gap_tracking_loss_and_latency(tmp_path: Path) -> None:
    rows = _synthetic_flight()
    resolved = {role: f"/{role}" for role in rows}
    bag = bag_flight_from_rows(rows, dict.fromkeys(resolved.values(), 1), resolved)
    args = argparse.Namespace(flight=tmp_path, course_map=None, course_to_odom=None, px4_frame="auto",
                              arrival_radius_m=0.12, ulog=None, truth=None, out=tmp_path)

    lines, legs = analyze.report(bag, args)
    text = "\n".join(lines)
    analyze.write_legs_csv(tmp_path / "legs.csv", legs)

    # One leg; the hole and the loss land on the timeline; latency is the 40 ms receive delay
    assert len(legs) == 1
    assert legs[0].target_ned == (1.0, 0.0, -0.8)
    assert legs[0].final_error_m == pytest.approx(0.0, abs=1e-6)
    assert legs[0].vision_gaps == 1
    assert "vision gap 0.50 s" in text
    assert "cuVSLAM vo_state 2" in text
    assert "1 while flying" in text
    assert "image -> cuVSLAM odometry received: p50 40.0 ms" in text
    assert "bridge's own age (timestamp - timestamp_sample): p50 42.0 ms" in text
    assert "bridge reset_counter changes: 1" in text
    assert "mode OFFBOARD" in text
    with (tmp_path / "legs.csv").open(encoding="utf-8") as stream:
        assert [row["target_n"] for row in csv.DictReader(stream)] == ["1.000"]


def test_the_synthetic_flights_px4_frame_is_chosen_by_its_agreement_with_vision(tmp_path: Path) -> None:
    # The synthetic PX4 moves north while vision moves +y: the `odom` mapping (ENU from NED) agrees
    rows = _synthetic_flight()
    resolved = {role: f"/{role}" for role in rows}
    bag = bag_flight_from_rows(rows, dict.fromkeys(resolved.values(), 1), resolved)
    args = argparse.Namespace(flight=tmp_path, px4_frame="auto")

    frame, lines = analyze._px4_frame(bag, args, [metrics.Span(1.0, 11.0)])

    assert frame is course.Px4Frame.ODOM
    assert any("`odom` mapping vs cuVSLAM odometry: median 0.00 m" in line for line in lines)


def test_without_the_kits_px4_setpoint_the_pose_setpoint_is_tracked_in_px4s_frame(tmp_path: Path) -> None:
    # The FlyX lab records no /fmu/in/trajectory_setpoint: the competitor's ENU pose stands in
    rows = _synthetic_flight()
    rows.pop("trajectory_setpoint")
    rows["pose_setpoint"] = [(log_ns, SimpleNamespace(header=_stamp(0.0), pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=1.0, z=0.8),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))) for log_ns, _ in rows["status"]]
    resolved = {role: f"/{role}" for role in rows}
    bag = bag_flight_from_rows(rows, dict.fromkeys(resolved.values(), 1), resolved)
    args = argparse.Namespace(flight=tmp_path, course_map=None, course_to_odom=None, px4_frame="auto",
                              arrival_radius_m=0.12, ulog=None, truth=None, out=tmp_path)

    lines, legs = analyze.report(bag, args)

    # Odom +y (1 m) is PX4 north under the `odom` mapping the synthetic vision agrees with
    assert [leg.target_ned for leg in legs] == [(1.0, 0.0, -0.8)]
    assert any("/skybeta/setpoint/pose, put in PX4's frame as `odom`" in line for line in lines)


def test_ulog_parameters_are_written_in_qgcs_params_format(tmp_path: Path) -> None:
    analyze.write_qgc_params(tmp_path / "px4.params", {"EKF2_EV_CTRL": 9, "EKF2_EVA_NOISE": 0.05})

    rows = [line.split("\t") for line in (tmp_path / "px4.params").read_text(encoding="utf-8").splitlines()
            if not line.startswith("#")]

    # QGC's MAV_PARAM_TYPE: 6 for INT32, 9 for REAL32
    assert rows == [["1", "1", "EKF2_EVA_NOISE", "0.05", "9"], ["1", "1", "EKF2_EV_CTRL", "9", "6"]]
