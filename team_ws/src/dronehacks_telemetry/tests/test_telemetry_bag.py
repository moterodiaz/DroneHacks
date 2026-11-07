"""Reading a real mcap file written with the ROS 2 encoding, and the analyzer's command line on it.

The schemas carry only the fields the report reads: the mcap decoder reads any message definition,
so a cut-down one exercises the same path as a bag recorded on the drone.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dronehacks_telemetry import analyze, bag  # noqa: E402

mcap_ros2_writer = pytest.importorskip("mcap_ros2.writer")

# Cut-down message definitions, with each field named and typed as px4_msgs and ROS name it
_STATUS = "uint64 timestamp\nuint8 arming_state\nuint8 nav_state\nbool failsafe\n"
_LOCAL_POSITION = ("uint64 timestamp\nfloat32 x\nfloat32 y\nfloat32 z\nfloat32 heading\n"
                   "float32 dist_bottom\nbool dist_bottom_valid\nuint8 xy_reset_counter\n")
_SETPOINT = "uint64 timestamp\nfloat32[3] position\nfloat32 yaw\n"
_STATE = "string data\n"

# The bag's receive clock starts here, ns since the epoch
_START_NS = 1_762_500_000 * 10**9


def _write_bag(folder: Path) -> None:
    """A 6 s flight: armed at 1 s, Offboard 2-5 s holding a setpoint 0.5 m north, reached at 3 s."""
    folder.mkdir(parents=True)
    with (folder / "flight_0.mcap").open("wb") as stream:
        writer = mcap_ros2_writer.Writer(stream)
        status = writer.register_msgdef("px4_msgs/msg/VehicleStatus", _STATUS)
        local = writer.register_msgdef("px4_msgs/msg/VehicleLocalPosition", _LOCAL_POSITION)
        setpoint = writer.register_msgdef("px4_msgs/msg/TrajectorySetpoint", _SETPOINT)
        state = writer.register_msgdef("std_msgs/msg/String", _STATE)
        for step in range(60):
            time_s = step / 10
            log_ns = _START_NS + step * 100_000_000
            writer.write_message("/fmu/out/vehicle_status_v1", status, {
                "timestamp": 0, "arming_state": 2 if 1.0 <= time_s < 5.5 else 1,
                "nav_state": 14 if 2.0 <= time_s < 5.0 else 2, "failsafe": False}, log_time=log_ns)
            writer.write_message("/fmu/out/vehicle_local_position", local, {
                "timestamp": 0, "x": min(0.5, max(0.0, (time_s - 2.0) / 2.0)), "y": 0.0, "z": -0.8,
                "heading": 0.0, "dist_bottom": 0.85, "dist_bottom_valid": True, "xy_reset_counter": 0},
                log_time=log_ns)
            writer.write_message("/fmu/in/trajectory_setpoint", setpoint, {
                "timestamp": 0, "position": [0.5, 0.0, -0.8], "yaw": 0.0}, log_time=log_ns)
            writer.write_message("/skybeta/state", state, {"data": '{"px4_link": true, "clamped": []}'},
                                 log_time=log_ns)

        writer.finish()


def test_a_recorded_mcap_reads_into_role_series_on_the_receive_clock(tmp_path: Path) -> None:
    _write_bag(tmp_path / "bag")

    flight = bag.read_bag(tmp_path)

    # The versioned status topic resolves to its role; every series is timed from the first message
    assert flight.resolved["status"] == "/fmu/out/vehicle_status_v1"
    assert flight.start_ns == _START_NS
    assert flight.duration_s == pytest.approx(5.9)
    assert flight.topics["/fmu/out/vehicle_local_position"] == 60
    assert flight.series["trajectory_setpoint"].fields["x"][0] == pytest.approx(0.5)
    assert flight.series["kit_state"].fields["px4_link"][0] == 1.0


def test_the_analyzer_writes_the_report_and_legs_csv_into_the_flight_folder(tmp_path: Path) -> None:
    _write_bag(tmp_path / "bag")

    status = analyze.main([str(tmp_path)])

    report = (tmp_path / "report.md").read_text(encoding="utf-8")
    legs = (tmp_path / "legs.csv").read_text(encoding="utf-8").splitlines()
    assert status == 0
    assert "| 1 | 2.0 | +0.50,+0.00,-0.80 |" in report
    assert "reach" in legs[0]
    assert len(legs) == 2


def test_a_folder_without_a_bag_is_refused_with_a_message(tmp_path: Path,
                                                          capsys: pytest.CaptureFixture[str]) -> None:
    assert analyze.main([str(tmp_path)]) == 1
    assert "no .mcap file" in capsys.readouterr().err
