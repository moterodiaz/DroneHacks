"""PX4's height paired with the odometry at its capture, and the local position topic it comes on."""
from __future__ import annotations

import sys
import types
from collections.abc import Callable

import pytest

from dronehacks_course_follower.height_anchor import STALE_S, Alignment, HeightAnchor, local_position_topic

# The node's clock, epoch seconds as ROS time reads, and PX4's, seconds since its boot
_NODE_START_S = 1_794_000_000.0
_PX4_START_S = 52.0

# PX4's local position reaches the node at 100 Hz, the tracker's odometry at 30 Hz
_PX4_PERIOD_S = 0.01
_ODOM_PERIOD_S = 1.0 / 30.0

# The tracker's latency from capture to arrival, the kit's order of magnitude
_LATENCY_S = 0.06

# The odom origin sits this far below PX4's: the offset the anchor must read throughout
_BIAS_M = 0.2

# A climb at the return's pace, so PX4 runs 3 cm ahead of an odometry captured one latency ago
_CLIMB_MPS = 0.5


def _climb(time_s: float) -> float:
    """PX4's height in a steady climb, true."""
    return 0.8 + _CLIMB_MPS * time_s


def _odometry_climb(time_s: float) -> float:
    """The tracker's height in that climb, below PX4's by the bias."""
    return _climb(time_s) - _BIAS_M


def _on_node_clock(time_s: float) -> float:
    """A capture stamp on the node's clock, as the camera driver stamps on the drone."""
    return _NODE_START_S + time_s


def _fly(anchor: HeightAnchor, px4_up: Callable[[float], float], odom_up: Callable[[float], float],
         odom_stamp: Callable[[float], float], duration_s: float = 2.0) -> list[float]:
    """Feeds `anchor` PX4's heights `px4_up(t)` as they happen and the odometry captured at t with up
    `odom_up(t)`, arriving one latency later stamped `odom_stamp(t)`, in arrival order; the offsets paired."""
    events = [(index * _PX4_PERIOD_S, "px4", index * _PX4_PERIOD_S)
              for index in range(round(duration_s / _PX4_PERIOD_S))]
    events += [(index * _ODOM_PERIOD_S + _LATENCY_S, "odom", index * _ODOM_PERIOD_S)
               for index in range(round(duration_s / _ODOM_PERIOD_S))]
    offsets = []
    for arrival_s, source, time_s in sorted(events):
        if source == "px4":
            anchor.add_px4(_NODE_START_S + arrival_s, _PX4_START_S + time_s, px4_up(time_s))
            continue

        paired = anchor.pair(_NODE_START_S + arrival_s, odom_stamp(time_s), odom_up(time_s))
        if paired is not None:
            offsets.append(paired)

    return offsets


def test_a_climb_with_odometry_lag_does_not_register_as_offset() -> None:
    anchor = HeightAnchor()
    offsets = _fly(anchor, _climb, _odometry_climb, _on_node_clock)
    assert len(offsets) > 50
    assert offsets == pytest.approx([_BIAS_M] * len(offsets), abs=1e-6)
    assert anchor.alignment is Alignment.NODE
    assert anchor.lag_s == pytest.approx(_LATENCY_S)


def test_odometry_stamped_on_px4s_clock_is_paired_by_px4s_sample_stamps() -> None:
    # In the simulator the odometry carries sim time, which is PX4's clock
    anchor = HeightAnchor()
    offsets = _fly(anchor, _climb, _odometry_climb, lambda time_s: _PX4_START_S + time_s)
    assert offsets == pytest.approx([_BIAS_M] * len(offsets), abs=1e-6)
    assert anchor.alignment is Alignment.PX4


def test_a_stamp_on_neither_clock_is_paired_on_arrival_and_reads_the_climb() -> None:
    # Unaligned, PX4's height has climbed on by the latency's worth before the odometry arrives; the
    # middle of the climb, since past PX4's last sample the newest is held
    anchor = HeightAnchor()
    offsets = _fly(anchor, _climb, _odometry_climb, lambda time_s: time_s)
    assert anchor.alignment is Alignment.ARRIVAL
    assert offsets[len(offsets) // 2] == pytest.approx(_BIAS_M + _CLIMB_MPS * _LATENCY_S, abs=_CLIMB_MPS * _PX4_PERIOD_S)


def _px4_over_boulder(time_s: float) -> float:
    """PX4's height as EKF2 takes a 0.3 m boulder entering the rangefinder's cone at 1 s for a drop."""
    return 0.8 - (0.3 if time_s >= 1.0 else 0.0)


def test_a_step_in_px4s_height_becomes_the_offset_once_the_camera_has_seen_past_it() -> None:
    # The drone itself holds 0.8 m, so the tracker's height does not move
    anchor = HeightAnchor()
    offsets = _fly(anchor, _px4_over_boulder, lambda _: 0.8 - _BIAS_M, _on_node_clock)
    assert offsets[0] == pytest.approx(_BIAS_M)
    assert offsets[-1] == pytest.approx(_BIAS_M - 0.3)


def test_the_anchor_is_off_until_px4_and_the_odometry_are_paired_and_once_px4_falls_silent() -> None:
    anchor = HeightAnchor()
    assert anchor.offset(_NODE_START_S) is None

    _fly(anchor, _climb, _odometry_climb, _on_node_clock, duration_s=1.0)
    assert anchor.offset(_NODE_START_S + 1.0) == pytest.approx(_BIAS_M)
    assert anchor.offset(_NODE_START_S + 1.0 + STALE_S + 0.1) is None


def test_without_px4_msgs_there_is_no_local_position_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    # A None entry in sys.modules makes the import fail as a missing package does
    monkeypatch.setitem(sys.modules, "px4_msgs", None)
    monkeypatch.setitem(sys.modules, "px4_msgs.msg", None)
    assert local_position_topic() is None


def _fake_px4_msgs(monkeypatch: pytest.MonkeyPatch, version: int | None) -> None:
    """Installs a px4_msgs whose VehicleLocalPosition declares `version`, or no version at all."""
    message_type = type("VehicleLocalPosition", (), {} if version is None else {"MESSAGE_VERSION": version})
    package, messages = types.ModuleType("px4_msgs"), types.ModuleType("px4_msgs.msg")
    messages.VehicleLocalPosition = message_type
    package.msg = messages
    monkeypatch.setitem(sys.modules, "px4_msgs", package)
    monkeypatch.setitem(sys.modules, "px4_msgs.msg", messages)


def test_the_topic_takes_the_version_suffix_px4_msgs_declares(monkeypatch: pytest.MonkeyPatch) -> None:
    # PX4 v1.16 leaves VehicleLocalPosition at version 0, a bare name; a later version adds `_v<N>`
    _fake_px4_msgs(monkeypatch, None)
    assert local_position_topic()[1] == "/fmu/out/vehicle_local_position"

    _fake_px4_msgs(monkeypatch, 1)
    assert local_position_topic()[1] == "/fmu/out/vehicle_local_position_v1"
