"""Triages a flight's PX4 log (.ulg) between practice slots: vision fusion, the vision delay,
vibration, modes and failsafes, the rangefinder and the vision stream's timing, as a short report.

Times are seconds from the log's start. PX4 stamps each message with `timestamp`, when it was
published, and many with `timestamp_sample`, when what it holds was measured; both microseconds on
PX4's clock. Positions are metres in the EKF's local NED frame, yaw radians from north. A lag is
how far vision runs behind the EKF: positive when vision shows a motion after the EKF does.

The vibration bands are Flight Review's Vibration Metrics plot (github.com/PX4/flight_review,
`app/plot_app/configured_plots.py`). A test ratio past 1 is a rejected sample, per PX4's EKF
tuning guide. The delay method is PX4's external position estimation guide's: the offset between
the EKF's rates and vision's, then the delay whose flights show the lowest vision innovations.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntEnum
from operator import attrgetter
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

import numpy as np

if TYPE_CHECKING:
    from pyulog import ULog

# Flags logged only on change are held on this grid; its step is a dropout's resolution
_FLAG_GRID_HZ = 100.0

# Rates are compared on this grid; its step is the lag's resolution
_LAG_GRID_HZ = 200.0

# The largest lag searched either way: past EKF2_DELAY_MAX's default, the most EKF2 can buffer
_MAX_LAG_S = 0.3

# The EKF's yaw rate that marks a turn: above a hover's yaw wander, below any commanded turn
TURN_RATE_RAD_S = 0.3

# The EKF's horizontal speed that marks a move between hovers
MOVE_SPEED_M_S = 0.2

# A turn or move shorter than this is noise, not a manoeuvre to time
_MIN_ACTIVE_S = 0.3

# Each turn or move is widened by this much either side, so its start and stop fall inside it
_SEGMENT_PAD_S = 1.0

# Both rates are smoothed alike over this window, so differentiated noise does not pick the lag
_RATE_SMOOTH_S = 0.1

# A step between two samples longer than this is a gap in the log, not motion
_MAX_STEP_S = 0.5

# A segment whose best-shifted rates correlate less than this has no lag worth reading
_MIN_CORRELATION = 0.8

# Flight Review shades accel_vibration_metric green below half a g, orange to a g, red above
VIBRATION_HIGH_M_S2 = 4.905
VIBRATION_TOO_HIGH_M_S2 = 9.81

# EKF2 rejects a sample whose innovation test ratio passes 1
_TEST_RATIO_GATE = 1.0

# A step this large between consecutive valid ranges is a jump: a wall top or an edge in the cone
RANGE_JUMP_M = 0.3

# distance_sensor's orientation for a sensor looking down, PX4's ROTATION_DOWNWARD_FACING
_DOWNWARD = 25

# The most jumps, or timeline entries, a section lists before it only counts the rest
_MAX_LISTED = 8
_MAX_EVENTS = 40

# The console levels the timeline shows: warnings and worse
_ALARM_LEVELS = {"EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING"}

# The estimator settings the header shows: vision delay and fusion, height source, range use
_SETTINGS = ("EKF2_EV_DELAY", "EKF2_EV_CTRL", "EKF2_HGT_REF", "EKF2_RNG_CTRL")

# The EKF2 flags the fusion section follows: vision position, vision yaw, range height
_FUSION_FLAGS = ("cs_ev_pos", "cs_ev_yaw", "cs_rng_hgt")

# estimator_innovations' vision fields and the heading's, their units, and the EKF2 flag that
# says each is fused: EKF2 computes innovations it does not fuse, as height EKF2_EV_CTRL leaves out
_INNOVATIONS = (("EV pos north", "ev_hpos[0]", "m", "cs_ev_pos"),
                ("EV pos east", "ev_hpos[1]", "m", "cs_ev_pos"),
                ("EV pos down", "ev_vpos", "m", "cs_ev_hgt"),
                ("EV vel north", "ev_hvel[0]", "m/s", "cs_ev_vel"),
                ("EV vel east", "ev_hvel[1]", "m/s", "cs_ev_vel"),
                ("EV vel down", "ev_vvel", "m/s", "cs_ev_vel"),
                ("heading", "heading", "rad", "cs_ev_yaw"))

# EKF2's per-sample record of each vision source: whether it fused, whether its gate rejected it
_AID_SOURCES = (("estimator_aid_src_ev_pos", "EV pos"), ("estimator_aid_src_ev_yaw", "EV yaw"))


class NavState(IntEnum):
    """vehicle_status.nav_state, as PX4 v1.16's VehicleStatus.msg numbers it."""

    MANUAL = 0
    ALTCTL = 1
    POSCTL = 2
    AUTO_MISSION = 3
    AUTO_LOITER = 4
    AUTO_RTL = 5
    POSITION_SLOW = 6
    ACRO = 10
    DESCEND = 12
    TERMINATION = 13
    OFFBOARD = 14
    STAB = 15
    AUTO_TAKEOFF = 17
    AUTO_LAND = 18
    AUTO_FOLLOW_TARGET = 19
    AUTO_PRECLAND = 20
    ORBIT = 21
    AUTO_VTOL_TAKEOFF = 22
    EXTERNAL1 = 23
    EXTERNAL2 = 24
    EXTERNAL3 = 25
    EXTERNAL4 = 26
    EXTERNAL5 = 27
    EXTERNAL6 = 28
    EXTERNAL7 = 29
    EXTERNAL8 = 30


class ArmingState(IntEnum):
    """vehicle_status.arming_state, as PX4 v1.16 numbers it."""

    DISARMED = 1
    ARMED = 2


@dataclass(frozen=True, slots=True)
class Topic:
    """One instance of a logged topic."""

    times_s: np.ndarray               # each message's `timestamp`, from the log's start
    fields: dict[str, np.ndarray]     # every field as floats, named as pyulog names them: `q[0]`


@dataclass(frozen=True, slots=True, kw_only=True)
class Flight:
    """What the report reads from one log."""

    name: str                                  # the log's file name
    version: str                               # PX4's release, as the log names it, or '?'
    duration_s: float                          # from the log's start to its last message
    parameters: dict[str, float]               # PX4's parameters as the log started
    topics: dict[str, list[Topic]]             # each topic's instances, in instance order
    messages: list[tuple[float, str, str]]     # PX4's console: time, level and text


class Span(NamedTuple):
    """A stretch of the log."""

    start_s: float
    end_s: float


class Dropout(NamedTuple):
    """A stretch a flag was false while flying."""

    start_s: float
    duration_s: float


class Rate(NamedTuple):
    """A signal's rate of change between consecutive samples."""

    times_s: np.ndarray    # midway between the two samples each rate is taken from
    values: np.ndarray     # the rate, in the signal's unit per second


class RangeJump(NamedTuple):
    """A step between consecutive valid rangefinder readings."""

    time_s: float
    from_m: float
    to_m: float


@dataclass(frozen=True, slots=True)
class FlagTime:
    """How an EKF2 status flag held while flying."""

    true_fraction: float           # of the flying time
    dropouts: list[Dropout]        # each stretch it was false while flying


@dataclass(frozen=True, slots=True)
class InnovationStats:
    """One innovation's size while flying."""

    rms: float                     # in the innovation's unit
    max_abs: float                 # in the innovation's unit
    max_test_ratio: float          # innovation over its gate; past 1 is rejected
    over_gate_fraction: float      # of the samples, with a test ratio past 1


@dataclass(frozen=True, slots=True)
class LagEstimate:
    """How far vision runs behind the EKF, over all turns or moves and in each one."""

    overall_s: float | None        # over every kept segment at once, None with none to time
    per_segment_s: list[float]     # each kept segment's own, in time order
    left_out: int                  # segments with a vision gap or rates that never agreed


@dataclass(frozen=True, slots=True)
class VisionTiming:
    """The vision stream's rate, and its age as the sender stamped it."""

    rate_hz: float                 # from the median interval between capture stamps
    interval_p95_s: float          # between capture stamps
    longest_gap_s: float           # between capture stamps
    age_p50_s: float               # the sender's publication stamp minus its capture stamp
    age_p95_s: float               # the sender's publication stamp minus its capture stamp


def read_flight(path: Path) -> Flight:
    """Reads a PX4 log with pyulog."""
    try:
        from pyulog import ULog
    except ImportError as missing:
        raise ImportError("reading a PX4 log needs pyulog: pip install pyulog") from missing

    return flight_from_ulog(ULog(str(path)), path.name)


def flight_from_ulog(ulog: ULog, name: str) -> Flight:
    """The topics, parameters and console of a parsed log, timed from its start."""
    start_us = ulog.start_timestamp
    topics: dict[str, list[Topic]] = {}

    # Every field as floats: microsecond stamps since boot stay exact in a double
    for data in sorted(ulog.data_list, key=attrgetter("name", "multi_id")):
        fields = {field: np.asarray(values, dtype=float) for field, values in data.data.items()}
        times_s = (fields["timestamp"] - start_us) * 1e-6
        topics.setdefault(data.name, []).append(Topic(times_s=times_s, fields=fields))

    messages = [((message.timestamp - start_us) * 1e-6, message.log_level_str(),
                 message.message.strip()) for message in ulog.logged_messages]
    return Flight(name=name, version=ulog.get_version_info_str() or "?",
                  duration_s=(ulog.last_timestamp - start_us) * 1e-6,
                  parameters=dict(ulog.initial_parameters), topics=topics, messages=messages)


def sample_times_s(topic: Topic) -> np.ndarray:
    """When each message's content was measured, from its `timestamp_sample`, on the log's clock."""
    return topic.times_s - (topic.fields["timestamp"] - topic.fields["timestamp_sample"]) * 1e-6


def flying_spans(flight: Flight) -> list[Span]:
    """The stretches PX4's land detector did not report landed, else those EKF2 called in air."""
    land = _first(flight, "vehicle_land_detected")
    flags = _first(flight, "estimator_status_flags")

    if land is not None:
        change_times_s, airborne = land.times_s, land.fields["landed"] < 0.5
    elif flags is not None:
        change_times_s, airborne = flags.times_s, flags.fields["cs_in_air"] > 0.5
    else:
        return []

    grid = np.arange(0.0, flight.duration_s, 1 / _FLAG_GRID_HZ)
    return _true_spans(grid, _held(change_times_s, airborne, grid), 1 / _FLAG_GRID_HZ)


def flag_time(change_times_s: np.ndarray, values: np.ndarray,
              flying: Sequence[tuple[float, float]]) -> FlagTime:
    """How long a flag logged on change was true while flying, and each stretch it was false."""
    total, true = 0, 0
    dropouts: list[Dropout] = []

    # Hold the flag on a grid over each flying stretch, so time counts rather than messages
    for start_s, end_s in flying:
        grid = np.arange(start_s, end_s, 1 / _FLAG_GRID_HZ)
        on = _held(change_times_s, values, grid) > 0.5
        total, true = total + grid.size, true + int(on.sum())
        dropouts += [Dropout(span.start_s, span.end_s - span.start_s)
                     for span in _true_spans(grid, ~on, 1 / _FLAG_GRID_HZ)]

    return FlagTime(true_fraction=true / total if total else 0.0, dropouts=dropouts)


def innovation_stats(innovations: np.ndarray, test_ratios: np.ndarray) -> InnovationStats | None:
    """An innovation's size and its test ratios, or None for a source EKF2 never fused."""
    if innovations.size == 0 or not np.any(innovations):
        return None

    return InnovationStats(rms=float(np.sqrt(np.mean(innovations**2))),
                           max_abs=float(np.abs(innovations).max()),
                           max_test_ratio=float(test_ratios.max()),
                           over_gate_fraction=float(np.mean(test_ratios > _TEST_RATIO_GATE)))


def wrap(angles: np.ndarray) -> np.ndarray:
    """Angles in radians wrapped into [-pi, pi)."""
    return (angles + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quaternions(q: np.ndarray) -> np.ndarray:
    """Yaw in radians from (n, 4) PX4 quaternions, w first, rotating body FRD into NED."""
    w, x, y, z = q.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_rate(times_s: np.ndarray, yaw: np.ndarray, resets: np.ndarray) -> Rate:
    """Yaw rate between consecutive samples, leaving out steps across a reset or a gap."""
    keep = _continuous(times_s, resets)
    rates = wrap(np.diff(yaw)[keep]) / np.diff(times_s)[keep]
    return Rate(_midpoints(times_s)[keep], rates)


def horizontal_speed(times_s: np.ndarray, xy: np.ndarray, resets: np.ndarray) -> Rate:
    """Horizontal speed from (n, 2) positions, leaving out steps across a reset or a gap."""
    keep = _continuous(times_s, resets)
    steps = np.diff(xy, axis=0)[keep]
    speeds = np.hypot(steps[:, 0], steps[:, 1]) / np.diff(times_s)[keep]
    return Rate(_midpoints(times_s)[keep], speeds)


def estimate_lag(reference: Rate, vision: Rate, flying: Sequence[tuple[float, float]],
                 threshold: float) -> LagEstimate:
    """How far `vision`'s rate runs behind `reference`'s, where the reference's passes `threshold`.

    Both rates are put on one grid over the flight and smoothed alike. Each run of the reference
    past `threshold` while flying, widened to take in its start and stop, is a segment; the lag is
    the shift of vision's rate that correlates best with the reference's, over every segment at
    once and over each alone. A segment across a vision gap, or whose rates never correlate well,
    is left out. This is the offset PX4's guide reads between the EKF's rates and vision's: what
    EKF2_EV_DELAY should cover, given vision's `timestamp_sample`.
    """
    if not flying or reference.times_s.size < 2 or vision.times_s.size < 2:
        return LagEstimate(overall_s=None, per_segment_s=[], left_out=0)

    grid = np.arange(flying[0][0], flying[-1][1], 1 / _LAG_GRID_HZ)
    reference_rate = _smoothed(np.interp(grid, reference.times_s, reference.values))
    vision_rate = _smoothed(np.interp(grid, vision.times_s, vision.values))

    # The segments: runs past the threshold while flying, kept a full search's shift from the ends
    max_shift = round(_MAX_LAG_S * _LAG_GRID_HZ)
    active = (np.abs(reference_rate) > threshold) & _in_spans(grid, flying)
    windows = _windows(active, max_shift)

    # Each segment's own lag, kept where vision has no gap and the shifted rates agree
    covered = _covered(grid, vision.times_s)
    fits = [_best_shift(reference_rate, vision_rate, [window], max_shift)
            if covered[window].all() else None for window in windows]
    kept = [window for window, fit in zip(windows, fits, strict=True) if fit is not None]
    overall = _best_shift(reference_rate, vision_rate, _merged(kept), max_shift) if kept else None

    return LagEstimate(overall_s=None if overall is None else overall / _LAG_GRID_HZ,
                       per_segment_s=[fit / _LAG_GRID_HZ for fit in fits if fit is not None],
                       left_out=len(windows) - len(kept))


def best_delay_ms(rows: Sequence[tuple[float, float]]) -> float:
    """The EKF2_EV_DELAY whose logs' vision innovation rms is lowest on average, from rows of
    (EKF2_EV_DELAY in ms, that log's rms)."""
    by_delay: dict[float, list[float]] = {}
    for delay_ms, rms in rows:
        by_delay.setdefault(delay_ms, []).append(rms)

    return min(by_delay, key=lambda delay_ms: float(np.mean(by_delay[delay_ms])))


def vibration_grade(accel_metric_m_s2: float) -> str:
    """Flight Review's band for an accel vibration metric."""
    if accel_metric_m_s2 < VIBRATION_HIGH_M_S2:
        return "good"

    if accel_metric_m_s2 < VIBRATION_TOO_HIGH_M_S2:
        return "high"

    return "too high"


def rises(times_s: np.ndarray, values: np.ndarray) -> list[tuple[float, float | None]]:
    """When a flag became true and when it next fell, None if it never did; true at the first
    sample counts as rising there."""
    edges = np.flatnonzero(np.diff(np.concatenate([[0], (values > 0.5).astype(int), [0]])))
    return [(float(times_s[start]), float(times_s[end]) if end < times_s.size else None)
            for start, end in zip(edges[::2], edges[1::2], strict=True)]


def state_changes(times_s: np.ndarray, values: np.ndarray) -> list[tuple[float, int]]:
    """Each time a state took a new value, its first sample included."""
    changed = np.concatenate([[values.size > 0], np.diff(values) != 0])[:values.size]
    return [(float(time_s), int(value))
            for time_s, value in zip(times_s[changed], values[changed], strict=True)]


def range_valid(distance_m: np.ndarray, quality: np.ndarray, min_m: np.ndarray | float,
                max_m: np.ndarray | float) -> np.ndarray:
    """Which rangefinder readings count: in the sensor's range, and not flagged invalid by a
    signal quality of 0 (-1 is unknown, so counts)."""
    return (quality != 0) & (distance_m >= min_m) & (distance_m <= max_m)


def range_jumps(times_s: np.ndarray, distance_m: np.ndarray, valid: np.ndarray,
                threshold_m: float) -> list[RangeJump]:
    """Each step past `threshold_m` between consecutive valid readings, timed at the second."""
    indices = np.flatnonzero(valid)
    steps_m = np.diff(distance_m[indices])
    return [RangeJump(float(times_s[indices[step + 1]]), float(distance_m[indices[step]]),
                      float(distance_m[indices[step + 1]]))
            for step in np.flatnonzero(np.abs(steps_m) > threshold_m)]


def vision_timing(times_s: np.ndarray, sample_times_s: np.ndarray) -> VisionTiming:
    """The vision stream's rate from its capture stamps, and its age when sent. Both stamps are
    the sender's, so transport after sending is not in the age."""
    intervals_s = np.diff(sample_times_s)
    ages_s = times_s - sample_times_s
    return VisionTiming(rate_hz=float(1 / np.median(intervals_s)),
                        interval_p95_s=float(np.percentile(intervals_s, 95)),
                        longest_gap_s=float(intervals_s.max()),
                        age_p50_s=float(np.percentile(ages_s, 50)),
                        age_p95_s=float(np.percentile(ages_s, 95)))


def ev_position_rms_m(flight: Flight, flying: Sequence[tuple[float, float]]) -> float | None:
    """The horizontal vision position innovation's rms while flying, None if never fused."""
    innovations = _first(flight, "estimator_innovations")
    if innovations is None:
        return None

    flown = _in_spans(innovations.times_s, flying)
    north, east = innovations.fields["ev_hpos[0]"][flown], innovations.fields["ev_hpos[1]"][flown]
    if north.size == 0 or not (np.any(north) or np.any(east)):
        return None

    return float(np.sqrt(np.mean(north**2 + east**2)))


def report(flight: Flight) -> list[str]:
    """The report's lines for one log; a log that never flew is read whole."""
    flying = flying_spans(flight)
    window = flying or [Span(0.0, flight.duration_s)]
    return [*_header(flight, flying), *_fusion_section(flight, window),
            *_delay_section(flight, window), *_vibration_section(flight, window),
            *_modes_section(flight), *_range_section(flight, window),
            *_timing_section(flight, window)]


def _first(flight: Flight, name: str) -> Topic | None:
    """A topic's first instance, None if the log lacks it."""
    instances = flight.topics.get(name)
    return instances[0] if instances else None


def _held(change_times_s: np.ndarray, values: np.ndarray, at_s: np.ndarray) -> np.ndarray:
    """A value logged on change, held from each change to the next and read at `at_s`; before the
    first change it reads the first value."""
    indices = np.searchsorted(change_times_s, at_s, side="right") - 1
    return values[np.clip(indices, 0, values.size - 1)]


def _true_spans(times_s: np.ndarray, inside: np.ndarray, step_s: float) -> list[Span]:
    """Each run of true samples on a grid of `step_s`, from its first sample to the step past
    its last."""
    if not inside.any():
        return []

    bounds_s = np.append(times_s, times_s[-1] + step_s)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], inside.astype(int), [0]])))
    return [Span(float(bounds_s[start]), float(bounds_s[end]))
            for start, end in zip(edges[::2], edges[1::2], strict=True)]


def _in_spans(times_s: np.ndarray, spans: Sequence[tuple[float, float]]) -> np.ndarray:
    """Which times fall inside any of `spans`."""
    inside = np.zeros(times_s.shape, dtype=bool)
    for start_s, end_s in spans:
        inside |= (times_s >= start_s) & (times_s < end_s)

    return inside


def _continuous(times_s: np.ndarray, resets: np.ndarray) -> np.ndarray:
    """Which steps between consecutive samples stay within one reset count and span no gap."""
    steps_s = np.diff(times_s)
    return (np.diff(resets) == 0) & (steps_s > 0) & (steps_s < _MAX_STEP_S)


def _midpoints(times_s: np.ndarray) -> np.ndarray:
    """The times midway between consecutive samples."""
    return (times_s[1:] + times_s[:-1]) / 2


def _smoothed(values: np.ndarray) -> np.ndarray:
    """A moving average over `_RATE_SMOOTH_S` on the lag grid."""
    width = max(1, round(_RATE_SMOOTH_S * _LAG_GRID_HZ))
    return np.convolve(values, np.ones(width) / width, mode="same")


def _windows(active: np.ndarray, margin: int) -> list[slice]:
    """Index ranges around each long enough run of `active`, widened by the segment pad and kept
    `margin` from either end; neighbours may overlap, so each turn keeps its own window."""
    pad, shortest = round(_SEGMENT_PAD_S * _LAG_GRID_HZ), round(_MIN_ACTIVE_S * _LAG_GRID_HZ)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], active.astype(int), [0]])))
    windows: list[slice] = []

    for start, end in zip(edges[::2], edges[1::2], strict=True):
        widened = slice(max(int(start) - pad, margin), min(int(end) + pad, active.size - margin))
        if end - start >= shortest and widened.stop > widened.start:
            windows.append(widened)

    return windows


def _merged(windows: list[slice]) -> list[slice]:
    """Overlapping windows joined, so no sample counts twice."""
    merged: list[slice] = []
    for window in windows:
        if merged and window.start <= merged[-1].stop:
            merged[-1] = slice(merged[-1].start, max(merged[-1].stop, window.stop))
        else:
            merged.append(window)

    return merged


def _covered(grid_s: np.ndarray, sample_times_s: np.ndarray) -> np.ndarray:
    """Which grid times fall between two samples less than a gap apart, so are not interpolated
    across a hole in the stream."""
    after = np.clip(np.searchsorted(sample_times_s, grid_s), 1, sample_times_s.size - 1)
    spacing_s = sample_times_s[after] - sample_times_s[after - 1]
    inside = (grid_s >= sample_times_s[0]) & (grid_s <= sample_times_s[-1])
    return inside & (spacing_s < _MAX_STEP_S)


def _best_shift(reference: np.ndarray, vision: np.ndarray, windows: list[slice],
                max_shift: int) -> int | None:
    """The shift, in grid steps, of `vision` that correlates best with `reference` over `windows`;
    None if the best correlation is weak or either signal is flat there."""
    reference_part = np.concatenate([reference[window] for window in windows])
    shifts = range(-max_shift, max_shift + 1)
    scores = [_correlation(reference_part, _shifted(vision, windows, shift)) for shift in shifts]

    if np.all(np.isnan(scores)) or np.nanmax(scores) < _MIN_CORRELATION:
        return None

    return shifts[int(np.nanargmax(scores))]


def _shifted(signal: np.ndarray, windows: list[slice], shift: int) -> np.ndarray:
    """`signal` over `windows`, each moved `shift` grid steps later, end to end."""
    return np.concatenate([signal[window.start + shift:window.stop + shift] for window in windows])


def _correlation(first: np.ndarray, second: np.ndarray) -> float:
    """Pearson correlation of two equal-length signals, NaN if either is flat."""
    first, second = first - first.mean(), second - second.mean()
    scale = np.sqrt(np.sum(first**2) * np.sum(second**2))
    return float(np.sum(first * second) / scale) if scale > 0 else float("nan")


def _parameter(flight: Flight, name: str) -> str:
    """A parameter's value as the log started, or '?' if it was not logged."""
    value = flight.parameters.get(name)
    return "?" if value is None else f"{value:g}"


def _header(flight: Flight, flying: list[Span]) -> list[str]:
    """The log's name, length, flying stretches and the estimator settings the report bears on."""
    flown = (", ".join(f"{span.start_s:.1f}-{span.end_s:.1f} s" for span in flying)
             or "never: the whole log is read")
    settings = ", ".join(f"{name} {_parameter(flight, name)}" for name in _SETTINGS)
    return [f"{flight.name}: PX4 {flight.version}, {flight.duration_s:.1f} s logged; "
            f"flying {flown}", f"  {settings}"]


def _fusion_section(flight: Flight, flying: list[Span]) -> list[str]:
    """Vision and range fusion while flying: EKF2's flags, the innovations and the rejections."""
    lines = ["", "Vision fusion, while flying"]
    flags = _first(flight, "estimator_status_flags")

    # The flags are logged on change, so each is held between its messages
    if flags is None:
        lines.append("  estimator_status_flags not logged")
    else:
        for name in _FUSION_FLAGS:
            held = flag_time(flags.times_s, flags.fields[name], flying)
            dropouts = ", ".join(f"{dropout.start_s:.1f} s for {dropout.duration_s:.1f} s"
                                 for dropout in held.dropouts[:_MAX_LISTED])
            lines.append(f"  {name:<11}{held.true_fraction:6.1%} true; "
                         f"{len(held.dropouts)} dropouts{': ' + dropouts if dropouts else ''}")

    return lines + _innovation_lines(flight, flying) + _aid_source_lines(flight, flying)


def _innovation_lines(flight: Flight, flying: list[Span]) -> list[str]:
    """Each vision innovation's rms, maximum and test ratios while flying, from EKF2's first
    instance."""
    innovations = _first(flight, "estimator_innovations")
    ratios = _first(flight, "estimator_innovation_test_ratios")
    if innovations is None or ratios is None:
        return ["  estimator_innovations not logged"]

    lines = [f"  {'innovation':<14}{'rms':>7}{'':5}{'max':>6}{'':5}{'max ratio':>9}"
             f"{'past gate':>11}"]
    flown = _in_spans(innovations.times_s, flying)
    ratios_flown = _in_spans(ratios.times_s, flying)

    for label, field, unit, flag in _INNOVATIONS:
        if field not in innovations.fields:
            continue

        stats = innovation_stats(innovations.fields[field][flown],
                                 ratios.fields[field][ratios_flown])
        if stats is None:
            lines.append(f"  {label:<14}not computed")
            continue

        fused = "" if _ever_true(flight, flag, flying) else "  computed, never fused"
        lines.append(f"  {label:<14}{stats.rms:7.3f} {unit:<4}{stats.max_abs:6.3f} {unit:<4}"
                     f"{stats.max_test_ratio:9.2f}{stats.over_gate_fraction:11.1%}{fused}")

    return lines


def _ever_true(flight: Flight, flag: str, flying: list[Span]) -> bool:
    """Whether an EKF2 status flag was true at some time while flying; true if it is not logged."""
    flags = _first(flight, "estimator_status_flags")
    if flags is None or flag not in flags.fields:
        return True

    return flag_time(flags.times_s, flags.fields[flag], flying).true_fraction > 0


def _aid_source_lines(flight: Flight, flying: list[Span]) -> list[str]:
    """How many vision samples EKF2 fused and rejected while flying, from its aid-source topics."""
    lines = []
    for name, label in _AID_SOURCES:
        source = _first(flight, name)
        if source is None:
            continue

        flown = _in_spans(source.times_s, flying)
        if not flown.any():
            continue

        fused = source.fields["fused"][flown] > 0.5
        rejected = source.fields["innovation_rejected"][flown] > 0.5
        lines.append(f"  {label} samples {flown.sum()}: fused {fused.mean():.1%}, "
                     f"rejected {rejected.mean():.1%}")

    return lines


def _delay_section(flight: Flight, flying: list[Span]) -> list[str]:
    """Vision's lag behind the EKF through turns and moves, and the EKF2_EV_DELAY it suggests."""
    lines = ["", "Vision delay: vision's lag behind the EKF, from rates (positive: vision late)"]
    vision = _first(flight, "vehicle_visual_odometry")
    if vision is None:
        return [*lines, "  vehicle_visual_odometry not logged"]

    yaw_lag = _yaw_lag(flight, vision, flying)
    position_lag = _position_lag(flight, vision, flying)
    lines += [_lag_line("yaw rate", "turns", yaw_lag),
              _lag_line("horizontal speed", "moves", position_lag)]

    # Yaw first: the EKF's yaw rate is the gyro's, untouched by vision's own noise
    chosen = yaw_lag if yaw_lag is not None and yaw_lag.overall_s is not None else position_lag
    if chosen is None or chosen.overall_s is None:
        return [*lines, "  no turn or move to time: fly a few yaw turns and legs"]

    suggested_ms = max(0.0, chosen.overall_s * 1000)
    return [*lines, f"  suggested EKF2_EV_DELAY {suggested_ms:.0f} ms "
                    f"(now {_parameter(flight, 'EKF2_EV_DELAY')}), "
                    "if vision's timestamp_sample is its capture time"]


def _yaw_lag(flight: Flight, vision: Topic, flying: list[Span]) -> LagEstimate | None:
    """Vision yaw's lag behind the EKF's attitude, None without an attitude or a vision
    orientation."""
    attitude = _first(flight, "vehicle_attitude")
    if attitude is None:
        return None

    # Vision may send position alone, its quaternion NaN
    vision_q = np.column_stack([vision.fields[f"q[{index}]"] for index in range(4)])
    seen = np.all(np.isfinite(vision_q), axis=1)
    if seen.sum() < 2:
        return None

    attitude_q = np.column_stack([attitude.fields[f"q[{index}]"] for index in range(4)])
    reference = yaw_rate(sample_times_s(attitude), yaw_from_quaternions(attitude_q),
                         attitude.fields["quat_reset_counter"])
    measured = yaw_rate(sample_times_s(vision)[seen], yaw_from_quaternions(vision_q[seen]),
                        vision.fields["reset_counter"][seen])
    return estimate_lag(reference, measured, flying, TURN_RATE_RAD_S)


def _position_lag(flight: Flight, vision: Topic, flying: list[Span]) -> LagEstimate | None:
    """Vision position's lag behind the EKF's, by horizontal speed so the frames' yaw does not
    matter."""
    local = _first(flight, "vehicle_local_position")
    if local is None:
        return None

    vision_xy = np.column_stack([vision.fields["position[0]"], vision.fields["position[1]"]])
    seen = np.all(np.isfinite(vision_xy), axis=1)
    if seen.sum() < 2:
        return None

    local_xy = np.column_stack([local.fields["x"], local.fields["y"]])
    reference = horizontal_speed(sample_times_s(local), local_xy, local.fields["xy_reset_counter"])
    measured = horizontal_speed(sample_times_s(vision)[seen], vision_xy[seen],
                                vision.fields["reset_counter"][seen])
    return estimate_lag(reference, measured, flying, MOVE_SPEED_M_S)


def _lag_line(signal: str, segments: str, lag: LagEstimate | None) -> str:
    """One signal's lag, overall and per segment."""
    if lag is None:
        return f"  {signal}: not logged"

    left_out = f"; {lag.left_out} left out (vision gap or no agreement)" if lag.left_out else ""
    if lag.overall_s is None:
        return f"  {signal}: no {segments} to time{left_out}"

    each = ", ".join(f"{lag_s * 1000:.0f}" for lag_s in lag.per_segment_s)
    return (f"  {signal}: {lag.overall_s * 1000:.0f} ms over {len(lag.per_segment_s)} {segments} "
            f"({each} ms){left_out}")


def _vibration_section(flight: Flight, flying: list[Span]) -> list[str]:
    """Each IMU's vibration metrics and clipping while flying, against Flight Review's bands."""
    lines = ["", "Vibration, while flying (Flight Review: accel under 0.5 g good, 1 g too high)"]
    instances = flight.topics.get("vehicle_imu_status", [])
    if not instances:
        return [*lines, "  vehicle_imu_status not logged"]

    for index, status in enumerate(instances):
        flown = _in_spans(status.times_s, flying)
        if not flown.any():
            continue

        accel = status.fields["accel_vibration_metric"][flown]
        gyro = status.fields["gyro_vibration_metric"][flown]
        accel_p95 = float(np.percentile(accel, 95))
        lines.append(f"  imu {index}: accel {accel_p95:.2f} m/s^2 p95, {accel.max():.2f} max: "
                     f"{vibration_grade(accel_p95)}; gyro {np.percentile(gyro, 95):.3f} rad/s p95; "
                     f"clipping accel {_clipping(status, 'accel', flown)}, "
                     f"gyro {_clipping(status, 'gyro', flown)}")

    return lines


def _clipping(status: Topic, sensor: str, flown: np.ndarray) -> int:
    """The clipping events one sensor counted while flying, summed over its axes."""
    counts = [status.fields[f"{sensor}_clipping[{axis}]"][flown] for axis in range(3)]
    return int(sum(count.max() - count.min() for count in counts))


def _modes_section(flight: Flight) -> list[str]:
    """One timeline of arming, mode changes, failsafes and PX4's warnings."""
    events: list[tuple[float, str]] = []
    status = _first(flight, "vehicle_status")
    armed_s = 0.0

    # Arming and the navigation state, from each change
    if status is not None:
        arming = state_changes(status.times_s, status.fields["arming_state"])
        events += [(time_s, _arming_name(state)) for time_s, state in arming]
        events += [(time_s, f"mode {_nav_name(state)}") for time_s, state in
                   state_changes(status.times_s, status.fields["nav_state"])]
        events += _rise_events(status.times_s, status.fields["failsafe"], "failsafe")
        armed_s = next((time_s for time_s, state in arming if state == ArmingState.ARMED), 0.0)

    # Each failsafe condition from the first arming, past the start-up's; mode_req fields are
    # bitmasks of modes, not conditions
    failsafes = _first(flight, "failsafe_flags")
    if failsafes is not None:
        since = failsafes.times_s >= armed_s
        for name, values in failsafes.fields.items():
            if name != "timestamp" and not name.startswith("mode_req_"):
                events += _rise_events(failsafes.times_s[since], values[since], name)

    events += [(time_s, f"PX4 {level.lower()}: {text}") for time_s, level, text in flight.messages
               if level in _ALARM_LEVELS]
    lines = ["", "Modes and failsafes (failsafe flags from the first arming)"]
    if status is None:
        lines.append("  vehicle_status not logged")

    events.sort(key=lambda event: event[0])
    lines += [f"  {time_s:7.1f} s  {text}" for time_s, text in events[:_MAX_EVENTS]]
    if len(events) > _MAX_EVENTS:
        lines.append(f"  ... {len(events) - _MAX_EVENTS} more")

    return lines


def _arming_name(state: int) -> str:
    """An arming state's name, lower case."""
    known = state in {member.value for member in ArmingState}
    return ArmingState(state).name.lower() if known else f"arming_state {state}"


def _nav_name(state: int) -> str:
    """A navigation state's name."""
    known = state in {member.value for member in NavState}
    return NavState(state).name if known else f"nav_state {state}"


def _rise_events(times_s: np.ndarray, values: np.ndarray, name: str) -> list[tuple[float, str]]:
    """Timeline entries for each time a flag rose and fell."""
    events = []
    for rise_s, fall_s in rises(times_s, values):
        events.append((rise_s, f"{name} true"))
        if fall_s is not None:
            events.append((fall_s, f"{name} false"))

    return events


def _range_section(flight: Flight, flying: list[Span]) -> list[str]:
    """Each rangefinder's valid fraction and jumps while flying."""
    lines = ["", f"Rangefinder, while flying (a jump: over {RANGE_JUMP_M:g} m between valid ranges)"]
    instances = flight.topics.get("distance_sensor", [])
    if not instances:
        return [*lines, "  distance_sensor not logged"]

    for index, sensor in enumerate(instances):
        flown = _in_spans(sensor.times_s, flying)
        if not flown.any():
            continue

        fields = {name: values[flown] for name, values in sensor.fields.items()}
        valid = range_valid(fields["current_distance"], fields["signal_quality"],
                            fields["min_distance"], fields["max_distance"])
        jumps = range_jumps(sensor.times_s[flown], fields["current_distance"], valid, RANGE_JUMP_M)
        orientation = fields["orientation"][0]
        facing = "down" if orientation == _DOWNWARD else f"orientation {orientation:g}"
        listed = ", ".join(f"{jump.time_s:.1f} s {jump.from_m:.2f}->{jump.to_m:.2f} m"
                           for jump in jumps[:_MAX_LISTED])
        lines.append(f"  distance_sensor {index} ({facing}): valid {valid.mean():.1%}; "
                     f"{len(jumps)} jumps{': ' + listed if listed else ''}")

    return lines


def _timing_section(flight: Flight, flying: list[Span]) -> list[str]:
    """The vision stream's rate, gaps and age when sent, while flying."""
    lines = ["", "Vision timing, while flying"]
    vision = _first(flight, "vehicle_visual_odometry")
    if vision is None:
        return [*lines, "  vehicle_visual_odometry not logged"]

    flown = _in_spans(vision.times_s, flying)
    if flown.sum() < 2:
        return [*lines, "  under two vision samples while flying"]

    timing = vision_timing(vision.times_s[flown], sample_times_s(vision)[flown])
    return [*lines, f"  {timing.rate_hz:.1f} Hz (median interval), interval p95 "
                    f"{timing.interval_p95_s * 1000:.0f} ms, longest gap "
                    f"{timing.longest_gap_s * 1000:.0f} ms",
            f"  age as sent (timestamp - timestamp_sample): p50 {timing.age_p50_s * 1000:.1f} ms, "
            f"p95 {timing.age_p95_s * 1000:.1f} ms"]


def _delay_comparison(rows: list[tuple[float, float]]) -> list[str]:
    """The logs' vision innovation rms by EKF2_EV_DELAY, and the delay with the lowest."""
    lines = ["", "EKF2_EV_DELAY by the logs' horizontal EV position innovation rms, while flying"]
    delays = sorted({delay_ms for delay_ms, _ in rows})

    for delay_ms in delays:
        values = [rms for each_ms, rms in rows if each_ms == delay_ms]
        lines.append(f"  {delay_ms:g} ms: {np.mean(values) * 100:.2f} cm over {len(values)} logs")

    if len(delays) < 2:
        return [*lines, "  every log at one delay: fly another EKF2_EV_DELAY to compare"]

    return [*lines, f"  lowest: {best_delay_ms(rows):g} ms"]


def build_parser() -> argparse.ArgumentParser:
    """The command line."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("logs", type=Path, nargs="+",
                        help="PX4 logs (.ulg); logs flown at different EKF2_EV_DELAY are compared")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Reads each log, prints its report, then compares the logs' vision innovations by delay."""
    args = build_parser().parse_args(argv)
    missing = [str(path) for path in args.logs if not path.is_file()]
    if missing:
        print(f"no such log: {', '.join(missing)}", file=sys.stderr)
        return 1

    rows = []
    for path in args.logs:
        flight = read_flight(path)
        print("\n".join(report(flight)), end="\n\n")
        rms = ev_position_rms_m(flight, flying_spans(flight) or [Span(0.0, flight.duration_s)])
        delay_ms = flight.parameters.get("EKF2_EV_DELAY")
        if rms is not None and delay_ms is not None:
            rows.append((float(delay_ms), rms))

    if len(args.logs) > 1 and rows:
        print("\n".join(_delay_comparison(rows)))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
