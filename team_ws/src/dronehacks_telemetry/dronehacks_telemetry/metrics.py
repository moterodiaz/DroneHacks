"""What a flight's time series measure: setpoint tracking per leg, stream rates and gaps, latency,
height agreement, clearance from the course's walls, and error against a reference track.

Times are seconds on one clock, the bag's receive time unless a function says otherwise.
Positions are metres; NED where they come from PX4, course east/north where they have been
placed on the map. Nothing here reads a file or knows a topic name.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

# A stream silent this long between header stamps has a gap: about ten frames of 30 Hz vision
GAP_S = 0.3

# A setpoint step longer than this starts a new leg; a setpoint moving slower stays in one leg
LEG_STEP_M = 0.05

# Arming and flying are held on this grid; its step is their resolution
_GRID_HZ = 50.0

# vehicle_status values: armed, and the Offboard navigation state (PX4 v1.16's VehicleStatus.msg)
ARMED = 2
NAV_OFFBOARD = 14


class Span(NamedTuple):
    """A stretch of the flight."""

    start_s: float
    end_s: float


class Percentiles(NamedTuple):
    """A quantity's median, 95th percentile and maximum."""

    p50: float
    p95: float
    max: float


@dataclass(frozen=True, slots=True)
class StreamTiming:
    """A stream's rate and gaps from its header stamps."""

    count: int                 # messages
    rate_hz: float             # from the median interval between stamps
    longest_gap_s: float       # between consecutive stamps
    gaps: list[Span]           # each interval past `GAP_S`, on the receive clock


@dataclass(frozen=True, slots=True, kw_only=True)
class Leg:
    """How PX4's estimate followed one setpoint leg."""

    index: int                             # from 1, in flight order
    start_s: float                         # when the leg's first setpoint arrived
    end_s: float                           # when the next leg's did, or Offboard ended
    target_ned: tuple[float, float, float] # the leg's last setpoint
    length_m: float                        # horizontal, from the estimate at the start to the target
    reach_s: float | None                  # from the start until within the arrival radius; None never
    final_error_m: float                   # horizontal, estimate to setpoint, at the leg's end
    max_error_m: float                     # horizontal, estimate to the setpoint held at each sample
    overshoot_m: float                     # furthest past the target along the leg's direction
    cross_track_m: float                   # furthest off the line from the start to the target
    height_error_m: float                  # largest vertical estimate-to-setpoint difference
    heading_swing_deg: float               # range of PX4's heading over the leg
    vision_gaps: int                       # vision gaps overlapping the leg


@dataclass(frozen=True, slots=True)
class HeightAgreement:
    """The rangefinder against PX4's height estimate, while flying."""

    offset_m: float            # median of range minus estimated height: the sensor's mount, mostly
    spread_p95_m: float        # 95th percentile of the disagreement once the offset is taken out
    worst_m: float             # largest disagreement once the offset is taken out
    valid_fraction: float      # of the flying range readings


@dataclass(frozen=True, slots=True, kw_only=True)
class Clearance:
    """The track's closest approach to the walls and its touches."""

    nearest_m: float                       # horizontal, to the nearest wall face, while flying
    nearest_at: tuple[float, float]        # east, north where that was
    nearest_s: float                       # when
    touches: list[Span]                    # each stretch nearer a wall than the prop reach
    goals_m: dict[str, float]              # each goal's closest horizontal pass


def held_at(change_times_s: np.ndarray, values: np.ndarray, at_s: np.ndarray) -> np.ndarray:
    """A value held from each sample to the next, read at `at_s`; the first value before it."""
    indices = np.searchsorted(change_times_s, at_s, side="right") - 1
    return values[np.clip(indices, 0, values.size - 1)]


def true_spans(times_s: np.ndarray, inside: np.ndarray) -> list[Span]:
    """Each run of true samples, from its first sample to the sample after its last, or to its
    last where it runs to the end."""
    edges = np.flatnonzero(np.diff(np.concatenate([[0], inside.astype(int), [0]])))
    return [Span(float(times_s[start]), float(times_s[min(end, times_s.size - 1)]))
            for start, end in zip(edges[::2], edges[1::2], strict=True)]


def in_spans(times_s: np.ndarray, spans: Sequence[tuple[float, float]]) -> np.ndarray:
    """Which times fall inside any of `spans`."""
    inside = np.zeros(times_s.shape, dtype=bool)
    for start_s, end_s in spans:
        inside |= (times_s >= start_s) & (times_s <= end_s)

    return inside


def state_spans(times_s: np.ndarray, values: np.ndarray, wanted: float,
                duration_s: float) -> list[Span]:
    """The stretches a state logged at `times_s` held `wanted`, on the arming grid."""
    if times_s.size == 0:
        return []

    grid = np.arange(0.0, duration_s + 1 / _GRID_HZ, 1 / _GRID_HZ)
    return true_spans(grid, held_at(times_s, values, grid) == wanted)


def flying_spans(armed: list[Span], landed_times_s: np.ndarray | None,
                 landed: np.ndarray | None, duration_s: float) -> list[Span]:
    """The armed stretches PX4's land detector did not call landed; armed alone without it."""
    if landed_times_s is None or landed is None or landed_times_s.size == 0:
        return armed

    grid = np.arange(0.0, duration_s + 1 / _GRID_HZ, 1 / _GRID_HZ)
    airborne = in_spans(grid, armed) & (held_at(landed_times_s, landed, grid) < 0.5)
    return true_spans(grid, airborne)


def stream_timing(times_s: np.ndarray, stamps_s: np.ndarray, gap_s: float = GAP_S) -> StreamTiming | None:
    """A stream's rate and gaps from its stamps, gaps timed on the receive clock; None under two
    messages."""
    if stamps_s.size < 2:
        return None

    intervals_s = np.diff(stamps_s)
    forward = intervals_s[intervals_s > 0]
    gaps = [Span(float(times_s[index]), float(times_s[index]) + float(intervals_s[index]))
            for index in np.flatnonzero(intervals_s > gap_s)]
    return StreamTiming(count=int(stamps_s.size),
                        rate_hz=float(1 / np.median(forward)) if forward.size else 0.0,
                        longest_gap_s=float(intervals_s.max()), gaps=gaps)


def percentiles(values: np.ndarray) -> Percentiles | None:
    """The finite values' median, 95th percentile and maximum; None with none."""
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None

    return Percentiles(float(np.percentile(finite, 50)), float(np.percentile(finite, 95)),
                       float(finite.max()))


def legs(setpoint_times_s: np.ndarray, setpoint_ned: np.ndarray, estimate_times_s: np.ndarray,
         estimate_ned: np.ndarray, heading_rad: np.ndarray, windows: Sequence[Span],
         arrival_radius_m: float, gaps: Sequence[Span] = ()) -> list[Leg]:
    """Each setpoint leg flown inside `windows` (Offboard), and how the estimate followed it.

    A leg is a run of setpoints none of which steps more than `LEG_STEP_M` from the one before,
    so a waypoint mission's legs are its waypoints. A setpoint giving no position (NaN, the kit
    commanding velocity) is left out.
    """
    found: list[Leg] = []
    for window in windows:
        inside = in_spans(setpoint_times_s, [window]) & np.all(np.isfinite(setpoint_ned), axis=1)
        times_s, targets = setpoint_times_s[inside], setpoint_ned[inside]
        if times_s.size == 0:
            continue

        # A step past the threshold starts a new leg
        steps_m = np.linalg.norm(np.diff(targets, axis=0), axis=1)
        starts = np.concatenate([[0], np.flatnonzero(steps_m > LEG_STEP_M) + 1])
        ends = np.append(starts[1:], times_s.size)
        for first, last in zip(starts, ends, strict=True):
            end_s = float(times_s[last]) if last < times_s.size else window.end_s
            found.append(_leg(len(found) + 1, times_s[first:last], targets[first:last], end_s,
                              estimate_times_s, estimate_ned, heading_rad, arrival_radius_m, gaps))

    return found


def _leg(index: int, times_s: np.ndarray, targets: np.ndarray, end_s: float,
         estimate_times_s: np.ndarray, estimate_ned: np.ndarray, heading_rad: np.ndarray,
         arrival_radius_m: float, gaps: Sequence[Span]) -> Leg:
    """One leg's tracking, from its setpoints and the estimate over its time."""
    start_s = float(times_s[0])
    target = targets[-1]
    during = (estimate_times_s >= start_s) & (estimate_times_s <= end_s)
    position = estimate_ned[during]
    if position.shape[0] == 0:
        return Leg(index=index, start_s=start_s, end_s=end_s, target_ned=_triple(target),
                   length_m=float("nan"), reach_s=None, final_error_m=float("nan"),
                   max_error_m=float("nan"), overshoot_m=float("nan"), cross_track_m=float("nan"),
                   height_error_m=float("nan"), heading_swing_deg=float("nan"),
                   vision_gaps=_overlapping(gaps, start_s, end_s))

    # The setpoint each estimate sample was chasing, held from its arrival
    chased = targets[np.clip(np.searchsorted(times_s, estimate_times_s[during], side="right") - 1,
                             0, len(targets) - 1)]
    error_m = np.linalg.norm(position[:, :2] - chased[:, :2], axis=1)
    to_target_m = np.linalg.norm(position[:, :2] - target[:2], axis=1)
    reached = np.flatnonzero(to_target_m < arrival_radius_m)

    # Overshoot and cross-track along the line from where the leg began to its target
    leg_vector = target[:2] - position[0, :2]
    length_m = float(np.linalg.norm(leg_vector))
    overshoot_m, cross_track_m = float("nan"), float("nan")
    if length_m > LEG_STEP_M:
        direction = leg_vector / length_m
        relative = position[:, :2] - position[0, :2]
        overshoot_m = max(0.0, float(np.max((position[:, :2] - target[:2]) @ direction)))
        cross_track_m = float(np.max(np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])))

    heading = np.unwrap(heading_rad[during])
    return Leg(index=index, start_s=start_s, end_s=end_s, target_ned=_triple(target), length_m=length_m,
               reach_s=float(estimate_times_s[during][reached[0]] - start_s) if reached.size else None,
               final_error_m=float(error_m[-1]), max_error_m=float(error_m.max()),
               overshoot_m=overshoot_m, cross_track_m=cross_track_m,
               height_error_m=float(np.max(np.abs(position[:, 2] - chased[:, 2]))),
               heading_swing_deg=float(np.degrees(heading.max() - heading.min())),
               vision_gaps=_overlapping(gaps, start_s, end_s))


def _triple(values: np.ndarray) -> tuple[float, float, float]:
    """Three values as a tuple of floats."""
    return float(values[0]), float(values[1]), float(values[2])


def _overlapping(spans: Sequence[Span], start_s: float, end_s: float) -> int:
    """How many spans overlap a stretch."""
    return sum(1 for span in spans if span.start_s < end_s and span.end_s > start_s)


def height_agreement(times_s: np.ndarray, height_m: np.ndarray, range_m: np.ndarray,
                     valid: np.ndarray, flying: Sequence[Span]) -> HeightAgreement | None:
    """The rangefinder against the estimated height, both at `times_s`; None without two valid
    flying readings."""
    flown = in_spans(times_s, flying)
    usable = flown & valid & np.isfinite(range_m) & np.isfinite(height_m)
    if usable.sum() < 2:
        return None

    difference_m = range_m[usable] - height_m[usable]
    offset_m = float(np.median(difference_m))
    deviation_m = np.abs(difference_m - offset_m)
    return HeightAgreement(offset_m=offset_m, spread_p95_m=float(np.percentile(deviation_m, 95)),
                           worst_m=float(deviation_m.max()),
                           valid_fraction=float(usable.sum() / max(1, flown.sum())))


def horizontal_error(times_s: np.ndarray, track_xy: np.ndarray, reference_times_s: np.ndarray,
                     reference_xy: np.ndarray, spans: Sequence[Span]) -> Percentiles | None:
    """The horizontal distance from a track to a reference interpolated at the track's times,
    inside `spans` and the reference's own time range."""
    inside = (in_spans(times_s, spans) & (times_s >= reference_times_s[0])
              & (times_s <= reference_times_s[-1])) if reference_times_s.size else np.zeros(0, bool)
    if not inside.any():
        return None

    reference = np.column_stack([np.interp(times_s[inside], reference_times_s, reference_xy[:, axis])
                                 for axis in range(2)])
    return percentiles(np.linalg.norm(track_xy[inside] - reference, axis=1))


def yaw_error_deg(times_s: np.ndarray, yaw_rad: np.ndarray, reference_times_s: np.ndarray,
                  reference_yaw_rad: np.ndarray, spans: Sequence[Span]) -> Percentiles | None:
    """The absolute yaw difference from a reference interpolated at the track's times, degrees."""
    inside = (in_spans(times_s, spans) & (times_s >= reference_times_s[0])
              & (times_s <= reference_times_s[-1])) if reference_times_s.size else np.zeros(0, bool)
    if not inside.any():
        return None

    reference = np.interp(times_s[inside], reference_times_s, np.unwrap(reference_yaw_rad))
    difference = (yaw_rad[inside] - reference + np.pi) % (2 * np.pi) - np.pi
    return percentiles(np.degrees(np.abs(difference)))


def clearance(times_s: np.ndarray, points_en: np.ndarray, wall_clearance_m: np.ndarray,
              flying: Sequence[Span], goals: dict[str, tuple[float, float]],
              reach_m: float) -> Clearance | None:
    """The flying track's closest wall approach, its touches within `reach_m`, and each goal's
    closest pass; None if it never flew."""
    flown = in_spans(times_s, flying)
    if not flown.any():
        return None

    masked = np.where(flown, wall_clearance_m, np.inf)
    nearest = int(np.argmin(masked))
    goals_m = {name: float(np.min(np.linalg.norm(points_en[flown] - np.array(at), axis=1)))
               for name, at in goals.items()}
    return Clearance(nearest_m=float(masked[nearest]),
                     nearest_at=(float(points_en[nearest, 0]), float(points_en[nearest, 1])),
                     nearest_s=float(times_s[nearest]), touches=true_spans(times_s, masked < reach_m),
                     goals_m=goals_m)
