"""Reads one flight folder and writes its report: `report.md` and `legs.csv` beside the bag.

The folder is what `scripts/record_flight.sh` writes (a `bag/`, a `meta/` and, once fetched,
PX4's `.ulg`), or any rosbag2 mcap folder from PX4 SITL or the FlyX lab. The same measures run on
all three; a `--truth` log from a simulator adds estimate-versus-truth. Times in the report are
seconds from the bag's first message.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path

import numpy as np

from dronehacks_telemetry import course as course_frames
from dronehacks_telemetry import metrics, ulog_triage
from dronehacks_telemetry.bag import BagFlight, Series, read_bag

# The mission's default arrival radius (the manual mission package's `arrival_radius_m`)
DEFAULT_ARRIVAL_RADIUS_M = 0.12

# cuVSLAM's VisualSlamStatus.vo_state for tracking (the kit bridge's VO_STATE_SUCCESS)
_VO_TRACKING = 1

# A frame mapping whose median residual against vision is past this is reported as not holding
_FRAME_AGREEMENT_M = 0.5

# The most timeline entries listed before the rest are only counted
_MAX_EVENTS = 60

# Receive and header clocks further apart than this are on different time bases: sim and wall
_CLOCK_MISMATCH_S = 3600.0


def build_parser() -> argparse.ArgumentParser:
    """The command line."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("flight", type=Path, help="the flight folder, or a bare rosbag2 mcap folder")
    parser.add_argument("--course-map", type=Path, default=None,
                        help="course map YAML; default the folder's meta/course_map.yaml")
    parser.add_argument("--course-to-odom", type=float, nargs=3, metavar=("YAW_RAD", "X_M", "Y_M"),
                        default=None, help="measured course-to-odom transform (contracts/frames.md); "
                                           "default the mission's parameters in meta/params, else the "
                                           "launch heading, assumed")
    parser.add_argument("--px4-frame", choices=["auto", *[frame.value for frame in course_frames.Px4Frame]],
                        default="auto", help="how PX4's local frame sits on the odom frame; auto picks "
                                             "the one agreeing with cuVSLAM")
    parser.add_argument("--arrival-radius-m", type=float, default=DEFAULT_ARRIVAL_RADIUS_M,
                        help="a leg is reached inside this horizontal distance of its target")
    parser.add_argument("--ulog", type=Path, default=None, help="PX4's log; default any .ulg in the folder")
    parser.add_argument("--truth", type=Path, default=None,
                        help="a simulator's truth CSV: sim ns, course ENU position, XYZW quaternion")
    parser.add_argument("--out", type=Path, default=None, help="where to write; default the folder")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Reads the flight, prints its report and writes it with the legs' CSV."""
    args = build_parser().parse_args(argv)
    if args.arrival_radius_m <= 0:
        print(f"--arrival-radius-m must be positive, not {args.arrival_radius_m}", file=sys.stderr)
        return 2

    try:
        bag = read_bag(args.flight)
    except (FileNotFoundError, ImportError, ValueError) as failure:
        print(f"cannot read the bag: {failure}", file=sys.stderr)
        return 1

    lines, legs = report(bag, args)
    out = args.out or args.flight
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    write_legs_csv(out / "legs.csv", legs)
    print("\n".join(lines))
    print(f"\nwrote {out / 'report.md'} and {out / 'legs.csv'}", file=sys.stderr)
    return 0


def report(bag: BagFlight, args: argparse.Namespace) -> tuple[list[str], list[metrics.Leg]]:
    """The report's lines and the legs it measured."""
    status = bag.series.get("status")
    armed = (metrics.state_spans(status.times_s, status.fields["arming_state"], metrics.ARMED,
                                 bag.duration_s) if status else [])
    land = bag.series.get("land_detected")
    flying = metrics.flying_spans(armed, land.times_s if land else None,
                                  land.fields["landed"] if land else None, bag.duration_s)
    vision_gaps = _vision_gaps(bag)
    frame, frame_lines = _px4_frame(bag, args, flying)

    legs, setpoint_source = _legs(bag, status, frame, args.arrival_radius_m, vision_gaps)
    lines = [*_header(bag, args.flight, armed, flying), *_timeline(bag, armed, flying, vision_gaps),
             *_tracking_section(legs, setpoint_source, args.arrival_radius_m),
             *_vision_section(bag, flying), *_latency_section(bag), *_height_section(bag, flying),
             *_course_section(bag, args, flying, frame, frame_lines), *_ulog_section(args, armed)]
    return lines, legs


def write_legs_csv(path: Path, legs: Sequence[metrics.Leg]) -> None:
    """One row per leg, every `Leg` field a column; the target as three columns."""
    columns = ["index", "start_s", "end_s", "target_n", "target_e", "target_d", "length_m", "reach_s",
               "final_error_m", "max_error_m", "overshoot_m", "cross_track_m", "height_error_m",
               "heading_swing_deg", "vision_gaps"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for leg in legs:
            row = asdict(leg)
            row["target_n"], row["target_e"], row["target_d"] = row.pop("target_ned")
            writer.writerow([_cell(row[column]) for column in columns])


def _cell(value: object) -> str:
    """A CSV cell: floats to the millimetre or millisecond, None and NaN empty."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""

    return f"{value:.3f}" if isinstance(value, float) else str(value)


def read_manifest(folder: Path) -> dict[str, str]:
    """The recorder's `meta/manifest.txt`, `key: value` per line; empty without one."""
    path = folder / "meta" / "manifest.txt"
    if not path.is_file():
        return {}

    pairs = (line.split(":", 1) for line in path.read_text(encoding="utf-8").splitlines() if ":" in line)
    return {key.strip(): value.strip() for key, value in pairs}


def mission_transform(folder: Path) -> course_frames.CourseToOdom | None:
    """The course-to-odom transform the mission flew with, from the recorder's parameter dumps;
    None where no node carries all three finite values."""
    for path in sorted((folder / "meta" / "params").glob("*.yaml")):
        text = path.read_text(encoding="utf-8", errors="replace")
        values = [re.search(rf"^\s*course_to_odom_{name}:\s*(\S+)", text, re.MULTILINE)
                  for name in ("yaw", "x", "y")]
        if not all(values):
            continue

        # A parameter left at its NaN default dumps as `.nan`, which is not a measurement
        try:
            numbers = [float(match.group(1)) for match in values if match]
        except ValueError:
            continue

        if all(math.isfinite(number) for number in numbers):
            return course_frames.CourseToOdom(*numbers)

    return None


def _header(bag: BagFlight, folder: Path, armed: list[metrics.Span],
            flying: list[metrics.Span]) -> list[str]:
    """The run's identity, what was recorded, and when it was armed and flying."""
    manifest = read_manifest(folder)
    lines = [f"# Flight report: {manifest.get('run_id', folder.name)}", ""]
    for key in ("profile", "label", "started_utc", "stopped_utc", "hostname", "team_git_sha",
                "team_git_dirty", "skybeta_version", "course_map", "course_map_sha256", "bag_size"):
        if key in manifest:
            lines.append(f"- {key}: {manifest[key]}")

    missing = [role for role in ("local_position", "status", "trajectory_setpoint", "vision",
                                 "vision_status", "bridge") if role not in bag.series]
    lines += [f"- bag: {bag.duration_s:.1f} s, {len(bag.topics)} topics, "
              f"{sum(bag.topics.values())} messages",
              f"- armed: {_spans_text(armed)}", f"- flying: {_spans_text(flying)}"]
    if missing:
        lines.append(f"- not in the bag: {', '.join(missing)}")

    if bag.undecodable:
        lines.append(f"- recorded without a message definition, so not read: {', '.join(bag.undecodable)}"
                     " (the recorder's environment lacked the package)")

    return lines


def _spans_text(spans: Sequence[metrics.Span]) -> str:
    """Spans as `a-b s` text, or 'never'."""
    return ", ".join(f"{span.start_s:.1f}-{span.end_s:.1f} s" for span in spans) or "never"


def _vision_gaps(bag: BagFlight) -> list[metrics.Span]:
    """The gaps in what the bridge sends PX4, else in cuVSLAM's odometry, by their stamps."""
    bridge = bag.series.get("bridge")
    if bridge is not None:
        timing = metrics.stream_timing(bridge.times_s, bridge.fields["sample_us"] * 1e-6)
        return timing.gaps if timing else []

    vision = bag.series.get("vision")
    timing = metrics.stream_timing(vision.times_s, vision.fields["stamp_s"]) if vision else None
    return timing.gaps if timing else []


def _timeline(bag: BagFlight, armed: list[metrics.Span], flying: list[metrics.Span],
              vision_gaps: list[metrics.Span]) -> list[str]:
    """Arming, modes, failsafes, tracking losses, vision gaps, fusion dropouts and logged
    warnings, in time order."""
    events: list[tuple[float, str]] = []
    status = bag.series.get("status")
    if status is not None:
        events += [(time_s, ulog_triage._arming_name(state)) for time_s, state in
                   ulog_triage.state_changes(status.times_s, status.fields["arming_state"])]
        events += [(time_s, f"mode {ulog_triage._nav_name(state)}") for time_s, state in
                   ulog_triage.state_changes(status.times_s, status.fields["nav_state"])]
        events += ulog_triage._rise_events(status.times_s, status.fields["failsafe"], "failsafe")

    # Failsafe conditions from the first arming: before it, start-up conditions are all true
    failsafes = bag.series.get("failsafe_flags")
    if failsafes is not None and armed:
        since = failsafes.times_s >= armed[0].start_s
        for name, values in failsafes.fields.items():
            events += ulog_triage._rise_events(failsafes.times_s[since], values[since], name)

    vision_status = bag.series.get("vision_status")
    if vision_status is not None:
        events += [(time_s, "cuVSLAM tracking" if state == _VO_TRACKING else f"cuVSLAM vo_state {state}")
                   for time_s, state in ulog_triage.state_changes(vision_status.times_s,
                                                                  vision_status.fields["vo_state"])]

    events += [(gap.start_s, f"vision gap {gap.end_s - gap.start_s:.2f} s") for gap in vision_gaps]
    events += _fusion_events(bag, flying) + _kit_events(bag)
    events += [(line.time_s, f"{line.node} [{line.level}]: {line.text}") for line in bag.log]
    events.sort(key=lambda event: event[0])

    lines = ["", "## Timeline", ""]
    lines += [f"- {time_s:7.2f} s  {text}" for time_s, text in events[:_MAX_EVENTS]]
    if len(events) > _MAX_EVENTS:
        lines.append(f"- ... {len(events) - _MAX_EVENTS} more")

    return lines


def _fusion_events(bag: BagFlight, flying: list[metrics.Span]) -> list[tuple[float, str]]:
    """Each EKF2 vision or range fusion dropout while flying, from the bag's status flags."""
    flags = bag.series.get("fusion_flags")
    if flags is None or not flying:
        return []

    events = []
    for name in ("cs_ev_pos", "cs_ev_yaw", "cs_rng_hgt"):
        values = flags.fields.get(name)
        if values is None or np.all(np.isnan(values)):
            continue

        held = ulog_triage.flag_time(flags.times_s, values, flying)
        events += [(dropout.start_s, f"EKF2 {name} off for {dropout.duration_s:.2f} s")
                   for dropout in held.dropouts]

    return events


def _kit_events(bag: BagFlight) -> list[tuple[float, str]]:
    """The flight interface losing PX4, freezing, clamping or leaving its envelope."""
    state = bag.series.get("kit_state")
    if state is None:
        return []

    events = []
    for name, label in (("clamped", "kit clamped a setpoint"), ("frozen", "kit frozen")):
        events += [(rise_s, label) for rise_s, _ in ulog_triage.rises(state.times_s, state.fields[name])]

    for name, label in (("px4_link", "kit lost PX4"), ("envelope_ok", "kit outside its envelope")):
        events += [(rise_s, label) for rise_s, _ in ulog_triage.rises(state.times_s, 1 - state.fields[name])]

    return events


def _legs(bag: BagFlight, status: Series | None, frame: course_frames.Px4Frame,
          arrival_radius_m: float, vision_gaps: list[metrics.Span]) -> tuple[list[metrics.Leg], str]:
    """The setpoint legs flown in Offboard against PX4's estimate, and what the setpoints were:
    the kit's PX4 setpoints, else the competitor's pose setpoints put in PX4's frame."""
    estimate = bag.series.get("local_position")
    if estimate is None or status is None:
        return [], "no PX4 local position or status"

    # The kit's own PX4 setpoint is already NED; a pose setpoint is odom ENU, unanchored as the
    # kit's default flies it
    setpoint = bag.series.get("trajectory_setpoint")
    pose = bag.series.get("pose_setpoint")
    if setpoint is not None:
        targets, times_s = _xyz(setpoint), setpoint.times_s
        source = "the kit's /fmu/in/trajectory_setpoint"
    elif pose is not None:
        targets, times_s = course_frames.px4_from_odom(_xyz(pose), frame), pose.times_s
        source = f"/skybeta/setpoint/pose, put in PX4's frame as `{frame.value}`"
    else:
        return [], "no setpoint recorded"

    offboard = metrics.state_spans(status.times_s, status.fields["nav_state"], metrics.NAV_OFFBOARD,
                                   bag.duration_s)
    legs = metrics.legs(times_s, targets, estimate.times_s, _xyz(estimate),
                        estimate.fields["heading"], offboard, arrival_radius_m, vision_gaps)
    return legs, source


def _xyz(series: Series) -> np.ndarray:
    """A series' x, y, z fields as an (n, 3) array."""
    return np.column_stack([series.fields["x"], series.fields["y"], series.fields["z"]])


def _tracking_section(legs: list[metrics.Leg], source: str, arrival_radius_m: float) -> list[str]:
    """Setpoint tracking per leg: the setpoints against PX4's own estimate, both NED."""
    lines = ["", "## Setpoint tracking (setpoint vs PX4's estimate, NED, in Offboard)", "",
             f"Setpoints: {source}."]
    if not legs:
        return [*lines, "No position setpoint while in Offboard."]

    lines += [f"Arrival radius {arrival_radius_m:.2f} m. Errors are horizontal unless named.", "",
              "| leg | start s | target N,E,D m | length m | reach s | final err m | max err m "
              "| overshoot m | cross-track m | height err m | heading swing deg | vision gaps |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for leg in legs:
        target = ",".join(f"{value:+.2f}" for value in leg.target_ned)
        reach = "never" if leg.reach_s is None else f"{leg.reach_s:.1f}"
        lines.append(f"| {leg.index} | {leg.start_s:.1f} | {target} | {leg.length_m:.2f} | {reach} | "
                     f"{leg.final_error_m:.2f} | {leg.max_error_m:.2f} | {leg.overshoot_m:.2f} | "
                     f"{leg.cross_track_m:.2f} | {leg.height_error_m:.2f} | {leg.heading_swing_deg:.0f} | "
                     f"{leg.vision_gaps} |")

    return lines


def _vision_section(bag: BagFlight, flying: list[metrics.Span]) -> list[str]:
    """Each stream's rate and gaps from its header stamps, and cuVSLAM's tracking state."""
    lines = ["", f"## Vision health (rates from header stamps; a gap is over {metrics.GAP_S:g} s)", ""]
    streams = (("camera (left camera_info)", "camera", "stamp_s", 1.0),
               ("IMU", "imu", "stamp_s", 1.0), ("cuVSLAM odometry", "vision", "stamp_s", 1.0),
               ("bridge to PX4 (timestamp_sample)", "bridge", "sample_us", 1e-6))
    for label, role, stamp_field, scale in streams:
        series = bag.series.get(role)
        timing = (metrics.stream_timing(series.times_s, series.fields[stamp_field] * scale)
                  if series else None)
        if timing is None:
            lines.append(f"- {label}: not recorded")
            continue

        listed = ", ".join(f"{gap.start_s:.1f} s ({gap.end_s - gap.start_s:.2f} s)" for gap in timing.gaps[:8])
        lines.append(f"- {label}: {timing.count} msgs, {timing.rate_hz:.1f} Hz, longest interval "
                     f"{timing.longest_gap_s * 1000:.0f} ms, {len(timing.gaps)} gaps"
                     f"{': ' + listed if listed else ''}")

    status = bag.series.get("vision_status")
    if status is not None:
        lost = metrics.true_spans(status.times_s, status.fields["vo_state"] != _VO_TRACKING)
        lost_flying = [span for span in lost if any(span.start_s < end and span.end_s > start
                                                    for start, end in flying)]
        lines.append(f"- cuVSLAM not tracking: {len(lost)} spans, {len(lost_flying)} while flying"
                     f"{': ' + _spans_text(lost_flying[:8]) if lost_flying else ''}")
        for name in ("track_execution_time_mean", "track_execution_time_max"):
            values = status.fields.get(name)
            stats = metrics.percentiles(values) if values is not None else None
            if stats is not None:
                lines.append(f"- {name}: p50 {stats.p50:.1f}, max {stats.max:.1f} (as cuVSLAM reports it)")

    bridge = bag.series.get("bridge")
    if bridge is not None:
        resets = np.flatnonzero(np.diff(bridge.fields["reset_counter"]) != 0)
        lines.append(f"- bridge reset_counter changes: {len(resets)}"
                     f"{': ' + ', '.join(f'{bridge.times_s[index + 1]:.1f} s' for index in resets[:8]) if resets.size else ''}")

    flags = bag.series.get("fusion_flags")
    if flags is not None and flying:
        for name in ("cs_ev_pos", "cs_ev_yaw", "cs_rng_hgt"):
            values = flags.fields.get(name)
            if values is not None and not np.all(np.isnan(values)):
                held = ulog_triage.flag_time(flags.times_s, values, flying)
                lines.append(f"- EKF2 {name} while flying: {held.true_fraction:.1%} true, "
                             f"{len(held.dropouts)} dropouts")
    elif flags is None:
        lines.append("- EKF2 fusion flags: not in the bag (the ULog section has them)")

    return lines


def _latency_section(bag: BagFlight) -> list[str]:
    """Camera to odometry to bridge latency, where the stamps allow."""
    lines = ["", "## Timing (receive time at the recorder minus the image's header stamp)", ""]
    vision, bridge = bag.series.get("vision"), bag.series.get("bridge")
    if vision is None and bridge is None:
        return [*lines, "- no cuVSLAM odometry or bridge output recorded"]

    if vision is not None:
        age = _received_clock_s(bag, vision) - vision.fields["stamp_s"]
        lines.append(_percentile_line("image -> cuVSLAM odometry received", age))

    if bridge is not None:
        sample_s = bridge.fields["sample_us"] * 1e-6
        lines.append(_percentile_line("image -> bridge message received", _received_clock_s(bag, bridge) - sample_s))
        lines.append(_percentile_line("bridge's own age (timestamp - timestamp_sample)",
                                      (bridge.fields["timestamp_us"] - bridge.fields["sample_us"]) * 1e-6))

    lines.append("- the serial hop to the FC is not in these; EKF2_EV_DELAY wants the ULog's lag")
    return lines


def _received_clock_s(bag: BagFlight, series: Series) -> np.ndarray:
    """A series' receive times on the clock its header stamps use: wall time, or the simulator's
    /clock where the stamps are sim time."""
    wall_s = bag.start_ns * 1e-9 + series.times_s
    clock = bag.series.get("clock")
    stamps = series.fields.get("stamp_s", series.fields.get("sample_us", np.zeros(1)) * 1e-6)
    if clock is None or abs(float(np.nanmedian(wall_s - stamps))) < _CLOCK_MISMATCH_S:
        return wall_s

    return np.interp(series.times_s, clock.times_s, clock.fields["sim_s"])


def _percentile_line(label: str, values_s: np.ndarray) -> str:
    """One latency's median, 95th percentile and maximum in milliseconds."""
    stats = metrics.percentiles(values_s)
    if stats is None:
        return f"- {label}: no samples"

    return f"- {label}: p50 {stats.p50 * 1000:.1f} ms, p95 {stats.p95 * 1000:.1f} ms, max {stats.max * 1000:.1f} ms"


def _height_section(bag: BagFlight, flying: list[metrics.Span]) -> list[str]:
    """The rangefinder against PX4's height estimate, while flying."""
    lines = ["", "## Height (rangefinder vs PX4's estimate, while flying)", ""]
    estimate = bag.series.get("local_position")
    if estimate is None:
        return [*lines, "- no PX4 local position"]

    height_m = -estimate.fields["z"]
    distance = bag.series.get("distance")
    if distance is not None:
        source = "/fmu/out/distance_sensor"
        range_m = distance.fields["current_distance"]
        valid = ulog_triage.range_valid(range_m, distance.fields["signal_quality"],
                                        distance.fields["min_distance"], distance.fields["max_distance"])
        height_m = np.interp(distance.times_s, estimate.times_s, height_m)
        agreement = metrics.height_agreement(distance.times_s, height_m, range_m, valid, flying)
    else:
        source = "vehicle_local_position.dist_bottom"
        agreement = metrics.height_agreement(estimate.times_s, height_m, estimate.fields["dist_bottom"],
                                             estimate.fields["dist_bottom_valid"] > 0.5, flying)

    if agreement is None:
        return [*lines, f"- {source}: under two valid readings while flying"]

    return [*lines, f"- {source}: valid {agreement.valid_fraction:.0%}; range minus height median "
                    f"{agreement.offset_m:+.3f} m (mount offset); disagreement beyond it p95 "
                    f"{agreement.spread_p95_m:.3f} m, worst {agreement.worst_m:.3f} m"]


def _course_section(bag: BagFlight, args: argparse.Namespace, flying: list[metrics.Span],
                    frame: course_frames.Px4Frame, frame_lines: list[str]) -> list[str]:
    """The flight placed on the course map: frame agreement with vision, wall clearance and goals,
    and estimate-versus-truth where a truth log is given."""
    lines = ["", "## Course (PX4's estimate placed on the course map)", "", *frame_lines]
    estimate = bag.series.get("local_position")
    map_path = args.course_map or args.flight / "meta" / "course_map.yaml"
    if estimate is None or not map_path.is_file():
        return [*lines, f"- needs PX4's local position and a course map ({map_path} not found)"]

    try:
        course = course_frames.load_course_map(map_path)
    except (ImportError, ValueError) as failure:
        return [*lines, f"- cannot read {map_path}: {failure}"]

    transform, source = _transform(args, course)
    odom_xyz, odom_yaw = course_frames.odom_from_px4(_xyz(estimate), estimate.fields["heading"], frame)
    track, track_yaw = course_frames.course_from_odom(odom_xyz, odom_yaw, course, transform)
    lines += [f"- map {map_path.name}; course-to-odom {source}: yaw {transform.yaw_rad:+.3f} rad, "
              f"odom ({transform.odom_x_m:+.3f}, {transform.odom_y_m:+.3f}) m"]

    # The launch mark is where the run starts, not a goal it passes
    goals = {name: at for name, at in course.goals.items() if name != "launch"}
    result = metrics.clearance(estimate.times_s, track[:, :2],
                               course_frames.wall_clearance_m(track[:, :2], course), flying, goals,
                               course_frames.PROP_REACH_M)
    if result is None:
        return [*lines, "- never flew: no clearance to measure"]

    boulder_m = course_frames.boulder_clearance_m(track[metrics.in_spans(estimate.times_s, flying), :2], course)
    lines += [f"- nearest wall face {result.nearest_m * 100:.0f} cm at ({result.nearest_at[0]:+.2f}, "
              f"{result.nearest_at[1]:+.2f}) at {result.nearest_s:.1f} s; prop reach "
              f"{course_frames.PROP_REACH_M * 100:.0f} cm; {len(result.touches)} touch spans"
              f"{': ' + _spans_text(result.touches[:8]) if result.touches else ''}",
              f"- nearest boulder edge (horizontal; passed over at flight height) "
              f"{float(boulder_m.min()) * 100:.0f} cm" if boulder_m.size else "- no boulders",
              "- closest pass per goal: " + ", ".join(f"{name} {distance_m * 100:.0f} cm"
                                                     for name, distance_m in result.goals_m.items()),
              "- this is the estimate's own view: drift moves the true track off it unseen"]
    if args.truth is not None:
        lines += _truth_lines(bag, args.truth, estimate, track, track_yaw, course, transform, flying)

    return lines


def _transform(args: argparse.Namespace,
               course: course_frames.CourseMap) -> tuple[course_frames.CourseToOdom, str]:
    """The course-to-odom transform and where it came from."""
    if args.course_to_odom is not None:
        return course_frames.CourseToOdom(*args.course_to_odom), "from the command line"

    flown = mission_transform(args.flight)
    if flown is not None:
        return flown, "the mission's parameters"

    return course_frames.launch_transform(course), "ASSUMED from the launch heading (not measured)"


def _px4_frame(bag: BagFlight, args: argparse.Namespace,
               flying: list[metrics.Span]) -> tuple[course_frames.Px4Frame, list[str]]:
    """The PX4 frame mapping to use, and lines giving each mapping's agreement with cuVSLAM."""
    estimate, vision = bag.series.get("local_position"), bag.series.get("vision")
    spans = flying or [metrics.Span(0.0, bag.duration_s)]
    residuals: dict[course_frames.Px4Frame, float] = {}
    if estimate is not None and vision is not None:
        vision_xy = np.column_stack([vision.fields["x"], vision.fields["y"]])
        for frame in course_frames.Px4Frame:
            odom_xyz, _ = course_frames.odom_from_px4(_xyz(estimate), estimate.fields["heading"], frame)
            error = metrics.horizontal_error(estimate.times_s, odom_xyz[:, :2], vision.times_s,
                                             vision_xy, spans)
            if error is not None:
                residuals[frame] = error.p50

    lines = [f"- PX4 `{frame.value}` mapping vs cuVSLAM odometry: median {residual:.2f} m apart"
             for frame, residual in residuals.items()]
    if args.px4_frame != "auto":
        chosen = course_frames.Px4Frame(args.px4_frame)
        return chosen, [*lines, f"- PX4 local frame taken as `{chosen.value}` (command line)"]

    if not residuals:
        return course_frames.Px4Frame.LAUNCH, [*lines, "- no cuVSLAM odometry to choose by: PX4 "
                                                       "local frame taken as `launch`, assumed"]

    best = min(residuals, key=residuals.__getitem__)
    lines.append(f"- PX4 local frame taken as `{best.value}`, the closer to vision")
    if residuals[best] > _FRAME_AGREEMENT_M:
        lines.append(f"- WARNING: neither mapping agrees with vision within {_FRAME_AGREEMENT_M} m")

    return best, lines


def _truth_lines(bag: BagFlight, truth_path: Path, estimate: Series, track: np.ndarray,
                 track_yaw: np.ndarray, course: course_frames.CourseMap,
                 transform: course_frames.CourseToOdom, flying: list[metrics.Span]) -> list[str]:
    """PX4's estimate and cuVSLAM's pose against a simulator's truth, aligned on /clock."""
    clock = bag.series.get("clock")
    if clock is None:
        return ["- truth given, but the bag has no /clock to align it on"]

    truth = np.array([row.split(",") for row in truth_path.read_text(encoding="utf-8").splitlines()
                      if row and row[0].isdigit()], dtype=float)
    truth_s = truth[:, 0] * 1e-9
    truth_yaw = np.array([math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                          for x, y, z, w in truth[:, 4:8]])
    sim_s = np.interp(estimate.times_s, clock.times_s, clock.fields["sim_s"])
    flying_sim = [metrics.Span(float(np.interp(start, clock.times_s, clock.fields["sim_s"])),
                               float(np.interp(end, clock.times_s, clock.fields["sim_s"])))
                  for start, end in flying]
    lines = [_error_line("PX4 estimate vs truth", metrics.horizontal_error(
                 sim_s, track[:, :2], truth_s, truth[:, 1:3], flying_sim), "m"),
             _error_line("PX4 heading vs truth", metrics.yaw_error_deg(
                 sim_s, track_yaw, truth_s, truth_yaw, flying_sim), "deg")]

    vision = bag.series.get("vision")
    if vision is not None:
        vision_xyz = np.column_stack([vision.fields["x"], vision.fields["y"], vision.fields["z"]])
        placed, _ = course_frames.course_from_odom(vision_xyz, vision.fields["yaw"], course, transform)
        vision_sim_s = np.interp(vision.times_s, clock.times_s, clock.fields["sim_s"])
        lines.append(_error_line("cuVSLAM vs truth", metrics.horizontal_error(
            vision_sim_s, placed[:, :2], truth_s, truth[:, 1:3], flying_sim), "m"))

    return lines


def _error_line(label: str, stats: metrics.Percentiles | None, unit: str) -> str:
    """One error's median, 95th percentile and maximum."""
    if stats is None:
        return f"- {label}: no overlap with the truth while flying"

    scale, shown = (100.0, "cm") if unit == "m" else (1.0, unit)
    return (f"- {label}: p50 {stats.p50 * scale:.1f} {shown}, p95 {stats.p95 * scale:.1f} {shown}, "
            f"max {stats.max * scale:.1f} {shown}")


def _ulog_section(args: argparse.Namespace, armed: list[metrics.Span]) -> list[str]:
    """The PX4 log's triage, its armed time against the bag's, and its parameters written out."""
    lines = ["", "## PX4 log (ULog)", ""]
    path = args.ulog or next(iter(sorted(args.flight.glob("**/*.ulg"))), None)
    if path is None:
        return [*lines, "No .ulg in the folder yet: fetch it (README) into px4/ and run again."]

    try:
        flight = ulog_triage.read_flight(path)
    except ImportError as missing:
        return [*lines, f"- {missing}"]

    status = ulog_triage._first(flight, "vehicle_status")
    ulog_armed_s = 0.0
    if status is not None:
        armed_ulog = metrics.state_spans(status.times_s, status.fields["arming_state"], metrics.ARMED,
                                         flight.duration_s)
        ulog_armed_s = sum(span.end_s - span.start_s for span in armed_ulog)

    bag_armed_s = sum(span.end_s - span.start_s for span in armed)
    out = args.out or args.flight
    out.mkdir(parents=True, exist_ok=True)
    write_qgc_params(out / "px4_params.params", flight.parameters)
    lines += [f"- {path.name}: armed {ulog_armed_s:.1f} s in the log, {bag_armed_s:.1f} s in the bag "
              "(a mismatch means the wrong log)",
              f"- parameters as the log started written to px4_params.params "
              f"({len(flight.parameters)}), QGC format", "", "```", *ulog_triage.report(flight), "```"]
    return lines


def write_qgc_params(path: Path, parameters: dict[str, float]) -> None:
    """Parameters in QGroundControl's .params format, integers as INT32 and the rest REAL32."""
    rows = ["# Onboard parameters from the flight's ULog", "# Vehicle-Id Component-Id Name Value Type"]
    for name in sorted(parameters):
        value = parameters[name]
        is_integer = isinstance(value, (int, np.integer)) and not isinstance(value, bool)
        rows.append(f"1\t1\t{name}\t{value if is_integer else repr(float(value))}\t{6 if is_integer else 9}")

    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
