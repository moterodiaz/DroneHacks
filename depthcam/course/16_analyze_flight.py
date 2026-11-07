#!/usr/bin/env python3
"""Flight log -> checkpoint times, tracking error, wall clearance, landing accuracy (+ a picture).

WHAT THIS DOES
    Reads a folder written by 15_fly_plan.py or the ROS 2 player (flight_log.csv,
    flight_events.json), finds the plan's summary JSON (next to the plan, else in course/out/) and
    the rules and map it was planned from, and runs ``course.flight_analysis.analyze``:
      * when the flown path crossed each checkpoint gate, from liftoff (the organisers' clock),
        next to the planned time; CP5 is the disarm on the pad;
      * setpoint-to-drone distance per phase (max / 95th percentile / RMS): size --margin and
        --a-max for 14_plan_course.py from these;
      * the closest the real drone outline (no margin) came to an obstacle, and every touch;
      * landing offset from the pad centre (whole drone inside the ring?) and from the launch mark
        after the return (within 2 ft?).
    Writes report.json and flight.png into the log folder.

HOW TO RUN (any OS)
    python course/16_analyze_flight.py course/out/flights/window_yaw90_20261028_101500
    python course/16_analyze_flight.py <log folder> --summary course/out/window_yaw90_summary.json

EXIT CODES
    0  analysed, no touches
    1  analysed, the drone outline touched an obstacle, or the run did not finish
    2  missing files
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from course.flight_analysis import analyze, report_lines  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log_dir", type=Path, help="folder with flight_log.csv and flight_events.json")
    ap.add_argument("--summary", type=Path, default=None, help="plan summary JSON (default: found from the log)")
    ap.add_argument("--no-image", action="store_true")
    args = ap.parse_args(argv)
    if not (args.log_dir / "flight_log.csv").is_file():
        print("no flight_log.csv in {}".format(args.log_dir), file=sys.stderr)
        return 2
    try:
        report, picture = analyze(args.log_dir, args.summary)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 2
    lines = report_lines(report)
    print("\n".join(lines))
    (args.log_dir / "report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    if picture and not args.no_image:
        import cv2
        from course.planner.render import render_flight
        cv2.imwrite(str(args.log_dir / "flight.png"), render_flight(picture["cmap"], picture, lines))
        print("wrote {}".format(args.log_dir / "flight.png"))
    touched = bool(report.get("clearance", {}).get("touches"))
    finished = report.get("finish_s") is not None
    return 1 if touched or not finished else 0


if __name__ == "__main__":
    sys.exit(main())
