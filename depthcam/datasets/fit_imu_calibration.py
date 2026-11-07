#!/usr/bin/env python3
"""Fit the accelerometer / gyroscope correction of an OAK from a six-pose recording.

Record once (camera still on each of its six faces for a few seconds, any order, exact levelling
not needed; lean it against something where the cable is in the way)::

    python datasets/07_record_dataset.py --seconds 60 --width 640 --height 400 --note "IMU six-pose"

then::

    python datasets/fit_imu_calibration.py recordings/euroc_<stamp>            # writes datasets/calibration/imu_<device id>.json
    python datasets/fit_imu_calibration.py recordings/euroc_<stamp> --dry-run  # print only
    python datasets/fit_imu_calibration.py --check recordings/euroc_<a> recordings/euroc_<b>

``--check`` prints, for every still stretch of the given recordings, the gravity magnitude before
and after the stored correction: the way to see whether the correction still fits (it drifts with
temperature).  No camera and no cuvslam needed.  Exit codes: 0 done, 1 the data cannot be fitted,
2 bad arguments or missing files.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cuvslam_oak import imu_calib  # noqa: E402


def device_id_of(recording: Path) -> str:
    """``device.device_id`` from the recording's ``dataset.json`` ('' when absent)."""
    for candidate in (recording / "dataset.json", recording.parent / "dataset.json"):
        if candidate.is_file():
            try:
                data = json.loads(candidate.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return ""
            return str((data.get("device") or {}).get("device_id") or "")
    return ""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fit or check the IMU correction of an OAK (six-pose method).")
    parser.add_argument("recording", nargs="*", type=Path,
                        help="dataset folder(s) written by datasets/07_record_dataset.py (one to fit, any number with --check)")
    parser.add_argument("--check", action="store_true",
                        help="do not fit: print |accel| of every still stretch before and after the stored correction")
    parser.add_argument("--file", type=Path, default=None, metavar="JSON",
                        help="correction file to write (fit) or read (--check); default datasets/calibration/imu_<device id>.json")
    parser.add_argument("--gravity", type=float, default=imu_calib.STANDARD_GRAVITY, metavar="M_PER_S2",
                        help="gravity magnitude to fit to (default %(default)s)")
    parser.add_argument("--dry-run", action="store_true", help="fit and print, write nothing")
    parser.add_argument("--notes", default="", help="free text stored in the file")
    return parser


def fit(args: argparse.Namespace) -> int:
    if len(args.recording) != 1:
        print("fit needs exactly one recording (got {})".format(len(args.recording)), file=sys.stderr)
        return 2
    recording: Path = args.recording[0]
    if not recording.exists():
        print("no such recording: {}".format(recording), file=sys.stderr)
        return 2
    try:
        t, gyro, accel = imu_calib.load_imu_csv(recording)
        correction, result, segments = imu_calib.fit_six_pose(t, gyro, accel, gravity=args.gravity)
    except imu_calib.ImuCalibError as exc:
        print("cannot fit {}: {}".format(recording, exc), file=sys.stderr)
        return 1
    print("Recording: {} ({} samples, {:.1f} s)".format(recording, len(t), t[-1]))
    print("Still poses:")
    for seg in segments:
        print("  {:5.1f}-{:5.1f} s  accel ({:+7.3f}, {:+7.3f}, {:+7.3f})  |a| {:6.3f} m/s^2  |gyro| {:.3f} rad/s".format(
            seg.start_s, seg.end_s, *seg.accel_mean, seg.norm, seg.gyro_norm_mean))
    print("Fit: {}".format(correction.describe()))
    print("     raw |a| {:.2f} .. {:.2f} m/s^2 -> corrected {:.3f} +/- {:.4f} ({} observations, condition {:.0f})".format(
        result.raw_norm_min, result.raw_norm_max, args.gravity, result.residual_std, len(result.residuals),
        result.condition))
    correction.device_id = device_id_of(recording)
    correction.source = recording.name
    correction.fitted = _dt.datetime.now().astimezone().isoformat(timespec="seconds")
    correction.notes = args.notes
    if args.dry_run:
        print("Dry run: nothing written.")
        return 0
    if args.file is None and not correction.device_id:
        print("the recording has no device id; give --file", file=sys.stderr)
        return 2
    path = correction.save(args.file if args.file is not None else imu_calib.default_path(correction.device_id))
    print("Wrote {}".format(path))
    return 0


def check(args: argparse.Namespace) -> int:
    if not args.recording:
        print("--check needs at least one recording", file=sys.stderr)
        return 2
    status = 0
    for recording in args.recording:
        if not recording.exists():
            print("no such recording: {}".format(recording), file=sys.stderr)
            return 2
        try:
            correction: Optional[imu_calib.ImuCorrection]
            if args.file is not None:
                correction = imu_calib.ImuCorrection.load(args.file)
            else:
                correction = imu_calib.load_for_device(device_id_of(recording))
            t, gyro, accel = imu_calib.load_imu_csv(recording)
        except imu_calib.ImuCalibError as exc:
            print("{}: {}".format(recording, exc), file=sys.stderr)
            return 2
        if correction is None:
            print("{}: no stored correction for device {!r}".format(recording, device_id_of(recording)), file=sys.stderr)
            return 2
        segments = imu_calib.find_static_segments(t, gyro, accel, min_duration_s=1.0)
        print("{} ({}): {} still stretch(es)".format(recording.name, correction.path or "correction", len(segments)))
        errors: List[float] = []
        for seg in segments:
            _g, fixed = correction.apply(seg.gyro_mean, seg.accel_mean)
            norm = float(np.linalg.norm(fixed))
            errors.append(abs(norm - correction.gravity))
            print("  {:5.1f}-{:5.1f} s  raw |a| {:6.3f}  corrected |a| {:6.3f}  ({:+.3f} from {:.3f})".format(
                seg.start_s, seg.end_s, seg.norm, norm, norm - correction.gravity, correction.gravity))
        if errors and max(errors) > 0.15:
            print("  -> off by up to {:.2f} m/s^2: refit with a new six-pose recording".format(max(errors)))
            status = 1
    return status


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return check(args) if args.check else fit(args)


if __name__ == "__main__":
    sys.exit(main())
