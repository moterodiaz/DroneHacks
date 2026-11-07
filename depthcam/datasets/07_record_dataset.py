#!/usr/bin/env python3
"""Record a cuVSLAM dataset from the live OAK-D S2: EuRoC ASL layout, stereo GRAY8 + BNO086 IMU.

What this script does
---------------------
* Opens the camera, reads its EEPROM calibration and streams the hardware-synchronised OV9282
  stereo pair (CAM_B = left = cam0, CAM_C = right = cam1) as GRAY8 at --width x --height
  (1280x800 native by default, 640x400 allowed) and --fps, plus ACCELEROMETER_RAW +
  GYROSCOPE_RAW at --imu-rate (200 Hz default) through
  ``cuvslam_oak.oak_source.OakStereoImuSource`` (the official PyCuVSLAM OAK-D graph + IMU).
* Undistorts every frame on the host with all 14 EEPROM coefficients, intrinsics unchanged
  (``cuvslam_oak.rig.undistort_maps``), and writes the recording through
  ``cuvslam_oak.dataset.EurocWriter`` into ``recordings/euroc_<YYYYmmdd_HHMMSS>/`` (or --out)::

      dataset.json, calibration.json, README.txt          # ours (datasets/dataset-format.md 12)
      mav0/cam0/{data/<ts_ns>.png, data.csv, sensor.yaml}  # left  = CAM_B, T_BS = identity (body = cam0)
      mav0/cam1/{data/<ts_ns>.png, data.csv, sensor.yaml}  # right = CAM_C, T_BS = B_from_C, metres
      mav0/imu0/{data.csv, sensor.yaml}                    # gyro rad/s then accel m/s^2, T_BS = B_from_IMU

  Zero distortion in ``sensor.yaml`` is the only camera model that NVIDIA's EuRoC loader and our
  reader both get exactly right (the EEPROM's rational model cannot be truncated to Brown), so the
  folder loads unmodified in PyCuVSLAM v17.0.0's ``examples/euroc/track_euroc.py`` and in
  ``methods/cuvslam-vio/08_cuvslam_track.py``.  ``--raw`` records the frames as delivered instead (an A/B
  variant only our reader can track).
* Timestamps: one device-monotonic clock for cameras and IMU (left frame mid-exposure for a
  pair, accelerometer report for an IMU row), plus one constant epoch offset so the values are
  19-digit Unix nanoseconds; the offset is stored in ``dataset.json.time.epoch_offset_ns``.
  Pairs are formed on the device (``dai.node.Sync``); a pair whose left/right stamps differ by
  more than 1 ms, a frame that goes backwards in time and an IMU sample that goes backwards are
  dropped and counted, never written.  The IMU stream starts before the first pair (format doc
  section 4): leading pairs stamped before the first IMU row are skipped and counted in
  ``dataset.json.record.frames_skipped_before_imu`` (usually one pair: the first frame after
  warm-up is stamped mid-exposure, ~30 ms before the IMU packets that survived the warm-up).
* The PNGs are written by a background thread with a bounded queue; when the disk cannot keep up
  the pair is dropped and counted (``drops.stereo_write_queue_dropped``), the stream itself is
  never stalled.
* Once per second prints frames, fps, the left/right device-timestamp difference in
  microseconds, the IMU rate, the PNG queue depth and the drop counters.  On exit prints the
  counts, the dataset path and the exact commands to track the dataset with
  ``methods/cuvslam-vio/08_cuvslam_track.py`` and with NVIDIA's unmodified ``track_euroc.py``.

How to run (from the repository root)
-------------------------------------
    python datasets/07_record_dataset.py --duration 30                    # 1280x800 @ 30 fps, IMU 200 Hz, no window
    python datasets/07_record_dataset.py --seconds 60 --preview --note "slow loop around the lab"
    python datasets/07_record_dataset.py --width 640 --height 400 --duration 120 --headless
    python datasets/07_record_dataset.py --out recordings/euroc_bench01 --imu-rate 400 --duration 20
    python datasets/07_record_dataset.py --device-id OAKDS2EXAMPLEID000 --duration 10
    python datasets/07_record_dataset.py --help

What you should see
-------------------
The device table and summary, the stream/calibration plan (baseline 0.0750 m, IMU extrinsics
available, undistortion on), the dataset folder, a warm-up line (~1.5 s, auto-exposure settles,
those frames are discarded), then one status line per second such as::

    [  12.0 s | 18.0 s left] frames   360  fps 29.9  L/R dt  +12.4 us (max 15.8)  imu 200.1 Hz (2413)  png queue  1/64  drops 0

``L/R dt`` is the right-minus-left device timestamp of the last pair (11-16 us on this unit; both
images get the left stamp).  With --preview a window shows the undistorted left frame with the
same numbers.  On exit: counts, measured rates, drops, warnings, the dataset path and the
tracking commands.  PNG encoding runs on a pool of --png-workers threads (default auto =
min(4, cpu_count - 2)) behind one committer thread that writes the files in frame order; at
1280x800 compression 1 one thread needs ~50 ms per pair (drops at 30 fps), four keep it
drop-free on this laptop.  Watch ``png queue`` and the write-queue drop counter anyway, and use
--png-workers 4+, --png-compression 0, --width 640 --height 400 or a lower --fps if it climbs.

Keys
----
--preview window: q or Esc stops the recording.  Ctrl-C works in every mode and keeps what was
recorded (the writer queue is drained and the csv files are trimmed to complete pairs); a
Ctrl-C while that finishing step runs is held and reported (press it again to abort, which
leaves no dataset.json).  --duration and q are honoured even while no pair arrives.

Exit codes
----------
0 recording complete (warnings, if any, on stderr and in dataset.json); 1 no pairs recorded,
stream fault (first-frame intrinsics guard, watchdog, pipeline died), calibration, unusable
--out or disk error (one line on stderr, no traceback, --out is checked before the camera is
opened); 2 no device / wrong --device-id (one stderr line with --device-id; without it an empty
enumeration is retried 3x over ~6 s with one progress line each, like scripts 00-06, because a
camera rebooting after another script's ``pipeline.stop()`` enumerates as nothing);
130 Ctrl-C before the stream was running, or twice while finishing.

Notes
-----
* ``pipeline.stop()`` closes the device; the camera reboots for a few seconds afterwards, so a
  script started right after this one may have to wait (``common.open_device`` retries).
* The IMU noise densities in ``imu0/sensor.yaml`` are NVIDIA's RealSense placeholders (rig
  preset ``realsense``), not measured on this BNO086; ``dataset.json.imu.noise_source`` says so.
* A calibration without IMU extrinsics still gets an ``imu0/sensor.yaml`` (NVIDIA's loader reads
  it unconditionally) with an identity ``T_BS`` placeholder and ``imu.available = false``: track
  such a dataset in stereo (Multicamera) mode only.
* ``--no-strict-intrinsics`` turns the mandatory first-frame guard (delivered ImgTransformation
  intrinsics vs ``getCameraIntrinsics``) into a warning; use it only if the guard misfires on the
  bench and read ``dataset.json.source.intrinsics_check`` afterwards.
"""

from __future__ import annotations

import argparse
import contextlib
import signal
import sys
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "camera"))      # camera/common.py
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(1, str(REPO_ROOT))

from common import (  # noqa: E402  (import after sys.path tweak on purpose)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    draw_label,
    open_device,
    print_device_summary,
    timestamp_str,
    wait_key_quit,
)
from cuvslam_oak import dataset, oak_source, rig  # noqa: E402

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_INTERRUPTED = 130

SCRIPT_REL = "datasets/07_record_dataset.py"
TRACK_SCRIPT_REL = "methods/cuvslam-vio/08_cuvslam_track.py"
NVIDIA_EXAMPLE_DIR = "~/cuvslam-src/examples/euroc"      # methods/cuvslam-vio/docs/cuvslam-setup.md section 4.3
DEFAULT_OUT_BASE = REPO_ROOT / "recordings"
OUT_PREFIX = "euroc"
WINDOW_NAME = "07_cuvslam_record"
PREVIEW_MAX_WIDTH = 640

DEFAULT_SIZE = oak_source.DEFAULT_SIZE                    # (1280, 800)
SUPPORTED_SIZES = oak_source.SUPPORTED_SIZES              # ((1280, 800), (640, 400), (320, 200))
DEFAULT_FPS = oak_source.DEFAULT_FPS                      # 30.0
DEFAULT_IMU_RATE_HZ = oak_source.DEFAULT_IMU_RATE_HZ      # 200
MAX_IMU_RATE_HZ = oak_source.MAX_IMU_RATE_HZ              # 400 (Luxonis: jitter above)
MAX_LR_SKEW_US = oak_source.MAX_LR_SKEW_US                # 1000 (cuVSLAM: cameras within 1 ms)
DEFAULT_WARMUP_S = 1.5
DEFAULT_QUEUE_SIZE = 64
DEFAULT_PNG_COMPRESSION = 1
DEFAULT_PNG_WORKERS = 0        # 0 = auto: min(4, cpu_count - 2) encoder threads (cuvslam_oak.dataset.resolve_png_workers)
IMU_BATCH = 1
POLL_TIMEOUT_S = 0.5            # next_frame() poll: --duration and q are re-checked at least this often
MAX_WAIT_FOR_IMU_S = 1.0        # skip leading pairs stamped before the first IMU row for at most this long

LEFT_SOCKET = oak_source.LEFT_SOCKET                      # CAM_B = cam0 = body
RIGHT_SOCKET = oak_source.RIGHT_SOCKET                    # CAM_C = cam1
EPOCH_OFFSET_SOURCE = ("time.time_ns() - ns(dai.Clock.now()) + (ns(getTimestamp()) - ns(getTimestampDevice())) "
                       "at the first message received (during warm-up)")
#: Source-side counters that are drops of the same kind as the writer's (the source filters
#: first, the writer only sees what passed, so the two counts add up without double counting).
SOURCE_DROP_KEYS = ("stereo_sequence_gaps", "stereo_sync_violations", "frames_out_of_order",
                    "imu_out_of_order", "imu_incomplete")


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def _one_line(text: Any) -> str:
    return " ".join(str(text).split())


def _show_warning(message: Any, category: Any, filename: str, lineno: int, file: Any = None,
                  line: Any = None) -> None:
    """One ``warning:`` line on stderr instead of the file:line traceback format."""
    print("warning: {}: {}".format(getattr(category, "__name__", "Warning"), _one_line(message)),
          file=sys.stderr, flush=True)


def shell_quote(text: str) -> str:
    """Double-quote a path for the printed shell commands when it needs it."""
    return '"{}"'.format(text) if any(ch in text for ch in " ()&;$'\"") else text


def dataset_ref(root: Path) -> Tuple[str, bool]:
    """POSIX-style path for the printed commands: relative to the repo root when the dataset lives
    inside it (the commands are run from there), else absolute.  Returns ``(text, inside_repo)``."""
    resolved = root.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix(), True
    except ValueError:
        return resolved.as_posix(), False


def tracking_commands(root: Path, imu_available: bool) -> List[str]:
    """The exact commands to track the dataset (a) with methods/cuvslam-vio/08_cuvslam_track.py, (b) with
    NVIDIA's unmodified track_euroc.py (methods/cuvslam-vio/docs/cuvslam-setup.md 4.3, datasets/dataset-format.md 10)."""
    ref, inside = dataset_ref(root)
    q = shell_quote(ref)
    lines = [
        "Track it with our reader (WSL2 / Jetson with cuvslam 17.0.0, run from the repo root; "
        "see {} --help):".format(TRACK_SCRIPT_REL),
        "  python {} {} --mode stereo --out {}".format(TRACK_SCRIPT_REL, q, shell_quote(ref + "/trajectory_stereo.tum")),
    ]
    if imu_available:
        lines.append("  python {} {} --mode vio --out {}".format(
            TRACK_SCRIPT_REL, q, shell_quote(ref + "/trajectory_vio.tum")))
    else:
        lines.append("  (no IMU extrinsics in this calibration: stereo mode only, never vio/Inertial)")
    link_target = '"$PWD/{}"'.format(ref) if inside else shell_quote(ref)
    linux_copy = "~/datasets/{}".format(shell_quote(root.resolve().name))
    lines += [
        "Track it with NVIDIA's unmodified example (PyCuVSLAM v17.0.0 checkout at {}; "
        "it hard-codes dataset/mav0 next to itself and opens a rerun window):".format(NVIDIA_EXAMPLE_DIR),
        "  ln -sfn {} {}/dataset       # Jetson, or any dataset on a Linux disk".format(link_target, NVIDIA_EXAMPLE_DIR),
        "  cp -r {} ~/datasets/ && ln -sfn {} {}/dataset".format(link_target, linux_copy, NVIDIA_EXAMPLE_DIR),
        "      # WSL2 instead: link the copy, reading thousands of PNGs through /mnt/c is slow (methods/cuvslam-vio/docs/cuvslam-setup.md 2.5)",
        "  cd {} && python track_euroc.py       # line 90: OdometryMode.{}".format(
            NVIDIA_EXAMPLE_DIR, "Inertial (default) or Multicamera for stereo-only" if imu_available
            else "Multicamera (this dataset has no IMU extrinsics)"),
        "Plot:  evo_traj tum {} --plot --plot_mode xz".format(shell_quote(ref + "/trajectory_stereo.tum")),
    ]
    return lines


def combined_drops(writer_drops: Dict[str, int], src_stats: Dict[str, Any]) -> Dict[str, int]:
    """Writer drop counters plus the source's own drops of the same kind (see SOURCE_DROP_KEYS)."""
    combined = {key: int(value) for key, value in writer_drops.items()}
    for key in SOURCE_DROP_KEYS:
        combined[key] = combined.get(key, 0) + int(src_stats.get(key, 0) or 0)
    return combined


def drops_text(drops: Dict[str, int]) -> str:
    """``'0'`` or ``'3 (stereo_write_queue_dropped 3)'``."""
    total = sum(int(v) for v in drops.values())
    nonzero = ", ".join("{} {}".format(key, value) for key, value in drops.items() if value)
    return "{}{}".format(total, " ({})".format(nonzero) if nonzero else "")


@contextlib.contextmanager
def deferred_interrupts(describe: Callable[[], str],
                        state: Optional[Dict[str, int]] = None) -> Iterator[Dict[str, int]]:
    """Hold Ctrl-C while a short finishing step runs (device release, PNG queue drain, dataset.json).

    The first Ctrl-C prints ``describe()`` on stderr and is only counted in the yielded dict
    (``state['count']``); a second one raises ``KeyboardInterrupt`` as usual (the user insists).
    Pass the same *state* to consecutive sections so the second press aborts across them.
    Outside the main thread, or without SIGINT support, Ctrl-C keeps its normal behaviour.
    """
    st = state if state is not None else {"count": 0}

    def handler(signum: Any, frame: Any) -> None:
        st["count"] = st.get("count", 0) + 1
        if st["count"] == 1:
            print("\nCtrl-C: {}; press Ctrl-C again to abort".format(describe()), file=sys.stderr, flush=True)
        else:
            raise KeyboardInterrupt

    try:
        previous = signal.signal(signal.SIGINT, handler)
    except (ValueError, OSError, AttributeError):      # not the main thread, or no SIGINT on this host
        yield st
        return
    try:
        yield st
    finally:
        signal.signal(signal.SIGINT, previous if previous is not None else signal.default_int_handler)


def prepare_out_dir(root: Path) -> List[Path]:
    """Create *root* (and missing parents) and prove it is writable, before the camera is opened.

    Returns the directories that were created, deepest first, for :func:`remove_empty_dirs`
    when the recording never starts (a failed device open leaves no empty folder behind).
    Raises ``OSError`` (*root* is an existing file, an unwritable parent, a read-only disk).
    """
    if root.exists() and not root.is_dir():
        raise NotADirectoryError("{} exists and is not a directory".format(root))
    created: List[Path] = []
    for candidate in (root, *root.parents):
        if candidate.exists():
            break
        created.append(candidate)
    root.mkdir(parents=True, exist_ok=True)
    probe = root / ".write_probe_{}".format(int(time.time_ns()))
    try:
        with open(probe, "wb"):
            pass
        probe.unlink()
    except OSError:
        remove_empty_dirs(created)
        raise
    return created


def remove_empty_dirs(paths: Sequence[Path]) -> None:
    """Remove the given directories (deepest first) while they are empty; never raises."""
    for path in paths:
        try:
            path.rmdir()
        except OSError:                                # not empty, already gone, or in use: keep it
            return


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record a cuVSLAM dataset (EuRoC ASL layout: undistorted GRAY8 stereo PNGs + IMU csv) "
                    "from the live OAK-D S2 into recordings/euroc_<timestamp>/ for methods/cuvslam-vio/08_cuvslam_track.py "
                    "and NVIDIA's examples/euroc/track_euroc.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python datasets/07_record_dataset.py --duration 30\n"
            "  python datasets/07_record_dataset.py --seconds 60 --preview --note \"slow loop around the lab\"\n"
            "  python datasets/07_record_dataset.py --width 640 --height 400 --duration 120 --headless\n"
            "  python datasets/07_record_dataset.py --out recordings/euroc_bench01 --imu-rate 400 --duration 20\n"
            "output: <out>/mav0/{{cam0,cam1,imu0}}/... + dataset.json, calibration.json, README.txt\n"
            "sizes: {}; timestamps: device clock + epoch offset (Unix ns); body frame = cam0 = CAM_B.".format(
                ", ".join("{}x{}".format(w, h) for w, h in SUPPORTED_SIZES))
        ),
    )
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument(
        "--seconds", dest="duration", type=float, default=argparse.SUPPRESS, metavar="SECONDS",
        help="alias for --duration: record this many seconds (0 = until q / Esc / Ctrl-C)",
    )
    parser.add_argument("--width", type=int, default=DEFAULT_SIZE[0],
                        help="frame width for both cameras (default {}; 640x400 is the light option)".format(DEFAULT_SIZE[0]))
    parser.add_argument("--height", type=int, default=DEFAULT_SIZE[1],
                        help="frame height for both cameras (default {})".format(DEFAULT_SIZE[1]))
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS,
                        help="stereo frame rate (default {:g}; the Sync threshold is 0.5 / fps)".format(DEFAULT_FPS))
    parser.add_argument("--imu-rate", dest="imu_rate", type=int, default=DEFAULT_IMU_RATE_HZ, metavar="HZ",
                        help="ACCELEROMETER_RAW + GYROSCOPE_RAW report rate (default {}, 400 max recommended)".format(
                            DEFAULT_IMU_RATE_HZ))
    parser.add_argument("--preview", action="store_true",
                        help="open a window with the undistorted left frame, fps and drop counters (q stops); "
                             "off by default, and --headless always wins")
    parser.add_argument("--note", default="", metavar="TEXT",
                        help="free text stored in dataset.json (what was recorded, how the camera moved)")
    parser.add_argument("--out", type=Path, default=None, metavar="DIR",
                        help="dataset folder (default: recordings/{}_<YYYYmmdd_HHMMSS> under the repo root); it is "
                             "created if needed and must not already hold a recording".format(OUT_PREFIX))
    parser.add_argument("--raw", action="store_true",
                        help="record the frames as delivered (distorted): sensor.yaml gets rational-polynomial with "
                             "the 8 OpenCV coefficients; only methods/cuvslam-vio/08_cuvslam_track.py can track such a dataset "
                             "(NVIDIA's track_euroc.py would build a wrong camera model)")
    parser.add_argument("--warmup", type=float, default=DEFAULT_WARMUP_S, metavar="SECONDS",
                        help="discard frames for this long after start so auto-exposure settles (default {:g})".format(
                            DEFAULT_WARMUP_S))
    parser.add_argument("--queue-size", dest="queue_size", type=int, default=DEFAULT_QUEUE_SIZE, metavar="PAIRS",
                        help="PNG writer input queue depth in stereo pairs; a full queue drops the pair.  It only "
                             "absorbs bursts (2 s at 30 fps, 2 MB per pair at 1280x800): with the default encoder "
                             "threads the queue stays at 0-1 pairs, and no queue size fixes a pipeline slower than "
                             "the frame rate (default {})".format(DEFAULT_QUEUE_SIZE))
    parser.add_argument("--png-compression", dest="png_compression", type=int, default=DEFAULT_PNG_COMPRESSION,
                        metavar="0-9", help="PNG compression level, 0 = fastest/largest (default {})".format(
                            DEFAULT_PNG_COMPRESSION))
    parser.add_argument("--png-workers", dest="png_workers", type=int, default=DEFAULT_PNG_WORKERS, metavar="N",
                        help="PNG encoder threads (cv2.imencode runs without the GIL, so they scale); one extra "
                             "committer thread writes the files and csv rows in frame order.  0 = auto = "
                             "min(4, cpu_count - 2), which keeps 1280x800 @ 30 fps drop-free on a laptop; 1 = the "
                             "old single-thread writer (~50 ms/pair at 1280x800, drops at 30 fps) (default {})".format(
                                 DEFAULT_PNG_WORKERS))
    parser.add_argument("--no-strict-intrinsics", dest="strict_intrinsics", action="store_false",
                        help="warn instead of aborting when the first frame's ImgTransformation intrinsics differ "
                             "from getCameraIntrinsics (bench diagnostics only)")
    args = parser.parse_args(argv)

    size = (args.width, args.height)
    if size not in SUPPORTED_SIZES:
        parser.error("--width/--height must be one of {} (16:10 sizes whose intrinsics are an exact scaling of the "
                     "calibration), got {}x{}".format(
                         ", ".join("{}x{}".format(w, h) for w, h in SUPPORTED_SIZES), args.width, args.height))
    if not (1.0 <= args.fps <= 120.0):
        parser.error("--fps must be within 1..120, got {}".format(args.fps))
    if not (1 <= args.imu_rate <= 1000):
        parser.error("--imu-rate must be within 1..1000 Hz, got {}".format(args.imu_rate))
    if args.duration < 0:
        parser.error("--duration must be >= 0")
    if args.warmup < 0:
        parser.error("--warmup must be >= 0")
    if args.queue_size < 1:
        parser.error("--queue-size must be >= 1")
    if not (0 <= args.png_compression <= 9):
        parser.error("--png-compression must be within 0..9")
    if args.png_workers < 0:
        parser.error("--png-workers must be >= 0 (0 = auto)")
    return args


# --------------------------------------------------------------------------------------
# Calibration -> RigInfo (numbers from cuvslam_oak.rig, file layout from cuvslam_oak.dataset)
# --------------------------------------------------------------------------------------

def build_rig_info(calib: Dict[str, Any], size: Tuple[int, int], fps: float, imu_rate_hz: int, *,
                   undistorted: bool, device_id: Optional[str],
                   sensor_names: Optional[Dict[str, str]] = None) -> Tuple[dataset.RigInfo, Dict[str, Any]]:
    """The :class:`dataset.RigInfo` for one recording from the ``eepromToJson()`` dict.

    ``T_BS`` = body_from_sensor with body = cam0 = CAM_B: identity for cam0,
    ``getCameraExtrinsics(CAM_C, CAM_B)`` (cm -> m) for cam1, ``getImuToCameraExtrinsics(CAM_B)``
    for imu0 (RAW streams keep the EEPROM rotation).  Intrinsics are ``getCameraIntrinsics(socket,
    w, h)``; undistorted datasets carry four zero coefficients, raw ones the 8 OpenCV coefficients.
    The IMU noise is the RealSense placeholder set (``rig.EUROC_NOISE_PRESET``).  A calibration
    without IMU extrinsics gets an identity ``T_BS`` placeholder (``facts['imu_available']`` False).
    Returns ``(rig_info, facts)``; raises :class:`rig.CalibrationError` / :class:`dataset.DatasetError`.
    """
    w, h = int(size[0]), int(size[1])
    names = sensor_names or {}
    imu_available = rig.has_imu_extrinsics(calib)
    noise = rig.imu_noise(calib, frequency_hz=float(imu_rate_hz), preset=rig.EUROC_NOISE_PRESET)
    sensors = rig.to_euroc_sensor_yaml(
        calib, (w, h), imu_rate_hz=float(imu_rate_hz), undistorted=undistorted, noise=noise,
        body_socket=LEFT_SOCKET, left=LEFT_SOCKET, right=RIGHT_SOCKET, fps=float(fps), imu_frame="raw",
        device_id=device_id, allow_missing_imu=not imu_available)
    borders = rig.default_borders((w, h), undistorted=undistorted)
    border_tuple = (borders["top"], borders["bottom"], borders["left"], borders["right"])

    def camera(sensor: Dict[str, Any], socket: str, role: str) -> dataset.CameraInfo:
        fu, fv, cu, cv_ = (float(v) for v in sensor["intrinsics"])
        return dataset.CameraInfo(
            width=w, height=h, fu=fu, fv=fv, cu=cu, cv=cv_, T_BS=sensor["T_BS"]["data"],
            distortion_model=str(sensor["distortion_model"]),
            distortion_coefficients=[float(c) for c in sensor["distortion_coefficients"]],
            rate_hz=float(fps), comment=str(sensor["comment"]), socket=socket, role=role,
            sensor=names.get(socket), image_type="GRAY8", undistorted=undistorted, borders=border_tuple)

    imu0 = sensors["imu0"]
    eeprom_noise: Optional[Dict[str, Any]] = None
    if noise.get("eeprom"):
        eeprom_noise = dict(noise["eeprom"])
        eeprom_noise["units"] = "undocumented, copied verbatim from calibration.json imuCalibrationParams.noise; not used"
    imu = dataset.ImuInfo(
        T_BS=imu0["T_BS"]["data"], rate_hz=float(imu_rate_hz),
        gyroscope_noise_density=float(imu0["gyroscope_noise_density"]),
        gyroscope_random_walk=float(imu0["gyroscope_random_walk"]),
        accelerometer_noise_density=float(imu0["accelerometer_noise_density"]),
        accelerometer_random_walk=float(imu0["accelerometer_random_walk"]),
        comment=str(imu0["comment"]), noise_source=str(noise["source"]),
        sensors=list(oak_source.IMU_SENSORS), timestamp_source="acceleroMeter.tsDevice", eeprom_noise=eeprom_noise)
    rig_info = dataset.RigInfo(cam0=camera(sensors["cam0"], LEFT_SOCKET, "left"),
                               cam1=camera(sensors["cam1"], RIGHT_SOCKET, "right"),
                               imu=imu, device_id=device_id or "")
    facts = {
        "imu_available": bool(imu_available),
        "baseline_m": rig_info.baseline_m(),
        "noise_preset": noise.get("preset"),
        "noise_source": noise.get("source"),
        "imu_name": noise.get("imu_name"),
    }
    return rig_info, facts


def undistort_maps(calib: Dict[str, Any], size: Tuple[int, int]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """``{'left': (map1, map2), 'right': (map1, map2)}`` from ``rig.undistort_maps`` (14 coefficients,
    newCameraMatrix = K, so the yaml intrinsics stay the EEPROM values)."""
    maps: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for socket, role in ((LEFT_SOCKET, "left"), (RIGHT_SOCKET, "right")):
        map1, map2 = rig.undistort_maps(calib, socket, size)
        if tuple(map1.shape[:2]) != (int(size[1]), int(size[0])):
            raise rig.CalibrationError("undistortion map for {} is {}x{}, expected {}x{}".format(
                socket, map1.shape[1], map1.shape[0], size[0], size[1]))
        maps[role] = (map1, map2)
    return maps


def initial_metadata(args: argparse.Namespace, info: Dict[str, Any], calib: Dict[str, Any], src: Any,
                     facts: Dict[str, Any], preview: bool) -> Dict[str, Any]:
    """The ``dataset.json`` fields the writer cannot know (deep-merged over its defaults)."""
    return {
        "generator": {"script": SCRIPT_REL, "depthai_version": getattr(dai, "__version__", None)},
        "device": {
            "device_id": info.get("device_id"), "name": info.get("name"), "product": info.get("product"),
            "platform": info.get("platform"), "usb_speed": info.get("usb_speed"),
            "board_name": calib.get("boardName"), "board_rev": calib.get("boardRev"),
            "imu": info.get("imu"), "imu_firmware": info.get("imu_firmware"),
            "eeprom_version": calib.get("version"), "eeprom_batch_time": calib.get("batchTime"),
        },
        "record": {
            "requested_duration_s": float(args.duration), "headless": not preview, "preview": preview,
            "warmup_seconds": float(args.warmup), "png_compression": int(args.png_compression),
            "writer_queue_size": int(args.queue_size),
            "png_workers": dataset.resolve_png_workers(args.png_workers),      # effective encoder thread count
            "png_workers_requested": int(args.png_workers),                      # the flag as given (0 = auto)
            "strict_intrinsics": bool(args.strict_intrinsics),
            "command_line": " ".join(sys.argv[1:]),
        },
        "note": args.note or None,
        "time": {"epoch_offset_source": EPOCH_OFFSET_SOURCE},
        "stereo": {
            "pairing": "dai.node.Sync, threshold_s {:.4f}, timestamp of the pair = left frame device timestamp "
                       "(CameraExposureOffset.MIDDLE)".format(src.sync_threshold_s),
            "max_left_right_skew_ms_allowed": MAX_LR_SKEW_US / 1e3,
        },
        "imu": {"batch_report_threshold": IMU_BATCH, "available": facts["imu_available"],
                "noise_preset": facts.get("noise_preset")},
        "calibration": {
            "source": "device.readCalibration() (EEPROM)",
            "T_BS_cam1_from": "getCameraExtrinsics(CAM_C, CAM_B)",
            "T_BS_imu0_from": ("getImuToCameraExtrinsics(CAM_B)" if facts["imu_available"]
                               else "identity placeholder (no IMU extrinsics in the EEPROM)"),
        },
    }


# --------------------------------------------------------------------------------------
# The recording loop
# --------------------------------------------------------------------------------------

@dataclass
class Outcome:
    """How the loop ended and what it did (the writer and the source hold the detailed counts)."""

    stopped_by: str = "unknown"      # duration | user | keyboard_interrupt | source | error
    reason: str = ""                 # the source's stop reason when stopped_by == 'source'
    error: Optional[str] = None      # one line when stopped_by == 'error'
    frames_seen: int = 0             # pairs the source delivered to us
    frames_accepted: int = 0         # pairs the writer queued
    frames_skipped_before_imu: int = 0   # leading pairs stamped before the first IMU row (not written)
    imu_seen: int = 0
    wall_s: float = 0.0


def undistort_pair(frame: Any, maps: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]]) -> Tuple[np.ndarray, np.ndarray]:
    if maps is None:
        return frame.left, frame.right
    left = cv2.remap(frame.left, maps["left"][0], maps["left"][1], cv2.INTER_LINEAR)
    right = cv2.remap(frame.right, maps["right"][0], maps["right"][1], cv2.INTER_LINEAR)
    return left, right


def preview_image(left: np.ndarray, text: str) -> np.ndarray:
    """BGR copy of the left frame, downscaled to PREVIEW_MAX_WIDTH, with the overlay text."""
    h, w = left.shape[:2]
    if w > PREVIEW_MAX_WIDTH:
        left = cv2.resize(left, (PREVIEW_MAX_WIDTH, int(round(h * PREVIEW_MAX_WIDTH / w))), interpolation=cv2.INTER_AREA)
    bgr = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
    return draw_label(bgr, text, org=(10, 22), scale=0.5)


def record(src: Any, writer: dataset.EurocWriter, maps: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]],
           clock: RunClock, *, preview: bool = False, status_interval_s: float = 1.0,
           poll_timeout_s: float = POLL_TIMEOUT_S, max_wait_for_imu_s: float = MAX_WAIT_FOR_IMU_S,
           interrupt_state: Optional[Dict[str, int]] = None) -> Outcome:
    """Stream until the clock expires, q is pressed, Ctrl-C, or the source ends; return the :class:`Outcome`.

    Uses only the source's public surface (``next_frame()``, ``imu_samples()``, ``epoch_ns()``,
    ``stop()``, ``stats()``), so it also runs on a stub source in tests.  ``next_frame`` is
    polled with *poll_timeout_s* so --duration, q and the status line are honoured while no
    pair arrives.  Every IMU sample the source has is written before the frame it precedes, and
    the first written pair is one with an IMU row at or before it (leading pairs stamped before
    the first IMU row are skipped for at most *max_wait_for_imu_s*, counted in
    ``frames_skipped_before_imu``); after the loop the source is stopped and the IMU tail it
    drained is written, so the IMU stream ends after the last pair.  That stop runs under
    :func:`deferred_interrupts` (*interrupt_state* shared with the caller's own finishing
    step).  The loop never raises: faults are recorded in the outcome.
    """
    out = Outcome()
    frame_fps = FpsCounter(2.0)
    imu_fps = FpsCounter(2.0)
    status = Periodic(status_interval_s)
    last_lr_us = 0.0
    imu_invalid_warned = False
    first_imu_ts: Optional[int] = None        # device clock of the first IMU row actually written
    imu_late_warned = False
    window_shown = False
    t_start = time.monotonic()

    def write_imu(samples: Sequence[Any]) -> None:
        nonlocal imu_invalid_warned, first_imu_ts
        for s in samples:
            gyro_ts = src.epoch_ns(s.gyro_ts_ns) if getattr(s, "gyro_ts_ns", 0) else None
            try:
                written = writer.add_imu(src.epoch_ns(s.ts_ns), s.gyro, s.accel, gyro_ts_ns=gyro_ts)
            except dataset.DatasetError as exc:
                writer.count_drop("imu_invalid")
                if not imu_invalid_warned:
                    imu_invalid_warned = True
                    writer.add_warning("IMU sample rejected: {}".format(exc))
                    print("warning: IMU sample rejected ({}); further ones are only counted".format(_one_line(exc)),
                          file=sys.stderr, flush=True)
                continue
            if written and first_imu_ts is None:
                first_imu_ts = int(s.ts_ns)
        if samples:
            imu_fps.tick(len(samples))
            out.imu_seen += len(samples)

    def imu_precedes(frame: Any) -> bool:
        """Whether an IMU row at or before this pair has been written (format doc section 4).

        Until the first pair is accepted, a pair stamped before the first IMU row is skipped;
        after *max_wait_for_imu_s* without such a row the pair is accepted with one warning.
        """
        nonlocal imu_late_warned
        if out.frames_accepted or (first_imu_ts is not None and first_imu_ts <= int(frame.ts_ns)):
            return True
        if time.monotonic() - t_start < max_wait_for_imu_s:
            return False
        if not imu_late_warned:
            imu_late_warned = True
            text = ("no IMU sample at or before the first pair within {:g} s ({}): the IMU stream starts after "
                    "the first pair".format(max_wait_for_imu_s, "no IMU sample written yet" if first_imu_ts is None
                                            else "first IMU row {:.1f} ms after the first pair".format(
                                                (first_imu_ts - int(frame.ts_ns)) / 1e6)))
            writer.add_warning(text)
            print("warning: {}".format(text), file=sys.stderr, flush=True)
        return True

    def status_text(compact: bool) -> str:
        st = writer.stats()
        ss = src.stats()
        drops = drops_text(combined_drops(st["drops"], ss))
        elapsed = time.monotonic() - t_start
        remaining = clock.remaining()
        if compact:   # preview overlay
            return ("cam0 {}  {:.1f} fps  frames {}\nL/R dt {:+.0f} us  imu {:.0f} Hz  png queue {}/{}  drops {}\n"
                    "{:.1f} s{}   q = stop".format(
                        LEFT_SOCKET, frame_fps.fps(), st["frames_accepted"], last_lr_us, imu_fps.fps(),
                        st["queue_depth"], st["queue_size"], drops, elapsed,
                        "" if remaining is None else " ({:.0f} s left)".format(remaining)))
        return ("[{:6.1f} s{}] frames {:5d}  fps {:4.1f}  L/R dt {:+6.1f} us (max {:.1f})  imu {:5.1f} Hz ({})  "
                "png queue {:2d}/{}  drops {}".format(
                    elapsed, "" if remaining is None else " | {:5.1f} s left".format(remaining),
                    st["frames_accepted"], frame_fps.fps(), last_lr_us, float(ss.get("max_lr_skew_us") or 0.0),
                    imu_fps.fps(), st["imu_written"], st["queue_depth"], st["queue_size"], drops))

    try:
        while True:
            try:
                frame = src.next_frame(poll_timeout_s)   # None: no pair within the poll (checked below)
            except oak_source.OakSourceStopped:
                break                                    # ended on its own: stopped_by 'source' below
            write_imu(src.imu_samples())                 # every pending sample, also while no pair arrives
            if frame is not None:
                out.frames_seen += 1
                last_lr_us = float(frame.lr_ts_diff_us)
                if not imu_precedes(frame):
                    out.frames_skipped_before_imu += 1
                else:
                    left, right = undistort_pair(frame, maps)
                    if writer.add_frame(src.epoch_ns(frame.ts_ns), left, right,
                                        right_ts_ns=src.epoch_ns(frame.right_ts_ns)):
                        out.frames_accepted += 1
                    frame_fps.tick()
                    if preview:
                        cv2.imshow(WINDOW_NAME, preview_image(left, status_text(compact=True)))
                        window_shown = True
            if window_shown and wait_key_quit(1) == "q":  # pumped every poll, so q works during a stall too
                out.stopped_by = "user"
                break
            if clock.expired():
                out.stopped_by = "duration"
                break
            if status.ready():
                print(status_text(compact=False), flush=True)
    except KeyboardInterrupt:
        out.stopped_by = "keyboard_interrupt"
        print("\nCtrl-C: stopping, keeping what was recorded", flush=True)
    except oak_source.OakStreamError as exc:
        out.stopped_by = "error"
        out.error = "oak source: {}".format(_one_line(exc))
    except dataset.DatasetError as exc:
        out.stopped_by = "error"
        out.error = "dataset writer: {}".format(_one_line(exc))
    except (RuntimeError, cv2.error) as exc:          # writer closed under us, remap/imshow failure, ...
        out.stopped_by = "error"
        out.error = "{}: {}".format(type(exc).__name__, _one_line(exc))

    # Releasing the device and writing the IMU tail take up to a few seconds: a Ctrl-C here is
    # held (and reported) instead of leaving the camera booted and the tail unwritten.
    with deferred_interrupts(lambda: "releasing the camera and writing the IMU tail", interrupt_state):
        src.stop()                                    # releases the device; drains the IMU tail
        write_imu(src.imu_samples())                  # samples after the last pair, written last
    if out.stopped_by == "unknown":                   # next_frame() raised OakSourceStopped
        out.stopped_by = "source"
        out.reason = str(src.stats().get("stop_reason") or "the pipeline ended")
    out.wall_s = time.monotonic() - t_start
    return out


# --------------------------------------------------------------------------------------
# Summary
# --------------------------------------------------------------------------------------

def print_summary(root: Path, summary: Dict[str, Any], outcome: Outcome, src_stats: Dict[str, Any],
                  imu_available: bool) -> List[str]:
    """Print counts, path and commands; return the list of problems (non-empty = exit 1)."""
    counts = summary.get("counts") or {}
    measured = summary.get("measured") or {}
    drops = summary.get("drops") or {}
    record_info = summary.get("record") or {}
    pairs = int(counts.get("stereo_pairs") or 0)
    problems: List[str] = []
    if outcome.error:
        problems.append(outcome.error)
    if outcome.stopped_by == "source":
        problems.append("the stream ended by itself: {}".format(outcome.reason))
    if pairs == 0:
        problems.append("no stereo pairs were recorded")

    stopped = {"duration": "--duration reached", "user": "q pressed", "keyboard_interrupt": "Ctrl-C",
               "source": "stream ended", "error": "error"}.get(outcome.stopped_by, outcome.stopped_by)
    print("\nRecording {} ({}): {} stereo pairs ({} accepted, {} written, {} failed), {} IMU samples, "
          "{} s of data in {:.1f} s".format(
              "incomplete" if problems else "complete", stopped, pairs, counts.get("frames_accepted", 0),
              counts.get("frames_written", 0), counts.get("frames_failed", 0), counts.get("imu_samples", 0),
              record_info.get("duration_s"), outcome.wall_s), flush=True)
    print("Measured: stereo {} fps, imu {} Hz, max stereo gap {} ms, max imu gap {} ms, max L/R skew {} us, "
          "accel/gyro skew {} ms, PNG {} ms/pair commit ({} ms encode, {} pairs/s, {} encoder thread(s)), "
          "{:.1f} MB".format(
              measured.get("stereo_fps"), measured.get("imu_hz"), measured.get("max_stereo_gap_ms"),
              measured.get("max_imu_gap_ms"), measured.get("max_left_right_skew_us"),
              measured.get("imu_max_accel_gyro_skew_ms"), measured.get("png_write_ms_mean"),
              measured.get("png_encode_ms_mean"), measured.get("png_throughput_pairs_per_s"),
              record_info.get("png_workers"), float(measured.get("png_bytes_total") or 0) / 1e6), flush=True)
    print("Drops: {}  (warm-up discarded {} pairs, {} IMU packets; {} leading pair(s) skipped so the IMU stream "
          "starts first)".format(drops_text(drops), src_stats.get("warmup_frames_discarded", 0),
                                 src_stats.get("warmup_imu_discarded", 0), outcome.frames_skipped_before_imu),
          flush=True)
    if drops.get("stereo_write_queue_dropped"):
        print("  the PNG pipeline could not keep up ({} pairs dropped with {} encoder thread(s)): try "
              "--png-workers 4 (or more), --png-compression 0, --width 640 --height 400 or a lower --fps".format(
                  drops["stereo_write_queue_dropped"], record_info.get("png_workers")), flush=True)
    for role in ("left", "right"):
        check = (src_stats.get("intrinsics_check") or {}).get(role) or {}
        if check.get("status") not in (None, "ok"):
            print("  first-frame intrinsics check {}: {} {}".format(
                role, check.get("status"), _one_line(check.get("error") or "")), flush=True)
    print("Dataset: {}".format(root.resolve()), flush=True)
    print("  mav0/cam0/data ({} PNG), mav0/cam1/data ({} PNG), mav0/imu0/data.csv ({} rows), dataset.json, "
          "README.txt, calibration.json".format(pairs, pairs, counts.get("imu_samples", 0)), flush=True)

    warnings_list = [str(w) for w in (summary.get("warnings") or [])]
    if warnings_list:
        print("\nWARNINGS (recorded in dataset.json):", file=sys.stderr, flush=True)
        for text in warnings_list:
            print("  - {}".format(text), file=sys.stderr, flush=True)

    print("", flush=True)
    for line in tracking_commands(root, imu_available):
        print(line, flush=True)
    return problems


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    warnings.showwarning = _show_warning
    preview = bool(args.preview) and not args.headless
    if args.preview and args.headless:
        print("--headless given: --preview ignored (no window, Ctrl-C or --duration stops)", flush=True)
    size = (int(args.width), int(args.height))
    root = Path(args.out) if args.out is not None else DEFAULT_OUT_BASE / "{}_{}".format(OUT_PREFIX, timestamp_str())
    if (root / "mav0" / "cam0" / "data.csv").exists() or (root / "mav0" / "imu0" / "data.csv").exists():
        print("{} already holds a recording (mav0/*/data.csv exists); choose another --out".format(root),
              file=sys.stderr, flush=True)
        return EXIT_ERROR
    try:                                               # an unusable --out must not cost a camera boot
        created_dirs = prepare_out_dir(root)
    except OSError as exc:
        print("cannot use --out {}: {}: {}".format(root, type(exc).__name__, _one_line(exc)), file=sys.stderr, flush=True)
        return EXIT_ERROR

    # An explicit --device-id never retries: one stderr line even when nothing enumerates.  Without
    # it an empty enumeration is retried 3x (a camera rebooting after pipeline.stop() enumerates as
    # nothing), the convention of scripts 00-06.
    try:
        device = open_device(args.device_id, retries=1 if args.device_id else 3)     # exits 2 on failure
    except SystemExit:
        remove_empty_dirs(created_dirs)                # no empty recordings/euroc_<stamp> left behind
        raise
    print_device_summary(device)
    info = oak_source.device_info(device)

    src: Optional[oak_source.OakStereoImuSource] = None
    writer: Optional[dataset.EurocWriter] = None
    try:
        src = oak_source.OakStereoImuSource(
            device, size[0], size[1], float(args.fps), int(args.imu_rate), IMU_BATCH, True,
            max_lr_skew_us=MAX_LR_SKEW_US, strict_intrinsics=bool(args.strict_intrinsics))
        src.build()                                            # pipeline + cached CalibrationHandler, not started
        calib = src.calibration()                              # eepromToJson() dict -> rig.py, calibration.json
        rig_info, facts = build_rig_info(calib, size, float(args.fps), int(args.imu_rate),
                                         undistorted=not args.raw, device_id=info.get("device_id"),
                                         sensor_names=info.get("camera_sensors") or {})
        maps = None if args.raw else undistort_maps(calib, size)

        limit = ("{:g} s".format(args.duration) if args.duration > 0
                 else ("until Ctrl-C" if not preview else "until q / Esc / Ctrl-C"))
        print("Streams: cam0 = {} left, cam1 = {} right: {}x{} GRAY8 @ {:g} fps, {}; imu0 = {} {} @ {} Hz".format(
            LEFT_SOCKET, RIGHT_SOCKET, size[0], size[1], args.fps,
            "RAW frames (distorted, rational-polynomial in sensor.yaml)" if args.raw
            else "host-undistorted (14 EEPROM coefficients, intrinsics unchanged)",
            info.get("imu") or "IMU", " + ".join(oak_source.IMU_SENSORS), args.imu_rate), flush=True)
        print("Calibration: baseline {:.6f} m, IMU extrinsics {}, body = cam0 = {}, IMU noise = {} placeholder ({})".format(
            facts["baseline_m"], "available (T_BS = getImuToCameraExtrinsics(CAM_B))" if facts["imu_available"]
            else "MISSING -> identity T_BS placeholder, stereo-only dataset",
            LEFT_SOCKET, facts.get("noise_preset"), facts.get("noise_source")), flush=True)
        print("Duration: {}   Dataset: {}".format(limit, root.resolve()), flush=True)

        # (--raw: the writer itself records the NVIDIA-loader note per camera in dataset.json.warnings)
        writer = dataset.EurocWriter(
            root, rig_info, cam_rate_hz=float(args.fps), imu_rate_hz=float(args.imu_rate),
            queue_size=int(args.queue_size), png_compression=int(args.png_compression),
            png_workers=int(args.png_workers),
            max_stereo_skew_ns=int(MAX_LR_SKEW_US * 1000), calibration=calib,
            metadata=initial_metadata(args, info, calib, src, facts, preview))

        src.start()
        print("Warm-up {:g} s (auto-exposure settles; frames discarded)...".format(args.warmup), flush=True)
        discarded = src.warm_up(float(args.warmup))
        print("Warm-up done: {} pairs discarded, epoch offset {}; recording...".format(
            discarded, "{} ns".format(src.epoch_offset_ns) if src.epoch_offset_ns is not None
            else "from the first message"), flush=True)
    except KeyboardInterrupt:
        if src is not None:
            src.stop()
        else:
            device.close()
        if writer is not None:
            writer.close(stopped_by="keyboard_interrupt")
        else:
            remove_empty_dirs(created_dirs)
        print("\nInterrupted before recording started.", file=sys.stderr, flush=True)
        return EXIT_INTERRUPTED
    except (RuntimeError, ValueError, NotImplementedError, OSError, cv2.error) as exc:
        # OakSourceError is a RuntimeError, CalibrationError / DatasetError are ValueErrors; a raw
        # depthai RuntimeError (no readable calibration, XLink fault) lands here too: one line, no traceback.
        if src is not None:
            src.stop()                                         # releases the device after a build()
        else:
            device.close()
        if writer is not None:
            writer.close(stopped_by="error", metadata={"record": {"stop_reason": _one_line(exc)}})
        else:
            remove_empty_dirs(created_dirs)
        print("{}: {}".format(type(exc).__name__, _one_line(exc)), file=sys.stderr, flush=True)
        return EXIT_ERROR

    clock = RunClock(args.duration)
    interrupts: Dict[str, int] = {"count": 0}                 # shared by both finishing steps
    try:
        outcome = record(src, writer, maps, clock, preview=preview, interrupt_state=interrupts)
    finally:
        if preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error:                                  # GUI-less OpenCV build
                pass

    src_stats = src.stats()
    for key in SOURCE_DROP_KEYS:                               # source-side drops join the writer's counters
        n = int(src_stats.get(key, 0) or 0)
        if n:
            writer.count_drop(key, n)
    if src.epoch_offset_ns is None:                            # seeded by the first message, so only when none came
        writer.add_warning("epoch offset never determined (no message carried host and device stamps): "
                           "timestamps are raw device-clock nanoseconds")
    final_meta: Dict[str, Any] = {
        "record": {
            "warmup_frames_discarded": src_stats.get("warmup_frames_discarded"),
            "warmup_imu_discarded": src_stats.get("warmup_imu_discarded"),
            "frames_skipped_before_imu": outcome.frames_skipped_before_imu,
            "wall_duration_s": round(outcome.wall_s, 3),
            "stop_reason": outcome.error or outcome.reason or None,
        },
        "time": {"epoch_offset_ns": src.epoch_offset_ns},
        "source": src_stats,
    }
    # The PNG queue drain (up to --queue-size pairs) and dataset.json: a Ctrl-C here is held, a
    # second one aborts (dataset.json is then missing and the csv files are untrimmed).
    try:
        with deferred_interrupts(lambda: "finishing the PNG queue ({} pair(s) left) and dataset.json".format(
                writer.queue_depth), interrupts):
            summary = writer.close(stopped_by=outcome.stopped_by, metadata=final_meta)
    except KeyboardInterrupt:
        print("\nAborted while finishing: {} has no dataset.json and untrimmed csv files "
              "(cuvslam_oak.dataset.trim_dataset can repair the csv files).".format(root.resolve()),
              file=sys.stderr, flush=True)
        return EXIT_INTERRUPTED
    except OSError as exc:
        print("could not write dataset.json / README.txt: {}".format(_one_line(exc)), file=sys.stderr, flush=True)
        return EXIT_ERROR
    problems = print_summary(root, summary, outcome, src_stats, facts["imu_available"])
    if writer.writer_error:
        problems.append("png writer stopped after a data.csv error: {}".format(writer.writer_error))
    if problems:
        print("\nRECORDING INCOMPLETE:", file=sys.stderr, flush=True)
        for problem in problems:
            print("  - {}".format(problem), file=sys.stderr, flush=True)
        return EXIT_ERROR
    return EXIT_OK


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:       # Ctrl-C while finishing up: no traceback
        print("\nInterrupted.", file=sys.stderr, flush=True)
        sys.exit(EXIT_INTERRUPTED)
