#!/usr/bin/env python3
"""Record an OAK-D S2 session (RGB + stereo depth + IMU) into a timestamped folder for offline work.

What this test checks
---------------------
* The camera can stream the colour camera (CAM_A), the stereo pair (CAM_B/CAM_C) and the
  BNO086 IMU at the same time while everything is written to disk (--mode video; the holistic
  default records CAM_B + CAM_C + IMU, see below).
* The files DepthAI writes are complete. Holistic recording on RVC2 has two *silent* failure
  modes - header-only 436-byte mp4s, and mp4s holding only a fraction of the session (see
  "Why holistic recording lost frames" below) - so at the end every mp4 is inspected (frames,
  duration and size read from its stsz / mvhd / tkhd boxes, stdlib only) and compared with what
  the host saw, the measured per-stream frame rates are printed and stored in session.json, and
  the exit code tells you whether camera/06_replay.py will be able to read the session.

Two recording modes
-------------------
--mode holistic (default)
    dai.RecordConfig + pipeline.enableHolisticRecord(): DepthAI records every *source* node
    itself (each Camera node and the IMU) and, when the pipeline object is destroyed, packs
    everything into ONE file, recording_YYYY-MM-DD_HH-MM-SS.tar, containing
    CameraCAM_x.mp4 + CameraCAM_x.mcap per camera (H264 when encoding is on, raw .avi with
    --no-encode), IMU.mcap, camera_info.json (full calibration) and record_config.json.
    Depth is NOT stored: on replay the same pipeline (Camera nodes + StereoDepth) is fed from
    the tar and depth is recomputed on the device, so holistic replay needs a connected OAK.
    Default streams (--cameras stereo): CAM_B + CAM_C, recorded at 640x400, + IMU - a depth-only
    session whose depth is recomputed on replay. --cameras rgb records CAM_A + IMU only (CAM_A is
    stored at 1344x1008 for the default --width/--height 1280x720; 148 of 148 frames verified).
    --cameras all = three encoded streams, which STILL fails on this RVC2 device (verified today:
    the three encoders throttle every camera to ~17 fps and only 17 frames per camera reach the
    mp4s with sync on, exit 1); it only works raw, --cameras all --no-encode (148 of 148 frames
    per camera, ~26 MB of .avi per 5 s). RGB + depth + IMU with the depth itself stored is what
    --mode video is for.

--mode video
    One dai.node.RecordVideo per stream, which always works on this device:
      rgb.mp4 + rgb.mcap        H264 (VideoEncoder H264_MAIN) at --width x --height, --fps
      depth/depth_NNNNNN.png    one 16-bit PNG per frame, uint16 millimetres, 0 = invalid,
      depth_index.csv           named by sequence number (--depth-stream depth, default)
        or
      disparity.mkv + .mcap     8-bit gray = disparity * 255 / max_disparity, written by
                                RecordVideo (lossless FFV1) via a small host node
                                (--depth-stream disparity; replayable with dai.node.ReplayVideo)
      imu.mcap                  RecordMetadataOnly (accelerometer + gyroscope RAW, 200 Hz)
      calibration.json          full EEPROM calibration dump (CalibrationHandler.eepromToJson)
    Why PNGs for depth: RecordVideo's raw path converts every frame to 8-bit BGR and writes
    FFV1 through cv::VideoWriter (YUV420), see depthai-core src/opencv/RecordReplay.cpp and
    src/pipeline/node/host/Record.cpp - uint16 depth would be garbled, so exact depth is
    stored as PNGs instead. Read one back with
        cv2.imread("depth/depth_000123.png", cv2.IMREAD_UNCHANGED)   # (H, W) uint16 mm
    Depth lives in the rectified right (CAM_C) frame at 640x400 (divided by --decimation)
    unless --align-rgb is given: then the RGB stream is requested undistorted at the stereo
    size (640x400, the only size-matched inputAlignTo configuration verified on this device),
    rgb.mp4 is recorded at that size and depth is aligned to it pixel for pixel
    (--width/--height must then be left at the default or be 640x400).
    StereoDepth throughput on RVC2: at 640x400 with --decimation 1 the DEFAULT preset's spatial
    + temporal post-filters hold StereoDepth at ~17 fps (only every other frame is produced), so
    by default the chosen --preset runs WITHOUT those two filters: measured today 29-30 fps with
    no sequence gaps for DEFAULT (7x7 median + speckle + subpixel kept), FAST_ACCURACY and
    FAST_DENSITY. --preset-filters keeps them; --decimation 2 (320x200 depth) is the other way
    to reach 30 fps with the full filter set. The measured depth rate is printed and a warning
    is stored in session.json when it falls below 70 % of --fps.

Why holistic recording lost frames (fixed here; verified today on this OAK-D S2, depthai 3.10.0)
----------------------------------------------------------------------------------------------
1. Record size. On RVC2 the holistic recorder stores each camera at the smallest sensor mode
   STRICTLY larger than the largest output requested from it (depthai-core
   src/opencv/HolisticRecordReplay.cpp, getCameraRecordSize) through its own ImageManip +
   VideoEncoder. A 640x400 request for CAM_B/CAM_C therefore recorded them at 1280x720, which
   throttled the cameras themselves to ~14 fps (69 of 139 frames never left the device) and,
   with syncCameraOutputs on, produced header-only mp4s in every run. This script requests the
   mono preview at 320x200, so the recorder uses the native 640x400 mode: 30 fps, no gaps,
   with sync on or off.
2. First-GOP loss. dai.node.RecordVideo without setFps() discards its first 9 frames to measure
   the frame rate (src/pipeline/node/host/Record.cpp, fpsInitLength = 10). That throws away the
   encoder's first SPS/PPS, and the mp4 writer then skips everything up to the next keyframe
   (frame 30, the VideoEncoder default), so every holistic mp4 lost exactly 30 frames (1 s at
   30 fps, 2+ s at 14 fps) and was stamped with a 33 fps timescale. The recorder's RecordVideo
   nodes only exist after pipeline.build(), so this script builds explicitly, calls setFps(--fps)
   on each of them and only then starts: 150-151 of 151 frames stored for a 5 s session, at a
   30 fps timescale (120 of 151 without the call).
The end-of-run check guards both: an mp4 with fewer than half the frames the host saw, or that
plays for less than half the session, fails the run (exit 1); below 90 % is a warning.

Both modes write session.json: device, settings, calibration (intrinsics at the recorded
sizes, baseline), the DepthAI node names to rebuild for holistic replay, every file with its
size, frame counts (host-side and stored), measured per-stream fps, warnings and the exact
replay command.

How to run (from the repo root)
-------------------------------
    python camera/05_record.py                              # holistic CAM_B + CAM_C + IMU, preview window, q stops
    python camera/05_record.py --seconds 10 --headless
    python camera/05_record.py --cameras all --no-encode --seconds 10   # + CAM_A as raw .avi (3 encoded streams fail)
    python camera/05_record.py --mode video --seconds 30 --width 1280 --height 720
    python camera/05_record.py --mode video --depth-stream disparity --align-rgb
    python camera/05_record.py --mode video --preset FAST_ACCURACY --decimation 2
    python camera/05_record.py --help

What you should see
-------------------
The device table and summary, the output folder, the record sizes, then either a small preview
window "05_record" with the live streams and an overlay (elapsed time, per-stream fps, time
left) or, with --headless, one status line per second. Auto-exposure needs ~1 s, so the first
frames of every recording are over-exposed - that is normal. On exit: the output folder, every
file written with its size (the tar's members for holistic, each mp4 with its stored frames /
duration / size), the measured frame rates, the integrity check result and the exact
camera/06_replay.py command.

Keys (window mode): q or Esc = stop the recording. Ctrl-C works in both modes.

Exit codes: 0 recording complete (warnings, if any, on stderr and in session.json), 1 recording
incomplete or pipeline error (one line per problem on stderr, no traceback), 2 no device /
wrong --device-id.
"""

# NOTE: no `from __future__ import annotations` here on purpose - dai.node.HostNode.link_args()
# type-checks the *evaluated* annotations of process(), and stringified annotations break it.

import argparse
import datetime
import gc
import json
import struct
import sys
import tarfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, BinaryIO, Callable, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402  (import after sys.path tweak on purpose)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    colorize_depth,
    colorize_disparity,
    draw_label,
    ensure_dir,
    open_device,
    print_device_summary,
    timestamp_dir,
    wait_key_quit,
)

EXIT_OK = 0
EXIT_INCOMPLETE = 1

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "recordings"
REPLAY_SCRIPT = Path(__file__).resolve().parent / "06_replay.py"

STEREO_SIZE: Tuple[int, int] = (640, 400)        # mono request size in video mode (native 1280x800 binned)
DEFAULT_RGB_SIZE: Tuple[int, int] = (1280, 720)  # CAM_A request unless --width/--height (or --align-rgb) say otherwise
#: Holistic mode: the CAM_B/CAM_C *preview* request. It must be strictly smaller than 640x400 in both
#: dimensions: the RVC2 holistic recorder records each camera at the smallest sensor mode strictly larger
#: than its largest request (640x400 -> recorded at 1280x720, which throttles the mono pair to ~14 fps and
#: broke every synced recording; 320x200 -> recorded at 640x400 = the stereo size, 30 fps, verified).
HOLISTIC_MONO_REQUEST_SIZE: Tuple[int, int] = (320, 200)
IMU_RATE_HZ = 200
IMU_BATCH_THRESHOLD = 10                        # 20 IMUData messages/s x 10 packets
IMU_MAX_BATCH_REPORTS = 20
HEADER_ONLY_MP4_BYTES = 436                     # an .mp4 of this size holds no samples (observed)
STORED_FRACTION_FAIL = 0.5                      # mp4 frames / host frames (or duration / session) below this = exit 1
STORED_FRACTION_WARN = 0.9                      # ... below this = warning (a frame or two in flight at stop is normal)
FPS_WARN_FRACTION = 0.7                         # a camera/depth stream below this share of --fps gets a warning
PREVIEW_TILE_HEIGHT = 300
WINDOW_NAME = "05_record"
TAR_WAIT_S = 15.0                               # safety net only; packing is normally done by then

SOCKET_NAMES = ("CAM_A", "CAM_B", "CAM_C")
HOST_STREAM_OF_NODE = {"CameraCAM_A": "rgb", "CameraCAM_B": "left", "CameraCAM_C": "right"}
#: PresetMode members of the installed depthai (3.10.0: FAST_ACCURACY, FAST_DENSITY, DEFAULT, FACE,
#: HIGH_DETAIL, ROBOTICS, DENSITY, ACCURACY).
PRESET_NAMES: List[str] = list(dai.node.StereoDepth.PresetMode.__members__)


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def _try(getter: Callable[[], Any], default: Any = None) -> Any:
    """Return ``getter()`` or *default* if it raises (device/calibration queries may fail)."""
    try:
        return getter()
    except Exception:  # noqa: BLE001 - best effort metadata only
        return default


def _enum_name(value: Any) -> str:
    return str(getattr(value, "name", value))


def human_size(num_bytes: int) -> str:
    """``12345678`` -> ``'11.8 MB'``."""
    size = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024.0 or unit == "GB":
            return "{:.1f} {}".format(size, unit) if unit != "B" else "{:d} B".format(int(size))
        size /= 1024.0
    return "{:.1f} GB".format(size)


def socket_of(name: str) -> dai.CameraBoardSocket:
    return getattr(dai.CameraBoardSocket, name)


def holistic_record_size(features: Sequence[Any], socket_name: str, request: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """Size the RVC2 holistic recorder will store *socket_name* at for a largest request of *request*.

    Mirrors getCameraRecordSize() in depthai-core src/opencv/HolisticRecordReplay.cpp (legacy branch):
    the smallest sensor config strictly larger than the request in both dimensions, then rounded to
    a multiple of 32 x 8 (down when that still covers the request, else up). Observed on this
    OAK-D S2: CAM_A 1280x720 -> 1344x1008, CAM_B/CAM_C 320x200 -> 640x400 and 640x400 -> 1280x720.
    Returns None when the socket is not in *features*.
    """
    socket = socket_of(socket_name)
    for cf in features:
        if cf.socket != socket:
            continue
        width, height = int(cf.width), int(cf.height)
        for cfg in cf.configs:
            if cfg.width > request[0] and cfg.height > request[1]:
                width, height = min(int(cfg.width), width), min(int(cfg.height), height)
        if width % 32:
            down = width - width % 32
            width = down if down >= request[0] else down + 32
        if height % 8:
            down = height - height % 8
            height = down if down >= request[1] else down + 8
        return min(width, int(cf.width)), min(height, int(cf.height))
    return None


def mp4_info(fh: BinaryIO, size: int) -> Dict[str, Any]:
    """Frames, duration and picture size of an mp4, read from its stsz / mvhd / tkhd boxes (stdlib only).

    Works on a plain file or a tar member (anything with seek/read); only box headers and the
    few small boxes needed are read, so a multi-GB mdat costs nothing. RecordVideo writes the
    mp4 through mp4v2, so ``frames`` = samples stored and ``duration_s`` = frames / the fps the
    RecordVideo node used. Returns {} for a header-only file (no moov / stsz).
    """
    info: Dict[str, Any] = {}

    def read_at(pos: int, count: int) -> bytes:
        fh.seek(pos)
        return fh.read(count)

    def walk(start: int, end: int) -> None:
        pos = start
        while pos + 8 <= end:
            head = read_at(pos, 8)
            if len(head) < 8:
                return
            box_size, box_type = struct.unpack(">I4s", head)
            header = 8
            if box_size == 1:
                big = read_at(pos + 8, 8)
                if len(big) < 8:
                    return
                box_size = struct.unpack(">Q", big)[0]
                header = 16
            elif box_size == 0:
                box_size = end - pos
            if box_size < header or pos + box_size > end:
                return
            body = pos + header
            if box_type in (b"moov", b"trak", b"mdia", b"minf", b"stbl"):
                walk(body, pos + box_size)
            elif box_type == b"mvhd":
                payload = read_at(body, 32)
                if len(payload) >= 32:
                    if payload[0] == 1:
                        timescale, duration = struct.unpack(">IQ", payload[20:32])
                    else:
                        timescale, duration = struct.unpack(">II", payload[12:20])
                    if timescale:
                        info["duration_s"] = round(duration / float(timescale), 3)
            elif box_type == b"tkhd" and "width" not in info:
                payload = read_at(body, 96)
                offset = 88 if payload[:1] == b"\x01" else 76
                if len(payload) >= offset + 8:
                    width, height = struct.unpack(">II", payload[offset:offset + 8])
                    if width and height:
                        info["width"], info["height"] = width >> 16, height >> 16
            elif box_type == b"stsz":
                payload = read_at(body, 12)
                if len(payload) >= 12:
                    info["frames"] = info.get("frames", 0) + struct.unpack(">I", payload[8:12])[0]
            pos += box_size

    walk(0, size)
    return info


def mp4_info_of_file(path: Path) -> Dict[str, Any]:
    with open(path, "rb") as fh:
        return mp4_info(fh, path.stat().st_size)


def describe_mp4(info: Dict[str, Any]) -> str:
    """``'150 frames, 5.0 s, 640x400'`` or ``'no frames (header only)'``."""
    if not info.get("frames"):
        return "no frames (header only)"
    parts = ["{} frames".format(info["frames"])]
    if info.get("duration_s") is not None:
        parts.append("{:.1f} s".format(info["duration_s"]))
    if info.get("width"):
        parts.append("{}x{}".format(info["width"], info["height"]))
    return ", ".join(parts)


@dataclass
class RunResult:
    """What happened during the streaming loop (filled by :func:`record_loop`) and after it."""

    frames: Dict[str, int] = field(default_factory=dict)
    imu_packets: int = 0
    elapsed: float = 0.0
    interrupted: bool = False
    error: Optional[str] = None
    max_disparity: Optional[float] = None
    depth_intrinsics: Optional[List[List[float]]] = None
    depth_size: Optional[Tuple[int, int]] = None
    depth_written: int = 0
    depth_gaps: int = 0
    node_names: List[str] = field(default_factory=list)
    cameras: List[str] = field(default_factory=list)
    record_sizes: Dict[str, Optional[Tuple[int, int]]] = field(default_factory=dict)   # holistic: per socket
    record_video_fps: Optional[int] = None      # holistic: setFps() applied to the recorder's RecordVideo nodes
    stereo_config: Dict[str, Any] = field(default_factory=dict)                       # video: effective StereoDepth setup
    mp4s: Dict[str, Dict[str, Any]] = field(default_factory=dict)                     # file name -> mp4_info
    warnings: List[str] = field(default_factory=list)

    def measured_fps(self) -> Dict[str, float]:
        if self.elapsed <= 0:
            return {name: 0.0 for name in self.frames}
        return {name: round(count / self.elapsed, 2) for name, count in self.frames.items()}


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record RGB + stereo depth + IMU from the OAK-D S2 into a timestamped folder "
                    "(holistic tar, or one RecordVideo/PNG stream per sensor) for camera/06_replay.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  python camera/05_record.py --seconds 10 --headless\n"
            "  python camera/05_record.py --cameras all --no-encode --seconds 10   # + CAM_A as raw .avi (3 encoded streams fail)\n"
            "  python camera/05_record.py --mode video --seconds 30 --width 1280 --height 720\n"
            "  python camera/05_record.py --mode video --depth-stream disparity --align-rgb\n"
            "  python camera/05_record.py --mode video --preset FAST_ACCURACY --decimation 2\n"
            "output: <out>/<mode>_<YYYYmmdd_HHMMSS>/ with session.json describing everything.\n"
            "holistic default = --cameras stereo (CAM_B + CAM_C at 640x400 + IMU, depth recomputed on replay);\n"
            "the mono preview is requested at 320x200 on purpose and every RecordVideo gets setFps(--fps) - both\n"
            "are needed on RVC2 for complete mp4s (see the module docstring). RGB + stored depth + IMU: --mode video."
        ),
    )
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument(
        "--seconds", dest="duration", type=float, default=argparse.SUPPRESS, metavar="SECONDS",
        help="alias for --duration: record this many seconds (0 = until q / Esc / Ctrl-C)",
    )
    parser.add_argument(
        "--mode", choices=("holistic", "video"), default="holistic",
        help="holistic = dai.RecordConfig + enableHolisticRecord -> one recording_*.tar (default); "
             "video = RecordVideo per stream (rgb.mp4, depth PNGs or disparity.mkv, imu.mcap)",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="camera frame rate for every stream (default 30)")
    parser.add_argument(
        "--width", type=int, default=None,
        help="RGB (CAM_A) request width, multiple of 32 (default {}). video mode: the rgb.mp4 size; holistic mode: "
             "DepthAI records the smallest sensor mode strictly above the request (1344x1008 for the default). "
             "--align-rgb fixes it to the stereo size {}x{}".format(DEFAULT_RGB_SIZE[0], *STEREO_SIZE),
    )
    parser.add_argument("--height", type=int, default=None,
                        help="RGB (CAM_A) request height, multiple of 8 (default {})".format(DEFAULT_RGB_SIZE[1]))
    parser.add_argument("--no-imu", action="store_true", help="do not record the IMU")
    parser.add_argument(
        "--out", type=Path, default=DEFAULT_OUT, metavar="DIR",
        help="base directory; the session goes to DIR/<mode>_<timestamp>/ (default: {})".format(DEFAULT_OUT),
    )
    parser.add_argument(
        "--cameras", choices=("all", "stereo", "rgb"), default=None,
        help="all = CAM_A + CAM_B + CAM_C (default in video mode; in holistic mode three ENCODED streams fail on "
             "this RVC2 device - verified today - so combine it with --no-encode), stereo = CAM_B + CAM_C only "
             "(default in holistic mode: recorded at 640x400, depth is recomputed on replay), rgb = CAM_A only "
             "(holistic: stored at 1344x1008, verified)",
    )
    parser.add_argument(
        "--depth-stream", choices=("depth", "disparity"), default="depth",
        help="video mode: depth = 16-bit PNG per frame in mm (default, exact); "
             "disparity = 8-bit disparity.mkv via RecordVideo (viewable, replayable)",
    )
    parser.add_argument(
        "--align-rgb", action="store_true",
        help="video mode: undistort the RGB stream, record it at the stereo size {}x{} and align depth/disparity "
             "to it (pixel-aligned with rgb.mp4)".format(*STEREO_SIZE),
    )
    parser.add_argument(
        "--preset", choices=PRESET_NAMES, default="DEFAULT",
        help="video mode: StereoDepth preset (default DEFAULT). Its spatial + temporal post-filters are switched "
             "off unless --preset-filters is given: with them StereoDepth manages ~17 fps at 640x400 on RVC2, "
             "without them 29-30 fps (measured for DEFAULT, FAST_ACCURACY, FAST_DENSITY)",
    )
    parser.add_argument(
        "--decimation", type=int, choices=(1, 2, 3, 4), default=1,
        help="video mode: StereoDepth decimation factor (default 1 = depth at the full {}x{}; 2 = {}x{})".format(
            STEREO_SIZE[0], STEREO_SIZE[1], STEREO_SIZE[0] // 2, STEREO_SIZE[1] // 2),
    )
    parser.add_argument(
        "--preset-filters", action="store_true",
        help="video mode: keep the preset's spatial and temporal post-filters (expect ~17 fps depth at 640x400 with "
             "DEFAULT on RVC2; combine with --decimation 2 for 30 fps)",
    )
    parser.add_argument("--no-sync", action="store_true",
                        help="holistic mode: RecordConfig.syncCameraOutputs = False (default on; both verified today)")
    parser.add_argument("--no-encode", action="store_true",
                        help="holistic mode: RecordConfig.videoEncoding.enabled = False (raw .avi, large)")
    args = parser.parse_args(argv)

    args.cameras_defaulted = args.cameras is None
    if args.cameras is None:
        args.cameras = "stereo" if args.mode == "holistic" else "all"
    if args.fps <= 0:
        parser.error("--fps must be > 0")
    if args.align_rgb:
        if args.mode != "video" or args.cameras != "all":
            parser.error("--align-rgb needs --mode video and --cameras all")
        for name, given, fixed in (("--width", args.width, STEREO_SIZE[0]), ("--height", args.height, STEREO_SIZE[1])):
            if given is not None and given != fixed:
                parser.error("{} {} cannot be combined with --align-rgb: depth-to-RGB alignment is only verified with "
                             "the RGB stream at the stereo size {}x{}, so --align-rgb records rgb.mp4 at that size; "
                             "drop --width/--height".format(name, given, *STEREO_SIZE))
        args.width, args.height = STEREO_SIZE
    if args.width is None:
        args.width = DEFAULT_RGB_SIZE[0]
    if args.height is None:
        args.height = DEFAULT_RGB_SIZE[1]
    if args.width < 32 or args.height < 8 or args.width % 32 or args.height % 8:
        parser.error("--width must be a multiple of 32 and --height a multiple of 8 (H264 encoder "
                     "requirement), e.g. 640x400, 1280x720, 1920x1080")
    return args


# --------------------------------------------------------------------------------------
# Host node for the 8-bit disparity stream (video mode, --depth-stream disparity)
# --------------------------------------------------------------------------------------

class DisparityToGray8(dai.node.HostNode):
    """Scale uint16 disparity (0..max_disparity) to an 8-bit GRAY8 frame RecordVideo can store.

    RecordVideo converts single-channel frames to BGR and writes FFV1 (lossless), so the 8-bit
    values survive; recover disparity as ``gray * max_disparity / 255`` (see session.json).
    """

    def build(self, disparity: dai.Node.Output, max_disparity: float) -> "DisparityToGray8":
        self.link_args(disparity)
        # HostNode.out defaults to Buffer; declare ImgFrame so RecordVideo.input accepts the link.
        self.out.setPossibleDatatypes([dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])
        self._scale = 255.0 / max(float(max_disparity), 1.0)
        return self

    def process(self, frame: dai.ImgFrame) -> None:
        disp = frame.getFrame()
        gray = np.clip(disp.astype(np.float32) * self._scale, 0.0, 255.0).astype(np.uint8)
        out = dai.ImgFrame()
        out.setCvFrame(gray, dai.ImgFrame.Type.GRAY8)
        out.setTimestamp(frame.getTimestamp())
        out.setTimestampDevice(frame.getTimestampDevice())
        out.setSequenceNum(frame.getSequenceNum())
        self.out.send(out)


# --------------------------------------------------------------------------------------
# 16-bit PNG depth writer (video mode, --depth-stream depth)
# --------------------------------------------------------------------------------------

class DepthPngWriter:
    """Write every depth frame as ``depth/depth_<seq>.png`` (uint16 mm) and index them in a CSV."""

    def __init__(self, folder: Path) -> None:
        self.dir = ensure_dir(folder / "depth")
        self.index_path = folder / "depth_index.csv"
        self._fh = open(self.index_path, "w", encoding="utf-8", newline="")
        self._fh.write("index,sequence,timestamp_s,device_timestamp_s,width,height,file\n")
        self.written = 0
        #: Missing sequence numbers between consecutive frames. StereoDepth numbers its outputs by the
        #: input pair, so a gap is a frame the device never produced (StereoDepth could not keep up) or,
        #: much more rarely, one that overflowed the host queue - it is NOT a PNG-writing failure.
        self.gaps = 0
        self._last_seq: Optional[int] = None

    def write(self, frame: dai.ImgFrame) -> None:
        arr = frame.getFrame()
        if arr.ndim != 2:
            arr = arr.reshape(frame.getHeight(), frame.getWidth())
        if arr.dtype != np.uint16:
            arr = arr.astype(np.uint16)
        seq = int(frame.getSequenceNum())
        if self._last_seq is not None and seq > self._last_seq + 1:
            self.gaps += seq - self._last_seq - 1
        self._last_seq = seq
        name = "depth_{:06d}.png".format(seq)
        if not cv2.imwrite(str(self.dir / name), arr, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise RuntimeError("could not write {}".format(self.dir / name))
        self._fh.write("{},{},{:.6f},{:.6f},{},{},depth/{}\n".format(
            self.written, seq, frame.getTimestamp().total_seconds(),
            frame.getTimestampDevice().total_seconds(), arr.shape[1], arr.shape[0], name))
        self.written += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------------------
# Shared streaming loop (preview / headless stats / duration / keys / Ctrl-C)
# --------------------------------------------------------------------------------------

def build_preview(latest: Dict[str, Any], counters: Dict[str, FpsCounter], clock: RunClock,
                  args: argparse.Namespace, res: RunResult) -> Optional[np.ndarray]:
    """Compose one BGR canvas from the latest message of every stream (None if nothing yet)."""
    tiles: List[np.ndarray] = []
    for name in ("rgb", "left", "right", "depth", "disparity"):
        msg = latest.get(name)
        if msg is None:
            continue
        try:
            if name == "depth":
                img = colorize_depth(msg.getFrame())
            elif name == "disparity":
                img = colorize_disparity(msg.getFrame(), res.max_disparity or 1.0)
            else:
                img = msg.getCvFrame()
        except Exception:  # noqa: BLE001 - never let a preview glitch kill the recording
            continue
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        h, w = img.shape[:2]
        scale = PREVIEW_TILE_HEIGHT / float(max(h, 1))
        img = cv2.resize(img, (max(1, int(round(w * scale))), PREVIEW_TILE_HEIGHT))
        draw_label(img, "{} {:.1f} fps".format(name, counters[name].fps()), (8, PREVIEW_TILE_HEIGHT - 10), 0.5)
        tiles.append(img)
    if not tiles:
        return None
    canvas = cv2.hconcat(tiles) if len(tiles) > 1 else tiles[0]
    remaining = clock.remaining()
    limit = "{:.0f} s left".format(remaining) if remaining is not None else "q / Esc to stop"
    text = "REC {}  {:.1f} s  ({})".format(args.mode, clock.elapsed(), limit)
    if "imu" in counters:
        text += "  imu {:.0f} msg/s".format(counters["imu"].fps())
    draw_label(canvas, text, (8, 22), 0.6)
    return canvas


def status_line(clock: RunClock, counters: Dict[str, FpsCounter], depth_writer: Optional[DepthPngWriter]) -> str:
    parts = ["[{:7.1f} s]".format(clock.elapsed())]
    for name, counter in counters.items():
        unit = "msg/s" if name == "imu" else "fps"
        parts.append("{} {:5.1f} {} ({})".format(name, counter.fps(), unit, counter.count))
    if depth_writer is not None:
        parts.append("png {} (gaps {})".format(depth_writer.written, depth_writer.gaps))
    remaining = clock.remaining()
    if remaining is not None:
        parts.append("{:.0f} s left".format(remaining))
    return "  ".join(parts)


def record_loop(pipeline: dai.Pipeline, queues: Dict[str, dai.MessageQueue], args: argparse.Namespace,
                res: RunResult, depth_writer: Optional[DepthPngWriter]) -> None:
    """Pump the queues until the duration expires, q is pressed, Ctrl-C, or the pipeline dies."""
    clock = RunClock(args.duration)
    counters = {name: FpsCounter() for name in queues}
    status = Periodic(1.0)
    latest: Dict[str, Any] = {}
    window_open = False
    try:
        while True:
            if not pipeline.isRunning():
                res.error = ("the pipeline stopped by itself - a node threw an exception "
                             "(see the [ThreadedNode] error above)")
                break
            if clock.expired():
                break
            got = 0
            new_frames = False
            for name, queue in queues.items():
                try:
                    msgs = queue.tryGetAll()
                except dai.MessageQueue.QueueException:
                    msgs = []
                if not msgs:
                    continue
                got += len(msgs)
                new_frames = new_frames or name != "imu"
                counters[name].tick(len(msgs))
                latest[name] = msgs[-1]
                if name == "imu":
                    res.imu_packets += sum(len(m.packets) for m in msgs)
                elif name == "depth" and depth_writer is not None:
                    for m in msgs:
                        depth_writer.write(m)
                if name in ("depth", "disparity") and res.depth_intrinsics is None:
                    m = msgs[-1]
                    res.depth_size = (int(m.getWidth()), int(m.getHeight()))
                    res.depth_intrinsics = _try(lambda: [list(map(float, row)) for row in
                                                         m.getTransformation().getIntrinsicMatrix()])
            if args.headless:
                if got == 0:
                    time.sleep(0.003)
                if status.ready():
                    print(status_line(clock, counters, depth_writer), flush=True)
                continue
            if new_frames:
                canvas = build_preview(latest, counters, clock, args, res)
                if canvas is not None:
                    try:
                        cv2.imshow(WINDOW_NAME, canvas)
                        window_open = True
                    except cv2.error as exc:
                        raise RuntimeError("cannot open a preview window ({}); use --headless".format(
                            str(exc).strip().splitlines()[-1] if str(exc).strip() else exc)) from None
            if window_open:
                if wait_key_quit(1) == "q":
                    print("q pressed - stopping the recording", flush=True)
                    break
            else:
                time.sleep(0.003)
    except KeyboardInterrupt:
        res.interrupted = True
        print("\nCtrl-C - stopping the recording", flush=True)
    res.elapsed = clock.elapsed()
    res.frames = {name: counter.count for name, counter in counters.items()}
    if depth_writer is not None:
        res.depth_written = depth_writer.written
        res.depth_gaps = depth_writer.gaps


def make_imu(pipeline: dai.Pipeline) -> dai.node.IMU:
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], IMU_RATE_HZ)
    imu.setBatchReportThreshold(IMU_BATCH_THRESHOLD)
    imu.setMaxBatchReports(IMU_MAX_BATCH_REPORTS)
    return imu


def imu_description(args: argparse.Namespace) -> Optional[Dict[str, Any]]:
    if args.no_imu:
        return None
    return {
        "sensors": ["ACCELEROMETER_RAW", "GYROSCOPE_RAW"],
        "report_rate_hz": IMU_RATE_HZ,
        "batch_report_threshold": IMU_BATCH_THRESHOLD,
        "max_batch_reports": IMU_MAX_BATCH_REPORTS,
        "units": {"accelerometer": "m/s^2", "gyroscope": "rad/s"},
    }


# --------------------------------------------------------------------------------------
# Mode 1: holistic record (RecordConfig + enableHolisticRecord)
# --------------------------------------------------------------------------------------

def run_holistic(device: dai.Device, args: argparse.Namespace, folder: Path, res: RunResult) -> None:
    """Record with enableHolisticRecord; fills *res* in place so partial results survive an exception."""
    want_rgb = args.cameras in ("all", "rgb")
    want_stereo = args.cameras in ("all", "stereo")
    fps_int = max(1, int(round(args.fps)))

    config = dai.RecordConfig()
    config.outputDir = folder.resolve()                 # must exist and be writable
    config.syncCameraOutputs = not args.no_sync
    config.videoEncoding.enabled = not args.no_encode
    config.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN

    with dai.Pipeline(device) as pipeline:
        pipeline.enableHolisticRecord(config)           # BEFORE creating any node
        queues: Dict[str, dai.MessageQueue] = {}
        if want_rgb:
            cam_a = pipeline.create(dai.node.Camera).build(socket_of("CAM_A"))
            # The recorder stores each camera at the smallest sensor mode strictly larger than its largest
            # requested output (see holistic_record_size), so --width/--height steer the recorded CAM_A size.
            queues["rgb"] = cam_a.requestOutput((args.width, args.height), fps=args.fps).createOutputQueue(
                maxSize=2, blocking=False)
            res.cameras.append("CAM_A")
            res.node_names.append("CameraCAM_A")
        if want_stereo:
            cam_b = pipeline.create(dai.node.Camera).build(socket_of("CAM_B"))
            cam_c = pipeline.create(dai.node.Camera).build(socket_of("CAM_C"))
            # 320x200 on purpose: it makes the recorder pick the 640x400 sensor mode (a 640x400 request would be
            # recorded at 1280x720, which throttles the pair to ~14 fps and broke every synced recording).
            queues["left"] = cam_b.requestOutput(HOLISTIC_MONO_REQUEST_SIZE, fps=args.fps).createOutputQueue(
                maxSize=2, blocking=False)
            queues["right"] = cam_c.requestOutput(HOLISTIC_MONO_REQUEST_SIZE, fps=args.fps).createOutputQueue(
                maxSize=2, blocking=False)
            res.cameras.extend(["CAM_B", "CAM_C"])
            res.node_names.extend(["CameraCAM_B", "CameraCAM_C"])
        if not args.no_imu:
            imu = make_imu(pipeline)
            queues["imu"] = imu.out.createOutputQueue(maxSize=50, blocking=False)
            res.node_names.append("IMU")

        # build() is where DepthAI creates the recorder's RecordVideo / VideoEncoder / ImageManip nodes
        # (setupHolisticRecordAndReplay); start() would do it implicitly, but then the RecordVideo nodes run
        # without an fps, drop their first 9 frames to measure it and lose the first GOP (see the docstring).
        pipeline.build()
        record_nodes = [node for node in pipeline.getAllNodes() if isinstance(node, dai.node.RecordVideo)]
        for node in record_nodes:
            node.setFps(fps_int)
        if record_nodes:
            res.record_video_fps = fps_int
        else:
            res.warnings.append("no RecordVideo node was created by enableHolisticRecord - DepthAI probably logged "
                                "'Record disabled: ...' above; nothing will be packed")

        pipeline.start()
        print("Recording (holistic: {}{}, encoding {}, sync {}, RecordVideo fps {})...".format(
            " + ".join(res.cameras), "" if args.no_imu else " + IMU",
            "off (raw .avi)" if args.no_encode else "H264_MAIN", "off" if args.no_sync else "on",
            res.record_video_fps if res.record_video_fps else "not set"), flush=True)
        try:
            record_loop(pipeline, queues, args, res, None)
        finally:
            pipeline.stop()
    # The tar is packed when the pipeline object is destroyed (function return + gc in main()).


# --------------------------------------------------------------------------------------
# Mode 2: RecordVideo / PNG per stream
# --------------------------------------------------------------------------------------

def run_video(device: dai.Device, args: argparse.Namespace, folder: Path, res: RunResult) -> None:
    """Record with RecordVideo/PNG per stream; fills *res* in place so partial results survive an exception."""
    want_rgb = args.cameras in ("all", "rgb")
    want_stereo = args.cameras in ("all", "stereo")
    fps_int = max(1, int(round(args.fps)))

    with dai.Pipeline(device) as pipeline:
        queues: Dict[str, dai.MessageQueue] = {}
        rgb_out: Optional[dai.Node.Output] = None
        if want_rgb:
            cam_a = pipeline.create(dai.node.Camera).build(socket_of("CAM_A"))
            request_kwargs: Dict[str, Any] = {"fps": args.fps}
            if args.align_rgb:
                # parse_args forced (width, height) == STEREO_SIZE: the undistorted stream feeds both the encoder
                # and stereo.inputAlignTo, so depth and rgb.mp4 share one size (the verified alignment setup).
                request_kwargs["enableUndistortion"] = True
            rgb_out = cam_a.requestOutput((args.width, args.height), dai.ImgFrame.Type.NV12, **request_kwargs)
            encoder = pipeline.create(dai.node.VideoEncoder).build(
                rgb_out, frameRate=args.fps, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)
            rec_rgb = pipeline.create(dai.node.RecordVideo)
            rec_rgb.setRecordVideoFile(folder / "rgb.mp4")          # encoded -> must be .mp4
            rec_rgb.setRecordMetadataFile(folder / "rgb.mcap")
            rec_rgb.setFps(fps_int)                                  # no fps-measuring drop of the first frames
            encoder.out.link(rec_rgb.input)
            queues["rgb"] = rgb_out.createOutputQueue(maxSize=2, blocking=False)
            res.cameras.append("CAM_A")
            res.node_names.append("CameraCAM_A")
        depth_writer: Optional[DepthPngWriter] = None
        if want_stereo:
            cam_b = pipeline.create(dai.node.Camera).build(socket_of("CAM_B"))
            cam_c = pipeline.create(dai.node.Camera).build(socket_of("CAM_C"))
            preset = dai.node.StereoDepth.PresetMode.__members__[args.preset]
            stereo = pipeline.create(dai.node.StereoDepth).build(
                left=cam_b.requestOutput(STEREO_SIZE, fps=args.fps),
                right=cam_c.requestOutput(STEREO_SIZE, fps=args.fps),
                presetMode=preset)
            stereo.setLeftRightCheck(True)
            post = stereo.initialConfig.postProcessing
            post.decimationFilter.decimationFactor = args.decimation
            if not args.preset_filters:
                # The spatial + temporal filters cap StereoDepth at ~17 fps for 640x400 on RVC2 (measured);
                # without them DEFAULT / FAST_ACCURACY / FAST_DENSITY all sustain 29-30 fps.
                post.spatialFilter.enable = False
                post.temporalFilter.enable = False
            if args.align_rgb and rgb_out is not None:
                rgb_out.link(stereo.inputAlignTo)
            res.max_disparity = float(stereo.initialConfig.getMaxDisparity())
            res.stereo_config = {
                "preset": args.preset,
                "left_right_check": True,
                "decimation_factor": args.decimation,
                "spatial_filter": bool(post.spatialFilter.enable),
                "temporal_filter": bool(post.temporalFilter.enable),
                "speckle_filter": bool(post.speckleFilter.enable),
                "median": _enum_name(_try(stereo.initialConfig.getMedianFilter, "?")),
                "subpixel": _try(stereo.initialConfig.getSubpixel),
                "extended_disparity": _try(stereo.initialConfig.getExtendedDisparity),
                "max_disparity": res.max_disparity,
            }
            if args.depth_stream == "disparity":
                to_gray = pipeline.create(DisparityToGray8).build(stereo.disparity, res.max_disparity)
                rec_disp = pipeline.create(dai.node.RecordVideo)
                rec_disp.setRecordVideoFile(folder / "disparity.mkv")   # raw -> .avi or .mkv
                rec_disp.setRecordMetadataFile(folder / "disparity.mcap")
                rec_disp.setFps(fps_int)
                to_gray.out.link(rec_disp.input)
                queues["disparity"] = stereo.disparity.createOutputQueue(maxSize=2, blocking=False)
            else:
                depth_writer = DepthPngWriter(folder)
                queues["depth"] = stereo.depth.createOutputQueue(maxSize=8, blocking=False)
            res.cameras.extend(["CAM_B", "CAM_C"])
            res.node_names.extend(["CameraCAM_B", "CameraCAM_C"])
        if not args.no_imu:
            imu = make_imu(pipeline)
            rec_imu = pipeline.create(dai.node.RecordMetadataOnly)
            rec_imu.setRecordFile(folder / "imu.mcap")
            imu.out.link(rec_imu.input)
            queues["imu"] = imu.out.createOutputQueue(maxSize=50, blocking=False)
            res.node_names.append("IMU")

        pipeline.start()
        streams = []
        if want_rgb:
            streams.append("rgb.mp4 {}x{}".format(args.width, args.height))
        if want_stereo:
            streams.append("{} [{} preset, decimation {}, spatial/temporal filters {}]".format(
                "depth PNGs" if args.depth_stream == "depth" else "disparity.mkv", args.preset, args.decimation,
                "on" if args.preset_filters else "off"))
        if not args.no_imu:
            streams.append("imu.mcap")
        print("Recording (video mode: {})...".format(", ".join(streams)), flush=True)
        try:
            record_loop(pipeline, queues, args, res, depth_writer)
        finally:
            pipeline.stop()
            if depth_writer is not None:
                depth_writer.close()


# --------------------------------------------------------------------------------------
# After the run: tar, file listing, integrity check, session.json
# --------------------------------------------------------------------------------------

def wait_for_tar(folder: Path, timeout_s: float) -> Optional[Path]:
    """Return recording_*.tar once DepthAI has packed it and removed the loose files.

    Packing happens synchronously in the pipeline destructor (gc.collect() in main), so this
    is normally immediate; the poll is a safety net. Loose ``<id>_v_*`` files still there
    after the timeout mean packing is either still running (a pipeline object is alive
    somewhere) or failed ('[depthai] [error] Record: Failed to create tar file' logged
    earlier); check_holistic reports the outcome either way.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        tars = sorted(folder.glob("recording_*.tar"))
        loose = [p for p in folder.iterdir() if p.is_file() and p.suffix in (".mcap", ".mp4", ".avi")]
        if tars and not loose:
            return tars[-1]
        if time.monotonic() > deadline:
            if loose:
                print("warning: {} loose recording files remain next to the tar: DepthAI is either still packing it "
                      "or packing failed (look for a '[depthai] [error] Record:' line above); the session is probably "
                      "incomplete".format(len(loose)), file=sys.stderr, flush=True)
            return tars[-1] if tars else None
        time.sleep(0.25)


def inspect_tar(tar: Path) -> Tuple[Dict[str, int], Dict[str, Dict[str, Any]], Optional[str]]:
    """Member sizes, mp4_info() of every ``*.mp4`` member, and an error string for a broken tar."""
    members: Dict[str, int] = {}
    mp4s: Dict[str, Dict[str, Any]] = {}
    try:
        with tarfile.open(tar) as tf:
            for member in tf.getmembers():
                members[member.name] = member.size
                if member.name.endswith(".mp4") and member.isfile():
                    fh = tf.extractfile(member)
                    if fh is not None:
                        with fh:
                            mp4s[member.name] = mp4_info(fh, member.size)
    except Exception as exc:  # noqa: BLE001 - ReadError, EOFError, OSError ... all mean "broken tar"
        return members, mp4s, "{}: {}".format(type(exc).__name__, exc)
    return members, mp4s, None


def list_files(folder: Path) -> List[Tuple[Path, int]]:
    return sorted(((p, p.stat().st_size) for p in folder.rglob("*") if p.is_file()), key=lambda t: str(t[0]))


def print_file_listing(folder: Path, files: List[Tuple[Path, int]], tar: Optional[Path],
                       members: Dict[str, int], mp4s: Dict[str, Dict[str, Any]], tar_error: Optional[str]) -> None:
    print("\nOutput folder: {}".format(folder.resolve()), flush=True)
    print("Files written:", flush=True)

    def is_depth_png(path: Path) -> bool:
        return path.parent.name == "depth" and path.suffix == ".png"

    pngs = [(p, s) for p, s in files if is_depth_png(p)]
    others = [(p, s) for p, s in files if not is_depth_png(p)]
    for path, size in others:
        rel = path.relative_to(folder).as_posix()
        extra = "   {}".format(describe_mp4(mp4s[rel])) if rel in mp4s else ""
        print("  {:<44} {:>14,} B  ({}){}".format(rel, size, human_size(size), extra), flush=True)
        if tar is not None and path == tar:
            for name, member_size in members.items():
                extra = "   {}".format(describe_mp4(mp4s[name])) if name in mp4s else ""
                print("      {:<40} {:>14,} B  ({}){}".format(name, member_size, human_size(member_size), extra),
                      flush=True)
            if tar_error:
                print("      (cannot list the tar: {})".format(tar_error), flush=True)
    if pngs:
        total = sum(s for _, s in pngs)
        print("  {:<44} {:>14,} B  ({})   {} .. {}".format(
            "depth/ ({} x 16-bit PNG)".format(len(pngs)), total, human_size(total),
            pngs[0][0].name, pngs[-1][0].name), flush=True)
    total_all = sum(s for _, s in files)
    print("  {:<44} {:>14,} B  ({})".format("total", total_all, human_size(total_all)), flush=True)


def check_mp4(name: str, size: int, info: Dict[str, Any], seen: int, elapsed: float,
              problems: List[str], warnings: List[str]) -> None:
    """Header-only / stored-fraction / duration checks for one mp4 (holistic member or rgb.mp4)."""
    frames = int(info.get("frames") or 0)
    if size <= HEADER_ONLY_MP4_BYTES or frames == 0:
        problems.append("{} holds no frames ({} B, header-only mp4)".format(name, size))
        return
    if seen > 0:
        share = frames / float(seen)
        if share < STORED_FRACTION_FAIL:
            problems.append("{} stores only {} of the {} frames the host saw ({:.0f} %)".format(
                name, frames, seen, 100.0 * share))
            return
        if share < STORED_FRACTION_WARN:
            warnings.append("{} stores {} of the {} frames the host saw ({:.0f} %)".format(
                name, frames, seen, 100.0 * share))
    duration = info.get("duration_s")
    if duration is not None and elapsed > 0 and duration < STORED_FRACTION_FAIL * elapsed:
        problems.append("{} plays for {:.1f} s of the {:.1f} s session (its frames were stamped at the wrong "
                        "rate or most of them are missing)".format(name, duration, elapsed))


def check_holistic(tar: Optional[Path], members: Dict[str, int], mp4s: Dict[str, Dict[str, Any]],
                   tar_error: Optional[str], res: RunResult, args: argparse.Namespace) -> Tuple[List[str], List[str]]:
    problems: List[str] = []
    warnings: List[str] = []
    if tar is None:
        problems.append("no recording_*.tar was written (DepthAI logs 'Record: Failed to create tar file' when a "
                        "camera stream never produced video)")
        return problems, warnings
    if tar_error:
        problems.append("{} is unreadable/truncated ({})".format(tar.name, tar_error))
        return problems, warnings
    video_ext = ".avi" if args.no_encode else ".mp4"
    for node in res.node_names:
        expected = [node + ".mcap"] + ([node + video_ext] if node.startswith("Camera") else [])
        for name in expected:
            size = members.get(name)
            if size is None:
                problems.append("{} is missing from the tar".format(name))
            elif size == 0:
                problems.append("{} is empty".format(name))
            elif name.endswith(".mp4"):
                seen = res.frames.get(HOST_STREAM_OF_NODE.get(node, ""), 0)
                check_mp4(name, size, mp4s.get(name, {}), seen, res.elapsed, problems, warnings)
    for name in ("record_config.json", "camera_info.json"):
        if not members.get(name):
            problems.append("{} is missing from the tar".format(name))
    return problems, warnings


def check_video(folder: Path, res: RunResult, args: argparse.Namespace) -> Tuple[List[str], List[str]]:
    problems: List[str] = []
    warnings: List[str] = []

    def size_of(name: str) -> int:
        path = folder / name
        return path.stat().st_size if path.is_file() else -1

    if "CAM_A" in res.cameras:
        size = size_of("rgb.mp4")
        if size < 0:
            problems.append("rgb.mp4 is missing")
        else:
            check_mp4("rgb.mp4", size, res.mp4s.get("rgb.mp4", {}), res.frames.get("rgb", 0), res.elapsed,
                      problems, warnings)
        if size_of("rgb.mcap") <= 0:
            problems.append("rgb.mcap is missing or empty")
    if "CAM_B" in res.cameras:
        if args.depth_stream == "depth":
            if res.depth_written == 0:
                problems.append("no depth PNG was written")
        else:
            if size_of("disparity.mkv") <= 0:
                problems.append("disparity.mkv is missing or empty")
            if size_of("disparity.mcap") <= 0:
                problems.append("disparity.mcap is missing or empty")
    if not args.no_imu and size_of("imu.mcap") <= 0:
        problems.append("imu.mcap is missing or empty")
    return problems, warnings


def rate_warnings(res: RunResult, args: argparse.Namespace) -> List[str]:
    """One warning per camera/depth stream that ran below FPS_WARN_FRACTION of --fps."""
    out: List[str] = []
    for name, fps in res.measured_fps().items():
        if name == "imu" or fps >= FPS_WARN_FRACTION * args.fps:
            continue
        if name in ("depth", "disparity"):
            hint = ("StereoDepth could not keep up on the device{} - try --decimation 2, a lighter --preset, or drop "
                    "--preset-filters".format(" ({} sequence gaps)".format(res.depth_gaps) if res.depth_gaps else ""))
        elif args.mode == "holistic":
            hint = "the holistic record path is throttling the camera (record size / encoder, see the docstring)"
        else:
            hint = "the camera or the encoder could not keep up"
        out.append("{} ran at {:.1f} fps, below {:.0f} % of the requested {:g} fps: {}".format(
            name, fps, FPS_WARN_FRACTION * 100, args.fps, hint))
    return out


def retry_hint(args: argparse.Namespace) -> str:
    """'Try: ...' line for an incomplete recording, listing only flags that were NOT already in effect."""
    options: List[str] = []
    if args.mode == "holistic":
        if not args.no_sync:
            options.append("--no-sync")
        if not args.no_encode:
            options.append("--no-encode (raw .avi)")
        if args.cameras == "all":
            options.append("--cameras stereo or --cameras rgb (fewer encoded streams)")
        options.append("--mode video (one RecordVideo per stream)")
    else:
        if args.cameras == "all":
            options.append("--cameras stereo / --cameras rgb to isolate the failing stream")
        if args.depth_stream == "depth" and args.cameras != "rgb":
            options.append("--depth-stream disparity")
        options.append("--mode holistic")
    return "  Try: " + "; ".join(options) + "."


def describe_device(device: dai.Device) -> Dict[str, Any]:
    return {
        "name": _try(device.getDeviceName),
        "product": _try(device.getProductName),
        "id": _try(device.getDeviceId),
        "platform": _try(device.getPlatformAsString),
        "usb_speed": _enum_name(_try(device.getUsbSpeed, "?")),
        "imu": _try(device.getConnectedIMU),
        "imu_firmware": _try(lambda: str(device.getIMUFirmwareVersion())),
        "depthai_version": getattr(dai, "__version__", "?"),
    }


def read_calibration(device: dai.Device, rgb_size: Tuple[int, int], mode: str) -> Tuple[Dict[str, Any], Optional[Any]]:
    """Intrinsics at the requested sizes + baseline (dict) and the full EEPROM json (or None)."""
    calib = _try(device.readCalibration)
    if calib is None:
        return {"error": "readCalibration() failed"}, None
    cam_a, cam_b, cam_c = socket_of("CAM_A"), socket_of("CAM_B"), socket_of("CAM_C")
    w, h = rgb_size
    info: Dict[str, Any] = {
        "baseline_cm": _try(calib.getBaselineDistance),
        "rgb_intrinsics_{}x{}".format(w, h): _try(lambda: calib.getCameraIntrinsics(cam_a, w, h)),
        "rgb_distortion": _try(lambda: calib.getDistortionCoefficients(cam_a)),
        "left_intrinsics_{}x{}".format(*STEREO_SIZE): _try(lambda: calib.getCameraIntrinsics(cam_b, *STEREO_SIZE)),
        "right_intrinsics_{}x{}".format(*STEREO_SIZE): _try(lambda: calib.getCameraIntrinsics(cam_c, *STEREO_SIZE)),
        "stereo_left": _enum_name(_try(calib.getStereoLeftCameraId, "?")),
        "stereo_right": _enum_name(_try(calib.getStereoRightCameraId, "?")),
    }
    if mode == "holistic":
        info["note"] = ("rgb intrinsics are scaled to the CAM_A request size {}x{}; the holistic recorder stores "
                        "CAM_A at the sensor mode it picks (holistic.record_size) and camera_info.json inside the "
                        "tar has the full calibration - rescale with "
                        "CalibrationHandler.getCameraIntrinsics(socket, w, h); the mono videos are stored at "
                        "{}x{} = the left/right intrinsics size".format(w, h, *STEREO_SIZE))
    else:
        info["note"] = ("rgb intrinsics at the recorded rgb.mp4 size {}x{}, left/right at the StereoDepth input size "
                        "{}x{}; the depth/disparity frames carry their own intrinsics under "
                        "video.depth.intrinsics / video.disparity.intrinsics".format(w, h, *STEREO_SIZE))
    return info, _try(calib.eepromToJson)


def replay_command(folder: Path, mode: str, tar: Optional[Path] = None, imu: bool = False) -> str:
    """Exact 06_replay.py invocation: the tar + --mode holistic, or the session folder (video).

    06_replay.py plays the videos and the depth PNG sequence by default; IMU .mcap files only
    with --imu, so that flag is appended when the IMU was recorded.
    """
    if mode == "holistic":
        target = tar.resolve() if tar is not None else folder.resolve()
        cmd = '"{}" "{}" "{}" --mode holistic'.format(sys.executable, REPLAY_SCRIPT, target)
    else:
        cmd = '"{}" "{}" "{}"'.format(sys.executable, REPLAY_SCRIPT, folder.resolve())
    return cmd + " --imu" if imu else cmd


def write_session_json(folder: Path, args: argparse.Namespace, device_info: Dict[str, Any],
                       calib_info: Dict[str, Any], res: RunResult, files: List[Tuple[Path, int]],
                       tar: Optional[Path], problems: List[str], warnings: List[str]) -> Path:
    manifest: Dict[str, Any] = {
        "script": "05_record.py",
        "mode": args.mode,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "duration_requested_s": args.duration,
        "duration_recorded_s": round(res.elapsed, 3),
        "interrupted": res.interrupted,
        "error": res.error,
        "complete": not problems,
        "problems": problems,
        "warnings": warnings,
        "device": device_info,
        "fps_requested": args.fps,
        "measured_fps": res.measured_fps(),          # per stream, frames seen on the host / recorded seconds
        "cameras": res.cameras,
        "node_names": res.node_names,
        "imu": imu_description(args),
        "stereo_size": list(STEREO_SIZE),
        "frames_seen_on_host": res.frames,
        "imu_packets_seen_on_host": res.imu_packets,
        "calibration": calib_info,
        "files": [{"name": p.relative_to(folder).as_posix(), "bytes": s} for p, s in files],
        "replay_command": replay_command(folder, args.mode, tar, imu=not args.no_imu),
    }
    if args.mode == "holistic":
        manifest["holistic"] = {
            "tar": tar.name if tar is not None else None,
            "cameras_defaulted_to_stereo": args.cameras_defaulted,
            "request_size": {"CAM_A": [args.width, args.height], "CAM_B": list(HOLISTIC_MONO_REQUEST_SIZE),
                             "CAM_C": list(HOLISTIC_MONO_REQUEST_SIZE)},
            "record_size": {socket: list(size) if size else None for socket, size in res.record_sizes.items()},
            "record_size_note": "DepthAI records each camera at the smallest RVC2 sensor mode strictly larger than "
                                "its largest request, rounded to 32x8 (CAM_A 1280x720 -> 1344x1008, CAM_B/CAM_C "
                                "320x200 -> 640x400); the mono preview is requested at 320x200 so the 640x400 "
                                "mode is recorded (a 640x400 request records 1280x720 at ~14 fps)",
            "record_video_fps": res.record_video_fps,
            "record_video_fps_note": "setFps() applied to the recorder's RecordVideo nodes after pipeline.build(); "
                                     "without it RecordVideo drops its first 9 frames and the mp4 loses the first GOP",
            "mp4": res.mp4s,                        # per member: frames stored, duration_s, width, height
            "sync_camera_outputs": not args.no_sync,
            "video_encoding_enabled": not args.no_encode,
            "video_profile": "H264_MAIN",
            "note": "replay with dai.Pipeline(True).enableHolisticReplay(tar) and the same Camera/IMU nodes "
                    "(node_names); depth is recomputed by a StereoDepth node on the device",
        }
    else:
        video: Dict[str, Any] = {
            "rgb": {"video": "rgb.mp4", "metadata": "rgb.mcap", "size": [args.width, args.height],
                    "codec": "H264_MAIN", "pixel_format": "NV12 before encoding",
                    "undistorted": bool(args.align_rgb), "stored": res.mp4s.get("rgb.mp4")}
            if "CAM_A" in res.cameras else None,
            "imu": "imu.mcap" if not args.no_imu else None,
            "calibration_dump": "calibration.json",
        }
        if "CAM_B" in res.cameras:
            frame = ("aligned to the undistorted RGB stream (same {}x{} size as rgb.mp4)".format(args.width, args.height)
                     if args.align_rgb else "rectified right camera (CAM_C)")
            if args.depth_stream == "depth":
                video["depth"] = {
                    "format": "16-bit PNG per frame, uint16 millimetres, 0 = invalid",
                    "directory": "depth", "index": "depth_index.csv",
                    "written": res.depth_written,
                    "sequence_gaps": res.depth_gaps,
                    "sequence_gaps_note": "frames StereoDepth never delivered (device could not keep up) or, rarely, "
                                          "that overflowed the host queue; not a PNG-writing failure",
                    "size": list(res.depth_size) if res.depth_size else None,
                    "frame": frame,
                    "intrinsics": res.depth_intrinsics,
                    "read": 'cv2.imread(path, cv2.IMREAD_UNCHANGED)',
                }
            else:
                video["disparity"] = {
                    "video": "disparity.mkv", "metadata": "disparity.mcap",
                    "format": "8-bit gray, FFV1 lossless; disparity_px = gray * max_disparity / 255 (subpixel units "
                              "as StereoDepth reports them)",
                    "max_disparity": res.max_disparity,
                    "size": list(res.depth_size) if res.depth_size else None,
                    "frame": frame,
                    "intrinsics": res.depth_intrinsics,
                }
            video["stereo"] = dict(res.stereo_config)
            video["stereo"]["measured_fps"] = res.measured_fps().get(args.depth_stream)
        manifest["video"] = video
    path = folder / "session.json"
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, default=str)
    return path


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # An explicit --device-id never retries: one stderr line even when nothing enumerates.
    # Without an id the open is retried while a camera reboots after another script.
    device = open_device(args.device_id, retries=1 if args.device_id else 3)  # exits 2 on failure
    print_device_summary(device)
    device_info = describe_device(device)
    calib_info, eeprom_json = read_calibration(device, (args.width, args.height), args.mode)
    features = _try(device.getConnectedCameraFeatures, []) or []

    folder = timestamp_dir(args.out, args.mode)
    print("Output folder: {}".format(folder.resolve()), flush=True)
    cams = {"all": "CAM_A + CAM_B + CAM_C", "stereo": "CAM_B + CAM_C", "rgb": "CAM_A"}[args.cameras]
    limit = "{:g} s".format(args.duration) if args.duration > 0 else ("until Ctrl-C" if args.headless else "until q / Esc / Ctrl-C")
    print("Mode: {}  cameras: {}  imu: {}  fps: {:g}  duration: {}".format(
        args.mode, cams, "off" if args.no_imu else "on", args.fps, limit), flush=True)

    res = RunResult()               # filled in place, so whatever was recorded survives an exception below
    if args.mode == "holistic":
        if args.cameras in ("all", "rgb"):
            res.record_sizes["CAM_A"] = holistic_record_size(features, "CAM_A", (args.width, args.height))
        if args.cameras in ("all", "stereo"):
            for socket_name in ("CAM_B", "CAM_C"):
                res.record_sizes[socket_name] = holistic_record_size(features, socket_name, HOLISTIC_MONO_REQUEST_SIZE)
        sizes = ", ".join("{} {}".format(s, "{}x{}".format(*sz) if sz else "?") for s, sz in res.record_sizes.items())
        print("Holistic record sizes (smallest RVC2 sensor mode strictly above each request): {}{}".format(
            sizes, "" if args.no_encode else "; H264 via the recorder's own ImageManip + VideoEncoder"), flush=True)
        if args.cameras_defaulted:
            print("Holistic default: CAM_B + CAM_C{} - a depth-only session, depth is recomputed on replay "
                  "(verified today: 150-151 of 151 frames stored per camera for 5 s, sync on or off). "
                  "--cameras all --no-encode adds CAM_A as raw .avi (three encoded streams fail); RGB + stored "
                  "depth + IMU is --mode video.".format("" if args.no_imu else " + IMU"), flush=True)
        elif args.cameras == "all" and not args.no_encode:
            print("Note: --cameras all with encoding = three encoded holistic streams, which failed on this RVC2 "
                  "device in every test today (the encoders throttle every camera to ~17 fps and only ~17 frames "
                  "per camera reach the mp4s with sync on): expect the check at the end to fail. Use --no-encode "
                  "(raw .avi, ~26 MB per 5 s, 148/148 frames verified) or --mode video.", flush=True)
    else:
        print("Video mode StereoDepth: preset {}  decimation {}  spatial/temporal filters {}  ({})".format(
            args.preset, args.decimation, "on" if args.preset_filters else "off",
            "expect ~17 fps at 640x400 on RVC2 with DEFAULT + filters" if args.preset_filters and args.decimation == 1
            else "29-30 fps measured at 640x400 without the filters"), flush=True)
        if eeprom_json is not None:
            try:
                with open(folder / "calibration.json", "w", encoding="utf-8") as fh:
                    json.dump(eeprom_json, fh, indent=2, default=str)
            except (OSError, TypeError, ValueError) as exc:
                print("warning: could not write calibration.json: {}".format(exc), file=sys.stderr, flush=True)

    try:
        if args.mode == "holistic":
            run_holistic(device, args, folder, res)
        else:
            run_video(device, args, folder, res)
    except KeyboardInterrupt:
        res.interrupted = True
        if res.frames:
            print("\nCtrl-C during shutdown - keeping what was recorded", flush=True)
        else:
            print("\nCtrl-C before the pipeline was running - nothing recorded", flush=True)
    except Exception as exc:  # noqa: BLE001 - one line instead of a traceback
        res.error = "{}: {}".format(type(exc).__name__, " ".join(str(exc).split()))
    finally:
        if not args.headless:           # a GUI-less OpenCV build (opencv-python-headless) raises here
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
    device = None           # the pipeline already closed it; drop our reference too
    gc.collect()            # destroy the pipeline object -> holistic tar gets packed now

    tar: Optional[Path] = None
    members: Dict[str, int] = {}
    tar_error: Optional[str] = None
    if args.mode == "holistic":
        print("Waiting for DepthAI to pack recording_*.tar ...", flush=True)
        tar = wait_for_tar(folder, TAR_WAIT_S)
        if tar is not None:
            members, res.mp4s, tar_error = inspect_tar(tar)
    else:
        rgb_mp4 = folder / "rgb.mp4"
        if rgb_mp4.is_file():
            res.mp4s["rgb.mp4"] = _try(lambda: mp4_info_of_file(rgb_mp4), {}) or {}

    files = list_files(folder)
    print_file_listing(folder, files, tar, members, res.mp4s, tar_error)

    if res.frames:
        measured = res.measured_fps()
        counts = ", ".join("{} {:.1f} {} ({})".format(name, measured[name], "msg/s" if name == "imu" else "fps", count)
                           for name, count in res.frames.items())
        print("\nRecorded {:.1f} s; measured on the host: {}{}".format(
            res.elapsed, counts,
            "; IMU packets {}".format(res.imu_packets) if "imu" in res.frames else ""), flush=True)
        if args.mode == "video" and args.depth_stream == "depth" and "CAM_B" in res.cameras:
            print("Depth PNGs written: {} (sequence gaps: {} - frames StereoDepth never delivered)".format(
                res.depth_written, res.depth_gaps), flush=True)

    if args.mode == "holistic":
        problems, warnings = check_holistic(tar, members, res.mp4s, tar_error, res, args)
    else:
        problems, warnings = check_video(folder, res, args)
    if res.error:
        problems.insert(0, res.error)
    warnings = res.warnings + warnings + rate_warnings(res, args)
    session = write_session_json(folder, args, device_info, calib_info, res, files, tar, problems, warnings)
    print("Session manifest: {}".format(session.relative_to(folder).as_posix()), flush=True)

    if warnings:
        print("\nWARNINGS (recorded in session.json):", file=sys.stderr, flush=True)
        for warning in warnings:
            print("  - {}".format(warning), file=sys.stderr, flush=True)

    if problems:
        print("\nRECORDING INCOMPLETE:", file=sys.stderr, flush=True)
        for problem in problems:
            print("  - {}".format(problem), file=sys.stderr, flush=True)
        print(retry_hint(args), file=sys.stderr, flush=True)
        print("\nReplay (may fail with an incomplete recording):\n  {}".format(
            replay_command(folder, args.mode, tar, imu=not args.no_imu)), flush=True)
        return EXIT_INCOMPLETE

    print("\nRecording complete{}. Replay it with:\n  {}".format(
        " (stopped with Ctrl-C)" if res.interrupted else "",
        replay_command(folder, args.mode, tar, imu=not args.no_imu)), flush=True)
    return EXIT_OK


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:       # Ctrl-C while finishing up (listing, packing): no traceback
        print("\nInterrupted.", file=sys.stderr, flush=True)
        sys.exit(EXIT_INCOMPLETE)
