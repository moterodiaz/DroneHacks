#!/usr/bin/env python3
"""Stereo depth test for the OAK-D S2 (DepthAI v3, depthai==3.10.0).

WHAT THIS TEST CHECKS
    * The CAM_B/CAM_C mono pair (OV9282, 7.5 cm baseline) streams through a ``StereoDepth``
      node and metric depth comes back as uint16 millimetres (0 = invalid) at the requested
      frame rate.
    * The StereoDepth presets and the classic knobs - left-right check, subpixel, extended
      disparity, median filter, confidence threshold, decimation - can be applied, and the
      effective configuration is read back from ``stereo.initialConfig`` and printed.
    * Depth values are plausible: once per second the script prints FPS, the valid-pixel
      fraction, min / median / max valid depth and the median depth of a small ROI at the
      image centre.  Point the camera at a flat wall at a known distance and compare.
    * Optionally (``--align-rgb``) depth is aligned to the colour camera CAM_A by linking an
      undistorted colour stream into ``stereo.inputAlignTo`` and blended over the colour image.

HOW TO RUN (from the repository root)
    python camera/02_stereo_depth.py                          # windowed, DEFAULT preset, 640x400 @ 30 fps
    python camera/02_stereo_depth.py --preset ROBOTICS --no-lrcheck --median 5 --decimation 1
    python camera/02_stereo_depth.py --headless --duration 10 --snapshot-on-exit
    python camera/02_stereo_depth.py --align-rgb --show-disparity
    python camera/02_stereo_depth.py --help

WHAT YOU SHOULD SEE
    A colour-mapped depth window (red = near, blue = far at --max-depth, black = invalid) that
    updates at roughly --fps.  A green box marks the centre ROI and its median depth, a yellow
    box follows the mouse and shows the depth under the cursor (median of the same ROI).  The
    console prints one status line per second, e.g.
        [   3.0 s]  29.8 fps | valid  82.4% | min 0.412 m med 1.503 m max 4.870 m | center(160,100) roi 5px: 1.498 m
    A flat wall 1.5 m away should read ~1.45-1.55 m in the centre.  The centre is (160,100)
    because the DEFAULT, ROBOTICS, HIGH_DETAIL and FACE presets enable a decimation factor of
    2, so the depth image is 320x200 for a 640x400 mono request (it is shown upscaled); pass
    ``--decimation 1`` for full-resolution depth and center(320,200).  In v3 the confidence
    threshold is inverted compared with v2: 255 is the most confident value and a HIGHER
    threshold gives FEWER valid pixels (presets use 15 / 55).
    ``--duration`` starts counting after the device has booted and the 1.5 s auto-exposure
    warm-up, so ``--duration 10`` streams for 10 s and the process lives ~11.5 s plus boot.
    ``--roi`` must be odd so the box is centred on the reported pixel; an even value is
    rounded up to the next odd size (printed in the settings block).

KEYS (windowed mode)
    q / Esc   quit
    s         save snapshots/depth_<stamp>.png (16-bit, millimetres) and
              snapshots/depth_color_<stamp>.png (colourised); with --align-rgb also
              depth_rgb_<stamp>.png (blend), with --show-disparity also disparity_color_<stamp>.png
    --snapshot-on-exit saves the same files when the script ends (also in --headless mode).

EXIT CODES
    0  normal end (--duration elapsed, q pressed, or Ctrl-C)
    1  the pipeline could not be built/started (e.g. unsupported --width/--height), stopped
       unexpectedly (device crash / disconnect), or no window could be opened; one line on stderr
    2  no device / wrong --device-id (one line on stderr, no traceback)
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402  (import after sys.path tweak)
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
    timestamp_str,
    wait_key_quit,
    warm_up,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SNAPSHOT_DIR = REPO_ROOT / "snapshots"

DEPTH_WINDOW = "depth  (q quit, s snapshot)"
DISPARITY_WINDOW = "disparity"

#: PresetMode members of the installed depthai (3.10.0: FAST_ACCURACY, FAST_DENSITY, DEFAULT,
#: FACE, HIGH_DETAIL, ROBOTICS, DENSITY, ACCURACY).
PRESET_NAMES: List[str] = list(dai.node.StereoDepth.PresetMode.__members__)

MEDIAN_CHOICES: Dict[str, dai.MedianFilter] = {
    "OFF": dai.MedianFilter.MEDIAN_OFF,
    "3": dai.MedianFilter.KERNEL_3x3,
    "5": dai.MedianFilter.KERNEL_5x5,
    "7": dai.MedianFilter.KERNEL_7x7,
}

GET_TIMEOUT = dt.timedelta(milliseconds=500)   # MessageQueue.get(timeout) returns None on timeout
WARM_UP_SECONDS = 1.5                            # auto-exposure settle (first ~1 s of depth is all zero)
STALL_WARN_SECONDS = 5.0
BLEND_ALPHA = 0.5                                # weight of the colourised depth in the --align-rgb blend

EXIT_OK = 0
EXIT_PIPELINE_STOPPED = 1


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Build the argument parser and parse *argv* (``None`` = ``sys.argv[1:]``)."""
    parser = argparse.ArgumentParser(
        description="Stream stereo depth from the OAK-D S2 mono pair (CAM_B/CAM_C) through "
                    "StereoDepth, show it colourised and print depth statistics once per second.",
        epilog="Keys (windowed): q/Esc quit, s save a 16-bit depth PNG + colourised PNG into "
               "the snapshot directory.  Options left at their default keep the preset's value.  "
               "--duration starts after device boot and the {:.1f} s auto-exposure warm-up.".format(
                   WARM_UP_SECONDS),
    )
    add_device_arg(parser)
    add_run_args(parser)

    stereo = parser.add_argument_group("stereo settings")
    stereo.add_argument("--preset", choices=PRESET_NAMES, default="DEFAULT",
                        help="StereoDepth preset (default: DEFAULT)")
    stereo.add_argument("--lrcheck", action=argparse.BooleanOptionalAction, default=None,
                        help="enable/disable left-right check (default: preset value)")
    stereo.add_argument("--subpixel", action=argparse.BooleanOptionalAction, default=None,
                        help="enable/disable subpixel disparity (default: preset value)")
    stereo.add_argument("--subpixel-bits", type=int, choices=(3, 4, 5), default=None,
                        metavar="{3,4,5}",
                        help="subpixel fractional bits; the median filter only works with 3 "
                             "(default: preset value)")
    stereo.add_argument("--extended", action=argparse.BooleanOptionalAction, default=None,
                        help="enable/disable extended disparity (0-190 px, closer minimum range; "
                             "default: preset value)")
    stereo.add_argument("--median", type=lambda s: str(s).upper(), choices=list(MEDIAN_CHOICES),
                        default=None, metavar="{OFF,3,5,7}",
                        help="median filter kernel: OFF, 3, 5 or 7 (default: preset value)")
    stereo.add_argument("--confidence", type=int, default=None, metavar="0-255",
                        help="confidence threshold; v3 semantics: 255 = most confident, higher = "
                             "fewer valid pixels (default: preset value, 15 or 55)")
    stereo.add_argument("--decimation", type=int, choices=(1, 2, 3, 4), default=None,
                        metavar="{1,2,3,4}",
                        help="decimation factor of the depth output (presets DEFAULT/ROBOTICS/"
                             "HIGH_DETAIL/FACE use 2 -> half-size depth; default: preset value)")

    streams = parser.add_argument_group("streams")
    streams.add_argument("--fps", type=float, default=30.0, metavar="FPS",
                         help="mono (and colour) frame rate (default: 30)")
    streams.add_argument("--width", type=int, default=640, metavar="PX",
                         help="mono stream width (default: 640)")
    streams.add_argument("--height", type=int, default=400, metavar="PX",
                         help="mono stream height (default: 400)")
    streams.add_argument("--align-rgb", action="store_true",
                         help="align depth to CAM_A (undistorted colour stream linked into "
                              "stereo.inputAlignTo) and blend it over the colour image")
    streams.add_argument("--show-disparity", action="store_true",
                         help="also show (and snapshot) the colourised disparity map")

    view = parser.add_argument_group("measurement / display")
    view.add_argument("--roi", type=int, default=5, metavar="PX",
                      help="side of the square ROI (pixels, odd; an even value is rounded up to "
                           "the next odd size) whose median is reported at the image centre and "
                           "under the mouse cursor (default: 5)")
    view.add_argument("--max-depth", type=float, default=5.0, metavar="METERS",
                      help="depth mapped to the far (blue) end of the colour map (default: 5.0)")
    view.add_argument("--snapshot-on-exit", action="store_true",
                      help="save the last depth frame (16-bit PNG + colourised PNG) when the "
                           "script ends; works in --headless mode too")
    view.add_argument("--snapshot-dir", default=str(DEFAULT_SNAPSHOT_DIR), metavar="DIR",
                      help="where snapshots are written (default: <repo>/snapshots)")

    args = parser.parse_args(argv)

    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.roi < 1:
        parser.error("--roi must be at least 1 pixel")
    if args.roi % 2 == 0:
        # roi_median_m / draw_roi_box use a (2*half+1)-pixel patch centred on the pixel, so an
        # even size would silently measure one pixel more than the labels claim.
        print("NOTE: --roi {} is even; using the next odd size {} so the ROI is centred.".format(
            args.roi, args.roi + 1), file=sys.stderr, flush=True)
        args.roi += 1
    if args.max_depth <= 0:
        parser.error("--max-depth must be positive")
    if args.confidence is not None and not 0 <= args.confidence <= 255:
        parser.error("--confidence must be in 0..255")
    if args.duration < 0:
        parser.error("--duration must be >= 0")
    return args


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

class DepthStats(NamedTuple):
    """Per-frame depth statistics in metres (``None`` when there is no valid pixel)."""

    valid_fraction: float
    min_m: Optional[float]
    median_m: Optional[float]
    max_m: Optional[float]
    center_m: Optional[float]
    center_xy: Tuple[int, int]


class LastFrames:
    """The most recent frames, kept for snapshots (``s`` key and ``--snapshot-on-exit``)."""

    def __init__(self) -> None:
        self.depth_mm: Optional[np.ndarray] = None
        self.rgb_bgr: Optional[np.ndarray] = None
        self.disparity: Optional[np.ndarray] = None


class MouseState:
    """Last known mouse position inside the depth window (window pixel coordinates)."""

    def __init__(self) -> None:
        self.x: int = -1
        self.y: int = -1

    def callback(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        """``cv2.setMouseCallback`` target: remember the pointer position."""
        if event == cv2.EVENT_MOUSEMOVE:
            self.x, self.y = int(x), int(y)


class FrameMatcher:
    """Keep the last few colour frames and pick the one closest in time to a depth frame."""

    def __init__(self, keep: int = 8) -> None:
        self._frames: Deque[dai.ImgFrame] = deque(maxlen=max(1, keep))

    def push_all(self, frames: Iterable[dai.ImgFrame]) -> None:
        """Remember newly arrived colour frames."""
        for frame in frames:
            self._frames.append(frame)

    def closest(self, timestamp: dt.timedelta) -> Optional[dai.ImgFrame]:
        """Return the remembered frame whose timestamp is nearest to *timestamp*."""
        best: Optional[dai.ImgFrame] = None
        best_delta: Optional[float] = None
        for frame in self._frames:
            delta = abs((frame.getTimestamp() - timestamp).total_seconds())
            if best_delta is None or delta < best_delta:
                best, best_delta = frame, delta
        return best


def fmt_m(value: Optional[float]) -> str:
    """Format metres with 3 decimals, ``n/a`` for None."""
    return "n/a" if value is None else "{:.3f} m".format(value)


def roi_median_m(depth_mm: np.ndarray, x: int, y: int, roi_px: int) -> Optional[float]:
    """Median depth (metres) of the valid pixels in a ``roi_px`` square centred on (x, y).

    ``roi_px`` is expected to be odd (parse_args() rounds even values up); the patch is
    ``2 * (roi_px // 2) + 1`` pixels wide, clipped at the image border.
    """
    height, width = depth_mm.shape[:2]
    if not (0 <= x < width and 0 <= y < height):
        return None
    half = max(0, int(roi_px) // 2)
    x0, x1 = max(0, x - half), min(width, x + half + 1)
    y0, y1 = max(0, y - half), min(height, y + half + 1)
    patch = depth_mm[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0


def compute_stats(depth_mm: np.ndarray, roi_px: int) -> DepthStats:
    """Valid-pixel fraction, min/median/max of valid depth and the centre-ROI median (metres)."""
    height, width = depth_mm.shape[:2]
    center = (width // 2, height // 2)
    valid = depth_mm[depth_mm > 0]
    fraction = float(valid.size) / float(depth_mm.size) if depth_mm.size else 0.0
    if valid.size:
        min_m: Optional[float] = float(valid.min()) / 1000.0
        median_m: Optional[float] = float(np.median(valid)) / 1000.0
        max_m: Optional[float] = float(valid.max()) / 1000.0
    else:
        min_m = median_m = max_m = None
    return DepthStats(fraction, min_m, median_m, max_m,
                      roi_median_m(depth_mm, center[0], center[1], roi_px), center)


def stats_line(elapsed_s: float, fps: float, stats: DepthStats, roi_px: int) -> str:
    """One console status line (printed once per second in both modes)."""
    cx, cy = stats.center_xy
    return ("[{:6.1f} s] {:5.1f} fps | valid {:5.1f}% | min {} med {} max {} | "
            "center({},{}) roi {}px: {}".format(
                elapsed_s, fps, stats.valid_fraction * 100.0,
                fmt_m(stats.min_m), fmt_m(stats.median_m), fmt_m(stats.max_m),
                cx, cy, roi_px, fmt_m(stats.center_m)))


def blend_rgb(color_bgr: np.ndarray, depth_color: np.ndarray, alpha: float = BLEND_ALPHA) -> np.ndarray:
    """Blend the colourised depth over the colour image (resized to the depth size if needed)."""
    if color_bgr.ndim == 2:
        color_bgr = cv2.cvtColor(color_bgr, cv2.COLOR_GRAY2BGR)
    if color_bgr.shape[:2] != depth_color.shape[:2]:
        color_bgr = cv2.resize(color_bgr, (depth_color.shape[1], depth_color.shape[0]),
                               interpolation=cv2.INTER_LINEAR)
    return cv2.addWeighted(color_bgr, 1.0 - alpha, depth_color, alpha, 0.0)


def unique_path(directory: Path, stem: str, ext: str) -> Path:
    """``directory/stem.ext`` or ``stem_2.ext``, ``stem_3.ext``, ... if it already exists."""
    candidate = directory / "{}.{}".format(stem, ext)
    index = 2
    while candidate.exists():
        candidate = directory / "{}_{}.{}".format(stem, index, ext)
        index += 1
    return candidate


def save_snapshot(out_dir: Path, last: LastFrames, max_depth_mm: float,
                  max_disparity: float) -> List[Path]:
    """Write the last depth frame as 16-bit PNG (mm) + colourised PNG (+ blend / disparity)."""
    if last.depth_mm is None:
        print("No depth frame received yet - nothing to save.", file=sys.stderr, flush=True)
        return []
    ensure_dir(out_dir)
    stamp = timestamp_str()
    depth16 = last.depth_mm.astype(np.uint16, copy=False)
    color = colorize_depth(depth16, max_depth_mm)
    files: List[Tuple[Path, np.ndarray]] = [
        (unique_path(out_dir, "depth_{}".format(stamp), "png"), depth16),
        (unique_path(out_dir, "depth_color_{}".format(stamp), "png"), color),
    ]
    if last.rgb_bgr is not None:
        files.append((unique_path(out_dir, "depth_rgb_{}".format(stamp), "png"),
                      blend_rgb(last.rgb_bgr, color)))
    if last.disparity is not None:
        files.append((unique_path(out_dir, "disparity_color_{}".format(stamp), "png"),
                      colorize_disparity(last.disparity, max_disparity)))
    saved: List[Path] = []
    for path, image in files:
        if cv2.imwrite(str(path), image):
            saved.append(path)
            print("Saved {}".format(path), flush=True)
        else:
            print("Failed to write {}".format(path), file=sys.stderr, flush=True)
    return saved


def draw_roi_box(img: np.ndarray, x: int, y: int, roi_px: int, scale: float,
                 color: Tuple[int, int, int]) -> None:
    """Outline the ROI centred on depth pixel (x, y) on the (possibly upscaled) view."""
    half = max(0, int(roi_px) // 2)
    x0 = int(round((x - half) * scale))
    y0 = int(round((y - half) * scale))
    x1 = int(round((x + half + 1) * scale)) - 1
    y1 = int(round((y + half + 1) * scale)) - 1
    cv2.rectangle(img, (x0, y0), (max(x0, x1), max(y0, y1)), color, 1, cv2.LINE_AA)


# --------------------------------------------------------------------------------------
# Pipeline
# --------------------------------------------------------------------------------------

def build_pipeline(pipeline: dai.Pipeline, args: argparse.Namespace,
                   preset: dai.node.StereoDepth.PresetMode
                   ) -> Tuple[dai.node.StereoDepth, Dict[str, dai.MessageQueue]]:
    """Create CAM_B/CAM_C -> StereoDepth (and optionally CAM_A for alignment); return queues."""
    size = (int(args.width), int(args.height))
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_out = left.requestOutput(size, fps=args.fps)      # default mono type RAW8
    right_out = right.requestOutput(size, fps=args.fps)

    stereo = pipeline.create(dai.node.StereoDepth).build(left=left_out, right=right_out,
                                                         presetMode=preset)
    stereo.setRectification(True)
    # Node setters (as in the official StereoDepth examples) - only when the user asked.
    if args.lrcheck is not None:
        stereo.setLeftRightCheck(bool(args.lrcheck))
    if args.subpixel is not None:
        stereo.setSubpixel(bool(args.subpixel))
    if args.subpixel_bits is not None:
        stereo.setSubpixelFractionalBits(int(args.subpixel_bits))
    if args.extended is not None:
        stereo.setExtendedDisparity(bool(args.extended))
    # These live on initialConfig in 3.10.0 (no setMedianFilter/setConfidenceThreshold on the node).
    if args.median is not None:
        stereo.initialConfig.setMedianFilter(MEDIAN_CHOICES[args.median])
    if args.confidence is not None:
        stereo.initialConfig.setConfidenceThreshold(int(args.confidence))
    if args.decimation is not None:
        stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = int(args.decimation)

    queues: Dict[str, dai.MessageQueue] = {}
    if args.align_rgb:
        cam_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_out = cam_a.requestOutput(size, dai.ImgFrame.Type.RGB888i, fps=args.fps,
                                      enableUndistortion=True)
        rgb_out.link(stereo.inputAlignTo)          # v3 alignment: depth takes CAM_A's perspective
        queues["rgb"] = rgb_out.createOutputQueue(maxSize=4, blocking=False)
    queues["depth"] = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
    if args.show_disparity:
        queues["disparity"] = stereo.disparity.createOutputQueue(maxSize=4, blocking=False)
    return stereo, queues


def describe_settings(stereo: dai.node.StereoDepth, args: argparse.Namespace) -> str:
    """Multi-line description of the effective StereoDepth configuration (from initialConfig)."""
    cfg = stereo.initialConfig
    pp = cfg.postProcessing
    lines = [
        "Stereo settings (effective, read back from stereo.initialConfig):",
        "  preset={} mono={}x{} @ {:g} fps  align_rgb={}  show_disparity={}".format(
            args.preset, args.width, args.height, args.fps, args.align_rgb, args.show_disparity),
        "  lrcheck={} subpixel={} (fractional bits {}) extended={}".format(
            cfg.getLeftRightCheck(), cfg.getSubpixel(), cfg.getSubpixelFractionalBits(),
            cfg.getExtendedDisparity()),
        "  median={} confidence={} (255 = most confident; higher = fewer valid px) maxDisparity={:g}".format(
            cfg.getMedianFilter().name, cfg.getConfidenceThreshold(), cfg.getMaxDisparity()),
        "  decimation={} spatial={} temporal={} speckle={} threshold={}-{} mm depthUnit={}".format(
            pp.decimationFilter.decimationFactor, pp.spatialFilter.enable, pp.temporalFilter.enable,
            pp.speckleFilter.enable, pp.thresholdFilter.minRange, pp.thresholdFilter.maxRange,
            cfg.getDepthUnit().name),
        "  roi={} px  colour map max depth={:g} m  snapshots -> {}".format(
            args.roi, args.max_depth, args.snapshot_dir),
    ]
    return "\n".join(lines)


def short_settings(stereo: dai.node.StereoDepth, preset_name: str) -> str:
    """Compact settings string for the window overlay."""
    cfg = stereo.initialConfig
    return "{} lr={} sub={} ext={} med={}".format(
        preset_name, int(cfg.getLeftRightCheck()), int(cfg.getSubpixel()),
        int(cfg.getExtendedDisparity()), cfg.getMedianFilter().name.replace("KERNEL_", ""))


def warn_about_median(stereo: dai.node.StereoDepth, args: argparse.Namespace) -> None:
    """The firmware disables the median filter when the disparity range exceeds 1024 values."""
    cfg = stereo.initialConfig
    if cfg.getMedianFilter() == dai.MedianFilter.MEDIAN_OFF:
        return
    if cfg.getSubpixel() and cfg.getSubpixelFractionalBits() != 3:
        print("NOTE: median filter requested with subpixel fractional bits = {}; the firmware "
              "disables the median filter unless the bits are 3 (pass --subpixel-bits 3)."
              .format(cfg.getSubpixelFractionalBits()), file=sys.stderr, flush=True)
    elif cfg.getSubpixel() and cfg.getExtendedDisparity():
        print("NOTE: median filter with subpixel + extended disparity exceeds the filter's "
              "1024-value range; the firmware will disable it.", file=sys.stderr, flush=True)


def read_calibration_hint(device: dai.Device, width: int, height: int
                          ) -> Optional[Tuple[float, float]]:
    """Return (baseline_cm, fx_px of CAM_C at width x height) or None if not readable."""
    try:
        calib = device.readCalibration()
        baseline_cm = float(calib.getBaselineDistance())          # centimetres by default
        matrix = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, width, height)
        return baseline_cm, float(matrix[0][0])
    except Exception as exc:  # noqa: BLE001 - calibration is informational only
        print("Calibration not readable ({}: {}) - continuing.".format(type(exc).__name__, exc),
              file=sys.stderr, flush=True)
        return None


def print_calibration_hint(hint: Optional[Tuple[float, float]], stereo: dai.node.StereoDepth,
                           width: int, height: int) -> None:
    """Print baseline / focal length and the theoretical minimum range for the chosen mode."""
    if hint is None:
        return
    baseline_cm, fx = hint
    max_px = 190 if stereo.initialConfig.getExtendedDisparity() else 95
    fxb_m = fx * baseline_cm / 100.0
    print("Calibration: baseline {:.2f} cm, CAM_C fx @ {}x{} = {:.1f} px -> depth[m] ~ {:.2f} / "
          "disparity[px]; theoretical minimum range ~{:.2f} m ({} px max disparity)".format(
              baseline_cm, width, height, fx, fxb_m, fxb_m / max_px, max_px), flush=True)


# --------------------------------------------------------------------------------------
# Streaming loop
# --------------------------------------------------------------------------------------

def render(depth_mm: np.ndarray, stats: DepthStats, args: argparse.Namespace, last: LastFrames,
           mouse: MouseState, fps_value: float, max_disparity: float, overlay: str
           ) -> Optional[str]:
    """Draw the depth (or blend) window, the disparity window, then pump keys; return key."""
    color = colorize_depth(depth_mm, args.max_depth * 1000.0)
    view = blend_rgb(last.rgb_bgr, color) if (args.align_rgb and last.rgb_bgr is not None) else color
    height, width = depth_mm.shape[:2]
    scale = max(1.0, float(args.width) / float(width))     # upscale decimated depth for viewing
    if scale != 1.0:
        view = cv2.resize(view, (int(round(width * scale)), int(round(height * scale))),
                          interpolation=cv2.INTER_NEAREST)
    else:
        view = np.ascontiguousarray(view)

    cursor_text = "cursor: move the mouse over the image"
    if 0 <= mouse.x < view.shape[1] and 0 <= mouse.y < view.shape[0]:
        dx = min(width - 1, int(mouse.x / scale))
        dy = min(height - 1, int(mouse.y / scale))
        cursor_text = "cursor ({},{}): {}".format(dx, dy, fmt_m(roi_median_m(depth_mm, dx, dy, args.roi)))
        draw_roi_box(view, dx, dy, args.roi, scale, (0, 255, 255))
    cx, cy = stats.center_xy
    draw_roi_box(view, cx, cy, args.roi, scale, (0, 255, 0))

    text = "\n".join([
        "{:.1f} fps  valid {:.0f}%  med {}  {}".format(fps_value, stats.valid_fraction * 100.0,
                                                       fmt_m(stats.median_m), overlay),
        "center ({},{}) roi {}px: {}".format(cx, cy, args.roi, fmt_m(stats.center_m)),
        cursor_text,
    ])
    draw_label(view, text, (10, 22), 0.55)
    cv2.imshow(DEPTH_WINDOW, view)
    if args.show_disparity and last.disparity is not None:
        cv2.imshow(DISPARITY_WINDOW, colorize_disparity(last.disparity, max_disparity))
    return wait_key_quit(1)


def stream(pipeline: dai.Pipeline, queues: Dict[str, dai.MessageQueue], args: argparse.Namespace,
           last: LastFrames, max_disparity: float, overlay: str) -> int:
    """Main loop: fetch depth, compute stats, print once per second, draw, handle keys."""
    depth_q = queues["depth"]
    rgb_q = queues.get("rgb")
    disp_q = queues.get("disparity")
    matcher = FrameMatcher() if rgb_q is not None else None
    mouse = MouseState()

    if not args.headless:
        try:
            cv2.namedWindow(DEPTH_WINDOW)
            cv2.setMouseCallback(DEPTH_WINDOW, mouse.callback)
            if disp_q is not None:
                cv2.namedWindow(DISPARITY_WINDOW)
        except cv2.error as exc:
            print("Cannot open an OpenCV window ({}); use --headless.".format(
                " ".join(str(exc).split())[:200]), file=sys.stderr, flush=True)
            return EXIT_PIPELINE_STOPPED

    clock = RunClock(args.duration)
    fps = FpsCounter()
    ticker = Periodic(1.0)
    snapshot_dir = Path(args.snapshot_dir)
    first_frame = True
    last_rx = time.monotonic()
    stall_warned = False
    exit_code = EXIT_OK

    while not clock.expired():
        if not pipeline.isRunning():
            print("Pipeline stopped unexpectedly (device disconnected or crashed).",
                  file=sys.stderr, flush=True)
            exit_code = EXIT_PIPELINE_STOPPED
            break
        try:
            msg = depth_q.get(GET_TIMEOUT)
        except dai.MessageQueue.QueueException:
            print("Depth queue closed - the pipeline stopped unexpectedly.", file=sys.stderr, flush=True)
            exit_code = EXIT_PIPELINE_STOPPED
            break

        if msg is None:
            if not stall_warned and time.monotonic() - last_rx > STALL_WARN_SECONDS:
                print("No depth frame for {:.0f} s - is the camera streaming?".format(
                    STALL_WARN_SECONDS), file=sys.stderr, flush=True)
                stall_warned = True
            if not args.headless:
                key = wait_key_quit(1)
                if key == "q":
                    break
                if key == "s":
                    save_snapshot(snapshot_dir, last, args.max_depth * 1000.0, max_disparity)
            continue

        last_rx = time.monotonic()
        stall_warned = False
        depth_mm = msg.getFrame()
        if depth_mm.ndim != 2:
            if first_frame:
                print("Unexpected depth frame shape {} (type {}) - skipping.".format(
                    depth_mm.shape, msg.getType()), file=sys.stderr, flush=True)
                first_frame = False
            continue
        if first_frame:
            first_frame = False
            print("First depth frame: {}x{} {} -> numpy {} {}".format(
                msg.getWidth(), msg.getHeight(), msg.getType().name, depth_mm.dtype, depth_mm.shape),
                flush=True)
        last.depth_mm = depth_mm
        fps.tick()

        if rgb_q is not None and matcher is not None:
            matcher.push_all(rgb_q.tryGetAll())
            rgb_frame = matcher.closest(msg.getTimestamp())
            if rgb_frame is not None:
                last.rgb_bgr = rgb_frame.getCvFrame()
        if disp_q is not None:
            for disp_msg in disp_q.tryGetAll():
                last.disparity = disp_msg.getFrame()

        stats = compute_stats(depth_mm, args.roi)
        if ticker.ready():
            print(stats_line(clock.elapsed(), fps.fps(), stats, args.roi), flush=True)

        if not args.headless:
            key = render(depth_mm, stats, args, last, mouse, fps.fps(), max_disparity, overlay)
            if key == "q":
                break
            if key == "s":
                save_snapshot(snapshot_dir, last, args.max_depth * 1000.0, max_disparity)

    elapsed = clock.elapsed()
    avg = fps.count / elapsed if elapsed > 0 else 0.0
    print("Done: {} depth frames in {:.1f} s (avg {:.1f} fps).".format(fps.count, elapsed, avg),
          flush=True)
    return exit_code


# --------------------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    """Open the device, run the stereo pipeline, return the process exit code."""
    args = parse_args(argv)
    preset = dai.node.StereoDepth.PresetMode.__members__[args.preset]
    last = LastFrames()
    max_disparity = 1.0
    exit_code = EXIT_OK

    try:
        # A wrong --device-id is a user error: fail fast with one line; without an id allow
        # a few retries in case the camera is still rebooting after a previous script.
        device = open_device(args.device_id, retries=1 if args.device_id else 3)
        print_device_summary(device)
        calib_hint = read_calibration_hint(device, args.width, args.height)

        with dai.Pipeline(device) as pipeline:
            stereo, queues = build_pipeline(pipeline, args, preset)
            print(describe_settings(stereo, args), flush=True)
            warn_about_median(stereo, args)
            print_calibration_hint(calib_hint, stereo, args.width, args.height)
            max_disparity = float(stereo.initialConfig.getMaxDisparity())
            overlay = short_settings(stereo, args.preset)

            pipeline.start()
            print("Pipeline started - letting auto-exposure settle for {:.1f} s ...".format(
                WARM_UP_SECONDS), flush=True)
            warm_up(*queues.values(), seconds=WARM_UP_SECONDS)
            exit_code = stream(pipeline, queues, args, last, max_disparity, overlay)
            pipeline.stop()
    except KeyboardInterrupt:
        print("\nInterrupted (Ctrl-C) - stopping.", flush=True)
    except RuntimeError as exc:
        # depthai raises RuntimeError (dai.XLinkError / XLinkReadError / XLinkWriteError /
        # EepromError subclass it) for an unsupported stream size, a device that disconnected
        # between open_device() and pipeline.start(), etc.  One readable line, no traceback.
        print("Failed to build/start the stereo pipeline: {}: {}".format(
            type(exc).__name__, " ".join(str(exc).split())[:400]), file=sys.stderr, flush=True)
        exit_code = EXIT_PIPELINE_STOPPED
    finally:
        if not args.headless:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    if args.snapshot_on_exit:
        save_snapshot(Path(args.snapshot_dir), last, args.max_depth * 1000.0, max_disparity)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
