#!/usr/bin/env python3
"""Basic obstacle detection with the OAK-D S2: depth -> nearest obstacle per 5 degree sector.

WHAT THIS DOES
    Runs on-device stereo depth (CAM_B/CAM_C at 640x400, the ROBOTICS preset with decimation 1
    and a 5x5 median so thin poles survive, 60 Hz anti-banding) and the raw accelerometer in one
    pipeline.  Each depth frame is levelled with the low-passed gravity direction and reduced by
    ``cuvslam_oak.obstacles.detect`` to
      * the horizontal range to the nearest obstacle in each 5 deg bearing sector (72 around the
        vehicle, about 16 inside the camera's view): obstacle / clear / unknown, where holes in
        the depth count as unknown, never as free;
      * the nearest obstacle in a 0.7 m wide corridor straight ahead ("ahead").
    Only points within --band-below / --band-above of the camera height count, so the floor and
    ceiling drop out.  ``--replay`` runs the same reduction on a ``camera/05_record.py --mode
    video`` recording (depth PNGs + session.json) without the camera.

HOW TO RUN (from the repository root)
    python methods/obstacle-detection/11_obstacle_detect.py                         # live, windowed
    python methods/obstacle-detection/11_obstacle_detect.py --headless --duration 15
    python methods/obstacle-detection/11_obstacle_detect.py --replay recordings/video_<stamp>
    python methods/obstacle-detection/11_obstacle_detect.py --replay recordings/video_<stamp> --headless --pitch-deg 10

WHAT YOU SHOULD SEE
    Left: the colour-mapped depth (red near, blue at --max-range, black invalid) with a bar
    along the top edge, one segment per sector: red < 1 m, orange < 2 m, yellow obstacle
    farther away, green clear, grey unknown.  Right: a top-down view with the camera at the
    bottom centre, a ring per metre, each sector drawn out to its obstacle and the corridor
    ahead.  The console prints one line per second, e.g.
        [   3.0 s]  29.9 fps  proc  3.1 ms | valid 61% | nearest 1.23 m @ +15 | ahead 2.10 m | view 9 obst 4 clear 3 unk | tilt  2.1 | chip 51 C
    A wall 1.5 m straight ahead should give "ahead 1.50 m" and the sectors around 0 deg at
    about 1.5 m; a broom handle held 1 m away should show up in its own sector.

KEYS (windowed mode)
    q / Esc   quit

LEVELLING
    Live, the accelerometer gives the gravity direction (right for hand-held tests; a flying
    multirotor's accelerometer measures thrust, so in flight the attitude must come from PX4 or
    cuVSLAM).  ``--no-imu`` and ``--replay`` use a fixed tilt from --pitch-deg / --roll-deg
    (default: level).

EXIT CODES
    0  normal end (--duration elapsed, q pressed, Ctrl-C, end of the recording)
    1  the pipeline could not be built/started or stopped unexpectedly; no window could be opened
    2  no device / wrong --device-id, or the --replay folder is missing or not a video recording
    4  stopped by the chip-temperature guard (--max-chip-temp)
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Iterator, List, NamedTuple, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "camera"))   # camera/common.py
sys.path.insert(0, str(REPO_ROOT))              # cuvslam_oak package

from common import (  # noqa: E402  (import after sys.path tweak)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    colorize_depth,
    draw_label,
    open_device,
    print_device_summary,
    wait_key_quit,
    warm_up,
)
from cuvslam_oak import obstacles  # noqa: E402
from cuvslam_oak import rig as oakrig  # noqa: E402

WINDOW = "obstacles  (q quit)"
SIZE = (640, 400)
IMU_RATE_HZ = 100
GET_TIMEOUT = dt.timedelta(milliseconds=500)
WARM_UP_SECONDS = 1.5
STALL_WARN_SECONDS = 5.0
RADAR_PX = SIZE[1]                              # square top-down view beside the depth image
BAR_PX = 12

MEDIAN_CHOICES: Dict[str, dai.MedianFilter] = {
    "OFF": dai.MedianFilter.MEDIAN_OFF,
    "3": dai.MedianFilter.KERNEL_3x3,
    "5": dai.MedianFilter.KERNEL_5x5,
    "7": dai.MedianFilter.KERNEL_7x7,
}

# BGR
COLOR_NEAR = (40, 40, 230)
COLOR_MID = (0, 150, 255)
COLOR_FAR = (0, 220, 230)
COLOR_CLEAR = (60, 170, 60)
COLOR_UNKNOWN = (110, 110, 110)

EXIT_OK = 0
EXIT_PIPELINE_STOPPED = 1
EXIT_BAD_INPUT = 2
EXIT_TOO_HOT = 4


class DepthFrame(NamedTuple):
    depth_mm: np.ndarray
    intrinsics: np.ndarray
    down: Optional[np.ndarray]


class RecordingError(Exception):
    """The --replay folder is missing or is not a 05_record.py video recording."""


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Build the argument parser and parse *argv* (``None`` = ``sys.argv[1:]``)."""
    defaults = obstacles.ObstacleConfig()
    parser = argparse.ArgumentParser(
        description="Reduce OAK-D S2 stereo depth to the nearest obstacle per 5 degree bearing sector "
                    "and in a corridor straight ahead, live or on a 05_record.py video recording.",
        epilog="--duration starts after device boot and the {:.1f} s auto-exposure warm-up.".format(WARM_UP_SECONDS),
    )
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument("--replay", metavar="FOLDER", default=None,
                        help="run on recordings/video_<stamp> (camera/05_record.py --mode video) instead of the camera")

    det = parser.add_argument_group("detection")
    det.add_argument("--max-range", type=float, default=defaults.max_range_m, metavar="M",
                     help="obstacles farther than this make a sector 'clear' (default: %(default)s)")
    det.add_argument("--band-below", type=float, default=defaults.band_below_m, metavar="M",
                     help="count points down to this far below the camera (default: %(default)s)")
    det.add_argument("--band-above", type=float, default=defaults.band_above_m, metavar="M",
                     help="count points up to this far above the camera (default: %(default)s)")
    det.add_argument("--min-points", type=int, default=defaults.min_points, metavar="N",
                     help="sampled points one surface needs within 15 cm of range (default: %(default)s)")
    det.add_argument("--stride", type=int, default=defaults.stride, metavar="N",
                     help="use every N-th pixel in both directions (default: %(default)s)")
    det.add_argument("--corridor-width", type=float, default=2.0 * defaults.corridor_half_width_m, metavar="M",
                     help="width of the corridor straight ahead (default: %(default)s)")

    tilt = parser.add_argument_group("levelling")
    tilt.add_argument("--no-imu", action="store_true",
                      help="live: do not level with the accelerometer, use --pitch-deg / --roll-deg")
    tilt.add_argument("--pitch-deg", type=float, default=0.0,
                      help="fixed camera pitch, nose down positive (--no-imu / --replay; default: 0)")
    tilt.add_argument("--roll-deg", type=float, default=0.0,
                      help="fixed camera roll, right side down positive (--no-imu / --replay; default: 0)")

    cam = parser.add_argument_group("camera (live)")
    cam.add_argument("--fps", type=float, default=30.0, help="mono camera frame rate (default: %(default)s)")
    cam.add_argument("--median", choices=list(MEDIAN_CHOICES), default="5",
                     help="StereoDepth median filter (default: %(default)s; the preset's 7 erases thin poles)")
    cam.add_argument("--spatial", action=argparse.BooleanOptionalAction, default=None,
                     help="enable/disable the spatial hole-filling filter (default: the ROBOTICS preset's value)")
    cam.add_argument("--confidence", type=int, default=None, metavar="0-255",
                     help="confidence threshold, v3 semantics: higher = fewer valid pixels (default: preset, 15)")
    cam.add_argument("--max-exposure-us", type=int, default=None, metavar="US",
                     help="cap auto-exposure to limit motion blur, e.g. 5000 (default: no cap)")
    cam.add_argument("--max-chip-temp", type=float, default=85.0, metavar="C",
                     help="stop when the camera chip reaches this temperature (default: %(default)s)")
    args = parser.parse_args(argv)
    if args.stride < 1 or args.min_points < 1:
        parser.error("--stride and --min-points must be >= 1")
    return args


def detection_config(args: argparse.Namespace) -> obstacles.ObstacleConfig:
    defaults = obstacles.ObstacleConfig()
    return obstacles.ObstacleConfig(
        max_range_m=args.max_range,
        evidence_range_m=max(defaults.evidence_range_m, args.max_range),
        band_below_m=args.band_below,
        band_above_m=args.band_above,
        stride=args.stride,
        min_points=args.min_points,
        corridor_half_width_m=args.corridor_width / 2.0,
    )


# --------------------------------------------------------------------------------------
# Replay source
# --------------------------------------------------------------------------------------

def load_recording(folder: Path) -> Tuple[np.ndarray, List[Tuple[float, Path]]]:
    """Depth intrinsics and ``(timestamp_s, png_path)`` per frame of a video recording."""
    session_path = folder / "session.json"
    index_path = folder / "depth_index.csv"
    if not session_path.is_file() or not index_path.is_file():
        raise RecordingError("{} is not a 05_record.py --mode video recording (need session.json and "
                             "depth_index.csv)".format(folder))
    try:
        session = json.loads(session_path.read_text(encoding="utf-8"))
        K = np.asarray(session["video"]["depth"]["intrinsics"], dtype=np.float64).reshape(3, 3)
        with index_path.open(newline="", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
    except OSError as exc:
        # An online-only OneDrive file that cannot be downloaded (e.g. the disk is full) fails
        # here with "[Errno 22] Invalid argument".
        raise RecordingError("cannot read {}: {}".format(folder, exc)) from exc
    except (ValueError, KeyError, TypeError) as exc:
        raise RecordingError("{}: no depth intrinsics under video.depth.intrinsics ({}: {})".format(
            session_path, type(exc).__name__, exc)) from exc
    frames: List[Tuple[float, Path]] = []
    for row in rows:
        try:
            frames.append((float(row["device_timestamp_s"]), folder / row["file"]))
        except (KeyError, ValueError) as exc:
            raise RecordingError("{}: bad row {} ({})".format(index_path, row, exc)) from exc
    if not frames:
        raise RecordingError("{} lists no depth frames".format(index_path))
    return K, frames


def replay_frames(folder: Path, down: np.ndarray, paced: bool) -> Iterator[DepthFrame]:
    """Yield the recording's depth frames; *paced* sleeps to the recorded frame times."""
    K, frames = load_recording(folder)
    start_wall = time.monotonic()
    t0 = frames[0][0]
    for t_s, path in frames:
        depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if depth is None or depth.ndim != 2:
            raise RecordingError("{} is not a readable 16-bit depth PNG (missing, or online-only in OneDrive "
                                 "with no disk space to download it)".format(path))
        if paced:
            delay = (t_s - t0) - (time.monotonic() - start_wall)
            if delay > 0:
                time.sleep(delay)
        yield DepthFrame(depth, K, down)


# --------------------------------------------------------------------------------------
# Live source
# --------------------------------------------------------------------------------------

def build_pipeline(pipeline: dai.Pipeline, args: argparse.Namespace, cfg: obstacles.ObstacleConfig
                   ) -> Tuple[dai.node.StereoDepth, Dict[str, dai.MessageQueue]]:
    """CAM_B/CAM_C -> StereoDepth (ROBOTICS, decimation 1) plus the raw accelerometer; return queues."""
    outputs = []
    for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
        cam = pipeline.create(dai.node.Camera).build(socket)
        cam.initialControl.setAntiBandingMode(dai.CameraControl.AntiBandingMode.MAINS_60_HZ)
        if args.max_exposure_us is not None:
            cam.initialControl.setAutoExposureLimit(int(args.max_exposure_us))
        outputs.append(cam.requestOutput(SIZE, fps=args.fps))

    stereo = pipeline.create(dai.node.StereoDepth).build(
        left=outputs[0], right=outputs[1], presetMode=dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setRectification(True)
    config = stereo.initialConfig
    config.setMedianFilter(MEDIAN_CHOICES[args.median])
    config.postProcessing.decimationFilter.decimationFactor = 1
    config.postProcessing.thresholdFilter.minRange = int(cfg.min_range_m * 1000)
    config.postProcessing.thresholdFilter.maxRange = int(cfg.evidence_range_m * 1000)
    if args.spatial is not None:
        config.postProcessing.spatialFilter.enable = bool(args.spatial)
    if args.confidence is not None:
        config.setConfidenceThreshold(int(args.confidence))

    queues = {"depth": stereo.depth.createOutputQueue(maxSize=4, blocking=False)}
    if not args.no_imu:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW], IMU_RATE_HZ)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(10)
        queues["imu"] = imu.out.createOutputQueue(maxSize=50, blocking=False)
    return stereo, queues


def describe_settings(stereo: dai.node.StereoDepth, args: argparse.Namespace) -> str:
    config = stereo.initialConfig
    pp = config.postProcessing
    return ("Stereo: ROBOTICS {}x{} @ {:g} fps, decimation {}, median {}, confidence {}, spatial {}, "
            "threshold {}-{} mm, 60 Hz anti-banding, AE limit {}; levelling: {}".format(
                SIZE[0], SIZE[1], args.fps, pp.decimationFilter.decimationFactor, config.getMedianFilter().name,
                config.getConfidenceThreshold(), "on" if pp.spatialFilter.enable else "off",
                pp.thresholdFilter.minRange, pp.thresholdFilter.maxRange,
                "{} us".format(args.max_exposure_us) if args.max_exposure_us else "none",
                "fixed pitch {:g} roll {:g} deg".format(args.pitch_deg, args.roll_deg) if args.no_imu
                else "accelerometer"))


class LiveSource:
    """Pull depth frames and accelerometer samples from a started pipeline."""

    def __init__(self, pipeline: dai.Pipeline, queues: Dict[str, dai.MessageQueue], device: dai.Device,
                 fixed_down: np.ndarray) -> None:
        self.pipeline = pipeline
        self.depth_q = queues["depth"]
        self.imu_q = queues.get("imu")
        self.device = device
        self.fixed_down = fixed_down
        self.gravity = obstacles.GravityEstimator()
        self.intrinsics: Optional[np.ndarray] = None
        self.R_cam_from_imu = np.eye(3)
        if self.imu_q is not None:
            # Depth is in the rectified stereo frame; the CAM_C rotation is within a fraction of a
            # degree of it, which is plenty for a height band.
            self.R_cam_from_imu = oakrig.rig_from_imu(device.readCalibration(), "CAM_C")[:3, :3]
        self.last_rx = time.monotonic()
        self.stall_warned = False

    def chip_temperature(self) -> Optional[float]:
        try:
            return float(self.device.getChipTemperature().average)
        except RuntimeError:
            return None

    def _pump_imu(self) -> None:
        if self.imu_q is None:
            return
        for msg in self.imu_q.tryGetAll():
            for packet in msg.packets:
                a = packet.acceleroMeter
                self.gravity.update(self.R_cam_from_imu @ np.array([a.x, a.y, a.z], dtype=np.float64),
                                    a.getTimestampDevice().total_seconds())

    def next(self) -> Optional[DepthFrame]:
        """The next depth frame, ``None`` on a 0.5 s timeout; raises RuntimeError if the pipeline died."""
        if not self.pipeline.isRunning():
            raise RuntimeError("pipeline stopped unexpectedly (device disconnected or crashed)")
        msg = self.depth_q.get(GET_TIMEOUT)
        self._pump_imu()
        if msg is None:
            if not self.stall_warned and time.monotonic() - self.last_rx > STALL_WARN_SECONDS:
                print("No depth frame for {:.0f} s - is the camera streaming?".format(STALL_WARN_SECONDS),
                      file=sys.stderr, flush=True)
                self.stall_warned = True
            return None
        self.last_rx = time.monotonic()
        self.stall_warned = False
        if self.intrinsics is None:
            self.intrinsics = np.asarray(msg.getTransformation().getIntrinsicMatrix(), dtype=np.float64)
            print("First depth frame: {}x{}, fx {:.1f} fy {:.1f} cx {:.1f} cy {:.1f}".format(
                msg.getWidth(), msg.getHeight(), self.intrinsics[0, 0], self.intrinsics[1, 1],
                self.intrinsics[0, 2], self.intrinsics[1, 2]), flush=True)
        down = self.fixed_down if self.imu_q is None else self.gravity.down
        return DepthFrame(msg.getFrame(), self.intrinsics, down)


# --------------------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------------------

def fmt_range(value: float) -> str:
    if math.isnan(value):
        return "unknown"
    if math.isinf(value):
        return "clear"
    return "{:.2f} m".format(value)


def half_fov_deg(K: np.ndarray, width: int) -> float:
    fx, cx = float(K[0, 0]), float(K[0, 2])
    return math.degrees(math.atan(max(cx, width - cx) / fx))


def in_view(scan: obstacles.ObstacleScan, K: np.ndarray, width: int) -> np.ndarray:
    """Sectors whose centre lies inside the camera's horizontal field of view."""
    return np.abs(scan.bearings_deg()) <= half_fov_deg(K, width)


def status_line(elapsed_s: float, fps: float, proc_ms: float, scan: obstacles.ObstacleScan,
                view: np.ndarray, down: Optional[np.ndarray], chip_c: Optional[float]) -> str:
    d = scan.distances_m[view]
    nearest = scan.nearest()
    near_txt = "none" if nearest is None else "{:.2f} m @ {:+.0f}".format(*nearest)
    chip = "" if chip_c is None else " | chip {:.0f} C".format(chip_c)
    return ("[{:6.1f} s] {:5.1f} fps  proc {:4.1f} ms | valid {:3.0f}% | nearest {} | ahead {} | "
            "view {} obst {} clear {} unk | tilt {:4.1f}{}".format(
                elapsed_s, fps, proc_ms, 100.0 * scan.valid_fraction, near_txt, fmt_range(scan.ahead_m),
                int(np.isfinite(d).sum()), int(np.isposinf(d).sum()), int(np.isnan(d).sum()),
                obstacles.tilt_deg(down), chip))


def sector_color(distance_m: float) -> Tuple[int, int, int]:
    if math.isnan(distance_m):
        return COLOR_UNKNOWN
    if math.isinf(distance_m):
        return COLOR_CLEAR
    if distance_m < 1.0:
        return COLOR_NEAR
    if distance_m < 2.0:
        return COLOR_MID
    return COLOR_FAR


def draw_sector_bar(img: np.ndarray, scan: obstacles.ObstacleScan, K: np.ndarray, view: np.ndarray) -> None:
    """One coloured segment per in-view sector along the top edge (columns for a level camera)."""
    fx, cx = float(K[0, 0]), float(K[0, 2])
    width = img.shape[1]
    half = scan.sector_deg / 2.0
    bearings = scan.bearings_deg()
    for i in np.flatnonzero(view):
        b = float(bearings[i])
        u0 = int(round(cx + fx * math.tan(math.radians(b - half))))
        u1 = int(round(cx + fx * math.tan(math.radians(b + half))))
        cv2.rectangle(img, (max(0, u0), 0), (min(width - 1, u1), BAR_PX), sector_color(scan.distances_m[i]), -1)
        cv2.line(img, (u0, 0), (u0, BAR_PX), (0, 0, 0), 1)


def draw_radar(scan: obstacles.ObstacleScan, view: np.ndarray, corridor_half_m: float) -> np.ndarray:
    """Top-down view: camera at the bottom centre, a ring per metre out to max range."""
    img = np.full((RADAR_PX, RADAR_PX, 3), 25, dtype=np.uint8)
    ox, oy = RADAR_PX // 2, RADAR_PX - 20
    scale = (RADAR_PX - 40) / scan.max_range_m

    def to_px(r: float, bearing_deg: float) -> Tuple[int, int]:
        b = math.radians(bearing_deg)
        return int(round(ox + r * math.sin(b) * scale)), int(round(oy - r * math.cos(b) * scale))

    half = scan.sector_deg / 2.0
    bearings = scan.bearings_deg()
    for i in np.flatnonzero(view):
        b = float(bearings[i])
        d = float(scan.distances_m[i])
        r = scan.max_range_m if not math.isfinite(d) else d
        arc = [to_px(r, b - half + k * scan.sector_deg / 4.0) for k in range(5)]
        cv2.fillPoly(img, [np.array([(ox, oy)] + arc, dtype=np.int32)], sector_color(d))
    for ring in range(1, int(scan.max_range_m) + 1):
        cv2.circle(img, (ox, oy), int(ring * scale), (80, 80, 80), 1)
        cv2.putText(img, "{} m".format(ring), (ox + 4, oy - int(ring * scale) + 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (160, 160, 160), 1, cv2.LINE_AA)
    w = int(round(corridor_half_m * scale))
    reach = scan.ahead_m if math.isfinite(scan.ahead_m) else scan.max_range_m
    top = int(round(oy - reach * scale))
    cv2.line(img, (ox - w, oy), (ox - w, top), (255, 255, 255), 1)
    cv2.line(img, (ox + w, oy), (ox + w, top), (255, 255, 255), 1)
    if math.isfinite(scan.ahead_m):
        cv2.line(img, (ox - w, top), (ox + w, top), (255, 255, 255), 2)
    cv2.circle(img, (ox, oy), 5, (255, 255, 255), -1)
    return img


def render(frame: DepthFrame, scan: obstacles.ObstacleScan, view: np.ndarray, corridor_half_m: float,
           text: str) -> np.ndarray:
    depth_img = colorize_depth(frame.depth_mm, max_mm=scan.max_range_m * 1000.0)
    if depth_img.shape[:2] != (SIZE[1], SIZE[0]):
        depth_img = cv2.resize(depth_img, SIZE, interpolation=cv2.INTER_NEAREST)
    K = frame.intrinsics * np.array([[SIZE[0] / frame.depth_mm.shape[1]], [SIZE[1] / frame.depth_mm.shape[0]], [1.0]])
    draw_sector_bar(depth_img, scan, K, view)
    draw_label(depth_img, text, org=(10, BAR_PX + 22), scale=0.5)
    return np.hstack([depth_img, draw_radar(scan, view, corridor_half_m)])


# --------------------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------------------

def run(next_frame, args: argparse.Namespace, cfg: obstacles.ObstacleConfig,
        chip_temperature=None) -> int:
    """Detect on every frame from *next_frame()* (``None`` = no frame yet, StopIteration = end)."""
    if not args.headless:
        try:
            cv2.namedWindow(WINDOW)
        except cv2.error as exc:
            print("Cannot open an OpenCV window ({}); use --headless.".format(" ".join(str(exc).split())[:200]),
                  file=sys.stderr, flush=True)
            return EXIT_PIPELINE_STOPPED

    clock = RunClock(args.duration)
    fps = FpsCounter()
    ticker = Periodic(1.0, first_immediately=True)
    proc_ms: List[float] = []
    chip_c: Optional[float] = None
    exit_code = EXIT_OK
    text = ""
    last: Optional[Tuple[obstacles.ObstacleScan, np.ndarray, Optional[np.ndarray]]] = None
    while not clock.expired():
        try:
            frame = next_frame()
        except StopIteration:
            break
        if frame is None:
            if not args.headless and wait_key_quit(1) == "q":
                break
            continue

        t0 = time.perf_counter()
        scan = obstacles.detect(frame.depth_mm, frame.intrinsics, cfg, down=frame.down)
        proc_ms.append((time.perf_counter() - t0) * 1000.0)
        fps.tick()
        view = in_view(scan, frame.intrinsics, frame.depth_mm.shape[1])
        last = (scan, view, frame.down)

        if ticker.ready():
            if chip_temperature is not None:
                chip_c = chip_temperature()
            text = status_line(clock.elapsed(), fps.fps(), float(np.mean(proc_ms[-30:])), scan, view,
                               frame.down, chip_c)
            print(text, flush=True)
            if chip_c is not None and chip_c >= args.max_chip_temp:
                print("Chip temperature {:.0f} C reached the --max-chip-temp limit of {:.0f} C - stopping.".format(
                    chip_c, args.max_chip_temp), file=sys.stderr, flush=True)
                exit_code = EXIT_TOO_HOT
                break
        if not args.headless:
            cv2.imshow(WINDOW, render(frame, scan, view, cfg.corridor_half_width_m, text.split(" | ", 1)[-1]))
            if wait_key_quit(1) == "q":
                break

    elapsed = clock.elapsed()
    if last is not None:
        print("Last frame: " + status_line(elapsed, fps.fps(), proc_ms[-1], *last, chip_c).split("] ", 1)[1],
              flush=True)
    if proc_ms:
        print("Done: {} depth frames in {:.1f} s (avg {:.1f} fps); detection {:.1f} ms mean, {:.1f} ms p95.".format(
            len(proc_ms), elapsed, len(proc_ms) / elapsed if elapsed > 0 else 0.0,
            float(np.mean(proc_ms)), float(np.percentile(proc_ms, 95))), flush=True)
    else:
        print("Done: no depth frames in {:.1f} s.".format(elapsed), flush=True)
    return exit_code


def main_replay(args: argparse.Namespace, cfg: obstacles.ObstacleConfig, down: np.ndarray) -> int:
    folder = Path(args.replay)
    try:
        load_recording(folder)               # fail before opening a window
        frames = replay_frames(folder, down, paced=not args.headless)
        return run(lambda: next(frames), args, cfg)
    except RecordingError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return EXIT_BAD_INPUT


def main_live(args: argparse.Namespace, cfg: obstacles.ObstacleConfig, down: np.ndarray) -> int:
    device = open_device(args.device_id, retries=1 if args.device_id else 3)
    print_device_summary(device)
    with dai.Pipeline(device) as pipeline:
        stereo, queues = build_pipeline(pipeline, args, cfg)
        print(describe_settings(stereo, args), flush=True)
        pipeline.start()
        print("Pipeline started - letting auto-exposure settle for {:.1f} s ...".format(WARM_UP_SECONDS), flush=True)
        warm_up(*queues.values(), seconds=WARM_UP_SECONDS)
        source = LiveSource(pipeline, queues, device, down)
        try:
            exit_code = run(source.next, args, cfg, chip_temperature=source.chip_temperature)
        except RuntimeError as exc:
            print("Stopped: {}".format(" ".join(str(exc).split())[:400]), file=sys.stderr, flush=True)
            exit_code = EXIT_PIPELINE_STOPPED
        pipeline.stop()
    return exit_code


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        cfg = detection_config(args)
        down = obstacles.down_from_tilt(args.pitch_deg, args.roll_deg)
        obstacles.level_rotation(down)
    except ValueError as exc:
        print("Bad detection settings: {}".format(exc), file=sys.stderr, flush=True)
        return EXIT_BAD_INPUT
    try:
        if args.replay:
            return main_replay(args, cfg, down)
        return main_live(args, cfg, down)
    except KeyboardInterrupt:
        print("\nInterrupted (Ctrl-C) - stopping.", flush=True)
        return EXIT_OK
    except RuntimeError as exc:
        # depthai raises RuntimeError (XLink errors subclass it) for a device that disconnected
        # between open_device() and pipeline.start(), an unsupported setting, etc.
        print("Failed to build/start the pipeline: {}: {}".format(
            type(exc).__name__, " ".join(str(exc).split())[:400]), file=sys.stderr, flush=True)
        return EXIT_PIPELINE_STOPPED
    finally:
        if not args.headless:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


if __name__ == "__main__":
    sys.exit(main())
