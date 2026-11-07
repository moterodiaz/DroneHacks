#!/usr/bin/env python3
"""01 - RGB preview: stream the OAK-D S2 colour camera (CAM_A, IMX378) with a DepthAI v3 Camera node.

What this test checks
---------------------
* The camera can be found and opened (``common.open_device``) and runs on USB 3.
* ``dai.node.Camera`` on ``CAM_A`` delivers BGR frames at the requested size and frame rate.
* Frames arrive at (close to) the requested FPS without sequence-number gaps, and the host
  latency (``dai.Clock.now() - frame.getTimestamp()``) stays small (tens of milliseconds).
* Per-frame metadata (exposure time, ISO, autofocus lens position) is populated.
* Snapshots can be written as PNG files.

How to run (from the repository root)
-------------------------------------
    python camera/01_rgb_preview.py                          # 1280x720 @ 30 fps in a window
    python camera/01_rgb_preview.py --width 1920 --height 1080 --fps 30
    python camera/01_rgb_preview.py --headless --duration 10 --snapshot-on-exit
    python camera/01_rgb_preview.py --device-id OAKDS2EXAMPLEID000
    python camera/01_rgb_preview.py --help

What you should see
-------------------
Windowed mode: a window "01 RGB preview (CAM_A)" with the live colour image and an overlay
showing the measured FPS, frame count / sequence number, the frame timestamp (host-synced
DepthAI clock), the device timestamp, host latency, exposure, ISO and lens position.  After
auto-exposure settles (~1 s) the FPS should sit at the requested value (30.0 by default).
Headless mode (``--headless``): one status line per second on stdout with the same numbers.
On exit (either mode): a summary with the average FPS, frame count, missing frames (sequence-
number gaps: dropped on the device or discarded by the host queue) and the paths of any
snapshots.

Keys (windowed mode only)
-------------------------
    q / Esc   quit (closing the window with its title-bar X button quits as well)
    s         save a PNG snapshot into snapshots/  (rgb_<W>x<H>_<YYYYmmdd_HHMMSS>_seq<N>.png)

Exit status
-----------
    0    normal exit: q / Esc, window closed, Ctrl-C inside the streaming loop, or ``--duration``
         elapsed
    1    the pipeline could not be built / produced no frames / stopped, the USB link dropped,
         or no display available
    2    no OAK device found, or ``--device-id`` not present (one line on stderr, no traceback)
    130  Ctrl-C outside the streaming loop: before it (opening the device, building the pipeline)
         or after it (snapshot / summary / shutdown; the summary is still printed if it was not
         yet)
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402  (import after the sys.path tweak, by design)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    draw_label,
    ensure_dir,
    open_device,
    print_device_summary,
    timestamp_str,
    wait_key_quit,
)

WINDOW_NAME = "01 RGB preview (CAM_A)"
#: Default snapshot folder: <repo root>/snapshots, independent of the current directory.
DEFAULT_SNAPSHOT_DIR = Path(__file__).resolve().parent.parent / "snapshots"
#: Give up when the camera delivers no frame for this long (first frame or a mid-run stall).
STALL_TIMEOUT_S = 10.0
#: Poll interval for the output queue; keeps Ctrl-C, --duration and the window responsive.
QUEUE_POLL = datetime.timedelta(milliseconds=200)
#: Largest window we open by default (the image itself keeps its full resolution).
MAX_WINDOW_WIDTH = 1280

RESIZE_MODES: Dict[str, dai.ImgResizeMode] = {
    "crop": dai.ImgResizeMode.CROP,
    "stretch": dai.ImgResizeMode.STRETCH,
    "letterbox": dai.ImgResizeMode.LETTERBOX,
}


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Build the argument parser (shared --device-id/--duration/--headless via common) and parse."""
    parser = argparse.ArgumentParser(
        description="Stream the OAK-D S2 colour camera (CAM_A) with a DepthAI v3 Camera node, "
                    "with an FPS/timestamp overlay and PNG snapshots.",
        epilog="Keys in the window: q/Esc quit (closing the window quits too), s save snapshot. "
               "Exit status 2 = no device.",
    )
    parser.add_argument("--width", type=int, default=1280, metavar="PX",
                        help="output width in pixels (default 1280)")
    parser.add_argument("--height", type=int, default=720, metavar="PX",
                        help="output height in pixels (default 720)")
    parser.add_argument("--fps", type=float, default=30.0, metavar="FPS",
                        help="requested frame rate (default 30)")
    parser.add_argument("--resize-mode", choices=sorted(RESIZE_MODES), default="crop",
                        help="how the sensor image is fitted into WIDTHxHEIGHT (default crop)")
    parser.add_argument("--snapshot-on-exit", action="store_true",
                        help="save one PNG of the last frame when the script exits (also headless)")
    parser.add_argument("--snapshot-dir", type=Path, default=DEFAULT_SNAPSHOT_DIR, metavar="DIR",
                        help="folder for snapshots (default: <repo>/snapshots)")
    add_device_arg(parser)
    add_run_args(parser)
    args = parser.parse_args(argv)
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive integers")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    return args


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def fmt_td(td: datetime.timedelta) -> str:
    """Format a timedelta as HH:MM:SS.mmm (DepthAI timestamps are timedeltas)."""
    total = td.total_seconds()
    sign = "-" if total < 0 else ""
    total = abs(total)
    hours, rem = divmod(total, 3600.0)
    minutes, seconds = divmod(rem, 60.0)
    return "{}{:02d}:{:02d}:{:06.3f}".format(sign, int(hours), int(minutes), seconds)


def _enum_name(value: object) -> str:
    return str(getattr(value, "name", value))


def _comm_failure(exc: BaseException) -> str:
    """Stop reason for an exception out of the device link (``dai.XLinkReadError`` & co.)."""
    return "device communication failed: {}: {}".format(
        type(exc).__name__, " ".join(str(exc).split())[:200])


def window_closed() -> bool:
    """``True`` once the preview window is gone (title-bar X button).

    HighGUI would silently re-create the window on the next ``cv2.imshow``, so the loop has to
    ask.  ``WND_PROP_VISIBLE`` reads 0.0 after the window is destroyed (verified with OpenCV
    5.0.0 on Windows); a backend that raises instead is treated as "closed" too.
    """
    try:
        return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


@dataclasses.dataclass
class FrameInfo:
    """The per-frame numbers shown in the overlay / status line."""

    seq: int
    width: int
    height: int
    type_name: str
    timestamp: datetime.timedelta
    timestamp_device: datetime.timedelta
    latency_ms: float
    exposure_ms: float
    iso: int
    lens_pos: int
    sensor_fps: float


def frame_info(frame: dai.ImgFrame) -> FrameInfo:
    """Read the metadata of one ``dai.ImgFrame`` (all getters confirmed in depthai 3.10.0)."""
    ts = frame.getTimestamp()
    latency = (dai.Clock.now() - ts).total_seconds() * 1000.0
    return FrameInfo(
        seq=frame.getSequenceNum(),
        width=frame.getWidth(),
        height=frame.getHeight(),
        type_name=_enum_name(frame.getType()),
        timestamp=ts,
        timestamp_device=frame.getTimestampDevice(),
        latency_ms=latency,
        exposure_ms=frame.getExposureTime().total_seconds() * 1000.0,
        iso=frame.getSensitivity(),
        lens_pos=frame.getLensPosition(),
        sensor_fps=frame.getFps(),
    )


def overlay_text(info: FrameInfo, fps_now: float, count: int, requested_fps: float) -> str:
    """Multi-line overlay for the window."""
    return (
        "{:5.1f} fps (req {:g}, sensor {:.1f})   frame {}   seq {}\n"
        "ts {}   dev {}   latency {:.0f} ms\n"
        "{}x{} {}   exp {:.2f} ms   iso {}   lens {}\n"
        "keys: q quit   s snapshot"
    ).format(
        fps_now, requested_fps, info.sensor_fps, count, info.seq,
        fmt_td(info.timestamp), fmt_td(info.timestamp_device), info.latency_ms,
        info.width, info.height, info.type_name, info.exposure_ms, info.iso, info.lens_pos,
    )


def status_line(info: FrameInfo, fps_now: float, count: int, elapsed: float) -> str:
    """One-line status for headless mode."""
    return (
        "[{:7.1f} s] fps {:5.1f}  frames {:6d}  seq {:6d}  ts {}  latency {:5.1f} ms  "
        "exp {:.2f} ms  iso {}  lens {}"
    ).format(elapsed, fps_now, count, info.seq, fmt_td(info.timestamp), info.latency_ms,
             info.exposure_ms, info.iso, info.lens_pos)


def save_snapshot(img: np.ndarray, seq: int, out_dir: Path) -> Optional[Path]:
    """Write *img* as ``rgb_<W>x<H>_<timestamp>_seq<N>.png`` under *out_dir*; return the path."""
    try:
        ensure_dir(out_dir)
    except OSError as exc:
        print("Cannot create snapshot folder {}: {}".format(out_dir, exc), file=sys.stderr, flush=True)
        return None
    height, width = img.shape[:2]
    path = out_dir / "rgb_{}x{}_{}_seq{}.png".format(width, height, timestamp_str(), seq)
    ok = False
    try:
        ok = bool(cv2.imwrite(str(path), img))
    except cv2.error as exc:
        print("cv2.imwrite failed for {}: {}".format(path, exc), file=sys.stderr, flush=True)
    if not ok:
        print("Snapshot NOT saved: {}".format(path), file=sys.stderr, flush=True)
        return None
    print("Snapshot saved: {}".format(path), flush=True)
    return path


# --------------------------------------------------------------------------------------
# Pipeline
# --------------------------------------------------------------------------------------

def build_pipeline(device: dai.Device, width: int, height: int, fps: float,
                   resize_mode: dai.ImgResizeMode) -> Tuple[dai.Pipeline, dai.MessageQueue]:
    """Create the pipeline on *device*: one Camera node on CAM_A with one BGR output queue.

    BGR888i is requested so ``getCvFrame()`` hands OpenCV a ready-to-show (H, W, 3) uint8 image.
    The queue is small and non-blocking so a slow host drops frames instead of building latency;
    drops show up as sequence-number gaps in the exit summary.
    """
    pipeline = dai.Pipeline(device)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    output = cam.requestOutput((width, height), dai.ImgFrame.Type.BGR888i,
                               resizeMode=resize_mode, fps=float(fps))
    queue = output.createOutputQueue(maxSize=4, blocking=False)
    return pipeline, queue


@dataclasses.dataclass
class RunResult:
    """Everything the exit summary needs, collected by :func:`stream`."""

    frames: int = 0
    first_time: Optional[float] = None
    last_time: Optional[float] = None
    first_seq: Optional[int] = None
    last_seq: Optional[int] = None
    sensor_fps: Optional[float] = None
    last_frame: Optional[np.ndarray] = None
    snapshots: List[Path] = dataclasses.field(default_factory=list)
    stop_reason: str = ""
    ok: bool = True


@dataclasses.dataclass
class _Progress:
    """How far main() got, for the ``__main__`` Ctrl-C handler (see the bottom of the file)."""

    result: Optional[RunResult] = None   # set as soon as stream() has returned
    requested_fps: float = 0.0
    summarised: bool = False             # print_summary() has been started


_PROGRESS = _Progress()


def stream(pipeline: dai.Pipeline, queue: dai.MessageQueue, args: argparse.Namespace,
           window_open: bool) -> RunResult:
    """Main loop: pull frames until q/Esc, Ctrl-C, --duration, or the pipeline dies."""
    result = RunResult()
    run_clock = RunClock(args.duration)
    fps_counter = FpsCounter(window_s=2.0)
    report = Periodic(interval_s=1.0, first_immediately=True)
    last_frame_at = time.monotonic()                 # start time, then the arrival of the last frame

    try:
        while True:
            if run_clock.expired():
                result.stop_reason = "--duration {:g} s elapsed".format(args.duration)
                break
            if not pipeline.isRunning():
                result.stop_reason = "pipeline stopped unexpectedly (device disconnected or firmware crash?)"
                result.ok = False
                break

            try:
                msg = queue.get(QUEUE_POLL)          # None on timeout (no exception)
            except dai.MessageQueue.QueueException:
                result.stop_reason = "output queue closed (pipeline stopped)"
                result.ok = False
                break
            except RuntimeError as exc:              # dai.XLinkReadError & co.: USB link dropped
                result.stop_reason = _comm_failure(exc)
                result.ok = False
                break

            if msg is None:
                if time.monotonic() - last_frame_at > STALL_TIMEOUT_S:
                    what = "no frame received" if result.frames == 0 else "no new frame"
                    result.stop_reason = "{} for {:.0f} s (after {} frames)".format(
                        what, STALL_TIMEOUT_S, result.frames)
                    result.ok = False
                    break
                # Keep the window (and its keys) alive while we wait for the next frame.
                if window_open:
                    if wait_key_quit(1) == "q":
                        result.stop_reason = "q / Esc pressed"
                        break
                    if window_closed():
                        result.stop_reason = "window closed"
                        break
                continue

            if not isinstance(msg, dai.ImgFrame):
                continue

            try:
                img = msg.getCvFrame()               # (H, W, 3) uint8 BGR, a copy we may draw on
            except RuntimeError as exc:              # frame data unreadable: treat as a link failure
                result.stop_reason = _comm_failure(exc)
                result.ok = False
                break
            now = time.monotonic()
            last_frame_at = now
            info = frame_info(msg)
            fps_counter.tick()
            result.frames += 1
            if result.first_time is None:
                result.first_time = now
                result.first_seq = info.seq
                print("First frame: {}x{} {} seq {} (sensor reports {:.1f} fps)".format(
                    info.width, info.height, info.type_name, info.seq, info.sensor_fps), flush=True)
            result.last_time = now
            result.last_seq = info.seq
            result.sensor_fps = info.sensor_fps
            result.last_frame = img
            fps_now = fps_counter.fps()

            if window_open:
                display = img.copy()                 # keep `img` clean for snapshots
                draw_label(display, overlay_text(info, fps_now, result.frames, args.fps), org=(10, 25))
                try:
                    cv2.imshow(WINDOW_NAME, display)
                    key = wait_key_quit(1)
                except cv2.error as exc:
                    result.stop_reason = "OpenCV display failed ({}); use --headless".format(
                        " ".join(str(exc).split())[:200])
                    result.ok = False
                    break
                if key == "q":
                    result.stop_reason = "q / Esc pressed"
                    break
                if window_closed():                  # title-bar X: the next imshow would re-create it
                    result.stop_reason = "window closed"
                    break
                if key == "s":
                    path = save_snapshot(img, info.seq, args.snapshot_dir)
                    if path is not None:
                        result.snapshots.append(path)
            elif report.ready():
                print(status_line(info, fps_now, result.frames, run_clock.elapsed()), flush=True)
    except KeyboardInterrupt:
        result.stop_reason = "Ctrl-C"
        print("", flush=True)   # newline after the ^C echo

    return result


def print_summary(result: RunResult, requested_fps: float) -> None:
    """Print the end-of-run report: average FPS, frame count, drops and snapshot paths."""
    print("--- Summary ---", flush=True)
    print("Stop reason: {}".format(result.stop_reason or "unknown"))
    print("Frames received: {}".format(result.frames))
    if (result.frames >= 2 and result.first_time is not None and result.last_time is not None
            and result.last_time > result.first_time):
        span = result.last_time - result.first_time
        avg = (result.frames - 1) / span
        sensor = "{:.1f}".format(result.sensor_fps) if result.sensor_fps is not None else "?"
        print("Average FPS: {:.2f} over {:.1f} s (requested {:g}, sensor reports {})".format(
            avg, span, requested_fps, sensor))
    else:
        print("Average FPS: n/a (fewer than 2 frames)")
    if result.first_seq is not None and result.last_seq is not None:
        expected = result.last_seq - result.first_seq + 1
        dropped = max(0, expected - result.frames)
        print("Sequence numbers {}..{}: {} frame(s) missing (dropped on the device, or discarded by "
              "the non-blocking host queue when this loop fell behind)".format(
                  result.first_seq, result.last_seq, dropped))
    print("Snapshots saved: {}".format(len(result.snapshots)))
    for path in result.snapshots:
        print("  {}".format(path))
    sys.stdout.flush()


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # An explicit --device-id never retries: one stderr line even when nothing enumerates.
    # Without an id the open is retried while a camera reboots after another script.
    device = open_device(args.device_id, retries=1 if args.device_id else 3)  # exits 2 on failure
    print_device_summary(device)

    pipeline: Optional[dai.Pipeline] = None
    window_open = False
    exit_code = 0
    try:
        try:
            pipeline, queue = build_pipeline(device, args.width, args.height, args.fps,
                                             RESIZE_MODES[args.resize_mode])
            pipeline.start()
        except Exception as exc:  # noqa: BLE001 - any binding error becomes one readable line
            print("Failed to build/start the pipeline ({}x{} @ {:g} fps): {}: {}".format(
                args.width, args.height, args.fps, type(exc).__name__, " ".join(str(exc).split())),
                file=sys.stderr, flush=True)
            return 1

        print("Streaming CAM_A {}x{} @ {:g} fps, resize={}, type=BGR888i{}".format(
            args.width, args.height, args.fps, args.resize_mode,
            " for {:g} s".format(args.duration) if args.duration > 0 else ""), flush=True)

        if args.headless:
            print("Headless: printing one status line per second; Ctrl-C to stop.", flush=True)
        else:
            try:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                scale = min(1.0, MAX_WINDOW_WIDTH / float(args.width))
                cv2.resizeWindow(WINDOW_NAME, int(args.width * scale), int(args.height * scale))
                window_open = True
            except cv2.error as exc:
                print("Cannot open an OpenCV window ({}); re-run with --headless.".format(
                    " ".join(str(exc).split())[:200]), file=sys.stderr, flush=True)
                return 1
            print("Window open: press q or Esc to quit, s to save a snapshot.", flush=True)

        result = stream(pipeline, queue, args, window_open)
        _PROGRESS.result = result
        _PROGRESS.requested_fps = args.fps

        if args.snapshot_on_exit:
            if result.last_frame is not None:
                path = save_snapshot(result.last_frame, result.last_seq or 0, args.snapshot_dir)
                if path is not None:
                    result.snapshots.append(path)
            else:
                print("--snapshot-on-exit: no frame was received, nothing to save.",
                      file=sys.stderr, flush=True)

        _PROGRESS.summarised = True          # set first so a Ctrl-C mid-summary does not repeat it
        print_summary(result, args.fps)
        exit_code = 0 if result.ok else 1
    finally:
        # Always: stop the pipeline (this also closes the device), close windows, close device.
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception as exc:  # noqa: BLE001
                print("Warning: pipeline.stop() raised {}: {}".format(type(exc).__name__, exc),
                      file=sys.stderr, flush=True)
        if window_open:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)   # let the GUI backend process the destroy on Linux
            except cv2.error:
                pass
        try:
            device.close()       # harmless when pipeline.stop() already closed it
        except Exception:  # noqa: BLE001
            pass
    return exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # A Ctrl-C inside the streaming loop is handled there (normal summary, exit 0).  One that
        # lands anywhere else ends up here: before the loop (opening the device, building or
        # starting the pipeline) or after it (snapshot-on-exit, the summary, the ~1 s
        # pipeline.stop()).  main()'s finally block has stopped the pipeline, which closes the
        # device, so nothing is left running.
        print("", flush=True)   # newline after the ^C echo
        if _PROGRESS.result is None:
            print("Interrupted by Ctrl-C before streaming started.", file=sys.stderr, flush=True)
        else:
            print("Interrupted by Ctrl-C after the streaming loop (during snapshot / summary / "
                  "shutdown).", file=sys.stderr, flush=True)
            if not _PROGRESS.summarised:
                _PROGRESS.summarised = True
                print_summary(_PROGRESS.result, _PROGRESS.requested_fps)
        sys.exit(130)
