"""Shared helpers for the DroneDepthCamTesting OAK-D S2 test scripts (DepthAI v3, depthai==3.10.0).

Every test script in ``camera/`` imports this module for:

* CLI plumbing: :func:`add_device_arg` (``--device-id``) and :func:`add_run_args`
  (``--duration``, ``--headless``).
* Device discovery and opening with human-readable failures: :func:`list_devices`,
  :func:`format_device_table`, :func:`open_device`, :func:`print_device_summary`.
* Run bookkeeping: :class:`RunClock`, :class:`FpsCounter`, :class:`Periodic`.
* Visualisation: :func:`colorize_depth`, :func:`colorize_disparity`, :func:`draw_label`,
  :func:`wait_key_quit`.
* Output paths: :func:`ensure_dir`, :func:`timestamp_str`, :func:`timestamp_dir`,
  :func:`timestamp_name`.
* Queue helpers: :func:`drain_queues`, :func:`warm_up` (auto-exposure settle).

Importing this module has no side effects: no device access, no windows, no output.
It runs unchanged on Windows 11 (Python 3.13) and on a Jetson Orin Nano (Ubuntu 22.04,
Python 3.10); it needs only the standard library, numpy, opencv-python and depthai.

Facts this module relies on (verified on the OAK-D S2 with depthai 3.10.0, see
``camera/docs/depthai-v3-cheatsheet.md`` section 1):

* ``dai.Device.getAllAvailableDevices()`` takes ~0.5 s, never boots the camera and returns
  ``[]`` while another process holds the camera (there is no
  ``X_LINK_DEVICE_ALREADY_IN_USE`` exception on this platform).
* A failed ``dai.Device(...)`` raises a plain ``RuntimeError`` whose text is either
  ``'No available devices'`` or ``'Cannot find any device with given deviceInfo'`` after a
  10-14 s search; both mean "not there or busy".
* ``getUsbSpeed()`` is ``UsbSpeed.SUPER`` on a good USB 3 link; ``HIGH`` means USB 2 fallback.
"""

from __future__ import annotations

import argparse
import collections
import sys
import time
from pathlib import Path
from typing import Deque, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np
import depthai as dai

__all__ = [
    "EXIT_NO_DEVICE",
    "add_device_arg",
    "add_run_args",
    "list_devices",
    "format_device_table",
    "open_device",
    "print_device_summary",
    "RunClock",
    "FpsCounter",
    "Periodic",
    "colorize_depth",
    "colorize_disparity",
    "draw_label",
    "ensure_dir",
    "timestamp_str",
    "timestamp_dir",
    "timestamp_name",
    "wait_key_quit",
    "drain_queues",
    "warm_up",
]

#: Exit status used by :func:`open_device` when no usable camera could be opened.
EXIT_NO_DEVICE = 2

# Message fragments of the RuntimeError raised by dai.Device(...) when the camera is absent
# or held by another process (verbatim from the cheat-sheet, depthai 3.10.0 on Windows 11).
_BUSY_FRAGMENTS = ("No available devices", "Cannot find any device with given deviceInfo")

# USB speeds that are not a USB 2 fallback.
_GOOD_USB_SPEEDS = (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS)


# --------------------------------------------------------------------------------------
# argparse helpers
# --------------------------------------------------------------------------------------

def add_device_arg(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add ``--device-id`` (optional) to *parser* and return the parser.

    The value lands in ``args.device_id`` (``None`` when omitted) and is meant to be passed
    straight to :func:`open_device`.  Matching is by device id (e.g. ``OAKDS2EXAMPLEID000``),
    case-insensitive.
    """
    parser.add_argument(
        "--device-id",
        dest="device_id",
        default=None,
        metavar="ID",
        help="OAK device id to open, e.g. OAKDS2EXAMPLEID000 (default: the first device found)",
    )
    return parser


def add_run_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add ``--duration SECONDS`` and ``--headless`` to *parser* and return the parser.

    ``args.duration`` (float, default ``0.0``) is the run length; ``0`` means run until the
    user presses ``q`` (or ``Esc``) in an OpenCV window or hits Ctrl-C.  Feed it to
    :class:`RunClock`.  ``args.headless`` (bool) means: open no OpenCV windows, do no key
    handling, and print periodic statistics to stdout instead (use :class:`Periodic`).
    """
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="stop after this many seconds (default 0 = run until 'q' / Esc / Ctrl-C)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="no OpenCV windows or key handling; print periodic stats to stdout instead",
    )
    return parser


# --------------------------------------------------------------------------------------
# Device discovery / opening
# --------------------------------------------------------------------------------------

def _enum_name(value: object) -> str:
    """Short name of a pybind enum member (``UsbSpeed.SUPER`` -> ``'SUPER'``)."""
    return str(getattr(value, "name", value))


def _one_line(text: str) -> str:
    """Collapse whitespace/newlines so a message fits on one line."""
    return " ".join(str(text).split())


def _safe(getter, default):
    """Call *getter* and return *default* if it raises (device queries can fail on odd hardware)."""
    try:
        return getter()
    except Exception:  # noqa: BLE001 - we only want a value or the default
        return default


def list_devices() -> List[dai.DeviceInfo]:
    """Enumerate OAK devices without opening (booting) any of them.

    Takes about 0.5 s over USB.  Returns an empty list when no camera is plugged in *and*
    when another process is currently holding the camera - depthai cannot tell those apart.
    """
    try:
        return list(dai.Device.getAllAvailableDevices())
    except Exception:  # noqa: BLE001 - enumeration failure is reported as "nothing found"
        return []


def format_device_table(infos: Sequence[dai.DeviceInfo]) -> str:
    """Render the enumeration result as a single line (one cell per device)."""
    if not infos:
        return "OAK devices found: none"
    cells = []
    for index, info in enumerate(infos, start=1):
        cells.append(
            "[{}] id={} state={} protocol={} platform={} name={}".format(
                index,
                _safe(info.getDeviceId, "") or "?",
                _enum_name(getattr(info, "state", "?")),
                _enum_name(getattr(info, "protocol", "?")),
                _enum_name(getattr(info, "platform", "?")),
                getattr(info, "name", "") or "-",
            )
        )
    return "OAK devices found: {}  {}".format(len(infos), " | ".join(cells))


def open_device(
    device_id: Optional[str] = None,
    *,
    max_usb_speed: Optional[dai.UsbSpeed] = None,
    retries: int = 3,
    retry_wait_s: float = 3.0,
) -> dai.Device:
    """Enumerate, print what was found, and open the requested (or first) OAK device.

    On success returns an open ``dai.Device`` (close it with ``device.close()`` or hand it to
    ``dai.Pipeline(device)``; note that ``pipeline.stop()`` closes it too).

    On failure - no camera, requested id not present, camera held by another process, or
    the open itself failing - prints ONE human-readable line to stderr and calls
    ``sys.exit(2)`` (:data:`EXIT_NO_DEVICE`); it never lets a traceback through.

    Because a rebooting camera (right after another script's ``pipeline.stop()``) or a
    busy camera both enumerate as "nothing found", the whole enumerate+open step is
    retried ``retries`` times with ``retry_wait_s`` seconds in between before giving up.
    The one failure that is never retried is a ``device_id`` missing from a NON-empty
    enumeration (a mistyped id or, with several cameras, one that is unplugged): no retry
    can fix that, so it fails at once with a single stderr line.  Pass ``retries=1`` for a
    fail-fast open in every case - the scripts do so whenever ``--device-id`` is given
    (``retries=1 if args.device_id else 3``), so an explicit id never retries and always
    produces exactly one stderr line, camera present or not.

    ``max_usb_speed`` (e.g. ``dai.UsbSpeed.HIGH``) forces a lower USB speed for testing.
    """
    wanted = device_id.strip().upper() if device_id else None
    attempts = max(1, int(retries))

    for attempt in range(1, attempts + 1):
        infos = list_devices()
        print(format_device_table(infos), flush=True)

        problem: Optional[str] = None
        retryable = True
        if not infos:
            problem = ("No OAK device found. Check the USB-C cable and that no other program "
                       "has the camera open.")
        else:
            target: Optional[dai.DeviceInfo] = None
            if wanted is None:
                target = infos[0]
            else:
                for info in infos:
                    if (_safe(info.getDeviceId, "") or "").upper() == wanted:
                        target = info
                        break
            if target is None:
                found = ", ".join(_safe(i.getDeviceId, "?") or "?" for i in infos)
                problem = ("Device {} not found (unplugged, or in use by another process). "
                           "Found: {}.".format(device_id, found))
                # Cameras were enumerated but none has this id: almost always a typo, which no
                # retry can fix (a busy or rebooting camera shows up as an EMPTY enumeration
                # and is retried).  Fail now instead of burning retries x retry_wait_s.
                retryable = False
            else:
                target_id = _safe(target.getDeviceId, "") or (device_id or "?")
                try:
                    if max_usb_speed is None:
                        return dai.Device(target)
                    return dai.Device(target, max_usb_speed)
                except RuntimeError as exc:  # includes dai.XLinkError subclasses
                    text = str(exc)
                    if any(fragment in text for fragment in _BUSY_FRAGMENTS):
                        problem = "Device {} is in use by another process.".format(target_id)
                    else:
                        problem = "Failed to open device {}: {}: {}".format(
                            target_id, type(exc).__name__, _one_line(text))
                except Exception as exc:  # noqa: BLE001 - anything else the binding throws
                    problem = "Failed to open device {}: {}: {}".format(
                        target_id, type(exc).__name__, _one_line(str(exc)))

        if attempt < attempts and retryable:
            print("{} Retrying in {:.0f} s ({}/{})...".format(problem, retry_wait_s, attempt, attempts),
                  file=sys.stderr, flush=True)
            time.sleep(max(0.0, retry_wait_s))
        else:
            print(problem, file=sys.stderr, flush=True)
            sys.exit(EXIT_NO_DEVICE)

    # Unreachable - the loop either returns or exits - but keeps type checkers honest.
    sys.exit(EXIT_NO_DEVICE)


def print_device_summary(device: dai.Device) -> str:
    """Print (and return) one line: name, product, device id, platform and USB speed.

    Prints a loud warning to stderr when the link is not USB 3 (``SUPER`` / ``SUPER_PLUS``),
    because a USB 2 fallback (``HIGH``) costs ~10x bandwidth and silently drops frames.
    """
    name = _safe(device.getDeviceName, "?")
    product = _safe(device.getProductName, "?")
    dev_id = _safe(device.getDeviceId, "?")
    platform = _safe(device.getPlatformAsString, "?")
    speed = _safe(device.getUsbSpeed, None)
    speed_name = _enum_name(speed) if speed is not None else "?"
    usb_ok = speed in _GOOD_USB_SPEEDS

    line = "Device: {} ({}) id={} platform={} usb={}{}".format(
        name, product, dev_id, platform, speed_name, "" if usb_ok else " (WARNING: not USB3)")
    print(line, flush=True)
    if not usb_ok:
        print("!!! WARNING: USB speed is {} instead of SUPER (USB 3). Expect ~10x less bandwidth "
              "and dropped frames - use a USB 3 cable and a USB 3 port. !!!".format(speed_name),
              file=sys.stderr, flush=True)
    return line


# --------------------------------------------------------------------------------------
# Run bookkeeping
# --------------------------------------------------------------------------------------

class RunClock:
    """Wall-clock budget for a run, started at construction.

    ``duration <= 0`` means "no limit": :meth:`expired` is then always ``False``.
    """

    def __init__(self, duration: float = 0.0) -> None:
        self.duration: float = float(duration or 0.0)
        self.start: float = time.monotonic()

    def elapsed(self) -> float:
        """Seconds since construction."""
        return time.monotonic() - self.start

    def remaining(self) -> Optional[float]:
        """Seconds left (never negative), or ``None`` when there is no limit."""
        if self.duration <= 0:
            return None
        return max(0.0, self.duration - self.elapsed())

    def expired(self) -> bool:
        """``True`` once ``duration`` seconds have passed (``False`` forever when duration is 0)."""
        return self.duration > 0 and self.elapsed() >= self.duration


class FpsCounter:
    """Frames-per-second over a rolling time window.

    Call :meth:`tick` once per frame (or ``tick(n)`` for a batch of *n* messages) and read
    :meth:`fps`.  Returns ``0.0`` until at least two ticks fall inside the window.
    """

    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s: float = max(0.05, float(window_s))
        self.count: int = 0  #: total ticks since construction
        self._times: Deque[float] = collections.deque()

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._times and self._times[0] < cutoff:
            self._times.popleft()

    def tick(self, n: int = 1) -> None:
        """Record *n* frames arriving now."""
        now = time.monotonic()
        n = max(1, int(n))
        for _ in range(n):
            self._times.append(now)
        self.count += n
        self._trim(now)

    def fps(self) -> float:
        """Current rate in frames per second over the window."""
        now = time.monotonic()
        self._trim(now)
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0.0:
            return 0.0
        return (len(self._times) - 1) / span


class Periodic:
    """Rate limiter for periodic work such as headless status lines.

    ``if stats.ready(): print(...)`` fires at most once per ``interval_s`` seconds.
    """

    def __init__(self, interval_s: float = 1.0, first_immediately: bool = False) -> None:
        self.interval_s: float = max(0.0, float(interval_s))
        now = time.monotonic()
        self._next: float = now if first_immediately else now + self.interval_s

    def ready(self) -> bool:
        """``True`` when the interval has elapsed since the last ``True``; re-arms itself."""
        now = time.monotonic()
        if now < self._next:
            return False
        self._next = now + self.interval_s
        return True


# --------------------------------------------------------------------------------------
# Visualisation
# --------------------------------------------------------------------------------------

def colorize_depth(depth_mm: np.ndarray, max_mm: float = 5000.0,
                   colormap: int = cv2.COLORMAP_JET) -> np.ndarray:
    """Colour-map a depth image (uint16 millimetres from ``StereoDepth.depth``) to BGR uint8.

    Near = red/warm, far (>= ``max_mm``) = blue; invalid pixels (depth ``0``, or NaN/inf for
    float input) are black.  Works for any 2-D numeric array.
    """
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("colorize_depth expects a 2-D depth image, got shape {}".format(depth.shape))
    invalid = depth <= 0
    depth_f = depth.astype(np.float32, copy=False)
    if depth.dtype.kind == "f":
        invalid = invalid | ~np.isfinite(depth_f)
        depth_f = np.nan_to_num(depth_f, nan=0.0, posinf=0.0, neginf=0.0)
    scale = 255.0 / max(float(max_mm), 1e-6)
    near_hot = (255.0 - np.clip(depth_f * scale, 0.0, 255.0)).astype(np.uint8)
    bgr = cv2.applyColorMap(near_hot, colormap)
    bgr[invalid] = 0
    return bgr


def colorize_disparity(disp: np.ndarray, max_disp: float,
                       colormap: int = cv2.COLORMAP_JET) -> np.ndarray:
    """Colour-map a disparity image to BGR uint8.

    ``max_disp`` should be ``stereo.initialConfig.getMaxDisparity()`` (v3 disparity is uint16
    scaled to that full range, e.g. 8192 for the DEFAULT preset).  Near (high disparity) =
    red/warm, far = blue; disparity ``0`` (invalid) is black.
    """
    disparity = np.asarray(disp)
    if disparity.ndim != 2:
        raise ValueError("colorize_disparity expects a 2-D image, got shape {}".format(disparity.shape))
    invalid = disparity <= 0
    disp_f = disparity.astype(np.float32, copy=False)
    if disparity.dtype.kind == "f":
        invalid = invalid | ~np.isfinite(disp_f)
        disp_f = np.nan_to_num(disp_f, nan=0.0, posinf=0.0, neginf=0.0)
    scale = 255.0 / max(float(max_disp), 1e-6)
    scaled = np.clip(disp_f * scale, 0.0, 255.0).astype(np.uint8)
    bgr = cv2.applyColorMap(scaled, colormap)
    bgr[invalid] = 0
    return bgr


def draw_label(img: np.ndarray, text: str, org: Tuple[int, int] = (10, 25),
               scale: float = 0.6, color: Tuple[int, int, int] = (255, 255, 255),
               thickness: int = 1) -> np.ndarray:
    """Draw readable overlay text (dark outline + light fill) on *img* in place and return it.

    *org* is the bottom-left corner of the first line (OpenCV convention).  Multi-line
    *text* (newline separated) is drawn line by line.  Works on BGR and single-channel images.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_h = int(round(30 * scale)) + 4
    x, y = int(org[0]), int(org[1])
    for i, line in enumerate(str(text).split("\n")):
        pos = (x, y + i * line_h)
        cv2.putText(img, line, pos, font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(img, line, pos, font, scale, color, thickness, cv2.LINE_AA)
    return img


def wait_key_quit(delay_ms: int = 1) -> Optional[str]:
    """Pump OpenCV's event loop for *delay_ms* and report ``'q'``, ``'s'`` or ``None``.

    ``'q'`` for q/Q/Esc (quit), ``'s'`` for s/S (save/snapshot), ``None`` for anything else.
    Only meaningful when at least one OpenCV window is open; do not call it in headless mode.
    """
    key = cv2.waitKey(int(delay_ms))
    if key < 0:
        return None
    key &= 0xFF
    if key in (ord("q"), ord("Q"), 27):
        return "q"
    if key in (ord("s"), ord("S")):
        return "s"
    return None


# --------------------------------------------------------------------------------------
# Output paths
# --------------------------------------------------------------------------------------

def ensure_dir(path: Union[str, Path]) -> Path:
    """Create *path* (and parents) if needed and return it as a ``Path``."""
    result = Path(path)
    result.mkdir(parents=True, exist_ok=True)
    return result


def timestamp_str() -> str:
    """Local-time stamp ``YYYYmmdd_HHMMSS`` (safe in file names on Windows and Linux)."""
    return time.strftime("%Y%m%d_%H%M%S")


def timestamp_dir(base: Union[str, Path], prefix: str) -> Path:
    """Create and return ``base/<prefix>_<timestamp>`` (``base/<timestamp>`` if prefix is empty)."""
    name = "{}_{}".format(prefix, timestamp_str()) if prefix else timestamp_str()
    return ensure_dir(Path(base) / name)


def timestamp_name(prefix: str, ext: str) -> str:
    """File name ``<prefix>_<timestamp>.<ext>`` (``ext`` with or without the leading dot)."""
    suffix = str(ext).lstrip(".")
    stem = "{}_{}".format(prefix, timestamp_str()) if prefix else timestamp_str()
    return "{}.{}".format(stem, suffix) if suffix else stem


# --------------------------------------------------------------------------------------
# Queue helpers
# --------------------------------------------------------------------------------------

def drain_queues(*queues: dai.MessageQueue) -> int:
    """Discard everything currently waiting in the given output queues; return the count."""
    total = 0
    for queue in queues:
        try:
            total += len(queue.tryGetAll())
        except Exception:  # noqa: BLE001 - a closed queue simply has nothing to drain
            pass
    return total


def warm_up(*queues: dai.MessageQueue, seconds: float = 1.5) -> int:
    """Let auto-exposure settle after ``pipeline.start()``: sleep, then drain the queues.

    The first ~1 s of frames after start are taken at maximum gain/exposure (saturated or
    black) and depth is all zero; the official ``point_cloud_visualizer.py`` does the same.
    Returns the number of stale messages discarded.
    """
    time.sleep(max(0.0, float(seconds)))
    return drain_queues(*queues)


# --------------------------------------------------------------------------------------
# Self-test: `python camera/common.py` lists devices; `--open` also boots and summarises one.
# --------------------------------------------------------------------------------------

def _main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="List (and optionally open) OAK devices.")
    add_device_arg(parser)
    parser.add_argument("--open", action="store_true", help="open the device and print its summary")
    args = parser.parse_args(argv)
    if not args.open:
        print(format_device_table(list_devices()))
        return 0
    device = open_device(args.device_id, retries=1 if args.device_id else 3)
    try:
        print_device_summary(device)
    finally:
        device.close()
    return 0


if __name__ == "__main__":
    sys.exit(_main())
