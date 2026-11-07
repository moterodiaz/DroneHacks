"""Live DepthAI v3 source for cuVSLAM: synchronised stereo GRAY8 pairs plus raw BNO086 IMU samples.

:class:`OakStereoImuSource` builds the pipeline the official PyCuVSLAM OAK-D example
(``examples/oak-d/run_stereo.py``, v17.0.0) uses, extended with the IMU node the
stereo-inertial mode needs, and hands out plain numpy data so the callers
(``datasets/07_record_dataset.py``, ``methods/cuvslam-vio/08_cuvslam_track.py``, ``cuvslam_oak.tracking``)
never touch DepthAI messages themselves.  Nothing here imports ``cuvslam``.

Pipeline (all facts from ``camera/docs/depthai-v3-cheatsheet.md`` and ``methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md``)::

    Camera CAM_B (left)  --requestOutput((w,h), GRAY8, CROP, fps)--> Sync.inputs["left"]  \\
                                                                                             Sync.out -> queue (non-blocking)
    Camera CAM_C (right) --requestOutput((w,h), GRAY8, CROP, fps)--> Sync.inputs["right"] /
    IMU (ACCELEROMETER_RAW + GYROSCOPE_RAW @ imu_rate_hz, batch threshold imu_batch, max 10) --> queue (non-blocking)

* ``dai.node.Camera.build(socket, sensorFps=fps)`` on CAM_B and CAM_C (the OV9282 global-shutter
  mono pair; CAM_A, the autofocus colour camera, is never used for VSLAM).
* ``requestOutput((width, height), type=GRAY8, resizeMode=CROP, fps=fps)``.  ``CROP`` is the mode
  whose intrinsics ``CalibrationHandler.getCameraIntrinsics(socket, w, h)`` (``keepAspectRatio=True``)
  reproduces exactly; for the supported 16:10 sizes (1280x800 native, 640x400, 320x200) it is a
  pure scale with no crop at all.  ``STRETCH`` would scale fx/fy differently and ``LETTERBOX``
  has no ``getCameraIntrinsics`` equivalent, so neither is offered.  1280x720 (the example's
  size) is a centre crop that moves ``cy`` by -40 px; it works but is warned about.
* ``dai.node.Sync`` with ``setSyncThreshold(timedelta(seconds=0.5 / fps))`` exactly as the
  example; timestamp source left at ``DEFAULT`` (= ``DEVICE`` when the node runs on the device).
* ``dai.node.IMU`` with ``enableIMUSensor([ACCELEROMETER_RAW, GYROSCOPE_RAW], imu_rate_hz)``,
  ``setBatchReportThreshold(imu_batch)``, ``setMaxBatchReports(10)``.  RAW streams are
  "sensor-native frame, no IMU extrinsics applied" and pair with
  ``rig_from_imu = getImuToCameraExtrinsics(rig_socket)`` (``cuvslam_oak.rig`` default,
  ``imu_frame='raw'``).  No ``ROTATION_VECTOR`` on the same node: it would throttle the packet
  stream to the slowest enabled rate.
* Both output queues are non-blocking (the newest messages win when the host falls behind) and
  every loss is visible in :meth:`OakStereoImuSource.stats` as a timestamp gap.
* The ``dai.CalibrationHandler`` is read once in :meth:`OakStereoImuSource.build`, before
  ``pipeline.start()``, and cached: no RPC is issued while streaming and
  :meth:`OakStereoImuSource.calibration` still works after :meth:`OakStereoImuSource.stop`.

Timestamp domain (the choice, as ``methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md`` section 6 prescribes):

* ``StereoFrame.ts_ns`` = integer nanoseconds of the **left** frame's
  ``getTimestampDevice(dai.CameraExposureOffset.MIDDLE)``: the device monotonic clock (shared by
  the cameras and the IMU, stamped on the RVC2), at mid-exposure (DepthAI stores the END of
  exposure; the OV9282 is a global shutter, so mid-exposure is the single capture instant; with
  auto-exposure up to 30 ms the two differ by up to 15 ms).  Both images of the pair get the
  left stamp; the right-minus-left difference is reported in ``lr_ts_diff_us`` (11-16 us on this
  unit) and pairs beyond ``max_lr_skew_us`` (default 1 ms, cuVSLAM's stated tolerance) are
  dropped and counted.
* ``ImuSample.ts_ns`` = ``packet.acceleroMeter.tsDevice`` (``sec * 1e9 + nsec``, nanosecond
  resolution, same device clock; the gyroscope report's own stamp is kept in ``gyro_ts_ns`` and
  the largest accel/gyro skew is a statistic).
* Conversions are integer arithmetic (``timedelta`` -> ``days/seconds/microseconds``), never
  ``total_seconds() * 1e9`` through a float.
* Host-synchronised stamps (``getTimestamp()``, comparable with ``dai.Clock.now()``) are carried
  along as ``host_ts_ns`` for sidecar files only; they are re-estimated by DepthAI's clock sync
  and can step, so cuVSLAM never sees them.  One constant :attr:`OakStereoImuSource.epoch_offset_ns`
  (computed once from the first message: wall-clock minus ``dai.Clock.now()`` plus the message's
  host-minus-device offset) turns device time into Unix-epoch nanoseconds when a dataset wants
  19-digit stamps; the source itself always reports raw device time.

Ordering rules for the consumer (cuVSLAM: ``track()`` strictly increasing, ``register_imu_measurement``
non-decreasing, the two interleaved non-decreasing, all from one thread): call
:meth:`OakStereoImuSource.imu_samples` before every ``track()`` and register the samples whose
``ts_ns <= frame.ts_ns`` first, or simply iterate :meth:`OakStereoImuSource.events`, which
yields :class:`ImuSample` and :class:`StereoFrame` objects already in that order.  Frames whose
device timestamp does not increase, IMU samples that go backwards and IMU packets without both
reports are dropped and counted, never yielded.  :meth:`OakStereoImuSource.stop` drains the IMU
queue one last time *before* the pipeline stops, so the samples that arrived after the last
frame are returned once more by :meth:`OakStereoImuSource.imu_samples` and flushed at the end
of :meth:`OakStereoImuSource.events` (only what the device had already delivered; nothing is
waited for).

Errors.  Three classes, so a consumer can tell "the run ended" from "the stream is wrong":

* :class:`OakSourceStopped` (expected end): the consumer called :meth:`OakStereoImuSource.stop`,
  the pipeline stopped on its own (a node threw: the queue is closed / ``isRunning()`` is
  false), or the stereo queue raised a device/communication error (an XLink fault surfaces
  from ``MessageQueue.get()`` as a plain exception; recorded as ``'stereo queue error: ...'``).
  :meth:`OakStereoImuSource.frames` / :meth:`OakStereoImuSource.events` swallow it and simply
  end; ``stats()['stop_reason']`` says why.
* :class:`OakStreamError` (fatal fault): the mandatory first-frame guard failed
  (``strict_intrinsics``), a frame has the wrong size/dtype/channels, the two images differ in
  size, or the watchdog fired (``max_silence_s`` without a pair while ``isRunning()``).  The
  source stops itself (device released), records the full message in ``stats()['stop_reason']``
  and ``stats()['fatal_error']``, and the exception **propagates** out of ``frames()`` /
  ``events()`` / ``next_frame()``; every later call raises it again, so a retry can never
  resume on a stream the rig does not describe.  One documented exception: a same-aspect
  integer-binned sensor mode (``getSourceSize()`` 640x400 for a 640x400 request; the OV9282 has
  a native 640x400@255.7 mode) whose delivered intrinsics and distortion still match the
  calibration is **not** a fault (calibration doc 1.3: the 0.25 px half-pixel offset of binning
  is below what cuVSLAM resolves); it is recorded as ``intrinsics_check[role]['source_size_note']``
  and warned about once.
* :class:`OakSourceError` (base, ``RuntimeError``): also the plain misuse cases (not started,
  depthai missing, invalid arguments) and a pipeline build that failed part-way (the device is
  closed before the error propagates, so a ``with`` block that never entered leaks nothing).

Typical use::

    device = common.open_device(args.device_id)            # camera/common.py (exit 2 on failure)
    calib = oak_source.dump_calibration(device)             # eepromToJson() dict for rig.py / calibration.json
    try:
        with OakStereoImuSource(device, 1280, 800, 30.0, 200) as src:   # start() on enter, stop() on exit
            src.warm_up(1.5)                                # auto-exposure settles; discards are counted
            for event in src.events():                      # ImuSample / StereoFrame in timestamp order
                ...                                         # ends cleanly on src.stop() / pipeline death
    except oak_source.OakStreamError as exc:                # wrong stream: abort with a clear message
        sys.exit("oak source: {}".format(exc))
    print(src.stats())

``pipeline.stop()`` (called by :meth:`OakStereoImuSource.stop`) closes the device: create a fresh
``dai.Device`` for every source (cheat-sheet section 1.2).  Statistics: frames delivered, frame
gaps (> 1.5 frame periods between consecutive left stamps *received*, with an estimate of the
frames lost; pairs the source itself drops for L/R skew did arrive and are therefore not gaps),
sequence-number gaps (left ``getSequenceNum()`` of every pair that *arrived*, accepted or
dropped here, so only numbers that never reached the host count), L/R sync violations, IMU
samples, IMU gaps (> 2.5 sample periods, the ``camera/04_imu.py`` rule; the BNO086 sequence
byte steps by 2 routinely and is not used), out-of-order IMU samples, accel/gyro skew, measured
rates, and the first-frame intrinsics check.

Runs unchanged on Windows (recording/tests), WSL2 and the Jetson (Python 3.10, pathlib, no
Windows-only calls).  Importing this module needs only numpy; ``depthai`` is imported at module
load when available and required only when a source is built.

``python -m cuvslam_oak.oak_source [--width W --height H --fps F --imu-rate R --imu-batch N]``
prints :meth:`OakStereoImuSource.plan` for that configuration and ``--self-check`` runs the
device-free checks (pure helpers, the pipeline graph on a stub pipeline, the first-frame guard
and the IMU tail flush on stub frames), exit code 1 on failure.  This entry point never opens a
camera; the live scripts (``datasets/07_record_dataset.py``) do that.
"""

from __future__ import annotations

import collections
import datetime
import json
import math
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, Iterator, List, Optional, Tuple, Union

import numpy as np

try:  # depthai is only needed to build/run a source; the pure helpers work without it
    import depthai as dai
except ImportError:  # pragma: no cover - exercised on hosts without the wheel
    dai = None  # type: ignore[assignment]

__all__ = [
    "NS_PER_S",
    "LEFT_SOCKET",
    "RIGHT_SOCKET",
    "DEFAULT_SIZE",
    "SUPPORTED_SIZES",
    "DEFAULT_FPS",
    "DEFAULT_IMU_RATE_HZ",
    "MAX_IMU_RATE_HZ",
    "IMU_SENSORS",
    "MAX_BATCH_REPORTS",
    "FRAME_GAP_FACTOR",
    "IMU_GAP_FACTOR",
    "MAX_LR_SKEW_US",
    "INTRINSICS_TOL_PX",
    "DISTORTION_TOL",
    "DEFAULT_MAX_SILENCE_S",
    "EXPOSURE_OFFSETS",
    "OakSourceError",
    "OakStreamError",
    "OakSourceStopped",
    "OakSourceWarning",
    "StereoFrame",
    "ImuSample",
    "StreamStats",
    "timedelta_ns",
    "to_ns",
    "imu_sample_from_packet",
    "stereo_frame_from_messages",
    "calibration_handler",
    "dump_calibration",
    "device_info",
    "OakStereoImuSource",
]

NS_PER_S = 1_000_000_000

#: Stereo sockets of the OAK-D S2 (``calib.getStereoLeftCameraId()`` / ``RightCameraId()``).
LEFT_SOCKET = "CAM_B"
RIGHT_SOCKET = "CAM_C"
#: Native OV9282 mode; the EEPROM mono calibration is at this size.
DEFAULT_SIZE: Tuple[int, int] = (1280, 800)
#: Sizes whose ``getCameraIntrinsics`` are an exact scaling of the calibration (16:10, no crop).
SUPPORTED_SIZES: Tuple[Tuple[int, int], ...] = ((1280, 800), (640, 400), (320, 200))
DEFAULT_FPS = 30.0
DEFAULT_IMU_RATE_HZ = 200
#: Luxonis: above 400 Hz the BNO08X gyroscope on RVC2 "can produce occasional jitter".
MAX_IMU_RATE_HZ = 400
IMU_SENSORS: Tuple[str, str] = ("ACCELEROMETER_RAW", "GYROSCOPE_RAW")
MAX_BATCH_REPORTS = 10
#: A gap between consecutive left-frame device stamps above this many frame periods is a drop.
FRAME_GAP_FACTOR = 1.5
#: Same rule for IMU samples (``camera/04_imu.py`` GAP_FACTOR).
IMU_GAP_FACTOR = 2.5
#: cuVSLAM: all cameras of one ``track()`` call must be captured within 1 ms.
MAX_LR_SKEW_US = 1000.0
#: First-frame guard: delivered ``ImgTransformation`` intrinsics vs ``getCameraIntrinsics``.
INTRINSICS_TOL_PX = 1e-2
#: First-frame guard: delivered distortion coefficients vs the EEPROM's, relative to the largest
#: coefficient (this unit's rational model has |k| up to ~40; the binding carries floats).
DISTORTION_TOL = 1e-4
#: Watchdog: seconds without a stereo pair while ``isRunning()`` is true before the source aborts.
DEFAULT_MAX_SILENCE_S = 10.0
EXPOSURE_OFFSETS: Tuple[str, str, str] = ("start", "middle", "end")

_SOCKET_ROLES = ((LEFT_SOCKET, "left"), (RIGHT_SOCKET, "right"))


class OakSourceError(RuntimeError):
    """A configuration or runtime problem of the live source (one clear message, no traceback needed)."""


class OakStreamError(OakSourceError):
    """Fatal stream fault: the delivered frames are not what the rig describes (intrinsics /
    source size / distortion mismatch in strict mode, wrong image size, dtype or channels,
    L/R size mismatch) or the watchdog fired.  The source stops itself and the error propagates
    out of :meth:`OakStereoImuSource.frames` / :meth:`OakStereoImuSource.events`."""


class OakSourceStopped(OakSourceError):
    """Expected end of the stream: :meth:`OakStereoImuSource.stop` was called, or the pipeline
    stopped on its own (queue closed after a node exception, ``isRunning()`` false).
    Swallowed by :meth:`OakStereoImuSource.frames`; the reason is in ``stats()['stop_reason']``."""


class OakSourceWarning(UserWarning):
    """Non-fatal oddities: unusual sizes, unavailable intrinsics check, sequence mismatches."""


def _warn(message: str) -> None:
    warnings.warn(message, OakSourceWarning, stacklevel=3)


def _safe(getter: Any, default: Any = None) -> Any:
    """``getter()`` or *default* when the binding raises (device queries can fail on odd hardware)."""
    try:
        return getter()
    except Exception:  # noqa: BLE001 - metadata is best effort
        return default


def _enum_name(value: Any) -> str:
    return str(getattr(value, "name", value))


def _integer_scale(source: Tuple[int, int], native: Tuple[int, int]) -> Optional[int]:
    """``n >= 2`` when *native* is exactly ``n`` times *source* on both axes (an ``n x n`` binned
    sensor mode, e.g. 640x400 for the 1280x800 OV9282), else ``None``."""
    sw, sh = int(source[0]), int(source[1])
    nw, nh = int(native[0]), int(native[1])
    if sw <= 0 or sh <= 0 or nw % sw or nh % sh:
        return None
    n = nw // sw
    return n if n >= 2 and n == nh // sh else None


def _require_dai() -> Any:
    if dai is None:
        raise OakSourceError("depthai is not installed; the live OAK source needs depthai==3.10.0 "
                             "(pip install depthai==3.10.0)")
    return dai


# --------------------------------------------------------------------------------------
# Timestamps
# --------------------------------------------------------------------------------------

def timedelta_ns(td: datetime.timedelta) -> int:
    """``datetime.timedelta`` -> exact integer nanoseconds (microsecond resolution, no float)."""
    return (td.days * 86_400 + td.seconds) * NS_PER_S + td.microseconds * 1_000


def to_ns(value: Any) -> int:
    """Integer nanoseconds from a ``timedelta``, a ``dai.Timestamp`` (``.sec``/``.nsec``) or an int.

    ``IMUReport.tsDevice`` keeps nanoseconds, ``ImgFrame.getTimestampDevice()`` returns a
    ``timedelta`` (microseconds): both come from the same device clock and are converted with
    integer arithmetic only.  Floats are rejected: ``total_seconds() * 1e9`` loses ~250 ns at
    epoch magnitude.
    """
    if isinstance(value, datetime.timedelta):
        return timedelta_ns(value)
    if isinstance(value, bool):
        raise TypeError("to_ns: bool is not a timestamp")
    if isinstance(value, (int, np.integer)):
        return int(value)
    sec = getattr(value, "sec", None)
    nsec = getattr(value, "nsec", None)
    if sec is not None and nsec is not None:
        return int(sec) * NS_PER_S + int(nsec)
    raise TypeError("to_ns: expected timedelta, dai.Timestamp or int, got {}".format(type(value).__name__))


# --------------------------------------------------------------------------------------
# Data records
# --------------------------------------------------------------------------------------

@dataclass
class StereoFrame:
    """One synchronised stereo pair as cuVSLAM wants it.

    ``ts_ns``: left frame, device monotonic clock, mid-exposure (see the module docstring).
    ``left``/``right``: ``(height, width)`` uint8, C-contiguous copies (safe to keep).
    ``seq``: left ``getSequenceNum()``.  ``lr_ts_diff_us``: right minus left device stamp.
    Extras for sidecars: ``host_ts_ns`` (host-synced stamp, same exposure offset),
    ``exposure_us``, ``right_seq``, ``right_ts_ns``.
    """

    ts_ns: int
    left: np.ndarray
    right: np.ndarray
    seq: int
    lr_ts_diff_us: float
    host_ts_ns: int = 0
    exposure_us: float = 0.0
    right_seq: int = -1
    right_ts_ns: int = 0

    @property
    def kind(self) -> str:
        return "stereo"

    @property
    def images(self) -> List[np.ndarray]:
        """``[left, right]`` in rig camera order (cuVSLAM camera index 0 and 1)."""
        return [self.left, self.right]

    @property
    def shape(self) -> Tuple[int, int]:
        return (int(self.left.shape[0]), int(self.left.shape[1]))


@dataclass
class ImuSample:
    """One BNO086 packet in the sensor-native frame (RAW streams).

    Positional field order is ``(ts_ns, gyro, accel)``: **gyro first**, exactly like
    ``cuvslam_oak.dataset.ImuSample`` and the ``imu0/data.csv`` columns (``w_RS_S_x..z`` then
    ``a_RS_S_x..z``), so the two record types can be built interchangeably (positional
    ``(ts_ns, gyro, accel)``; neither is iterable, read the fields by name).
    ``gyro`` rad/s and ``accel`` m/s^2 (specific force, gravity included) are float64 ``(3,)``;
    ``ts_ns`` is the accelerometer report's device stamp.  ``gyro_ts_ns`` is the gyroscope
    report's own stamp, ``host_ts_ns`` the host-synced accelerometer stamp, ``sequence`` the
    8-bit report counter.  Prefer keyword construction anyway.
    """

    ts_ns: int
    gyro: np.ndarray
    accel: np.ndarray
    gyro_ts_ns: int = 0
    host_ts_ns: int = 0
    sequence: int = -1

    @property
    def kind(self) -> str:
        return "imu"

    @property
    def accel_gyro_skew_ns(self) -> int:
        return int(self.gyro_ts_ns - self.ts_ns) if self.gyro_ts_ns else 0

    def measurement_kwargs(self) -> Dict[str, Any]:
        """Keyword arguments for ``cuvslam.ImuMeasurement(**kwargs)`` (v17.0.0 keyword-only ctor)."""
        return {
            "timestamp_ns": int(self.ts_ns),
            "linear_accelerations": [float(v) for v in self.accel],
            "angular_velocities": [float(v) for v in self.gyro],
        }


# --------------------------------------------------------------------------------------
# Gap / rate bookkeeping (pure, testable without depthai)
# --------------------------------------------------------------------------------------

class StreamStats:
    """Timestamp bookkeeping for one stream: count, gaps, drop estimate, rate, ordering.

    ``note(ts_ns)`` returns ``'ok'``, ``'gap'`` (counted and accepted) or ``'backwards'``
    (rejected: not counted, ``last_ts_ns`` unchanged; the caller drops the message).
    ``skip(ts_ns)`` is for a message that *arrived* but is dropped by policy (an L/R skew
    violation): it advances ``last_ts_ns`` and detects a gap **before** it like ``note`` but
    does not count it as delivered (``skipped`` instead), so ``gaps``/``dropped_estimate`` keep
    meaning "messages that never reached the host".  ``strict=True`` (frames) rejects equal
    stamps too, because cuVSLAM ``track()`` needs strictly increasing timestamps; IMU samples
    only need to be non-decreasing.  ``measured_rate_hz`` is accepted messages per second of
    device time between the first and the last message seen.
    """

    def __init__(self, nominal_rate_hz: float, gap_factor: float, *, strict: bool = False) -> None:
        rate = float(nominal_rate_hz)
        if not rate > 0.0:
            raise ValueError("nominal_rate_hz must be > 0, got {!r}".format(nominal_rate_hz))
        self.nominal_rate_hz = rate
        self.period_ns = int(round(NS_PER_S / rate))
        self.gap_factor = float(gap_factor)
        self.strict = bool(strict)
        self.count = 0
        self.skipped = 0
        self.gaps = 0
        self.dropped_estimate = 0
        self.out_of_order = 0
        self.max_gap_ns = 0
        self.first_ts_ns: Optional[int] = None
        self.last_ts_ns: Optional[int] = None

    def _advance(self, ts: int) -> str:
        result = "ok"
        last = self.last_ts_ns
        if last is None:
            self.first_ts_ns = ts
        else:
            gap = ts - last
            if gap > self.max_gap_ns:
                self.max_gap_ns = gap
            if gap > self.gap_factor * self.period_ns:
                self.gaps += 1
                self.dropped_estimate += max(1, int(round(gap / self.period_ns)) - 1)
                result = "gap"
        self.last_ts_ns = ts
        return result

    def _is_backwards(self, ts: int) -> bool:
        last = self.last_ts_ns
        return last is not None and (ts < last or (self.strict and ts == last))

    def note(self, ts_ns: int) -> str:
        ts = int(ts_ns)
        if self._is_backwards(ts):
            self.out_of_order += 1
            return "backwards"
        result = self._advance(ts)
        self.count += 1
        return result

    def skip(self, ts_ns: int) -> str:
        """Register a message that arrived but is dropped by policy (see the class docstring)."""
        ts = int(ts_ns)
        if self._is_backwards(ts):
            return "backwards"          # would have been rejected anyway; nothing to advance
        result = self._advance(ts)
        self.skipped += 1
        return result

    def span_s(self) -> float:
        if self.first_ts_ns is None or self.last_ts_ns is None:
            return 0.0
        return (self.last_ts_ns - self.first_ts_ns) / NS_PER_S

    def measured_rate_hz(self) -> float:
        """Rate from the device timestamps (accepted messages over first-to-last seen)."""
        span = self.span_s()
        if self.count < 2 or span <= 0.0:
            return 0.0
        return (self.count - 1) / span

    def as_dict(self) -> Dict[str, Any]:
        return {
            "count": self.count,
            "skipped": self.skipped,
            "gaps": self.gaps,
            "dropped_estimate": self.dropped_estimate,
            "out_of_order": self.out_of_order,
            "max_gap_ms": self.max_gap_ns / 1e6,
            "nominal_rate_hz": self.nominal_rate_hz,
            "measured_rate_hz": round(self.measured_rate_hz(), 3),
            "span_s": round(self.span_s(), 3),
            "first_ts_ns": self.first_ts_ns,
            "last_ts_ns": self.last_ts_ns,
        }


# --------------------------------------------------------------------------------------
# Message -> record conversion (duck-typed: works on dai messages and on test doubles)
# --------------------------------------------------------------------------------------

def imu_sample_from_packet(packet: Any) -> Optional[ImuSample]:
    """Convert one ``dai.IMUPacket`` (``acceleroMeter`` + ``gyroscope`` reports) into an :class:`ImuSample`.

    Returns ``None`` when either report is missing (device stamp 0): with only the two RAW
    sensors enabled at one rate every packet carries both, so ``None`` means a configuration
    problem (or a report that has not been delivered yet) and is counted by the caller.
    Values are copied without conversion (DepthAI m/s^2 and rad/s are cuVSLAM's units).
    """
    acc = getattr(packet, "acceleroMeter", None)
    gyr = getattr(packet, "gyroscope", None)
    if acc is None or gyr is None:
        return None
    ts_ns = to_ns(acc.tsDevice)
    gyro_ts_ns = to_ns(gyr.tsDevice)
    if ts_ns <= 0 or gyro_ts_ns <= 0:
        return None
    accel = np.array([float(acc.x), float(acc.y), float(acc.z)], dtype=np.float64)
    gyro = np.array([float(gyr.x), float(gyr.y), float(gyr.z)], dtype=np.float64)
    host_ts = _safe(lambda: to_ns(acc.getTimestamp()), 0)
    return ImuSample(ts_ns=ts_ns, gyro=gyro, accel=accel, gyro_ts_ns=gyro_ts_ns,
                     host_ts_ns=int(host_ts or 0), sequence=int(_safe(lambda: acc.sequence, -1)))


def _gray_image(frame: Any, expected_shape: Optional[Tuple[int, int]], what: str) -> np.ndarray:
    """``getCvFrame()`` as a C-contiguous ``(H, W)`` uint8 array, validated against *expected_shape*."""
    img = np.asarray(frame.getCvFrame())
    if img.ndim == 3 and img.shape[2] == 1:
        img = img[:, :, 0]
    if img.ndim != 2:
        raise OakStreamError("{} frame is not single-channel (shape {}); request GRAY8 on the mono "
                             "sockets".format(what, img.shape))
    if img.dtype != np.uint8:
        raise OakStreamError("{} frame dtype is {} (expected uint8)".format(what, img.dtype))
    if expected_shape is not None and tuple(img.shape) != tuple(expected_shape):
        raise OakStreamError("{} frame is {}x{} but {}x{} was requested".format(
            what, img.shape[1], img.shape[0], expected_shape[1], expected_shape[0]))
    return np.ascontiguousarray(img)


def stereo_frame_from_messages(left: Any, right: Any, *, exposure_offset: Any = None,
                               expected_shape: Optional[Tuple[int, int]] = None) -> StereoFrame:
    """Build a :class:`StereoFrame` from two ``dai.ImgFrame`` messages of one Sync group.

    *exposure_offset* is a ``dai.CameraExposureOffset`` (``MIDDLE`` in the source) passed to
    ``getTimestampDevice``/``getTimestamp``; ``None`` uses the plain getters (end of exposure).
    Raises :class:`OakStreamError` when an image is not a ``(H, W)`` uint8 array of
    *expected_shape* or the two images differ in size.  No gap/skew policy here: that is the
    source's job.
    """

    def dev_ts(frame: Any) -> int:
        if exposure_offset is None:
            return to_ns(frame.getTimestampDevice())
        return to_ns(frame.getTimestampDevice(exposure_offset))

    def host_ts(frame: Any) -> int:
        if exposure_offset is None:
            return to_ns(frame.getTimestamp())
        return to_ns(frame.getTimestamp(exposure_offset))

    t_left = dev_ts(left)
    t_right = dev_ts(right)
    left_img = _gray_image(left, expected_shape, "left")
    right_img = _gray_image(right, expected_shape, "right")
    if left_img.shape != right_img.shape:
        raise OakStreamError("left {} and right {} frames differ in size".format(left_img.shape, right_img.shape))
    exposure = _safe(lambda: timedelta_ns(left.getExposureTime()) / 1e3, 0.0)
    return StereoFrame(
        ts_ns=t_left,
        left=left_img,
        right=right_img,
        seq=int(left.getSequenceNum()),
        lr_ts_diff_us=(t_right - t_left) / 1e3,
        host_ts_ns=int(_safe(lambda: host_ts(left), 0) or 0),
        exposure_us=float(exposure or 0.0),
        right_seq=int(_safe(right.getSequenceNum, -1)),
        right_ts_ns=t_right,
    )


# --------------------------------------------------------------------------------------
# Calibration / device metadata
# --------------------------------------------------------------------------------------

def calibration_handler(source: Any) -> Any:
    """``dai.CalibrationHandler`` from an open ``dai.Device`` (``readCalibration()``), an existing
    handler (returned as is) or a path to a saved ``calibration.json`` (no device needed)."""
    if source is None:
        raise TypeError("calibration_handler: source is None (open the device first)")
    if hasattr(source, "eepromToJson"):
        return source
    if isinstance(source, (str, Path)):
        path = Path(source)
        if not path.is_file():
            raise FileNotFoundError("calibration file not found: {}".format(path))
        return _require_dai().CalibrationHandler(path)
    read = getattr(source, "readCalibration", None)
    if callable(read):
        return read()
    raise TypeError("calibration_handler: expected dai.Device, dai.CalibrationHandler or a calibration.json "
                    "path, got {}".format(type(source).__name__))


def dump_calibration(device: Any) -> Dict[str, Any]:
    """The EEPROM calibration as the ``eepromToJson()`` dict.

    Exactly what ``camera/05_record.py`` writes to ``calibration.json`` and what
    ``cuvslam_oak.rig.load_calibration_json`` / ``normalize_calibration`` / ``build_rig`` accept
    (top-level ``cameraData`` as ``[[socket_id, info], ...]``, ``imuExtrinsics``,
    ``stereoRectificationData``, ``version`` ...).  *device* may be an open ``dai.Device``, a
    ``dai.CalibrationHandler`` or a path to a saved dump (round-trips bit for bit).  Raises
    :class:`OakSourceError` when the device has no calibration.
    """
    handler = calibration_handler(device)
    data = handler.eepromToJson()
    if not isinstance(data, dict) or "cameraData" not in data:
        raise OakSourceError("device returned no EEPROM calibration (eepromToJson() has no 'cameraData')")
    return data


def device_info(device: Any) -> Dict[str, Any]:
    """Metadata block for ``dataset.json``: id, names, platform, USB speed, IMU, cameras, stereo pair.

    Every field is best effort (``None`` when the query fails or the method does not exist);
    nothing here boots or configures the device beyond what an open ``dai.Device`` already did.
    """

    def query(name: str, default: Any = None) -> Any:
        method = getattr(device, name, None)
        return _safe(method, default) if callable(method) else default

    speed = query("getUsbSpeed")
    imu_fw = query("getIMUFirmwareVersion")
    boot = query("getBootloaderVersion")
    cams = query("getConnectedCameras", []) or []
    sensors = query("getCameraSensorNames", {}) or {}
    pairs = query("getStereoPairs", []) or []
    stereo = None
    if pairs:
        pair = pairs[0]
        stereo = {"left": _enum_name(_safe(lambda: pair.left)), "right": _enum_name(_safe(lambda: pair.right)),
                  "baseline_cm": _safe(lambda: float(pair.baseline))}
    return {
        "device_id": query("getDeviceId"),
        "name": query("getDeviceName"),
        "product": query("getProductName"),
        "platform": query("getPlatformAsString"),
        "usb_speed": _enum_name(speed) if speed is not None else None,
        "usb3": speed is not None and _enum_name(speed) in ("SUPER", "SUPER_PLUS"),
        "imu": query("getConnectedIMU"),
        "imu_firmware": str(imu_fw) if imu_fw is not None else None,
        "bootloader": str(boot) if boot is not None else None,
        "cameras": [_enum_name(c) for c in cams],
        "camera_sensors": {_enum_name(k): str(v) for k, v in dict(sensors).items()},
        "stereo_pair": stereo,
        "depthai_version": getattr(dai, "__version__", None) if dai is not None else None,
    }


# --------------------------------------------------------------------------------------
# The live source
# --------------------------------------------------------------------------------------

class OakStereoImuSource:
    """Synchronised GRAY8 stereo pairs + raw IMU from an open ``dai.Device`` (see the module docstring).

    Parameters: *device* is an open ``dai.Device`` (from ``scripts.common.open_device``; it is
    closed by :meth:`stop`); *width*/*height* the request size (1280x800 native or 640x400;
    other sizes are accepted with a warning when they change the aspect ratio); *fps* the sensor
    rate; *imu_rate_hz* the report rate for both RAW sensors (200 default, 400 max recommended);
    *imu_batch* packets per ``IMUData`` message (1 = lowest latency); *gray* requests
    ``ImgFrame.Type.GRAY8`` (``False`` = the sensor default, RAW8 on the mono sockets, same
    ``(H, W)`` uint8 array).  Keyword options: *max_lr_skew_us* (drop pairs beyond it; ``None``
    keeps everything), *strict_intrinsics* (raise :class:`OakStreamError` on the first-frame
    guard instead of warning; a same-aspect integer-binned sensor mode whose intrinsics and
    distortion still match is never fatal, only noted), *exposure_offset* (``'middle'``
    default, ``'start'``, ``'end'``),
    *stereo_queue_size* / *imu_queue_size* (non-blocking output queues; the IMU default holds
    two seconds of messages), *max_silence_s* (watchdog: abort with :class:`OakStreamError`
    when no stereo pair arrives for that long although ``isRunning()`` stays true, i.e. the
    Sync node never emits; ``None`` polls forever, so callers must then enforce a duration).

    Lifecycle: :meth:`build` (pipeline + nodes + queues + cached calibration; a pre-made
    pipeline can be injected for introspection tests), :meth:`start`, :meth:`warm_up`,
    :meth:`frames` / :meth:`imu_samples` / :meth:`events`, :meth:`stop`; or use the object as a
    context manager.  All methods are meant for one consumer thread.
    """

    def __init__(self, device: Any, width: int = DEFAULT_SIZE[0], height: int = DEFAULT_SIZE[1],
                 fps: float = DEFAULT_FPS, imu_rate_hz: int = DEFAULT_IMU_RATE_HZ, imu_batch: int = 1,
                 gray: bool = True, *, max_lr_skew_us: Optional[float] = MAX_LR_SKEW_US,
                 strict_intrinsics: bool = True, exposure_offset: str = "middle",
                 stereo_queue_size: int = 8, imu_queue_size: Optional[int] = None,
                 max_silence_s: Optional[float] = DEFAULT_MAX_SILENCE_S) -> None:
        if device is None or isinstance(device, (str, Path)):
            raise TypeError("device must be an open dai.Device (use scripts.common.open_device(device_id)); "
                            "got {!r}".format(device))
        try:
            w, h = int(width), int(height)
        except (TypeError, ValueError):
            raise OakSourceError("width/height must be integers, got {!r}x{!r}".format(width, height)) from None
        if w <= 0 or h <= 0:
            raise OakSourceError("width/height must be positive, got {}x{}".format(w, h))
        if not (isinstance(fps, (int, float)) and math.isfinite(float(fps)) and float(fps) > 0.0):
            raise OakSourceError("fps must be a positive number, got {!r}".format(fps))
        try:
            rate = int(imu_rate_hz)
        except (TypeError, ValueError):
            raise OakSourceError("imu_rate_hz must be an integer, got {!r}".format(imu_rate_hz)) from None
        if rate <= 0 or rate > 1000:
            raise OakSourceError("imu_rate_hz must be within 1..1000, got {}".format(rate))
        try:
            batch = int(imu_batch)
        except (TypeError, ValueError):
            raise OakSourceError("imu_batch must be an integer, got {!r}".format(imu_batch)) from None
        if batch < 1 or batch > MAX_BATCH_REPORTS:
            raise OakSourceError("imu_batch must be within 1..{}, got {}".format(MAX_BATCH_REPORTS, batch))
        if exposure_offset not in EXPOSURE_OFFSETS:
            raise OakSourceError("exposure_offset must be one of {}, got {!r}".format(EXPOSURE_OFFSETS, exposure_offset))
        if max_lr_skew_us is not None and float(max_lr_skew_us) < 0.0:
            raise OakSourceError("max_lr_skew_us must be >= 0 or None")
        if int(stereo_queue_size) < 1:
            raise OakSourceError("stereo_queue_size must be >= 1")
        if max_silence_s is not None and not (math.isfinite(float(max_silence_s)) and float(max_silence_s) > 0.0):
            raise OakSourceError("max_silence_s must be a positive number or None, got {!r}".format(max_silence_s))

        self.device = device
        self.width, self.height = w, h
        self.fps = float(fps)
        self.imu_rate_hz = rate
        self.imu_batch = batch
        self.gray = bool(gray)
        self.max_lr_skew_us = None if max_lr_skew_us is None else float(max_lr_skew_us)
        self.strict_intrinsics = bool(strict_intrinsics)
        self.exposure_offset = exposure_offset
        self.stereo_queue_size = int(stereo_queue_size)
        self.imu_queue_size = (int(imu_queue_size) if imu_queue_size is not None
                               else max(64, int(math.ceil(2.0 * rate / batch))))
        self.max_silence_s = None if max_silence_s is None else float(max_silence_s)
        self.sync_threshold_s = 0.5 / self.fps

        if (w, h) not in SUPPORTED_SIZES:
            native_w, native_h = DEFAULT_SIZE
            if abs(w / h - native_w / native_h) > 1e-3:
                _warn("requested size {}x{} does not keep the native {}x{} aspect ratio: the CROP resize "
                      "moves the principal point (1280x720: cy - 40 px); getCameraIntrinsics(socket, {}, {}) "
                      "still describes the delivered frames, but prefer 1280x800 or 640x400".format(
                          w, h, native_w, native_h, w, h))
            else:
                _warn("requested size {}x{} is not one of the verified sizes {}".format(w, h, SUPPORTED_SIZES))
        if rate > MAX_IMU_RATE_HZ:
            _warn("imu_rate_hz {} is above {} Hz: Luxonis warns of gyroscope jitter on RVC2/BNO08X".format(
                rate, MAX_IMU_RATE_HZ))

        # Pipeline objects (None until build()).
        self._pipeline: Any = None
        self._nodes: Dict[str, Any] = {}
        self._stereo_queue: Any = None
        self._imu_queue: Any = None
        self._imu_tail: List[Any] = []          # IMU messages drained by stop(), returned once by imu_samples()
        self._handler: Any = None
        self._offset_enum: Any = None

        # Run state and statistics.
        self._started_at: Optional[float] = None
        self._last_group_at: Optional[float] = None
        self._stopped = False
        self._stop_reason: Optional[str] = None
        self._fatal: Optional[str] = None
        self._frame_stats = StreamStats(self.fps, FRAME_GAP_FACTOR, strict=True)
        self._imu_stats = StreamStats(float(rate), IMU_GAP_FACTOR, strict=False)
        self._counters: Dict[str, int] = {
            "stereo_sequence_gaps": 0, "stereo_sync_violations": 0, "stereo_seq_mismatch": 0,
            "imu_messages": 0, "imu_incomplete": 0, "imu_late_dropped": 0,
            "warmup_frames_discarded": 0, "warmup_imu_discarded": 0,
        }
        self._max_lr_skew_seen_us = 0.0
        self._max_accel_gyro_skew_ns = 0
        self._last_seq: Optional[int] = None
        self._seq_mismatch_warned = False
        self._intrinsics_check: Dict[str, Dict[str, Any]] = {}
        self._first_frame_checked = False
        self.epoch_offset_ns: Optional[int] = None

    # -- description ----------------------------------------------------------------------

    @property
    def size(self) -> Tuple[int, int]:
        """``(width, height)`` requested from both cameras = cuVSLAM ``Camera.size``."""
        return (self.width, self.height)

    @property
    def shape(self) -> Tuple[int, int]:
        """``(height, width)`` of every delivered image."""
        return (self.height, self.width)

    @property
    def period_ns(self) -> int:
        return self._frame_stats.period_ns

    @property
    def handler(self) -> Any:
        """``dai.CalibrationHandler`` of the device: cached by :meth:`build` (read before the
        pipeline starts); read on first access when :meth:`build` has not run yet."""
        if self._handler is None:
            self._handler = calibration_handler(self.device)
        return self._handler

    def calibration(self) -> Dict[str, Any]:
        """:func:`dump_calibration` of the device (``eepromToJson()`` dict; works after :meth:`stop`
        once :meth:`build` cached the handler)."""
        return dump_calibration(self.handler)

    def plan(self) -> Dict[str, Any]:
        """What :meth:`build` will create, as a JSON-serialisable dict (no device access)."""
        image_type = "GRAY8" if self.gray else "sensor default (RAW8 on the mono sockets)"
        cameras = []
        for socket, role in _SOCKET_ROLES:
            cameras.append({
                "socket": socket, "role": role, "node": "Camera", "sensor_fps": self.fps,
                "output": {"size": [self.width, self.height], "type": image_type, "resize_mode": "CROP",
                           "fps": self.fps, "undistortion": False},
                "sync_input": role,
            })
        return {
            "cameras": cameras,
            "sync": {"node": "Sync", "inputs": [role for _, role in _SOCKET_ROLES],
                     "threshold_s": self.sync_threshold_s, "threshold_ms": self.sync_threshold_s * 1e3,
                     "timestamp_source": "DEFAULT (DEVICE when the node runs on the device)",
                     "run_on_host": False},
            "imu": {"node": "IMU", "sensors": list(IMU_SENSORS), "rate_hz": self.imu_rate_hz,
                    "batch_report_threshold": self.imu_batch, "max_batch_reports": MAX_BATCH_REPORTS,
                    "units": {"accelerometer": "m/s^2", "gyroscope": "rad/s"}, "frame": "sensor-native (RAW)"},
            "queues": {"stereo": {"max_size": self.stereo_queue_size, "blocking": False},
                       "imu": {"max_size": self.imu_queue_size, "blocking": False}},
            "timestamps": {
                "domain": "device monotonic clock (getTimestampDevice / tsDevice)",
                "frame": "left.getTimestampDevice(CameraExposureOffset.{})".format(self.exposure_offset.upper()),
                "imu": "packet.acceleroMeter.tsDevice (sec * 1e9 + nsec)",
                "pair_rule": "left stamp for both images; |right - left| <= {} us else dropped".format(
                    "inf" if self.max_lr_skew_us is None else self.max_lr_skew_us),
                "frame_gap_factor": FRAME_GAP_FACTOR, "imu_gap_factor": IMU_GAP_FACTOR,
            },
            "guards": {
                "first_frame": "ImgTransformation intrinsics/size/distortion/source size vs CalibrationHandler "
                               "(a same-aspect integer-binned source with matching intrinsics is noted, not fatal)",
                "strict_intrinsics": self.strict_intrinsics, "intrinsics_tol_px": INTRINSICS_TOL_PX,
                "distortion_tol_relative": DISTORTION_TOL, "max_silence_s": self.max_silence_s,
            },
        }

    # -- pipeline -------------------------------------------------------------------------

    def _exposure_offset_enum(self) -> Any:
        if self._offset_enum is None:
            self._offset_enum = getattr(_require_dai().CameraExposureOffset, self.exposure_offset.upper())
        return self._offset_enum

    def build(self, pipeline: Any = None) -> Any:
        """Create the pipeline, nodes and output queues; returns the pipeline (idempotent).

        *pipeline* defaults to ``dai.Pipeline(device)``; an injected object with the same
        ``create()`` interface lets tests inspect the graph without a camera.  The
        ``CalibrationHandler`` is read here (before ``start()``) and cached for the first-frame
        guard and :meth:`calibration`; when the device has none, a warning is issued and the
        guard reports ``'unavailable'``.  Raises :class:`OakSourceError` when the device is closed
        or when creating a node/queue fails part-way (an XLink/RPC error): the device is closed
        first and the source is marked stopped, so nothing stays booted behind the exception.
        """
        if self._pipeline is not None:
            return self._pipeline
        if self._stopped:
            raise OakSourceError("this source was stopped; create a new OakStereoImuSource with a fresh device")
        d = _require_dai()
        if pipeline is None:
            is_closed = getattr(self.device, "isClosed", None)
            if callable(is_closed) and _safe(is_closed, False):
                raise OakSourceError("the dai.Device is closed (pipeline.stop() closes it): open a fresh one")
            pipeline = d.Pipeline(self.device)

        if self._handler is None:
            try:
                self._handler = calibration_handler(self.device)
            except Exception as exc:  # noqa: BLE001 - a device without EEPROM calibration
                _warn("calibration could not be read from the device ({}); the first-frame intrinsics "
                      "check will be unavailable".format(exc))

        try:
            image_type = d.ImgFrame.Type.GRAY8 if self.gray else None
            sync = pipeline.create(d.node.Sync)
            sync.setSyncThreshold(datetime.timedelta(seconds=self.sync_threshold_s))
            self._nodes["sync"] = sync
            for socket, role in _SOCKET_ROLES:
                cam = pipeline.create(d.node.Camera).build(getattr(d.CameraBoardSocket, socket), sensorFps=self.fps)
                output = cam.requestOutput((self.width, self.height), type=image_type,
                                           resizeMode=d.ImgResizeMode.CROP, fps=self.fps)
                output.link(sync.inputs[role])
                self._nodes["camera_" + role] = cam
                self._nodes["output_" + role] = output
            self._stereo_queue = sync.out.createOutputQueue(maxSize=self.stereo_queue_size, blocking=False)

            imu = pipeline.create(d.node.IMU)
            imu.enableIMUSensor([getattr(d.IMUSensor, name) for name in IMU_SENSORS], self.imu_rate_hz)
            imu.setBatchReportThreshold(self.imu_batch)
            imu.setMaxBatchReports(MAX_BATCH_REPORTS)
            self._nodes["imu"] = imu
            self._imu_queue = imu.out.createOutputQueue(maxSize=self.imu_queue_size, blocking=False)
        except Exception as exc:  # noqa: BLE001 - release the device, then report
            self._nodes.clear()
            self._stereo_queue = None
            self._imu_queue = None
            self._stopped = True
            self._stop_reason = "build failed: {}: {}".format(type(exc).__name__, exc)
            close = getattr(self.device, "close", None)
            if callable(close):
                _safe(close)                     # the pipeline never started: close the device directly
            raise OakSourceError("pipeline build failed ({}: {}); the device was closed, open a fresh one".format(
                type(exc).__name__, exc)) from exc

        self._pipeline = pipeline
        return pipeline

    @property
    def pipeline(self) -> Any:
        return self._pipeline

    @property
    def nodes(self) -> Dict[str, Any]:
        """Created nodes/outputs by role (``camera_left``, ``output_left``, ..., ``sync``, ``imu``)."""
        return dict(self._nodes)

    def start(self) -> "OakStereoImuSource":
        """Build if needed and start streaming; returns ``self``."""
        if self._stopped:
            raise OakSourceError("this source was stopped; create a new OakStereoImuSource with a fresh device")
        pipeline = self.build()
        if self._started_at is None:
            pipeline.start()
            self._started_at = time.monotonic()
            self._last_group_at = self._started_at
        return self

    @property
    def is_running(self) -> bool:
        if self._pipeline is None or self._started_at is None or self._stopped:
            return False
        return bool(_safe(self._pipeline.isRunning, False))

    def stop(self) -> None:
        """Stop the pipeline and close the device.  Idempotent, never raises.

        The IMU queue is drained once *before* ``pipeline.stop()`` closes it (non-blocking: only
        messages the device already delivered); :meth:`imu_samples` returns them one last time
        after the stop and :meth:`events` flushes them.  ``pipeline.stop()`` closes the device by
        itself (cheat-sheet 1.2); after a :meth:`build` that never started, ``device.close()`` is
        called instead so the camera is released either way (``close()`` on an already-closed
        device is harmless).
        """
        if self._stopped:
            return
        self._stopped = True
        if self._stop_reason is None:
            self._stop_reason = "stop"
        if self._pipeline is None:
            return
        if self._started_at is not None:
            self._imu_tail = self._try_get_all(self._imu_queue)
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001 - a dead pipeline is already stopped
                pass
        close = getattr(self.device, "close", None)
        if callable(close):
            _safe(close)

    def __enter__(self) -> "OakStereoImuSource":
        return self.start()

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.stop()

    # -- warm-up --------------------------------------------------------------------------

    def warm_up(self, seconds: float = 1.5) -> int:
        """Discard everything the cameras and the IMU deliver for *seconds* (auto-exposure settles in ~1 s).

        Both queues are drained every 50 ms while waiting, so the returned count (also
        ``warmup_frames_discarded``; IMU packets as ``warmup_imu_discarded``) is the number of
        stereo groups actually produced during the warm-up (about ``fps * seconds``), not the
        ``stereo_queue_size`` tail that would survive a plain sleep, and no queue overflows
        meanwhile.  The first discarded message still seeds :attr:`epoch_offset_ns`.  Gap
        statistics start after it.
        """
        if self._started_at is None:
            raise OakSourceError("warm_up() needs a started source")
        self._raise_if_stopped()
        deadline = time.monotonic() + max(0.0, float(seconds))
        groups = 0
        packets = 0
        while True:
            for group in self._try_get_all(self._stereo_queue):
                groups += 1
                self._last_group_at = time.monotonic()
                if self.epoch_offset_ns is None:
                    _safe(lambda: self._note_epoch(group["left"]))
            for msg in self._try_get_all(self._imu_queue):
                reports = _safe(lambda: list(msg.packets), []) or []
                packets += len(reports)
                if reports and self.epoch_offset_ns is None:
                    _safe(lambda: self._note_epoch(reports[0].acceleroMeter))
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or self._stopped:
                break
            time.sleep(min(0.05, remaining))
        self._counters["warmup_frames_discarded"] += groups
        self._counters["warmup_imu_discarded"] += packets
        return groups

    # -- frames ---------------------------------------------------------------------------

    def next_frame(self, timeout_s: float = 1.0) -> Optional[StereoFrame]:
        """The next accepted stereo pair, or ``None`` when *timeout_s* passes or a pair is dropped.

        Raises :class:`OakSourceStopped` once the stream has ended (consumer :meth:`stop`,
        queue closed after a node exception, ``isRunning()`` false, or any other exception from
        ``MessageQueue.get()`` such as an XLink communication error: the source stops itself and
        ``stats()['stop_reason']`` is ``'stereo queue error: <type>: <message>'``) and
        :class:`OakStreamError` on a fatal fault (first-frame guard, wrong image size/dtype,
        watchdog); a fatal fault stops the source and is raised again by every later call.
        """
        if self._started_at is None:
            raise OakSourceError("next_frame() needs a started source")
        self._raise_if_stopped()
        d = _require_dai()
        try:
            group = self._stereo_queue.get(datetime.timedelta(seconds=max(0.0, float(timeout_s))))
        except d.MessageQueue.QueueException as exc:
            self._raise_if_stopped()                 # stop() closed the queue under us: keep 'stop'
            self._end("queue closed: {}".format(exc))
            raise OakSourceStopped("stereo queue closed ({}); the pipeline stopped (a node threw an exception, "
                                   "see the [ThreadedNode] error above)".format(exc)) from None
        except Exception as exc:  # noqa: BLE001 - XLink / device communication faults surface here
            self._raise_if_stopped()
            reason = "stereo queue error: {}: {}".format(type(exc).__name__, exc)
            self._end(reason)
            raise OakSourceStopped("{} (the device connection failed or the pipeline died)".format(reason)) from exc
        self._raise_if_stopped()                     # stop() during the wait: the frame is not delivered
        if group is None:
            if not _safe(self._pipeline.isRunning, False):
                self._end("pipeline not running")
                raise OakSourceStopped("the pipeline is no longer running")
            if self.max_silence_s is not None:
                silent = time.monotonic() - (self._last_group_at or self._started_at)
                if silent > self.max_silence_s:
                    self._fail("no stereo pair for {:.1f} s although the pipeline reports running: the Sync "
                               "node never emitted (check the camera links, fps and exposure); watchdog "
                               "max_silence_s={}".format(silent, self.max_silence_s))
                    raise OakStreamError(self._fatal or "watchdog")
            return None
        self._last_group_at = time.monotonic()
        try:
            return self._accept_group(group)
        except OakStreamError as exc:
            self._fail(str(exc))
            raise

    def frames(self, timeout_s: float = 1.0) -> Iterator[StereoFrame]:
        """Generator of accepted :class:`StereoFrame` objects until :meth:`stop` or the pipeline dies.

        Blocks up to *timeout_s* per poll (checking ``isRunning()`` and the watchdog in
        between); dropped pairs are skipped silently (counted in :meth:`stats`).  Ends cleanly
        (:class:`OakSourceStopped` swallowed) when the consumer stops the source or the pipeline
        stops by itself; the reason is in ``stats()['stop_reason']``.  A fatal
        :class:`OakStreamError` (first-frame guard, wrong frames, watchdog) propagates after the
        source stopped itself, and iterating again raises it again.
        """
        while True:
            try:
                frame = self.next_frame(timeout_s)
            except OakSourceStopped:
                return
            if frame is not None:
                yield frame

    def _accept_group(self, group: Any) -> Optional[StereoFrame]:
        """Apply the pairing/ordering policy to one Sync group; ``None`` = dropped (counted).

        Raises :class:`OakStreamError` (from the first-frame guard or the image checks); the
        guard stays armed until it completes, so it cannot be skipped by a retry.
        """
        left = group["left"]
        right = group["right"]
        if self.epoch_offset_ns is None:
            _safe(lambda: self._note_epoch(left))
        if not self._first_frame_checked:
            self._check_first_frame(left, right)
            self._first_frame_checked = True
        frame = stereo_frame_from_messages(left, right, exposure_offset=self._exposure_offset_enum(),
                                           expected_shape=self.shape)
        skew = abs(frame.lr_ts_diff_us)
        if skew > self._max_lr_skew_seen_us:
            self._max_lr_skew_seen_us = skew
        if self.max_lr_skew_us is not None and skew > self.max_lr_skew_us:
            self._counters["stereo_sync_violations"] += 1
            self._frame_stats.skip(frame.ts_ns)      # arrived: advances the clock, not a timestamp gap after it
            self._note_seq(frame.seq)                # arrived: not a sequence gap either
            return None
        if frame.right_seq >= 0 and frame.right_seq != frame.seq:
            self._counters["stereo_seq_mismatch"] += 1
            if not self._seq_mismatch_warned:
                self._seq_mismatch_warned = True
                _warn("Sync paired left seq {} with right seq {} (timestamps {:.0f} us apart); further "
                      "mismatches are only counted".format(frame.seq, frame.right_seq, frame.lr_ts_diff_us))
        status = self._frame_stats.note(frame.ts_ns)
        self._note_seq(frame.seq)                    # every pair that reached the host, accepted or not
        if status == "backwards":
            return None
        return frame

    def _note_seq(self, seq: int) -> None:
        """Count left sequence numbers that never reached the host.  Every pair that arrived is
        noted (accepted, skew-dropped or rejected as backwards); a repeated or lower number (a
        stale pair) does not move the counter back."""
        if self._last_seq is None:
            self._last_seq = seq
        elif seq > self._last_seq:
            self._counters["stereo_sequence_gaps"] += seq - self._last_seq - 1
            self._last_seq = seq

    def _check_first_frame(self, left: Any, right: Any) -> None:
        """First-frame guard (``methods/cuvslam-vio/docs/oak-to-cuvslam-calibration.md`` 1.4) on both cameras.

        Compares, from the delivered ``ImgTransformation``: ``getIntrinsicMatrix()`` with
        ``getCameraIntrinsics(socket, w, h)`` (:data:`INTRINSICS_TOL_PX`), ``getSize()`` with the
        requested size, when the frame carries them ``getDistortionCoefficients()`` with the
        EEPROM's (:data:`DISTORTION_TOL` relative: zeros mean an undistorted stream), and
        ``getSourceSize()`` with the native sensor size (``getDefaultIntrinsics(socket)``,
        1280x800 here).  A source size that is an exact same-aspect integer binning of the native
        one (640x400 = the OV9282's own 2x2-binned mode) while intrinsics and distortion match is
        recorded as ``source_size_note`` and warned about, never fatal (calibration doc 1.3: the
        0.25 px half-pixel offset is below what cuVSLAM resolves); any other mismatch is
        :class:`OakStreamError` when ``strict_intrinsics`` else a warning.  The outcome is kept in
        ``stats()['intrinsics_check']`` (with ``source_intrinsics`` when the frame exposes
        ``getSourceIntrinsicMatrix()``).  An unavailable/invalid transformation (the accessor is
        untested on ``Camera`` outputs of this unit) or a missing calibration is reported as
        ``'unavailable'`` and warned about, never fatal.  Only the handler cached by :meth:`build`
        is used: no RPC is issued while streaming.  A distortion model name that differs is
        recorded and warned about only.
        """
        d = _require_dai()
        handler = self._handler                      # cached by build(); None = not readable there
        for frame, (socket, role) in ((left, _SOCKET_ROLES[0]), (right, _SOCKET_ROLES[1])):
            result: Dict[str, Any] = {"socket": socket, "status": "unavailable"}
            self._intrinsics_check[role] = result
            sock = getattr(d.CameraBoardSocket, socket)
            if handler is None:
                result["error"] = "calibration not available (could not be read in build())"
                _warn("{}: intrinsics check unavailable ({})".format(socket, result["error"]))
                continue
            try:
                expected = np.asarray(handler.getCameraIntrinsics(sock, self.width, self.height), dtype=np.float64)
            except Exception as exc:  # noqa: BLE001 - no calibration on the device
                result["error"] = "getCameraIntrinsics failed: {}".format(exc)
                _warn("{}: intrinsics check unavailable ({})".format(socket, result["error"]))
                continue
            native = _safe(lambda: tuple(int(v) for v in handler.getDefaultIntrinsics(sock)[1:3]), None) or DEFAULT_SIZE
            expected_dist = _safe(lambda: [float(v) for v in handler.getDistortionCoefficients(sock)], None)
            expected_model = _safe(lambda: _enum_name(handler.getDistortionModel(sock)), None)
            result["expected"] = expected.tolist()
            result["expected_source_size"] = [int(v) for v in native]
            result["expected_distortion"] = expected_dist
            result["expected_model"] = expected_model
            try:
                tr = frame.getTransformation()
                if hasattr(tr, "isValid") and not bool(tr.isValid()):
                    raise ValueError("ImgTransformation.isValid() is False")
                delivered = np.asarray(tr.getIntrinsicMatrix(), dtype=np.float64)
                result["delivered"] = delivered.tolist()
                result["source_size"] = [int(v) for v in tr.getSourceSize()]
                result["size"] = [int(v) for v in tr.getSize()]
            except Exception as exc:  # noqa: BLE001 - accessor missing/invalid on this frame
                result["error"] = str(exc)
                _warn("{}: delivered intrinsics unavailable ({}); trusting getCameraIntrinsics".format(socket, exc))
                continue
            delivered_dist = _safe(lambda: [float(v) for v in tr.getDistortionCoefficients()], None)
            delivered_model = _safe(lambda: _enum_name(tr.getDistortionModel()), None)
            result["distortion"] = delivered_dist
            result["model"] = delivered_model
            result["source_intrinsics"] = _safe(
                lambda: np.asarray(tr.getSourceIntrinsicMatrix(), dtype=np.float64).tolist(), None)

            problems: List[str] = []
            diff = float(np.abs(delivered - expected).max()) if delivered.shape == expected.shape else float("inf")
            result["max_abs_diff_px"] = diff
            if diff > INTRINSICS_TOL_PX:
                problems.append("delivered intrinsics {} differ from getCameraIntrinsics({}x{}) {} by {:.3g} px".format(
                    np.round(delivered, 3).tolist(), self.width, self.height, np.round(expected, 3).tolist(), diff))
            if tuple(result["size"]) != (self.width, self.height):
                problems.append("transformation size {}x{} is not the requested {}x{}".format(
                    result["size"][0], result["size"][1], self.width, self.height))
            if delivered_dist and expected_dist:
                n = max(len(delivered_dist), len(expected_dist))
                got = np.zeros(n, dtype=np.float64)
                got[:len(delivered_dist)] = delivered_dist
                want = np.zeros(n, dtype=np.float64)
                want[:len(expected_dist)] = expected_dist
                ddiff = float(np.abs(got - want).max())
                tol = DISTORTION_TOL * max(1.0, float(np.abs(want).max()))
                result["max_abs_distortion_diff"] = ddiff
                result["distortion_status"] = "ok" if ddiff <= tol else "mismatch"
                if ddiff > tol:
                    problems.append("delivered distortion coefficients {} differ from the EEPROM's {} (max diff "
                                    "{:.3g}): the stream looks undistorted or re-projected".format(
                                        np.round(got[:8], 4).tolist(), np.round(want[:8], 4).tolist(), ddiff))
            else:
                result["distortion_status"] = "unavailable"
            source = (int(result["source_size"][0]), int(result["source_size"][1]))
            if source != tuple(native):
                binning = _integer_scale(source, native)
                described = diff <= INTRINSICS_TOL_PX and result["distortion_status"] != "mismatch"
                if binning is not None and described:
                    result["source_size_note"] = (
                        "source {}x{} is the {}x{}-binned sensor mode, not the native {}x{}; the delivered "
                        "intrinsics match getCameraIntrinsics({}x{}) so the rig still describes the stream "
                        "(the {:.2f} px half-pixel offset of binning is below what cuVSLAM resolves, "
                        "calibration doc 1.3)".format(source[0], source[1], binning, binning, native[0], native[1],
                                                       self.width, self.height, (binning - 1) / (2.0 * binning)))
                    _warn("{}: {}".format(socket, result["source_size_note"]))
                else:
                    problems.append("source size {}x{} is not the native {}x{} sensor mode the calibration "
                                    "describes (cropped or differently binned sensor configuration)".format(
                                        source[0], source[1], native[0], native[1]))
            if delivered_model and expected_model and delivered_model != expected_model:
                result["model_note"] = "delivered {} vs EEPROM {}".format(delivered_model, expected_model)
                _warn("{}: delivered distortion model {} differs from the EEPROM's {} (recorded, not fatal)".format(
                    socket, delivered_model, expected_model))
            if problems:
                result["status"] = "mismatch"
                message = "{}: {}; the stream is not the calibrated {}x{} CROP output the rig describes".format(
                    socket, "; ".join(problems), self.width, self.height)
                if self.strict_intrinsics:
                    raise OakStreamError(message)
                _warn(message)
                continue
            result["status"] = "ok"

    # -- IMU ------------------------------------------------------------------------------

    def imu_samples(self) -> List[ImuSample]:
        """Drain every pending ``IMUData`` message and return its packets as ordered :class:`ImuSample` list.

        Non-decreasing ``ts_ns`` is guaranteed across calls (samples that go backwards are
        dropped and counted as ``imu_out_of_order``; packets missing a report as
        ``imu_incomplete``).  Returns ``[]`` before :meth:`start`.  After :meth:`stop` the
        messages that :meth:`stop` drained from the queue just before the pipeline stopped are
        returned once (then ``[]``), so a consumer that stops inside its loop still gets the
        samples between the last frame and the stop.
        """
        if self._imu_queue is None or self._started_at is None:
            return []
        if self._stopped:
            messages, self._imu_tail = self._imu_tail, []
        else:
            messages = self._try_get_all(self._imu_queue)
        samples: List[ImuSample] = []
        for msg in messages:
            self._counters["imu_messages"] += 1
            packets = _safe(lambda: list(msg.packets), None)
            if packets is None:
                self._counters["imu_incomplete"] += 1
                continue
            for packet in packets:
                sample = imu_sample_from_packet(packet)
                if sample is None:
                    self._counters["imu_incomplete"] += 1
                    continue
                if self.epoch_offset_ns is None:
                    _safe(lambda: self._note_epoch(packet.acceleroMeter))
                if self._imu_stats.note(sample.ts_ns) == "backwards":
                    continue
                skew = abs(sample.accel_gyro_skew_ns)
                if skew > self._max_accel_gyro_skew_ns:
                    self._max_accel_gyro_skew_ns = skew
                samples.append(sample)
        return samples

    def events(self, timeout_s: float = 1.0, *, drop_late_imu: bool = True) -> Iterator[Union[ImuSample, StereoFrame]]:
        """Yield :class:`ImuSample` and :class:`StereoFrame` objects in non-decreasing timestamp order.

        Before each frame, every pending IMU sample with ``ts_ns <= frame.ts_ns`` is yielded
        (the RealSense example's ``register_imu_until`` pattern); newer samples wait for the next
        frame.  With *drop_late_imu* (tracking) a sample older than the previous frame's stamp
        is dropped and counted (``imu_late_dropped``) because cuVSLAM rejects it; a recorder
        passes ``False`` to keep every sample.  At the end (:meth:`stop` was called, typically
        from inside the loop, or the pipeline died) the samples still pending plus those
        :meth:`stop` drained from the queue before the pipeline stopped are flushed in order, so
        the IMU stream ends after the last frame; samples the device had not yet delivered at
        that moment are not waited for.  Ends like :meth:`frames`: cleanly on :meth:`stop` /
        pipeline death, with :class:`OakStreamError` on a fatal fault (no flush then).
        """
        pending: Deque[ImuSample] = collections.deque()
        last_frame_ts: Optional[int] = None
        for frame in self.frames(timeout_s):
            pending.extend(self.imu_samples())
            while pending and pending[0].ts_ns <= frame.ts_ns:
                sample = pending.popleft()
                if drop_late_imu and last_frame_ts is not None and sample.ts_ns < last_frame_ts:
                    self._counters["imu_late_dropped"] += 1
                    continue
                yield sample
            yield frame
            last_frame_ts = frame.ts_ns
        pending.extend(self.imu_samples())
        for sample in pending:
            if drop_late_imu and last_frame_ts is not None and sample.ts_ns < last_frame_ts:
                self._counters["imu_late_dropped"] += 1
                continue
            yield sample

    # -- statistics -----------------------------------------------------------------------

    def stats(self) -> Dict[str, Any]:
        """Counters and measured rates (JSON-serialisable).

        ``frames`` = pairs delivered to the consumer; ``frame_gaps`` / ``frames_dropped_estimate``
        / ``stereo_sequence_gaps`` = pairs that never reached the host (a pair dropped here for
        L/R skew, ``stereo_sync_violations``, did arrive and is not a gap; a pair rejected for
        going backwards in time, ``frames_out_of_order``, does not advance the timestamp clock
        but its sequence number is still noted, so it is not a sequence gap either).  Keys
        match ``dataset.DROP_COUNTERS`` where the concept is the same (``stereo_sequence_gaps``,
        ``stereo_sync_violations``, ``imu_gaps``, ``imu_out_of_order``).  ``stop_reason`` is
        ``'stop'`` after a consumer stop, the queue/pipeline state after a pipeline death, or
        the full fault message (also in ``fatal_error``) after an :class:`OakStreamError`.
        """
        frames = self._frame_stats
        imu = self._imu_stats
        elapsed = 0.0 if self._started_at is None else time.monotonic() - self._started_at
        return {
            "frames": frames.count,
            "frame_gaps": frames.gaps,
            "frames_dropped_estimate": frames.dropped_estimate,
            "frames_out_of_order": frames.out_of_order,
            "max_frame_gap_ms": frames.max_gap_ns / 1e6,
            "frame_rate_hz": round(frames.measured_rate_hz(), 3),
            "stereo_sequence_gaps": self._counters["stereo_sequence_gaps"],
            "stereo_sync_violations": self._counters["stereo_sync_violations"],
            "stereo_seq_mismatch": self._counters["stereo_seq_mismatch"],
            "max_lr_skew_us": self._max_lr_skew_seen_us,
            "imu_samples": imu.count,
            "imu_messages": self._counters["imu_messages"],
            "imu_gaps": imu.gaps,
            "imu_dropped_estimate": imu.dropped_estimate,
            "imu_out_of_order": imu.out_of_order,
            "imu_incomplete": self._counters["imu_incomplete"],
            "imu_late_dropped": self._counters["imu_late_dropped"],
            "max_imu_gap_ms": imu.max_gap_ns / 1e6,
            "imu_max_accel_gyro_skew_ms": self._max_accel_gyro_skew_ns / 1e6,
            "imu_rate_hz": round(imu.measured_rate_hz(), 3),
            "warmup_frames_discarded": self._counters["warmup_frames_discarded"],
            "warmup_imu_discarded": self._counters["warmup_imu_discarded"],
            "first_frame_ts_ns": frames.first_ts_ns,
            "last_frame_ts_ns": frames.last_ts_ns,
            "first_imu_ts_ns": imu.first_ts_ns,
            "last_imu_ts_ns": imu.last_ts_ns,
            "epoch_offset_ns": self.epoch_offset_ns,
            "intrinsics_check": {role: dict(v) for role, v in self._intrinsics_check.items()},
            "elapsed_s": round(elapsed, 3),
            "running": self.is_running,
            "stop_reason": self._stop_reason,
            "fatal_error": self._fatal,
            "config": {"size": [self.width, self.height], "fps": self.fps, "imu_rate_hz": self.imu_rate_hz,
                       "imu_batch": self.imu_batch, "gray": self.gray, "exposure_offset": self.exposure_offset,
                       "max_lr_skew_us": self.max_lr_skew_us, "strict_intrinsics": self.strict_intrinsics,
                       "max_silence_s": self.max_silence_s},
        }

    def epoch_ns(self, ts_ns: int) -> int:
        """Device timestamp -> Unix-epoch nanoseconds using :attr:`epoch_offset_ns` (0 until known)."""
        return int(ts_ns) + int(self.epoch_offset_ns or 0)

    # -- internals ------------------------------------------------------------------------

    def _raise_if_stopped(self) -> None:
        """Re-raise a recorded fatal fault, or :class:`OakSourceStopped` after :meth:`stop`."""
        if self._fatal is not None:
            raise OakStreamError("source stopped after a fatal error: {}".format(self._fatal))
        if self._stopped:
            raise OakSourceStopped("source stopped ({})".format(self._stop_reason or "stop"))

    def _end(self, reason: str) -> None:
        """The pipeline ended by itself: record *reason* (first one wins) and release the device."""
        if self._stop_reason is None:
            self._stop_reason = reason
        self.stop()

    def _fail(self, message: str) -> None:
        """Fatal fault: keep the full message (first one wins), stop the source, release the device."""
        if self._fatal is None:
            self._fatal = message
        if self._stop_reason is None:
            self._stop_reason = message
        self.stop()

    def _note_epoch(self, message: Any) -> None:
        """Seed ``epoch_offset_ns`` from one message carrying both host and device stamps."""
        d = _require_dai()
        host_ns = to_ns(message.getTimestamp())
        device_ns = to_ns(message.getTimestampDevice())
        now_wall = time.time_ns()
        now_clock = timedelta_ns(d.Clock.now())
        self.epoch_offset_ns = (now_wall - now_clock) + (host_ns - device_ns)

    @staticmethod
    def _try_get_all(queue: Any) -> List[Any]:
        if queue is None:
            return []
        try:
            return list(queue.tryGetAll())
        except Exception:  # noqa: BLE001 - a closed queue has nothing left
            return []


# --------------------------------------------------------------------------------------
# Device-free self-check and command line (``python -m cuvslam_oak.oak_source``)
# --------------------------------------------------------------------------------------

class _PlanOnlyDevice:
    """Placeholder ``device`` for :meth:`OakStereoImuSource.plan`: never opened, never queried."""


def _self_check(verbose: bool = True) -> List[Tuple[str, bool, str]]:
    """Run the device-free checks (pure helpers; with depthai installed also the pipeline graph on
    a stub pipeline, the first-frame guard, the IMU tail flush, build/queue failure paths) and
    return ``[(name, passed, detail), ...]``.  Nothing here touches a camera."""
    results: List[Tuple[str, bool, str]] = []

    def check(name: str, cond: Any, detail: Any = "") -> None:
        results.append((name, bool(cond), str(detail)))
        if verbose:
            print("{} {}{}".format("PASS" if cond else "FAIL", name, " :: {}".format(detail) if detail != "" else ""))

    def raises(exc_type: Any, fn: Any) -> Any:
        try:
            fn()
        except exc_type as exc:
            return exc
        except Exception as exc:  # noqa: BLE001 - report the wrong type
            return ("WRONG TYPE", exc)
        return None

    class Ts:
        def __init__(self, ns: int) -> None:
            self.sec, self.nsec = ns // NS_PER_S, ns % NS_PER_S

    class Report:
        def __init__(self, x: float, y: float, z: float, ns: int, seq: int = 1) -> None:
            self.x, self.y, self.z, self.tsDevice, self.sequence, self._ns = x, y, z, Ts(ns), seq, ns

        def getTimestamp(self) -> datetime.timedelta:
            return datetime.timedelta(microseconds=(self._ns + 2_000_000) // 1000)

        def getTimestampDevice(self) -> datetime.timedelta:
            return datetime.timedelta(microseconds=self._ns // 1000)

    class Packet:
        def __init__(self, ns: int, seq: int = 1) -> None:
            self.acceleroMeter, self.gyroscope = Report(0.1, 9.8, 0.2, ns, seq), Report(0.01, 0.02, 0.03, ns + 100, seq)

    class ImuMsg:
        def __init__(self, ns_list: List[int]) -> None:
            self.packets = [Packet(ns) for ns in ns_list]

    # -- pure helpers ---------------------------------------------------------------------
    check("timedelta_ns is exact integer arithmetic",
          timedelta_ns(datetime.timedelta(days=2, seconds=3, microseconds=4)) == (2 * 86_400 + 3) * NS_PER_S + 4_000)
    check("to_ns: Timestamp-like / int / np.int64", to_ns(Ts(5_000_000_007)) == 5_000_000_007 and to_ns(3) == 3
          and to_ns(np.int64(9)) == 9 and isinstance(to_ns(np.int64(9)), int))
    check("to_ns rejects bool/float/str", all(isinstance(raises(TypeError, lambda b=b: to_ns(b)), TypeError) for b in (True, 1.5, "x")))
    st = StreamStats(30.0, FRAME_GAP_FACTOR, strict=True)
    p = st.period_ns
    check("StreamStats strict: ok / equal=backwards / gap counted / backwards rejected",
          st.note(1000) == "ok" and st.note(1000) == "backwards" and st.note(1000 + 3 * p) == "gap" and st.gaps == 1
          and st.dropped_estimate == 2 and st.note(500) == "backwards" and st.out_of_order == 2 and st.count == 2, st.as_dict())
    st2 = StreamStats(30.0, FRAME_GAP_FACTOR, strict=True)
    st2.note(0)
    check("StreamStats.skip advances the clock without counting a delivery",
          st2.skip(3 * p) == "gap" and st2.note(4 * p) == "ok" and st2.count == 2 and st2.skipped == 1 and st2.gaps == 1)
    sample = imu_sample_from_packet(Packet(NS_PER_S, seq=7))
    check("imu_sample_from_packet: accel stamp, gyro stamp, gyro-first record, kwargs",
          sample is not None and sample.ts_ns == NS_PER_S and sample.gyro_ts_ns == NS_PER_S + 100 and sample.sequence == 7
          and sample.accel.tolist() == [0.1, 9.8, 0.2] and sample.gyro.tolist() == [0.01, 0.02, 0.03]
          and sample.measurement_kwargs()["angular_velocities"] == [0.01, 0.02, 0.03] and sample.kind == "imu")
    check("imu_sample_from_packet: a report with device stamp 0 -> None", imu_sample_from_packet(Packet(0)) is None)
    check("_integer_scale: 640x400 and 320x200 of 1280x800 are 2x / 4x binning, 1280x720 and native are not",
          _integer_scale((640, 400), DEFAULT_SIZE) == 2 and _integer_scale((320, 200), DEFAULT_SIZE) == 4
          and _integer_scale((1280, 720), DEFAULT_SIZE) is None and _integer_scale(DEFAULT_SIZE, DEFAULT_SIZE) is None)
    check("exception hierarchy", issubclass(OakStreamError, OakSourceError) and issubclass(OakSourceStopped, OakSourceError)
          and issubclass(OakSourceError, RuntimeError) and not issubclass(OakStreamError, OakSourceStopped))
    plan = OakStereoImuSource(_PlanOnlyDevice()).plan()
    check("plan() is JSON-serialisable and describes the cheat-sheet graph",
          json.loads(json.dumps(plan))["cameras"][0]["output"] == {"size": [1280, 800], "type": "GRAY8", "resize_mode": "CROP",
                                                                    "fps": 30.0, "undistortion": False}
          and plan["imu"]["sensors"] == list(IMU_SENSORS) and plan["sync"]["threshold_s"] == 0.5 / 30.0)
    if dai is None:
        check("depthai not installed: graph/guard/stream checks skipped", True)
        return results

    # -- stubs around the real depthai enums ------------------------------------------------
    d = dai
    W, H = 640, 400
    K_NATIVE = {"CAM_B": np.array([[799.1, 0.0, 649.5], [0.0, 798.9, 391.5], [0.0, 0.0, 1.0]]),
                "CAM_C": np.array([[797.5, 0.0, 652.9], [0.0, 797.5, 368.2], [0.0, 0.0, 1.0]])}
    DIST = [-4.0, 12.0, 0.001, -0.002, -3.0, -4.1, 12.5, -2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def k_at(socket: str, w: int, h: int) -> np.ndarray:
        k = K_NATIVE[socket].copy()
        s = w / DEFAULT_SIZE[0]
        k[0, :] *= s
        k[1, :] *= s
        k[1, 2] -= (DEFAULT_SIZE[1] * s - h) / 2.0     # CROP keeps the centre
        k[2, 2] = 1.0
        return k

    class Handler:
        def getCameraIntrinsics(self, sock: Any, w: int, h: int) -> List[List[float]]:
            return k_at(_enum_name(sock), w, h).tolist()

        def getDefaultIntrinsics(self, sock: Any) -> Tuple[Any, int, int]:
            return (K_NATIVE[_enum_name(sock)].tolist(), DEFAULT_SIZE[0], DEFAULT_SIZE[1])

        def getDistortionCoefficients(self, sock: Any) -> List[float]:
            return list(DIST)

        def getDistortionModel(self, sock: Any) -> str:
            return "Perspective"

        def eepromToJson(self) -> Dict[str, Any]:
            return {"cameraData": [], "version": 7}

    class Tr:
        def __init__(self, K: np.ndarray, dist: Optional[List[float]] = DIST, source: Tuple[int, int] = DEFAULT_SIZE,
                     size: Tuple[int, int] = (W, H)) -> None:
            self.K, self.dist, self.source, self.size = K, dist, source, size

        def isValid(self) -> bool:
            return True

        def getIntrinsicMatrix(self) -> List[List[float]]:
            return self.K.tolist()

        def getSourceIntrinsicMatrix(self) -> List[List[float]]:
            return self.K.tolist()

        def getSourceSize(self) -> Tuple[int, int]:
            return self.source

        def getSize(self) -> Tuple[int, int]:
            return self.size

        def getDistortionCoefficients(self) -> List[float]:
            if self.dist is None:
                raise RuntimeError("no distortion")
            return list(self.dist)

        def getDistortionModel(self) -> str:
            return "Perspective"

    class Img:
        def __init__(self, ns: int, seq: int, tr: Tr, shape: Tuple[int, int] = (H, W), exposure_us: int = 10_000) -> None:
            self.ns, self.seq, self.tr, self.shape, self.exposure_us = ns, seq, tr, shape, exposure_us

        def _off(self, off: Any) -> int:
            if off is None or off == d.CameraExposureOffset.END:
                return 0
            return self.exposure_us * (500 if off == d.CameraExposureOffset.MIDDLE else 1000)

        def getTimestampDevice(self, off: Any = None) -> datetime.timedelta:
            return datetime.timedelta(microseconds=(self.ns - self._off(off)) // 1000)

        def getTimestamp(self, off: Any = None) -> datetime.timedelta:
            return datetime.timedelta(microseconds=(self.ns + 30_000_000 - self._off(off)) // 1000)

        def getSequenceNum(self) -> int:
            return self.seq

        def getExposureTime(self) -> datetime.timedelta:
            return datetime.timedelta(microseconds=self.exposure_us)

        def getCvFrame(self) -> np.ndarray:
            return np.full(self.shape, 7, dtype=np.uint8)

        def getTransformation(self) -> Tr:
            return self.tr

    def group(ns: int, seq: int, lr_us: float = 12.0, tr_left: Optional[Tr] = None, tr_right: Optional[Tr] = None,
              **kw: Any) -> Dict[str, Img]:
        return {"left": Img(ns, seq, tr_left or Tr(k_at("CAM_B", W, H)), **kw),
                "right": Img(ns + int(lr_us * 1000), seq, tr_right or Tr(k_at("CAM_C", W, H)), **kw)}

    class Queue:
        def __init__(self, max_size: int, blocking: bool) -> None:
            self.max_size, self.blocking, self.items, self.closed = max_size, blocking, collections.deque(), False
            self.on_empty: Any = None
            self.raise_on_get: Any = None

        def get(self, timeout: datetime.timedelta) -> Any:
            if self.raise_on_get is not None:
                raise self.raise_on_get
            if self.closed:
                raise d.MessageQueue.QueueException("MessageQueue was closed")
            if self.items:
                return self.items.popleft()
            if self.on_empty is not None:
                self.on_empty()
            if self.closed:
                raise d.MessageQueue.QueueException("MessageQueue was closed")
            return None

        def tryGetAll(self) -> List[Any]:
            if self.closed:
                raise d.MessageQueue.QueueException("MessageQueue was closed")
            out, self.items = list(self.items), collections.deque()
            return out

    class Output:
        def __init__(self, owner: Any, args: Dict[str, Any]) -> None:
            self.owner, self.args, self.links, self.queue = owner, args, [], None

        def link(self, inp: Any) -> None:
            self.links.append(inp)

        def createOutputQueue(self, maxSize: int = 16, blocking: bool = False) -> Queue:
            self.queue = Queue(maxSize, blocking)
            return self.queue

    class Camera:
        def __init__(self) -> None:
            self.socket, self.build_kw, self.outputs = None, None, []

        def build(self, socket: Any, **kw: Any) -> "Camera":
            self.socket, self.build_kw = socket, kw
            return self

        def requestOutput(self, size: Any, type: Any = None, resizeMode: Any = None, fps: Any = None, **kw: Any) -> Output:
            out = Output(self, dict(size=size, type=type, resizeMode=resizeMode, fps=fps, **kw))
            self.outputs.append(out)
            return out

    class Sync:
        def __init__(self) -> None:
            self.threshold, self.inputs, self.out = None, {"left": "IN_left", "right": "IN_right"}, Output(self, {})

        def setSyncThreshold(self, td: datetime.timedelta) -> None:
            self.threshold = td

    class Imu:
        def __init__(self) -> None:
            self.sensors, self.rate, self.batch, self.max_batch, self.out = None, None, None, None, Output(self, {})

        def enableIMUSensor(self, sensors: Any, rate: int) -> None:
            self.sensors, self.rate = list(sensors), rate

        def setBatchReportThreshold(self, n: int) -> None:
            self.batch = n

        def setMaxBatchReports(self, n: int) -> None:
            self.max_batch = n

    class Pipeline:
        def __init__(self, fail_on: Optional[str] = None) -> None:
            self.nodes, self.running, self.stop_calls, self.fail_on = [], False, 0, fail_on

        def create(self, cls: Any) -> Any:
            name = cls.__name__
            if name == self.fail_on:
                raise RuntimeError("stub: cannot create {}".format(name))
            node = {"Camera": Camera, "Sync": Sync, "IMU": Imu}[name]()
            self.nodes.append(node)
            return node

        def start(self) -> None:
            self.running = True

        def stop(self) -> None:
            self.running, self.stop_calls = False, self.stop_calls + 1
            for node in self.nodes:
                for out in ([node.out] if hasattr(node, "out") else node.outputs):
                    if out.queue is not None:
                        out.queue.closed = True

        def isRunning(self) -> bool:
            return self.running

    class Device:
        def __init__(self, handler: Any = None) -> None:
            self.closed, self.close_calls, self.calib_reads, self._handler = False, 0, 0, handler

        def isClosed(self) -> bool:
            return self.closed

        def close(self) -> None:
            self.closed, self.close_calls = True, self.close_calls + 1

        def readCalibration(self) -> Any:
            self.calib_reads += 1
            if self._handler is None:
                raise RuntimeError("stub: no calibration")
            return self._handler

    def fresh(**kw: Any) -> Tuple[OakStereoImuSource, Pipeline, Queue, Queue, Device]:
        dev = Device(kw.pop("handler", Handler()))
        src = OakStereoImuSource(dev, W, H, 30.0, 200, **kw)
        pipe = Pipeline()
        src.build(pipe)
        src.start()
        sync = [n for n in pipe.nodes if isinstance(n, Sync)][0]
        imu = [n for n in pipe.nodes if isinstance(n, Imu)][0]
        return src, pipe, sync.out.queue, imu.out.queue, dev

    F = 33_333_000
    t0 = 2 * NS_PER_S

    # -- graph vs camera/docs/depthai-v3-cheatsheet.md --------------------------------------------
    src, pipe, sq, iq, dev = fresh()
    cams = [n for n in pipe.nodes if isinstance(n, Camera)]
    sync = [n for n in pipe.nodes if isinstance(n, Sync)][0]
    imu = [n for n in pipe.nodes if isinstance(n, Imu)][0]
    check("graph: Camera.build(CAM_B / CAM_C, sensorFps) then Sync and IMU, nothing else",
          len(pipe.nodes) == 4 and [c.socket for c in cams] == [d.CameraBoardSocket.CAM_B, d.CameraBoardSocket.CAM_C]
          and all(c.build_kw == {"sensorFps": 30.0} for c in cams))
    check("graph: requestOutput((w, h), GRAY8, CROP, fps), one output per camera, no undistortion",
          all(len(c.outputs) == 1 and c.outputs[0].args == {"size": (W, H), "type": d.ImgFrame.Type.GRAY8,
                                                            "resizeMode": d.ImgResizeMode.CROP, "fps": 30.0} for c in cams),
          cams[0].outputs[0].args)
    check("graph: outputs linked to Sync.inputs['left'/'right'], threshold 0.5/fps",
          cams[0].outputs[0].links == ["IN_left"] and cams[1].outputs[0].links == ["IN_right"]
          and sync.threshold == datetime.timedelta(seconds=0.5 / 30.0))
    check("graph: IMU ACCELEROMETER_RAW + GYROSCOPE_RAW @ 200 Hz, batch 1, max 10",
          imu.sensors == [d.IMUSensor.ACCELEROMETER_RAW, d.IMUSensor.GYROSCOPE_RAW] and imu.rate == 200
          and imu.batch == 1 and imu.max_batch == MAX_BATCH_REPORTS)
    check("graph: non-blocking queues, stereo 8, IMU 2 s (400)",
          sq.max_size == 8 and sq.blocking is False and iq.max_size == 400 and iq.blocking is False)
    check("calibration read once in build(), handler cached", dev.calib_reads == 1 and src.handler is not None)

    # -- streaming, ordering, IMU tail at stop() ------------------------------------------
    for i in range(3):
        sq.items.append(group(t0 + i * F, i))
    iq.items.append(ImuMsg([t0 - 10_000_000 + 5_000_000 * j for j in range(16)]))
    events: List[Any] = []
    for ev in src.events(0.01, drop_late_imu=False):
        events.append(ev)
        if ev.kind == "stereo" and ev.seq == 2:
            iq.items.append(ImuMsg([t0 + 2 * F + 5_000_000 * (j + 1) for j in range(3)]))   # after the last frame
            src.stop()
    ts = [e.ts_ns for e in events]
    tail = [e for e in events if e.kind == "imu" and e.ts_ns > t0 + 2 * F]
    stats = src.stats()
    check("events(): 3 frames + IMU in non-decreasing order, IMU <= frame yielded before it",
          sum(e.kind == "stereo" for e in events) == 3 and all(a <= b for a, b in zip(ts, ts[1:]))
          and events[0].kind == "imu", [e.kind for e in events][:8])
    check("events(): IMU samples still queued at stop() are drained and flushed after the last frame",
          len(tail) == 3 and events[-1].kind == "imu", len(tail))
    check("stats after stop: frames 3, stop_reason 'stop', device closed once, pipeline stopped once, calibration() ok",
          stats["frames"] == 3 and stats["stop_reason"] == "stop" and dev.close_calls == 1 and pipe.stop_calls == 1
          and dev.calib_reads == 1 and stats["running"] is False and src.calibration()["version"] == 7
          and src.imu_samples() == [] and json.dumps(stats) is not None,
          {k: stats[k] for k in ("frames", "imu_samples", "stop_reason", "frame_gaps", "imu_gaps")})
    last_frame = [e for e in events if e.kind == "stereo"][-1]
    check("mid-exposure stamp: ts_ns = END - exposure/2, both images carry the left stamp",
          last_frame.ts_ns == t0 + 2 * F - 5_000_000 and last_frame.right_ts_ns == last_frame.ts_ns + 12_000
          and abs(last_frame.lr_ts_diff_us - 12.0) < 1e-9, (last_frame.ts_ns - t0, last_frame.lr_ts_diff_us))
    check("after stop(): next_frame raises OakSourceStopped, frames() ends, start() refused",
          isinstance(raises(OakSourceStopped, lambda: src.next_frame(0.0)), OakSourceStopped) and list(src.frames(0.0)) == []
          and isinstance(raises(OakSourceError, src.start), OakSourceError))

    # -- first-frame guard ----------------------------------------------------------------
    src, pipe, sq, iq, dev = fresh(strict_intrinsics=True)
    sq.items.append(group(t0, 0, tr_left=Tr(k_at("CAM_B", W, H), source=(W, H)), tr_right=Tr(k_at("CAM_C", W, H), source=(W, H))))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        frame = src.next_frame(0.01)
    ic = src.stats()["intrinsics_check"]
    check("guard: 2x-binned source 640x400 with matching K and distortion -> accepted, 'ok' + source_size_note, one warning per camera",
          frame is not None and all(v["status"] == "ok" and "source_size_note" in v and v["distortion_status"] == "ok" for v in ic.values())
          and sum("binned" in str(w.message) for w in caught) == 2, {r: v.get("source_size_note", "")[:60] for r, v in ic.items()})
    src.stop()
    src, pipe, sq, iq, dev = fresh(strict_intrinsics=True)
    sq.items.append(group(t0, 0, tr_left=Tr(k_at("CAM_B", W, H), source=(1280, 720))))
    err = raises(OakStreamError, lambda: src.next_frame(0.01))
    check("guard: source 1280x720 (not a binning of 1280x800) with matching K -> OakStreamError (strict), source stopped",
          isinstance(err, OakStreamError) and "source size 1280x720" in str(err) and dev.closed and src.stats()["fatal_error"] == str(err), err)
    check("guard: fatal fault raised again by every later call", isinstance(raises(OakStreamError, lambda: list(src.frames(0.0))), OakStreamError))
    src, pipe, sq, iq, dev = fresh(strict_intrinsics=True)
    k_wrong = k_at("CAM_B", W, H)
    k_wrong[1, 2] -= 40.0
    sq.items.append(group(t0, 0, tr_left=Tr(k_wrong, source=(W, H))))
    err = raises(OakStreamError, lambda: src.next_frame(0.01))
    check("guard: binned source but K off by 40 px -> still fatal (strict)", isinstance(err, OakStreamError) and "40 px" in str(err), err)
    src, pipe, sq, iq, dev = fresh(strict_intrinsics=True)
    sq.items.append(group(t0, 0, tr_left=Tr(k_at("CAM_B", W, H), dist=[0.0] * 14)))
    err = raises(OakStreamError, lambda: src.next_frame(0.01))
    check("guard: zero distortion (undistorted stream) -> OakStreamError (strict)", isinstance(err, OakStreamError) and "undistorted" in str(err), err)
    src, pipe, sq, iq, dev = fresh(strict_intrinsics=False)
    sq.items.append(group(t0, 0, tr_left=Tr(k_wrong)))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        frame = src.next_frame(0.01)
    check("guard: non-strict mismatch -> warned once, delivered, status 'mismatch'",
          frame is not None and src.stats()["intrinsics_check"]["left"]["status"] == "mismatch"
          and sum("differ" in str(w.message) for w in caught) == 1)
    src.stop()
    with warnings.catch_warnings(record=True):
        warnings.simplefilter("ignore")              # build() warns that the calibration is unavailable
        src, pipe, sq, iq, dev = fresh(handler=None)
        sq.items.append(group(t0, 0))
        frame = src.next_frame(0.01)
    check("guard: no calibration -> 'unavailable', frame delivered, no readCalibration RPC while streaming",
          frame is not None and src.stats()["intrinsics_check"]["left"]["status"] == "unavailable" and dev.calib_reads == 1, dev.calib_reads)
    src.stop()

    # -- accounting, build/queue failure paths -------------------------------------------
    src, pipe, sq, iq, dev = fresh()
    sq.items.extend([group(t0, 0), group(t0 + F, 1, lr_us=1500.0), group(t0 + 2 * F, 2), group(t0 + 5 * F, 5), group(t0 + 4 * F, 6), group(t0 + 6 * F, 7)])
    sq.on_empty = src.stop
    got = [f.seq for f in src.frames(0.01)]
    stats = src.stats()
    check("accounting: skew drop, timestamp gap and a backwards pair: seq gaps count only pairs that never arrived",
          got == [0, 2, 5, 7] and stats["stereo_sync_violations"] == 1 and stats["frames_out_of_order"] == 1
          and stats["frame_gaps"] == 1 and stats["frames_dropped_estimate"] == 2 and stats["stereo_sequence_gaps"] == 2,
          {k: stats[k] for k in ("stereo_sync_violations", "frames_out_of_order", "frame_gaps", "frames_dropped_estimate", "stereo_sequence_gaps")})
    dev = Device(Handler())
    src = OakStereoImuSource(dev, W, H)
    err = raises(OakSourceError, lambda: src.build(Pipeline(fail_on="IMU")))
    check("build() failure part-way -> OakSourceError, device closed, source stopped, start() refused",
          isinstance(err, OakSourceError) and "IMU" in str(err) and dev.close_calls == 1
          and isinstance(raises(OakSourceError, src.start), OakSourceError) and src.stats()["stop_reason"].startswith("build failed"), err)
    src, pipe, sq, iq, dev = fresh()
    sq.raise_on_get = RuntimeError("Communication exception - possible device error/misconfiguration")
    got = list(src.frames(0.01))
    check("a non-QueueException error from get() -> frames() ends, 'stereo queue error' recorded, device closed",
          got == [] and src.stats()["stop_reason"].startswith("stereo queue error: RuntimeError") and dev.closed
          and src.stats()["fatal_error"] is None, src.stats()["stop_reason"])
    src, pipe, sq, iq, dev = fresh()
    sq.items.append(group(t0, 0))
    sq.on_empty = lambda: setattr(sq, "closed", True)
    got = list(src.frames(0.01))
    check("queue closed (pipeline death) -> clean end, reason recorded, device closed",
          len(got) == 1 and src.stats()["stop_reason"].startswith("queue closed") and dev.closed)
    return results


def _main(argv: Optional[List[str]] = None) -> int:
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        prog="python -m cuvslam_oak.oak_source",
        description="Print the pipeline plan of the live OAK stereo+IMU source and/or run its device-free "
                    "self-checks.  This never opens a camera (datasets/07_record_dataset.py does).")
    parser.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    parser.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--imu-rate", type=int, default=DEFAULT_IMU_RATE_HZ, help="IMU report rate in Hz")
    parser.add_argument("--imu-batch", type=int, default=1, help="IMU packets per message")
    parser.add_argument("--no-plan", action="store_true", help="do not print the plan JSON")
    parser.add_argument("--self-check", action="store_true",
                        help="run the device-free checks (pure helpers, stub pipeline graph, first-frame guard, "
                             "IMU tail flush, failure paths); exit 1 when one fails")
    args = parser.parse_args(argv)
    code = 0
    if not args.no_plan:
        try:
            src = OakStereoImuSource(_PlanOnlyDevice(), args.width, args.height, args.fps, args.imu_rate, args.imu_batch)
        except OakSourceError as exc:
            print("oak_source: {}".format(exc), file=sys.stderr)
            return 2
        print(json.dumps(src.plan(), indent=2))
    if args.self_check:
        results = _self_check(verbose=True)
        failed = [name for name, ok, _ in results if not ok]
        print("{} checks, {} failed{}".format(len(results), len(failed), (": " + ", ".join(failed)) if failed else ""))
        code = 1 if failed else 0
    return code


if __name__ == "__main__":  # pragma: no cover - command line only
    raise SystemExit(_main())
