#!/usr/bin/env python3
"""04_imu.py - stream the OAK-D S2 IMU (BNO086) with DepthAI v3 and sanity-check it.

What this test checks
---------------------
* The device reports an IMU: type (``Device.getConnectedIMU``) and firmware version
  (``Device.getIMUFirmwareVersion``, plus the embedded/upgradable version).
* ``dai.node.IMU`` streams ACCELEROMETER_RAW + GYROSCOPE_RAW at ``--rate`` Hz (default 200)
  and the measured packet rate matches: it is computed both from wall-clock arrival and
  from the device timestamps of consecutive accelerometer reports (spacing / gaps shown).
* With ``--rotation`` the fused ROTATION_VECTOR (unit quaternion i, j, k, real + an accuracy
  estimate in radians) is streamed too and shown as roll / pitch / yaw.  IMUs without sensor
  fusion (BMI family) are reported and the option is skipped instead of failing.
* Stationary statistics: during the first ``--calib-seconds`` (default 3) the camera must be
  held still.  The script then prints the gyroscope bias per axis (rad/s and deg/s), the
  accelerometer mean / standard deviation per axis and the mean norm of the acceleration
  vector, which should be close to g = 9.81 m/s^2 when only gravity acts on the sensor.
  The bias is subtracted from the gyro values printed afterwards.
* Live samples are printed ``--print-hz`` times per second (default 5) together with the
  measured sample rate and the host latency; ``--plot`` adds a scrolling matplotlib plot.

How to run (from the repository root)
-------------------------------------
    python camera/04_imu.py                          # 200 Hz accel + gyro, 3 s bias estimate
    python camera/04_imu.py --rotation               # + rotation vector (roll/pitch/yaw)
    python camera/04_imu.py --rate 400 --duration 10 --headless
    python camera/04_imu.py --plot --plot-window 5   # live plot (needs matplotlib)
    python camera/04_imu.py --device-id OAKDS2EXAMPLEID000 --calib-seconds 5 --print-hz 2

What you should see
-------------------
* ``IMU: BNO086 firmware 3.9.9 (embedded firmware 3.9.9)``.
* Calibration block after ~3 s: gyro bias of a few 1e-3 rad/s or less, gyro std ~1e-3 rad/s,
  accel std of a few 1e-2 m/s^2, accel norm 9.7-9.9 m/s^2 (this unit at rest, standing upright:
  accel ~(-0.06, -0.79, -9.94), gravity mostly along -z).  A WARNING is printed when the sensor
  did not look stationary (large std).
* Live lines: ``rate 200.0 Hz`` (199-201 expected at the default; ~400 with --rate 400), gyro
  near 0 while still and changing sign when you rotate the camera about each axis, accel norm
  staying ~9.8 while stationary.  With --rotation: roll/pitch/yaw in degrees following the
  camera pose and the accuracy estimate dropping from 3.14 rad to well below 0.1 rad after a
  few seconds.  Raw sensors report ``accuracy = UNRELIABLE`` by design; that is not an error.
* A final summary: packet count, both measured rates, timestamp spacing (mean/max), number of
  gaps larger than 2.5 sample periods (0 expected), and the gyro bias that was applied.  Both
  rates and the packet/message counts cover the same interval: from the first packet accepted
  after the 0.25 s settle window (whose packets are discarded and reported as such) to the last.

Keys / exit
-----------
Ctrl-C in the terminal stops the run cleanly and prints the summary (exit status 0).  In the
``--plot`` window press ``q`` or close the window.  ``--duration SECONDS`` stops automatically
(exit status 0).  ``--headless`` only suppresses the plot; the text output is the same.
Exit status: 0 normal / Ctrl-C while streaming, 1 pipeline error (device lost, queue closed),
2 no device or wrong ``--device-id`` (one line on stderr, no traceback), 3 device has no IMU,
130 Ctrl-C outside the streaming loop (while the device boots, or a second Ctrl-C during the
summary) - one line on stderr, no traceback.

Notes
-----
* All sensors enabled on one IMU node are emitted as one packet stream at the *slowest*
  enabled rate (verified on this unit: RAW @ 200 Hz + ROTATION_VECTOR @ 100 Hz gave 100
  packets/s), so ``--rotation-rate`` defaults to ``--rate``.
* Sequence numbers of IMU reports are 8-bit and step by 1 or 2 between consecutive packets on
  the BNO086, so drops are detected from timestamp gaps, not from sequence numbers.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import datetime
import math
import sys
import time
from pathlib import Path
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import (  # noqa: E402 - imported after the sys.path tweak above
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    open_device,
    print_device_summary,
)

EXIT_OK = 0
EXIT_PIPELINE_ERROR = 1
EXIT_NO_IMU = 3
EXIT_INTERRUPTED_EARLY = 130

#: Packets arriving in the first SETTLE_S after start are discarded before the stationary
#: window begins (lets the sensor's first reports and the host clock sync settle).
SETTLE_S = 0.25
#: Live sample lines start this long after the first counted packet, so that the printed rate
#: covers a real window (a single message would read 0.0 Hz, two a noisy value).
LIVE_WARMUP_S = 0.2
#: Rolling window of the live rate (seconds).
RATE_WINDOW_S = 2.0
#: Standard gravity used for the accelerometer-norm sanity check.
GRAVITY_MS2 = 9.80665
#: Thresholds above which the calibration window is reported as "not stationary".
MAX_STILL_GYRO_STD = 0.02  # rad/s
MAX_STILL_ACCEL_STD = 0.15  # m/s^2
#: A spacing between consecutive accelerometer timestamps above this many nominal periods
#: counts as a gap (dropped packets).
GAP_FACTOR = 2.5
#: Plot refresh rate (Hz); keeps the matplotlib window responsive without stalling the queue.
PLOT_REFRESH_HZ = 15.0

Vec3 = Tuple[float, float, float]


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Build the argument parser (shared --device-id/--duration/--headless + IMU options)."""
    parser = argparse.ArgumentParser(
        description=(
            "Stream the OAK IMU: raw accelerometer + gyroscope at --rate Hz, optionally the "
            "fused rotation vector. Estimates gyro bias / accel statistics while the camera is "
            "held still for the first --calib-seconds, then prints live samples with the "
            "measured sample rate. Ctrl-C prints a summary and exits cleanly."
        ),
    )
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument(
        "--rate", type=int, default=200, metavar="HZ",
        help="report rate for ACCELEROMETER_RAW + GYROSCOPE_RAW (default 200; the BNO086 "
             "supports up to 400 Hz gyro / 500 Hz accel, the firmware rounds to what it can do)",
    )
    parser.add_argument(
        "--rotation", action="store_true",
        help="also enable ROTATION_VECTOR (fused orientation quaternion + accuracy); skipped "
             "with a message on IMUs without sensor fusion",
    )
    parser.add_argument(
        "--rotation-rate", type=int, default=0, metavar="HZ",
        help="ROTATION_VECTOR report rate (default 0 = same as --rate; a slower rate throttles "
             "the whole packet stream of the node to that rate)",
    )
    parser.add_argument(
        "--calib-seconds", type=float, default=3.0, metavar="S",
        help="stationary window at the start used to estimate the gyro bias and accel "
             "statistics (default 3; 0 = skip)",
    )
    parser.add_argument(
        "--print-hz", type=float, default=5.0, metavar="HZ",
        help="how often live sample lines are printed (default 5; 0 = only calibration and "
             "summary)",
    )
    parser.add_argument(
        "--batch", type=int, default=1, metavar="N",
        help="IMU batchReportThreshold = packets per IMUData message (default 1 = lowest "
             "latency; 10 = fewer, larger messages)",
    )
    parser.add_argument(
        "--plot", action="store_true",
        help="live scrolling matplotlib plot of accel / gyro (/ roll-pitch-yaw); skipped with "
             "a message if matplotlib is missing, the backend has no display, or --headless",
    )
    parser.add_argument(
        "--plot-window", type=float, default=5.0, metavar="S",
        help="seconds of history shown in the --plot window (default 5)",
    )
    args = parser.parse_args(argv)
    if args.rate <= 0:
        parser.error("--rate must be a positive integer")
    if args.rotation_rate < 0:
        parser.error("--rotation-rate must be >= 0")
    if args.calib_seconds < 0:
        parser.error("--calib-seconds must be >= 0")
    if args.print_hz < 0:
        parser.error("--print-hz must be >= 0")
    if args.batch < 1:
        parser.error("--batch must be >= 1")
    if args.plot_window <= 0:
        parser.error("--plot-window must be > 0")
    return args


# --------------------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------------------

def version_text(getter: Callable[[], object]) -> str:
    """``dai.Version`` -> '3.9.9' (or '?' if the query fails on exotic hardware)."""
    try:
        version = getter()
    except Exception:  # noqa: BLE001 - informational only
        return "?"
    try:
        return str(version.toStringSemver())  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        return str(version)


def rotation_vector_supported(imu_name: str) -> bool:
    """Sensor fusion (ROTATION_VECTOR) exists on the BNO08x; BMI-family IMUs only have raw
    sensors.  Same rule as the official ``IMU/imu_rotation_vector.py`` example."""
    return not imu_name.strip().upper().startswith("BMI")


def quaternion_to_euler_deg(i: float, j: float, k: float, real: float) -> Vec3:
    """Unit quaternion -> (roll about x, pitch about y, yaw about z) in degrees.

    Same formula as the official ``imu_rotation_vector.py`` example.
    """
    sinr_cosp = 2.0 * (real * i + j * k)
    cosr_cosp = 1.0 - 2.0 * (i * i + j * j)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (real * j - k * i)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (real * k + i * j)
    cosy_cosp = 1.0 - 2.0 * (j * j + k * k)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def enum_name(value: object) -> str:
    """Short name of a pybind enum member (``Accuracy.HIGH`` -> ``'HIGH'``)."""
    return str(getattr(value, "name", value))


def fmt3(values: Sequence[float], spec: str) -> str:
    """Format three numbers with one format spec, e.g. fmt3(v, '+.4f')."""
    return " ".join(format(float(v), spec) for v in values)


# --------------------------------------------------------------------------------------
# Stationary (bias) statistics
# --------------------------------------------------------------------------------------

@dataclasses.dataclass
class CalibResult:
    """Statistics of the stationary window (numpy arrays are per-axis x, y, z)."""

    samples: int
    gyro_bias: np.ndarray
    gyro_std: np.ndarray
    accel_mean: np.ndarray
    accel_std: np.ndarray
    norm_mean: float
    norm_std: float

    def looks_stationary(self) -> bool:
        return (float(self.gyro_std.max()) <= MAX_STILL_GYRO_STD
                and float(self.accel_std.max()) <= MAX_STILL_ACCEL_STD)


class StationaryStats:
    """Collects accel/gyro samples while the device is assumed to be at rest."""

    def __init__(self) -> None:
        self._accel: List[Vec3] = []
        self._gyro: List[Vec3] = []

    def add(self, accel: Vec3, gyro: Vec3) -> None:
        self._accel.append(accel)
        self._gyro.append(gyro)

    def __len__(self) -> int:
        return len(self._accel)

    def result(self) -> Optional[CalibResult]:
        if not self._accel:
            return None
        accel = np.asarray(self._accel, dtype=np.float64)
        gyro = np.asarray(self._gyro, dtype=np.float64)
        norms = np.linalg.norm(accel, axis=1)
        return CalibResult(
            samples=int(accel.shape[0]),
            gyro_bias=gyro.mean(axis=0),
            gyro_std=gyro.std(axis=0),
            accel_mean=accel.mean(axis=0),
            accel_std=accel.std(axis=0),
            norm_mean=float(norms.mean()),
            norm_std=float(norms.std()),
        )


def print_calibration(result: CalibResult, window_s: float, complete: bool) -> None:
    """Print the stationary statistics block."""
    status = "" if complete else " - INCOMPLETE (run ended early)"
    print("Stationary calibration ({:.1f} s window, {} samples){}:".format(
        window_s, result.samples, status), flush=True)
    bias_deg = np.degrees(result.gyro_bias)
    print("  gyro bias  [rad/s]: {}   (= {} deg/s)".format(
        fmt3(result.gyro_bias, "+.5f"), fmt3(bias_deg, "+.3f")), flush=True)
    print("  gyro std   [rad/s]: {}".format(fmt3(result.gyro_std, ".5f")), flush=True)
    print("  accel mean [m/s^2]: {}".format(fmt3(result.accel_mean, "+.4f")), flush=True)
    print("  accel std  [m/s^2]: {}".format(fmt3(result.accel_std, ".4f")), flush=True)
    deviation = 100.0 * (result.norm_mean - GRAVITY_MS2) / GRAVITY_MS2
    print("  accel norm [m/s^2]: mean {:.4f} std {:.4f}   (g = {:.3f}, deviation {:+.2f} %)".format(
        result.norm_mean, result.norm_std, GRAVITY_MS2, deviation), flush=True)
    axis = int(np.argmax(np.abs(result.accel_mean)))
    sign = "-" if result.accel_mean[axis] < 0 else "+"
    print("  gravity is mostly along {}{} of the IMU frame".format(sign, "xyz"[axis]), flush=True)
    if not result.looks_stationary():
        print("  WARNING: the sensor did not look stationary (gyro std > {} rad/s or accel std > "
              "{} m/s^2); hold the camera still during the first {:.1f} s and rerun.".format(
                  MAX_STILL_GYRO_STD, MAX_STILL_ACCEL_STD, window_s), flush=True)


# --------------------------------------------------------------------------------------
# Optional live plot (matplotlib is imported lazily; never required)
# --------------------------------------------------------------------------------------

class LivePlot:
    """Scrolling plot of accel / gyro (/ roll-pitch-yaw).  Raises ImportError when
    matplotlib is missing and RuntimeError when its backend cannot open a window."""

    NON_INTERACTIVE = ("agg", "pdf", "ps", "svg", "template", "cairo", "pgf")

    def __init__(self, window_s: float, with_rotation: bool, rate_hz: float) -> None:
        import matplotlib  # optional dependency - only needed for --plot
        import matplotlib.pyplot as plt

        backend = str(matplotlib.get_backend())
        if backend.lower() in self.NON_INTERACTIVE:
            raise RuntimeError("matplotlib backend '{}' cannot open a window (no display?)".format(backend))
        self._plt = plt
        self.window_s = float(window_s)
        self.closed = False
        maxlen = int(self.window_s * max(rate_hz, 1.0) * 1.5) + 16
        self._t: Deque[float] = collections.deque(maxlen=maxlen)
        self._data: Dict[str, Deque[float]] = {}
        self._lines: Dict[str, object] = {}

        panels: List[Tuple[str, Tuple[str, str, str]]] = [
            ("accel [m/s^2]", ("ax", "ay", "az")),
            ("gyro - bias [rad/s]", ("gx", "gy", "gz")),
        ]
        if with_rotation:
            panels.append(("rotation vector [deg]", ("roll", "pitch", "yaw")))
        self.fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(9.0, 2.6 * len(panels)))
        self._axes = list(np.atleast_1d(axes))
        for axis, (label, keys) in zip(self._axes, panels):
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            for key in keys:
                self._data[key] = collections.deque(maxlen=maxlen)
                (line,) = axis.plot([], [], label=key, linewidth=1.0)
                self._lines[key] = line
            axis.legend(loc="upper left", fontsize="small", ncol=3)
        self._axes[-1].set_xlabel("time since start [s]")
        self.fig.suptitle("OAK IMU - press q or close the window to stop")
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        try:
            self.fig.tight_layout()
        except Exception:  # noqa: BLE001 - cosmetic only
            pass
        plt.show(block=False)

    def _on_close(self, _event: object) -> None:
        self.closed = True

    def add(self, t: float, accel: Vec3, gyro: Vec3, rpy: Optional[Vec3]) -> None:
        self._t.append(t)
        for key, value in zip(("ax", "ay", "az"), accel):
            self._data[key].append(value)
        for key, value in zip(("gx", "gy", "gz"), gyro):
            self._data[key].append(value)
        if "roll" in self._data:
            values = rpy if rpy is not None else (math.nan, math.nan, math.nan)
            for key, value in zip(("roll", "pitch", "yaw"), values):
                self._data[key].append(value)

    def refresh(self) -> None:
        """Redraw and pump the GUI event loop (call ~15x per second)."""
        if self.closed:
            return
        if self._t:
            times = list(self._t)
            for key, line in self._lines.items():
                values = list(self._data[key])
                n = min(len(times), len(values))
                line.set_data(times[-n:], values[-n:])  # type: ignore[attr-defined]
            t_max = times[-1]
            self._axes[0].set_xlim(max(0.0, t_max - self.window_s), max(t_max, self.window_s))
            for axis in self._axes:
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.start_event_loop(0.001)
        except Exception:  # noqa: BLE001 - fall back to the classic pump
            try:
                self._plt.pause(0.001)
            except Exception:  # noqa: BLE001 - window already gone
                self.closed = True

    def close(self) -> None:
        try:
            self._plt.close(self.fig)
        except Exception:  # noqa: BLE001
            pass


def make_plot(args: argparse.Namespace, with_rotation: bool) -> Optional[LivePlot]:
    """Create the live plot for --plot, or explain why it is skipped and return None."""
    if not args.plot:
        return None
    if args.headless:
        print("--plot ignored because --headless was given.", flush=True)
        return None
    try:
        return LivePlot(args.plot_window, with_rotation, float(args.rate))
    except ImportError as exc:
        print("matplotlib is not available ({}); continuing without --plot. "
              "Install it with: pip install matplotlib".format(exc), flush=True)
    except Exception as exc:  # noqa: BLE001 - e.g. no display / backend failure
        print("Cannot open the live plot ({}: {}); continuing without --plot.".format(
            type(exc).__name__, exc), flush=True)
    return None


# --------------------------------------------------------------------------------------
# Streaming session
# --------------------------------------------------------------------------------------

@dataclasses.dataclass
class RotSample:
    i: float
    j: float
    k: float
    real: float
    accuracy_rad: float
    accuracy_name: str

    def rpy(self) -> Vec3:
        return quaternion_to_euler_deg(self.i, self.j, self.k, self.real)


@dataclasses.dataclass
class Sample:
    t: float  # seconds since streaming started (host clock)
    accel: Vec3
    gyro: Vec3
    latency_ms: float
    rot: Optional[RotSample]


def batched_rate(packets: int, messages: int, span_s: float) -> float:
    """Packets per second when *packets* arrived in *messages* IMUData messages whose first and
    last arrivals are *span_s* apart: mean packets per message x message rate.

    All packets of one message share one host arrival time, so the naive
    ``(packets - 1) / span`` over-reads by ``1 / (messages - 1)`` with ``--batch`` > 1; this
    form is exact for evenly batched messages and reduces to it for one packet per message.
    """
    if messages < 2 or span_s <= 0.0:
        return 0.0
    return packets * (messages - 1) / (messages * span_s)


class RollingRate:
    """Packet rate over the last *window_s* seconds of message arrivals (batch-aware)."""

    def __init__(self, window_s: float = RATE_WINDOW_S) -> None:
        self.window_s = max(0.05, float(window_s))
        self._msgs: Deque[Tuple[float, int]] = collections.deque()

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._msgs and self._msgs[0][0] < cutoff:
            self._msgs.popleft()

    def note(self, now: float, packets: int) -> None:
        self._msgs.append((now, packets))
        self._trim(now)

    def hz(self) -> float:
        now = time.monotonic()
        self._trim(now)
        if len(self._msgs) < 2:
            return 0.0
        span = self._msgs[-1][0] - self._msgs[0][0]
        return batched_rate(sum(n for _, n in self._msgs), len(self._msgs), span)


class StreamStats:
    """Packet totals and accelerometer-timestamp spacing statistics.

    Counts only what arrives after the settle window, so ``packets``, ``messages``, the
    wall-clock span and both measured rates all describe the same interval (from the first
    counted packet to the last); ``discarded`` is what the settle window threw away.
    """

    def __init__(self, nominal_rate_hz: float) -> None:
        self.nominal_period = 1.0 / float(nominal_rate_hz)
        self.messages = 0  # IMUData messages with at least one counted packet
        self.packets = 0
        self.discarded = 0  # packets dropped during the settle window
        self.first_ts: Optional[float] = None  # device timestamps (s)
        self.last_ts: Optional[float] = None
        self.first_wall: Optional[float] = None  # host arrival (time.monotonic)
        self.last_wall: Optional[float] = None
        self.max_gap = 0.0
        self.gaps = 0
        self.rate = RollingRate(RATE_WINDOW_S)

    def note_message(self, now: float, counted: int, discarded: int) -> None:
        """Book one IMUData message: *counted* packets were accepted, *discarded* skipped."""
        self.discarded += discarded
        if counted > 0:
            self.messages += 1
            self.rate.note(now, counted)

    def note_packet(self, ts_device_s: float, now: float) -> None:
        self.packets += 1
        if self.first_wall is None:
            self.first_wall = now
        self.last_wall = now
        if self.first_ts is None:
            self.first_ts = ts_device_s
        elif self.last_ts is not None:
            spacing = ts_device_s - self.last_ts
            if spacing > self.max_gap:
                self.max_gap = spacing
            if spacing > GAP_FACTOR * self.nominal_period:
                self.gaps += 1
        self.last_ts = ts_device_s

    def wall_span(self) -> float:
        """Seconds between the host arrival of the first and the last counted packet."""
        if self.first_wall is None or self.last_wall is None:
            return 0.0
        return max(0.0, self.last_wall - self.first_wall)

    def wall_rate(self) -> float:
        """Packets per second measured from host arrival times of the counted messages."""
        return batched_rate(self.packets, self.messages, self.wall_span())

    def timestamp_rate(self) -> float:
        """Packets per second measured from device timestamps (first to last)."""
        if self.first_ts is None or self.last_ts is None or self.packets < 2:
            return 0.0
        span = self.last_ts - self.first_ts
        return (self.packets - 1) / span if span > 0 else 0.0

    def mean_spacing_ms(self) -> float:
        rate = self.timestamp_rate()
        return 1000.0 / rate if rate > 0 else 0.0


class ImuSession:
    """Owns the run loop: calibration window, live printing, plot updates, summary."""

    def __init__(self, args: argparse.Namespace, queue: dai.MessageQueue, pipeline: dai.Pipeline,
                 rotation_on: bool, plot: Optional[LivePlot]) -> None:
        self.args = args
        self.queue = queue
        self.pipeline = pipeline
        self.rotation_on = rotation_on
        self.plot = plot
        self.stats = StreamStats(args.rate)
        self.calib = StationaryStats()
        self.calib_result: Optional[CalibResult] = None
        self.calib_done = args.calib_seconds <= 0
        self.calib_start: Optional[float] = None
        self.bias = np.zeros(3, dtype=np.float64)
        self.printer: Optional[Periodic] = Periodic(1.0 / args.print_hz) if args.print_hz > 0 else None
        self.calib_ticker = Periodic(1.0)
        self.plot_ticker = Periodic(1.0 / PLOT_REFRESH_HZ)
        self.last: Optional[Sample] = None
        self.t0 = time.monotonic()
        self.settle_until = self.t0 + SETTLE_S
        self.clock = RunClock(args.duration)

    # -- main loop -----------------------------------------------------------------------

    def loop(self) -> str:
        """Run until a stop condition; returns the reason as text."""
        timeout = datetime.timedelta(milliseconds=250)
        if not self.calib_done:
            print("Hold the camera still for {:.1f} s (stationary calibration)...".format(
                self.args.calib_seconds), flush=True)
        while True:
            if self.clock.expired():
                return "duration of {:.1f} s reached".format(self.args.duration)
            if self.plot is not None and self.plot.closed:
                return "plot window closed"
            if not self.pipeline.isRunning():
                return "pipeline stopped"
            message = self.queue.get(timeout)  # None on timeout (keeps Ctrl-C responsive)
            if message is None:
                self._refresh_plot()
                continue
            batch = [message]
            batch.extend(self.queue.tryGetAll())  # drain bursts so nothing piles up
            for data in batch:
                now = time.monotonic()  # one host arrival time per message
                counted = 0
                for packet in data.packets:
                    if self._packet(packet, now):
                        counted += 1
                self.stats.note_message(now, counted, len(data.packets) - counted)
            self._print_live()
            self._refresh_plot()

    # -- per packet ----------------------------------------------------------------------

    def _packet(self, packet: dai.IMUPacket, now: float) -> bool:
        """Process one packet received at host time *now*; False if it fell in the settle window."""
        if now < self.settle_until:
            return False
        acc = packet.acceleroMeter
        gyr = packet.gyroscope
        accel: Vec3 = (float(acc.x), float(acc.y), float(acc.z))
        gyro: Vec3 = (float(gyr.x), float(gyr.y), float(gyr.z))
        self.stats.note_packet(acc.getTimestampDevice().total_seconds(), now)
        latency_ms = (dai.Clock.now() - acc.getTimestamp()).total_seconds() * 1000.0

        rot: Optional[RotSample] = None
        if self.rotation_on:
            rv = packet.rotationVector
            rot = RotSample(float(rv.i), float(rv.j), float(rv.k), float(rv.real),
                            float(rv.rotationVectorAccuracy), enum_name(rv.accuracy))

        if not self.calib_done:
            if self.calib_start is None:
                self.calib_start = now
            self.calib.add(accel, gyro)
            if now - self.calib_start >= self.args.calib_seconds:
                self.finish_calibration(complete=True)

        self.last = Sample(now - self.t0, accel, gyro, latency_ms, rot)
        if self.plot is not None:
            corrected = np.asarray(gyro, dtype=np.float64) - self.bias
            self.plot.add(self.last.t, accel,
                          (float(corrected[0]), float(corrected[1]), float(corrected[2])),
                          rot.rpy() if rot is not None else None)
        return True

    def finish_calibration(self, complete: bool) -> None:
        """Compute and print the stationary statistics once; apply the gyro bias."""
        if self.calib_done and self.calib_result is not None:
            return
        self.calib_done = True
        result = self.calib.result()
        if result is None:
            if not complete and self.args.calib_seconds > 0:
                print("Stationary calibration: no samples collected.", flush=True)
            return
        self.calib_result = result
        self.bias = np.array(result.gyro_bias, dtype=np.float64)
        print_calibration(result, self.args.calib_seconds, complete)
        if complete:
            print("Streaming (gyro shown with bias removed)...", flush=True)

    # -- output --------------------------------------------------------------------------

    def _print_live(self) -> None:
        if self.last is None or self.stats.first_wall is None:
            return
        if not self.calib_done:
            if self.calib_ticker.ready():
                print("  calibrating: {} samples, {:.1f} Hz".format(
                    len(self.calib), self.stats.rate.hz()), flush=True)
            return
        if time.monotonic() - self.stats.first_wall < LIVE_WARMUP_S:
            return  # the rate window is still too short to mean anything
        if self.printer is None or not self.printer.ready():
            return
        s = self.last
        corrected = np.asarray(s.gyro, dtype=np.float64) - self.bias
        norm = float(np.linalg.norm(s.accel))
        print("[{:7.2f} s] rate {:6.1f} Hz | acc {} (|a| {:.3f}) m/s^2 | gyro {} rad/s | lat {:5.1f} ms".format(
            s.t, self.stats.rate.hz(), fmt3(s.accel, "+7.3f"), norm, fmt3(corrected, "+8.4f"),
            s.latency_ms), flush=True)
        if s.rot is not None:
            roll, pitch, yaw = s.rot.rpy()
            quat = fmt3((s.rot.i, s.rot.j, s.rot.k), "+.4f") + " " + format(s.rot.real, "+.4f")
            print("            rot: roll {:+7.2f} pitch {:+7.2f} yaw {:+7.2f} deg | q(i,j,k,real) {} | "
                  "accuracy {:.4f} rad ({})".format(
                      roll, pitch, yaw, quat, s.rot.accuracy_rad, s.rot.accuracy_name), flush=True)

    def _refresh_plot(self) -> None:
        if self.plot is not None and self.plot_ticker.ready():
            self.plot.refresh()

    def print_summary(self, reason: str) -> None:
        st = self.stats
        print("Summary ({}): {} packets in {} IMUData messages over {:.2f} s (run time {:.2f} s; "
              "{} packets discarded in the first {:.2f} s)".format(
                  reason, st.packets, st.messages, st.wall_span(), self.clock.elapsed(),
                  st.discarded, SETTLE_S), flush=True)
        print("  requested {} Hz -> measured {:.1f} Hz (wall clock), {:.1f} Hz (device timestamps)".format(
            self.args.rate, st.wall_rate(), st.timestamp_rate()), flush=True)
        if st.packets >= 2:
            print("  accel timestamp spacing: mean {:.2f} ms, max {:.2f} ms; gaps > {:.1f}x period: {}".format(
                st.mean_spacing_ms(), st.max_gap * 1000.0, GAP_FACTOR, st.gaps), flush=True)
        if self.calib_result is not None:
            print("  gyro bias applied [rad/s]: {}".format(fmt3(self.bias, "+.5f")), flush=True)
        if self.last is not None and self.last.rot is not None:
            roll, pitch, yaw = self.last.rot.rpy()
            print("  last rotation vector: roll {:+.2f} pitch {:+.2f} yaw {:+.2f} deg, accuracy {:.4f} rad ({})".format(
                roll, pitch, yaw, self.last.rot.accuracy_rad, self.last.rot.accuracy_name), flush=True)
        if st.packets == 0:
            print("  WARNING: no IMU packets were received.", flush=True)


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    device: Optional[dai.Device] = None
    plot: Optional[LivePlot] = None
    pipeline: Optional[dai.Pipeline] = None
    session: Optional[ImuSession] = None
    exit_code = EXIT_OK
    try:
        # Inside the try so that Ctrl-C during the ~3 s boot (or a retry wait) is reported as
        # one stderr line + exit 130 instead of a traceback.  open_device exits 2 on failure:
        # an explicit --device-id never retries (one stderr line even when nothing enumerates);
        # without an id the open is retried while a camera reboots after another script.
        device = open_device(args.device_id, retries=1 if args.device_id else 3)
        print_device_summary(device)
        try:
            imu_name = str(device.getConnectedIMU() or "")
        except Exception:  # noqa: BLE001 - treat a failing query as "no IMU"
            imu_name = ""
        if not imu_name.strip() or imu_name.strip().upper() in ("NONE", "UNKNOWN"):
            print("This device reports no IMU (getConnectedIMU() = {!r}); nothing to test.".format(imu_name),
                  file=sys.stderr, flush=True)
            return EXIT_NO_IMU
        print("IMU: {} firmware {} (embedded firmware {})".format(
            imu_name, version_text(device.getIMUFirmwareVersion),
            version_text(device.getEmbeddedIMUFirmwareVersion)), flush=True)

        # Decide about the rotation vector before building the pipeline.
        rotation_on = False
        rotation_rate = args.rotation_rate or args.rate
        if args.rotation and not rotation_vector_supported(imu_name):
            print("ROTATION_VECTOR is not supported by this IMU ({}); streaming raw sensors only.".format(
                imu_name), flush=True)
        elif args.rotation:
            rotation_on = True

        plot = make_plot(args, rotation_on)

        pipeline = dai.Pipeline(device)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], args.rate)
        if rotation_on:
            try:
                imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, rotation_rate)
            except Exception as exc:  # noqa: BLE001 - keep going with raw sensors
                rotation_on = False
                print("Could not enable ROTATION_VECTOR ({}: {}); streaming raw sensors only.".format(
                    type(exc).__name__, exc), flush=True)
        imu.setBatchReportThreshold(args.batch)
        imu.setMaxBatchReports(max(10, args.batch))
        queue = imu.out.createOutputQueue(maxSize=100, blocking=False)

        sensors = "ACCELEROMETER_RAW + GYROSCOPE_RAW @ {} Hz".format(args.rate)
        if rotation_on:
            sensors += " + ROTATION_VECTOR @ {} Hz".format(rotation_rate)
            if rotation_rate != args.rate:
                sensors += " (note: one node emits packets at the slowest enabled rate)"
        print("IMU node: {}; batchReportThreshold={} maxBatchReports={}".format(
            sensors, args.batch, max(10, args.batch)), flush=True)

        pipeline.start()
        session = ImuSession(args, queue, pipeline, rotation_on, plot)
        try:
            reason = session.loop()
        except KeyboardInterrupt:
            reason = "Ctrl-C"
        except dai.MessageQueue.QueueException as exc:
            reason = "queue closed - {}".format(" ".join(str(exc).split()))
            exit_code = EXIT_PIPELINE_ERROR
        except RuntimeError as exc:
            reason = "pipeline error - {}: {}".format(type(exc).__name__, " ".join(str(exc).split()))
            exit_code = EXIT_PIPELINE_ERROR
        if not session.calib_done or session.calib_result is None:
            session.finish_calibration(complete=False)
        session.print_summary(reason)
        if exit_code != EXIT_OK:
            print("Run ended with an error: {}".format(reason), file=sys.stderr, flush=True)
        return exit_code
    except KeyboardInterrupt:
        # Ctrl-C while the device boots / the pipeline is built, or a second Ctrl-C while the
        # summary is being printed (the streaming loop handles its own Ctrl-C above).
        when = "while shutting down" if session is not None else "before streaming started"
        print("Interrupted {}.".format(when), file=sys.stderr, flush=True)
        return EXIT_INTERRUPTED_EARLY
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()  # also closes the device
            except Exception:  # noqa: BLE001 - already stopped / device gone
                pass
        if device is not None:
            try:
                device.close()
            except Exception:  # noqa: BLE001 - harmless after pipeline.stop()
                pass
        if plot is not None:
            plot.close()


if __name__ == "__main__":
    sys.exit(main())
