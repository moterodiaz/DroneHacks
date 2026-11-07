"""Oradar MS200 / MS200K serial reader that keeps every point's own time and angle.

Why not the vendor ROS node (``MS200_Ros-V1.3.4``, ``oradar_scan``)
---------------------------------------------------------------------
Scan-smear correction (``lidar_localize.match`` with ``deskew``) needs each point's time, and the
vendor node loses it:

* it drops the sensor's per-frame timestamps;
* it stamps a scan with the moment it started *waiting* for it;
* it sets ``time_increment`` from however long that wait took (near zero if a scan was queued);
* it re-bins the points onto an even angle grid;
* with ``clockwise: false`` it stores them in reverse time order.

This module reads the serial stream itself (230400 baud) and returns every revolution with
per-point host times, exact angles, ranges and confidences.  Parsing needs only numpy;
``pyserial`` is needed only to open a port.  One program at a time can hold the port: stop the
vendor node before using this.

Frame format (vendor SDK ``sdk/src/ordlidar_protocol.h``; the LD06 / LD19 layout)
---------------------------------------------------------------------------------
47 bytes, little-endian::

    0x54 0x2C | speed u16 deg/s | start angle u16 0.01 deg | 12 x (distance u16 mm, confidence u8)
              | end angle u16 0.01 deg | timestamp u16 ms | CRC-8 (poly 0x4D) of the 46 bytes before

The 12 points are evenly spaced from the start to the end angle.  The angle grows clockwise seen
from above (the vendor node mirrors it for ROS when ``clockwise`` is false).  Distance 0 means no
return.

Times
-----
The frame timestamp is the sensor's clock in ms.  It wraps: LD06-family sensors wrap at 30000,
and a sensor using the full u16 wraps at 65536.  Both are handled, by comparing with the host clock.

Sensor time is mapped to the host's ``time.monotonic()``.  The map uses the smallest (host arrival
- sensor time) seen in the last ``sync_window_s``, which is the least-delayed frame, minus
``latency_s``.  ``latency_s`` is the part of the delay that no frame avoids:

* the frame being sent after its 12th point;
* 2 ms on the wire;
* the USB adapter.

``latency_s`` is a guess until measured.  An error in it shifts every time alike, which PX4's
``EKF2_EV_DELAY`` absorbs; it does not bend the scan.

Within a frame, points are spaced by the angle step divided by the rotation speed.  Assumed, not
documented: the timestamp belongs to the frame's first point.  If it belongs to the last point,
every time is 2.4 ms early (5 mm at 2 m/s).
"""

from __future__ import annotations

import struct
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator, List, Optional, Tuple

import numpy as np

BAUD = 230400
HEADER = b"\x54\x2c"
POINTS_PER_FRAME = 12
_FRAME = struct.Struct("<BBHH" + "HB" * POINTS_PER_FRAME + "HHB")
FRAME_LEN = _FRAME.size  # 47
STAMP_WRAPS = (30000, 65536)


def _crc_table(poly: int = 0x4D) -> bytes:
    table = []
    for byte in range(256):
        c = byte
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return bytes(table)


_CRC = _crc_table()  # equal to the SDK's CrcTable


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = _CRC[c ^ b]
    return c


@dataclass
class Frame:
    speed_dps: float           # rotation speed, deg/s (10 Hz = 3600)
    start_deg: float
    end_deg: float
    distance_mm: np.ndarray    # (12,), 0 = no return
    confidence: np.ndarray     # (12,)
    stamp_ms: int              # sensor clock, wraps
    host_t: float              # host time.monotonic() when the frame had fully arrived

    @property
    def step_deg(self) -> float:
        return ((self.end_deg - self.start_deg) % 360.0) / (POINTS_PER_FRAME - 1)

    def angles_deg(self) -> np.ndarray:
        return (self.start_deg + self.step_deg * np.arange(POINTS_PER_FRAME)) % 360.0


def encode_frame(speed_dps: float, start_deg: float, end_deg: float, distance_mm, confidence, stamp_ms: int) -> bytes:
    """One frame as the sensor sends it (for tests and synthetic streams)."""
    fields = [0x54, 0x2C, int(round(speed_dps)) & 0xFFFF, int(round(start_deg * 100)) % 36000]
    for d, c in zip(distance_mm, confidence):
        fields += [int(d), int(c)]
    fields += [int(round(end_deg * 100)) % 36000, int(stamp_ms) & 0xFFFF, 0]
    raw = bytearray(_FRAME.pack(*fields))
    raw[-1] = crc8(raw[:-1])
    return bytes(raw)


class FrameParser:
    """Bytes -> frames.  Resyncs on the 0x54 0x2C header.  A frame whose CRC fails costs only its
    two header bytes, since the payload may hold the next real header.  The sensor's start-up text
    (serial number, firmware, ``AA 55`` frames) is skipped the same way."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes, host_t: float) -> List[Frame]:
        self.buf += data
        out: List[Frame] = []
        while True:
            i = self.buf.find(HEADER)
            if i < 0:
                keep = 1 if self.buf[-1:] == HEADER[:1] else 0
                self.skipped_bytes += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if i:
                self.skipped_bytes += i
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                return out
            raw = bytes(self.buf[:FRAME_LEN])
            if crc8(raw[:-1]) != raw[-1]:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            del self.buf[:FRAME_LEN]
            f = _FRAME.unpack(raw)
            pts = np.array(f[4:4 + 2 * POINTS_PER_FRAME]).reshape(POINTS_PER_FRAME, 2)
            out.append(Frame(float(f[2]), f[3] / 100.0, (f[28] % 36000) / 100.0, pts[:, 0], pts[:, 1], f[29], host_t))
            self.frames += 1


@dataclass
class Revolution:
    """One turn of the sensor, in the same terms as ``lidar_sim.Scan``."""

    t: np.ndarray             # (n,) host time of every point, s (time.monotonic clock)
    angle: np.ndarray         # (n,) beam angle in the body frame, rad, CCW from the nose
    range: np.ndarray         # (n,) m, NaN = no return, out of limits or low confidence
    confidence: np.ndarray    # (n,)
    raw_deg: np.ndarray       # (n,) the sensor's own angle
    rate_hz: float            # rotation rate reported by the sensor
    missing_frames: int       # frames lost inside this turn (angle gaps)
    clock_ok: bool            # the host <-> sensor clock map had settled

    @property
    def t_end(self) -> float:
        return float(self.t[-1])


class Assembler:
    """Frames -> revolutions with per-point host times.

    ``clockwise``: the sensor angle grows clockwise seen from above (sensor mounted right way up).
    ``yaw_offset_deg``: where the sensor's 0 deg points, CCW from the drone's nose.
    ``range_min``: about 0.25 m on the drone, so its own arms are ignored."""

    def __init__(self, clockwise: bool = True, yaw_offset_deg: float = 0.0, range_min: float = 0.05,
                 range_max: float = 12.0, min_confidence: int = 0, latency_s: float = 0.005,
                 sync_window_s: float = 2.0) -> None:
        self.sign = -1.0 if clockwise else 1.0
        self.yaw_offset = np.radians(yaw_offset_deg)
        self.range_min, self.range_max, self.min_confidence = range_min, range_max, min_confidence
        self.latency_s, self.sync_window_s = latency_s, sync_window_s
        self.clock_resets = 0
        self.last_excess_s = 0.0
        self.dropped_partial = 0
        self._reset_clock()
        self._cur: List[Tuple[np.ndarray, ...]] = []
        self._cur_missing = 0
        self._cur_speeds: List[float] = []
        self._last_raw: Optional[float] = None
        self._last_end: Optional[float] = None

    def _reset_clock(self) -> None:
        self._prev_stamp: Optional[int] = None
        self._prev_host = 0.0
        self._sensor_t = 0.0
        self._first_host: Optional[float] = None
        self._lows: deque = deque()  # (host_t, offset), increasing offsets: the sliding minimum

    def _frame_time(self, frame: Frame) -> float:
        """Unwrapped sensor time of the frame's first point (s); updates the clock map."""
        if self._prev_stamp is not None:
            step = frame.stamp_ms - self._prev_stamp
            host_dt = frame.host_t - self._prev_host
            if step < 0:
                wraps = [step % m for m in STAMP_WRAPS if self._prev_stamp < m]
                step = min(wraps, key=lambda s: abs(s / 1000.0 - host_dt))
            if abs(step / 1000.0 - host_dt) > 1.0:  # the sensor restarted, or a long stall
                self._reset_clock()
                self.clock_resets += 1
                # the turn in progress keeps only its points on the new clock; starting mid-turn,
                # _close drops it as partial
                self._cur, self._cur_speeds, self._cur_missing = [], [], 0
            else:
                self._sensor_t += step / 1000.0
        if self._prev_stamp is None:
            self._sensor_t = 0.0
            self._first_host = frame.host_t
        self._prev_stamp, self._prev_host = frame.stamp_ms, frame.host_t
        offset = frame.host_t - self._sensor_t
        while self._lows and self._lows[-1][1] >= offset:
            self._lows.pop()
        self._lows.append((frame.host_t, offset))
        while self._lows[0][0] < frame.host_t - self.sync_window_s:
            self._lows.popleft()
        self.last_excess_s = offset - self._lows[0][1]  # how much later than the fastest frame (diagnostics)
        return self._sensor_t

    @property
    def clock_ok(self) -> bool:
        return self._first_host is not None and self._prev_host - self._first_host >= 0.5 * self.sync_window_s

    def add(self, frame: Frame) -> List[Revolution]:
        s0 = self._frame_time(frame)
        raw = frame.angles_deg()
        dt = frame.step_deg / frame.speed_dps if frame.speed_dps > 0 else 1.0 / 4500.0
        s = s0 + dt * np.arange(POINTS_PER_FRAME)  # sensor times; _close maps them to host time
        rng = frame.distance_mm / 1000.0
        bad = ((frame.distance_mm == 0) | (rng < self.range_min) | (rng > self.range_max)
               | (frame.confidence < self.min_confidence))
        rng = np.where(bad, np.nan, rng)
        if self._last_end is not None and frame.step_deg > 0:
            gap = (frame.start_deg - self._last_end) % 360.0
            self._cur_missing += int(round(max(gap - frame.step_deg, 0.0) / (POINTS_PER_FRAME * frame.step_deg)))
        self._last_end = frame.end_deg
        prev = np.concatenate([[self._last_raw if self._last_raw is not None else raw[0]], raw[:-1]])
        wrap = np.flatnonzero(prev - raw > 180.0)
        out: List[Revolution] = []
        cut = int(wrap[0]) if len(wrap) else POINTS_PER_FRAME
        self._cur.append((s[:cut], raw[:cut], rng[:cut], frame.confidence[:cut]))
        self._cur_speeds.append(frame.speed_dps)
        if len(wrap):
            rev = self._close()
            if rev is not None:
                out.append(rev)
            self._cur = [(s[cut:], raw[cut:], rng[cut:], frame.confidence[cut:])]
            self._cur_speeds = [frame.speed_dps]
        self._last_raw = float(raw[-1])
        return out

    def _close(self) -> Optional[Revolution]:
        sensor_t, raw, rng, conf = (np.concatenate(parts) for parts in zip(*self._cur))
        missing, self._cur_missing = self._cur_missing, 0
        if len(raw) == 0 or raw[0] > 10.0:  # joined mid-turn: not a whole revolution
            self.dropped_partial += 1
            return None
        # one clock offset for the whole turn, the best one so far: no step inside the scan
        t = sensor_t + self._lows[0][1] - self.latency_s
        angle = np.mod(self.sign * np.radians(raw) + self.yaw_offset, 2 * np.pi)
        return Revolution(t, angle, rng, conf, raw, float(np.median(self._cur_speeds)) / 360.0, missing, self.clock_ok)


def revolutions(chunks: Iterable[Tuple[float, bytes]], parser: Optional[FrameParser] = None,
                assembler: Optional[Assembler] = None) -> Iterator[Revolution]:
    """(host time, bytes) chunks, from a port or a log, -> revolutions."""
    parser = parser or FrameParser()
    assembler = assembler or Assembler()
    for host_t, data in chunks:
        for frame in parser.feed(data, host_t):
            yield from assembler.add(frame)


def synthetic_stream(t: np.ndarray, angle: np.ndarray, range_m: np.ndarray, rate_hz: float, latency_s: float = 0.005,
                     jitter_s: float = 0.003, frames_per_read: int = 4, clock_start_ms: int = 29000,
                     wrap: int = 30000, clockwise: bool = True, seed: int = 0) -> List[Tuple[float, bytes]]:
    """Beams (true times, body angles CCW from the nose, ranges or NaN; e.g. ``lidar_sim`` scans
    joined end to end) -> the (host time, bytes) reads a port would deliver.  Each frame carries the
    sensor's own wrapping ms clock (starting at ``clock_start_ms``) for its first point and arrives
    ``latency_s`` plus random jitter (exponential, mean ``jitter_s``) after it.  Reads bundle a few
    frames, as a busy host would.  For tests and dry runs without the sensor."""
    rng = np.random.default_rng(seed)
    n = len(t) // POINTS_PER_FRAME * POINTS_PER_FRAME
    sign = -1.0 if clockwise else 1.0
    raw = np.mod(sign * np.degrees(angle[:n]), 360.0).reshape(-1, POINTS_PER_FRAME)
    dist = np.where(np.isfinite(range_m[:n]), np.round(range_m[:n] * 1000.0), 0).reshape(-1, POINTS_PER_FRAME)
    conf = np.where(dist > 0, 200, 0)
    first = t[:n].reshape(-1, POINTS_PER_FRAME)[:, 0]
    arrival = first + latency_s + rng.exponential(jitter_s, len(first))
    arrival = np.maximum.accumulate(arrival)  # a serial line keeps frames in order
    reads: List[Tuple[float, bytes]] = []
    pending = b""
    for k in range(len(first)):
        stamp = (clock_start_ms + int(round((first[k] - first[0]) * 1000.0))) % wrap
        pending += encode_frame(360.0 * rate_hz, raw[k, 0], raw[k, -1], dist[k], conf[k], stamp)
        if (k + 1) % frames_per_read == 0 or k == len(first) - 1:
            reads.append((float(arrival[k]), pending))
            pending = b""
    return reads


# --- the serial port ------------------------------------------------------------------------------

def open_port(port: str, baud: int = BAUD):
    try:
        import serial  # pyserial: pip install pyserial
    except ImportError as exc:
        raise RuntimeError("reading the lidar needs pyserial: pip install pyserial") from exc
    return serial.Serial(port, baud, timeout=0.05)


def read_port(ser, duration_s: Optional[float] = None) -> Iterator[Tuple[float, bytes]]:
    """(host time, bytes) as they arrive; the time is taken when each read returns."""
    end = None if duration_s is None else time.monotonic() + duration_s
    while end is None or time.monotonic() < end:
        data = ser.read(ser.in_waiting or 1)
        if data:
            yield time.monotonic(), data


def command(cmd: int, payload: bytes) -> bytes:
    """A write command as the SDK builds it: A5 F5, cmd, C1, length, payload, XOR, 31 F2."""
    body = struct.pack("<HBBB", 0xF5A5, cmd, 0xC1, len(payload)) + payload
    check = 0
    for b in body:
        check ^= b
    return body + bytes([check, 0x31, 0xF2])


def set_motor_hz(ser, hz: int) -> None:
    """Stop, set the rotation rate (5-15 Hz), start: the SDK's SetRotationSpeed sequence.
    Not yet tried on a sensor."""
    ser.write(command(0xA2, b"\x80"))
    time.sleep(0.4)
    ser.write(command(0xA1, bytes([max(5, min(15, int(hz))), 0])))
    time.sleep(0.4)
    ser.write(command(0xA2, b"\x81"))


# --- recordings: every chunk with its host time, for replay ----------------------------------------

LOG_MAGIC = b"MS200LOG1\n"
_RECORD = struct.Struct("<dI")


class Recorder:
    def __init__(self, path: Path) -> None:
        self.fh: BinaryIO = Path(path).open("wb")
        self.fh.write(LOG_MAGIC)

    def write(self, host_t: float, data: bytes) -> None:
        self.fh.write(_RECORD.pack(host_t, len(data)) + data)

    def close(self) -> None:
        self.fh.close()


def read_log(path: Path) -> Iterator[Tuple[float, bytes]]:
    with Path(path).open("rb") as fh:
        if fh.read(len(LOG_MAGIC)) != LOG_MAGIC:
            raise ValueError("{} is not an MS200 recording".format(path))
        while True:
            head = fh.read(_RECORD.size)
            if len(head) < _RECORD.size:
                return
            host_t, n = _RECORD.unpack(head)
            yield host_t, fh.read(n)
