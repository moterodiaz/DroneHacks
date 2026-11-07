"""Oradar MS200K 360-degree lidar as a virtual serial device (``/dev/ms200``).

The team's driver (``course/ms200.py``) reads the raw serial stream, so the
simulation feeds it the raw stream: 47-byte LD06-layout frames at 230400 baud,
``0x54 0x2C | speed u16 deg/s | start u16 0.01 deg | 12 x (mm u16, conf u8) |
end u16 0.01 deg | stamp u16 ms | CRC-8 poly 0x4D``.

* 10 Hz rotation, 4500 points/s -> 375 frames/s, 17.6 kB/s on the wire.
* Every point is raycast against the STEP mesh at its *own* time, from the
  sensor's true pose then, so motion during a revolution smears the scan exactly
  as on the drone, and tilt makes far beams hit the floor or pass over walls.
* Sensor angle grows clockwise seen from above; its 0 deg points
  ``mount_yaw_deg`` counter-clockwise from the nose (the team's
  ``yaw_offset_deg`` convention), at ``mount_xyz`` in the body FLU frame.
* Range 0.03-12 m, ~1 cm noise; no return -> distance 0, confidence 0.  The black
  drone net returns only some beams (``net_return_prob``); beams that leave the
  net see nothing (the hall is beyond range).
* The sensor clock is in ms and wraps at 30000, with an offset and a small
  drift against the Jetson's clock.
"""
from __future__ import annotations

import math
import struct

import numpy as np

from ..mathutil import q_rotate
from ..world.course import ned_to_course

POINTS_PER_FRAME = 12
_FRAME = struct.Struct('<BBHH' + 'HB' * POINTS_PER_FRAME + 'HHB')


def _rotmat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _crc_table(poly=0x4D):
    table = []
    for byte in range(256):
        c = byte
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return bytes(table)


_CRC = _crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = _CRC[c ^ b]
    return c


def encode_frame(speed_dps, start_deg, end_deg, distance_mm, confidence, stamp_ms):
    fields = [0x54, 0x2C, int(round(speed_dps)) & 0xFFFF, int(round(start_deg * 100)) % 36000]
    for d, c in zip(distance_mm, confidence):
        fields += [int(d) & 0xFFFF, int(c) & 0xFF]
    fields += [int(round(end_deg * 100)) % 36000, int(stamp_ms) & 0xFFFF, 0]
    raw = bytearray(_FRAME.pack(*fields))
    raw[-1] = crc8(raw[:-1])
    return bytes(raw)


DEFAULTS = {
    'rotation_hz': 10.0,
    'points_per_s': 4500.0,
    'baud': 230400,
    'mount_yaw_deg': 87.4,
    'mount_xyz': (0.0, 0.0, 0.035),
    'range_min_m': 0.03,
    'range_max_m': 12.0,
    'noise_m': 0.008,
    'net_return_prob': 0.45,
    'stamp_wrap': 30000,
    'clock_drift_ppm': 40.0,
    'usb_burst_s': 0.016,            # USB-UART adapters hand data over in packets, not bytes
    'enabled': True,
}


class MS200Device:
    def __init__(self, rt, physics, course, rng, cfg=None, history=None):
        self.rt = rt
        self.physics = physics
        self.course = course
        self.rng = rng
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg or {})
        self.history = history               # callable(t_s) -> (pos_ned, q) truth at time t
        self.buf = bytearray()
        self.next_frame_t = None             # sim seconds: when the next frame finishes
        self.angle_deg = rng.uniform(0.0, 360.0)
        self.stamp_offset_ms = rng.uniform(0.0, 30000.0)
        self.is_open = False
        self.frames_sent = 0
        self.last_scan = []                  # recent (angle_body_rad, range_m) for the GUI
        self._scan_acc = []
        n = self.cfg['points_per_s'] / POINTS_PER_FRAME
        self.frame_period = 1.0 / n
        self.byte_time = 10.0 / self.cfg['baud']

    # -- serial-device protocol (rosfake.devices) -------------------------------------------------
    def opened(self, baud):
        if int(baud) != int(self.cfg['baud']):
            self.rt.sim_log('WARN', 'ms200', f'/dev/ms200 opened at {baud} baud; the sensor talks at '
                                             f"{self.cfg['baud']}: the driver will read garbage")
        self.is_open = True
        if self.next_frame_t is None:
            self.next_frame_t = self.rt.now_s + self.frame_period

    def closed(self):
        self.is_open = False

    def write(self, data):
        pass        # motor-speed commands are accepted and ignored

    def _burst_after(self, t):
        b = self.cfg['usb_burst_s']
        return (math.floor(t / b) + 1) * b

    def next_byte_ns(self, after_ns):
        """When the USB adapter next hands bytes to the host (bursts of ``usb_burst_s``)."""
        if self.next_frame_t is None:
            return after_ns + 10_000_000
        ready = self.next_frame_t + 47 * self.byte_time
        return int(max(self._burst_after(ready - 1e-9), after_ns / 1e9 + 1e-6) * 1e9)

    def bytes_until(self, t_ns):
        if not self.cfg['enabled'] or self.next_frame_t is None:
            return b''
        b = self.cfg['usb_burst_s']
        delivered = math.floor(t_ns / 1e9 / b) * b      # data reaches the host on burst boundaries
        starts = []
        while self.next_frame_t + 47 * self.byte_time <= delivered:
            starts.append(self.next_frame_t - self.frame_period)
            self.next_frame_t += self.frame_period
        if starts:
            self.buf.extend(self._make_frames(starts))
        out = bytes(self.buf)
        self.buf.clear()
        return out

    # -- frames ----------------------------------------------------------------------------------
    def _make_frames(self, starts):
        cfg = self.cfg
        speed = 360.0 * cfg['rotation_hz']
        step_t = self.frame_period / POINTS_PER_FRAME
        step_deg = speed * step_t
        ts, angles, heads = [], [], []
        angle = self.angle_deg
        for t0 in starts:
            heads.append((t0, angle))
            for k in range(POINTS_PER_FRAME):
                ts.append(t0 + k * step_t)
                angles.append((angle + k * step_deg) % 360.0)
            angle = (angle + POINTS_PER_FRAME * step_deg) % 360.0
        self.angle_deg = angle
        ranges = self._ranges(ts, angles)
        drift = 1.0 + cfg['clock_drift_ppm'] * 1e-6
        out = bytearray()
        for f, (t0, start) in enumerate(heads):
            dist_mm, conf = [], []
            for r in ranges[f * POINTS_PER_FRAME:(f + 1) * POINTS_PER_FRAME]:
                if r is None:
                    dist_mm.append(0)
                    conf.append(0)
                else:
                    dist_mm.append(int(round(r * 1000.0)))
                    conf.append(int(max(20, min(240, 230 - 12 * r))))
            end = (start + (POINTS_PER_FRAME - 1) * step_deg) % 360.0
            stamp = int(self.stamp_offset_ms + t0 * 1000.0 * drift) % int(cfg['stamp_wrap'])
            out += encode_frame(speed, start, end, dist_mm, conf, stamp)
            self.frames_sent += 1
        return bytes(out)

    def _ranges(self, ts, angles):
        """Raycast every beam from the sensor pose at its frame's time (one pose per 12-point frame:
        the drone moves under 3 mm in the 2.7 ms a frame takes)."""
        cfg = self.cfg
        mx, my, mz = cfg['mount_xyz']
        yaw0 = math.radians(cfg['mount_yaw_deg'])
        phi = yaw0 - np.radians(np.asarray(angles))       # body FLU angle, CCW from the nose
        n = len(ts)
        nf = n // POINTS_PER_FRAME
        origins = np.empty((n, 3))
        dirs = np.empty((n, 3))
        # beam directions in body FRD: (cos phi, -sin phi, 0)
        d_frd = np.stack([np.cos(phi), -np.sin(phi), np.zeros(n)], axis=1)
        for f in range(nf):
            sl = slice(f * POINTS_PER_FRAME, (f + 1) * POINTS_PER_FRAME)
            t_mid = 0.5 * (ts[sl.start] + ts[sl.stop - 1])
            pos, q = self.history(t_mid) if self.history else (self.physics.state.pos, self.physics.state.q)
            R = _rotmat(q)
            o_ned = np.asarray(pos) + R @ np.array([mx, -my, -mz])
            d_ned = d_frd[sl] @ R.T
            origins[sl] = (o_ned[1], o_ned[0], -o_ned[2])
            dirs[sl, 0], dirs[sl, 1], dirs[sl, 2] = d_ned[:, 1], d_ned[:, 0], -d_ned[:, 2]
        body_angles = phi.tolist()
        dist, sid, _n = self.course.raycast_course(origins, dirs, cfg['range_max_m'])
        out = []
        for k in range(len(ts)):
            d = float(dist[k])
            ok = math.isfinite(d) and d >= cfg['range_min_m']
            if ok and sid[k] >= 0 and self.course.solid_kind[int(sid[k])] == 'net':
                ok = self.rng.random() < cfg['net_return_prob']
            r = d + self.rng.gauss(0.0, cfg['noise_m']) if ok else None
            out.append(r)
            self._scan_acc.append((body_angles[k], r))
        if len(self._scan_acc) >= cfg['points_per_s'] / cfg['rotation_hz']:
            self.last_scan = self._scan_acc
            self._scan_acc = []
        return out
