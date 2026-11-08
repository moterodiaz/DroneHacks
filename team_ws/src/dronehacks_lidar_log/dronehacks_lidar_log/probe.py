"""Read the lidar for a few seconds and report whether valid MS200 frames arrive. Read-only.

    ros2 run dronehacks_lidar_log lidar_probe [seconds]      # in the stack container

Counts 47-byte frames (header 54 2C, CRC-8 poly 0x4D, as depthcam/course/ms200.py parses them) and
prints the rotation speed and a few ranges, so a wrong baud or a dead port shows before a flight.
"""
from __future__ import annotations

import struct
import sys
import time

from .logger import open_port

HEADER = b"\x54\x2c"
FRAME = struct.Struct("<BBHH" + "HB" * 12 + "HHB")


def _table(poly: int = 0x4D) -> bytes:
    out = []
    for byte in range(256):
        c = byte
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        out.append(c)
    return bytes(out)


CRC = _table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC[c ^ b]
    return c


def frames(buf: bytearray):
    """Pop every complete, CRC-valid frame from `buf`; yields (speed deg/s, start deg, distances mm)."""
    while True:
        i = buf.find(HEADER)
        if i < 0:
            del buf[:max(0, len(buf) - 1)]
            return
        del buf[:i]
        if len(buf) < FRAME.size:
            return
        raw = bytes(buf[:FRAME.size])
        if crc8(raw[:-1]) != raw[-1]:
            del buf[:2]
            continue
        del buf[:FRAME.size]
        f = FRAME.unpack(raw)
        yield f[2], f[3] / 100.0, f[4:28:2]


def main(argv=None) -> int:
    seconds = float((argv or sys.argv[1:] or ["3"])[0])
    name, opener = open_port("/dev/ms200", 230400)
    if name is None:
        print("probe: no lidar port found (no serial device and no CH340 on USB)")
        return 1
    print(f"probe: reading {name} for {seconds:.0f} s")
    total, n, speeds, sample, buf = 0, 0, [], None, bytearray()
    with opener() as port:
        t_end = time.time() + seconds
        while time.time() < t_end:
            data = port.read(4096)
            total += len(data)
            buf += data
            for speed, start, dist in frames(buf):
                n += 1
                speeds.append(speed)
                if sample is None and any(dist):
                    sample = (start, dist)
    print(f"probe: {total} bytes, {n} valid frames ({n / seconds:.0f}/s; skybeta-11's MS200K sent 377/s at 10 Hz)")
    if n:
        print(f"probe: rotation {sum(speeds) / len(speeds) / 360:.1f} Hz; ranges at {sample[0] if sample else 0:.1f} deg: "
              f"{list(sample[1]) if sample else 'all zero'} mm")
        print("probe: PASS")
        return 0
    print("probe: FAIL: " + ("no bytes at all (port or power)" if not total else "bytes but no valid frames (baud?)"))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
