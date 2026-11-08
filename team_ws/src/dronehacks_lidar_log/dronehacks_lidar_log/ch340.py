"""Read a CH340 USB-serial adapter with no kernel driver, through Linux usbfs (/dev/bus/usb).

skybeta-11's L4T kernel (5.15.185-tegra) is built without CONFIG_USB_SERIAL_CH341, so the MS200K's
CH340 adapter (1a86:7523) gets no /dev/ttyUSB*. The stack container runs privileged as root with /dev
mounted, so it can claim the device directly. Standard library only (no pyusb in the image).

The setup follows Linux's drivers/usb/serial/ch341.c (5.15): serial init, the baud divisor and
prescaler register, the line-control register (8N1, RX and TX on), then DTR and RTS. Only
interface 0 of this one device is claimed; nothing else on the bus is touched.
"""
from __future__ import annotations

import ctypes
import errno
import glob
import os
from typing import Optional, Tuple

VID, PID = 0x1A86, 0x7523
BULK_IN = 0x82
READ_CHUNK = 256
CLKRATE = 48_000_000

# usbfs ioctls (linux/usbdevice_fs.h), _IOC(dir, 'U', nr, size) for a 64-bit kernel
USBDEVFS_CONTROL = (3 << 30) | (24 << 16) | (0x55 << 8) | 0
USBDEVFS_BULK = (3 << 30) | (24 << 16) | (0x55 << 8) | 2
USBDEVFS_CLAIMINTERFACE = (2 << 30) | (4 << 16) | (0x55 << 8) | 15
USBDEVFS_RELEASEINTERFACE = (2 << 30) | (4 << 16) | (0x55 << 8) | 16

REQ_READ_VERSION, REQ_WRITE_REG, REQ_SERIAL_INIT, REQ_MODEM_CTRL = 0x5F, 0x9A, 0xA1, 0xA4
REG_DIVISOR_PRESCALER, REG_LCR2_LCR = 0x1312, 0x2518
LCR_8N1_RX_TX = 0x80 | 0x40 | 0x03
DTR, RTS = 0x20, 0x40


class _Ctrl(ctypes.Structure):
    _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8), ("wValue", ctypes.c_uint16),
                ("wIndex", ctypes.c_uint16), ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


class _Bulk(ctypes.Structure):
    _fields_ = [("ep", ctypes.c_uint32), ("len", ctypes.c_uint32), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


if ctypes.sizeof(ctypes.c_void_p) == 8:  # the ioctl numbers above encode 24-byte structs
    assert ctypes.sizeof(_Ctrl) == 24 and ctypes.sizeof(_Bulk) == 24


def divisor_value(baud: int, version: int = 0x30) -> int:
    """The 0x1312 register value for `baud`, as ch341_get_divisor + set_baudrate_lcr compute it."""
    for ps in (3, 2, 1, 0):  # largest prescaler whose lowest rate is still at or below `baud`
        if baud >= CLKRATE // ((1 << (12 - 3 * ps - 1)) * 512):
            break
    else:
        raise ValueError(f"baud {baud} too low")
    fact = 1
    clk_div = 1 << (12 - 3 * ps - fact)
    div = CLKRATE // (clk_div * baud)
    if div < 9 or div > 255:
        raise ValueError(f"baud {baud} out of range")
    if 16 * CLKRATE // (clk_div * div) - 16 * baud >= 16 * baud - 16 * CLKRATE // (clk_div * (div + 1)):
        div += 1  # the next divisor is closer
    if fact == 1 and div % 2 == 0:  # prefer the lower base clock for an even divisor
        div //= 2
        fact = 0
    val = ((0x100 - div) << 8) | (fact << 2) | ps
    if version > 0x27:  # bit 7: don't buffer until a full 32-byte packet (inverted on 0x27)
        val |= 0x80
    return val


def find() -> Optional[Tuple[str, str]]:
    """(usbfs path, sysfs name) of the first CH340, or None."""
    for d in sorted(glob.glob("/sys/bus/usb/devices/*")):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                vid = int(f.read(), 16)
            with open(os.path.join(d, "idProduct")) as f:
                pid = int(f.read(), 16)
            if (vid, pid) != (VID, PID):
                continue
            with open(os.path.join(d, "busnum")) as f:
                bus = int(f.read())
            with open(os.path.join(d, "devnum")) as f:
                dev = int(f.read())
        except (OSError, ValueError):
            continue
        return f"/dev/bus/usb/{bus:03d}/{dev:03d}", os.path.basename(d)
    return None


class Ch340:
    """`with Ch340(path, 230400) as s: data = s.read(4096)`: read() returns b"" on a timeout."""

    def __init__(self, path: str, baud: int, timeout_ms: int = 100) -> None:
        self.path, self.baud, self.timeout_ms = path, baud, timeout_ms
        self.fd = -1
        self.version = 0
        self.buf = ctypes.create_string_buffer(4096)

    def __enter__(self) -> "Ch340":
        global fcntl
        import fcntl  # Linux only; imported here so the encodings stay testable elsewhere
        self.fd = os.open(self.path, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, USBDEVFS_CLAIMINTERFACE, ctypes.c_uint32(0))
            self._control(0xC0, REQ_READ_VERSION, 0, 0, 2)
            self.version = self.buf.raw[0]
            self._control(0x40, REQ_SERIAL_INIT, 0, 0)
            self._control(0x40, REQ_WRITE_REG, REG_DIVISOR_PRESCALER, divisor_value(self.baud, self.version))
            if self.version >= 0x30:  # older chips keep line control in other registers (8N1 by default)
                self._control(0x40, REQ_WRITE_REG, REG_LCR2_LCR, LCR_8N1_RX_TX)
            self._control(0x40, REQ_MODEM_CTRL, ~(DTR | RTS) & 0xFFFF, 0)
        except Exception:
            self.close()
            raise
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _control(self, req_type: int, req: int, value: int, index: int, length: int = 0) -> int:
        c = _Ctrl(req_type, req, value, index, length, 1000, ctypes.addressof(self.buf) if length else None)
        return fcntl.ioctl(self.fd, USBDEVFS_CONTROL, c)

    def read(self, n: int = 4096) -> bytes:
        # usbfs drops everything received when a bulk transfer times out, and a steady stream of full
        # 32-byte packets never ends one early: ask for little enough to finish well inside the timeout
        # (256 B is about 11 ms at 230400 baud).
        b = _Bulk(BULK_IN, min(n, READ_CHUNK, len(self.buf)), self.timeout_ms, ctypes.addressof(self.buf))
        try:
            got = fcntl.ioctl(self.fd, USBDEVFS_BULK, b)
        except OSError as exc:
            if exc.errno == errno.ETIMEDOUT:
                return b""
            raise
        return self.buf.raw[:got]

    def close(self) -> None:
        if self.fd >= 0:
            try:
                fcntl.ioctl(self.fd, USBDEVFS_RELEASEINTERFACE, ctypes.c_uint32(0))
            except OSError:
                pass
            os.close(self.fd)
            self.fd = -1
