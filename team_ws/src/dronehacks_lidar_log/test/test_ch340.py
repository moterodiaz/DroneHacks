import ctypes, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from dronehacks_lidar_log import ch340, probe


def test_usbfs_ioctl_numbers_match_linux_on_64_bit():
    assert ch340.USBDEVFS_CONTROL == 0xC0185500
    assert ch340.USBDEVFS_BULK == 0xC0185502
    assert ch340.USBDEVFS_CLAIMINTERFACE == 0x8004550F
    assert ch340.USBDEVFS_RELEASEINTERFACE == 0x80045510
    if ctypes.sizeof(ctypes.c_void_p) == 8:
        assert ctypes.sizeof(ch340._Ctrl) == 24 and ctypes.sizeof(ch340._Bulk) == 24


def test_divisor_matches_the_linux_ch341_driver():
    # 48 MHz / (4 * 26) at prescaler 3, fact 0: 230769 baud, 0.16 % fast
    assert ch340.divisor_value(230400, version=0x31) == 0xE683
    assert ch340.divisor_value(230400, version=0x27) == 0xE603   # bit 7 left clear on 0x27
    assert ch340.divisor_value(115200, version=0x31) == 0xCC83
    assert ch340.divisor_value(9600, version=0x31) == 0xB282     # the driver's default rate


def test_probe_parses_frames_and_skips_noise_and_bad_crc():
    def frame(start, dists):
        raw = bytearray(probe.FRAME.pack(0x54, 0x2C, 3600, int(start * 100), *sum(([d, 200] for d in dists), []), int((start + 7.5) * 100), 123, 0))
        raw[-1] = probe.crc8(raw[:-1])
        return bytes(raw)
    good = frame(10.0, range(1000, 1012))
    bad = bytearray(frame(20.0, [5] * 12)); bad[10] ^= 0xFF
    buf = bytearray(b"MS200 boot text" + good[:20])
    assert list(probe.frames(buf)) == []
    buf += good[20:] + bytes(bad) + good
    got = list(probe.frames(buf))
    assert [g[0] for g in got] == [3600, 3600] and got[0][1] == 10.0 and list(got[0][2]) == list(range(1000, 1012))
