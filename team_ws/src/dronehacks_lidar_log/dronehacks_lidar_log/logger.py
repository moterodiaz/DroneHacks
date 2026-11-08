"""Record the MS200K lidar's raw serial stream from arming until 5 s after disarm.

Read-only: it subscribes to the kit's /skybeta/state for `armed` and reads the lidar's port. It
publishes nothing and never touches PX4, so the flight code beside it is unaffected.

Each armed window becomes /root/team_ws/flights/<UTC>-lidar/ (the host's ~/team_ws/flights):
  lidar.bin   b"DHLIDAR1\\n", then records of struct "<dI" (host wall-clock time, length) + bytes
  meta.json   device, baud, start/stop times, bytes written
Wall-clock time (time.time) is what the flight bags use, so scans pair with their poses. The raw
bytes are MS200 frames: parse them with depthcam/course/ms200.py (FrameParser, Assembler).
"""
from __future__ import annotations

import datetime
import glob
import json
import os
import struct
import threading
import time
from typing import Optional

MAGIC = b"DHLIDAR1\n"
RECORD = struct.Struct("<dI")
CANDIDATES = ("/dev/ms200", "/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon_Labs*", "/dev/ttyUSB*")


def find_device(preferred: str) -> Optional[str]:
    """The configured device if present, else the first lidar-looking serial port."""
    for pattern in ((preferred,) if preferred else ()) + CANDIDATES:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


class Recorder:
    """Arming state in, files out. No ROS and no serial here, so it is testable on its own."""

    def __init__(self, root: str, tail_s: float = 5.0) -> None:
        self.root, self.tail_s = root, tail_s
        self.armed = False
        self.disarmed_at: Optional[float] = None
        self.fh = None
        self.folder: Optional[str] = None
        self.meta: dict = {}
        self.lock = threading.Lock()

    @property
    def recording(self) -> bool:
        return self.fh is not None

    def on_state(self, armed: bool, now: float, device: str = "", baud: int = 0) -> Optional[str]:
        """Feed every state message. Returns a log line when something changes."""
        with self.lock:
            if armed and not self.armed:
                self.armed, self.disarmed_at = True, None
                if not self.recording:
                    return self._open(now, device, baud)
                return "re-armed within the tail: still recording " + self.folder
            if not armed and self.armed:
                self.armed, self.disarmed_at = False, now
                return f"disarmed: recording {self.tail_s:.0f} s more"
            if self.recording and not self.armed and self.disarmed_at is not None \
                    and now - self.disarmed_at >= self.tail_s:
                return self._close(now)
            return None

    def on_bytes(self, t: float, data: bytes) -> None:
        with self.lock:
            if self.fh is not None and data:
                self.fh.write(RECORD.pack(t, len(data)) + data)
                self.fh.flush()  # a power cut keeps everything up to here
                self.meta["bytes"] += len(data)

    def close(self, now: float) -> Optional[str]:
        with self.lock:
            return self._close(now) if self.recording else None

    def _open(self, now: float, device: str, baud: int) -> str:
        stamp = datetime.datetime.fromtimestamp(now, datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
        self.folder = os.path.join(self.root, f"{stamp}-lidar")
        os.makedirs(self.folder, exist_ok=True)
        self.fh = open(os.path.join(self.folder, "lidar.bin"), "wb")
        self.fh.write(MAGIC)
        self.meta = {"device": device, "baud": baud, "armed_wall_time": now, "bytes": 0,
                     "format": "MAGIC then <dI (wall time, length) + raw MS200 bytes"}
        self._write_meta()
        return f"armed: recording the lidar into {self.folder}"

    def _close(self, now: float) -> str:
        self.fh.close()
        self.fh = None
        self.meta["closed_wall_time"] = now
        self._write_meta()
        return f"closed {self.folder}: {self.meta['bytes']} bytes"

    def _write_meta(self) -> None:
        with open(os.path.join(self.folder, "meta.json"), "w") as f:
            json.dump(self.meta, f, indent=1)


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("lidar_logger")
    device_param = node.declare_parameter("device", "/dev/ms200").value
    baud = int(node.declare_parameter("baud", 230400).value)
    root = node.declare_parameter("root", "/root/team_ws/flights").value
    rec = Recorder(root, float(node.declare_parameter("tail_s", 5.0).value))
    log = node.get_logger()
    port = {"dev": ""}
    stop = threading.Event()

    def reader() -> None:
        import serial  # pyserial ships with the image (MAVProxy depends on it)
        warned = False
        while not stop.is_set():
            dev = find_device(device_param)
            if dev is None:
                if not warned:
                    log.warn(f"no lidar serial port ({device_param} or {', '.join(CANDIDATES)}): retrying every 2 s")
                    warned = True
                stop.wait(2.0)
                continue
            try:
                with serial.Serial(dev, baud, timeout=0.1) as s:
                    port["dev"] = dev
                    log.info(f"lidar on {dev} at {baud} baud; recording only while armed")
                    warned = False
                    while not stop.is_set():
                        data = s.read(4096)
                        rec.on_bytes(time.time(), data)
            except Exception as exc:  # unplugged, permissions: keep the flight going, retry
                log.warn(f"lidar port {dev}: {exc}; retrying in 2 s")
                port["dev"] = ""
                stop.wait(2.0)

    def on_state(msg: String) -> None:
        try:
            armed = bool(json.loads(msg.data).get("armed", False))
        except (ValueError, AttributeError):
            return
        line = rec.on_state(armed, time.time(), port["dev"], baud)
        if line:
            log.info(line)
            if armed and not port["dev"]:
                log.warn("armed with no lidar port open: the file stays empty until it appears")

    def tick() -> None:  # closes the file after the tail even if state messages stop
        line = rec.on_state(rec.armed, time.time())
        if line:
            log.info(line)

    threading.Thread(target=reader, daemon=True).start()
    node.create_subscription(String, "/skybeta/state", on_state, 10)
    node.create_timer(0.5, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        stop.set()
        line = rec.close(time.time())
        if line:
            log.info(line)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
