"""Frame bridge: the camera on one machine, cuVSLAM on another, poses back for a live view.

cuVSLAM needs Linux and CUDA; on the laptop the OAK is attached to Windows.  Instead of tunnelling
USB into WSL2, the Windows side captures (or replays a recording), undistorts the frames exactly
as the recorder does and streams them over TCP to a small server in WSL2 (or on any Linux box with
the wheel), which runs the ordinary tracking loop (:func:`cuvslam_oak.tracking.run_tracking`, same
rig, same guard) and sends every pose back.  ``localhost`` on Windows reaches a port opened inside
WSL2; measured 2026-09-29: 148 raw 1280x800 stereo pairs per second through that path.

Wire format: every message is a 16-byte header (``b"OAKB"``, type, payload length) plus payload.

====== ========= ============================================================================
type   direction payload
====== ========= ============================================================================
HELLO  c -> s    JSON: protocol, mode, size, fps, imu_rate_hz, calibration (EEPROM dict), ...
READY  s -> c    JSON: cuvslam version, the rig as built, the tracking configuration
IMU    c -> s    N records of ``<q6d``: ts_ns, gyro xyz (rad/s), accel xyz (m/s^2)
FRAME  c -> s    ``<qIHH`` ts_ns, seq, width, height, then the left and the right GRAY8 image
POSE   s -> c    JSON: ts, index, p (xyz, m), q (xyzw), ms, g (gravity, rig frame), f (features),
                 coast (true for an estimate made while the engine gave no pose), s / sq (the SLAM
                 pose when the client asked for SLAM)
SKIP   s -> c    JSON: ts of a frame that produced no pose (guard, loss, non-finite)
STATS  s -> c    JSON: the ``on_stats`` dict of the tracking loop
END    c -> s    empty: no more data
SUMMARY s -> c   JSON: the summary of the run (without the row list)
ERROR  s -> c    JSON: message
====== ========= ============================================================================

Ordering is the tracker's: every IMU record with ``ts <= frame ts`` travels before that frame.
The client never blocks the camera: frames wait in a short queue and the oldest *frame* is dropped
when the server falls behind; IMU records are never dropped.

Only the standard library and numpy are imported here; ``cuvslam`` comes in through the *vslam*
argument of :func:`serve_connection`.
"""
from __future__ import annotations

import collections
import json
import queue
import socket
import struct
import threading
import time
from typing import Any, Callable, Deque, Dict, Iterator, List, Optional, Tuple

import numpy as np

from . import imu_calib as _imu_calib
from . import rig as _rig
from . import tracking as _tracking

__all__ = [
    "PROTOCOL_VERSION",
    "DEFAULT_PORT",
    "LISTENING_MARKER",
    "MSG_HELLO", "MSG_READY", "MSG_IMU", "MSG_FRAME", "MSG_POSE", "MSG_SKIP", "MSG_STATS", "MSG_END",
    "MSG_SUMMARY", "MSG_ERROR",
    "BridgeError",
    "send_json", "send_imu_records", "send_frame", "recv_message",
    "make_hello",
    "serve_connection", "run_server",
    "BridgeClient",
]

PROTOCOL_VERSION = 1
DEFAULT_PORT = 47800
#: Printed (and flushed) by the server once it accepts connections; launchers wait for it.
LISTENING_MARKER = "BRIDGE LISTENING"

MSG_HELLO, MSG_READY, MSG_IMU, MSG_FRAME, MSG_POSE = 1, 2, 3, 4, 5
MSG_SKIP, MSG_STATS, MSG_END, MSG_SUMMARY, MSG_ERROR = 6, 7, 8, 9, 10
_NAMES = {MSG_HELLO: "HELLO", MSG_READY: "READY", MSG_IMU: "IMU", MSG_FRAME: "FRAME", MSG_POSE: "POSE",
          MSG_SKIP: "SKIP", MSG_STATS: "STATS", MSG_END: "END", MSG_SUMMARY: "SUMMARY", MSG_ERROR: "ERROR"}

_MAGIC = b"OAKB"
_HEADER = struct.Struct("<4sBxxxQ")
_FRAME_HEAD = struct.Struct("<qIHH")
_IMU_RECORD = struct.Struct("<q6d")
_MAX_JSON = 64 * 1024 * 1024
_MAX_IMAGE_SIDE = 4096


class BridgeError(RuntimeError):
    """Protocol violation, closed connection or an error reported by the other side."""


# --------------------------------------------------------------------------------------
# Wire helpers
# --------------------------------------------------------------------------------------

def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (set, tuple)):
        return list(value)
    return str(value)


def send_json(sock: socket.socket, mtype: int, obj: Any) -> None:
    payload = json.dumps(obj, default=_json_default, separators=(",", ":")).encode("utf-8")
    sock.sendall(_HEADER.pack(_MAGIC, mtype, len(payload)) + payload)


def send_imu_records(sock: socket.socket, packed: bytes) -> None:
    """*packed* is a concatenation of ``<q6d`` records (see :func:`pack_imu`)."""
    if len(packed) % _IMU_RECORD.size:
        raise BridgeError("IMU payload of {} bytes is not a multiple of {}".format(len(packed), _IMU_RECORD.size))
    sock.sendall(_HEADER.pack(_MAGIC, MSG_IMU, len(packed)) + packed)


def pack_imu(ts_ns: int, gyro: Any, accel: Any) -> bytes:
    return _IMU_RECORD.pack(int(ts_ns), float(gyro[0]), float(gyro[1]), float(gyro[2]),
                            float(accel[0]), float(accel[1]), float(accel[2]))


def send_frame(sock: socket.socket, ts_ns: int, seq: int, left: np.ndarray, right: np.ndarray) -> None:
    if left.dtype != np.uint8 or left.ndim != 2 or right.shape != left.shape or right.dtype != np.uint8:
        raise BridgeError("frames must be two uint8 (height, width) arrays of one shape, got {} {} and {} {}".format(
            left.dtype, left.shape, right.dtype, right.shape))
    h, w = left.shape
    head = _FRAME_HEAD.pack(int(ts_ns), int(seq) & 0xFFFFFFFF, w, h)
    sock.sendall(_HEADER.pack(_MAGIC, MSG_FRAME, len(head) + 2 * w * h) + head)
    sock.sendall(memoryview(np.ascontiguousarray(left)).cast("B"))
    sock.sendall(memoryview(np.ascontiguousarray(right)).cast("B"))


def _recv_into(sock: socket.socket, view: memoryview) -> None:
    got = 0
    while got < len(view):
        n = sock.recv_into(view[got:])
        if n == 0:
            raise BridgeError("connection closed by the other side")
        got += n


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray(n)
    _recv_into(sock, memoryview(buf))
    return bytes(buf)


def recv_message(sock: socket.socket) -> Tuple[int, Any]:
    """Next ``(type, value)``: a dict for JSON messages, ``(ts_ns, seq, left, right)`` for a frame,
    a list of ``(ts_ns, gyro, accel)`` for IMU, ``None`` for END.  Raises :class:`BridgeError`
    when the connection closes or the stream is not this protocol."""
    magic, mtype, length = _HEADER.unpack(_recv_exact(sock, _HEADER.size))
    if magic != _MAGIC:
        raise BridgeError("not a bridge stream (bad magic {!r})".format(magic))
    if mtype == MSG_FRAME:
        ts, seq, w, h = _FRAME_HEAD.unpack(_recv_exact(sock, _FRAME_HEAD.size))
        if not (0 < w <= _MAX_IMAGE_SIDE and 0 < h <= _MAX_IMAGE_SIDE) or length != _FRAME_HEAD.size + 2 * w * h:
            raise BridgeError("bad frame header: {}x{} with {} payload bytes".format(w, h, length))
        left = np.empty((h, w), dtype=np.uint8)
        right = np.empty((h, w), dtype=np.uint8)
        _recv_into(sock, memoryview(left).cast("B"))
        _recv_into(sock, memoryview(right).cast("B"))
        return mtype, (ts, seq, left, right)
    if mtype == MSG_IMU:
        if length % _IMU_RECORD.size or length > _MAX_JSON:
            raise BridgeError("bad IMU payload length {}".format(length))
        data = _recv_exact(sock, length)
        records = []
        for values in _IMU_RECORD.iter_unpack(data):
            records.append((values[0], values[1:4], values[4:7]))
        return mtype, records
    if mtype == MSG_END:
        if length:                                     # never read it: a peer could ask for any size
            raise BridgeError("END carries no payload (got a length of {})".format(length))
        return mtype, None
    if mtype not in _NAMES:
        raise BridgeError("unknown message type {}".format(mtype))
    if length > _MAX_JSON:
        raise BridgeError("{} payload of {} bytes is too large".format(_NAMES[mtype], length))
    try:
        return mtype, json.loads(_recv_exact(sock, length).decode("utf-8")) if length else {}
    except ValueError as exc:
        raise BridgeError("{} payload is not JSON: {}".format(_NAMES[mtype], exc)) from exc


def make_hello(*, mode: str, size: Tuple[int, int], fps: float, imu_rate_hz: float, calibration: Dict[str, Any],
               device_id: str = "", source: str = "camera", rig_socket: str = "CAM_B", features: bool = True,
               imu_correction: Optional[Dict[str, Any]] = None, async_sba: bool = False,
               options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The HELLO of a client.  *options* are :class:`cuvslam_oak.tracking.TrackConfig` fields
    (``guard``, ``max_speed_mps``, ``max_rot_dps``, ``guard_reset_after``, ``multicam_mode``, ...);
    the frames are host-undistorted, so the server builds a pinhole rig."""
    return {
        "protocol": PROTOCOL_VERSION, "mode": _tracking.normalize_mode(mode), "size": [int(size[0]), int(size[1])],
        "fps": float(fps), "imu_rate_hz": float(imu_rate_hz), "calibration": calibration,
        "device_id": str(device_id or ""), "source": source, "rig_socket": rig_socket, "undistorted": True,
        "features": bool(features), "imu_correction": imu_correction, "async_sba": bool(async_sba),
        "options": dict(options or {}),
    }


# --------------------------------------------------------------------------------------
# Server
# --------------------------------------------------------------------------------------

#: TrackConfig fields a client may set.
_CLIENT_OPTIONS = ("guard", "min_observations", "guard_zero_covariance", "max_speed_mps", "max_rot_dps",
                   "guard_reset_after", "multicam_mode", "use_denoising", "use_motion_model", "stitch_resets",
                   "coast", "coast_velocity_window_s", "coast_decay_s", "coast_max_s", "coast_gyro",
                   "coast_keep_engine_rotation", "slam", "slam_max_map_size", "slam_planar")


class _SocketStream:
    """The merged IMU + frame stream of one client, in the form the tracking loop reads.

    A frame that was handed to the loop and produced no pose by the time the loop asks for the
    next item is reported to the client as SKIP.
    """

    def __init__(self, conn: socket.socket, size: Tuple[int, int]) -> None:
        self.conn = conn
        self.size = size
        self.pending: Deque[Any] = collections.deque()
        self.awaiting_ts: Optional[int] = None
        self.last_posed_ts: Optional[int] = None
        self.frames = 0
        self.imu = 0
        self.ended = False
        self.closed_early = False

    def __iter__(self) -> Iterator[Any]:
        return self

    def report_skip(self) -> None:
        if self.awaiting_ts is not None:
            if self.awaiting_ts != self.last_posed_ts:
                send_json(self.conn, MSG_SKIP, {"ts": self.awaiting_ts})
            self.awaiting_ts = None

    def __next__(self) -> Any:
        self.report_skip()
        while not self.pending:
            if self.ended:
                raise StopIteration
            try:
                mtype, value = recv_message(self.conn)
            except (BridgeError, OSError):
                self.ended = self.closed_early = True
                raise StopIteration
            if mtype == MSG_IMU:
                self.imu += len(value)
                self.pending.extend(("imu", ts, gyro, accel) for ts, gyro, accel in value)
            elif mtype == MSG_FRAME:
                ts, _seq, left, right = value
                if (left.shape[1], left.shape[0]) != self.size:
                    raise _tracking.TrackingError("frame of {}x{} does not match the announced {}x{}".format(
                        left.shape[1], left.shape[0], *self.size))
                self.frames += 1
                self.pending.append(("stereo", ts, [left, right]))
            elif mtype == MSG_END:
                self.ended = True
            else:
                raise _tracking.TrackingError("unexpected {} message while tracking".format(_NAMES.get(mtype, mtype)))
        item = self.pending.popleft()
        if item[0] == "stereo":
            self.awaiting_ts = int(item[1])
        return item


def _features(tracker: Any, limit: int = 600) -> List[int]:
    """Flat ``[u0, v0, u1, v1, ...]`` (rounded pixels) of the camera-0 observations, [] when unavailable."""
    try:
        observations = tracker.get_last_observations(0)
    except Exception:  # noqa: BLE001 - optional overlay only
        return []
    flat: List[int] = []
    for obs in list(observations)[:limit]:
        flat.append(int(round(float(obs.u))))
        flat.append(int(round(float(obs.v))))
    return flat


def serve_connection(conn: socket.socket, vslam: Any, log: Callable[[str], None] = print) -> Dict[str, Any]:
    """Serve one client on the connected socket *conn*; returns the run summary.

    Every failure is reported to the client as ERROR before it is raised.
    """
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        mtype, hello = recv_message(conn)
        if mtype != MSG_HELLO:
            raise BridgeError("expected HELLO, got {}".format(_NAMES.get(mtype, mtype)))
        if int(hello.get("protocol", -1)) != PROTOCOL_VERSION:
            raise BridgeError("protocol {} is not supported (this server speaks {})".format(
                hello.get("protocol"), PROTOCOL_VERSION))
        mode = _tracking.normalize_mode(str(hello["mode"]))
        size = (int(hello["size"][0]), int(hello["size"][1]))
        rig, info = _rig.build_rig(hello["calibration"], size, mode, imu_rate_hz=float(hello.get("imu_rate_hz", 200.0)),
                                   vslam=vslam, rig_socket=str(hello.get("rig_socket") or "CAM_B"),
                                   undistorted=bool(hello.get("undistorted", True)),
                                   # the preset the recorder writes into imu0/sensor.yaml, so a live run and
                                   # a run on a recording see the same rig (every measured figure used it)
                                   noise_preset=str(hello.get("noise_preset") or _rig.EUROC_NOISE_PRESET))
        options = {k: v for k, v in dict(hello.get("options") or {}).items() if k in _CLIENT_OPTIONS and v is not None}
        correction = hello.get("imu_correction")
        imu_block = info.get("imu") or {}
        cfg = _tracking.TrackConfig(
            realtime=bool(hello.get("async_sba", False)), warmup_frames=0, verbose=False, stats_interval_s=0.5,
            imu_correction=_imu_calib.ImuCorrection.from_dict(correction) if correction else None,
            coast_rig_from_imu=imu_block.get("rig_from_imu"),       # gyro coasting in stereo mode too
            log=lambda message: log("[track] " + message), **options)
        log("client: {} {}x{} @ {:g} fps, mode {}, device {}".format(
            hello.get("source", "?"), size[0], size[1], float(hello.get("fps", 0)), mode, hello.get("device_id") or "?"))
        tracker = _tracking.make_tracker(vslam, rig, mode, cfg)
        version = getattr(vslam, "__version__", "?")
        send_json(conn, MSG_READY, {
            "cuvslam_version": str(version), "fake": bool(_tracking._is_fake(vslam)), "mode": mode,
            "odometry_mode": info.get("odometry_mode"), "baseline_m": info.get("baseline_m"),
            "config": cfg.to_dict(),
        })
        log("engine ready (cuvslam {}), tracking".format(version))

        stream = _SocketStream(conn, size)
        want_features = bool(hello.get("features", True))

        def on_pose(event: Any) -> None:
            stream.last_posed_ts = int(event.ts_ns)
            message = {"ts": int(event.ts_ns), "i": int(event.index),
                       "p": [float(v) for v in event.translation], "q": [float(v) for v in event.rotation_xyzw],
                       "ms": round(float(event.track_ms), 3), "seg": int(event.segment),
                       "g": None if event.gravity is None else [float(v) for v in event.gravity],
                       "coast": bool(event.coasted)}
            if event.slam_translation is not None and event.slam_rotation_xyzw is not None:
                message["s"] = [float(v) for v in event.slam_translation]
                message["sq"] = [float(v) for v in event.slam_rotation_xyzw]
            if want_features and not event.coasted:
                message["f"] = _features(event.tracker)
            send_json(conn, MSG_POSE, message)

        def on_stats(stats: Dict[str, Any]) -> None:
            send_json(conn, MSG_STATS, stats)

        summary = _tracking.run_tracking(vslam, rig, mode, stream, None, cfg, on_pose=on_pose, on_stats=on_stats,
                                         tracker=tracker)
        if not stream.closed_early:
            stream.report_skip()
        summary = {k: v for k, v in summary.items() if k not in ("rows", "slam_rows", "slam_optimized_rows")}
        summary["bridge"] = {"frames_received": stream.frames, "imu_received": stream.imu,
                             "client_closed_early": stream.closed_early}
        if not stream.closed_early:
            send_json(conn, MSG_SUMMARY, summary)
        log("done: {} frame(s) received, {} tracked, {} guarded, {} coasted, {} reset(s)".format(
            stream.frames, summary.get("frames_tracked"), summary.get("frames_guarded"),
            summary.get("frames_coasted"), summary.get("resets")))
        return summary
    except Exception as exc:  # noqa: BLE001 - tell the client, then let the caller decide
        try:
            send_json(conn, MSG_ERROR, {"message": "{}: {}".format(type(exc).__name__, exc)})
        except OSError:
            pass
        raise


def run_server(vslam: Any, host: str = "0.0.0.0", port: int = DEFAULT_PORT, *, once: bool = True,
               log: Callable[[str], None] = print, accept_timeout_s: Optional[float] = None,
               on_listening: Optional[Callable[[int], None]] = None) -> int:
    """Listen on *host*:*port* and serve clients one at a time; returns the number served.

    ``port=0`` picks a free port (reported through *on_listening* and the log line).  With *once*
    the server returns after its first client; *accept_timeout_s* bounds the wait for a client.
    """
    served = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, int(port)))
        listener.listen(1)
        actual = listener.getsockname()[1]
        listener.settimeout(accept_timeout_s)
        log("{} on {}:{}".format(LISTENING_MARKER, host, actual))
        if on_listening is not None:
            on_listening(actual)
        while True:
            try:
                conn, address = listener.accept()
            except socket.timeout:
                log("no client within {:g} s".format(accept_timeout_s or 0.0))
                return served
            log("connection from {}".format(address[0]))
            with conn:
                conn.settimeout(None)
                try:
                    serve_connection(conn, vslam, log)
                except Exception as exc:  # noqa: BLE001 - one bad client must not kill a long-lived server
                    log("client failed: {}: {}".format(type(exc).__name__, exc))
            served += 1
            if once:
                return served


# --------------------------------------------------------------------------------------
# Client
# --------------------------------------------------------------------------------------

class BridgeClient:
    """Client side: a sender thread with a short frame queue and a receiver thread.

    ``connect()`` sends HELLO and waits for READY (the engine start takes a few seconds).  Then
    ``send_imu`` / ``send_frame`` from the capture thread, ``poll()`` from the display thread,
    ``end()`` when the source is exhausted, ``close()`` always.  ``send_frame`` never blocks: when
    more than *max_queued_frames* wait, the oldest waiting frame is dropped (``frames_dropped``).
    The default queue holds one second at 30 fps (about 60 MB at 1280x800): the engine's one slow
    frame at start-up (0.3 to 0.5 s in ``vio`` mode) is then absorbed instead of costing 11 frames,
    as measured with a queue of 4; the engine needs 5 to 9 ms per frame, so it catches up at once.
    """

    def __init__(self, host: str, port: int, hello: Dict[str, Any], *, max_queued_frames: int = 30,
                 connect_timeout_s: float = 10.0, ready_timeout_s: float = 90.0) -> None:
        self.host, self.port = host, int(port)
        self.hello = hello
        self.max_queued_frames = max(1, int(max_queued_frames))
        self.connect_timeout_s = float(connect_timeout_s)
        self.ready_timeout_s = float(ready_timeout_s)
        self.ready: Dict[str, Any] = {}
        self.sock: Optional[socket.socket] = None
        self.frames_sent = 0
        self.frames_dropped = 0
        self.imu_sent = 0
        self.error: Optional[str] = None
        self.summary: Optional[Dict[str, Any]] = None
        self._imu_pending = bytearray()
        self._queue: Deque[Tuple[Any, ...]] = collections.deque()
        self._cond = threading.Condition()
        self._closing = False
        self._events: "queue.Queue[Tuple[int, Any]]" = queue.Queue()
        self._sender: Optional[threading.Thread] = None
        self._receiver: Optional[threading.Thread] = None
        self._finished = threading.Event()

    # ---------------------------------------------------------------- lifecycle
    def connect(self) -> Dict[str, Any]:
        sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = sock
        try:
            send_json(sock, MSG_HELLO, self.hello)
            sock.settimeout(self.ready_timeout_s)
            mtype, value = recv_message(sock)
        except socket.timeout as exc:
            self.close()
            raise BridgeError("the server did not answer within {:g} s".format(self.ready_timeout_s)) from exc
        except (OSError, BridgeError):
            self.close()
            raise
        if mtype == MSG_ERROR:
            self.close()
            raise BridgeError("server: {}".format(value.get("message", "unknown error")))
        if mtype != MSG_READY:
            self.close()
            raise BridgeError("expected READY, got {}".format(_NAMES.get(mtype, mtype)))
        sock.settimeout(None)
        self.ready = value
        self._sender = threading.Thread(target=self._send_loop, name="bridge-sender", daemon=True)
        self._receiver = threading.Thread(target=self._receive_loop, name="bridge-receiver", daemon=True)
        self._sender.start()
        self._receiver.start()
        return value

    def end(self) -> None:
        """No more data: flush, send END (the server answers with SUMMARY)."""
        with self._cond:
            self._flush_imu_locked()
            self._queue.append(("end",))
            self._cond.notify()

    def wait_finished(self, timeout_s: float = 10.0) -> bool:
        """Wait for SUMMARY / ERROR / a closed connection after :meth:`end`."""
        return self._finished.wait(timeout_s)

    def close(self) -> None:
        with self._cond:
            self._closing = True
            self._cond.notify_all()
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        for thread in (self._sender, self._receiver):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2.0)

    def __enter__(self) -> "BridgeClient":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()

    # ---------------------------------------------------------------- input
    @property
    def alive(self) -> bool:
        return self.sock is not None and self.error is None and not self._finished.is_set()

    @property
    def queued_frames(self) -> int:
        with self._cond:
            return sum(1 for item in self._queue if item[0] == "frame")

    def send_imu(self, ts_ns: int, gyro: Any, accel: Any) -> None:
        with self._cond:
            self._imu_pending += pack_imu(ts_ns, gyro, accel)
            if len(self._imu_pending) >= 64 * _IMU_RECORD.size:
                self._flush_imu_locked()
                self._cond.notify()

    def _flush_imu_locked(self) -> None:
        if self._imu_pending:
            self.imu_sent += len(self._imu_pending) // _IMU_RECORD.size
            self._queue.append(("imu", bytes(self._imu_pending)))
            self._imu_pending = bytearray()

    def send_frame(self, ts_ns: int, seq: int, left: np.ndarray, right: np.ndarray) -> bool:
        """Queue one pair; ``False`` when an older waiting pair had to be dropped to make room."""
        dropped = False
        with self._cond:
            self._flush_imu_locked()
            waiting = [i for i, item in enumerate(self._queue) if item[0] == "frame"]
            if len(waiting) >= self.max_queued_frames:
                del self._queue[waiting[0]]
                self.frames_dropped += 1
                dropped = True
            self._queue.append(("frame", int(ts_ns), int(seq), left, right))
            self._cond.notify()
        return not dropped

    def poll(self, limit: int = 1000) -> List[Tuple[int, Any]]:
        """Messages received since the last call: ``(MSG_POSE | MSG_SKIP | MSG_STATS | ..., dict)``."""
        out: List[Tuple[int, Any]] = []
        try:
            while len(out) < limit:
                out.append(self._events.get_nowait())
        except queue.Empty:
            pass
        return out

    # ---------------------------------------------------------------- threads
    def _send_loop(self) -> None:
        while True:
            with self._cond:
                while not self._queue and not self._closing:
                    self._cond.wait(0.5)
                if self._closing and not self._queue:
                    return
                item = self._queue.popleft()
            sock = self.sock
            if sock is None:
                return
            try:
                if item[0] == "imu":
                    send_imu_records(sock, item[1])
                elif item[0] == "frame":
                    send_frame(sock, item[1], item[2], item[3], item[4])
                    self.frames_sent += 1
                else:
                    sock.sendall(_HEADER.pack(_MAGIC, MSG_END, 0))
                    return
            except (OSError, BridgeError) as exc:
                if not self._closing and self.error is None:
                    self.error = "send failed: {}".format(exc)
                self._finished.set()
                return

    def _receive_loop(self) -> None:
        while True:
            sock = self.sock
            if sock is None:
                break
            try:
                mtype, value = recv_message(sock)
            except (OSError, BridgeError) as exc:
                if not self._closing and self.summary is None and self.error is None:
                    self.error = "connection lost: {}".format(exc)
                break
            if mtype == MSG_ERROR:
                self.error = "server: {}".format(value.get("message", "unknown error"))
                self._events.put((mtype, value))
                break
            if mtype == MSG_SUMMARY:
                self.summary = value
                self._events.put((mtype, value))
                break
            value["_received"] = time.monotonic()
            self._events.put((mtype, value))
        self._finished.set()
