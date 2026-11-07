#!/usr/bin/env python3
"""Live view of cuVSLAM tracking: the camera (or a recording) here, the engine in WSL2.

cuVSLAM runs on Linux with CUDA only, the OAK is attached to Windows.  This script captures the
stereo pair and the IMU exactly as the recorder does (host-undistorted 1280x800 GRAY8, raw BNO086
at 200 Hz), streams them to ``methods/cuvslam-vio/10_cuvslam_bridge_server.py`` (started inside WSL2
automatically) and draws what comes back: the camera image with the tracked features, a top-down
map of the trajectory, the height, and whether the tracker is tracking or the guard is skipping.

Windows laptop, repo root, venv active::

    python methods/cuvslam-vio/09_cuvslam_live_view.py                          # camera, stereo + IMU, until q
    python methods/cuvslam-vio/09_cuvslam_live_view.py --duration 60                # or stop by itself
    python methods/cuvslam-vio/09_cuvslam_live_view.py --replay recordings/euroc_20260929_181933   # a recording, no camera
    python methods/cuvslam-vio/09_cuvslam_live_view.py --replay recordings/euroc_<stamp> --speed 2
    python methods/cuvslam-vio/09_cuvslam_live_view.py --mode stereo --max-speed 2
    python methods/cuvslam-vio/09_cuvslam_live_view.py --save-tum recordings/live.tum --snapshot recordings/live.png

Defaults are the best configuration measured on this camera (README "Verification status"):
stereo-inertial (``--mode vio``), full resolution, the starvation guard at 3 m/s.  Keys in the
window: ``q`` / Esc quit, ``c`` clear the drawn trail, ``f`` features on/off.

A run lasts until ``q`` (or ``--duration`` seconds).  The camera heats up: the window shows how long
it has been on and turns that orange after five minutes.

Other setups: ``--server HOST:PORT`` connects to a server that is already running (another
machine, or WSL started by hand); ``--local-server`` runs the engine in this process (Linux with
the wheel, e.g. the Jetson with a display); ``--fake-cuvslam`` is a dry run with synthetic poses
that needs no GPU and no WSL.  ``--headless`` prints one status line per second instead of
opening a window.

Exit codes: 0 finished, 1 stream or tracking error, 2 no camera / unusable recording / bad
arguments, 3 the server could not be started or reached.
"""
from __future__ import annotations

import argparse
import collections
import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
CAMERA_DIR = REPO_ROOT / "camera"        # common.py
for _path in (str(REPO_ROOT), str(CAMERA_DIR)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import cuvslam_oak  # noqa: E402
from cuvslam_oak import bridge, dataset, imu_calib, live_view  # noqa: E402
from cuvslam_oak import rig as oakrig  # noqa: E402
from cuvslam_oak import tracking as oaktrack  # noqa: E402

EXIT_OK, EXIT_ERROR, EXIT_NO_INPUT, EXIT_NO_SERVER = 0, 1, 2, 3
WARM_AFTER_S = 300.0                     # the camera runs hot: the on-time turns orange after this
WINDOW = "cuVSLAM live view"
LEFT_SOCKET, RIGHT_SOCKET = "CAM_B", "CAM_C"


def note(message: str) -> None:
    print(message, flush=True)


def fail(message: str, code: int) -> int:
    print(message, file=sys.stderr, flush=True)
    return code


# --------------------------------------------------------------------------------------
# Server (WSL2, this process, or somebody else's)
# --------------------------------------------------------------------------------------

class WslServer:
    """``methods/cuvslam-vio/tools/run_bridge_server.sh`` inside WSL2, its output echoed with a ``[wsl]`` prefix."""

    def __init__(self, distro: str, port: int, venv: str, extra: Optional[List[str]] = None) -> None:
        self.distro, self.port, self.venv = distro, int(port), venv
        self.extra = list(extra or [])
        self.proc: Optional[subprocess.Popen] = None
        self.lines: Deque[str] = collections.deque(maxlen=200)
        self.listening = threading.Event()
        self._reader: Optional[threading.Thread] = None

    def start(self, timeout_s: float = 90.0) -> None:
        command = ["wsl.exe", "-d", self.distro, "--cd", str(REPO_ROOT), "--", "bash", "methods/cuvslam-vio/tools/run_bridge_server.sh",
                   str(self.port), self.venv] + self.extra
        try:
            self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL, text=True, encoding="utf-8", errors="replace")
        except OSError as exc:
            raise RuntimeError("cannot start wsl.exe: {}".format(exc)) from exc
        self._reader = threading.Thread(target=self._read, name="wsl-output", daemon=True)
        self._reader.start()
        deadline = time.monotonic() + timeout_s
        while not self.listening.wait(0.2):
            if self.proc.poll() is not None:
                self._reader.join(timeout=2.0)
                raise RuntimeError("the server in WSL ended at once (exit {}): {}".format(
                    self.proc.returncode, " | ".join(list(self.lines)[-3:]) or "no output"))
            if time.monotonic() > deadline:
                self.stop()
                raise RuntimeError("the server in WSL did not start within {:g} s".format(timeout_s))

    def _read(self) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        for raw in self.proc.stdout:
            line = raw.replace("\x00", "").rstrip()
            if not line or "USB protocol not available" in line:
                continue
            self.lines.append(line)
            note("[wsl] " + line)
            if bridge.LISTENING_MARKER in line:
                self.listening.set()

    def stop(self, wait_s: float = 5.0) -> None:
        if self.proc is None:
            return
        try:
            self.proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class LocalServer:
    """The server in a thread of this process (``--local-server`` / ``--fake-cuvslam``)."""

    def __init__(self, vslam: Any) -> None:
        self.vslam = vslam
        self.port = 0
        self._ready = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[str] = None

    def start(self, timeout_s: float = 10.0) -> None:
        def run() -> None:
            try:
                bridge.run_server(self.vslam, "127.0.0.1", 0, once=True, accept_timeout_s=120.0,
                                  log=lambda message: note("[server] " + message), on_listening=listening)
            except Exception as exc:  # noqa: BLE001 - reported by the client side
                self.error = "{}: {}".format(type(exc).__name__, exc)
                self._ready.set()

        def listening(port: int) -> None:
            self.port = port
            self._ready.set()

        self._thread = threading.Thread(target=run, name="bridge-server", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout_s) or self.error:
            raise RuntimeError("the local server did not start: {}".format(self.error or "timeout"))

    def stop(self, wait_s: float = 5.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=wait_s)


# --------------------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------------------

class Shared:
    """What the capture thread tells the display thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.done = threading.Event()
        self.error: Optional[str] = None
        self.status = "starting"
        self.frames = 0
        self.source_stats: Dict[str, Any] = {}


def undistort_maps(calib: Dict[str, Any], size: Tuple[int, int]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """The recorder's maps: all 14 EEPROM coefficients removed, intrinsics unchanged."""
    return {"left": oakrig.undistort_maps(calib, LEFT_SOCKET, size),
            "right": oakrig.undistort_maps(calib, RIGHT_SOCKET, size)}


def capture_camera(src: Any, maps: Dict[str, Tuple[np.ndarray, np.ndarray]], client: bridge.BridgeClient,
                   view: live_view.LiveView, shared: Shared, warmup_s: float) -> None:
    from cuvslam_oak import oak_source

    try:
        shared.status = "camera warm-up"
        src.start()
        src.warm_up(warmup_s)
        shared.status = "tracking"
        for item in src.events(timeout_s=1.0):
            if shared.stop.is_set() or not client.alive:
                break
            if item.kind == "imu":
                client.send_imu(item.ts_ns, item.gyro, item.accel)
                continue
            captured = time.monotonic()
            left = cv2.remap(item.left, maps["left"][0], maps["left"][1], cv2.INTER_LINEAR)
            right = cv2.remap(item.right, maps["right"][0], maps["right"][1], cv2.INTER_LINEAR)
            client.send_frame(item.ts_ns, item.seq, left, right)
            shared.frames += 1
            with shared.lock:
                view.add_image(item.ts_ns, left, captured)
    except oak_source.OakSourceStopped:
        pass
    except Exception as exc:  # noqa: BLE001 - one line for the user, the device is released below
        shared.error = "camera: {}: {}".format(type(exc).__name__, exc)
    finally:
        try:
            shared.source_stats = dict(src.stats() or {})
        except Exception:  # noqa: BLE001
            pass
        try:
            src.stop()                                  # releases the device
        except Exception as exc:  # noqa: BLE001
            shared.error = shared.error or "camera stop: {}".format(exc)
        shared.done.set()


def capture_replay(reader: dataset.EurocReader, mode: str, client: bridge.BridgeClient, view: live_view.LiveView,
                   shared: Shared, speed: float, max_frames: int) -> None:
    try:
        shared.status = "tracking"
        wall0: Optional[float] = None
        ts0: Optional[int] = None
        shape = reader.rig.cam0.shape if reader.rig is not None else None
        for item in reader.merged(load_images=False, with_imu=True):      # the IMU also steers coasting in stereo mode
            if shared.stop.is_set() or not client.alive:
                break
            if item.kind == "imu":
                client.send_imu(item.ts_ns, item.gyro, item.accel)
                continue
            left, right = item.images(shape)
            if speed > 0:
                if ts0 is None:
                    ts0, wall0 = item.ts_ns, time.monotonic()
                due = wall0 + (item.ts_ns - ts0) / 1e9 / speed          # type: ignore[operator]
                delay = due - time.monotonic()
                if delay > 0 and shared.stop.wait(delay):
                    break
            else:                                                       # as fast as the engine takes them, no drops
                while client.queued_frames >= client.max_queued_frames and client.alive:
                    if shared.stop.wait(0.002):
                        break
            client.send_frame(item.ts_ns, item.index, left, right)
            shared.frames += 1
            with shared.lock:
                view.add_image(item.ts_ns, left, time.monotonic())
            if max_frames and shared.frames >= max_frames:
                break
    except Exception as exc:  # noqa: BLE001
        shared.error = "replay: {}: {}".format(type(exc).__name__, exc)
    finally:
        shared.done.set()


# --------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Live view of cuVSLAM tracking (camera or recording here, engine in WSL2).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Exit codes:" + (__doc__ or "").split("Exit codes:")[-1])
    src = parser.add_argument_group("source")
    src.add_argument("--replay", type=Path, default=None, metavar="DATASET",
                     help="stream a recording made by datasets/07_record_dataset.py instead of the camera")
    src.add_argument("--speed", type=float, default=1.0,
                     help="replay pace: 1 = as recorded (default), 2 = twice as fast, 0 = as fast as the engine goes")
    src.add_argument("--device-id", default=None, metavar="ID", help="OAK device id (default: the first one found)")
    src.add_argument("--width", type=int, default=1280, help="camera frame width (default %(default)s)")
    src.add_argument("--height", type=int, default=800, help="camera frame height (default %(default)s)")
    src.add_argument("--fps", type=float, default=30.0, help="camera frame rate (default %(default)s)")
    src.add_argument("--imu-rate", type=int, default=200, metavar="HZ", help="IMU report rate (default %(default)s)")
    src.add_argument("--warmup", type=float, default=1.5, metavar="SECONDS",
                     help="camera frames discarded while the exposure settles (default %(default)s)")
    src.add_argument("--duration", type=float, default=0.0, metavar="SECONDS",
                     help="stop by itself after this long (default 0 = until q, or the end of the recording)")
    src.add_argument("--max-frames", type=int, default=0, metavar="N", help="replay: stop after N pairs (0 = all)")

    trk = parser.add_argument_group("tracking")
    trk.add_argument("--mode", default="vio", help="vio = stereo + IMU (default, the most consistent), or stereo")
    trk.add_argument("--no-guard", dest="guard", action="store_false", help="switch the starvation guard off")
    trk.add_argument("--max-speed", type=float, default=None, metavar="M_PER_S",
                     help="guard speed limit (default {:g}; raise it for a vehicle that is faster)".format(
                         oaktrack.DEFAULT_MAX_SPEED_MPS))
    trk.add_argument("--max-rot-rate", type=float, default=None, metavar="DEG_PER_S",
                     help="guard rotation limit (default {:g})".format(oaktrack.DEFAULT_MAX_ROT_DPS))
    trk.add_argument("--guard-reset-after", type=int, default=None, metavar="N",
                     help="restart the engine after N skipped frames in a row (default {})".format(
                         oaktrack.DEFAULT_GUARD_RESET_AFTER))
    trk.add_argument("--slam", action="store_true",
                     help="also run cuVSLAM's SLAM layer (loop closure); its trajectory is drawn in magenta")
    trk.add_argument("--no-coast", dest="coast", action="store_false",
                     help="no estimate while the engine gives no pose (default: coast at the last velocity, "
                          "heading from the gyro; such poses are drawn in orange)")
    trk.add_argument("--imu-calibration", default="none", metavar="PATH|auto|none",
                     help="apply the stored accelerometer / gyroscope correction (default none)")
    trk.add_argument("--async-sba", action="store_true",
                     help="cuVSLAM's asynchronous bundle adjustment (NVIDIA's live examples); default off, the "
                          "setting every measurement in the README was made with")

    srv = parser.add_argument_group("engine")
    srv.add_argument("--server", default=None, metavar="HOST:PORT", help="use a bridge server that is already running")
    srv.add_argument("--local-server", action="store_true", help="run the engine in this process (Linux with the wheel)")
    srv.add_argument("--fake-cuvslam", action="store_true", help="dry run: synthetic poses, no GPU, no WSL")
    srv.add_argument("--distro", default="Ubuntu-22.04", help="WSL distribution (default %(default)s)")
    srv.add_argument("--venv", default="~/venvs/cuvslam", help="Python environment inside WSL (default %(default)s)")
    srv.add_argument("--port", type=int, default=bridge.DEFAULT_PORT, help="TCP port (default %(default)s)")

    out = parser.add_argument_group("display and output")
    out.add_argument("--headless", action="store_true", help="no window: one status line per second")
    out.add_argument("--no-features", dest="features", action="store_false", help="do not draw the tracked features")
    out.add_argument("--save-tum", type=Path, default=None, metavar="FILE", help="write the poses as a TUM trajectory")
    out.add_argument("--snapshot", type=Path, default=None, metavar="PNG", help="write the final view as an image")
    return parser


def parse_server(text: str) -> Tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit():
        raise ValueError("--server must be HOST:PORT, got {!r}".format(text))
    return host, int(port)


def resolve_correction(choice: str, device_id: Optional[str]) -> Optional[Dict[str, Any]]:
    if not choice or choice.lower() == "none":
        return None
    if choice.lower() == "auto":
        correction = imu_calib.load_for_device(device_id)
        if correction is None:
            raise imu_calib.ImuCalibError("--imu-calibration auto: no {}".format(
                imu_calib.default_path(device_id or "unknown")))
    else:
        correction = imu_calib.ImuCorrection.load(choice)
    note("IMU correction: {}".format(correction.describe()))
    return correction.to_dict()


def status_line(view: live_view.LiveView, shared: Shared) -> str:
    positions = view.positions()
    where = "-" if not len(positions) else "({:+.2f}, {:+.2f}, up {:+.2f}) m".format(
        positions[-1][0], positions[-1][2], -positions[-1][1])
    return ("frames {:5d}  poses {:5d}  skipped {:4d}  dropped {:3d}  restarts {}  delay {} ms  position {}  "
            "travelled {:.2f} m".format(
                shared.frames, len(view.poses), len(view.skips), view.bridge["dropped"], view.stats.get("resets", 0),
                "-" if view.latency_ms is None else "{:.0f}".format(view.latency_ms), where, view.path_length()))


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        mode = oaktrack.normalize_mode(args.mode)
        remote = parse_server(args.server) if args.server else None
    except ValueError as exc:
        return fail(str(exc), EXIT_NO_INPUT)
    if args.speed < 0 or args.fps <= 0 or args.width <= 0 or args.height <= 0:
        return fail("--speed must be >= 0 and --fps, --width, --height positive", EXIT_NO_INPUT)
    replay = args.replay is not None
    duration = max(0.0, float(args.duration))

    # ---- the recording is checked before anything is started
    reader: Optional[dataset.EurocReader] = None
    calib: Optional[Dict[str, Any]] = None
    device_id = ""
    if replay:
        try:
            reader = dataset.EurocReader(args.replay)
            if reader.rig is None or reader.frame_count == 0:
                raise dataset.DatasetError("no stereo pairs")
            calib_path = reader.root / "calibration.json"
            calib = json.loads(calib_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            return fail("cannot replay {}: {}".format(args.replay, exc), EXIT_NO_INPUT)
        if mode == "vio" and reader.imu_count == 0:
            return fail("{} has no IMU samples: use --mode stereo".format(args.replay), EXIT_NO_INPUT)
        if not reader.cam0.undistorted:
            return fail("{} was recorded with --raw (distorted frames); the bridge expects undistorted ones".format(
                args.replay), EXIT_NO_INPUT)
        size = (int(reader.cam0.width), int(reader.cam0.height))
        fps, imu_rate = float(reader.cam0.rate_hz), float(reader.imu.rate_hz)
        device_id = reader.rig.device_id or ""
        note("Source: recording {} ({} pairs {}x{}, {:.1f} s, pace x{:g})".format(
            reader.root.name, reader.frame_count, size[0], size[1], reader.duration_s, args.speed))
    else:
        size = (args.width, args.height)
        fps, imu_rate = float(args.fps), float(args.imu_rate)

    # ---- engine
    server: Any = None
    host, port = "127.0.0.1", args.port
    try:
        if remote is not None:
            host, port = remote
        elif args.local_server or args.fake_cuvslam:
            try:
                vslam = cuvslam_oak.import_cuvslam(prefer_fake=bool(args.fake_cuvslam))
            except ImportError as exc:
                return fail("cuvslam is not importable here: {}".format(str(exc).splitlines()[0]), EXIT_NO_SERVER)
            if cuvslam_oak.is_fake(vslam):
                note("FAKE cuVSLAM in use: the poses are synthetic, NOT tracking results")
            server = LocalServer(vslam)
            server.start()
            port = server.port
        else:
            note("Starting the engine in WSL ({})...".format(args.distro))
            server = WslServer(args.distro, args.port, args.venv)
            server.start()
    except RuntimeError as exc:
        return fail(str(exc), EXIT_NO_SERVER)

    src: Any = None
    client: Optional[bridge.BridgeClient] = None
    shared = Shared()
    thread: Optional[threading.Thread] = None
    code = EXIT_OK
    view: Optional[live_view.LiveView] = None
    window_open = False
    camera_opened = False
    try:
        # ---- camera (opened only now: the engine side is known to be up)
        maps = None
        if not replay:
            import common
            from cuvslam_oak import oak_source

            device = common.open_device(args.device_id, retries=1 if args.device_id else 3)   # exits 2 on failure
            camera_opened = True
            try:
                info = oak_source.device_info(device)
                device_id = str(info.get("device_id") or "")
                src = oak_source.OakStereoImuSource(device, size[0], size[1], fps, int(imu_rate), 1, True)
                src.build()
                calib = src.calibration()
                maps = undistort_maps(calib, size)
            except Exception as exc:  # noqa: BLE001
                if src is not None:
                    src.stop()
                else:
                    device.close()
                src = None
                return fail("camera: {}: {}".format(type(exc).__name__, exc), EXIT_ERROR)
            note("Source: camera {} at {}x{} @ {:g} fps, IMU {:g} Hz, host-undistorted".format(
                device_id, size[0], size[1], fps, imu_rate))

        try:
            correction = resolve_correction(args.imu_calibration, device_id or None) if mode == "vio" else None
        except ValueError as exc:
            return fail(str(exc), EXIT_NO_INPUT)
        options = {"guard": bool(args.guard), "max_speed_mps": args.max_speed, "max_rot_dps": args.max_rot_rate,
                   "guard_reset_after": args.guard_reset_after, "coast": bool(args.coast), "slam": bool(args.slam)}
        hello = bridge.make_hello(mode=mode, size=size, fps=fps, imu_rate_hz=imu_rate, calibration=calib or {},
                                  device_id=device_id, source="recording" if replay else "camera",
                                  features=bool(args.features), imu_correction=correction,
                                  async_sba=bool(args.async_sba), options=options)
        client = bridge.BridgeClient(host, port, hello)
        note("Connecting to the engine at {}:{} (its start takes a few seconds)...".format(host, port))
        try:
            ready = client.connect()
        except (OSError, bridge.BridgeError) as exc:
            return fail("engine: {}".format(exc), EXIT_NO_SERVER)
        cfg = ready.get("config") or {}
        note("Engine ready: cuvslam {}{}, {} mode, guard {} (max {} m/s, {} deg/s, restart after {} skipped)".format(
            ready.get("cuvslam_version"), " (FAKE)" if ready.get("fake") else "", ready.get("odometry_mode") or mode,
            "on" if cfg.get("guard") else "off", cfg.get("max_speed_mps"), cfg.get("max_rot_dps"),
            cfg.get("guard_reset_after")))

        view = live_view.LiveView(mode, size, title=("recording " + reader.root.name) if reader is not None
                                  else "camera " + device_id, duration_s=duration if duration > 0 else None,
                                  show_features=bool(args.features), warm_after_s=None if replay else WARM_AFTER_S)
        if replay:
            assert reader is not None
            thread = threading.Thread(target=capture_replay, name="replay", daemon=True,
                                      args=(reader, mode, client, view, shared, float(args.speed), int(args.max_frames)))
        else:
            thread = threading.Thread(target=capture_camera, name="capture", daemon=True,
                                      args=(src, maps, client, view, shared, float(args.warmup)))
        thread.start()
        if not args.headless:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, view.canvas[0], view.canvas[1])
            window_open = True
        note("Running{}: {}".format(
            "" if duration <= 0 else " for {:g} s".format(duration),
            "Ctrl-C stops" if args.headless else "q or Esc in the window stops"))

        ended_sent = False
        last_print = time.monotonic()
        stop_reason = "end of data"
        while True:
            events = client.poll()
            with shared.lock:
                view.status = shared.status
                view.handle(events)
                view.set_bridge(client.frames_sent, client.frames_dropped, client.queued_frames)
                picture = None if args.headless else view.render()
            if picture is not None:
                cv2.imshow(WINDOW, picture)
                key = cv2.waitKey(15) & 0xFF
                if key in (ord("q"), 27):
                    stop_reason = "q"
                    shared.stop.set()
                elif key == ord("c"):
                    with shared.lock:
                        view.clear_trail()
                elif key == ord("f"):
                    view.show_features = not view.show_features
                try:
                    if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                        stop_reason = "window closed"
                        shared.stop.set()
                except cv2.error:
                    pass
            else:
                time.sleep(0.02)
            now = time.monotonic()
            if args.headless and now - last_print >= 1.0:
                last_print = now
                note(status_line(view, shared))
            remaining = view.remaining_s()
            if remaining is not None and remaining <= 0 and not shared.stop.is_set():
                stop_reason = "duration"
                shared.stop.set()
            if client.error and not shared.stop.is_set():
                stop_reason = "engine"
                shared.stop.set()
            if (shared.done.is_set() or shared.stop.is_set()) and not ended_sent:
                if shared.stop.is_set():
                    shared.done.wait(3.0)               # let the capture thread release the camera
                client.end()
                ended_sent = True
                ended_at = now
            if ended_sent and (view.summary is not None or view.error or client.error or now - ended_at > 15.0):
                break

        if shared.error:
            code = fail(shared.error, EXIT_ERROR)
        if client.error and view.summary is None:
            code = fail("engine: {}".format(client.error), EXIT_ERROR)
        note("Stopped by: {}".format(stop_reason))
        note(status_line(view, shared))
        summary = view.summary or {}
        if summary:
            note("Engine summary: tracked {} of {} frames, guard skipped {} (degenerate {}, jump {}, spin {}), "
                 "{} coasted, {} engine restart(s), path {:.2f} m, end-point offset {:.2f} m".format(
                     summary.get("frames_tracked"), summary.get("frames_attempted"), summary.get("frames_guarded"),
                     summary.get("guard_degenerate"), summary.get("guard_jump"), summary.get("guard_spin"),
                     summary.get("frames_coasted"), summary.get("resets"), float(summary.get("path_length_m") or 0.0),
                     float(summary.get("endpoint_offset_m") or 0.0)))
            slam = summary.get("slam")
            if slam:
                note("SLAM summary: {} loop closure(s), {} map reset(s), end-point offset {} m as tracked, {} m "
                     "re-optimised".format(slam.get("loop_closures"), slam.get("map_resets"),
                                           slam.get("endpoint_offset_m"), slam.get("optimized_endpoint_offset_m")))
        if shared.source_stats:
            note("Camera: frames {}, sync rejections {}, IMU samples {}".format(
                shared.source_stats.get("frames"), shared.source_stats.get("stereo_sync_violations"),
                shared.source_stats.get("imu_samples")))
        if args.save_tum is not None:
            rows = view.all_poses()
            flags = view.all_coasted()
            offset = getattr(src, "epoch_offset_ns", None) if src is not None else None
            if offset:
                rows = [(int(r[0]) + int(offset),) + tuple(r[1:]) for r in rows]
            path = oaktrack.write_tum(args.save_tum, rows, header=[
                "live view, mode {}, {}".format(mode, "recording " + reader.root.name if reader is not None
                                                else "camera " + device_id),
                "{} of {} rows are coasted estimates (no vision)".format(sum(1 for f in flags if f), len(rows))])
            note("TUM: {} ({} rows)".format(path, len(rows)))
        if args.snapshot is not None:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            with shared.lock:
                cv2.imwrite(str(args.snapshot), view.render())
            note("Snapshot: {}".format(args.snapshot))
        return code
    except KeyboardInterrupt:
        note("\nInterrupted.")
        return code
    finally:
        shared.stop.set()
        if thread is not None:
            thread.join(timeout=5.0)
        elif src is not None:
            try:
                src.stop()
            except Exception:  # noqa: BLE001
                pass
        if client is not None:
            client.close()
        if window_open:
            cv2.destroyAllWindows()
        if server is not None:
            server.stop()
        if camera_opened:
            note("Camera released: safe to unplug.")


if __name__ == "__main__":
    sys.exit(main())
