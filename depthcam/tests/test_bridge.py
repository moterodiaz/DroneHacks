"""Frame bridge and live view: wire format, server + client against the fake engine, the drawing,
and the two scripts.  No camera, no GPU, no WSL, no window.
"""
from __future__ import annotations

import math
import os
import socket
import subprocess
import sys
import threading
import time
import types
import warnings
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest

import cuvslam_oak
from cuvslam_oak import bridge, live_view, rig, tracking

from test_guard import (ACCEL, FRAME_PERIOD_NS, GYRO, IMU_PERIOD_NS, SHAPE, SIZE, T0, walk,  # noqa: E402
                        write_tiny_dataset)

REPO_ROOT = Path(__file__).resolve().parents[1]
VIEW_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "09_cuvslam_live_view.py"
SERVER_SCRIPT = REPO_ROOT / "methods" / "cuvslam-vio" / "10_cuvslam_bridge_server.py"
CALIBRATION = REPO_ROOT / "tests" / "data" / "calibration_OAKDS2EXAMPLEID000.json"
TIMEOUT_S = 90.0


@pytest.fixture(scope="module")
def vslam() -> types.ModuleType:
    module = cuvslam_oak.import_cuvslam(prefer_fake=True)
    assert cuvslam_oak.is_fake(module)
    return module


@pytest.fixture(scope="module")
def calib() -> Dict[str, Any]:
    if not CALIBRATION.is_file():
        pytest.skip("the vendored calibration is missing")
    return rig.load_calibration_json(CALIBRATION)


@pytest.fixture(autouse=True)
def clean_fake_sources(vslam: types.ModuleType) -> Any:
    vslam.set_default_pose_source(None)
    yield
    vslam.set_default_pose_source(None)


def run_cli(script: Path, *args: Any) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.pop(cuvslam_oak.FAKE_ENV_VAR, None)
    env.setdefault("PYTHONIOENCODING", "utf-8")
    return subprocess.run([sys.executable, str(script), *(str(a) for a in args)], cwd=str(REPO_ROOT),
                          capture_output=True, text=True, encoding="utf-8", errors="replace", env=env,
                          timeout=TIMEOUT_S, check=False)


class Server:
    """``bridge.run_server`` with the fake engine on a free local port."""

    def __init__(self, vslam: Any) -> None:
        self.log: List[str] = []
        self.port = 0
        ready = threading.Event()

        def listening(port: int) -> None:
            self.port = port
            ready.set()

        def run() -> None:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                bridge.run_server(vslam, "127.0.0.1", 0, once=True, log=self.log.append, accept_timeout_s=20.0,
                                  on_listening=listening)

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        assert ready.wait(10.0)

    def join(self) -> None:
        self.thread.join(timeout=20.0)
        assert not self.thread.is_alive()


def hello(calib: Dict[str, Any], mode: str = "vio", **extra: Any) -> Dict[str, Any]:
    return bridge.make_hello(mode=mode, size=SIZE, fps=30.0, imu_rate_hz=200.0, calibration=calib,
                             device_id="TESTUNIT", source="test", **extra)


def stream(client: bridge.BridgeClient, n_frames: int) -> List[int]:
    """Send *n_frames* pairs with their IMU records in tracker order; returns the frame timestamps."""
    rng = np.random.default_rng(3)
    stamps = []
    next_imu = T0 - IMU_PERIOD_NS
    for i in range(n_frames):
        ts = T0 + i * FRAME_PERIOD_NS
        while next_imu <= ts:
            client.send_imu(next_imu, GYRO, ACCEL)
            next_imu += IMU_PERIOD_NS
        while client.queued_frames >= client.max_queued_frames:
            time.sleep(0.001)
        client.send_frame(ts, i, rng.integers(0, 256, SHAPE, dtype=np.uint8), rng.integers(0, 256, SHAPE, dtype=np.uint8))
        stamps.append(ts)
    return stamps


def collect(client: bridge.BridgeClient) -> List[Tuple[int, Any]]:
    client.end()
    assert client.wait_finished(30.0)
    return client.poll(100000)


# --------------------------------------------------------------------------------------
# Wire format
# --------------------------------------------------------------------------------------

def test_wire_round_trip() -> None:
    a, b = socket.socketpair()
    try:
        left = np.arange(12, dtype=np.uint8).reshape(3, 4)
        right = left[::-1].copy()
        bridge.send_json(a, bridge.MSG_HELLO, {"protocol": 1, "values": np.array([1.5, 2.5]), "n": np.int64(3)})
        bridge.send_imu_records(a, bridge.pack_imu(10, (1, 2, 3), (4, 5, 6)) + bridge.pack_imu(20, (7, 8, 9), (0, 1, 2)))
        bridge.send_frame(a, 1234567890123456789, 7, left, right)
        a.sendall(bridge._HEADER.pack(b"OAKB", bridge.MSG_END, 0))
        assert bridge.recv_message(b) == (bridge.MSG_HELLO, {"protocol": 1, "values": [1.5, 2.5], "n": 3})
        mtype, records = bridge.recv_message(b)
        assert mtype == bridge.MSG_IMU
        assert records == [(10, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0)), (20, (7.0, 8.0, 9.0), (0.0, 1.0, 2.0))]
        mtype, (ts, seq, got_left, got_right) = bridge.recv_message(b)
        assert (mtype, ts, seq) == (bridge.MSG_FRAME, 1234567890123456789, 7)
        assert np.array_equal(got_left, left) and np.array_equal(got_right, right)
        assert bridge.recv_message(b) == (bridge.MSG_END, None)
    finally:
        a.close()
        b.close()


def test_wire_rejects_rubbish() -> None:
    def received(data: bytes) -> socket.socket:
        a, b = socket.socketpair()
        a.sendall(data)
        a.close()
        return b

    cases = ((b"GET / HTTP/1.1 and some more", "magic"),
             (bridge._HEADER.pack(b"OAKB", bridge.MSG_FRAME, 5) + bridge._FRAME_HEAD.pack(0, 0, 640, 400), "bad frame header"),
             (bridge._HEADER.pack(b"OAKB", bridge.MSG_IMU, 57) + bytes(57), "bad IMU payload"),
             (bridge._HEADER.pack(b"OAKB", 77, 0), "unknown message type"),
             (bridge._HEADER.pack(b"OAKB", bridge.MSG_POSE, 4) + b"{{{{", "not JSON"),
             (bridge._HEADER.pack(b"OAKB", bridge.MSG_POSE, 400) + b"{}", "closed"),
             (b"", "closed"))
    for data, expected in cases:
        b = received(data)
        try:
            with pytest.raises(bridge.BridgeError, match=expected):
                bridge.recv_message(b)
        finally:
            b.close()
    a, b = socket.socketpair()
    try:
        with pytest.raises(bridge.BridgeError):
            bridge.send_frame(a, 0, 0, np.zeros((2, 2), np.float32), np.zeros((2, 2), np.float32))
    finally:
        a.close()
        b.close()


def test_client_queue_drops_the_oldest_frame_never_imu(calib: Dict[str, Any]) -> None:
    client = bridge.BridgeClient("127.0.0.1", 1, hello(calib), max_queued_frames=2)   # never connected
    image = np.zeros(SHAPE, np.uint8)
    results = []
    for i in range(5):
        client.send_imu(100 + i, GYRO, ACCEL)
        results.append(client.send_frame(1000 + i, i, image, image))
    assert results == [True, True, False, False, False]
    assert client.frames_dropped == 3 and client.queued_frames == 2
    kinds = [item[0] for item in client._queue]
    assert kinds.count("imu") == 5 and kinds.count("frame") == 2
    assert [item[1] for item in client._queue if item[0] == "frame"] == [1003, 1004]   # the newest two survive


# --------------------------------------------------------------------------------------
# Server + client with the fake engine
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["vio", "stereo"])
def test_end_to_end(vslam: Any, calib: Dict[str, Any], mode: str) -> None:
    server = Server(vslam)
    n = 25
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, mode)) as client:
        ready = client.connect()
        assert ready["fake"] is True and ready["mode"] == mode
        assert ready["config"]["guard"] is True
        assert ready["config"]["max_speed_mps"] == tracking.DEFAULT_MAX_SPEED_MPS
        stamps = stream(client, n)
        events = collect(client)
        assert client.error is None and client.frames_sent == n and client.frames_dropped == 0
    server.join()
    poses = [value for mtype, value in events if mtype == bridge.MSG_POSE]
    assert [pose["ts"] for pose in poses] == stamps
    assert all(len(pose["p"]) == 3 and len(pose["q"]) == 4 for pose in poses)
    assert all(len(pose["f"]) > 0 and len(pose["f"]) % 2 == 0 for pose in poses)
    assert not [1 for mtype, _ in events if mtype == bridge.MSG_SKIP]
    assert events[-1][0] == bridge.MSG_SUMMARY
    summary = events[-1][1]
    assert summary["frames_tracked"] == n and "rows" not in summary
    assert summary["bridge"] == {"frames_received": n, "imu_received": client.imu_sent, "client_closed_early": False}
    if mode == "vio":
        assert summary["imu_registered"] > 0 and any(pose["g"] is not None for pose in poses)
    assert any("done: 25 frame(s) received" in line for line in server.log)


def test_guarded_frame_is_coasted_or_reported_as_skip(vslam: Any, calib: Dict[str, Any]) -> None:
    # default: the guarded frame gets a coasted estimate, flagged, no SKIP
    vslam.set_default_pose_source(walk(vslam, jumps={10: 3.0}))
    server = Server(vslam)
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, "stereo")) as client:
        client.connect()
        stamps = stream(client, 20)
        events = collect(client)
    server.join()
    poses = [value for mtype, value in events if mtype == bridge.MSG_POSE]
    assert [pose["ts"] for pose in poses] == stamps
    assert [pose["coast"] for pose in poses] == [i == 10 for i in range(20)]
    assert "f" not in poses[10] and "f" in poses[9]
    assert not [1 for mtype, _ in events if mtype == bridge.MSG_SKIP]
    assert events[-1][1]["frames_guarded"] == 1 and events[-1][1]["frames_coasted"] == 1

    # coasting off: the frame is reported as SKIP, in place
    vslam.set_default_pose_source(walk(vslam, jumps={10: 3.0}))
    server = Server(vslam)
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, "stereo", options={"coast": False})) as client:
        client.connect()
        stamps = stream(client, 20)
        events = collect(client)
    server.join()
    skips = [value["ts"] for mtype, value in events if mtype == bridge.MSG_SKIP]
    poses = [value["ts"] for mtype, value in events if mtype == bridge.MSG_POSE]
    assert skips == [stamps[10]]
    assert poses == stamps[:10] + stamps[11:]
    order = [(mtype, value.get("ts")) for mtype, value in events if mtype in (bridge.MSG_POSE, bridge.MSG_SKIP)]
    assert order.index((bridge.MSG_SKIP, stamps[10])) == 10          # reported in place, before the next pose
    assert events[-1][1]["frames_guarded"] == 1 and events[-1][1]["frames_coasted"] == 0


def test_client_options_and_correction_reach_the_engine(vslam: Any, calib: Dict[str, Any]) -> None:
    server = Server(vslam)
    correction = {"accel_bias": [0.05, -0.69, -0.12], "accel_scale": [1.0, 1.0, 1.0], "gyro_bias": [0, 0, 0]}
    options = {"guard": False, "max_speed_mps": 7.5, "guard_reset_after": None, "log": "not a client option"}
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, "vio", options=options,
                                                            imu_correction=correction, features=False)) as client:
        ready = client.connect()
        stream(client, 5)
        events = collect(client)
    server.join()
    cfg = ready["config"]
    assert cfg["guard"] is False and cfg["max_speed_mps"] == 7.5
    assert cfg["guard_reset_after"] == tracking.DEFAULT_GUARD_RESET_AFTER       # None = the server's default
    assert cfg["imu_correction"]["accel_bias"] == [0.05, -0.69, -0.12]
    assert cfg["log"] is True                                                    # the server's own logger
    assert all("f" not in value for mtype, value in events if mtype == bridge.MSG_POSE)


def test_server_reports_errors(vslam: Any, calib: Dict[str, Any]) -> None:
    server = Server(vslam)
    bad = hello(calib)
    bad["protocol"] = 99
    client = bridge.BridgeClient("127.0.0.1", server.port, bad)
    with pytest.raises(bridge.BridgeError, match="protocol 99"):
        client.connect()
    server.join()
    assert any("client failed" in line for line in server.log)

    server = Server(vslam)
    broken = hello({"not": "a calibration"})
    with pytest.raises(bridge.BridgeError, match="server:"):
        bridge.BridgeClient("127.0.0.1", server.port, broken).connect()
    server.join()


def test_wrong_frame_size_ends_the_run_with_an_error(vslam: Any, calib: Dict[str, Any]) -> None:
    server = Server(vslam)
    with bridge.BridgeClient("127.0.0.1", server.port, hello(calib, "stereo")) as client:
        client.connect()
        client.send_frame(T0, 0, np.zeros((200, 320), np.uint8), np.zeros((200, 320), np.uint8))
        assert client.wait_finished(20.0)
        assert client.error is not None and "320x200" in client.error
    server.join()


# --------------------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------------------

def test_levelling_rotation() -> None:
    for gravity in ([0.0, 9.81, 0.0], [0.0, 8.0, 5.0], [3.0, -2.0, 9.0], [0.0, -9.81, 0.0]):
        R = live_view.levelling_rotation(gravity)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9) and math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-9)
        assert np.allclose(R @ (np.array(gravity) / np.linalg.norm(gravity)), [0.0, 1.0, 0.0], atol=1e-9)
    assert np.array_equal(live_view.levelling_rotation([0.0, 0.0, 0.0]), np.eye(3))
    assert np.allclose(live_view.quaternion_matrix([0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])
                       @ np.array([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], atol=1e-9)
    assert live_view.nice_grid(2.0) == 0.25 and live_view.nice_grid(8.0) == 1.0 and live_view.nice_grid(64.0) == 10.0


def pose_event(i: int, p: Tuple[float, float, float], gravity: Any = None) -> Tuple[int, Dict[str, Any]]:
    return bridge.MSG_POSE, {"ts": T0 + i * FRAME_PERIOD_NS, "i": i, "p": list(p), "q": [0.0, 0.0, 0.0, 1.0],
                             "ms": 5.0, "seg": 0, "g": gravity, "f": [10, 20, 600, 380]}


def test_live_view_state_and_render() -> None:
    view = live_view.LiveView("vio", (1280, 800), title="test", duration_s=30.0)
    empty = view.render()
    assert empty.shape == (720, 1280, 3) and empty.dtype == np.uint8
    assert view.remaining_s() is None                       # the clock starts with the first frame

    tilt = math.radians(20.0)                               # the camera started pitched: gravity partly along z
    gravity = [0.0, 9.81 * math.cos(tilt), 9.81 * math.sin(tilt)]
    image = np.full((800, 1280), 90, np.uint8)
    events = []
    for i in range(60):
        view.add_image(T0 + i * FRAME_PERIOD_NS, image)
        events.append(pose_event(i, (0.0, 0.0, 0.05 * i), gravity))
    events[40] = (bridge.MSG_POSE, dict(events[40][1], coast=True))
    events.insert(30, (bridge.MSG_SKIP, {"ts": T0 - 1}))
    events.append((bridge.MSG_STATS, {"frames_guarded": 1, "frames_failed": 0, "resets": 0}))
    view.handle(events)
    view.set_bridge(60, 2, 1)
    assert len(view.poses) == 60 and len(view.skips) == 1 and view.skips[0][1] == 30
    assert view.levelled
    levelled = view.positions()
    assert np.allclose(np.linalg.norm(levelled, axis=1), 0.05 * np.arange(60), atol=1e-9)
    assert levelled[-1][1] == pytest.approx(0.05 * 59 * math.sin(tilt), abs=1e-6)     # moving along z sinks
    assert view.path_length() == pytest.approx(0.05 * 59, abs=1e-9)
    assert view.latency_ms is not None and 0.0 <= (view.remaining_s() or 0.0) <= 30.0
    picture = view.render()
    assert picture.shape == (720, 1280, 3)
    assert not np.array_equal(picture, empty)
    assert (picture[:400, :640] == np.array(live_view._FEATURE, np.uint8)).all(axis=2).any()    # features drawn
    assert (picture[:, 640:] == np.array(live_view._TRAIL, np.uint8)).all(axis=2).any()         # trail drawn
    assert (picture[:, 640:] == np.array(live_view._COAST, np.uint8)).all(axis=2).any()         # coasted segment drawn
    assert view.coasted.count(True) == 1 and len(view.coasted) == len(view.poses)

    view.show_features = False
    assert not (view.render()[:400, :640] == np.array(live_view._FEATURE, np.uint8)).all(axis=2).any()
    view.clear_trail()
    assert view.poses == [] and view.path_length() == 0.0 and len(view.all_poses()) == 60
    assert view.all_coasted().count(True) == 1 and len(view.all_coasted()) == 60
    view.handle([(bridge.MSG_SUMMARY, {"frames_tracked": 60})])
    assert view._state()[0] == "ENDED"
    view.handle([(bridge.MSG_ERROR, {"message": "boom"})])
    assert view._state()[0] == "ERROR" and view.render().shape == (720, 1280, 3)


def test_travelled_ignores_jitter() -> None:
    view = live_view.LiveView("stereo", SIZE)
    rng = np.random.default_rng(5)
    view.handle([pose_event(i, tuple(rng.normal(0.0, 0.001, 3))) for i in range(600)])      # a camera at rest
    assert view.path_length() == 0.0
    assert not view.levelled                                                                 # no gravity in stereo
    assert np.array_equal(view.rotation(), np.eye(3))


# --------------------------------------------------------------------------------------
# Scripts
# --------------------------------------------------------------------------------------

def test_view_script_replays_a_recording(tmp_path: Path, calib: Dict[str, Any]) -> None:
    root = tmp_path / "euroc_tiny"
    stamps = write_tiny_dataset(root, calib, n_frames=8)
    tum, png = tmp_path / "out" / "live.tum", tmp_path / "out" / "live.png"
    proc = run_cli(VIEW_SCRIPT, "--replay", root, "--fake-cuvslam", "--headless", "--speed", 0, "--save-tum", tum,
                   "--snapshot", png, "--max-speed", 9)
    assert proc.returncode == 0, proc.stderr + proc.stdout
    assert "Traceback" not in proc.stderr
    assert "FAKE cuVSLAM in use" in proc.stdout
    assert "guard on (max 9.0 m/s" in proc.stdout
    assert "tracked 8 of 8 frames" in proc.stdout
    assert "Camera released" not in proc.stdout                  # no camera was opened
    rows = tracking.read_tum(tum)
    assert [row[0] for row in rows] == stamps
    assert png.is_file() and png.stat().st_size > 10_000


def test_view_script_input_errors(tmp_path: Path, calib: Dict[str, Any]) -> None:
    proc = run_cli(VIEW_SCRIPT, "--replay", tmp_path / "nothing", "--fake-cuvslam", "--headless")
    assert proc.returncode == 2 and "cannot replay" in proc.stderr and "Traceback" not in proc.stderr
    proc = run_cli(VIEW_SCRIPT, "--replay", tmp_path, "--server", "nonsense", "--headless")
    assert proc.returncode == 2 and "HOST:PORT" in proc.stderr
    root = tmp_path / "euroc_tiny"
    write_tiny_dataset(root, calib, n_frames=3)
    with socket.socket() as probe:                               # a port nobody listens on
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    proc = run_cli(VIEW_SCRIPT, "--replay", root, "--server", "127.0.0.1:{}".format(port), "--headless")
    assert proc.returncode == 3 and proc.stderr.startswith("engine:") and "Traceback" not in proc.stderr


def test_scripts_help_and_idle_server() -> None:
    for script, flags in ((VIEW_SCRIPT, ("--replay", "--duration", "--max-speed", "--imu-calibration", "--server",
                                         "--headless", "--snapshot")),
                          (SERVER_SCRIPT, ("--port", "--keep-serving", "--accept-timeout", "--fake-cuvslam"))):
        proc = run_cli(script, "--help")
        assert proc.returncode == 0, proc.stderr
        assert all(flag in proc.stdout for flag in flags)
    proc = run_cli(SERVER_SCRIPT, "--fake-cuvslam", "--port", 0, "--host", "127.0.0.1", "--accept-timeout", 0.5)
    assert proc.returncode == 0, proc.stderr
    assert bridge.LISTENING_MARKER in proc.stdout and "no client within" in proc.stdout
