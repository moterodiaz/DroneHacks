"""``course/live_view.py``: cuVSLAM poses on the map, the view's state, and the drawing (no display needed)."""

from __future__ import annotations

import importlib.util
import math
import time
from pathlib import Path

import numpy as np
import pytest

from course import live_view as lv
from course.course_map import CourseMap

REPO_ROOT = Path(__file__).resolve().parents[1]


def _rot(axis, deg):
    """Rotation matrix about ``axis`` (camera frame) by ``deg``."""
    u = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    a = math.radians(deg)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def _quat(r):
    w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    return [(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w]


def test_camera_forward_moves_the_body_forward_on_the_map() -> None:
    anchor = (1.0, 2.0, math.pi / 2)                     # nose along +y
    x, y, yaw = lv.vio_to_map(anchor, np.zeros(3), np.eye(3), np.array([0.0, 0.0, 1.0]), np.eye(3))  # camera z = forward
    assert (x, y, yaw) == pytest.approx((1.0, 3.0, math.pi / 2))
    x, y, _ = lv.vio_to_map(anchor, np.zeros(3), np.eye(3), np.array([-1.0, 0.0, 0.0]), np.eye(3))  # camera -x = left
    assert (x, y) == pytest.approx((0.0, 2.0))


def test_camera_turning_left_turns_the_body_left() -> None:
    r = _rot([0, -1, 0], 30)                              # about camera -y = body up: a left turn
    _, _, yaw = lv.vio_to_map((0.0, 0.0, 0.0), np.zeros(3), np.eye(3), np.zeros(3), r)
    assert math.degrees(yaw) == pytest.approx(30.0)
    assert lv.quat_to_matrix(_quat(r)) == pytest.approx(r)
    _, _, yaw = lv.vio_to_map((0.0, 0.0, 0.0), np.zeros(3), np.eye(3), np.zeros(3), r, cam_yaw=math.radians(90))
    assert math.degrees(yaw) == pytest.approx(30.0)       # a camera looking left sees the same turn


def test_motion_is_relative_to_the_anchor_pose() -> None:
    r0, t0 = _rot([0, -1, 0], 50), np.array([3.0, -1.0, 2.0])   # cuVSLAM's world is arbitrary
    step = r0 @ np.array([0.0, 0.0, 0.5])                        # 0.5 m along the camera's own z
    x, y, yaw = lv.vio_to_map((0.0, 0.0, 0.0), t0, r0, t0 + step, r0)
    assert (x, y, yaw) == pytest.approx((0.5, 0.0, 0.0), abs=1e-9)


@pytest.fixture(scope="module")
def room() -> CourseMap:
    spec = importlib.util.spec_from_file_location("bench_map", REPO_ROOT / "course" / "20_bench_map.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return CourseMap(mod.room_map(4.2, 6.1, (2.1, 1.0), 2.5, []))


def test_state_and_drawing(room) -> None:
    state = lv.ViewState()
    angles = np.linspace(-math.pi, math.pi, 450, endpoint=False).astype(np.float32)
    lidar = {"src": "lidar", "t": 1.0, "pose": (2.1, 1.0, math.pi / 2), "match_pose": (2.1, 1.0, math.pi / 2), "lost": 0,
             "reason": "", "rms_m": 0.02, "used_frac": 0.98, "ranges": np.full(450, 1.5, np.float32), "angles": angles,
             "age_ms": 25.0, "sent": 10, "px4": (2.11, 1.0, math.pi / 2), "px4_rp": (0.0, 0.01), "timesync": 600}
    state.handle(lidar)
    state.handle({"src": "vio", "t": 1.0, "ts_ns": 1, "segment": 0, "translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1],
                  "track_ms": 9.0, "thumb": np.zeros((240, 384), np.uint8)})
    assert state.vio_anchor is not None and state.last_vio_pose == pytest.approx((2.1, 1.0, math.pi / 2))
    state.handle({"src": "vio", "t": 1.1, "ts_ns": 2, "segment": 0, "translation": [0, 0, 0.3], "rotation_xyzw": [0, 0, 0, 1],
                  "track_ms": 9.0})
    assert state.last_vio_pose[:2] == pytest.approx((2.1, 1.3))
    state.handle({"src": "vio_stats", "t": 1.1, "frames_attempted": 30, "frames_tracked": 30, "resets": 0, "fps": 30.0, "track_ms": 9.0})
    canvas = lv.MapCanvas(room, 1.05)
    frame = lv.render(state, canvas, canvas.background(), 1.2)
    assert frame.shape == (canvas.h, canvas.w + 520, 3)
    assert (frame[:, : canvas.w] != 255).any(axis=2).sum() > 1000   # map, scan and trails were drawn


def test_px4_trail_restarts_when_px4_jumps_onto_the_lidar() -> None:
    state = lv.ViewState()
    base = {"src": "lidar", "pose": (0.0, 0.0, 1.57), "match_pose": (0.0, 0.0, 1.57), "lost": 0, "ranges": None, "angles": None}
    state.handle({**base, "t": 1.0, "px4": (26.5, 87.8, 0.0)})     # PX4 still on its own, far away
    state.handle({**base, "t": 1.1, "px4": (0.01, 0.0, 1.57)})      # then it fuses the lidar
    assert len(state.px4) == 1 and state.px4[0][1] == pytest.approx(0.01)


def test_growing_canvas_widens_to_keep_new_map_points_in_view() -> None:
    canvas = lv.GrowingCanvas(min_span_m=6.0)
    assert canvas.bounds[1] - canvas.bounds[0] == pytest.approx(6.0)
    assert canvas.update([np.array([[1.0, 1.0], [2.0, -1.0]])], np.zeros((1, 2)))       # new points: redraw
    assert canvas.bounds[1] - canvas.bounds[0] == pytest.approx(6.0)                   # still fits
    canvas.update([np.array([[9.0, 0.0]])], np.zeros((1, 2)))
    x0, x1, _, _ = canvas.bounds
    assert x0 < 0.0 and x1 > 9.0                                                        # widened to keep both
    assert canvas.background().shape == (canvas.h, canvas.w, 3)


def _dies_like_cuvslam(q, stop, repo_root, args, log_path, sync_ba):
    """Stand-in for vio_worker: posts one pose, then aborts (cuVSLAM's std::terminate -> SIGABRT)."""
    import os
    import signal

    lv.post(q, {"src": "vio", "t": 0.0, "ts_ns": 1, "segment": 0, "translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1],
                "track_ms": 1.0, "log": log_path})
    time.sleep(0.2)
    os.kill(os.getpid(), signal.SIGABRT)


def test_supervisor_restarts_a_dead_cuvslam_and_the_view_re_anchors(tmp_path) -> None:
    import multiprocessing as mp

    ctx = mp.get_context("fork")
    q, stop = ctx.Queue(), ctx.Event()
    sup = lv.VioSupervisor(ctx, q, stop, str(REPO_ROOT), ["--out", str(tmp_path / "vio.tum")], tmp_path, max_restarts=2,
                           target=_dies_like_cuvslam)
    sup.start()
    deadline = time.time() + 15
    while sup.restarts < 2 and time.time() < deadline:
        time.sleep(0.2)
    time.sleep(1.5)
    stop.set()
    sup.join(timeout=5)
    assert sup.restarts == 2 and sup.exits[:2] == [-6, -6]           # SIGABRT, reported, replaced twice
    msgs = []
    while not q.empty():
        msgs.append(q.get())
    logs = [m["log"] for m in msgs if m["src"] == "vio"]
    assert logs[:3] == [str(tmp_path / "vio.log"), str(tmp_path / "vio.r1.log"), str(tmp_path / "vio.r2.log")]
    state = lv.ViewState()
    state.vio_anchor, state.last_vio_pose = ((1.0, 1.0, 0.0), np.zeros(3), np.eye(3)), (1.0, 1.0, 0.0)
    state.handle(next(m for m in msgs if m["src"] == "vio_restart"))
    assert state.vio_anchor is None and "restarted" in state.vio_info  # placed again at the next lidar pose
