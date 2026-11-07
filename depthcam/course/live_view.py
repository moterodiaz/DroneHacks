"""The live localisation view's pieces: cuVSLAM poses on the map, the drawing, and its two worker processes.

``22_live_localization.py`` runs the lidar -> PX4 bridge in its own process and starts:

* :func:`vio_worker`: ``methods/cuvslam-vio/08_cuvslam_track.py --live`` unchanged, its ``on_pose`` /
  ``on_stats`` callbacks wrapped to post every pose and a small camera thumbnail;
* :func:`gui_worker`: the OpenCV window (or PNG snapshots with ``--headless``), drawing from what the
  bridge and cuVSLAM post.  Drawing never runs in the bridge's process, so it cannot delay ODOMETRY.

cuVSLAM poses are ``world_from_rig`` with the rig = the left camera (x right, y down, z forward).
:func:`vio_to_map` turns the motion since an anchor into the body frame (FLU) and puts it on the map
at the lidar pose taken when cuVSLAM started (``r`` in the window re-anchors).
"""

from __future__ import annotations

import collections
import math
import queue
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

#: Camera (x right, y down, z forward) -> body FLU (x forward, y left, z up): v_body = CAM_TO_BODY @ v_cam.
CAM_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

COLORS = {"lidar": (215, 120, 40), "px4": (30, 140, 245), "vio": (190, 60, 200), "scan": (60, 170, 60),
          "scan_lost": (60, 60, 220), "wall": (70, 70, 70), "fill": (225, 225, 225), "grid": (238, 238, 238),
          "text": (30, 30, 30), "ok": (40, 150, 40), "warn": (0, 140, 230), "bad": (40, 40, 210)}


def quat_to_matrix(xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def vio_to_map(anchor_map: Sequence[float], t0: np.ndarray, r0: np.ndarray, t: np.ndarray, r: np.ndarray,
               cam_yaw: float = 0.0) -> Tuple[float, float, float]:
    """Map pose (x, y, yaw) of the body now, from the camera pose at the anchor (``t0``, ``r0``) and
    now (``t``, ``r``), both world_from_camera, and the body's map pose at the anchor.  ``cam_yaw``:
    the camera's heading on the body, CCW from the nose (0 = looking forward)."""
    r_rel = r0.T @ r
    t_rel = r0.T @ (np.asarray(t, dtype=float) - np.asarray(t0, dtype=float))
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    mount = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ CAM_TO_BODY  # camera -> body
    rb = mount @ r_rel @ mount.T
    tb = mount @ t_rel
    ax, ay, ayaw = anchor_map
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    dyaw = math.atan2(rb[1, 0], rb[0, 0])
    return ax + ca * tb[0] - sa * tb[1], ay + sa * tb[0] + ca * tb[1], ayaw + dyaw


def jetson_temps() -> Dict[str, float]:
    out = {}
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            kind = (zone / "type").read_text().strip()
            if kind in ("tj-thermal", "cpu-thermal", "gpu-thermal"):
                out[kind.split("-")[0]] = int((zone / "temp").read_text()) / 1000.0
        except (OSError, ValueError):
            continue
    return out


def post(q, msg: Dict[str, Any]) -> None:
    """Hand a message to the window without ever blocking the sender (a full queue drops it)."""
    try:
        q.put_nowait(msg)
    except queue.Full:
        pass


# --- cuVSLAM worker ---------------------------------------------------------------------------------

def vio_worker(q, stop, repo_root: str, args: List[str], log_path: str, sync_ba: bool = False) -> None:
    """Run 08_cuvslam_track.py --live in this process, posting poses, stats and thumbnails to ``q``.
    ``sync_ba``: bundle adjustment in the tracking thread (TrackConfig.realtime False, as every
    offline replay runs) instead of cuVSLAM's background thread."""
    import importlib.util
    import sys

    sys.path.insert(0, repo_root)
    import faulthandler
    import os
    import signal

    log = open(log_path, "w", buffering=1)
    sys.stdout = sys.stderr = log
    Path(log_path + ".pid").write_text(str(os.getpid()))
    faulthandler.register(signal.SIGUSR1, file=log, all_threads=True)  # kill -USR1 <pid>: every thread's stack to vio.log
    try:
        import cv2
        spec = importlib.util.spec_from_file_location("track08", Path(repo_root) / "methods" / "cuvslam-vio" / "08_cuvslam_track.py")
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        orig_run = m.oaktrack.run_tracking
        state = {"n": 0}

        def run_tracking(*a, on_pose=None, on_stats=None, **k):
            def pose_cb(ev):
                # 8 s without a pose: every thread's stack goes to vio.log, from a watchdog outside the GIL
                faulthandler.dump_traceback_later(8.0, repeat=False, file=log)
                if on_pose is not None:
                    on_pose(ev)
                state["n"] += 1
                msg = {"src": "vio", "t": time.monotonic(), "ts_ns": int(ev.ts_ns), "segment": int(ev.segment),
                       "translation": np.asarray(ev.translation, dtype=float).tolist(),
                       "rotation_xyzw": np.asarray(ev.rotation_xyzw, dtype=float).tolist(),
                       "track_ms": float(ev.track_ms), "coasted": bool(getattr(ev, "coasted", False))}
                if state["n"] % 6 == 0 and ev.images:
                    img = np.asarray(ev.images[0])
                    msg["thumb"] = cv2.resize(img, (384, int(384 * img.shape[0] / img.shape[1])), interpolation=cv2.INTER_AREA)
                post(q, msg)

            def stats_cb(st):
                if on_stats is not None:
                    on_stats(st)
                keep = ("frames_seen", "frames_attempted", "frames_tracked", "resets", "frames_guarded", "fps", "track_ms", "final")
                post(q, {"src": "vio_stats", "t": time.monotonic(), **{k: st.get(k) for k in keep}})

            return orig_run(*a, on_pose=pose_cb, on_stats=stats_cb, **k)

        m.oaktrack.run_tracking = run_tracking
        m.make_should_stop = lambda duration, viewer: (lambda: stop.is_set() or viewer.stop_requested)
        if sync_ba:
            orig_config = m.make_config

            def make_config(*a, **k):
                cfg = orig_config(*a, **k)
                cfg.realtime = False
                return cfg

            m.make_config = make_config
        code = m.main(args)
        post(q, {"src": "vio_info", "t": time.monotonic(), "text": "cuVSLAM stopped (exit {})".format(code)})
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # shown in the window, not just the log
        post(q, {"src": "vio_info", "t": time.monotonic(), "text": "cuVSLAM failed: {}".format(exc)})
        raise
    finally:
        log.close()


class VioSupervisor:
    """Keeps :func:`vio_worker` running: cuVSLAM 17.0.0 can abort its process (an uncaught
    ``std::out_of_range`` in its background bundle-adjustment thread; docs/bench-results.md).  A thread
    in the main process checks once a second; a dead worker is reported (exit code) and replaced, its
    log continuing in vio.r<n>.log, and the window re-places cuVSLAM at the lidar pose."""

    def __init__(self, ctx, q, stop, repo_root: str, args: List[str], log_dir: Path, sync_ba: bool = False,
                 max_restarts: int = 50, target=None) -> None:
        self.target = target or vio_worker
        self.ctx, self.q, self.stop, self.repo_root, self.args = ctx, q, stop, repo_root, list(args)
        self.log_dir, self.sync_ba, self.max_restarts = Path(log_dir), sync_ba, max_restarts
        self.restarts, self.exits = 0, []
        self.proc = None
        self._thread = None

    def _spawn(self) -> None:
        tag = "" if self.restarts == 0 else ".r{}".format(self.restarts)
        args = [a if not a.endswith("vio.tum") else str(self.log_dir / "vio{}.tum".format(tag)) for a in self.args]
        self.proc = self.ctx.Process(target=self.target, name="cuvslam", daemon=True,
                                     args=(self.q, self.stop, self.repo_root, args, str(self.log_dir / "vio{}.log".format(tag)), self.sync_ba))
        self.proc.start()

    def _watch(self) -> None:
        while not self.stop.wait(1.0):
            if self.proc.is_alive():
                continue
            code = self.proc.exitcode
            self.exits.append(code)
            if self.restarts >= self.max_restarts:
                post(self.q, {"src": "vio_info", "t": time.monotonic(), "text": "cuVSLAM died (exit {}); no more restarts".format(code)})
                return
            self.restarts += 1
            print("cuVSLAM process died (exit code {}): restart {}".format(code, self.restarts), flush=True)
            post(self.q, {"src": "vio_restart", "t": time.monotonic(),
                          "text": "cuVSLAM died (exit {}): restarted {}x".format(code, self.restarts)})
            self._spawn()

    def start(self) -> None:
        import threading

        self._spawn()
        self._thread = threading.Thread(target=self._watch, name="vio-supervisor", daemon=True)
        self._thread.start()

    def join(self, timeout: float = 15.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.proc is not None:
            self.proc.join(timeout=timeout)
            if self.proc.is_alive():
                self.proc.terminate()
                self.proc.join(timeout=5)
                print("cuvslam process did not stop: terminated")


# --- the window -------------------------------------------------------------------------------------

@dataclass
class ViewState:
    trail_s: float = 60.0
    lidar: Deque[Tuple[float, float, float]] = field(default_factory=collections.deque)
    px4: Deque[Tuple[float, float, float]] = field(default_factory=collections.deque)
    vio: Deque[Tuple[float, float, float]] = field(default_factory=collections.deque)
    last_lidar: Optional[Dict[str, Any]] = None
    last_vio_pose: Optional[Tuple[float, float, float]] = None
    vio_stats: Dict[str, Any] = field(default_factory=dict)
    vio_anchor: Optional[Tuple[Tuple[float, float, float], np.ndarray, np.ndarray]] = None
    vio_raw: Optional[Tuple[np.ndarray, np.ndarray]] = None
    vio_segment: Optional[int] = None
    vio_info: str = ""
    vio_last_t: float = 0.0
    thumb: Optional[np.ndarray] = None
    ages: Deque[float] = field(default_factory=lambda: collections.deque(maxlen=300))
    lost_recent: Deque[int] = field(default_factory=lambda: collections.deque(maxlen=100))
    px4_err: Deque[float] = field(default_factory=lambda: collections.deque(maxlen=100))
    lidar_times: Deque[float] = field(default_factory=lambda: collections.deque(maxlen=50))
    bridge_info: str = ""
    cam_yaw: float = 0.0
    new_points: List[np.ndarray] = field(default_factory=list)   # map points the live mapper added (no-map mode)

    def _trail(self, trail, t, pose) -> None:
        trail.append((t, pose[0], pose[1]))
        while trail and trail[0][0] < t - self.trail_s:
            trail.popleft()

    def last_lidar_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.last_lidar is None or self.last_lidar["lost"]:
            return None
        return tuple(self.last_lidar["pose"])

    def anchor_vio(self) -> bool:
        """Put cuVSLAM's current pose at the lidar's current pose."""
        lp = self.last_lidar_pose()
        if lp is None or self.vio_raw is None:
            return False
        self.vio_anchor = (lp, self.vio_raw[0].copy(), self.vio_raw[1].copy())
        self.vio.clear()
        return True

    def handle(self, msg: Dict[str, Any]) -> None:
        src, t = msg["src"], msg["t"]
        if src == "lidar":
            self.last_lidar = msg
            self.lidar_times.append(t)
            self.lost_recent.append(int(msg["lost"]))
            if msg.get("age_ms") is not None:
                self.ages.append(msg["age_ms"])
            if not msg["lost"]:
                self._trail(self.lidar, t, msg["pose"])
            if msg.get("px4") is not None:
                if self.px4 and math.hypot(msg["px4"][0] - self.px4[-1][1], msg["px4"][1] - self.px4[-1][2]) > 0.5:
                    self.px4.clear()  # PX4 jumped (onto the lidar when it starts fusing): no line across the jump
                self._trail(self.px4, t, msg["px4"])
                if not msg["lost"]:
                    self.px4_err.append(math.hypot(msg["px4"][0] - msg["pose"][0], msg["px4"][1] - msg["pose"][1]))
        elif src == "vio":
            tr, rot = np.array(msg["translation"]), quat_to_matrix(msg["rotation_xyzw"])
            if self.vio_segment is not None and msg["segment"] != self.vio_segment and self.vio_anchor and self.last_vio_pose:
                # a new cuVSLAM segment restarts its world: continue from where the trail was
                self.vio_anchor = (self.last_vio_pose, tr.copy(), rot.copy())
            self.vio_segment = msg["segment"]
            self.vio_raw = (tr, rot)
            self.vio_last_t = t
            if self.vio_anchor is None:
                self.anchor_vio()
            if self.vio_anchor is not None:
                a, t0, r0 = self.vio_anchor
                self.last_vio_pose = vio_to_map(a, t0, r0, tr, rot, self.cam_yaw)
                self._trail(self.vio, t, self.last_vio_pose)
            if msg.get("thumb") is not None:
                self.thumb = msg["thumb"]
        elif src == "vio_stats":
            self.vio_stats = msg
        elif src == "vio_info":
            self.vio_info = msg["text"]
        elif src == "vio_restart":  # a new cuVSLAM process: a new origin, placed again at the next lidar pose
            self.vio_info = msg["text"]
            self.vio_anchor, self.vio_raw, self.vio_segment, self.last_vio_pose = None, None, None, None
            self.vio.clear()
        elif src == "bridge_info":
            self.bridge_info = msg["text"]
        elif src == "map_points":
            self.new_points.append(np.asarray(msg["points"], dtype=float))


class MapCanvas:
    """World (map metres) <-> pixels for a map panel of ``size`` pixels."""

    def __init__(self, cmap, scan_z: float, size: Tuple[int, int] = (760, 760), margin_m: float = 0.3) -> None:
        self.w, self.h = size
        (x0, x1), (y0, y1) = cmap.boundary["x"], cmap.boundary["y"]
        x0, x1, y0, y1 = x0 - margin_m, x1 + margin_m, y0 - margin_m, y1 + margin_m
        self.scale = min(self.w / (x1 - x0), self.h / (y1 - y0))
        self.x0 = x0 - (self.w / self.scale - (x1 - x0)) / 2
        self.y1 = y1 + (self.h / self.scale - (y1 - y0)) / 2
        self.polys = [np.asarray(o.polygon, dtype=float) for o in cmap.obstacles_2d(scan_z - 0.02, scan_z + 0.02, rocks_full_height=False)]
        marks = [f for f in cmap.of_kind("marking")]
        self.marks = [np.asarray(f["centre"], dtype=float) for f in marks]
        self.bounds = (x0, x1, y0, y1)

    def px(self, xy) -> np.ndarray:
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        return np.stack([(xy[:, 0] - self.x0) * self.scale, (self.y1 - xy[:, 1]) * self.scale], axis=1).round().astype(np.int32)

    def background(self) -> np.ndarray:
        import cv2

        img = np.full((self.h, self.w, 3), 255, np.uint8)
        x0, x1, y0, y1 = self.bounds
        for gx in np.arange(math.ceil(x0), x1, 1.0):
            cv2.line(img, tuple(self.px([gx, y0])[0]), tuple(self.px([gx, y1])[0]), COLORS["grid"], 1)
        for gy in np.arange(math.ceil(y0), y1, 1.0):
            cv2.line(img, tuple(self.px([x0, gy])[0]), tuple(self.px([x1, gy])[0]), COLORS["grid"], 1)
        for p in self.polys:
            cv2.fillPoly(img, [self.px(p)], COLORS["fill"])
            cv2.polylines(img, [self.px(p)], True, COLORS["wall"], 1, cv2.LINE_AA)
        for m in self.marks:
            c = tuple(self.px(m)[0])
            cv2.drawMarker(img, c, (40, 40, 160), cv2.MARKER_TILTED_CROSS, 12, 2)
        cv2.putText(img, "grid 1 m", (8, self.h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1, cv2.LINE_AA)
        return img


class GrowingCanvas(MapCanvas):
    """No map: the live mapper's points, drawn as they come, the view widened to keep them, the
    trails and the start in sight (at least ``min_span_m`` across, centred on the content)."""

    def __init__(self, size: Tuple[int, int] = (760, 760), min_span_m: float = 6.0, cell_m: float = 0.04) -> None:
        self.w, self.h = size
        self.min_span, self.cell = min_span_m, cell_m
        self.cells = np.zeros((0, 2), np.int64)
        self.polys, self.marks = [], [np.zeros(2)]
        self._fit(np.zeros((1, 2)))

    def _fit(self, xy: np.ndarray) -> None:
        lo, hi = xy.min(0), xy.max(0)
        span = max(self.min_span, float((hi - lo).max()) * 1.25)
        c = (lo + hi) / 2
        x0, x1, y0, y1 = c[0] - span / 2, c[0] + span / 2, c[1] - span / 2, c[1] + span / 2
        self.scale = min(self.w, self.h) / span
        self.x0, self.y1 = x0, y1
        self.bounds = (x0, x1, y0, y1)

    def _inside(self, xy: np.ndarray, margin: float = 0.05) -> bool:
        x0, x1, y0, y1 = self.bounds
        m = margin * (x1 - x0)
        return bool(len(xy) == 0 or ((xy[:, 0] > x0 + m) & (xy[:, 0] < x1 - m) & (xy[:, 1] > y0 + m) & (xy[:, 1] < y1 - m)).all())

    def update(self, new_points: List[np.ndarray], keep_in_view: np.ndarray) -> bool:
        """Add the mapper's new points; True when the background must be redrawn."""
        changed = False
        if new_points:
            cells = np.unique(np.floor(np.vstack(new_points) / self.cell).astype(np.int64), axis=0)
            self.cells = np.unique(np.vstack([self.cells, cells]), axis=0)
            changed = True
        content = np.vstack([self.cells * self.cell, keep_in_view.reshape(-1, 2), np.zeros((1, 2))])
        if not self._inside(content):
            self._fit(content)
        return changed

    def background(self) -> np.ndarray:
        import cv2

        img = super().background()
        for p in self.px((self.cells + 0.5) * self.cell):
            cv2.circle(img, (int(p[0]), int(p[1])), 1, COLORS["wall"], -1)
        cv2.putText(img, "no map: lidar mapping from the start (x)", (8, self.h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (120, 120, 120), 1, cv2.LINE_AA)
        return img


def _cm(metres: float) -> str:
    return "-" if not math.isfinite(metres) else "{:.1f} cm".format(100 * metres)


def _arrow(img, canvas, pose, color, label) -> None:
    import cv2

    x, y, yaw = pose
    a = canvas.px([x, y])[0]
    b = canvas.px([x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)])[0]
    cv2.circle(img, tuple(a), 6, color, -1, cv2.LINE_AA)
    cv2.arrowedLine(img, tuple(a), tuple(b), color, 2, cv2.LINE_AA, tipLength=0.35)
    cv2.putText(img, label, (int(a[0]) + 8, int(a[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def render(state: ViewState, canvas: MapCanvas, bg: np.ndarray, now: float, panel_w: int = 520) -> np.ndarray:
    """One frame of the window: the map panel and the status panel."""
    import cv2

    img = bg.copy()
    li = state.last_lidar
    if li is not None and li.get("ranges") is not None:  # the scan, placed at the lidar pose (or the rejected match)
        r, a = np.asarray(li["ranges"], dtype=float), np.asarray(li["angles"], dtype=float)
        ok = np.isfinite(r)
        px, py, pyaw = li["pose"] if not li["lost"] else li["match_pose"]
        th = pyaw + a[ok]
        pts = canvas.px(np.stack([px + r[ok] * np.cos(th), py + r[ok] * np.sin(th)], axis=1))
        color = COLORS["scan_lost"] if li["lost"] else COLORS["scan"]
        for p in pts:
            cv2.circle(img, (int(p[0]), int(p[1])), 2, color, -1)
    for name in ("px4", "vio", "lidar"):
        trail = getattr(state, name)
        if len(trail) > 1:
            cv2.polylines(img, [canvas.px([(x, y) for _, x, y in trail])], False, COLORS[name], 2, cv2.LINE_AA)
    if li is not None and li.get("px4") is not None:
        _arrow(img, canvas, li["px4"], COLORS["px4"], "PX4")
    if state.last_vio_pose is not None and now - state.vio_last_t < 1.0:
        _arrow(img, canvas, state.last_vio_pose, COLORS["vio"], "cuVSLAM")
    if li is not None:
        _arrow(img, canvas, li["pose"], COLORS["lidar"], "lidar" + (" (held)" if li["lost"] else ""))

    panel = np.full((canvas.h, panel_w, 3), 250, np.uint8)
    y = 10
    if state.thumb is not None:
        th = state.thumb if state.thumb.ndim == 3 else cv2.cvtColor(state.thumb, cv2.COLOR_GRAY2BGR)
        th = th[: min(th.shape[0], 260), : min(th.shape[1], panel_w - 20)]
        panel[y:y + th.shape[0], 10:10 + th.shape[1]] = th
        if now - state.vio_last_t > 1.0:
            cv2.putText(panel, "camera: no new frames", (20, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS["bad"], 2, cv2.LINE_AA)
        y += th.shape[0] + 12
    else:
        cv2.putText(panel, "camera: waiting for cuVSLAM", (10, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS["warn"], 1, cv2.LINE_AA)
        y += 40

    def line(text, color=COLORS["text"], scale=0.48, bold=1):
        nonlocal y
        y += 20
        cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, bold, cv2.LINE_AA)

    def head(text, color):
        nonlocal y
        y += 8
        line(text, color, 0.56, 2)

    # lidar
    if li is None:
        head("LIDAR  waiting", COLORS["warn"])
    else:
        rate = (len(state.lidar_times) - 1) / max(state.lidar_times[-1] - state.lidar_times[0], 1e-6) if len(state.lidar_times) > 1 else 0.0
        lost_pct = 100.0 * sum(state.lost_recent) / max(len(state.lost_recent), 1)
        status = ("LOST: " + (li.get("reason") or "")) if li["lost"] else "tracking"
        head("LIDAR  " + status, COLORS["bad"] if li["lost"] else COLORS["ok"])
        line("x {:+.3f}  y {:+.3f}  yaw {:6.1f} deg   {:.1f} Hz".format(li["pose"][0], li["pose"][1], math.degrees(li["pose"][2]), rate))
        line("fit {:.1f} cm   beams used {:.0f} %   lost (last 10 s) {:.0f} %".format(
            100 * (li.get("rms_m") or float("nan")), 100 * (li.get("used_frac") or 0.0), lost_pct))
        if state.ages:
            ages = np.array(state.ages)
            p99 = float(np.percentile(ages, 99))
            line("to PX4: sent {}   age p50 {:.0f} ms  p99 {:.0f} ms  (limit 100)".format(li.get("sent", 0), float(np.median(ages)), p99),
                 COLORS["bad"] if p99 > 90 else COLORS["text"])
        else:
            line("to PX4: nothing sent yet", COLORS["warn"])
    # PX4
    if li is None or li.get("px4") is None:
        head("PX4    no position", COLORS["warn"])
    else:
        err = float(np.median(state.px4_err)) if state.px4_err else float("nan")
        fused = state.px4_err and err < 0.10
        head("PX4    " + ("using the lidar" if fused else "not following the lidar"), COLORS["ok"] if fused else COLORS["warn"])
        line("x {:+.3f}  y {:+.3f}  yaw {:6.1f} deg   vs lidar {}".format(li["px4"][0], li["px4"][1], math.degrees(li["px4"][2]), _cm(err)))
        rp = li.get("px4_rp")
        line("roll {:+.1f}  pitch {:+.1f} deg   clock sync {}".format(
            math.degrees(rp[0]) if rp else float("nan"), math.degrees(rp[1]) if rp else float("nan"),
            "converged" if li.get("timesync", 0) >= 520 else "{} / 520".format(li.get("timesync", 0))))
    # cuVSLAM
    vs = state.vio_stats
    fresh = now - state.vio_last_t < 1.0
    if not vs and not fresh:
        head("cuVSLAM  " + (state.vio_info or "starting"), COLORS["warn"])
    else:
        tracked = 100.0 * (vs.get("frames_tracked") or 0) / max(vs.get("frames_attempted") or 1, 1)
        head("cuVSLAM  " + ("tracking" if fresh else "no poses"), COLORS["ok"] if fresh else COLORS["bad"])
        line("{:.0f} fps   tracked {:.1f} %   resets {}   {:.0f} ms/frame".format(
            vs.get("fps") or 0.0, tracked, vs.get("resets") or 0, vs.get("track_ms") or 0.0))
        if state.last_vio_pose is not None:
            lp = state.last_lidar_pose()
            d = math.hypot(state.last_vio_pose[0] - lp[0], state.last_vio_pose[1] - lp[1]) if lp else float("nan")
            line("x {:+.3f}  y {:+.3f}  yaw {:6.1f} deg   vs lidar {}".format(
                state.last_vio_pose[0], state.last_vio_pose[1], math.degrees(state.last_vio_pose[2]), _cm(d)))
        elif state.vio_anchor is None:
            line("not on the map yet: waits for a trusted lidar pose", COLORS["warn"])
        if state.vio_info:
            line(state.vio_info, COLORS["warn"])
    # Jetson
    temps = jetson_temps()
    head("JETSON", COLORS["text"])
    line("  ".join("{} {:.0f} C".format(k, v) for k, v in sorted(temps.items())) or "temperatures unavailable")
    if state.bridge_info:
        line(state.bridge_info, COLORS["warn"])
    y = canvas.h - 12
    cv2.putText(panel, "q quit   r re-anchor cuVSLAM   c clear trails", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
    legend = [("lidar", "lidar"), ("px4", "PX4"), ("vio", "cuVSLAM"), ("scan", "scan")]
    for k, (key, name) in enumerate(legend):
        cv2.putText(img, name, (10 + 90 * k, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[key], 2, cv2.LINE_AA)
    return np.hstack([img, panel])


def gui_worker(q, stop, map_path: Optional[str], scan_z: float, headless_dir: Optional[str], trail_s: float,
               cam_yaw_deg: float) -> None:
    """The window: draws ~10 times a second from ``q``; ``q`` in the window (or ``stop``) ends the run.
    ``map_path`` None: no map, the live mapper's points are drawn as they arrive (:class:`GrowingCanvas`)."""
    import cv2
    import sys

    from course.course_map import CourseMap

    state = ViewState(trail_s=trail_s, cam_yaw=math.radians(cam_yaw_deg))
    canvas = MapCanvas(CourseMap.load(map_path), scan_z) if map_path else GrowingCanvas()
    bg = canvas.background()
    title = "Live localization: lidar + PX4 + cuVSLAM"
    if headless_dir is None:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    last_draw, last_snap = 0.0, 0.0
    try:
        while not stop.is_set():
            try:
                while True:
                    state.handle(q.get(timeout=0.02))
            except queue.Empty:
                pass
            now = time.monotonic()
            if now - last_draw < 0.1:
                continue
            last_draw = now
            if isinstance(canvas, GrowingCanvas):
                poses = [(x, y) for trail in (state.lidar, state.px4, state.vio) for _, x, y in list(trail)[-1:]]
                old = canvas.bounds
                if canvas.update(state.new_points, np.array(poses or [(0.0, 0.0)])) or canvas.bounds != old:
                    bg = canvas.background()
                state.new_points.clear()
            frame = render(state, canvas, bg, now)
            if headless_dir is None:
                cv2.imshow(title, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    stop.set()
                elif key == ord("r"):
                    state.vio_info = "" if state.anchor_vio() else "re-anchor needs a trusted lidar pose and a cuVSLAM pose"
                elif key == ord("c"):
                    for trail in (state.lidar, state.px4, state.vio):
                        trail.clear()
            elif now - last_snap >= 2.0:
                last_snap = now
                cv2.imwrite(str(Path(headless_dir) / "live_view.png"), frame)
    except KeyboardInterrupt:
        stop.set()
    finally:
        if headless_dir is None:
            cv2.destroyAllWindows()
        sys.stdout.flush()
