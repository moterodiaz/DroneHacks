"""Live view of a tracking run: camera image with the tracked features, a top-down map of the
trajectory, a height trace and the state of the tracker, drawn into one BGR image with OpenCV.

:class:`LiveView` only keeps state and draws; showing the image (``cv2.imshow``) and feeding it
(:class:`cuvslam_oak.bridge.BridgeClient`) is the caller's job, so the drawing can be tested
without a display.

Frames: the tracker reports ``world_from_rig`` with the world = the left camera at the first
tracked frame (x right, y down, z forward).  In ``vio`` mode the view is *levelled* with the
gravity the tracker reports (rotated so gravity points along +y), so "height" is real height even
when the camera started tilted; in ``stereo`` mode there is no gravity and the start pose defines
"level".
"""
from __future__ import annotations

import collections
import math
import time
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import bridge as _bridge

__all__ = ["LiveView", "quaternion_matrix", "levelling_rotation", "nice_grid"]

# BGR
_BG = (24, 24, 24)
_PANEL = (34, 34, 34)
_GRID = (62, 62, 62)
_AXIS = (110, 110, 110)
_TEXT = (230, 230, 230)
_DIM = (150, 150, 150)
_GOOD = (110, 200, 90)
_BAD = (70, 70, 235)
_WARN = (60, 180, 240)
_TRAIL = (235, 170, 60)
_COAST = (60, 190, 240)          # estimate without vision (orange)
_SLAM = (220, 120, 230)          # SLAM (loop-closed) trajectory (magenta)
_FEATURE = (90, 230, 120)

_SPANS = (2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0, 96.0, 128.0, 256.0, 512.0)


def quaternion_matrix(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix of a unit quaternion ``(x, y, z, w)``."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def levelling_rotation(gravity_world: Sequence[float]) -> np.ndarray:
    """The smallest rotation that turns *gravity_world* onto +y (down); identity for a zero vector."""
    g = np.asarray(gravity_world, dtype=np.float64)
    norm = float(np.linalg.norm(g))
    if norm < 1e-9:
        return np.eye(3)
    g = g / norm
    down = np.array([0.0, 1.0, 0.0])
    axis = np.cross(g, down)
    s = float(np.linalg.norm(axis))
    c = float(np.dot(g, down))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = axis / s
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def nice_grid(span: float) -> float:
    """Grid spacing (m) that puts 4 to 10 lines across *span*."""
    for step in (0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0):
        if span / step <= 10.0:
            return step
    return 200.0


class LiveView:
    """State and drawing of the live view.

    Feed it with :meth:`add_image` (every frame handed to the bridge) and :meth:`handle` (what
    ``BridgeClient.poll()`` returned); :meth:`render` returns the picture.  ``poses`` holds the
    rows as received (``(ts_ns, x, y, z, qx, qy, qz, qw)``, not levelled) for a TUM file.
    """

    #: Motion smaller than this since the last counted point does not count as travelled distance.
    TRAVEL_STEP_M = 0.02

    def __init__(self, mode: str, size: Tuple[int, int], *, title: str = "", duration_s: Optional[float] = None,
                 canvas: Tuple[int, int] = (1280, 720), level: bool = True, show_features: bool = True,
                 warm_after_s: Optional[float] = None) -> None:
        self.mode = mode
        self.size = (int(size[0]), int(size[1]))
        self.title = title
        self.duration_s = duration_s
        self.warm_after_s = warm_after_s            # camera on-time after which the clock turns orange
        self.canvas = (int(canvas[0]), int(canvas[1]))
        self.level = bool(level)
        self.show_features = bool(show_features)
        self.status = "starting"
        self.started = time.monotonic()
        self.stream_started: Optional[float] = None
        self.poses: List[Tuple[int, float, float, float, float, float, float, float]] = []
        self.coasted: List[bool] = []                   # one flag per pose
        self.slam_poses: List[Tuple[int, float, float, float, float, float, float, float]] = []
        self.skips: List[Tuple[int, int]] = []          # (ts_ns, number of poses at that moment)
        self.stats: Dict[str, Any] = {}
        self.summary: Optional[Dict[str, Any]] = None
        self.error: Optional[str] = None
        self.bridge = {"sent": 0, "dropped": 0, "queued": 0}
        self.latency_ms: Optional[float] = None
        self.engine_ms: Optional[float] = None
        self.last_event: str = "none"                   # 'pose' or 'skip'
        self.last_features: List[int] = []
        self.last_pose_ts: Optional[int] = None
        self._gravity_sum = np.zeros(3)
        self._gravity_n = 0
        self._images: "collections.OrderedDict[int, np.ndarray]" = collections.OrderedDict()
        self._captured: "collections.OrderedDict[int, float]" = collections.OrderedDict()
        self._latest_image: Optional[np.ndarray] = None
        self._pose_times: Deque[float] = collections.deque(maxlen=90)
        self._anchor: Optional[np.ndarray] = None       # last point that counted towards `travelled`
        self.travelled = 0.0
        self._cam_size = (self.canvas[0] // 2, int(round(self.canvas[0] // 2 * self.size[1] / self.size[0])))

    # ---------------------------------------------------------------- input
    def add_image(self, ts_ns: int, image: np.ndarray, captured: Optional[float] = None) -> None:
        """Remember the left image of frame *ts_ns* (kept small) and when it was captured."""
        import cv2

        small = cv2.resize(image, self._cam_size, interpolation=cv2.INTER_AREA)
        self._latest_image = small
        self._images[int(ts_ns)] = small
        self._captured[int(ts_ns)] = time.monotonic() if captured is None else float(captured)
        if self.stream_started is None:
            self.stream_started = time.monotonic()
        while len(self._images) > 120:
            self._images.popitem(last=False)
        while len(self._captured) > 600:
            self._captured.popitem(last=False)

    def set_bridge(self, sent: int, dropped: int, queued: int) -> None:
        self.bridge = {"sent": int(sent), "dropped": int(dropped), "queued": int(queued)}

    def handle(self, events: Iterable[Tuple[int, Any]]) -> None:
        for mtype, value in events:
            if mtype == _bridge.MSG_POSE:
                ts = int(value["ts"])
                p, q = value["p"], value["q"]
                self.poses.append((ts, float(p[0]), float(p[1]), float(p[2]),
                                   float(q[0]), float(q[1]), float(q[2]), float(q[3])))
                coasted = bool(value.get("coast"))
                self.coasted.append(coasted)
                if value.get("s") is not None and value.get("sq") is not None:
                    sp, sq = value["s"], value["sq"]
                    self.slam_poses.append((ts, float(sp[0]), float(sp[1]), float(sp[2]),
                                            float(sq[0]), float(sq[1]), float(sq[2]), float(sq[3])))
                self.last_event = "coast" if coasted else "pose"
                self.last_pose_ts = ts
                here = np.array([float(p[0]), float(p[1]), float(p[2])])
                if self._anchor is None:
                    self._anchor = here
                else:
                    moved = float(np.linalg.norm(here - self._anchor))
                    if moved >= self.TRAVEL_STEP_M:      # below it the motion is estimator jitter
                        self.travelled += moved
                        self._anchor = here
                self.last_features = list(value.get("f") or [])
                if not coasted:
                    self.engine_ms = float(value.get("ms") or 0.0)
                g = value.get("g")
                if g is not None and all(math.isfinite(float(v)) for v in g):
                    world = quaternion_matrix(q) @ np.asarray(g, dtype=np.float64)
                    if 5.0 < float(np.linalg.norm(world)) < 15.0:
                        self._gravity_sum += world
                        self._gravity_n += 1
                received = float(value.get("_received", time.monotonic()))
                self._pose_times.append(received)
                captured = self._captured.get(ts)
                if captured is not None:
                    ms = (received - captured) * 1e3
                    self.latency_ms = ms if self.latency_ms is None else 0.9 * self.latency_ms + 0.1 * ms
            elif mtype == _bridge.MSG_SKIP:
                self.skips.append((int(value["ts"]), len(self.poses)))
                self.last_event = "skip"
            elif mtype == _bridge.MSG_STATS:
                self.stats = value
            elif mtype == _bridge.MSG_SUMMARY:
                self.summary = value
                self.status = "ended"
            elif mtype == _bridge.MSG_ERROR:
                self.error = str(value.get("message", "unknown error"))
                self.status = "error"

    # ---------------------------------------------------------------- derived
    @property
    def levelled(self) -> bool:
        return self.level and self._gravity_n >= 15

    def rotation(self) -> np.ndarray:
        return levelling_rotation(self._gravity_sum) if self.levelled else np.eye(3)

    def positions(self) -> np.ndarray:
        """``(N, 3)`` positions in the display frame (levelled when gravity is known)."""
        if not self.poses:
            return np.zeros((0, 3))
        return np.asarray([row[1:4] for row in self.poses], dtype=np.float64) @ self.rotation().T

    def slam_positions(self) -> np.ndarray:
        if not self.slam_poses:
            return np.zeros((0, 3))
        return np.asarray([row[1:4] for row in self.slam_poses], dtype=np.float64) @ self.rotation().T

    def path_length(self) -> float:
        """Distance travelled, counted in steps of :attr:`TRAVEL_STEP_M` so that the millimetre jitter of
        a camera at rest does not add up (a still camera \"travelled\" 0.34 m in 20 s without it)."""
        return self.travelled

    def pose_rate(self) -> float:
        if len(self._pose_times) < 2:
            return 0.0
        span = self._pose_times[-1] - self._pose_times[0]
        return (len(self._pose_times) - 1) / span if span > 0 else 0.0

    def remaining_s(self) -> Optional[float]:
        if self.duration_s is None or self.duration_s <= 0 or self.stream_started is None:
            return None
        return max(0.0, self.duration_s - (time.monotonic() - self.stream_started))

    # ---------------------------------------------------------------- drawing
    def render(self) -> np.ndarray:
        import cv2

        width, height = self.canvas
        img = np.full((height, width, 3), _BG, dtype=np.uint8)
        cam_w, cam_h = self._cam_size
        positions = self.positions()
        self._draw_camera(cv2, img, 0, 0, cam_w, cam_h)
        self._draw_status(cv2, img, 0, cam_h, cam_w, height - cam_h, positions)
        self._draw_map(cv2, img, cam_w, 0, width - cam_w, height, positions)
        return img

    @staticmethod
    def _text(cv2: Any, img: np.ndarray, text: str, org: Tuple[int, int], color: Tuple[int, int, int] = _TEXT,
              scale: float = 0.5, thickness: int = 1) -> None:
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    def _state(self) -> Tuple[str, Tuple[int, int, int]]:
        if self.error:
            return "ERROR", _BAD
        if self.status == "ended" or self.summary is not None:
            return "ENDED", _DIM
        if not self.poses and not self.skips:
            return self.status.upper(), _WARN
        if self.last_event == "skip":
            return "NO POSE (guard / loss)", _BAD
        if self.last_event == "coast":
            return "COASTING (estimate, no vision)", _COAST
        return "TRACKING", _GOOD

    def _draw_camera(self, cv2: Any, img: np.ndarray, x0: int, y0: int, w: int, h: int) -> None:
        picture = None
        if self.last_pose_ts is not None and self.last_event in ("pose", "coast"):
            picture = self._images.get(self.last_pose_ts)
        with_features = picture is not None
        if picture is None:
            picture = self._latest_image
        if picture is None:
            cv2.rectangle(img, (x0, y0), (x0 + w - 1, y0 + h - 1), _PANEL, -1)
            self._text(cv2, img, "waiting for the first frame", (x0 + 20, y0 + h // 2), _DIM, 0.6)
        else:
            img[y0:y0 + h, x0:x0 + w] = cv2.cvtColor(picture, cv2.COLOR_GRAY2BGR)
            if with_features and self.show_features and self.last_features:
                sx, sy = w / self.size[0], h / self.size[1]
                pts = np.asarray(self.last_features, dtype=np.float64).reshape(-1, 2)
                for u, v in pts:
                    cv2.circle(img, (x0 + int(u * sx), y0 + int(v * sy)), 2, _FEATURE, -1, cv2.LINE_AA)
        _label, color = self._state()
        cv2.rectangle(img, (x0, y0), (x0 + w - 1, y0 + h - 1), color, 2)

    def _draw_status(self, cv2: Any, img: np.ndarray, x0: int, y0: int, w: int, h: int, positions: np.ndarray) -> None:
        cv2.rectangle(img, (x0, y0), (x0 + w - 1, y0 + h - 1), _PANEL, -1)
        label, color = self._state()
        self._text(cv2, img, label, (x0 + 14, y0 + 30), color, 0.8, 2)
        remaining = self.remaining_s()
        if remaining is not None and self.status != "ended":
            self._text(cv2, img, "auto stop in {:3.0f} s".format(remaining), (x0 + w - 190, y0 + 28),
                       _WARN if remaining < 15 else _DIM, 0.55)
        elif self.stream_started is not None and self.status != "ended":
            on_s = time.monotonic() - self.stream_started
            warm = self.warm_after_s is not None and on_s >= self.warm_after_s
            self._text(cv2, img, "{} {:d}:{:02d}".format("camera warm, stop soon" if warm else "on for",
                                                         int(on_s // 60), int(on_s % 60)),
                       (x0 + w - (250 if warm else 150), y0 + 28), _WARN if warm else _DIM, 0.55)
        lines: List[Tuple[str, Tuple[int, int, int]]] = []
        if self.error:
            lines.append((self.error[:78], _BAD))
        if len(positions):
            p = positions[-1]
            lines.append(("position   right {:+6.2f} m   forward {:+6.2f} m   up {:+6.2f} m".format(p[0], p[2], -p[1]),
                          _TEXT))
            lines.append(("from start {:5.2f} m      travelled {:6.2f} m".format(
                float(np.linalg.norm(p)), self.path_length()), _TEXT))
        else:
            lines.append(("position   -", _DIM))
            lines.append(("from start -", _DIM))
        guarded = self.stats.get("frames_guarded", 0)
        failed = self.stats.get("frames_failed", 0)
        resets = self.stats.get("resets", 0)
        n_coast = sum(1 for flag in self.coasted if flag)
        lines.append(("poses {:5d} ({} coasted)   skipped {:4d} (guard {}, lost {})   engine restarts {}".format(
            len(self.poses) - n_coast, n_coast, len(self.skips), guarded, failed, resets),
            _TEXT if not self.skips and not n_coast else _WARN))
        if self.slam_poses:
            sp = self.slam_positions()[-1]
            lines.append(("SLAM: {} poses, {} loop closure(s)   position right {:+.2f}  forward {:+.2f}  from start {:.2f} m".format(
                len(self.slam_poses), self.stats.get("loop_closures", 0), sp[0], sp[2], float(np.linalg.norm(sp))),
                _SLAM))
        lines.append(("bridge: sent {}   dropped {}   waiting {}".format(
            self.bridge["sent"], self.bridge["dropped"], self.bridge["queued"]),
            _TEXT if not self.bridge["dropped"] else _WARN))
        lines.append(("rate {:4.1f} poses/s   engine {} ms   delay {} ms".format(
            self.pose_rate(), "-" if self.engine_ms is None else "{:.1f}".format(self.engine_ms),
            "-" if self.latency_ms is None else "{:.0f}".format(self.latency_ms)), _TEXT))
        lines.append(("{}   {}x{}   {}".format(
            "stereo + IMU" if self.mode == "vio" else "stereo only", self.size[0], self.size[1],
            "levelled with gravity" if self.levelled else "not levelled (start pose = level)"), _DIM))
        y = y0 + 58
        for text, colour in lines:
            self._text(cv2, img, text, (x0 + 14, y), colour, 0.48)
            y += 21
        self._draw_height(cv2, img, x0 + 14, y + 2, w - 28, max(40, y0 + h - y - 30), positions)
        self._text(cv2, img, "q quit   c clear trail   f features", (x0 + 14, y0 + h - 10), _DIM, 0.42)

    def _draw_height(self, cv2: Any, img: np.ndarray, x0: int, y0: int, w: int, h: int, positions: np.ndarray) -> None:
        cv2.rectangle(img, (x0, y0), (x0 + w - 1, y0 + h - 1), _BG, -1)
        self._text(cv2, img, "height (m), last 60 s", (x0 + 6, y0 + 14), _DIM, 0.4)
        if len(positions) < 2:
            return
        ts = np.asarray([row[0] for row in self.poses], dtype=np.float64) / 1e9
        keep = ts >= ts[-1] - 60.0
        t, up = ts[keep], -positions[keep, 1]
        half = max(0.5, float(np.max(np.abs(up))) * 1.15)
        zero = y0 + h // 2
        cv2.line(img, (x0, zero), (x0 + w - 1, zero), _GRID, 1)
        self._text(cv2, img, "+{:.1f}".format(half), (x0 + w - 44, y0 + 14), _DIM, 0.4)
        self._text(cv2, img, "-{:.1f}".format(half), (x0 + w - 44, y0 + h - 5), _DIM, 0.4)
        xs = x0 + ((t - (t[-1] - 60.0)) / 60.0 * (w - 1)).astype(np.int32)
        ys = zero - (up / half * (h / 2 - 3)).astype(np.int32)
        cv2.polylines(img, [np.stack([xs, ys], axis=1).reshape(-1, 1, 2)], False, _TRAIL, 1, cv2.LINE_AA)

    def _draw_map(self, cv2: Any, img: np.ndarray, x0: int, y0: int, w: int, h: int, positions: np.ndarray) -> None:
        cv2.rectangle(img, (x0, y0), (x0 + w - 1, y0 + h - 1), _PANEL, -1)
        title = "top view" + (" - " + self.title if self.title else "")
        self._text(cv2, img, title, (x0 + 14, y0 + 24), _TEXT, 0.55)
        top, margin = 40, 16
        area_w, area_h = w - 2 * margin, h - top - margin
        if len(positions):
            lo = np.minimum(positions[:, [0, 2]].min(axis=0), 0.0)
            hi = np.maximum(positions[:, [0, 2]].max(axis=0), 0.0)
        else:
            lo, hi = np.zeros(2), np.zeros(2)
        need = float(max(hi[0] - lo[0], (hi[1] - lo[1]) * area_w / area_h)) * 1.2
        span = next((s for s in _SPANS if s >= need), _SPANS[-1])     # metres across the panel width
        step = nice_grid(span)
        scale = area_w / span                                           # pixels per metre
        centre = np.round((lo + hi) / 2.0 / step) * step
        cx, cy = x0 + margin + area_w / 2.0, y0 + top + area_h / 2.0

        def to_px(x: float, z: float) -> Tuple[int, int]:
            return int(round(cx + (x - centre[0]) * scale)), int(round(cy - (z - centre[1]) * scale))

        left, right = x0 + margin, x0 + margin + area_w
        upper, lower = y0 + top, y0 + top + area_h
        half_w, half_h = span / 2.0, span / 2.0 * area_h / area_w
        first = math.ceil((centre[0] - half_w) / step) * step
        for i in range(int(2 * half_w / step) + 2):
            x = first + i * step
            px = to_px(x, 0.0)[0]
            if left <= px <= right:
                cv2.line(img, (px, upper), (px, lower), _AXIS if abs(x) < 1e-9 else _GRID, 1)
        first = math.ceil((centre[1] - half_h) / step) * step
        for i in range(int(2 * half_h / step) + 2):
            z = first + i * step
            py = to_px(0.0, z)[1]
            if upper <= py <= lower:
                cv2.line(img, (left, py), (right, py), _AXIS if abs(z) < 1e-9 else _GRID, 1)
        cv2.rectangle(img, (left, upper), (right, lower), _GRID, 1)
        self._text(cv2, img, "grid {:g} m".format(step), (right - 96, y0 + 24), _DIM, 0.48)
        self._text(cv2, img, "forward", (to_px(0.0, 0.0)[0] + 6, upper + 16), _DIM, 0.42)

        ox, oy = to_px(0.0, 0.0)
        cv2.drawMarker(img, (ox, oy), _TEXT, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
        if not len(positions):
            return
        pts = np.asarray([to_px(p[0], p[2]) for p in positions], dtype=np.int32)
        if len(pts) >= 2:
            # a polyline per run of measured / coasted segments; a segment takes the colour of its end point
            flags = (list(self.coasted) + [False] * len(pts))[:len(pts)]
            start = 0
            while start < len(pts) - 1:
                flag = flags[start + 1]
                end = start + 1
                while end + 1 < len(pts) and flags[end + 1] == flag:
                    end += 1
                cv2.polylines(img, [pts[start:end + 1].reshape(-1, 1, 2)], False, _COAST if flag else _TRAIL, 2,
                              cv2.LINE_AA)
                start = end
        for _ts, index in self.skips[-400:]:
            if 0 < index <= len(pts):
                cv2.circle(img, tuple(int(v) for v in pts[index - 1]), 3, _BAD, -1, cv2.LINE_AA)
        slam_pos = self.slam_positions()
        if len(slam_pos) >= 2:
            spts = np.asarray([to_px(p[0], p[2]) for p in slam_pos], dtype=np.int32)
            cv2.polylines(img, [spts.reshape(-1, 1, 2)], False, _SLAM, 1, cv2.LINE_AA)
            cv2.circle(img, tuple(int(v) for v in spts[-1]), 5, _SLAM, 1, cv2.LINE_AA)
        forward = self.rotation() @ (quaternion_matrix(self.poses[-1][4:8]) @ np.array([0.0, 0.0, 1.0]))
        heading = math.atan2(forward[0], forward[2])
        tip = np.array([math.sin(heading), -math.cos(heading)])
        side = np.array([-tip[1], tip[0]])
        here = pts[-1].astype(np.float64)
        triangle = np.array([here + 16 * tip, here - 8 * tip + 8 * side, here - 8 * tip - 8 * side], dtype=np.int32)
        _label, color = self._state()
        cv2.fillConvexPoly(img, triangle, color, cv2.LINE_AA)
        self._text(cv2, img, "measured", (left + 8, lower - 8), _TRAIL, 0.42)
        self._text(cv2, img, "coasted", (left + 90, lower - 8), _COAST, 0.42)
        if self.slam_poses:
            self._text(cv2, img, "SLAM", (left + 160, lower - 8), _SLAM, 0.42)

    # ---------------------------------------------------------------- actions
    def clear_trail(self) -> None:
        """Forget the drawn trajectory (the tracker keeps its origin; the rows for a TUM file are kept apart)."""
        self._cleared = getattr(self, "_cleared", []) + self.poses
        self._cleared_flags = getattr(self, "_cleared_flags", []) + self.coasted
        self.poses = []
        self.coasted = []
        self.slam_poses = []
        self.skips = []
        self._anchor = None
        self.travelled = 0.0

    def all_poses(self) -> List[Tuple[int, float, float, float, float, float, float, float]]:
        """Every pose received, also those cleared from the display."""
        return list(getattr(self, "_cleared", [])) + list(self.poses)

    def all_coasted(self) -> List[bool]:
        """The ``coasted`` flag of every pose of :meth:`all_poses`."""
        return list(getattr(self, "_cleared_flags", [])) + list(self.coasted)
