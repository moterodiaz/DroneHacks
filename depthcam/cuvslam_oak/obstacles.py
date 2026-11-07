"""Basic obstacle detection from one OAK-D S2 depth frame: depth image -> bearing sectors.

What it does
------------
:func:`detect` reduces one ``StereoDepth.depth`` frame (uint16 millimetres, 0 = invalid) to
the horizontal range of the nearest obstacle in each ``sector_deg`` bearing sector around the
vehicle, plus the nearest obstacle in a straight corridor ahead.  It is the same reduction the
ArduPilot RealSense companion script and PX4 Collision Prevention use (one distance per 5 degree
ray), done in numpy so it runs unchanged on the laptop and on the Jetson:

1. back-project every ``stride``-th pixel with a depth between ``min_range_m`` and
   ``evidence_range_m`` through the depth frame's intrinsics (OpenCV camera frame: x right,
   y down, z forward, metres);
2. rotate the points into a *level* frame whose +y axis is gravity (:func:`level_rotation`),
   so the height band means the same thing when the camera pitches or rolls;
3. keep the points within ``band_below_m`` below / ``band_above_m`` above the camera, which
   drops the floor and the ceiling at flight altitude;
4. bin them by horizontal bearing ``atan2(x, z)``: 0 = straight ahead, positive = to the right,
   i.e. clockwise seen from above, the MAVLink ``OBSTACLE_DISTANCE`` convention; sector ``i``
   is centred on ``i * sector_deg`` (sector 0 straight ahead, the last one just left of it);
5. per sector take the nearest *supported* range: the smallest horizontal range ``r`` with at
   least ``min_points`` points in ``[r, r + cluster_m]``.  Isolated speckles never count; a
   2.5 cm pole a few pixel columns wide does (about 40 points at 3 m with the defaults).

Unknown is not free
-------------------
The S2 has no IR projector, so a blank wall, foam pad or evenly painted box gives holes, not
depth.  Each sector therefore has three states in :attr:`ObstacleScan.distances_m`:

* a finite range (m): a supported surface at or closer than ``max_range_m``;
* ``inf`` (clear): the nearest supported surface is farther than ``max_range_m``, so there is
  positive evidence of free space out to that range;
* ``nan`` (unknown): no supported surface at all - holes, too few points, or a sector outside
  the camera's ~77 degree horizontal field of view.  Treat it as blocked.

The corridor (:attr:`ObstacleScan.ahead_m`) uses the same three states for the points with
``|x| <= corridor_half_width_m`` in the level frame, measured as forward distance ``z``.

Levelling
---------
``down`` is the gravity direction in the depth camera frame.  :class:`GravityEstimator`
low-passes the OAK's accelerometer into one, which is right for hand-held and bench tests.  On a
flying multirotor the accelerometer measures thrust, not gravity (it reads "level" while the
vehicle is pitched to accelerate), so in flight take ``down`` from the PX4 attitude or the
cuVSLAM pose instead.  ``down=None`` assumes a level camera.

Nothing here imports ``depthai`` or ``cv2``; the scan converts itself to the MAVLink
``OBSTACLE_DISTANCE`` field layout (:meth:`ObstacleScan.to_obstacle_distance`) without
importing ``pymavlink`` either.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "MAVLINK_SECTORS",
    "MAV_DISTANCE_SENSOR_UNKNOWN",
    "MAV_FRAME_BODY_FRD",
    "UINT16_MAX",
    "GravityEstimator",
    "ObstacleConfig",
    "ObstacleScan",
    "detect",
    "down_from_accel",
    "down_from_tilt",
    "level_rotation",
    "tilt_deg",
]

#: ``OBSTACLE_DISTANCE.distances`` is a fixed uint16[72] array.
MAVLINK_SECTORS = 72
#: MAVLink "unknown / not used" distance value.
UINT16_MAX = 65535
#: ``MAV_FRAME_BODY_FRD``: x forward, y right, z down; angles clockwise from forward.
MAV_FRAME_BODY_FRD = 12
#: ``MAV_DISTANCE_SENSOR_UNKNOWN``: stereo is none of laser / ultrasound / infrared / radar.
MAV_DISTANCE_SENSOR_UNKNOWN = 4

#: Gravity direction of a level camera in the OpenCV camera frame (y down).
_DOWN_LEVEL = (0.0, 1.0, 0.0)
#: Refuse to level a camera looking within this angle of straight up or down: its forward
#: direction has no horizontal component worth binning by bearing.
_MIN_FORWARD_ELEVATION_MARGIN_DEG = 10.0
#: Offset that keeps the per-sector windows of the combined sort key from overlapping.
_SECTOR_KEY_STRIDE_M = 1.0e4


@dataclass(frozen=True)
class ObstacleConfig:
    """Tuning for :func:`detect`.  The defaults suit 640x400 depth at decimation 1 on a drone.

    ``min_points`` counts *sampled* points (after ``stride``), so halve it when doubling
    ``stride`` would otherwise make thin obstacles vanish, and raise it if speckles survive.
    """

    sector_deg: float = 5.0
    #: Closer depth is discarded; 0.32 m is the 400P stereo floor of this unit (normal disparity).
    min_range_m: float = 0.3
    #: Supported surfaces farther than this make a sector "clear" (``inf``).
    max_range_m: float = 5.0
    #: Valid depth up to this range still counts as evidence; farther depth is discarded.
    evidence_range_m: float = 8.0
    band_below_m: float = 0.3
    band_above_m: float = 0.3
    stride: int = 2
    min_points: int = 15
    cluster_m: float = 0.15
    corridor_half_width_m: float = 0.35

    def __post_init__(self) -> None:
        if not (0.0 < self.sector_deg <= 90.0):
            raise ValueError("sector_deg must be in (0, 90], got {}".format(self.sector_deg))
        n = 360.0 / self.sector_deg
        if abs(n - round(n)) > 1e-9:
            raise ValueError("sector_deg must divide 360, got {}".format(self.sector_deg))
        if not (0.0 <= self.min_range_m < self.max_range_m <= self.evidence_range_m):
            raise ValueError("need 0 <= min_range_m < max_range_m <= evidence_range_m, got {} / {} / {}".format(
                self.min_range_m, self.max_range_m, self.evidence_range_m))
        if self.band_below_m < 0.0 or self.band_above_m < 0.0:
            raise ValueError("band_below_m and band_above_m must be >= 0")
        if self.stride < 1 or self.min_points < 1:
            raise ValueError("stride and min_points must be >= 1")
        if self.cluster_m <= 0.0 or self.corridor_half_width_m <= 0.0:
            raise ValueError("cluster_m and corridor_half_width_m must be > 0")

    @property
    def n_sectors(self) -> int:
        return int(round(360.0 / self.sector_deg))


@dataclass
class ObstacleScan:
    """Result of :func:`detect`; see the module docstring for the ``nan`` / ``inf`` states."""

    #: Horizontal range per sector in metres: finite = obstacle, ``inf`` = clear, ``nan`` = unknown.
    distances_m: np.ndarray
    #: In-band points per sector (diagnostics; a sector needs ``min_points`` in one cluster).
    support: np.ndarray
    #: Forward distance to the nearest obstacle in the corridor ahead (same three states).
    ahead_m: float
    sector_deg: float
    min_range_m: float
    max_range_m: float
    #: Valid depth pixels / sampled pixels, before the height band.
    valid_fraction: float
    points_in_band: int

    @property
    def n_sectors(self) -> int:
        return int(self.distances_m.size)

    def bearings_deg(self) -> np.ndarray:
        """Centre bearing of every sector in degrees, wrapped to [-180, 180)."""
        b = np.arange(self.n_sectors, dtype=np.float64) * self.sector_deg
        return (b + 180.0) % 360.0 - 180.0

    def nearest(self) -> Optional[Tuple[float, float]]:
        """``(range_m, bearing_deg)`` of the closest obstacle sector, or ``None`` if there is none."""
        finite = np.isfinite(self.distances_m)
        if not finite.any():
            return None
        i = int(np.argmin(np.where(finite, self.distances_m, np.inf)))
        return float(self.distances_m[i]), float(self.bearings_deg()[i])

    def counts(self) -> Dict[str, int]:
        """Number of obstacle / clear / unknown sectors (unknown includes those out of view)."""
        d = self.distances_m
        return {
            "obstacle": int(np.isfinite(d).sum()),
            "clear": int(np.isposinf(d).sum()),
            "unknown": int(np.isnan(d).sum()),
        }

    def to_obstacle_distance(self, time_usec: int = 0) -> Dict[str, Any]:
        """The scan as MAVLink ``OBSTACLE_DISTANCE`` fields (``pymavlink`` keyword names).

        Obstacles are centimetres clipped to ``[min_distance, max_distance]``, clear sectors are
        ``max_distance + 1`` ("no obstacle"), unknown sectors and the padding up to 72 entries
        are ``UINT16_MAX``.  Element 0 is straight ahead (``angle_offset`` 0) and the frame is
        ``MAV_FRAME_BODY_FRD``, so the camera must face the vehicle's forward axis.
        """
        if self.n_sectors > MAVLINK_SECTORS:
            raise ValueError("OBSTACLE_DISTANCE holds at most {} sectors, this scan has {} (sector_deg {})".format(
                MAVLINK_SECTORS, self.n_sectors, self.sector_deg))
        min_cm = int(round(self.min_range_m * 100.0))
        max_cm = int(round(self.max_range_m * 100.0))
        out = np.full(MAVLINK_SECTORS, UINT16_MAX, dtype=np.int64)
        d = self.distances_m
        finite = np.isfinite(d)
        out[: self.n_sectors][finite] = np.clip(np.rint(d[finite] * 100.0), min_cm, max_cm).astype(np.int64)
        out[: self.n_sectors][np.isposinf(d)] = max_cm + 1
        return {
            "time_usec": int(time_usec),
            "sensor_type": MAV_DISTANCE_SENSOR_UNKNOWN,
            "distances": [int(v) for v in out],
            "increment": int(round(self.sector_deg)),
            "min_distance": min_cm,
            "max_distance": max_cm,
            "increment_f": float(self.sector_deg),
            "angle_offset": 0.0,
            "frame": MAV_FRAME_BODY_FRD,
        }


# --------------------------------------------------------------------------------------
# Levelling
# --------------------------------------------------------------------------------------

def _unit(v: Any, what: str) -> np.ndarray:
    a = np.asarray(v, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(a))
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError("{} must be a finite non-zero 3-vector, got {}".format(what, a.tolist()))
    return a / n


def level_rotation(down_cam: Optional[Sequence[float]] = None) -> np.ndarray:
    """``level_from_camera`` (3x3) for gravity direction *down_cam* in the camera frame.

    The level frame keeps the OpenCV axis names: +y is gravity, +z the horizontal projection of
    the camera's optical axis, +x = y cross z (to the right).  ``None`` gives the identity.
    """
    d = _unit(_DOWN_LEVEL if down_cam is None else down_cam, "down_cam")
    optical = np.array([0.0, 0.0, 1.0])
    fwd = optical - float(optical @ d) * d
    if np.linalg.norm(fwd) < math.sin(math.radians(_MIN_FORWARD_ELEVATION_MARGIN_DEG)):
        raise ValueError("the camera looks within {:.0f} deg of straight up or down (down_cam {}); "
                         "bearings are undefined".format(_MIN_FORWARD_ELEVATION_MARGIN_DEG, d.round(3).tolist()))
    fwd /= np.linalg.norm(fwd)
    right = np.cross(d, fwd)
    return np.stack([right, d, fwd])


def down_from_accel(accel_cam: Sequence[float]) -> np.ndarray:
    """Gravity direction from one accelerometer reading already rotated into the camera frame.

    An accelerometer at rest reads the reaction to gravity (``+9.81 m/s^2`` pointing up), so
    down is the negated, normalised reading.
    """
    return -_unit(accel_cam, "accel_cam")


def down_from_tilt(pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Gravity direction for a camera pitched nose-down by *pitch_deg* and rolled right-side-down by *roll_deg*."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return np.array([math.sin(r) * math.cos(p), math.cos(r) * math.cos(p), math.sin(p)])


def tilt_deg(down_cam: Optional[Sequence[float]]) -> float:
    """Angle between *down_cam* and a level camera's gravity direction, in degrees."""
    if down_cam is None:
        return 0.0
    d = _unit(down_cam, "down_cam")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(d[1])))))


class GravityEstimator:
    """Low-pass accelerometer samples (camera frame, m/s^2) into a gravity direction.

    First-order filter with time constant *time_constant_s* on the device timestamps.  Valid
    while the camera is not accelerating for long, i.e. hand-held and bench tests; see the
    module docstring for why a flying multirotor needs its attitude from PX4 or cuVSLAM.
    """

    def __init__(self, time_constant_s: float = 0.3) -> None:
        if time_constant_s <= 0.0:
            raise ValueError("time_constant_s must be > 0")
        self.time_constant_s = float(time_constant_s)
        self._accel: Optional[np.ndarray] = None
        self._t_s: Optional[float] = None

    def update(self, accel_cam: Sequence[float], t_s: float) -> None:
        a = np.asarray(accel_cam, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(a)):
            return
        if self._accel is None or self._t_s is None:
            self._accel, self._t_s = a, float(t_s)
            return
        dt = float(t_s) - self._t_s
        if dt <= 0.0:
            return
        alpha = 1.0 - math.exp(-dt / self.time_constant_s)
        self._accel = self._accel + alpha * (a - self._accel)
        self._t_s = float(t_s)

    @property
    def down(self) -> Optional[np.ndarray]:
        """Current gravity direction, ``None`` before the first sample."""
        return None if self._accel is None else down_from_accel(self._accel)


# --------------------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------------------

def _nearest_supported(keys: np.ndarray, values: np.ndarray, n_keys: int, min_points: int,
                       window: float) -> np.ndarray:
    """Per key, the smallest value with at least *min_points* values in ``[value, value + window]``.

    Returns ``nan`` for keys with no such value.  Vectorised: one sort on ``key * stride +
    value`` and one ``searchsorted``; *stride* keeps each key's windows apart.
    """
    out = np.full(n_keys, np.nan)
    if values.size < min_points:
        return out
    order = np.lexsort((values, keys))
    k = keys[order]
    v = values[order].astype(np.float64)
    combined = k * _SECTOR_KEY_STRIDE_M + v
    end = np.searchsorted(combined, combined + window, side="right")
    supported = (end - np.arange(combined.size)) >= min_points
    first_keys, first_idx = np.unique(k[supported], return_index=True)
    out[first_keys] = v[supported][first_idx]
    return out


def _three_state(nearest: np.ndarray, max_range_m: float) -> np.ndarray:
    """Map nearest supported ranges to obstacle (finite) / clear (inf) / unknown (nan)."""
    out = np.full(nearest.shape, np.nan)
    known = np.isfinite(nearest)
    out[known & (nearest <= max_range_m)] = nearest[known & (nearest <= max_range_m)]
    out[known & (nearest > max_range_m)] = np.inf
    return out


def detect(depth_mm: np.ndarray, intrinsics: Any, config: Optional[ObstacleConfig] = None, *,
           down: Optional[Sequence[float]] = None) -> ObstacleScan:
    """Reduce one depth frame to per-sector obstacle ranges (see the module docstring).

    *depth_mm*: 2-D depth in millimetres, 0 / NaN = invalid (``StereoDepth.depth`` or a
    ``05_record.py`` depth PNG).  *intrinsics*: the 3x3 camera matrix of that frame
    (``ImgFrame.getTransformation().getIntrinsicMatrix()``, or ``session.json``'s
    ``video.depth.intrinsics``).  *down*: gravity direction in the camera frame, ``None`` = level.
    """
    cfg = config or ObstacleConfig()
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2-D image, got shape {}".format(depth.shape))
    K = np.asarray(intrinsics, dtype=np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    n = cfg.n_sectors

    s = cfg.stride
    sub = depth[::s, ::s].astype(np.float64) * 0.001
    with np.errstate(invalid="ignore"):
        valid = (sub >= max(cfg.min_range_m, 1e-6)) & (sub <= cfg.evidence_range_m)
    rows, cols = np.nonzero(valid)
    z = sub[rows, cols]
    x = (cols * s - cx) / fx * z
    y = (rows * s - cy) / fy * z
    p = np.stack([x, y, z], axis=1) @ level_rotation(down).T

    height = -p[:, 1]
    p = p[(height >= -cfg.band_below_m) & (height <= cfg.band_above_m)]
    lx, lz = p[:, 0], p[:, 2]
    rng = np.hypot(lx, lz)
    sector = np.rint(np.degrees(np.arctan2(lx, lz)) / cfg.sector_deg).astype(np.int64) % n

    nearest = _nearest_supported(sector, rng, n, cfg.min_points, cfg.cluster_m)
    in_corridor = (np.abs(lx) <= cfg.corridor_half_width_m) & (lz > 0.0)
    ahead = _nearest_supported(np.zeros(int(in_corridor.sum()), dtype=np.int64), lz[in_corridor], 1,
                               cfg.min_points, cfg.cluster_m)

    return ObstacleScan(
        distances_m=_three_state(nearest, cfg.max_range_m),
        support=np.bincount(sector, minlength=n),
        ahead_m=float(_three_state(ahead, cfg.max_range_m)[0]),
        sector_deg=float(cfg.sector_deg),
        min_range_m=float(cfg.min_range_m),
        max_range_m=float(cfg.max_range_m),
        valid_fraction=float(valid.mean()) if valid.size else 0.0,
        points_in_band=int(p.shape[0]),
    )
