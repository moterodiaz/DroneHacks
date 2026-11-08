"""Course map: classify STEP solids into course features, store them as JSON, slice them for planning.

Frame
-----
The map keeps the STEP file's frame: metres, +x east, +y north, +z up (ENU).  For the competition
course the origin is the centre of the floor.  Gazebo uses the same frame, so the world generator
places features without conversion; the planner writes PX4 NED setpoints separately.

Feature kinds
-------------
``wall``     thin vertical box (thickness <= 0.25 m, length >= 3x thickness): centre line ``p0 -> p1``,
             ``thickness``, ``z`` range, and rectangular ``openings`` (``s`` along the wall from ``p0``,
             ``z``) found as holes in the wall's large side faces (the window).
``tube``     a hollow tube whose ends are closed circles/ellipses with a horizontal axis: axis end
             centres ``p0``/``p1`` and ``outer``/``inner`` half width and half height.
``rock``     a low round obstacle with a closed circle at its base (boulders): ``centre``, ``radius``.
``support``  a wall or block that sits entirely under a tube (tube cradles).  Never in the way.
``net``      a thin sheet at the boundary (sides and roof).  ``boundary`` is derived from these.
``marking``  a flat solid lying on the floor (top below 1 cm: lines, pads, rings): top-face
             ``polygons`` with holes.
``floor``    the floor slab.
``block``    anything else: convex ``footprint`` and ``z`` range.

Every feature also has ``name`` (the CAD part name), ``kind``, ``color`` and ``z``.

Slicing
-------
:meth:`CourseMap.obstacles_2d` returns convex 2D polygons of everything a drone body occupying the
height band ``[z_lo, z_hi]`` could touch: walls minus the openings the band fits through, the two
side walls of a tube when the band fits inside it, rocks (by default regardless of height, because
flying over them corrupts a downward rangefinder), nets and blocks.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from course.geometry import convex_hull, min_area_rect, points_in_convex, rect_polygon, regular_polygon
from course.step_reader import Conic, Solid, read_solids

MAP_FORMAT = "course-map/1"
FRAME_NOTE = "metres; +x east, +y north, +z up (ENU); origin as in the STEP file"

WALL_MAX_THICKNESS_M = 0.25
WALL_MIN_ASPECT = 3.0
MARKING_MAX_HEIGHT_M = 0.01
NET_MAX_THICKNESS_M = 0.005


@dataclass
class Obstacle2D:
    name: str
    kind: str
    polygon: np.ndarray


# ------------------------------------------------------------------------------ classification


def _extent_along(c: Conic, e: np.ndarray) -> float:
    """Half extent of a conic along a unit vector."""
    return math.hypot(c.a * float(np.dot(c.x_dir, e)), c.b * float(np.dot(c.y_dir, e)))


def _as_tube(solid: Solid) -> Optional[Dict[str, Any]]:
    closed = [c for c in solid.conics if c.closed and abs(c.axis[2]) < 0.1]
    if len(closed) < 4:
        return None
    u = closed[0].axis / np.linalg.norm(closed[0].axis)
    # A conic's normal may point either way along the tube; fix the sign (largest component
    # positive) so ``p0`` is always the end with the lower coordinate along the axis, whichever
    # end face the exporter wrote first.  The rules' ``tube_end ... end p0`` depends on it.
    if u[int(np.argmax(np.abs(u)))] < 0.0:
        u = -u
    if any(abs(float(np.dot(c.axis, u))) < 0.999 for c in closed):
        return None
    groups: Dict[int, List[Conic]] = {}
    for c in closed:
        groups.setdefault(int(round(float(np.dot(c.centre, u)) * 1000.0)), []).append(c)
    if len(groups) != 2 or any(len(g) < 2 for g in groups.values()):
        return None
    w = np.cross(np.array([0.0, 0.0, 1.0]), u)
    w /= np.linalg.norm(w)
    vertical = np.array([0.0, 0.0, 1.0])
    ends, outer, inner = [], [], []
    for key in sorted(groups):
        g = sorted(groups[key], key=lambda c: -_extent_along(c, w))
        ends.append(np.mean([c.centre for c in g], axis=0))
        outer.append((_extent_along(g[0], w), _extent_along(g[0], vertical)))
        inner.append((_extent_along(g[-1], w), _extent_along(g[-1], vertical)))
    outer_m, inner_m = np.mean(outer, axis=0), np.mean(inner, axis=0)
    return {
        "kind": "tube",
        "p0": ends[0].tolist(),
        "p1": ends[1].tolist(),
        "outer": {"half_width": float(outer_m[0]), "half_height": float(outer_m[1])},
        "inner": {"half_width": float(inner_m[0]), "half_height": float(inner_m[1])},
    }


def _wall_openings(solid: Solid, p0: np.ndarray, d: np.ndarray, t: np.ndarray) -> List[Dict[str, Any]]:
    found: Dict[Tuple[int, int, int, int], Dict[str, Any]] = {}
    for face in solid.faces:
        if face.normal is None or len(face.loops) < 2:
            continue
        if abs(float(np.dot(face.normal[:2], t))) < 0.99:
            continue
        for hole in face.loops[1:]:
            s = (hole[:, :2] - p0) @ d
            key = tuple(int(round(v * 1000.0)) for v in (s.min(), s.max(), hole[:, 2].min(), hole[:, 2].max()))
            found[key] = {"s": [float(s.min()), float(s.max())],
                          "z": [float(hole[:, 2].min()), float(hole[:, 2].max())]}
    return [found[k] for k in sorted(found)]


def classify_solid(solid: Solid) -> Dict[str, Any]:
    """One map feature for one solid (see the module docstring for the kinds)."""
    pts = solid.points()
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    dz = float(hi[2] - lo[2])
    centre, angle, length, width = min_area_rect(pts[:, :2])
    feature: Dict[str, Any] = {
        "name": solid.name,
        "color": list(solid.color) if solid.color is not None else None,
        "z": [float(lo[2]), float(hi[2])],
    }

    if hi[2] <= 1e-4 and lo[2] < -1e-4 and length * width > 4.0:
        feature.update(kind="floor", footprint=convex_hull(pts[:, :2]).tolist())
        return feature

    if hi[2] < MARKING_MAX_HEIGHT_M and lo[2] >= -1e-4:
        polygons = []
        for face in solid.faces:
            if face.normal is None or abs(face.normal[2]) < 0.99:
                continue
            if float(np.mean(face.loops[0][:, 2])) < hi[2] - 1e-4:
                continue
            polygons.append({"outer": face.loops[0][:, :2].tolist(),
                             "holes": [loop[:, :2].tolist() for loop in face.loops[1:]]})
        feature.update(kind="marking", centre=[float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)],
                       half_size=[float((hi[0] - lo[0]) / 2), float((hi[1] - lo[1]) / 2)], polygons=polygons)
        return feature

    if (width < NET_MAX_THICKNESS_M and dz > 1.0) or (dz < NET_MAX_THICKNESS_M and lo[2] > 1.0):
        feature.update(kind="net", footprint=rect_polygon(centre, angle, length, width).tolist())
        return feature

    tube = _as_tube(solid)
    if tube is not None:
        feature.update(tube)
        return feature

    base = [c for c in solid.conics if c.closed and abs(c.axis[2]) > 0.9 and abs(c.centre[2] - lo[2]) < 1e-3]
    if base and dz < 1.0 and length < 1.3 * max(width, 1e-9):
        c = max(base, key=lambda c: max(c.a, c.b))
        feature.update(kind="rock", centre=[float(c.centre[0]), float(c.centre[1])], radius=float(max(c.a, c.b)))
        return feature

    if width <= WALL_MAX_THICKNESS_M and length >= WALL_MIN_ASPECT * width:
        d = np.array([math.cos(angle), math.sin(angle)])
        t = np.array([-d[1], d[0]])
        p0, p1 = centre - d * length / 2.0, centre + d * length / 2.0
        feature.update(kind="wall", p0=p0.tolist(), p1=p1.tolist(), thickness=float(width),
                       openings=_wall_openings(solid, p0, d, t))
        return feature

    feature.update(kind="block", footprint=convex_hull(pts[:, :2]).tolist())
    return feature


def _footprint(feature: Dict[str, Any]) -> np.ndarray:
    kind = feature["kind"]
    if kind == "wall" or (kind == "support" and "p0" in feature):  # a support can be a wall that sits under a tube
        p0, p1 = np.array(feature["p0"]), np.array(feature["p1"])
        e = p1 - p0
        return rect_polygon((p0 + p1) / 2.0, math.atan2(e[1], e[0]), float(np.linalg.norm(e)), feature["thickness"])
    if kind == "tube":
        p0, p1 = np.array(feature["p0"])[:2], np.array(feature["p1"])[:2]
        e = p1 - p0
        return rect_polygon((p0 + p1) / 2.0, math.atan2(e[1], e[0]), float(np.linalg.norm(e)),
                            2.0 * feature["outer"]["half_width"])
    if kind == "rock":
        return regular_polygon(feature["centre"], feature["radius"])
    if kind == "marking":
        return convex_hull(np.concatenate([np.array(p["outer"]) for p in feature["polygons"]], axis=0))
    return np.array(feature["footprint"])


def _mark_supports(features: List[Dict[str, Any]]) -> None:
    tubes = [f for f in features if f["kind"] == "tube"]
    for f in features:
        if f["kind"] not in ("wall", "block"):
            continue
        for tube in tubes:
            centre_z = 0.5 * (tube["p0"][2] + tube["p1"][2])
            if f["z"][1] < centre_z and points_in_convex(_footprint(f), _footprint(tube), tol=0.01).all():
                f["kind"] = "support"
                f["under"] = tube["name"]
                break


def _boundary(features: List[Dict[str, Any]]) -> Dict[str, Any]:
    nets = [f for f in features if f["kind"] == "net"]
    side = [f for f in nets if f["z"][1] - f["z"][0] > 1.0]
    roof = [f for f in nets if f["z"][1] - f["z"][0] <= 1.0]
    if side:
        middle = np.mean([np.mean(_footprint(f), axis=0) for f in side], axis=0)
    floor = [f for f in features if f["kind"] == "floor"]
    if not side and not floor:
        return {}
    x_lo, x_hi, y_lo, y_hi = -math.inf, math.inf, -math.inf, math.inf
    for f in side:
        fp = _footprint(f)
        centre_xy = fp.mean(axis=0)
        span = fp.max(axis=0) - fp.min(axis=0)
        if span[0] < span[1]:  # thin in x: an east or west net
            if centre_xy[0] < middle[0]:
                x_lo = max(x_lo, float(fp[:, 0].max()))
            else:
                x_hi = min(x_hi, float(fp[:, 0].min()))
        else:
            if centre_xy[1] < middle[1]:
                y_lo = max(y_lo, float(fp[:, 1].max()))
            else:
                y_hi = min(y_hi, float(fp[:, 1].min()))
    if floor:
        fp = np.array(floor[0]["footprint"])
        x_lo = x_lo if math.isfinite(x_lo) else float(fp[:, 0].min())
        x_hi = x_hi if math.isfinite(x_hi) else float(fp[:, 0].max())
        y_lo = y_lo if math.isfinite(y_lo) else float(fp[:, 1].min())
        y_hi = y_hi if math.isfinite(y_hi) else float(fp[:, 1].max())
    out: Dict[str, Any] = {"x": [x_lo, x_hi], "y": [y_lo, y_hi]}
    out["z"] = [0.0, float(min(f["z"][0] for f in roof))] if roof else [0.0, None]
    return out


def _rounded(value: Any, digits: int = 5) -> Any:
    if isinstance(value, float):
        return round(value, digits) + 0.0
    if isinstance(value, dict):
        return {k: _rounded(v, digits) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_rounded(v, digits) for v in value]
    if isinstance(value, np.ndarray):
        return _rounded(value.tolist(), digits)
    if isinstance(value, np.generic):
        return _rounded(value.item(), digits)
    return value


def extract_map(step_path: Path) -> Dict[str, Any]:
    """Read a STEP file and return the course map as a JSON-ready dict."""
    solids, info = read_solids(step_path)
    features = [classify_solid(s) for s in solids]
    _mark_supports(features)
    data = {
        "format": MAP_FORMAT,
        "frame": FRAME_NOTE,
        "source": {
            "step": Path(step_path).name,
            "sha256": hashlib.sha256(Path(step_path).read_bytes()).hexdigest(),
            "products": info["products"][:1],
            "unit_to_metres": info["unit_to_metres"],
            "extracted": dt.date.today().isoformat(),
            "unsupported_curves": sum(s.unsupported_curves for s in solids),
        },
        "boundary": _boundary(features),
        "features": sorted(features, key=lambda f: (f["kind"], f["name"])),
    }
    return _rounded(data)


# ------------------------------------------------------------------------------- map + slicing


class CourseMap:
    """A loaded course map with lookup and slicing helpers."""

    def __init__(self, data: Dict[str, Any]) -> None:
        if data.get("format") != MAP_FORMAT:
            raise ValueError("not a {} file (format={!r})".format(MAP_FORMAT, data.get("format")))
        self.data = data
        self.features: List[Dict[str, Any]] = data["features"]
        self.boundary: Dict[str, Any] = data["boundary"]

    @classmethod
    def load(cls, path: Path) -> "CourseMap":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path: Path) -> None:
        # LF on every OS: generated planner maps record this file's SHA-256
        Path(path).write_text(json.dumps(self.data, indent=1) + "\n", encoding="utf-8", newline="\n")

    def feature(self, name: str) -> Dict[str, Any]:
        for f in self.features:
            if f["name"] == name:
                return f
        raise KeyError("no feature named {!r} in the map (have: {})".format(
            name, ", ".join(sorted(f["name"] for f in self.features))))

    def of_kind(self, *kinds: str) -> List[Dict[str, Any]]:
        return [f for f in self.features if f["kind"] in kinds]

    def footprint(self, name: str) -> np.ndarray:
        return _footprint(self.feature(name))

    def obstacles_2d(self, z_lo: float, z_hi: float, rocks_full_height: bool = True) -> List[Obstacle2D]:
        """Convex 2D obstacles for a body that spans heights ``[z_lo, z_hi]``."""
        out: List[Obstacle2D] = []
        for f in self.features:
            kind = f["kind"]
            overlaps = f["z"][0] < z_hi and f["z"][1] > z_lo
            if kind in ("marking", "floor", "support"):
                continue
            if kind == "rock":
                if overlaps or rocks_full_height:
                    out.append(Obstacle2D(f["name"], kind, regular_polygon(f["centre"], f["radius"])))
                continue
            if not overlaps:
                continue
            if kind == "wall":
                out.extend(Obstacle2D(f["name"], kind, poly) for poly in _wall_pieces(f, z_lo, z_hi))
            elif kind == "tube":
                out.extend(Obstacle2D(f["name"], kind, poly) for poly in _tube_pieces(f, z_lo, z_hi))
            else:
                out.append(Obstacle2D(f["name"], kind, _footprint(f)))
        return out


def _wall_pieces(f: Dict[str, Any], z_lo: float, z_hi: float) -> List[np.ndarray]:
    p0, p1 = np.array(f["p0"]), np.array(f["p1"])
    e = p1 - p0
    length = float(np.linalg.norm(e))
    d = e / length
    angle = math.atan2(d[1], d[0])
    gaps = sorted((max(0.0, o["s"][0]), min(length, o["s"][1])) for o in f.get("openings", [])
                  if o["z"][0] <= z_lo and z_hi <= o["z"][1])
    pieces, s = [], 0.0
    for g0, g1 in gaps + [(length, length)]:
        if g0 - s > 1e-6:
            mid = p0 + d * (s + g0) / 2.0
            pieces.append(rect_polygon(mid, angle, g0 - s, f["thickness"]))
        s = max(s, g1)
    return pieces


def _half_width_at(half_width: float, half_height: float, dz: float) -> float:
    r = 1.0 - (dz / half_height) ** 2
    return half_width * math.sqrt(r) if r > 0.0 else 0.0


def _tube_pieces(f: Dict[str, Any], z_lo: float, z_hi: float) -> List[np.ndarray]:
    p0, p1 = np.array(f["p0"]), np.array(f["p1"])
    zc = 0.5 * (p0[2] + p1[2])
    axis = p1[:2] - p0[:2]
    length = float(np.linalg.norm(axis))
    angle = math.atan2(axis[1], axis[0])
    w = np.array([-axis[1], axis[0]]) / length
    mid = (p0[:2] + p1[:2]) / 2.0
    ow, oh = f["outer"]["half_width"], f["outer"]["half_height"]
    iw, ih = f["inner"]["half_width"], f["inner"]["half_height"]
    if z_lo < zc - ih or z_hi > zc + ih:  # the body does not fit inside: the whole tube is in the way
        return [rect_polygon(mid, angle, length, 2.0 * ow)]
    inner = min(_half_width_at(iw, ih, z_lo - zc), _half_width_at(iw, ih, z_hi - zc))
    z_near = min(max(zc, z_lo), z_hi)
    outer = _half_width_at(ow, oh, z_near - zc)
    width = max(outer - inner, 0.005)
    offset = inner + width / 2.0
    return [rect_polygon(mid + side * offset * w, angle, length, width) for side in (-1.0, 1.0)]
