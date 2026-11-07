"""Turn the team's STEP extraction (``course-map/1`` + ``course-rules/1`` JSON) into a
``contracts/course.md`` map file the planner reads.

    python -m dronehacks_planner.convert MAP.json RULES.json --launch-heading 0 1 --out course.yaml \\
        [--route-gate gate_b between "C2b chicane wall" "A1 approach wall"] [--slice-height 0.8]

The JSON pair comes from the STEP extractor on the lidar stream's branch
(``course/12_extract_course_map.py``). This module only reads its output. It
slices at the flight height across the drone's whole vertical band
``[slice - half_height, slice + half_height]``:

* **Walls** with a height range covering the band become rectangles:
  outline edges, thickness 0, so square ends stay square. A wall is split
  wherever an opening covers the band.
* **Nets** become their footprint edges.
* **The tube** becomes two thin side walls along its axis. The inner wall is
  placed at the tube's narrowest within the band, which errs safe; the outer
  wall at the widest in the band, so paths beside the tube are not credited
  with clearance the obstacle does not have.
* **Rocks** become boulders, with their height.
* **Supports and markings** below the band are left out. The start and
  finish markings give the launch position and the pad.

Goals come from the rules' checkpoints, in order:
* ``tube_end`` gives the tunnel (an oval, flown out through the named end);
* ``routes`` gives a group of alternatives: an ``opening`` becomes a
  ``rect`` (window) goal. A ``gate`` goal comes from ``between`` two
  features (across their closest points at flight height) or from the
  ``ends`` of two walls (between their facing end points);
* ``land`` gives the pad.

Openings in a group are flown towards the pad. Segment and between gates
before the tunnel are written as ``checkpoints``, which the planner does not
read yet (PROPOSED). ``--route-gate`` replaces a route's gate, e.g. to follow
``contracts/course.md`` where the rules file disagrees with it.

The launch heading is not in the STEP: pass it (``contracts/frames.md``: facing
north up the first corridor, ``0 1``). No course coordinate is written in this
module (AGENTS.md section 3.7).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Pt = Tuple[float, float]


class ConvertError(ValueError):
    pass


def convert(cmap: dict, rules: dict, launch_heading: Sequence[float], slice_height: float = 0.8,
            half_height: float = 0.10, route_gates: Optional[Dict[str, list]] = None) -> dict:
    if cmap.get("format") != "course-map/1" or rules.get("format") != "course-rules/1":
        raise ConvertError("expected a course-map/1 map and a course-rules/1 rules file")
    lo, hi = slice_height - half_height, slice_height + half_height
    features = {f["name"]: f for f in cmap["features"]}
    segments: List[dict] = []
    boulders: List[dict] = []

    def covers(z) -> bool:
        return z[0] <= lo and z[1] >= hi

    def overlaps(z) -> bool:
        return z[1] > lo and z[0] < hi

    for f in cmap["features"]:
        kind = f["kind"]
        if kind == "wall" and overlaps(f["z"]):
            for rect in _wall_pieces(f, covers):
                segments += _edges(rect, f["name"])
        elif kind == "net" and overlaps(f["z"]) and f["z"][1] - f["z"][0] > 2 * half_height:
            segments += _edges(f["footprint"], f["name"])
        elif kind == "tube":
            for rect in _tube_walls(f, lo, hi):
                segments += _edges(rect, f["name"])
        elif kind == "rock" and f["z"][1] > 0:
            boulders.append({"centre": _r(f["centre"]), "radius": round(f["radius"], 5), "height": round(f["z"][1], 5),
                             "name": f["name"]})
        elif kind == "support" and overlaps(f["z"]):
            for rect in _wall_pieces(f, covers):
                segments += _edges(rect, f["name"])

    start = _marking(features, rules["start"]["marking"])
    finish = _marking(features, rules["finish"]["marking"])
    pad_radius = rules["finish"].get("inside_radius_m") or finish["half_size"][0]
    pad = {"name": "pad", "shape": "disc", "centre": _r(finish["centre"]), "params": {"radius": round(pad_radius, 5)}}

    goals: List[dict] = []
    checkpoints: List[dict] = []
    route_gates = route_gates or {}
    for cp in rules["checkpoints"]:
        if cp.get("land"):
            goals.append(pad)
        elif "gate" in cp and "tube_end" in cp["gate"]:
            goals.append(_tunnel(features[cp["gate"]["tube_end"]], cp["gate"]["end"], lo, hi))
        elif "routes" in cp:
            group = _slug(cp["name"])
            for name, route in cp["routes"].items():
                gate = route_gates.get(name, route["gate"])
                goal = _gate_goal(name, gate, features, finish["centre"], covers, lo, hi)
                goal["group"] = group
                goals.append(goal)
        elif "gate" in cp:
            checkpoints.append({"name": cp["name"], "line": [_r(p) for p in _gate_line(cp["gate"], features, covers)]})
        else:
            raise ConvertError(f"checkpoint {cp.get('name')!r} has no gate this converter understands")
    if not any(g["shape"] == "disc" for g in goals):
        goals.append(pad)

    hx, hy = float(launch_heading[0]), float(launch_heading[1])
    n = math.hypot(hx, hy)
    if n < 1e-9:
        raise ConvertError("--launch-heading has zero length")
    return {
        "origin": [0.0, 0.0],
        "slice_height": slice_height,
        "launch": {"position": _r(start["centre"]), "heading": [round(hx / n, 6), round(hy / n, 6)]},
        "segments": segments,
        "boulders": [{k: v for k, v in b.items() if k != "name"} for b in boulders],
        "goals": goals,
        "checkpoints": checkpoints,
    }


# -- geometry ----------------------------------------------------------------------------


def _frame(p0, p1) -> Tuple[Pt, Pt, float]:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        raise ConvertError("a wall has zero length")
    u = (dx / length, dy / length)
    return u, (-u[1], u[0]), length


def _wall_pieces(f: dict, covers) -> List[List[Pt]]:
    """The wall's rectangle(s) at flight height, split at openings that cover the drone's band."""
    p0, p1, t = f["p0"][:2], f["p1"][:2], f["thickness"]
    u, nrm, length = _frame(p0, p1)
    cuts = sorted((o["s"][0], o["s"][1]) for o in f.get("openings", []) if covers(o["z"]))
    pieces, s = [], 0.0
    for a, b in cuts:
        if a > s:
            pieces.append((s, a))
        s = max(s, b)
    if s < length:
        pieces.append((s, length))
    rects = []
    for a, b in pieces:
        if b - a < 1e-6:
            continue
        c0 = (p0[0] + u[0] * a, p0[1] + u[1] * a)
        c1 = (p0[0] + u[0] * b, p0[1] + u[1] * b)
        h = t / 2.0
        rects.append([(c0[0] + nrm[0] * h, c0[1] + nrm[1] * h), (c1[0] + nrm[0] * h, c1[1] + nrm[1] * h),
                      (c1[0] - nrm[0] * h, c1[1] - nrm[1] * h), (c0[0] - nrm[0] * h, c0[1] - nrm[1] * h)])
    return rects


def _ellipse_half_width(half_width: float, half_height: float, centre_z: float, lo: float, hi: float,
                        widest: bool = False) -> float:
    """Half-width of an elliptical section over heights lo..hi.

    ``widest=False``: the narrowest width in the band - for the tube's inner
    wall, which errs safe. ``widest=True``: the width at the height closest
    to the ellipse centre (the full half-width if the band contains it) -
    for the outer wall, so paths beside the tube are not given clearance
    the obstacle does not have."""
    if widest:
        dz = 0.0 if lo <= centre_z <= hi else min(abs(lo - centre_z), abs(hi - centre_z))
    else:
        dz = max(abs(lo - centre_z), abs(hi - centre_z))
    if dz >= half_height:
        return 0.0
    return half_width * math.sqrt(1.0 - (dz / half_height) ** 2)


def _tube_walls(f: dict, lo: float, hi: float) -> List[List[Pt]]:
    p0, p1, zc = f["p0"][:2], f["p1"][:2], f["p0"][2]
    u, nrm, length = _frame(p0, p1)
    inner = _ellipse_half_width(f["inner"]["half_width"], f["inner"]["half_height"], zc, lo, hi)
    outer = _ellipse_half_width(f["outer"]["half_width"], f["outer"]["half_height"], zc, lo, hi, widest=True)
    outer = max(inner, outer)
    rects = []
    for side in (1.0, -1.0):
        a = (p0[0] + nrm[0] * side * inner, p0[1] + nrm[1] * side * inner)
        b = (p1[0] + nrm[0] * side * inner, p1[1] + nrm[1] * side * inner)
        c = (p1[0] + nrm[0] * side * outer, p1[1] + nrm[1] * side * outer)
        d = (p0[0] + nrm[0] * side * outer, p0[1] + nrm[1] * side * outer)
        rects.append([a, b, c, d])
    return rects


def _edges(poly: Sequence[Sequence[float]], name: str) -> List[dict]:
    pts = [(float(p[0]), float(p[1])) for p in poly]
    out = []
    for a, b in zip(pts, pts[1:] + pts[:1]):
        if math.dist(a, b) > 1e-6:
            out.append({"start": _r(a), "end": _r(b), "thickness": 0.0})
    return out


def _footprint(f: dict, covers) -> List[List[Pt]]:
    if f["kind"] in ("wall", "support"):
        return _wall_pieces(f, covers)
    if "footprint" in f:
        return [[tuple(p[:2]) for p in f["footprint"]]]
    raise ConvertError(f"feature {f['name']!r} has no footprint at flight height")


def _closest_between(polys_a, polys_b) -> Tuple[Pt, Pt]:
    best = (math.inf, None, None)
    for pa in polys_a:
        for pb in polys_b:
            for a0, a1 in zip(pa, pa[1:] + pa[:1]):
                for b0, b1 in zip(pb, pb[1:] + pb[:1]):
                    d, x, y = _segment_segment(a0, a1, b0, b1)
                    if d < best[0]:
                        best = (d, x, y)
    return best[1], best[2]


def _segment_segment(a0, a1, b0, b1):
    def closest(p, s0, s1):
        dx, dy = s1[0] - s0[0], s1[1] - s0[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - s0[0]) * dx + (p[1] - s0[1]) * dy) / l2))
        return (s0[0] + t * dx, s0[1] + t * dy)
    cands = [(a0, closest(a0, b0, b1)), (a1, closest(a1, b0, b1)),
             (closest(b0, a0, a1), b0), (closest(b1, a0, a1), b1)]
    d, x, y = min(((math.dist(p, q), p, q) for p, q in cands), key=lambda c: c[0])
    return d, x, y


def _gate_line(gate: dict, features: dict, covers) -> Tuple[Pt, Pt]:
    if "segment" in gate:
        a, b = gate["segment"]
        return tuple(a[:2]), tuple(b[:2])
    if "between" in gate:
        fa, fb = (features[n] for n in gate["between"])
        return _closest_between(_footprint(fa, covers), _footprint(fb, covers))
    if "ends" in gate:
        # The line between two walls' facing ends (centreline end points): how a gap
        # "from wall A to wall B" is measured, e.g. contracts/course.md's Gate B.
        fa, fb = (features[n] for n in gate["ends"])
        ends_a = [tuple(fa["p0"][:2]), tuple(fa["p1"][:2])]
        ends_b = [tuple(fb["p0"][:2]), tuple(fb["p1"][:2])]
        return min(((a, b) for a in ends_a for b in ends_b), key=lambda ab: math.dist(*ab))
    raise ConvertError(f"gate {gate!r}: not a segment, between or ends gate")


def _toward(axis: Pt, centre: Pt, target: Sequence[float]) -> Pt:
    d = (target[0] - centre[0]) * axis[0] + (target[1] - centre[1]) * axis[1]
    return axis if d >= 0 else (-axis[0], -axis[1])


def _gate_goal(name: str, gate: dict, features: dict, pad_centre, covers, lo: float, hi: float) -> dict:
    if "opening" in gate:
        wall = features[gate["opening"]]
        u, nrm, _ = _frame(wall["p0"][:2], wall["p1"][:2])
        ops = [o for o in wall.get("openings", []) if covers(o["z"])]
        if len(ops) != 1:
            raise ConvertError(f"{gate['opening']!r}: expected one opening at flight height, found {len(ops)}")
        o = ops[0]
        s = (o["s"][0] + o["s"][1]) / 2.0
        centre = (wall["p0"][0] + u[0] * s, wall["p0"][1] + u[1] * s)
        axis = _toward(nrm, centre, pad_centre)
        return {"name": name, "shape": "rect", "centre": _r(centre),
                "params": {"axis": _r(axis, 6), "width": round(o["s"][1] - o["s"][0], 5),
                           "depth": round(wall["thickness"], 5), "bottom": round(o["z"][0], 5),
                           "top": round(o["z"][1], 5)}}
    a, b = _gate_line(gate, features, covers)
    if math.dist(a, b) < 1e-6:
        raise ConvertError(f"route {name!r}: its gate {gate!r} has no gap (the features touch)")
    centre = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    u, nrm, width = _frame(a, b)
    axis = _toward(nrm, centre, pad_centre)
    depth = min((features[n].get("thickness", 0.0) for n in gate.get("between", gate.get("ends", []))), default=0.0)
    return {"name": name, "shape": "gate", "centre": _r(centre),
            "params": {"axis": _r(axis, 6), "width": round(width, 5), "depth": round(depth, 5)}}


def _tunnel(tube: dict, end: str, lo: float, hi: float) -> dict:
    p0, p1 = tube["p0"][:2], tube["p1"][:2]
    out_end, other = (p0, p1) if end == "p0" else (p1, p0)
    u, _, length = _frame(other, out_end)
    centre = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
    inner = _ellipse_half_width(tube["inner"]["half_width"], tube["inner"]["half_height"], tube["p0"][2], lo, hi)
    return {"name": "tunnel", "shape": "oval", "centre": _r(centre),
            "params": {"axis": _r(u, 6), "length": round(length, 5), "width": round(2 * inner, 5),
                       "height": round(2 * tube["inner"]["half_height"], 5)}}


def _marking(features: dict, name: str) -> dict:
    if name not in features:
        raise ConvertError(f"marking {name!r} is not in the map")
    return features[name]


def _r(p, nd: int = 5) -> List[float]:
    return [round(float(p[0]), nd), round(float(p[1]), nd)]


def _slug(s: str) -> str:
    return "".join(c if c.isalnum() else "_" for c in s.lower()).strip("_")


# -- command line ------------------------------------------------------------------------------


def _portable(args: Sequence[str]) -> List[str]:
    """The command line with file paths reduced to their names, so the header doesn't record a machine."""
    return [Path(a).name if ("/" in a or "\\" in a) else a for a in args]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m dronehacks_planner.convert", description=__doc__.splitlines()[0])
    ap.add_argument("map_json")
    ap.add_argument("rules_json")
    ap.add_argument("--launch-heading", nargs=2, type=float, required=True, metavar=("EAST", "NORTH"))
    ap.add_argument("--slice-height", type=float, default=0.8, help="flight height above the floor, m")
    ap.add_argument("--half-height", type=float, default=0.10, help="half the drone's height, m")
    ap.add_argument("--route-gate", nargs="+", action="append", default=[], metavar="ROUTE",
                    help="ROUTE {between|ends} FEATURE FEATURE: replace a route's gate")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    route_gates = {}
    for spec in args.route_gate:
        if len(spec) != 4 or spec[1] not in ("between", "ends"):
            ap.error("--route-gate takes: ROUTE {between|ends} FEATURE FEATURE")
        route_gates[spec[0]] = {spec[1]: [spec[2], spec[3]]}
    map_path, rules_path = Path(args.map_json), Path(args.rules_json)
    cmap = json.loads(map_path.read_text(encoding="utf-8"))
    rules = json.loads(rules_path.read_text(encoding="utf-8"))
    try:
        doc = convert(cmap, rules, args.launch_heading, args.slice_height, args.half_height, route_gates)
    except (ConvertError, KeyError) as exc:
        print(f"convert: {exc}", file=sys.stderr)
        return 2
    src = cmap.get("source", {})
    header = [
        "# Course map for the planner (contracts/course.md format), generated from the STEP model.",
        f"# STEP: {src.get('step')} sha256 {src.get('sha256')} (extracted {src.get('extracted')})",
        f"# via {map_path.name} sha256 {_sha(map_path)} and {rules_path.name} sha256 {_sha(rules_path)}",
        "# by: python -m dronehacks_planner.convert " + " ".join(
            f'"{a}"' if " " in a else a for a in _portable(argv if argv is not None else sys.argv[1:])),
        f"# Sliced at {args.slice_height} m over the drone's band +-{args.half_height} m. Not surveyed: the real",
        "# course may sit a few cm off its model (contracts/course.md, Layout tolerance).",
        "# Fields not in contracts/course.md yet (launch, depth, gate, group, boulder height,",
        "# checkpoints) are PROPOSED; see planner/README.md.",
    ]
    body = yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=120)
    Path(args.out).write_text("\n".join(header) + "\n" + body, encoding="utf-8", newline="\n")
    print(f"wrote {args.out}: {len(doc['segments'])} edges, {len(doc['boulders'])} boulders, goals "
          + " > ".join(g["name"] + (f"[{g['group']}]" if "group" in g else "") for g in doc["goals"])
          + f", {len(doc['checkpoints'])} checkpoint lines")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
