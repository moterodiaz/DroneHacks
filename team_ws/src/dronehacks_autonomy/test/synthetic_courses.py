"""Made-up courses in the contracts/course.md map format. None of them is the real one.

``random_course`` varies what a layout-specific route could wrongly rely on: the net's size,
how many maze dividers there are (including none) and which side each hangs from, where the
tunnel and window sit, where the launch and pad are, which way the launch faces and the
whole course's rotation. Every course is solvable by construction.

``rigid`` and ``jitter_walls`` derive new courses from an existing document.
"""

from __future__ import annotations

import copy
import math
import random
from typing import Callable, Optional, Tuple

THICKNESS = 0.1
NET = 0.00127

Pt = Tuple[float, float]


def _seg(a: Pt, b: Pt, thickness: float = THICKNESS) -> dict:
    return {"start": list(a), "end": list(b), "thickness": thickness}


def random_course(seed: int, dividers: Optional[int] = None, clearance_radius: Optional[float] = None) -> dict:
    """Maze north of a barrier, a tunnel through the barrier, then a wall with a gap and a
    window in front of the pad. ``clearance_radius``: the window's reviewed budget (None = blocked)."""
    rng = random.Random(seed)
    n_div = rng.randint(0, 4) if dividers is None else dividers
    lane = [rng.uniform(1.05, 1.4) for _ in range(n_div + 1)]
    tunnel_width = rng.uniform(1.3, 1.6)
    tunnel_length = rng.uniform(1.2, 1.7)

    west = 0.0
    launch_x = west + lane[0] / 2.0
    div_x = []
    x = west
    for w in lane[:-1]:
        x += w
        div_x.append(x)
    # Far enough east for the landing wall and pad to fit west of the tunnel's exit, however
    # few dividers there are.
    tunnel_x = max(x + lane[-1] + tunnel_width / 2.0 + rng.uniform(0.1, 0.4), 4.2 + rng.uniform(0.0, 0.4))
    # Room south of the barrier for the wall, the window and the pad.
    width = max(tunnel_x + tunnel_width / 2.0 + rng.uniform(0.3, 0.8), 6.5)
    north = rng.uniform(2.6, 3.4)
    south = -rng.uniform(3.2, 3.8)

    segs = [_seg((west, south), (width, south), NET), _seg((width, south), (width, north), NET),
            _seg((width, north), (west, north), NET), _seg((west, north), (west, south), NET),
            _seg((west, 0.0), (tunnel_x - tunnel_width / 2.0, 0.0))]
    if width - (tunnel_x + tunnel_width / 2.0) > 0.05:
        segs.append(_seg((tunnel_x + tunnel_width / 2.0, 0.0), (width, 0.0)))

    hang_north = rng.random() < 0.5
    for dx in div_x:
        gap = rng.uniform(1.0, 1.25)
        segs.append(_seg((dx, north), (dx, gap)) if hang_north else _seg((dx, 0.0), (dx, north - gap)))
        hang_north = not hang_north

    # The wall guarding the landing area: from the south net, with a gap below the barrier.
    # West of the tunnel's side wall by enough to fly between them, or the course is sealed.
    wall_x = rng.uniform(2.4, min(3.4, tunnel_x - tunnel_width / 2.0 - 0.7))
    gap = rng.uniform(1.1, 1.35)
    wall_index = len(segs)
    segs.append(_seg((wall_x, south), (wall_x, -gap)))
    side = 0.914
    window_y = rng.uniform(south + side / 2.0 + 0.5, -gap - side / 2.0 - 0.5)

    pad_x = rng.uniform(1.0, wall_x - 1.1)
    pad_y = rng.uniform(south + 1.1, -1.1)

    tunnel_goal = {
        "name": "tunnel", "shape": "oval", "centre": [tunnel_x, 0.0],
        "params": {"axis_start": [tunnel_x, tunnel_length / 2.0], "axis_end": [tunnel_x, -tunnel_length / 2.0],
                   "width": tunnel_width, "height": 1.8},
    }
    window_goal = {
        "name": "window", "shape": "rect", "centre": [wall_x, window_y],
        "params": {"wall_segment_index": wall_index, "axis_start": [wall_x + 0.05, window_y],
                   "axis_end": [wall_x - 0.05, window_y], "side": side, "sill_height": 0.305,
                   "head_height": 1.219, "clearance_radius": clearance_radius},
    }
    pad_goal = {"name": "pad", "shape": "disc", "centre": [pad_x, pad_y], "params": {"radius": 0.762}}
    launch_y = rng.uniform(0.6, 1.2)
    launch_goal = {"name": "launch", "shape": "point", "centre": [launch_x, launch_y],
                   "params": {"heading_deg": rng.choice([0.0, 45.0, 90.0, 180.0, 270.0])}}
    goals = [tunnel_goal, window_goal, pad_goal, launch_goal]

    boulders = []
    east_room = width - wall_x
    if east_room > 2.2:
        for _ in range(rng.randint(0, 3)):
            boulders.append({"centre": [rng.uniform(wall_x + 1.2, width - 0.4),
                                        rng.uniform(south + 0.4, south + 1.0)], "radius": 0.2286})

    doc = {"origin": [0.0, 0.0], "slice_height": 0.8, "segments": segs, "boulders": boulders, "goals": goals}
    # Centre the course on the origin, then turn all of it to an arbitrary angle.
    cx, cy = width / 2.0, (north + south) / 2.0
    return rigid(doc, rng.uniform(0.0, 2.0 * math.pi), (-cx, -cy))


def _map_points(doc: dict, f: Callable[[list], list]) -> dict:
    out = copy.deepcopy(doc)
    for s in out["segments"]:
        s["start"], s["end"] = f(s["start"]), f(s["end"])
    for b in out.get("boulders") or []:
        b["centre"] = f(b["centre"])
    for g in out["goals"]:
        g["centre"] = f(g["centre"])
        for key in ("axis_start", "axis_end"):
            if key in g["params"]:
                g["params"][key] = f(g["params"][key])
    return out


def rigid(doc: dict, angle: float, shift_first: Pt = (0.0, 0.0)) -> dict:
    """Shift, then rotate the whole course counter-clockwise by ``angle`` radians about the origin."""
    c, s = math.cos(angle), math.sin(angle)

    def f(p):
        x, y = p[0] + shift_first[0], p[1] + shift_first[1]
        return [x * c - y * s, x * s + y * c]

    out = _map_points(doc, f)
    for g in out["goals"]:
        if g["name"] == "launch":   # heading is clockwise from north; the turn is counter-clockwise
            g["params"]["heading_deg"] = g["params"]["heading_deg"] - math.degrees(angle)
    return out


def mirror_east_west(doc: dict) -> dict:
    out = _map_points(doc, lambda p: [-p[0], p[1]])
    for g in out["goals"]:
        if g["name"] == "launch":
            g["params"]["heading_deg"] = -g["params"]["heading_deg"]
    return out


def jitter_walls(doc: dict, seed: int, max_shift_m: float) -> dict:
    """Move every wall, and the goals' surroundings with it, by a different small offset: the
    "layout may change slightly" case. The window's wall carries the window, so it stays valid."""
    rng = random.Random(seed)
    out = copy.deepcopy(doc)
    window = next(g for g in out["goals"] if g["name"] == "window")
    for i, s in enumerate(out["segments"]):
        if s["thickness"] < 0.01:   # the net does not move
            continue
        dx, dy = rng.uniform(-max_shift_m, max_shift_m), rng.uniform(-max_shift_m, max_shift_m)
        s["start"] = [s["start"][0] + dx, s["start"][1] + dy]
        s["end"] = [s["end"][0] + dx, s["end"][1] + dy]
        if i == window["params"]["wall_segment_index"]:
            window["centre"] = [window["centre"][0] + dx, window["centre"][1] + dy]
            for key in ("axis_start", "axis_end"):
                p = window["params"][key]
                window["params"][key] = [p[0] + dx, p[1] + dy]
    return out
