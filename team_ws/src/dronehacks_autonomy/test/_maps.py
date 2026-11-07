"""Synthetic maps for tests. Every course here is made up; none is the real one."""

from __future__ import annotations

import random
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from dronehacks_autonomy.planning import parse_map

FIXTURES = Path(__file__).resolve().parent / "fixtures"
PUBLISHED = FIXTURES / "published_v3_fieldguide.yaml"

Seg = Tuple[Tuple[float, float], Tuple[float, float]]


def box(w: float, h: float) -> List[Seg]:
    """The four sides of a ``w`` x ``h`` net centred on the origin."""
    x, y = w / 2, h / 2
    c = [(-x, -y), (x, -y), (x, y), (-x, y)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


def make_map(segments: Iterable[Seg], goals: Sequence[dict], launch: Optional[Tuple[float, float]] = None,
             boulders: Sequence[dict] = (), thickness: float = 0.1, slice_height: float = 0.8):
    doc = {
        "origin": [0.0, 0.0],
        "slice_height": slice_height,
        "segments": [{"start": list(a), "end": list(b), "thickness": thickness} for a, b in segments],
        "boulders": list(boulders),
        "goals": list(goals),
    }
    if launch is not None:
        doc["launch"] = {"position": list(launch)}
    return parse_map(doc, source="synthetic")


def pad(centre, name="pad", radius=0.5) -> dict:
    return {"name": name, "shape": "disc", "centre": list(centre), "params": {"radius": radius}}


def wall_with_gap(y: float, x_min: float, x_max: float, gap_lo: float, gap_hi: float) -> List[Seg]:
    """A horizontal wall at ``y`` from ``x_min`` to ``x_max`` with an opening ``gap_lo..gap_hi``."""
    return [((x_min, y), (gap_lo, y)), ((gap_hi, y), (x_max, y))]


def serpentine(seed: int):
    """A random serpentine maze in an 8 x 4 m net: walls hang alternately from
    the north and south nets, leaving gaps of 0.95-1.3 m; then a pad. Always
    solvable by construction."""
    rng = random.Random(seed)
    w, h = 8.0, 4.0
    segs = box(w, h)
    x = -w / 2
    north = rng.random() < 0.5
    while True:
        x += rng.uniform(1.0, 1.6)
        if x > w / 2 - 1.6:
            break
        gap = rng.uniform(0.95, 1.3)
        if north:
            segs.append(((x, h / 2), (x, -h / 2 + gap)))
        else:
            segs.append(((x, -h / 2), (x, h / 2 - gap)))
        north = not north
    return make_map(segs, [pad((w / 2 - 0.8, 0.0), radius=0.5)], launch=(-w / 2 + 0.55, 0.0))
