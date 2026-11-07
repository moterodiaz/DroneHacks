"""A* over the cost grid, to one or several targets.

Pure Python with ``heapq``: no scipy, so it runs in the stock skybeta image
without a team Docker layer. A 5 cm grid over the 6.1 m net is about 17k
cells, which this searches in well under a second.

Moves are 8-connected. A diagonal move is allowed only when both cells it
cuts past are free, so a path never squeezes between two lethal cells that
touch at a corner. A step from cell u to cell v costs
``step_length * (mult[u] + mult[v]) / 2``.

With several targets the heuristic is the octile distance to the nearest one.
That is consistent, so every target popped has its optimal cost, and the
search runs until all targets are popped (or the frontier is empty).
"""

from __future__ import annotations

import dataclasses
import heapq
import math
from typing import Dict, Iterable, List, Sequence

INF = math.inf
_SQRT2 = math.sqrt(2.0)
_MOVES = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)


@dataclasses.dataclass
class SearchResult:
    source: int
    costs: Dict[int, float]   # target -> cost (inf if unreachable)
    parent: List[int]         # -1 where unvisited

    def path(self, target: int) -> List[int]:
        """Cells from source to target inclusive. Raises if target was not reached."""
        if self.costs.get(target, INF) == INF:
            raise ValueError(f"target {target} was not reached")
        out = [target]
        while out[-1] != self.source:
            out.append(self.parent[out[-1]])
        out.reverse()
        return out


def search(mult: Sequence[float], nx: int, ny: int, res: float,
           source: int, targets: Iterable[int]) -> SearchResult:
    """Cheapest paths from ``source`` to each of ``targets`` over ``mult`` (row-major, inf = lethal)."""
    remaining = set(targets)
    goals = [(t % nx, t // nx) for t in remaining]
    n = nx * ny
    g = [INF] * n
    parent = [-1] * n
    closed = bytearray(n)
    costs: Dict[int, float] = {t: INF for t in remaining}

    def h(i: int) -> float:
        x, y = i % nx, i // nx
        best = INF
        for gx, gy in goals:
            dx = abs(x - gx)
            dy = abs(y - gy)
            v = dx + dy + (_SQRT2 - 2.0) * (dx if dx < dy else dy)
            if v < best:
                best = v
        return best * res

    g[source] = 0.0
    heap = [(h(source), 0.0, source)]
    while heap and remaining:
        _, gu, u = heapq.heappop(heap)
        if closed[u]:
            continue
        closed[u] = 1
        if u in remaining:
            remaining.discard(u)
            costs[u] = gu
            if not remaining:
                break
        ux, uy = u % nx, u // nx
        mu = mult[u]
        for dx, dy, step in _MOVES:
            vx = ux + dx
            vy = uy + dy
            if vx < 0 or vy < 0 or vx >= nx or vy >= ny:
                continue
            v = vy * nx + vx
            if closed[v]:
                continue
            mv = mult[v]
            if mv == INF:
                continue
            if dx and dy and (mult[uy * nx + vx] == INF or mult[vy * nx + ux] == INF):
                continue
            gv = gu + step * res * 0.5 * (mu + mv)
            if gv < g[v]:
                g[v] = gv
                parent[v] = u
                heapq.heappush(heap, (gv + h(v), gv, v))
    return SearchResult(source=source, costs=costs, parent=parent)
