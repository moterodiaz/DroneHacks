"""Path smoothing: trade length against curvature, keep gates on their segments, never enter obstacles.

The path is a polyline ``P`` of ``N`` points about ``spacing`` apart.  With ``h`` the spacing,

    E(P) = (1/h)   sum |p[i+1] - p[i]|^2                      ~ path length
         + (lam/h^3) sum |p[i-1] - 2 p[i] + p[i+1]|^2          ~ lam * integral of curvature^2 ds
         + w h     sum max(0, clearance - d(p[i]))^2           obstacle penalty

where ``d`` is the signed distance to the grown obstacles.  Each iteration takes a semi-implicit
step on the quadratic part (one banded linear solve by conjugate gradients, CHOMP-style) and an
explicit step on the penalty,
then projects: gate points back onto their free gate segment, points inside an obstacle out to its
boundary.  The start and the goal are fixed; every few iterations each section between anchors is
re-spaced evenly so the points do not bunch up.

``lam`` (metres) is the knob: 0 gives the taut-string shortest path with sharp corners, larger
values round the corners off.  The fastest value depends on the course and the acceleration
limit, so the planner tries several and keeps the one with the lowest flight time.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

from course.planner.cspace import CSpace
from course.planner.route import Gate


def resample_sections(points: np.ndarray, anchors: Sequence[int], spacing: float) -> Tuple[np.ndarray, List[int]]:
    """Evenly re-space every section between anchors (about ``spacing`` apart); anchors stay exact."""
    out: List[np.ndarray] = [points[0][None]]
    new_anchors = [0]
    for s0, s1 in zip(anchors[:-1], anchors[1:]):
        section = points[s0:s1 + 1]
        n = max(2, int(math.ceil(_length(section) / spacing)) + 1)
        out.append(_respace(section, n)[1:])
        new_anchors.append(new_anchors[-1] + n - 1)
    return np.concatenate(out, axis=0), new_anchors


def _length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _respace(points: np.ndarray, n: int) -> np.ndarray:
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0.0:
        return np.repeat(points[:1], n, axis=0)
    target = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(target, s, points[:, 0]), np.interp(target, s, points[:, 1])], axis=1)


class _StepSystem:
    """The step matrix ``M = 2A + s I`` (A from the length and curvature terms) restricted to the
    free points, applied without forming it: first and second differences are array slices.
    ``M`` is symmetric positive definite and well conditioned (``s`` is the mean diagonal of
    ``2A``), so conjugate gradients from the previous iterate converge in a handful of steps."""

    def __init__(self, n: int, h: float, smoothness: float, fixed: np.ndarray) -> None:
        self.c1 = 2.0 / h
        self.c2 = 2.0 * smoothness / h ** 3
        diag = np.full(n, 2.0 * self.c1)
        diag[[0, -1]] = self.c1
        k_diag = np.full(n, 6.0)
        k_diag[[0, -1]], k_diag[[1, -2]] = 1.0, 5.0
        self.shift = float(np.mean(diag + self.c2 * k_diag))
        self.free_mask = np.ones(n, dtype=bool)
        self.free_mask[fixed] = False

    def full(self, x: np.ndarray) -> np.ndarray:
        """``(2A + s I) x`` on every point."""
        dx = np.diff(x, axis=0)
        out = self.shift * x
        out[:-1] -= self.c1 * dx
        out[1:] += self.c1 * dx
        if self.c2:
            kx = x[:-2] - 2.0 * x[1:-1] + x[2:]
            out[:-2] += self.c2 * kx
            out[1:-1] -= 2.0 * self.c2 * kx
            out[2:] += self.c2 * kx
        return out

    def solve(self, rhs: np.ndarray, fixed_values: np.ndarray, x0: np.ndarray) -> np.ndarray:
        """Free points of the solution of ``M x = rhs`` with the fixed points held at their values."""
        mask = self.free_mask[:, None]
        base = np.where(mask, 0.0, fixed_values)
        b = np.where(mask, rhs - self.full(base), 0.0)
        x = np.where(mask, x0, 0.0)
        r = b - np.where(mask, self.full(x), 0.0)
        p = r.copy()
        rr = (r * r).sum(axis=0)
        tol = 1e-20 * max(float((b * b).sum()), 1e-30)
        for _ in range(200):
            if rr.sum() <= tol:
                break
            mp = np.where(mask, self.full(p), 0.0)
            alpha = rr / np.maximum((p * mp).sum(axis=0), 1e-300)
            x += alpha * p
            r -= alpha * mp
            rr_new = (r * r).sum(axis=0)
            p = r + (rr_new / np.maximum(rr, 1e-300)) * p
            rr = rr_new
        return np.where(mask, x, fixed_values)


def smooth_path(points: np.ndarray, anchors: Sequence[int], gates: Sequence[Gate], cs: CSpace,
                spacing: float, smoothness: float, clearance: float = 0.01,
                iterations: int = 400, respace_every: int = 20) -> np.ndarray:
    """Smooth a resampled path.  ``anchors`` = [start, gate 1, ..., gate k, goal] indices."""
    p = points.copy()
    n = len(p)
    h = spacing
    fixed = np.array([0, n - 1])
    system = _StepSystem(n, h, smoothness, fixed)
    step_weight = system.shift  # 1/eta: each step goes about halfway towards the quadratic optimum
    w = step_weight / (2.0 * h)  # obstacle stiffness matched to the step, so the explicit term is stable
    gate_index = list(zip(anchors[1:-1], gates))

    for it in range(iterations):
        # Penalise the points and the segment midpoints (a chord can cut a corner between two
        # points that are both clear); a midpoint's push is shared by its two end points.
        d, grad = cs.distance_fast(np.concatenate([p, 0.5 * (p[:-1] + p[1:])]))
        viol = np.clip(clearance - d, 0.0, None)[:, None] * grad
        obstacle_grad = -2.0 * w * h * viol[:n]
        obstacle_grad[:-1] -= w * h * viol[n:]
        obstacle_grad[1:] -= w * h * viol[n:]
        new = system.solve(step_weight * p - obstacle_grad, p, p)
        new = _push_out(new, cs.distance_fast, clearance, fixed, gate_index)
        moved = float(np.abs(new - p).max())
        p = new
        # Re-spacing cuts corners slightly, so stop doing it well before the end: the last
        # iterations' projections then leave every point outside the obstacles.
        if respace_every and (it + 1) % respace_every == 0 and it < 0.8 * iterations:
            sections = [0] + [i for i, _ in gate_index] + [n - 1]
            for s0, s1 in zip(sections[:-1], sections[1:]):
                p[s0:s1 + 1] = _respace(p[s0:s1 + 1], s1 - s0 + 1)
        if moved < 1e-6:
            break
    # Final repair with the exact distance: the grid lookup is a few millimetres optimistic at
    # corners, and the last smoothing step may leave a chord just inside one.
    for _ in range(50):
        d, _ = cs.distance(np.concatenate([p, 0.5 * (p[:-1] + p[1:])]))
        if d.min() >= 0.0:
            break
        p = _push_out(p, cs.distance, clearance, fixed, gate_index)
    return p


def _push_out(p: np.ndarray, distance, clearance: float, fixed: np.ndarray, gate_index) -> np.ndarray:
    """Move points (and both ends of any segment whose midpoint is) inside an obstacle back out to
    half the clearance, keeping the fixed points and putting gate points back on their gates."""
    n = len(p)
    out = p.copy()
    for i, gate in gate_index:
        out[i] = gate.project(out[i])
    d, grad = distance(np.concatenate([out, 0.5 * (out[:-1] + out[1:])]))
    push = np.clip(0.5 * clearance - d, 0.0, None)[:, None] * grad * (d < 0.0)[:, None]
    shift = push[:n].copy()
    shift[:-1] += push[n:]
    shift[1:] += push[n:]
    shift[fixed] = 0.0
    out += shift
    for i, gate in gate_index:
        out[i] = gate.project(out[i])
    return out
