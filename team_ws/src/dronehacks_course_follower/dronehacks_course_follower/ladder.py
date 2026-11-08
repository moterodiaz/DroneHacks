"""The speed ladder across runs: each run of the course flies the ladder's current step, and a run
that lands and disarms on the pad moves the next one up a step.

The stack launches team code with no arguments, so the step lives in a small JSON file in the team
workspace (`/root/team_ws` in the container, `~/team_ws` on the host): `{"step": N, "runs": [...]}`.
Editing `step` by hand repeats or skips a speed; deleting the file starts again at the slowest.
"""
from __future__ import annotations

import json
import time
from collections.abc import Sequence
from pathlib import Path


def parse_steps(text: str) -> tuple[float, ...]:
    """The ladder's top speeds from a comma list, m/s; raises `ValueError` unless they rise."""
    steps = tuple(float(step) for step in text.split(",") if step.strip())
    if not steps or any(step <= 0.0 for step in steps) or list(steps) != sorted(set(steps)):
        raise ValueError(f"the speed ladder must be positive speeds that rise, not {text!r}")

    return steps


def read_step(path: Path, count: int) -> int:
    """The step the next run flies: the file's, pulled into range, or the first without a readable file."""
    try:
        step = int(json.loads(path.read_text(encoding="utf-8"))["step"])
    except (OSError, ValueError, KeyError, TypeError):
        return 0

    return min(max(step, 0), count - 1)


def record_run(path: Path, step: int, steps: Sequence[float], landed: bool) -> int:
    """Records a run flown at `step` and returns the next run's step: one up after a landing on the
    pad, the same otherwise."""
    try:
        runs = list(json.loads(path.read_text(encoding="utf-8")).get("runs", []))
    except (OSError, ValueError, AttributeError):
        runs = []

    next_step = min(step + 1, len(steps) - 1) if landed else step
    runs.append({"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "step": step,
                 "speed_mps": steps[step], "landed": landed})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"step": next_step, "runs": runs}, indent=1), encoding="utf-8")
    return next_step
