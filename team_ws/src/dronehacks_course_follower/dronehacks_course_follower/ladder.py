"""The speed ladder across runs: each run of the course flies the ladder's current step, which only
the team moves, by hand, between runs; a run records its outcome and leaves the step alone.

A step is a top speed for the smooth follower, or `stop:<m/s>` for stop-and-go: straight legs at
that speed with a hold at every waypoint; a `+return` suffix adds the return to the launch mark
after the pad landing. The ladder rises by speed, then smooth over stop-and-go, then with the
return over without, and at its top it stays there. The stack launches team code with no arguments, so the
step lives in a small JSON file in the team workspace (`/root/team_ws` in the container, `~/team_ws`
on the host): `{"step": N, "runs": [...]}`. Editing `step` by hand repeats or skips a step;
deleting the file starts again at the bottom.
"""
from __future__ import annotations

import json
import time
from collections.abc import Sequence
from pathlib import Path
from typing import NamedTuple

# The prefix that makes a ladder entry stop-and-go, and the suffix that adds the return
STOP_PREFIX = "stop:"
RETURN_SUFFIX = "+return"

# The ladder the stack flies: stop-and-go at the square test's 0.3 m/s first, then the smooth
# follower up to PX4's MPC_XY_VEL_MAX on the kit, 1.0 m/s, and at the top the return added
DEFAULT_LADDER = "stop:0.3,0.5,0.7,0.85,1.0,1.0+return"

# The route modes a launch can force; empty leaves it to the ladder
ROUTE_MODES = ("", "stop", "smooth")

# The return a launch can force; empty leaves it to the ladder
RETURN_CHOICES = ("", "true", "false")


class LadderStep(NamedTuple):
    """One step of the ladder."""

    speed_mps: float          # the smooth follower's top speed, or stop-and-go's leg speed
    stop: bool                # stop-and-go: straight legs with a hold at every waypoint
    return_home: bool = False   # after the pad landing, the return to the launch mark

    def __str__(self) -> str:
        """The step as the ladder writes it."""
        return f"{STOP_PREFIX if self.stop else ''}{self.speed_mps:g}{RETURN_SUFFIX if self.return_home else ''}"


def parse_steps(text: str) -> tuple[LadderStep, ...]:
    """The ladder's steps from a comma list such as `stop:0.3,0.5,1.0,1.0+return`; raises `ValueError`
    unless the speeds are positive and the steps rise."""
    try:
        steps = tuple(_step(entry.strip()) for entry in text.split(",") if entry.strip())
    except ValueError as failure:
        raise ValueError(f"the speed ladder must be speeds, stop:<speed> or either +return, not {text!r}") from failure

    ranks = [(step.speed_mps, not step.stop, step.return_home) for step in steps]
    if not steps or any(step.speed_mps <= 0.0 for step in steps) or ranks != sorted(set(ranks)):
        raise ValueError(f"the speed ladder must be positive speeds that rise, not {text!r}")

    return steps


def _step(entry: str) -> LadderStep:
    """One ladder entry: `stop:<m/s>` or a plain speed, either with `+return`."""
    return_home = entry.endswith(RETURN_SUFFIX)
    entry = entry.removesuffix(RETURN_SUFFIX)
    if entry.startswith(STOP_PREFIX):
        return LadderStep(float(entry[len(STOP_PREFIX):]), stop=True, return_home=return_home)

    return LadderStep(float(entry), stop=False, return_home=return_home)


def chosen_run(step: LadderStep | None, speed_mps: float, stop_speed_mps: float, route_mode: str,
               return_home: str) -> LadderStep:
    """What this run flies: the ladder's `step`, or with none a smooth run at `speed_mps`, with the
    route mode (`stop` at `stop_speed_mps`, or `smooth`) and the return (`true` or `false`) forced
    where the launch gives them. Raises `ValueError` for any other mode or return."""
    if route_mode not in ROUTE_MODES or return_home not in RETURN_CHOICES:
        raise ValueError(f"route_mode must be one of {ROUTE_MODES} and return_home one of {RETURN_CHOICES}, "
                         f"not {route_mode!r} and {return_home!r}")

    base = step if step is not None else LadderStep(speed_mps, stop=False)
    stop = base.stop if route_mode == "" else route_mode == "stop"
    speed = stop_speed_mps if stop and not base.stop else base.speed_mps
    return LadderStep(speed, stop, base.return_home if return_home == "" else return_home == "true")


def read_step(path: Path, count: int) -> int:
    """The step the next run flies: the file's, pulled into range, or the first without a readable file."""
    try:
        step = int(json.loads(path.read_text(encoding="utf-8"))["step"])
    except (OSError, ValueError, KeyError, TypeError):
        return 0

    return min(max(step, 0), count - 1)


def record_run(path: Path, step: int, steps: Sequence[LadderStep], landed: bool) -> None:
    """Records a run flown at `step` and its outcome, leaving the step where it is: the team moves it
    by hand, so no run flies faster than they chose."""
    try:
        runs = list(json.loads(path.read_text(encoding="utf-8")).get("runs", []))
    except (OSError, ValueError, AttributeError):
        runs = []

    runs.append({"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "step": step,
                 "speed_mps": steps[step].speed_mps, "mode": "stop" if steps[step].stop else "smooth",
                 "return": steps[step].return_home, "landed": landed})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"step": step, "runs": runs}, indent=1), encoding="utf-8")
