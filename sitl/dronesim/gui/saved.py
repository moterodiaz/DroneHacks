"""Saved runs as map overlays: the planned path and the flown trail of runs saved earlier.

A saved run is a folder written by ``Simulation.save`` (GUI Save, CLI ``run --out``): ``result.json``,
``trajectory.csv`` (the true course-frame pose over time) and, for versions with a plan observer,
``plan.json``. The GUI's **Load paths...** button and ``dronesim gui --load DIR`` take one run's
folder, or a folder of run folders, and draw each run over the map: its plan dashed, its trail solid,
in one colour per run, with a legend. Nothing here needs Tk.
"""
from __future__ import annotations

import csv
import json
import math
import os

#: one colour per loaded run, in load order (cycled); none is the live trail's blue or the plan's orange
COLOURS = ('#8e24aa', '#00897b', '#3949ab', '#c0ca33', '#6d4c41', '#d81b60', '#00acc1', '#f4511e')
MAX_TRAIL_POINTS = 1500
MAX_PLAN_POINTS = 400


def is_run(path):
    return os.path.isfile(os.path.join(path, 'trajectory.csv')) or os.path.isfile(os.path.join(path, 'plan.json'))


def find_runs(path):
    """The run folders at ``path``: itself if it is one, else its sub-folders that are, sorted by name."""
    if is_run(path):
        return [path]
    if not os.path.isdir(path):
        return []
    return [os.path.join(path, d) for d in sorted(os.listdir(path)) if is_run(os.path.join(path, d))]


def _thin(points, max_points):
    if len(points) <= max_points:
        return list(points)
    step = int(math.ceil(len(points) / float(max_points)))
    out = list(points[::step])
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out


def _read_json(path):
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _trail(path, min_step_m=0.01):
    """Course-frame (x, y) of the drone while it was armed, from ``trajectory.csv``."""
    out = []
    try:
        with open(path, 'r', encoding='utf-8', newline='') as fh:
            for row in csv.DictReader(fh):
                if str(row.get('armed', '1')).strip() in ('0', 'False', 'false'):
                    continue
                p = (float(row['x']), float(row['y']))
                if not out or math.dist(out[-1], p) >= min_step_m:
                    out.append(p)
    except (OSError, KeyError, ValueError):
        return []
    return out


def label_for(run_dir, result, plan):
    """A short legend label: the folder name, the flight line, the seed and the referee's outcome."""
    parts = [os.path.basename(os.path.normpath(run_dir))]
    sc = (result or {}).get('scenario') or {}
    opts = sc.get('options') or {}
    if opts.get('line'):
        parts.append(str(opts['line']))
    if sc.get('seed') is not None:
        parts.append(f"seed {sc['seed']}")
    ref = (result or {}).get('referee') or {}
    if ref:
        pts = ref.get('checkpoint_points')
        bonus = ref.get('window_bonus') if ref.get('finished') and ref.get('window_route') else 0
        outcome = f"{pts:g}" + (f"+{bonus}" if bonus else '') + ' pts' if pts is not None else ''
        if ref.get('finished') and ref.get('time_s') is not None:
            outcome += f", {ref['time_s']:.1f} s"
        if ref.get('touches'):
            outcome += f", {ref['touches']} touches"
        if ref.get('ended'):
            outcome += ', ended early'
        parts.append(outcome)
    elif plan and plan.get('duration_s') is not None:
        parts.append(f"plan {plan['length_m']:.1f} m / {plan['duration_s']:.1f} s")
    return ': '.join([parts[0], ', '.join(p for p in parts[1:] if p)]) if len(parts) > 1 else parts[0]


def load_run(run_dir, colour=COLOURS[0]):
    """One overlay: ``{'label', 'colour', 'arena', 'plan' [(x, y)], 'trail' [(x, y)], 'dir'}``."""
    result = _read_json(os.path.join(run_dir, 'result.json'))
    plan = _read_json(os.path.join(run_dir, 'plan.json'))
    arena = ((result or {}).get('arena') or {}).get('name')
    return {
        'dir': run_dir,
        'label': label_for(run_dir, result, plan),
        'colour': colour,
        'arena': arena,
        'plan': _thin([tuple(p) for p in (plan or {}).get('xy') or []], MAX_PLAN_POINTS),
        'trail': _thin(_trail(os.path.join(run_dir, 'trajectory.csv')), MAX_TRAIL_POINTS),
    }


def load_paths(paths, start_index=0):
    """Overlays for every run found under ``paths``, coloured in order from ``start_index``."""
    out = []
    for p in paths:
        for d in find_runs(p):
            out.append(load_run(d, COLOURS[(start_index + len(out)) % len(COLOURS)]))
    return out


def common_arena(overlays):
    """The arena all overlays were flown in, if they agree (None otherwise or if unknown)."""
    names = {o.get('arena') for o in overlays}
    return names.pop() if len(names) == 1 else None
