"""Observers: simulator-only instrumentation that records what a version planned.

Unlike the follower injections (``hooks.py``), an observer changes nothing the node does: it wraps
a function the node calls, passes every argument and result through unchanged, and hands a copy
of the result to the running simulation (``set_sink``). The GUI draws it on the map and a saved
run writes it to ``plan.json``. Observers run inside the node's process before its ``main()``,
like the hooks; one is on for every version that builds ``dronehacks_traj`` (``OBSERVERS``).
"""
from __future__ import annotations

import math

#: the running simulation's receiver of observed plans (one simulation per process at a time)
_SINK = [None]
#: (package, executable) -> observer
OBSERVERS = {}

MARK = '__dronesim_plan_observer__'


def set_sink(fn):
    _SINK[0] = fn


def _emit(record):
    sink = _SINK[0]
    if sink is not None:
        sink(record)


def traj_plan_summary(prepared, map_source=None):
    """Plain data from ``dronehacks_traj.factory.Prepared``: the trajectory's course-frame points
    (``plan.trajectory.xy``, at the map's ``slice_height``), its timing and the route it flies."""
    plan = prepared.plan
    traj = plan.trajectory
    xy = [[round(float(x), 4), round(float(y), 4)] for x, y in traj.xy]
    t = [round(float(v), 3) for v in traj.t]
    speed = [round(float(v), 3) for v in plan.profile.v]
    route = plan.route
    cmap = prepared.cmap
    return {
        'source': 'dronehacks_traj.mission_node.prepare (observed, simulator only)',
        'frame': 'course (x east, y north, m)',
        'map_file': map_source if isinstance(map_source, str) else getattr(cmap, 'source', None),
        'map_sha256': getattr(cmap, 'sha256', None),
        'slice_height_m': float(prepared.cmap.slice_height),
        'goals': list(route.goals),
        'choices': dict(getattr(route, 'choices', {}) or {}),
        'length_m': round(float(traj.length), 3),
        'duration_s': round(float(plan.duration), 3),
        'v_max_mps': float(prepared.v_max_mps),
        'speed_note': prepared.speed_note,
        'alternatives': [str(a) for a in (prepared.alternatives or [])],
        'waypoints': [{'xy': [round(float(w.position[0]), 4), round(float(w.position[1]), 4)], 'kind': w.kind,
                       'goal': w.goal} for w in route.waypoints],
        'xy': xy, 't': t, 'speed_mps': speed,
    }


def traj_plan_observer():
    """Record every plan ``dronehacks_traj``'s mission node prepares (it plans once, at start)."""
    import dronehacks_traj.mission_node as mission_node
    current = mission_node.prepare
    if getattr(current, MARK, False):
        return None

    def prepare(*args, **kwargs):
        prepared = current(*args, **kwargs)
        try:
            _emit(traj_plan_summary(prepared, args[0] if args else kwargs.get('map_source')))
        except Exception as exc:  # noqa: BLE001 - instrumentation must never break the node
            _emit({'error': f'could not read the plan: {exc!r}'})
        return prepared

    setattr(prepare, MARK, True)
    prepare.__wrapped__ = current
    mission_node.prepare = prepare
    return None


OBSERVERS[('dronehacks_traj', 'mission')] = traj_plan_observer


def display_points(plan, max_points=400):
    """The plan's points thinned for drawing: every n-th, plus the last."""
    xy = (plan or {}).get('xy') or []
    if len(xy) <= max_points:
        return list(xy)
    step = int(math.ceil(len(xy) / float(max_points)))
    out = xy[::step]
    if out[-1] != xy[-1]:
        out.append(xy[-1])
    return out
