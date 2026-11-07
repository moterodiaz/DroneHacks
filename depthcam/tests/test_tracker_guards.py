"""``lidar_localize.Tracker`` guards for the live tools (``LIVE_GUARDS``): a wrong fit is "lost", never a pose.

A bench run of 19_lidar_px4_bridge.py rejected good matches at the start (its guess drifted with PX4's
unfused velocity), then accepted a false lock (24 cm fit, 17 % of the beams) and sent it to PX4 for a
minute.  These cases rebuild that in a 20_bench_map.py room with simulated scans.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from course.course_map import CourseMap
from course.lidar_localize import LIVE_GUARDS, DistanceField, MatchConfig, Tracker
from course.lidar_sim import LidarSpec, Scene, Trajectory, simulate

REPO_ROOT = Path(__file__).resolve().parents[1]
ROOM, START, HEIGHT, MOUNT_Z = (4.2, 6.1), (2.1, 1.0), 0.95, 0.10


def _room(size, start, boxes=()):
    spec = importlib.util.spec_from_file_location("bench_map", REPO_ROOT / "course" / "20_bench_map.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return CourseMap(mod.room_map(*size, start, 2.5, list(boxes)))


@pytest.fixture(scope="module")
def room() -> CourseMap:
    return _room(ROOM, START, [(3.4, 2.0, 4.2, 2.6, 1.2)])


@pytest.fixture(scope="module")
def field(room) -> DistanceField:
    return DistanceField(room, HEIGHT + MOUNT_Z)


def _scans(cmap, way, seconds, seed=0):
    w = np.array(way, dtype=float)
    t = np.arange(0.0, seconds + 0.2, 0.01)
    pos = np.stack([np.interp(t, w[:, 0], w[:, 1]), np.interp(t, w[:, 0], w[:, 2]), np.full(len(t), HEIGHT)], axis=1)
    yaw = np.radians(np.interp(t, w[:, 0], w[:, 3]))
    traj = Trajectory(t, pos, np.stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw], axis=1))
    return simulate(traj, Scene.from_map(cmap), LidarSpec(mount=(0.0, 0.0, MOUNT_Z)), 0.0, seconds, seed=seed)


def _tracker(field, start, guarded):
    return Tracker(field, MatchConfig(mount=(0.0, 0.0, MOUNT_Z)), np.array(start, dtype=float),
                   **(LIVE_GUARDS if guarded else {}))


def _run(tracker, scans, vel=None):
    out = []
    for s in scans:
        pose, st = tracker.update(s.t, s.range, s.angle, np.zeros((len(s.t), 2)), HEIGHT, vel)
        out.append((pose.copy(), st, s))
    return out


def _err(pose, scan):
    return math.hypot(pose[0] - scan.true_pos[-1, 0], pose[1] - scan.true_pos[-1, 1])


def test_a_start_off_the_mark_is_acquired(room, field) -> None:
    """Rig 0.5 m and 25 deg from where the tracker assumes: the old gates (0.3 m, 17 deg) never let go."""
    scans = _scans(room, [(0, 2.6, 1.0, 115), (3, 2.6, 1.0, 115)], 3.0)
    old = _run(_tracker(field, (*START, math.pi / 2), False), scans)
    assert all("lost" in st for _, st, _ in old)
    new = _run(_tracker(field, (*START, math.pi / 2), True), scans)
    first = next(i for i, (_, st, _) in enumerate(new) if "lost" not in st)
    assert first == 0 and new[0][1].get("acquired") == 1
    assert max(_err(p, s) for p, st, s in new) < 0.02


def test_a_drifting_external_velocity_does_not_drag_the_guess(room, field) -> None:
    """PX4's velocity before it fuses the lidar (0.3 m/s here) must not move the guess off a still rig."""
    scans = _scans(room, [(0, *START, 90), (3, *START, 90)], 3.0)
    res = _run(_tracker(field, (*START, math.pi / 2), True), scans, vel=np.array([0.2, 0.22]))
    assert all("lost" not in st for _, st, _ in res)
    assert max(_err(p, s) for p, st, s in res) < 0.02


def test_scans_of_another_room_are_never_trusted(field) -> None:
    """A scene that is not the map (a 2.5 x 3 m room): every match is a poor fit or a jump, none a pose."""
    other = _room((2.5, 3.0), (1.25, 1.0))
    scans = _scans(other, [(0, 1.25, 1.0, 90), (2, 1.25, 1.0, 90), (4, 1.25, 2.0, 90)], 4.0)
    res = _run(_tracker(field, (*START, math.pi / 2), True), scans)
    assert all("lost" in st for _, st, _ in res)
    assert {st["reason"] for _, st, _ in res} <= {"poor fit", "jump", "failed"}


def test_a_dropout_while_moving_is_reacquired(room, field) -> None:
    """Lidar blind for 1 s while the rig moves 0.6 m: lost, then re-acquired where the rig really is."""
    scans = _scans(room, [(0, *START, 90), (1, *START, 90), (2.5, 2.1, 1.6, 90), (4, 2.1, 1.6, 90)], 4.0)
    for s in scans:
        if 1.0 <= s.t_end < 2.0:
            s.range[:] = np.nan                                 # nothing back: match() fails
    res = _run(_tracker(field, (*START, math.pi / 2), True), scans)
    lost_t = [s.t_end for _, st, s in res if "lost" in st]
    assert lost_t and min(lost_t) >= 1.0
    back = [(p, st, s) for p, st, s in res if s.t_end >= 2.5]
    assert any(st.get("acquired") for _, st, sc in res if sc.t_end >= 2.0)  # re-acquired, not just at the start
    assert all("lost" not in st for _, st, _ in back) and max(_err(p, s) for p, st, s in back) < 0.02


def test_a_normal_walk_is_unchanged(room, field) -> None:
    """With the rig where the tracker expects it, the guards change nothing."""
    scans = _scans(room, [(0, *START, 90), (1, *START, 90), (4, 2.1, 2.5, 90), (5.5, 2.1, 2.5, 30), (7, 2.8, 2.5, 30)], 7.0)
    old = _run(_tracker(field, (*START, math.pi / 2), False), scans)
    new = _run(_tracker(field, (*START, math.pi / 2), True), scans)
    assert all("lost" not in st for _, st, _ in new)
    assert max(np.hypot(*(a[:2] - b[:2])) for (a, _, _), (b, _, _) in zip(old, new)) < 0.001
