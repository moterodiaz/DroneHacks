"""Stop-and-go: legs and holds in order, the yaw only while holding, a hold that waits for the
drone or times out, straight legs clear of the walls, and a dry run to the pad."""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from dronehacks_course_follower import dry_run
from dronehacks_course_follower.follower import Drone, HeadingMode
from dronehacks_course_follower.planner import (MARGIN_M, REACH_M, STOP_HOLD_SPACING_M, clearance_m, course_grid,
                                                leg_clear, plan_flight, straight_legs)
from dronehacks_course_follower.stop_and_go import Stage, StopAndGo, StopPlan

# The node's tick
_PERIOD_S = 0.02

# An L-shaped route in open space: 1 m east, then 1 m north, a quarter turn at the corner
_L_ROUTE = [(1.0, 0.0), (1.0, 1.0)]


def _fly(stop_and_go: StopAndGo, seconds: float, drone_at: tuple[float, float] | None = None) -> list[tuple]:
    """Steps it with a drone on its pose facing its heading, or held at `drone_at`; each tick's
    (stage, waypoint index, commanded yaw)."""
    ticks = []
    for _ in range(int(seconds / _PERIOD_S)):
        east, north = stop_and_go.reference() if drone_at is None else drone_at
        pose = stop_and_go.step(_PERIOD_S, Drone(east, north, 0.8, stop_and_go.yaw_rad))
        ticks.append((stop_and_go.stage, stop_and_go.index, pose.yaw_rad))
        if stop_and_go.finished:
            break

    return ticks


def test_it_holds_at_every_waypoint_and_flies_the_legs_between_in_order() -> None:
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, math.pi / 2, StopPlan())
    ticks = _fly(stop_and_go, 60.0)
    order = [tick[:2] for index, tick in enumerate(ticks) if index == 0 or tick[:2] != ticks[index - 1][:2]]
    assert order == [(Stage.HOLD, 0), (Stage.MOVE, 0), (Stage.HOLD, 1), (Stage.MOVE, 1), (Stage.HOLD, 2)]
    assert stop_and_go.finished and stop_and_go.reference() == _L_ROUTE[-1]


def test_it_yaws_only_while_holding_and_no_faster_than_its_turn_rate() -> None:
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, math.pi / 2, StopPlan())
    ticks = _fly(stop_and_go, 60.0)
    for before, after in zip(ticks, ticks[1:], strict=False):
        change = abs(math.remainder(after[2] - before[2], 2 * math.pi))
        assert change <= math.radians(StopPlan().turn_rate_dps) * _PERIOD_S + 1e-9
        if after[0] is Stage.MOVE and before[0] is Stage.MOVE:
            assert change == 0.0

    # Facing north at the start, it turns east for the first leg and north for the second
    first_leg = next(tick for tick in ticks if tick[0] is Stage.MOVE and tick[1] == 0)
    second_leg = next(tick for tick in ticks if tick[0] is Stage.MOVE and tick[1] == 1)
    assert abs(first_leg[2]) < 1e-9 and abs(second_leg[2] - math.pi / 2) < 1e-9


def test_the_turn_waits_until_the_drone_itself_reaches_the_waypoint() -> None:
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, 0.0, StopPlan())
    _fly(stop_and_go, 1.5)

    # Along the first leg the drone trails the pose by 0.25 m
    while stop_and_go.stage is Stage.MOVE:
        stop_and_go.step(_PERIOD_S, Drone(max(stop_and_go.reference()[0] - 0.25, 0.0), 0.0, 0.8, 0.0))

    assert (stop_and_go.stage, stop_and_go.index) == (Stage.HOLD, 1)

    # The pose has reached the corner, but the drone trails it by 0.25 m, as on skybeta-11 at the
    # tunnel's exit: no turn yet, so the drone is not swung round short of the corner
    _fly(stop_and_go, 3.0, drone_at=(0.75, 0.0))
    assert stop_and_go.yaw_rad == 0.0

    # Once the drone is over the corner the turn begins
    _fly(stop_and_go, 1.0, drone_at=(1.0, 0.0))
    assert stop_and_go.yaw_rad > 0.0


def test_with_a_fixed_heading_it_never_yaws() -> None:
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, 2.0, StopPlan(), heading_mode=HeadingMode.FIXED)
    ticks = _fly(stop_and_go, 60.0)
    assert stop_and_go.finished and all(tick[2] == 2.0 for tick in ticks)


def test_a_hold_waits_for_the_drone_to_settle_over_its_waypoint() -> None:
    # Drifted 20 cm off for 5 s, as after a yaw on the real drone: the leg waits, then 1 s settled
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, 0.0, StopPlan())
    _fly(stop_and_go, 5.0, drone_at=(0.0, 0.2))
    assert stop_and_go.stage is Stage.HOLD

    ticks = _fly(stop_and_go, 3.0, drone_at=(0.0, 0.0))
    left_at_s = next(index for index, tick in enumerate(ticks) if tick[0] is Stage.MOVE) * _PERIOD_S
    assert math.isclose(left_at_s, StopPlan().settle_s, abs_tol=2 * _PERIOD_S)


def test_a_hold_that_never_settles_goes_on_after_its_timeout_and_says_so() -> None:
    events: list[str] = []
    stop_and_go = StopAndGo((0.0, 0.0), _L_ROUTE, 0.0, StopPlan(), events=events)
    ticks = _fly(stop_and_go, 20.0, drone_at=(0.0, 0.5))
    left_at_s = next(index for index, tick in enumerate(ticks) if tick[0] is Stage.MOVE) * _PERIOD_S
    assert math.isclose(left_at_s, StopPlan().hold_timeout_s, abs_tol=2 * _PERIOD_S)
    assert len(events) == 1 and "not settled" in events[0]


def _open_course(segments: list[list[float]]) -> dict[str, Any]:
    """A 6 m square course with these walls and nothing else."""
    return {"height_m": 0.8, "segments": segments, "bounds": [-3.0, 3.0, -3.0, 3.0], "boulders": []}


def test_a_straight_leg_through_a_wall_gets_the_a_star_corners_between_its_ends() -> None:
    grid = course_grid(_open_course([[0.0, -1.0, 0.0, 1.0]]))
    legs = straight_legs(grid, (-1.5, 0.0), [(1.5, 0.0)])
    assert len(legs) > 1 and legs[-1] == (1.5, 0.0)
    points = [(-1.5, 0.0), *legs]
    assert all(leg_clear(grid, start, end) for start, end in zip(points, points[1:], strict=False))

    # A clear leg is left as it is
    assert straight_legs(grid, (-1.5, 2.0), [(1.5, 2.0)]) == [(1.5, 2.0)]


def test_no_two_stop_and_go_holds_on_the_packaged_map_crowd_each_other(course_map: dict[str, Any]) -> None:
    # Two holds nearer than the spacing overlap: the second is only a needless yaw and settle,
    # as the hops beside skybeta-11's window leg were, through the window or by Gate B
    for allow_window in (True, False):
        planned = plan_flight(course_map, allow_window=allow_window, tunnel_lead_m=0.75)
        points = [tuple(course_map["launch"][:2]), *planned.stop_route]
        assert min(math.dist(start, end) for start, end in zip(points, points[1:], strict=False)) \
            >= STOP_HOLD_SPACING_M


def test_every_stop_and_go_leg_on_the_packaged_map_is_clear(planned: Any, course_map: dict[str, Any]) -> None:
    points = [tuple(course_map["launch"][:2]), *planned.stop_route]
    grid = course_grid(course_map, boulder_margin_m=0.0)
    assert all(leg_clear(grid, start, end) for start, end in zip(points, points[1:], strict=False))
    assert min(clearance_m(np.linspace(start, end, 20), planned.walls).min()
               for start, end in zip(points, points[1:], strict=False)) > REACH_M + MARGIN_M


def test_the_dry_run_flies_stop_and_go_to_the_pad_and_back() -> None:
    assert dry_run.main(["--steps", "0", "--route-mode", "stop"]) == 0
    assert dry_run.main(["--steps", "0", "--route-mode", "stop", "--return-home"]) == 0
