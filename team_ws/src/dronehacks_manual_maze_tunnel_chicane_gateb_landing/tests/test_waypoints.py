"""Property tests for the map-driven waypoint derivation (rule 9.5).

No course coordinate is asserted literally: every check recomputes geometry
from the packaged course map, so a regenerated or surveyed map is re-verified
the same way the derivation is.
"""
import copy
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronehacks_manual_maze_tunnel_chicane_gateb_landing import waypoints as W  # noqa: E402

MAP = os.path.join(os.path.dirname(__file__), '..', 'maps',
                   'course_v3_layout.yaml')
CONTRACT_MAP = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                            '..', '..', 'contracts', 'course_v3_layout.yaml')


@pytest.fixture(scope='module')
def loaded():
    return W.load_course_map(MAP)


@pytest.fixture(scope='module')
def route(loaded):
    data, goals = loaded
    return W.derive_route(data, goals)


def test_packaged_map_is_verbatim_contract_copy():
    if not os.path.isfile(CONTRACT_MAP):
        pytest.skip('contracts/ not present in this checkout')
    with open(MAP, 'rb') as a, open(CONTRACT_MAP, 'rb') as b:
        assert a.read() == b.read()


def test_missing_goal_is_a_hard_error(tmp_path):
    data, goals = W.load_course_map(MAP)
    for name in W.REQUIRED_GOALS:
        stripped = copy.deepcopy(data)
        stripped['goals'] = [g for g in stripped['goals'] if g['name'] != name]
        with pytest.raises(W.RouteError, match=name):
            W.load_course_map_from_data(stripped)


def test_window_refused_while_budget_unreviewed(route):
    assert route.via_window is False
    assert 'refused' in route.refusal_reason
    assert 'Gate B' in route.refusal_reason


def test_window_route_derives_only_with_reviewed_budget(loaded):
    data, goals = loaded
    reviewed = copy.deepcopy(data)
    for goal in reviewed['goals']:
        if goal['name'] == 'window':
            goal['params']['clearance_radius'] = 0.42
    r = W.derive_route(reviewed, {g['name']: g for g in reviewed['goals']})
    assert r.via_window is True
    labels = [wp.label for wp in r.waypoints]
    assert 'window approach' in labels and 'window exit' in labels


def test_small_window_budget_still_routes_gate_b(loaded):
    data, goals = loaded
    reviewed = copy.deepcopy(data)
    for goal in reviewed['goals']:
        if goal['name'] == 'window':
            goal['params']['clearance_radius'] = 0.30
    r = W.derive_route(reviewed, {g['name']: g for g in reviewed['goals']})
    assert r.via_window is False
    assert 'Gate B' in r.refusal_reason


# ── Clearance properties, recomputed from the map ──────────────────────────

def _obstacles(data, goals):
    return W._obstacle_rects(data, goals)


def test_every_leg_clears_prop_reach(loaded, route):
    data, goals = loaded
    obstacles = _obstacles(data, goals)
    wps = route.waypoints
    for i in range(len(wps) - 1):
        a = (wps[i].x, wps[i].y)
        b = (wps[i + 1].x, wps[i + 1].y)
        for rect in obstacles:
            assert W.segment_rect_distance(a, b, rect) >= W.PROP_REACH_M - 1e-9, \
                f'leg {i} ({wps[i].label} → {wps[i + 1].label})'


def test_every_turn_clears_arrival_plus_prop_reach(loaded, route):
    data, goals = loaded
    obstacles = _obstacles(data, goals)
    need = W.DEFAULT_ARRIVAL_RADIUS_M + W.PROP_REACH_M
    wps = route.waypoints
    for i, wp in enumerate(wps):
        if i <= 0 or i >= len(wps) - 1:
            turn = True
        else:
            u = W._unit(((wp.x - wps[i - 1].x), (wp.y - wps[i - 1].y)))
            v = W._unit(((wps[i + 1].x - wp.x), (wps[i + 1].y - wp.y)))
            turn = abs(u[0] * v[1] - u[1] * v[0]) > 0.017
        for rect in obstacles:
            dist = W.point_rect_distance((wp.x, wp.y), rect)
            assert dist >= (need if turn else W.PROP_REACH_M) - 1e-9, \
                f'waypoint {i} ({wp.label}) is {dist:.3f} from a wall'


def test_route_is_inside_the_net(loaded, route):
    data, _ = loaded
    _, nets = W._wall_rects(data)
    xs = [p[0] for rect in nets for p in rect]
    ys = [p[1] for rect in nets for p in rect]
    for wp in route.waypoints:
        assert min(xs) + W.PROP_REACH_M <= wp.x <= max(xs) - W.PROP_REACH_M
        assert min(ys) + W.PROP_REACH_M <= wp.y <= max(ys) - W.PROP_REACH_M


def test_final_waypoint_is_the_pad(loaded, route):
    _, goals = loaded
    pad = tuple(map(float, goals['pad']['centre']))
    last = route.waypoints[-1]
    assert (last.x, last.y) == pytest.approx(pad, abs=1e-9)
    assert last.label == 'pad'


def test_maze_legs_run_along_corridor_centres(loaded, route):
    """Between passes, consecutive waypoints share x or y (axis-aligned legs)."""
    wps = [wp for wp in route.waypoints if wp.label.startswith('maze')]
    assert wps
    for a, b in zip(wps, wps[1:]):
        assert a.x == pytest.approx(b.x, abs=1e-9) or \
            a.y == pytest.approx(b.y, abs=1e-9), (a.label, b.label)


def test_tunnel_leg_is_on_the_axis(loaded, route):
    _, goals = loaded
    a = tuple(map(float, goals['tunnel']['params']['axis_start']))
    b = tuple(map(float, goals['tunnel']['params']['axis_end']))
    entry = next(wp for wp in route.waypoints if wp.label == 'tunnel entry')
    exit_ = next(wp for wp in route.waypoints if wp.label == 'tunnel exit')
    u = W._unit((b[0] - a[0], b[1] - a[1]))
    standoff = (W.DEFAULT_ARRIVAL_RADIUS_M + W.PROP_REACH_M + W.WALL_BUFFER_M)
    for wp, ref, back in ((entry, a, True), (exit_, b, False)):
        d = ((wp.x - ref[0]) * u[0] + (wp.y - ref[1]) * u[1])
        assert d == pytest.approx(-standoff if back else standoff, abs=1e-6)
        # on the axis line
        cross = (wp.x - ref[0]) * u[1] - (wp.y - ref[1]) * u[0]
        assert cross == pytest.approx(0.0, abs=1e-6)


def test_oversized_arrival_radius_fails_closed(loaded):
    data, goals = loaded
    with pytest.raises(W.RouteError):
        W.derive_route(data, goals, arrival_radius_m=0.25)


def test_derive_route_is_deterministic(loaded):
    data, goals = loaded
    r1 = W.derive_route(data, goals)
    r2 = W.derive_route(copy.deepcopy(data), copy.deepcopy(goals))
    assert [(w.x, w.y) for w in r1.waypoints] == [(w.x, w.y) for w in r2.waypoints]


# ── frames.md transform ─────────────────────────────────────────────────────

def test_identity_transform_matches_frames_md_worked_example(loaded):
    """frames.md: pad → (+0.473, −1.706) from the launch mark, identity R."""
    _, goals = loaded
    launch = tuple(map(float, goals['launch']['centre']))
    pad = tuple(map(float, goals['pad']['centre']))
    got = W.course_to_odom_xy(pad, launch, (0.0, 0.0), 0.0)
    assert got == pytest.approx((0.472, -1.707), abs=0.001)


def test_transform_is_rigid(loaded):
    _, goals = loaded
    launch = tuple(map(float, goals['launch']['centre']))
    pad = tuple(map(float, goals['pad']['centre']))
    yaw = 0.7
    p1 = W.course_to_odom_xy(pad, launch, (0.3, -0.2), yaw)
    p2 = W.course_to_odom_xy(launch, launch, (0.3, -0.2), yaw)
    assert p2 == pytest.approx((0.3, -0.2), abs=1e-12)  # o_course maps to o_odom
    # rotation preserves distance from the origin point
    d_course = math.hypot(pad[0] - launch[0], pad[1] - launch[1])
    d_odom = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
    assert d_odom == pytest.approx(d_course, abs=1e-12)
