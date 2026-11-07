import math

import numpy as np
import pytest

from core.costmap import LETHAL_CLEARANCE, Costmap, build_costmap, obstacle_clearance
from core.frames import course_to_odom, enu_to_ned, flu_to_frd, odom_to_course
from core.planner import _prune_goal_snap, plan_route
from core.trajectory import V_MAX, build_trajectory


def synthetic_map():
    return {
        "origin": [0.0, 0.0],
        "slice_height": 0.8,
        "segments": [
            {"start": [-3.0, -3.0], "end": [3.0, -3.0], "thickness": 0.10},
            {"start": [3.0, -3.0], "end": [3.0, 3.0], "thickness": 0.10},
            {"start": [3.0, 3.0], "end": [-3.0, 3.0], "thickness": 0.10},
            {"start": [-3.0, 3.0], "end": [-3.0, -3.0], "thickness": 0.10},
            {"start": [0.8, -3.0], "end": [0.8, 3.0], "thickness": 0.10},
        ],
        "boulders": [{"centre": [-0.1, 1.0], "radius": 0.22}],
        "goals": [
            {"name": "launch", "shape": "point", "centre": [-2.3, -1.5], "params": {"heading_deg": 0}},
            {
                "name": "tunnel",
                "shape": "oval",
                "centre": [-1.2, 0.0],
                "params": {"axis_start": [-1.8, 0.0], "axis_end": [-0.6, 0.0], "width": 1.473, "height": 1.524},
            },
            {
                "name": "window",
                "shape": "rect",
                "centre": [0.8, 0.0],
                "params": {
                    "wall_segment_index": 4,
                    "axis_start": [0.4, 0.0],
                    "axis_end": [1.2, 0.0],
                    "side": 1.0,
                    "sill_height": 0.25,
                    "head_height": 1.30,
                    "clearance_radius": 0.32,
                },
            },
            {"name": "pad", "shape": "disc", "centre": [2.1, 1.4], "params": {"radius": 0.7}},
        ],
    }


def test_frames_contract_worked_example():
    rotation = np.eye(3)
    # The contract's 0.73 m is target floor height 0.80 minus measured reference 0.07.
    result = course_to_odom(
        (-2.042, -0.792, 0.80),
        (-2.515, 0.914, 0.07),
        (0.0, 0.0, 0.0),
        rotation,
    )
    assert np.allclose(result, (0.473, -1.706, 0.73), atol=0.001)
    assert np.allclose(
        odom_to_course(result, (-2.515, 0.914, 0.07), (0, 0, 0), rotation),
        (-2.042, -0.792, 0.80),
    )
    assert enu_to_ned(*result) == pytest.approx((-1.706, 0.473, -0.73))
    assert flu_to_frd(1, 2, 3) == (1, -2, -3)


def test_missing_window_clearance_is_hard_error():
    course = synthetic_map()
    del course["goals"][2]["params"]["clearance_radius"]
    with pytest.raises(ValueError, match="clearance_radius"):
        build_costmap(course)


def _segment_surface(point, segment):
    start, end = np.asarray(segment["start"]), np.asarray(segment["end"])
    delta = end - start
    t = np.clip((point - start) @ delta / (delta @ delta), 0.0, 1.0)
    return np.linalg.norm(point - (start + t * delta)) - segment["thickness"] / 2


def _sample_legs(route, spacing=0.01):
    for a, b in zip(route, route[1:]):
        count = max(2, int(np.linalg.norm(b - a) / spacing) + 1)
        yield from (a + t * (b - a) for t in np.linspace(0, 1, count))


def test_costmap_plan_clearance_and_passage_slack():
    course = synthetic_map()
    costmap = build_costmap(course)
    assert costmap.costs.shape == (122, 122)
    route = plan_route(course, costmap)
    assert np.allclose(route[0], course["goals"][0]["centre"])
    for name in ("tunnel", "window", "pad"):
        centre = np.asarray(next(g["centre"] for g in course["goals"] if g["name"] == name))
        assert np.any(np.linalg.norm(route - centre, axis=1) < 1e-9)

    window = course["goals"][2]
    half_opening = window["params"]["side"] / 2
    clearance = window["params"]["clearance_radius"]
    narrowest_slack = half_opening - clearance
    max_portal_offset = 0.0
    for point in _sample_legs(route):
        obstacle_distances = []
        for index, segment in enumerate(course["segments"]):
            # The window is a portal, so the containing wall is absent inside its opening.
            in_portal = index == window["params"]["wall_segment_index"] and abs(point[1] - window["centre"][1]) <= narrowest_slack + 1e-9
            if not in_portal:
                obstacle_distances.append(_segment_surface(point, segment))
        for boulder in course["boulders"]:
            obstacle_distances.append(np.linalg.norm(point - boulder["centre"]) - boulder["radius"])
        assert min(obstacle_distances) >= LETHAL_CLEARANCE
        if abs(point[0] - window["centre"][0]) < 0.06:
            max_portal_offset = max(max_portal_offset, abs(point[1] - window["centre"][1]))
    assert max_portal_offset <= narrowest_slack


def test_trajectory_sampling_speed_and_tunnel_yaw():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda _: 0.70)
    assert np.all(trajectory.corner_radii >= 0.20)
    assert np.all(trajectory.corner_radii <= 0.35)
    assert np.max(trajectory.speed) <= V_MAX
    tunnel = course["goals"][1]
    tunnel_s = trajectory.s[np.argmin(np.linalg.norm(trajectory.points - tunnel["centre"], axis=1))]
    axis = np.subtract(tunnel["params"]["axis_end"], tunnel["params"]["axis_start"])
    assert trajectory.yaw_at(tunnel_s) == pytest.approx(math.atan2(axis[1], axis[0]))
    assert trajectory.position_at(0) == pytest.approx(route[0])
    assert trajectory.position_at(trajectory.length) == pytest.approx(route[-1])


def test_full_stack_integration_with_real_obstacle_clearance():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda p: obstacle_clearance(p, course))
    assert np.all(trajectory.corner_radii >= 0.20)
    assert np.all(trajectory.corner_radii <= 0.35)
    # The portal-aware clearance at the window opening reflects the free opening.
    window_centre = np.asarray(course["goals"][2]["centre"], dtype=float)
    assert obstacle_clearance(window_centre, course) >= LETHAL_CLEARANCE
    # Every sampled trajectory point keeps the lethal 0.32 m clearance.
    for point in trajectory.points:
        assert obstacle_clearance(point, course) >= LETHAL_CLEARANCE - 1e-6


def test_collinear_interior_waypoint_passes_without_fillet_gate():
    course = synthetic_map()
    # A straight interior waypoint would fail the old unconditional clearance gate.
    trajectory = build_trajectory([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], course, clearance_at=lambda _: 0.001)
    assert len(trajectory.corner_radii) == 0
    assert trajectory.position_at(trajectory.length) == pytest.approx((2.0, 0.0))


def test_infeasible_geometry_fails_closed():
    course = synthetic_map()
    # Legs too short for any fillet radius in the allowed band at a 90-degree turn.
    with pytest.raises(ValueError, match="fillet"):
        build_trajectory([(0.0, 0.0), (0.05, 0.0), (0.05, 0.05)], course, clearance_at=lambda _: 0.70)
    # A turning corner with less clearance than the 0.20 m minimum fillet.
    with pytest.raises(ValueError, match="clearance"):
        build_trajectory([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], course, clearance_at=lambda _: 0.40)


def test_short_leg_turn_clamps_radius_into_band():
    course = synthetic_map()
    # 0.35 m tangent points exceed the 0.6 m legs; the radius clamps to fit instead of raising.
    trajectory = build_trajectory([(0.0, 0.0), (0.6, 0.0), (0.6, 0.6)], course, clearance_at=lambda _: 0.70)
    assert len(trajectory.corner_radii) == 1
    radius = float(trajectory.corner_radii[0])
    assert 0.20 <= radius <= 0.35
    assert radius * math.tan(math.pi / 4) <= 0.6 / 2 + 1e-9


def _turning_waypoints(route):
    for i in range(1, len(route) - 1):
        incoming = route[i] - route[i - 1]
        outgoing = route[i + 1] - route[i]
        cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
        cosine = float(np.clip((incoming @ outgoing) / (np.linalg.norm(incoming) * np.linalg.norm(outgoing)), -1.0, 1.0))
        if abs(cross) > 1e-8 and math.acos(cosine) > 1e-5:
            yield i


def _feasible_speed_step(v_lo, ds, a_lat=1.5):
    return math.sqrt(v_lo**2 + 2.0 * a_lat * ds) - v_lo


def test_velocity_continuous_across_corner_boundaries():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda p: obstacle_clearance(p, course))
    # Globally: no adjacent sample pair jumps more than an a_lat-feasible step.
    for i in range(len(trajectory.speed) - 1):
        step = trajectory.s[i + 1] - trajectory.s[i]
        lo = min(trajectory.speed[i], trajectory.speed[i + 1])
        assert abs(trajectory.speed[i + 1] - trajectory.speed[i]) <= _feasible_speed_step(lo, step) + 1e-9
    # Locally: velocity_at is continuous at +-1e-4 and +-1e-6 around each corner band edge.
    turning = list(_turning_waypoints(route))
    assert len(turning) == len(trajectory.corner_radii)
    for k, i in enumerate(turning):
        vertex_s = trajectory.s[int(np.argmin(np.linalg.norm(trajectory.points - route[i], axis=1)))]
        radius = float(trajectory.corner_radii[k])
        for boundary in (vertex_s - radius, vertex_s, vertex_s + radius):
            for delta in (1e-4, 1e-6):
                v_before = float(np.linalg.norm(trajectory.velocity_at(boundary - delta)))
                v_after = float(np.linalg.norm(trajectory.velocity_at(boundary + delta)))
                lo = min(v_before, v_after)
                assert abs(v_after - v_before) <= _feasible_speed_step(lo, 0.02) + 1e-9


def test_time_profile_respects_caller_bounds():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda p: obstacle_clearance(p, course))
    a_lon, jerk_max = 0.6, 1.2
    t, s, v = trajectory.build_time_profile(a_lon, jerk_max)
    assert t[0] == 0.0 and s[0] == 0.0 and v[0] == 0.0  # start from rest
    assert np.all(np.diff(t) > 0)
    assert np.all(np.diff(s) >= 0)
    assert s[-1] >= trajectory.length - 2e-3
    assert np.max(v) <= V_MAX + 5e-3
    assert v[-1] <= 0.05  # ends at rest
    accel = np.diff(v) / np.diff(t)
    assert np.max(np.abs(accel)) <= a_lon + 1e-6
    jerk = np.diff(accel) / np.diff(t)[:-1]
    assert np.max(np.abs(jerk)) <= jerk_max + 1e-6
    with pytest.raises(ValueError):
        trajectory.build_time_profile(0.0, 1.0)
    with pytest.raises(ValueError):
        trajectory.build_time_profile(0.5, 0.0)


def test_time_profile_never_exceeds_arc_length_speed_cap():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda p: obstacle_clearance(p, course))
    t, s, v = trajectory.build_time_profile(0.6, 1.2)
    tol = 5e-3  # integration/interp error only
    # The positional speed cap (V_MAX plus the corner bands) must hold at
    # every profile sample. The old code sampled its target only at the far
    # end of the look-ahead window, so it accelerated out of corner bands
    # early: measured overshoot 0.0516 m/s over a 0.7246 m/s band cap at
    # s ~= 1.20, while the clamped profile stays >= 0.1186 m/s below it.
    cap = np.interp(s, trajectory.s, trajectory._speed_ceiling)
    assert np.all(v <= cap + tol)
    # Mid-course the time profile must also stay under the arc-length speed
    # profile the default follower flies. The first/last 0.5 m are excluded:
    # there the arc profile carries its smoothstep rest-to-rest envelope,
    # which the time profile replaces with its own jerk-limited ramps, and
    # which no a_lon-bounded profile can track all the way to the end (the
    # envelope speed falls as distance^2 to zero, faster than any linear
    # deceleration can bleed speed).
    mid = (s >= 0.5) & (s <= trajectory.length - 0.5)
    assert np.all(v[mid] <= np.interp(s[mid], trajectory.s, trajectory.speed) + tol)


def test_prune_goal_snap_contract():
    # Hand-built flat costmap: 0.2 m cells, lower-left corner at (1.0, 2.0).
    flat = Costmap(costs=np.ones((8, 8)), origin=(1.8, 2.8), resolution=0.2)
    start = np.array([1.0, 2.25])
    snap = np.array([2.0, 2.25])  # within one resolution of the goal
    goal = np.array([2.0, 2.15])
    # An acceptable chord drops the snapped micro-leg.
    pruned = _prune_goal_snap([start, snap, goal], flat, flat.resolution)
    assert len(pruned) == 2
    # A leg longer than one resolution is never a candidate.
    far = np.array([2.3, 2.15])
    assert len(_prune_goal_snap([start, far, goal], flat, flat.resolution)) == 3
    # The straight chord crosses a cell the bent subpath avoids: keep the snap.
    blocked = np.ones((8, 8))
    blocked[flat.world_to_grid((1.7, 2.1))] = np.inf
    blocked_map = Costmap(costs=blocked, origin=flat.origin, resolution=flat.resolution)
    kept = _prune_goal_snap([start, snap, goal], blocked_map, blocked_map.resolution)
    assert len(kept) == 3


def test_tunnel_yaw_corridor_comes_from_map_width():
    course = synthetic_map()
    route = plan_route(course, build_costmap(course))
    trajectory = build_trajectory(route, course, clearance_at=lambda _: 0.70)
    width = course["goals"][1]["params"]["width"]
    assert trajectory.tunnel_half_width == pytest.approx(width / 2.0)
    # A course map whose tunnel omits width fails closed.
    del course["goals"][1]["params"]["width"]
    with pytest.raises(ValueError, match="width"):
        build_trajectory(route, course, clearance_at=lambda _: 0.70)
