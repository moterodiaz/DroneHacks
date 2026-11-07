import math
import unittest

try:
    import numpy as np
    import pytest
except ModuleNotFoundError as exc:  # the CI sandbox installs no third-party packages
    raise unittest.SkipTest("third-party test dependencies missing: {}".format(exc))

from core.costmap import LETHAL_CLEARANCE, RASTER_BUFFER, Costmap, build_costmap, obstacle_clearance
from core.frames import course_to_odom, enu_to_ned, flu_to_frd, odom_to_course
from core.planner import _chord_acceptable, _line_cells, _prune_goal_snap, plan_route, shortcut_path
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


def test_window_portal_cells_keep_reviewed_clearance():
    course = synthetic_map()
    costmap = build_costmap(course)
    window = course["goals"][2]
    params = window["params"]
    wall = course["segments"][params["wall_segment_index"]]
    centre = np.asarray(window["centre"], dtype=float)
    wall_unit = (np.asarray(wall["end"]) - np.asarray(wall["start"])) / np.linalg.norm(
        np.asarray(wall["end"]) - np.asarray(wall["start"])
    )
    safe_half = params["side"] / 2 - params["clearance_radius"]
    # Every cell the portal frees inside the window wall's own footprint must
    # keep the reviewed clearance radius to the opening edge at every point of
    # the cell, not only at its centre: the old centre-only cut-out freed the
    # cell at lateral offset 0.175 m, whose far edge reaches 0.20 m and leaves
    # only 0.30 m of margin to the opening edge.
    worst = 0.0
    for cell in map(tuple, np.argwhere(np.isfinite(costmap.costs))):
        world = costmap.grid_to_world(cell)
        if _segment_surface(world, wall) > 0:
            continue  # outside the wall's footprint: not a portal-freed cell
        worst = max(worst, abs(float((world - centre) @ wall_unit)))
    assert worst + RASTER_BUFFER <= safe_half + 1e-12


def test_shortcut_never_crosses_blocked_cell():
    costs = np.ones((4, 4))
    costs[(1, 0)] = np.inf  # crossed by the (0,0)->(3,2) chord's interior
    costs[(2, 2)] = np.inf  # crossed by the (1,1)->(3,2) chord's interior
    cells = [(0, 0), (1, 1), (2, 1), (3, 2)]
    # The straight chord (0,0)->(3,2) crosses the interior of blocked (1,0),
    # but the old point-sampled _line_cells returned only (0,0),(1,1),(2,1),
    # (3,2) and shortcut_path replaced the subpath with a segment through it.
    assert (1, 0) in _line_cells((0, 0), (3, 2))
    assert shortcut_path(cells, costs) == cells


def test_chord_acceptable_checks_every_crossed_cell():
    costs = np.ones((4, 4))
    costs[(1, 0)] = np.inf
    costmap = Costmap(costs=costs, origin=(2.0, 2.0), resolution=1.0)  # lower-left corner (0, 0)
    a, b = np.array([0.7, 2.3]), np.array([1.0, 1.8])
    # The chord clips the interior of blocked cell (1, 0) between the old
    # half-resolution sample points; every crossed cell must be checked.
    assert not _chord_acceptable(costmap, a, b, replaced_max=1.0)


def test_fillet_arc_keeps_lethal_clearance():
    # A 150-degree turn with 3 m legs at the 0.35 m radius cap cuts the fillet
    # 0.35 * (1 / cos(75 deg) - 1) = 1.002 m into the corner. A small boulder
    # 1.28 m from the vertex along the interior bisector sits 0.321 m from
    # both legs and 1.27 m from the vertex, so the vertex clearance gate
    # passes, but the arc passes only 0.268 m from its surface: the turn must
    # fail closed instead of returning a trajectory that breaks the 0.32 m rule.
    course = synthetic_map()
    boulder = 1.28 * np.array([-math.sin(math.radians(75)), math.cos(math.radians(75))])

    def clearance_at(p):
        return float(np.linalg.norm(np.asarray(p, dtype=float) - boulder) - 0.01)

    waypoints = [
        (-3.0, 0.0),
        (0.0, 0.0),
        tuple(3.0 * np.array([math.cos(math.radians(150)), math.sin(math.radians(150))])),
    ]
    # The premise of the scenario: both legs and the vertex keep the lethal
    # clearance, so only the arc can violate it.
    leg2 = np.array([math.cos(math.radians(150)), math.sin(math.radians(150))])
    assert abs(boulder[1]) - 0.01 >= LETHAL_CLEARANCE  # distance to leg 1 (the x-axis)
    assert abs(leg2[0] * boulder[1] - leg2[1] * boulder[0]) - 0.01 >= LETHAL_CLEARANCE  # distance to leg 2
    assert clearance_at((0.0, 0.0)) >= LETHAL_CLEARANCE  # vertex clearance
    with pytest.raises(ValueError, match="rounded turn"):
        build_trajectory(waypoints, course, clearance_at=clearance_at)


def test_grid_corner_goal_chord_and_prune():
    from core.planner import _polyline_max_cost

    costmap = Costmap(np.zeros((4, 4)), (0.0, 0.0), 1.0)
    start = np.array([-0.5, -1.5])
    goal = np.array([-1.0, -1.0])
    snap = np.array([-0.5, -0.5])
    for _ in range(3):
        for a, b in ((start, goal), (goal, start)):
            assert _chord_acceptable(costmap, a, b, 0.0)
            assert _polyline_max_cost(costmap, [a, b]) == 0.0
        pruned = _prune_goal_snap([start, snap, goal], costmap, 1.0)
        assert len(pruned) == 2
        assert np.array_equal(pruned[-1], goal)
    costmap.costs[costmap.world_to_grid(goal)] = np.inf
    assert not _chord_acceptable(costmap, start, goal, 0.0)


def test_fillet_gate_checks_chords_between_arc_samples():
    # The fillet gate's checked points are the flown polyline's vertices, but
    # the flown path between two consecutive vertices is a straight chord, and
    # near a point obstacle the chord's midpoint sits measurably closer than
    # either endpoint. With caller-tunable coarse arc sampling (ds = 0.5 m
    # gives 3 samples on this 150-degree turn) a corner placed under the first
    # chord's midpoint keeps the lethal clearance at every fillet vertex yet
    # breaks it on the flown chord between them: the vertex-only gate passed
    # such a turn, the midpoint check must fail it.
    course = synthetic_map()
    turn = math.radians(150.0)
    radius = 0.35  # R_MAX; the vertex gate below leaves it unclamped
    vertex = np.array([0.0, 0.0])
    incoming = np.array([1.0, 0.0])
    outgoing = np.array([math.cos(turn), math.sin(turn)])
    tangent_distance = radius * math.tan(turn / 2.0)
    entry = vertex - incoming * tangent_distance
    centre = entry + radius * np.array([0.0, 1.0])  # left of incoming, cross > 0
    start_angle = math.atan2(*(entry - centre)[::-1])
    arc_count = 3  # max(3, ceil(radius * turn / 0.5)) == 3 for these numbers
    first_arc = centre + radius * np.array(
        [math.cos(start_angle + turn / arc_count), math.sin(start_angle + turn / arc_count)]
    )
    midpoint = (entry + first_arc) / 2.0
    outward = (midpoint - centre) / np.linalg.norm(midpoint - centre)
    obstacle = midpoint + 0.30 * outward  # 0.30 m of clearance under the chord

    def clearance_at(p):
        return float(np.linalg.norm(np.asarray(p, dtype=float) - obstacle))

    waypoints = [(-3.0, 0.0), (0.0, 0.0), tuple(3.0 * outgoing)]
    # The premise of the scenario: every fillet vertex keeps the rule, the
    # flown chord between the first two does not, and the vertex gate clears
    # the radius cap.
    assert clearance_at(vertex) - LETHAL_CLEARANCE >= radius
    assert clearance_at(entry) >= LETHAL_CLEARANCE
    assert clearance_at(first_arc) >= LETHAL_CLEARANCE
    assert clearance_at(midpoint) < LETHAL_CLEARANCE
    with pytest.raises(ValueError, match="rounded turn"):
        build_trajectory(waypoints, course, clearance_at=clearance_at, ds=0.5)


def test_resampling_preserves_safe_fillet_geometry():
    turn, radius = math.radians(150.0), 0.35
    outgoing = np.array([math.cos(turn), math.sin(turn)])
    waypoints = np.array([[-3.0, 0.0], [0.0, 0.0], 3.0 * outgoing])
    entry = np.array([-radius * math.tan(turn / 2), 0.0])
    centre = entry + np.array([0.0, radius])
    fillet = [
        centre + radius * np.array([
            math.cos(-math.pi / 2 + turn * j / 3),
            math.sin(-math.pi / 2 + turn * j / 3),
        ])
        for j in range(4)
    ]
    chord = fillet[1] - fillet[0]
    normal = np.array([-chord[1], chord[0]]) / np.linalg.norm(chord)
    obstacle = fillet[0] + 0.25 * chord + 0.34 * normal

    def clearance_at(p):
        return float(np.linalg.norm(np.asarray(p) - obstacle)) - 0.01

    def segment_clearance(a, b):
        delta = b - a
        t = np.clip(float((obstacle - a) @ delta / (delta @ delta)), 0.0, 1.0)
        return clearance_at(a + t * delta)

    # Both the input polyline and the generated coarse fillet are safe.
    assert min(segment_clearance(a, b) for a, b in zip(waypoints, waypoints[1:])) >= 0.32
    geometry = [waypoints[0], *fillet, waypoints[-1]]
    assert min(segment_clearance(a, b) for a, b in zip(geometry, geometry[1:])) >= 0.32
    checked = [*fillet, *((a + b) / 2 for a, b in zip(fillet, fillet[1:]))]
    assert min(map(clearance_at, checked)) >= 0.32
    for ds in (0.5, 0.25, 0.125, 0.02):
        trajectory = build_trajectory(
            waypoints, synthetic_map(), clearance_at=clearance_at, ds=ds,
        )
        # Analytic distance along every delivered segment, not a sampled oracle.
        assert min(
            segment_clearance(a, b)
            for a, b in zip(trajectory.points, trajectory.points[1:])
        ) >= 0.32
        if ds == 0.5:
            for vertex in geometry:
                assert np.min(np.linalg.norm(trajectory.points - vertex, axis=1)) < 1e-12
        # Repeated public sampling must trace that same safe geometry.
        probes = np.linspace(0.0, trajectory.length, 1001)
        for _ in range(2):
            assert min(clearance_at(trajectory.position_at(s)) for s in probes) >= 0.32
