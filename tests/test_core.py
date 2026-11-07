import math

import numpy as np
import pytest

from core.costmap import LETHAL_CLEARANCE, build_costmap
from core.frames import course_to_odom, enu_to_ned, flu_to_frd, odom_to_course
from core.planner import plan_route
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
