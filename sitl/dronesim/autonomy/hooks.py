"""Follower injections: run inside a node's process before its ``main()``.

Some autonomy versions fix their follower in code (``dronehacks_traj`` builds
``dronehacks_planner.follower.Follower`` with a default ``FollowerConfig``; no
ROS parameter selects another).  To compare followers on the *same* unmodified
mission, the SITL patches the module the node imports, exactly as the planner's
bench does offline.  These are simulator-only: the GUI labels every injected
follower as such, because the drone would run the shipped default.
"""
from __future__ import annotations

import dataclasses
import functools


def traj_follower_config(**overrides):
    """Make dronehacks_traj's mission build its Follower with FollowerConfig(**overrides)."""
    from dronehacks_planner.follower import FollowerConfig

    import dronehacks_traj.mission_node as mission_node
    cfg = dataclasses.replace(FollowerConfig(), **overrides)
    original = mission_node.prepare
    mission_node.prepare = functools.partial(original, follower=cfg)
    return f'dronehacks_traj mission: Follower built with {cfg}'


def traj_waypoint_follower(arrival_radius_m=0.3, done_radius_m=0.10):
    """Replace the trajectory Follower by the kit-style WaypointFollower (bench baseline "A").

    Waypoints are the planner's own route waypoints (as in dronehacks_planner.bench),
    one pose at a time, advancing within ``arrival_radius_m``.
    """
    from dronehacks_planner.follower import WaypointFollower

    import dronehacks_traj.mission as mission
    routes = {}
    original_init = mission.Mission.__init__

    def init(self, plan, *args, **kwargs):
        original_init(self, plan, *args, **kwargs)
        route = getattr(plan, 'route', None)
        traj = getattr(plan, 'trajectory', None)
        if route is not None and traj is not None:
            routes[id(traj)] = [w.position for w in route.waypoints]

    def factory(traj, field, limits=None, cfg=None):
        wps = routes.get(id(traj))
        if wps is None:
            wps = [p for p in traj.xy[::max(1, len(traj.xy) // 20)]]
        return WaypointFollower(wps, arrival_radius_m=arrival_radius_m, done_radius_m=done_radius_m)

    mission.Mission.__init__ = init
    mission.Follower = factory
    return f'dronehacks_traj mission: kit-style WaypointFollower (arrival {arrival_radius_m} m)'


HOOKS = {
    'traj_follower_config': traj_follower_config,
    'traj_waypoint_follower': traj_waypoint_follower,
}
