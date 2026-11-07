"""DroneHacks navigation: course map in, route, smooth trajectory and follower setpoints out.

See planner/README.md. The core is ROS-free (numpy + PyYAML) so it can be
tested anywhere; a ROS 2 node wraps it later.
"""

from .course_map import CourseMap, Goal, MapError, load_map, parse_map
from .route import PlannerConfig, PlanningError, Route, Waypoint, plan

__all__ = [
    "CourseMap", "Goal", "MapError", "load_map", "parse_map",
    "PlannerConfig", "PlanningError", "Route", "Waypoint", "plan",
]
