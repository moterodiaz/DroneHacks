"""Course planner: configuration space, route through ordered gates, smoothing, speed profile.

Pipeline (``course.planner.plan.plan_course``):

1. ``cspace``  slice the map at the cruise height and grow every obstacle by the drone's footprint
               (a rectangle at a fixed heading, plus the safety margin), so the drone becomes a point.
2. ``route``   shortest grid route (A*) from the launch mark through every gate in order to the pad,
               then straightened by line-of-sight shortcuts.
3. ``smooth``  trades path length against curvature (CHOMP-style, gates stay on their segments,
               obstacles are a hard constraint), for several weights.
4. ``speed``   time-optimal speed along each candidate for a point mass with a total acceleration
               limit (forward/backward pass); the fastest candidate wins.
5. ``render``  top-down picture of the map, the grown obstacles, the gates and the speed-coloured path.
"""
