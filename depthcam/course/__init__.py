"""Course tooling: STEP -> course map, course map -> Gazebo world, course map + rules -> trajectory.

Modules
-------
``step_reader``  minimal ISO-10303-21 reader: entities, B-rep solids as sampled face loops, colours,
                 assembly placements (numpy only).
``course_map``   classifies the solids (walls with openings, tube, rocks, nets, floor markings), writes
                 and reads the map JSON, and slices the map at a flight altitude into 2D obstacles.
``geometry``     2D convex-polygon helpers shared by the map, the world generator and the planner.
``worldgen``     writes a Gazebo (Harmonic, SDF 1.9) model + world for PX4 SITL.
``planner``      route through ordered gates, path smoothing, time-optimal speed profile, rendering.

The numbered scripts next to this file are the command-line entry points (12, 13, 14).
"""
