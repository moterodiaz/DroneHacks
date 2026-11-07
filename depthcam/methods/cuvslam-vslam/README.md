# cuVSLAM SLAM layer: loop closure and maps (tried, not useful here)

cuVSLAM can run a SLAM layer (loop closure, pose-graph optimisation, map save / relocalisation) next
to its odometry, in the same `Tracker`. It is wired into the VIO code: `08 --slam`, `09 --slam`,
`--slam-map-size`, `--slam-save DIR` (see [../cuvslam-vio/README.md](../cuvslam-vio/README.md) and
[../cuvslam-vio/docs/cuvslam.md](../cuvslam-vio/docs/cuvslam.md)); the tracking loop keeps the SLAM pose
of every frame, the re-optimised trajectory (`trajectory_<mode>_slam.tum`), loop-closure counts and the
saved map; the live view draws the SLAM trajectory in magenta.

## What it did on our recordings (2026-09-30)

`08 --slam` (and `09 --slam`) runs the engine's loop-closure / pose-graph layer next to the odometry (`Tracker(rig, odom_cfg, SlamConfig)`; synchronous and deterministic offline, its own thread live; `--slam-map-size`, `--slam-save DIR`). The loop keeps the SLAM pose of every accepted frame (`slam_rows`, anchored like the odometry rows), the re-optimised trajectory from `get_all_slam_poses()` (`trajectory_<mode>_slam.tum`), loop-closure and pose-graph-optimisation counts (rising edges of `get_slam_metrics()`), and saves the map (`save_map`, an LMDB folder: 47 MB for the 57 s walk). The live view draws the SLAM trajectory in magenta. Measured on the four walks and the still recording, one run each: the SLAM pose was **identical to the odometry pose in every frame of every run** (largest difference 0.000 m), the re-optimised trajectory moved the end point by at most 6 mm, and loop closures fired only on `001403` (1 in the guarded run; 2 per pass when the whole recording was fed twice in a row, at the same spot each time, so not a recognition of the revisited route). Two causes are established: the guard's engine restart destroys the map (`slam.map_resets` counts it; every walk but one restarts at least once), and loop-closure search is gated by the current position estimate, so it cannot recognise a place the odometry has drifted away from. A third is a documentation fact: reading the pose graph and the map landmarks needs the C++ `EnableReadingData(layer)` call, which the Python binding does not expose, so `pose_graph_nodes` / the landmark layers read 0 even when the map exists. Conclusion for flight: do not count on loop closure from this engine in these rooms; the odometry with the guard and the coasting is what flies. `localize_in_map` (start-pose anchoring in a saved map) is wired in the API but was not tried.

## How to reproduce

```
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --slam --summary
python methods/cuvslam-vio/08_cuvslam_track.py ~/datasets/euroc_<stamp> --mode stereo --slam --slam-save ~/datasets/_maps/<name>
python methods/cuvslam-vio/09_cuvslam_live_view.py --slam
```

The probes in [probes/](probes/) are the WSL2 experiments behind the numbers: `slam_probe.py` (one pass,
all the SLAM getters), `slam_probe_two_pass.py` (NVIDIA's own calling pattern, the recording fed
twice in a row), `slam_probe_two_pass_guarded.py` (the same through our guarded loop).

## What would be needed to make it useful

- Keep the engine alive through blank-wall stretches instead of restarting it (every restart destroys
  the map), which means a room textured enough that the guard never fires.
- `localize_in_map` (anchoring a start pose in a saved map) is in the API and untried.
- Reading the pose graph / landmarks needs the C++ `EnableReadingData` call, not exposed in Python.
