# Methods

One folder per localisation approach. Each folder is self-contained apart from the shared pieces at
the repo root (`camera/` for the DepthAI basics, `datasets/` for recordings and calibration,
`cuvslam_oak/` for the dataset reader / writer and camera pipeline, `tests/`).

| Folder | What | Status |
|---|---|---|
| [cuvslam-vio/](cuvslam-vio/) | NVIDIA cuVSLAM stereo / stereo-inertial odometry with a starvation guard and coasting; live view through a WSL2 bridge | working, measured |
| [cuvslam-vslam/](cuvslam-vslam/) | cuVSLAM's loop-closure / map layer | tried, no benefit on our recordings |
| [lidar-vo-fusion/](lidar-vo-fusion/) | the Guard on cuVSLAM, the MS200K lidar localiser on the course map, and the drift correction applied to the flight commands sent to the kit (PX4 keeps flying on raw cuVSLAM); per-run `lidar_policy` (shared code in `pose_fusion/`, matcher in `course/`) | written and packaged (`dronehacks_lidar`, with the route follower); unit-tested on synthetic data incl. a closed loop; ROS tests written, not yet run; not flown |
| [obstacle-detection/](obstacle-detection/) | obstacles from the OAK-D S2's stereo depth (ported from `course-planner-lidar`) | see the script's docstring |
| [_template/](_template/) | copy this to start a new method | |

## Adding a method

1. Copy `_template/` to `methods/<your-method>/` and fill in its README: what it is, how to install it,
   how to run it on a recording and live, what it measured, what is unverified.
2. Read recordings with `cuvslam_oak.dataset.EurocReader` (or any EuRoC loader) and write trajectories
   as TUM files (`cuvslam_oak.tracking.write_tum`) so `evo` and the existing comparison scripts work on
   them.
3. Keep method-specific code, scripts, docs and tools inside your folder; put anything reusable by
   everyone (a new recorder feature, a calibration tool) in `datasets/` or `cuvslam_oak/` with a test.
4. Evaluate with the same protocol: a walk that returns to a floor mark, end-point offset / path length,
   end-of-walk heading error, repeated runs (`methods/cuvslam-vio/docs/cuvslam.md` section 4).
