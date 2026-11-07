# Obstacle-avoidance mapping and path planning for a single-forward-stereo-camera quadcopter on Jetson Orin Nano (as of 2026-10-25)

Scope: algorithm families and concrete open-source implementations for indoor obstacle-course flight with one forward OAK-D S2 (passive stereo), cuVSLAM VIO, a Jetson Orin Nano 8 GB shared with VIO, and PX4 offboard control. Depth-sensor quality, VIO internals and the PX4 offboard API are covered elsewhere. They appear here only where they change which planner to pick.

Source-quality note: several facts below came from search-result snippets rather than full pages, or from third-party mirrors. These are marked "(snippet)" or "(secondary)". Everything else came from the primary page (repo README, official docs, arXiv full text or abstract).

---

## 1. Reactive / map-less methods (VFH family, potential fields, depth-image gap finding / motion primitives, NanoMap, PX4 Collision Prevention)

### Takeaway
Reactive methods are the cheapest and fastest to integrate. They tolerate VIO drift because they need little or no world-frame memory, but they fall into local minima and dead ends, and with one forward camera they know nothing about what has left the field of view. The best documented modern example is CMU's 2024 forward-arc motion-primitive planner on NanoMap: 12 Hz replanning on an Orin AGX, up to 6 m/s, with no global planner. It still hit a dead end that needed backtracking. PX4-Avoidance (3DVFH*) has been archived since Aug 1, 2024. PX4's built-in Collision Prevention works only in Position mode. It cannot be the offboard planner, but it is a useful model for a safety layer.

### Cited Findings
**PX4-Avoidance (3DVFH* local planner): deprecated**
- The repo was "archived by the owner on Aug 1, 2024. It is now read-only," and the README says "This project is currently not maintained." It targets Ubuntu 20.04 + ROS Noetic + Gazebo 11, needs MAVROS >= 0.29.0, and recommends RealSense firmware 5.9.13.0 — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)
- local_planner is "VFH+*-based", uses "a vector field histogram and some history", and is the most developed of the three planners. It needs less compute but does not find optimal paths. global_planner runs on an OctoMap occupancy grid, needs accurate global position and heading, and its Gazebo section is marked "advanced, not flight tested" — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)
- Input is `sensor_msgs::PointCloud2` from any depth camera. The officially supported sensor is the RealSense D435 (D415, R200 and Structure Core also tested). The local planner should get point clouds at about 10-20 Hz. Tested compute: Intel NUC, Jetson TX2 and Intel Atom x7-Z8750 for the local planner, Odroid for the global planner. No minimum spec is given — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)
- 3DVFH* was introduced by T. Baumann (ETH master's thesis). It combines 2D VFH with A*-style search adapted to 3D. It builds a polar histogram, picks a direction, and checks it with A* over a memory histogram of points from earlier timesteps. Cost weights include a "pitch cost" (vertical evasion) and an "obstacle cost" — [Springer CEAS paper (snippet)](https://link.springer.com/article/10.1007/s13272-023-00691-w); [Thomessen 2023 (snippet)](https://opus.bibliothek.fh-aachen.de/opus4/files/10924/2023_Thomessen_10924.pdf)
- Being a local planner, 3DVFH* can plan into dead ends. A bio-inspired climb-preferring variant cut the dead-end probability from 70.7 % to 18.6 % in city-like simulated worlds (Matlab/Simulink) — [Thomessen et al. 2023 (snippet)](https://opus.bibliothek.fh-aachen.de/opus4/files/10924/2023_Thomessen_10924.pdf)
- MathWorks ships `controllerVFH3D` (3DVFH+) in the UAV Toolbox, which is useful for offline prototyping — [MathWorks controllerVFH3D](https://www.mathworks.com/help/uav/ref/controllervfh3d-system-object.html)

**PX4 Collision Prevention (built into the firmware)**
- It works for multicopters only in acceleration-based Position mode (`MPC_POS_MODE` = Acceleration based). The docs do not mention Offboard mode — [PX4 docs: Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- Parameters: `CP_DIST` (minimum distance, measured to the sensor and not to the prop tips; negative disables); `CP_DELAY` (sensor + tracking delay; up to about 0.2 s for external vision, 0.1-0.5 s typical tracking); `CP_GUIDE_ANG` (allowed deviation for "sliding" around obstacles, 30 deg suggested, 0 disables guidance); `CP_GO_NO_DATA` (default 0, which blocks motion in directions with no sensor coverage) — [PX4 docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- A companion computer feeds it `OBSTACLE_DISTANCE` MAVLink messages. The docs say 72 sectors (5 deg) in one place and 36 in another. Tests used 10 Hz at 4 m/s. If no data arrives for more than 0.5 s, horizontal velocity is set to zero, and after 5 s the vehicle switches to Hold. Companion-computer integration is labelled untested because the original companion project is archived — [PX4 docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)

**Depth-image planners with no fused map (gap finding / motion primitives)**
- RAPPIDS (Bucki et al., 2020) evaluates "thousands of candidate trajectories in milliseconds" against a single depth image, using rectangular-pyramid free-space partitioning. Flights ran at 30 Hz on an ODROID-XU4. Plain CMake with no ROS, GPL-3.0, 4 commits — [arXiv 2003.01245](https://arxiv.org/abs/2003.01245); [nlbucki/RAPPIDS](https://github.com/nlbucki/RAPPIDS)
- CMU "Rapid Quadrotor Navigation in Diverse Environments using an Onboard Depth Camera" (arXiv 2411.04326, Nov 2024) is a reactive planner that scores forward-arc motion primitives. Forward speed is fixed, and vertical velocity and yaw rate are discretized. Cost is the distance from the primitive endpoint to the goal, and a stopping primitive is always scheduled as a fallback — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
  - Hardware: Orin AGX 32 GB, RealSense D455 at 424x240 / 30 Hz, VINS mono VIO, 12 Hz replanning, 100 Hz outer loop, PX4 on a Pixracer. Max 6 m/s, 571 m of collision-free successful runs. Simulation: 82 % success over 1,350 trials, beating the RAPPIDS and NanoMap/Florence baselines — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
  - Failure modes reported: in the forest, 3 collisions out of 9 trials from thin branches the depth sensor missed. In the cave, 1 collision from VINS drift and 1 timeout from a dead end that needed backtracking ("no global planner is used") — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
  - No code link for Forward-Arc. It links only the baselines RAPPIDS and nanomap_ros — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
- NanoMap (Florence, Carter, Ware, Tedrake; ICRA 2018) does not fuse depth into a global map. It stores a history of depth frames plus noisy relative transforms and answers a proximity query from the minimum-uncertainty view. Poses can be updated cheaply, for example after loop closure. Demonstrated up to 10 m/s. With 5 % drift and no uncertainty modelling the drone crashed more than once every four flights, and with uncertainty modelling the crash rate fell to 2 %. Open-source (`nanomap_ros`) — [arXiv 1802.09076](https://arxiv.org/pdf/1802.09076); [IEEE Spectrum](https://spectrum.ieee.org/modeling-uncertainty-helps-mits-drone-zip-around-obstacles)
  - In the CMU forward-arc system the hardware NanoMap history was 1 s of frames. Queries are transformed back through earlier sensor frames until one contains the point — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)

### Inferences
- **Compute:** a VFH-style polar histogram or a primitive-scoring loop over a downsampled depth image (for example 80x50) costs on the order of a millisecond on one CPU core. RAPPIDS ran at 30 Hz on an ODROID-XU4, far weaker than the Orin Nano CPU. This is the lightest option alongside cuVSLAM.
- **Artificial potential fields:** no fetched source specific to this platform. The textbook failure modes are local minima between two obstacles, oscillation in narrow passages, and goal-unreachable-near-obstacle. A field computed only from the current depth image has no memory of obstacles that left the FOV, so a pure APF is the weakest choice for a cluttered course.
- **PX4 Collision Prevention:** because it is documented for Position mode only, it cannot protect an offboard-controlled flight. Its design is still worth copying in the companion code: stop distance plus delay compensation, sliding within a guide angle, blocking motion into no-data directions, and zero velocity on 0.5 s data loss. A manual pilot fallback in Position mode would get it for free, and it is worth enabling for safety-pilot takeovers if the OBSTACLE_DISTANCE stream is published.
- **Main reactive failure in a netted course:** dead ends and U-shaped obstacle arrangements. The CMU cave dead end, the 70.7 % dead-end rate of 3DVFH* in city worlds, and FlightBench's maze results (section 2) all point the same way. Mitigation: add a coarse global route (known waypoints or a 2D grid A*) and use the reactive layer only for local deviation.

### Gaps
- No Jetson Orin Nano timing was found for 3DVFH*, RAPPIDS or NanoMap. Their published hardware is older (TX2, ODROID-XU4, NUC).
- The original Baumann thesis PDF was not retrieved. The description is from secondary papers.
- No source states whether PX4 Collision Prevention has any effect in Offboard mode. The docs are silent, and the inference above is "assume no."

---

## 2. Map-based methods: voxel/ESDF mapping (nvblox, Voxblox, OctoMap, UFOMap) and planners (EGO-Planner/EGO-Swarm, Fast-Planner, FASTER, MINCO/GCOPTER, SUPER), with Jetson rates

### Takeaway
nvblox is the only mapper with published Orin Nano timings: TSDF about 1.2-2.1 ms, ESDF about 5.1-6.2 ms, meshing 4.2-13 ms per update at 5 cm voxels. On JetPack 6.x the supported path is Isaac ROS 3.2 + ROS 2 Humble. Users report setup failures and out-of-memory problems when running it alongside VSLAM on the Orin Nano. EGO-Planner is the most practical depth-camera trajectory planner (planning about 1 ms, about 25 ms total per cycle on an Orin NX in FlightBench). It is ROS 1, its ROS 2 port is simulation-only, and its success collapses in maze-like occlusion scenarios. Fast-Planner, FASTER, GCOPTER and Voxblox are ROS 1-era code with no recent maintenance, and SUPER/Bubble-style planners are LiDAR-centric.

### Cited Findings
**nvblox / Isaac ROS nvblox (NVIDIA, GPU TSDF/ESDF)**
- The current Isaac ROS nvblox docs show Isaac ROS 5.0 (Updates table: 5.0.0 on 2026-09-21). Supported: Jetson Thor and Jetson Orin on JetPack 7.2, x86 with Ampere+ GPU and 8 GB+ RAM on Ubuntu 24.04 / CUDA 13.2+, and ROS 2 "Lyrical", with Jazzy support listed from 2025-10-24 — [Isaac ROS nvblox docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- Inputs: depth image + color image + pose from VSLAM or another estimator, from depth cameras and/or 3D LiDAR. Outputs: a 2D costmap for Nav2 through a costmap plugin, plus an optional mesh visualization. The modes are static, people (PeopleSemSegNet UNet mask) and dynamic reconstruction — [Isaac ROS nvblox docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- Per-component timing at 0.05 m voxels, AGX Orin vs Orin Nano: Replica TSDF 0.8 / 2.1 ms, Color 1.1 / 3.6 ms, Meshing 2.3 / 13 ms, ESDF 1.7 / 6.2 ms. Redwood TSDF 0.5 / 1.2 ms, Color 0.8 / 2.6 ms, Meshing 0.9 / 4.2 ms, ESDF 1.5 / 5.1 ms. "Dynamics not supported on Jetson Nano," and the Orin Nano dynamics column is N/A — [Isaac ROS nvblox docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- Version and platform matrix from NVIDIA forum staff: as of July 8, 2026, "Isaac ROS 3.2 with JetPack 6.1/6.2 and ROS 2 Humble" was the supported path for Jetson Orin. On Aug 21, 2026, Isaac ROS 4.6.0 "adds support for Jetson Orin with JetPack 7.2" on ROS 2 Jazzy — [NVIDIA forum: Roadmap for Isaac ROS Jazzy on Orin](https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982)
- Orin Nano field reports:
  - A user on JetPack 6.2 + Orin Nano 8 GB + RealSense D455 failed for several days to run the nvblox stack (release-3.2, v3.2-1, v3.2-15; Docker and native). Staff replied only "do the quickstart with a rosbag first," and there is no resolution (Feb 23-24, 2026) — [NVIDIA forum](https://forums.developer.nvidia.com/t/jetson-orin-nano-8gb-com-problemas-para-executar-pilha-isaac-ros-nvblox/361504)
  - A user reports being unable to run VSLAM and nvblox together on an Orin Nano because it "simply runs out of memory" (ZED X camera) — [NVIDIA forum thread surfaced by search (snippet; thread not opened)](https://forums.developer.nvidia.com/t/for-mapping-and-navigation-what-approach-should-i-take/282915)
  - A forum reply says nvblox runs on Jetson Orin but "its performance may decrease if there are multiple nodes running or if your GPU usage is high" (Feb 19, 2025; poster's role unconfirmed) — [NVIDIA forum](https://forums.developer.nvidia.com/t/nvblox-is-fully-supported-on-jetson-orin-nano-super/323631)
- nvblox paper (Millane, Oleynikova et al., ICRA 2024): up to 177x faster TSDF and up to 31x faster ESDF than CPU methods (v2 figures; v1 abstract said 7-63x for ESDF), real-time "even on embedded GPUs" — [arXiv 2311.00626](https://arxiv.org/abs/2311.00626v2)
- The core nvblox library can be driven from Python, C++ or ROS 2, and the repo contains an `nvblox_torch` folder — [nvidia-isaac/nvblox](https://github.com/nvidia-isaac/nvblox). The first nvblox_torch release does not support mapping with moving elements and is "not as highly optimized" as the core library. Exceeding GPU memory crashes it (snippet) — [search summary of nvblox docs](https://docsearch.algolia.com/mcp/docs/repo/nvidia-isaac/nvblox). cuRobo's docs (older NVlabs nvblox_torch) say native Jetson installation is not supported and recommend Docker — [cuRobo install docs](https://curobo.org/get_started/1_install_instructions.html)

**Other volumetric maps**
- Voxblox (ETH ASL): no archive notice, 64 open issues and 13 open PRs, ROS 1 (`voxblox_ros`, `.rosinstall`), and no successor named on the page — [ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)
- UFOMap: a probabilistic 3D map with explicit unknown/free/occupied voxels and a `ufomap_ros` wrapper. The ROS 1/2 target is not stated, there are no releases, and the claim is "real-time (2 Hz) at 2 mm voxel size" colored — [UnknownFreeOccupied/ufomap](https://github.com/UnknownFreeOccupied/ufomap)
- OctoMap is used by PX4-Avoidance's global_planner (see section 1), which is "more computationally expensive" and needs accurate global pose — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)

**Planners**
- EGO-Planner (ZJU FAST Lab) is an "ESDF-free gradient-based local planner." Total planning time is "around 1ms." Its mapping takes depth images or point clouds. It is ROS 1 (Ubuntu 16.04/18.04/20.04, catkin), GPL-3.0 with 2.7k stars, and depends on Armadillo. The README recommends EGO-Swarm as "more robust and safe" — [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)
- EGO paper: EGO 0.81 ms planning with no ESDF, vs Fast-Planner 3.29 ms + 4.01 ms ESDF and EWOK 1.39 + 6.43 ms. Planning horizon about 7 m with about 0.3 m control-point spacing. Real flights used a RealSense D435 with the emitter strobed every other frame and reached 3.56 m/s indoors. The paper notes local minima because "the camera has no vision of the back of the obstacle," says it targets static environments, and names limited FOV as a challenge — [arXiv 2008.08835](https://arxiv.org/html/2008.08835v2)
- EGO-Swarm `ros2_version` branch: Humble (CycloneDDS required; "Using ROS2's default FastDDS causes significant lag"). It contains only simulation launch files and makes no mention of real-drone or Jetson deployment — [ego-planner-swarm ros2_version](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)
- FlightBench (2024) benchmarked ego-vision methods in simulation with a depth camera model:
  - Onboard **Jetson Orin NX** total time per cycle: EGO 24.95 ms (planning 0.91 ms), Fast-Planner 39.18 ms (mapping 36.85 ms is the bottleneck), TGK 37.65 ms, Agile (learned) 27.46 ms, LPA (learned) 12.29 ms, LMT (learned) 13.21 ms — [FlightBench arXiv 2406.05687](https://arxiv.org/html/2406.05687v2)
  - Success at the hardest test per scenario: Forest EGO 1.00 / Fast 0.90. Maze EGO 0.20 / Fast 0.60 / TGK 0.50 / Agile 0.50. Multi-waypoint EGO 0.50 / Fast 0.80. Ego-vision methods in Maze and multi-waypoint are "generally below 0.6." Failure modes: walls occluding later walls, local optima between walls, and sharp turns near columns — [FlightBench](https://arxiv.org/html/2406.05687v2)
  - The optimization-based planners were capped at 3 m/s (about 2.3-2.5 m/s average) — [FlightBench](https://arxiv.org/html/2406.05687v2)
- Fast-Planner (HKUST): ROS Melodic/Noetic (Ubuntu 18.04/20.04), NLopt 2.7.1 + Armadillo. `plan_env` takes depth (640x480 default, `skip_pixel=2`) or point cloud + odometry pairs, raycasts into a probabilistic volumetric map and builds an ESDF. It has kinodynamic search + B-spline optimization and topological path-guided optimization, with perception-aware planning listed as "to appear." Newest dated entry: Mar 13, 2021 — [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
- RAPTOR (Fast-Planner family) adds path-guided optimization over topological paths, a perception-aware strategy so the drone observes unknown obstacles before reaching them, and yaw planning to scan space relevant to safety. Code was intended for the Fast-Planner repo — [arXiv 2007.03465](https://arxiv.org/pdf/2007.03465)
- FASTER (MIT ACL): takes a depth image + State pose and uses JPS3D and DecompROS third-party code. It **requires Gurobi** (tested 8.1/9.0/9.1) and ROS Kinetic/Melodic, reports no compute times, and was published in IEEE T-RO — [mit-acl/faster](https://github.com/mit-acl/faster)
- GCOPTER/MINCO (ZJU): a trajectory optimizer with corridor generation (FIRI). It has no mapping module, catkin/ROS 1 style, newest dated update July 20, 2022, and the core paper is IEEE T-RO 2022 — [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
- SUPER (HKU MaRS, Science Robotics, Jan 29, 2025) is **LiDAR-based**, above 20 m/s. Each cycle it generates two trajectories, one in known-free space for safety and one through known + unknown space for speed. The reported failure rate is 35.9x lower than baselines with half the planning time. A preview planning module was released for ROS 1 and ROS 2 — [HKUST research portal](https://researchportal.hkust.edu.hk/en/publications/safety-assured-high-speed-navigation-for-mavs/); [HKU MechE news](https://mech.hku.hk/post/safety-assured-high-speed-navigation-for-mavs); [code mirror](https://gitee.com/max0617/SUPER)
- Fast-Drone-250 (ZJU reference build) combines VINS-Fusion, EGO-Planner and PX4Ctrl with a RealSense D435i, an Intel NUC (or equivalent) and a PX4 flight controller — [DeepWiki summary (secondary)](https://deepwiki.com/ZJU-FAST-Lab/Fast-Drone-250)
- Closest real Orin-class hardware reports found: EGO-Planner-v2 on a Jetson Orin NX (forest-inventory drone), and a learned NEO-Planner on an Orin NX + D435 at 1.0 m/s. Both come from search snippets of the papers' text. No EGO-Planner + Orin Nano + depth camera flight paper was found (snippet) — [arXiv 2501.12073](https://arxiv.org/pdf/2501.12073)

### Inferences
- **Compute on Orin Nano:** nvblox TSDF + ESDF at 5 cm is about 6-8 ms of GPU time per integrated frame (skip color and meshing). At 10-15 Hz integration that is roughly 10 % GPU duty, which is plausible alongside cuVSLAM. The binding constraint reported by users is 8 GB unified memory and setup friction, not GPU milliseconds.
- **CPU planners:** EGO's planning is about 1 ms, but its CPU grid-map/raycast stage dominates its 25 ms Orin NX cycle. The Orin Nano CPU is weaker than the Orin NX CPU (fewer and lower-clocked A78AE cores; spec not re-verified here), so expect roughly 30-50 ms per cycle at default depth resolution. That still allows 10-20 Hz replanning, and downsampling depth helps.
- **ROS friction is the biggest integration cost for this team.** EGO, Fast-Planner, FASTER, GCOPTER, Voxblox and PX4-Avoidance are ROS 1. JetPack 6 is Ubuntu 22.04, where ROS 1 Noetic is not officially packaged (and Noetic went end-of-life in May 2025; both facts are general knowledge, not re-verified in this pass), so they would need a Noetic container. The only ROS 2 port of EGO is sim-only with DDS lag issues. The team's current pipeline is Python (PyCuVSLAM), with no ROS.
- **nvblox on JetPack 6** means Isaac ROS 3.2 / Humble, not the current 5.0 docs. Reflashing to JetPack 7.2 for Isaac ROS 4.6+ is a multi-day risk on its own. If GPU mapping is wanted, the nvblox core library with a C++/Python binding, fed by PyCuVSLAM poses, avoids ROS. Its Jetson build path for Python is unverified.
- **Fit for "multiple obstacle sizes in a netted course":** FlightBench's maze and multi-waypoint results (EGO 0.20 / 0.50 success) are the closest analogue to an indoor course with occluding obstacles and turns. Map-based local planners alone are not enough. A global route (waypoints and/or grid search over the remembered map) is needed to escape occlusion-induced local minima.
- **Sampling-based (RRT\*) and MPC-with-obstacle-constraints:** no Orin-class flight benchmarks were found in this pass. For a 2D or 2.5D grid of about 100x100 cells, A*/Dijkstra is cheaper and deterministic, so RRT* brings little benefit at this scale.

### Gaps
- No official Orin Nano end-to-end nvblox rate (Hz) or memory footprint was found. The table gives per-component milliseconds only.
- No Orin Nano numbers were found for EGO-Planner, Fast-Planner or FASTER. Orin NX numbers (FlightBench) are the closest.
- "Bubble Planner" (HKU MaRS) was not verified in this pass. Searches returned only SUPER. My understanding is that it is LiDAR-oriented, but this is unconfirmed.
- OctoMap's and UFOMap's current ROS 2 maintenance status was not verified.
- No exact nvblox_torch install instructions for JetPack 6 aarch64 were found. The docs URL tried returned 404.

---

## 3. Simple 2.5D approach: fixed altitude + 2D occupancy grid / costmap + A*; handling obstacles of different heights

### Takeaway
The most widely supported 2.5D pattern is to slice 3D depth into a height band around the flight altitude, collapse it into a 2D occupancy grid or costmap, and run A*/Dijkstra or a Nav2 planner. nvblox does exactly this with configurable min/max obstacle heights for its Nav2 costmap. For a mostly planar course, a hand-rolled numpy grid replicates it in a day. Obstacles of different heights are handled by choosing the band, which decides what counts as "go around." Flying over something requires seeing its top and is safer as a deliberate per-obstacle decision than as an emergent planner behaviour.

### Cited Findings
- nvblox's 2D ESDF slice uses `min_height` (default 0.0) and `max_height` (default 1.0) to bound which obstacles count toward the slice, plus `slice_height` (default 1.0) for the output distance slice. A forum reply uses the names `esdf_2d_min_height` / `esdf_2d_max_height` with the same meaning. Newer configs use `esdf_mode: "2d"` / `"3d"` (snippet; parameter names vary by version) — [isaac_ros_nvblox fork README (snippet)](https://github.com/jahaniam/isaac_ros_nvblox); [NVIDIA forum: nvblox layer in Nav2 costmap (snippet)](https://forums.developer.nvidia.com/t/issues-with-nvblox-layer-in-nav2-costmap/308347)
- The nvblox Nav2 layer plugin subscribes to `/nvblox_node/static_map_slice`, and an example config sets `convert_to_binary_costmap: True` (snippet) — [NVIDIA forum (snippet)](https://forums.developer.nvidia.com/t/issues-with-nvblox-layer-in-nav2-costmap/308347)
- Isaac ROS nvblox's primary documented output is "a 2D costmap for Nav2" through a costmap plugin — [Isaac ROS nvblox docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)
- Vertical vs horizontal evasion in 3DVFH* is controlled by a "pitch cost" weight. A climb-preferring variant cut dead ends from 70.7 % to 18.6 % in simulation, so allowing vertical escape can matter a lot in 3D clutter — [Thomessen 2023 (snippet)](https://opus.bibliothek.fh-aachen.de/opus4/files/10924/2023_Thomessen_10924.pdf)
- The CMU forward-arc planner varies vertical velocity as well as yaw rate in its primitive set. Even a reactive planner can include a small vertical degree of freedom — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)

### Inferences
- **Recipe for a hand-rolled 2.5D grid** (Python/numpy, about 1 day):
  1. Downsample the OAK-D depth (for example 1280x800 to 160x100) and back-project to points in the camera frame, then the body frame.
  2. Transform to the odom frame with the cuVSLAM pose.
  3. Keep points with `z` in [z_fly - h_below, z_fly + h_above] and drop the floor and ceiling/net-top.
  4. Mark hits and ray-cast free cells from the camera to each hit, or to max range for no-return pixels, in a drone-centred rolling grid (for example 12x12 m at 0.1 m). Inflate by drone radius plus margin.
  5. Plan A* from the current cell to the next waypoint, then take the first few path cells and convert them to a velocity + yaw setpoint.
- **Height-band choice for multi-size obstacles:**
  - (a) Obstacles shorter than (z_fly - h_below - clearance) are ignored, which effectively means flying over them.
  - (b) Everything else is "go around."
  - (c) Pick z_fly high enough to clear the short-obstacle class if the course ceiling or net allows, otherwise low.
  - A forward camera loses sight of a low obstacle's top as it passes beneath the lower edge of the vertical FOV. Fly-over needs that obstacle remembered in the map before it leaves view, which is a rolling-map argument.
  - Overhangs (obstacles with a gap below) are not representable in a single 2D slice. Treat any return inside the band as blocked.
- **Nav2 vs hand-rolled:** Nav2 is a ground-robot stack (planner, controller, behaviour trees, costmaps) and assumes a TF tree and odometry topics. It would need a PX4 bridge and a holonomic controller. With days to integrate and no ROS currently, a numpy/CuPy grid plus a 100-line A* has fewer moving parts. Nav2 is worth it only if the team already adopts ROS 2 + nvblox.
- **When to go 3D instead:** if the course has obstacles meant to be flown over or under at different heights (hoops, low bars, stacked boxes), a 2.5D slice at one altitude wastes options. A small 3D voxel grid (nvblox or a numpy 3D array at 0.1 m) with 3D A* over a few altitude layers is a moderate step up.

### Gaps
- No published drone team write-up was found using Nav2 + nvblox 2D slice with PX4 offboard. This is a pattern inference.
- No source was found on how competition-style indoor drone teams specifically choose fly-over vs fly-around.

---

## 4. Learned policies (end-to-end depth-to-action) and whether any are deployable in days without training infrastructure

### Takeaway
Two pretrained options are realistic to try: UZH agile_autonomy (ROS Noetic + TF 2.4 checkpoint for 1-10 m/s) and TJU YOPO (pretrained `epoch50.pth`, about 1 h data collection plus 2-3 h retraining if needed). Neither has been validated on an OAK-D S2 or Orin Nano, and both are ROS 1. The SJTU differentiable-physics policy is very light (16x12 depth input, ran on a $21 SBC) and odometry-free, but no pretrained weights were found. FlightBench shows learned methods are fast but unstable under VO and occlusion stress and need latency randomization. For a days-long build, learned policies are a stretch goal, not a primary plan.

### Cited Findings
- **UZH "Learning High-Speed Flight in the Wild"** (Loquercio et al., Science Robotics 2021): trained only in simulation via privileged learning. Hardware: Jetson TX2 (GPU inference), RealSense T265 VIO at 200 Hz, D435i stereo depth at VGA/30 Hz (~90 deg HFOV, tilted 30 deg for at least 7 m/s), 890 g platform on Betaflight — [arXiv 2110.05113 (snippet)](https://arxiv.org/pdf/2110.05113)
- The agile_autonomy repo was tested on Ubuntu 20.04 + ROS Noetic, Python 3.7 + tensorflow-gpu 2.4, with Flightmare and rpg_mpl_ros. Its checkpoint "will work for all speeds in the range [1,10] m/s." The network must run at least 15 Hz or "you won't be able to fly successfully." DAgger data collection takes "several days," fine-tuning needs 1/10 of the dataset, and only simulation testing is described. 63 open issues — [uzh-rpg/agile_autonomy](https://github.com/uzh-rpg/agile_autonomy)
- **SJTU "Back to Newton's Laws"** (Zhang et al., arXiv 2407.10648):
  - Training: the policy is trained by backpropagating through a point-mass model plus depth renderer. 50,000 iterations at batch 64 on a CUDA simulator implemented as a PyTorch extension — [arXiv 2407.10648](https://arxiv.org/html/2407.10648v2)
  - Input/output: depth is inverted and max-pooled to **16x12**, with a D435i at 15 Hz. A CNN + GRU outputs desired thrust acceleration plus a velocity estimate, and attitude is an input — [arXiv 2407.10648](https://arxiv.org/html/2407.10648v2)
  - Hardware: runs on a Mango Pi (Cortex-A53, 11 g, "$21") — [arXiv 2407.10648](https://arxiv.org/html/2407.10648v2)
  - Odometry: the odometry-free variant uses only attitude + depth — [arXiv 2407.10648](https://arxiv.org/html/2407.10648v2)
  - Results: up to 20 m/s in a forest and 90 % vs 60 % success over the previous state of the art in simulation — [arXiv 2407.10648](https://arxiv.org/html/2407.10648v2)
  - The repo has training/eval code (PyTorch 2.2.2, CUDA 11.8) and `eval.py --resume <checkpoint>`. Releases are empty and no pretrained weights or real-robot deployment instructions are documented — [HenryHuYu/DiffPhysDrone](https://github.com/HenryHuYu/DiffPhysDrone)
- **YOPO** (TJU Aerial Robotics, "You Only Plan Once," IEEE 2024): a one-stage learned planner that folds mapping, front-end search and back-end optimization into one network.
  - Code and weights: pretrained `saved/YOPO_1/epoch50.pth`, data collection about 1 h and training 2-3 h, Python 3.8 conda + catkin — [gittrend summary of TJU-Aerial-Robotics/YOPO (secondary)](https://gittrend.io/repo/TJU-Aerial-Robotics/YOPO)
  - Transfer: a ground-truth-depth-trained policy transferred zero-shot to stereo cameras — [gittrend summary (secondary)](https://gittrend.io/repo/TJU-Aerial-Robotics/YOPO)
  - Inference: about 20 ms on an RK3566 with INT8 — [gittrend summary (secondary)](https://gittrend.io/repo/TJU-Aerial-Robotics/YOPO)
  - Repo status: last push Dec 23, 2025, about 1.0k stars — [gittrend summary (secondary)](https://gittrend.io/repo/TJU-Aerial-Robotics/YOPO)
- **FlightBench** (2024): learned methods were much faster in Forest (LMT 11.84 m/s, LPA 8.96 m/s vs about 2.3-2.5 m/s for capped optimization planners). They "struggle with instability in scenarios with high challenges related to VO and AOL" and fly less smoothly. Training without latency randomization dropped LMT success from 0.9 to 0.3 and LPA from 0.5 to 0.0 on a hard multi-waypoint test, and "Integrating latency randomization ... is essential" — [FlightBench](https://arxiv.org/html/2406.05687v2)
- **2026 works:**
  - "Anticipatory Risk-Guided RL for Safe Flight Through Dynamic Clutter" (arXiv 2607.23565, IROS 2026): depth sequences, zero-shot sim-to-real. The abstract mentions no code release. A search snippet says Orin NX + D435i with about 5 ms TensorRT inference and an indoor box course — [arXiv 2607.23565](https://arxiv.org/abs/2607.23565)
  - Curriculum-RL racing with random obstacles (arXiv 2602.24030): depth policy on a Raspberry Pi + D435i at 30 Hz, zero-shot (snippet) — [arXiv 2602.24030](https://arxiv.org/pdf/2602.24030)

### Inferences
- **Deployable in days?** Only as an experiment. The pretrained checkpoints were trained on simulated or RealSense-like depth (often ground-truth depth). The OAK-D S2's passive stereo has different hole and noise patterns, especially on low-texture obstacles, and a 7-inch quad has different dynamics. Each paper stresses domain randomization or latency randomization for transfer, which this team cannot redo in 48 h.
- **Odometry-free policies (DiffPhys) are attractive** because they sidestep cuVSLAM glitches. Course navigation still needs a goal direction from somewhere, so VIO is still required for waypointing.
- **Integration effort:** agile_autonomy and YOPO both carry ROS 1 + simulator dependencies. Running only the inference network from a Python script (TF 2.4 or PyTorch) on the Orin Nano is possible in principle but would require reimplementing their input preprocessing and trajectory/controller interfaces.

### Gaps
- No public, pretrained depth-to-action policy was found that has been validated on an OAK-D or on a Jetson Orin Nano.
- YOPO details come from a third-party repo summary. The official GitHub README was not opened.
- No pretrained-weight release was confirmed for DiffPhysDrone or the 2026 RL papers.

---

## 5. Single forward FOV: safety limits and mitigations

### Takeaway
With one forward camera of about 70-80 deg HFOV, any motion component outside the FOV cone is blind. PX4's own collision prevention blocks such motion by default (`CP_GO_NO_DATA=0`). The standard mitigations are:
- yaw to face the direction of travel (turn, then go);
- forbid reverse and large sideways components, or allow them only into remembered free space;
- keep a short memory of obstacles that left the view (rolling map or NanoMap-style frame history);
- plan yaw to look at where you will be (RAPTOR-style);
- cap speed so stopping distance stays within reliable depth range.

### Cited Findings
- PX4 CP default `CP_GO_NO_DATA = 0` blocks movement in directions with no sensor coverage, and setting it to 1 is described as risky — [PX4 docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)
- EGO-Planner's own paper attributes local minima to "the camera has no vision of the back of the obstacle" and says limited FOV "puts greater challenges" when a new threat appears — [arXiv 2008.08835](https://arxiv.org/html/2008.08835v2)
- RAPTOR plans yaw so the quadrotor scans space relevant to safe navigation, and uses a perception-aware strategy to observe unknown obstacles before reaching them — [arXiv 2007.03465](https://arxiv.org/pdf/2007.03465)
- The CMU forward-arc planner restricts itself to forward arcs (vertical velocity + yaw rate varied, forward speed fixed). It uses NanoMap's 1 s frame history to remember obstacles that left the FOV and always schedules a stopping primitive — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
- SUPER keeps a safety trajectory inside known-free space alongside an exploratory trajectory through unknown space, as a fallback if the unknown region turns out to be occupied — [HKUST research portal](https://researchportal.hkust.edu.hk/en/publications/safety-assured-high-speed-navigation-for-mavs/)
- In FlightBench, occluded walls that "appear suddenly" were a primary collision cause in the maze scenario — [FlightBench](https://arxiv.org/html/2406.05687v2)
- Thin obstacles are a depth-sensor blind spot: the CMU forest collisions came from thin branches the D455 missed — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
- For high-speed flight UZH tilted the D435i 30 deg to look forward — [arXiv 2110.05113 (snippet)](https://arxiv.org/pdf/2110.05113)

### Inferences
- **Concrete rules for this platform:**
  - Command yaw = direction of the planned velocity, filtered.
  - Only translate when the heading error is below about (HFOV/2 - 10 deg), roughly 25 deg. Otherwise rotate in place first.
  - Never command reverse except along the exact path just flown, with the recorded map cells still free.
  - When blocked, or when the path turns sharply, do a stop-and-scan yaw sweep to fill the rolling map before choosing a direction.
- **Speed cap from depth reliability:** stopping distance is about v^2/(2a) + v*t_latency. For example, a=3 m/s^2 with 0.3 s total latency gives about 1.3 m at 2 m/s and about 2.4 m at 3 m/s.
- **Passive stereo depth error** grows with z^2. With f of about 880 px (1280 px width, about 72 deg HFOV) and b = 0.075 m, a 1 px disparity error is about 0.14 m at 3 m and about 0.38 m at 5 m.
- These numbers suggest cruising at no more than about 2 m/s in clutter so that obstacles are reliably mapped well beyond the stopping distance. This is an estimate, not a measured OAK-D figure.
- **Nets and thin poles** may produce few or no stereo matches. Hard-code the course boundary as a geofence in the odom frame from known course dimensions instead of relying on depth to see the net. "No return" pixels in the central window should count as unknown, not free.

### Gaps
- No OAK-D S2-specific measurement of usable depth range on low-texture obstacles or netting was found in this pass. This belongs to the depth-sensor researcher.
- No source was found that quantifies how much a yaw-toward-velocity policy improves safety for single-camera drones.

---

## 6. Effect of odometry drift and pose glitches on map-based planning; making the map robust

### Takeaway
Fusing depth into a world-frame map turns pose error into map error. Slow drift smears and duplicates obstacles ("ghosts"). Pose lag or a jump turns ray-cast clearing into an "eraser" that deletes real obstacles. NanoMap showed that ignoring 5 % drift made crashes happen more than once in four flights, versus 2 % with uncertainty modelling. Robust practice:
- keep maps local, rolling and short-lived;
- weight occupied evidence over free evidence;
- refuse to integrate frames during flagged VIO glitches;
- keep a pose-independent depth-image emergency stop.

### Cited Findings
- Traditional occupancy mapping "focuses on incrementally fusing 3D data into a common world frame," which makes it "sensitive to state estimation errors." NanoMap instead stores relative transforms + depth frames so poses can be updated cheaply. With 5 % drift and no uncertainty modelling the drone crashed more than once in every four flights, vs a 2 % crash rate with uncertainty modelling — [arXiv 1802.09076](https://arxiv.org/pdf/1802.09076); [IEEE Spectrum](https://spectrum.ieee.org/modeling-uncertainty-helps-mits-drone-zip-around-obstacles)
- **"Camera into eraser"** (NVIDIA forum, Jan 2026): pose from PX4 EKF2 lagged the depth image, so integrating into nvblox cleared valid voxels at the edge of view when the camera turned. NVIDIA's suggested mitigations — [NVIDIA forum](https://forums.developer.nvidia.com/t/nvblox-sensor-delay-turns-camera-into-eraser/358129):
  - lower `free_region_occupancy_probability` toward 0.5;
  - raise `occupied_region_occupancy_probability`;
  - raise `min_duration_since_occupied_for_freespace_ms` from 2000 to 3000-5000 ms;
  - raise `max_unobserved_to_keep_consecutive_occupancy_ms` beyond 10000 ms — "so NVBlox only clears cells that haven't been reaffirmed as occupied for a while." No confirmed fix appears in the thread.
- nvblox local-map controls: `map_clearing_radius_m` clears the map outside a radius around `map_clearing_frame_id` (usually base_link). One table lists a default of 5.0, and values <= 0 disable it. A `tsdf_decay_factor` (example 0.95) appears under a TSDF decay integrator. Setting `decay_tsdf_rate_hz: 0.0` disables decay, at the cost of a continuously growing map and GPU memory (snippets; names vary by version) — [NVIDIA forum: nvblox loop closure (snippet)](https://forums.developer.nvidia.com/t/does-nvblox-support-loop-closure/275738); [NVIDIA forum: long-term reconstruction D435i (snippet)](https://forums.developer.nvidia.com/t/how-can-i-use-nvblox-to-perform-long-term-3d-reconstruction-with-my-own-realsense-d435i-camera/350746)
- Fast-Planner's `plan_env` raycasts each depth/pose pair into a probabilistic volumetric map, so free-space observations clear cells — [Fast-Planner README](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
- The EGO-Planner abstract says obstacle information "will be stored only if the trajectory hits new obstacles," a minimal-memory design — [arXiv 2008.08835](https://arxiv.org/abs/2008.08835)
- A real-world failure from drift: one CMU cave collision was caused by VINS drift, even with a reactive planner and only 1 s of NanoMap memory — [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)
- PX4-Avoidance's global planner "needs accurate global position and heading," which is why the local (histogram) planner was preferred — [PX4/PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)

### Inferences
- **Drift budget:** at the measured 2-3 % drift, a rolling map whose memory covers the last about 5 m of travel accumulates about 0.10-0.15 m of relative error. That fits inside a 0.2-0.3 m inflation margin. Memory over about 20 m of travel (0.4-0.6 m error) would create ghost or doubled obstacles big enough to block gaps. Size the memory window by distance travelled, or about 5-10 s at 1 m/s, rather than by course length.
- **Glitch handling:** when the cuVSLAM speed/rotation guard trips:
  - skip map integration for that frame;
  - if the pose jumped, reset the rolling map to just the current frame (re-seed from depth) instead of fusing across the jump;
  - drop the velocity command to hover until the guard clears.
- **Timing:** timestamp-match the depth frame to the cuVSLAM pose of the same stereo frame. The two come from the same OAK-D frame pair, so this should be exact, which avoids the nvblox "eraser" lag case. Do not use the PX4 EKF2 pose for integration if it lags.
- **Occupied over free:** use asymmetric log-odds (hit +0.85, miss -0.4, clamped) and require several misses or a minimum time before clearing an occupied cell. Cells outside the FOV only decay, never clear.
- **Pose-independent last line of defence:** a direct depth-image check (minimum valid depth in the central window scaled to the velocity direction < stop distance, so brake) needs no pose. It covers both map corruption and VIO glitches.

### Gaps
- No published quantitative study was found on ghost-obstacle rates vs VIO drift for EGO-Planner or nvblox on drones.
- Authoritative nvblox 3.2 parameter defaults for decay and clearing were not retrieved. The values above are from forum snippets.

---

## 7. Recommended minimal stack (48-hour build) vs stronger stack; ranked candidates

### Takeaway
For a 48-hour build, use a ROS-free Python pipeline:
- OAK-D depth;
- a drone-centred rolling 2.5D occupancy grid with ray-cast clearing and distance-bounded memory;
- A* toward the next waypoint;
- velocity-aligned yaw with offboard velocity setpoints (MAVSDK);
- a pose-independent depth-image emergency brake;
- a hover-on-VIO-glitch rule.

With more time, the stronger stack is nvblox (Isaac ROS 3.2 / Humble on JetPack 6, or the core library) providing a 3D ESDF or 2D slice, plus an EGO-Planner/MINCO-class local trajectory optimizer and a global grid planner. Learned policies and PX4-Avoidance are not recommended for this timeline.

### Cited Findings

Candidate matrix. Every cell is sourced in sections 1-6. Sources are repeated in the last column.

| Candidate | Inputs → outputs | Compute evidence | Maturity / status | ROS dependency | Sources |
|---|---|---|---|---|---|
| PX4-Avoidance local_planner (3DVFH*) | PointCloud2 → MAVROS setpoints | Ran on TX2 / NUC / Atom; point clouds at 10-20 Hz | **Archived Aug 1, 2024**, unmaintained | ROS 1 Noetic + MAVROS | [repo](https://github.com/PX4/PX4-Avoidance) |
| PX4 Collision Prevention | OBSTACLE_DISTANCE (72 x 5 deg) → velocity limiting in FC | Runs on FC; tested 10 Hz at 4 m/s | Maintained in PX4, but **Position mode only**; companion path "untested" | None (MAVLink) | [PX4 docs](https://docs.px4.io/main/en/computer_vision/collision_prevention.html) |
| RAPPIDS | single depth image → collision-free primitives | 30 Hz on ODROID-XU4 | Research code, 4 commits, GPL-3.0 | None | [repo](https://github.com/nlbucki/RAPPIDS), [arXiv](https://arxiv.org/abs/2003.01245) |
| NanoMap (+ forward-arc primitives) | depth frames + relative poses → proximity queries | 12 Hz replanning on Orin AGX (CMU 2024); 10 m/s flights (MIT 2018) | nanomap_ros open source; forward-arc code not released | ROS 1 (nanomap_ros) | [arXiv 2411.04326](https://arxiv.org/html/2411.04326v1), [arXiv 1802.09076](https://arxiv.org/pdf/1802.09076) |
| nvblox / Isaac ROS nvblox | depth + pose (+color) → TSDF/ESDF, 2D Nav2 costmap | Orin Nano: TSDF 1.2-2.1 ms, ESDF 5.1-6.2 ms; dynamics N/A on Nano | Active (Isaac ROS 5.0, Sep 2026); **JP6 → Isaac ROS 3.2 / Humble**; Orin Nano setup and memory complaints | ROS 2 (core lib has Python/C++) | [docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html), [forum](https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982) |
| EGO-Planner / EGO-Swarm | depth or cloud + odom → B-spline traj | ~1 ms planning; 24.9 ms total/cycle on Orin NX | Widely used; ROS 2 branch is sim-only (Humble, CycloneDDS) | ROS 1 (ROS 2 branch partial) | [repo](https://github.com/ZJU-FAST-Lab/ego-planner), [FlightBench](https://arxiv.org/html/2406.05687v2), [ROS 2 branch](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version) |
| Fast-Planner (+RAPTOR) | depth + odom → ESDF → kinodynamic + B-spline | 39.2 ms/cycle on Orin NX (mapping 36.9 ms) | Last dated update Mar 2021 | ROS 1 Melodic/Noetic | [repo](https://github.com/HKUST-Aerial-Robotics/Fast-Planner), [FlightBench](https://arxiv.org/html/2406.05687v2) |
| FASTER | depth + state → JPS + convex corridors → traj | No timings in README | ROS Kinetic/Melodic; **needs Gurobi** | ROS 1 | [repo](https://github.com/mit-acl/faster) |
| GCOPTER / MINCO | corridors/map → optimal traj (no mapper) | "real-time" (no numbers) | Last dated update Jul 2022 | ROS 1 (catkin) | [repo](https://github.com/ZJU-FAST-Lab/GCOPTER) |
| SUPER | LiDAR point cloud → dual trajectories | >20 m/s, half the baseline planning time | Science Robotics 2025; preview code | ROS 1 / ROS 2 | [HKUST](https://researchportal.hkust.edu.hk/en/publications/safety-assured-high-speed-navigation-for-mavs/) |
| agile_autonomy (learned) | depth + state + reference → trajectories | needs >= 15 Hz; TX2 onboard in paper | Pretrained ckpt (1-10 m/s); sim-only repo docs | ROS 1 Noetic + TF 2.4 | [repo](https://github.com/uzh-rpg/agile_autonomy) |
| YOPO (learned) | depth + state → trajectory | ~20 ms INT8 on RK3566 (secondary) | Pretrained weights; last push Dec 2025 | ROS 1 catkin | [summary](https://gittrend.io/repo/TJU-Aerial-Robotics/YOPO) |
| DiffPhysDrone (learned) | 16x12 depth + attitude → thrust accel | Runs on $21 Mango Pi | Training code; no weights found | None stated | [paper](https://arxiv.org/html/2407.10648v2), [repo](https://github.com/HenryHuYu/DiffPhysDrone) |

- Reference integration on Orin Nano: a community GPS-denied drone runs Isaac ROS VSLAM on an Orin Nano with PX4 and a RealSense D435i, fused in an EKF. No nvblox, and no rates given (snippet) — [Hackster project](https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417)

### Inferences

**Ranking for a team with days to integrate** (fit = obstacle coverage x integration risk x compute headroom):
1. **Hand-rolled 2.5D rolling grid + A* + reactive safety layer (Python, no ROS).**
   - Pros: fits the existing PyCuVSLAM process model, CPU-only (leaves the GPU to cuVSLAM), handles memory of out-of-FOV obstacles and dead ends via grid search, and every failure is debuggable. Fly-over is limited to "ignore below band."
   - Cons: no smooth trajectory optimization. Mitigate with a velocity cap of about 1-2 m/s and pure-pursuit on the A* path.
2. **Depth-image reactive planner (gap-finding / VFH-polar or forward-arc primitives) with a 1-3 s NanoMap-like frame history, steering toward a waypoint.**
   - Pros: simplest, most drift-tolerant.
   - Cons: dead ends and U-traps, and it needs a scripted "stop, yaw-scan, pick a new heading" recovery. A good fallback, or the inner safety layer of option 1.
3. **nvblox (3D ESDF or 2D slice) + own A*/EGO-style optimizer**, via Isaac ROS 3.2 / Humble on JetPack 6 or via the nvblox core library.
   - Pros: better maps, GPU-fast, built-in decay/clearing, and a 3D option for multi-height obstacles.
   - Cons: documented Orin Nano setup failures and memory pressure next to VSLAM, plus ROS 2 + pose bridge work. It is the right "stronger stack" if there is 1-2 weeks.
4. **EGO-Planner / EGO-Swarm (ROS 1 in a Noetic container, or the sim-only ROS 2 branch) + a PX4 bridge** (Fast-Drone-250-style PX4Ctrl or MAVROS).
   - Pros: proven depth-camera planner with good trajectories.
   - Cons: ROS 1 on JetPack 6, D435-specific driver assumptions, low maze success in FlightBench, and it still needs a global route. Strong if time allows and the team knows ROS.
5. **Learned policies (YOPO / agile_autonomy pretrained).** Experimental only. There is a domain gap to OAK-D passive stereo and the 7-inch dynamics, and ROS 1 stacks.
6. **PX4-Avoidance (archived Aug 2024).** Not recommended: unmaintained, ROS 1 + MAVROS, D435-oriented.
7. **PX4 Collision Prevention.** Not usable as the offboard planner (Position mode only). Optionally enable it for manual Position-mode takeovers by streaming OBSTACLE_DISTANCE from the companion.

**Minimal 48-hour architecture** (single Python process or two processes, consistent with "one process per camera"):
- **Thread A:** the existing cuVSLAM loop publishes pose + guard flag + timestamp.
- **Thread B, depth/map at 10-15 Hz:**
  - Take OAK-D on-device stereo depth (keeps Orin GPU free; on-device depth is standard OAK behaviour, not re-verified here), median-filtered and downsampled to about 160x100.
  - Back-project, height-band filter, then ray-cast into a 12x12 m, 0.1 m rolling log-odds grid.
  - Clearing only inside the current FOV, occupied-biased, distance-bounded memory (reset on a guard trip). Course boundary as a geofence.
- **Thread C, planner at 10 Hz:**
  - Inflate the grid, A* to the current waypoint (unknown = high cost), lookahead point, then velocity vector (cap 1.5-2 m/s, reduced near obstacles) + yaw toward the velocity direction (turn in place if the error exceeds about 25 deg).
- **Thread D, offboard I/O at 20-50 Hz:**
  - MAVSDK offboard velocity_ned + yaw setpoints.
  - Depth-image emergency brake (independent of pose).
  - Hover if pose is stale for more than 0.2 s, the guard trips, or the depth stream drops for more than 0.3 s, mirroring PX4 CP's 0.5 s rule.
- **Test order:** replay recorded OAK-D datasets (the repo already has `datasets/`) through the mapper and planner offline before any flight.

**Stronger stack (2+ weeks):**
- nvblox 3D TSDF/ESDF at 5-10 cm, fed by cuVSLAM poses, with decay and clearing tuned per the forum guidance.
- 3D grid A* or JPS global path through waypoints, then an EGO/MINCO-style local trajectory optimizer.
- RAPTOR-style yaw planning to look ahead.
- A SUPER-style known-free backup trajectory for safety.
- A trajectory follower on the Jetson sending position + velocity + acceleration setpoints to PX4.

### Gaps
- The time actually needed to bring up Isaac ROS 3.2 nvblox on an Orin Nano with an OAK-D (rather than RealSense) is unknown. No OAK-D + nvblox Orin Nano report was found.
- No measured combined GPU and memory load of PyCuVSLAM + any mapper on an Orin Nano 8 GB was found. This should be measured with jetson-stats on the team's own hardware.
- No source documented a MAVSDK-based (non-ROS) integration of EGO-Planner or nvblox with PX4.
