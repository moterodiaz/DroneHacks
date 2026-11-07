# Fence the net, map the obstacles

The most reliable way through a netted indoor obstacle course with this kit is to stop asking the stereo camera to see the net. Enforce the netting as a virtual box in the cuVSLAM frame, spend the OAK-D S2's depth on the obstacles, and fly a ROS-free Python stack at **1 to 1.5 m/s**. The stack has four parts: on-device 640x400 stereo, a drone-centred rolling 2.5D occupancy grid with A* toward measured waypoints, a depth-image brake that does not depend on the pose estimate, and MAVLink Offboard velocity setpoints to PX4 v1.17. A 2 mm net strand fills a whole pixel only inside about **1.6 m at 1280x800 (0.8 m at 640x400)**, and Skydio's own safety guide warns users not to fly near netting, so depth cannot stop the drone in time above walking pace. PX4's built-in Collision Prevention shapes only pilot stick input in Position mode, and PX4-Avoidance is archived, so the companion computer makes every avoidance decision. The precedents agree: teams that scored in netted indoor events flew waypoints in a known or measured course frame and used perception to correct drift, while purely reactive or full map-and-plan stacks crashed or stalled. Where the notes disagree, this report uses the unit's own calibration (MinZ 0.32 m at 400P, not the brochure's 0.4 m) and PX4's jerk-limited braking model with a 0.4 s delay (about 1.3 m to stop from 1 m/s and 2.1 m from 1.5 m/s). It also treats the team's 2.3 % hand-held drift as 3 % nominal and 5 % pessimistic in flight. The 12 days of preparation should build and replay that stack against recorded data and PX4 SITL. The 48 hours should go to measuring the course, re-anchoring drift on visible landmarks, and banking one slow hands-off run before chasing speed.

## On-device 400P stereo gives a trustworthy window of about 0.3 to 5 m

Depth from this camera is geometry first. Depth equals focal length times baseline divided by disparity. The RVC2 searches 0 to 95 px of disparity (0 to 190 px with extended disparity), so the closest measurable distance shrinks as resolution drops ([Luxonis: Configuring Stereo Depth](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/)). This unit's calibration gives fx = 799.13 px at 1280x800 and 399.6 px at 640x400, with a 74.97 mm baseline. From those, MinZ is **63 cm at 800P, 32 cm at 400P (or 800P with extended disparity) and about 20 cm at 400P with extended disparity**, where the lenses' 19.6 cm fixed-focus floor takes over ([unit spec sheet](../camera/docs/oak-d-s2-specs.md)). The product page's 80 / 40 / 20 cm figures are family boilerplate printed identically for several models, so the calibrated values win ([OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2)).

Error grows with the square of distance. Luxonis rates median error **below 2 % under 4 m, below 4 % at 4 to 7 m and below 6 % beyond**, but it measured that on a textured random-noise board with 5-bit subpixel, using an OAK-D Pro rather than an S2 ([Luxonis Depth Accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/)). At 400P, one pixel of disparity spans 13 cm of depth at 2 m and 53 cm at 4 m before subpixel refinement (derived from the same calibration). The useful horizon for avoiding small obstacles is therefore about 4 to 5 m. Beyond 7 m, depth says only that something is there.

### Two outputs from one pipeline resolve the resolution conflict

RVC2's compute budget pushes depth down to 400P, but cuVSLAM needs full resolution. LR-check plus subpixel takes **10.1 ms at 400P but 30.5 ms at 800P**, plus 1.5 ms versus 6 ms of rectification ([Luxonis v2 StereoDepth node](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)). At 800P, stereo alone fills the whole 33 ms frame at 30 fps. Meanwhile, the team's own walk downscaled to 640x400 drifted **12 to 15 %, against 2.3 % at 1280x800** ([team cuVSLAM guide](../methods/cuvslam-vio/docs/cuvslam.md)). DepthAI v3 resolves this on paper, because a Camera node can serve several `requestOutput` calls ([Luxonis v2-to-v3 porting guide](https://docs.luxonis.com/software-v3/depthai/tutorials/v2-vs-v3/)). One pipeline can stream raw 800P mono to the host for cuVSLAM while feeding 400P into on-device StereoDepth. Everything stays in the single process that the USB-boot camera requires ([unit spec sheet](../camera/docs/oak-d-s2-specs.md)).

No Luxonis example confirms that one RVC2 mono camera can emit both sizes at once, so that is the first bench test. The fallback is to feed 800P into StereoDepth with ×2 decimation, which costs about 30 ms of stereo time and returns MinZ to 63 cm. The combined stream is about 0.67 Gb/s, comfortable within USB3's roughly 2.5 Gb/s but over USB2's 0.48 Gb/s ([Luxonis: Optimizing FPS & Latency](https://docs.luxonis.com/software/depthai/optimizing/)). This unit enumerated as USB2 once in five boots, so the stack should refuse to arm unless the link reports SUPER ([unit spec sheet](../camera/docs/oak-d-s2-specs.md)).

### The navigation preset needs two changes to keep thin obstacles

Luxonis recommends the **ROBOTICS** preset for navigation. It turns off the temporal filter, which smears during motion, and uses LR-check, 3-bit subpixel, a 7x7 median, ×2 decimation and spatial hole-filling ([Luxonis v3 StereoDepth node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth/)). Two of those settings erase thin obstacles. A 2.5 cm pole is 5 px wide at 2 m and 2.5 px at 4 m at 400P; ×2 decimation halves that, and a 7x7 median then removes it. Run decimation 1 with a 3x3 or 5x5 median, a 0.2 to 8 m threshold, about 3 px of edge invalidation, and a brightness ceiling near 250 to drop saturated light fixtures. Even then, 2 to 3 cm poles are reliable only to about 3 to 4 m.

The confidence threshold is a trap of its own, because some Luxonis pages still describe the inverted v2 semantics. On this unit with depthai 3.10.0, 255 is most confident and a higher threshold keeps fewer pixels. At the preset value of 15, **38 to 40 % of pixels were invalid in a bright scene against 85 % in a dim one** ([depthai v3 cheatsheet](../camera/docs/depthai-v3-cheatsheet.md)). That swing in fill-rate is the strongest argument that an invalid pixel must mean unknown, never free.

The S2 has no projector and uses IR-cut filters ([OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2)). Plain walls, foam pads and evenly painted boxes therefore return holes, with depth only at edges and seams. Horizontal ropes and bars run parallel to the baseline, which defeats any horizontal stereo pair ([CMU wire detection, IROS 2018](https://www.ri.cmu.edu/app/uploads/2018/08/IROS18_1856_FI.pdf)). Handle them with map memory and physical margins.

Exposure is the hidden cost. OV9282 auto-exposure runs up to 25 to 33 ms, and DepthAI's default anti-banding is 50 Hz, which is wrong for US mains ([Luxonis OV9282 page](https://docs.luxonis.com/hardware/sensors/OV9282); [Luxonis v3 CameraControl](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/camera_control/)). A 60°/s yaw during a 25 ms exposure smears about 10 px at 400P, enough to erase a pole. Capping exposure at 3 to 6 ms cuts the smear to about 2 px. The same frames feed cuVSLAM, though, and a still camera in a dark room drifted 1.3 m in 10 s under VIO ([team cuVSLAM guide](../methods/cuvslam-vio/docs/cuvslam.md)). So set 60 Hz anti-banding and tune the exposure cap for depth and VIO together.

### Turning depth into obstacles is a numpy job

Luxonis ships building blocks but no drone avoidance pipeline, and on RVC2 its PointCloud node runs on the host anyway ([Luxonis v3 PointCloud node](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/pointcloud/)). ArduPilot's RealSense script is the proven pattern. It takes a horizontal band of the depth image, compensates for pitch, cuts the field of view into 72 rays of 5°, and sends the minimum along each ray as OBSTACLE_DISTANCE at 10 Hz or more ([ArduPilot RealSense depth camera](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)).

For this stack, each frame takes six steps. Back-project the 400P depth, level the points with the attitude estimate, and keep a height band around the vehicle. Bin the points into 5° sectors; only about 16 fall inside the field of view. Take a robust minimum per sector, such as the 5th percentile with at least 5 to 20 supporting points within 10 to 15 cm. Mark every other sector unknown. The planner and PX4 both consume that single ring.

Host-GPU stereo is a fallback, not a plan. NVIDIA's ESS runs at 34.8 fps at 576p on an Orin Nano Super ([Isaac ROS DNN stereo](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)). But the Orin Nano has no DLA or PVA ([Connect Tech Orin comparison](https://connecttech.com/orin-module-comparison/)), so ESS would compete with cuVSLAM for one 1024-core GPU, and it needs ROS 2.

Heat and latency bound every session. The S2 is rated to 50 °C ambient at full load and shuts down at a 105 °C die temperature. Community fans cut chip temperature by 15 to 20 °C ([OAK-D S2 product page](https://docs.luxonis.com/hardware/products/OAK-D%20S2); [Luxonis forum, fan cooling](https://discuss.luxonis.com/d/4994-error-xlink-read-rel-resp)). The team's unit overheats within 5 to 10 minutes on the bench in still air. In flight, keep the housing in prop-wash, turn off the colour camera and encoder, log `getChipTemperature()`, and cut the pipeline at about 85 to 90 °C. A forum measurement puts 400P depth at **21 to 24 ms** from frame timestamp to host ([Luxonis forum 957](https://discuss.luxonis.com/d/957-impact-of-latency-on-practical-applications/27)). Budget 25 to 70 ms until it is measured on this unit.

No published flight test reports numbers for OAK-family obstacle avoidance. The one forum attempt with an OAK-D Lite and a Jetson Nano showed distances on the ground station, but the copter never stopped ([ArduPilot forum](https://discuss.ardupilot.org/t/simple-object-avoidance-using-oak-d-lite-camera-and-pymavlink-is-not-working/103145)).

## The net vanishes from stereo beyond about 1.6 m, so fence it in software

### Thin strands and open mesh defeat the matcher three ways

At 1280x800, a single strand covers a full pixel only within **0.8 m (1 mm strand), 1.6 m (2 mm) or 2.4 m (3 mm)**, from pinhole arithmetic on the calibrated focal length ([unit spec sheet](../camera/docs/oak-d-s2-specs.md)). Every one of those distances halves at 640x400. Drone netting typically has openings of 20 to 100 mm ([US Netting](https://www.usnetting.com/drone-netting/)), so **72 to 96 % of any matching window on the net is background seen through it**. As a result, the matcher usually reports the hall wall behind the net, or nothing.

The mesh's regularity adds a third failure. A 3 cm mesh seen at 2 m repeats every 12 px against a true disparity of 30 px. Equally good false matches then sit at 18, 42 and 54 px, all inside the search range, and SGM's smoothness terms can lock onto a consistent phantom wall.

Classical stereo has a long record here. A typical two-camera algorithm detected only **5 % of wire pixels** in CMU's tests ([Keller & Scherer, arXiv 1910.04874](https://arxiv.org/abs/1910.04874)). Intel documents repetitive structures producing false, very close depth that makes drones react to obstacles that are not there ([RealSense drone depth whitepaper](https://dev.realsenseai.com/?p=41319)). An OAK-D user reported ghost obstacles from fridge and door grilles ([Luxonis forum 1961](https://discuss.luxonis.com/d/1961-wrong-depth-estimation-cased-by-repetitive-patterns)). The net produces false stops as well as missed detections.

### Commercial systems disclaim nets, and no detector is ready

Systems with far more engineering behind them disclaim nets outright. Skydio 2's guide reads: "Do not fly around thin branches, telephone or power lines, ropes, netting, wires, chain link fencing or other objects less than ½ inch in diameter" ([Skydio 2 Safety and Operating Guide](https://support.skydio.com/hc/article_attachments/360052168754/Skydio_2_Safety_and_Operating_Guide.pdf)). That 12.7 mm floor is **4 to 13 times thicker** than a 1 to 3 mm strand. DJI's manuals warn about "obstacles with small surface areas" and repeating patterns ([DJI Air 3 manual transcription](https://mavicpilots.com/threads/settings.143545/post-1616681)). DJI's answer for 12 mm wires is a LiDAR add-on ([DroneDJ](https://dronedj.com/2025/05/07/dji-drone-obstacle-sensing-module/)).

Compare the window with the stopping distance. Even the 1.6 m window of a 2 mm strand at full resolution is barely longer than the 1.3 m that PX4's braking model needs from 1 m/s, before any keep-out is added. Stereo is at best an intermittent last-moment hint near the net.

Published detectors exist but none fits a Jetson build in days:

| Approach | Evidence | Fit for this build |
|---|---|---|
| CMU dilated-CNN wire segmentation | Ran above 3 Hz on a 2017 Jetson TX2 ([Madaan et al., IROS 2017](https://www.ri.cmu.edu/publications/wire-detection-using-synthetic-data-and-dilated-convolutional-networks-for-unmanned-aerial-vehicles)) | Needs training data for this net |
| U-Net de-fencing segmenters | Trained on synthetic fence masks; the recipe is published ([arXiv 2301.10759](https://arxiv.org/pdf/2301.10759)) | The recipe could be copied for synthetic net masks |
| DDOS dataset | The only dataset with a "fences and nets" class; synthetic aerial imagery ([DDOS, CVPRW 2024](https://arxiv.org/html/2312.12494v2)) | Not indoor nets |
| ICUAS 2023 optic-flow safety-net detector | Published; full text unread ([ISM/HAL record](https://ism.univ-amu.fr/doc/hal-04139578/_1)) | Method and success rate unknown |

The cheapest experiment uses the colour camera, where strands are 3 to 4 times wider in pixels than on the mono pair: at 1080p a 2 mm strand stays at one pixel out to about 2.9 m. An FFT over image tiles would look for the mesh's period. This is the researchers' own design, not a published method, and it is a stretch goal for after the core stack flies.

### A virtual box in the odometry frame is the real defence

The defence that works is a box drawn inside the net in the cuVSLAM frame. The companion clamps every setpoint against it and limits velocity toward each wall to sqrt(2·a·(distance − margin)), the same braking logic PX4 Collision Prevention uses. That stops a drift error from turning into a fast impact.

The inset from the net is the sum of five terms: drift error, boundary-measurement error, stopping distance, vehicle half-span and net sag. At 3 % drift over 40 m flown, drift alone is 1.2 m. Adding 0.2 m of measurement error, 0.7 m to stop at 2 m/s, 0.3 m of half-span and 0.2 m of sag gives about **2.6 m**, half the width of a 10 m course. Re-anchoring every 10 m of path cuts the inset to about **1.5 m**, so drift resets matter more than raw drift (arithmetic from the research notes, using the team's measured drift in the [team cuVSLAM README](../methods/cuvslam-vio/README.md)).

The best anchors are large, stereo-visible structures: net posts and frame tubes, floor markings, the take-off pad, hoops and big obstacles. UZH did exactly this at IROS 2017, running ICP between front depth and a CAD model of the track, because it feared VIO drift would send the drone into "the protection nets" ([Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)). A slow 360° yaw at take-off can range the posts and fit the real rectangle instead of trusting nominal dimensions. Fiducial tags are an option only if the rules allow them; at IMAV 2026, installing beacons cut the autonomy factor to 0.7 ([IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)).

PX4's geofence is a backstop, not the primary layer. It checks global WGS84 position against Home ([PX4 v1.17 geofence.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/navigator/geofence.cpp)), so indoors it works only after the companion sends a synthetic `SET_GPS_GLOBAL_ORIGIN`. PX4 neither documents nor tests that path indoors. Both layers move with the VIO estimate, so neither catches a large odometry jump. A VIO health monitor that triggers hold or land on lost tracking or a velocity-innovation spike closes that gap.

"Unknown means occupied" needs a scope, or the drone will freeze in front of every blank wall. Treat a sudden rise in invalid pixels in the near-field forward window as an obstacle only within about 2.5 m of a virtual wall.

### No public rulebook puts a price on touching the net

No rulebook found sets an explicit penalty for net contact. IMAV requires only that drones stay in their zone, land if they stray, and cut motors if they go too far, on pain of "penalties or disqualification" ([IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)). Plan as if any contact is a failed run, and ask the organisers on day one.

## A rolling 2.5D grid with A* beats ROS-era planners on a 12-day clock

### Reactive planners are light but trap themselves in dead ends

Reactive planners are the lightest option and the most tolerant of drift. CMU's 2024 forward-arc planner scored motion primitives on a RealSense depth image with 1 s of NanoMap history. It replanned at 12 Hz on an Orin AGX at up to 6 m/s, with **82 % success over 1,350 simulated trials** and 571 m of collision-free real flight ([CMU, arXiv 2411.04326](https://arxiv.org/html/2411.04326v1)). Its real failures are instructive: 3 collisions in 9 forest trials from thin branches the camera missed, one from VIO drift, and one dead end that needed backtracking because "no global planner is used". 3DVFH*, the planner inside PX4-Avoidance, planned into dead ends 70.7 % of the time in city-like simulations ([Thomessen et al. 2023](https://opus.bibliothek.fh-aachen.de/opus4/files/10924/2023_Thomessen_10924.pdf)), and the repository has been archived since August 2024 ([PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)).

### Map-based stacks cost a week of ROS integration

Map-based stacks perform better but bring ROS costs that this timeline cannot absorb. **nvblox** is the only mapper with published Orin Nano timings: TSDF 1.2 to 2.1 ms and ESDF 5.1 to 6.2 ms per update at 5 cm voxels ([Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)). On JetPack 6, however, its supported path is Isaac ROS 3.2 with ROS 2 Humble ([NVIDIA forum roadmap](https://forums.developer.nvidia.com/t/roadmap-for-isaac-ros-jazzy-on-orin-devices/375982)). One Orin Nano user failed for days to bring it up ([NVIDIA forum](https://forums.developer.nvidia.com/t/jetson-orin-nano-8gb-com-problemas-para-executar-pilha-isaac-ros-nvblox/361504)). Another found 8 GB too little for VSLAM, nvblox and a camera wrapper together ([NVIDIA forum](https://forums.developer.nvidia.com/t/drone-autonomous-exploration-with-jetson-orin-3d-lidar-and-intel-realsense-depth-camera/285490)).

**EGO-Planner** plans in about 1 ms and takes 24.95 ms per full cycle on an Orin NX. It is ROS 1, though, and its ROS 2 branch is simulation-only ([EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner); [EGO-Swarm ROS 2 branch](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)). In FlightBench its success fell to **0.20 in the maze and 0.50 on multi-waypoint courses**, mostly from walls hiding later walls and local optima between obstacles ([FlightBench, arXiv 2406.05687](https://arxiv.org/html/2406.05687v2)). An indoor course with occluding obstacles of mixed size looks more like that maze than an open forest.

### Learned policies stay experiments without retraining

The learned policies (UZH agile_autonomy, YOPO and DiffPhysDrone) were trained on simulated or RealSense-like depth. None has been validated on an OAK-D or an Orin Nano ([agile_autonomy](https://github.com/uzh-rpg/agile_autonomy); [DiffPhysDrone](https://arxiv.org/html/2407.10648v2)). FlightBench found that training without latency randomization cut one method's success from 0.9 to 0.3. These policies are not deployable in 48 hours.

### A height-banded grid handles obstacles of every size

A hand-rolled 2.5D grid is the remaining option, and it fits the existing Python cuVSLAM process. Each depth frame is back-projected, downsampled, and transformed with the cuVSLAM pose of the same stereo frame. Because depth and pose come from one OAK-D exposure, the timestamps match exactly. Points inside a height band around the flight altitude are ray-cast into a drone-centred grid about 12×12 m at 0.1 m. Free space is cleared only along rays to valid pixels inside the field of view. Occupancy uses asymmetric log-odds, roughly +0.85 per hit and −0.4 per miss, so it takes several misses to clear a cell. The grid is inflated by the drone's radius plus a margin, and A* runs to the next waypoint with unknown cells at high cost. Pure pursuit on the resulting path yields a velocity and a yaw.

nvblox's own 2D costmap slice works the same way through its `min_height` and `max_height` parameters ([nvblox 2D slice parameters](https://github.com/jahaniam/isaac_ros_nvblox)). The band also decides how obstacles of different sizes are handled: anything below it is flown over, anything inside it is flown around. Flying over still needs map memory, because a forward camera with a 55° vertical field of view loses an obstacle's top as the drone passes above it.

### Five metres of memory keeps drift inside the margin

Drift sets how long the map may remember. NanoMap's authors found that ignoring 5 % drift crashed more than one flight in four, against 2 % with uncertainty modelling ([NanoMap, arXiv 1802.09076](https://arxiv.org/pdf/1802.09076)). At 2 to 3 % drift, a map covering the last 5 m of travel carries 0.10 to 0.15 m of relative error, which fits inside the inflation margin. Memory over 20 m carries 0.4 to 0.6 m, enough to close real gaps with ghost obstacles.

Pose lag fails the other way: ray-cast clearing becomes an "eraser" that deletes real obstacles, as an nvblox user found with lagged PX4 poses ([NVIDIA forum](https://forums.developer.nvidia.com/t/nvblox-sensor-delay-turns-camera-into-eraser/358129)). So bound memory by distance travelled, skip integration while the cuVSLAM guard is tripped, and reset the grid on a pose jump.

The single forward camera imposes the rest, following PX4's precedent of blocking motion into sectors with no data ([PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)). The drone yaws toward its planned velocity and translates only when the heading error is under about 25°. It never reverses except along the path it just flew, and it stops and sweeps yaw whenever it is blocked. Beneath all of this sits a brake that ignores the map entirely. It fires when the minimum valid depth in the window along the velocity direction falls below the stopping distance, which covers both map corruption and VIO glitches.

## PX4 v1.17 will fuse the VIO but will not dodge anything in Offboard

### Stay on v1.17.0 and distrust pre-v1.16 tutorials

Target **PX4 v1.17.0**, the stable release from 2026-05-13; v1.18 was still at rc1 on 2026-09-10 ([PX4 releases](https://github.com/PX4/PX4-Autopilot/releases)). v1.16 moved Collision Prevention to acceleration-based Position mode and removed the companion avoidance interface. Older tutorials therefore cite parameters such as `COM_OBS_AVOID` and `MAV_ODOM_LP` that no longer exist ([v1.16 release notes](https://docs.px4.io/main/en/releases/1.16.html)).

### MAVSDK's default silently drops the VIO

VIO should go to PX4 as MAVLink **ODOMETRY**: `frame_id = MAV_FRAME_LOCAL_FRD`, `child_frame_id = MAV_FRAME_BODY_FRD`, `estimator_type = VIO`, full covariances, `reset_counter` and `quality`. Stream it at **30 to 50 Hz**. VISION_POSITION_ESTIMATE cannot carry velocity or quality, and EKF2 does not fuse vision that arrives too slowly ([PX4 External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)).

The worst trap is silent. MAVSDK's Mocap plugin defaults `estimator_type` to MOCAP ([MAVSDK mocap_impl.cpp](https://github.com/mavlink/MAVSDK/blob/main/cpp/src/mavsdk/plugins/mocap/mocap_impl.cpp)), and PX4 v1.17 routes MOCAP odometry to a topic EKF2 never reads ([PX4 v1.17 mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/mavlink/mavlink_receiver.cpp)). The VIO would be streamed but never fused. Build ODOMETRY with pymavlink instead.

### Map the cuVSLAM guard onto PX4's reset and quality fields

The guard's states map onto two fields EKF2 already watches ([PX4 v1.17 ev_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp)). Increment `reset_counter` only when cuVSLAM re-initialises or the guard re-seeds the pose, so EKF2 snaps to the new position instead of fighting the jump. Lower `quality` while the guard is coasting on the gyro. With `EKF2_EV_QMIN` set just above that value, EKF2 stops fusing extrapolated poses.

EKF2 declares vision stopped after 400 ms without samples, so any gap over about 0.5 s should trigger a hold from the mission logic. PX4's MAVLink timesync needs 500 samples to converge, about 50 s at 10 Hz. Until then PX4 stamps vision with its arrival time, so wait a minute after connecting before arming ([PX4 v1.17 Timesync.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/timesync/Timesync.cpp)).

### Collision Prevention only protects a safety pilot

Collision Prevention runs only inside `StickAccelerationXY`, so it shapes stick input in Position mode and leaves Offboard setpoints untouched ([PX4 v1.17 StickAccelerationXY.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp)). It is still worth feeding as a net for a safety pilot who flips to Position mode near the netting. Send OBSTACLE_DISTANCE in `BODY_FRD` with `increment_f = 5` and `angle_offset = −35°`, filling about 15 sectors and setting every other one to `UINT16_MAX`. Set `min_distance` to 32 cm and stream at 15 to 20 Hz ([MAVLink OBSTACLE_DISTANCE](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE); [PX4 v1.17 CollisionPrevention.cpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/collision_prevention/CollisionPrevention.cpp)). PX4 has no handler for OBSTACLE_DISTANCE_3D, so ArduPilot's 3x3-grid OAK-D script does not carry over.

### Offboard needs a steady setpoint stream over TELEM2

PX4 enters Offboard only after a setpoint stream of at least 2 Hz has run for more than a second ([PX4 v1.17 Offboard docs](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flight_modes/offboard.md)). MAVSDK resends setpoints at 20 Hz automatically ([MAVSDK Offboard guide](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)). MAVSDK v4, released 2026-09-22, replaced the Python package with native bindings ([MAVSDK v4.0.0](https://github.com/mavlink/MAVSDK/releases/tag/v4.0.0)), so pin an exact version and test the aarch64 wheel early. The Jetson connects to **TELEM2** (`/dev/ttyS3`, full flow control, 921600 baud by default), since the 6C has no Ethernet ([PX4 v1.17 Pixhawk 6C](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/flight_controller/pixhawk6c.md)). PX4 SITL with Gazebo Harmonic ships `gz_x500_depth`, with a forward depth camera "modelled on the OAK-D", and `gz_x500_vision`, which injects odometry the way external vision arrives ([PX4 v1.17 Gazebo vehicles](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/sim_gazebo_gz/vehicles.md)). Together they cover the whole loop before any props spin.

### A dozen parameter changes make PX4 an indoor VIO vehicle

The settings below come from the v1.17 source and ModalAI's indoor-VIO preset ([params_external_vision.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/params_external_vision.yaml); [ModalAI indoor_vio.params](https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-px4-params/-/blob/master/params/v1.18/EKF2_helpers/indoor_vio.params); [PX4 v1.17 commander_params.c](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/commander/commander_params.c); [PX4 v1.17 safety.md](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/docs/en/config/safety.md)).

| Parameter | v1.17 default | Set to | Why |
|---|---|---|---|
| `EKF2_EV_CTRL` | 0 | 15 | Fuse EV horizontal and vertical position, velocity and yaw |
| `EKF2_HGT_REF` | 1 (GPS) | 3 (vision); 2 with a downward rangefinder over a flat floor | No GPS; height follows the reference source |
| `EKF2_GPS_CTRL` | 7 | 0, set after `HGT_REF` | Otherwise PX4 resets it to 7 |
| `EKF2_MAG_TYPE` | 0 | 5 (none) | Indoor fields are unreliable; heading comes from EV yaw |
| `EKF2_OF_CTRL` | 1 (since v1.16) | 0 unless a flow sensor is fitted | The default enables a sensor that is absent |
| `EKF2_EV_QMIN` | 0 | Just above the guard's coasting quality | Stops fusing gyro-coasted poses |
| `EKF2_EV_DELAY` | 0 ms | Measured, then tuned in 5 to 10 ms steps | Minimises EV innovations |
| `EKF2_NOAID_TOUT` | 5 s | 1 to 2 s | Fails safe before large silent drift |
| `COM_OBL_RC_ACT` | 0 (Position) | 5 (Hold) or 4 (Land) | Offboard-loss action |
| `NAV_RCL_ACT` | 2 (Return) | 3 (Land) | Return has no meaning without a GPS home |
| `MPC_XY_VEL_MAX` | 12 m/s | 2 m/s (2.5 for the fastest-run attempt) | Hard ceiling that also applies in Offboard |
| `MPC_TILTMAX_AIR` | 45° | 25 to 30° | Gentler 7-inch flight indoors |
| `CP_DIST` / `CP_GO_NO_DATA` | −1 / 0 | 0.8 m / 1 | Position-mode safety net that still lets the pilot back away from the net |

## Winners flew known waypoints slowly and corrected drift on landmarks

### No team finished the netted IROS races

The closest precedent is the IROS Autonomous Drone Racing events of 2016 and 2017. They ran a cluttered indoor track surrounded by protection nets, with all sensing and computing onboard, and **no team completed it** ([Moon et al., "Challenges and Implemented Technologies Used in Autonomous Drone Racing"](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)). Because the layout was released in advance, the best teams flew waypoints and recognised gates.

INAOE won in 2017 at about **0.7 m/s**. It went waypoint by waypoint on monocular SLAM, used gate poles to correct heading drift, and lost minutes to a yaw controller that took 25 s to reach its first reference. UZH set waypoints on a CAD model and corrected its VIO with depth-to-CAD ICP. TU Delft was fastest, with 7 gates in 25 s. It dropped downward optical flow because blur ruined it "even with abundant texture on the floor", and it hit a filter bias bug that appeared only after a 180° turn and "was not observed during the testing in a smaller test area".

### IMAV organisers add texture and penalise outside help

IMAV's indoor events show what organisers expect. IMAV 2026 used a 14×7×4 m cage with obstacles in several sizes: two window sizes, bars to fly over or under at adjustable heights, and a 10 cm pipe. It added "textured flooring (flower wallpaper)... to improve navigation". Scores were multiplied by 0.7 for off-board computing or installed beacons and by 0.4 for any in-flight intervention ([IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)).

ISAE-SUPAERO's IMAV 2023 drone ran a ROS-less Jetson Nano on unmodified PX4 over µXRCE-DDS. It found windows by thresholding RealSense depth to 0.3 to 4 m and fitting edges into quadrilaterals, and the team itself named "frequent noise in the depth image near the window" as a weakness ([ISAE-SUPAERO IMAV 2023 paper](https://www.imavs.org/papers/2023/26.pdf)). The team chose attitude setpoints believing PX4 position setpoints need GNSS. PX4's VIO documentation shows that position control works on external-vision feedback ([PX4 VIO docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)).

At IMAV 2024 the winning TU Delft team "bumped into plants and knocked the lion off Pride Rock" and still won by completing missions autonomously ([TU Delft](https://tudelft.nl/ai/biomorphic-intelligence-lab/news-1/messages/winning-imav2024-drone-competition)).

### Landmark correction halved racing-drone drift

A2RL's 2025 races put a number on landmark correction. KAIST's gate-based correction cut mean translation error from **1.04 m to 0.56 m**, while uncorrected OpenVINS "drifted by several meters" on the longest track ([Azhari et al., arXiv 2512.20475](https://arxiv.org/html/2512.20475v1)). The same paper records risk backfiring. A 14 m/s attempt crashed on its second lap, a misconfigured controller sampling time slowed early runs, and the best-of-two format "encouraged high-risk runs".

### Integration bugs, not algorithms, stalled depth-to-autopilot builds

ArduPilot's 2020 RealSense build oscillated around its 1.5 m threshold. Its lessons were to ground-test message rates, fly gently first and pin library versions ([ArduPilot GSoC 2020](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx-depth-camera-for-simple-obstacle-avoidance/58474)). A ModalAI user with correct distances saw no stopping, and the moderator pointed to `MPC_POS_MODE` ([ModalAI forum](https://forum.modalai.com/topic/2671/collision-prevention)).

Hardware failures cluster in the same unglamorous places. Luxonis staff attribute OAK X_LINK errors mostly to power or a bad USB3 cable ([Luxonis forum](https://discuss.luxonis.com/d/996-x-link-error-couldnt-read-data-from-stream-video-in-long-term-testing/12)). Luxonis also told an Orin Nano Super user that OAK cameras randomly disconnect after a few hours, in a thread that remains unresolved ([NVIDIA forum](https://forums.developer.nvidia.com/t/oak-d-disconnects-in-jetson-orin-nano-super-developer-kit-due-issues-with-xusb-tegra/349466)). Simulation flatters every planner: on TU Delft's OAK-D drone, simulated success was "always better" than real at similar traversability ([AvoidBench, arXiv 2301.07430](https://arxiv.org/pdf/2301.07430)).

### The competition scores mapping and a pitch, not just the run

The competition itself is thinly documented. The organisers' public prize page splits scoring **65 % course run and 35 % technical pitch**, and says the course covers "obstacles, walls, hoops, mapping, and localization". It also lists a separate prize for the single fastest run. The link is omitted here to keep the event's name out of this report. No public rulebook, course map, net description or collision penalty was found.

Three implications follow. Because mapping and localization are scored, the grid and trajectory should be displayed, not just used. Hoops are thin rings that stereo captures only partly, so they probably need image-based detection like the IMAV windows, and once found they double as drift fixes. And because the organisers' announcement places the main event in a student-union building rather than a flight lab, the run should start from an RC switch and never depend on venue Wi-Fi, which competition teams report as congested ([myNARC 2024 team notes](https://silverwing-domain.notion.site/Aerial-Robotic-Competition-myNARC-2024-1317ad54d12c80aba2cbf3d945b3ca7b)).

## Bank one slow hands-off run, then buy speed with measured latency

### Reconciled numbers set 1.5 m/s as the cruise ceiling

The notes disagree on several numbers because they assumed different resolutions, latencies and braking models. The table settles each one and states the assumption behind it.

| Quantity | What the notes say | Value used here | Assumption |
|---|---|---|---|
| MinZ at the avoidance resolution | 80 / 40 / 20 cm (product page); 70 / 35 cm (Luxonis worked example); 63 / 32 / ~20 cm (unit calibration) | **0.32 m at 400P, normal disparity**; OBSTACLE_DISTANCE `min_distance` 32 cm | Calibrated fx 399.6 px and B 74.97 mm beat family boilerplate. Extended disparity (~0.2 m) is unnecessary because nothing should get inside a 0.7 m keep-out. It also adds ~10 ms and disables the median filter when combined with subpixel ([depthai v3 cheatsheet](../camera/docs/depthai-v3-cheatsheet.md)). |
| Resolution for net detection | Netting notes favour 800P with extended disparity, which doubles the net window; depth notes favour 400P | **400P in flight**; 800P with extended disparity only for the day-one net bench test | At 800P, LR, extended and subpixel together take 62 ms on RVC2, about 16 fps ([Luxonis v2 StereoDepth](https://docs.luxonis.com/software/depthai-components/nodes/stereo_depth/)), and the 1.6 m window still barely exceeds the 1 m/s stopping distance |
| Stopping distance | 0.32 / 0.97 m at 1 / 2 m/s (0.15 s, 3 m/s²); 0.25 to 0.42 m at 1 m/s (other assumptions); 1.32 / 2.10 / 2.97 m at 1 / 1.5 / 2 m/s (PX4 model) | **1.3 m at 1 m/s, 2.1 m at 1.5 m/s, 3.0 m at 2 m/s** | PX4's jerk-limited formula with `MPC_ACC_HOR` 3 m/s², `MPC_JERK_MAX` 8 m/s³ and a 0.4 s delay, the `CP_DELAY` default ([PX4 v1.17 TrajMath.hpp](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/lib/mathlib/math/TrajMath.hpp)). The constant-deceleration figures are the best case once latency is measured. |
| End-to-end latency | 21 to 24 ms (forum, depth to host); 25 to 40 ms; 40 to 70 ms; total 0.15 to 0.4 s | **0.4 s for planning until measured** | Covers host processing, MAVLink and PX4 tracking, which the PX4 docs put at 0.1 to 0.5 s |
| Required detection range | Not stated directly | **~3.2 m at 1.5 m/s; ~4.1 m at 2 m/s** | Stopping distance plus a 0.8 m keep-out plus a 0.25 to 0.3 m half-span |
| Reliable detection range | Textured boxes and walls 4 to 5 m; 2 to 3 cm poles 3 to 4 m at 400P; net strands under 1.6 m at 800P and under 0.8 m at 400P | Same | Thin poles become marginal at 2 m/s |
| Cruise speed | 0.5 to 1 m/s first run; 1 to 1.5 m/s; up to 2 m/s; 2 to 3 m/s; 3 m/s VIO guard | **0.5 to 1 m/s to bank a run; 1 to 1.5 m/s cruise; up to 2 m/s in open stretches for the fastest run** | The 3 m/s guard limit ([08_cuvslam_track.py](../methods/cuvslam-vio/08_cuvslam_track.py)) is a VIO sanity ceiling, not a target |
| VIO drift | 2 to 3 % hand-held; 2.3 % (8 of 10 runs) and 3.6 % (2 of 10) on one 8 m full-resolution walk; 2.9 to 21 % on 640x400 walks with blank walls | **3 % nominal, 5 % pessimistic in flight** | One hand-held walk is not a flight; vibration and blank-wall stretches push drift up ([team cuVSLAM README](../methods/cuvslam-vio/README.md)) |
| Net inset | 2.6 m (40 m flown, no re-anchor); 1.5 m (re-anchor every ~10 m); 1 m inside the nets (PX4 geofence example) | **1.5 m with re-anchoring; 2.5 m if no landmark is usable** | Net sag measured on site |
| Stereo HFOV | 80° (spec); 77.4° (unit intrinsics); 71.9° (Luxonis example) | **~77°, giving 15 to 16 sectors of 5°** | Calibrated fx, not the spec sheet |
| Confidence threshold | v2 docs: 0 is most confident; v3 measured: 255 is most confident | **15, with v3 semantics** | depthai 3.10.0 behaviour on this unit |
| Collision Prevention sectors | PX4 docs say 36 in one place and 72 in another | **72 × 5°** | `BIN_COUNT = 72` in the v1.17 source |

### One process, four loops, five watchdogs

The flight software is one Python process that owns the camera and runs four loops. The existing cuVSLAM loop publishes pose, velocity, covariance, guard state and timestamp. A depth-and-map loop at 10 to 15 Hz turns 400P depth into the 72-sector ring and the rolling grid. A 10 Hz planner runs A* to the next waypoint inside the virtual box and outputs a velocity and yaw, capped by the braking model and slowed near walls. A MAVLink loop at 20 to 50 Hz uses MAVSDK for arming, mode changes and Offboard setpoints, and pymavlink for ODOMETRY at 30 Hz and OBSTACLE_DISTANCE at 15 to 20 Hz.

The MAVLink loop also hosts the watchdogs. It commands a hold, then a landing if the fault persists, when any of five things happens: the pose is stale for more than 0.2 s, the guard trips, depth is stale for more than 0.3 s, the camera reaches its temperature cutoff, or the flight mode leaves Offboard. This mirrors PX4's own 0.5 s rule for lost obstacle data ([PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention)). Deliberately left out are PX4-Avoidance, nvblox, EGO-Planner (unless a teammate already runs ROS 2 on Jetson), learned policies, any reliance on Collision Prevention during Offboard, and the Return failsafe.

### Twelve days prove everything except the Jetson

The schedule's largest risk is the Jetson path. `setup_jetson.sh` has never run, because no Jetson has been on hand, and cuVSLAM has so far run only in WSL2 ([team cuVSLAM README](../methods/cuvslam-vio/README.md)). Everything that can be proven on the laptop, in SITL and on recorded data should be proven before the event, and bring-up should be scripted so it takes hours, not a day.

| Days | Build | Exit test |
|---|---|---|
| 1 to 2 | Dual-output pipeline (800P to host, 400P to StereoDepth); tuned depth settings; 60 Hz anti-banding; USB-speed and temperature guards; bench fan | Measured depth-to-host latency, chip-temperature curve, both streams in one process |
| 3 to 4 | Datasets (depth, raw frames, IMU) of boxes, poles, foam and, if obtainable, netting, walking from 4 m to 0.5 m; sector extractor with k-of-N voting | Valid-pixel fraction against distance per material; no phantom sectors on replay |
| 5 to 6 | Rolling grid, A*, virtual box, braking clamp and depth brake, all run offline on replays; map and trajectory visualisation | Sensible planner output on every recording; visuals ready for the pitch |
| 7 to 8 | PX4 v1.17 SITL with `gz_x500_vision`: MAVSDK state machine, pymavlink ODOMETRY (`estimator_type` VIO), parameter file, failsafe tests (kill the VIO process) | EV fusion flags set; hold or land within 0.5 s of an injected fault |
| 9 to 10 | SITL with `gz_x500_depth`: obstacle world with net panels, measured-waypoint following, speed profiles, one-command launch, onboard logging | Full simulated course at 1 m/s with no virtual-wall breach |
| 11 to 12 | Jetson bring-up kit (pinned wheels, udev rules, TELEM2 wiring notes, checklists); written questions for the organisers; buffer | Dry run of the setup scripts; printed checklists |

### The 48 hours climb a five-rung flight ladder

The first six hours are bring-up and measurement. Connect the Jetson over TELEM2 and verify the axes: lifting the vehicle should lower z, and moving it forward should raise x. Confirm that timesync has converged and EV fusion is active. Carry the camera around the course by hand, walk it toward the real net to measure its valid-pixel window, and range the posts to set the virtual box. The same morning, ask the organisers about net-contact rules, floor texture, whether markers are allowed, and whether the layout will be released.

The next twelve hours climb from a low Position-mode hover, with Collision Prevention as a net, to an Offboard hold and then a 1 m box, with RC takeover on a switch. By hour 30 the drone should fly the full course at 0.5 m/s with the virtual box and depth brake active and every flight logged. The goal of that stage is one complete hands-off run from take-off to landing.

Only after that run is banked should cruise rise to 1 to 1.5 m/s and hoop detection be added as both a scoring item and a drift fix. Polish the map visuals for the pitch, and attempt the fastest run last: up to 2 m/s in open stretches, with `MPC_XY_VEL_MAX` at 2.5 m/s. Throughout, keep every camera session short and temperature-logged, and cut the camera's load between flights.

## Conclusion

The decisive variable on this course is not depth quality but the drift accumulated between position fixes. Every netting margin, ghost obstacle and phantom wall in this analysis grows with the metres flown since the last re-anchor. The highest-value engineering is therefore cheap: a measured take-off pose, a yaw sweep that ranges the posts, hoops used as position fixes, and a map that forgets after about 5 m. Better stereo, whether 800P, extended disparity or learned depth, moves the net-detection window by a metre at most, and it costs latency that the stopping-distance budget cannot spare. The scored "mapping and localization" element and the 35 % pitch reward the grid and trajectory the drone already navigates by, so the visualisation is part of the stack, not an extra.

The likeliest failures are integration defaults, not algorithms: MAVSDK's MOCAP estimator type, EKF2's default-on optical flow, a Return failsafe with no GPS home, a USB2 enumeration, an OAK cooking in still air. Each is invisible in simulation and fatal in the hall, which is why the 12 days are best spent making them fail loudly on the bench. Two unknowns can still reshape the plan on day one. The first is how the real net looks to this camera, which takes ten minutes to measure. The second is whether the organisers release the layout, which decides between flying measured waypoints with reactive corrections and exploring with the grid. The architecture here supports either.
