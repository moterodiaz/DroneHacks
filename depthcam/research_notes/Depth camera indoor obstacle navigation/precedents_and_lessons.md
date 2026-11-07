# Precedents and lessons: depth-camera indoor obstacle navigation for small drones (competitions, hackathons, student builds)

Scope: indoor, GPS-denied drones that fly an obstacle course using onboard sensing, with a focus on systems built under tight time limits. Platform being prepared for: 7-inch quad, Pixhawk 6C / PX4, Jetson Orin Nano 8 GB, Luxonis OAK-D S2 (passive stereo, 7.5 cm baseline, no IR projector) facing forward, cuVSLAM stereo VIO. Research date: 2026-10-25. The target event is called "the competition" throughout.

---

## Q1. Which competitions and events use indoor (often netted) obstacle courses for autonomous drones, and what are their layouts, scoring, and rules?

### Takeaway
The closest public analogues are the IMAV indoor competitions (2023, 2024, 2026) and the IROS Autonomous Drone Racing events (2016-2017). They share a pattern: a netted cage of about 14 x 7 x 4 m, windows, gates, bars and pipes of several sizes, points scaled down for any human intervention or off-board computing, short fixed time slots (about 30 min) and a mandatory kill switch and safety pilot. Where organisers handed out the course layout in advance, the teams that did best flew waypoints against that known layout and used perception to correct drift, rather than relying on pure reactive avoidance.

### Cited Findings
**IMAV 2026 indoor competition (Strasbourg, Sept 21-25, 2026; rulebook V3 dated July 22, 2026)**
- The flight arena is a cage 14 m long, 7 m wide and 4 m high, set up in a hall with a 7.90 m ceiling. "To improve navigation, textured flooring (flower wallpaper) is added under the obstacle course and under the two side passages." — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Mission 1, the obstacle course, has two areas with two types of window (red- or blue-bordered), a fixed red bar to fly over at an adjustable height, two fixed blue bars to fly under at an adjustable height, and a pipe passage (PVC pipe 10 cm in diameter). Points depend on which window size is passed (1 pt per red window, 0.5 pt per blue window) and on the bar heights chosen (0.25/0.5/0.75 pt). Harder bar settings score more. — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Autonomy factor: A = 1.0 when nothing is done during the mission and all computation is onboard; A = 0.7 for off-board computation, or if teams install navigation beacons such as UWB; A = 0.4 if any action occurs during the mission (mode change, switch, ground-station action). "Manual flight control is prohibited; only occasional interventions (switch changes, mode changes, etc.) are permitted." — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Format: each team gets a 30-minute slot to validate as many missions as possible, with 10 minutes of preparation while the previous team flies. A mission counts only "once the drone has safely returned to its designated landing area." All flights are fully autonomous with a safety pilot present. A kill switch is mandatory, indoor MTOW is 3 kg, and at most 3 drones may fly at once. A drone that leaves its zone must land, and cut its engines if it goes too far. Each drone needs a "green card" safety check on the training day. — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Bonuses: a mass factor of 1.13 (3 kg) to 2.0 (0.1 kg) multiplies the completion score. Landing on a static or moving 80x80 cm ArUco platform earns 1 or 2 pts. A multi-mission bonus rewards chaining missions in one flight. — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)

**IMAV 2024 indoor (Bristol, Sept 2024)**
- Tasks were autonomous navigation through multiple gates, landing on small and narrow targets, grasping objects, and stopping at key locations to identify projected animals. The winning TU Delft team "bumped into plants and knocked the lion off Pride Rock" and still won by completing the mission autonomously several times. — [TU Delft news](https://tudelft.nl/ai/biomorphic-intelligence-lab/news-1/messages/winning-imav2024-drone-competition)
- The indoor competition's stated themes are "precise flight in unknown spaces, physical interaction with the environment, and autonomous operations" (from the search summary of the rules PDF; I could not load the PDF itself because of a TLS certificate error). — [IMAV2024 Combined Rules 1.7](https://2024.imavs.org/wp-content/uploads/2024/03/IMAV2024_Combined_Competition_Rules_1.7.pdf)

**IMAV 2023 indoor (Aachen, Sept 11-15, 2023)**
- The challenge was collecting and stacking cone blocks. Teams could choose path alternatives of different complexity and score, such as landing on a rotating platform or passing through a window with a moving obstacle. — [ISAE-SUPAERO IMAV 2023 paper](https://www.imavs.org/papers/2023/26.pdf)
- Indoor stacking podium: 1st DIPOL (HS Nordhausen), 2nd Fantail (Univ. of Auckland), 3rd Black Bee Drones (UNIFEI, Brazil). — [IMAV 2023 results page (via search)](https://2023.imavs.org/?p=2323)

**IROS Autonomous Drone Racing, 2016 and 2017 (paper c. 2019)**
- The task was to navigate "a cluttered indoor environment without relying on any external sensing", with all sensing and computing onboard. The 2016 track had 26 gates (4 open, 22 closed, one dynamic), each with a QR code. The 2017 track replaced the open gates with tree-like obstacles and cut the closed gates to 9, again including one dynamic gate. The track was surrounded by protection nets. — [Moon et al., "Challenges and Implemented Technologies Used in Autonomous Drone Racing"](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- "No team could complete the whole racing track so far." The most successful teams "implemented waypoint tracking methods and robust visual recognition of the gates... because the complete environmental information was given to participants before the events." In 2017 INAOE won by reaching gate 9 at 3:11.6, UZH reached gate 8 at 0:35.8, and TU Delft reached gate 7 at 0:25.7. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)

**A2RL x DCL Autonomous Drone Racing (Abu Dhabi, April 11-12, 2025), outdoor-style racing included for contrast**
- Organisers supplied identical drones in a fixed configuration, and teams could change only the camera mounting direction. Navigation could use only a single forward-facing rolling-shutter RGB camera plus the flight controller's IMU, with no external aids. — [Azhari et al., arXiv 2512.20475](https://arxiv.org/html/2512.20475v1)
- Practice ran 30-60 min per day for about 10 days on a single track, and no two drones could fly at once. Each Grand Challenge phase gave two 15-min slots with only the best result counting, "which encouraged high-risk runs". The drag race allowed one official attempt with no trial. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)

**Other student events and formats**
- DASC 2026 Student UAS Competition (Orlando, Sept 15, 2026): the university tier must autonomously fly a basic course using computer vision and identify rubber ducks, each confirmed only after a 2 s hover above it. Runs are scored on speed, precision, algorithm performance and object identification. Teams get multiple runs, the best counts, and battery swaps are allowed. In the high-school indoor tier, missing a required gate (LED-lit, about 4x4 ft) disqualifies that run. — [DASC 2026 UAS Competition](https://dasconline.org/2026/uas-competition)
- ICUAS'23 UAV competition: drones had to navigate waypoints in a "GPS-denial arena" with obstacles in unknown layouts. Teams did not know the drone hardware, flight settings or arena in advance and had to deploy code in person on competition day. A simulation round of 39 teams selected 5 finalists. — [PolyU news](https://www.polyu.edu.hk/aae/news-and-events/news/2023/jun09_icuas23/)
- ICUAS'25 and '26 used Crazyflie swarms positioned by OptiTrack motion capture, so they were not GPS-denied in practice. — [Bitcraze blog, May 2025](https://www.bitcraze.io/2025/05/crazyflies-locating-threats-in-urban-environments/)
- UAV Hackathon 2023 (York AAIP + Oxford ORI): this was a six-month "hackathon", not a 48-hour one, with 34 participants in 5 teams. The task was a safe return-to-home in a GNSS-denied mine, flown on an enclosed, scaled obstacle course in a lab with roof, wall and floor obstacles. The listed mine hazards include salt dust that clogs propellers and sensors, strong drafts, and loss of communication around corners. — [ORI UAV Hackathon](https://ori.ox.ac.uk/projects/uav-hackathon)
- WPI RBE595 "Hands-On Autonomous Aerial Robotics" (Fall 2023): a semester of projects in groups of three that ends in "an autonomous drone race on an obstacle course". Projects include "Mini Drone Race" (perception, then planning and control) and "Navigating The Unknown", in a Python/OpenCV/PyTorch stack. — [WPI PeAR RBE595](https://pear.wpi.edu/teaching/rbe595/fall2023.html)

### Inferences
- The IMAV 2026 design (multiple window sizes, bar heights and pipes, with harder options worth more) is the closest public analogue to a course of "obstacles of multiple sizes." It suggests a scoring strategy: bank a safe completion with the easy options first, then try harder variants with the remaining time or attempts.
- Organisers who add textured floors (IMAV 2026) or bright tape on gates (IROS 2017, Q4) are compensating for known failure modes of vision navigation. Teams should ask the competition organisers early whether the floor and walls will be textured.
- The common rules (kill switch, safety pilot, return-to-landing-area validation, penalties for any mode switch) mean a team should plan for a fully hands-off run from takeoff to landing, with takeover reserved for safety only.

### Gaps
- None of the rulebooks I read specified a numeric penalty for touching the safety net. IMAV 2026 says only that drones must stay in their zones or land or cut motors. I could not open the IMAV 2023 and 2024 indoor rulebook PDFs (TLS certificate mismatch, then 404 on direct download), so their collision and net rules are unverified.
- I found no team write-up from a 48-hour autonomy hackathon with an indoor netted course. The nearest documented precedents are multi-day competitions (IMAV, IROS ADR, A2RL).
- I found no F1TENTH-style drone league with standard hardware, and no IEEE or AIAA student indoor UAS obstacle event with public rules for 2020-2026.

---

## Q2. What software stacks did winning or well-documented teams use (reactive avoidance vs mapping and planning vs scripted waypoints with avoidance as a safety layer)? What indoor speeds? Did they use prior knowledge of the course?

### Takeaway
Under time pressure, scripted waypoints in a known or measured course frame, flown on onboard VIO and corrected online by perception (gate or landmark detection, or depth-to-map alignment), beat both pure reactive avoidance and full mapping and planning. Documented indoor speeds for robust runs are low, from about 0.7 m/s up to a few m/s. Teams that pushed speed or aggressiveness crashed, or saw their estimators fail.

### Cited Findings
- IROS ADR 2017 winner INAOE did not build a 3D map beforehand. It navigated waypoint by waypoint, each relative to the previous one, using metric monocular SLAM and colour-segmentation gate detection, and used the detected gate's pole base to correct heading drift. Gate dimensions and approximate positions were known in advance. Its drone flew at about 0.7 m/s and reached gate 9 at 3:11.6. Much of the time was lost to a badly tuned yaw controller that took 25 s to reach its first yaw reference. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- UZH (IROS 2017) placed waypoints on a CAD model of the track and flew on onboard VIO. They measured the start pose by hand to get an initial world-to-odometry transform, then refined it online by running ICP between a front depth-camera point cloud and the CAD point cloud. The correction was constrained to yaw plus translation, since VIO observes gravity, and applied only after enough baseline motion. They did this because "small errors in the relative orientation... as well as drift in the Visual-Inertial Odometry, could potentially lead to crashes with the gates or the protection nets." — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- TU Delft (IROS 2017) was the fastest drone (7 gates in 25 s). It used a "snake-gate" colour detector on 315x160 images plus a drag-model velocity estimate from the accelerometer, fused in an EKF. They dropped bottom-camera optical flow because motion blur at speed degraded it "even with abundant texture on the floor". — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- A2RL 2025, KAIST team: monocular VIO (OpenVINS) with gate-based drift correction and pre-computed perception-aware trajectories tracked by MPC. Mean translational error was 1.04 m for plain OpenVINS and 0.56 m with gate correction. Top speed was 43.2 km/h in the Grand Challenge. The authors note that pre-computed trajectories "are rigid": the system cannot replan after a tracking or estimation error. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- MonoRace won A2RL 2025 against the other AI teams and three human world-champion pilots, at speeds up to 100 km/h, handling camera interference and IMU saturation. This is outdoor-scale racing, included for contrast. — [MonoRace arXiv 2601.15222](https://arxiv.org/html/2601.15222v1)
- IMAV 2023, ISAE-SUPAERO team: Holybro X500 V2, Pixhawk 6X with unmodified PX4, and a Jetson Nano companion running a state machine and its own PID guidance that sent attitude and thrust setpoints every 20 ms in Offboard mode. It was ROS-less, using µXRCE-DDS directly. The PX4 offboard timeout switches to hold if the link fails, and a transmitter switch drops to Stabilized. Sensors were a forward RealSense D455 and a down-looking fisheye. Their stated reason for attitude setpoints was that PX4 position and velocity setpoints "require" GNSS. — [ISAE-SUPAERO IMAV 2023 paper](https://www.imavs.org/papers/2023/26.pdf). PX4's VIO docs contradict that rationale: position control works with external-vision position feedback. — [PX4 VIO docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
- The same team detected windows by thresholding the D455 depth image to 0.3-4 m, then applying Canny edges and Ramer-Douglas-Peucker quadrilateral fitting, then PnP. They named the limitations themselves: non-quadrilateral windows, "frequent noise in the depth image near the window", and lighting-dependent edges. — [ISAE-SUPAERO IMAV 2023 paper](https://www.imavs.org/papers/2023/26.pdf)
- PX4 Collision Prevention, the built-in reactive layer, was tested at 10 Hz sensor updates at 4 m/s. The docs note that sensor range can restrict maximum speed. — [PX4 Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- Fast-Drone-250 (ZJU FAST Lab, open-source build guide, c. 2022): a mapping and planning stack of VINS-Fusion VIO, EGO-Planner trajectory planning and the PX4Ctrl controller on a 250 mm quad with a RealSense D435(i). — [DeepWiki summary of Fast-Drone-250](https://deepwiki.com/ZJU-FAST-Lab/Fast-Drone-250). The EGO-Planner repo recommends its successor EGO-Swarm as "more robust and safe". — [EGO-Planner project summary](https://awesome.ecosyste.ms/projects/github.com%2Fzju-fast-lab%2Fego-planner)
- AvoidBench (TU Delft, Jan 2023) compared Ego-Planner, MBPlanner and Agile-Autonomy. In simulation Ego-Planner had the higher success rate, and MBPlanner had the longest processing time. — [AvoidBench arXiv 2301.07430](https://arxiv.org/pdf/2301.07430)

### Inferences
- If the competition gives or allows measuring the start pose, goal and approximate obstacle layout, the best-evidenced plan is a waypoint path in a course frame tied to a measured takeoff pose (the UZH approach). Fly it on cuVSLAM, and use depth only to (a) nudge or stop near obstacles and (b) correct drift against known structures such as walls and hoops. Full online mapping and planning (EGO-Planner class) is more fragile to integrate in 12 + 2 days.
- Because the scoring includes a "Fastest Run" award but most points come from completion (see Q6), the precedents argue for a slow, reliable run first (around 0.5-1 m/s, as INAOE won at 0.7 m/s), then raising speed only once completion is banked.
- Gate-based drift correction halved A2RL VIO error. By analogy, detecting hoops or windows and using them as position fixes would be worth implementing if hoops are scored.

### Gaps
- I found no published speed or success numbers for EGO-Planner or Fast-Drone-250 in indoor obstacle courses. AvoidBench's per-algorithm real-world success rates are only in figures I could not read numerically.
- No IMAV 2024 or 2026 winner has published a technical stack description that I could find.

---

## Q3. Documented builds with OAK-D (Lite, S2) or RealSense D435/D455 on PX4 or ArduPilot for obstacle avoidance: architecture, rates, results

### Takeaway
The standard, proven pattern is to reduce stereo depth to a ring of minimum distances (OBSTACLE_DISTANCE, 72 sectors, at 10-15 Hz or more) and let the autopilot's built-in collision prevention cap velocity: ArduPilot AVOID_*, or PX4 CP_* in Position mode. Research builds (AvoidBench with an OAK-D) instead use dense depth (400p at 30 Hz) feeding a planner on a Jetson. PX4's companion-side avoidance (PX4-Avoidance, Path Planning Interface) is archived or removed, so on PX4 the autonomy layer has to implement its own avoidance logic or rely on CP in Position mode.

### Cited Findings
**OAK-D with ArduPilot (forum post by Rishabh Singh, Dec 6, 2021)**
- Processing runs entirely on the camera with no companion computer. The stereo depth view is split into a 3x3 grid, and the closest distance per cell is sent as OBSTACLE_DISTANCE_3D over the camera's ESP32 serial port. Requirements and caveats: Copter 4.1+, scripts in beta and "not yet finalised", a 3.3 V-tolerant UART (or level shifter) with soldered RX/TX/GND. A February 2022 question about which values are sent and whether frame transforms are needed went unanswered. — [ArduPilot forum, OAK-D Part 2](https://discuss.ardupilot.org/t/easy-way-to-integrate-ai-with-ardupilot-oak-d-part-2/79307)

**RealSense D4xx with ArduPilot (GSoC 2020, Thien Nguyen, June 30, 2020)**
- Hardware: an Up Squared companion (x86, recommended for Intel library compatibility; RPi and Jetson "may have installation problems"), a D435 at 640x480 and 30 FPS, plus a TeraRanger One rangefinder and PX4Flow optical flow for stable flight.
- The filter chain is decimation, threshold, depth-to-disparity, spatial, temporal, hole filling, then disparity-to-depth, to reduce "black holes and noise".
- Depth is sampled along one horizontal line into 72 rays sent as OBSTACLE_DISTANCE (chosen over DISTANCE_SENSOR because it carries 72 distances), by default at 15 Hz; "10 Hz or higher" is needed depending on speed.
- Settings: AVOID_MARGIN 1.5 m with AVOID_BEHAVE = stop in Loiter.
- Logs showed oscillation around the 1.5 m threshold when the pilot pushed toward an obstacle. A later user on a Jetson Nano with JetPack 4.5.1 hit pyrealsense2 API incompatibilities that were not resolved.
- Lessons: ground-test first with the MAVLink Inspector and check the message rate; fly gently, then increase aggressiveness; measure the real FOV and depth limits; pin library versions. — [ArduPilot GSoC 2020 D4xx post](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx-depth-camera-for-simple-obstacle-avoidance/58474)
- ArduPilot's own doc says to mount the camera forward-facing and vibration-isolated where possible, and builds its non-ROS setup around an UP Squared. — [ArduPilot RealSense Depth Camera docs](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)

**PX4 Collision Prevention (current main docs)**
- Works for multicopters in acceleration-based Position mode (MPC_POS_MODE = Acceleration based). The docs do not say it works in Offboard or Mission.
- Input is OBSTACLE_DISTANCE from a companion. The companion setup is marked untested, and the original companion project is archived.
- If no data arrives for more than 0.5 s, horizontal velocity setpoints go to zero. After 5 s the vehicle switches to Hold.
- CP_DIST is the minimum distance, measured to the sensor. CP_DELAY covers sensor plus tracking delay (up to about 0.2 s for vision; 0.1-0.5 s tracking). CP_GUIDE_ANG is 30° as a good starting value. CP_GO_NO_DATA defaults to 0, which blocks motion into directions with no sensor coverage. — [PX4 Collision Prevention docs](https://docs.px4.io/main/en/computer_vision/collision_prevention)
- The PX4 Path Planning Interface (obstacle avoidance in missions, safe landing) is marked removed in v1.15 and "no longer supported or maintained". — [PX4 docs v1.15](https://docs.px4.io/v1.15/en/computer_vision/path_planning_interface). A Jan 2025 dev call said the ROS-side PX4-Avoidance project was archived some time earlier and proposed removing the PX4-side interface for 1.16. — [PX4 Sync Q&A Jan 15, 2025](https://discuss.px4.io/t/px4-sync-q-a-jan-15-2025/43274)

**ModalAI VOXL (Dronecode description; forum threads c. 2020-2023)**
- The pipeline is onboard stereo depth from DFS, then an obstacle map ("VOA"), then MAVLink to PX4 collision prevention. — [Dronecode](https://dronecode.org/?p=4767)
- Debug checklist from a ModalAI engineer: confirm obstacle points are published (`voxl-inspect-points voa_pc_out`), then on PX4 confirm Position mode, avoidance enabled, CP_DIST > 0 and a home position set. — [ModalAI forum](https://forum.modalai.com/post/13196)
- Users of older releases reported the DFS server not running and a missing stereo calibration file. — [ModalAI forum, collision prevention thread](https://forum.modalai.com/topic/382/collision-prevention)

**OAK-D on PX4 research platform: AvoidBench (TU Delft, Jan 2023)**
- Hardware: a 6-inch Armattan frame with 5-inch props, a Jetson Xavier NX outputting collective thrust and body rates to a Pixhawk 4 mini (PX4), and an OAK-D giving "dense depth in 400p resolution at 30 Hz".
- Three real maps of different complexity, with 20 trials each, were surveyed by lidar to compute traversability. "At similar normalized traversability, simulation results are always better than those of real world." — [AvoidBench](https://arxiv.org/pdf/2301.07430)

**RealSense on an indoor competition drone: IMAV 2023 ISAE (see Q2)**
- D455 depth thresholded to 0.3-4 m for window detection, on a Jetson Nano and Pixhawk 6X. — [ISAE-SUPAERO IMAV 2023 paper](https://www.imavs.org/papers/2023/26.pdf)

**Orin Nano + Isaac ROS VSLAM + PX4 (Hackster "GPS-Denied Drone With NVIDIA Jetson Orin Nano", c. mid-2025)**
- Stack: Isaac ROS VSLAM fused through MAVROS into the PX4 EKF, on PX4 v1.15.4, a D435i, an Orin Nano on JetPack 6.2 and an F450 frame, connected by USB-to-UART on TELEM2. NVMe storage holds container images and rosbags. Motion capture was used only for ground truth. — [DroneXL article, June 24, 2025](https://dronexl.co/2025/06/24/vslam-drone-gps-denied-navigation); [Hackster project page](https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417) (page returned 403 to me; details are from the DroneXL article and search snippets)
- An NVIDIA forum post reports that 8 GB of RAM is not enough to run Isaac ROS VSLAM, Isaac ROS nvblox and the ZED wrapper together. — [NVIDIA forum](https://forums.developer.nvidia.com/t/drone-autonomous-exploration-with-jetson-orin-3d-lidar-and-intel-realsense-depth-camera/285490) (via search summary)

**OAK-D S2 sensor facts relevant to avoidance**
- Stereo: two OV9282 global-shutter mono sensors with a 7.5 cm baseline and mono HFOV of 80° (DFOV 89.5°, VFOV 55°). Ideal depth range is about 80 cm to 12 m. MinZ is about 20 cm (400P with extended disparity), about 40 cm (400P, or 800P with extended disparity) and about 80 cm (800P). Median depth error is under 2% below 4 m and under 4% at 4-7 m. The IMU is a BNO086 (some 2021-2023 units have a BMI270). — [Luxonis OAK-D S2 docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- Power is 2.5-3 W base, plus up to 1 W for AI, 0.5 W for stereo and 0.5 W for the encoder. Operating range is -20 to 50 °C ambient with the VPU fully used. The RVC2 runs continuously up to 105 °C, after which depthai shuts the device down. No dot projector and no IR. Vibration-tested to EN 60068-2-6. — [Luxonis OAK-D S2 docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- On the OAK-D Pro, the IR dot projector gives "much better" depth on a blank wall at about 200 mA versus passive. It is disabled by default. — [Luxonis OAK-D Pro docs](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)

### Inferences
- On PX4 with Offboard control, CP_* is documented only for Position mode. A team flying Offboard trajectories should implement its own companion-side velocity cap or stop, the same "OBSTACLE_DISTANCE-like" minimum-distance-per-sector logic, inside the autonomy node, and not count on PX4 CP. CP in Position mode remains a useful safety net for manual or assisted tests.
- With a single forward camera, PX4 CP's default CP_GO_NO_DATA = 0 would block sideways and backward motion in Position mode. That matters if CP is used during assisted tests.
- Running depth at 800P puts MinZ near 80 cm, so obstacles closer than about 0.8 m drop out or become invalid. At 400P with extended disparity MinZ falls to about 20 cm. For a 7-inch quad near walls and hoops, 400P plus extended disparity for the avoidance stream is the safer default, even if cuVSLAM uses 1280x800 mono frames.
- The 3x3-grid, on-camera approach (2021) shows the minimal viable depth-to-avoidance path can be built in hours. The 72-sector OBSTACLE_DISTANCE format is a well-trodden interface to reuse internally.

### Gaps
- I found no public flight-test numbers (success rate, minimum clearances, speed) for an OAK-D S2 specifically on a PX4 drone.
- I found no Luxonis case study of an OAK-based drone doing obstacle avoidance in an indoor competition.

---

## Q4. Common failure modes reported, and how they were mitigated

### Takeaway
The recurring killers are estimator drift or divergence (in feature-poor scenes, after large turns, during fast motion), perception degraded by lighting and blur, thin structures that stereo cannot see (nets, wires, especially ones parallel to the baseline), USB disconnects of the camera, untuned timing or controller settings, and scarce flight time. Mitigations that worked were textured floors or tape added by organisers, landmark or gate-based drift correction, soft-mounted cameras, reconnect logic, conservative speeds, and heavy logging.

### Cited Findings
**VIO and odometry**
- On A2RL's longest track, uncorrected OpenVINS "drifted by several meters, an error that would cause a crash". VIO initialisation needs enough IMU excitation, and heading drifted during the manual initialisation motion. Long gate-free stretches ("perceptual deserts") left the drone on VIO alone. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- TU Delft (IROS 2017): a Kalman-filter bias-estimation bug made bias and velocity estimates diverge after the drone turned 180°, and steering went wrong although gates were correctly detected. The bug "was not observed during the testing in a smaller test area before the competition". — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- vSLAM struggles in featureless planar scenes, and loop closure failed there (ZED2i user). NVIDIA pointed to global loop closure in Isaac ROS 3.0. D435i users report that losing image features (for example, covering the camera) makes the pose jump or "teleport", with IMU message drops. — [NVIDIA forum threads via search summary](https://forums.developer.nvidia.com/t/isaac-vslam-with-realsense-d435i/321329); [NVIDIA forum, lost IMU msgs](https://forums.developer.nvidia.com/t/lost-imu-msgs-6-frame-time-delta-0-033387-s-drop-ratio-100-000000/329984)
- PX4 troubleshooting: drift or flyaways that occur in flight but not when the vehicle is carried point to vibration, so soft-mount the camera (the T265 was called out as vibration-sensitive). "Toilet-bowling" points to a camera-orientation transform or velocity-frame (FRD) error. — [PX4 VIO docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
- EKF2 fuses external vision only if it is streamed at about 30-50 Hz. EKF2_EV_DELAY usually needs empirical tuning: estimate it from the log offset between IMU and EV, or minimise innovations during dynamic manoeuvres. — [PX4 external position estimation docs v1.14](https://docs.px4.io/v1.14/en/ros/external_position_estimation)

**Lighting, blur and texture**
- IROS 2017: gates "were hardly visible in the 315x160 pixel onboard images... in the low light condition of the basement hall", and organisers added bright orange tape before the competition. Bottom optical flow degraded with blur at speed. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- A2RL: shortening exposure to reduce VIO motion blur made images darker and hurt gate detection, and training tracks differed from the final venue. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- IMAV 2026 organisers add textured flower wallpaper under the obstacle course "to improve navigation". — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- Passive stereo on a blank wall gives poor depth; the active dot projector on the OAK-D Pro fixes it. The S2 has no projector. — [Luxonis OAK-D Pro docs](https://docs.luxonis.com/hardware/products/OAK-D%20Pro); [Luxonis OAK-D S2 docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2)

**Depth noise, thin obstacles and netting**
- RealSense raw depth has "black holes and noise", handled with a 7-stage filter chain. Logs showed avoidance oscillating around the threshold. — [ArduPilot GSoC 2020](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx-depth-camera-for-simple-obstacle-avoidance/58474)
- Depth noise is frequent near window edges, which breaks quadrilateral detection. — [ISAE-SUPAERO IMAV 2023](https://www.imavs.org/papers/2023/26.pdf)
- A horizontal stereo pair "does not help" for wires running parallel to the baseline on the drone's path, because they evade the disparity image. Wires are also hard to see against cluttered backgrounds, at low contrast, or when barely visible in thickness (CMU DROAN, IROS 2018). — [CMU IROS18 wire detection paper](https://www.ri.cmu.edu/app/uploads/2018/08/IROS18_1856_FI.pdf)
- Thin wires and kite strings are "notoriously difficult" for RGB, LiDAR and depth cameras, and over 10 small-drone crashes in 2025 were attributed to kite strings. — [UCorr arXiv 2509.14989](https://arxiv.org/html/2509.14989v1)
- UZH (IROS 2017) explicitly feared VIO drift would send the drone into "the protection nets around the track" and added depth-to-CAD ICP alignment for that reason. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)

**USB, camera and compute reliability**
- Luxonis forum X_LINK_ERROR threads: staff attribute such crashes "usually" to power and ask whether units are externally powered. In one long-term test a defective USB cable was blamed, because "all 8 lines need to have contact" for USB3. Staff suggested try/except reconnect logic keyed to the device MXID, though some users needed a physical replug. One drone user suspected vibration, cold or power, and staff thought power or host throughput more likely. This is from the search summary; I could not load the thread pages. — [Luxonis forum d/996](https://discuss.luxonis.com/d/996-x-link-error-couldnt-read-data-from-stream-video-in-long-term-testing/12); [Luxonis forum d/3740](https://discuss.luxonis.com/d/3740-x-link-error-camera-disconnects-after-one-hour-ish)
- Oct 29, 2025: Luxonis told an Orin Nano Super user that OAK-D (Lite) cameras randomly disconnect after a few hours, possibly due to the xusb-tegra driver. NVIDIA said it had no reports of USB issues on JetPack 6.2.1. The thread is unresolved. — [NVIDIA forum](https://forums.developer.nvidia.com/t/oak-d-disconnects-in-jetson-orin-nano-super-developer-kit-due-issues-with-xusb-tegra/349466)
- The OAK-D S2 operating ambient limit is 50 °C with the VPU fully used, and depthai auto-shuts the device at a 105 °C chip temperature. — [Luxonis OAK-D S2 docs](https://docs.luxonis.com/hardware/products/OAK-D%20S2)
- An NVIDIA staffer says running a Jetson module without a heatsink or fan "is not recommended" because airflow on drones "might not be consistent". This was said about the original Jetson Nano. — [NVIDIA forum](https://forums.developer.nvidia.com/t/nano-heat-sink-options/74072)
- Memory: 8 GB is insufficient for VSLAM + nvblox + a camera wrapper together (user report). — [NVIDIA forum](https://forums.developer.nvidia.com/t/drone-autonomous-exploration-with-jetson-orin-3d-lidar-and-intel-realsense-depth-camera/285490)

**Control and configuration mistakes**
- A2RL: a "miss-configured controller sampling time" made early final runs slower. A 14 m/s peak-reference attempt was a second faster on lap one, then crashed into gate 12 on lap two (tracking error 0.46 m). In the multi-drone race, without collision avoidance, 2 of 4 drones crashed. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- IROS 2017 INAOE: poor yaw-controller tuning (25 s to reach the first yaw reference) and a 5 Hz IMU stream cost minutes. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)

**Radio and Wi-Fi in venues**
- A myNARC 2024 team's lesson: "Too much wifi interference - DJI Tello can't send or receive response to the computer very well", so they recommend an RPi + Pixhawk platform. — [myNARC 2024 team notes](https://silverwing-domain.notion.site/Aerial-Robotic-Competition-myNARC-2024-1317ad54d12c80aba2cbf3d945b3ca7b)
- One forum user found that "in populated areas the advantages of 2.4 GHz are largely negated by the congestion and interference" (anecdotal). — [ArduPilot forum](https://discuss.ardupilot.org/t/will-wi-fi-based-telemetry-interfere-with-2-4-ghz-radio-control/131073)

**Batteries and time**
- A2RL practice time was 30-60 min per day for about 10 days, with no simultaneous flights. — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- In a CMU MRSD 2025 progress report, the team traced missed flight-duration targets to swollen batteries about two years old. — [CMU MRSD 2025 Team B ILR04](https://mrsdprojects.ri.cmu.edu/2025teamb/wp-content/uploads/sites/84/2025/04/TeamB_yuejung_ILR04.pdf)
- UWARG flight-test log: time was lost to an uncharged transmitter battery, missing tools and a loose mount, and the fix was "better checklists". — [UWARG 4/8/2018 flight test summary](https://uwarg-docs.atlassian.net/wiki/x/F4AdCg)

### Inferences
- Safety netting is likely to be at least partly invisible to the OAK-D S2: thin strands, low texture, and horizontal strands parallel to the 7.5 cm baseline. The precedent fix is not to rely on depth for the boundary. Instead, enforce a geofence in the VIO or course frame (with a margin larger than the expected drift: 2-3% of distance flown, plus the start-pose error), and correct drift against large, visible structures (walls, hoops, obstacles).
- The TU Delft 180° bug and the A2RL "perceptual desert" drift both argue for rehearsing the full course geometry, including every large yaw turn and the longest featureless stretch, before the event. Small test rooms hide estimator failures.
- Given the OAK-D S2's 50 °C ambient limit and the team's own bench observation (project context, not a published source) of overheating within 5-10 minutes, flights should stay short with logged chip temperature. On the ground, cut camera duty (lower FPS, disable unused streams) between runs.
- USB robustness measures to pre-build: a short, high-quality, strain-relieved USB3 cable; reconnect-on-X_LINK logic keyed to MXID; and a companion-side watchdog that commands hover or land if depth or VIO frames stop. This mirrors PX4 CP's 0.5 s zero-velocity rule.
- Venue Wi-Fi will be crowded with 100+ participants. Teams should make the autonomy run independent of any ground link: start it via RC switch or onboard trigger, keep the RC link separate from Wi-Fi, and log onboard.

### Gaps
- I found no first-hand report quantifying OAK-D or OAK-D S2 thermal shutdowns in flight, or Orin Nano throttling inside a drone enclosure.
- I found no direct measurement of how stereo cameras see competition safety netting (detection rate versus mesh size and distance). This should be tested at the venue on day 1.
- I found no primary source on prop wash or dust effects on depth or VIO indoors beyond the mine-hackathon hazard list.

---

## Q5. Process lessons for a 48-hour build: what to pre-build, sim vs real ratio, logging and replay, a flight-test ladder, and scoping a minimum viable autonomous run

### Takeaway
The precedents point to a few rules. Arrive with a working end-to-end pipeline and a deploy script. Treat simulation as optimistic. Turn every practice flight into logged data. Climb a ladder: ground checks, then assisted hover, then a short scripted segment, then the full course slow, then faster. Bank one safe, fully autonomous completion before attempting speed or harder options, because limited slots and best-of formats punish risk-taking without a banked score.

### Cited Findings
- A2RL team lessons: they "prioritized rapid, iterative development and easy deployment"; used each practice flight both to collect gate-detection data and to tune parameters; and, without motion capture, benchmarked VIO candidates on a public dataset and built ground truth by batch optimisation (MAPLAB) checked against known gate poses. The best-of-two 15-min-slot format "encouraged high-risk runs". — [Azhari et al.](https://arxiv.org/html/2512.20475v1)
- ArduPilot GSoC: ground-test first (MAVLink Inspector, message rate), use USB to isolate telemetry problems, fly gently first and then increase aggressiveness, measure the real FOV and depth limits and add margin, and pin library versions. — [ArduPilot GSoC 2020](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx-depth-camera-for-simple-obstacle-avoidance/58474)
- PX4 VIO pre-flight checks: verify that lifting the vehicle decreases z, that moving forward increases x and moving right increases y, that the quaternion is near unit when level, and that velocities are in FRD. For the first flight, arm on the ground, switch to Position mode, and confirm the "green light" for position feedback. Report VIO health through CompanionProcessStatus (ACTIVE, CRITICAL, FLIGHT_TERMINATION). — [PX4 VIO docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
- The ISAE IMAV 2023 team tuned their companion-side controllers in a Simulink and PX4 SITL model before flight. They relied on PX4's native offboard-loss timeout falling back to hold, plus a transmitter switch to Stabilized for takeover. — [ISAE-SUPAERO IMAV 2023](https://www.imavs.org/papers/2023/26.pdf)
- Simulation is consistently optimistic: at similar traversability, sim success was "always better" than real (AvoidBench, OAK-D drone). — [AvoidBench](https://arxiv.org/pdf/2301.07430)
- UZH measured the takeoff pose by hand to initialise the course-to-odometry transform, then refined it online. — [Moon et al.](https://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)
- ICUAS'23 finalists wrote their code before the event and deployed it on unknown hardware on competition day. — [PolyU news](https://www.polyu.edu.hk/aae/news-and-events/news/2023/jun09_icuas23/)
- In IMAV-style formats a mission counts only after a safe return to the landing area, inside a 30-min slot with 10 min of preparation. — [IMAV 2026 Rulebook V3](https://2026.imavs.org/wp-content/uploads/2026/07/Rulebook_IMAV2026_V3.pdf)
- General student-project lessons: run "smaller, low risk tests... more often than higher risk tests" (L3Harris senior design) — [USU senior design report](https://engineering.usu.edu/files/pdfs/senior-design/fall2022-spring2023/mae/pigeon-post-the-modern-homing-pigeon-l3harris.pdf). Use checklists — [UWARG](https://uwarg-docs.atlassian.net/wiki/x/F4AdCg).

### Inferences
- A suggested ladder, assembled from the sources above (not a single source's prescription):
  0. Bench: camera and VIO run while carried by hand around the course; check axes and frames; log rosbag or equivalent.
  1. Ground: arm with props off; check the OBSTACLE_DISTANCE-style minimum-distance output and message rates.
  2. Manual hover in Position mode on VIO, with PX4 CP enabled as a net.
  3. Offboard hold, then short scripted moves (1 m box) with RC takeover on a switch.
  4. Full course at 0.5 m/s with a geofence and stop-on-obstacle.
  5. Raise speed in small steps.
- Pre-build during the 12 days of preparation: the course-frame waypoint follower; companion-side stop or slow-down from depth sectors; a geofence; watchdogs (VIO-lost, depth-lost, USB reconnect, thermal) that hover or land; one-command launch and onboard logging; and a replay tool that re-runs perception on recorded data. The event's 48 hours then go to course measurement, tuning and rehearsal.
- Minimum viable autonomous run: takeoff, fly a pre-measured path at low speed with stop-if-closer-than-X, land in the designated area, all hands-off. Hoop detection, mapping output and speed come after that run is banked.

### Gaps
- I found no source giving a quantitative sim-to-real time ratio for successful hackathon or competition teams.
- I found no public 48-hour drone hackathon post-mortem documenting a specific test ladder or failure log.

---

## Q6. Public information about the competition (university-hosted 48-hour GPS-denied UAV autonomy hackathon, November 2026)

The event's name, host, venue and links are left out of this repository; the sources are the competition's own website, prizes page and announcement post.

### Takeaway
Public pages confirm a 48-hour, indoor, GPS-denied autonomy event on Nov 6-8, 2026, with identical kits: a 7-inch carbon-fibre quad built by the host's student UAV group, a Pixhawk 6C (PX4), an Orin Nano and an OAK-D S2. Judging is 65% course run and 35% technical pitch, and the course covers "obstacles, walls, hoops, mapping, and localization", with a separate Fastest Run prize. No public rulebook, course dimensions, netting description, time limits or collision penalties were found, and no prior edition was identified.

### Cited Findings
- Dates are Nov 6-8, 2026 at the host university, presented with its robotics center. Applications for 2026 are closed. The site claims 100+ students, 48 hours and $15K in prizes. Teams receive "identical autonomous drone kits and compete to build the best software stack for GPS-denied flight", and build algorithms for "perception, navigation, obstacle avoidance, and autonomous decision-making". — competition website
- The kit, as listed on the official site, is a 7" drone, Pixhawk 6C, Luxonis depth cameras and NVIDIA Orin Nanos. Mentors come from Google Wing, Skydio "and more". Travel is funded for accepted participants, priority admission goes to the host university's students, and the event is open to university students across North America. — competition website
- Prizes and scoring: Grand Prize $7,000 for the "highest combined score across the course and the technical pitch"; Runner-Up $3,000; Third $2,000; Most Technologically Impressive $2,000, judged on the pitch alone (autonomy stack, novelty, engineering); Fastest Run $1,000, "for the single fastest run through the course". "Scoring is split 65% course run and 35% technical pitch." The course run covers "obstacles, walls, hoops, mapping, and localization". The pitch is 5-7 minutes. — competition prizes page
- The announcement blog post (known from search snippets, since the page refused connections when fetched) says teams spend 48 hours building systems "capable of navigating a GPS-denied indoor environment". Each drone must "navigate a complex indoor obstacle course, perform real-time mapping of an unknown environment, and execute fully autonomous flight without external positioning infrastructure". The main competition is hosted indoors on the second floor of a student-union building. The host's student UAV group designs and builds the kit: a custom carbon-fibre quadcopter frame, a Luxonis OAK-D S2, an NVIDIA Jetson Orin Nano and a PX4-compatible flight controller. Dates were originally May 22-24, 2026 and were postponed to Nov 6-8. The post cites a "$20k+ prize pool". — competition announcement post
- The FAQ questions are visible but the answers are collapsed: "Is travel funded?", "Who should apply?", "Do I need a team?", "What should I bring?", and a Code of Conduct question. — competition website
- Source conflicts: the prize pool is $15K on the official site but "$20k+" in the announcement post. Participant counts vary between 100+ and 150+ across pages (search summary). — competition website; competition announcement post

### Inferences
- Since "mapping" and "localization" are explicitly scored, a map or trajectory product, such as an occupancy grid or point cloud of the course plus the VIO trajectory, should be produced and shown, not just used internally. The 35% pitch rewards explaining the stack, so logging and visualisation tools do double duty.
- "Walls" and "hoops" suggest large planar obstacles and fly-through rings, comparable to the IMAV windows and bars. Hoops are thin rings, likely only partly captured by stereo depth, so detection may need the mono or RGB image (shape or colour), as in IMAV and ADR gate detection.
- An "unknown environment" for mapping hints that the full layout may not be given in advance. The UZH-style "measured start pose + known gross geometry" approach may only partly apply. Teams should be ready both to measure the course on site and to rely on depth-based reactive avoidance.
- The venue is a student-union building, not a dedicated flight lab. Expect crowded Wi-Fi, mixed lighting and possibly plain walls or floors (see Q4 mitigations).

### Gaps
- No public rulebook, course map, arena dimensions, netting description, per-run time limit, number of attempts, collision or net-touch penalties, manual-takeover rules, or definition of how mapping and localization are scored was found.
- The announcement post could not be fetched directly (connection refused), so its details come from search-engine summaries and should be re-checked.
- No prior edition was found. This appears to be the first edition, but that is not confirmed.
- Whether the flight area is enclosed by safety netting (as the platform brief states) is not confirmed by any public page I found.
