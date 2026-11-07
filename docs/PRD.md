# PRD: Stanford DroneHacks 2026 Autonomous Flight Stack (Team-08)

## Status and evidence

This is a **docs-only** design proposal, not implemented flight software or
permission to deploy. `contracts/` remains authoritative; this PR changes no
contract, PX4 allowlist, organizer interface, safety control or hardware.
Decision [#18](https://github.com/moterodiaz/DroneHacks/issues/18) authorizes
documentation corrections only. Simulator results below are historical design
inputs, not demonstrated performance on the issued kit.

Kit behavior is checked against the read-only snapshot under
`drone-snapshot/home/dronehacks/skybeta/`: `README.md` (§Flight sequence and
§Frames), `docs/competitor_guide.md`,
`src/skybeta_flight_interface/skybeta_flight_interface/{flight_interface,frames}.py`
and `src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py`.
The inspected `flight_interface.py` Git blob is
`e8340178b0438e4700c74e44cb2ce3f1e32a472f`.
The fleet README names v1.16.2 while older simulator/reference notes use
v1.16.1; verify actual firmware and configuration before relying on either.

**Blocked features:** upstream vision Guard insertion, dynamic covariance,
vision-loss tunnel traversal, autonomous landing/disarm/rearm and return after
disarm. Each needs organizer-supported interfaces, reviewed contract scope and
kit measurements before implementation or flight. No safety guarantee follows
from this document's thresholds, state machines or simulator anecdotes.

## 0. Architectural Rationale & Literature Adaptations (MonoRace / SkyDreamer / VIO)

### 0.1 Why Direct-to-Motor RL Control is Rejected for Sky Beta
While end-to-end neural motor policies ($u_1, u_2, u_3, u_4$) in **MonoRace** and **SkyDreamer** eliminate inner-loop PID lag and actuator saturation during $6g\text{--}7g$ racing maneuvers at $75\text{--}100\text{ km/h}$, mapping motor commands straight from the Jetson on the **Sky Beta** kit is counterproductive for four reasons:
1. **Starter Interface & Rule 8.5 Constraints:** The kit uses `skybeta_flight_interface`, and the organisers' guide states: *"Do not publish to `/fmu/in/*` yourself."* Furthermore, **Rule 8.5** strictly prohibits modifying the organisers' PX4 safety settings, failsafes, or limits. Bypassing PX4's attitude/rate controllers to send raw `ActuatorMotors` commands breaks the `COM_OF_LOSS_T` ($1\text{ s}$ Offboard loss) fallback to Position mode.
2. **Control Loop Rate & Latency Mismatch:** MonoRace runs `G&CNet` at $500\text{ Hz}$ directly on the STM32H743 microcontroller, and SkyDreamer runs at $90\text{ Hz}$ on the Jetson coupled with a $2000\text{ Hz}$ gyro stream and bidirectional DShot motor RPM feedback ($\omega_1..\omega_4$). On the Sky Beta kit, stereo runs at $30\text{ fps}$, the IMU at $200\text{ Hz}$, and commands cross a `uXRCE-DDS` bridge with a **$68.7\text{ ms}$ median latency**. Stabilizing raw motor thrusts over a $\sim 69\text{ ms}$ delayed link without RPM feedback causes immediate oscillation.
3. **The $1.0\text{ m/s}$ Speed Cap:** PX4 on the kit is capped at `MPC_XY_VEL_MAX = 1.0` $\text{m/s}$. At $1.0\text{ m/s}$, aerodynamic drag and non-linear rotor wash are negligible.
4. **Trajectory Tracking Through the Supplied Interface:** The previous fixed-gain 50 Hz led-pose proposal is withdrawn. The kit accepts both pose and twist, emits setpoints at 10 Hz by default, and must not be patched. Simulator lag measurements do not establish zero-lag kit behavior; cadence and tracking accuracy need props-off and approved flight validation.

### 0.2 Key Techniques Adapted from MonoRace, SkyDreamer & VIO Standards
1. **Health State Gating (`odom_guard`, blocked proposal):** VIO health enums (`HEALTHY`, `DEGRADED`, `LOST`) motivate detecting confident-but-wrong blank-view poses. The current kit bridge consumes cuVSLAM directly; no Guard interception or dynamic covariance is authorized by this PR. IMU/rangefinder coasting does not bound horizontal drift.
2. **Re-Anchoring (simulation proposal):** Withholding blank frames and resuming raw caused a $2.2\text{ m}$ jump ($149\text{ cm}$ RMS error) in simulation. Matching recovery to a drifting PX4 estimate may hide a discontinuity without recovering true course position. It is not evidence of safe kit recovery; see §3.3.
3. **Landmark-Based Map Anchoring (2D Depth Slice ICP vs. GateNet PnP):** Instead of U-Net gate segmentation masks, `map_corrector` slices the OAK-D's stereo depth at $0.80\text{ m}$ flight height and uses an Iterated Kalman Filter + ICP to align observed walls and the C2a Window against the preloaded v3 CAD map (cutting drift from $8\text{ cm}$ to $2\text{ cm}$ while staying 100% compliant with Rule 6.4).
4. **Adaptive Frustum Culling:** Inspired by MonoRace's state-based ROI cropping, `map_corrector` uses the current EKF pose to frustum-cull depth points outside the $6.096 \times 6.096\text{ m}$ net boundary so background room objects seen through the mesh net never corrupt wall matching.
5. **Active Perception Look-Ahead Gaze:** Facing a maze end-wall at $0.3\text{ m}$ starves `cuVSLAM` (withholding 390 of 543 frames). `trajectory_follower` points the nose $0.80\text{ m}$ ahead along the path so the camera rotates into the next corridor before reaching the corner.
6. **Proposed Competition Profiles (`bank` vs. `attack`):** `bank` routes via Gate B; `attack` proposes the Window bonus. These CLI profiles are not implemented and neither guarantees a finish. Autonomous return after landing/disarm is separately blocked, not enabled by selecting a profile.

---

## 1. Project Overview & Strategic Rules
**Objective:** Build a fully autonomous, GPS-denied onboard software stack for the issued **Sky Beta** quadcopter to navigate an indoor obstacle course (First Corridor $\rightarrow$ Maze $\rightarrow$ Dark Tunnel $\rightarrow$ Chicane/Window $\rightarrow$ Landing Pad $\rightarrow$ Return Flight to Launch) within a 3-minute limit after a single start command.

This is the eventual scoring objective, not current capability. The organizer
safety pilot arms on RC in Altitude, then switches to Position on the ground.
With `auto_offboard_on_arm:=true` and readiness checks satisfied, the supplied
interface enters Offboard once per arming. Team software never arms the drone;
pilot takeover prevents automatic re-entry until disarm/rearm.

### Strategic North Star
1. **Finish First, Speed Second:** Every finished run flown below the walls ($1.727\text{ m}$ / $68\text{ in}$) scores strictly higher than every unfinished run. Top speed is capped at $1.0\text{ m/s}$ (`MPC_XY_VEL_MAX = 1.0`), meaning cornering smoothness and trajectory tracking lag—not top speed—determine run time.
2. **Surgical Code Footprint:** Use the kit's provided software (`depthai-ros`, Isaac ROS `cuVSLAM` stereo-only, `px4_odom_bridge.py`, and PX4 EKF2). Proposed custom nodes require independent review and kit validation; no hardware benchmark is supplied by this PR:
   - **`odom_guard` (blocked)**: Research health detection and recovery offline; do not insert it into the organizer's vision path.
   - **`course_planner`**: Generates collision-free paths from the preloaded v3 CAD map (and live wall-offset updates) using A* with Nav2 inflation costs and line-of-sight shortcutting. Zero hard-coded waypoints (protects the 15-pt Generality score and passes Rule 9.5 AI code review).
   - **`trajectory_follower` (planned)**: Arc-blended, jerk-limited trajectories with configurable look-ahead yaw, using the supplied pose/twist interface without a rate upgrade or fixed-gain lead requirement.
   - **`map_corrector` (Stretch / Precision Multiplier):** Slices stereo depth at flight height, frustum-culls points outside the net, and runs Iterated Kalman Filter + ICP wall-matching to eliminate VIO drift and estimate physical wall-placement offsets.
3. **Strict Rule Compliance:**
   - **Rule 5.1:** Fully autonomous after ONE start command. Laptop is read-only during runs.
   - **Rule 6.4 (Maps Allowed, Run Data Banned):** Preloading the published v3 CAD map (and tape-measure survey corrections of the physical course) is 100% legal. Saving SLAM maps, point clouds, or Gaussian splats from one flight to feed into a subsequent flight is strictly prohibited. All live wall-offset estimates must reset between runs. Offline parameter tuning (e.g., camera extrinsics, EKF noise, PID lead gains) from flight logs is legal.
   - **Rule 8.5:** NEVER modify the organisers' PX4 safety parameters (`COM_OF_LOSS_T`, `COM_POSCTL_NAVL`, `NAV_RCL_ACT`, `COM_RC_OVERRIDE`, `COM_FAIL_ACT_T`, `COM_KILL_DISARM`, geofences, or battery limits).
   - **Rule 9.5:** AI code review screens the repository for hard-coded coordinates, timings, or routes. All goals (tunnel axis, window center, landing pad) must be parsed programmatically from the map file.

---

## 2. Hardware, Firmware & Environment Specifications

### 2.1 The "Sky Beta" Kit (`SN0001` Reference)
- **Airframe:** Carbon quadcopter, $4\times$ DYS X2807 $1500\text{ KV}$ motors, 7-inch props, 4S $5000\text{ mAh}$ LiPo, mass $\sim 1.25\text{ kg}$.
- **Dimensions:** Footprint $0.37 \times 0.48\text{ m}$; propeller reach $0.28\text{ m}$ radius from center. Resting body height on launch mark: $\sim 0.07\text{ m}$ (verify with tape measure on Day 1).
- **Companion Computer:** NVIDIA Jetson Orin Nano 8 GB (Ubuntu / JetPack 6, ROS 2 Humble).
  - *Note:* The hosted **FlyX Lab** cloud simulator runs ROS 2 **Jazzy**, `cuVSLAM` 17, $640\times 400$ stereo @ $15\text{ Hz}$, and lacks OpenCV. Code must remain compatible with both Humble and Jazzy.
- **Flight Controller:** Holybro Pixhawk 6C running **PX4 v1.16.1**, communicating via `uXRCE-DDS` (`ROS_DOMAIN_ID = 1`, matching `UXRCE_DDS_DOM_ID = 1`).
- **Primary Vision Sensor:** Luxonis **OAK-D S2** mounted forward and level, $\sim 0.11\text{ m}$ ahead of center.
  - Stereo pair: $2\times$ 1 MP global-shutter mono cameras, $75\text{ mm}$ baseline, $1024\times 640$ @ $30\text{ fps}$ via `depthai-ros`.
  - Internal IMU: $200\text{ Hz}$.
- **Height Sensor:** Holybro **VL53L1X** downward Time-of-Flight (ToF) rangefinder (up to $4\text{ m}$ range, $27^\circ$ FOV).

### 2.2 Course Geometry & Coordinate Frames
- **World Map Frame (Course CAD v3):** Origin $(0.0, 0.0)$ at floor center; $+X$ East, $+Y$ North, $+Z$ Up (ENU).
  - **Flight Net:** $6.096 \times 6.096\text{ m}$ ($20\times 20\text{ ft}$), $3.048\text{ m}$ ($10\text{ ft}$) tall. Bounds: $x, y \in [-3.048, +3.048]$.
  - **Walls:** $1.727\text{ m}$ ($68\text{ in}$) tall, $0.102\text{ m}$ ($4\text{ in}$) thick. Drone must keep some part of airframe below $1.727\text{ m}$ at all times (nominal flight altitude: **$0.80\text{ m}$ above floor** $\rightarrow z_{\text{setpoint}} = 0.80 - z_{\text{rest}} \approx 0.73\text{ m}$).
  - **Corridors:** $1.02\text{--}1.07\text{ m}$ wide; narrowest bottom passage is $0.93\text{ m}$ wide. With $0.28\text{ m}$ prop reach, total lateral error budget in the narrowest passage is **$\pm 0.18\text{ m}$**.
  - **Launch Mark:** $(-2.515, +0.914)\text{ m}$, facing **North ($+Y$)** up the first corridor.
  - **Dark Tunnel (Checkpoint 3 - 10 pts):** Oval tube ($58 \times 72 \times 60\text{ in}$, $\sim 1.5\text{ m}$ long) against the East net. Dark interior; lit exit spans $52^\circ$ of the $80^\circ$ FOV from entrance.
  - **Window in Wall C2a (Checkpoint 4 Bonus - +5 pts):** $0.914 \times 0.914\text{ m}$ opening, $0.305\text{ m}$ to $1.219\text{ m}$ above floor. Center height is $0.762\text{ m}$ (matches our $0.80\text{ m}$ flight altitude).
  - **Boulders ($\times 5$):** $0.457\text{ m}$ diameter, $0.533\text{ m}$ tall cylinders in the southern section. *Hazard:* If flown over, the downward VL53L1X rangefinder reads them as a $+0.53\text{ m}$ floor step, causing EKF2 to spike altitude.
  - **Landing Pad (Checkpoint 5 - 7 pts):** $1.524\text{ m}$ ($5\text{ ft}$) outer ring diameter, centered at $(-2.042, -0.792)\text{ m}$.
- **Local Tracker / ROS Frame (ENU / FLU):**
  - `odom` is a startup-relative **world** frame with ENU labels, not geographic alignment. Body `base_link` is FLU (forward/left/up), with its origin at the Pixhawk primary IMU. Startup facing north is not sufficient to identify odom axes; arming does not reset them.
  - Measure a rigid course-to-odom transform at each tracking initialization using known independent course displacements and a level reference. All coordinates and heights come from the surveyed map and measured IMU origin, not literal launch offsets in code.
  - **Illustrative transform (symbols supplied by a future map/calibration loader, not runnable mission code):**
    ```python
    p_odom = o_odom + R_odom_course @ (p_course - o_course)
    ```
  - `o_course` is the surveyed IMU reference; `o_odom` is its simultaneous odometry reading. Transform heading directions by the same rotation. The example $0.80-0.07=0.73\text{ m}$ displacement is valid only if the kit's IMU reference height and vertical alignment are actually measured to match it.
- **PX4 World / Body Frames:**
  - World ENU→NED is `(x,y,z) -> (y,x,-z)`; body FLU→FRD is `(x,y,z) -> (x,-y,-z)`. NED north is not body forward.
  - PX4's EKF has its own origin/yaw. The organizer bridge declares `POSE_FRAME_FRD` (world-fixed arbitrary heading), not north-referenced vision. The interface defaults to `anchor_setpoints=False`, `odom_to_ned_yaw_offset_rad=0.0`; plain permutation alone does not align odom to EKF2. Measure residual yaw and simultaneous origins, verify any organizer-permitted anchoring configuration, and block flight if it cannot represent the measured transform. Never patch the interface.
  - Never publish raw `/fmu/in/*` from mission nodes. PX4 timestamps use microseconds; the kit owns their time-domain handling.

---

## 3. System Architecture & Node Specifications

```text
Current organizer vision path:
cuVSLAM -> organizer px4_odom_bridge -> PX4 EKF2
    |
    +-> /visual_slam/tracking/odometry -> planned team nodes (read-only input)

Planned competitor command path:
surveyed map -> course_planner -> trajectory_follower
                                      |
                       /skybeta/setpoint/{pose,twist}
                                      |
                       unmodified skybeta_flight_interface -> PX4

odom_guard and map_corrector: offline research only until separately authorized.
```

### 3.1 Supplied Flight Interface (No Patch or Rate Upgrade)

The organizer README forbids modifying `skybeta_flight_interface` or publishing
to `/fmu/in/*`. The former simulator two-line patch and 50 Hz upgrade are
withdrawn requirements. Do not gate its heartbeat/setpoint publisher from team code.

The supplied interface accepts `geometry_msgs/PoseStamped` on
`/skybeta/setpoint/pose` and world-ENU `geometry_msgs/TwistStamped` on
`/skybeta/setpoint/twist`, not competitor acceleration feed-forward.
Defaults: `setpoint_rate_hz=10.0` (10 Hz), `setpoint_timeout_s=1.0`,
`min_altitude_m=0.3`, `max_altitude_m=3.0`, twist clamp `max_speed_mps=1.5`.
The PX4 1.0 m/s cap still applies; the interface has no horizontal geofence.
Publish fresh inputs with margin inside the timeout and before pilot arming.
The organizer minimum of once per second at startup is not a robust ongoing
cadence at the 1 s timeout boundary. Verify actual launch settings with props off.

### 3.2 PX4 Parameter & Pre-Flight Calibration Protocol
- **Backup First:** Dump all parameters via QGroundControl (`SN0001_backup.params`) before changing anything.
- **Estimator candidates, not an instruction to write parameters:** Only an independently approved `contracts/px4.md` allowlist may authorize writes. That contract is proposed in PR #5 and is not present at this PR's base. Until it lands and kit validation is complete, the following historical candidates are blocked. Do NOT touch safety/failsafe parameters:
  - `EKF2_EVA_NOISE = 0.01` (PX4 v1.16.1 bug uses this parameter unsquared, making default vision yaw weight $5\times$ weaker than v1.18).
  - `EKF2_EV_QMIN = 0` (Starter bridge emits `quality = 0`; any `QMIN > 0` rejects 100% of vision messages).
  - `EKF2_EV_DELAY = 0` (simulator result only; 4 of 5 wall strikes with 70 ms in that experiment do not establish kit latency. Timestamp at capture does not remove transport/processing delay. Measure before relying on a value).
- **Camera Exposure & Timestamping (`depthai-ros`):**
  - Cap auto-exposure maximum shutter time to **$8\text{--}10\text{ ms}$** and meter on the central ROI so the dark tunnel walls do not over-expose the lit exit and motion blur stays $4\times$ lower.
  - Enable `i_add_exposure_offset: true` to timestamp frames at mid-exposure.
- **Preflight Diagnostics (not authorization to arm):**
  - Follow the organizer instruction to leave the drone still for the **first minute** with vision tracking. This PR does not implement an arming gate, and `odom_guard` is not running in the shipped path.
  - Estimate any static camera pitch/roll mount offset ($\theta_{\text{cam}}$) during settle.
  - Block mission start until:
    1. The **magnitude** of `estimator_sensor_bias.gyro_bias[2]` is $< 0.001\text{ rad/s}$ and steady, not its signed value. The steady-state observation window and acceptance criteria require validation.
    2. The wrapped heading difference is $<2.0^\circ$ **after transforming both headings to the same world frame**, not raw subtraction across odom and PX4 frames.

Illustrative diagnostic arithmetic, preserving the proposed thresholds without
claiming they are sufficient for safety. Missing/non-finite telemetry fails closed:

```python
import math

def preflight_diagnostics(yaw_bias_rad_s, px4_yaw_rad, aligned_vision_yaw_rad):
    values = (yaw_bias_rad_s, px4_yaw_rad, aligned_vision_yaw_rad)
    if not all(math.isfinite(value) for value in values):
        return False
    delta = px4_yaw_rad - aligned_vision_yaw_rad
    yaw_error = math.atan2(math.sin(delta), math.cos(delta))
    return abs(yaw_bias_rad_s) < 0.001 and abs(yaw_error) < math.radians(2.0)
```

This only checks two instantaneous diagnostics. Alignment, settling, tracking
health and the safety pilot's readiness procedure remain separate requirements.

---

### 3.3 Node 1: `odom_guard` (Vision Fault Protection & Re-Anchoring)
**Status:** Guard insertion is **blocked**. This section is an offline research
proposal, not instructions to remap the organizer bridge or publish competing
vision. Inputs/outputs and health enums below are proposed, not existing APIs.
**Purpose:** `cuVSLAM` fails silently on textureless maze walls and inside the dark tunnel—reporting hundreds of features and a stationary pose with tiny covariance while the drone is actually moving. Passing this to PX4 causes runaway acceleration into walls.

**Inputs:**
- `/visual_slam/tracking/odometry` (`nav_msgs/Odometry` from `cuVSLAM`)
- `/oak/left/image_rect_raw` (or stereo image for fast texture/gradient check)
- `/fmu/out/vehicle_local_position` & `/fmu/out/vehicle_attitude` (Best-Effort QoS)
- `/fmu/out/sensor_combined` (for gyroscope angular velocity $\Omega_{\text{imu}}$)

**Outputs:**
- `/guarded/odometry` (`nav_msgs/Odometry` fed into `px4_odom_bridge.py`)
- `/guard/health_state` (`uint8`: `0 = HEALTHY`, `1 = DEGRADED`, `2 = LOST`)

**Core Logic & Mathematical Gating:**
1. **Candidate Offline Frame Checks (not safety limits):**
   - The historical 10 m/s and 720°/s thresholds below reject gross outliers only; they allow errors far beyond the ±0.18 m clearance budget. No replacement flight thresholds are selected here. Derive and validate any future limits from measured estimator/control error and clearance, with a reviewed abort policy.
   - **Texture Starvation:** Compute fast image gradient / edge density (e.g., Sobel or Laplacian variance on downsampled stride-4 image). If textured pixel ratio $< 1.0\%$, withhold frame.
   - **Velocity Spike:** If implied linear speed $\|\Delta \mathbf{p} / \Delta t\| > 10.0\text{ m/s}$ or angular speed $\|\Delta \mathbf{R} / \Delta t\| > 720^\circ\text{/s}$ ($12.56\text{ rad/s}$), withhold frame.
   - **Gyro Disagreement (MonoRace Innovation Gate):** Compare frame-to-frame visual rotation rate $\Omega_{\text{vis}}$ against low-pass filtered IMU gyro rate $\Omega_{\text{imu}}$:
     $$\|\Omega_{\text{vis}} - \Omega_{\text{imu}}\|_2 > \tau_{\text{gyro}} \quad (\text{default } \tau_{\text{gyro}} = 0.8\text{ rad/s})$$
   - **Mahalanobis Position Gate:** Reject if visual innovation against PX4's current prediction exceeds $4\sigma$ of the expected uncertainty envelope.
2. **State Machine & Seamless Re-Anchoring (`reset_counter` logic):**
   - Maintain a rigid 6-DoF anchor transform $\mathbf{T}_{\text{anchor}} = (\mathbf{R}_{\text{anchor}}, \mathbf{t}_{\text{anchor}})$ mapping raw `cuVSLAM` poses into the continuous guarded frame:
     $$\mathbf{T}_{\text{guarded}}(t) = \mathbf{T}_{\text{anchor}} \cdot \mathbf{T}_{\text{cuvslam}}(t)$$
   - While `LOST` in an offline experiment: withhold the proposed guarded stream. The shipped bridge is unaffected. IMU/rangefinder coasting has no demonstrated horizontal-clearance guarantee. **Flight traversal during vision loss remains blocked** until a measured maximum loss duration and worst-case drift/recovery error fit the course budget, including latency and control error, with an organizer/pilot-approved abort procedure. Thirty lost frames is a historical reset proposal, not a validated safe duration or permission to reset tracking in flight.
   - On transition `LOST -> HEALTHY` (requires $K = 3$ consecutive valid frames): Recompute $\mathbf{T}_{\text{anchor}}$ so the newly recovered visual pose seamlessly equals PX4's current dead-reckoned pose (`vehicle_local_position` / `vehicle_attitude`):
     $$\mathbf{T}_{\text{anchor}} \leftarrow \mathbf{T}_{\text{px4}}(t_{\text{recover}}) \cdot \mathbf{T}_{\text{cuvslam}}(t_{\text{recover}})^{-1}$$
     Both transforms must use the same world/body conventions and synchronized timestamps. Re-anchoring to PX4 can preserve numerical continuity while retaining its drift; it does not restore course truth or prove a safe recovery. Reset-counter ownership and frame-reset semantics must be reviewed against the organizer bridge before any implementation.
3. **Dynamic Covariance Injection ($\mathbf{R}_k$):**
   - **Blocked:** Before relying on message variances, pin the actual PX4 version and inspect `EKF2_EV_NOISE_MD` semantics, selected mode, noise floors and how pose/yaw variances reach EKF2. The PR has not established that inflated message covariance changes fusion weighting. This document does not authorize writing `EKF2_EV_NOISE_MD` or expanding an allowlist; dynamic covariance remains blocked until that evidence and organizer permission exist.
   - Historical offline candidate variances (not calibrated kit uncertainty or active fusion settings):
     - `HEALTHY` (texture $> 5\%$, $\|\Omega\| < 1.0\text{ rad/s}$): $\sigma_{\text{pos}}^2 = (0.02\text{ m})^2$, $\sigma_{\text{yaw}}^2 = (0.02\text{ rad})^2$.
     - `DEGRADED` (texture $1\%\text{--}5\%$ or high yaw rate): Proposed variance multiplier $\alpha \in [2.0, 10.0]$ based on inverse texture ratio (inspired by MonoRace's $\sigma^2 \propto 1/N_c^2$). Any actual effect on EKF2 weighting remains unverified and blocked as above.

---

### 3.4 Node 2: `course_planner` (Map-Driven A* & Topological Shortcutting)
**Purpose:** Parse the course geometry file (`v3` CAD mesh / JSON layout) to generate optimal waypoints without hard-coding coordinates.

All geometry, goal positions, flight height and route selection must be parsed
from the authoritative map/schema once approved. Numeric examples below explain
the published layout; they are not constants to paste into an implementation.
Algorithm/timing/clearance settings need reviewed configuration, not new files
asserted to exist by this PR. Missing map semantics must fail closed.

**Algorithm Steps:**
1. **2.5D Horizontal Slice & Obstacle Inflation:**
   - Slice the 3D course model at flight altitude $z = 0.80\text{ m}$ to extract all wall segments and outer net boundaries ($6.096 \times 6.096\text{ m}$).
   - Project the 5 southern boulders ($r = 0.2285\text{ m}$) onto the 2D grid even though they are $0.533\text{ m}$ tall (below flight height). Expand boulder radii by the drone's lateral tolerance + VL53L1X downward cone footprint ($r_{\text{total}} = 0.2285 + 0.19 + 0.10\text{ m}$) so the rangefinder never pings a boulder top.
2. **Nav2 Costmap Discretization ($5\text{ cm}$ Grid):**
   - For the example layout this is about $122 \times 122$ cells at $0.05\text{ m}$ resolution; derive grid dimensions from map bounds and configured resolution rather than fixing cell counts.
   - Compute Euclidean distance $d(c)$ from every cell $c$ to the nearest wall/net/boulder segment:
     - **Lethal / Inscribed Zone ($d \le 0.32\text{ m}$):** Impassable (`cost = INF`). Accounts for $0.28\text{ m}$ prop reach + $0.04\text{ m}$ safety margin. *(Note: For the $0.914\text{ m}$ Window passage, temporarily relax inscribed radius to $0.29\text{ m}$ along the window axis).*
     - **Exponential Inflation Decay ($0.32\text{ m} < d \le 0.80\text{ m}$):**
       $$\text{Cost}(d) = C_{\text{max}} \cdot \exp\left(-\lambda \cdot (d - r_{\text{inscribed}})\right)$$
       Forces the path directly down the centerlines of $0.93\text{ m}\text{--}1.07\text{ m}$ corridors.
3. **Sequential Checkpoint Goal Extraction (Programmatic from Map):**
   - Parse semantic regions directly from map geometry:
     1. `Start`: Launch mark $(-2.515, +0.914)$.
     2. `Goal 1`: First corridor north exit.
     3. `Goal 2`: Maze exit (Gate A).
     4. `Goal 3`: Dark Tunnel entry $\rightarrow$ centerline axis $\rightarrow$ South exit (strictly colinear along tunnel axis so no yaw turn occurs inside).
     5. `Goal 4`: Window center in wall C2a (`--profile=attack`) OR Gate B center (`--profile=bank`).
     6. `Goal 5`: Landing Pad center $(-2.042, -0.792)$.
4. **A* Search + Line-of-Sight Shortcutting:**
   - Run 8-connected (or Theta*) A* across sequential goals.
   - Prune redundant collinear/staircase grid nodes: replace subpath $(p_i \dots p_j)$ with segment $\overline{p_i p_j}$ iff every cell intersected by $\overline{p_i p_j}$ has a cost no greater than the maximum cost along the original grid subpath. Produces $\sim 19$ clean legs ($\sim 17\text{ m}$ total distance).
   - Support dynamic re-planning during flight if `map_corrector` updates wall segment offsets.

---

### 3.5 Node 3: `trajectory_follower` (Supplied Pose/Twist & Active Perception Gaze)
**Purpose:** Plan smooth trajectories and configurable camera look-ahead using
the unmodified kit interface (§3.1). No zero-lag or corner-clearance guarantee
is supplied; swept-volume and tracking-error validation are required.

**Core Logic:**
1. **Corner Arc Blending (Filleting):**
   - At each interior waypoint $\mathbf{W}_i$ where segment direction changes by angle $\theta_i$, insert a circular/clothoid fillet of radius $R_{\text{blend}} \in [0.20, 0.35]\text{ m}$ (clamped so the arc never violates the $0.32\text{ m}$ wall clearance).
2. **Jerk-Limited Velocity Profile:**
   - Compute 1D arc-length parameterization $s(t)$ along the blended path using PX4's configured kinematic limits:
     - $v_{\text{max}} = 1.0\text{ m/s}$ ($0.75\text{ m/s}$ at the first maze corner in `bank` profile to prevent PX4 v1.16.1 attitude overshoot).
     - Centripetal cornering limit: $v_{\text{corner}} = \sqrt{a_{\text{lat,max}} \cdot R_{\text{blend}}}$.
3. **Active Perception Look-Ahead Yaw (Anti-Starvation Gaze):**
   - **Never** point yaw tangent to current velocity when approaching a $90^\circ$ wall corner (facing a wall at $0.3\text{ m}$ starves `cuVSLAM`, dropping 390 of 543 frames).
   - Compute desired heading $\psi_{\text{ref}}(t)$ by pointing at a look-ahead point $s(t) + L_{\text{lookahead}}$ where **$L_{\text{lookahead}} = 0.80\text{ m}$**:
     $$\psi_{\text{ref}}(t) = \text{atan2}\left(y(s + 0.80) - y(s), \, x(s + 0.80) - x(s)\right)$$
   - **Tunnel Exception:** Inside the Dark Tunnel (`Goal 3`), lock $\psi_{\text{ref}}$ strictly along the tunnel centerline axis toward the south exit until the drone completely clears the tube.
4. **Interface-Compatible Tracking:**
   - Choose pose or world-ENU twist through the supplied topics, with input freshness inside the configured timeout. The fixed-gain lead formula and rate upgrade are withdrawn, not an alternative implementation path.
   - Verify tracking errors across corners against the clearance budget with the actual kit/configuration. Simulator cross-track experiments are not permission to change PX4 gains.
5. **Landing and Return (`--enable-return`, not implemented):**
   - Landing/disarm/rearm is **blocked** through the current interface. It has no arm path and clamps altitude at `min_altitude_m=0.3`; a below-floor pose or negative twist is not a supported landing API. Do not bypass the clamp, enable SITL-only mode requests for flight, modify the interface or send raw PX4 commands.
   - Checkpoint 5 requires landing/disarm, so a complete scored finish is not yet supported by this design. Obtain an organizer-supported landing procedure and return/arming policy before specifying an executable state machine.
   - Any future return route must derive goals, barrier/net clearances, takeoff height and timing from the surveyed map and approved configuration, not the former 1.5 s/2.2 m hard-coded sequence.

---

### 3.6 Node 4: `map_corrector` (2D Depth Slice ICP & Layout Filter)
**Status:** Offline stretch proposal only; not a deployed authority for vision,
frame anchoring or map geometry. Kit depth-stream compatibility, uncertainty
and organizer-approved insertion points remain unverified. Numerical settings
below are experimental examples, not validated flight thresholds.
**Purpose:** Eliminate slow VIO drift ($\sim 8\text{ cm} \rightarrow \sim 2\text{ cm}$) and estimate if physical wall units were placed $5\text{--}10\text{ cm}$ off the published CAD file.

**Algorithm Steps:**
1. **Horizontal Depth Band Slicing & Net Frustum Culling:**
   - Take stereo depth from OAK-D S2, transform points inside the altitude band $z \in [0.65, 0.95]\text{ m}$ into the 2D horizontal plane using current roll/pitch from EKF2 (compensated for static mount tilt $\theta_{\text{cam}}$).
   - **Spatial Trust Gating:** Discard depth points beyond $3.5\text{ m}$, points with sparse neighbor density, and any point whose world projection falls within $0.15\text{ m}$ of or outside the $6.096\text{ m}$ mesh net boundary (prevents matching background room furniture seen through the net).
2. **2D Point-to-Line ICP / Iterated Kalman Filter:**
   - Match filtered 2D scan points against the v3 CAD wall line segments.
   - Apply a low-gain correction to $\mathbf{T}_{\text{anchor}}$ (bounded to max $2\text{ cm/s}$ drift correction so PX4 EKF2 never sees a velocity spike).
   - Maintain a per-wall-unit offset state $\Delta \mathbf{w}_k$ (like AlphaPilot gate offset estimation). If a wall unit is consistently observed shifted by $> 4\text{ cm}$, trigger `course_planner` to shift that obstacle segment and re-shortcut the remaining route. Reset all offsets on node restart (Rule 6.4).

---

## 4. Repository Structure

The following is a **proposed layout, not existing files** or launchable software.
Do not create a second source of contract constants; use approved `contracts/`
and its map schema. The interface remains the organizer's unmodified package.

```text
dronehacks_2026/
├── docs/PRD.md                     # This design document
├── README.md                       # Setup, run instructions, and open-source credits (Rule 9.1/9.4)
├── contracts/
│   └── <approved map/schema>       # Future authoritative geometry; no duplicate constants file
├── launch/
│   ├── autonomy_run.launch.py      # Planned competitor planner/follower; no blocked vision insertion
│   └── bench_check.launch.py       # Props-off verification launch
├── src/
│   ├── odom_guard/                 # Blocked offline research; not in organizer vision path
│   ├── course_planner/             # Node 2: 2.5D slice, Nav2 costmap, A*, line-of-sight shortcut
│   ├── trajectory_follower/        # Planned supplied pose/twist consumer; no land/rearm API
│   └── map_corrector/              # Node 4: Depth band slicer, net culling, ICP wall matcher
└── scripts/
    ├── day1_kit_checks.sh          # Automated CLI check for versions, DDS, QoS, clock, biases
    ├── survey_adjust_map.py        # CLI tool to input tape-measured wall offsets into layout JSON
    └── offline_iou_calibrator.py   # Post-flight log optimizer for camera pitch/roll extrinsics
```

---

## 5. Execution Timeline & Verification Checklist

### Phase 1: Bench & Kit Unblocking (Friday 7:00 PM – Midnight)
- [ ] **Save Parameters:** Backup factory PX4 params via QGC (`SN0001_factory.params`).
- [ ] **Estimator Review:** Verify actual firmware and measured latency; do not write estimator candidates until an approved contract allowlist and kit validation authorize them. Record `EKF2_EV_NOISE_MD` read-only when assessing covariance semantics.
- [ ] **Fast DDS Shared Memory Fix:** Configure XML profile with $> 4\text{ MB}$ shared memory segment so $1\text{ MB}$ stereo frames never drop under Best-Effort QoS.
- [ ] **Props-Off Carry Test (Frame Sign Check):**
  - With props off, record simultaneous odom/PX4/IMU references and known independent course east/north/up displacements. Measure `R_odom_course` and odom→PX4 residual yaw/translation, not assumed axis signs from startup heading.
  - Compare predicted odom coordinates and the interface-converted targets against actual readings within the approved survey/error tolerance. Repeat at a different startup heading and after tracker reset; invalidate old alignment on reset. No hardware checks are performed by this PR.
- [ ] **Vision-Loss Acceptance (blocked):** Design an organizer-approved props-off/offline loss/recovery experiment that records worst-case drift, duration, resets and recovery error, rather than asserting zero jumps. A safe flight-loss bound and abort procedure are prerequisites to tunnel flight; do not modify the running organizer bridge to run a test.
- [ ] **Physical Course Survey:** Record physical geometry in the approved map format once available. The named map/survey tooling in §4 is proposed, not an existing command to run.

### Phase 2: Incremental Flight Progression (Saturday 9:00 AM – 7:00 PM)
- [ ] **Step 1:** Organizer's first-minute stationary vision initialization, measured alignment and pilot readiness procedure; only then an organizer-approved manual test and latency/noise assessment. No parameter value is preselected by this checklist.
- [ ] **Step 2 (blocked pending approved landing procedure):** Rehearse safety-pilot arming and automatic Offboard takeover with props off and the unmodified interface. Any later takeoff/hover height must be map/calibration-derived. Plan a supported landing before flight; do not assume `HEIGHT_M = 0.73` fits this kit.
- [ ] **Step 3:** Straight $2.0\text{ m}$ leg up First Corridor (Checkpoint 1).
- [ ] **Step 4 (blocked pending loss bounds):** Maze/tunnel test only after §3.3 loss/drift/abort acceptance, approved exposure configuration and landing support. No deployed Guard is assumed.
- [ ] **Step 5 (bank):** Planned forward route **via Gate B** to the pad, not the Window. Full finish remains blocked by landing/disarm support and prior safety gates. Three zero-touch runs are a future acceptance target, not an observed result.
- [ ] **Step 6 (attack, blocked pending validation):** Window passage needs separate clearance/tracking evidence. `--enable-return` stays unavailable until the organizer-supported landing/rearm policy and map-derived return trajectory are reviewed.

### Phase 3: Scored Blocks & Pitch (Saturday 10:00 PM – Sunday 4:00 PM)
- [ ] **Scored Block 1 (conditional):** Only after all blocked prerequisites are resolved and independently reviewed, consider `bank` via Gate B. A banked finish does not by itself authorize attack or return, and no profile guarantees a score.
- [ ] **Sunday 12:30 PM Deadline:** Submit GitHub repo link, verify README credits (`skybeta` starter, NVIDIA Isaac ROS / `cuVSLAM`, Luxonis `depthai-ros`, PX4, ROS 2, Nav2 cost formulas, SciPy), and confirm zero hard-coded waypoints for Rule 9.5 check.