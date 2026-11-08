# DroneHacks 2026: Project Context and Rules

**Event:** Stanford DroneHacks, November 6-8, 2026. **Mission:** program the issued drone to navigate an indoor obstacle course autonomously, without GPS or pilot input. The course is inside the drone net in Tresidder Oak.

This standalone reference combines the confirmed challenge, hardware, scoring, logistics and constraints for humans and coding agents. It summarizes all 65 numbered rules by topic; parenthetical rule numbers refer to the official rulebook.

## 1. How to use this document

- **Official requirements** below come from the supplied official challenge and drone webpages, checked on **2026-11-06**. Their URLs and snapshot fingerprints appear at the end.
- The official rulebook takes precedence over conflicting project notes, old research and rules from other competitions. Announced organizer amendments can supersede this snapshot.
- **Missing information stays missing.** Do not turn silence into permission, a prohibition, a device specification or an organizer approval.
- **Engineering assumptions** are listed separately. They are neither official requirements nor verified properties of the issued drone.
- The requirements audit is settled; no individual approval questions remain. This document does not establish that any particular implementation complies.

## 2. Issued drone and software environment

Every team receives the same assembled, configured and flight-tested drone. The Jetson processes camera images/depth and flight state, plans motion and requests position or velocity targets. The Pixhawk makes the fast stabilizing corrections; the ESCs and motors produce thrust.

The following are **official webpage specifications**, not measurements of a particular issued unit.

| Component | Confirmed description |
|---|---|
| **NVIDIA Jetson Orin Nano 8GB** | Autonomy companion computer. 1024-core Ampere GPU, 32 tensor cores; 6-core 64-bit Arm Cortex-A78AE CPU; 8 GB 128-bit LPDDR5, 68 GB/s. 40 TOPS INT8; **67 TOPS in Super mode**. |
| **Holybro Pixhawk 6C** | PX4 flight controller for attitude, rates and safety envelope. STM32H743 Cortex-M7 at 480 MHz; 2 MB flash, 1 MB RAM; ICM-42688-P and BMI055 redundant IMUs; BMP388 barometer. |
| **Luxonis OAK-D S2** | Stereo depth and on-device inference. Sony IMX378 colour sensor, 12 MP / 4056 x 3040; two 1 MP global-shutter mono cameras; Myriad X VPU, 4 TOPS. Metric stereo depth **approximately 0.35 m and beyond**. |
| **Holybro ST VL53L1X** | Downward time-of-flight height rangefinder. **Up to 4 m**, less in bright light; **up to 50 Hz**; 27-degree field of view, narrower with programmable region of interest; 2.4 g. |

The documented programming environment is:

- Stock PX4, pre-flashed and tuned for the airframe; **exact version unspecified**.
- **ROS 2 Humble** on the Jetson, with PX4 messages and a bridged **uXRCE-DDS agent**.
- **DepthAI and depthai-ros**, providing RGB, depth and detections; depth is computed on the camera.
- A starter project with an Offboard control node, depth-to-obstacle example, launch files and Gazebo simulation target for the laptop and Jetson.
- A workflow of laptop simulation, ROS 2 nodes on the Jetson, and position/velocity setpoints to PX4 in Offboard mode with state feedback.

The drone page offers a CAD ZIP containing the assembled drone STEP and OAK-D mount STL, in **millimetres, +Z up**. Alternative middleware acceptance, exact wiring and installed package versions are not established by these descriptions.

## 3. Autonomy, localization and mapping

**One start command, then autonomous flight until completion or abort.** After start, external commands may only abort through kill or land. No piloting, joystick inputs, externally supplied waypoints, mode changes or parameter changes during a scored run. An RC transmitter, if used, is for the kill switch only. Other human input ends the run; earlier checkpoints still count. (rules 5.1-5.3, 7.11)

All perception, localization, planning and control run **on the drone**, using the Jetson, Pixhawk and declared auxiliary sensors. No laptop, server or cloud service may be in the flight loop. A connected laptop may watch telemetry, video and logs read-only; wireless links carry only start, abort and read-only monitoring. (rules 5.4-5.6)

- **No GNSS**, including GPS, GLONASS, Galileo or BeiDou, even as fallback. PX4 GNSS input remains disabled. (rule 6.1)
- No external positioning through motion capture, UWB, Wi-Fi/Bluetooth ranging, beacons, lights or markers, except elements organizers supply as part of the course. (rule 6.2)
- Do not place, move, mark or alter anything in or around the course, during practice or judging. (rule 6.3)
- **Preloaded maps are allowed**, including maps built from the published layout. Course-specific hard-coding is allowed but lowers generality scoring.
- **Do not reuse prior course-run data as direct input to another run.** This applies to practice and scored runs, including saved point clouds, Gaussian splats, captured maps or recorded-flight replays. Tuning code from lessons learned is allowed. (rule 6.4)

## 4. Course, checkpoints and run behavior

The published layout is **preliminary**. Changes are posted on the challenge page and announced in Slack before scored runs open. The drone still needs to localize and react to what it sees.

| Published feature | Description |
|---|---|
| Drone net | 20 x 20 ft footprint; 10 ft tall |
| Walls | 68 in tall; 4 in thick |
| Boulders | Diagram describes them as 18 in across |
| Route | Maze, dark tunnel through the barrier, chicane, landing |
| Downloads | OBJ + MTL, STEP, GLB; course files in metres; OBJ/STEP Z-up, GLB Y-up |

Follow the floor-marked maze route, pass through the dark tunnel, then enter the landing area through Gate B at the north end or the window in the lower wall.

| Checkpoint | Completion condition | Points |
|---|---|---:|
| 1 | Take off and reach the end of the first corridor, where the floor line turns east | 4 |
| 2 | Leave the maze where the floor line ends, through Gate A | 7 |
| 3 | Exit the south end of the dark tunnel | 10 |
| 4 | Enter the landing area through Gate B or the window, without landing | 7 |
| 5 | Land and disarm with the **whole drone inside the pad's outer ring** | 7 |

Checkpoints count **in order, only once the whole drone has passed**. A skipped checkpoint limits scoring to the last checkpoint before it. Centre crossing or merely approaching the line is insufficient. (rule 7.6)

> **Layout change, 2026-11-07 (recorded by the team, @jsun4231; not part of the webpage snapshot above).** The course changed: the three maze dividers were removed and two walls added, and **checkpoint 1 was removed**. Checkpoints 2–5 are otherwise unchanged and keep their numbers and points, so the below-wall checkpoints now sum to **31 points**. The updated model is the repository's `dronehacks-course-v3.step`; `contracts/course.md` ("Layout revision 2026-11-07", "Checkpoints") holds the geometry and the checkpoint lines.

### Start, clock, contacts and termination

- Place the drone on the launch mark. The team arms it in the referee's presence and starts it on the referee's signal. (rule 7.4)
- The clock starts when the drone **leaves the launch mark** and stops at pad disarm or when the run ends. Overhead video supplies checkpoint times. The referee's times, checkpoint count and chicane route are official; teams may also film. A local liftoff threshold is only an estimate. (rules 7.5, 7.12)
- Some part of the drone's **frame** must remain below the **68-in wall tops**, except during the optional return. A checkpoint reached by flying over a wall or barrier anywhere since the previous checkpoint earns only **15% of its points**. (rule 7.7)
- Each wall, tunnel or boulder touch that does not end the run adds **5 seconds** to the finish time and every later checkpoint time. A net-only contact penalty is unspecified. (rule 7.8)
- A run ends on pad disarm without a return attempt, a crash, ground contact anywhere else, leaving the course, reaching **3 minutes**, team abort or referee stop. (rule 7.9)

### Optional return

After pad landing and disarm, the drone may take off again **by itself**, fly over the barrier into the start area, then land and disarm **within 2 ft of the launch mark**, all within the same **3-minute limit**, without team input. The run ends when the return ends. A return crash or miss loses only the bonus, preserving the outbound run. The text does not define the reference point used to measure the 2 ft. (rule 7.10)

## 5. Scoring, presentation and awards

| Component | Points / assessment |
|---|---|
| Run | Up to 60 before the return bonus |
| Technical presentation | 25: autonomy design, technical execution, originality, testing and iteration |
| Generality | 15: how well the software would work on another course |
| Successful optional return | Multiply earned **run points** by **1.25**; the total can exceed 100 |

Let `C` be earned checkpoint points, `t` the team's penalty-adjusted time to its last checkpoint or finish, and `t_best` the fastest eligible time to that same checkpoint or finish across all teams and either route.

```text
No checkpoint:       run_points = 0
Unfinished run:      run_points = C + 7 * (t_best / t)
Finished run:        run_points = C + 20 * (t_best / t) + window_bonus
window_bonus:       5 for a finished window-route run; otherwise 0
Successful return:  run_points = run_points * 1.25
Total:              run_points + presentation_points + generality_points
```

All below-wall checkpoints sum to **35 points**. Over-wall checkpoints receive the reduction above. An over-wall run cannot provide the reference fastest time and cannot win Fastest Run. The page states that reaching another checkpoint beats completing fewer faster, and every below-wall finish beats every unfinished run. Faster run time breaks ties.

The **5-7-minute technical presentation**, followed by questions, explains the system, key design decisions and results. Effective auxiliary-sensor use is considered; simply carrying a sensor earns nothing. (rule 4.6)

Generality is assessed through answers and code review. After repository links are due, every repository receives an AI review with the same instructions, looking for layout-specific waypoints, routes and timings. Flags inform judges' questions and generality scoring; the review itself does not add or remove points. Published-layout maps are valid, while hard-coding scores lower for generality. (rule 9.5)

| Award | Team prize | Criterion |
|---|---:|---|
| Rothberg Catalyzer Award | $7,000 | Highest overall score |
| Runner-Up | $3,000 | Second-highest combined score |
| Third Place | $2,000 | Third-highest combined score |
| Most Technologically Impressive | $2,000 | Presentation and generality scores alone |
| Fastest Run | $1,000 | Fastest finished below-wall run, including touch penalties |

Prizes are per team; five awards total **$15,000**.

## 6. Hardware changes, auxiliary sensors and safety

### Keep the issued core kit

The frame, motors, ESCs, propellers, power system, Jetson Orin Nano, Pixhawk 6C, OAK-D S2 and VL53L1X must stay **on the drone and in use**. Do not remove, replace or bypass them. Motors, ESCs, propellers, battery and power wiring may not be changed. (rules 3.1, 3.3)

Remounting/re-angling parts or printing brackets is allowed, followed by reinspection. Secure every added component. Only compliant auxiliary sensors may add powered hardware; no additional computers. A microcontroller used only to read a sensor counts as part of that sensor. (rules 3.2, 3.4-3.6)

### Auxiliary-sensor limits

- Each sensor must have a normal new retail market value **strictly under US$100** and mass **strictly under 50 g**. Discounts, loans and gifts do not change market value. (rules 4.1-4.2)
- Organizer-weighed mass includes the as-sold board, enclosure and lens; excludes cables and mounting hardware. Necessary breakout boards or sensor-reading microcontrollers count toward both price and mass. (rules 4.3-4.4)
- Before flying, declare each sensor's part number, retail-price listing and mass at inspection. Undeclared sensors may not fly. (rule 4.5)
- Power sensors from the drone's battery; **no extra batteries**. (rule 4.7)
- Regardless of price, no GNSS receivers, sensors relying on off-drone equipment such as UWB anchors/beacons/motion capture, or **lasers above Class 1**. (rule 4.8)

### Flight and battery safety

- Everyone handling or flying the drone attends the flight-net safety briefing before flying. (rule 8.1)
- Fly and arm **only inside the closed drone net**. Remove propellers for work at the table. (rule 8.2)
- During every flight, a designated teammate watches the drone, ready to activate the kill switch. Obey referee/organizer kill orders immediately. (rule 8.3)
- Pass organizer inspection before the first flight and after any hardware change: secure propellers, strapped battery, tidy wiring, working kill switch and declared sensors. (rule 8.4)
- **Preserve organizer-configured PX4 failsafes, kill switch, geofence and battery limits.** Earlier tuning recipes do not override this restriction. (rule 8.5)
- Use only issued batteries; charge only at the charging station, never unattended. Immediately hand swollen or damaged batteries to an organizer. (rule 8.6)
- Follow the shared-net queue; one drone flies at a time unless organizers say otherwise. Unsafe flying can suspend a team temporarily or for the event. (rules 8.7-8.8)

## 7. Kit custody, software and fair play

- The captain signs out one kit and is responsible until check-in. The kit remains organizer property and must be returned complete Sunday **4:30-5:30 PM**. Only the issued aircraft may fly in practice or judging; use only issued propellers and batteries. (rules 2.1-2.4)
- Immediately report damage/faults. Replacement parts are at organizer discretion; return broken parts. (rule 2.5)
- Build the autonomy stack **during the event**. Open-source libraries, pretrained models, public datasets and the organizer starter project are allowed; credit them in both README and presentation. Eligibility of particular pre-event custom code is not established. (rule 9.1)
- AI coding tools are allowed and encouraged; the team must explain how its software works. (rule 9.2)
- Submit the team repository link by Sunday **12:30 PM**. Everything flown must be pushed by Sunday **4:00 PM**; pushing before each run is not required. Include a README explaining the stack and how to run it. (rules 9.3-9.4)
- Teams may discuss ideas and help each other, but each team's code and presentation must be its own; copying another team's code is prohibited. (rule 10.1)
- Do not interfere with another team's drone, laptop, network or radio link: no jamming, deauthentication or connecting to their Jetson. Do not tamper with the course, timing or organizer equipment. (rules 10.2-10.3)
- Misstating sensor price/mass, flying undeclared hardware, or flying code absent from the repository at Sunday's 4:00 PM deadline is cheating. (rule 10.4)
- The head judge determines proportionate penalties: time penalty, unfinished-run scoring, run disqualification or team disqualification from awards. Report suspected breaches privately to an organizer. (rules 10.5-10.6)

Rules apply from Friday kit pickup through Sunday return. The DroneHacks Code of Conduct applies; violations can disqualify a team. Organizers may amend rules, announcing changes in Slack and on the page with their effective time. For uncovered actions, the rulebook directs participants to ask an organizer; conduct against the spirit of autonomous GPS-denied flight on the issued drone is prohibited. Head-judge scoring, penalty and disqualification decisions are final. (rules 1.1-1.5)

## 8. Schedule and scored-net logistics

All times are **Pacific Standard Time (PST)**, November 6-8, 2026. Flight-net access depends on supervision and availability.

| Date | Time (PST) | Event / deadline |
|---|---|---|
| Fri Nov 6 | 4:45 PM | Hacking begins after opening ceremony |
| Fri Nov 6 | 7:50 PM | Oak opens: safety briefing and kit pickup |
| Sat Nov 7 | Daytime | Build and practice |
| Sat Nov 7 | 7:30 PM | Run blocks and presentation slots posted in Slack |
| Sat Nov 7 - Sun Nov 8 | 10:00 PM - 2:00 AM | Scored runs in blocks |
| Sun Nov 8 | 8:00 AM - 4:00 PM | Scored blocks, then recovered forfeited time; runs close at 4:00 PM |
| Sun Nov 8 | **12:30 PM** | **Repository link due** |
| Sun Nov 8 | 12:30 PM - 3:10 PM | Technical presentations, two concurrent panels |
| Sun Nov 8 | **4:00 PM** | **All flown code pushed** |
| Sun Nov 8 | 4:30 PM - 5:30 PM | Complete kit return |
| Sun Nov 8 | 6:00 PM | Closing ceremony and awards in Simonyi |

Each team receives equal scored net time in **two equal blocks**, one in each half of scored runs, with **1.5-minute changeovers**. Lengths/order are posted Saturday at 7:30 PM around presentation slots. Arrive charged **10 minutes early**; blocks start on time regardless of readiness. Fly as many runs as fit; the best counts. A flight continuing at block end is stopped and scored on cleared checkpoints. (rules 7.1-7.2)

For failures beyond a quick fix, a team may forfeit its remaining block time. That time, minus a 1.5-minute changeover, is recovered after Sunday's final scheduled block, in forfeiture order, until 4:00 PM. Time that does not fit is lost; recovered time cannot be forfeited again. (rule 7.3)

## 9. Missing information and engineering assumptions

These entries deliberately remain unconfirmed. They are not extra requirements, permissions or requests for approval.

| Topic | Evidence boundary |
|---|---|
| Onboard lights | No confirmed blanket ban on every onboard indicator. Apply the explicit external-positioning and added-powered-hardware restrictions; a broader reported verbal instruction lacks webpage evidence. |
| MS200K/MS200 lidar eligibility | Exact retail value, counted mass, laser class and inspection acceptance are missing. The previous repo's lidar experiments do not establish eligibility. |
| Issued-kit details | Exact frame size/material/builder, motors, ESCs, props, battery, JetPack/PX4/package versions, wiring, ports and baud rates are not established beyond the confirmed descriptions above. |
| Middleware alternatives | ROS 2 Humble/uXRCE-DDS is the documented supplied environment. Organizer acceptance of a direct-MAVLink or ROS-free replacement is unspecified. |
| Course geometry | The old repo's CAD/map has not been matched to the latest official download or final judged layout. Exact checkpoint coordinates, tunnel/window dimensions and pad radius remain unverified. |
| Course surveys | Permission for a separate survey outside a run is unspecified. The explicit ban on feeding captured practice/scored-run data into later runs still applies. |
| Net-only contact | No explicit penalty if it does not also crash the drone or leave the course; do not invent one. |
| Pre-event custom code | Permitted reuse categories are listed, but eligibility of particular pre-event custom autonomy tools is undetermined. |
| Core-sensor use | All core components must be installed and in use; minimum qualifying use of each sensor and the actual deployed roles are unverified. |
| Organizer materials | Exact block lengths, individual slots, starter-repository URL/contents, full Code of Conduct and later amendments are absent from the supplied snapshots. |
| Actual device performance | Maximum webpage capabilities do not establish installed operating modes, actual sensor rates, depth performance or enabled Super mode. |
| Return measurement | The 2-ft rule does not identify centre versus whole-outline measurement. |

There is **no confirmed hoops task, separately scored mapping/localization deliverable or mandatory map display** in these snapshots. Visualizations may support the presentation; they are optional.

The previous repo used the following **engineering inputs**, not official rules or verified issued-drone capabilities: drone dimensions **0.40 x 0.50 x 0.20 m**, pad radius **0.7366 m**, cruise height **0.8 m**, return height **2.3 m**, general margin **0.05 m**, additional tunnel margin **0.10 m**, fixed heading, **50 Hz** setpoints, **2.5 m/s** horizontal speed and **3 m/s^2** acceleration. Copying this reference does not adopt those values as requirements.

If migrating code from the previous repository, preserve these known limitations in the new project's issue tracking:

1. **Checkpoint logic:** its planner uses centre crossings, and its analyzer also accepts proximity within 0.10 m. Both differ from the official whole-drone-passage requirement. Documentation corrections did not fix the algorithms.
2. **Start procedure:** its player automatically commands mode/arming; how deployment gates this against team arming and the referee's start signal remains unverified. Autonomous rearming for the optional return is separately permitted.

## 10. Source provenance

Based on supplied saved copies of the [official challenge/rules page](https://dronehacks.groups.stanford.edu/connect/challenge) and [official drone page](https://dronehacks.groups.stanford.edu/connect/challenge/drone), checked on **2026-11-06**. Live access was unsuccessful; subsequent website/Slack amendments have not been verified. This is a condensed reference, not a claim that later rules are unchanged.

Original saved HTML SHA-256 fingerprints:

```text
Challenge/rules: 9bddff7073f33fc36140cb3018e58198b2991ac93b510bc33afe7a2984b782b8
Drone:           a679db7362f3bece5993b7cdd420f11e19bacfabd685bc313642389b3ba09339
```

This file has no dependency on the previous repository, its local evidence files or its source code. It can be copied into a new repository as `DRONEHACKS.md`.
