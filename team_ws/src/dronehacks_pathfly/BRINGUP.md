# Bring-up: flying the path-planning route on drone 11

This takes `dronehacks_pathfly` from a laptop to a full autonomous run in the
maze. It covers:
- loading the package;
- setting up START so drift is as small as possible;
- ground checks with the props off;
- three staged test flights;
- the full mission.

**What the full mission does.**
1. Lift off to 0.2 m.
2. Fly the planned path while climbing to 1.25 m.
3. Touch down at the GOAL.
4. Climb to 2 m and fly straight back to START over the walls.
5. Touch down on START.

The nose points east the whole time; the drone never yaws. A full run takes
about 60 s from arming to disarm.

**Roles.**
- **Operator:** laptop and SSH.
- **Safety pilot:** transmitter and kill switch.
- **Spotter:** watches the walls in the tight stretches.

The safety pilot can take over at any moment and lands the drone by hand.

> **Open item. Settle it before any flight past stage 1.** The path plan puts
> the north wall 0.54 m inside the outer boundary (map y = 5.556). Another
> measurement of the taped arena put it 0.61 m inside. If it is really 0.61 m,
> the drone's top edge clears the wall by about 1 cm instead of 8 cm.
> Tape-measure it near map x ≈ 2 m (see 3.1). If it is not 0.54 ± 0.01 m,
> re-plan in the path-planning repo and reload the path (README, "Path data").

---

## 0. What to bring

- [ ] Laptop on the arena Wi-Fi, with SSH. Bring a USB-C cable as a fallback
      (the drone answers on `192.168.55.1` over USB).
- [ ] Tape measure, a straight edge at least 1 m long (a level or a
      yardstick), painter's tape, a marker.
- [ ] Charged batteries: one per flight. You will power-cycle the drone
      between flights (section 5).
- [ ] A tether for stages 1 and 2, if the organisers allow or require one.
- [ ] This package, with its tests passing. On the laptop, from
      `team_ws/src/dronehacks_pathfly`, run
      `python3 -m unittest discover -s test`. It should report 34 tests,
      OK.

---

## 1. Load the package onto the drone

1. Connect and **check that you are on drone 11**:

   ```bash
   ssh dronehacks@<drone-11-wifi-ip>      # or ssh dronehacks@192.168.55.1 over USB
   hostname                               # must print skybeta-11
   ```

   If it prints anything else, log out immediately and tell the organisers.
   Logging in to another team's drone disqualifies the team, even if it was
   an accident.

2. Copy the package from the laptop (from the repo's `team_ws/src`):

   ```bash
   scp -r dronehacks_pathfly dronehacks@<drone-11-wifi-ip>:~/team_ws/src/
   ```

   The stack builds **everything** in `~/team_ws/src` at start-up. A broken
   package next to this one stops this one from starting too. Leave only
   packages that build.

3. Health check: `~/skybeta/host/doctor.sh` must exit 0.

4. Point the stack at this package, **once**. Edit
   `sudo nano /etc/skybeta/drone.env` and set:

   ```bash
   COMPETITOR_PKG=dronehacks_pathfly
   COMPETITOR_LAUNCH=pathfly.launch.py
   ```

   Change nothing else in that file. In particular, never change
   `DRONE_ID`, `ROS_DOMAIN_ID` or `TEAM`.

   Then choose the stage with a file in the team workspace. Writing it needs
   no sudo, and it is read each time the stack starts:

   ```bash
   echo 1 > ~/team_ws/pathfly_stage      # 1, 2, 3, or "full" (no file = full)
   ```

   The `competitor` window's first lines say which one it read, either
   `[pathfly] STAGED TEST 1` or `[pathfly] FULL MISSION`. `stageN.launch.py`
   still works in `COMPETITOR_LAUNCH` and overrides the file.

5. The change takes effect when the stack restarts. Restart it the way
   section 4 describes: power-cycle the drone with it sitting on START.

**Do not change PX4 parameters for this mission.** The mission needs none, and
the team's rules allow only listed parameters.

---

## 2. How the drone knows where it is (why the setup matters)

The drone has **no GPS and no compass**. Its position comes from the stereo
camera (cuVSLAM) and is relative to where it was when tracking started.

The mission works out where the maze is from one assumption: at the moment the
pilot hands over (Offboard starts), the drone sits on START with its nose
pointing exactly east. Every error in that pose carries into the whole flight:

| Error at hand-over | What it does in flight |
|---|---|
| 1 cm off the START marker | The whole path is shifted 1 cm |
| **1° of nose error** | The path is rotated about START: **2.4 cm** off at the north corridor, **5.7 cm** off at the GOAL |
| 1° of heading mismatch between the camera and PX4 (section 4.3) | Velocity commands are rotated by 1°; the controller corrects it, at the cost of a few cm |

The planner leaves only **8 cm** of margin in two places:
- about 0.8 m of the top corridor under the north wall, at map x 1.8–2.3 m;
- the GOAL.

Placement and heading are therefore worth more than any tuning: aim for
**±1 cm and ±0.5°**.

---

## 3. Arena setup (once per session)

### 3.1 Check the map against the real arena

Map frame: metres from the arena's inside south-west corner. x points east,
y points north.

- [ ] **North wall:** measure from the outer boundary to the taped wall at
      x ≈ 1.8 m and x ≈ 2.3 m. The plan assumes 0.54 m (open item above).
- [ ] **START marker:** its centre should be 0.580 m from the west wall
      (map x = 0) and, if the wall is where the plan says, 0.960 m south of
      the north wall.
- [ ] **GOAL marker:** centre at map (0.900, 1.350).
- [ ] Nothing new stands on the straight line from GOAL to START. The return
      leg crosses the wall block there at 2 m, about 0.35 m above 1.65 m
      walls.

### 3.2 Make an alignment jig at START

The jig decides your heading error, so do it carefully:
1. Lay a strip of tape running **east-west** through the centre of START.
   Make it parallel to the north wall: measure from the north wall at two
   points 1 m apart; the two distances must agree within 5 mm.
   - The same check works from the south boundary, if that is easier to
     reach.
   - 5 mm over 1 m is 0.3°.
2. Put the straight edge on that line.
3. Mark the drone's centre on the airframe, if it isn't obvious. Use the
   flight controller or the frame centre, not the battery.
4. Decide which flat face of the airframe you will butt against the straight
   edge. It must be a face parallel to the drone's nose direction, which is
   where the camera looks.

**Placement for every flight:**
1. Centre on START.
2. Butt that flat face against the straight edge, with the **camera facing
   east**, down the top corridor.
3. Check the centre is on the marker within 1 cm.

### 3.3 Give the camera something good to look at

The camera looks east down the top corridor for the whole flight. Stereo
visual odometry drifts least when the view is:
- [ ] **Textured**: posters, tape patterns or anything with edges and
      corners. Blank walls and bare floors give it nothing to hold. Add
      texture only where the organisers allow.
- [ ] **Static**: nobody walks in front of the camera. People and moving
      shadows look like the world moving. During the flight everyone stands
      west of START, behind the drone's starting nose direction. Keep the
      corridor ahead empty.
- [ ] **Evenly lit**: no direct sun on the floor, which also blinds the
      downward rangefinder, and no flicker.
- [ ] **Clean and secure**: wipe the OAK-D lenses. The camera mount and cable
      must be tight. Never change `CAMERA_*` in `drone.env`; the organisers
      measured them.

---

## 4. Power-up and ground checks (props off)

### 4.1 Power up in place

1. Put the drone in the jig: centred on START, nose east (3.2).
2. **Then** plug in the battery. Starting the camera tracking here makes its
   coordinates line up with the map from the outset.
3. **Do not touch it for at least 60 s.** Heading and position are measured
   from start-up.
4. Wait until the clock has synchronised:

   ```bash
   timedatectl | grep synchronized        # System clock synchronized: yes
   ```

   When the clock syncs, the stack restarts itself. If that happens, give it
   another 60 s untouched.

### 4.2 Stack checks

**Quick way:** one read-only command runs every check below. It ends with
`READY` or `NOT READY`, with the list of what failed:

```bash
ssh skybeta11 'bash ~/team_ws/src/dronehacks_pathfly/scripts/preflight.sh'
```

The manual steps follow.

Use two SSH sessions on the drone:

```bash
# session A (host): watch the stack; the competitor window is this node
docker exec -it skybeta-stack tmux attach -t skybeta   # Ctrl-b d to leave

# session B: a shell inside the running stack, for every ros2 command below
~/skybeta/host/run.sh shell
```

- [ ] In the `competitor` window, the build succeeded and the node printed its
      summary:
      - `path 12.27 m, 124 points ...`
      - `NOSE AT LIFT-OFF: 0.0 deg ... (east, along arena +x)`
      - for stages, a `STAGED TEST: ...` line.

      There must be **no** `WARNING: the path was planned for ...` line.
- [ ] `ros2 topic echo /skybeta/state --once`: `px4_link: true` and
      `local_enu` not null.
- [ ] `ros2 topic echo /visual_slam/status --field vo_state --once`: 1
      (tracking). 2 means lost.
- [ ] `ros2 topic echo /fmu/in/vehicle_visual_odometry --field pose_frame --once`:
      2. If it shows anything else, stop and tell the organisers.

If `px4_link` stays false after a stack restart, power-cycle the whole drone.
On some firmware the flight controller loses the link until it reboots.

### 4.3 Drift checks with `pathfly_check`

`pathfly_check` commands nothing; it only listens. Run it in session B. Start
it with the drone **sitting in the jig**, because it anchors to the drone's
pose at that moment:

```bash
ros2 run dronehacks_pathfly pathfly_check
```

Twice a second it prints:
- where the drone thinks it is, in map coordinates, and its height;
- its nose direction;
- how far it is from the planned path, and how much of the planner's wall
  margin would be left there;
- distance from START by the camera and by PX4;
- the rotation between the camera's frame and PX4's, measured from motion
  (with the attitude heading difference shown for information only).

Ctrl-C prints a summary.

1. **Standing still, 60 s.** The position should stay within about 1 cm of
   START and the nose within 0.2°. If it wanders, fix the view (3.3) before
   going further.
2. **Frame rotation.** This is measured from motion, so it comes out of the
   carry test (step 3).
   - Once the drone has been carried more than 0.5 m, the tool prints
     `frame rotation ... deg`. The summary prints
     `odom_to_ned_yaw_offset_rad from motion`. That number, and only that
     number, is the value for `config/pathfly.yaml`.
   - Set it only if it is steady through a 1–2 m carry and larger than
     about 1°. The launch file passes it to the flight interface too.
   - **Do not use the `attitude heading diff` reading.** It compares PX4's
     and the camera's *orientation*, so the camera mounting and frame
     conventions feed into it. It is also meaningless when the drone is
     tilted far or upside down: heading and roll mix there.
     - On 2026-11-08 it read about −171°, with the drone lying upside down
       on a desk.
     - Setting a value like that would make the drone fly its velocity
       commands backwards.
   - After changing it, restart (section 5). Stage 1 confirms the sign: with
     the right sign it tracks better, not worse.
   - *Measured on 2026-11-08 on the bench:* `odom_enu` and `local_enu`
     agreed to within 5 cm about 15 m from the vision origin. Position does
     not depend on how the drone is tilted, so the frames are aligned and
     the offset stays **0.0**.
   - Do the still test and the carry test with the drone **upright**. An
     inverted or tilted camera sees the desk or ceiling up close, and its
     drift numbers mean nothing for the arena.
3. **Carry test.** One person carries the drone along the path at about
   1.25 m, nose kept east, walking slowly. Put tape marks at a few path
   points to compare against: (1.81, 5.21) and (2.31, 5.22) in the tight
   corridor, (5.23, 2.90), and the GOAL (0.90, 1.35).
   - At each mark, the printed map position should agree with the mark
     within a few cm.
   - The camera and PX4 "from start" distances should agree.
   - Carry it back and put it in the jig. In the summary, **closure** should
     be under about 5 cm and **nose drift** under 1°.
   - More than 10 cm of closure means the camera tracking is not good enough
     for an 8 cm corridor. Improve the view (3.3) before flying the full
     path.

   Your hand's wobble shows up as "off-path". The point of this test is the
   camera's own error, not your carrying.

### 4.4 Rehearse the hand-over (props off)

1. Start a bag recording for this and every flight, from a third SSH
   session on the host: `cd ~/skybeta && host/run.sh bag`. It writes into
   `~/skybeta/bags`.
2. Test the kill switch.
3. The pilot arms in **Altitude**, throttle low, then switches to
   **Position**.
4. The `competitor` window should show `WAIT -> TAKEOFF`.
5. With the props off the drone cannot lift. After about 8 s it logs
   `did not leave the ground` and goes to HOLD. PX4 then disarms (10 s rule).
   This confirms the whole hand-over chain.

If nothing happens on Position, `flight_interface` logs the reason
(`auto offboard: staying out of Offboard: ...`).

---

## 5. Between flights: reset to START

The node makes **one attempt per start**. After it finishes or aborts, it does
nothing until it is restarted. For the cleanest reset, which is also the best
for drift, do this before every flight:
1. Unplug the battery.
2. Carry the drone to START and put it in the jig (3.2).
3. Plug in a fresh battery. Wait 60 s or more untouched, and check that the
   clock has synchronised (4.1).
4. Run the quick checks: summary line, `px4_link`, `vo_state` 1 (4.2).
5. Start `pathfly_check` (it is a live monitor in flight too) and a bag
   recording.

To switch stage, write the new stage to `~/team_ws/pathfly_stage` before
step 1.

A quicker reset is `docker rm -f skybeta-stack`, with the drone in the jig;
the stack is back in about 10 s. Then wait 60 s untouched. If `px4_link` does
not come back, power-cycle the drone.

---

## 6. Staged flights

Every stage begins the same way:
1. The drone is in the jig, the checks in section 5 are done, and
   `pathfly_check` and a bag are running.
2. The pilot arms in Altitude, then switches to Position.

The pilot keeps a thumb near the throttle for the whole flight. Raising it
towards centre takes over.

**Stop the session and investigate** after any of these:
- a wall touch;
- `HOLD` in the log;
- oscillation;
- a landing more than 5 cm from its target;
- `pathfly_check` showing less than 3 cm of margin left in flight.

| Stage | `~/team_ws/pathfly_stage` | What it flies | Lands at (map) | Arm to disarm |
|---|---|---|---|---|
| 1 | `1` | Lift-off to 0.2 m, then 0.4 m of path while climbing to ~0.37 m | (0.93, 4.79) | ~14 s |
| 2 | `2` | Climb to 1.25 m, through the 8 cm stretch under the north wall | (3.03, 5.14), in the top corridor | ~21 s |
| 3 | `3` | The whole path | GOAL (0.90, 1.35) | ~39 s |
| 4 | `full` (or no file) | Full mission: GOAL, then 2 m return over the walls | START (0.58, 4.60) | ~60 s |

**Stage 1 (tethered, low).** This checks:
- lift-off at 0.2 m (close to the rangefinder's minimum range: watch for
  bobbing);
- the twist commands;
- touchdown detection;
- auto-disarm.

Measure where it landed against (0.93, 4.79). If you set a heading offset,
compare the landing against a stage 1 flight without it. If it lands further
off with the offset, the sign is wrong: negate it.

**Stage 2.** This is the first pass through the tight stretch. The spotter
stands beside the corridor, out of the camera's view, and watches the gap to
the north wall. In `pathfly_check`, "left" should stay above 3 cm through
path metres 1.4–1.9.

**Stage 3.** The whole path. Check the GOAL touchdown against the marker:
- **lands short along the corridor:** fine;
- **lands sideways off it:** heading error. Redo the jig, the offset check
  and the carry test.

**Stage 4.** The full mission. After touchdown at the GOAL, the drone
re-climbs within about 0.5 s. PX4 disarms after 5 s on the ground, so this
gap matters. The return leg is flown sideways at 2 m, and the drone lands on
START.

---

## 7. Competition run checklist

- [ ] `COMPETITOR_LAUNCH=pathfly.launch.py` and `~/team_ws/pathfly_stage`
      says `full` (or is deleted). The `competitor` window shows
      `[pathfly] FULL MISSION`, and the summary shows no `STAGED TEST` and no
      `WARNING`.
- [ ] North wall position confirmed (open item). Map checked (3.1).
- [ ] Jig on START, drone centred, nose east, battery plugged in **after**
      placing, 60 s or more untouched, clock synchronised.
- [ ] `px4_link: true`, `vo_state` 1, `pose_frame` 2.
- [ ] Camera view: lenses clean, corridor ahead empty, team standing west of
      START.
- [ ] `pathfly_check` and a bag running. Kill switch tested.
- [ ] Pilot: arm in Altitude, switch to Position, thumb near the throttle.

---

## 8. Least-drift summary (most important first)

1. **Nose exactly east, centre exactly on START.** 1° is 5.7 cm at the GOAL.
   Use the jig every flight.
2. **Power up in place and stay still for 60 s.** Wait for clock sync before
   flying.
3. **Give the camera a textured, static, evenly lit view east.** Keep people
   out of it.
4. **Measure the camera-to-PX4 frame rotation from motion** (4.3), never
   from attitude; set it only if it is steady
   and above 1°.
5. **Carry test before flying:** closure under 5 cm, nose drift under 1°.
6. **Short flights, fresh battery, fresh start each time.** Drift grows with
   distance travelled and with tracking hiccups.
7. **Do not fly faster than the defaults** (0.6 m/s, 0.25 m/s in tight
   places). Motion blur costs tracking, and slow is what keeps the error
   inside 8 cm.
8. **Do not change the heading.** The planner's 8 cm margins are only valid
   with the nose east. The node warns if it is configured otherwise.

---

## 9. Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| Nothing happens when the pilot switches to Position | No hand-over: wrong order (armed in Position), stale odometry, or the node is not running | Read `auto offboard: staying out of Offboard: ...` in the `competitor` window. The node publishes only while odometry is fresh |
| Log: `camera and PX4 positions disagree by X m ... Offboard will be refused` | The camera tracking has drifted away from PX4's estimate. On 2026-11-08, after the drone sat upside down on a desk, the camera height was 25 m off. The node sends nothing, so the pilot keeps control | Power-cycle the drone sitting on START, upright. Wait 60 s and check the line is gone |
| `HOLD (zero velocity)` in flight: `camera and PX4 positions drifted apart ...` | The camera tracking drifted more than 0.5 m horizontally, or 0.4 m in height over the floor, since take-off | Pilot takes over and lands. Check the camera's view (3.3) and the bag |
| Summary shows `WARNING: the path was planned for ...` | `start_yaw_deg` is not 0, or `hold_heading` is false | Restore the defaults in `config/pathfly.yaml` |
| Takes off and goes straight to HOLD | It drifted more than 0.5 m off the path, odometry was lost, or it went above 2.5 m | Pilot lands. Check `vo_state`, then the view and the jig |
| Drifts steadily to one side along the corridor | Heading error (jig) or a camera-to-PX4 frame rotation | Redo the jig; measure the frame rotation from motion (4.3) |
| Path looks rotated: fine near START, worse further away | Nose not east at hand-over | Jig, and check `start_yaw_deg` is 0.0 |
| Bobs at 0.2 m | Rangefinder near its minimum range | Raise `liftoff_height_m` to 0.3 in `config/pathfly.yaml` |
| Disarms on the ground at the GOAL instead of re-climbing | The node lost Offboard, or PX4 disarmed first | Check the log for `ABORTED`. The node waits only 0.5 s on the ground |
| Lands near, but not on, a marker | Normal camera drift, a few cm | Note it per stage. If it grows from stage to stage, look at the view and the offset |
| `px4_link` false after a restart | The flight controller lost the link | Power-cycle the drone |

Logs: the `competitor` window shows every phase change. The bag
(`~/skybeta/bags`, when started from `~/skybeta`) records the camera images,
odometry, setpoints and PX4 state, for working out a bad flight afterwards.
