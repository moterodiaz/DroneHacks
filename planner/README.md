# planner: course map in, flyable trajectory out

The Planner and Follower boxes in `contracts/ros-graph.md` (PR #5). The
planner reads the course map file from `contracts/course.md` and returns a
route. The rest of the pipeline smooths that route, works out the speed at
every point, and turns the result into the position commands the kit's
`skybeta_flight_interface` accepts. It is pure Python (numpy + PyYAML, both
in the stock skybeta image), with no ROS dependency yet.

**Status: not flown, not wired into ROS.** Every timing below comes from a
simulator with one guessed parameter, run on a *provisional* map transcribed
from the field guide's drawing (`test/fixtures/published_v3_fieldguide.yaml`).

```sh
cd planner
python -m dronehacks_planner test/fixtures/published_v3_fieldguide.yaml --svg route.svg   # route only
python -m dronehacks_planner.bench test/fixtures/published_v3_fieldguide.yaml --out ../runs/bench
python -m dronehacks_planner.bench test/fixtures/published_v3_fieldguide.yaml --out ../runs/bench \
    --lidar-follower ../../DroneHacks-depthcam/depthcam      # also fly the lidar stream's follower
python -m unittest discover -s test          # 83 tests, ~12 s; or: python -m pytest test
```

On the drone, `planner/` is an ament_python package, `dronehacks_planner`.
Copy it into `~/team_ws/src` and the kit's `team_build.sh` builds it. It
provides `ros2 run dronehacks_planner plan MAP.yaml` and `... bench MAP.yaml`.
The flight stack that flies it is `nav_traj/`: package `dronehacks_traj`, on
branch `navigation/vo-traj-follower` (worktree `../DroneHacks-traj`). It has no
lidar, and this Follower flies its FOLLOW state.

## Results (simulated, provisional map)

`bench` flies each method in `sim.py`, a stand-in for PX4 that uses the
kit's real controller gains and limits (see "The simulator" below). At the
1.0 m/s speed cap in drone 1's parameter file:

| Method | Planned | Simulated | Wall touches | Closest wall | Max off-path |
|---|---|---|---|---|---|
| A: waypoints, as in the kit example | — | 30.0 s | 0 | 0.37 m | 0.13 m |
| B: smoothed path, one speed throughout | 65.6 s | 65.6 s | 0 | 0.41 m | 0.05 m |
| **C: smoothed path, speed profile, lead (the Follower, `lead=True` as then)** | **23.0 s** | **23.5 s** | **0** | **0.35 m** | **0.18 m** |
| C0: C without the lead (= the shipped default now) | 23.0 s | 51.0 s | 0 | 0.41 m | 0.04 m |
| L: the lidar stream's follower, its defaults | — | 54.8 s | 0 | 0.41 m | 0.06 m |
| L, with a 1.0 m carrot (our tuning, not theirs) | — | 39.5 s | 0 | 0.40 m | 0.12 m |

"Closest wall" is from the drone's centre; its props reach 0.28 m. Every
method passed both openings whole-drone and landed inside the pad's ring.
"Max off-path" is measured against the path each follower flies: the
smoothed path for B and C, the planner's polyline for A and L.

Defaults changed after issue #25: the window is now opt-in
(`PlannerConfig.allow_windows`) and the lead is off by default, so the
default route is Gate B and the default follower publishes plain poses. The
numbers above were measured with the window route and the lead on; `bench`
now labels the lead-on run "C1" and flies plain poses for C.

### Two followers: both kept, ours flagged as faster

The lidar stream (`../DroneHacks-depthcam`, `depthcam/follower/`) has its own
Follower. It reads this planner's route JSON through its own loader and works
in its tracker frame. It moves a "carrot" 0.4 m ahead of the drone along the
polyline, stopped at each segment's end, so it never cuts a corner. It has no
speed profile and no lead. `bench --lidar-follower` flies its code unmodified
in the same simulator.

**Flagged: this stream's Follower (C) performs better.** It is 2.3× faster
(23.5 s vs 54.8 s), and still 16 s faster than the lidar stream's follower
with a longer carrot. Both had zero touches.

The lidar stream's follower is better in two ways that matter:
- **It keeps more clearance:** 0.41 m from walls vs 0.35 m, and it strays at
  most 0.06 m from its path vs 0.18 m.
- **It is a complete mission.** It waits for Offboard, takes off, holds over
  the pad and lands. Ours has none of that yet.

Its speed comes from the carrot distance, about 0.95 × carrot, so it does not
get faster with a higher speed cap: 54.8 s at 2 m/s too. Both followers are
kept. The obvious combination is the lidar stream's mission states, with
this Follower doing the flying in its FOLLOW state.
At an unvalidated 2 m/s cap, C takes 22.2 s and A 28.2 s. On planned time
the window route is faster than Gate B (23.0 s to 24.5 s), but
`contracts/course.md` leaves the window's `clearance_radius` unreviewed, so
Gate B is the default and the window is opt-in
(`PlannerConfig.allow_windows`).

What the comparison shows:

* **Speed profiling is worth 7 s over the kit's waypoint approach (−22%).**
  B shows why: a single safe speed must be the slowest corner's.
* **The lead is worth 27 s.** Without it, PX4's position loop lags a moving
  reference by `v / MPC_XY_P`, about 1 m at 1 m/s. The progress governor
  then holds the reference back for 98% of the flight.
* **Jerk limits cost 3.5 s.** C with acceleration limits only would plan
  19.4 s, but with 40 m/s³ jerk spikes. At `j_max` = 8 (`MPC_JERK_MAX`)
  instead of 4 (`MPC_JERK_AUTO`), it plans 20.8 s. Neither value is
  measured; this is the first thing to calibrate.
* **The worst tracking error is 0.18 m, at the turn onto the window's axis.**
  The route arrives heading south and turns 109°, and a boulder leaves room
  for only a 0.3 m run-in. That is where to look first on a real flight.

Regenerate with the `bench` command above. It writes `results.md`,
`results.json` and figures (`flights_*.png`, `speed_*.png`, `times_*.png`)
to `runs/bench/`, which git ignores.

## The pipeline

| Stage | Module | What it does |
|---|---|---|
| Route | `route.py`, `grid_search.py`, `clearance.py` | A\* on a 5 cm grid with Nav2-style inflation; openings flown straight; alternatives; exact final check |
| Smooth | `smoothing.py` | Corner fillets, then curvature smoothing; proven clearance bound; repairs |
| Speed | `speed.py` | Speed caps, forward-backward acceleration pass, jerk limits, timestamps |
| Trajectory | `trajectory.py` | Position, velocity, acceleration and heading at any time t |
| Follow | `follower.py` | Plain poses by default, progress governor, yaw; the withdrawn lead formula stays as an opt-in |
| Choose | `pipeline.py` | Builds every allowed alternative (Gate B by default; the window only with `allow_windows`) and keeps the fastest that passes every check |
| Check | `sim.py`, `bench.py` | PX4 stand-in, benchmark, figures |

### Route (planner)

1. **Exact distances** from every 5 cm cell to the nearest wall, net or
   boulder, computed from the map's geometry.
2. **Cost**, as in Nav2's inflation layer. A cell is impassable within
   0.32 m of a wall (0.28 m prop reach + 0.04 m). Beyond that, cost falls
   off as `exp(-3 * margin)` and reaches zero 1 m out.
3. **Openings** (tunnel, window, gate) are flown in a straight line along
   their direction of travel:
   - The exit is far enough out that the whole drone is 0.10 m past the far
     face (rule 7.6).
   - The entry adds a straight run-in of up to 0.30 m.
   - A gate, being an unframed gap, may be crossed up to 35° off its axis.
4. **A\*** links the points, moving in 8 directions. Alternatives are compared
   on cost, looking one stage ahead; `pipeline.fastest` then re-decides on
   flight time.
5. **Shortcut**: a straight line replaces grid steps only if it costs no more
   and comes no closer to obstacles. Short stubs next to openings are merged
   the same way before smoothing.
6. **Exact check** of every segment against every obstacle (segment-to-segment
   distance, no sampling). A failure is an error.

### Smooth

Each corner gets the widest circular arc that fits. That is limited by half
of each shared segment, by the run-in before an opening and the room after
it, and by the rule that the arc comes no closer to an obstacle than the
polyline did. The path is then sampled every 1 cm and averaged with a
triangular kernel 0.35 m wide on each side. Curvature then ramps instead of
jumping, and straight runs stay exactly straight.

The result is checked with a **proven** bound: margin can change by at most
1 cm per cm moved, so the smallest sampled margin less half a sample gap
bounds it everywhere. The check requires the bound to be at least 0, and
within 1 cm of the polyline's own closest approach. The whole-drone crossing
of each opening is re-checked too. Any failure shrinks the nearest arc (or
the smoothing) and rebuilds. The last resort is the polyline itself.

A 0.35 m kernel instead of 0.15 m was 10–17% faster on all six routes tried,
for 0.7–1.5 cm less clearance.

### Speed

Speed caps at every point; the tightest wins:

| Cap | Formula | Why |
|---|---|---|
| speed | `v_max` | PX4's `MPC_XY_VEL_MAX` |
| lateral | `sqrt(a_lat / κ)` | sideways acceleration on curves |
| lateral jerk | `(j_max / |dκ/ds|)^(1/3)` | flying a curvature ramp at v takes `v³ dκ/ds` |
| braking | `v²/(2 a_brake) + v τ + margin ≤ free distance ahead` | §11: can always stop before the props reach a wall straight ahead; found by ray-marching the exact wall distance |
| opening (optional) | a fixed cap | narrow openings only |

1. **Forward-backward pass.** This is time-optimal path parameterisation
   (Pham & Pham's reachability analysis, reduced to one path coordinate):
   `v[i+1]² ≤ v[i]² + 2 a Δs` forwards and `v[i]² ≤ v[i+1]² + 2 b Δs` backwards.
   Here `a` and `b` are the acceleration and braking limits, further limited
   by `a_long² + a_lat² ≤ a_horizontal²`.
2. **Jerk.** Speed over time is averaged over `T = (a_acc + a_brake)/j_max`
   seconds. That provably keeps acceleration within its limits and
   along-path jerk ≤ `j_max`, and it smooths the take-off and landing
   ramps. Averaging raises the slowest points above their caps, so those caps
   are lowered and the profile is redone until nothing is over. Lateral jerk
   (both `2vaκ` and `v³ dκ/ds`) is measured and lowered the same way.
3. **`check_profile`** re-measures everything on the result, at the
   interface's 10 Hz. It never assumes the construction worked.

An earlier jerk method (erode, then average) also had a guarantee, but cost
10 s on this course and was slower at 2 m/s than at 1 m/s. It was dropped.

### Follow

The kit's interface sends PX4 a position *or* a velocity, never both, so the
trajectory's velocity and acceleration can't be passed directly.
The kit's interface sends PX4 a position *or* a velocity, never both, so the
trajectory's velocity and acceleration can't be passed directly.
By default the Follower publishes the trajectory's plain poses. The lead
formula from `contracts/ros-graph.md`,
`pose + v/0.95 + a/(0.95·1.8)` (`MPC_XY_P`, `MPC_XY_VEL_P_ACC`), is marked
withdrawn there ("not an alternative implementation path"), so it is off by
default and stays only as an explicit opt-in (`FollowerConfig(lead=True)`)
for offline research. The Follower always adds:

* **No unsafe target.** The interface keeps resending the last pose if our
  node dies, and PX4 flies straight to it. So the line from the drone to the
  pose actually published must be clear. If it is not - the drone drifted
  round an inside corner and every approach is blocked - the reference is
  not commanded: the Follower freezes the reference clock, holds where the
  drone is, and sets the `no_safe_target` flag.
* **Delay compensation.** The drone's position is predicted forward 68.7 ms
  using odometry velocity, and the lead looks a further half interface tick
  (0.05 s) ahead.
* **Progress governor.** The reference slows once the drone is more than
  0.15 m behind it and stops at 0.50 m, so it never runs away.
* **Heading** looks 0.8 m ahead (field guide), is locked to the axis inside
  openings, and turns at most 1 rad/s.

With the lead opted in, it is also **collision-checked and bounded**: cut
back until the straight lines from the reference, and from the drone, to the
led pose are clear, and capped at 2.5 m. This cuts back about 30% of poses.

`lead_mode="path"` (also opt-in) publishes the trajectory's own position
`1/MPC_XY_P` seconds ahead. That expands to `pose + v/kp + a/(2kp²)`, the
formula to second order (0.554 s² vs 0.585 s²), and stays on the path. It was
about 1 s slower in simulation.

The Follower works in course-frame `(east, north)`. The ROS node, not yet
written, must apply `contracts/frames.md`'s transform to the `odom` frame.

### The simulator

`sim.py` models the chain from our node to the airframe:
- our node at 50 Hz, reading odometry delayed 68.7 ms (organisers' measurement);
- the interface forwarding at 10 Hz (its `setpoint_rate_hz`);
- PX4's position P and velocity PID with drone 1's parameter values;
- the `MPC_XY_VEL_MAX` speed cap and a 25° tilt limit;
- the airframe reaching commanded acceleration with a 0.1 s lag. **This lag is
  a guess.** Results held across 0.05–0.2 s.

Not modelled: height, yaw dynamics, prop wash, vision dropouts, EKF2 error.

## Limits, and what needs a real flight

Every vehicle limit lives in `limits.py` with its source. **None has been
validated in flight.** Where sources disagree, the default is the most
restrictive.

| Limit | Default | Source | Calibrate by |
|---|---|---|---|
| `v_max_mps` | 1.0 | drone 1's params file (`SN0001`): `MPC_XY_VEL_MAX` = 1.0; the baseline file says 12 | read the live parameter |
| `a_lat_mps2` | 1.0 | simulation: 2.0 was no faster (the governor held back more) | cross-track error on corners vs speed |
| `a_acc_mps2`, `a_brake_mps2` | 2.0 | under `MPC_ACC_HOR` = 3 | step tests |
| `a_horizontal_mps2` | 3.0 | `MPC_ACC_HOR` | — |
| `j_max_mps3` | 4.0 | `MPC_JERK_AUTO`; `MPC_JERK_MAX` = 8 saves 2.2 s | tracking error vs jerk |
| `latency_s` | 0.17 | 68.7 ms vision + 0.1 s interface tick | bag: setpoint to motion |
| sim `accel_lag_s` | 0.10 | guessed | bag: commanded vs actual acceleration |

Rule 8.5 forbids changing the organisers' PX4 parameters. These values
describe what the drone does; they configure nothing on it.

## A map from the STEP model

Without a survey, the map comes from the organisers' STEP model
(`dronehacks-course-v3.step`, PR #4). The lidar stream's extractor turns the
STEP into a pair of JSON files: `course-map/1` (walls, openings, tube, rocks,
nets, markings) and `course-rules/1` (checkpoints, start, finish).
`convert.py` turns that pair into this planner's map:

```sh
python -m dronehacks_planner.convert course_v3_map.json course_v3_rules.json --launch-heading 0 1 \
    --route-gate gate_b ends "C2b chicane wall" "A1 approach wall" --out course_v3_step.yaml
```

* It slices across the drone's whole band, 0.7–0.9 m. Walls become
  rectangles, split at openings that cover the band. The tube is cut at its
  narrowest within the band (1.409 m inside, not 1.461 m).
* `--launch-heading 0 1` is `contracts/frames.md`'s "facing north".
* `--route-gate gate_b ends ...` follows `contracts/course.md`: "Gate B, 1.13 m
  gap, C2b to A1". The rules file instead puts Gate B between C2b and
  Barrier 2. The contract wins (AGENTS.md §3.1).

On it, the planner gives 17.46 m through the window (17.49 m on the
provisional map) and 18.75 m via Gate B (18.78 m). The default route is
Gate B; the window needs `PlannerConfig(allow_windows=True)` or
`--allow-windows`. The generated map and its inputs live with the flight
package: `nav_traj/maps/`.

## Input: the map file

`contracts/course.md`'s schema, plus these fields, marked PROPOSED in the
code and fixture. They need a contracts/ PR with a `contracts/CHANGELOG.md`
entry before they are normative (AGENTS.md §3.1).

| Field | Why |
|---|---|
| `launch: {position, heading}` | The schema has no start; `frames.md` puts the origin at the launch mark |
| `default_thickness` | `course.md` gives a default thickness; putting it in the map keeps course numbers out of code (§3.7) |
| `boulders[].height` | A boulder lower than the drone gets a smaller radius: the rangefinder's footprint, 0.8·tan 13.5° + 0.06 = 0.25 m. Unknown height gets the wall radius |
| `goals[].params.axis`, **directed** | Direction of travel. A checkpoint counts one way only ("out of the south end") |
| `goals[].params.bidirectional` | Opt out of the above |
| `params` for `oval` (`length`, `width`) and `rect`/`gate` (`width`, `depth`, `bottom`, `top`) | The schema leaves `params` empty except for the pad |
| `shape: gate` | A plain gap between wall ends (Gate B), crossable off-axis |
| `goals[].group` | Alternatives (window or Gate B); exactly one is flown |

## Generality (rule 9.5)

There are no course coordinates in the code; `test_package_has_no_course_numbers`
enforces this against the fixture. The tests plan, smooth and profile:
- the published layout with boulders moved up to ±15 cm (with the window
  allowed, it switches to Gate B when a boulder blocks the window's exit);
  by default the window is opt-in and the route is Gate B;
- the whole course rotated 90°;
- random serpentine mazes it has never seen.

## Not done yet

1. **The real map.** Slice the organisers' v3 STEP (PR #4) at 0.8 m, then
   survey the course to 5 cm. Until then the fixture stands in.
2. **Contract PRs.** The proposed map fields; the route and trajectory
   formats in `ros-graph.md`; `lead_mode="path"` if flight data favours it.
3. **ROS 2 node.** An ament_python package for `~/team_ws/src`: load the map,
   run `pipeline.fastest`, then run `Follower.update` at 50 Hz on
   `/visual_slam/tracking/odometry` and publish to `/skybeta/setpoint/pose`
   after `offboard_active`, via `frames.md`. Take-off and landing.
4. **Replanning (§10).** `plan(start=...)` and `ProfileConfig.v_start_mps`
   support it, but nothing triggers it yet. Neither does switching
   trajectories smoothly.
5. **Joint geometry-and-time optimisation (§7)** was deliberately not built.
   The speed cap and the clearance budget, not thrust, set the time here, and
   it would need CasADi in a team Docker image.
6. **Maze checkpoint lines** in the map, with whole-drone crossing checks.
7. **The optional return** over the barrier: a 3D plan.
8. **Calibration** of the limits above from a recorded flight.

## Credits

Written during the event (rule 9.1); no earlier code reused. Methods:
- Nav2's inflation layer (the cost shape);
- Pham & Pham, *Time-Optimal Path Parameterization Based on Reachability
  Analysis*, IEEE T-RO 2018 (the forward-backward pass);
- Zhou et al., arXiv:1907.01531 (checking a smoothed curve against obstacles
  rather than trusting the smoothing);
- Qin et al., arXiv:2309.06837 (treating openings as crossing constraints,
  in a simplified form).

Mellinger & Kumar's minimum-snap and Foehn et al.'s time-optimal planning
were considered and not used.

Libraries: numpy and PyYAML; matplotlib for the benchmark figures only.
