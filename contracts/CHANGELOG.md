# Contracts changelog

Newest entries first. One entry per contract-affecting change, linking the PR.

## 2026-11-07 — course follower implemented; twist landing (branch `navigation/course-follower`, no PR yet)

Requested by Michael Bar On for tonight's scored runs. `contracts/ros-graph.md`:
the planned Follower now exists as `team_ws/src/dronehacks_course_follower`
(map-planned route, led pose setpoints at its own 50 Hz input rate into the
unmodified 10 Hz interface, look-ahead heading). Flight stays blocked until the
section's props-off cadence/timeout check, which the package README's bench
check includes. `contracts/px4.md`: landing on the pad by a 0.5 m/s downward
twist until PX4's own auto-disarm is the follower's default (`land:=false`
holds over the pad); autonomous return stays blocked.

Why: in the organisers' lab simulator (2026-11-07) their example mission's
step setpoints overshot legs by 38-60 cm and turned in place facing a wall
until cuVSLAM failed, and a pilot landing ends a scored run and forfeits the
landing checkpoint. Alternatives: the manual navigator's 0.12 m-radius pose
steps with the pilot landing (kept on main, unchanged). Evidence: the
follower author's SITL against the organisers' unmodified bridge and
interface; PX4's land detector needs a setpoint descending at
1.1 × `LNDMC_Z_VEL_MAX`. Neither is organizer-confirmed or kit-validated.
No PX4 parameter, allowlist, frame or course-map contract changes.
Affected owners to notify: @moterodiaz (navigation, contracts).

## 2026-11-07 — autonomous repository decisions and guarded merges

[PR #31](https://github.com/moterodiaz/DroneHacks/pull/31):
remove per-PR human approval for code, contract, cross-area and guard decisions.
Require documented reasoning, affected-owner notification, independent exact-head
review, recorded relevant tests and the trusted guarded squash-merge helper.
Bootstrap is restricted to this policy PR and the pinned pre-rollout main
review workflow, with independently authored exact-head/base review evidence.
Review follow-up: preserve change requests across later comment-only reviews,
honor newer non-pass verdicts, bind the tracking comment to the action's
pre-model log, and require independently executed exact-head/base sandboxed CI
tests rather than copied author results. Add explicit intent reconciliation
without retries of uncertain same-head writes. Extend the existing CI job,
not its job count. No flight, organizer or physical-evidence constraint changes.
Also preserve explicit open issue/PR dependencies in the merge gate and local
controller, even when the dependency itself has no blocking label.
Second review follow-up: validate Make entry-point arguments as environment
data without evaluating shell/Make payloads; grant only the Python merge helper,
not a blanket Make command. Remove raw `gh api` from reviewer tools, retaining
read views and the intended review-comment/board operations. Exercise Make
argument attacks and valid dry-run/execute forwarding in the isolated CI image.
Give every merge attempt a unique journal ID so a previously reconciled attempt
cannot hide a later uncertain request at the same head and base.
Same-repository team PRs only; no self-sign-off, force pushes or admin bypass.
Spending, live flight, deployment and access changes remain manual. All physical
validation, organizer restrictions, immutable sources and secrets protections
remain unchanged. Existing holds require individual evidence-based reassessment.
Affected owner: @moterodiaz (all currently assigned areas).

## 2026-11-07 — regenerate course map fail-closed, from CAD-derived goal geometry (PR #22, decision #23 Option 1)

Maintainer authorized the correction in
[decision #23](https://github.com/moterodiaz/DroneHacks/issues/23). No
contract text changed; `contracts/course_v3_layout.yaml` was regenerated and
`course/step_to_map.py` corrected so an unreviewed artifact can no longer be
mistaken for a valid flight-input map:

- `window.clearance_radius` is now emitted as `null` with an explicit
  UNREVIEWED marker instead of the forbidden `0.29` placeholder, so a
  conforming consumer rejects window traversal until a reviewed budget is
  supplied via `--window-clearance` (course.md's no-default rule).
- Every goal parameter is now derived from the CAD model (circle centres and
  radii, end-face ellipse axes, interior opening geometry), not typed
  literals: tunnel axis from the end-face ellipses with the exit derived
  toward the pad (was hard-wired north→south); window axis spanning the
  containing wall's faces and departing toward the pad (was reversed);
  pad/launch/boulder centres from CAD circle centres (were the leftmost
  points); window sill/head from interior z levels (were z-literal filters).
- Generation and self-check validation now run **before** the output file is
  written: a failing check preserves the previous map (exit 1), generation
  errors never write (exit 2). The PASS report prints measured values, and
  focused tests under `course/tests/` cover translation/rotation of the CAD,
  fail-closed rejection of unsupported geometry, and output preservation.

Measured-vs-nominal deltas now recorded by the artifact (course.md: a
surveyed row supersedes the nominal): tunnel `height` is measured **1.8288 m
(72 in)** — the geometry table's tunnel row is 58 × 72 × 60 in with 72 in
vertical, while the course.md schema example's `height: 1.524 # 60 in` was
the tube's passage *length*, not its height. This PR corrects that schema
example in `contracts/course.md` to `height: 1.829 # 72 in` (measured
1.8288, within tolerance), and adds the fail-closed sentence for
`clearance_radius: null` (consumers must refuse the window route, never
substitute a default). Window `side` measures 0.9144 m and sill/head
0.3048/1.2192 m, within tolerance of the nominal 0.914/0.305/1.219 rows.
Window traversal remains blocked pending a reviewed clearance budget and
actual-kit vertical-clearance validation; nothing here is flight acceptance.
Affected owner: @moterodiaz (contracts, course).

## 2026-11-07 — add generated course map YAML (`core/step-to-map`)

Added `contracts/course_v3_layout.yaml`, generated by `course/step_to_map.py`
from `dronehacks-course-v3.step` per the schema in `contracts/course.md`.
No contract text changed; this entry exists only because `contracts-guard`
flags any new path under `contracts/`, generated artifacts included.
`window.clearance_radius` was supplied as a placeholder (`0.29`) via the
script's required `--window-clearance` flag — still needs a reviewed
human safety value before flight, per `contracts/course.md`'s rule against
substituting a geometric default.

## 2026-11-07 — contract the lidar-stack team-side variant graph (PR #27, decision #29)

Maintainer accepted the lidar localization architecture in session per
recommendation A: [decision #29](https://github.com/moterodiaz/DroneHacks/issues/29)
items 1, 2 and 4. `ros-graph.md` gains a "Lidar-stack variant (contracted
team-side nodes, PR #27)" section with the three team-side nodes:
`lidar_localizer` (MS200K lidar revolutions matched to the course map, body
pose in the course frame with covariance), `vo_guard` (withholds untrusted
cuVSLAM poses and feeds ONLY team nodes — it never sits upstream of PX4 or the
organizer's `px4_odom_bridge.py`, which stay exactly as the kit ships them),
and `pose_fusion` (estimates the planar cuVSLAM drift C and sends
C⁻¹-corrected setpoints to `/skybeta/setpoint/{pose,twist}`, slew-limited
0.10 m/s and 3°/s). **PX4's path is unchanged**: the kit bridge and EKF2 are
untouched, and the correction reaches PX4 only through the competitor setpoint
inputs the kit itself provides. MS200K lidar use remains a required per-run
choice with no default, still subject to organizer permission per the
existing blocked-proposal text. Affected owner: @moterodiaz (localization and
navigation). The new head still requires fresh independent review; this is
not merge or flight acceptance.

## 2026-11-07 — reconcile instruction identifiers without merging (PR #5, decision #19)

Maintainer approved the docs-only integration correction in
[decision #19](https://github.com/moterodiaz/DroneHacks/issues/19). Preserve
main's drone-1 isolation, immutable-snapshot boundary and Acro prohibition
(invariant 6), including its reserved invariant 7. Retain this PR's existing
parameter allowlist and map-only geometry constraints unchanged as invariants
8 and 9; update all four contract citations.

This reconciles the sole conflicting path, `AGENTS.md`, with the instructions
already on main. No safety constraint, parameter value, firmware pin, runtime,
snapshot or workflow changes. No merge commit, rebase or history rewrite.
The source catalog remains the evidence path when a branch omits the snapshot.
Affected owner: @moterodiaz (contracts, course, localization and navigation).
The new head still requires fresh independent review and board/thread
reconciliation; this decision is not merge or flight acceptance.

## 2026-11-07 — make remaining review evidence and clearance semantics explicit (PR #5)

Follow-up to the current-head review: record immutable organizer-source links
and Git blob IDs in [the evidence catalog](README.md#organizer-source-evidence),
since the snapshot is absent from this branch. Mark answered day-one questions
as answered rather than leaving Q1/Q2 under an open heading.

Distinguish the window's geometric inradius (`side / 2`) from the planned
robot-centre costmap `clearance_radius`. The inherited 0.29 m value is an
unvalidated design example, not a geometric property or an enabled flight
setting. No consumer implements the old field name; a future map must provide
a reviewed clearance budget. Explicitly block follower cadence selection until
the required props-off validation, without changing the kit's 10 Hz default.

Affected owner: @moterodiaz (contracts, course, localization and navigation).
No snapshot, runtime, allowlist or hardware changes. Merge conflicts with main
are not resolved by this documentation correction.

## 2026-11-07 — reconcile published kit interfaces and frame alignment (PR #5)

Supersedes the earlier ENU/no-rotation startup assumption: distinguish world
from body frames, require measured course→odom and odom→PX4 alignment, and
make the lab-height/pad arithmetic a conditional example, not a kit calibration.
Correct window sill/head comments to floor-relative heights, require its wall
reference and directed traversal axis, and define launch heading clockwise
from course north. Remove the nonexistent tunnel-yaw rule.

Reconcile the ROS graph with the published kit: direct cuVSLAM→organizer bridge,
10 Hz default interface with pose **and** twist, no 50 Hz lead formula or
interface patch. Mark Guard insertion, tunnel vision-loss guarantees and
unsupported landing/disarm/rearm as blocked proposals. Describe the actual
safety-pilot arming/automatic Offboard sequence. The PX4 allowlist and version
pin are unchanged; the running firmware/configuration still needs verification.
Clarify that capture timestamps alone do not justify a zero vision delay;
the simulator candidate still needs measured kit latency/hover confirmation.

Evidence: read-only organizer README and bridge/interface/frame source in
`drone-snapshot/home/dronehacks/skybeta/`. Earlier x500 findings below remain
historical simulator evidence, not approval to deploy or guaranteed kit safety.
Affected owners: @moterodiaz (contracts, course, localization, navigation and
docs). Maintainer authorized finishing this correction in the session; new
head still requires independent review. Props-off alignment, vision-loss and
flight acceptance checks have **not** been performed by this documentation PR.

## 2026-11-06 — ENU competitor output, course goal schema, Q1/Q2 answered
Fixes PR #5's two open review findings: `contracts/frames.md` now states the
ENU output path to `/skybeta/setpoint/pose` (the NED/FRD transform is
log/bench-only — the kit's `skybeta_flight_interface` does that conversion
internally); `contracts/course.md`'s goal schema now defines `params` for
`tunnel`, `window`, and `launch`, plus the reject-if-missing rule.

### Findings
- 2026-11-06 — Q1 and Q2 (below) are answered by the kit's own published
  docs, mirrored read-only into `drone-snapshot/home/dronehacks/skybeta/`
  (`README.md`, `CLAUDE.md`) — not a live organiser answer, but the
  organisers' own source of truth. **Q1:** the kit's `px4_odom_bridge.py`
  consumes `/visual_slam/tracking/odometry` directly; there is no guard
  node upstream of it, and team code never touches `/fmu/in/*`. **Q2:**
  no — `skybeta_flight_interface` must not be modified (stated explicitly
  in `skybeta/README.md`). The 50 Hz led-pose design is dropped; team code
  publishes ENU setpoints to `/skybeta/setpoint/pose`/`/skybeta/setpoint/twist`
  at whatever rate `skybeta_flight_interface` expects (see
  `contracts/ros-graph.md`).
  The snapshot is not present in this branch: use the immutable archived
  sources and hashes in [the evidence catalog](README.md#organizer-source-evidence).

## 2026-11-06 — first four contracts, extracted from the field guide
Added `contracts/frames.md`, `contracts/course.md`, `contracts/px4.md`,
`contracts/ros-graph.md`. Extracted from `docs/field-guide.html` (the team's
background/reasoning doc for Stanford DroneHacks, committed the same PR).
Notified: all area owners (none claimed yet — see docs/BOARD.md).

### Findings
Dated, `measured` on the x500 stand-in sim unless noted. A finding is a
contract too — once recorded here, code may rely on it.

- 2026-11-06 — cuVSLAM answers confidently on a blank view (hundreds of
  features, "standing still") instead of reporting loss. Withholding blank
  frames alone still produced a 2.2 m jump on return (149 cm rms); adding
  re-anchoring onto PX4's estimate brought it to 24.5 cm. See
  `contracts/ros-graph.md` guard section.
- 2026-11-06 — stereo-only cuVSLAM beats its inertial mode in flight:
  inertial mode's in-flight IMU init blocks tracking (one break per run);
  stereo-only ran 5 of 5 clean. PX4's EKF2 already does the inertial fusion.
- 2026-11-06 — latency is the dominant cost, and the organisers'
  `EKF2_EV_DELAY 70` double-counts it (their bridge already stamps at
  capture). With 50 ms added latency, `70` touched a wall on 4 of 5 runs;
  keep `0` and confirm from the first hover's log. See `contracts/px4.md`.
- 2026-11-06 — corners set the time, not top speed: 1.0, 1.5 and 2.0 m/s all
  took 27–31 s launch to pad; 1.0 m/s was cleanest (0 of 5 touched). Fly at
  the kit's 1.0 m/s cap.

### Day-one question status

These are the field guide's original question numbers, with current status.
Published-source answers are not a claim of a live organizer decision.

1. **Answered by published source:** the organizer bridge consumes cuVSLAM
   directly; neither a team bridge nor upstream Guard is authorized. See the
   Q1/Q2 finding above and [source evidence](README.md#organizer-source-evidence).
2. **Answered by published source:** do not fork the flight interface or
   require a 50 Hz upgrade. The supplied pose/twist path is the chosen scope;
   alternative advanced-track permission is not inferred.
3. **Partly answered:** the safety pilot arms on RC, then the interface may
   enter Offboard after the specified ground sequence. Autonomous landing,
   disarm/rearm and return support remain blocked (`contracts/px4.md`).
4. **Open:** can stereo depth run beside cuVSLAM in the issued launch?
5. **Documented, verify issued kit:** the recorded PX4 cap is 1.0 m/s
   (`contracts/px4.md`); this is not permission to change it.
6. **Open:** verify actual firmware with the organizer. Older simulator notes
   name v1.16.1; the archived fleet README names v1.16.2.
7. **Open:** may the physical course be tape-surveyed and that map used
   under rule 6.4?
8. **Open:** how dark is the tunnel and what is its floor? No safe exposure,
   vision-loss duration or Guard threshold is established by this question.

## 2026-11-06 — contracts layer established
No contracts exist yet. This file and `contracts/README.md` establish the rule
(contracts beat code; CI's `contracts-guard` enforces this file gets touched
whenever `contracts/` does) ahead of the first real contract.
