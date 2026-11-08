# Course maps

`course_v3_step.yaml` is the map the drone flies when no other map is given.
The window flight lines fly `course_arena_2026-11-08.yaml` instead: the course as
the team taped it (below).
It's in `contracts/course.md`'s format, generated from the course STEP
model. **It isn't surveyed:** the real course may sit a few centimetres off
its model (`course.md`, "Layout tolerance"). A surveyed map placed at
`~/team_ws/course_map.yaml` on the drone takes precedence (`docs/deploy.md`).

## The 2026-11-07 layout

The course changed on 2026-11-07. The team's updated CAD
(`dronehacks-course-v3.step`, a SolidWorks export, SHA-256
`2194664e…cb90b876`) drops the maze dividers M1–M3 (hidden in the assembly)
and adds two walls in one part, `Part67`:

* `Part67 / Boss-Extrude1[1]`: a 1.8 × 0.35 m block in the north-west corner,
  against the west and north nets (a `block` in the extraction);
* `Part67 / Boss-Extrude1[2]`: a 0.10 m thick diagonal wall, about 1.97 m
  long, from Barrier 1 north-east to (−0.69, 1.81).

The narrowest new gap is 0.998 m, between the two. Every other part is where
it was. The net roof is not in the export either; the map never used it.

Checkpoint 1 was removed. Checkpoints 2–5 are unchanged and keep their
numbers. CP2 "Gate A" was the gap between M3's north end and the north net;
with M3 gone it is the same line written out as a segment,
`[[0.3048, 2.1209], [0.3048, 3.048]]` (M3's centreline). Nothing physical
stops a route passing south of it now, so `convert` turns it into a `gate`
goal, `cp2_gate_a`, and the planner flies straight through it, east towards
the tunnel. The route is `cp2_gate_a > tunnel > gate_b > pad`, 15.6 m (the
window route, opt-in, 14.4 m).

The previous map (2026-10-27, Onshape export `b0e52800…4d4500`) is kept as
`test/fixtures/course_v3_step_2026-10-27.yaml` for one regression test.

## How it was made

1. **STEP:** `dronehacks-course-v3.step` at the repository root, SHA-256
   `2194664e852e341f86fb63a19ecc4f8f0ef34da6a51867e49beb3157cb90b876`.
2. **Extraction:** `source/course_v3_map.json`, produced from that STEP by the
   course extractor `depthcam/course/12_extract_course_map.py`, with its
   SolidWorks-export fixes (product names, B-spline edges, conics fitted to
   B-spline loops, a fixed tube end order; branch
   `course/course-v3-layout-change`). The map JSON records the STEP's SHA-256,
   and it matches. `source/course_v3_rules.json` is the checkpoint file above
   (no CP1; CP2 a segment).
3. **Conversion**, from this directory:

   ```sh
   PYTHONPATH=../../planner python -m dronehacks_planner.convert source/course_v3_map.json source/course_v3_rules.json \
       --launch-heading 0 1 --route-gate gate_b ends "C2b chicane wall" "A1 approach wall" --out course_v3_step.yaml
   ```

   * `--launch-heading 0 1`: `contracts/frames.md`, "facing north up the first corridor".
   * `--route-gate gate_b ends ...`: `contracts/course.md`, "Gate B, 1.13 m gap,
     C2b to A1". The rules file puts Gate B between C2b and Barrier 2; the
     contract wins.

`test/test_maps.py` regenerates the map and checks it matches this file
byte for byte. It also checks the map plans and flies in the dry run.
`.gitattributes` keeps the bytes as generated, so the SHA-256s in the header
stay true.

## The taped arena (2026-11-08)

`course_arena_2026-11-08.yaml` is the course as the team measured it on
2026-11-07/08. It is Michael Bar On's map from `core/flight-iteration`
(`team_ws/src/dronehacks_course_follower/maps/course_map.json` at b5d640e,
copied byte-exact as `source/course_map_flight-iteration_b5d640e.json`), and
the team (@jsun4231) takes it as the real arena wherever it differs from the
STEP. The two window flight lines (`launch/traj_nav_competitor_window_*.launch.py`)
fly it. The default launches still fly the STEP map.

It differs from the STEP in the following ways. `source/arena_2026-11-08_edits.json`
lists each change with its source and whether it was taped, estimated or assumed.

* **North wall:** a wall 24 in (0.61 m) inside the north net, where skybeta-11 hit
  it. The map assumes it runs the whole north side, south face at y = 2.4384.
* **Diagonal wall:** the taped one, (−1.798, 0.508) to (−0.813, 1.648), 0.10 m
  thick. The CAD's longer diagonal and its north-west block are gone.
* **Window:** open 0.914–1.549 m above the floor (taped), not 0.305–1.219 m. The
  map is sliced at its middle, **1.23 m**, so the whole course is flown at that
  height.
* **Pad:** centred at (−2.1336, −1.8288), about a metre south of the STEP's.
* **Rocks:** the one south of the tunnel's mouth is gone. One sits against the
  east net, one against the south net east of the window, and the one west of the
  window sits a foot from the south net. The flight lines ignore them
  (`avoid_boulders: false`).
* **CP2 Gate A (assumed):** only 0.32 m of the published line is south of the new
  north wall, so no drone fits. The edits keep the line where it was (x = 0.3048,
  where the maze guideline ends) and run it across the whole corridor, from
  Barrier 3's face to the north wall. Every route to the tunnel crosses it, so
  it is no goal (`planner_goal: false`). `test/test_maps.py` checks that the
  whole drone crosses it. **Ask the organisers how Gate A is scored now.**

The route is `tunnel > window > pad`, 13.31 m: north-east along the diagonal,
round its top end, east to the tunnel, south through it, west through the window,
then onto the pad. The closest it comes is 0.341 m at the diagonal's top end,
and the window pass keeps 0.137 m over the inscribed radius.

Made from this directory with:

```sh
PYTHONPATH=../../planner python -m dronehacks_planner.convert source/course_v3_map.json source/course_v3_rules.json     --edits source/arena_2026-11-08_edits.json --launch-heading 0 1 --slice-height 1.23     --route-gate gate_b ends "C2b chicane wall" "A1 approach wall" --out course_arena_2026-11-08.yaml
```

`test/test_maps.py` regenerates it, and checks that every taped number in the
edits matches Michael's map.
