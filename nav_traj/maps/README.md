# Course maps

`course_v3_step.yaml` is the map the drone flies when no other map is given.
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
