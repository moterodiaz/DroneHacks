# Course maps

`course_v3_step.yaml` is the map the drone flies when no other map is given.
It's in `contracts/course.md`'s format, generated from the organisers' STEP
model. **It isn't surveyed:** the real course may sit a few centimetres off
its model (`course.md`, "Layout tolerance"). A surveyed map placed at
`~/team_ws/course_map.yaml` on the drone takes precedence (`docs/deploy.md`).

How it was made:

1. **STEP:** `dronehacks-course-v3.step` (PR #4), SHA-256 `b0e52800…4d4500`.
2. **Extraction:** `source/course_v3_map.json` and `source/course_v3_rules.json`.
   These were produced from that STEP by the lidar stream's extractor
   (`course/12_extract_course_map.py` on `core/port-depth-camera-testing`,
   commit `324ff23`). The map JSON records the STEP's SHA-256, and it matches.
   The files are copied here unchanged.
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
