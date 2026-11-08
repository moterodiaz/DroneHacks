# Architecture diagrams

Standalone HTML diagrams of the tech stack, drawn from `origin/main` at `3728cf173d461d7dfd7ad8c6f6954a292f5cb785`. Open any file in a browser. Component sources cite files and lines at that commit, so a diagram can go stale: check the cited lines before relying on it after later merges.

| Page | Shows |
|---|---|
| [`index.html`](index.html) | The one-page summary: methods per layer, test coverage, paper-derived optimizations, next checks. |
| [`unified-architecture.html`](unified-architecture.html) | Sensors to PX4: the organizer kit path, the mission node, and the lidar-fusion branch, with topic names. |
| [`planning-pipeline.html`](planning-pipeline.html) | STEP CAD to a time-indexed trajectory, with the algorithm at each stage. |
| [`planner-designs.html`](planner-designs.html) | The three planner designs in the repo and which one a ROS node uses. |
| [`localization-fusion.html`](localization-fusion.html) | cuVSLAM, the Guard, lidar matching and the drift-transform EKF. |
| [`tests-ci.html`](tests-ci.html) | Test counts per suite, and which suites CI runs. |
| [`optimizations.html`](optimizations.html) | Timeline of techniques borrowed from MonoRace, SkyDreamer and VIO work, with the measured result for each. |

Miro overview board: <https://miro.com/app/board/uXjVEQ_1oiA=/> (same facts, editable shapes, one frame per topic).

## Reading the numbers

- Test counts are `def test_` counts, not pytest items.
- Latency, drift and speed figures come from simulation or bench runs. None come from a flight on the issued kit; `docs/PRD.md` says the same of its simulator figures.
- Items marked proposed or rejected have no code on `main`.
