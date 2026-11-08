# Repository layout

What each top-level folder on `main` is for. File counts are left out on purpose: run
`git ls-files | cut -d/ -f1 | sort | uniq -c` for the current numbers.

| Path | Remit |
|---|---|
| [`contracts/`](../contracts/) | The interface rules code must follow (AGENTS.md §3.1: contracts beat code): PX4 parameter allowlist, course map format and the v3 layout, frames, ROS graph, changelog. |
| [`team_ws/src/`](../team_ws/src/) | ROS 2 packages that go into `~/team_ws/src` on the drone. `dronehacks_nav`: the navigation node forked from the kit's square mission. `dronehacks_manual_maze_tunnel_chicane_gateb_landing`: the full-course mission, with waypoints derived from the course map. |
| [`core/`](../core/) | Pure navigation primitives, no ROS: frames, grid, costmap, planner, trajectory. |
| [`planner/`](../planner/) | `dronehacks_planner`: course map in, flyable trajectory out (route, clearance, speed limits, follower), pure Python. Not yet flown or wired into ROS; see its README. |
| [`course/`](../course/) | `step_to_map.py`: generates the course map YAML from the organisers' STEP model. |
| [`depthcam/`](../depthcam/) | OAK-D S2 localization research: camera checks, cuVSLAM, pose fusion, the lidar stack, methods, datasets and their tests. See its README. |
| [`drone-snapshot/`](../drone-snapshot/) | Read-only copy of what came preloaded on the drone's Jetson: the organisers' skybeta stack and `etc/skybeta/drone.env`. Never edited. |
| [`tests/`](../tests/) | Repository-level tests: core, course map, grid, and the merge gate (`test_merge_reviewed.py`). |
| [`docs/`](./) | Board workflow, contributing rules, the PRD, the field guide, and the dev log in `history/`. |
| [`.github/`](../.github/) | CI, the board label workflow, the Claude reviewer, issue sweep, the guarded merge script, CODEOWNERS, and PR and issue templates. |
| Root files | `AGENTS.md` and `CLAUDE.md` (rules for agents), `DRONEHACKS.md` (event rules and hardware), `dronehacks-course-v3.step` (the course CAD), `Makefile` (board and merge commands), `.gitignore`, `.claude/settings.json`. |

Add a row when a new top-level folder lands; keep each remit to one line.
