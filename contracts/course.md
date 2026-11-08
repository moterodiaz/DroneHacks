# Course

Reasoning and background: `docs/field-guide.html` §course and §numbers. This
file is the normative statement; where they disagree, this file wins
(AGENTS.md §3.1).

## Geometry

| element | value | source |
|---|---|---|
| Net | 6.096 × 6.096 m (20 ft), 3.048 m tall | `organisers` |
| Walls | 1.727 m (68 in) tall, 0.102 m thick | `organisers` |
| Walls added 2026-11-07 | 1.727 m tall; one 0.100 m thick, one a 0.350 m deep block (see Layout revision) | `team` |
| Corridors | 1.02–1.07 m; narrowest passage 0.93 m | `organisers` |
| Tunnel | oval, 58 × 72 × 60 in, against the east net | `organisers` |
| Window | 0.914 m square, 0.305–1.219 m up, in wall C2a | `organisers` |
| Gate B | 1.13 m gap, C2b to A1 | `organisers` |
| Boulders | five, 0.457 m across, 0.533 m tall | `organisers` |
| Landing pad | 1.524 m (5 ft) across, centre `(−2.042, −0.792)` | `organisers` |
| Launch mark | `(−2.515, 0.914)`, facing north | `organisers` |

Coordinates are course-frame `(east, north)` metres, origin at the floor
centre. See `contracts/frames.md` for the measured transform into competitor
odom-frame setpoints; the kit alone performs the PX4 conversion.

`team` rows come from the team's 2026-11-07 update of the course CAD and its
checkpoint list (@jsun4231), described under Layout revision below.

## Layout revision 2026-11-07

The course model is `dronehacks-course-v3.step` at the repository root,
SHA-256 `2194664e852e341f86fb63a19ecc4f8f0ef34da6a51867e49beb3157cb90b876`
(a SolidWorks export; it replaces the 2026-10-27 Onshape export
`b0e52800ad77e7f173b328601e466083aad32a163ec035b8090e43bb624d4500`). Source:
`team`. Changes from the 2026-10-27 model:

- **Removed:** the maze dividers M1, M2 and M3 (hidden in the CAD assembly,
  so absent from the export).
- **Added**, both in CAD part `Part67`, floor to wall top (1.727 m):
  - `Part67 / Boss-Extrude1[1]`: a block, footprint
    `(−3.049, 2.698)`–`(−1.249, 3.048)` (1.800 × 0.350 m), against the west
    and north nets;
  - `Part67 / Boss-Extrude1[2]`: a diagonal wall 0.100 m thick, footprint
    `(−2.167, 0.508)`, `(−2.020, 0.508)`, `(−0.659, 1.775)`, `(−0.727, 1.848)`,
    running north-east from Barrier 1's north face at 42.9° from east.
- The narrowest gap the new walls make is **0.998 m**, between the two of them
  near `(−0.99, 2.27)` (`derived` from the CAD at 0.8 m).
- The net roof is not in the export either. The net is unchanged: its sides
  are still 3.048 m tall (Net row).
- Every other part is where it was, to 0.1 mm.

## Checkpoints

| order | name | line | points |
|---|---|---|---|
| ~~1~~ | ~~First corridor~~ | **removed 2026-11-07** (`team`) | — |
| 2 | Maze | leave the maze through Gate A: the line `(0.3048, 2.1209)`–`(0.3048, 3.048)` | 7 |
| 3 | Tunnel | out of the south end of the dark tunnel | 10 |
| 4 | Chicane | into the landing area through Gate B or the window, without landing | 7 |
| 5 | Landing | land and disarm with the whole drone inside the pad's outer ring | 7 |

A checkpoint counts only when the **whole drone** crosses its line, in order.
Source: `organisers`; the 2026-11-07 changes: `team`. Checkpoints 2–5 are
unchanged and keep their numbers; the four that remain are worth 31 points.
Gate A was the gap between M3's north end and the north net. M3 is gone, so
the same line is now given explicitly, on M3's old centreline. Nothing stops a
route passing south of it any more: a planner must route through the line,
not just through open space.

## Derived error budget

Drone 0.37 × 0.48 m, props reach 0.28 m from centre. In the 0.93 m narrowest
passage that leaves **±0.18 m** for all error (planner + estimator + control).
This is derived from the geometry row above, not an independent fact — if the
corridor or drone dimensions change, re-derive it. The 2026-11-07 layout's new
0.998 m gap leaves ±0.22 m; the 0.93 m figure still binds elsewhere (Gate B's
neighbourhood measures 0.936 m in the CAD).

## Map-file format (planner input)

The planner's only input, and the only thing that changes on a new layout:
the course model above, sliced at 0.8 m (flight height) into wall segments,
plus the net and boulder footprints. The planner must read this map and
derive goals (tunnel middle, window, pad) from it — never from typed
coordinates. This is rule 9.5's generality requirement
(`organisers`) made concrete; see also AGENTS.md §3.9.

### Schema (YAML)

```yaml
origin: [<east_m>, <north_m>]  # course-frame origin, metres
slice_height: <m>               # height at which walls are sliced (planner flight height)
segments:                       # wall and net footprints at slice_height
  - start: [<east_m>, <north_m>]
    end:   [<east_m>, <north_m>]
    thickness: <m>              # wall thickness (default 0.102 m from geometry table)
boulders:                       # obstacle discs
  - centre: [<east_m>, <north_m>]
    radius: <m>
goals:                          # planner derives waypoints from these; no typed coords in code
  - name: <tunnel|window|pad|launch>
    shape: <oval|rect|disc|point>
    centre: [<east_m>, <north_m>]
    params: {}                  # shape-specific dimensions, required per name below
```

A map missing any of `tunnel`, `window`, or `pad` is a **hard parse error** —
the planner must refuse to run, never substitute a default coordinate
(AGENTS.md §3.9). `launch` is required too, but only as the path origin and
yaw reference, not a routed waypoint.

### Required `params` by goal name

- **`tunnel`** (shape `oval`): straight-line axis through the tube for
  route/heading planning. This does not establish safe vision-loss traversal;
  that remains blocked by the validation requirements in `contracts/ros-graph.md`.
  ```yaml
  params:
    axis_start: [<east_m>, <north_m>]  # tube entrance centre
    axis_end:   [<east_m>, <north_m>]  # tube exit centre
    width:  1.473   # 58 in, Tube oval cross-section
    height: 1.829   # 72 in vertical; the 60 in dimension is the passage length, carried by the axis endpoints
  ```
- **`window`** (shape `rect`): a square portal, not an obstacle disc.
  Its geometric inscribed-circle radius is `side / 2`, separate from robot
  clearance. The traversal axis is a directed
  normal through the opening centre in course east/north coordinates.
  `wall_segment_index` identifies the containing segment in `segments`;
  the opening is a portal in that wall, not an additional obstacle disc.
  ```yaml
  params:
    wall_segment_index: <nonnegative_int> # index into segments
    axis_start: [<east_m>, <north_m>]     # approach side of the opening
    axis_end:   [<east_m>, <north_m>]     # departure side; not equal to start
    side: 0.914            # 0.914 m square opening
    sill_height: 0.305     # metres above the floor where the opening starts
    head_height: 1.219     # metres above the floor where the opening ends
    clearance_radius: <m>  # reviewed robot-centre exclusion radius, not opening radius
  ```
- **`pad`** (shape `disc`): see worked example below.
- **`launch`** (shape `point`): the drone starts here, facing the heading
  below. Not a planner waypoint. Use the measured course-to-odom transform
  in `contracts/frames.md`; arming does not reset the vision origin or
  prove its alignment.
  ```yaml
  params:
    heading_deg: 0   # clockwise from course north (east = 90 degrees)
  ```

For the documented square opening, `0.914 / 2 = 0.457 m` is the geometric
inradius (`derived` from the organizer dimensions above). The inherited
0.29 m costmap value is a design example from the earlier contract revision,
not a validated flight setting. With the documented 0.28 m prop reach it
would add only `0.29 - 0.28 = 0.01 m` radial margin and leave
`0.457 - 0.29 = 0.167 m` centre-position margin on each side of the opening.
Those calculations do not validate the choice. Supply `clearance_radius`
from a reviewed combined airframe/yaw, planner, estimator and control-error
budget; do not substitute 0.29 m or the opening inradius as a default.
Window traversal remains blocked until that budget and vertical clearance
are validated on the actual kit. A generated map without a reviewed budget
emits `clearance_radius: null`; consumers must treat that as
traversal-blocked and refuse the window route — never substitute a default.

Every listed goal parameter is required. Reject an invalid wall index,
non-finite geometry, zero-length traversal axis, or an axis that does not
cross the opening centre normally to the referenced wall. A flight slice
must lie between floor-relative sill/head heights with airframe and tracking
clearance; do not infer a traversable portal from the centre height alone.

### Worked example — pad goal

```yaml
goals:
  - name: pad
    shape: disc
    centre: [-2.042, -0.792]  # from geometry table (course-frame east, north)
    params:
      radius: 0.762           # 1.524 m diameter / 2
```

## Layout tolerance

The organisers state the layout "may change slightly" (`organisers`). Survey
the real course to 5 cm and plan on the survey, not the published file: a
course 10 cm off its file touched a wall on 5 of 5 runs (`measured`). A
`surveyed` row supersedes the matching `organisers` row above and gets a
`contracts/CHANGELOG.md` entry when it does.
