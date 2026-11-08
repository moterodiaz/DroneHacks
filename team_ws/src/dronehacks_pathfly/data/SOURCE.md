# Path data source

`path_dense.csv` and `path_waypoints.csv` are copied unchanged from
<https://github.com/jsun4231/path-planning> at commit
`11db65f0267135991c208ff3338d2588e8e80922` ("Tighten tunnel walls in the map;
plan for maximum margin"). That commit was made locally on 2026-11-08 and may
not be pushed yet; its parent is e0f22d8. That repository's `plan_path.py` generated them from
its `STEP MAP.STEP`.

| File | SHA-256 |
|---|---|
| `path_dense.csv` | `63f863e71e06e3dcc00221d1b5195f5af7aa4425f91dfb94f1985ec3c140d0a3` |
| `path_waypoints.csv` | `066ab76a8019b85c73d2391f6edb30da25a7a429a6c200ad05d4496c1c16b4be` |

**The whole source repository at that commit** is copied unchanged into
[`../path_planning/`](../path_planning/): `plan_path.py`, `STEP MAP.STEP`,
`path.png`, both CSVs and its `.gitignore`. You can re-run or audit the
planner from there:

```bash
cd path_planning && python3 plan_path.py
```

That needs NumPy and Matplotlib, and it rewrites the CSVs and `path.png` in
place.

| `path_planning/` file | SHA-256 |
|---|---|
| `STEP MAP.STEP` | `10dbd45f663b3105de6fb9d989837de700e4a01489ff4c1b4bff9d4f3f5a5519` |
| `plan_path.py` | `c741e0d7fc8157e0e406e0db4cb679a452ee3aff10bb90e38d3e59374bb55215` |
| `path.png` | `1e7e85982288d6e0fc29b0e6a9ac43811bce7b9e9cbc84c9a935643dadda3bef` |
| `.gitignore` | `862263fa1f46c20f0d1e4dac5ffcc75abd55c08211b2c3864c5f8764b9d87793` |

The two CSVs there are byte-identical to `data/`; a test checks this.
Every file in `path_planning/` has the same git blob ID as in the source
repository (`git ls-files -s` there). Its files are stored with LF line
endings, and `.gitattributes` keeps them byte-exact on checkout.
`path_planning/` is not installed onto the drone (`setup.py` packages only
`data/`).

**Frame.** Both files use top-down metres with the origin at the arena's inside
south-west corner: x = CAD X (east) and y = −CAD Z (north). The frame is
right-handed with z up.

- Start: the smaller floor marker, (0.580, 4.596).
- Goal: the larger floor marker, (0.900, 1.350).
- In this map the north wall is at y = 5.556 (it was 6.096 in `73fa255`),
  0.54 m inside the outer boundary.
- Since 11db65f the tunnel block's east face is at x = 4.83 (it was 4.63), so
  the tunnel corridor is x 4.83–6.10, 1.27 m wide.

**Objective.** 11db65f plans **margin first**: margin target 1.0 m (above any
margin the arena allows) and weight 60, so the path follows the line of
maximum margin through every corridor and room, and length only breaks ties.
On average the path is within 2 mm of the best margin across its corridor.
It is 13.31 m long, about 1 m longer than e0f22d8.

**Margins.** `margin_m` is the planner's gap between the drone's real
0.50 × 0.40 m footprint and the nearest wall at that point. The footprint is
held at a **fixed heading, nose east (+x)**: 0.40 m along x and 0.50 m along y.
The margins hold only if the drone flies the whole path facing east. The
flight node's defaults do (`start_yaw_deg: 0.0`, `hold_heading: true`), and it
logs a warning otherwise. The node slows down where the margin is small.

- The tightest stretches are the most their corridors allow with the nose
  east:
  - 79–112 mm over path metres 1.4–2.2, under the north wall around
    x = 1.8–2.6 m, y ≈ 5.22 m;
  - 80–117 mm over the last 0.5 m into the goal.
- The tunnel: 0.434 m, centred at x = 5.46.
- `path_dense.csv` is the smooth path the node flies (0.10 m spacing).
- `path_waypoints.csv` is the straight-line simplification. It is kept for
  reference and is not flown.

To fly a re-planned path, replace `path_dense.csv`, or point the `path_csv`
parameter at a new file. Keep the same columns, and update this note and the
point count and length in `test/test_mission.py`.
