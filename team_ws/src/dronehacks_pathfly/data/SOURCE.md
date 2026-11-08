# Path data source

`path_dense.csv` and `path_waypoints.csv` are copied unchanged from
<https://github.com/jsun4231/path-planning> at commit
`e0f22d8b422049d1e71a1ba7b68d4f0e8232ed53` ("Add north wall to map; plan for
fixed east-facing drone"). That repository's `plan_path.py` generated them from
its `STEP MAP.STEP`.

| File | SHA-256 |
|---|---|
| `path_dense.csv` | `7d5a9aa6ae93b7c09010e9909bc5b368d75a1755ba40b36c9d345f156da1c0cc` |
| `path_waypoints.csv` | `a70bf44fdcc48f8b722d38420a511d760d2de1c422fcf749adff2408efcb0a45` |

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
| `STEP MAP.STEP` | `372e9b4795d95efeee9dda353ff59ad465a5670e98a41737ccdfb0a2eca6a9fe` |
| `plan_path.py` | `5b2a51ea5c1132066b4b2baa4175f9ec5b031d80ec061bc21664c29995852aee` |
| `path.png` | `1c9ed9eb0bf92b06db48ce5a18f536ddedab8b91f11fe47bc7d7570832d1524a` |
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

**Margins.** `margin_m` is the planner's gap between the drone's real
0.50 × 0.40 m footprint and the nearest wall at that point. The footprint is
held at a **fixed heading, nose east (+x)**: 0.40 m along x and 0.50 m along y.
The margins hold only if the drone flies the whole path facing east. The
flight node's defaults do (`start_yaw_deg: 0.0`, `hold_heading: true`), and it
logs a warning otherwise. The node slows down where the margin is small.

- The tightest stretches are 0.079–0.082 m:
  - about 0.8 m of the top corridor under the north wall, around
    x = 1.8–2.3 m, y ≈ 5.22 m;
  - the goal.
- `path_dense.csv` is the smooth path the node flies (0.10 m spacing).
- `path_waypoints.csv` is the straight-line simplification. It is kept for
  reference and is not flown.

To fly a re-planned path, replace `path_dense.csv`, or point the `path_csv`
parameter at a new file. Keep the same columns, and update this note and the
point count and length in `test/test_mission.py`.
