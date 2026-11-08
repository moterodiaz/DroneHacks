import sys, json, math
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
pkg, out = Path(sys.argv[1]), sys.argv[2]
sys.path.insert(0, str(pkg))
from dronehacks_course_follower.planner import load_course_map, plan_flight, course_grid
m = load_course_map(pkg / "maps" / "course_map.json")
pc = plan_flight(m, allow_window=True, tunnel_lead_m=0.0)
g = course_grid(m, boulder_margin_m=0.0)
cost = g.cost; rows, cols = cost.shape; res = 0.05
x0, y0 = g.origin_m
fig, ax = plt.subplots(figsize=(6, 6), dpi=200)
leth = np.isinf(cost)
ax.imshow(np.where(leth, 1.0, np.nan), origin="lower", cmap="Reds", vmin=0, vmax=4, alpha=0.45,
          extent=(x0 - res/2, x0 + (cols-0.5)*res, y0 - res/2, y0 + (rows-0.5)*res), interpolation="nearest")
for x1, y1, x2, y2 in pc.walls: ax.plot([x1, x2], [y1, y2], color="#222", lw=1.6, solid_capstyle="butt")
for bx, by, r in m.get("boulders", []): ax.add_patch(plt.Circle((bx, by), r, color="#7a4a2a"))
pad = m["goals"]["pad"]["at"]; ax.add_patch(plt.Circle(pad, 0.762, fill=False, color="#1F3A5F", lw=1.2))
r = np.array(pc.stop_route)
r = np.vstack([[m["launch"][0], m["launch"][1]], r]) if np.hypot(*(r[0]-m["launch"][:2])) > 0.05 else r
ax.plot(r[:, 0], r[:, 1], color="#1F3A5F", lw=2.0, marker="o", ms=3.5)
for i, (x, y) in enumerate(r[1:], 1): ax.annotate(str(i), (x, y), xytext=(4, 4), textcoords="offset points", fontsize=7, color="#1F3A5F")
lx, ly, _ = m["launch"]; ax.plot(lx, ly, marker="^", color="#111", ms=8)
ax.text(lx + 0.12, ly - 0.05, "launch", fontsize=8)
ax.text(pad[0] + 0.2, pad[1] + 0.45, "pad", fontsize=8, color="#1F3A5F")
t = m["goals"]["tunnel"]["at"]; ax.text(t[0] - 0.3, t[1] + 0.55, "tunnel", fontsize=8, rotation=90)
w = m["goals"]["window"]["at"]; ax.text(w[0] + 0.12, w[1] - 0.42, "window", fontsize=8)
b = m["bounds"]; ax.set_xlim(b[0] - 0.05, b[1] + 0.05); ax.set_ylim(b[2] - 0.05, b[3] + 0.05)
ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
for s in ax.spines.values(): s.set_visible(False)
fig.tight_layout(pad=0.1); fig.savefig(out)
print("holds", len(r), "window", pc.window_flown, "length %.1f m" % float(np.sum(np.hypot(*np.diff(r, axis=0).T))))
