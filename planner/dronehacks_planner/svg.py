"""Draw a map and a route as a standalone SVG, for review and the presentation.

North is up. The pale band around each obstacle is the lethal zone: the
drone's centre may not enter it. It is drawn as each segment stroked at its
thickness plus two inscribed radii with round caps, which is exactly the set of
points within the inscribed radius.
"""

from __future__ import annotations

from typing import List, Optional

from .course_map import CourseMap
from .route import PlannerConfig, Route

_PX = 100.0  # px per metre


def render_svg(cmap: CourseMap, route: Optional[Route] = None,
               config: Optional[PlannerConfig] = None, title: str = "") -> str:
    config = config or (route.config if route is not None else PlannerConfig())
    model = config.cost_model()
    min_x, min_y, max_x, max_y = cmap.bounds()
    pad = 0.4
    x0, y0 = min_x - pad, min_y - pad
    w, h = (max_x - min_x + 2 * pad) * _PX, (max_y - min_y + 2 * pad) * _PX

    def X(x: float) -> float:
        return (x - x0) * _PX

    def Y(y: float) -> float:
        return h - (y - y0) * _PX

    out: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.1f} {h + 40:.1f}" '
        f'width="{w:.0f}" height="{h + 40:.0f}" font-family="sans-serif">',
        "<style>"
        ".bg{fill:#fff}.grid{stroke:#eee;stroke-width:1}"
        ".infl{stroke:#f6c9c4;stroke-linecap:round;fill:none}"
        ".inflb{fill:#f6c9c4}"
        ".wall{stroke:#333;stroke-linecap:round;fill:none}"
        ".boulder{fill:#8a7a66}"
        ".pad{fill:none;stroke:#2a7;stroke-width:2}"
        ".goal{stroke:#36c;stroke-width:2;fill:none;stroke-dasharray:4 3}"
        ".route{fill:none;stroke:#1565c0;stroke-width:3;stroke-linejoin:round}"
        ".pass{fill:none;stroke:#e65100;stroke-width:4}"
        ".wp{fill:#1565c0}.start{fill:#000}"
        ".lbl{font-size:12px;fill:#333}"
        "</style>",
        f'<rect class="bg" x="0" y="0" width="{w:.1f}" height="{h + 40:.1f}"/>',
    ]
    gx = int(x0) - 1
    while gx <= max_x + pad:
        out.append(f'<line class="grid" x1="{X(gx):.1f}" y1="0" x2="{X(gx):.1f}" y2="{h:.1f}"/>')
        gx += 1
    gy = int(y0) - 1
    while gy <= max_y + pad:
        out.append(f'<line class="grid" x1="0" y1="{Y(gy):.1f}" x2="{w:.1f}" y2="{Y(gy):.1f}"/>')
        gy += 1

    for s in cmap.segments:
        sw = (s.thickness + 2 * model.inscribed_radius_m) * _PX
        out.append(f'<line class="infl" x1="{X(s.start[0]):.1f}" y1="{Y(s.start[1]):.1f}" '
                   f'x2="{X(s.end[0]):.1f}" y2="{Y(s.end[1]):.1f}" stroke-width="{sw:.1f}"/>')
    for b, br in zip(cmap.boulders, config.boulder_radii(cmap)):
        r = (b.radius + br) * _PX
        out.append(f'<circle class="inflb" cx="{X(b.centre[0]):.1f}" cy="{Y(b.centre[1]):.1f}" r="{r:.1f}"/>')
    for s in cmap.segments:
        sw = max(s.thickness * _PX, 1.5)
        out.append(f'<line class="wall" x1="{X(s.start[0]):.1f}" y1="{Y(s.start[1]):.1f}" '
                   f'x2="{X(s.end[0]):.1f}" y2="{Y(s.end[1]):.1f}" stroke-width="{sw:.1f}"/>')
    for b in cmap.boulders:
        out.append(f'<circle class="boulder" cx="{X(b.centre[0]):.1f}" cy="{Y(b.centre[1]):.1f}" '
                   f'r="{b.radius * _PX:.1f}"/>')

    for g in cmap.goals:
        cx, cy = g.centre
        if g.shape == "disc":
            out.append(f'<circle class="pad" cx="{X(cx):.1f}" cy="{Y(cy):.1f}" r="{g.params["radius"] * _PX:.1f}"/>')
        else:
            ax, ay = g.axis
            half_w = g.params["width"] / 2.0
            nx, ny = -ay, ax  # across the opening
            out.append(f'<line class="goal" x1="{X(cx - nx * half_w):.1f}" y1="{Y(cy - ny * half_w):.1f}" '
                       f'x2="{X(cx + nx * half_w):.1f}" y2="{Y(cy + ny * half_w):.1f}"/>')
        out.append(f'<text class="lbl" x="{X(cx) + 6:.1f}" y="{Y(cy) - 6:.1f}">{_esc(g.name)}</text>')

    if route is not None:
        pts = " ".join(f"{X(w_.position[0]):.1f},{Y(w_.position[1]):.1f}" for w_ in route.waypoints)
        out.append(f'<polyline class="route" points="{pts}"/>')
        for a, b in zip(route.waypoints, route.waypoints[1:]):
            if a.straight_to_next:
                out.append(f'<line class="pass" x1="{X(a.position[0]):.1f}" y1="{Y(a.position[1]):.1f}" '
                           f'x2="{X(b.position[0]):.1f}" y2="{Y(b.position[1]):.1f}"/>')
        for w_ in route.waypoints:
            cls = "start" if w_.kind == "start" else "wp"
            out.append(f'<circle class="{cls}" cx="{X(w_.position[0]):.1f}" cy="{Y(w_.position[1]):.1f}" r="3.5"/>')
        mx, my = route.min_clearance_at
        out.append(f'<circle cx="{X(mx):.1f}" cy="{Y(my):.1f}" r="7" fill="none" stroke="#c00" stroke-width="2"/>')
        caption = (f"{title + ' | ' if title else ''}{route.length_m:.2f} m, {len(route.waypoints)} waypoints, "
                   f"nearest obstacle {route.min_clearance_m:.3f} m (red ring), goals: {' > '.join(route.goals)}")
    else:
        caption = title or cmap.source
    out.append(f'<text class="lbl" x="8" y="{h + 25:.1f}">{_esc(caption)}</text>')
    out.append("</svg>")
    return "\n".join(out) + "\n"


def _esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
