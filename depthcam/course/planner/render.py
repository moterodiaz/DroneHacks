"""Top-down picture of a plan with OpenCV (no matplotlib): map, grown obstacles, gates, timed path.

Colours: obstacles dark grey (walls), mid grey (tube walls), blue-grey (rocks); the area the drone
centre may not enter is pink; floor markings are faint; gates are blue with their free part thick;
the path is coloured by speed (turbo colour map, legend at the side); a drone outline is drawn
every half second.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import cv2
import numpy as np

from course.course_map import CourseMap
from course.planner.plan import Plan

PX_PER_M = 130
BORDER_PX = 40
PANEL_PX = 380


class _View:
    def __init__(self, cmap: CourseMap) -> None:
        b = cmap.boundary
        self.x0, self.x1 = b["x"]
        self.y0, self.y1 = b["y"]
        self.w = int(round((self.x1 - self.x0) * PX_PER_M)) + 2 * BORDER_PX
        self.h = int(round((self.y1 - self.y0) * PX_PER_M)) + 2 * BORDER_PX

    def px(self, xy: np.ndarray) -> np.ndarray:
        xy = np.atleast_2d(xy)
        u = BORDER_PX + (xy[:, 0] - self.x0) * PX_PER_M
        v = BORDER_PX + (self.y1 - xy[:, 1]) * PX_PER_M
        return np.round(np.stack([u, v], axis=1)).astype(np.int32)


def _bgr(rgb, scale: float = 255.0) -> Tuple[int, int, int]:
    return (int(rgb[2] * scale), int(rgb[1] * scale), int(rgb[0] * scale))


def _turbo(value: float, top: float) -> Tuple[int, int, int]:
    level = np.array([[int(np.clip(value / max(top, 1e-9), 0.0, 1.0) * 255)]], dtype=np.uint8)
    c = cv2.applyColorMap(level, cv2.COLORMAP_TURBO)[0, 0]
    return int(c[0]), int(c[1]), int(c[2])


def render_plan(plan: Plan, cmap: CourseMap, title: str) -> np.ndarray:
    view = _View(cmap)
    img = np.full((view.h, view.w + PANEL_PX, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, tuple(view.px(np.array([view.x0, view.y1]))[0]),
                  tuple(view.px(np.array([view.x1, view.y0]))[0]), (238, 238, 238), -1)

    for f in cmap.of_kind("marking"):
        colour = _bgr(f["color"]) if f.get("color") else (120, 120, 120)
        faint = tuple(int(255 - 0.35 * (255 - c)) for c in colour)
        for poly in f["polygons"]:
            contours = [view.px(np.array(poly["outer"]))] + [view.px(np.array(h)) for h in poly["holes"]]
            cv2.fillPoly(img, contours, faint, lineType=cv2.LINE_AA)

    overlay = img.copy()
    for poly in plan.cspace.polygons[:len(plan.cspace.obstacles)]:  # the rest are slabs outside the boundary
        cv2.fillPoly(overlay, [view.px(poly)], (200, 190, 245), lineType=cv2.LINE_AA)
    img = cv2.addWeighted(overlay, 0.6, img, 0.4, 0.0)

    kind_colour = {"wall": (60, 60, 60), "tube": (120, 120, 120), "rock": (150, 120, 100), "net": (40, 40, 40)}
    for o in plan.cspace.obstacles:
        cv2.fillPoly(img, [view.px(o.polygon)], kind_colour.get(o.kind, (90, 90, 90)), lineType=cv2.LINE_AA)

    for i, gate in enumerate(plan.gates):
        a, b = view.px(gate.a)[0], view.px(gate.b)[0]
        cv2.line(img, tuple(a), tuple(b), (230, 160, 60), 1, cv2.LINE_AA)
        fa, fb = view.px(gate.point(gate.lo))[0], view.px(gate.point(gate.hi))[0]
        cv2.line(img, tuple(fa), tuple(fb), (200, 110, 20), 3, cv2.LINE_AA)
        label = plan.checkpoints[i]["name"].split(" ")[0] if i < len(plan.checkpoints) else gate.name
        cv2.putText(img, label, tuple(fb + np.array([6, -6])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 80, 10), 1,
                    cv2.LINE_AA)

    top = plan.config.v_max
    pts = view.px(plan.path)
    for k in range(len(pts) - 1):
        cv2.line(img, tuple(pts[k]), tuple(pts[k + 1]), _turbo(plan.speed[k], top), 3, cv2.LINE_AA)

    footprint = plan.config.drone.footprint(math.radians(plan.config.yaw_deg))
    next_t = 0.0
    for k, tk in enumerate(plan.path_times):
        if tk >= next_t:
            cv2.polylines(img, [view.px(plan.path[k] + footprint)], True, (90, 90, 90), 1, cv2.LINE_AA)
            next_t += 0.5
    for xy, label in ((plan.start, "launch"), (plan.goal, "pad")):
        cv2.circle(img, tuple(view.px(xy)[0]), 5, (0, 0, 200), -1, cv2.LINE_AA)
        cv2.putText(img, label, tuple(view.px(xy)[0] + np.array([8, 16])), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 160), 1, cv2.LINE_AA)

    _panel(img, view.w + 10, plan, title)
    return img


def _panel(img: np.ndarray, x: int, plan: Plan, title: str) -> None:
    lines: List[Tuple[str, float]] = [(title, 0.55)]
    cfg = plan.config
    d = cfg.drone
    lines += [
        ("drone {:.2f} x {:.2f} m, margin {:.2f} m".format(d.length_m, d.width_m, d.margin_m), 0.42),
        ("heading {:.0f} deg (ENU), height {:.2f} m".format(cfg.yaw_deg, cfg.cruise_z_m), 0.42),
        ("v_max {:.1f} m/s, a_max {:.1f} m/s^2".format(cfg.v_max, cfg.a_max), 0.42),
        ("smoothness {:g}, course {:.1f} m".format(
            plan.smoothness, float(np.linalg.norm(np.diff(plan.path, axis=0), axis=1).sum())), 0.42),
        ("", 0.42),
        ("checkpoint times (from liftoff):", 0.45),
    ]
    for cp in plan.checkpoints:
        lines.append(("  {:5.2f} s  {}".format(cp["time_s"], cp["name"]), 0.42))
    lines.append(("finish (disarm) {:.2f} s".format(plan.finish_s), 0.5))
    if plan.return_s is not None:
        lines.append(("return {:.1f} s (untimed)".format(plan.return_s), 0.42))
    lines += [("", 0.42), ("room beyond the margin:", 0.45)]
    for c in plan.clearance:
        lines.append(("  {:+.3f} m  {}".format(c["min_room_m"], c["section"][:34]), 0.38))
    lines += [("", 0.42), ("gate free width (centre):", 0.45)]
    for g in plan.gates:
        lines.append(("  {:.3f} m  {}".format(g.free_width, g.name[:30]), 0.38))
    for note in plan.notes:
        lines.append((note[:48], 0.38))
    y = 30
    for text, scale in lines:
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (30, 30, 30), 1, cv2.LINE_AA)
        y += int(34 * scale) + 4
    y += 10
    cv2.putText(img, "speed", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (30, 30, 30), 1, cv2.LINE_AA)
    for i in range(200):
        cv2.line(img, (x + 50 + i, y - 10), (x + 50 + i, y + 2), _turbo(i / 199.0 * plan.config.v_max, plan.config.v_max), 1)
    cv2.putText(img, "0", (x + 46, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(img, "{:.1f} m/s".format(plan.config.v_max), (x + 220, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (30, 30, 30), 1, cv2.LINE_AA)


def render_flight(cmap: CourseMap, picture: dict, lines: List[str], error_top: float = 0.3) -> np.ndarray:
    """Planned path (grey) and flown path (coloured by horizontal tracking error, 0..``error_top`` m)
    over the obstacles at cruise height; touches circled red; the report text at the side."""
    view = _View(cmap)
    img = np.full((view.h, view.w + 2 * PANEL_PX, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, tuple(view.px(np.array([view.x0, view.y1]))[0]),
                  tuple(view.px(np.array([view.x1, view.y0]))[0]), (238, 238, 238), -1)
    for f in cmap.of_kind("marking"):
        colour = _bgr(f["color"]) if f.get("color") else (120, 120, 120)
        faint = tuple(int(255 - 0.35 * (255 - c)) for c in colour)
        for poly in f["polygons"]:
            contours = [view.px(np.array(poly["outer"]))] + [view.px(np.array(h)) for h in poly["holes"]]
            cv2.fillPoly(img, contours, faint, lineType=cv2.LINE_AA)
    kind_colour = {"wall": (60, 60, 60), "tube": (120, 120, 120), "rock": (150, 120, 100), "net": (40, 40, 40)}
    for o in picture["obstacles"]:
        cv2.fillPoly(img, [view.px(o.polygon)], kind_colour.get(o.kind, (90, 90, 90)), lineType=cv2.LINE_AA)
    for gate in picture["gates"]:
        cv2.line(img, tuple(view.px(gate.a)[0]), tuple(view.px(gate.b)[0]), (200, 110, 20), 2, cv2.LINE_AA)
    planned = view.px(picture["planned"])
    cv2.polylines(img, [planned], False, (150, 150, 150), 2, cv2.LINE_AA)
    flown = view.px(picture["flown"])
    for k in range(len(flown) - 1):
        cv2.line(img, tuple(flown[k]), tuple(flown[k + 1]), _turbo(picture["error"][k], error_top), 3, cv2.LINE_AA)
    for touch in picture["touches"]:
        cv2.circle(img, tuple(view.px(np.array(touch["at"]))[0]), 12, (0, 0, 230), 2, cv2.LINE_AA)
    x, y = view.w + 10, 30
    for text in lines:
        cv2.putText(img, text[:96], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (30, 30, 30), 1, cv2.LINE_AA)
        y += 18
    y += 14
    cv2.putText(img, "tracking error", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (30, 30, 30), 1, cv2.LINE_AA)
    for i in range(200):
        cv2.line(img, (x + 110 + i, y - 10), (x + 110 + i, y + 2), _turbo(i / 199.0 * error_top, error_top), 1)
    cv2.putText(img, "0", (x + 106, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(img, "{:.2f} m".format(error_top), (x + 290, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(img, "grey: planned   colour: flown   red circle: drone outline touched an obstacle",
                (x, y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (30, 30, 30), 1, cv2.LINE_AA)
    return img
