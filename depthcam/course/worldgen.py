"""Course map -> Gazebo (Harmonic, SDF 1.9) model and world for PX4 SITL.

Output layout (``write_world``)::

    <out>/models/<model>/model.config
    <out>/models/<model>/model.sdf                 one static model, one link
    <out>/models/<model>/meshes/tube_<i>.obj       hollow tube visuals
    <out>/models/<model>/meshes/floor.obj/.mtl/.png  the floor markings as one texture
    <out>/worlds/<world>.sdf                       physics settings, sun, ground plane, the model

How each feature kind becomes SDF:

``wall``     boxes; a wall with an opening is split into the pieces around it, so the window is a
             real hole.
``tube``     visual: a generated OBJ (outer and inner elliptical surfaces and the end rings);
             collision: a ring of thin boxes around the cross-section (works with every physics
             engine, unlike concave mesh collision).
``rock``     visual: a half-buried ellipsoid (a dome); collision: a cylinder of the same radius and
             height (conservative).
``net``      collision-only boxes, 2 cm thick, outside the course boundary (``visible_nets`` adds a
             faint visual; a GPU lidar then sees the nets as solid, which a real lidar may not).
``marking``  drawn into one floor texture, visual only.
``support``  left out: the tube is static, and the cradles sit under it.
``block``    a box over its footprint's minimum-area rectangle.

The frame is the map frame (x east, y north, z up), which is also Gazebo's ENU world frame.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from course.course_map import CourseMap
from course.geometry import min_area_rect

TEXTURE_M_PER_PX = 0.004
NET_THICKNESS_M = 0.02
TUBE_VISUAL_SEGMENTS = 64
TUBE_COLLISION_SEGMENTS = 24

PX4_SYSTEMS = [
    ("gz-sim-physics-system", "gz::sim::systems::Physics"),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ("gz-sim-imu-system", "gz::sim::systems::Imu"),
    ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ("gz-sim-apply-link-wrench-system", "gz::sim::systems::ApplyLinkWrench"),
]


def _fmt(values: Sequence[float]) -> str:
    return " ".join("{:.6g}".format(float(v) + 0.0) for v in values)


def _sub(parent: ET.Element, tag: str, text: Optional[str] = None, **attrib: str) -> ET.Element:
    el = ET.SubElement(parent, tag, attrib)
    if text is not None:
        el.text = text
    return el


def _material(parent: ET.Element, rgb: Sequence[float], alpha: float = 1.0) -> None:
    m = _sub(parent, "material")
    colour = _fmt(list(rgb) + [alpha])
    _sub(m, "ambient", colour)
    _sub(m, "diffuse", colour)
    _sub(m, "specular", "0.1 0.1 0.1 1")


def _safe(name: str) -> str:
    return "".join(c if c.isalnum() else "_" for c in name).strip("_")


def rpy_from_matrix(r: np.ndarray) -> Tuple[float, float, float]:
    """SDF roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = math.atan2(-r[1, 2], r[1, 1]), 0.0
    return roll, pitch, yaw


class _Link:
    def __init__(self, link: ET.Element) -> None:
        self.link = link
        self.names: Dict[str, int] = {}

    def _name(self, base: str) -> str:
        base = _safe(base)
        self.names[base] = self.names.get(base, -1) + 1
        return "{}_{}".format(base, self.names[base])

    def box(self, name: str, pose: Sequence[float], size: Sequence[float], rgb: Optional[Sequence[float]],
            collision: bool = True, visual: bool = True, alpha: float = 1.0) -> None:
        tag = self._name(name)
        if collision:
            c = _sub(self.link, "collision", name=tag + "_c")
            _sub(c, "pose", _fmt(pose))
            _sub(_sub(_sub(c, "geometry"), "box"), "size", _fmt(size))
        if visual:
            v = _sub(self.link, "visual", name=tag + "_v")
            _sub(v, "pose", _fmt(pose))
            _sub(_sub(_sub(v, "geometry"), "box"), "size", _fmt(size))
            _material(v, rgb or (0.5, 0.5, 0.5), alpha)
            if alpha < 1.0:
                _sub(v, "transparency", "{:.3f}".format(1.0 - alpha))

    def mesh_visual(self, name: str, uri: str, rgb: Sequence[float]) -> None:
        v = _sub(self.link, "visual", name=self._name(name) + "_v")
        _sub(_sub(_sub(v, "geometry"), "mesh"), "uri", uri)
        _material(v, rgb)

    def rock(self, name: str, centre: Sequence[float], radius: float, height: float, rgb: Sequence[float]) -> None:
        tag = self._name(name)
        c = _sub(self.link, "collision", name=tag + "_c")
        _sub(c, "pose", _fmt([centre[0], centre[1], height / 2.0, 0, 0, 0]))
        cyl = _sub(_sub(c, "geometry"), "cylinder")
        _sub(cyl, "radius", _fmt([radius]))
        _sub(cyl, "length", _fmt([height]))
        v = _sub(self.link, "visual", name=tag + "_v")
        _sub(v, "pose", _fmt([centre[0], centre[1], 0.0, 0, 0, 0]))
        _sub(_sub(_sub(v, "geometry"), "ellipsoid"), "radii", _fmt([radius, radius, height]))
        _material(v, rgb)


def _wall_boxes(f: dict) -> List[Tuple[Tuple[float, ...], Tuple[float, float, float]]]:
    """(pose, size) of the boxes that make up a wall, leaving its openings open."""
    p0, p1 = np.array(f["p0"]), np.array(f["p1"])
    length = float(np.linalg.norm(p1 - p0))
    d = (p1 - p0) / length
    yaw = math.atan2(d[1], d[0])
    z0, z1 = f["z"]
    pieces: List[Tuple[float, float, float, float]] = []  # s0, s1, za, zb
    s = 0.0
    for o in sorted(f.get("openings", []), key=lambda o: o["s"][0]):
        s0, s1 = max(0.0, o["s"][0]), min(length, o["s"][1])
        if s0 - s > 1e-6:
            pieces.append((s, s0, z0, z1))
        if o["z"][0] - z0 > 1e-6:
            pieces.append((s0, s1, z0, o["z"][0]))
        if z1 - o["z"][1] > 1e-6:
            pieces.append((s0, s1, o["z"][1], z1))
        s = max(s, s1)
    if length - s > 1e-6:
        pieces.append((s, length, z0, z1))
    out = []
    for sa, sb, za, zb in pieces:
        c = p0 + d * (sa + sb) / 2.0
        out.append(((c[0], c[1], (za + zb) / 2.0, 0.0, 0.0, yaw), (sb - sa, f["thickness"], zb - za)))
    return out


def _tube_frame(f: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    p0, p1 = np.array(f["p0"]), np.array(f["p1"])
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    w = np.cross(np.array([0.0, 0.0, 1.0]), u)
    w /= np.linalg.norm(w)
    z = np.cross(u, w)
    return p0, p1, u, w, z


def tube_obj(f: dict, segments: int = TUBE_VISUAL_SEGMENTS) -> str:
    """Wavefront OBJ text of a hollow elliptical tube, every face wound to face outwards."""
    p0, p1, u, w, z = _tube_frame(f)
    theta = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)

    def ring(centre: np.ndarray, hw: float, hh: float) -> np.ndarray:
        return centre + np.outer(hw * np.cos(theta), w) + np.outer(hh * np.sin(theta), z)

    o, i = f["outer"], f["inner"]
    rings = [ring(p0, o["half_width"], o["half_height"]), ring(p1, o["half_width"], o["half_height"]),
             ring(p0, i["half_width"], i["half_height"]), ring(p1, i["half_width"], i["half_height"])]
    verts = np.concatenate(rings)
    tris: List[Tuple[int, int, int]] = []

    def quad(a: int, b: int, c: int, d: int, want: np.ndarray) -> None:
        for tri in ((a, b, c), (a, c, d)):
            n = np.cross(verts[tri[1]] - verts[tri[0]], verts[tri[2]] - verts[tri[0]])
            tris.append(tri if np.dot(n, want(tri) if callable(want) else want) >= 0 else (tri[0], tri[2], tri[1]))

    mid = (p0 + p1) / 2.0
    for k in range(segments):
        k2 = (k + 1) % segments
        o0, o1, i0, i1 = 0, segments, 2 * segments, 3 * segments
        radial = lambda tri: verts[list(tri)].mean(axis=0) - (mid + u * np.dot(verts[list(tri)].mean(axis=0) - mid, u))
        quad(o0 + k, o0 + k2, o1 + k2, o1 + k, radial)
        quad(i0 + k, i0 + k2, i1 + k2, i1 + k, lambda tri: -radial(tri))
        quad(o0 + k, o0 + k2, i0 + k2, i0 + k, -u)
        quad(o1 + k, o1 + k2, i1 + k2, i1 + k, u)
    lines = ["# hollow tube generated by course/worldgen.py from {}".format(f["name"])]
    lines += ["v {:.6f} {:.6f} {:.6f}".format(*v) for v in verts]
    # One normal per triangle (flat shading): without normals ogre2 renders the mesh unlit white.
    for a, b, c in tris:
        n = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        lines.append("vn {:.6f} {:.6f} {:.6f}".format(*(n / np.linalg.norm(n))))
    lines += ["f {0}//{3} {1}//{3} {2}//{3}".format(a + 1, b + 1, c + 1, k + 1) for k, (a, b, c) in enumerate(tris)]
    return "\n".join(lines) + "\n"


def tube_collision_boxes(f: dict, segments: int = TUBE_COLLISION_SEGMENTS):
    """(pose, size) of thin boxes around the tube's cross-section (mid-wall ellipse)."""
    p0, p1, u, w, z = _tube_frame(f)
    hw = 0.5 * (f["outer"]["half_width"] + f["inner"]["half_width"])
    hh = 0.5 * (f["outer"]["half_height"] + f["inner"]["half_height"])
    thick = max(f["outer"]["half_width"] - f["inner"]["half_width"], 0.01)
    length = float(np.linalg.norm(p1 - p0))
    mid = (p0 + p1) / 2.0
    out = []
    for k in range(segments):
        t0, t1 = 2 * math.pi * k / segments, 2 * math.pi * (k + 1) / segments
        a = mid + hw * math.cos(t0) * w + hh * math.sin(t0) * z
        b = mid + hw * math.cos(t1) * w + hh * math.sin(t1) * z
        chord = b - a
        q = chord / np.linalg.norm(chord)
        n = np.cross(u, q)
        roll, pitch, yaw = rpy_from_matrix(np.column_stack([u, q, n]))
        c = (a + b) / 2.0
        out.append(((c[0], c[1], c[2], roll, pitch, yaw), (length, float(np.linalg.norm(chord)) * 1.05, thick)))
    return out


def floor_texture(cmap: CourseMap) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """The floor markings drawn top-down (row 0 = north edge), and the (x0, y0, x1, y1) it covers."""
    floor = cmap.of_kind("floor")
    if floor:
        fp = np.array(floor[0]["footprint"])
        x0, y0 = fp.min(axis=0)
        x1, y1 = fp.max(axis=0)
        base = floor[0].get("color") or (0.85, 0.85, 0.85)
    else:
        (x0, x1), (y0, y1) = cmap.boundary["x"], cmap.boundary["y"]
        base = (0.85, 0.85, 0.85)
    w = int(round((x1 - x0) / TEXTURE_M_PER_PX))
    h = int(round((y1 - y0) / TEXTURE_M_PER_PX))
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = [int(base[2] * 255), int(base[1] * 255), int(base[0] * 255)]

    def px(points: Sequence[Sequence[float]]) -> np.ndarray:
        p = np.asarray(points, dtype=float)
        return np.round(np.stack([(p[:, 0] - x0) / TEXTURE_M_PER_PX, (y1 - p[:, 1]) / TEXTURE_M_PER_PX], axis=1)
                        ).astype(np.int32)

    import cv2  # only the texture and world writers need OpenCV; the lidar code imports this module

    markings = sorted(cmap.of_kind("marking"), key=lambda f: (f["z"][1], -f["half_size"][0] * f["half_size"][1]))
    for f in markings:
        rgb = f.get("color") or (0.2, 0.2, 0.2)
        colour = (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255))
        for poly in f["polygons"]:
            cv2.fillPoly(img, [px(poly["outer"])] + [px(hole) for hole in poly["holes"]], colour, lineType=cv2.LINE_AA)
    return img, (float(x0), float(y0), float(x1), float(y1))


def floor_obj(bounds: Tuple[float, float, float, float], z: float, flip_v: bool = False) -> Tuple[str, str]:
    x0, y0, x1, y1 = bounds
    v0, v1 = (1.0, 0.0) if flip_v else (0.0, 1.0)
    obj = "\n".join([
        "# floor markings quad generated by course/worldgen.py",
        "mtllib floor.mtl",
        "v {:.6f} {:.6f} {:.6f}".format(x0, y0, z), "v {:.6f} {:.6f} {:.6f}".format(x1, y0, z),
        "v {:.6f} {:.6f} {:.6f}".format(x1, y1, z), "v {:.6f} {:.6f} {:.6f}".format(x0, y1, z),
        "vt 0 {}".format(v0), "vt 1 {}".format(v0), "vt 1 {}".format(v1), "vt 0 {}".format(v1),
        "vn 0 0 1",
        "usemtl floor_markings",
        "f 1/1/1 2/2/1 3/3/1", "f 1/1/1 3/3/1 4/4/1",
    ]) + "\n"
    mtl = "newmtl floor_markings\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd floor.png\n"
    return obj, mtl


def _xml(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def build_model(cmap: CourseMap, model: str, visible_nets: bool = False) -> Tuple[str, Dict[str, str], List[str]]:
    """model.sdf text, mesh files {relative path: text}, and notes about what was simplified."""
    sdf = ET.Element("sdf", version="1.9")
    m = _sub(sdf, "model", name=model)
    _sub(m, "static", "true")
    link = _Link(_sub(m, "link", name="course"))
    meshes: Dict[str, str] = {}
    notes: List[str] = []
    centre = np.array([np.mean(cmap.boundary["x"]), np.mean(cmap.boundary["y"])])
    tube_count = 0
    for f in cmap.features:
        kind, rgb = f["kind"], f.get("color")
        if kind == "wall":
            for pose, size in _wall_boxes(f):
                link.box(f["name"], pose, size, rgb)
        elif kind == "tube":
            rel = "meshes/tube_{}.obj".format(tube_count)
            tube_count += 1
            meshes[rel] = tube_obj(f)
            link.mesh_visual(f["name"], "model://{}/{}".format(model, rel), rgb or (0.3, 0.3, 0.3))
            for pose, size in tube_collision_boxes(f):
                link.box(f["name"] + " wall", pose, size, None, visual=False)
        elif kind == "rock":
            link.rock(f["name"], f["centre"], f["radius"], f["z"][1], rgb or (0.6, 0.6, 0.65))
        elif kind == "net":
            fp = np.array(f["footprint"])
            c, angle, length, width = min_area_rect(fp)
            z0, z1 = f["z"]
            if z1 - z0 > 1.0:
                normal = np.array([-math.sin(angle), math.cos(angle)])
                if np.dot(c - centre, normal) < 0:
                    normal = -normal
                box_c = c + normal * (NET_THICKNESS_M / 2.0 - width / 2.0)
                link.box(f["name"], (box_c[0], box_c[1], (z0 + z1) / 2.0, 0, 0, angle),
                         (length, NET_THICKNESS_M, z1 - z0), (0.1, 0.1, 0.1), visual=visible_nets, alpha=0.25)
            else:
                lo, hi = fp.min(axis=0), fp.max(axis=0)
                link.box(f["name"], ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, z0 + NET_THICKNESS_M / 2, 0, 0, 0),
                         (hi[0] - lo[0], hi[1] - lo[1], NET_THICKNESS_M), None, visual=False)
        elif kind == "block":
            c, angle, length, width = min_area_rect(np.array(f["footprint"]))
            z0, z1 = f["z"]
            link.box(f["name"], (c[0], c[1], (z0 + z1) / 2, 0, 0, angle), (length, width, z1 - z0), rgb)
            notes.append("{}: generic block, modelled as its bounding rectangle".format(f["name"]))
        elif kind == "support":
            notes.append("{}: support under {}, left out (the tube is static)".format(f["name"], f.get("under")))
    floor_vis = _sub(link.link, "visual", name="floor_markings_v")
    _sub(floor_vis, "cast_shadows", "false")
    _sub(_sub(_sub(floor_vis, "geometry"), "mesh"), "uri", "model://{}/meshes/floor.obj".format(model))
    return _xml(sdf), meshes, notes


def build_world(world: str, model: str, embed_systems: bool = False) -> str:
    """World SDF.  Like PX4's own worlds it lists no system plugins by default: gz-sim then loads
    PX4's ``server.config`` (``GZ_SIM_SERVER_CONFIG_PATH``), which adds PX4's optical-flow and
    camera-streaming systems.  ``embed_systems`` lists the standard ones in the file instead, for
    running the world without PX4's environment (gz-sim skips the server config when a world has
    plugins)."""
    sdf = ET.Element("sdf", version="1.9")
    wd = _sub(sdf, "world", name=world)
    physics = _sub(wd, "physics", name="4ms", type="ignored")
    _sub(physics, "max_step_size", "0.004")
    _sub(physics, "real_time_factor", "1.0")
    _sub(physics, "real_time_update_rate", "250")
    if embed_systems:
        for filename, name in PX4_SYSTEMS:
            _sub(wd, "plugin", filename=filename, name=name)
        sensors = _sub(wd, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
        _sub(sensors, "render_engine", "ogre2")
    _sub(wd, "gravity", "0 0 -9.8")
    _sub(wd, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
    _sub(wd, "atmosphere", type="adiabatic")
    scene = _sub(wd, "scene")
    _sub(scene, "grid", "false")
    _sub(scene, "ambient", "0.35 0.35 0.35 1")
    _sub(scene, "background", "0.7 0.7 0.7 1")
    _sub(scene, "shadows", "true")
    sun = _sub(wd, "light", name="sun", type="directional")
    _sub(sun, "cast_shadows", "true")
    _sub(sun, "pose", "0 0 10 0 0 0")
    _sub(sun, "diffuse", "0.8 0.8 0.8 1")
    _sub(sun, "specular", "0.2 0.2 0.2 1")
    _sub(sun, "direction", "-0.3 0.2 -0.9")
    ground = _sub(wd, "model", name="ground_plane")
    _sub(ground, "static", "true")
    glink = _sub(ground, "link", name="link")
    for tag in ("collision", "visual"):
        el = _sub(glink, tag, name=tag)
        plane = _sub(_sub(el, "geometry"), "plane")
        _sub(plane, "normal", "0 0 1")
        _sub(plane, "size", "40 40")
        if tag == "visual":
            _material(el, (0.55, 0.55, 0.55))
    inc = _sub(wd, "include")
    _sub(inc, "uri", "model://{}".format(model))
    _sub(inc, "name", "course")
    _sub(inc, "pose", "0 0 0 0 0 0")
    # PX4's default origin.  The <magnetic_field> above is PX4's too and matches this place; moving
    # the origin without changing the field gives EKF2 a heading disagreement and the drone drifts
    # into the walls (seen in SITL with the origin moved to California).
    sph = _sub(wd, "spherical_coordinates")
    _sub(sph, "surface_model", "EARTH_WGS84")
    _sub(sph, "world_frame_orientation", "ENU")
    _sub(sph, "latitude_deg", "47.397971057728974")
    _sub(sph, "longitude_deg", "8.546163739800146")
    _sub(sph, "elevation", "0")
    return _xml(sdf)


def write_world(cmap: CourseMap, out: Path, world: str, visible_nets: bool = False,
                flip_floor_v: bool = False, embed_systems: bool = False) -> Dict[str, object]:
    """Write the model and world files; returns their paths and the generator's notes."""
    model = world + "_model"
    model_dir = out / "models" / model
    (model_dir / "meshes").mkdir(parents=True, exist_ok=True)
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    model_sdf, meshes, notes = build_model(cmap, model, visible_nets)
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")
    (model_dir / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>{0}</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.sdf</sdf>\n  <description>Generated by course/13_make_gazebo_world.py from {1} '
        '({2}).</description>\n</model>\n'.format(model, cmap.data["source"]["step"],
                                                 ", ".join(cmap.data["source"]["products"])),
        encoding="utf-8")
    for rel, text in meshes.items():
        (model_dir / rel).write_text(text, encoding="utf-8")
    image, bounds = floor_texture(cmap)
    import cv2
    cv2.imwrite(str(model_dir / "meshes" / "floor.png"), image)
    obj, mtl = floor_obj(bounds, 0.0005, flip_floor_v)
    (model_dir / "meshes" / "floor.obj").write_text(obj, encoding="utf-8")
    (model_dir / "meshes" / "floor.mtl").write_text(mtl, encoding="utf-8")
    world_path = out / "worlds" / "{}.sdf".format(world)
    world_path.write_text(build_world(world, model, embed_systems), encoding="utf-8")
    return {"model_dir": model_dir, "world": world_path, "notes": notes}
