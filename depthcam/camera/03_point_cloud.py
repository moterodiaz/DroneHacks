#!/usr/bin/env python3
"""03_point_cloud.py - live 3-D point cloud from OAK-D S2 stereo depth (DepthAI v3, depthai 3.10.0).

What this test checks
---------------------
* ``dai.node.StereoDepth`` (CAM_B / CAM_C mono pair, 640x400, DEFAULT preset with the 2x
  decimation switched off) feeds ``dai.node.PointCloud`` and the node emits
  ``dai.PointCloudData`` in METRES (``initialConfig.setLengthUnit(dai.LengthUnit.METER)``),
  unorganised (only pixels with valid depth become points).
* The point count, bounding box and centroid are sane for the scene.  Camera frame:
  x right, y down, z forward (away from the camera), origin at the right mono camera
  (or at CAM_A with ``--color``).
* ``--color``: depth is aligned to an undistorted CAM_A RGB stream (``requestOutput(...,
  enableUndistortion=True)`` linked into ``StereoDepth.inputAlignTo`` *and*
  ``PointCloud.inputColor``) and ``getPointsRGB()`` returns a per-point RGBA colour.
* Optional Open3D live viewer and PLY export (Open3D binary PLY, or a built-in ASCII writer).

How to run (from the repo root, with the project venv)
------------------------------------------------------
  python camera/03_point_cloud.py                        # Open3D viewer + depth window, until q / Esc
  python camera/03_point_cloud.py --color                # RGB-coloured cloud
  python camera/03_point_cloud.py --headless --duration 10
  python camera/03_point_cloud.py --max-depth 3 --downsample 4 --save-on-exit
  python camera/03_point_cloud.py --device-id OAKDS2EXAMPLEID000 --fps 15
  python camera/03_point_cloud.py --help

What you should see
-------------------
Windowed: an Open3D window "03_point_cloud" showing the cloud in 3-D (coloured by distance,
near = red, far = blue; true colours with ``--color``) with a 10 cm coordinate frame at the
camera, plus an OpenCV window "03_point_cloud depth" with the depth image that feeds the
cloud and a stats overlay.  The cloud re-shapes as things move in front of the camera at
roughly the requested fps (default 30).  Without a working Open3D the script prints the
import error plus an install hint and behaves like ``--headless``.

Headless: one line per second, e.g.
  [  3.0s] points= 38707 (raw  41220)  fps=29.7  bbox x[-1.23,+1.45] y[-0.80,+0.62] z[0.55,4.90] m  centroid=(+0.02,-0.11,+1.63) m

Keys (either window)
--------------------
  q / Esc   quit
  s         save the current cloud as snapshots/pointcloud_<YYYYmmdd_HHMMSS>.ply
Open3D window mouse: left-drag rotate, right/middle-drag pan, wheel zoom; ``r`` resets the
view, ``+`` / ``-`` change the point size, ``h`` prints Open3D's full key help to the console.

``--duration`` counts from ``pipeline.start()`` and includes the 1.5 s auto-exposure warm-up.
A wrong ``--device-id`` fails fast; without an id the open is retried while a camera reboots.

Exit codes: 0 normal (duration elapsed, q / Esc, Ctrl-C), 2 no device or wrong ``--device-id``
(readable lines on stderr, no traceback), 1 the pipeline stopped or raised on its own (node /
link error: one ``Pipeline error: ...`` line on stderr).
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time
from pathlib import Path
from typing import Any, Optional, Tuple

import cv2
import numpy as np
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402  (import after sys.path tweak)
    FpsCounter,
    Periodic,
    RunClock,
    add_device_arg,
    add_run_args,
    colorize_depth,
    draw_label,
    ensure_dir,
    open_device,
    print_device_summary,
    timestamp_name,
    wait_key_quit,
    warm_up,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
SNAPSHOT_DIR = REPO_ROOT / "snapshots"

STREAM_SIZE: Tuple[int, int] = (640, 400)  # mono (and colour) request size -> depth / cloud grid
WINDOW_3D = "03_point_cloud"
WINDOW_DEPTH = "03_point_cloud depth"
DEFAULT_COLOUR_RANGE_M = 5.0  # z range used to colour the cloud / depth image when --max-depth is 0
WARM_UP_SECONDS = 1.5  # auto-exposure settle after pipeline.start(); counted inside --duration
SAVE_DEBOUNCE_S = 0.5  # at most one PLY save per this interval (held / auto-repeating "s" key)

# GLFW key codes and the key action used by Open3D's key callbacks.
_GLFW_KEY_Q = ord("Q")
_GLFW_KEY_S = ord("S")
_GLFW_KEY_ESCAPE = 256
_GLFW_PRESS = 1  # register_key_action_callback actions: 0 release, 1 press, 2 repeat


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Live point cloud from OAK-D S2 stereo depth (dai.node.PointCloud, DepthAI v3).",
        epilog="Keys in the Open3D / depth window: q or Esc = quit, s = save PLY into snapshots/.",
    )
    add_device_arg(parser)
    add_run_args(parser)
    parser.add_argument("--fps", type=float, default=30.0, metavar="FPS",
                        help="camera frame rate for the stereo pair (and colour camera) (default 30)")
    parser.add_argument("--max-depth", dest="max_depth", type=float, default=0.0, metavar="METERS",
                        help="drop points farther than this (default 0 = keep everything)")
    parser.add_argument("--downsample", type=int, default=1, metavar="N",
                        help="keep every Nth point on the host (default 1 = all points)")
    parser.add_argument("--color", action="store_true",
                        help="colourise the cloud with CAM_A RGB (depth aligned to the undistorted colour stream)")
    parser.add_argument("--save-on-exit", dest="save_on_exit", action="store_true",
                        help="write the last cloud to snapshots/pointcloud_<timestamp>.ply when the script ends")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.fps <= 0:
        parser.error("--fps must be > 0")
    if args.max_depth < 0:
        parser.error("--max-depth must be >= 0 (0 = no limit)")
    if args.downsample < 1:
        parser.error("--downsample must be >= 1")


# --------------------------------------------------------------------------------------
# Optional Open3D
# --------------------------------------------------------------------------------------

def try_import_open3d() -> Tuple[Optional[Any], Optional[str]]:
    """Return ``(open3d module, None)``, or ``(None, '<ExcType>: <message>')`` when it fails.

    The error text is kept so the hint can tell "not installed" (``ImportError: No module
    named 'open3d'``) from a broken native library on the Jetson (an ``ImportError`` from
    ``libOpen3D.so`` / a GLIBC mismatch), where ``pip install`` would be the wrong advice.
    """
    try:
        import open3d as o3d  # noqa: WPS433 (optional dependency)
    except Exception as exc:  # noqa: BLE001 - ImportError, or a broken native lib on Jetson
        text = " ".join(str(exc).split())[:300] or "no message"
        return None, "{}: {}".format(type(exc).__name__, text)
    return o3d, None


def open3d_hint(error: Optional[str]) -> str:
    """One stderr line: why there is no 3-D window (with the import error) and how to fix it."""
    return ("open3d is not importable ({}): no 3-D window, running like --headless. "
            "Install it with: {} -m pip install open3d".format(error or "unknown error", sys.executable))


# --------------------------------------------------------------------------------------
# Cloud maths
# --------------------------------------------------------------------------------------

class CloudState:
    """Latest filtered cloud kept for the stats line, the viewer and PLY export."""

    def __init__(self) -> None:
        self.points: Optional[np.ndarray] = None   # (N,3) float32, metres
        self.colors: Optional[np.ndarray] = None   # (N,3) uint8 RGB or None
        self.raw_count: int = 0                    # points delivered by the node before filtering
        self.clouds: int = 0                       # messages processed

    def update(self, points: np.ndarray, colors: Optional[np.ndarray], raw_count: int) -> None:
        self.points, self.colors, self.raw_count = points, colors, raw_count
        self.clouds += 1


def extract_cloud(msg: dai.PointCloudData) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """``PointCloudData`` -> (points (N,3) float32, colours (N,3) uint8 RGB or None)."""
    if msg.isColor():
        points, rgba = msg.getPointsRGB()
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        colors: Optional[np.ndarray] = np.asarray(rgba, dtype=np.uint8).reshape(-1, 4)[:, :3]
    else:
        points = np.asarray(msg.getPoints(), dtype=np.float32).reshape(-1, 3)
        colors = None
    return points, colors


def filter_cloud(points: np.ndarray, colors: Optional[np.ndarray],
                 max_depth_m: float, downsample: int) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Drop invalid (z <= 0, non-finite) and too-far points, then keep every Nth point."""
    keep = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.0)
    if max_depth_m > 0.0:
        keep &= points[:, 2] <= max_depth_m
    points = points[keep]
    if colors is not None:
        colors = colors[keep]
    if downsample > 1:
        points = points[::downsample]
        if colors is not None:
            colors = colors[::downsample]
    return np.ascontiguousarray(points), (None if colors is None else np.ascontiguousarray(colors))


def describe_cloud(points: np.ndarray, raw_count: int) -> str:
    """One-line bounding box / centroid summary in metres."""
    if len(points) == 0:
        return "points=     0 (raw {:6d})  no valid depth".format(raw_count)
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    c = points.mean(axis=0)
    return ("points={:6d} (raw {:6d})  bbox x[{:+.2f},{:+.2f}] y[{:+.2f},{:+.2f}] z[{:.2f},{:.2f}] m  "
            "centroid=({:+.2f},{:+.2f},{:+.2f}) m").format(
                len(points), raw_count, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], c[0], c[1], c[2])


def depth_colours(points: np.ndarray, range_m: float) -> np.ndarray:
    """Jet colours by z (near = red, far = blue), (N,3) float64 in 0..1 for Open3D."""
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    scale = 255.0 / max(float(range_m), 1e-6)
    near_hot = (255.0 - np.clip(points[:, 2] * scale, 0.0, 255.0)).astype(np.uint8).reshape(-1, 1)
    bgr = cv2.applyColorMap(near_hot, cv2.COLORMAP_JET).reshape(-1, 3)
    return bgr[:, ::-1].astype(np.float64) / 255.0  # BGR -> RGB


# --------------------------------------------------------------------------------------
# PLY export
# --------------------------------------------------------------------------------------

def write_ply_ascii(path: Path, points: np.ndarray, colors: Optional[np.ndarray]) -> None:
    """Minimal ASCII PLY writer (x y z [red green blue]); used when Open3D is unavailable."""
    header = [
        "ply",
        "format ascii 1.0",
        "comment DroneDepthCamTesting 03_point_cloud.py  units=metres  frame: x right, y down, z forward",
        "element vertex {}".format(len(points)),
        "property float x",
        "property float y",
        "property float z",
    ]
    if colors is not None:
        header += ["property uchar red", "property uchar green", "property uchar blue"]
    header.append("end_header")
    with path.open("w", encoding="ascii", newline="\n") as fh:
        fh.write("\n".join(header) + "\n")
        if len(points) == 0:
            return
        if colors is None:
            np.savetxt(fh, points, fmt="%.5f %.5f %.5f")
        else:
            data = np.hstack([points.astype(np.float64), colors.astype(np.float64)])
            np.savetxt(fh, data, fmt="%.5f %.5f %.5f %d %d %d")


def unique_path(path: Path) -> Path:
    """Return *path*, or ``<stem>_1.ply``, ``_2`` ... if it already exists (two saves in one second)."""
    candidate = path
    counter = 1
    while candidate.exists():
        candidate = path.with_name("{}_{}{}".format(path.stem, counter, path.suffix))
        counter += 1
    return candidate


def save_ply(state: CloudState, out_dir: Path, o3d: Optional[Any]) -> Optional[Path]:
    """Write the latest cloud to ``out_dir/pointcloud_<timestamp>.ply``; return the path or None."""
    if state.points is None:
        print("Nothing to save yet (no point cloud received).", flush=True)
        return None
    if len(state.points) == 0:
        # open3d.io.write_point_cloud refuses 0-point clouds (returns False) and the ASCII
        # writer would produce a header-only file: neither is useful, so say so and skip.
        print("Nothing to save: the last cloud had no valid points (check --max-depth / the scene).",
              flush=True)
        return None
    path = unique_path(ensure_dir(out_dir) / timestamp_name("pointcloud", "ply"))
    points, colors = state.points, state.colors
    try:
        if o3d is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
            if not o3d.io.write_point_cloud(str(path), pcd, write_ascii=False):
                raise RuntimeError("open3d.io.write_point_cloud returned False")
            how = "open3d binary PLY"
        else:
            write_ply_ascii(path, points, colors)
            how = "ASCII PLY"
    except Exception as exc:  # noqa: BLE001 - report, never crash the run because of a save
        print("Failed to save {}: {}: {}".format(path, type(exc).__name__, exc), file=sys.stderr, flush=True)
        return None
    print("Saved {} points{} to {} ({}, {:.1f} KB)".format(
        len(points), " with colour" if colors is not None else "", path, how,
        path.stat().st_size / 1024.0), flush=True)
    return path


# --------------------------------------------------------------------------------------
# Pipeline
# --------------------------------------------------------------------------------------

def build_pipeline(pipeline: dai.Pipeline, args: argparse.Namespace
                   ) -> Tuple[dai.node.StereoDepth, dai.node.PointCloud]:
    """Stereo pair -> StereoDepth -> PointCloud (+ optional CAM_A colour aligned into the depth)."""
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth).build(
        left=left.requestOutput(STREAM_SIZE, fps=args.fps),
        right=right.requestOutput(STREAM_SIZE, fps=args.fps),
        presetMode=dai.node.StereoDepth.PresetMode.DEFAULT,
    )
    # The DEFAULT preset decimates 2x (320x200 depth). Keep the full 640x400 grid so the cloud is
    # dense and, with --color, the same size as the colour stream it is aligned to.
    stereo.initialConfig.postProcessing.decimationFilter.decimationFactor = 1

    pc = pipeline.create(dai.node.PointCloud)
    pc.setRunOnHost(True)  # the default; explicit as in the official point_cloud_visualizer.py
    pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
    pc.initialConfig.setOrganized(False)  # only z > 0 points are emitted

    if args.color:
        cam_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb = cam_a.requestOutput(STREAM_SIZE, dai.ImgFrame.Type.RGB888i, fps=args.fps,
                                  enableUndistortion=True)
        rgb.link(stereo.inputAlignTo)  # depth in the colour camera's perspective (isAlignedTo == True)
        rgb.link(pc.inputColor)
    stereo.depth.link(pc.inputDepth)
    return stereo, pc


def print_first_cloud(msg: dai.PointCloudData) -> None:
    print("First cloud: seq={} {}x{} color={} organized={} raw points={} z=[{:.2f}, {:.2f}] m".format(
        msg.getSequenceNum(), msg.getWidth(), msg.getHeight(), msg.isColor(), msg.isOrganized(),
        len(msg.getPoints()) if not msg.isColor() else len(msg.getPointsRGB()[0]),
        msg.getMinZ(), msg.getMaxZ()), flush=True)


# --------------------------------------------------------------------------------------
# Run loops
# --------------------------------------------------------------------------------------

def run_banner(clock: RunClock) -> str:
    """`` for N s (M s left after the warm-up)``, or ``''`` when there is no time limit."""
    remaining = clock.remaining()
    if remaining is None:
        return ""
    return " for {:.1f} s ({:.1f} s left after the {:.1f} s warm-up)".format(
        clock.duration, remaining, WARM_UP_SECONDS)


def run_headless(pipeline: dai.Pipeline, q_pc: dai.MessageQueue, args: argparse.Namespace,
                 state: CloudState, clock: RunClock) -> int:
    """Blocking get with timeout; one stats line per second. Returns the exit code.

    *clock* was started by :func:`main` right after ``pipeline.start()``, so ``--duration``
    includes the auto-exposure warm-up.
    """
    fps = FpsCounter()
    stats = Periodic(1.0)
    silent_s = 0.0
    print("Streaming (headless){}; Ctrl-C to stop.".format(run_banner(clock)), flush=True)
    while pipeline.isRunning() and not clock.expired():
        remaining = clock.remaining()
        wait_s = 1.0 if remaining is None else max(0.05, min(1.0, remaining))
        try:
            msg = q_pc.get(datetime.timedelta(seconds=wait_s))
        except dai.MessageQueue.QueueException:
            print("Point cloud queue closed - the pipeline stopped.", file=sys.stderr, flush=True)
            return 1
        if msg is None:
            silent_s += wait_s
            if silent_s >= 5.0:
                print("WARNING: no point cloud for {:.0f} s".format(silent_s), file=sys.stderr, flush=True)
                silent_s = 0.0
            continue
        silent_s = 0.0
        raw_points, raw_colors = extract_cloud(msg)
        if state.clouds == 0:
            print_first_cloud(msg)
        points, colors = filter_cloud(raw_points, raw_colors, args.max_depth, args.downsample)
        state.update(points, colors, len(raw_points))
        fps.tick()
        if stats.ready():
            print("[{:5.1f}s] {}  fps={:.1f}".format(
                clock.elapsed(), describe_cloud(points, len(raw_points)), fps.fps()), flush=True)
    return 0 if pipeline.isRunning() else 1


def run_windowed(pipeline: dai.Pipeline, q_pc: dai.MessageQueue, q_depth: dai.MessageQueue,
                 o3d: Any, args: argparse.Namespace, state: CloudState, clock: RunClock) -> int:
    """Open3D viewer + OpenCV depth window; q/Esc quit, s saves a PLY. Returns the exit code."""
    flags = {"quit": False, "save": False}

    # Open3D's plain register_key_callback fires for GLFW press *and* auto-repeat, so the
    # action variant is used and only action == GLFW_PRESS counts (as in the official
    # rgbd_o3d.py example): holding "s" must not write one PLY per repeat event.
    def on_quit(_vis: Any, action: int, _mods: int) -> bool:
        if action == _GLFW_PRESS:
            flags["quit"] = True
        return False

    def on_save(_vis: Any, action: int, _mods: int) -> bool:
        if action == _GLFW_PRESS:
            flags["save"] = True
        return False

    vis = o3d.visualization.VisualizerWithKeyCallback()
    if not vis.create_window(WINDOW_3D, width=1280, height=800):
        print("Open3D could not create a window (no display?); running like --headless.",
              file=sys.stderr, flush=True)
        return run_headless(pipeline, q_pc, args, state, clock)
    for key in (_GLFW_KEY_Q, _GLFW_KEY_ESCAPE):
        vis.register_key_action_callback(key, on_quit)
    vis.register_key_action_callback(_GLFW_KEY_S, on_save)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1))
    pcd = o3d.geometry.PointCloud()
    cloud_added = False

    colour_range_m = args.max_depth if args.max_depth > 0 else DEFAULT_COLOUR_RANGE_M
    fps = FpsCounter()
    exit_code = 0
    last_save: Optional[float] = None  # monotonic time of the last PLY save (key debounce)
    print("Streaming{}; q / Esc to quit, s to save a PLY.".format(run_banner(clock)), flush=True)
    try:
        while pipeline.isRunning() and not clock.expired():
            got_any = False
            try:
                msg = q_pc.tryGet()
                depth_msg = q_depth.tryGet()
            except dai.MessageQueue.QueueException:
                print("Queue closed - the pipeline stopped.", file=sys.stderr, flush=True)
                exit_code = 1
                break

            if msg is not None:
                got_any = True
                raw_points, raw_colors = extract_cloud(msg)
                if state.clouds == 0:
                    print_first_cloud(msg)
                points, colors = filter_cloud(raw_points, raw_colors, args.max_depth, args.downsample)
                state.update(points, colors, len(raw_points))
                fps.tick()
                if len(points) > 0:
                    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                    if colors is not None:
                        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
                    else:
                        pcd.colors = o3d.utility.Vector3dVector(depth_colours(points, colour_range_m))
                    if not cloud_added:
                        vis.add_geometry(pcd)
                        ctr = vis.get_view_control()
                        ctr.set_front([0.0, 0.0, -1.0])   # look along +z from behind the camera
                        ctr.set_up([0.0, -1.0, 0.0])      # image y points down
                        ctr.set_lookat([0.0, 0.0, 1.0])
                        ctr.set_zoom(0.3)
                        cloud_added = True
                    else:
                        vis.update_geometry(pcd)

            if depth_msg is not None:
                got_any = True
                depth_mm = depth_msg.getFrame()
                img = colorize_depth(depth_mm, max_mm=colour_range_m * 1000.0)
                label = "points {}{}  fps {:.1f}  {}x{} depth  range {:.1f} m\nq/Esc quit   s save PLY".format(
                    len(state.points) if state.points is not None else 0,
                    " (colour)" if args.color else "", fps.fps(), depth_mm.shape[1], depth_mm.shape[0],
                    colour_range_m)
                draw_label(img, label)
                cv2.imshow(WINDOW_DEPTH, img)

            if not vis.poll_events():  # window closed by the user
                break
            vis.update_renderer()
            key = wait_key_quit(1)
            if key == "q" or flags["quit"]:
                break
            if key == "s" or flags["save"]:
                flags["save"] = False
                now = time.monotonic()
                # cv2.waitKey also reports the OS auto-repeat while "s" is held down: allow
                # at most one save per SAVE_DEBOUNCE_S from either window.
                if last_save is None or now - last_save >= SAVE_DEBOUNCE_S:
                    last_save = now
                    save_ply(state, SNAPSHOT_DIR, o3d)
            if not got_any:
                time.sleep(0.005)
        if exit_code == 0 and not pipeline.isRunning():
            exit_code = 1  # the loop ended because the pipeline stopped on its own
    finally:
        try:
            vis.destroy_window()
        except Exception:  # noqa: BLE001 - window may already be gone
            pass
        cv2.destroyAllWindows()
    return exit_code


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

def main(argv: Optional[list] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    o3d: Optional[Any] = None
    windowed = not args.headless
    if windowed:
        o3d, o3d_error = try_import_open3d()
        if o3d is None:
            print(open3d_hint(o3d_error), file=sys.stderr, flush=True)
            windowed = False
    elif args.save_on_exit:
        o3d, _ = try_import_open3d()  # only for the PLY writer; the ASCII fallback works without it

    if 0.0 < args.duration <= WARM_UP_SECONDS:
        print("WARNING: --duration {:g} s is not longer than the {:.1f} s auto-exposure warm-up it "
              "includes; expect few or no point clouds.".format(args.duration, WARM_UP_SECONDS),
              file=sys.stderr, flush=True)

    # A wrong --device-id is a user error: fail fast with one stderr line. Without an id the
    # open is retried a few times in case the camera is still rebooting after another script.
    device = open_device(args.device_id, retries=1 if args.device_id else 3)
    print_device_summary(device)

    state = CloudState()
    exit_code = 0
    print("Pipeline: CAM_B+CAM_C {}x{} @ {:g} fps -> StereoDepth(DEFAULT, decimation 1) -> PointCloud "
          "(metres, unorganised){}; max-depth={}, downsample={}".format(
              STREAM_SIZE[0], STREAM_SIZE[1], args.fps,
              " + CAM_A RGB888i undistorted colour aligned via inputAlignTo" if args.color else "",
              "{:g} m".format(args.max_depth) if args.max_depth > 0 else "none", args.downsample),
          flush=True)

    with dai.Pipeline(device) as pipeline:
        try:
            stereo, pc = build_pipeline(pipeline, args)
            q_pc = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
            q_depth: Optional[dai.MessageQueue] = None
            if windowed:
                q_depth = pc.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
            pipeline.start()
            clock = RunClock(args.duration)  # --duration counts from here, warm-up included
            print("Pipeline started (max disparity {:.0f}); letting auto-exposure settle for "
                  "{:.1f} s...".format(stereo.initialConfig.getMaxDisparity(), WARM_UP_SECONDS),
                  flush=True)
            if q_depth is not None:
                warm_up(q_pc, q_depth, seconds=WARM_UP_SECONDS)
            else:
                warm_up(q_pc, seconds=WARM_UP_SECONDS)
            if windowed and q_depth is not None:
                exit_code = run_windowed(pipeline, q_pc, q_depth, o3d, args, state, clock)
            else:
                exit_code = run_headless(pipeline, q_pc, args, state, clock)
        except KeyboardInterrupt:
            print("\nInterrupted (Ctrl-C), stopping.", flush=True)
            exit_code = 0
        except RuntimeError as exc:  # dai.XLinkError & co.: crash at build/start, bad --fps, link dropped
            print("Pipeline error: {}: {}".format(type(exc).__name__, " ".join(str(exc).split())),
                  file=sys.stderr, flush=True)
            exit_code = 1
        finally:
            cv2.destroyAllWindows()
            try:
                if pipeline.isRunning():
                    pipeline.stop()  # also closes the device (cheat-sheet 1.2)
            except Exception as exc:  # noqa: BLE001 - never mask the real exit path
                print("pipeline.stop() failed: {}: {}".format(type(exc).__name__, exc),
                      file=sys.stderr, flush=True)

    print("Processed {} point clouds.".format(state.clouds), flush=True)
    if args.save_on_exit:
        save_ply(state, SNAPSHOT_DIR, o3d)
    return exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:  # Ctrl-C while opening the device / booting the pipeline
        print("\nInterrupted (Ctrl-C).", flush=True)
        sys.exit(0)
