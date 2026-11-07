#!/usr/bin/env python3
"""Benchmark ``cuvslam_oak.dataset.EurocWriter``'s PNG pipeline without a camera.

Replays one real stereo pair (by default the first pair of ``recordings/euroc_20260920_034208``,
1280x800; synthetic noise when that folder is missing) through a real ``EurocWriter`` at a target
frame rate and reports whether the writer kept up: pairs offered / written / dropped, the deepest
the input queue got, the achieved rates, the mean encode and commit times and the wall time.  It
is the tool behind the ``--png-workers`` default of ``datasets/07_record_dataset.py``.

How to run (from the repository root, Windows or Linux, no camera needed)
-------------------------------------------------------------------------
    python datasets/bench_png_writer.py                                  # 300 pairs @ 30 fps, auto workers
    python datasets/bench_png_writer.py --png-workers 1,0,4              # one row per setting
    python datasets/bench_png_writer.py --out recordings --png-workers 0 # write into the OneDrive folder
    python datasets/bench_png_writer.py --width 640 --height 400 --png-compression 0
    python datasets/bench_png_writer.py --source recordings/euroc_20260921_000641 --index 100

The output folder is a fresh ``bench_png_<time>_w<N>`` directory (under ``--out``, or the system
temp directory) and is deleted afterwards unless ``--keep`` is given.  Exit code 0 when every run
was drop-free, 1 otherwise.

Reading the numbers
-------------------
``written/s (wall)`` is pairs written over the whole run including the drain at close, i.e. what a
recorder would get; ``throughput (busy)`` is the writer's own ``png_throughput_pairs_per_s`` (pairs
per second of committer busy time), i.e. the sustainable rate of this pipeline on this disk;
``encode ms`` is the mean per-pair ``cv2.imencode`` time inside the pool (it grows with the thread
count because the threads share the cores), ``commit ms`` the mean committer wall time per pair.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cuvslam_oak import dataset  # noqa: E402  (import after sys.path tweak on purpose)

DEFAULT_SOURCE = REPO_ROOT / "recordings" / "euroc_20260920_034208"
DEFAULT_FPS = 30.0
DEFAULT_PAIRS = 300
DEFAULT_QUEUE_SIZE = 64
DEFAULT_PNG_COMPRESSION = 1
DEFAULT_PNG_WORKERS = "0"
T0_NS = 1_758_000_000_000_000_000


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, metavar="DIR",
                        help="recording (root or mav0) whose PNG pair is replayed (default {}; synthetic noise "
                             "when missing)".format(DEFAULT_SOURCE.relative_to(REPO_ROOT).as_posix()))
    parser.add_argument("--index", type=int, default=0, metavar="ROW",
                        help="which csv row of the source to replay (default 0 = the first pair)")
    parser.add_argument("--width", type=int, default=None, help="resize the source pair to this width")
    parser.add_argument("--height", type=int, default=None, help="resize the source pair to this height")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="offered frame rate (default {:g})".format(
        DEFAULT_FPS))
    parser.add_argument("--pairs", type=int, default=DEFAULT_PAIRS, metavar="N",
                        help="pairs offered per run (default {})".format(DEFAULT_PAIRS))
    parser.add_argument("--png-workers", dest="png_workers", type=str, default=DEFAULT_PNG_WORKERS, metavar="N[,N...]",
                        help="encoder thread counts to benchmark, one run each; 0 = auto (default {})".format(
                            DEFAULT_PNG_WORKERS))
    parser.add_argument("--png-compression", dest="png_compression", type=int, default=DEFAULT_PNG_COMPRESSION,
                        metavar="0-9", help="PNG compression level (default {})".format(DEFAULT_PNG_COMPRESSION))
    parser.add_argument("--queue-size", dest="queue_size", type=int, default=DEFAULT_QUEUE_SIZE, metavar="PAIRS",
                        help="writer input queue depth (default {})".format(DEFAULT_QUEUE_SIZE))
    parser.add_argument("--out", type=Path, default=None, metavar="DIR",
                        help="parent folder for the benchmark datasets (default: the system temp directory); a "
                             "fresh bench_png_<time>_w<N> folder is created under it per run")
    parser.add_argument("--keep", action="store_true", help="keep the benchmark datasets instead of deleting them")
    args = parser.parse_args(argv)
    try:
        args.worker_list = [int(v) for v in args.png_workers.split(",") if v.strip() != ""]
    except ValueError:
        parser.error("--png-workers must be integers separated by commas, got {!r}".format(args.png_workers))
    if not args.worker_list or any(v < 0 for v in args.worker_list):
        parser.error("--png-workers values must be >= 0")
    if args.pairs < 1 or args.fps <= 0 or args.queue_size < 1:
        parser.error("--pairs and --queue-size must be >= 1, --fps > 0")
    if not (0 <= args.png_compression <= 9):
        parser.error("--png-compression must be within 0..9")
    if (args.width is None) != (args.height is None):
        parser.error("--width and --height go together")
    return args


# --------------------------------------------------------------------------------------
# Source pair
# --------------------------------------------------------------------------------------

def load_source_pair(source: Path, index: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    """``(left, right, description)`` from row *index* of the source recording; ``(None, None, why)`` if unusable."""
    try:
        _, mav0 = dataset.resolve_mav0(source)
    except (dataset.DatasetError, OSError) as exc:
        return None, None, "{}: {}".format(source, exc)
    csv0 = mav0 / "cam0" / "data.csv"
    if not csv0.is_file():
        return None, None, "{} has no cam0/data.csv".format(mav0)
    rows = [line.split(",") for line in csv0.read_text(encoding="utf-8").splitlines()[1:] if line.strip()]
    if not rows or index >= len(rows) or index < 0:
        return None, None, "{} has {} rows, no row {}".format(csv0, len(rows), index)
    name = rows[index][1].strip()
    imgs = []
    for cam in ("cam0", "cam1"):
        path = mav0 / cam / "data" / name
        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if img is None or img.ndim != 2 or img.dtype != np.uint8:
            return None, None, "{}: not an 8-bit single-channel PNG".format(path)
        imgs.append(img)
    return imgs[0], imgs[1], "{} row {} ({})".format(mav0.parent.name, index, name)


def synthetic_pair(width: int, height: int, seed: int = 1234) -> Tuple[np.ndarray, np.ndarray]:
    """Two independent uniform-noise images: the worst case for PNG (nothing to compress)."""
    rng = np.random.default_rng(seed)
    return (rng.integers(0, 256, size=(height, width), dtype=np.uint8),
            rng.integers(0, 256, size=(height, width), dtype=np.uint8))


def make_rig(width: int, height: int) -> dataset.RigInfo:
    """A pinhole rig of the given size with a 75 mm baseline (the writer only needs the geometry)."""
    f = 0.5 * width
    cam0 = dataset.CameraInfo(width=width, height=height, fu=f, fv=f, cu=width / 2.0, cv=height / 2.0,
                              socket="CAM_B", role="left", sensor="bench")
    T1 = np.eye(4)
    T1[0, 3] = 0.075
    cam1 = dataset.CameraInfo(width=width, height=height, fu=f, fv=f, cu=width / 2.0, cv=height / 2.0,
                              T_BS=T1, socket="CAM_C", role="right", sensor="bench")
    return dataset.RigInfo(cam0=cam0, cam1=cam1, imu=dataset.ImuInfo(), device_id="bench")


# --------------------------------------------------------------------------------------
# One run
# --------------------------------------------------------------------------------------

def run_once(out_parent: Path, left: np.ndarray, right: np.ndarray, *, fps: float, pairs: int, png_workers: int,
             png_compression: int, queue_size: int, keep: bool) -> Dict[str, Any]:
    """Replay *pairs* copies of the pair at *fps* through one ``EurocWriter``; return the numbers."""
    height, width = left.shape
    root = out_parent / "bench_png_{}_w{}".format(time.strftime("%H%M%S"), png_workers)
    root.mkdir(parents=True, exist_ok=False)
    period = 1.0 / fps
    writer = dataset.EurocWriter(root, make_rig(width, height), cam_rate_hz=fps, imu_rate_hz=200.0,
                                 queue_size=queue_size, png_compression=png_compression, png_workers=png_workers)
    max_depth = 0
    late_ms_max = 0.0
    t_start = time.perf_counter()
    try:
        for i in range(pairs):
            deadline = t_start + i * period
            while True:
                now = time.perf_counter()
                remaining = deadline - now
                if remaining <= 0.0:
                    break
                time.sleep(remaining if remaining < 0.002 else remaining - 0.001)
            late_ms_max = max(late_ms_max, 1000.0 * (time.perf_counter() - deadline))
            writer.add_frame(T0_NS + int(round(i * period * 1e9)), left, right)
            max_depth = max(max_depth, writer.queue_depth)
        t_offered = time.perf_counter()
        summary = writer.close(stopped_by="bench")
    finally:
        if not writer.closed:
            writer.close(stopped_by="exception")
    t_end = time.perf_counter()
    counts, measured, drops, record = summary["counts"], summary["measured"], summary["drops"], summary["record"]
    written = int(counts["frames_written"])
    result = {
        "png_workers": int(record["png_workers"]), "requested_workers": png_workers, "root": root,
        "offered": pairs, "written": written, "dropped": int(drops["stereo_write_queue_dropped"]),
        "failed": int(counts["frames_failed"]), "max_queue_depth": max_depth, "queue_size": queue_size,
        "offered_fps": pairs / (t_offered - t_start) if t_offered > t_start else 0.0,
        "written_per_s_wall": written / (t_end - t_start) if t_end > t_start else 0.0,
        "throughput_busy": float(measured["png_throughput_pairs_per_s"]),
        "encode_ms": float(measured["png_encode_ms_mean"]), "commit_ms": float(measured["png_write_ms_mean"]),
        "wall_s": t_end - t_start, "drain_s": t_end - t_offered, "late_ms_max": late_ms_max,
        "bytes_per_pair": (float(measured["png_bytes_total"]) / written) if written else 0.0,
        "warnings": list(summary.get("warnings") or []),
    }
    if not keep:
        shutil.rmtree(root, ignore_errors=True)
        result["root"] = None
    return result


def format_row(r: Dict[str, Any]) -> str:
    return ("{:>7d}  {:>7d}  {:>7d}  {:>7d}  {:>6d}  {:>9.1f}  {:>10.1f}  {:>10.1f}  {:>9.1f}  {:>9.1f}  {:>7.2f}  "
            "{:>7.2f}".format(r["png_workers"], r["written"], r["dropped"], r["failed"], r["max_queue_depth"],
                              r["offered_fps"], r["written_per_s_wall"], r["throughput_busy"], r["encode_ms"],
                              r["commit_ms"], r["wall_s"], r["drain_s"]))


HEADER = ("workers  written  dropped   failed  max q  offer fps  written/s   thru/s     encode ms  commit ms  "
          "wall s   drain s")


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    left, right, source_desc = load_source_pair(args.source, args.index)
    if left is None or right is None:
        width, height = (args.width or 1280), (args.height or 800)
        left, right = synthetic_pair(width, height)
        print("source: synthetic uniform noise {}x{} ({}; PNG worst case)".format(width, height, source_desc))
    else:
        if args.width is not None and args.height is not None and (args.width, args.height) != (
                left.shape[1], left.shape[0]):
            left = cv2.resize(left, (args.width, args.height), interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, (args.width, args.height), interpolation=cv2.INTER_AREA)
        print("source: {} -> {}x{}, mean {:.1f}, std {:.1f}".format(source_desc, left.shape[1], left.shape[0],
                                                                  float(left.mean()), float(left.std())))
    out_parent = args.out.expanduser().resolve() if args.out is not None else Path(tempfile.gettempdir()).resolve()
    out_parent.mkdir(parents=True, exist_ok=True)
    print("out: {}   {} pairs @ {:g} fps, compression {}, queue {}, cpu_count {}, auto workers = {}".format(
        out_parent, args.pairs, args.fps, args.png_compression, args.queue_size, dataset.os.cpu_count(),
        dataset.resolve_png_workers(0)))
    print(HEADER)
    results: List[Dict[str, Any]] = []
    for workers in args.worker_list:
        r = run_once(out_parent, left, right, fps=args.fps, pairs=args.pairs, png_workers=workers,
                     png_compression=args.png_compression, queue_size=args.queue_size, keep=args.keep)
        results.append(r)
        print(format_row(r), flush=True)
    for r in results:
        extra = "  workers {} ({}): {:.0f} KB/pair, pacing late max {:.1f} ms".format(
            r["png_workers"], "auto" if r["requested_workers"] == 0 else "requested", r["bytes_per_pair"] / 1024.0,
            r["late_ms_max"])
        if r["root"] is not None:
            extra += ", kept at {}".format(r["root"])
        print(extra)
        for w in r["warnings"]:
            print("    warning: {}".format(w))
    return 0 if all(r["dropped"] == 0 and r["failed"] == 0 for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
