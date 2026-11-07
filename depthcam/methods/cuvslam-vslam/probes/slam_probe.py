"""One pass through our loop with SLAM on: every SLAM getter afterwards.

WSL2 experiment behind the numbers in ../README.md (2026-09-30).  Needs the real cuvslam wheel and a
recording copied to ~/datasets/<name>.  Run from anywhere:
    python slam_probe.py <recording name> <stereo|vio> <max_map_size> <sync|async> [final]
"""
import os, sys
from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))
import time
import cuvslam
from cuvslam_oak import dataset, tracking as T
import numpy as np
name = sys.argv[1]; mode = sys.argv[2]; map_size = int(sys.argv[3]); sync = sys.argv[4] == "sync"
reader = dataset.EurocReader(os.path.expanduser(f"~/datasets/{name}"))
import importlib.util
spec = importlib.util.spec_from_file_location("s08", str(REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py")); s08 = importlib.util.module_from_spec(spec); spec.loader.exec_module(s08)
rig = s08.rig_from_dataset(cuvslam, reader.rig, mode)
cfg = T.TrackConfig(verbose=False, warmup_frames=0, slam=True, slam_max_map_size=map_size, slam_sync=sync, coast=False, final_landmarks=(len(sys.argv) > 5 and sys.argv[5] == "final"))
tracker, odom_cfg = T.make_tracker(cuvslam, rig, mode, cfg)
scfg = T.slam_config(cuvslam, cfg)
print("SlamConfig:", {k: getattr(scfg, k) for k in ("sync_mode", "enable_reading_internals", "max_map_size", "map_cell_size", "max_landmarks_distance", "throttling_time_ms")})
print("tracker.slam is", type(tracker.slam).__name__ if tracker.slam else None)
lc_frames = []
def on_pose(e):
    m = tracker.get_slam_metrics()
    if m is not None and (m.lc_status or m.pgo_status): lc_frames.append((e.index, bool(m.lc_status), bool(m.pgo_status), m.lc_selected_landmarks_count, m.lc_tracked_landmarks_count, m.lc_pnp_landmarks_count, m.lc_good_landmarks_count))
summary = T.run_tracking(cuvslam, rig, mode, reader.merged(load_images=True, with_imu=(mode=="vio")), None, cfg, on_pose=on_pose, tracker=(tracker, odom_cfg))
time.sleep(2.0)
g = tracker.get_pose_graph(); m = tracker.get_slam_metrics(); lc = tracker.get_loop_closure_poses(); allp = tracker.get_all_slam_poses()
print("tracked", summary["frames_tracked"], "odom offset %.3f" % summary["endpoint_offset_m"], "| slam rows", summary["slam"]["rows"], "slam offset %.3f" % summary["slam"]["endpoint_offset_m"], "opt offset %.3f" % summary["slam"]["optimized_endpoint_offset_m"])
print("pose graph:", None if g is None else (len(list(g.nodes)), len(list(g.edges))), "| all slam poses:", len(allp), "| loop closure poses:", len(lc))
print("metrics:", None if m is None else {k: getattr(m, k) for k in ("timestamp_ns", "lc_status", "pgo_status", "lc_selected_landmarks_count", "lc_tracked_landmarks_count", "lc_pnp_landmarks_count", "lc_good_landmarks_count")})
print("frames with lc/pgo status set:", len(lc_frames), lc_frames[:8])
for layer in ("Landmarks", "Map", "LoopClosure"):
    try:
        lm = tracker.get_slam_landmarks(getattr(cuvslam.Tracker.SlamDataLayer if hasattr(cuvslam.Tracker, "SlamDataLayer") else cuvslam.core.Slam.DataLayer, layer))
        print("landmarks layer", layer, ":", None if lm is None else len(list(lm.landmarks)))
    except Exception as ex:
        print("landmarks layer", layer, "error:", ex)
