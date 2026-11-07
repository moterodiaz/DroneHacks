"""NVIDIA's calling pattern, the recording fed twice in a row.

WSL2 experiment behind the numbers in ../README.md (2026-09-30).  Needs the real cuvslam wheel and a
recording copied to ~/datasets/<name>.  Run from anywhere:
    python slam_probe_two_pass.py <recording name> <stereo|vio> <rect|norect> [map_cell_size]
"""
import os, sys
from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))
import time, copy
import cuvslam
import numpy as np
from cuvslam_oak import dataset, tracking as T
import importlib.util
spec = importlib.util.spec_from_file_location("s08", str(REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py")); s08 = importlib.util.module_from_spec(spec); spec.loader.exec_module(s08)
name, mode = sys.argv[1], sys.argv[2]; rectified = sys.argv[3] == "rect"; cell = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
reader = dataset.EurocReader(os.path.expanduser(f"~/datasets/{name}"))
rig = s08.rig_from_dataset(cuvslam, reader.rig, mode)
odom_cfg = cuvslam.Tracker.OdometryConfig(async_sba=False, enable_observations_export=True, enable_landmarks_export=True,
                                          enable_final_landmarks_export=True, rectified_stereo_camera=rectified,
                                          odometry_mode=cuvslam.Tracker.OdometryMode.Multicamera if mode == "stereo" else cuvslam.Tracker.OdometryMode.Inertial)
SYNC = os.environ.get("SLAM_SYNC", "1") == "1"; MAPSZ = int(os.environ.get("SLAM_MAP", "0"))
s_cfg = cuvslam.Tracker.SlamConfig(sync_mode=SYNC, enable_reading_internals=True, max_map_size=MAPSZ, map_cell_size=cell)
print("sync", SYNC, "map size", MAPSZ)
tracker = cuvslam.Tracker(rig, odom_cfg, s_cfg)
frames = list(reader.frames(load_images=False))
span = frames[-1].ts_ns - frames[0].ts_ns + 1_000_000_000
n_lc = 0; last_lc = 0; lc_at = []; pgo_frames = []; diverged = False
pos_end = {}
for p in range(2):
    for f in frames:
        ts = f.ts_ns + p * span
        imgs = f.images()
        est, slam_pose = tracker.track(ts, imgs)
        if est.world_from_rig is None: continue
        lc = tracker.get_loop_closure_poses()
        if len(lc) != last_lc:
            n_lc += 1; lc_at.append((p, f.index)); last_lc = len(lc)
        mm = tracker.get_slam_metrics()
        if mm.pgo_status or mm.lc_status:
            pgo_frames.append((p, f.index, bool(mm.lc_status), bool(mm.pgo_status)))
        d = np.linalg.norm(np.array(est.world_from_rig.pose.translation) - np.array(slam_pose.translation))
        if d > 0.01 and not diverged:
            diverged = True; print(f"  slam pose diverges from odometry at pass {p} frame {f.index}: {d:.3f} m")
        pos_end[p] = (np.array(est.world_from_rig.pose.translation), np.array(slam_pose.translation))
    print(f"pass {p}: odom end {np.round(pos_end[p][0], 2)} slam end {np.round(pos_end[p][1], 2)}; loop closures so far {n_lc} at {lc_at[:6]}")
m = tracker.get_slam_metrics(); g = tracker.get_pose_graph()
print("frames with lc/pgo flags:", len(pgo_frames), pgo_frames[:10])
time.sleep(3)
print("metrics:", {k: getattr(m, k) for k in ("lc_status", "pgo_status", "lc_selected_landmarks_count", "lc_good_landmarks_count")}, "| graph", (len(list(g.nodes)), len(list(g.edges))), "| all slam poses", len(tracker.get_all_slam_poses()))
for layer in ("Landmarks", "Map", "LoopClosure"):
    lm = tracker.get_slam_landmarks(getattr(cuvslam.core.Slam.DataLayer, layer)); print(" layer", layer, len(list(lm.landmarks)))
