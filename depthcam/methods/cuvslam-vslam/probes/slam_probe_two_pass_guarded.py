"""The recording fed twice through our guarded loop with SLAM on.

WSL2 experiment behind the numbers in ../README.md (2026-09-30).  Needs the real cuvslam wheel and a
recording copied to ~/datasets/<name>.  Run from anywhere:
    python slam_probe_two_pass_guarded.py <recording name> <stereo|vio>
"""
import os, sys
from pathlib import Path
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT))
import time
import cuvslam, numpy as np
from cuvslam_oak import dataset, tracking as T
import importlib.util
spec = importlib.util.spec_from_file_location("s08", str(REPO_ROOT / "methods" / "cuvslam-vio" / "08_cuvslam_track.py")); s08 = importlib.util.module_from_spec(spec); spec.loader.exec_module(s08)
name, mode = sys.argv[1], sys.argv[2]; sync = os.environ.get("SLAM_SYNC", "1") == "1"; mapsz = int(os.environ.get("SLAM_MAP", "0"))
reader = dataset.EurocReader(os.path.expanduser(f"~/datasets/{name}"))
rig = s08.rig_from_dataset(cuvslam, reader.rig, mode)
span = reader.last_ts_ns - reader.first_ts_ns + 1_000_000_000
def shifted(item, k):
    if item.kind == "imu":
        return ("imu", item.ts_ns + k * span, item.gyro, item.accel)
    return ("stereo", item.ts_ns + k * span, item.images())
def stream():
    for k in range(2):
        for item in reader.merged(load_images=False, with_imu=(mode == "vio")):
            yield shifted(item, k)
cfg = T.TrackConfig(verbose=False, warmup_frames=0, slam=True, slam_sync=sync, slam_max_map_size=mapsz, coast=True)
events = []
summary = T.run_tracking(cuvslam, rig, mode, stream(), None, cfg, on_pose=events.append)
first_end = reader.last_ts_ns
p0 = [e for e in events if e.ts_ns <= first_end and not e.coasted]; p1 = [e for e in events if e.ts_ns > first_end and not e.coasted]
def div(evs):
    d = [np.linalg.norm(e.translation - e.slam_translation) for e in evs if e.slam_translation is not None]
    return (max(d) if d else 0.0, sum(1 for x in d if x > 0.02))
print(f"{name} {mode} sync={sync} map={mapsz}: tracked {summary['frames_tracked']}, guarded {summary['frames_guarded']}, resets {summary['resets']}, loop closures {summary['slam']['loop_closures']}, pgo {summary['slam']['pgo_events']}, map resets {summary['slam']['map_resets']}")
print("  pass 1 end odom", np.round(p0[-1].translation, 2), "slam", np.round(p0[-1].slam_translation, 2), "| max |slam-odom| %.3f m, frames > 2 cm: %d" % div(p0))
print("  pass 2 end odom", np.round(p1[-1].translation, 2), "slam", np.round(p1[-1].slam_translation, 2), "| max |slam-odom| %.3f m, frames > 2 cm: %d" % div(p1))
print("  optimised rows", summary["slam"]["optimized_rows"], "optimised end offset %.3f" % summary["slam"]["optimized_endpoint_offset_m"], "odom end offset %.3f" % summary["endpoint_offset_m"])
