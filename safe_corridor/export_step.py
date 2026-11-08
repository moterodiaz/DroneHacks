"""CAD -> triangle mesh for the safe-corridor planner.

Uses the SITL's STEP mesher (sitl/dronesim/world), including a course-edits/1 file such as the
course as taped on 2026-11-08, and writes every solid's triangles (course frame, metres) to .npz.

    py export_step.py --sitl-dir ../sitl --step ../dronehacks-course-v3.step \
        --edits ../sitl/course/arena_2026-11-08_edits.json --out courses/taped_2026-11-08.npz
"""
import argparse
import os
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sitl-dir', required=True)
    ap.add_argument('--step', required=True)
    ap.add_argument('--edits')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    sys.path.insert(0, os.path.abspath(a.sitl_dir))
    from dronesim.world.course import Course
    cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
    c = Course(a.step, cache, a.edits)
    tris, names, kinds = [], [], []
    for s, k in zip(c.solids, c.kinds):
        t = np.asarray(s.triangles, dtype=np.float32).reshape(-1, 3, 3)
        tris.append(t)
        names += [s.name] * len(t)
        kinds += [k] * len(t)
    np.savez_compressed(a.out, triangles=np.concatenate(tris), names=np.array(names), kinds=np.array(kinds),
                        source=np.array(f'{os.path.basename(a.step)} sha256 {c.sha256}; edits: {c.arena}'))
    print(f'{a.out}: {sum(len(t) for t in tris)} triangles from {len(c.solids)} solids ({c.arena})')


if __name__ == '__main__':
    main()
