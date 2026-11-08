"""Saved runs as GUI map overlays (dronesim/gui/saved.py): finding, reading and labelling them."""
import json
import os
import shutil
import tempfile
import unittest

from dronesim.gui import saved


def write_run(d, arena='taped-2026-11-08', line='window_2mps', seed=2, armed_from=2, rows=6, plan=True):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'trajectory.csv'), 'w', encoding='utf-8', newline='') as fh:
        fh.write('t,x,y,z,yaw,speed,nav_state,armed,ekf_x,ekf_y,ekf_up,vo\n')
        for i in range(rows):
            fh.write(f'{0.05 * i:.2f},{0.1 * i:.4f},{0.2 * i:.4f},1.0,0,0,14,{1 if i >= armed_from else 0},0,0,0,\n')
    result = {'scenario': {'options': {'line': line, 'land': 'land'}, 'seed': seed},
              'arena': {'name': arena},
              'referee': {'checkpoint_points': 31.0, 'window_bonus': 5, 'finished': True, 'window_route': True,
                          'time_s': 22.4, 'touches': 0, 'ended': None}}
    with open(os.path.join(d, 'result.json'), 'w', encoding='utf-8') as fh:
        json.dump(result, fh)
    if plan:
        with open(os.path.join(d, 'plan.json'), 'w', encoding='utf-8') as fh:
            json.dump({'xy': [[0.0, 0.0], [1.0, 0.5], [2.0, 1.0]], 'length_m': 2.2, 'duration_s': 3.0}, fh)


class SavedRunsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='dronesim_saved_')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_a_folder_of_runs_or_one_run(self):
        write_run(os.path.join(self.tmp, 'b_run'))
        write_run(os.path.join(self.tmp, 'a_run'))
        os.makedirs(os.path.join(self.tmp, 'not_a_run'))
        self.assertEqual([os.path.basename(p) for p in saved.find_runs(self.tmp)], ['a_run', 'b_run'])
        one = os.path.join(self.tmp, 'a_run')
        self.assertEqual(saved.find_runs(one), [one])
        self.assertEqual(saved.find_runs(os.path.join(self.tmp, 'missing')), [])

    def test_a_run_gives_its_plan_its_armed_trail_and_a_label(self):
        d = os.path.join(self.tmp, 'v2_s2')
        write_run(d)
        o = saved.load_run(d, '#123456')
        self.assertEqual(o['colour'], '#123456')
        self.assertEqual(o['arena'], 'taped-2026-11-08')
        self.assertEqual(o['plan'], [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)])
        self.assertEqual(len(o['trail']), 4)                    # the two disarmed rows are left out
        self.assertEqual(o['trail'][0], (0.2, 0.4))
        self.assertEqual(o['label'], 'v2_s2: window_2mps, seed 2, 31+5 pts, 22.4 s')

    def test_long_trails_are_thinned_and_keep_their_end(self):
        d = os.path.join(self.tmp, 'long')
        write_run(d, armed_from=0, rows=4000)
        o = saved.load_run(d)
        self.assertLessEqual(len(o['trail']), saved.MAX_TRAIL_POINTS + 1)
        self.assertAlmostEqual(o['trail'][-1][0], 0.1 * 3999, places=3)

    def test_colours_follow_the_load_order_and_the_arena_is_shared_or_none(self):
        write_run(os.path.join(self.tmp, 'a'))
        write_run(os.path.join(self.tmp, 'b'), plan=False)
        overlays = saved.load_paths([self.tmp], start_index=1)
        self.assertEqual([o['colour'] for o in overlays], list(saved.COLOURS[1:3]))
        self.assertEqual(overlays[1]['plan'], [])
        self.assertEqual(saved.common_arena(overlays), 'taped-2026-11-08')
        write_run(os.path.join(self.tmp, 'c'), arena='step')
        self.assertIsNone(saved.common_arena(saved.load_paths([self.tmp])))


if __name__ == '__main__':
    unittest.main()
