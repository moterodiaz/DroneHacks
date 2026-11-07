"""Several simulator processes at once: the shared caches, and the scenario's speed cap.

Running simulations in parallel used to fail: each new run deleted the build trees of the others,
live ones included, because their age was measured on the *simulated* clock (a day ahead of the
real one), so their nodes could not import their own packages.
"""
from __future__ import annotations

import os
import subprocess
import tempfile
import threading
import unittest
from unittest import mock

from dronesim import sim
from dronesim.autonomy import catalog
from dronesim.px4.params import load_profile
from dronesim.rosfake import runtime

REPO_ROOT = sim.REPO_ROOT


class WorkspaceCleanupTest(unittest.TestCase):
    def test_ages_are_measured_on_the_real_clock(self):
        with tempfile.TemporaryDirectory() as root:
            live = os.path.join(root, 'run1234-live')
            old = os.path.join(root, 'run5678-old')
            other = os.path.join(root, 'notes')
            for d in (live, old, other):
                os.makedirs(d)
            past = runtime.REAL_TIME() - 7 * 3600
            os.utime(old, (past, past))
            os.utime(other, (past, past))
            # while a simulation runs, time.time is its clock: a day ahead of the real one
            with mock.patch('time.time', lambda: runtime.REAL_TIME() + 26 * 3600):
                sim._clean_old_workspaces(root)
            self.assertTrue(os.path.isdir(live), 'a live run tree was removed')
            self.assertFalse(os.path.isdir(old), 'a tree older than the limit was kept')
            self.assertTrue(os.path.isdir(other), 'only run trees are cleaned')


class ArchiveOnceTest(unittest.TestCase):
    def setUp(self):
        self.sha = subprocess.run(['git', '-C', REPO_ROOT, 'rev-parse', 'HEAD'], capture_output=True,
                                  text=True, check=True).stdout.strip()
        self.pkgs = ['sitl/course']

    def test_parallel_extractions_of_one_commit_all_see_the_whole_tree(self):
        with tempfile.TemporaryDirectory() as root:
            results, errors = [], []

            def work():
                try:
                    results.append(catalog._archive_once(REPO_ROOT, self.sha, self.pkgs, root))
                except Exception as exc:  # noqa: BLE001 - collected for the assertion
                    errors.append(exc)

            threads = [threading.Thread(target=work) for _ in range(6)]
            for th in threads:
                th.start()
            for th in threads:
                th.join()
            self.assertEqual(errors, [])
            self.assertEqual(len(set(results)), 1)
            dest = results[0]
            self.assertTrue(os.path.isfile(os.path.join(dest, '.complete')))
            self.assertTrue(os.path.isfile(os.path.join(dest, 'sitl', 'course', 'course_v3_rules.json')))
            self.assertEqual(os.listdir(root), [os.path.basename(dest)], 'temporary trees left behind')

    def test_an_interrupted_extraction_is_replaced_not_reused(self):
        with tempfile.TemporaryDirectory() as root:
            dest = os.path.join(root, self.sha[:12])
            os.makedirs(os.path.join(dest, 'sitl'))
            with open(os.path.join(dest, 'sitl', 'half-written.py'), 'w', encoding='utf-8') as fh:
                fh.write('# interrupted\n')
            self.assertEqual(catalog._archive_once(REPO_ROOT, self.sha, self.pkgs, root), dest)
            self.assertTrue(os.path.isfile(os.path.join(dest, '.complete')))
            self.assertFalse(os.path.exists(os.path.join(dest, 'sitl', 'half-written.py')))
            self.assertEqual(os.listdir(root), [os.path.basename(dest)])


class SpeedCapTest(unittest.TestCase):
    def test_the_speed_cap_overrides_mpc_xy_vel_max_last(self):
        sc = sim.Scenario(param_overrides={'MPC_XY_VEL_MAX': 0.5, 'EKF2_EV_DELAY': 0.0}, speed_cap_mps=1.5)
        over = sim.scenario_param_overrides(sc)
        self.assertEqual(over, {'MPC_XY_VEL_MAX': 1.5, 'EKF2_EV_DELAY': 0.0})
        self.assertEqual(load_profile(REPO_ROOT, 'drone1-fleet', over).f('MPC_XY_VEL_MAX'), 1.5)
        self.assertEqual(sim.scenario_param_overrides(sim.Scenario()), {})

    def test_a_speed_cap_must_be_positive(self):
        for bad in (0.0, -1.0, float('nan')):
            with self.assertRaises(ValueError):
                sim.scenario_param_overrides(sim.Scenario(speed_cap_mps=bad))


if __name__ == '__main__':
    unittest.main()
