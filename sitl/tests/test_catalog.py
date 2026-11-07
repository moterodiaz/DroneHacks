"""versions.yaml resolution: options (launch file, suffix, requirements), followers and tunables."""
from __future__ import annotations

import os
import unittest

from dronesim.autonomy.catalog import VersionError, load_catalog, resolve_selection

CATALOG = load_catalog(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'versions.yaml'))


class RocksOptionTest(unittest.TestCase):
    def test_flying_over_the_rocks_picks_the_over_rocks_launch_variant(self):
        for vid, expected in (('lidar_allowed', 'lidar_nav_allowed_over_rocks.launch.py'),
                              ('lidar_forbidden', 'lidar_nav_forbidden_over_rocks.launch.py'),
                              ('vo_nolidar', 'vo_nav_over_rocks.launch.py')):
            launch, _params, _hook, notes = resolve_selection(CATALOG[vid], {'rocks': 'fly_over'})
            self.assertEqual(launch['file'], expected, vid)
            self.assertTrue(any(n.startswith('Rocks: fly over') for n in notes), notes)

    def test_the_default_avoids_the_rocks(self):
        launch, params, _hook, _notes = resolve_selection(CATALOG['lidar_allowed'], {})
        self.assertEqual(launch['file'], 'lidar_nav_allowed.launch.py')
        _launch, params, _hook, _notes = resolve_selection(CATALOG['traj'], {})
        self.assertNotIn('avoid_boulders', params.get('mission', {}))

    def test_traj_flies_over_by_parameter_on_any_route(self):
        _launch, params, _hook, _notes = resolve_selection(CATALOG['traj'], {'rocks': 'fly_over', 'route': 'window'})
        self.assertIs(params['mission']['avoid_boulders'], False)

    def test_the_over_rocks_route_exists_only_for_the_window(self):
        with self.assertRaisesRegex(VersionError, 'needs CP4 route = window'):
            resolve_selection(CATALOG['lidar_allowed'], {'rocks': 'fly_over', 'route': 'gate_b'})


class TunablesTest(unittest.TestCase):
    def test_track_speed_and_descent_set_each_versions_own_parameters(self):
        tun = {'track_speed_mps': 0.45, 'descent_mps': 0.25}
        _l, params, _h, _n = resolve_selection(CATALOG['traj'], {}, None, tun)
        self.assertEqual((params['mission']['track_speed_mps'], params['mission']['descent_mps']), (0.45, 0.25))
        for vid in ('lidar_allowed', 'lidar_forbidden', 'return_flight', 'vo_nolidar'):
            _l, params, _h, _n = resolve_selection(CATALOG[vid], {}, None, tun)
            self.assertEqual((params['follower']['track_speed_mps'], params['follower']['descent_mps']),
                             (0.45, 0.25), vid)

    def test_unset_tunables_leave_the_version_alone(self):
        _l, params, _h, _n = resolve_selection(CATALOG['vo_nolidar'], {}, None, {'track_speed_mps': None})
        self.assertNotIn('track_speed_mps', params.get('follower', {}))

    def test_a_version_without_the_parameter_says_so(self):
        _l, params, _h, notes = resolve_selection(CATALOG['kit_square'], {}, None, {'track_speed_mps': 0.5})
        self.assertEqual(params, {})
        self.assertTrue(any('not adjustable in this version' in n for n in notes), notes)


class RefResolutionTest(unittest.TestCase):
    def test_a_branch_only_on_origin_resolves_in_a_fresh_clone(self):
        import subprocess
        import tempfile
        from dronesim.autonomy.catalog import resolve_ref
        with tempfile.TemporaryDirectory() as repo:
            def git(*args):
                return subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True, check=True).stdout.strip()
            git('init', '-q')
            git('-c', 'user.name=t', '-c', 'user.email=t@example.invalid', 'commit', '-q', '--allow-empty', '-m', 'x')
            sha = git('rev-parse', 'HEAD')
            git('update-ref', 'refs/remotes/origin/navigation/some-branch', sha)     # remote-tracking only
            got, subject = resolve_ref(repo, 'navigation/some-branch')
            self.assertEqual(got, sha)
            self.assertIn('via origin/navigation/some-branch', subject)
            with self.assertRaisesRegex(VersionError, 'not found locally or as origin/'):
                resolve_ref(repo, 'navigation/missing')


if __name__ == '__main__':
    unittest.main()
