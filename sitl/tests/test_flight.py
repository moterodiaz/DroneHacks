"""End-to-end flights: the organizer procedure on the real kit code and team code.

These run the full simulation (tens of seconds of CPU each).  The traj and
landing flights need navigation/new-course-layout and the return
flight needs localization/return-flight (a local branch); each is skipped if
its ref is missing.
"""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL = os.path.dirname(HERE)
REPO = os.path.dirname(SITL)
sys.path.insert(0, SITL)


def _have_ref(ref):
    # the catalog's own rule: the ref itself, else origin/<ref> (a fresh clone)
    return any(subprocess.run(['git', '-C', REPO, 'rev-parse', '--verify', candidate],
                              capture_output=True).returncode == 0 for candidate in (ref, 'origin/' + ref))


def _run(version, until, **kw):
    code = (
        'import sys, json; sys.path.insert(0, %r)\n'
        'from dronesim.sim import Scenario, Simulation\n'
        'sc = Scenario(version=%r, duration_s=%r, **%r)\n'
        'sim = Simulation(sc)\n'
        'sim.run()\n'
        'r = sim.result(); st = sim.snapshot()\n'
        'tr = sim.trajectory; hi = [i for i, row in enumerate(tr) if row[3] > 0.6]\n'
        'td = next((row[0] for row in tr[hi[-1]:] if row[3] < 0.08), None) if hi else None\n'
        'print("RESULT" + json.dumps({"events": r["events"], "ref": r["referee"], "procs": r["processes"],'
        ' "nav": st["px4"]["nav_name"], "z": st["true"]["z"], "msgs": r["px4_messages"], "touchdown": td}))\n'
        'sim.close()\n' % (SITL, version, until, kw))
    out = subprocess.run([sys.executable, '-B', '-c', code], capture_output=True, text=True, timeout=1500)
    for line in out.stdout.splitlines():
        if line.startswith('RESULT'):
            import json
            return json.loads(line[6:])
    raise AssertionError(out.stdout[-3000:] + out.stderr[-3000:])


class KitProcedureTest(unittest.TestCase):
    def test_arm_then_position_enters_offboard_and_takes_off(self):
        r = _run('kit_square', 26.0)
        msgs = ' | '.join(m[2] for m in r['msgs'])
        self.assertIn('Armed by RC switch', msgs)
        self.assertEqual(r['nav'], 'OFFBOARD')
        self.assertGreater(r['z'], 1.2)               # the example hovers at 1.5 m above its start
        self.assertTrue(all(p['alive'] for p in r['procs']))


@unittest.skipUnless(_have_ref('navigation/new-course-layout'), 'traj ref not present')
class TrajFlightTest(unittest.TestCase):
    def test_traj_reaches_the_tunnel_exit_when_the_tunnel_is_lit(self):
        # with the default dark tunnel cuVSLAM drops out inside the tube (see README "Known results")
        r = _run('traj', 70.0, vo={'tunnel_lighting': 'lit'})
        rows = {c['id']: c for c in r['ref']['checkpoints']}
        self.assertIsNotNone(rows[2]['whole_drone_s'])     # Gate A
        self.assertIsNotNone(rows[3]['whole_drone_s'])     # tunnel south end
        self.assertEqual(r['ref']['touches'], 0)


@unittest.skipUnless(_have_ref('navigation/new-course-layout'), 'traj ref not present')
class LandingTest(unittest.TestCase):
    def test_px4_disarms_within_a_few_seconds_of_touchdown_on_the_pad(self):
        # mc_pos_control's flying-but-ground-contact thrust cut lets the land detector finish within
        # LNDMC_TRIG_TIME of touchdown; then COM_DISARM_LAND (2 s). Without it the vehicle sat on the
        # pad armed for about 21 s while the velocity integrator wound the thrust down.
        r = _run('traj', 100.0, vo={'tunnel_lighting': 'lit'}, options={'land': 'land', 'route': 'window'},
                 track_speed_mps=0.8)
        disarm = next(m[0] for m in r['msgs'] if 'Disarmed by auto disarm on land' in m[2])
        self.assertIsNotNone(r['touchdown'])
        self.assertLess(disarm - r['touchdown'], 4.0)
        self.assertEqual(r['ref']['touches'], 0)


@unittest.skipUnless(_have_ref('localization/return-flight'), 'return_flight ref not present')
class ReturnFlightTest(unittest.TestCase):
    def test_the_optional_return_lands_on_the_launch_mark_and_is_scored(self):
        # rule 7.10: after the pad disarm the pilot re-arms; the drone flies back and disarms
        # within 2 ft of the launch mark inside the 3 minutes; run points x 1.25
        r = _run('return_flight', 175.0, vo={'tunnel_lighting': 'lit'}, options={'return': 'on'})
        self.assertTrue(r['ref']['finished'])
        ret = r['ref']['return']
        self.assertEqual(ret['state'], 'success', ret)
        self.assertLess(ret['distance_from_launch_m'], 0.6096)
        self.assertEqual(ret['run_points_multiplier'], 1.25)


class GuiWorkerTest(unittest.TestCase):
    def test_a_stopped_run_can_still_be_saved(self):
        import multiprocessing as mp
        import tempfile
        from dronesim.gui.worker import worker_main
        ctx = mp.get_context('spawn')
        parent, child = ctx.Pipe()
        proc = ctx.Process(target=worker_main, args=(child, {'version': 'kit_square', 'duration_s': 40.0}, None),
                           daemon=True)
        proc.start()
        try:
            got_tel = False
            while not got_tel and parent.poll(120):
                msg = parent.recv()
                if msg[0] == 'error':
                    self.fail(msg[1])
                got_tel = msg[0] == 'tel' and msg[1]['t'] > 5.0
            parent.send(('stop',))
            with tempfile.TemporaryDirectory() as out:
                parent.send(('save', out))
                saved = None
                while saved is None and parent.poll(60):
                    msg = parent.recv()
                    if msg[0] == 'saved':
                        saved = msg[1]
                self.assertEqual(saved, out)
                self.assertTrue(os.path.isfile(os.path.join(out, 'result.json')))
            parent.send(('quit',))
            proc.join(30)
            self.assertFalse(proc.is_alive())
        finally:
            if proc.is_alive():
                proc.terminate()


if __name__ == '__main__':
    unittest.main()
