"""Host-only: the course8 controller with a kinematic fake Supervisor."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

from pil.sil_line import ROOT

SPIN = Path.home() / ('Projects/core-review-sw5/build/webots/'
                      'sil-curve-full-spin-20261115T132255Z/spin-check.json')


class Device:
    def __init__(self):
        self.velocity = 0.0
        self.value = 0.0

    def setPosition(self, _):
        pass

    def setAvailableTorque(self, _):
        pass

    def setVelocity(self, v):
        self.velocity = v

    def enable(self, _):
        pass

    def getValue(self):
        return self.value


class FakeRobot:
    """Unicycle at 1 ms; optional roll window to drive the tilted path."""

    def __init__(self, start, yaw_deg, roll=None):
        self.ms = 0
        self.x, self.y = start[0] / 1000, start[1] / 1000
        self.yaw = math.radians(yaw_deg)
        self.devices = {n: Device() for n in ('left_motor', 'right_motor', 'left_encoder',
                                              'right_encoder', 'sonar_servo')}
        self.roll = roll or {}
        self.quit = None

    def getBasicTimeStep(self):
        return 1

    def getDevice(self, name):
        return self.devices[name]

    def getTime(self):
        return self.ms / 1000

    def getSelf(self):
        return self

    def step(self, _):
        vl = self.devices['left_motor'].velocity
        vr = self.devices['right_motor'].velocity
        self.devices['left_encoder'].value += vl * .001
        self.devices['right_encoder'].value += vr * .001
        v = (vl + vr) / 2 * .0325
        self.x += v * math.cos(self.yaw) * .001
        self.y += v * math.sin(self.yaw) * .001
        self.yaw += (vr - vl) * .0325 / .1115 * .001
        self.ms += 1
        return 0

    def getPosition(self):
        return [self.x, self.y, .0425]

    def getOrientation(self):
        r = math.radians(self.roll.get(self.ms, 0.0))
        cy, sy, cr, sr = math.cos(self.yaw), math.sin(self.yaw), math.cos(r), math.sin(r)
        # Rz(yaw) * Rx(roll), row-major.
        return [cy, -sy * cr, sy * sr, sy, cy * cr, -cy * sr, 0, sr, cr]

    def simulationQuit(self, code):
        self.quit = code


@unittest.skipUnless(SPIN.is_file(), 'pinned spin JSON not on this host')
class Course8ControllerTests(unittest.TestCase):
    def test_short_run_incomplete_rows_replay_and_tilt_path(self):
        sys.path.insert(0, str(ROOT / 'sim/webots/tools'))
        import sil_course8_run as runner
        (ROOT / 'build/webots').mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(dir=ROOT / 'build/webots', prefix='c8-host-test-'))
        try:
            project, _, course = runner.prepare(tmp / 'p', 'default', SPIN)
            config = json.loads((project / 'sil-manifest.json').read_text())
            config['duration_ms'] = 3000
            (project / 'sil-manifest.json').write_text(json.dumps(config))
            spec = importlib.util.spec_from_file_location(
                'c8ctl', project / 'controllers/sil_course8/sil_course8.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            roll = {ms: 4.0 for ms in range(1500, 1600)}
            robot = FakeRobot(course.data['start']['position_mm'],
                              course.data['start']['yaw_deg'], roll)
            module.run(robot, project)
            summary = json.loads((project / 'sil-result/summary.json').read_text())
            # 3 s cannot finish 2 laps: INCOMPLETE, never PASS.
            self.assertEqual(summary['status'], 'SIL_COURSE8_INCOMPLETE')
            self.assertIsNone(summary['error'])
            self.assertEqual(robot.quit, 1)
            _, names, rows = runner.load_rows(project)
            self.assertEqual(len(rows), 3000)
            i = {n: k for k, n in enumerate(names)}
            self.assertEqual(sum(r[i['tilted']] for r in rows), 100)
            self.assertTrue(all(r[i['module']] == 1 for r in rows))
            self.assertGreater(rows[-1][i['progress_mm']], 300)
            self.assertLess(max(abs(r[i['error_mm']]) for r in rows), 20)
            self.assertTrue(module.replay(project, rows))
            bad = list(rows[100])
            bad[i['left_pct']] += 1
            with self.assertRaises(ValueError):
                module.replay(project, rows[:100] + [tuple(bad)])
            with (project / 'sil-result/samples.csv').open() as stream:
                self.assertEqual(next(csv.reader(stream)), list(names))
        finally:
            shutil.rmtree(tmp)

    def test_tilt_limit_stops_incomplete(self):
        sys.path.insert(0, str(ROOT / 'sim/webots/tools'))
        import sil_course8_run as runner
        (ROOT / 'build/webots').mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(dir=ROOT / 'build/webots', prefix='c8-host-test-'))
        try:
            project, _, course = runner.prepare(tmp / 'p', 'default', SPIN)
            config = json.loads((project / 'sil-manifest.json').read_text())
            config['duration_ms'] = 600
            (project / 'sil-manifest.json').write_text(json.dumps(config))
            spec = importlib.util.spec_from_file_location(
                'c8ctl2', project / 'controllers/sil_course8/sil_course8.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            robot = FakeRobot(course.data['start']['position_mm'],
                              course.data['start']['yaw_deg'], {500: 9.0})
            module.run(robot, project)
            summary = json.loads((project / 'sil-result/summary.json').read_text())
            self.assertEqual(summary['status'], 'SIL_COURSE8_INCOMPLETE')
            self.assertIn('outside tilt domain', summary['error'])
            self.assertEqual(summary['samples'], 499)
        finally:
            shutil.rmtree(tmp)


if __name__ == '__main__':
    unittest.main()
