"""Deterministic material checks and explicitly assumed recorded-scan reconstructions."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest
sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM / 'core'), str(SIM / 'viewer')]
from scenario import load, validate, ScenarioError
from protocol import normalise
from render import display_data
LAUNCHER = Path(sys.argv[1]).resolve()


def material(kind='specular', incidence=20, corner=0):
    return {'material': kind, 'max_incidence_deg': incidence, 'corner_echo_max_mm': corner}


def target(distance=200, angle=0, size=None):
    return {'id': 'face', 'type': 'box', 'pose': {'x_mm': 65+distance+0.5,
            'y_mm': 0, 'heading_deg': angle}, 'size_mm': size or [1, 120],
            'contact': 'freeze-both', 'echo': material()}


def probe(obstacle, scan=False):
    _, s, _ = load(SIM / 'scenarios/no_box_straight.json')
    s.update(end_us=3000000 if scan else 100000, events=[], obstacles=[obstacle])
    validate(s)
    result = subprocess.run([str(LAUNCHER.parent / 'sim_ultrasonic_probe')],
        input=normalise(s), capture_output=True, text=True, timeout=10,
        env={**os.environ, 'SIM_PROBE_MODE': 'scan' if scan else 'single'})
    if result.returncode:
        raise AssertionError(result.stderr)
    return [json.loads(line) for line in result.stdout.splitlines()]


def readings(obstacle):
    points = [e for e in probe(obstacle, True) if e['type'] == 'scan_point']
    return ['NR' if (e['word'] >> 9) & 3 else e['word'] & 511 for e in points]


def fit():
    # Width/pose were not measured: this matches the recorded visibility bins,
    # NOT an identification of object type, physical width or face normal.
    face = target(165)
    low, high = 165*math.tan(math.radians(-10.1)), 165*math.tan(math.radians(5.1))
    face['size_mm'] = [1, high-low]
    face['pose']['y_mm'] = (low+high)/2
    slant = target()
    radius = 180+120/math.sqrt(2)
    slant.update(size_mm=[120,120], pose={'x_mm':65+radius*math.cos(math.radians(-25)),
        'y_mm':radius*math.sin(math.radians(-25)), 'heading_deg':20})
    bottle = {'id':'bottle', 'type':'bottle', 'diameter_mm':66, 'echo':material(),
        'contact':'freeze-both', 'pose':{'x_mm':65+219*math.cos(math.radians(7.5)),
        'y_mm':219*math.sin(math.radians(7.5)), 'heading_deg':0}}
    return {'assumed_max_incidence_deg':20, 'assumed_corner_echo_max_mm':0,
        'angles_deg':list(range(-30,31,5)),
        'scan_run_1':{'assumed_obstacle':face,
            'recorded':['NR',187,175,166,166,162,166,167,171,183,196,'NR','NR'],
            'simulated':readings(face)},
        'bypass_run_11':{'assumed_obstacle':slant,
            'recorded_at_minus_20_minus_25_minus_30':['NR','NR','NR'],
            'simulated_at_minus_20_minus_25_minus_30':[readings(slant)[i] for i in [2,1,0]]},
        'bypass_bottle_run_1':{'assumed_obstacle':bottle,
            'recorded':['NR','NR','NR','NR','NR','NR',190,186,186,206,'NR','NR','NR'],
            'simulated':readings(bottle)},
        'limitations':['Object type, width and face normal for scan run 1 were not recorded.',
            'Run 11 pose and face normal were not recorded; servo angle is not incidence.',
            'Nearest-point +/-15 degree sector cannot fit the full rising profile or the narrow bottle visibility.',
            '20 degrees is a conservative assumed cut from the documented 20-25 degree flat-face loss, not a uniquely measured fit.',
            'Corner range 0 is assumed off: no isolated corner-return range was recorded.']}


class Specular(unittest.TestCase):
    def test_square_face_returns(self):
        events = probe(target())
        echo = next(e for e in events if e['type']=='ultrasonic_trigger')
        self.assertTrue(echo['hit'])
        self.assertAlmostEqual(echo['distance_mm'],200)

    def test_slanted_face_no_return_and_saved_marker(self):
        events = probe(target(300,45,[1,1000]))
        echo = next(e for e in events if e['type']=='ultrasonic_trigger')
        self.assertFalse(echo['hit'])
        self.assertEqual(next(e['word'] for e in events if e['type']=='probe_result'),1536)
        missed = next(e for e in events if e['type']=='ultrasonic_no_echo')
        self.assertEqual(missed['reason'],'incidence')
        trace={'header':{'scenario':{},'source_commit':'test','assumed_values':[]},
            'events':events,'complete':True}
        trace['header']['scenario']={'name':'test','end_us':100000}
        self.assertEqual(display_data(trace)['triggers'][0]['no_echo'],missed)

    def test_bottle_returns(self):
        o=target()
        o.update(type='bottle',diameter_mm=66)
        o.pop('size_mm');o['pose']['x_mm']=65+200+33
        echo=next(e for e in probe(o) if e['type']=='ultrasonic_trigger')
        self.assertTrue(echo['hit']);self.assertAlmostEqual(echo['distance_mm'],200)

    def test_corner_range_on_off_and_boundary(self):
        o=target(300-0.5,45,[120,120])
        distance=300-120/math.sqrt(2)
        for limit,hit in [(0,False),(distance-0.001,False),(distance,True),(distance+1,True)]:
            o['echo']=material(corner=limit)
            echo=next(e for e in probe(o) if e['type']=='ultrasonic_trigger')
            self.assertEqual(echo['hit'],hit)
            if hit:self.assertAlmostEqual(echo['distance_mm'],distance,places=6)

    def test_ideal_alias_reproduces_legacy_events(self):
        o=target(300,45,[120,120]);o['echo']='ideal-sector'
        legacy=probe(o)
        o['echo']='ideal';self.assertEqual(probe(o),legacy)
        o['echo']=material('ideal');self.assertEqual(probe(o),legacy)

    def test_schema_ranges(self):
        for incidence,corner in [(-1,0),(91,0),(20,-1)]:
            o=target();o['echo']=material(incidence=incidence,corner=corner)
            with self.assertRaises(ScenarioError):probe(o)


if __name__=='__main__':
    if '--fit' in sys.argv:
        print(json.dumps(fit(),indent=2,sort_keys=True))
    else:
        unittest.main(argv=[sys.argv[0]],verbosity=2)
