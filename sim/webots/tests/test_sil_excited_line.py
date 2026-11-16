import copy
import math
import unittest
from pathlib import Path
from pil.sil_excited_line import DelayPair,apply_geometry,window_metrics
from pil.robot_profile import RobotProfile


class ExcitedLineTests(unittest.TestCase):
    def test_explicit_delay_exact_and_bounded(self):
        d=DelayPair(20)
        for ms in range(1,101):self.assertEqual(d.update(ms,(ms,-ms)),(max(1,ms-20),-max(1,ms-20)))
        self.assertLessEqual(len(d.queue),21)
        with self.assertRaises(ValueError):d.update(102,(1,2))
        with self.assertRaises(ValueError):DelayPair(40)
        self.assertEqual(DelayPair(0).update(1,(4,5)),(4,5))

    def test_fixed_height_scale_and_geometry_not_refit(self):
        p=RobotProfile.load(Path(__file__).resolve().parents[1]/'calibration/robot_assumed.json')
        config=dict(profiles={n:dict(reference_height_mm=8,sigma_x_mm=2.8,sigma_y_mm=2.8,assumptions=[])for n in ('GP27','GP26')},mounts={n:{}for n in ('GP27','GP26')})
        spin=dict(calibration_rows=[[1,2,0]],calibration=dict(endpoints=[1,2,3,4]))
        c=apply_geometry(config,p,spin)
        self.assertEqual(c['mounts']['GP27']['forward_mm'],122)
        self.assertEqual(c['mounts']['GP27']['left_mm'],14.7)
        self.assertEqual(c['mounts']['GP26']['left_mm'],0)
        self.assertEqual(c['profiles']['GP27']['reference_height_mm'],9)
        self.assertAlmostEqual(c['profiles']['GP27']['sigma_x_mm'],2.8*9/8)
        self.assertEqual(c['calibration_rows'],spin['calibration_rows'])
        self.assertIn('NOT a9mm',c['calibration_provenance'])

    def test_truth_yaw_cycle_and_quiet_not_encoder_quantization(self):
        rows=[]
        for ms in range(2000,4001):
            phase=2*math.pi*ms/300;r=[0.]*36;r[0]=ms;r[1]=2000-1000*math.sin(phase);r[2]=2000+1000*math.sin(phase)
            r[15]=.04*math.sin(phase);r[27]=1;r[28]=1;rows.append(r)
        m=window_metrics(rows,2000,4000);self.assertEqual(m['period_ms'],300)
        self.assertGreater(m['yaw_rate_amplitude_deg_s'],45)
        self.assertEqual(m['non_LINE_rows'],0)
        for r in rows:r[15]=0
        self.assertIsNone(window_metrics(rows,2000,4000)['period_ms'])
