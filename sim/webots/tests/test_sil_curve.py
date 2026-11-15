import math
import unittest
from unittest.mock import patch
from pil.sil_curve import curve_course,nearest_path,analyze_curve,RADIUS_MM
from pil.course_builder import build_course
from pil.sil_optics import make_sampler
from pil.s3_assumed_line import load_boot5209_line_profile


class CurveTests(unittest.TestCase):
    def test_tile_ports_radius_and_truth_error(self):
        built=build_course(curve_course())
        self.assertEqual(RADIUS_MM,157.5)
        self.assertEqual(built.port_issues,('t0_1:W unconnected/mismatched port','t4_4:N unconnected/mismatched port'))
        for point in((300,157.5),(630,157.5),(787.5,315),(787.5,700)):
            self.assertAlmostEqual(nearest_path(*point)['error_mm'],0)
        a=-math.pi/4
        self.assertAlmostEqual(abs(nearest_path(630+160.5*math.cos(a),315+160.5*math.sin(a))['error_mm']),3)
        self.assertEqual(nearest_path(787.5,700)['segment'],'exit')

    def test_authoritative_full_scene_no_fast_fallback(self):
        scene=make_sampler(build_course(curve_course()),'full')
        with patch('pil.sil_optics.FastScene.from_course',side_effect=AssertionError('fallback')):
            p=load_boot5209_line_profile('GP26')
            a=-math.pi/4;sample=scene.sample(p,(630+RADIUS_MM*math.cos(a),315+RADIUS_MM*math.sin(a),18),sample_us=1)
            self.assertTrue(sample.valid,sample.reason);self.assertGreater(sample.rays,1)
        with self.assertRaises(ValueError):make_sampler(None,'unknown')

    def test_completion_requires_axle_and_exit_heading(self):
        row=[0]*36;row[0]=5000;row[12]=.79475;row[13]=.6;row[15]=math.pi/2
        self.assertTrue(analyze_curve([row])['completes_curve'])
        row[15]=0
        self.assertFalse(analyze_curve([row])['completes_curve'])
