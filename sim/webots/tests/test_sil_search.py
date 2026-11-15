import math
from pathlib import Path
import tempfile
import unittest
from pil.sil_motor import signed_velocity,DM1_REVERSE
from pil.sil_search import build_search_library,SearchController


class SearchTests(unittest.TestCase):
    def test_reverse_measured_rows_interpolation_and_rejection(self):
        self.assertEqual(signed_velocity(-20,-20,32.5),(-131/32.5,-134/32.5))
        for pct,left,right in DM1_REVERSE:
            self.assertEqual(signed_velocity(-pct,-pct,32.5),(left/32.5,right/32.5))
        self.assertEqual(signed_velocity(-22.5,22.5,1),(-151,150))
        for invalid in(-101,101,float('nan'),float('inf')):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):signed_velocity(invalid,0,32.5)
        self.assertEqual(signed_velocity(0,0,32.5),(0,0))
        for duty in(0,4.9,5,6):
            self.assertEqual(signed_velocity(duty,-duty,32.5),(0,0))
        self.assertEqual(signed_velocity(7,7,32.5),(0,0))
        self.assertEqual(signed_velocity(-7,-7,32.5),(-4/32.5,-1/32.5))

    def test_real_shell_modules_and_search_handoffs(self):
        with tempfile.TemporaryDirectory()as tmp:
            c=SearchController(build_search_library(Path(tmp)))
            c.calibrate([[200,200,0]]*16+[[3700,3700,0]]*4)
            # Force outcomes through the real module API, not lp private fields.
            r=c.module_step(10,event=5|(1<<5))
            self.assertEqual(r['module'],2);self.assertEqual(r['searches'],1)
            self.assertLess(r['desired_left_mm_s']*r['desired_right_mm_s'],0)
            r=c.module_step(20,event=6)
            self.assertEqual(r['module'],1);self.assertEqual(r['found_by_search'],1)
            self.assertEqual(r['shell_fault'],0)

    def test_wide_world_is_explicit_geometry_not_sampler_override(self):
        from pil.course_builder import Course,build_course
        from pil.s3_fast import FastScene
        from pil.s3_assumed_line import load_assumed_line_profile
        from pil.sil_line import ROOT
        course=Course.load(ROOT/'sim/webots/courses/sil/straight_search_assumed.json')
        data=course.data
        self.assertEqual(len(data['tiles']),30)
        self.assertEqual(sum(t['kind']=='BLANK'for t in data['tiles']),20)
        self.assertIn('Tile seams NOT modelled',data['provenance']['source'])
        scene=FastScene.from_course(build_course(course))
        profile=load_assumed_line_profile('GP26')
        for point in((196,350,18),(196,-100,18),(-100,157.5,18)):
            self.assertTrue(scene.sample(profile,point,sample_us=1).valid)

    def test_search_exhaustion_and_motor_safety(self):
        with tempfile.TemporaryDirectory()as tmp:
            c=SearchController(build_search_library(Path(tmp)))
            r=c.module_step(10,event=5)
            r=c.module_step(20,counts=(-400,400))
            r=c.module_step(30,counts=(400,-400))
            self.assertEqual(r['shell_fault'],2)
            self.assertEqual(r['action'],1)
            self.assertEqual(r['desired_left_mm_s'],0)
            c.lib.sil_reset()
            r=c.module_step(10,motor_fault=1)
            self.assertEqual(r['shell_fault'],1);self.assertEqual(r['action'],1)
