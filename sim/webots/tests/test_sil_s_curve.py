import math
import unittest
from pil.course_builder import build_course
from pil.sil_s_curve import s_curve_course,nearest_s_path,analyze_s_curve
from pil.sil_optics import make_sampler
from pil.s3_assumed_line import load_boot5209_line_profile


class SCurveTests(unittest.TestCase):
    def test_back_to_back_ports_nominal_radius_and_white_margins(self):
        course=s_curve_course();built=build_course(course)
        self.assertEqual(len(course.data['tiles']),50)
        self.assertIn('nominal, vendor tile sheet; not ruler-surveyed',course.data['provenance']['source'])
        self.assertEqual(built.port_issues,('t0_1:W unconnected/mismatched port','t8_2:E unconnected/mismatched port'))
        arcs=[(name,g)for name,centre,g in built.tile_geometries if any(':curve:'in p.surface_id for p in g.patches)]
        self.assertEqual(len(arcs),2)

    def test_exact_geometric_path_and_authoritative_samples(self):
        points=[(300.,157.5),(630.,157.5),(787.5,315.),(945.,472.5),(1500.,472.5)]
        for cx,start,step in((630.,-math.pi/2,math.pi/2),(945.,math.pi,-math.pi/2)):
            a=start+step/2;points.append((cx+157.5*math.cos(a),315.+157.5*math.sin(a)))
        for x,y in points:self.assertAlmostEqual(nearest_s_path(x,y)['error_mm'],0)
        scene=make_sampler(build_course(s_curve_course()),'full');profile=load_boot5209_line_profile('GP26')
        for x,y in points[-2:]:
            sample=scene.sample(profile,(x,y,18),sample_us=1)
            self.assertTrue(sample.valid,sample.reason);self.assertGreater(sample.rays,1)

    def test_completion_needs_both_arcs_exit_distance_and_heading(self):
        def row(ms,x,y,yaw):
            r=[0]*36;r[0]=ms;r[15]=yaw
            r[12]=(x-119*math.cos(yaw)+7.25*math.sin(yaw))/1000
            r[13]=(y-119*math.sin(yaw)-7.25*math.cos(yaw))/1000
            return r
        a=-math.pi/4;b=3*math.pi/4
        left=row(1000,630+157.5*math.cos(a),315+157.5*math.sin(a),math.pi/4)
        right=row(2000,945+157.5*math.cos(b),315+157.5*math.sin(b),math.pi/4)
        end=row(8000,1260,472.5,0)
        self.assertTrue(analyze_s_curve([left,right,end])['completes_curve'])
        self.assertFalse(analyze_s_curve([left,end])['completes_curve'])
        self.assertFalse(analyze_s_curve([right,end])['completes_curve'])
        self.assertFalse(analyze_s_curve([left,right,row(8000,1260,472.5,math.pi/2)])['completes_curve'])
