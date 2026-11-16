from dataclasses import replace
import math
import unittest
from pil.scene import SceneSnapshot
from pil.s3_geometry import PolygonPatch
from pil.s3_sil_barcode import load_sil_barcode_profile
from pil.s3_gp28_nonlinear import board_axis_u
from pil.s3_referee import fine_sample
from pil.sil_uniform_mat import UniformMatCertificate
from pil.sil_junction_barcode import junction_barcode_course
from pil.course_builder import build_course


def patch(name,material,x0,y0,x1,y1,z=10):
    return PolygonPatch(name,material,((x0,y0,z),(x1,y0,z),(x1,y1,z),(x0,y1,z)),0)


class UniformTests(unittest.TestCase):
    def test_rotated_box_not_unsafe_ellipse(self):
        s=SceneSnapshot.from_patches((patch('small','MAT_WHITE',-26,-26,26,26),))
        p=load_sil_barcode_profile(sigma_along_mm=8)
        self.assertIsNone(UniformMatCertificate(s).sample(s,p,(0,0,15.5),board_axis_u(math.pi/4)))

    def test_holes_seams_unknown_slivers_and_touch_always_use_full(self):
        p=load_sil_barcode_profile(sigma_along_mm=1)
        for extras in((patch('ink','PAPER_INK',-.01,-.01,.01,.01),),
                       (patch('unknown','UNKNOWN',-.01,-.01,.01,.01),),
                       (patch('touch','MAT_LINE',4*p.sigma_y_mm,-1,20,1),),
                       (patch('raised','MAT_WHITE',-1,-1,1,1,11),)):
            s=SceneSnapshot.from_patches((patch('base','MAT_WHITE',-100,-100,100,100),*extras))
            self.assertIsNone(UniformMatCertificate(s).sample(s,p,(0,0,15.5),board_axis_u(0)))
        for whites in((patch('left','MAT_WHITE',-100,-100,0,100),patch('right','MAT_WHITE',0,-100,100,100)),
                       (patch('left','MAT_WHITE',-100,-100,-1,100),patch('right','MAT_WHITE',1,-100,100,100),
                        patch('top','MAT_WHITE',-1,1,1,100),patch('bottom','MAT_WHITE',-1,-100,1,-1))):
            s=SceneSnapshot.from_patches(whites)
            self.assertIsNone(UniformMatCertificate(s).sample(s,p,(0,0,15.5),board_axis_u(0)))

    def test_scene_epsilon_near_below_and_ambiguous_layer_never_certify(self):
        p=load_sil_barcode_profile(sigma_along_mm=1)
        base=patch('base','MAT_WHITE',-100,-100,100,100)
        dark=PolygonPatch('near-below','MAT_LINE',patch('d','MAT_LINE',-100,-100,100,100,10-5e-8).vertices_mm,30)
        s=SceneSnapshot.from_patches((base,dark))
        self.assertIsNone(UniformMatCertificate(s).sample(s,p,(0,0,15.5),board_axis_u(0)))
        self.assertEqual(fine_sample(s,p,(0,0,15.5),axis_u=board_axis_u(0),tolerance=1e-7).adc,2321)
        shifted=patch('offset-white','MAT_WHITE',-100,-100,100,100,10-5e-8)
        lower=PolygonPatch('offset-dark','MAT_LINE',patch('d2','MAT_LINE',-100,-100,100,100,10-1.1e-7).vertices_mm,30)
        offset=SceneSnapshot.from_patches((shifted,lower))
        self.assertEqual(offset.first_hit((0,0,15.5),(0,0,-1),100).material_id,'MAT_LINE')
        self.assertIsNone(UniformMatCertificate(offset).sample(offset,p,(0,0,15.5),board_axis_u(0)))
        ambiguous=SceneSnapshot.from_patches((base,patch('same-layer','MAT_LINE',-1,-1,1,1)))
        self.assertIsNone(UniformMatCertificate(ambiguous).sample(ambiguous,p,(0,0,15.5),board_axis_u(0)))

    def test_revision_height_tilt_and_profile_values(self):
        s=SceneSnapshot.from_patches((patch('base','MAT_WHITE',-100,-100,100,100),))
        p=load_sil_barcode_profile(sigma_along_mm=1);c=UniformMatCertificate(s)
        self.assertEqual(c.sample(s,p,(0,0,15.5),board_axis_u(0)).adc,178)
        other=SceneSnapshot.from_patches((patch('new','MAT_WHITE',-100,-100,100,100),))
        self.assertIsNone(c.sample(other,p,(0,0,15.5),board_axis_u(0)))
        self.assertIsNone(c.sample(s,p,(0,0,16.5),board_axis_u(0)))
        self.assertIsNone(c.sample(s,p,(0,0,15.5),board_axis_u(0),direction=(.1,0,-1)))
        q=replace(p,levels_adc={**p.levels_adc,'MAT_WHITE':200})
        self.assertEqual(c.sample(s,q,(0,0,15.5),board_axis_u(0)).adc,200)

    def test_actual_lead_poses_equal_authoritative_reference(self):
        scene=build_course(junction_barcode_course()).scene;c=UniformMatCertificate(scene)
        for n in(1,4,8):
            p=load_sil_barcode_profile(sigma_along_mm=n)
            for x in(670.5,750.,850.):
                origin=(x,227.25,15.5);axis=board_axis_u(.03)
                sample=c.sample(scene,p,origin,axis)
                self.assertIsNotNone(sample)
                reference=fine_sample(scene,p,origin,axis_u=axis,tolerance=1e-7)
                self.assertTrue(reference.valid);self.assertAlmostEqual(sample.mean_adc,reference.mean_adc,delta=1e-8)
