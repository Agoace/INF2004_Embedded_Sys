import tempfile
import unittest
from pathlib import Path
from pil.sil_line import build_library,LineController,forward_velocity,DM1_FORWARD

ROOT=Path(__file__).resolve().parents[3]

class SILTests(unittest.TestCase):
    def test_all_measured_forward_rows_and_rejections(self):
        for duty,left,right in DM1_FORWARD:
            speeds=forward_velocity(duty,duty,32.5)
            self.assertAlmostEqual(speeds[0],left/32.5);self.assertAlmostEqual(speeds[1],right/32.5)
        self.assertEqual(forward_velocity(0,0,32.5),(0.,0.))
        self.assertAlmostEqual(forward_velocity(22.5,22.5,32.5)[0],148.5/32.5)
        for bad in(-1,1,4.99,100.01,float('nan')):
            with self.assertRaises(ValueError):forward_velocity(bad,20,32.5)

    def test_real_c_calibration_pair_order_sign_and_replay(self):
        with tempfile.TemporaryDirectory()as tmp:
            library=build_library(Path(tmp))
            controller=LineController(library)
            calibration=[[200,200,0]]*16+[[3700,3700,0]]*4
            info=controller.calibrate(calibration)
            self.assertEqual(info['result'],0);self.assertEqual(info['bad_mask'],0)
            self.assertTrue(info['line_high']);self.assertEqual(info['endpoints'],[200,3700,200,3700])
            centered=controller.update(1900,1900,0)
            self.assertEqual(centered['position'],0)
            right=controller.update(200,3000,10)
            self.assertGreater(right['position'],0);self.assertGreater(right['left_pct'],right['right_pct'])
            fresh=LineController(library);fresh.calibrate(calibration)
            self.assertEqual(centered,fresh.update(1900,1900,0))
            self.assertEqual(right,fresh.update(200,3000,10))
            lost=None
            for ms in range(20,500):lost=fresh.update(200,200,ms)
            self.assertFalse(lost['line_seen']);self.assertEqual((lost['left_pct'],lost['right_pct']),(0,0))
            with self.assertRaises(ValueError):fresh.update(200,200,10)

    def test_generated_project_hashes_and_initial_sensor_pair(self):
        import importlib.util,json,hashlib
        from pil.s3_fast import FastScene
        from pil.s3_optics import SensorProfile
        from pil.course_builder import Course,build_course
        path=ROOT/'sim/webots/tools/sil_line_smoke.py'
        spec=importlib.util.spec_from_file_location('sil_tool',path);tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
        with tempfile.TemporaryDirectory(dir=ROOT/'build')as tmp:
            project,world=tool.prepare_project(Path(tmp)/'project')
            self.assertIn('controller "sil_line"',world.read_text())
            config=json.loads((project/'sil-manifest.json').read_text())
            scene=FastScene.from_course(build_course(Course.load(project/'course.json')))
            for name in('GP26','GP27'):
                mount=config['mounts'][name];sample=scene.sample(SensorProfile(**config['profiles'][name]),
                    (196+mount['forward_mm'],150.25+mount['left_mm'],19))
                self.assertTrue(sample.valid,sample.reason)
                self.assertEqual(sample.adc,1565)
            controller_path=ROOT/'sim/webots/controllers/sil_line/sil_line.py'
            spec=importlib.util.spec_from_file_location('sil_controller_test',controller_path)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            native=LineController(project/'libraries/libsil_line.so');native.calibrate(config['calibration_rows'])
            actual=native.update(1565,1565,1)
            targets=forward_velocity(actual['left_pct'],actual['right_pct'],32.5)
            row=[1,1565,1565,*actual.values(),.196,.15025,.0425,0.,0.,0.,*targets,0.,1.]
            self.assertTrue(module.replay(project,[row]))
            row[7]+=1
            with self.assertRaises(ValueError):module.replay(project,[row])
            manifest=json.loads((project/'manifest.json').read_text())
            for name,digest in manifest['artifacts_sha256'].items():
                self.assertEqual(hashlib.sha256((project/name).read_bytes()).hexdigest(),digest,name)

    def test_public_calibration_rejects_no_contrast(self):
        with tempfile.TemporaryDirectory()as tmp:
            controller=LineController(build_library(Path(tmp)))
            with self.assertRaises(ValueError):controller.calibrate([[200,200,0]]*20)
