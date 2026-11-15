import tempfile
from pathlib import Path
import unittest
from pil.sil_motor import build_motor_library,MotorLineController,step_response,LagPlant


class MotorSILTests(unittest.TestCase):
    def test_complete_native_motor_plus_plant_matches_ede2(self):
        with tempfile.TemporaryDirectory()as tmp:
            library=build_motor_library(Path(tmp))
            result=step_response(library)
            self.assertTrue(all(88<=t<=132 for t in result['raw_t90_ms']),result['raw_t90_ms'])
            self.assertEqual(result['motor_config']['accel_cpt_q8'],2048)
            self.assertEqual(result['motor_config']['max_speed_mm_s'],460)

    def test_robot_p7_config_and_public_calibration(self):
        with tempfile.TemporaryDirectory()as tmp:
            c=MotorLineController(build_motor_library(Path(tmp)))
            self.assertEqual(c.metadata()['steer'],[146,85,85,62,378,50,340])
            info=c.calibrate([[200,200,0]]*16+[[3700,3700,0]]*4)
            self.assertEqual(info['endpoints'],[200,3700,200,3700])
            for ms in range(1,30):r=c.update(1565,1565,ms,0,0)
            self.assertEqual(r['desired_left_mm_s'],146);self.assertEqual(r['desired_right_mm_s'],146)
            self.assertGreaterEqual(r['left_pct'],12)

    def test_lag_no_extrapolation_and_nonfinite(self):
        p=LagPlant()
        self.assertEqual(p.advance(0,0),(0.,0.))
        a=p.advance(20,20);b=p.advance(20,20)
        self.assertTrue(0<a[0]<b[0]<129/32.5)
        with self.assertRaises(ValueError):p.advance(-1,20)
