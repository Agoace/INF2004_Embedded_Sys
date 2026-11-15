from pathlib import Path
import tempfile
import unittest
from pil.sil_calibration import CalibrationController,CalibrationSequence
from pil.sil_search import build_search_library,SearchController


class CalibrationTests(unittest.TestCase):
    def test_real_config_and_two_outward_goals_then_homing(self):
        with tempfile.TemporaryDirectory()as tmp:
            c=CalibrationController(build_search_library(Path(tmp)))
            self.assertEqual(list(c.spin_config.values()),[131,633,6000,200,900,500,50])
            q=CalibrationSequence(c.spin_config)
            self.assertEqual(q.advance(10,(0,0))['speeds'],(-131,131))
            self.assertTrue(q.advance(400,(-633,633))['stop'])
            self.assertEqual(q.stage,'pause_negative')
            self.assertEqual(q.advance(600,(-640,640))['speeds'],(131,-131))
            self.assertFalse(q.advance(900,(0,0))['stop'])
            self.assertTrue(q.advance(1200,(633,-633))['stop'])
            r=q.advance(1400,(640,-640));self.assertTrue(r['end']);self.assertEqual(r['speeds'],(-131,131))
            self.assertTrue(q.advance(1600,(50,-50),centre=600,peak=650)['stop'])
            self.assertTrue(q.done)
            with self.assertRaises(ValueError):CalibrationSequence(c.spin_config).advance(6000,(0,0))
            with self.assertRaises(ValueError):CalibrationSequence(c.spin_config).advance(10,(0,0),faults=1)

    def test_sampled_endpoints_replay_without_synthetic_extrema(self):
        with tempfile.TemporaryDirectory()as tmp:
            library=build_search_library(Path(tmp));c=CalibrationController(library)
            for ms,adc in enumerate([(172,158)]*16+[(2487,2611)]*4,1):c.sample(adc,ms,True)
            actual=c.finish();self.assertEqual(actual['endpoints'],[172,2487,158,2611])
            d=SearchController(library);self.assertEqual(d.calibrate(c.rows),actual)
