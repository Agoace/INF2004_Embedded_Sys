import subprocess
import tempfile
from pathlib import Path
import unittest
from pil.sil_line_candidates import build_candidate,REVISION,ROOT
from pil.sil_excited_line import DelayPair


class CandidateTests(unittest.TestCase):
    def test_30ms_transport_is_exact_not40ms_motor_delay(self):
        d=DelayPair(30)
        for ms in range(1,100):self.assertEqual(d.update(ms,(ms,ms)),(max(1,ms-30),)*2)
        self.assertLessEqual(len(d.queue),31)

    def test_real_sw9_adapter_native_config_and_original_tests(self):
        try:subprocess.run(['git','cat-file','-e',REVISION],cwd=ROOT,check=True,capture_output=True)
        except subprocess.CalledProcessError:self.skipTest('SW9 exact source not present; no substitute')
        test=subprocess.check_output(['git','show',f'{REVISION}:tests/test_line_p_near.c'],cwd=ROOT)
        for gain in (85,60,40):
            with tempfile.TemporaryDirectory()as tmp:
                directory=Path(tmp);record=build_candidate(directory,gain)
                self.assertEqual(record['configuration']['steer'],[146,gain,85,62,378,50,340])
                self.assertEqual(record['source_commit'],REVISION)
                self.assertIsNone(record['adapter'])if gain==85 else self.assertIsNotNone(record['adapter'])
                tree=directory/'sw9-source';(tree/'tests').mkdir();source=tree/'tests/test_line_p_near.c';source.write_bytes(test)
                executable=directory/'test';subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',f'-DLINE_P_NEAR={gain}',
                    str(source),str(tree/'app_program/subsystem3/line_steer.c'),'-o',str(executable)],check=True,capture_output=True)
                subprocess.run([str(executable)],check=True,capture_output=True)
