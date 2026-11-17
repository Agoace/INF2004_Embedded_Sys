import csv
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
from pil.sil_line_base_timing import position_shares,analyze_window
from pil.sil_line_candidates import build_candidate,BASE_REVISION,ROOT
from pil.sil_search import SearchController


class BaseTimingTests(unittest.TestCase):
    def test_position_boundaries_and_empty_far(self):
        rows=[]
        for p in (-53,-52,-45,-44,0,44,45,52,53):
            r=[0.]*36;r[3]=p;r[24]=180;r[25]=110;rows.append(r)
        m=position_shares(rows)
        self.assertEqual(m['counts_lt45_45to52_gt52'],[3,4,2]);self.assertEqual(m['far_mean_abs_asked_difference_mm_s'],70)
        self.assertIsNone(position_shares([])['percentages_lt45_45to52_gt52'])
        self.assertIsNone(position_shares(rows[1:-1])['far_mean_abs_asked_difference_mm_s'])

    def test_mixed_window_no_LINE_period_claim_and_explicit_denominators(self):
        rows=[]
        for ms in range(1,3001):
            r=[0.]*36;r[0]=ms;r[15]=.04*math.sin(2*math.pi*ms/300);r[27]=1 if ms>100 else 2;r[28]=1;rows.append(r)
        m=analyze_window(rows,0,3000)
        self.assertEqual(m['non_LINE_rows'],100);self.assertIsNone(m['LINE_cycle_period_ms'])
        self.assertEqual(m['position_shares']['all_20ms_rows']['rows'],150)
        self.assertEqual(m['position_shares']['LINE_20ms_rows']['rows'],145)
        self.assertEqual(m['position_shares']['all_1ms_rows']['rows'],3000)

    def test_sw9_original_base_tests_and_tick_order(self):
        try:subprocess.run(['git','cat-file','-e',BASE_REVISION],cwd=ROOT,check=True,capture_output=True)
        except subprocess.CalledProcessError:self.skipTest('exact SW9 LINE_BASE source unavailable')
        source=subprocess.check_output(['git','show',f'{BASE_REVISION}:tests/test_line_base.c'],cwd=ROOT)
        for base,same in ((146,False),(120,False),(100,False),(146,True)):
            with tempfile.TemporaryDirectory()as tmp:
                directory=Path(tmp);record=build_candidate(directory,85,base_mm_s=base,same_tick=same,revision=BASE_REVISION)
                self.assertEqual(record['configuration']['steer'],[base,85,85,62,378,50,340])
                tree=directory/'sw9-source';(tree/'tests').mkdir();test=tree/'tests/test_line_base.c';test.write_bytes(source)
                executable=directory/'test';subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',f'-DLINE_BASE={base}',
                    str(test),str(tree/'app_program/subsystem3/line_steer.c'),'-o',str(executable)],check=True,capture_output=True)
                subprocess.run([str(executable)],check=True,capture_output=True)
                c=SearchController(directory/'libsil_line.so');c.calibrate([[200,200,0]]*16+[[3700,3700,0]]*4)
                for ms in range(1,101):
                    c.update(1700,1700,ms,0,0)
                    if ms%10==0:
                        tick,stamp,_,_,mode,has_command,age=c.timing()
                        self.assertEqual((tick,mode),(ms,int(same)))
                        if ms>10:
                            self.assertTrue(has_command);self.assertEqual(age,0 if same else 10)
                            self.assertEqual(stamp,ms if same else ms-10)

    def test_default_overlay_replays_existing_P85_without_output_changes(self):
        paths=(ROOT/'build/webots/taskA-excited-20261116T150638Z/offset10-yaw-10-delay20',
            ROOT/'build/webots/taskA-candidates-20261116T151447Z/P85-delay30')
        if not all((p/'sil-result/samples.csv').exists()for p in paths):self.skipTest('retained ignored recordings not present')
        with tempfile.TemporaryDirectory()as tmp:
            directory=Path(tmp);build_candidate(directory,85,revision=BASE_REVISION)
            for p in paths:
                c=SearchController(directory/'libsil_line.so');config=json.loads((p/'sil-manifest.json').read_text());c.calibrate(config['calibration_rows'])
                with(p/'sil-result/samples.csv').open()as f:
                    reader=csv.reader(f);next(reader)
                    for packed in reader:
                        r=list(map(float,packed));actual=c.update(int(r[1]),int(r[2]),int(r[0]),int(r[22]),int(r[23]))
                        self.assertEqual(list(actual.values()),r[3:12]+r[24:])
