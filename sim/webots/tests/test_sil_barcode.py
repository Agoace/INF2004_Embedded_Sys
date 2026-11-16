import ctypes
import importlib.util
from pathlib import Path
import tempfile
import unittest
from pil.sil_junction_barcode import BarcodeController,build_barcode_library,junction_barcode_course
from pil.course_builder import build_course
from pil.s3_sil_barcode import barcode_sheet


class BarcodeSILTests(unittest.TestCase):
    def test_exact_a4_geometry_and_gap(self):
        course=junction_barcode_course();built=build_course(course)
        self.assertEqual(len(course.data['barcodes']),1)
        sheet=barcode_sheet(sheet_start_mm=970.5,line_centre_y_mm=157.5,surface_z_mm=10)
        self.assertEqual(sheet.junction_centre_x_mm-sheet.sheet_bounds_mm[2],150)
        self.assertEqual(sheet.sheet_bounds_mm,(970.5,182.5,1267.5,392.5))
        self.assertEqual(course.data['start']['position_mm'],[590.5,150.25])
        expected={(p.material_id,p.vertices_mm)for p in sheet.geometry.patches}
        actual={(p.material_id,p.vertices_mm)for p in built.patches if p.surface_id.startswith('sheet_a:')}
        self.assertEqual(expected,actual)

    def test_predeclared_native_stop_not_retroactive_tail_truncation(self):
        path=Path(__file__).resolve().parents[1]/'controllers/sil_line/sil_line.py'
        spec=importlib.util.spec_from_file_location('barcode_duration',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        config=dict(barcode_trial=True,stop_after_junction=True,duration_ms=5300)
        rows=[[0]*51 for _ in range(12)]
        self.assertFalse(module.completed_segment(config,rows))
        rows[-1][47]=1;self.assertTrue(module.completed_segment(config,rows))
        self.assertFalse(module.completed_segment({**config,'stop_after_junction':False},rows))
        rows[-2][47]=1;self.assertFalse(module.completed_segment(config,rows))
        self.assertEqual(junction_barcode_course(lead_mm=100).data['start']['position_mm'],[790.5,150.25])
        self.assertEqual(junction_barcode_course(lead_mm=75).data['start']['position_mm'],[815.5,150.25])
        with self.assertRaises(ValueError):junction_barcode_course(lead_mm=99)

    def test_native_reader_decodes_but_never_admits_or_forces_action(self):
        with tempfile.TemporaryDirectory()as tmp:
            c=BarcodeController(build_barcode_library(Path(tmp)))
            c.calibrate([[172,158,0]]*16+[[2487,2611,0]]*4)
            rows=[[i*1000,*([178 if i<100 else 2321]*3)]for i in range(1,151)]
            c.barcode_calibrate(rows,1)
            c.lib.barcode_pattern.argtypes=[ctypes.c_char];c.lib.barcode_pattern.restype=ctypes.c_uint16
            runs=[]
            for index,ch in enumerate(b'*A*'):
                if index:runs.append(5.8)
                mask=c.lib.barcode_pattern(bytes([ch]))
                runs.extend(17.5 if mask&(0x100>>n)else 5.8 for n in range(9))
            for ms in range(1,4001):
                x=ms*.146-80;raw=178
                for n,width in enumerate(runs):
                    if x<0:break
                    if x<width:raw=1000 if n%2==0 else 178;break
                    x-=width
                counts=round(ms*.146*12.414)
                line=(2487,2611)if 3200<=ms<=3300 else(1075,1115)
                r=c.update(*line,ms,counts,counts,raw)
            self.assertEqual(r['bc_letter'],ord('A'))
            self.assertEqual(r['bc_valid'],0)
            self.assertEqual(r['bc_flags']&0x40,0)
            self.assertEqual(r['bc_flags']&0x80,0x80)
            self.assertEqual(r['action_command'],0)
            self.assertGreaterEqual(r['junction_confirmations'],1)
            self.assertEqual(r['shell_fault'],0)
            self.assertTrue(c.events())
            self.assertTrue(all(e['reason']!=0 for e in c.events()))
            self.assertEqual(r['bc_admission_reason'],1)
            self.assertTrue(r['bc_quiet_pass']);self.assertTrue(r['bc_start_pass'])
            stream=c.inputs();details=c.details()
            self.assertEqual(len(stream),4000)
            self.assertEqual(c.component_replay(stream),details)
            with self.assertRaises(ValueError):c.component_replay([[0]*17])
            # Overflow is a failed replay, never silently truncated evidence.
            with self.assertRaises(ValueError):c.component_replay([[0]*18]*10000)
