import math
import unittest
from pil.sil_barcode_replay import motion_truth,validate_inputs,attempts,REASONS
from pil.sil_junction_barcode import junction_barcode_course


class ReplayTests(unittest.TestCase):
    def test_f_start_preserves_true_sensor_lead(self):
        c=junction_barcode_course(lead_mm=75,letter='F',offset_mm=5,yaw_deg=-5).data
        self.assertEqual(c['barcodes'][0]['letter'],'F')
        p=c['start'];yaw=math.radians(p['yaw_deg'])
        self.assertAlmostEqual(p['position_mm'][0]+80*math.cos(yaw)-77*math.sin(yaw),970.5-75)
        self.assertEqual(p['position_mm'][1],155.25)
        self.assertEqual(p['yaw_deg'],-5)

    def test_held_motor_stamp_not_current_encoder(self):
        rows=[]
        for ms,counts in ((1,0),(10,10),(11,999)):
            r=[0.]*51;r[0]=ms;r[22]=r[23]=counts;r[36]=178;rows.append(r)
        inputs=[]
        for ms,stamp,counts in ((1,0,0),(10,10000,10),(11,10000,10)):
            i=[0]*18;i[0]=ms;i[3]=stamp;i[4]=i[5]=counts;i[9]=178;inputs.append(i)
        validate_inputs(inputs,rows)
        inputs[-1][4]=999
        with self.assertRaises(ValueError):validate_inputs(inputs,rows)

    def test_zero_counts_are_not_pose_truth(self):
        rows=[]
        for ms,x in ((10,1.),(20,1.001)):
            r=[0.]*51;r[0]=ms;r[12]=x;rows.append(r)
        e=dict(reason=4,prior_motor_us=10000,motor_us=20000,left=0,right=0,
            prior_left=0,prior_right=0,prior_valid=1,motor_faults=0)
        d=motion_truth(e,rows)
        self.assertTrue(d['zero_count_update']);self.assertFalse(d['net_truth_reversal'])
        self.assertAlmostEqual(d['gp28_projected_dx_mm'],1)
        rows[-1][12]=.999
        self.assertTrue(motion_truth(e,rows)['net_truth_reversal'])
        e['prior_valid']=0;self.assertIsNone(motion_truth(e,rows)['gp28_projected_dx_mm'])
        self.assertEqual(REASONS[5],'BW_START');self.assertEqual(REASONS[6],'BW_WIDTH')
