import math
from pathlib import Path
import tempfile
import unittest
from pil.sil_line_oscillation import (decode,parse_log,segments,cycle_metrics,lag,Native)


def row(ms,**kw):
    return dict(dict(ms=ms,state=1,module=1,raw_l=1000,raw_c=1000,position=0,
        ask_l=146,ask_r=146,v_l=146,v_r=146,events=0,flags=0),**kw)


class LineOscillationTests(unittest.TestCase):
    def test_signed_layout_time_not_index(self):
        w=7 | (1<<16) | (5<<18) | (4095<<21) | (1234<<33)
        w|=(((-99)&255)<<45) | (((-41)&1023)<<53) | (378<<63)
        w|=(((-80)&1023)<<73) | (511<<83) | (251<<116) | (1<<124)
        r=decode(w.to_bytes(16,'little').hex())
        self.assertEqual([r[k]for k in ('ms','module','raw_l','raw_c','position','ask_l','ask_r','v_l','v_r','events','flags')],
            [140,5,4095,1234,-99,-41,378,-80,511,251,1])
        with self.assertRaises(ValueError):decode('00')

    def test_final_connect_and_dump_endpoint_label(self):
        with tempfile.TemporaryDirectory()as tmp:
            p=Path(tmp)/'a.log';payload=(1<<16|1<<18).to_bytes(16,'little').hex()
            p.write_text('=== 10:11:12 connect\nKR 0 '+payload+'\n=== 20:51:03 connect\n'
                '[robot] KR1 reset power/RUN; stop\n[robot] line L floor/line 200/3200 C 100/3300\n'
                'KR 0 '+payload+'\nKR 2 '+payload+'\n')
            b=parse_log(p,last_connect=True)
            self.assertEqual(len(b),1);self.assertEqual(b[0]['dump_endpoints'],[200,3200,100,3300])
            self.assertNotIn('calibration',b[0]);self.assertEqual(b[0]['packed'][1][0],2)

    def test_reject_unsupported_record(self):
        with tempfile.TemporaryDirectory()as tmp:
            p=Path(tmp)/'bad.log';p.write_text('KR 0 0000\n')
            with self.assertRaisesRegex(ValueError,'unsupported KR size'):parse_log(p)

    def test_zero_wheel_p7_correction_is_not_removed(self):
        r=[row(t,ask_l=0 if t==1000 else 146)for t in range(0,3001,20)]
        s,e=segments(r,None);self.assertEqual(len(s),1);self.assertFalse(e)
        r[75]['ask_l']=-1;s,e=segments(r,None)
        self.assertEqual(len(s),2);self.assertEqual(e['stop/reversing pivot'],1)

    def test_excludes_search_scan_detour_and_gap(self):
        r=[row(t)for t in range(0,5001,20)]
        for i,m in zip((55,100,150,200),(2,3,4,5)):r[i]['module']=m
        s,e=segments(r,None);self.assertEqual(e['not running LINE'],4)
        for part in s:self.assertTrue(all(a['module']==1 for a in part))
        s,_=segments([row(t)for t in list(range(0,1501,20))+list(range(2000,4001,20))],None)
        self.assertEqual(len(s),2)

    def test_dump_high_both_mask_is_optional_not_ground_truth(self):
        r=[row(t,raw_l=3900,raw_c=3900)for t in range(0,3001,20)]
        self.assertEqual(len(segments(r,[200,3200,200,3200])[0]),1)
        s,e=segments(r,[200,3200,200,3200],mask_high_both=True)
        self.assertFalse(s);self.assertTrue(e)

    def test_period_and_signed_lag(self):
        r=[]
        for t in range(0,2401,20):
            a=math.sin(2*math.pi*t/400);b=math.sin(2*math.pi*(t-60)/400)
            r.append(row(t,position=50*a,raw_l=1800-1200*a,raw_c=1800+1200*a,
                ask_l=146+40*a,ask_r=146-40*a,v_l=146+30*b,v_r=146-30*b))
        m=cycle_metrics(r,None);self.assertEqual(m['period_ms'],400)
        self.assertEqual(m['sensor_command_lag']['ms'],0);self.assertEqual(m['command_wheel_lag']['ms'],60)
        self.assertGreater(m['raw_balance_amplitude_adc'],2200)
        quiet=cycle_metrics([row(t)for t in range(0,2401,20)],None)
        self.assertIsNone(quiet['period_ms'])

    def test_native_previous_state_envelope_and_replay(self):
        with tempfile.TemporaryDirectory()as tmp:
            n=Native(tmp)
            self.assertEqual(n.c.metadata()['steer'],[146,85,85,62,378,50,340])
            # Missing historical refs deliberately disable an image claim;
            # shallow checkouts must still test the current native ABI.
            self.assertTrue(all(isinstance(v,bool)for v in n.provenance['image_matches'].values()))
            r=[row(t,position=50,ask_l=182,ask_r=110)for t in range(0,2001,20)]
            self.assertEqual(n.steering(r)['unknown_previous_position_envelope_exact_fraction'],1)
            r[0]['ask_l']=460
            self.assertLess(n.steering(r)['unknown_previous_position_envelope_exact_fraction'],1)
            replay=n.motor([row(t)for t in range(0,2001,20)])
            self.assertLess(replay['predicted_yaw_amplitude_deg_s'],6)
