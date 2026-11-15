import importlib.util
import json
import math
import struct
from pathlib import Path
import tempfile
import unittest
from pil.passive_replay import PassiveReplay,replay_export
from pil.wire import decode,Type,u16

ROOT=Path(__file__).resolve().parents[1]
VALUES=(0.,0.,0.,0.,0.,9.81,1.,0.,0.,0.,0.,0.0425)

class PassiveReplayTests(unittest.TestCase):
    def test_real_codec_order_sampling_and_deterministic_replay(self):
        p=PassiveReplay(2535)
        for n in range(10):p.sample(1000+n*1000,5000+n*1000,5002+n*1000,VALUES)
        sensor=decode(bytes(p.wire[:66]));output=decode(bytes(p.wire[66:124]))
        self.assertEqual(sensor.type,Type.SENSOR);self.assertEqual(output.type,Type.OUTPUT)
        self.assertEqual(sensor.source_us,0);self.assertEqual(u16(sensor.payload,36),24)
        self.assertNotEqual(output.payload[:4],bytes(4))
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run';summary=p.export(out)
            self.assertTrue(summary['simulation_only']);self.assertFalse(summary['motors_applied'])
            self.assertTrue(replay_export(out)['byte_match'])
            wire=out/'io.bin';data=bytearray(wire.read_bytes());data[70]^=1;wire.write_bytes(data)
            with self.assertRaises(ValueError):replay_export(out)
            data[70]^=1;wire.write_bytes(data);(out/'io.csv').write_text('')
            with self.assertRaises(ValueError):replay_export(out)

    def test_actual_encoder_observations_reach_wire_without_truth_pose(self):
        p=PassiveReplay(2535);p.sample(1000,1,2,VALUES)
        moved=(2*math.pi/2535,0.,*VALUES[2:9],999.,888.,777.)
        p.sample(2000,1001,1002,moved)
        second=decode(bytes(p.wire[124:190]))
        self.assertEqual(struct.unpack_from('<ii',second.payload,8),(1,0))
        self.assertEqual(u16(second.payload,36),24)
        self.assertFalse(u16(second.payload,36)&0x800)  # No physical timing claim.

    def test_diagnostic_excess_never_fatal_and_early_is_separate(self):
        p=PassiveReplay(2535)
        for n in range(10):p.sample(1000+n*1000,5000+n*3000,5001+n*3000,VALUES)
        self.assertGreater(p.diagnostics['phase_over600_count'],0)
        self.assertGreater(p.diagnostics['rolling_quota_excess_count'],0)
        self.assertGreater(p.diagnostics['burst_excess_count'],0)
        self.assertEqual(len(p.samples),10)
        p=PassiveReplay(2535)
        for n in range(3):p.sample(1000+n*1000,5000+n*10,5001+n*10,VALUES)
        self.assertEqual(p.diagnostics['early_count'],2)
        self.assertEqual(p.diagnostics['positive_late_count'],0)

    def test_bounded_and_invalid_samples_not_silently_repaired(self):
        p=PassiveReplay(2535,max_samples=1);p.sample(1000,1,2,VALUES)
        with self.assertRaises(ValueError):p.sample(2000,3,4,VALUES)
        p=PassiveReplay(2535);p.sample(1000,1,2,VALUES)
        with self.assertRaises(ValueError):p.sample(3000,3,4,VALUES)
        with self.assertRaises(ValueError):PassiveReplay(2535).sample(1000,1,2,(float('nan'),)+VALUES[1:])

    def test_project_overlay_is_local_passive_and_preserves_artifact_hashes(self):
        import hashlib
        spec=importlib.util.spec_from_file_location('passive_tool',ROOT/'tools/passive_smoke.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(dir=ROOT.parents[1]/'build')as tmp:
            project,world=module.prepare_project(Path(tmp)/'project')
            manifest=json.loads((project/'manifest.json').read_text())
            self.assertIn('basicTimeStep 1',world.read_text())
            self.assertIn('controller "passive_replay"',world.read_text())
            self.assertEqual(manifest['passive_overlay']['course'],'home_mat_1_assumed')
            for name,digest in manifest['artifacts_sha256'].items():
                self.assertEqual(hashlib.sha256((project/name).read_bytes()).hexdigest(),digest,name)

    def test_passive_controller_ten_seconds_never_applies_scripted_output(self):
        spec=importlib.util.spec_from_file_location('passive_controller',ROOT/'controllers/passive_replay/passive_replay.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        actuations=[]
        class Device:
            def enable(self,n):assert n==1
            def setTorque(self,value):
                assert value==0;actuations.append(('zero_torque',value))
            def setPosition(self,value):
                assert value==0;actuations.append(('initial_servo',value))
            def getValue(self):return 0.
            def getValues(self):return [0.,0.,1.]
        class Robot:
            n=0
            def getBasicTimeStep(self):return 1
            def getDevice(self,name):return Device()
            def step(self,n):self.n+=1;assert n==1;return 0
            def getTime(self):return self.n/1000
            def getSelf(self):return self
            def getPosition(self):return [0.,0.,0.0425]
            def simulationQuit(self,status):self.quit=status
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);(path/'manifest.json').write_text(json.dumps({'robot_parameters':{'counts_per_revolution':{'value':2535}}}))
            robot=Robot();ticks=iter(range(50000))
            module.run(robot,path,raw_clock=lambda:next(ticks))
            result=json.loads((path/'passive-result/summary.json').read_text())
            self.assertEqual(robot.n,10000);self.assertEqual(robot.quit,0)
            self.assertEqual(actuations,[('zero_torque',0),('zero_torque',0),('initial_servo',0)])
            self.assertEqual(result['samples'],10000)
            self.assertEqual(result['simulation_end_us'],10000000)
            self.assertFalse(result['motors_applied'])
            self.assertTrue(replay_export(path/'passive-result')['byte_match'])
            tool_spec=importlib.util.spec_from_file_location('passive_tool_check',ROOT/'tools/passive_smoke.py')
            tool=importlib.util.module_from_spec(tool_spec);tool_spec.loader.exec_module(tool)
            # Webots can discard final stdout during quit. Sealed artifacts,
            # independent replay and clean process/log are authoritative.
            self.assertTrue(tool.verify_result(path,0,'INFO: controller terminated')[0])
            self.assertFalse(tool.verify_result(path,124,'')[0])
