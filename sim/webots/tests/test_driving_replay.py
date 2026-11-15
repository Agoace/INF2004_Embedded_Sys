import csv
import importlib.util
import itertools
import json
import math
from pathlib import Path
import tempfile
import unittest
from pil.driving_replay import DrivingReplay,map_pwm_to_velocity,drive_command,replay_export

ROOT=Path(__file__).resolve().parents[1]
VALUES=(0.,0.,0.,0.,0.,9.81,1.,0.,0.,.196,.1575,.0425)

class DrivingTests(unittest.TestCase):
    def test_measured_endpoints_and_explicit_rejection(self):
        self.assertEqual(map_pwm_to_velocity(0,0,32.5),(0.,0.))
        for pwm,left,right in((1000,96,95),(1250,129,129),(1125,112.5,112)):
            actual=map_pwm_to_velocity(pwm,pwm,32.5)
            self.assertAlmostEqual(actual[0],left/32.5);self.assertAlmostEqual(actual[1],right/32.5)
        for bad in(-1250,1,999,1251,6250):
            with self.assertRaises(ValueError):map_pwm_to_velocity(bad,1250,32.5)
        rec=DrivingReplay(2535,32.5,command_script=lambda n:(999,1250,'bad'))
        with self.assertRaises(ValueError):rec.sample(1000,1,2,VALUES)
        self.assertEqual(rec.rejections[0]['pwm_left'],999)
        with tempfile.TemporaryDirectory()as tmp:
            rec.export(Path(tmp)/'rejected')
            self.assertEqual(len(json.loads((Path(tmp)/'rejected/rejections.json').read_text())),1)

    def test_driving_project_artifacts(self):
        import hashlib
        spec=importlib.util.spec_from_file_location('driving_tool',ROOT/'tools/driving_smoke.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(dir=ROOT.parents[1]/'build')as tmp:
            project,world=module.prepare_project(Path(tmp)/'new')
            manifest=json.loads((project/'manifest.json').read_text())
            self.assertIn('controller "driving_replay"',world.read_text())
            self.assertNotIn('passive_overlay',manifest)
            self.assertIn('driving_overlay',manifest)
            for name,digest in manifest['artifacts_sha256'].items():
                self.assertEqual(hashlib.sha256((project/name).read_bytes()).hexdigest(),digest,name)

    def test_fixed_script_boundaries(self):
        for n,expected in((0,(0,0,'stop')),(999,(0,0,'stop')),(1000,(1250,1250,'straight')),
            (1999,(1250,1250,'straight')),(2000,(1000,1250,'left')),(2499,(1000,1250,'left')),
            (2500,(1250,1000,'right')),(2999,(1250,1000,'right')),(3000,(0,0,'stop')),(9999,(0,0,'stop'))):
            self.assertEqual(drive_command(n),expected)

    def test_nonfinite_motion_fails_closed_and_zeroes_targets(self):
        spec=importlib.util.spec_from_file_location('driving_controller_bad',ROOT/'controllers/driving_replay/driving_replay.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        robot=FakeRobot();robot.getOrientation=lambda:[float('nan')]*9
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);(path/'manifest.json').write_text(json.dumps({'robot_parameters':{
                'counts_per_revolution':{'value':2535},'wheel_diameter_mm':{'value':65},'stall_torque_nm':{'value':.12}}}))
            clock=itertools.count();module.run(robot,path,raw_clock=lambda:next(clock))
            summary=json.loads((path/'driving-result/summary.json').read_text())
            self.assertEqual(robot.quit,1);self.assertEqual(robot.targets,[0.,0.])
            self.assertEqual(summary['error'],'nonfinite motion')
            self.assertFalse((path/'driving-replay-check.json').exists())

    def test_controller_direction_stop_recording_and_replay(self):
        spec=importlib.util.spec_from_file_location('driving_controller',ROOT/'controllers/driving_replay/driving_replay.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        robot=FakeRobot()
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);(path/'manifest.json').write_text(json.dumps({'robot_parameters':{
                'counts_per_revolution':{'value':2535},'wheel_diameter_mm':{'value':65},'stall_torque_nm':{'value':.12}}}))
            clock=itertools.count()
            module.run(robot,path,raw_clock=lambda:next(clock))
            summary=json.loads((path/'driving-result/summary.json').read_text())
            self.assertEqual(robot.n,10000);self.assertEqual(robot.quit,0)
            self.assertTrue(all(summary['checks'].values()),summary)
            self.assertTrue(summary['motors_applied'])
            self.assertEqual(robot.targets,[0.,0.])
            self.assertTrue(replay_export(path/'driving-result')['byte_match'])
            motion=path/'driving-result/motion.csv'
            with motion.open()as stream:rows=list(csv.reader(stream))
            rows[1500][-1]='999'
            with motion.open('w',newline='')as f:csv.writer(f).writerows(rows)
            with self.assertRaises(ValueError):replay_export(path/'driving-result')


class FakeRobot:
    def __init__(self):
        self.n=0;self.targets=[0.,0.];self.angles=[0.,0.];self.x=.196;self.y=.1575;self.yaw=0.;self.vx=0.;self.vy=0.;self.wz=0.
    def getBasicTimeStep(self):return 1
    def getDevice(self,name):return FakeDevice(self,name)
    def step(self,n):
        assert n==1;self.n+=1
        speed=.0325*sum(self.targets)/2;self.wz=.0325*(self.targets[1]-self.targets[0])/.111
        mid=self.yaw+self.wz*.0005;self.vx=speed*math.cos(mid);self.vy=speed*math.sin(mid)
        self.x+=self.vx*.001;self.y+=self.vy*.001;self.yaw+=self.wz*.001
        for i in(0,1):self.angles[i]+=self.targets[i]*.001
        return 0
    def getTime(self):return self.n/1000
    def getSelf(self):return self
    def getPosition(self):return [self.x,self.y,.0425]
    def getOrientation(self):
        c,s=math.cos(self.yaw),math.sin(self.yaw);return [c,-s,0,s,c,0,0,0,1]
    def getVelocity(self):return [self.vx,self.vy,0,0,0,self.wz]
    def simulationQuit(self,status):self.quit=status

class FakeDevice:
    def __init__(self,robot,name):self.r=robot;self.name=name
    def enable(self,n):assert n==1
    def setPosition(self,v):assert v==float('inf') if self.name.endswith('_motor')else v==0
    def setAvailableTorque(self,v):assert v==.12
    def setVelocity(self,v):
        assert 0<=v<=129/32.5
        self.r.targets[0 if self.name=='left_motor' else 1]=v
    def getValue(self):return self.r.angles[0 if self.name=='left_encoder' else 1]if self.name!='servo_angle'else 0.
    def getValues(self):return [0.,0.,9.81]if self.name=='imu_accelerometer'else [1.,0.,0.]
