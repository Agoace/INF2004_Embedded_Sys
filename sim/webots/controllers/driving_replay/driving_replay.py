"""Approved forward-only velocity smoke; no serial or navigation."""
import json
import math
from pathlib import Path
import sys
import time

PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'libraries'))
from pil.driving_replay import DrivingReplay,replay_export


def run(robot,project,*,raw_clock=None):
    project=Path(project)
    if robot.getBasicTimeStep()!=1:raise ValueError('driving smoke requires1ms basicTimeStep')
    clock=raw_clock or(lambda:time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)//1000)
    params=json.loads((project/'manifest.json').read_text())['robot_parameters']
    rec=DrivingReplay(params['counts_per_revolution']['value'],params['wheel_diameter_mm']['value']/2,
                      torque_limit_nm=params['stall_torque_nm']['value'])
    motors=[robot.getDevice(name)for name in('left_motor','right_motor')]
    for motor in motors:
        motor.setPosition(float('inf'));motor.setAvailableTorque(rec.torque_limit_nm);motor.setVelocity(0.)
    robot.getDevice('sonar_servo').setPosition(0.)
    devices={name:robot.getDevice(name)for name in('left_encoder','right_encoder','servo_angle',
                                                'imu_accelerometer','imu_compass_direction_only')}
    for device in devices.values():device.enable(1)
    try:
        for slot in range(10000):
            if robot.step(1)==-1:raise ValueError('simulation ended early')
            sim_us=round(robot.getTime()*1_000_000)
            if sim_us!=(slot+1)*1000:raise ValueError('skipped simulation step')
            start=clock();node=robot.getSelf()
            values=(devices['left_encoder'].getValue(),devices['right_encoder'].getValue(),
                    devices['servo_angle'].getValue(),*devices['imu_accelerometer'].getValues(),
                    *devices['imu_compass_direction_only'].getValues(),*node.getPosition())
            orientation=node.getOrientation();twist=node.getVelocity()
            yaw=math.atan2(orientation[3],orientation[0]);end=clock()
            rec.sample(sim_us,start,end,values,io_clock=clock)
            # Decoded and range-checked OUTPUT targets. Changes take effect at
            # the NEXT robot.step, never retroactively at the sampled pose.
            for motor,target in zip(motors,rec.pending[2:]):motor.setVelocity(target)
            rec.observe_motion(yaw,orientation[8],twist,clock())
    except Exception as error:
        rec.error=str(error)
    finally:
        for motor in motors:motor.setVelocity(0.)
    output=project/'driving-result';summary=rec.export(output)
    proof=None
    if summary['status']=='DRIVING_SIMULATION_COMPLETE':
        proof=replay_export(output)
        (project/'driving-replay-check.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(summary['status']+' '+json.dumps(dict(checks=summary['checks'],error=summary['error'])),flush=True)
    robot.simulationQuit(0 if proof else 1)


if __name__=='__main__':
    from controller import Supervisor
    run(Supervisor(),PROJECT)
