"""Ten simulated seconds, passive wheels, sampled devices, scripted OUTPUT ignored."""
import json
from pathlib import Path
import sys
import time

PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'libraries'))
from pil.passive_replay import PassiveReplay,replay_export


def run(robot,project,*,raw_clock=None):
    project=Path(project)
    if robot.getBasicTimeStep()!=1:raise ValueError('passive smoke requires 1ms basicTimeStep')
    clock=raw_clock or (lambda:time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)//1000)
    manifest=json.loads((project/'manifest.json').read_text())
    recording=PassiveReplay(manifest['robot_parameters']['counts_per_revolution']['value'])
    for name in('left_motor','right_motor'):robot.getDevice(name).setTorque(0.0)
    robot.getDevice('sonar_servo').setPosition(0.0)
    devices={name:robot.getDevice(name) for name in('left_encoder','right_encoder','servo_angle',
                                                 'imu_accelerometer','imu_compass_direction_only')}
    for device in devices.values():device.enable(1)
    for slot in range(10000):
        if robot.step(1)==-1:raise RuntimeError('simulation ended before ten seconds')
        sim_us=round(robot.getTime()*1_000_000)
        if sim_us!=(slot+1)*1000:raise ValueError('unexpected simulation time or skipped step')
        start=clock()
        values=(devices['left_encoder'].getValue(),devices['right_encoder'].getValue(),
                devices['servo_angle'].getValue(),*devices['imu_accelerometer'].getValues(),
                *devices['imu_compass_direction_only'].getValues(),*robot.getSelf().getPosition())
        end=clock()
        recording.sample(sim_us,start,end,values,io_clock=clock)
    # No CSV, JSON, hex logging, flushes or file writes inside the stepping loop.
    output=project/'passive-result'
    recording.export(output)
    proof=replay_export(output)
    (project/'passive-replay-check.json').write_text(json.dumps(proof,indent=2)+'\n')
    print('PASSIVE_SIMULATION_COMPLETE '+json.dumps(proof),flush=True)
    robot.simulationQuit(0)


if __name__=='__main__':
    from controller import Supervisor
    run(Supervisor(),PROJECT)
