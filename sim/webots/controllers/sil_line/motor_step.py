"""One-second simulated150mm/s step through real motor_ctl; no line tuning."""
import csv
import json
import math
from pathlib import Path
import sys
PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'libraries'))
from pil.sil_motor import MotorLineController,LagPlant,TAU_MS,FIT_SOURCE


def run(robot,project):
    project=Path(project);controller=MotorLineController(project/'libraries/libsil_line.so');plant=LagPlant()
    motors=[robot.getDevice(n)for n in('left_motor','right_motor')]
    encoders=[robot.getDevice(n)for n in('left_encoder','right_encoder')]
    for motor in motors:motor.setPosition(float('inf'));motor.setAvailableTorque(.12);motor.setVelocity(0.)
    for encoder in encoders:encoder.enable(1)
    rows=[];duty=(0.,0.);last=[0.,0.];t90=[None,None];error=None
    try:
        for ms in range(1,1001):
            if robot.step(1)==-1:raise ValueError('early step end')
            angles=[e.getValue()for e in encoders];counts=[round(a*2535/(2*math.pi))for a in angles]
            speeds=[(a-b)*32500 for a,b in zip(angles,last)];last=angles
            if ms%10==0:
                l,r,fault=controller.tick(ms,counts,(150,150))
                if fault:raise ValueError('native motor fault')
                duty=(l/10,r/10)
            target=plant.advance(*duty)
            for motor,v in zip(motors,target):motor.setVelocity(v)
            for side in(0,1):
                if t90[side]is None and speeds[side]>=135:t90[side]=ms-10
            rows.append((ms,*counts,*duty,*speeds,*target))
    except Exception as exception:error=str(exception)
    finally:
        for motor in motors:motor.setVelocity(0.)
    with (project/'motor-step.csv').open('w',newline='')as stream:
        writer=csv.writer(stream);writer.writerow(('sim_ms','count_l','count_r','duty_l_pct','duty_r_pct',
            'speed_l_mm_s','speed_r_mm_s','lag_l_rad_s','lag_r_rad_s'));writer.writerows(rows)
    passed=error is None and all(t is not None and 88<=t<=132 for t in t90)
    summary=dict(passed=passed,raw_t90_ms=t90,measured_raw_t90_ms=[110,110],allowed_ms=[88,132],
                 tau_ms=TAU_MS,fit_source=FIT_SOURCE,motor_config=controller.metadata(),error=error)
    (project/'motor-step.json').write_text(json.dumps(summary,indent=2)+'\n')
    robot.simulationQuit(0 if passed else 1)


if __name__=='__main__':
    from controller import Supervisor
    run(Supervisor(),PROJECT)
