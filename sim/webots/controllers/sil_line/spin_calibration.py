"""Actual simulated motor/optical spin; sim-owned equivalent of vehicle_task sequencing."""
import csv
import json
import math
from pathlib import Path
import sys
PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'libraries'))
from pil.sil_calibration import CalibrationController,CalibrationSequence
from pil.sil_motor import LagPlant
from pil.course_builder import Course,build_course
from pil.s3_fast import FastScene
from pil.s3_optics import SensorProfile


def run(robot,project):
    project=Path(project);config=json.loads((project/'sil-manifest.json').read_text())
    controller=CalibrationController(project/'libraries/libsil_line.so')
    reader=None;bc_rows=[];bc_cal=None;bc_profiles={}
    if config.get('barcode_trial'):
        from pil.sil_junction_barcode import BarcodeController
        reader=BarcodeController(project/'libraries/libsil_line.so')
        reader.barcode_begin();controller.lib.sil_calib_begin()
        bc_profiles={k:SensorProfile(**v)for k,v in config['barcode_profiles'].items()}
    sequence=CalibrationSequence(controller.spin_config);plant=LagPlant(bidirectional=True)
    from pil.sil_optics import make_sampler
    scene=make_sampler(build_course(Course.load(project/'course.json')),config.get('optical_sampler','fast'))
    profiles={name:SensorProfile(**data)for name,data in config['profiles'].items()}
    motors=[robot.getDevice(n)for n in('left_motor','right_motor')]
    encoders=[robot.getDevice(n)for n in('left_encoder','right_encoder')]
    for motor in motors:motor.setPosition(float('inf'));motor.setAvailableTorque(.12);motor.setVelocity(0.)
    for encoder in encoders:encoder.enable(1)
    rows=[];duties=(0.,0.);error=None
    try:
        for ms in range(1,10001):
            if robot.step(1)==-1:raise ValueError('early calibration step end')
            node=robot.getSelf();position=node.getPosition();rotation=node.getOrientation()
            if rotation[8]<.999:raise ValueError('calibration tilt outside flat optics')
            yaw=math.atan2(rotation[3],rotation[0]);c,s=math.cos(yaw),math.sin(yaw);adc=[]
            for name in('GP27','GP26'):
                mount=config['mounts'][name];f,l=mount['forward_mm'],mount['left_mm']
                origin=(position[0]*1000+c*f-s*l,position[1]*1000+s*f+c*l,10+mount['flat_clearance_mm'])
                sample=scene.sample(profiles[name],origin,axis_u=(c,s,0),sample_us=ms*1000)
                if not sample.valid:raise ValueError('calibration optical sample invalid: '+sample.reason)
                adc.append(sample.adc)
            counts=[round(e.getValue()*2535/(2*math.pi))for e in encoders]
            bc_raw=[]
            if reader is not None:
                from pil.sil_optics import sample_gp28
                for n in(1,4,8):
                    sample=sample_gp28(scene,bc_profiles[str(n)],position,yaw,config['barcode_mount'],
                        tolerance=config['barcode_fine_tolerance'],uniform_mat=config.get('barcode_uniform_mat',False))
                    if not sample.valid:raise ValueError('GP28 calibration sample: '+sample.reason)
                    bc_raw.append(sample.adc)
                if sequence.collecting:
                    reader.barcode_add(ms*1000,bc_raw);bc_rows.append([ms*1000,*bc_raw])
            centre=controller.sample(adc,ms,sequence.collecting)
            if ms%10==0:
                command=sequence.advance(ms,counts,centre,controller.peak)
                if command['end']:
                    controller.finish()
                    if reader is not None:bc_cal=reader.barcode_end()
                if command['stop']:controller.stop()
                left,right,fault=controller.tick(ms,counts,command['speeds'])
                if fault:raise ValueError('motor fault during spin')
                duties=(left/10,right/10)
            targets=plant.advance(*duties)
            for motor,value in zip(motors,targets):motor.setVelocity(value)
            rows.append((ms,*adc,*counts,*duties,*position,yaw,centre,controller.peak,sequence.stage,*bc_raw))
            if sequence.done:break
        if not sequence.done:raise ValueError('calibration total time limit')
    except Exception as exception:error=str(exception)
    finally:
        for motor in motors:motor.setVelocity(0.)
    output=project/'spin-result';output.mkdir()
    with (output/'samples.csv').open('w',newline='')as stream:
        writer=csv.writer(stream);writer.writerow(('sim_ms','left_adc','centre_adc','count_l','count_r','duty_l_pct',
            'duty_r_pct','x_m','y_m','z_m','yaw_rad','norm_centre','norm_peak','stage',*(['bc_raw1','bc_raw4','bc_raw8']if reader is not None else[])));writer.writerows(rows)
    result=dict(complete=error is None and sequence.done,calibration=controller.info,
        calibration_rows=controller.rows,sequence=sequence.history,config=controller.spin_config,error=error,
        barcode_calibration=bc_cal,barcode_calibration_rows=bc_rows,gp28_calls=getattr(scene,'gp28_calls',None),
        optical_sampler=config.get('optical_sampler','fast'),source='vehicle_task.c689–788 sequencing equivalent; line_service.c141 public lp_calib_add; NOT SDK task execution')
    (output/'calibration.json').write_text(json.dumps(result,indent=2)+'\n')
    robot.simulationQuit(0 if result['complete']else 1)


if __name__=='__main__':
    from controller import Supervisor
    run(Supervisor(),PROJECT)
