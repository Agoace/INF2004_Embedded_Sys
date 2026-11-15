"""Synchronous SIL: simulated optical ADC -> unchanged pure C -> wheel velocity."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'libraries'))
from pil.sil_line import LineController,forward_velocity
from pil.course_builder import Course,build_course
from pil.s3_fast import FastScene
from pil.s3_optics import SensorProfile

FIELDS=('sim_ms','left_adc','centre_adc','position','event','line_seen','junction','left_pct','right_pct',
        'norm_left','norm_centre','fault','x_m','y_m','z_m','yaw_rad','left_rad','right_rad',
        'target_left_rad_s','target_right_rad_s','pair_error_mm','upright_cos')


def extra_fields(config):
    if not config.get('motor_fidelity'):return ()
    from pil.sil_search import NATIVE_FIELDS,SEARCH_FIELDS
    return (*NATIVE_FIELDS,*(SEARCH_FIELDS if config.get('line_search')else()))


def native_controller(project,config):
    from pil.sil_motor import MotorLineController,LagPlant
    if config.get('line_search'):
        from pil.sil_search import SearchController
        return SearchController(project/'libraries/libsil_line.so'),LagPlant(bidirectional=True)
    return MotorLineController(project/'libraries/libsil_line.so'),LagPlant()


def replay(project,rows):
    manifest=json.loads((project/'sil-manifest.json').read_text())
    native=manifest.get('motor_fidelity',False)
    if native:
        controller,plant=native_controller(project,manifest)
    else:controller=LineController(project/'libraries/libsil_line.so')
    calibration=controller.calibrate(manifest['calibration_rows'])
    if manifest.get('spin_endpoints')and calibration['endpoints']!=manifest['spin_endpoints']:raise ValueError('spin endpoint replay mismatch')
    for row in rows:
        actual=controller.update(int(row[1]),int(row[2]),int(row[0]),*(map(int,row[22:24])if native else()))
        if list(actual.values())[:9]!=list(row[3:12]):raise ValueError('C recorded-input replay mismatch')
        if native and list(actual.values())[9:]!=list(row[24:]):raise ValueError('native setpoint replay mismatch')
        targets=(plant.advance(actual['left_pct'],actual['right_pct'])if native else
                 forward_velocity(actual['left_pct'],actual['right_pct'],32.5))
        if targets!=tuple(row[18:20]):raise ValueError('plant target replay mismatch')
    return True


def replay_export(project):
    project=Path(project);path=project/'sil-result/samples.csv'
    if not path.is_file()or path.stat().st_size>8_000_000:raise ValueError('SIL replay file budget')
    rows=[]
    with path.open(newline='')as stream:
        reader=csv.reader(stream)
        expected=[*FIELDS,*extra_fields(json.loads((project/'sil-manifest.json').read_text()))]
        if next(reader,None)!=expected:raise ValueError('SIL replay fields')
        for row in reader:
            if len(rows)>=10000 or len(row)!=len(expected):raise ValueError('SIL replay row budget/width')
            converted=tuple(int(v)if i in(0,1,2,3,4,5,6,9,10,11)or i>=22 else float(v)for i,v in enumerate(row))
            if converted[0]!=len(rows)+1 or any(not math.isfinite(v)for v in converted):raise ValueError('SIL replay time/value')
            rows.append(converted)
    if len(rows)!=10000:raise ValueError('SIL incomplete replay')
    return replay(project,rows)


def run(robot,project):
    project=Path(project);config=json.loads((project/'sil-manifest.json').read_text())
    if robot.getBasicTimeStep()!=1:raise ValueError('SIL needs1ms step')
    scene=FastScene.from_course(build_course(Course.load(project/'course.json')))
    profiles={name:SensorProfile(**data)for name,data in config['profiles'].items()}
    native=config.get('motor_fidelity',False)
    if native:
        controller,plant=native_controller(project,config)
    else:controller=LineController(project/'libraries/libsil_line.so')
    calibration=controller.calibrate(config['calibration_rows'])
    if config.get('spin_endpoints')and calibration['endpoints']!=config['spin_endpoints']:raise ValueError('spin endpoints mismatch')
    wheels=[robot.getDevice(n)for n in('left_motor','right_motor')]
    for wheel in wheels:wheel.setPosition(float('inf'));wheel.setAvailableTorque(.12);wheel.setVelocity(0.)
    robot.getDevice('sonar_servo').setPosition(0.)
    encoders=[robot.getDevice(n)for n in('left_encoder','right_encoder')]
    for encoder in encoders:encoder.enable(1)
    rows=[];rejections=[];error=None;fault=False
    try:
        for slot in range(10000):
            if robot.step(1)==-1 or round(robot.getTime()*1000)!=slot+1:raise ValueError('SIL step sequence')
            node=robot.getSelf();position=node.getPosition();rotation=node.getOrientation()
            yaw=math.atan2(rotation[3],rotation[0]);c,s=math.cos(yaw),math.sin(yaw)
            # Flat F1 projection: actual XY/yaw, configured clearance above flat
            # mat. Bounce/tilt response UNMODELLED; reject a gross tilt instead.
            if rotation[8]<.999:raise ValueError('outside flat optical model tilt domain')
            adc=[]
            for name in('GP27','GP26'):
                mount=config['mounts'][name];f,l=mount['forward_mm'],mount['left_mm']
                origin=(position[0]*1000+c*f-s*l,position[1]*1000+s*f+c*l,10+mount['flat_clearance_mm'])
                sample=scene.sample(profiles[name],origin,axis_u=(c,s,0),sample_us=(slot+1)*1000)
                if not sample.valid:raise ValueError('optical sample invalid: '+sample.reason)
                adc.append(sample.adc)
            angles=[encoder.getValue()for encoder in encoders]
            counts=[round(a*2535/(2*math.pi))for a in angles]
            actual=controller.update(*adc,slot+1,*(counts if native else()))
            if actual['fault']:fault=True
            try:targets=(plant.advance(actual['left_pct'],actual['right_pct'])if native else
                         forward_velocity(actual['left_pct'],actual['right_pct'],32.5))
            except ValueError as exception:
                rejections.append(dict(sim_ms=slot+1,left_pct=actual['left_pct'],right_pct=actual['right_pct'],reason=str(exception)))
                raise
            for wheel,target in zip(wheels,targets):wheel.setVelocity(target)
            pair_error=position[1]*1000+s*119+c*7.25-157.5
            row=(slot+1,*adc,*list(actual.values())[:9],*position,yaw,*angles,*targets,pair_error,rotation[8])
            if native:row=(*row,*counts,*list(actual.values())[9:])
            if any(not math.isfinite(v)for v in row):raise ValueError('nonfinite SIL state')
            if not .015<position[2]<.15 or abs(pair_error)>(250 if config.get('line_search')else 100):raise ValueError('SIL gross motion bound')
            rows.append(row)
            if fault and not config.get('recovery_trial'):raise ValueError('native line lost; stop latched')
    except Exception as exception:error=str(exception)
    finally:
        for wheel in wheels:wheel.setVelocity(0.)
    checks=dict(duration=len(rows)==10000,line_acquired=bool(rows)and rows[0][5]==1,
        line_retained=len(rows)==10000 and all(r[5]for r in rows),no_false_junctions=bool(rows)and not any(r[6]for r in rows),
        forward=bool(rows)and rows[-1][12]-rows[0][12]>.5,
        encoders_forward=bool(rows)and all(rows[-1][i]>rows[0][i]for i in(16,17)),
        within_strip=bool(rows)and max(abs(r[20])for r in rows)<10,
        inputs_accepted=error is None and not rejections and not fault)
    recovery=None
    if config.get('recovery_trial'):
        from pil.sil_recovery import analyze_recovery
        recovery=analyze_recovery(rows)if rows else None
        if recovery is not None and config.get('line_search'):
            recovery.update(searches=rows[-1][32],found_by_search=rows[-1][33],
                final_module=rows[-1][27],shell_fault=rows[-1][29],shell_fault_detail=rows[-1][30])
        complete=len(rows)==10000 and error is None and not rejections
    else:complete=all(checks.values())
    proof=replay(project,rows)if complete else False
    output=project/'sil-result';output.mkdir()
    with (output/'samples.csv').open('w',newline='')as stream:
        writer=csv.writer(stream);writer.writerow((*FIELDS,*extra_fields(config)));writer.writerows(rows)
    (output/'rejections.json').write_text(json.dumps(rejections,indent=2)+'\n')
    success_status='SIL_RECOVERY_EXPERIMENT_COMPLETE'if config.get('recovery_trial')else'SIL_LINE_COMPLETE'
    summary=dict(status=success_status if complete and proof else'SIL_LINE_INCOMPLETE',samples=len(rows),
        simulation_only=True,hardware_used=False,serial_used=False,recorded_input_c_replay=proof,error=error,
        calibration=calibration,configuration=controller.metadata()if native else config['controller_config'],checks=checks,recovery=recovery,
        max_pair_error_mm=max((abs(r[20])for r in rows),default=None),
        line_seen_samples=sum(r[5]for r in rows),junction_samples=sum(r[6]for r in rows),
        travel_x_m=rows[-1][12]-rows[0][12]if rows else None,
        encoder_delta_rad=[rows[-1][i]-rows[0][i]for i in(16,17)]if rows else None,
        samples_sha256=hashlib.sha256((output/'samples.csv').read_bytes()).hexdigest(),
        limits='ASSUMED fitted optics/flat projection/velocity plant; pure C subset, not full robot or physical control')
    (output/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    robot.simulationQuit(0 if complete and proof else 1)


if __name__=='__main__':
    from controller import Supervisor
    run(Supervisor(),PROJECT)
