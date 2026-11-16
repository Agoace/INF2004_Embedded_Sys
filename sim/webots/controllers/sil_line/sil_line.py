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
    from pil.sil_junction_barcode import BC_FIELDS
    return (*NATIVE_FIELDS,*(SEARCH_FIELDS if config.get('line_search')else()),*(BC_FIELDS if config.get('barcode_trial')else()))


def native_controller(project,config):
    from pil.sil_motor import MotorLineController,LagPlant
    if config.get('barcode_trial'):
        from pil.sil_junction_barcode import BarcodeController
        return BarcodeController(project/'libraries/libsil_line.so'),LagPlant(bidirectional=True)
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
    if manifest.get('barcode_trial'):
        info=controller.barcode_calibrate(manifest['barcode_calibration_rows'],manifest['barcode_sigma_along_mm'])
        if info!=manifest['barcode_calibration']:raise ValueError('GP28 calibration replay mismatch')
    for row in rows:
        actual=controller.update(int(row[1]),int(row[2]),int(row[0]),*(map(int,row[22:24])if native else()),*( [int(row[36])]if manifest.get('barcode_trial')else[]))
        if list(actual.values())[:9]!=list(row[3:12]):raise ValueError('C recorded-input replay mismatch')
        if native and list(actual.values())[9:]!=list(row[24:]):raise ValueError('native setpoint replay mismatch')
        targets=(plant.advance(actual['left_pct'],actual['right_pct'])if native else
                 forward_velocity(actual['left_pct'],actual['right_pct'],32.5))
        if targets!=tuple(row[18:20]):raise ValueError('plant target replay mismatch')
    return True


def completed_segment(config,rows):
    ticks=config.get('duration_ms',10000)
    if config.get('stop_after_junction'):
        return bool(config.get('barcode_trial')and rows and len(rows)<=ticks and
            rows[-1][47]>=1 and not any(r[47]for r in rows[:-1]))
    return len(rows)==ticks


def replay_export(project):
    project=Path(project);path=project/'sil-result/samples.csv'
    if not path.is_file()or path.stat().st_size>8_000_000:raise ValueError('SIL replay file budget')
    rows=[];config=json.loads((project/'sil-manifest.json').read_text());ticks=config.get('duration_ms',10000)
    with path.open(newline='')as stream:
        reader=csv.reader(stream)
        expected=[*FIELDS,*extra_fields(json.loads((project/'sil-manifest.json').read_text()))]
        if next(reader,None)!=expected:raise ValueError('SIL replay fields')
        for row in reader:
            if len(rows)>=ticks or len(row)!=len(expected):raise ValueError('SIL replay row budget/width')
            converted=tuple(int(v)if i in(0,1,2,3,4,5,6,9,10,11)or i>=22 else float(v)for i,v in enumerate(row))
            if converted[0]!=len(rows)+1 or any(not math.isfinite(v)for v in converted):raise ValueError('SIL replay time/value')
            rows.append(converted)
    if not completed_segment(config,rows):raise ValueError('SIL incomplete replay')
    return replay(project,rows)


def run(robot,project):
    project=Path(project);config=json.loads((project/'sil-manifest.json').read_text())
    if robot.getBasicTimeStep()!=1:raise ValueError('SIL needs1ms step')
    from pil.sil_optics import make_sampler
    scene=make_sampler(build_course(Course.load(project/'course.json')),config.get('optical_sampler','fast'))
    profiles={name:SensorProfile(**data)for name,data in config['profiles'].items()}
    native=config.get('motor_fidelity',False)
    if native:
        controller,plant=native_controller(project,config)
    else:controller=LineController(project/'libraries/libsil_line.so')
    calibration=controller.calibrate(config['calibration_rows'])
    bc_profiles={}
    if config.get('barcode_trial'):
        info=controller.barcode_calibrate(config['barcode_calibration_rows'],config['barcode_sigma_along_mm'])
        if info!=config['barcode_calibration']:raise ValueError('GP28 calibration mismatch')
        bc_profiles={k:SensorProfile(**v)for k,v in config['barcode_profiles'].items()}
    if config.get('spin_endpoints')and calibration['endpoints']!=config['spin_endpoints']:raise ValueError('spin endpoints mismatch')
    wheels=[robot.getDevice(n)for n in('left_motor','right_motor')]
    for wheel in wheels:wheel.setPosition(float('inf'));wheel.setAvailableTorque(.12);wheel.setVelocity(0.)
    robot.getDevice('sonar_servo').setPosition(0.)
    encoders=[robot.getDevice(n)for n in('left_encoder','right_encoder')]
    for encoder in encoders:encoder.enable(1)
    rows=[];rejections=[];error=None;fault=False;ticks=config.get('duration_ms',10000)
    optical_history=[]
    if config.get('taskA_geometry'):
        from pil.sil_excited_line import DelayPair
        adc_delay=DelayPair(config.get('sensor_transport_delay_ms',0))
    try:
        for slot in range(ticks):
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
            if config.get('taskA_geometry'):
                fresh=tuple(adc);adc=list(adc_delay.update(slot+1,adc))
                optical_history.append((slot+1,*fresh,*adc))
            angles=[encoder.getValue()for encoder in encoders]
            counts=[round(a*2535/(2*math.pi))for a in angles]
            if config.get('barcode_trial'):
                from pil.sil_optics import sample_gp28
                sample=sample_gp28(scene,bc_profiles[str(config['barcode_sigma_along_mm'])],position,yaw,
                    config['barcode_mount'],tolerance=config['barcode_fine_tolerance'],uniform_mat=config.get('barcode_uniform_mat',False))
                if not sample.valid:raise ValueError('GP28 invalid: '+sample.reason)
                adc.append(sample.adc)
            actual=controller.update(*adc[:2],slot+1,*(counts if native else()),*(adc[2:]if config.get('barcode_trial')else()))
            if actual.get('bc_event_full'):raise ValueError('barcode evidence event budget full')
            if actual['fault']:fault=True
            try:targets=(plant.advance(actual['left_pct'],actual['right_pct'])if native else
                         forward_velocity(actual['left_pct'],actual['right_pct'],32.5))
            except ValueError as exception:
                rejections.append(dict(sim_ms=slot+1,left_pct=actual['left_pct'],right_pct=actual['right_pct'],reason=str(exception)))
                raise
            for wheel,target in zip(wheels,targets):wheel.setVelocity(target)
            if config.get('curve_trial'):
                if config.get('s_curve_trial'):
                    from pil.sil_s_curve import nearest_s_path as nearest_path
                else:from pil.sil_curve import nearest_path
                pair_error=nearest_path(position[0]*1000+c*119-s*7.25,position[1]*1000+s*119+c*7.25)['error_mm']
            else:
                forward=config['mounts']['GP26']['forward_mm'];half_left=config['mounts']['GP27']['left_mm']/2
                pair_error=position[1]*1000+s*forward+c*half_left-157.5
            row=(slot+1,*adc[:2],*list(actual.values())[:9],*position,yaw,*angles,*targets,pair_error,rotation[8])
            if native:row=(*row,*counts,*list(actual.values())[9:])
            if any(not math.isfinite(v)for v in row):raise ValueError('nonfinite SIL state')
            if not .015<position[2]<.15 or abs(pair_error)>(250 if config.get('line_search')else 100):raise ValueError('SIL gross motion bound')
            rows.append(row)
            if fault and not config.get('recovery_trial'):raise ValueError('native line lost; stop latched')
            if config.get('stop_after_junction')and actual['junction_confirmations']>=1:break
    except Exception as exception:error=str(exception)
    finally:
        for wheel in wheels:wheel.setVelocity(0.)
    segment_complete=completed_segment(config,rows)
    checks=dict(duration=segment_complete,line_acquired=bool(rows)and rows[0][5]==1,
        line_retained=segment_complete and all(r[5]for r in rows),no_false_junctions=bool(rows)and not any(r[6]for r in rows),
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
        complete=len(rows)==ticks and error is None and not rejections
    else:complete=all(checks.values())
    curve=None
    if config.get('curve_trial'):
        if config.get('s_curve_trial'):
            from pil.sil_s_curve import analyze_s_curve as analyze_curve
        else:from pil.sil_curve import analyze_curve
        curve=analyze_curve(rows)
    if config.get('barcode_trial'):
        checks.pop('no_false_junctions')
        checks.update(junction_detected=bool(rows)and any(r[6]for r in rows),
            native_junction_confirmed=bool(rows)and rows[-1][47]>=1,
            forward=bool(rows)and rows[-1][12]>rows[0][12],
            shadow_preserved=bool(rows)and all(r[44]==0 and not(int(r[45])&0x40)and r[49]==0 for r in rows))
        complete=all(checks.values())and not fault
    evidence=None
    if config.get('barcode_trial')and getattr(controller,'has_diagnostics',False):
        evidence=(controller.inputs(),controller.details(),controller.events())
    proof=replay(project,rows)if complete else False
    if proof and evidence is not None:
        checks['acquisition_replay_match']=evidence==(controller.inputs(),controller.details(),controller.events())
        proof=checks['acquisition_replay_match']
    output=project/'sil-result';output.mkdir()
    with (output/'samples.csv').open('w',newline='')as stream:
        writer=csv.writer(stream);writer.writerow((*FIELDS,*extra_fields(config)));writer.writerows(rows)
    barcode=None
    if config.get('barcode_trial')and rows:
        last=rows[-1]
        barcode=dict(letter=chr(last[38])if last[38]else None,attempts=last[37],publications=last[39],
            failures=last[40],window_reason=last[41],quiet_pass=bool(last[42]),start_pass=bool(last[43]),
            valid=last[44],flags=last[45],admission_reason=last[46],junction_confirmations=last[47],
            junction_detected=any(r[6]for r in rows),junction_phase=last[48],action_command=last[49],
            final_module=last[27],final_shell_state=last[28],final_shell_fault=last[29])
        (output/'barcode-events.json').write_text(json.dumps(controller.events(),indent=2)+'\n')
        if evidence is not None:
            (output/'barcode-inputs.json').write_text(json.dumps(evidence[0],separators=(',',':'))+'\n')
            (output/'barcode-details.json').write_text(json.dumps(evidence[1],indent=2)+'\n')
            (output/'barcode-events.json').write_text(json.dumps(evidence[2],indent=2)+'\n')
    if config.get('taskA_geometry'):
        with (output/'taskA-optical-inputs.csv').open('w',newline='')as stream:
            writer=csv.writer(stream);writer.writerow(('ms','fresh_l','fresh_c','delivered_l','delivered_c'));writer.writerows(optical_history)
    (output/'rejections.json').write_text(json.dumps(rejections,indent=2)+'\n')
    success_status='SIL_RECOVERY_EXPERIMENT_COMPLETE'if config.get('recovery_trial')else'SIL_LINE_COMPLETE'
    summary=dict(status=success_status if complete and proof else'SIL_LINE_INCOMPLETE',samples=len(rows),
        simulation_only=True,hardware_used=False,serial_used=False,recorded_input_c_replay=proof,error=error,
        calibration=calibration,configuration=controller.metadata()if native else config['controller_config'],checks=checks,recovery=recovery,
        curve=curve,barcode=barcode,gp28_calls=getattr(scene,'gp28_calls',None),
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
