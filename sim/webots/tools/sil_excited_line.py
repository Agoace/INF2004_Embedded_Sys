#!/usr/bin/env python3
"""Explicitly approved excitation/candidate cases. One launch/case, no retry."""
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent))
from sil_line_smoke import prepare_project,launch_project,SOURCE
from pil.robot_profile import RobotProfile
from pil.sil_excited_line import apply_geometry,window_metrics

CASES={'offset10-yaw-10':(10,-10,0),'offset15-yaw0':(15,0,0),'offset10-yaw-10-delay20':(10,-10,20)}
CANDIDATES={f'P{gain}-delay{delay}':(gain,delay)for gain,delay in ((60,20),(40,20),(85,30),(60,30),(40,30))}
CASES.update({name:(10,-10,delay)for name,(_,delay)in CANDIDATES.items()})
BASE_TIMING={f'B{base}-delay{delay}':(base,delay,False)for base in (120,100)for delay in (20,30)}
BASE_TIMING.update({f'same-tick-delay{delay}':(146,delay,True)for delay in (20,30)})
CASES.update({name:(10,-10,delay)for name,(_,delay,_)in BASE_TIMING.items()})

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',choices=CASES,required=True)
    p.add_argument('--spin',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('fresh directory required; no retries')
    if os.environ.get('DISPLAY')!=':0'or not os.environ.get('XAUTHORITY'):p.error('approved existingDISPLAY=:0/XAUTHORITY required')
    offset,yaw,delay=CASES[a.case];spin=json.loads(a.spin.read_text())
    if not spin.get('complete'):p.error('historical spin incomplete')
    profile=RobotProfile.load(SOURCE/'calibration/robot_assumed.json')
    project,world=prepare_project(a.output,offset_mm=offset,yaw_deg=yaw,recovery=True,line_search=True,line_profile='B')
    config_path=project/'sil-manifest.json';config=json.loads(config_path.read_text())
    config=apply_geometry(config,profile,spin);config['sensor_transport_delay_ms']=delay
    if a.case in CANDIDATES:
        from pil.sil_line_candidates import build_candidate
        gain,_=CANDIDATES[a.case];record=build_candidate(project/'libraries',gain)
        config['LINE_P_NEAR']=gain;config['candidate_source_commit']=record['source_commit']
        config['candidate_source_policy']=record['source_policy']
    if a.case in BASE_TIMING:
        from pil.sil_line_candidates import build_candidate,BASE_REVISION
        base,_,same_tick=BASE_TIMING[a.case]
        record=build_candidate(project/'libraries',85,base_mm_s=base,same_tick=same_tick,revision=BASE_REVISION)
        config.update(LINE_P_NEAR=85,LINE_BASE=base,same_tick_apply=same_tick,record_motor_timing=True,
            candidate_source_commit=record['source_commit'],candidate_source_policy=record['source_policy'],
            motor_order='shell step then motor tick'if same_tick else'motor tick then shell step',
            delay_audit='Explicit ADC transport; existing DM1/80ms unchanged; same-tick changes order only, removes one10ms modeled publication wait, not a separately measured8ms delay.')
    config['taskA_case']=a.case;config['historical_spin_path']=str(a.spin.resolve())
    config['historical_spin_sha256']=hashlib.sha256(a.spin.read_bytes()).hexdigest()
    config_path.write_text(json.dumps(config,indent=2,sort_keys=True)+'\n')
    (project/'historical-spin.json').write_bytes(a.spin.read_bytes())
    # Re-seal this NEW project's exact launch inputs; historical seals unchanged.
    manifest_path=project/'manifest.json';manifest=json.loads(manifest_path.read_text())
    for path in project.rglob('*'):
        if path.is_file()and path!=manifest_path:
            manifest['artifacts_sha256'][str(path.relative_to(project))]=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    start=time.monotonic();status,pid,command,contents=launch_project(project,world,1200,label='taskA-excited-line')
    elapsed=time.monotonic()-start;summary=None;metrics=[];proof=False;error=None
    try:
        summary=json.loads((project/'sil-result/summary.json').read_text())
        if summary.get('recorded_input_c_replay'):
            spec=importlib.util.spec_from_file_location('taskA_replay',project/'controllers/sil_line/sil_line.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);proof=module.replay_export(project)
        with (project/'sil-result/samples.csv').open()as stream:
            reader=csv.reader(stream);next(reader);rows=[list(map(float,r))for r in reader]
        if a.case in BASE_TIMING:
            from pil.sil_line_base_timing import WINDOWS,analyze_window
            for start_ms,end_ms in WINDOWS:
                if rows and rows[-1][0]>=end_ms:metrics.append(analyze_window(rows,start_ms,end_ms,profile['track_mm']))
        else:
            for start_ms,end_ms in ((2000,4000),(4000,6000),(6000,8000),(8000,10000)):
                if rows and rows[-1][0]>=end_ms:metrics.append(window_metrics(rows,start_ms,end_ms))
    except (ValueError,OSError)as e:error=str(e)
    result=dict(case=a.case,owned_pgid=pid,exit_status=status,wall_seconds=elapsed,cap_seconds=1200,command=command,
        complete=bool(status==0 and proof and summary and summary['samples']==10000 and summary['error']is None and 'ERROR:'not in contents and 'WARNING:'not in contents),
        recorded_input_native_replay=proof,summary=summary,analysis_windows=metrics,error=error)
    (project/'taskA-result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,sort_keys=True),flush=True)
    return 0 if result['complete']else 1

if __name__=='__main__':raise SystemExit(main())
