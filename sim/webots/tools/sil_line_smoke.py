#!/usr/bin/env python3
"""Build/run deterministic-input fast SIL line subset. No serial or Pico."""
import argparse
from dataclasses import asdict,fields
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from passive_smoke import SOURCE,launch_project
from pil.course_builder import Course
from pil.robot_profile import RobotProfile
from pil.world import write_project
from pil.sil_line import build_library,CONFIG
from pil.s3_assumed_line import load_assumed_line_profile,load_assumed_line_mount


def prepare_project(output,*,offset_mm=0,yaw_deg=0,recovery=False,motor_fidelity=False,line_search=False):
    course=Course.load(SOURCE/'courses/sil'/('straight_search_assumed.json'if line_search else'straight_assumed.json'))
    if recovery:
        data=course.data;data['start']['position_mm'][1]+=offset_mm;data['start']['yaw_deg']=yaw_deg
        course=Course.from_dict(data)
    profile=RobotProfile.load(SOURCE/'calibration/robot_assumed.json')
    project=write_project(course,profile,output)
    world=project/'worlds/sil_straight_assumed.wbt'
    world.write_text(world.read_text().replace('RoboPico {\n','RoboPico {\n controller "sil_line"\n'))
    destination=project/'controllers/sil_line';destination.mkdir()
    shutil.copyfile(SOURCE/'controllers/sil_line/sil_line.py',destination/'sil_line.py')
    libs=project/'libraries';libs.mkdir(exist_ok=True)
    for source in (SOURCE/'pil').glob('*.py'):
        target=libs/'pil'/source.name;target.parent.mkdir(exist_ok=True);shutil.copyfile(source,target)
    if line_search:
        from pil.sil_search import build_search_library
        build_search_library(libs)
    elif motor_fidelity:
        from pil.sil_motor import build_motor_library
        build_motor_library(libs)
    else:build_library(libs)
    def profile_dict(name):
        profile=load_assumed_line_profile(name)
        return {f.name:dict(getattr(profile,f.name))if f.name=='levels_adc'else getattr(profile,f.name)for f in fields(profile)}
    config=dict(profiles={n:profile_dict(n)for n in('GP26','GP27')},
        mounts={n:asdict(load_assumed_line_mount(n))for n in('GP26','GP27')},
        calibration_rows=[[200,200,0]]*16+[[3700,3700,0]]*4,
        calibration_provenance='SYNTHETIC broad uniform floor/ink training, NOT a thin-strip sweep',
        controller_config=CONFIG,motor_fidelity=motor_fidelity or line_search,line_search=line_search,recovery_trial=recovery,start_offset_mm=offset_mm,start_yaw_deg=yaw_deg,flat_projection='ASSUMED actualXY/yaw projected to operator approximate8mm clearance; tilt response unmodelled')
    (project/'sil-manifest.json').write_text(json.dumps(config,indent=2,sort_keys=True)+'\n')
    manifest_path=project/'manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest['sil_scope']=('unchanged vehicle_sm+LINE(line_trace)+SEARCH(line_search)+native motor_ctl; no other registered modules or RTOS'
        if line_search else'unchanged pure line_position+steering C'+(' + native motor_ctl'if motor_fidelity else'')+'; not native tasks/RTOS/full vehicle')
    for path in project.rglob('*'):
        if path.is_file()and path!=manifest_path:
            manifest['artifacts_sha256'][str(path.relative_to(project))]=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    return project,world


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout-seconds',type=int,default=90);args=parser.parse_args()
    if not 20<=args.timeout_seconds<=180:parser.error('timeout20..180s')
    if os.environ.get('DISPLAY')!=':0' or not os.environ.get('XAUTHORITY'):parser.error('existing approved DISPLAY=:0/XAUTHORITY required')
    project,world=prepare_project(args.output)
    status,pid,command,contents=launch_project(project,world,args.timeout_seconds,label='sil')
    summary=None;error=None;independent_replay=False
    try:
        summary=json.loads((project/'sil-result/summary.json').read_text())
        spec=importlib.util.spec_from_file_location('sil_replay',project/'controllers/sil_line/sil_line.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        independent_replay=module.replay_export(project)
        if hashlib.sha256((project/'sil-result/samples.csv').read_bytes()).hexdigest()!=summary['samples_sha256']:
            raise ValueError('SIL sample hash mismatch')
    except (OSError,ValueError)as exception:error=str(exception)
    complete=bool(status==0 and summary and summary['status']=='SIL_LINE_COMPLETE' and
        independent_replay and summary['recorded_input_c_replay'] and 'ERROR:'not in contents and 'WARNING:'not in contents)
    result=dict(complete=complete,independent_replay=independent_replay,exit_status=status,owned_pgid=pid,command=command,summary=summary,error=error)
    (project/'sil-smoke-summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,sort_keys=True));print(f'Retained evidence: {project}')
    return 0 if complete else 1


if __name__=='__main__':raise SystemExit(main())
