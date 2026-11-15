#!/usr/bin/env python3
"""Approved10s host-only forward velocity smoke, same assumed home world."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
from passive_smoke import SOURCE,prepare_project as prepare_passive,launch_project
from pil.driving_replay import replay_export,DM1_SOURCE


def prepare_project(output):
    project,world=prepare_passive(output)
    files={
        'controllers/driving_replay/driving_replay.py':SOURCE/'controllers/driving_replay/driving_replay.py',
        'libraries/pil/driving_replay.py':SOURCE/'pil/driving_replay.py'}
    for relative,source in files.items():
        target=project/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
    world.write_text(world.read_text().replace('controller "passive_replay"','controller "driving_replay"'))
    manifest_path=project/'manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest.pop('passive_overlay')
    manifest['driving_overlay']=dict(controller='driving_replay',simulation_only=True,hardware_used=False,serial_used=False,
        mapping='PWM/6250 -> DM1 forward16..20% speed -> wheel rad/s; exact0 -> velocity0; reject all other duties',
        source=DM1_SOURCE,reverse_braking='UNMODELLED; Webots zero-velocity hold is not hardware coast/brake',
        constants='radius/torque/interpolation/transients/smoke thresholds ASSUMED/UNMEASURED; see robot_parameters',
        sequence='source0..1s stop;1..2s straight20/20%;2..2.5s left16/20%;2.5..3s right20/16%;3..10s stop')
    for relative in(*files,str(world.relative_to(project))):
        manifest['artifacts_sha256'][relative]=hashlib.sha256((project/relative).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    return project,world


def verify_result(project,status,contents):
    proof=None;error=None
    try:
        proof=replay_export(project/'driving-result')
        controller_proof=json.loads((project/'driving-replay-check.json').read_text())
        summary=json.loads((project/'driving-result/summary.json').read_text())
        valid=(proof==controller_proof and all(proof['checks'].values()) and summary['samples']==10000 and
               summary['commands_issued']==10000 and summary['motors_applied'] and
               summary['status']=='DRIVING_SIMULATION_COMPLETE' and not summary['rejected_outputs'])
    except (OSError,ValueError,KeyError)as exception:
        valid=False;error=str(exception)
    return status==0 and valid and 'ERROR:'not in contents and 'WARNING:'not in contents,proof,error


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout-seconds',type=int,default=90)
    args=parser.parse_args()
    if not 20<=args.timeout_seconds<=180:parser.error('timeout must be20..180seconds')
    if os.environ.get('DISPLAY')!=':0' or not os.environ.get('XAUTHORITY'):
        parser.error('approved DISPLAY=:0 and existing XAUTHORITY required; no display server started')
    project,world=prepare_project(args.output)
    status,pid,command,contents=launch_project(project,world,args.timeout_seconds,label='driving')
    complete,proof,error=verify_result(project,status,contents)
    applied=None  # UNKNOWN if the controller did not retain its completion/failure artifact.
    try:applied=json.loads((project/'driving-result/summary.json').read_text())['motors_applied']
    except (OSError,ValueError,KeyError):pass
    result=dict(driving_smoke_complete=complete,simulation_only=True,hardware_used=False,serial_used=False,
        motors_applied=applied,exit_status=status,owned_pgid=pid,command=command,replay=proof,error=error,
        scope='forward-only simulation sign/stop smoke; reverse/braking and physical transients UNMODELLED')
    (project/'driving-smoke-summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,sort_keys=True));print(f'Retained evidence: {project}')
    return 0 if complete else 1


if __name__=='__main__':raise SystemExit(main())
