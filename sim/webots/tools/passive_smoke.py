#!/usr/bin/env python3
"""Approved passive, headless, ten-simulated-second smoke; never opens serial."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

SOURCE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SOURCE))
from pil.course_builder import Course
from pil.robot_profile import RobotProfile
from pil.world import write_project
from pil.passive_replay import replay_export


def prepare_project(output):
    course=Course.load(SOURCE/'courses/home_mat_1_assumed.json')
    profile=RobotProfile.load(SOURCE/'calibration/robot_assumed.json')
    project=write_project(course,profile,output)
    world=project/'worlds'/f'{course.name}.wbt'
    text=world.read_text()
    if text.count('RoboPico {\n')!=1:raise ValueError('expected one generated robot')
    world.write_text(text.replace('RoboPico {\n','RoboPico {\n controller "passive_replay"\n'))
    files={'controllers/passive_replay/passive_replay.py':'controllers/passive_replay/passive_replay.py'}
    for name in('__init__.py','wire.py','passive_replay.py'):files['libraries/pil/'+name]='pil/'+name
    for target,source in files.items():
        path=project/target;path.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(SOURCE/source,path)
    manifest_path=project/'manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest['passive_overlay']=dict(course=course.name,simulation_only=True,motors_applied=False,
        basic_time_step_ms=1,steps=10000,logging='bounded in-memory; export after stepping',
        sensors='wire encoders only; raw accelerometer/compass/servo/pose diagnostic; other VALID bits clear',
        output='scripted real-codec frames; decoded and ignored, no device handshake')
    for name in(*files,str(world.relative_to(project))):
        manifest['artifacts_sha256'][name]=hashlib.sha256((project/name).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    return project,world


def verify_result(project,status,contents):
    """Use sealed fresh artifacts, not a stdout banner that quit may discard."""
    proof=None;error=None
    try:
        proof=replay_export(project/'passive-result')
        controller_proof=json.loads((project/'passive-replay-check.json').read_text())
        summary=json.loads((project/'passive-result/summary.json').read_text())
        valid=(controller_proof==proof and summary['samples']==10000 and
               summary['simulation_end_us']==10000000 and not summary['motors_applied'])
    except (ValueError,OSError,KeyError)as exception:
        valid=False;error=str(exception)
    return (status==0 and valid and 'ERROR:' not in contents and 'WARNING:' not in contents),proof,error


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout-seconds',type=int,default=90)
    args=parser.parse_args()
    if not 20<=args.timeout_seconds<=180:parser.error('timeout must be20..180seconds')
    if os.environ.get('DISPLAY')!=':0' or not os.environ.get('XAUTHORITY'):
        parser.error('this smoke requires operator-approved DISPLAY=:0 and XAUTHORITY; no display server is started')
    project,world=prepare_project(args.output)
    env=os.environ.copy();env['XDG_CONFIG_HOME']=str(project/'private-config')
    env['WEBOTS_TMPDIR']=str(SOURCE.parents[1]/'build/wb')
    Path(env['XDG_CONFIG_HOME']).mkdir();Path(env['WEBOTS_TMPDIR']).mkdir(exist_ok=True)
    command=['timeout',f'{args.timeout_seconds}s','webots','--batch','--mode=fast',
             '--no-rendering','--minimize','--stdout','--stderr',str(world)]
    log=project/'passive-smoke.log'
    with log.open('w')as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        pid=process.pid
        (project/'owned-process.json').write_text(json.dumps(dict(owned_pgid=pid,command=command,
            environment=dict(DISPLAY=env['DISPLAY'],XAUTHORITY='existing operator Xwayland authority; contents not read/logged')),
            indent=2)+'\n')
        try:
            status=process.wait(timeout=args.timeout_seconds+8)
        except subprocess.TimeoutExpired:
            os.killpg(pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(pid,signal.SIGKILL);process.wait(timeout=3)
            status=124
        finally:
            # Explicitly owned process group only. Never discover/kill by name.
            try:os.killpg(pid,signal.SIGTERM)
            except ProcessLookupError:pass
    contents=log.read_text(errors='replace')
    complete,proof,error=verify_result(project,status,contents)
    result=dict(passive_smoke_complete=complete,simulation_only=True,hardware_used=False,serial_used=False,
                motors_applied=False,exit_status=status,owned_pgid=pid,command=command,replay=proof,error=error,
                controller_banner_observed='PASSIVE_SIMULATION_COMPLETE' in contents,
                scope='passive sampled-device and codec smoke; no control, timing or sensor-calibration acceptance')
    (project/'passive-smoke-summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,sort_keys=True));print(f'Retained evidence: {project}')
    return 0 if complete else 1


if __name__=='__main__':raise SystemExit(main())
