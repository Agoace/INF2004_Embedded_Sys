#!/usr/bin/env python3
"""Approved host/Webots step-response fit check; no hardware."""
import argparse
import json
from pathlib import Path
import shutil
from sil_line_smoke import prepare_project,launch_project,SOURCE
from pil.sil_motor import step_response


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    project,world=prepare_project(args.output,motor_fidelity=True)
    world.write_text(world.read_text().replace('controller "sil_line"','controller "motor_step"'))
    destination=project/'controllers/motor_step';destination.mkdir()
    shutil.copyfile(SOURCE/'controllers/sil_line/motor_step.py',destination/'motor_step.py')
    result=step_response(project/'libraries/libsil_line.so')
    (project/'host-step.json').write_text(json.dumps(result,indent=2)+'\n')
    status,pid,command,contents=launch_project(project,world,90,label='motor-step')
    summary=json.loads((project/'motor-step.json').read_text())
    summary.update(exit_status=status,owned_pgid=pid,clean_log='ERROR:'not in contents and 'WARNING:'not in contents)
    (project/'motor-bench-check.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,sort_keys=True));print('Evidence:',project)
    return 0 if summary['passed']and status==0 and summary['clean_log']else 1


if __name__=='__main__':raise SystemExit(main())
