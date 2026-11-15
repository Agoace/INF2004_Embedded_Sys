#!/usr/bin/env python3
"""One aligned simulated calibration spin; output rows for independent start-pose trials."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from sil_line_smoke import prepare_project,launch_project,SOURCE


def run_spin(output,*,line_profile='B'):
    project,world=prepare_project(output,line_search=True,line_profile=line_profile)
    world.write_text(world.read_text().replace('controller "sil_line"','controller "spin_calibration"'))
    destination=project/'controllers/spin_calibration';destination.mkdir()
    shutil.copyfile(SOURCE/'controllers/sil_line/spin_calibration.py',destination/'spin_calibration.py')
    status,pid,command,contents=launch_project(project,world,90,label='sil-spin')
    result=json.loads((project/'spin-result/calibration.json').read_text())
    profiles=json.loads((project/'sil-manifest.json').read_text())['profiles']
    result['profiles_sha256']=hashlib.sha256(json.dumps(profiles,sort_keys=True).encode()).hexdigest()
    result['sampled_rows_sha256']=hashlib.sha256(json.dumps(result['calibration_rows'],separators=(',',':')).encode()).hexdigest()
    result.update(exit_status=status,owned_pgid=pid,clean_log='ERROR:'not in contents and 'WARNING:'not in contents)
    (project/'spin-check.json').write_text(json.dumps(result,indent=2)+'\n')
    (project/'spin-result/calibration.json').write_text(json.dumps(result,indent=2)+'\n')
    if status!=0 or not result['complete']or not result['clean_log']:raise ValueError('spin failed: '+str(result.get('error')))
    return project,result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--line-profile',choices=('A','B'),default='B')
    args=parser.parse_args()
    project,result=run_spin(args.output,line_profile=args.line_profile)
    print(json.dumps({k:v for k,v in result.items()if k!='calibration_rows'},sort_keys=True));print('Evidence:',project)
    return 0


if __name__=='__main__':raise SystemExit(main())
