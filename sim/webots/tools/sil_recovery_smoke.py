#!/usr/bin/env python3
"""Twenty-one approved straight-strip start poses; no controller tuning."""
import argparse
import csv
import importlib.util
import json
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from sil_line_smoke import prepare_project,launch_project,SOURCE


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--motor-fidelity',action='store_true')
    parser.add_argument('--line-search',action='store_true')
    args=parser.parse_args();output=args.output.resolve()
    if not output.is_relative_to(SOURCE.parents[1]/'build'):parser.error('output below owned build required')
    if os.environ.get('DISPLAY')!=':0' or not os.environ.get('XAUTHORITY'):parser.error('approved DISPLAY=:0/XAUTHORITY required')
    output.mkdir(parents=True,exist_ok=False);results=[]
    for offset in(0,5,-5,10,-10,15,-15):
        for yaw in(0,5,-5):
            name=f'offset{offset:+d}-yaw{yaw:+d}'
            project,world=prepare_project(output/name,offset_mm=offset,yaw_deg=yaw,recovery=True,motor_fidelity=args.motor_fidelity,line_search=args.line_search)
            status,pid,command,contents=launch_project(project,world,90,label='sil-recovery')
            summary=None;error=None;proof=False
            try:
                summary=json.loads((project/'sil-result/summary.json').read_text())
                spec=importlib.util.spec_from_file_location('recovery_replay',project/'controllers/sil_line/sil_line.py')
                module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                proof=module.replay_export(project)
            except (ValueError,OSError)as exception:error=str(exception)
            result=dict(offset_mm=offset,yaw_deg=yaw,owned_pgid=pid,exit_status=status,
                experiment_complete=bool(status==0 and proof and 'ERROR:'not in contents and 'WARNING:'not in contents),
                independent_replay=proof,error=(summary['error']if summary else None)or error,
                replay_error=error,samples=summary['samples']if summary else 0,
                recovery=summary['recovery']if summary else None,
                samples_sha256=summary['samples_sha256']if summary else None)
            (project/'recovery-check.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
            results.append(result);print(json.dumps(result,sort_keys=True),flush=True)
    (output/'recovery-grid.json').write_text(json.dumps(results,indent=2,sort_keys=True)+'\n')
    flat=[dict(offset_mm=r['offset_mm'],yaw_deg=r['yaw_deg'],experiment_complete=r['experiment_complete'],**(r['recovery']or{}))for r in results]
    with (output/'recovery-grid.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
    return 0 if all(r['experiment_complete']for r in results)else 1


if __name__=='__main__':raise SystemExit(main())
