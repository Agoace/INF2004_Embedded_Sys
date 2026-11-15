#!/usr/bin/env python3
"""Single untuned curve, authoritative optics and actual simulated spin endpoints."""
import argparse
import importlib.util
import json
from pathlib import Path
from time import monotonic
from sil_line_smoke import prepare_project,launch_project
from pil.sil_curve import curve_course


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--spin-calibration',type=Path,required=True)
    parser.add_argument('--s-curve',action='store_true')
    parser.add_argument('--timeout-seconds',type=int,default=1200);args=parser.parse_args()
    if not 20<=args.timeout_seconds<=1200:parser.error('timeout20..1200s; extended runs need explicit approval')
    spin=json.loads(args.spin_calibration.read_text())
    if not spin.get('complete')or spin.get('optical_sampler')!='full':parser.error('completed full-scene aligned spin required')
    if args.s_curve:
        from pil.sil_s_curve import s_curve_course
        course=s_curve_course()
    else:course=curve_course()
    project,world=prepare_project(args.output,line_search=True,recovery=True,spin_calibration=spin,
        course_override=course,optical_sampler='full')
    start=monotonic();status,pid,command,contents=launch_project(project,world,args.timeout_seconds,label='sil-curve')
    wall_seconds=monotonic()-start;summary=None;error=None;proof=False
    try:
        summary=json.loads((project/'sil-result/summary.json').read_text())
        spec=importlib.util.spec_from_file_location('curve_replay',project/'controllers/sil_line/sil_line.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        proof=module.replay_export(project)
    except (ValueError,OSError)as exception:error=str(exception)
    result=dict(experiment_complete=status==0 and proof and 'ERROR:'not in contents and 'WARNING:'not in contents,
        exit_status=status,owned_pgid=pid,wall_seconds=wall_seconds,independent_replay=proof,
        curve=summary['curve']if summary else None,error=(summary['error']if summary else None)or error,
        calibration=summary['calibration']if summary else None,samples_sha256=summary['samples_sha256']if summary else None)
    (project/'curve-check.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,sort_keys=True));print('Evidence:',project)
    return 0 if result['experiment_complete']else 1


if __name__=='__main__':raise SystemExit(main())
