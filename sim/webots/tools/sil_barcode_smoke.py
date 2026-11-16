#!/usr/bin/env python3
"""One explicit observe-only *A* physics run, stopping at native confirmation."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from time import monotonic

SOURCE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(SOURCE));sys.path.insert(0,str(SOURCE/'tools'))
from driving_smoke import launch_project
from sil_line_smoke import prepare_project
from pil.sil_junction_barcode import junction_barcode_course,FAMILIES
from pil.s3_sil_barcode import barcode_sheet


def speed_check(rows,family):
    """Post-run observation only; never changes a controller input or command."""
    sheet=barcode_sheet(sheet_start_mm=970.5,line_centre_y_mm=157.5,surface_z_mm=10)
    from pil.s3_sil_barcode import load_sil_barcode_profile
    profile=load_sil_barcode_profile(sigma_along_mm=family)
    result={}
    for name,boundary,support in(('paper_support',sheet.sheet_bounds_mm[0],True),
            ('paper_centre',sheet.sheet_bounds_mm[0],False),('first_ink_centre',sheet.ink_bounds_mm[0],False)):
        for i,row in enumerate(rows):
            yaw=row[15];x=row[12]*1000+80*math.cos(yaw)-77*math.sin(yaw)
            extent=4*(profile.sigma_x_mm*abs(math.sin(yaw))+profile.sigma_y_mm*abs(math.cos(yaw)))if support else 0
            if x+extent<boundary:continue
            if i<10:raise ValueError('insufficient speed history')
            mean=[32.5*(row[j]-rows[i-10][j])/.010 for j in(16,17)]
            instant=[32.5*(row[j]-rows[i-1][j])/.001 for j in(16,17)]
            result[name]=dict(sim_ms=row[0],gp28_x_mm=x,left_mm_s=instant[0],right_mm_s=instant[1],
                trailing10ms_mm_s=mean,within5pct146=all(abs(v-146)<=.05*146 for v in mean))
            break
        if name not in result:raise ValueError('boundary not reached: '+name)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--spin-calibration',type=Path,required=True)
    parser.add_argument('--sigma',type=int,choices=FAMILIES,required=True)
    parser.add_argument('--lead-mm',type=int,choices=(75,100,200,300),default=100)
    parser.add_argument('--letter',choices=('A','F'),default='A');parser.add_argument('--zigzag',action='store_true')
    parser.add_argument('--timeout-seconds',type=int,choices=(1200,1500),default=1200);args=parser.parse_args()
    if args.letter=='F' and(not args.zigzag or args.sigma!=1 or args.lead_mm!=75 or args.timeout_seconds!=1200):
        parser.error('F task approved only zigzag/along1/lead75/1200s')
    if args.zigzag and args.letter!='F':parser.error('zigzag task uses F only')
    if args.timeout_seconds==1500 and(args.sigma!=8 or args.lead_mm!=75):parser.error('1500s approved ONLY for along8/lead75 final task run')
    if os.environ.get('DISPLAY')!=':0'or not os.environ.get('XAUTHORITY'):parser.error('approved existing DISPLAY=:0/XAUTHORITY required')
    if args.output.exists():parser.error('output must be new')
    spin=json.loads(args.spin_calibration.read_text())
    if not spin.get('complete')or not spin.get('calibration_rows')or not spin.get('barcode_calibration_rows'):parser.error('complete actual spin with GP28 required')
    args.output.mkdir(parents=True);results=[]
    for family in(args.sigma,):
        project,world=prepare_project(args.output/f'along{family}',line_search=True,line_profile='B',
            spin_calibration=spin,course_override=junction_barcode_course(lead_mm=args.lead_mm,letter=args.letter,
                offset_mm=5 if args.zigzag else 0,yaw_deg=-5 if args.zigzag else 0),optical_sampler='full',
            barcode=True,sigma_along_mm=family,duration_ms=5300,stop_after_junction=True,window_revision='52244f2')
        begin=monotonic()
        status,pid,command,contents=launch_project(project,world,args.timeout_seconds,label='sil-barcode')
        wall_seconds=monotonic()-begin
        summary=None;error=None;proof=False;speeds=None
        try:
            summary=json.loads((project/'sil-result/summary.json').read_text())
            spec=importlib.util.spec_from_file_location('barcode_replay',project/'controllers/sil_line/sil_line.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);proof=module.replay_export(project)
            path=project/'sil-result/samples.csv'
            if hashlib.sha256(path.read_bytes()).hexdigest()!=summary['samples_sha256']:raise ValueError('sample hash')
            with path.open(newline='')as stream:reader=csv.reader(stream);next(reader);rows=[tuple(float(v)for v in r)for r in reader]
            speeds=speed_check(rows,family)
        except(OSError,ValueError)as exc:error=str(exc)
        complete=bool(status==0 and summary and summary['status']=='SIL_LINE_COMPLETE'and proof and
            summary['recorded_input_c_replay']and speeds and all(v['within5pct146']for v in speeds.values())and
            'ERROR:'not in contents and 'WARNING:'not in contents)
        result=dict(complete=complete,along_sigma_mm=family,timeout_seconds=args.timeout_seconds,max_simulated_ms=5300,
            wall_seconds=wall_seconds,lead_mm=args.lead_mm,stop_after_junction=True,
            independent_replay=proof,exit_status=status,owned_pgid=pid,command=command,summary=summary,
            speed_at_boundaries=speeds,error=error,expected_letter=args.letter,expected_map='LEFT'if args.letter=='A'else'UNMAPPED',
            zigzag_start=dict(offset_mm=5,yaw_deg=-5,source='SIL_SEARCH_RESULTS_20261115.md;gridB/offset+5-yaw-5')if args.zigzag else None,
            admission='OBSERVE_ONLY_SHADOW_UNCHANGED',physical_gp28_edge_gate='FAIL_RETAINED')
        (project/'barcode-smoke-summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');results.append(result)
        print(json.dumps(dict(along_sigma_mm=family,complete=complete,speeds=speeds,
            barcode=summary.get('barcode')if summary else None,error=error,wall_seconds=wall_seconds),sort_keys=True),flush=True)
    (args.output/'barcode-results.json').write_text(json.dumps(results,indent=2,sort_keys=True)+'\n')
    return 0 if all(r['complete']for r in results)else 1


if __name__=='__main__':raise SystemExit(main())
