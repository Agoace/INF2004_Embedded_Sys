#!/usr/bin/env python3
"""Offline task A audit. Does not launch Webots or access hardware."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.sil_line_oscillation import quantile,lag
from pil.sil_excited_line import window_metrics


def read_rows(path):
    with path.open()as f:reader=csv.reader(f);next(reader);return [list(map(float,r))for r in reader]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('fresh supplemental output required')
    runs=[]
    for name in ('offset10-yaw-10','offset15-yaw0','offset10-yaw-10-delay20'):
        directory=a.root/name;r=json.loads((directory/'taskA-result.json').read_text());config=json.loads((directory/'sil-manifest.json').read_text())
        manifest=json.loads((directory/'manifest.json').read_text());artifact_mismatches=[]
        for path,expected in manifest['artifacts_sha256'].items():
            if hashlib.sha256((directory/path).read_bytes()).hexdigest()!=expected:artifact_mismatches.append(path)
        rows=read_rows(directory/'sil-result/samples.csv');optical=read_rows(directory/'sil-result/taskA-optical-inputs.csv')
        delay=config['sensor_transport_delay_ms'];track=manifest['robot_parameters']['track_mm']['value']
        mismatches=[]
        for i,row in enumerate(optical):
            expected=optical[max(0,i-delay)][1:3]
            if row[3:5]!=expected or row[0]!=i+1 or rows[i][1:3]!=row[3:5]:mismatches.append(i)
        windows=[];lookup={int(v[0]):v for v in rows}
        for start in (2000,4000,6000,8000):
            w=window_metrics(rows,start,start+2000);period=w['period_ms'];commands=[];wheel=[];fresh=[]
            for ms in range(start,start+2001,20):
                row=lookup[ms];before=lookup[ms-10]
                v=[math.trunc((row[j]-before[j])*1000000/(12414*10))for j in (22,23)]
                commands.append(row[24]-row[25]);wheel.append(v[0]-v[1]);fresh.append(optical[ms-1][2]-optical[ms-1][1])
            w['KR_like_yaw_amplitude_deg_s']=(quantile(wheel,.95)-quantile(wheel,.05))/2*180/(math.pi*track)
            w['fresh_sensor_command_lag']=lag(fresh,commands,period)if period else None
            w['command_wheel_lag']=lag(commands,wheel,period)if period else None
            windows.append(w)
        try:os.killpg(r['owned_pgid'],0);gone=False
        except ProcessLookupError:gone=True
        files={str(f.relative_to(directory)):hashlib.sha256(f.read_bytes()).hexdigest()for f in
            (directory/'taskA-result.json',directory/'manifest.json',directory/'sil-manifest.json',directory/'sil-result/samples.csv',directory/'sil-result/taskA-optical-inputs.csv')}
        runs.append(dict(case=name,complete=r['complete'],wall_seconds=r['wall_seconds'],owned_pgid=r['owned_pgid'],owned_pgid_gone=gone,
            hashes=files,artifact_mismatches=artifact_mismatches,optical_delay_mismatch_indices=mismatches,
            measured_geometry={key:manifest['robot_parameters'][key]for key in ('track_mm','line_forward_mm','line_left_mm','line_clearance_mm','barcode_forward_mm','barcode_left_mm','total_mass_kg')},
            windows=windows,searches=r['summary']['recovery']['searches'],found_by_search=r['summary']['recovery']['found_by_search']))
    report=dict(scope='Offline supplements only; original launch/result records unchanged',runs=runs,
        complete=all(r['complete']and r['owned_pgid_gone']and not r['artifact_mismatches']and not r['optical_delay_mismatch_indices']for r in runs))
    a.output.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n');print(json.dumps(report,indent=2,sort_keys=True));return 0 if report['complete']else 1

if __name__=='__main__':raise SystemExit(main())
