#!/usr/bin/env python3
"""Offline audit: two retained P85 recordings and six base/order experiments."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.sil_line_base_timing import WINDOWS,analyze_window

CASES=('B120-delay20','B120-delay30','B100-delay20','B100-delay30','same-tick-delay20','same-tick-delay30')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def read_rows(path):
    with path.open()as f:reader=csv.reader(f);next(reader);return [list(map(float,r))for r in reader]


def audit(path,label,*,historical=False):
    config=json.loads((path/'sil-manifest.json').read_text());manifest=json.loads((path/'manifest.json').read_text())
    result=json.loads((path/'taskA-result.json').read_text());samples=path/'sil-result/samples.csv';rows=read_rows(samples)
    if len(rows)!=10000 or any(r[0]!=i+1 for i,r in enumerate(rows)):raise ValueError('sample timeline/length')
    mismatches=[p for p,h in manifest['artifacts_sha256'].items()if sha(path/p)!=h]
    optical=read_rows(path/'sil-result/taskA-optical-inputs.csv');delay=config['sensor_transport_delay_ms']
    delay_bad=[i for i,r in enumerate(optical)if r[0]!=i+1 or r[3:5]!=optical[max(0,i-delay)][1:3]or rows[i][1:3]!=r[3:5]]
    if len(optical)!=len(rows):raise ValueError('optical recording length')
    timing_report=None
    if not historical:
        timing=read_rows(path/'sil-result/motor-timing.csv');same=config['same_tick_apply'];lookup={int(r[0]):r for r in rows};bad=[];ages=[]
        if len(timing)!=1000:raise ValueError('timing record length')
        for i,(ms,stamp,left,right,mode,active,age)in enumerate(timing):
            if ms!=(i+1)*10 or mode!=int(same):bad.append(i);continue
            if active:
                ages.append(age);target=lookup.get(int(stamp))
                expected=[math.trunc(max(-460,min(460,target[j]))*12414*10*256/1000000)for j in (24,25)]if target else None
                if age!=(0 if same else 10)or expected!=[left,right]or stamp!=ms-age:bad.append(i)
        timing_report=dict(rows=len(timing),active_rows=len(ages),age_ms_values=sorted(set(ages)),mismatch_indices=bad)
    try:os.killpg(result['owned_pgid'],0);gone=False
    except ProcessLookupError:gone=True
    files={str(p.relative_to(path)):sha(p)for p in (path/'sil-manifest.json',path/'manifest.json',path/'taskA-result.json',samples,path/'sil-result/taskA-optical-inputs.csv')}
    if not historical:files['sil-result/motor-timing.csv']=sha(path/'sil-result/motor-timing.csv')
    complete=result['complete']and not mismatches and not delay_bad and gone and (historical or not timing_report['mismatch_indices'])
    return dict(label=label,path=str(path),historical=historical,complete=complete,wall_seconds=result['wall_seconds'],
        owned_pgid=result['owned_pgid'],owned_pgid_gone=gone,source_commit=config.get('candidate_source_commit'),
        base_mm_s=result['summary']['configuration']['steer'][0],configuration=result['summary']['configuration'],
        same_tick_apply=config.get('same_tick_apply',False),transport_ms=delay,searches=result['summary']['recovery']['searches'],
        calibration=result['summary']['calibration'],hashes=files,artifact_mismatches=mismatches,delay_mismatch_indices=delay_bad,
        motor_timing=timing_report,windows=[analyze_window(rows,a,b,manifest['robot_parameters']['track_mm']['value'])for a,b in WINDOWS],
        whole_10s=analyze_window(rows,0,10000,manifest['robot_parameters']['track_mm']['value']))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--sil-build',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('fresh supplemental output required')
    historical=(('existing-P85-delay20','taskA-excited-20261116T150638Z/offset10-yaw-10-delay20'),
        ('existing-P85-delay30','taskA-candidates-20261116T151447Z/P85-delay30'))
    runs=[audit(a.sil_build/path,name,historical=True)for name,path in historical]
    runs += [audit(a.root/name,name)for name in CASES]
    reference_config=json.loads((Path(runs[0]['path'])/'sil-manifest.json').read_text())
    reference_robot=json.loads((Path(runs[0]['path'])/'manifest.json').read_text())['robot_parameters']
    condition_checks={}
    for r in runs:
        directory=Path(r['path']);config=json.loads((directory/'sil-manifest.json').read_text())
        robot=json.loads((directory/'manifest.json').read_text())['robot_parameters']
        condition_checks[r['label']]=dict(robot_parameters=robot==reference_robot,
            profiles=config['profiles']==reference_config['profiles'],mounts=config['mounts']==reference_config['mounts'],
            calibration_rows=config['calibration_rows']==reference_config['calibration_rows'])
    report=dict(scope='Offline retained-data audit; no launches, hardware or serial',condition_checks=condition_checks,
        method='0-3s is samples1..3000 inclusive; each post2s window includes both endpoints; position thresholds<45,45..52,>52; all/LINE and1ms/20ms denominators separate',
        complete=all(r['complete']for r in runs)and all(all(c.values())for c in condition_checks.values()),runs=runs)
    a.output.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n');print(json.dumps(report,indent=2,sort_keys=True));return 0 if report['complete']else 1

if __name__=='__main__':raise SystemExit(main())
