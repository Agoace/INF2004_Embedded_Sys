#!/usr/bin/env python3
"""Analyze retained KR/SIL recordings; no Webots/hardware execution."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics as st
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pil.sil_line_oscillation import inventory,segments,cycle_metrics,Native,sha


def windows(parts):
    out=[]
    for part in parts:
        # Same fixed window/skip for all runs; not selected for low yaw.
        for start in range(part[0]['ms']+500,part[-1]['ms']-1999,1000):
            r=[v for v in part if start<=v['ms']<=start+2000]
            if r and r[-1]['ms']-r[0]['ms']>=1960:out.append(r)
    return out


def summarize(run,native):
    cal=run.get('dump_endpoints');parts,excluded=segments(run['rows'],cal)
    win=windows(parts);metrics=[cycle_metrics(r,cal)for r in win]
    # Detection is reported explicitly, including quiet windows; no forced period.
    detected=[(i,m)for i,m in enumerate(metrics)if m['period_ms']and m['position_amplitude']>=5]
    aggregate={}
    for name in ('period_ms','yaw_amplitude_deg_s','position_amplitude','raw_balance_amplitude_adc',
                 'command_difference_amplitude_mm_s','single_sensor_fraction'):
        a=[m[name]for _,m in detected if m.get(name)is not None]
        aggregate[name]=st.median(a)if a else None
    for key in ('sensor_command_lag','command_wheel_lag'):
        a=[m[key]['ms']for _,m in detected if m.get(key)]
        aggregate[key+'_ms']=st.median(a)if a else None
    run.update(segments=[[p[0]['ms'],p[-1]['ms']]for p in parts],excluded_rows=excluded,
        windows=metrics,oscillatory_windows=len(detected),all_windows=len(metrics),aggregate=aggregate,
        sample_intervals_ms=sorted(set(b['ms']-a['ms']for a,b in zip(run['rows'],run['rows'][1:]))),
        straightness='operator home straight prefix; high-both later sections NOT independently located'if run['id'].startswith('home_')else'LINE candidates; tile straightness NOT independently encoded')
    if run['id'].startswith('home_'):
        # Fixed prefix: exclude late high-both/held-position phase rather than
        # treating the full LINE module interval as a surveyed straight.
        prefix=[v for v in run['rows']if 500<=v['ms']<=2500]
        run['primary_straight_prefix']=cycle_metrics(prefix,cal)
        run['primary_steering']=native.steering(prefix)
        run['primary_motor_comparison']=[native.motor(prefix,linear=v)for v in (False,True)]
    # Quiet SIL must still have amplitudes reported, not just None from the
    # cycle detector. Keep this separate from oscillatory-window statistics.
    run['all_window_medians']={key:st.median([m[key]for m in metrics if m[key]is not None])if any(m[key]is not None for m in metrics)else None for key in
        ('yaw_amplitude_deg_s','body_yaw_rate_amplitude_deg_s','position_amplitude','raw_balance_amplitude_adc','command_difference_amplitude_mm_s')}
    if detected and run['image']and native.provenance['image_matches'].get(run['image']):
        i,m=detected[0];r=win[i]
        run['native_steering']=native.steering(r)
        run['motor_comparison']=[native.motor(r,linear=v)for v in (False,True)]
        run['motor_window']=[r[0]['ms'],r[-1]['ms']]
    else:run['replay_limit']='no coherent cycle or no independently attributed image; no image-specific replay claim'
    # Diagnostic mask sensitivity only: dump-time adaptive ends are NOT spin calibration.
    masked,_=segments(run['rows'],cal,mask_high_both=True)
    run['high_both_mask_windows']=len(windows(masked))if cal else None
    run.pop('rows');run.pop('packed',None)
    return run


def sil_runs(base):
    cases=(('SIL full A prefix','sil-bc-A-20261116T074205Z/along1',1.),
           ('SIL aligned B','sil-search-B-20261115T130824Z/offset+0-yaw+0',1.),
           ('SIL F settling','sil-F-zigzag-20261116T122633Z/along1',.6716))
    out=[]
    for label,name,mass in cases:
        path=Path(base)/name
        with(path/'sil-result/samples.csv').open()as f:reader=csv.reader(f);next(reader);values=[list(map(float,r))for r in reader]
        lookup={int(r[0]):r for r in values};rows=[]
        for r in values:
            ms=int(r[0])
            if ms%20:continue
            before=lookup.get(ms-10)
            if before is None:continue
            native=len(r)>35
            if not native:raise ValueError('native speed/setpoint evidence required')
            v=[int((r[j]-before[j])*1000000/(12414*10))for j in (22,23)]
            rows.append(dict(ms=ms,state=int(r[28]),module=int(r[27])if not r[6]else 0,
                raw_l=r[1],raw_c=r[2],position=r[3],ask_l=r[24],ask_r=r[25],v_l=v[0],v_r=v[1],events=0,flags=0,
                body_yaw_deg=r[15]*180/math.pi))
        out.append(dict(id=label,image=None,rows=rows,sources=[dict(path=str(path/'sil-result/samples.csv'),sha256=sha((path/'sil-result/samples.csv').read_bytes()))],
            mass_kg=mass,scope='existing physics only; no new Webots; native encoder10ms speed reconstructed like KR'))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-build',type=Path,required=True);p.add_argument('--main-build',type=Path,required=True)
    p.add_argument('--sil-build',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('fresh output required')
    a.output.mkdir(parents=True);native=Native(a.output/'native')
    runs,files,mqtt=inventory(a.robot_build,a.main_build)
    results=[summarize(r,native)for r in runs]
    sil=[summarize(r,native)for r in sil_runs(a.sil_build)]
    report=dict(scope='retained-data analysis and nativeC/1D surrogate replay only; no physics or hardware run',
        cadence='KR50Hz nominal;20ms floor stamps; speeds10ms count-derived snapshots; no per-row counts/duty/ages',
        method='2s windows at1s stride after500ms per contiguous nonnegative-wheel LINE segment; gaps>40ms split; period autocorrelation .12.. .66s, >=.3 coherence and >=5 position amplitude; detected/all counts explicit; school straightness not identifiable',
        normalization='raw C-L is primary. Dump-time adaptive endpoints only diagnostic; never called run calibration',
        native=native.provenance,sources=files,mqtt_inventory=mqtt,runs=results,sil=sil)
    (a.output/'analysis.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print('RUN | cycles/windows | period | yaw amp | pos amp | raw C-L amp | sensor-command/command-wheel lag')
    for r in (*results,*sil):
        m=r['aggregate'];print(r['id'],r['oscillatory_windows'],r['all_windows'],m)
        if r.get('motor_comparison'):print(' MOTOR',r['motor_window'],r['native_steering'],r['motor_comparison'])
    return 0

if __name__=='__main__':raise SystemExit(main())
