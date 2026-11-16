"""Read-only KR1/SIL oscillation analysis. No Webots, SDK, serial or hardware.

KR rows are sampled at about50Hz, NOT the100Hz vehicle cadence. Speeds are
encoder-derived10ms snapshots, NOT counts. Course straightness is not encoded.
"""
import bisect
import collections
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import re
import statistics as st
import subprocess
from .sil_line import ROOT
from .sil_motor import LagPlant,MotorLineController

FIELDS=('ms','state','module','raw_l','raw_c','position','ask_l','ask_r','v_l','v_r','events','flags')
SOURCE_FILES=('app_program/subsystem3/line_steer.c','app_program/subsystem3/line_steer.h',
    'app_program/subsystem3/line_position.c','app_program/subsystem3/line_position.h',
    'app_program/subsystem2/motor_ctl.c','app_program/subsystem2/motor_ctl.h','app_program/subsystem2/motor_ctl_robot.h',
    'app_program/vehicle/vehicle_log.c','app_program/vehicle/vehicle_log.h')
# Operator image/boot ledger; never infer these from CSV filenames alone.
BOOT_IMAGES={4278455808:'451e562',95335756:'451e562',2115510079:'451e562',
    888116552:'d8adbcc',3901313091:'d8adbcc',1937965271:'f65aaf8',
    2146435199:'39321d9',2938093583:'381b7dc',2455129503:'381b7dc-v350'}
RECENT=('kram_robot_e06667d','kram_robot_2413c4a','kram_robot_b0c3a9a','home_size_7a6cb1c')
OLDER=('mtk_logs/s5_b2a_j5_20261110_usb.txt','mtk_logs/sys_s1a_20261110_usb.txt',
    'mtk_logs/sys_s1b_20261110_usb.txt','mtk_logs/b2b_shadow_F_20261111_usb.txt',
    'school_20261113/t3_force0_gp20_usb.txt','school_20261113/t3b_force0_gp20_usb.txt',
    'school_20261113/p1_force0_gp20_usb.txt','school_20261113/p1_smp0_gp20_usb.txt',
    'school_20261113/ctrl_smp0_gp20_usb.txt','school_20261113/usb_console_before_p1.txt')
CAL=re.compile(r'line L floor/line (\d+)/(\d+) C (\d+)/(\d+)')
KR=re.compile(r'KR (\d+) ([0-9a-fA-F]+)\s*$')


def sha(data):return hashlib.sha256(data).hexdigest()

def decode(payload):
    if len(payload)!=32:raise ValueError('unsupported KR layout; require16bytes')
    w=int.from_bytes(bytes.fromhex(payload),'little')
    def b(o,n,s=False):
        x=(w>>o)&((1<<n)-1);return x-(1<<n)if s and x&(1<<(n-1))else x
    return dict(zip(FIELDS,(b(0,16)*20,b(16,2),b(18,3),b(21,12),b(33,12),b(45,8,True),
        b(53,10,True),b(63,10,True),b(73,10,True),b(83,10,True),b(116,8),b(124,4))))


def parse_log(path,*,last_connect=False):
    text=Path(path).read_text(errors='replace');lines=text.splitlines();first=0
    if last_connect:
        connects=[i for i,l in enumerate(lines)if re.fullmatch(r'===.*\bconnect\s*',l)]
        if connects:first=connects[-1]
    blocks=[];pending={};active=None
    def flush():
        nonlocal active
        if active and active['packed']:blocks.append(active)
        active=None
    for number,line in enumerate(lines[first:],first+1):
        if '[robot] KR1 reset' in line:
            flush();pending=dict(header=line,header_line=number)
        cal=CAL.search(line)
        if cal:pending['dump_endpoints']=list(map(int,cal.groups()))
        m=KR.fullmatch(line)
        if not m:continue
        index=int(m[1]);payload=m[2].lower()
        if index==0 and active:flush()
        if active is None:active=dict(pending,packed=[],sources=[dict(path=str(path),first_line=number)],unsupported=[])
        if len(payload)!=32:raise ValueError(f'{path}:{number}: unsupported KR size {len(payload)}')
        active['packed'].append((index,payload))
    flush();return blocks


def image_for(path):
    s=str(path)
    for d in RECENT:
        if d in s:return d.rsplit('_',1)[-1]
    if any(n in s for n in ('t3_force0','t3b_force0','ctrl_smp0')):return'451e562'
    if any(n in s for n in ('p1_force0','p1_smp0')):return'd8adbcc'
    return None


def inventory(robot_build,main_build):
    runs=[];files=[];byhash={}
    def add(block,path,label=None):
        packed=block['packed'];key=sha(json.dumps(packed,separators=(',',':')).encode())
        if key in byhash:
            byhash[key]['sources']+=block['sources'];return byhash[key]
        block.update(id=label or f'old{len(runs)+1:02}',image=image_for(path),sha256=key)
        block['rows']=[dict(decode(p),index=i)for i,p in packed]
        byhash[key]=block;runs.append(block);return block
    for d in RECENT:
        for p in sorted((Path(robot_build)/d).glob('*.final.log')):
            files.append(dict(path=str(p),sha256=sha(p.read_bytes())))
            for b in parse_log(p,last_connect=True):add(b,p,d+'/'+p.stem)
    for name in OLDER:
        p=Path(main_build)/name;files.append(dict(path=str(p),sha256=sha(p.read_bytes())))
        for b in parse_log(p):add(b,p)
    # MQTT robot/log carries actual packed high-rate rows as well as slow state.
    # Match payload+index to GP20 dumps; never join by wall time or filename alone.
    csvs=[]
    for d in RECENT:csvs+=sorted((Path(robot_build)/d).glob('mqtt_*.csv'))
    csvs += [Path(main_build)/'mtk_logs/robot-mqtt-20261111-163412.csv']
    csvs += [Path(main_build)/f'school_20261113/robot-mqtt-20261113-{t}.csv'for t in ('112618','155730','155932')]
    mqtt=[]
    for p in csvs:
        files.append(dict(path=str(p),sha256=sha(p.read_bytes())))
        groups=collections.defaultdict(dict);state=0;state_times=collections.defaultdict(list)
        with p.open(newline='')as f:
            for r in csv.DictReader(f):
                if r.get('topic')=='robot/state':
                    state+=1
                    if r.get('received_unix_s'):state_times[r.get('boot_session','')].append(float(r['received_unix_s']))
                payload=r.get('kr1_hex','')
                if r.get('topic')=='robot/log'and len(payload)==32 and r.get('index','').isdigit():
                    group=groups[(r.get('boot_session',''),r.get('generation',''))];index=int(r['index'])
                    if index in group and group[index]!=payload:raise ValueError(f'{p}: conflicting packed row {index}')
                    group[index]=payload
        info=dict(path=str(p),state_rows=state,groups=[])
        for (boot,generation),values in groups.items():
            pairs=sorted(values.items());pairset=set(pairs)
            # Explicit split-image exclusions from SESSION_NOTES.md. They
            # can also appear as retained/live rows in mixed MQTT captures.
            excluded_boots={3250257766,1442838703,941374591}
            match=re.match(r'boot:([0-9a-fA-F]+):',boot)
            if match and int(match[1],16)in excluded_boots:
                info['groups'].append(dict(boot=boot,generation=generation,rows=len(pairs),run=None,status='EXCLUDED split DEADLINE image'))
                continue
            matches=[(len(pairset.intersection(b['packed'])),b)for b in runs]
            count,b=max(matches,key=lambda a:a[0])if matches else(0,None)
            if count>=max(5,.5*min(len(pairs),len(b['packed']))):
                b['sources'].append(dict(path=str(p),boot=boot,generation=generation,matched_rows=count))
                label=b['id'];status='matched packed rows; no extra run'
            elif len(pairs)>=100:
                block=dict(packed=pairs,sources=[dict(path=str(p),boot=boot,generation=generation)],
                    header='MQTT packed log only; calibration/image attribution not independently proven',unsupported=[])
                # Do not infer the flashed image from a CSV directory alone.
                b=add(block,p,Path(p).parent.name+'/'+p.stem+'/'+boot+'/'+generation)
                boot_id=int(match[1],16)if match else None
                b['image']=BOOT_IMAGES.get(boot_id);label=b['id']
                b['identity_basis']='SESSION_NOTES boot ledger'if b['image']else'unresolved; not inferred from CSV directory'
                status='additional CSV-only run; documented boot image'if b['image']else'additional CSV-only run; identity unresolved'
            else:label=None;status='insufficient/retained context, not an independent cycle record'
            info['groups'].append(dict(boot=boot,generation=generation,rows=len(pairs),run=label,status=status))
        info['state_cadence_s']={boot:st.median([b-a for a,b in zip(ts,ts[1:])if b>a])for boot,ts in state_times.items()if len(ts)>1}
        mqtt.append(info)
    return runs,files,mqtt


def quantile(a,q):
    a=sorted(a)
    if not a:return None
    x=(len(a)-1)*q;i=int(x);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(x-i)


def normal(row,cal):
    if not cal:return None
    fl,bl,fc,bc=cal
    if bl<=fl or bc<=fc:return None
    return (max(0,min(1,(row['raw_l']-fl)/(bl-fl))),max(0,min(1,(row['raw_c']-fc)/(bc-fc))))


def segments(rows,cal,*,mask_high_both=False):
    result=[];active=[];excluded=collections.Counter()
    for r in rows:
        n=normal(r,cal)
        reason=''
        if r['state']!=1 or r['module']!=1:reason='not running LINE'
        elif min(r['ask_l'],r['ask_r'])<0 or max(r['ask_l'],r['ask_r'])==0:reason='stop/reversing pivot'
        # P7 can legitimately command ONE wheel zero during a large LINE
        # correction. Removing those rows would remove the zigzag itself.
        elif r['flags']&1:reason='packed clamp'
        elif mask_high_both and n and sum(n)>=1.25:reason='diagnostic high-both; dump endpoints NOT run calibration'
        if reason or active and r['ms']-active[-1]['ms']>40:
            if active:result.append(active);active=[]
        if reason:excluded[reason]+=1
        else:active.append(r)
    if active:result.append(active)
    return [s for s in result if s[-1]['ms']-s[0]['ms']>=1000],dict(excluded)


def resample(rows,step=20):
    """Linear interpolation only within an already gap-limited segment."""
    times=[r['ms']for r in rows];out=[]
    for t in range(times[0],times[-1]+1,step):
        k=max(0,bisect.bisect_right(times,t)-1);a=rows[k];b=rows[min(k+1,len(rows)-1)]
        u=(t-a['ms'])/(b['ms']-a['ms'])if b['ms']>a['ms']else 0
        out.append({key:(a[key]+u*(b[key]-a[key])if isinstance(a[key],(float,int))else a[key])for key in a})
        out[-1]['ms']=t
    return out


def correlation(a,b):
    if len(a)<3:return 0.
    ma,mb=st.mean(a),st.mean(b);x=[v-ma for v in a];y=[v-mb for v in b]
    den=math.sqrt(sum(v*v for v in x)*sum(v*v for v in y))
    return sum(i*j for i,j in zip(x,y))/den if den else 0.


def lag(a,b,period_ms):
    # Local signed delay; periodic ambiguity is reported, not hidden.
    limit=min(10,max(1,int(period_ms/40)))
    scores=[]
    for k in range(-limit,limit+1):
        x=a[:len(a)-k]if k>=0 else a[-k:];y=b[k:]if k>=0 else b[:len(b)+k]
        scores.append((correlation(x,y),k*20))
    c,t=max(scores,key=lambda z:z[0]);return dict(ms=t,correlation=c,uncertainty_ms=40,periodic=True)


def cycle_metrics(rows,cal):
    r=resample(rows);p=[v['position']for v in r];u=[v['ask_l']-v['ask_r']for v in r]
    w=[v['v_l']-v['v_r']for v in r];bal=[]
    for v in r:bal.append(v['raw_c']-v['raw_l'])
    # Autocorrelation local maxima, not a forced0.4s answer. Need>=3cycles.
    peaks=[]
    for k in range(6,min(61,len(p)//3)):
        c=correlation(p[:-k],p[k:]);before=correlation(p[:-(k-1)],p[k-1:]);after=correlation(p[:-(k+1)],p[k+1:])
        if c>before and c>=after:peaks.append((c,k*20))
    best=max(peaks,default=(0,None))
    period=best[1]if best[0]>=.3 else None
    amp=lambda a:(quantile(a,.95)-quantile(a,.05))/2
    truth_rate=[]
    if all('body_yaw_deg'in v for v in r):
        for a,b in zip(r,r[1:]):
            delta=(b['body_yaw_deg']-a['body_yaw_deg']+180)%360-180
            truth_rate.append(delta*1000/(b['ms']-a['ms']))
    normals=[normal(v,cal)for v in r];good=[n for n in normals if n]
    return dict(start_ms=r[0]['ms'],end_ms=r[-1]['ms'],samples=len(r),period_ms=period,period_correlation=best[0],
        yaw_amplitude_deg_s=amp(w)*180/(math.pi*111),yaw_mean_deg_s=st.mean(w)*-180/(math.pi*111),
        body_yaw_rate_amplitude_deg_s=amp(truth_rate)if truth_rate else None,
        position_amplitude=amp(p),command_difference_amplitude_mm_s=amp(u),wheel_difference_amplitude_mm_s=amp(w),
        raw_balance_amplitude_adc=amp(bal),raw_balance_mean_adc=st.mean(bal),
        dump_normalized_balance_amplitude=amp([n[1]-n[0]for n in good])if good else None,
        single_sensor_fraction=sum((a>=.05)!=(b>=.05)for a,b in good)/len(good)if good else None,
        raw_L_amplitude_adc=amp([v['raw_l']for v in r]),raw_C_amplitude_adc=amp([v['raw_c']for v in r]),
        sensor_command_lag=lag(bal,u,period)if period else None,
        command_wheel_lag=lag(u,w,period)if period else None)


class Native:
    def __init__(self,output):
        p=Path(output);p.mkdir(parents=True,exist_ok=True);self.path=p/'libline_metrics.so'
        sources=('sim/webots/controllers/sil_line/line_metrics_adapter.c',
            'app_program/subsystem3/line_position.c','app_program/subsystem3/line_steer.c','app_program/subsystem2/motor_ctl.c')
        subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC','-Wall','-Wextra','-Wconversion','-Wshadow','-Werror',
            '-I',str(ROOT),*(str(ROOT/s)for s in sources),'-o',str(self.path)],check=True,capture_output=True)
        self.c=MotorLineController(self.path);self.lib=self.c.lib;ptr=ctypes.POINTER(ctypes.c_int32)
        self.lib.metrics_steer.argtypes=[ctypes.c_int8,ctypes.c_int8,ptr];self.lib.metrics_steer.restype=None
        self.lib.metrics_motor.argtypes=[ctypes.c_uint32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int16,ctypes.c_int16,ptr];self.lib.metrics_motor.restype=None
        self.possible={}
        for position in range(-100,101):
            values=set()
            for previous in range(-100,101):
                o=(ctypes.c_int32*2)();self.lib.metrics_steer(previous,position,o);values.add(tuple(o))
            self.possible[position]=values
        self.provenance=dict(sources_sha256={s:sha((ROOT/s).read_bytes())for s in (*sources,*SOURCE_FILES)},
            library_sha256=sha(self.path.read_bytes()),config=self.c.metadata(),image_matches={})
        def line_config(text):
            m=re.search(r'static const lt_config_t LINE_CFG = \{.*?\n\};',text,re.S)
            return re.sub(r'\s+','',m[0])if m else None
        cfg='app_program/vehicle/vehicle_robot_config.h'
        root_cfg=line_config((ROOT/cfg).read_text());self.provenance['sources_sha256'][cfg]=sha((ROOT/cfg).read_bytes())
        self.provenance['image_config_sha256']={}
        for image in ('451e562','d8adbcc','e06667d','2413c4a','b0c3a9a','7a6cb1c','f65aaf8','39321d9','381b7dc','381b7dc-v350'):
            ref='381b7dc'if image=='381b7dc-v350'else image
            try:
                header=subprocess.check_output(['git','show',f'{ref}:{cfg}'],cwd=ROOT,stderr=subprocess.PIPE)
                self.provenance['image_config_sha256'][image]=sha(header)
                self.provenance['image_matches'][image]=line_config(header.decode())==root_cfg and all(
                    subprocess.check_output(['git','show',f'{ref}:{s}'],cwd=ROOT,stderr=subprocess.PIPE)==(ROOT/s).read_bytes()for s in SOURCE_FILES)
            except subprocess.CalledProcessError:
                self.provenance['image_matches'][image]=False # No absent-history proof.
        self.provenance['v350_scope']='381b7dc source; operator v350 detour settings. P7 unchanged; not a claim of whole-image binary identity.'

    def steering(self,rows):
        errors=[];coarse=[]
        for r in rows:
            choices=self.possible.get(r['position'],set());errors.append(min((abs(a-r['ask_l'])+abs(b-r['ask_r']))/2 for a,b in choices)if choices else 999)
        for a,b in zip(rows,rows[1:]):
            o=(ctypes.c_int32*2)();self.lib.metrics_steer(a['position'],b['position'],o)
            coarse.append((abs(o[0]-b['ask_l'])+abs(o[1]-b['ask_r']))/2)
        return dict(rows=len(errors),unknown_previous_position_envelope_exact_fraction=sum(e==0 for e in errors)/len(errors),
            envelope_p95_error_mm_s=quantile(errors,.95),coarse20ms_replay_mae_mm_s=st.mean(coarse)if coarse else None,
            limitation='KR omits10ms previous state; envelope compatibility is not exact state replay')

    def motor(self,rows,linear=False):
        rows=resample(rows);times=[r['ms']-rows[0]['ms']+1000 for r in rows]
        self.lib.sil_reset();plant=LagPlant(bidirectional=True);angles=[0.,0.];duties=[0.,0.];out=[];last_v=(0,0)
        for ms in range(1,times[-1]+1):
            v=plant.advance(*duties)
            for j in (0,1):angles[j]+=v[j]*.001
            k=max(0,bisect.bisect_right(times,ms)-1);r=rows[k]
            target=[r['ask_l'],r['ask_r']]
            if linear and k+1<len(times)and ms>=times[k]:
                f=(ms-times[k])/(times[k+1]-times[k]);target=[target[j]+f*(rows[k+1]['ask_l'if j==0 else'ask_r']-target[j])for j in (0,1)]
            if ms%10==0:
                counts=[round(a*2535/(2*math.pi))for a in angles];o=(ctypes.c_int32*5)()
                self.lib.metrics_motor(ms,*counts,*map(round,target),o)
                if o[2]:raise ValueError('native motor replay fault')
                duties=[o[0]/10,o[1]/10];last_v=(o[3],o[4])
            if ms>=1000 and ms%20==0:
                out.append(dict(r,ms=ms-1000+rows[0]['ms'],v_l=last_v[0],v_r=last_v[1]))
        real_u=[r['ask_l']-r['ask_r']for r in rows];model_w=[r['v_l']-r['v_r']for r in out]
        real_w=[r['v_l']-r['v_r']for r in rows];amp=lambda x:(quantile(x,.95)-quantile(x,.05))/2
        cm=cycle_metrics(rows,None);period=cm['period_ms']
        return dict(reconstruction='linear'if linear else'held',predicted_yaw_amplitude_deg_s=amp(model_w)*180/(math.pi*111),
            real_yaw_amplitude_deg_s=amp(real_w)*180/(math.pi*111),
            predicted_command_wheel_lag=lag(real_u,model_w,period)if period else None,
            real_command_wheel_lag=lag(real_u,real_w,period)if period else None,
            limitation='1D nativeMC+DM1+80ms surrogate; no ODE/contact physics; omitted commands reconstructed; initial state warmed1s')
