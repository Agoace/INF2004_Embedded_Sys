"""Unchanged pure robot C logic and forward measured-row plant mapping."""
import bisect
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[3]
C_SOURCES=('sim/webots/controllers/sil_line/line_adapter.c',
           'app_program/subsystem3/line_position.c','app_program/subsystem3/line_steer.c')
DM1_SOURCE='docs/archive/s2-motion/DUTY_MAP.md, DM1 2026-11-04 boot5d51, ALL forward rows5..100%'
DM1_FORWARD=((5,0,0),(6,0,0),(7,0,0),(8,24,24),(9,34,33),(10,43,42),(11,52,51),
 (12,60,59),(14,78,77),(16,96,95),(18,114,114),(20,129,129),(25,168,171),
 (30,211,214),(35,253,257),(40,296,299),(45,332,334),(50,376,378),
 (60,464,463),(70,561,554),(80,654,637),(90,730,706),(100,807,781))
CONFIG={'base_pct':20,'gain_pct':100,'kd_pct':100,'min_pct':8,'max_pct':40,
        'right_trim_pct':0,'far_from':50,'far_gain_pct':0,'update_ms':10,
        'provenance':'ASSUMED SIL plumbing configuration; NOT locked robot tuning'}


def forward_velocity(left_pct,right_pct,radius_mm):
    if not math.isfinite(radius_mm)or radius_mm<=0:raise ValueError('invalid ASSUMED radius')
    result=[];duties=[row[0]for row in DM1_FORWARD]
    for side,duty in enumerate((left_pct,right_pct),1):
        if not math.isfinite(duty):raise ValueError('nonfinite duty')
        if duty==0:result.append(0.);continue
        if not 5<=duty<=100:raise ValueError('duty outside measured forward5..100% or exact0; rejected')
        hi=bisect.bisect_left(duties,duty)
        if duties[hi]==duty:speed=DM1_FORWARD[hi][side]
        else:
            lo=hi-1;a,b=DM1_FORWARD[lo],DM1_FORWARD[hi]
            speed=a[side]+(b[side]-a[side])*(duty-a[0])/(b[0]-a[0])
        result.append(speed/radius_mm)
    return tuple(result)


def build_library(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    library=directory/'libsil_line.so'
    command=['cc','-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Wconversion',
             '-Wshadow','-Werror','-I',str(ROOT),*(str(ROOT/name)for name in C_SOURCES),'-o',str(library)]
    subprocess.run(command,check=True,capture_output=True,text=True)
    meta=dict(command=command,compiler=subprocess.check_output(['cc','--version'],text=True).splitlines()[0],
              sources_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()for name in C_SOURCES},
              library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),config=CONFIG,
              mapping_source=DM1_SOURCE,forward_rows=DM1_FORWARD)
    (directory/'sil-build.json').write_text(json.dumps(meta,indent=2,sort_keys=True)+'\n')
    return library


class LineController:
    """One instance per loaded SO path; C state is private to that library path."""
    def __init__(self,library):
        self.lib=ctypes.CDLL(str(Path(library).resolve()));self.last_ms=None;self.calibrated=False
        self.lib.sil_calib_add.argtypes=[ctypes.c_uint16,ctypes.c_uint16]
        self.lib.sil_calib_end.argtypes=[ctypes.POINTER(ctypes.c_int32)]
        if hasattr(self.lib,'sil_update'):
            self.lib.sil_update.argtypes=[ctypes.c_uint16,ctypes.c_uint16,ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32)]
            self.lib.sil_update.restype=None
        for name in('sil_reset','sil_calib_begin','sil_calib_add','sil_calib_end'):getattr(self.lib,name).restype=None
        self.lib.sil_reset()
    def calibrate(self,rows):
        self.lib.sil_reset();self.lib.sil_calib_begin();self.last_ms=None
        if not 1<=len(rows)<=10000:raise ValueError('calibration row budget')
        for row in rows:
            if len(row)!=3 or row[2]!=0 or any(type(v)is not int or not 0<=v<=4095 for v in row):raise ValueError('invalid calibration ADC')
            self.lib.sil_calib_add(row[0],row[1])
        out=(ctypes.c_int32*7)();self.lib.sil_calib_end(out)
        info=dict(result=out[0],bad_mask=out[1],line_high=bool(out[2]),endpoints=list(out[3:7]))
        self.calibrated=out[0]==0 and out[1]==0
        if not self.calibrated:raise ValueError('public C calibration failed: '+str(info))
        return info
    def update(self,left,centre,ms):
        if not self.calibrated:raise ValueError('calibration required')
        if any(type(v)is not int or not 0<=v<=4095 for v in(left,centre)):raise ValueError('invalid12bit ADC')
        if type(ms)is not int or not 0<=ms<=20000 or (self.last_ms is not None and ms<=self.last_ms):raise ValueError('SIL source time order')
        out=(ctypes.c_int32*9)();self.lib.sil_update(left,centre,ms,out);self.last_ms=ms
        return dict(zip(('position','event','line_seen','junction','left_pct','right_pct','norm_left','norm_centre','fault'),map(int,out)))
