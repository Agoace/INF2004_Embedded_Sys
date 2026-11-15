"""Real P7 steer_mm + motor_ctl. Lag fitted ONLY to B3 bootede2 step data."""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess
from .sil_line import ROOT,LineController,forward_velocity,DM1_SOURCE,DM1_FORWARD

# Effective plant fit, NOT a unique measured motor constant. Identified with
# real closed-loop motor_ctl against bootede2's150mm/s raw t90=110ms L/R.
TAU_MS=80.0
FIT_SOURCE='docs/archive/s2-motion/MOTOR_CONTROLLER.md B3; main build/steer_logs/text_ede2.txt MB50->61'
MOTOR_SOURCES=('sim/webots/controllers/sil_line/motor_adapter.c','app_program/subsystem3/line_position.c',
    'app_program/subsystem3/line_steer.c','app_program/subsystem2/motor_ctl.c')
CONFIG_HEADERS=('app_program/vehicle/vehicle_robot_config.h','app_program/subsystem2/motor_ctl_robot.h',
    'app_program/subsystem2/motor_ctl.h','app_program/subsystem3/line_position.h','app_program/subsystem3/line_steer.h')


def build_motor_library(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);library=directory/'libsil_line.so'
    command=['cc','-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Wconversion','-Wshadow','-Werror',
             '-I',str(ROOT),*(str(ROOT/n)for n in MOTOR_SOURCES),'-o',str(library)]
    subprocess.run(command,check=True,capture_output=True,text=True)
    (directory/'sil-build.json').write_text(json.dumps(dict(command=command,
        compiler=subprocess.check_output(['cc','--version'],text=True).splitlines()[0],
        sources_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()for n in(*MOTOR_SOURCES,*CONFIG_HEADERS)},
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),lag_tau_ms=TAU_MS,fit_source=FIT_SOURCE,
        mapping_source=DM1_SOURCE,forward_rows=DM1_FORWARD,
        counts_projection='joint radians*2535/(2pi), nearest ties-even; not physical GPIO edges',
        cadence='1ms physics/ADC;10ms motor tick before steer_mm command for next tick'),indent=2)+'\n')
    return library


class LagPlant:
    def __init__(self,tau_ms=TAU_MS):
        if not math.isfinite(tau_ms)or tau_ms<=0:raise ValueError('lag tau must be positive')
        self.alpha=-math.expm1(-1/tau_ms);self.velocity=(0.,0.)
    def advance(self,left_pct,right_pct):
        try:targets=forward_velocity(left_pct,right_pct,32.5)
        except ValueError as error:
            raise ValueError(f'DM1 rejected left_pct={left_pct!r} right_pct={right_pct!r}: {error}')from error
        self.velocity=tuple(v+self.alpha*(t-v)for v,t in zip(self.velocity,targets))
        return self.velocity


class MotorLineController(LineController):
    def __init__(self,library):
        super().__init__(library)
        self.lib.sil_motor_meta.argtypes=[ctypes.POINTER(ctypes.c_int32)];self.lib.sil_motor_meta.restype=None
        self.lib.sil_motor_update.argtypes=[ctypes.c_uint16,ctypes.c_uint16,ctypes.c_uint32,ctypes.c_int32,ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_motor_update.restype=None
        self.lib.sil_motor_tick.argtypes=[ctypes.c_uint32,ctypes.c_int32,ctypes.c_int32,ctypes.c_int16,ctypes.c_int16,ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_motor_tick.restype=None
    def metadata(self):
        out=(ctypes.c_int32*10)();self.lib.sil_motor_meta(out)
        return dict(steer=list(out[:7]),accel_cpt_q8=out[7],max_speed_mm_s=out[8],tick_ms=out[9])
    def update(self,left,centre,ms,left_count,right_count):
        if not self.calibrated or self.last_ms is not None and ms<=self.last_ms:raise ValueError('native SIL calibration/time')
        if any(type(v)is not int or not 0<=v<=4095 for v in(left,centre)):raise ValueError('native ADC')
        out=(ctypes.c_int32*12)();self.lib.sil_motor_update(left,centre,ms,left_count,right_count,out);self.last_ms=ms
        result=dict(zip(('position','event','line_seen','junction','left_pct','right_pct','norm_left','norm_centre','fault'),map(int,out[:9])))
        result['left_pct']/=10;result['right_pct']/=10
        result.update(desired_left_mm_s=out[9],desired_right_mm_s=out[10],motor_faults=out[11])
        return result
    def tick(self,ms,counts,speeds):
        out=(ctypes.c_int32*3)();self.lib.sil_motor_tick(ms,*counts,*speeds,out);return tuple(out)


def step_response(library,*,tau_ms=TAU_MS):
    controller=MotorLineController(library);plant=LagPlant(tau_ms)
    angles=[0.,0.];duties=(0.,0.);history=[];t90=[None,None]
    for ms in range(1,1001):
        velocity=plant.advance(*duties)
        for side in(0,1):angles[side]+=velocity[side]*.001
        counts=[round(a*2535/(2*math.pi))for a in angles]
        if ms%10==0:
            left,right,fault=controller.tick(ms,counts,(150,150))
            if fault:raise ValueError('native motor bench fault')
            duties=(left/10,right/10)
        # First command occurs10ms; raw velocity, not50ms average.
        for side in(0,1):
            if t90[side]is None and velocity[side]*32.5>=135:t90[side]=ms-10
        history.append((ms,*counts,*duties,*(v*32.5 for v in velocity)))
    return dict(raw_t90_ms=t90,motor_config=controller.metadata(),tau_ms=tau_ms,history=history,fit_source=FIT_SOURCE)
