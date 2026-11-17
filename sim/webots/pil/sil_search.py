"""Unchanged vehicle shell with only actual LINE/SEARCH modules registered."""
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
from .sil_line import ROOT
from .sil_motor import MotorLineController,MOTOR_SOURCES,CONFIG_HEADERS,TAU_MS,DM1_REVERSE

SEARCH_FIELDS=('module','shell_state','shell_fault','shell_fault_detail','handoffs','searches',
               'found_by_search','last_outcome','action')
NATIVE_FIELDS=('count_left','count_right','desired_left_mm_s','desired_right_mm_s','motor_faults')
SEARCH_SOURCES=('sim/webots/controllers/sil_line/search_adapter.c',*MOTOR_SOURCES[1:],
    'app_program/vehicle/vehicle_sm.c','app_program/subsystem3/line_trace.c',
    'app_program/subsystem3/line_search.c','app_program/subsystem5/obstacle.c')


def build_search_library(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);library=directory/'libsil_line.so'
    command=['cc','-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Wconversion','-Wshadow','-Werror',
             '-I',str(ROOT),*(str(ROOT/n)for n in SEARCH_SOURCES),'-o',str(library)]
    subprocess.run(command,check=True,capture_output=True,text=True)
    sources=(*SEARCH_SOURCES,*CONFIG_HEADERS,MOTOR_SOURCES[0],
        'app_program/vehicle/vehicle_sm.h','app_program/vehicle/vehicle_inputs.h',
        'app_program/subsystem3/line_trace.h','app_program/subsystem3/line_search.h',
        'app_program/subsystem5/obstacle.h')
    (directory/'sil-build.json').write_text(json.dumps(dict(command=command,
        compiler=subprocess.check_output(['cc','--version'],text=True).splitlines()[0],
        sources_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()for n in sources},
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),lag_tau_ms=TAU_MS,
        reverse_rows=((0,0,0),*DM1_REVERSE),dead_band='DM1 dead band, steady-state only; dynamic braking UNMODELLED',registered_modules=['LINE(line_trace)','SEARCH(line_search)'],
        seams=['echo_new=false; no sonar inputs','no SCAN/DETOUR/junction wrapper registered',
               'motor tick then shell step10ms; line ADC1ms; pending last LOST/FOUND retained']),indent=2)+'\n')
    return library


class SearchController(MotorLineController):
    def __init__(self,library):
        super().__init__(library)
        self.lib.sil_search_update.argtypes=self.lib.sil_motor_update.argtypes
        self.lib.sil_search_update.restype=None
        self.lib.sil_search_module_step.argtypes=[ctypes.c_uint32,ctypes.c_int32,ctypes.c_int32,
            ctypes.c_uint16,ctypes.c_uint8,ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_search_module_step.restype=None
        self.lib.sil_search_config.argtypes=[ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_search_config.restype=None
    def metadata(self):
        result=super().metadata();out=(ctypes.c_int32*4)();self.lib.sil_search_config(out)
        result.update(search_speed_mm_s=out[0],search_turn_counts=out[1],
                      line_straight_counts=out[2],line_hint_counts=out[3],registered_modules=['LINE','SEARCH'])
        return result
    def timing(self):
        output=(ctypes.c_int32*7)()
        function=self.lib.sil_search_timing
        function.argtypes=[ctypes.POINTER(ctypes.c_int32)];function.restype=None
        function(output)
        return list(output)
    @staticmethod
    def _decode(out):
        result=dict(zip(('position','event','line_seen','junction','left_pct','right_pct','norm_left',
                         'norm_centre','fault','desired_left_mm_s','desired_right_mm_s','motor_faults',
                         *SEARCH_FIELDS),map(int,out)))
        result['left_pct']/=10;result['right_pct']/=10
        return result
    def update(self,left,centre,ms,left_count,right_count):
        if not self.calibrated or self.last_ms is not None and ms<=self.last_ms:raise ValueError('native SIL calibration/time')
        if any(type(v)is not int or not 0<=v<=4095 for v in(left,centre)):raise ValueError('native ADC')
        out=(ctypes.c_int32*21)();self.lib.sil_search_update(left,centre,ms,left_count,right_count,out)
        self.last_ms=ms;return self._decode(out)
    def module_step(self,ms,*,counts=(0,0),event=0,motor_fault=0):
        out=(ctypes.c_int32*21)();self.lib.sil_search_module_step(ms,*counts,event,motor_fault,out)
        return self._decode(out)
