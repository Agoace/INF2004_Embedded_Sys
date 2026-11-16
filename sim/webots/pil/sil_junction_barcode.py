"""Observe-only A4 Code39 A -> junction; all geometry explicitly labelled."""
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
from .course_builder import Course
from .sil_line import ROOT
from .sil_motor import MotorLineController,CONFIG_HEADERS
from .sil_search import SEARCH_FIELDS,NATIVE_FIELDS,SEARCH_SOURCES
from .s3_sil_barcode import barcode_sheet

FAMILIES=(1,4,8)
LINE_Y_MM=157.5
SHEET_START_MM=970.5


def junction_barcode_course(*,lead_mm=300):
    if lead_mm not in(100,200,300):raise ValueError('approved lead100/200/300mm')
    sheet=barcode_sheet(sheet_start_mm=SHEET_START_MM,line_centre_y_mm=LINE_Y_MM,surface_z_mm=10,letter='A')
    tiles=[]
    for col in range(1,6):
        for row in range(-1,2):
            kind='BLANK';rotation=0
            if row==0:kind='CROSS'if col==4 else'STRAIGHT';rotation=0 if col==4 else 90
            elif col==4 and abs(row)==1:kind='STRAIGHT'
            tiles.append(dict(id=f't{col+2}_{row+2}',column=col,row=row,kind=kind,rotation_deg=rotation))
    course=Course.from_dict(dict(schema=1,name='sil_junction_barcode_assumed',
        provenance=dict(assumed=True,source='Observe-only *A* on A4landscape297x210mm; barheight160+/-10mm PHOTO_ESTIMATED; centredprint andinneredge line+25mm ASSUMED; paperEND->junctionCENTRE150mm ASSUMED. Cytron315x315x10mm tiles/18mm line nominalvendor dimensions; white margins/layout/seams ASSUMED. GP28MATproxies178/2321 MEASURED(MQTT2026-11-11,unlocated), assignment/responsepower1 ASSUMED. No action admission or controller tuning.'),
        origin_mm=[0,0],start=dict(position_mm=[970.5-lead_mm-80,150.25],yaw_deg=0),allow_open_ports=True,
        texture_mm_per_pixel=2,tiles=tiles,boxes=[],humps=[],barcodes=[dict(id='sheet_a',letter='A',
            centre_mm=list(sheet.sheet_centre_mm),rotation_deg=0,bar_height_mm=160,
            paper_size_mm=[297,210],paper_thickness_mm=0,reverse=False)]))
    assert sheet.junction_centre_x_mm==1417.5
    return course


BC_FIELDS=('raw_bc','bc_attempts','bc_letter','bc_publications','bc_failures','bc_window_reason',
    'bc_quiet_pass','bc_start_pass','bc_valid','bc_flags','bc_admission_reason','junction_confirmations',
    'junction_phase','action_command','bc_event_full')
CAL_FIELDS=('generation','samples','minimum','maximum','white_samples','black_samples','white_mean',
    'black_mean','black_hold_us','max_gap_us','rise','fall','mode','faults')
BC_SOURCES=('sim/webots/controllers/sil_line/barcode_adapter.c',*SEARCH_SOURCES[1:],
    'app_program/vehicle/junction_action.c','app_program/vehicle/junction_line.c',
    'app_program/vehicle/junction_live.c','app_program/subsystem3/barcode.c',
    'app_program/subsystem3/barcode_window.c','app_program/subsystem3/barcode_live_producer.c',
    'app_program/subsystem3/barcode_calibration.c')


def barcode_calibration_course():
    data=junction_barcode_course().data;data['name']='sil_bc_calibration_assumed'
    data['start']['position_mm']=[196,150.25];data['barcodes']=[]
    data['tiles']=[dict(id=f'c{col+1}_{row+1}',column=col,row=row,
        kind='STRAIGHT'if row==0 else'BLANK',rotation_deg=90 if row==0 else 0)
        for col in range(-1,2)for row in range(-1,2)]
    data['provenance']['source']+=' Aligned calibration-only3x3 white-tile world; GP28families are passive hypotheses of the same spin.'
    return Course.from_dict(data)


def build_barcode_library(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);library=directory/'libsil_line.so'
    command=['cc','-std=c11','-O2','-fPIC','-shared','-Wall','-Wextra','-Wconversion','-Wshadow','-Werror',
        '-DJUNCTION_TEST_CMD=0','-DBARCODE_LIVE_ENABLE=0','-DBARCODE_MEASURED_CLEAR_MM=0',
        '-I',str(ROOT),*(str(ROOT/n)for n in BC_SOURCES),'-lm','-o',str(library)]
    subprocess.run(command,check=True,capture_output=True,text=True)
    sources=(*BC_SOURCES,*CONFIG_HEADERS,'sim/webots/controllers/sil_line/motor_adapter.c')
    (directory/'sil-build.json').write_text(json.dumps(dict(command=command,
        compiler=subprocess.check_output(['cc','--version'],text=True).splitlines()[0],
        sources_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()for n in sources},
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        scope='Unchanged native B2b SHADOW reader and jl_module; synchronous SDK board/acquisition replacement, no flag/command injection'),indent=2)+'\n')
    return library


class BarcodeController(MotorLineController):
    def __init__(self,library):
        super().__init__(library)
        pointer=ctypes.POINTER(ctypes.c_int32)
        self.lib.sil_bc_select.argtypes=[ctypes.c_uint32];self.lib.sil_bc_select.restype=None
        self.lib.sil_bc_cal_begin.argtypes=[];self.lib.sil_bc_cal_begin.restype=None
        self.lib.sil_bc_cal_add.argtypes=[ctypes.c_uint32,ctypes.c_uint32,ctypes.c_uint16];self.lib.sil_bc_cal_add.restype=None
        self.lib.sil_bc_cal_end.argtypes=[pointer];self.lib.sil_bc_cal_end.restype=None
        self.lib.sil_bc_events.argtypes=[pointer];self.lib.sil_bc_events.restype=None
        self.lib.sil_barcode_update.argtypes=[ctypes.c_uint16,ctypes.c_uint16,ctypes.c_uint32,ctypes.c_int32,ctypes.c_int32,ctypes.c_uint16,pointer]
        self.lib.sil_barcode_update.restype=None
    def barcode_begin(self):self.lib.sil_bc_cal_begin()
    def barcode_add(self,us,raws):
        for i,raw in enumerate(raws):self.lib.sil_bc_cal_add(i,us,raw)
    def barcode_end(self):
        out=(ctypes.c_int32*42)();self.lib.sil_bc_cal_end(out)
        return {str(n):dict(zip(CAL_FIELDS,map(int,out[i*14:(i+1)*14])))for i,n in enumerate(FAMILIES)}
    def barcode_calibrate(self,rows,family):
        if family not in FAMILIES:raise ValueError('unsupported GP28 family')
        self.barcode_begin()
        for row in rows:self.barcode_add(row[0],row[1:])
        info=self.barcode_end();self.lib.sil_bc_select(FAMILIES.index(family));return info
    def metadata(self):
        info=super().metadata();info.update(registered_modules=['LINE(jl_module+line_trace+junction_action)','SEARCH'],
            barcode_mode='SHADOW',barcode_live_enable=0,barcode_measured_clear_mm=0)
        return info
    def update(self,left,centre,ms,left_count,right_count,raw_bc):
        if not self.calibrated or self.last_ms is not None and ms<=self.last_ms:raise ValueError('calibration/time')
        out=(ctypes.c_int32*36)();self.lib.sil_barcode_update(left,centre,ms,left_count,right_count,raw_bc,out);self.last_ms=ms
        fields=('position','event','line_seen','junction','left_pct','right_pct','norm_left','norm_centre','fault',
            'desired_left_mm_s','desired_right_mm_s','motor_faults',*SEARCH_FIELDS,*BC_FIELDS)
        result=dict(zip(fields,map(int,out)));result['left_pct']/=10;result['right_pct']/=10
        return result
    def events(self):
        out=(ctypes.c_int32*(1+128*14))();self.lib.sil_bc_events(out)
        fields=('sim_ms','serial','reason','letter','accepted','start_us','read_us','rise','fall',
                'leading_um','narrow_um','wide_um','producer_failures','publications')
        return [dict(zip(fields,map(int,out[1+i*14:1+(i+1)*14])))for i in range(out[0])]
