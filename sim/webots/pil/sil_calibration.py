"""Sim-owned equivalent of vehicle_task.c calibration sequencing, not SDK task code."""
import ctypes
from .sil_motor import MotorLineController


class CalibrationController(MotorLineController):
    def __init__(self,library):
        super().__init__(library)
        self.lib.sil_cal_spin_config.argtypes=[ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_cal_spin_config.restype=None
        self.lib.sil_cal_motor_stop.argtypes=[];self.lib.sil_cal_motor_stop.restype=None
        self.lib.sil_cal_observe.argtypes=[ctypes.c_uint16,ctypes.c_uint16,ctypes.c_uint32,ctypes.POINTER(ctypes.c_int32)]
        self.lib.sil_cal_observe.restype=None
        v=(ctypes.c_int32*7)();self.lib.sil_cal_spin_config(v)
        self.spin_config=dict(zip(('speed_mm_s','step_counts','timeout_ms','pause_ms','centre_stop',
                                  'centre_min','peak_drop'),map(int,v)))
        self.rows=[];self.info=None;self.peak=0;self.lib.sil_calib_begin()
    def sample(self,adc,ms,collect):
        if collect:
            self.rows.append([*adc,0]);self.lib.sil_calib_add(*adc)
            return 0
        out=(ctypes.c_int32*1)();self.lib.sil_cal_observe(*adc,ms,out)
        self.peak=max(self.peak,out[0]);return out[0]
    def finish(self):
        out=(ctypes.c_int32*7)();self.lib.sil_calib_end(out)
        self.info=dict(result=out[0],bad_mask=out[1],line_high=bool(out[2]),endpoints=list(out[3:7]))
        if out[0]!=0 or out[1]!=0:raise ValueError('spin calibration failed: '+str(self.info))
        self.calibrated=True;return self.info
    def stop(self):self.lib.sil_cal_motor_stop()


class CalibrationSequence:
    """Same wheel goals, timing, directions and homing test as vehicle_task.c689–788."""
    def __init__(self,config,home=(0,0)):
        self.cfg=config;self.home=tuple(home);self.start=(0,0)
        self.stage='out_negative';self.since=0;self.running=[True,True]
        self.history=[dict(stage=self.stage,ms=0,counts=list(home))]
    @property
    def collecting(self):return self.stage in('out_negative','pause_negative','out_positive','pause_positive')
    @property
    def done(self):return self.stage=='done'
    def _enter(self,stage,ms,counts):
        self.stage=stage;self.since=ms;self.start=tuple(c-h for c,h in zip(counts,self.home))
        self.running=[True,True];self.history.append(dict(stage=stage,ms=ms,counts=list(counts)))
    def advance(self,ms,counts,centre=0,peak=0,*,faults=0):
        if faults:raise ValueError('latched motor fault during calibration')
        if self.done:return dict(speeds=(0,0),stop=True,end=False)
        end=False;stop=False
        if self.stage.startswith('pause'):
            if ms-self.since<self.cfg['pause_ms']:return dict(speeds=(0,0),stop=True,end=False)
            if self.stage=='pause_negative':self._enter('out_positive',ms,counts)
            else:
                self._enter('home',ms,counts);end=True
        if ms-self.since>=self.cfg['timeout_ms']:raise ValueError('calibration spin timeout')
        direction=1 if self.stage=='out_positive'else -1
        on_line=self.stage=='home'and(centre>=self.cfg['centre_stop']or
            peak>=self.cfg['centre_min']and centre+self.cfg['peak_drop']<=peak)
        for side,count in enumerate(counts):
            from_home=count-self.home[side]
            if self.stage=='home':goal=from_home==0 or(from_home<0)!=(self.start[side]<0)
            else:
                started_out=abs(self.start[side])>=self.cfg['step_counts']//2
                goal=(not started_out or(from_home<0)!=(self.start[side]<0))and abs(from_home)>=self.cfg['step_counts']
            if on_line or goal:self.running[side]=False
        if not any(self.running):
            following={'out_negative':'pause_negative','out_positive':'pause_positive','home':'done'}[self.stage]
            self._enter(following,ms,counts);stop=True
        signs=(direction,-direction)
        speeds=(0,0)if stop else tuple(sign*self.cfg['speed_mm_s']if active else 0 for sign,active in zip(signs,self.running))
        return dict(speeds=speeds,stop=stop,end=end)
