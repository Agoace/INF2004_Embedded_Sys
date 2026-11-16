"""Task A geometry/height projection and explicit transport diagnostic.

No fitted parameter is adjusted from a run result. Fixed angular divergence
scales the B footprint by9/8; ADC endpoints/radiometry remain assumptions.
"""
import collections
import math
from .sil_line_oscillation import quantile,correlation


class DelayPair:
    def __init__(self,delay_ms):
        if type(delay_ms)is not int or not 0<=delay_ms<=20:raise ValueError('delay0..20ms')
        self.delay=delay_ms;self.queue=collections.deque();self.last_ms=0
    def update(self,ms,pair):
        if ms!=self.last_ms+1:raise ValueError('delay requires contiguous1ms samples')
        self.last_ms=ms;self.queue.append(tuple(pair))
        if len(self.queue)>self.delay+1:self.queue.popleft()
        return self.queue[0]


def apply_geometry(config,profile,spin):
    height=profile['line_clearance_mm']
    for name in ('GP27','GP26'):
        p=config['profiles'][name];scale=height/p['reference_height_mm']
        p['sigma_x_mm']*=scale;p['sigma_y_mm']*=scale;p['reference_height_mm']=height
        p['assumptions']+=['TASK_A_FIXED_ANGULAR_DIVERGENCE_9_OVER_8_NOT_CALIBRATED']
        config['mounts'][name].update(forward_mm=profile['line_forward_mm'],
            left_mm=profile['line_left_mm']if name=='GP27'else 0,flat_clearance_mm=height)
    config['calibration_rows']=spin['calibration_rows'];config['spin_endpoints']=spin['calibration']['endpoints']
    config['calibration_provenance']='HISTORICAL actual8mm aligned Bspin rows, reused unchanged; NOT a9mm spin calibration. No extra spin run.'
    config['flat_projection']='ActualXY/yaw; configured9mm flat clearance, fixed angular footprint9/8 scaling; height radiometry/tilt UNCALIBRATED'
    config['taskA_geometry']=True
    config['delay_audit']='P7/MC10ms + next motor tick publication + existingDM1/80ms wholeloop lag already yields40-60ms command-wheel phase; no extra40ms delay. Thirdcase20ms ADCtransport diagnostic, not a measured causal transport.'
    return config


def window_metrics(rows,start,end):
    selected=[r for r in rows if start<=r[0]<=end];rates=[];balance=[];pos=[]
    for a,b in zip(selected,selected[1:]):
        delta=(b[15]-a[15]+math.pi)%(2*math.pi)-math.pi
        rates.append(delta*180/math.pi*1000/(b[0]-a[0]))
    balance=[r[2]-r[1]for r in selected];pos=[r[3]for r in selected]
    # Fixed20ms grid; local autocorrelation maxima, >=3cycles. Do not force
    # real260-380ms period or hide quiet windows.
    grid=rates[::20];peaks=[]
    for k in range(6,min(61,len(grid)//3)):
        score=correlation(grid[:-k],grid[k:])
        if score>correlation(grid[:-(k-1)],grid[k-1:])and score>=correlation(grid[:-(k+1)],grid[k+1:]):peaks.append((score,k*20))
    score,period=max(peaks,default=(0,None));amp=lambda v:(quantile(v,.95)-quantile(v,.05))/2 if v else None
    return dict(start_ms=start,end_ms=end,samples=len(selected),period_ms=period if score>=.3 and amp(rates)>=1 else None,
        period_correlation=score,yaw_rate_amplitude_deg_s=amp(rates),raw_C_minus_L_amplitude_adc=amp(balance),position_amplitude=amp(pos),
        yaw_angle_amplitude_deg=amp([r[15]*180/math.pi for r in selected]),
        non_LINE_rows=sum(len(r)>28 and(r[27]!=1 or r[28]!=1)for r in selected))
