"""Offline position-band and timing metrics for the bounded base/order trial."""
import math
import statistics as st
from .sil_excited_line import window_metrics
from .sil_line_oscillation import quantile

WINDOWS=((0,3000),(2000,4000),(4000,6000),(6000,8000),(8000,10000))


def position_shares(rows):
    counts=[0,0,0];far=[]
    for row in rows:
        p=abs(row[3]);index=0 if p<45 else 1 if p<=52 else 2
        counts[index]+=1
        if index==2:far.append(abs(row[24]-row[25]))
    return dict(rows=len(rows),counts_lt45_45to52_gt52=counts,
        percentages_lt45_45to52_gt52=[100*n/len(rows)for n in counts]if rows else None,
        far_mean_abs_asked_difference_mm_s=st.mean(far)if far else None,
        far_p95_abs_asked_difference_mm_s=quantile(far,.95),
        far_max_abs_asked_difference_mm_s=max(far)if far else None)


def analyze_window(rows,start,end,track_mm=111.5):
    selected=[r for r in rows if start<=r[0]<=end]
    m=window_metrics(rows,start,end);lookup={int(r[0]):r for r in rows}
    grid=[r for r in selected if int(r[0])%20==0]
    line=lambda r:r[27]==1 and r[28]==1
    m['position_shares']={
        'all_1ms_rows':position_shares(selected),'LINE_1ms_rows':position_shares([r for r in selected if line(r)]),
        'all_20ms_rows':position_shares(grid),'LINE_20ms_rows':position_shares([r for r in grid if line(r)])}
    wheel_rates=[]
    for r in grid:
        before=lookup.get(int(r[0])-10)
        if before is None:continue
        speeds=[math.trunc((r[j]-before[j])*1000000/(12414*10))for j in (22,23)]
        wheel_rates.append((speeds[1]-speeds[0])*180/(math.pi*track_mm))
    body_rates=[]
    for a,b in zip(selected,selected[1:]):
        delta=(b[15]-a[15]+math.pi)%(2*math.pi)-math.pi
        body_rates.append(delta*180/math.pi*1000/(b[0]-a[0]))
    for name,values in (('body_yaw',body_rates),('KR_like_wheel_yaw',wheel_rates)):
        m[name+'_rms_deg_s']=math.sqrt(st.mean(v*v for v in values))if values else None
        m[name+'_absolute_peak_deg_s']=max(map(abs,values))if values else None
    m['KR_like_wheel_yaw_amplitude_deg_s']=(quantile(wheel_rates,.95)-quantile(wheel_rates,.05))/2 if wheel_rates else None
    m['LINE_cycle_period_ms']=m['period_ms']if m['non_LINE_rows']==0 else None
    m['cycle_scope']='RUNNING LINE only'if m['non_LINE_rows']==0 else'MIXED modules; aggregate period is NOT a LINE cycle'
    return m
