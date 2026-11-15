"""Diagnostic recovery measurements; never controller inputs or tuning."""
import math


def analyze_recovery(rows):
    if not rows:raise ValueError('no recovery observations')
    initial=rows[0][20];sign=1 if initial>1e-6 else -1 if initial< -1e-6 else 0
    settle=None;entered=None;first_hold=None
    for row in rows:
        if len(row)not in(22,27) or any(not math.isfinite(v)for v in row):raise ValueError('invalid recovery row')
        if abs(row[20])<3:
            if entered is None:entered=int(row[0])
            if row[0]-entered+1>=500 and first_hold is None:first_hold=entered
        else:entered=None
    if entered is not None and rows[-1][0]-entered+1>=500:settle=entered
    reached=[int(r[0])for r in rows if max(r[9:11])>=300]
    return dict(initial_pair_error_mm=initial,initial_in_sensor_reach=max(rows[0][9:11])>=300,
        line_acquired=bool(reached),acquired_ms=reached[0]if reached else None,settle_ms=settle,first_500ms_hold_ms=first_hold,
        opposite_side_overshoot_mm=max((max(0.,-sign*r[20])for r in rows),default=0.),
        max_abs_error_mm=max(abs(r[20])for r in rows),final_error_mm=rows[-1][20],
        final_in_band=abs(rows[-1][20])<3,lost_events=sum((int(r[4])&31)==5 for r in rows),
        found_events=sum((int(r[4])&31)==6 for r in rows),stop_latched=bool(rows[-1][11]))
