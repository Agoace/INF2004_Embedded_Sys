"""Back-to-back nominal tile arcs; geometry/truth only, never controller input."""
import math
from .course_builder import Course
from .sil_curve import RADIUS_MM


def s_curve_course():
    tiles=[]
    for column in range(-2,8):
        for row in range(-1,4):
            kind='BLANK';rotation=0
            if row==0 and column<2:kind='STRAIGHT';rotation=90
            elif row==0 and column==2:kind='CURVE';rotation=90
            elif row==1 and column==2:kind='CURVE';rotation=270
            elif row==1 and 3<=column<=6:kind='STRAIGHT';rotation=90
            tiles.append(dict(id=f't{column+2}_{row+1}',column=column,row=row,kind=kind,rotation_deg=rotation))
    return Course.from_dict(dict(schema=1,name='sil_s_curve_assumed',
        provenance=dict(assumed=True,source='Straight -> left90deg -> right90deg back to back -> straight; Cytron315x315x10mm tiles/18mm line; corner-centred quarter circles radius157.5mm: nominal, vendor tile sheet; not ruler-surveyed. Vendor source: docs/archive/simulation/SIMULATION.md Published track specification. White margins/layout ASSUMED; tile seams NOT modelled; no controller tuning.'),
        origin_mm=[0,0],start=dict(position_mm=[196,150.25],yaw_deg=0),allow_open_ports=True,
        texture_mm_per_pixel=2,tiles=tiles,boxes=[],humps=[],barcodes=[]))


def nearest_s_path(x,y):
    candidates=[]
    px=max(-630.,min(630.,x));candidates.append((math.hypot(x-px,y-157.5),y-157.5,px+630.,'entry'))
    a=max(-math.pi/2,min(0.,math.atan2(y-315.,x-630.)))
    px=630.+RADIUS_MM*math.cos(a);py=315.+RADIUS_MM*math.sin(a)
    error=-math.sin(a)*(y-py)-math.cos(a)*(x-px)
    candidates.append((math.hypot(x-px,y-py),error,1260.+RADIUS_MM*(a+math.pi/2),'left_arc'))
    a=max(math.pi/2,min(math.pi,math.atan2(y-315.,x-945.)%(2*math.pi)))
    px=945.+RADIUS_MM*math.cos(a);py=315.+RADIUS_MM*math.sin(a)
    error=math.sin(a)*(y-py)+math.cos(a)*(x-px)
    candidates.append((math.hypot(x-px,y-py),error,1260.+RADIUS_MM*math.pi/2+RADIUS_MM*(math.pi-a),'right_arc'))
    px=max(945.,min(2205.,x));candidates.append((math.hypot(x-px,y-472.5),y-472.5,
        1260.+math.pi*RADIUS_MM+px-945.,'exit'))
    distance,error,progress,segment=min(candidates,key=lambda c:c[0])
    return dict(error_mm=math.copysign(distance,error if error else 1),progress_mm=progress,segment=segment)


def analyze_s_curve(rows):
    completed=None;segments=set()
    for row in rows:
        yaw=row[15];x=row[12]*1000+119*math.cos(yaw)-7.25*math.sin(yaw)
        y=row[13]*1000+119*math.sin(yaw)+7.25*math.cos(yaw)
        p=nearest_s_path(x,y);segments.add(p['segment'])
        if p['segment']=='exit'and x>=1260 and row[12]*1000>=945 and abs(math.atan2(math.sin(yaw),math.cos(yaw)))<=math.radians(15)and{'left_arc','right_arc'}<=segments:
            completed=row[0];break
    return dict(completes_curve=completed is not None,completion_sim_ms=completed,
        max_tracking_error_mm=max((abs(r[20])for r in rows),default=None),
        lost_events=sum((int(r[4])&31)==5 for r in rows),found_events=sum((int(r[4])&31)==6 for r in rows),
        searches=rows[-1][32]if rows else 0,found_by_search=rows[-1][33]if rows else 0,
        final_shell_fault=rows[-1][29]if rows else None,duration_sim_ms=rows[-1][0]if rows else 0)
