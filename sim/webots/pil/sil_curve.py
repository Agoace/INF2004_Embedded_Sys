"""One documented tile-radius curve; truth metrics never enter native control."""
import math
from .course_builder import Course

RADIUS_MM=157.5
CENTRE_MM=(630.,315.)


def curve_course():
    tiles=[]
    for column in range(-2,5):
        for row in range(-1,5):
            kind='BLANK';rotation=0
            if row==0 and column<2:kind='STRAIGHT';rotation=90
            elif row==0 and column==2:kind='CURVE';rotation=90
            elif column==2 and 1<=row<=3:kind='STRAIGHT'
            tiles.append(dict(id=f't{column+2}_{row+1}',column=column,row=row,kind=kind,rotation_deg=rotation))
    return Course.from_dict(dict(schema=1,name='sil_curve_assumed',
        provenance=dict(assumed=True,source='Straight -> left90deg arc -> straight on315mm tiles; radius157.5mm from docs/simulation/S3_SENSOR_MODEL.md tile convention, NOT an independent measured-radius survey. White margins ASSUMED(real mat wider), tile seams NOT modelled; no controller tuning.'),
        origin_mm=[0,0],start=dict(position_mm=[196,150.25],yaw_deg=0),allow_open_ports=True,
        texture_mm_per_pixel=2,tiles=tiles,boxes=[],humps=[],barcodes=[]))


def nearest_path(x,y):
    """Return signed pair-centre error, arc-length progress, and segment."""
    candidates=[]
    # Entry straight fromx-630..630, y157.5.
    px=max(-630.,min(630.,x));candidates.append((math.hypot(x-px,y-157.5),y-157.5,px+630.,'entry'))
    cx,cy=CENTRE_MM;angle=max(-math.pi/2,min(0.,math.atan2(y-cy,x-cx)))
    px=cx+RADIUS_MM*math.cos(angle);py=cy+RADIUS_MM*math.sin(angle)
    error=-math.sin(angle)*(y-py)-math.cos(angle)*(x-px)
    candidates.append((math.hypot(x-px,y-py),error,1260.+RADIUS_MM*(angle+math.pi/2),'arc'))
    py=max(315.,min(1260.,y));candidates.append((math.hypot(x-787.5,y-py),787.5-x,
        1260.+math.pi*RADIUS_MM/2+py-315.,'exit'))
    distance,error,progress,segment=min(candidates,key=lambda c:c[0])
    # Euclidean distance includes finite endpoints; error sign is diagnostic.
    return dict(error_mm=math.copysign(distance,error if error else 1),progress_mm=progress,segment=segment)


def analyze_curve(rows):
    completed=None
    for row in rows:
        # Pair midpoint must leave the arc by one tile; axle must also be on exit.
        yaw=row[15];x=row[12]*1000+119*math.cos(yaw)-7.25*math.sin(yaw)
        y=row[13]*1000+119*math.sin(yaw)+7.25*math.cos(yaw)
        p=nearest_path(x,y)
        if p['segment']=='exit'and y>=630 and row[13]*1000>=315 and abs(math.atan2(math.sin(yaw-math.pi/2),math.cos(yaw-math.pi/2)))<=math.radians(15):
            completed=row[0];break
    return dict(completes_curve=completed is not None,completion_sim_ms=completed,
        max_tracking_error_mm=max((abs(r[20])for r in rows),default=None),
        lost_events=sum((int(r[4])&31)==5 for r in rows),found_events=sum((int(r[4])&31)==6 for r in rows),
        searches=rows[-1][32]if rows else 0,found_by_search=rows[-1][33]if rows else 0,
        final_shell_fault=rows[-1][29]if rows else None,duration_sim_ms=rows[-1][0]if rows else 0)
