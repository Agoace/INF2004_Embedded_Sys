"""Forward-band velocity smoke. No serial, physical PWM transient or brake model."""
from collections import namedtuple
import csv
import json
import math
from pathlib import Path
import struct
from .passive_replay import PassiveReplay,SAMPLE_FIELDS,IO_FIELDS,SESSION
from .wire import Frame,Type,encode,i16

PWM_FULL_SCALE=6250  # Frozen protocol effective PWM scale, not a fitted constant.
DM1_SOURCE='569672a:docs/archive/s2-motion/DUTY_MAP.md; Run DM1 2026-11-04 boot5d51; forward table rows16% and20%'
DM1_FORWARD_MM_S={'16':(96.,95.),'20':(129.,129.)}
# Every threshold here is ASSUMED/UNMEASURED software-smoke acceptance, not calibration.
THRESHOLDS={'max_travel_m':1.,'min_z_m':.015,'max_z_m':.15,'min_upright_cos':.9,
    'min_forward_x_m':.05,'max_straight_y_m':.05,'max_straight_yaw_rad':.1,
    'min_turn_yaw_rad':.02,'stop_speed_m_s':.005,'stop_yaw_rate_rad_s':.05,
    'stop_drift_m':.002,'stop_encoder_counts':2}
Motion=namedtuple('Motion','slot sim_us yaw_rad upright_cos vx vy vz wx wy wz command_issued_raw_us pwm_left pwm_right target_left_rad_s target_right_rad_s')


def map_pwm_to_velocity(left,right,radius_mm):
    if not math.isfinite(radius_mm) or radius_mm<=0:raise ValueError('invalid ASSUMED radius')
    speeds=[]
    for side,pwm in enumerate((left,right)):
        if type(pwm)is not int:raise ValueError('OUTPUT PWM must be integral')
        if pwm==0:speeds.append(0.);continue
        if not 1000<=pwm<=1250:raise ValueError('OUTPUT outside measured16..20% forward band; rejected, not clamped')
        # Endpoints measured; interpolation between endpoints is ASSUMED.
        speed=DM1_FORWARD_MM_S['16'][side]+(pwm-1000)/250*(DM1_FORWARD_MM_S['20'][side]-DM1_FORWARD_MM_S['16'][side])
        speeds.append(speed/radius_mm)
    return tuple(speeds)


def drive_command(slot):
    if type(slot)is not int or not 0<=slot<10000:raise ValueError('script slot out of range')
    if slot<1000 or slot>=3000:return 0,0,'stop'
    if slot<2000:return 1250,1250,'straight'
    if slot<2500:return 1000,1250,'left'
    return 1250,1000,'right'


class DrivingReplay(PassiveReplay):
    OUTPUT_DIRECTION='OUTPUT_CONSUMED'
    def __init__(self,cpr,radius_mm,*,torque_limit_nm=.12,command_script=drive_command):
        super().__init__(cpr)
        if not math.isfinite(torque_limit_nm) or torque_limit_nm<=0:raise ValueError('invalid ASSUMED torque cap')
        map_pwm_to_velocity(0,0,radius_mm)
        self.radius_mm=radius_mm;self.torque_limit_nm=torque_limit_nm;self.command_script=command_script
        self.motion=[];self.rejections=[];self.pending=None;self.error=None

    def sample(self,sim_us,raw_start_us,raw_end_us,values,**kwargs):
        if self.motion and raw_start_us<self.motion[-1].command_issued_raw_us:raise ValueError('command/sample clock order')
        return super().sample(sim_us,raw_start_us,raw_end_us,values,**kwargs)

    def output_packet(self,slot):
        left,right,_=self.command_script(slot);payload=bytearray(32)
        struct.pack_into('<hhH',payload,0,left,right,1500);payload[7]=1
        return encode(Frame(Type.OUTPUT,SESSION,slot,slot*1000,slot,bytes(payload)))

    def consume_output(self,frame):
        left,right=i16(frame.payload,0),i16(frame.payload,2)
        try:targets=map_pwm_to_velocity(left,right,self.radius_mm)
        except ValueError as error:
            self.rejections.append(dict(slot=frame.sequence,pwm_left=left,pwm_right=right,error=str(error)))
            raise
        self.pending=(left,right,*targets)

    def observe_motion(self,yaw,upright,twist,issued_raw_us):
        if len(self.motion)!=len(self.samples)-1 or self.pending is None:raise ValueError('motion/sample order')
        if len(twist)!=6 or any(not math.isfinite(v)for v in(yaw,upright,*twist)):
            raise ValueError('nonfinite motion')
        if type(issued_raw_us)is not int or issued_raw_us<self.samples[-1][-1]:raise ValueError('command clock order')
        self.motion.append(Motion(len(self.motion),self.samples[-1][0],yaw,upright,*twist,issued_raw_us,*self.pending))

    def checks(self):
        keys=('complete_duration','finite_bounds','forward_x','forward_encoders','left_yaw','right_yaw',
              'left_encoder_order','right_encoder_order','stop_after_one_second','final_still','outputs_accepted')
        result={k:False for k in keys};result['outputs_accepted']=not self.rejections and self.error is None
        if len(self.samples)!=10000 or len(self.motion)!=10000:return result
        result['complete_duration']=self.samples[-1][0]==10000000
        p=self.samples;m=self.motion;t=THRESHOLDS;x0,y0=p[0][12:14]
        result['finite_bounds']=all(math.hypot(s[12]-x0,s[13]-y0)<=t['max_travel_m'] and
            t['min_z_m']<=s[14]<=t['max_z_m'] and row.upright_cos>=t['min_upright_cos']
            for s,row in zip(p,m))
        def delta(a,b):return math.remainder(m[b-1].yaw_rad-m[a-1].yaw_rad,2*math.pi)
        def enc(a,b):return tuple(p[b-1][i]-p[a-1][i]for i in(3,4))
        a,b=1050,1950
        result['forward_x']=(p[b-1][12]-p[a-1][12]>t['min_forward_x_m'] and
            abs(p[b-1][13]-p[a-1][13])<t['max_straight_y_m'] and abs(delta(a,b))<t['max_straight_yaw_rad'])
        result['forward_encoders']=all(v>0 for v in enc(a,b))
        result['left_yaw']=delta(2050,2450)>t['min_turn_yaw_rad']
        result['right_yaw']=delta(2550,2950)<-t['min_turn_yaw_rad']
        left=enc(2050,2450);right=enc(2550,2950)
        result['left_encoder_order']=0<left[0]<left[1]
        result['right_encoder_order']=0<right[1]<right[0]
        def stopped(rows):return all(math.hypot(r.vx,r.vy)<t['stop_speed_m_s'] and abs(r.wz)<t['stop_yaw_rate_rad_s']for r in rows)
        result['stop_after_one_second']=stopped(m[3999:4200])
        result['final_still']=(stopped(m[8999:]) and
            math.hypot(p[-1][12]-p[8999][12],p[-1][13]-p[8999][13])<t['stop_drift_m'] and
            all(abs(v)*self.cpr/(2*math.pi)<t['stop_encoder_counts']for v in enc(9000,10000)))
        return result

    def observations(self):
        if len(self.motion)!=10000:return {}
        p=self.samples;m=self.motion
        return dict(forward_dx_m=p[1949][12]-p[1049][12],
            left_dyaw_rad=math.remainder(m[2449].yaw_rad-m[2049].yaw_rad,2*math.pi),
            right_dyaw_rad=math.remainder(m[2949].yaw_rad-m[2549].yaw_rad,2*math.pi),
            final_position_m=list(p[-1][12:15]),final_yaw_rad=m[-1].yaw_rad,
            total_encoder_counts=[(p[-1][i]-p[0][i])*self.cpr/(2*math.pi)for i in(3,4)],
            final_speed_m_s=math.hypot(m[-1].vx,m[-1].vy),
            max_speed_4_to_4_2s_m_s=max(math.hypot(r.vx,r.vy)for r in m[3999:4200]))

    def summary(self):
        result=super().summary();checks=self.checks()
        result.update(status='DRIVING_SIMULATION_COMPLETE'if all(checks.values())else'DRIVING_SIMULATION_INCOMPLETE',
            motors_applied=bool(self.motion),output_source='scripted real-codec OUTPUT; wheel velocity targets issued',
            mapping=dict(kind='PWM -> DM1 forward steady mm/s -> velocity rad/s',pwm_full_scale=PWM_FULL_SCALE,
                measured_source=DM1_SOURCE,measured_endpoints_mm_s={k:list(v)for k,v in DM1_FORWARD_MM_S.items()},
                interpolation='ASSUMED/UNMEASURED linear16..20%; no extrapolation; reverse/braking unmodelled',
                radius_mm=self.radius_mm,radius_provenance='ASSUMED/UNMEASURED existing robot profile',
                torque_limit_nm=self.torque_limit_nm,torque_provenance='ASSUMED/UNMEASURED existing robot profile'),
            smoke_thresholds_assumed_unmeasured=THRESHOLDS,checks=checks,observations=self.observations(),error=self.error,
            rejected_outputs=len(self.rejections),commands_issued=len(self.motion),
            scope='HOST SIMULATION ONLY; sign/stop smoke, not physical motor/feedback/timing calibration')
        return result

    def export(self,directory):
        super().export(directory);directory=Path(directory)
        with (directory/'motion.csv').open('w',newline='')as stream:
            writer=csv.writer(stream);writer.writerow(Motion._fields);writer.writerows(self.motion)
        (directory/'rejections.json').write_text(json.dumps(self.rejections,indent=2)+'\n')
        return self.summary()


def replay_export(directory):
    """Replay recorded observations/commands, not Webots physics or scheduling."""
    directory=Path(directory)
    # The passive replay implementation cannot validate a different OUTPUT script.
    # Apply the same bounded-file checks without pretending it was a passive run.
    import stat
    for name,bound in(('summary.json',100000),('samples.csv',8_000_000),('io.csv',8_000_000),
                       ('io.bin',1_240_000),('motion.csv',8_000_000),('rejections.json',100000)):
        info=(directory/name).stat()
        if not stat.S_ISREG(info.st_mode) or info.st_size>bound:raise ValueError('driving replay file budget/type')
    summary=json.loads((directory/'summary.json').read_text());mapping=summary['mapping']
    model=DrivingReplay(summary['counts_per_revolution'],mapping['radius_mm'],torque_limit_nm=mapping['torque_limit_nm'])
    with (directory/'samples.csv').open(newline='')as samples_file,(directory/'motion.csv').open(newline='')as motion_file:
        rows=csv.reader(samples_file);motion=csv.reader(motion_file)
        if next(rows,None)!=list(SAMPLE_FIELDS)or next(motion,None)!=list(Motion._fields):raise ValueError('driving replay schema')
        for sample in rows:
            if len(model.samples)>=10000:raise ValueError('driving replay row budget')
            row=next(motion,None)
            if len(sample)!=len(SAMPLE_FIELDS)or row is None or len(row)!=len(Motion._fields):raise ValueError('driving replay width')
            model.sample(*(int(v)for v in sample[:3]),tuple(float(v)for v in sample[3:15]),io_stamps=tuple(int(v)for v in sample[15:]))
            original=Motion(int(row[0]),int(row[1]),*(float(v)for v in row[2:10]),int(row[10]),
                            *(int(v)for v in row[11:13]),*(float(v)for v in row[13:]))
            model.observe_motion(original.yaw_rad,original.upright_cos,original[4:10],original.command_issued_raw_us)
            if model.motion[-1]!=original:raise ValueError('command/mapping replay mismatch')
        if next(motion,None)is not None:raise ValueError('extra motion row')
    if json.loads((directory/'rejections.json').read_text())or model.summary()!=summary or model.wire!=(directory/'io.bin').read_bytes():
        raise ValueError('driving byte/counter/check replay mismatch')
    with (directory/'io.csv').open(newline='')as f:
        reader=csv.reader(f)
        if next(reader,None)!=list(IO_FIELDS):raise ValueError('driving I/O schema')
        for expected in model.io_rows():
            if next(reader,None)!=[str(v)for v in expected]:raise ValueError('driving I/O order')
        if next(reader,None)is not None:raise ValueError('extra driving I/O row')
    return dict(byte_match=True,samples=len(model.samples),io_sha256=summary['io_sha256'],checks=model.checks(),
                scope='recorded-input command replay; NOT deterministic physics or wall scheduling')
