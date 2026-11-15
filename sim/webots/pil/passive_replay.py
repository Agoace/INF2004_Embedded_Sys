"""Bounded passive simulation records. No serial, motor API or timing acceptance."""
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import stat
from .wire import Frame,Type,NO_ACK,encode,decode

SESSION=0x53494c31
MAX_SAMPLES=10000
SAMPLE_FIELDS=('sim_us','raw_start_us','raw_end_us','left_rad','right_rad','servo_rad',
               'accel_x','accel_y','accel_z','compass_x','compass_y','compass_z','pose_x','pose_y','pose_z',
               'sensor_encoded_raw_us','output_consumed_raw_us')
IO_FIELDS=('order','direction','slot','source_us','sim_us','raw_start_us','raw_end_us','event_raw_us','offset','length')
POLICY={'nominal_phase_us':250,'max_phase_us':600,'window_slots':1000,
        'max_late_in_window':3,'max_consecutive_late':2,'fatal':False,
        'reference':'unpaced RAW wall versus 1ms source reference; NOT a qualified device clock'}


def scripted_output(slot):
    """Nonzero commands deliberately demonstrate that consumption is not actuation."""
    payload=bytearray(32)
    pwm=1200 if (slot//250)%2==0 else -1200
    struct.pack_into('<hhH',payload,0,pwm,-pwm,1500)
    payload[7]=1
    return encode(Frame(Type.OUTPUT,SESSION,slot,slot*1000,slot,bytes(payload)))


class PassiveReplay:
    OUTPUT_DIRECTION='OUTPUT_IGNORED'

    def output_packet(self,slot):return scripted_output(slot)

    def consume_output(self,frame):
        """Passive default intentionally does not actuate anything."""

    def __init__(self,counts_per_revolution,*,max_samples=MAX_SAMPLES):
        if type(counts_per_revolution)not in(int,float) or not math.isfinite(counts_per_revolution) or counts_per_revolution<=0:
            raise ValueError('invalid encoder profile')
        if type(max_samples)is not int or not 1<=max_samples<=MAX_SAMPLES:raise ValueError('invalid sample bound')
        self.cpr=counts_per_revolution;self.max_samples=max_samples
        self.samples=[];self.wire=bytearray();self.origin=None;self.previous_counts=(0,0)
        self.late_slots=deque();self.burst=0;self.last_late=None
        self.diagnostics={key:0 for key in('early_count','positive_late_count','phase_over250_count',
            'phase_over600_count','rolling_quota_excess_count','burst_excess_count','max_rolling_late',
            'max_consecutive_late','max_phase_us')}

    def sample(self,sim_us,raw_start_us,raw_end_us,values,*,io_clock=None,io_stamps=None):
        slot=len(self.samples)
        if slot>=self.max_samples:raise ValueError('sample budget exhausted')
        if any(type(x)is not int for x in(sim_us,raw_start_us,raw_end_us)) or min(sim_us,raw_start_us)<0 or raw_end_us<raw_start_us:
            raise ValueError('invalid sample clocks')
        if len(values)!=12 or any(not math.isfinite(x) for x in values):raise ValueError('nonfinite/invalid sensor sample')
        if slot and (sim_us!=self.samples[-1][0]+1000 or raw_start_us<self.samples[-1][-1]):
            raise ValueError('skipped simulation step or backward RAW clock')
        if self.origin is None:self.origin=tuple(values[:2])
        counts=tuple(round((values[i]-self.origin[i])*self.cpr/(2*math.pi))for i in(0,1))
        payload=bytearray(40);valid=0
        for i,count in enumerate(counts):
            struct.pack_into('<i',payload,8+4*i,((count+2**31)%2**32)-2**31)
            if abs(count-self.previous_counts[i])<=16:valid|=1<<(3+i)
        # Only encoder channels are modelled on the wire. All other VALID/NEW,
        # including physical timing validity, remain clear. Raw devices are logged.
        struct.pack_into('<H',payload,36,valid)
        sensor=encode(Frame(Type.SENSOR,SESSION,slot,slot*1000,slot-1 if slot else NO_ACK,bytes(payload)))
        encoded_at=io_clock() if io_clock is not None else raw_end_us
        output=self.output_packet(slot)
        consumed=decode(output)
        if consumed.type!=Type.OUTPUT or consumed.ack!=slot or consumed.session!=SESSION:raise ValueError('scripted OUTPUT order/session')
        self.consume_output(consumed)
        consumed_at=io_clock() if io_clock is not None else raw_end_us
        if io_stamps is not None:encoded_at,consumed_at=io_stamps
        if (type(encoded_at)is not int or type(consumed_at)is not int or
                not raw_end_us<=encoded_at<=consumed_at):raise ValueError('invalid I/O event clocks')
        self.wire.extend(sensor);self.wire.extend(output)
        self.samples.append((sim_us,raw_start_us,raw_end_us,*values,encoded_at,consumed_at));self.previous_counts=counts
        target=self.samples[0][1]+slot*1000
        start,end=raw_start_us-target,raw_end_us+1-target
        phase=max(abs(start),abs(end));d=self.diagnostics
        d['max_phase_us']=max(d['max_phase_us'],phase)
        d['early_count']+=start<0;d['phase_over250_count']+=phase>250;d['phase_over600_count']+=phase>600
        while self.late_slots and self.late_slots[0]<=slot-1000:self.late_slots.popleft()
        # An early fast-mode sample is counted separately, never called tolerated late.
        if start>=0 and end>250:
            self.burst=self.burst+1 if self.last_late==slot-1 else 1
            self.last_late=slot;self.late_slots.append(slot);d['positive_late_count']+=1
            d['rolling_quota_excess_count']+=len(self.late_slots)>3
            d['burst_excess_count']+=self.burst>2
            d['max_consecutive_late']=max(d['max_consecutive_late'],self.burst)
        else:self.burst=0
        d['max_rolling_late']=max(d['max_rolling_late'],len(self.late_slots))

    def io_rows(self):
        offset=0
        for slot,sample in enumerate(self.samples):
            for index,(direction,length) in enumerate((('SENSOR',66),(self.OUTPUT_DIRECTION,58))):
                yield (slot*2+index,direction,slot,slot*1000,*sample[:3],sample[15+index],offset,length)
                offset+=length

    def summary(self):
        return dict(status='PASSIVE_SIMULATION_COMPLETE',simulation_only=True,hardware_used=False,
            serial_used=False,motors_applied=False,samples=len(self.samples),
            simulation_start_us=0,simulation_end_us=self.samples[-1][0] if self.samples else None,
            source_epoch_sim_us=self.samples[0][0] if self.samples else None,
            source_step_us=1000,raw_clock='CLOCK_MONOTONIC_RAW',
            output_source='scripted, decoded, ignored; not firmware control or handshake',
            wire_channels='encoder totals only; other channels and physical timing INVALID',
            counts_per_revolution=self.cpr,diagnostic_policy=POLICY,diagnostics=self.diagnostics,
            io_sha256=hashlib.sha256(self.wire).hexdigest(),
            scope='PASSIVE SIMULATION ONLY; no device clock, physical timing or driving claim')

    def export(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
        for name,fields,rows in(('samples.csv',SAMPLE_FIELDS,self.samples),('io.csv',IO_FIELDS,self.io_rows())):
            with (directory/name).open('w',newline='')as stream:
                writer=csv.writer(stream);writer.writerow(fields);writer.writerows(rows)
        (directory/'io.bin').write_bytes(self.wire)
        result=self.summary();(directory/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
        return result


def replay_export(directory):
    """Rebuild every packet/counter from recorded inputs, without Webots or motors."""
    directory=Path(directory)
    for name,bound in(('summary.json',100000),('samples.csv',8_000_000),('io.csv',8_000_000),('io.bin',1_240_000)):
        info=(directory/name).stat()
        if not stat.S_ISREG(info.st_mode) or info.st_size>bound:raise ValueError('replay file type/budget exceeded')
    summary=json.loads((directory/'summary.json').read_text())
    model=PassiveReplay(summary['counts_per_revolution'])
    with (directory/'samples.csv').open(newline='')as stream:
        reader=csv.reader(stream)
        if next(reader,None)!=list(SAMPLE_FIELDS):raise ValueError('sample schema mismatch')
        for row in reader:
            if len(row)!=len(SAMPLE_FIELDS):raise ValueError('sample width mismatch')
            model.sample(*(int(x)for x in row[:3]),tuple(float(x)for x in row[3:15]),
                         io_stamps=tuple(int(x)for x in row[15:17]))
    if model.wire!=(directory/'io.bin').read_bytes() or model.summary()!=summary:
        raise ValueError('deterministic packet/counter replay mismatch')
    with (directory/'io.csv').open(newline='')as stream:
        reader=csv.reader(stream)
        if next(reader,None)!=list(IO_FIELDS):raise ValueError('I/O schema mismatch')
        for expected in model.io_rows():
            if next(reader,None)!=[str(x)for x in expected]:raise ValueError('I/O order mismatch')
        if next(reader,None)is not None:raise ValueError('extra I/O row')
    return dict(byte_match=True,samples=len(model.samples),io_sha256=summary['io_sha256'],
                scope='recorded-input replay only; wall scheduling not reproduced')
