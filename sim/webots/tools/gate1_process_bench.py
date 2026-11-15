#!/usr/bin/env python3
"""No-device before/after recorder + complete frozen-loop CPU cost benchmark."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import types

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'sim/webots'))
from pil import gate1
from pil.gate1_replay import load_fixture,ReplayClock,RecordedUART
from pil.gate1_process_log import ProcessLog,legacy_details

BEFORE='edc9dfb495551f0f1357b739e12cb6bbcf112560'
FIXTURE=ROOT/'sim/webots/tests/fixtures/pil_gate1_cap4_synthetic_replay.json'
RAW=lambda:time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def old_module(name,path):
    source=subprocess.check_output(['git','show',f'{BEFORE}:{path}'],cwd=ROOT)
    module=types.ModuleType('pil.'+name);module.__package__='pil';sys.modules[module.__name__]=module
    exec(compile(source,str(path)+'@'+BEFORE,'exec'),module.__dict__)
    return module


def percentiles(rows):
    rows=sorted(rows)
    return dict(n=len(rows),p50_us=rows[math.ceil(len(rows)*.5)-1]/1000,
                p99_us=rows[math.ceil(len(rows)*.99)-1]/1000,max_us=rows[-1]/1000)


class BenchClock(ReplayClock):
    def __init__(self,start,active_from):
        super().__init__(start);self.active_from=active_from;self.wall=[];self.cpu=[];self.active_wall=[];self.active_cpu=[];self.last=None
    def sleep_us(self,delay):
        wall=RAW();cpu=time.thread_time_ns()
        if self.last is not None:
            dw,dc=wall-self.last[0],cpu-self.last[1];self.wall.append(dw);self.cpu.append(dc)
            if self.now>=self.active_from:self.active_wall.append(dw);self.active_cpu.append(dc)
        super().sleep_us(delay);self.last=(RAW(),time.thread_time_ns())


def comparable(output,summary):
    with (output/'events.csv').open(newline='')as stream:rows=list(csv.DictReader(stream))
    for row in rows:
        details=json.loads(row['details_json'])
        if row['event']=='run_metadata':details['pc_logging']='MODE'
        row['details_json']=json.dumps(details,sort_keys=True)
    normalized=dict(summary);normalized.pop('pc_logging')
    return rows,normalized


def run(output,repeats):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'build'):raise ValueError('bench output must be below owned build/')
    output.mkdir(parents=True,exist_ok=False)
    before=old_module('_bench_before_gate1','sim/webots/pil/gate1.py')
    before.HostTransport=old_module('_bench_before_transport','sim/webots/pil/transport.py').HostTransport
    fixture=load_fixture(FIXTURE)
    from pil.wire import decode,Type
    active_from=next(row['pc_us']for row in fixture['tx']if decode(bytes.fromhex(row['hex'])).type==Type.SENSOR)
    report=dict(before_revision=BEFORE,fixture_sha256=hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
        repeats=repeats,scope='HOST CPU/fake-clock replay only; not hardware or a scheduling bound',
        timing='RAW wall + thread CPU; setup/close and fake sleep excluded; unchanged observer in both arms',modes={})
    reference=None
    for label,module in(('before',before),('after',gate1)):
        all_wall=[];all_cpu=[];active_wall=[];active_cpu=[];producer_wall=[];producer_cpu=[]
        original=module.Recorder.record;current_clock=None;current_io=None
        def measured(self,*args,**kwargs):
            if current_clock is None or current_clock.last is None or current_io.closed:
                return original(self,*args,**kwargs)
            w,c=RAW(),time.thread_time_ns()
            try:return original(self,*args,**kwargs)
            finally:
                producer_wall.append(RAW()-w);producer_cpu.append(time.thread_time_ns()-c)
        module.Recorder.record=measured
        try:
            for index in range(repeats):
                clock=BenchClock(fixture['start_pc_us'],active_from);io=RecordedUART(fixture,clock);path=output/f'{label}-{index}'
                current_clock,current_io=clock,io
                summary=module.run_soak(io,path,clock=clock,duration_us=fixture['duration_us'],session=fixture['session'],transport_only=True)
                if not io.complete or summary['verdict']!='INCOMPLETE':raise ValueError('frozen stream/verdict mismatch')
                data=comparable(path,summary)
                if reference is None:reference=data
                if data!=reference:raise ValueError('artifact semantic mismatch beyond logging-mode metadata')
                all_wall.extend(clock.wall);all_cpu.extend(clock.cpu);active_wall.extend(clock.active_wall);active_cpu.extend(clock.active_cpu)
        finally:module.Recorder.record=original
        report['modes'][label]=dict(all_loop_wall=percentiles(all_wall),all_loop_thread_cpu=percentiles(all_cpu),
            active_loop_wall=percentiles(active_wall),active_loop_thread_cpu=percentiles(active_cpu),
            record_wall=percentiles(producer_wall),record_thread_cpu=percentiles(producer_cpu))
    # Fixed host-only mixed record workload. Baseline receives legacy HEX strings;
    # preparing them is OUTSIDE the timed microbench (conservative baseline).
    dataset=[('source_capture',123,dict(source_us=1000,capture_end_pc_us=124,phase_us=50.5,
        uncertainty_us=200.,source_lower_pico_us=999900.,source_upper_pico_us=1000100.,no_early_midpoint=True)),
        ('raw_rx',124,dict(packet_hex=bytes(range(256))*16)),
        ('pc_write',125,dict(type=4,seq=1,offset=0,bytes=66,device_available=False)),
        ('rx_complete',126,dict(type=5,seq=1,source_us=1000,session=123,ack=1,payload_hex=bytes(32)))]
    for label in('before','after'):
        path=output/f'cpu-{label}';path.mkdir()
        sink=before.Recorder(path/'records')if label=='before'else ProcessLog(path)
        rows=[(e,p,legacy_details(d)if label=='before'else d)for e,p,d in dataset]
        walls=[];cpus=[];backlog=0
        for n in range(2000):
            event,pc,details=rows[n%len(rows)];w,c=RAW(),time.thread_time_ns()
            if label=='before':sink.record(event,pc,**details);sink.flush()
            else:sink.record(event,pc,details);sink.poll()
            walls.append(RAW()-w);cpus.append(time.thread_time_ns()-c)
            if label=='after':backlog=max(backlog,sink.sent-sum(r[3].value for r in sink.rings))
        if label=='before':sink.stream.close()
        else:
            meta=sink.finish(127,serial_closed=True,inhibited=True)
            if not meta['evidence_complete']:raise ValueError('CPU bench recorder incomplete')
        report['modes'][label]['mixed_record_wall']=percentiles(walls)
        report['modes'][label]['mixed_record_thread_cpu']=percentiles(cpus)
        if label=='after':report['recorder_cpu_bench']=dict(max_observed_backlog=backlog,**meta)
    # Compare literal legacy CSV bytes after changing ONLY run_metadata logging mode.
    def normalized_bytes(path):
        with path.open(newline='')as stream:rows=list(csv.reader(stream))
        for row in rows[1:]:
            if row[0]=='run_metadata':
                d=json.loads(row[2]);d['pc_logging']='MODE';row[2]=json.dumps(d,sort_keys=True)
        import io
        stream=io.StringIO(newline='');csv.writer(stream).writerows(rows);return stream.getvalue().encode()
    before_bytes=(output/'before-0/events.csv').read_bytes();after_bytes=(output/'after-0/events.csv').read_bytes()
    report['artifact_equality']=dict(semantic_equal=True,literal_csv_byte_equal=before_bytes==after_bytes,
        nonmetadata_rows_byte_equal=[r for r in before_bytes.splitlines(keepends=True)if not r.startswith(b'run_metadata,')]==
                                   [r for r in after_bytes.splitlines(keepends=True)if not r.startswith(b'run_metadata,')],
        normalized_csv_byte_equal=
        normalized_bytes(output/'before-0/events.csv')==normalized_bytes(output/'after-0/events.csv'),
        intentional_difference='pc_logging metadata only; additive binary spool/close/recorder sidecars')
    (output/'bench.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n');return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=10);args=parser.parse_args()
    if not 1<=args.repeats<=30:parser.error('repeats1..30')
    print(json.dumps(run(args.output,args.repeats),indent=2,sort_keys=True))


if __name__=='__main__':main()
