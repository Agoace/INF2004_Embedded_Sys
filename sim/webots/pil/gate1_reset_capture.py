"""Bounded post-fault evidence, never a session restart or estimator input."""
from array import array
import json
from pathlib import Path
import re
import struct
from .wire import StreamDecoder, Type
from .gate1_diagnostics import decode_dump, DiagnosticError

RESET = re.compile(rb'(?m)^\[PILG1RESET1 ((?:[0-9a-f]{8} ){11}[0-9a-f]{8})\]\n')


def parse_reset_records(data):
    records=[]
    for m in RESET.finditer(data):
        v=[int(x,16)for x in m[1].split(b' ')]
        if v[1]!=0:continue  # Immutable boot session, not a live D1 reason.
        records.append(dict(nonce=v[0],session=v[1],watchdog_reason=v[2],chip_reset=v[3],
                            scratch=v[4:],offset=m.start(),length=m.end()-m.start()))
    return records


class FailureTail:
    def __init__(self,clock,requested_us,reason,capacity=131072):
        self.clock,self.requested_us,self.reason=clock,requested_us,reason
        self.buffer=bytearray(capacity);self.used=0
        self.stamps=array('q',[0])*3000  # at most1000 chunks, three words each
        self.chunks=0;self.error=None;self.full=False;self.start=self.end=0

    def collect(self,io):
        self.start=self.clock.now_us();deadline=self.start+self.requested_us
        while self.clock.now_us()<deadline:
            try:data=io.read(4096)
            except OSError as e:self.error=str(e);break
            stamp=self.clock.now_us()
            if data:
                if self.used+len(data)>len(self.buffer)or self.chunks>=1000:
                    self.full=True;break
                self.buffer[self.used:self.used+len(data)]=data
                p=self.chunks*3
                self.stamps[p:p+3]=array('q',(stamp,self.used,len(data)))
                self.used+=len(data);self.chunks+=1
            # RX-only post-failure pacing, NOT the source-loop sleep policy.
            self.clock.sleep_us(1000)
        self.end=self.clock.now_us()

    def finish(self,directory,labels,*,serial_closed):
        data=bytes(self.buffer[:self.used]);(directory/'post-fail-rx.bin').write_bytes(data)
        decoder=StreamDecoder();boots=[];chunks=[]
        for i in range(self.chunks):
            stamp,start,size=self.stamps[i*3:i*3+3];chunks.append([stamp,start,size])
            for frame in decoder.feed(data[start:start+size]):
                if frame.type==Type.BOOT and len(frame.payload)==16 and len(boots)<32:
                    boots.append(dict(pc_us=stamp,nonce=struct.unpack_from('<I',frame.payload)[0],
                        sequence=frame.sequence,session=frame.session))
        meta=dict(labels,original_failure=self.reason,requested_us=self.requested_us,
            start_pc_us=self.start,end_pc_us=self.end,actual_us=self.end-self.start,
            bytes=self.used,byte_capacity=len(self.buffer),full=self.full,error=self.error,
            chunks=chunks,boots=boots,decoder_bad_frames=decoder.bad_frames,
            serial_closed_before_export=serial_closed,scope='RX ONLY; no readiness/lease/clock credit; original FAIL retained')
        (directory/'post-fail-rx.json').write_text(json.dumps(meta,indent=2)+'\n')
        return meta


def capture_cdc(open_io,output,clock,*,duration_us=700000000,labels=None,max_segments=16):
    """Injected opener is ONLY for the separately named Pico USB CDC, never UART.

    Each reopen creates a new segment. No write method is called. Completion
    means a reset record after reconnect, or stopped D1+stack plus1s reset
    watch; not a PASS. D1 completion alone must not hide a later reset.
    """
    if not 1_000<=duration_us<=700_000_000:raise ValueError('invalid CDC capture duration')
    if type(max_segments)is not int or not 1<=max_segments<=16:raise ValueError('invalid segment budget')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    labels=labels or {};events=[];segments=[];deadline=clock.now_us()+duration_us
    reason='deadline; incomplete';io=None;stream=None;data=bytearray();index=-1
    stopped_seen=None
    stop_grace_us=1000000
    try:
        while clock.now_us()<deadline:
            if stopped_seen is not None and clock.now_us()>=stopped_seen+stop_grace_us:
                reason='stopped D1 and stack plus1s reset-watch grace';break
            if io is None:
                try:io=open_io()
                except OSError as e:
                    if len(events)<4096:events.append(dict(event='open_unavailable',pc_us=clock.now_us(),error=str(e)))
                    clock.sleep_us(10000);continue
                index+=1
                if index>=max_segments:reason='segment budget exhausted';break
                data=bytearray();name=f'pico-usb-{index:03d}.bin';stream=(output/name).open('xb')
                segments.append(dict(file=name,opened_pc_us=clock.now_us(),bytes=0,reset_records=[],chunks=[]))
            try:
                chunk=io.read(4096)
                if not chunk:raise OSError('CDC EOF/disconnected')
            except BlockingIOError:
                clock.sleep_us(10000);continue
            except OSError as e:
                segments[-1].update(closed_pc_us=clock.now_us(),close_reason=str(e))
                io.close();io=None;stream.close();stream=None;continue
            stamp=clock.now_us()
            segments[-1]['chunks'].append([stamp,len(data),len(chunk)])
            stream.write(chunk);data.extend(chunk);segments[-1]['bytes']=len(data)
            records=parse_reset_records(data)
            for record in records:
                end=record['offset']+record['length']
                record['complete_pc_us']=next(t for t,start,size in segments[-1]['chunks'] if start<end<=start+size)
            segments[-1]['reset_records']=records
            if len(data)>4_000_000 or len(segments[-1]['chunks'])>=4096:reason='segment byte/chunk budget exhausted';break
            if index>0 and segments[-1]['reset_records']:
                reason='post-reopen boot record captured';break
            if b'[PILG1] stopped; reset required; worker stack untouched 'in data and data.endswith(b'\n'):
                try:decode_dump(data)
                except DiagnosticError:pass
                else:
                    if stopped_seen is None:stopped_seen=clock.now_us()
    finally:
        if io is not None:
            try:io.close()
            except OSError as e:
                reason='CDC close failed; incomplete'
                events.append(dict(event='close_error',error=str(e),pc_us=clock.now_us()))
        if stream is not None:stream.close()
        if segments:segments[-1].setdefault('closed_pc_us',clock.now_us())
        result=dict(labels,scope='Pico CDC RX/reopen only; NOT Gate1 acceptance',
                    reason=reason,segments=segments,events=events,
                    stopped_seen_pc_us=stopped_seen,post_stop_reset_watch_us=stop_grace_us,
                    pc_clock_domain=getattr(clock,'domain','INJECTED_TEST_CLOCK'),
                    reset_record_scope='complete122B LF row only; enqueue is not delivery ACK; partial/lost UNKNOWN; raw scratch may include ROM effects')
        (output/'cdc-capture.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
