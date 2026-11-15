"""Bounded post-fault evidence, never a session restart or estimator input."""
from array import array
import json
from pathlib import Path
import re
import struct
from .wire import StreamDecoder, Type
from .gate1_diagnostics import decode_dump, DiagnosticError

class CaptureStopped(Exception):
    """Owned capture process requested to close/export; never a success verdict."""
    def __init__(self, signum):
        self.signum = signum
        super().__init__(f'signal {signum}')


STOPPED = re.compile(rb'(?m)^\[PILG1\] stopped; reset required; worker stack untouched [^\r\n]*\r?\n')
RESET = re.compile(rb'(?m)^\[PILG1RESET1 ((?:[0-9a-f]{8} ){11}[0-9a-f]{8})\]\n')


def parse_reset_records(data):
    records=[]
    for m in RESET.finditer(data):
        v=[int(x,16)for x in m[1].split(b' ')]
        if v[1]!=0:continue  # Immutable boot session, not a live D1 reason.
        records.append(dict(nonce=v[0],session=v[1],watchdog_reason=v[2],chip_reset=v[3],
                            scratch=v[4:],offset=m.start(),length=m.end()-m.start()))
    return records


BREADCRUMB = re.compile(rb'(?m)^\[PILG1BC1 ((?:[0-9a-f]{8} ){29}[0-9a-f]{8})\]\n')
BC_MAGIC, BC_PRODUCER, BC_SALT = 0x50424731, 0x00010001, 0x9e3779b9


def _breadcrumb_mask(w, reason):
    """Mirror the frozen native independent-publication integrity checks."""
    if ((reason & 3) == 0 or w[0] != BC_MAGIC or w[1] != BC_PRODUCER or not w[2] or
            w[3] != (BC_SALT ^ w[0] ^ w[1] ^ w[2])):
        return 0
    mask = 1
    for base in (4, 8, 12):
        if (w[base] and (w[base] & 1) == 0 and
                w[base+3] == (BC_SALT ^ base ^ w[base] ^ w[base+1] ^ w[base+2])):
            mask |= 1 << (base//4)
    if 1 <= w[16] <= 3: mask |= 32
    check = BC_SALT ^ w[16]
    for word in w[17:26]: check ^= word
    if w[16] in (2, 3) and w[26] == check and w[27] == (check ^ 0xffffffff):
        mask |= 16
        if w[16] == 2: mask |= 64
    return mask


def parse_breadcrumb_records(data, *, watchdog_reason=None):
    """Strict BC1 framing/checks. Missing RESET1 reason context is UNKNOWN.

    Explicit reason is for standalone retained fixtures/records. CDC instead
    requires the preceding same-nonce RESET1. Invalid publications stay null;
    raw previous words remain available and are never promoted to live state.
    """
    if watchdog_reason is not None and (type(watchdog_reason) is not int or not 0 <= watchdog_reason <= 0xffffffff):
        raise ValueError('watchdog reason must be uint32')
    resets = parse_reset_records(data) if watchdog_reason is None else []
    records = []
    for match in BREADCRUMB.finditer(data):
        values = [int(word, 16) for word in match[1].split(b' ')]
        nonce, mask, w = values[0], values[1], values[2:]
        reason = watchdog_reason
        if reason is None:
            preceding = [r for r in resets if r['nonce'] == nonce and r['offset'] < match.start()]
            if not preceding: continue
            reason = preceding[-1]['watchdog_reason']
        if mask & ~0x7f or mask != _breadcrumb_mask(w, reason): continue
        motion = (dict(sequence=w[4], time_us=w[5], caller_task_id=w[6]) if mask & 2 else None)
        pil = None
        if mask & 4:
            packed = w[10]
            pil = dict(sequence=w[8], time_us=w[9], packed_tid_state=packed,
                       task_id=packed >> 8 if packed != 0xffffffff else None,
                       state=packed & 255 if packed != 0xffffffff else None,
                       state_name={1:'ENTER', 2:'BEFORE_SLEEP'}.get(packed & 255, 'UNKNOWN'))
        tick = (dict(sequence=w[12], entry_time_us=w[13], thread_task_id=w[14]) if mask & 8 else None)
        fault = None
        if mask & 1:
            complete = bool(mask & 16)
            fault = dict(status_raw=w[16], entry_marker_observed=bool(mask & 32),
                         integrity_valid=complete, frame_valid=bool(mask & 64),
                         time_us=w[17] if complete else None,
                         frame=dict(pc=w[18], lr=w[19], xpsr=w[20]) if mask & 64 else None,
                         original_sp=w[21] if complete else None,
                         exc_return=w[22] if complete else None,
                         primask=w[23] if complete else None, ipsr=w[24] if complete else None,
                         core_id=w[25] if complete else None)
        records.append(dict(nonce=nonce, validity_mask=mask, raw_previous_words=w,
            previous_owner_nonce=w[2] if mask & 1 else None,
            producer_tag=w[1] if mask & 1 else None, header_status='VALID' if mask & 1 else 'UNKNOWN',
            watchdog_reason=reason, motion=motion, pil=pil, alarm0_tick=tick, fault=fault,
            offset=match.start(), length=match.end()-match.start(),
            scope='previous boot only; null fields UNKNOWN; no old/new TIMER subtraction or no-fault inference'))
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
    means a reset+BC1 companion after reconnect, a bounded missing-companion
    wait, or stopped D1+stack plus1s reset watch; never a PASS.
    """
    if not 1_000<=duration_us<=700_000_000:raise ValueError('invalid CDC capture duration')
    if type(max_segments)is not int or not 1<=max_segments<=16:raise ValueError('invalid segment budget')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    labels=labels or {};events=[];segments=[];deadline=clock.now_us()+duration_us
    reason='deadline; incomplete';io=None;stream=None;data=bytearray();index=-1
    stopped_seen=None
    signal_received=None
    stop_grace_us=1000000
    companion_wait_us=1000000
    reopened_reset_seen=None
    try:
        while clock.now_us()<deadline:
            if reopened_reset_seen is not None and clock.now_us()>=reopened_reset_seen+companion_wait_us:
                reason='post-reopen boot record captured; breadcrumb UNKNOWN';break
            if reopened_reset_seen is None and stopped_seen is not None and clock.now_us()>=stopped_seen+stop_grace_us:
                reason='stopped D1 and stack plus1s reset-watch grace';break
            if io is None:
                try:io=open_io()
                except OSError as e:
                    if len(events)<4096:events.append(dict(event='open_unavailable',pc_us=clock.now_us(),error=str(e)))
                    clock.sleep_us(10000);continue
                index+=1
                if index>=max_segments:reason='segment budget exhausted';break
                data=bytearray();name=f'pico-usb-{index:03d}.bin';stream=(output/name).open('xb',buffering=0)
                segments.append(dict(file=name,opened_pc_us=clock.now_us(),bytes=0,reset_records=[],chunks=[],
                                     breadcrumb_records=[],breadcrumb_status='UNKNOWN'))
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
            if stream.write(chunk)!=len(chunk):
                reason='short raw evidence write; incomplete';break
            data.extend(chunk);segments[-1]['bytes']=len(data)
            records=parse_reset_records(data)
            for record in records:
                end=record['offset']+record['length']
                record['complete_pc_us']=next(t for t,start,size in segments[-1]['chunks'] if start<end<=start+size)
            segments[-1]['reset_records']=records
            breadcrumbs=parse_breadcrumb_records(data)
            for record in breadcrumbs:
                end=record['offset']+record['length']
                record['complete_pc_us']=next(t for t,start,size in segments[-1]['chunks'] if start<end<=start+size)
            segments[-1]['breadcrumb_records']=breadcrumbs
            matched=[r for r in breadcrumbs if records and r['nonce']==records[-1]['nonce'] and r['offset']>records[-1]['offset']]
            segments[-1]['breadcrumb_status']='COMPLETE_ROW' if matched else 'UNKNOWN'
            if len(data)>4_000_000 or len(segments[-1]['chunks'])>=4096:reason='segment byte/chunk budget exhausted';break
            if index>0 and segments[-1]['reset_records']:
                if reopened_reset_seen is None:reopened_reset_seen=records[0]['complete_pc_us']
                if matched:
                    reason='post-reopen reset and breadcrumb companion captured';break
            if STOPPED.search(data):
                try:decode_dump(bytes(data))
                except DiagnosticError:pass
                else:
                    if stopped_seen is None:stopped_seen=clock.now_us()
    except CaptureStopped as error:
        signal_received=error.signum
        reason=f'signal {error.signum}; capture interrupted; incomplete'
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
                    signal_received=signal_received,post_reopen_companion_wait_us=companion_wait_us,
                    reopened_reset_seen_pc_us=reopened_reset_seen,
                    breadcrumb_record_scope='strict281B BC1; matching RESET1 reason/nonce; independent native checks; previous boot only; missing/invalid UNKNOWN',
                    pc_clock_domain=getattr(clock,'domain','INJECTED_TEST_CLOCK'),
                    reset_record_scope='complete122B LF row only; enqueue is not delivery ACK; partial/lost UNKNOWN; raw scratch may include ROM effects')
        (output/'cdc-capture.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
