"""Bounded binary evidence handoff. No serial, threads or JSON on producer path."""
import csv
import json
import multiprocessing as mp
import os
import signal
from pathlib import Path
import struct
import time
import zlib

SLOT_BYTES=8192
HEADER=struct.Struct('<4sIQII')  # magic, version, global sequence, used bytes, CRC32
CLOSE=struct.Struct('<4sIQQQII')
U16=struct.Struct('<H');I64=struct.Struct('<q');U64=struct.Struct('<Q');F64=struct.Struct('<d')
# Stable version1 schema IDs. Absent/None/bool/int/float remain distinguishable.
FAST_SCHEMAS=(
    ('raw_rx',(), 'packet_hex'),('raw_rx_after_disarm',(),'packet_hex'),
    ('startup_discard',('bytes','used_for_bootstrap'),'packet_hex'),
    ('raw_pc_write',('device_available','best_effort_stop'),'packet_hex'),
    ('rx_complete',('type','seq','source_us','session','ack'),'payload_hex'),
    ('pc_write',('type','seq','offset','bytes','device_available'),None),
    ('tx_queued',('type','seq'),None),
    ('source_capture',('source_us','capture_end_pc_us','source_lower_pico_us','source_upper_pico_us',
        'phase_us','uncertainty_us','start_midpoint_error_us','no_early_midpoint'),None),
    ('source_clock_bounds',('source_us','capture_end_pc_us','lower_pico_us','upper_pico_us','intended_pico_us'),None),
    ('clock_bounds',('lower_us','upper_us'),None),
    ('io_write_timing',('call_enter_pc_us','bytes','stream_start','stream_end','best_effort_stop'),None))
FAST_BY_EVENT={name:(index,fields,raw,frozenset((*fields,*((raw,)if raw else()))))
               for index,(name,fields,raw)in enumerate(FAST_SCHEMAS)}
FAST_HEAD=struct.Struct('<BHHHHHq')

class EvidenceFailure(OSError):pass


class BinaryRecordCodec:
    """Version1 typed cells inside a FIXED8192-byte slot; bounded depth/cell count.

    Bytes represent legacy *_hex fields and become hex ONLY in the child.
    int/float/bool/None distinctions survive JSON rendering exactly.
    """
    def __init__(self):self.symbols={};self.formats={}
    def _fast(self,event,pc,details):
        schema=FAST_BY_EVENT.get(event)
        if schema is None or type(pc)is not int:return False
        index,fields,raw,allowed=schema
        if any(k not in allowed for k in details):return False
        payload=details.get(raw,b'')if raw else b''
        if raw and not isinstance(payload,(bytes,bytearray)):return False
        present=floats=booleans=nones=0;values=[]
        for i,key in enumerate(fields):
            v=details.get(key);bit=1<<i
            if key in details:present|=bit
            if v is None:nones|=bit;values.append(0)
            elif type(v)is bool:booleans|=bit;values.append(v)
            elif type(v)is float:floats|=bit;values.append(v)
            elif isinstance(v,int)and -2**63<=v<2**63:values.append(v)
            else:return False
        if raw in details:present|=1<<len(fields)
        key=(index,floats);fmt=self.formats.get(key)
        if fmt is None:
            fmt=struct.Struct(FAST_HEAD.format+''.join('d'if floats&(1<<i)else'q'for i in range(len(fields))))
            if len(self.formats)<128:self.formats[key]=fmt
        self._need(fmt.size+(2+len(payload)if raw else 0))
        fmt.pack_into(self.buf,self.pos,10,index,present,floats,booleans,nones,pc,*values);self.pos+=fmt.size
        if raw:
            U16.pack_into(self.buf,self.pos,len(payload));self.pos+=2
            self.buf[self.pos:self.pos+len(payload)]=payload;self.pos+=len(payload)
        return True
    def _need(self,n):
        if self.pos+n>self.limit:raise ValueError('binary record exceeds fixed slot')
    def _byte(self,n):self._need(1);self.buf[self.pos]=n;self.pos+=1
    def _scalar(self,fmt,v):self._need(8);fmt.pack_into(self.buf,self.pos,v);self.pos+=8
    def _value(self,v,depth=0):
        self.cells+=1
        if depth>12 or self.cells>512:raise ValueError('binary record depth/cell budget')
        if v is None:self._byte(0)
        elif type(v)is bool:self._byte(2 if v else 1)
        elif isinstance(v,int):self._byte(3 if v<2**63 else 9);self._scalar(I64 if v<2**63 else U64,v)
        elif type(v)is float:self._byte(4);self._scalar(F64,v)
        elif isinstance(v,(str,bytes,bytearray)):
            if isinstance(v,str):
                data=self.symbols.get(v)
                if data is None:
                    data=v.encode('utf8')
                    if len(self.symbols)<512:self.symbols[v]=data
                tag=5
            else:data=v;tag=6
            self._need(3+len(data));self._byte(tag);U16.pack_into(self.buf,self.pos,len(data));self.pos+=2
            self.buf[self.pos:self.pos+len(data)]=data;self.pos+=len(data)
        elif isinstance(v,(list,tuple,dict)):
            self._need(3);self._byte(8 if isinstance(v,dict)else 7)
            U16.pack_into(self.buf,self.pos,len(v));self.pos+=2
            if isinstance(v,dict):
                for key,value in v.items():
                    if not isinstance(key,str):raise ValueError('binary evidence keys must be strings')
                    self._value(key,depth+1);self._value(value,depth+1)
            else:
                for value in v:self._value(value,depth+1)
        else:raise ValueError('unsupported binary evidence value')
    def encode_into(self,buffer,offset,sequence,event,pc_us,details):
        self.buf=memoryview(buffer).cast('B');self.pos=offset+HEADER.size;self.limit=offset+SLOT_BYTES;self.cells=0
        if not self._fast(event,pc_us,details):self._value((event,pc_us,details))
        size=self.pos-offset
        crc=zlib.crc32(self.buf[offset+HEADER.size:self.pos])
        HEADER.pack_into(self.buf,offset,b'GPR1',1,sequence,size,crc)
        return size
    @staticmethod
    def decode(data):
        if len(data)<=HEADER.size:raise ValueError('truncated evidence header/payload')
        magic,version,sequence,size,crc=HEADER.unpack_from(data)
        if magic!=b'GPR1' or version!=1 or size!=len(data) or size>SLOT_BYTES or zlib.crc32(data[HEADER.size:])!=crc:
            raise ValueError('binary evidence framing/CRC')
        position=HEADER.size;cells=0
        if data[position]==10:
            if size<position+FAST_HEAD.size:raise ValueError('truncated fixed evidence')
            _,index,present,floats,booleans,nones,pc=FAST_HEAD.unpack_from(data,position)
            if index>=len(FAST_SCHEMAS):raise ValueError('fixed evidence schema')
            event,fields,raw=FAST_SCHEMAS[index];count=len(fields)
            if (present>> (count+bool(raw)) or (floats|booleans|nones)>>count or
                    floats&booleans or floats&nones or booleans&nones):raise ValueError('fixed evidence masks')
            fmt=struct.Struct(FAST_HEAD.format+''.join('d'if floats&(1<<i)else'q'for i in range(count)))
            if position+fmt.size>size:raise ValueError('truncated fixed fields')
            values=fmt.unpack_from(data,position)[7:];position+=fmt.size;details={}
            for i,key in enumerate(fields):
                if present&(1<<i):details[key]=None if nones&(1<<i)else bool(values[i])if booleans&(1<<i)else values[i]
            if raw:
                if position+2>size:raise ValueError('truncated fixed bytes')
                n=U16.unpack_from(data,position)[0];position+=2
                if position+n>size:raise ValueError('truncated fixed bytes')
                if present&(1<<count):details[raw]=data[position:position+n].hex()
                position+=n
            if position!=size:raise ValueError('fixed evidence trailing bytes')
            return sequence,event,pc,details
        def take(n):
            nonlocal position
            if position+n>size:raise ValueError('truncated evidence cell')
            result=data[position:position+n];position+=n;return result
        def value(depth=0):
            nonlocal cells
            cells+=1
            if depth>12 or cells>512:raise ValueError('evidence cell budget')
            tag=take(1)[0]
            if tag==0:return None
            if tag in(1,2):return tag==2
            if tag in(3,4,9):return {3:I64,4:F64,9:U64}[tag].unpack(take(8))[0]
            if tag in(5,6):
                raw=take(U16.unpack(take(2))[0]);return raw.decode('utf8')if tag==5 else raw.hex()
            if tag in(7,8):
                count=U16.unpack(take(2))[0]
                if count>512:raise ValueError('evidence collection budget')
                if tag==7:return [value(depth+1)for _ in range(count)]
                result={}
                for _ in range(count):
                    key=value(depth+1)
                    if not isinstance(key,str)or key in result:raise ValueError('evidence key')
                    result[key]=value(depth+1)
                return result
            raise ValueError('unknown evidence cell')
        event,pc,details=value()
        if position!=size or not isinstance(event,str)or not isinstance(details,dict):raise ValueError('evidence row schema')
        return sequence,event,pc,details


def legacy_details(value):
    """Synchronous/post-close compatibility renderer, NEVER producer-side."""
    if isinstance(value,(bytes,bytearray)):return value.hex()
    if isinstance(value,dict):return {k:legacy_details(v)for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [legacy_details(v)for v in value]
    return value


def _ring(ctx,capacity):
    return (ctx.RawArray('B',capacity*SLOT_BYTES),ctx.Semaphore(capacity),ctx.Semaphore(0),ctx.RawValue('Q',0))


def _write_all(stream,data,short=False):
    offset=0
    while offset<len(data):
        count=stream.write(data[offset:offset+7]if short else data[offset:])
        if not count:raise OSError('zero-progress evidence spool write')
        offset+=count


def _worker(output,rings,wake,gate,connection,fault):
    written=0;cpu_start=time.process_time_ns()
    signal.signal(signal.SIGINT,signal.SIG_IGN)  # Parent owns operator stop and close.
    try:
        output=Path(output)
        with (output/'evidence.bin').open('xb',buffering=0)as stream:
            connection.send(('ready',))
            gate.acquire()
            if fault=='hang':time.sleep(60)
            tails=[0,0];held={};expected=None
            while expected is None or written<expected:
                if connection.poll():
                    message=connection.recv()
                    if message[0]!='finish':raise ValueError('recorder control message')
                    expected=message[1]
                    if written==expected:break
                if not wake.acquire(timeout=.05):continue
                for index,(raw,free,ready,consumed)in enumerate(rings):
                    if index not in held and ready.acquire(False):
                        view=memoryview(raw).cast('B');offset=(tails[index]%(len(raw)//SLOT_BYTES))*SLOT_BYTES
                        header=HEADER.unpack_from(view,offset);size=header[3]
                        if not HEADER.size<=size<=SLOT_BYTES:raise ValueError('shared slot length')
                        held[index]=bytes(view[offset:offset+size])
                if not held:raise ValueError('publication credit without record')
                index=min(held,key=lambda i:HEADER.unpack_from(held[i])[2]);data=held.pop(index)
                if HEADER.unpack_from(data)[2]!=written:raise ValueError('recorder sequence gap')
                if zlib.crc32(data[HEADER.size:])!=HEADER.unpack_from(data)[4]:raise ValueError('shared record CRC')
                if fault=='write_error':raise OSError('injected spool write failure')
                _write_all(stream,data,short=fault=='short_write')
                written+=1;tails[index]+=1;rings[index][3].value=tails[index];rings[index][1].release()
        spool_cpu=time.process_time_ns()-cpu_start;render_start=time.process_time_ns()
        with (output/'evidence.bin').open('rb')as stream,(output/'events.csv').open('x',newline='')as csv_stream:
            writer=csv.writer(csv_stream);writer.writerow(('event','pc_us','details_json'))
            count=0
            while header:=stream.read(HEADER.size):
                if len(header)!=HEADER.size:raise ValueError('truncated spool header')
                size=HEADER.unpack(header)[3]
                if not HEADER.size<=size<=SLOT_BYTES:raise ValueError('spool length')
                data=header+stream.read(size-HEADER.size)
                seq,event,pc,details=BinaryRecordCodec.decode(data)
                if seq!=count:raise ValueError('spool order')
                writer.writerow((event,pc,json.dumps(details,sort_keys=True)));count+=1
            if count!=written:raise ValueError('spool/export count')
        connection.send(('finished',written,spool_cpu,time.process_time_ns()-render_start))
    except BaseException as error:
        try:connection.send(('error',written,type(error).__name__+': '+str(error)))
        except (OSError,EOFError):pass
    finally:connection.close()


class ProcessLog:
    def __init__(self,output,*,normal_capacity=8192,stop_capacity=256,paused=False,test_fault=None):
        if any(type(v)is not int or not 1<=v<=8192 for v in(normal_capacity,stop_capacity)):raise ValueError('recorder capacities1..8192')
        self.output=Path(output);self.output.mkdir(exist_ok=True)
        ctx=mp.get_context('spawn');self.rings=(_ring(ctx,normal_capacity),_ring(ctx,stop_capacity))
        self.wake=ctx.Semaphore(0);self.gate=ctx.Semaphore(0);self.mailbox=ctx.RawArray('B',CLOSE.size)
        self.connection,child=ctx.Pipe();self.codec=BinaryRecordCodec();self.heads=[0,0]
        self.sent=0;self.lost=0;self.first_missing=2**64-1;self.failure=None;self.stopping=False;self.done=False
        self.process=ctx.Process(target=_worker,args=(str(self.output),self.rings,self.wake,self.gate,child,test_fault))
        try:
            self.process.start();child.close()
            (self.output/'process-recorder-owned.json').write_text(json.dumps(dict(owned_pid=self.process.pid,parent_pid=os.getpid()))+'\n')
            if not self.connection.poll(10)or self.connection.recv()!=('ready',):raise EvidenceFailure('recorder failed READY')
            if not paused:self.resume()
        except BaseException:
            child.close()
            if self.process.pid is not None:self._stop_child()
            self.connection.close()
            raise
    def resume(self):self.gate.release()
    def begin_stop(self):self.stopping=True
    def _stop_child(self):
        if self.process.is_alive():self.process.terminate()
        self.process.join(2)
        if self.process.is_alive():self.process.kill();self.process.join(2)
    def _put(self,index,event,pc,details):
        raw,free,ready,_=self.rings[index]
        if not free.acquire(False):return False
        offset=(self.heads[index]%(len(raw)//SLOT_BYTES))*SLOT_BYTES
        try:self.codec.encode_into(raw,offset,self.sent,event,pc,details)
        except BaseException:free.release();raise
        self.heads[index]+=1;self.sent+=1;ready.release();self.wake.release();return True
    def record(self,event,pc,details):
        if self.done:raise EvidenceFailure('recorder already finalized')
        dead=not self.process.is_alive();error=None
        index=1 if self.stopping or dead else 0
        try:stored=self._put(index,event,pc,details)
        except KeyboardInterrupt:
            self.failure=self.failure or 'producer publication interrupted';self.lost+=1
            if self.first_missing==2**64-1:self.first_missing=self.sent
            raise
        except (ValueError,OverflowError,struct.error)as exception:
            error='record encoding failed: '+str(exception);stored=False
        if not stored and index==0 and error is None:
            self.stopping=True;error='normal ring full'
            try:stored=self._put(1,event,pc,details)
            except (ValueError,OverflowError,struct.error)as exception:
                error='record encoding failed in reserve: '+str(exception);stored=False
        elif not stored and error is None:error='stop ring full'
        if not stored:
            if self.first_missing==2**64-1:self.first_missing=self.sent
            self.lost+=1
        if dead:error=error or 'recorder process died';self.stopping=True
        if error:
            self.failure=self.failure or error
            raise EvidenceFailure(error)
    def poll(self):
        if not self.process.is_alive():
            self.failure=self.failure or 'recorder process died';raise EvidenceFailure(self.failure)
    def wait_consumed(self,count,timeout=5):
        """Test/bench control only; NEVER used by the timed producer."""
        deadline=time.monotonic()+timeout
        while sum(r[3].value for r in self.rings)<count:
            if time.monotonic()>deadline:raise EvidenceFailure('test consumer wait')
            self.poll();time.sleep(.001)
    def finish(self,pc_us,*,serial_closed,inhibited,timeout=5):
        try:return self._finish(pc_us,serial_closed=serial_closed,inhibited=inhibited,timeout=timeout)
        finally:
            if self.process.is_alive():self._stop_child()

    def _finish(self,pc_us,*,serial_closed,inhibited,timeout=5):
        if self.done:raise EvidenceFailure('duplicate recorder finalization')
        self.done=True;self.resume()
        flags=int(serial_closed)|int(inhibited)<<1|int(self.failure is not None)<<2
        CLOSE.pack_into(self.mailbox,0,b'GPC1',1,pc_us,self.sent,self.first_missing,flags,self.lost)
        (self.output/'recorder-close.bin').write_bytes(bytes(self.mailbox))  # AFTER close attempt only.
        try:self.connection.send(('finish',self.sent))
        except (OSError,EOFError):self.failure=self.failure or 'recorder control connection failed'
        try:self.process.join(timeout)
        except KeyboardInterrupt:
            self.failure=self.failure or 'interrupted during recorder finalization';self._stop_child()
        if self.process.is_alive():self.failure=self.failure or 'recorder finish timed out';self._stop_child()
        reply=None
        while self.connection.poll():
            try:reply=self.connection.recv()
            except (EOFError,OSError):break
        self.connection.close()
        written=sum(r[3].value for r in self.rings)
        complete=(reply is not None and reply[0]=='finished'and reply[1]==self.sent and written==self.sent and
                  not self.lost and self.failure is None and bool(serial_closed)and bool(inhibited)and self.process.exitcode==0)
        if not complete:self.failure=self.failure or ('recorder export failed: '+str(reply))
        if not complete:
            with (self.output/'evidence-pending.bin').open('xb')as stream:
                pending=[]
                for index,(raw,_,_,consumed)in enumerate(self.rings):
                    view=memoryview(raw).cast('B');capacity=len(raw)//SLOT_BYTES
                    for position in range(consumed.value,self.heads[index]):
                        offset=(position%capacity)*SLOT_BYTES;size=HEADER.unpack_from(view,offset)[3]
                        if HEADER.size<=size<=SLOT_BYTES:pending.append(bytes(view[offset:offset+size]))
                for data in sorted(pending,key=lambda d:HEADER.unpack_from(d)[2]):stream.write(data)
        meta=dict(schema=1,mode='process-binary-ring',slot_bytes=SLOT_BYTES,
            normal_capacity=len(self.rings[0][0])//SLOT_BYTES,stop_capacity=len(self.rings[1][0])//SLOT_BYTES,
            sent_records=self.sent,written_records=written,lost_records=self.lost,
            evidence_complete=complete,failure=self.failure,serial_closed=bool(serial_closed),inhibited=bool(inhibited),
            owned_pid=self.process.pid,exitcode=self.process.exitcode,
            spool_cpu_ns=reply[2]if reply and reply[0]=='finished'else None,
            render_cpu_ns=reply[3]if reply and reply[0]=='finished'else None)
        (self.output/'process-recorder.json').write_text(json.dumps(meta,indent=2,sort_keys=True)+'\n')
        return meta
