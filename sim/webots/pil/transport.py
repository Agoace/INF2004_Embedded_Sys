"""Host-only PIL session and bounded transport core, with injectable time/I/O.

No serial port is opened; no Webots APIs are imported. Gate1 must supply and
qualify the real serial worker. A successful PC write is NOT device availability.
"""

from collections import deque
import struct

from .clock import ClockBounds, ClockError, Timeline, unwrap32
from .wire import (CAPABILITIES, MAX_SOURCE_US, NO_ACK, Frame, ProtocolError,
                   StreamDecoder, Type, encode, newer, u16, u32)


class TransportFault(RuntimeError):
    pass


class HostTransport:
    QUEUE_LIMIT = 8
    EVENT_LIMIT = 1024
    MAX_RX_CHUNK = 4096

    def __init__(self, session, *, clock=None):
        if type(session) is not int or not 1 <= session <= 0xFFFFFFFF:
            raise ValueError("nonzero uint32 host session required")
        self.session, self.clock = session, clock if clock is not None else ClockBounds()
        self.decoder = StreamDecoder()
        self.state, self.failure = "NEW", None
        self.boot, self.hello_frame, self.ack_frame, self.timeline = None, None, None, None
        self.tx, self.outputs, self.events = deque(), deque(), deque()
        self.send_sequences, self.receive_sequences, self.sync_pending = {}, {}, {}
        self.last_output, self.applied_output = None, NO_ACK
        self.last_output_pc_us, self.arm_pc_us = None, None
        self.next_sensor, self.last_sensor_sent = 0, NO_ACK
        self.last_pc_us = None

    @property
    def model_inhibited(self):
        return self.failure is not None or self.state != "ARMED"

    def _time(self, pc_us):
        if type(pc_us) is not int or pc_us < 0 or (
                self.last_pc_us is not None and pc_us < self.last_pc_us):
            self._fault("nonmonotonic host time")
        self.last_pc_us = pc_us

    def _record(self, kind, **values):
        if len(self.events) >= self.EVENT_LIMIT:
            self._fault("evidence queue overflow")
        self.events.append({"event": kind, **values})

    def drain_events(self):
        result = tuple(self.events)
        self.events.clear()
        return result

    def _fault(self, reason):
        if self.failure is None:
            self.failure = reason
        self.state = "FAULT"
        self.outputs.clear()
        # Never silently resume queued commands after a fault. A delimiter is
        # needed before best-effort DISARM if a packet was partially transmitted.
        self.tx.clear()
        raise TransportFault(self.failure)

    def _require_live(self):
        if self.failure is not None:
            raise TransportFault(self.failure)

    def _queue(self, kind, payload, pc_us, ack=NO_ACK, source_us=None):
        self._time(pc_us)
        if len(self.tx) >= self.QUEUE_LIMIT:
            self._fault("TX queue overflow")
        sequence = self.send_sequences.get(kind, 0)
        frame = Frame(kind, self.session, sequence,
                      pc_us & 0xFFFFFFFF if source_us is None else source_us, ack, payload)
        packet = encode(frame)
        self.tx.append((frame, packet, 0))
        self.send_sequences[kind] = (sequence + 1) & 0xFFFFFFFF
        self._record("tx_queued", pc_us=pc_us, type=int(kind), seq=sequence)
        return frame

    def hello(self, pc_us):
        self._require_live()
        if self.state not in ("BOOT", "NEGOTIATED") or self.boot is None:
            raise TransportFault("HELLO requires BOOT, before ARM")
        nonce, caps = u32(self.boot.payload, 0), u32(self.boot.payload, 4)
        if caps & CAPABILITIES != CAPABILITIES:
            self._fault("Pico lacks operational capabilities; skeletons cannot arm")
        payload = struct.pack("<6I", nonce, self.session, CAPABILITIES, 1000, 4000, 5000)
        self.hello_frame = self._queue(Type.HELLO, payload, pc_us, self.boot.sequence)
        self.ack_frame, self.timeline = None, None
        self.state = "BOOT"
        return self.hello_frame

    def sync_request(self, pc_us):
        self._require_live()
        if self.ack_frame is None:
            raise TransportFault("SYNC requires established HELLO session")
        if len(self.sync_pending) >= self.QUEUE_LIMIT:
            self._fault("SYNC request queue overflow")
        frame = self._queue(Type.SYNC_REQ, struct.pack("<Q", pc_us), pc_us,
                            self.receive_sequences.get(Type.SYNC_REPLY, NO_ACK))
        self.sync_pending[frame.sequence] = pc_us
        return frame

    def arm(self, pc_us):
        self._require_live()
        self._time(pc_us)
        if self.state != "NEGOTIATED" or self.timeline is None:
            raise TransportFault("ARM requires accepted HELLO_ACK")
        try:
            now = self.clock.pico_range(pc_us)
        except ClockError as error:
            self._fault(str(error))
        if self.timeline.epoch_pico_us - now.upper < 20_000:
            self._fault("less than20ms conservative ARM setup lead")
        payload = struct.pack("<II", u32(self.boot.payload, 0), self.session)
        frame = self._queue(Type.ARM, payload, pc_us, self.ack_frame.sequence)
        self.state, self.arm_pc_us = "ARM_PENDING", pc_us
        return frame

    def sensor(self, payload, source_us, captured_pc_us):
        self._require_live()
        self._time(captured_pc_us)
        if self.state not in ("ARM_PENDING", "ARMED") or self.timeline is None:
            raise TransportFault("SENSOR requires requested ARM")
        if source_us != self.next_sensor * 1000 or source_us > MAX_SOURCE_US:
            self._fault("missing/replayed source step")
        if len(payload) != 40:
            self._fault("invalid SENSOR payload")
        if not u16(payload, 36) & 0x800 or u16(payload, 38):
            self._fault("model timing invalid or missed steps")
        if self.next_sensor < 2 and payload[6]:
            self._fault("initial buttons must be neutral")
        if self.next_sensor == 0 and (u32(payload, 8) or u32(payload, 12)):
            self._fault("initial encoder origin must be zero")
        try:
            phase, uncertainty = self.timeline.validate_capture(
                source_us, captured_pc_us, self.clock)
            frame = self._queue(Type.SENSOR, payload, captured_pc_us,
                                self.applied_output, source_us)
        except (ClockError, ProtocolError) as error:
            self._fault(str(error))
        self.next_sensor += 1
        self.last_sensor_sent = frame.sequence
        self._record("source_capture", source_us=source_us, pc_us=captured_pc_us,
                     phase_us=phase, uncertainty_us=uncertainty)
        return frame

    def peek_tx(self, maximum=130):
        if type(maximum) is not int or maximum <= 0:
            raise ValueError("positive byte capacity required")
        return self.tx[0][1][self.tx[0][2]:self.tx[0][2] + maximum] if self.tx else b""

    def consume_tx(self, count, pc_us):
        self._require_live()
        self._time(pc_us)
        if not self.tx:
            raise ValueError("no pending write")
        frame, packet, offset = self.tx[0]
        if type(count) is not int or not 0 <= count <= len(packet) - offset:
            raise ValueError("invalid completed-write count")
        if count == 0:
            return
        self._record("pc_write", pc_us=pc_us, type=int(frame.type), seq=frame.sequence,
                     offset=offset, bytes=count, device_available=False)
        if offset + count == len(packet):
            self.tx.popleft()
        else:
            self.tx[0] = (frame, packet, offset + count)

    def _output_lease(self, pc_us):
        if self.state in ("ARM_PENDING", "ARMED"):
            last = (self.last_output_pc_us if self.last_output_pc_us is not None
                    else self.arm_pc_us)
            if pc_us - last >= 5000:
                self._fault("OUTPUT progress lease expired; MODEL inhibited, Pico PWM unknown")

    def receive(self, data, received_pc_us):
        self._require_live()
        self._time(received_pc_us)
        self._output_lease(received_pc_us)
        if not isinstance(data, (bytes, bytearray)) or len(data) > self.MAX_RX_CHUNK:
            self._fault("RX chunk exceeds bounded transport input")
        for frame in self.decoder.feed(data):
            self._record("rx_complete", pc_us=received_pc_us, type=frame.type,
                         seq=frame.sequence, source_us=frame.source_us)
            if frame.type == Type.BOOT:
                if self.boot is not None and u32(frame.payload, 0) != u32(self.boot.payload, 0):
                    self._fault("Pico reset/new nonce; new host session required")
                if self.state in ("NEW", "BOOT"):
                    self.boot, self.state = frame, "BOOT"
                continue
            if frame.session != self.session:
                continue
            previous = self.receive_sequences.get(frame.type)
            if previous is not None and not newer(frame.sequence, previous):
                continue
            if frame.type == Type.HELLO_ACK:
                if (self.state != "BOOT" or self.hello_frame is None or
                        frame.ack != self.hello_frame.sequence or
                        frame.payload[:24] != self.hello_frame.payload):
                    continue
                anchor = (frame.source_us if self.clock.pico_anchor is None
                          else self.clock.pico_anchor)
                generated = unwrap32(frame.source_us, anchor)
                epoch = unwrap32(u32(frame.payload, 24), generated)
                if epoch - generated < 100_000:
                    self._fault("ACK epoch has insufficient generation lead")
                if self.clock.interval is None:
                    self.clock.pico_anchor = generated
                self.ack_frame, self.timeline = frame, Timeline(epoch)
                self.state = "NEGOTIATED"
            elif frame.type == Type.SYNC_REPLY:
                sent = self.sync_pending.get(frame.ack)
                if sent is None or struct.unpack_from("<Q", frame.payload)[0] != sent:
                    continue
                try:
                    self.clock.observe(sent, received_pc_us, u32(frame.payload, 8),
                                       frame.source_us)
                except ClockError as error:
                    self._fault(str(error))
                del self.sync_pending[frame.ack]
            elif frame.type == Type.OUTPUT:
                if self.state not in ("NEGOTIATED", "ARM_PENDING", "ARMED"):
                    continue
                if self.last_output is not None and not newer(
                        frame.source_us, self.last_output.source_us):
                    continue
                if self.state in ("ARM_PENDING", "ARMED"):
                    try:
                        now = self.clock.pico_range(received_pc_us)
                        snapshot = unwrap32(frame.source_us, round(now.midpoint))
                    except ClockError as error:
                        self._fault(str(error))
                    if snapshot > now.upper or now.upper - snapshot > 5000:
                        self._fault("OUTPUT snapshot outside conservative source-age bound")
                if frame.ack != NO_ACK and (self.last_sensor_sent == NO_ACK or
                                           newer(frame.ack, self.last_sensor_sent)):
                    self._fault("OUTPUT acknowledges an unsent SENSOR")
                if self.last_output is not None and self.last_output.ack != NO_ACK:
                    if frame.ack == NO_ACK or (frame.ack != self.last_output.ack and
                                              not newer(frame.ack, self.last_output.ack)):
                        self._fault("OUTPUT SENSOR acknowledgement regressed")
                self.last_output, self.last_output_pc_us = frame, received_pc_us
                if frame.payload[7] & (2 | 8):
                    self._fault("Pico reported link fault/underflow")
                if frame.payload[7] & 1:
                    if self.state == "ARM_PENDING" and frame.ack != NO_ACK:
                        self.state = "ARMED"
                    elif self.state != "ARMED":
                        self._fault("unsolicited ARMED output")
                elif self.state == "ARMED":
                    self._fault("Pico inhibited/disarmed during run")
                if self.state == "ARMED":
                    if len(self.outputs) >= self.QUEUE_LIMIT:
                        self._fault("OUTPUT queue overflow")
                    self.outputs.append(frame)
            self.receive_sequences[frame.type] = frame.sequence

    def next_output(self):
        self._require_live()
        return self.outputs[0] if self.outputs else None

    def mark_output_applied(self, sequence, applied_pc_us):
        """Call on controller thread AFTER the output has entered physics."""
        self._require_live()
        self._time(applied_pc_us)
        if not self.outputs or self.outputs[0].sequence != sequence:
            self._fault("OUTPUT application must follow receive order")
        frame = self.outputs.popleft()
        self.applied_output = frame.sequence
        self._record("physics_applied", seq=sequence, pc_us=applied_pc_us,
                     pico_snapshot_us=frame.source_us, pico_pwm=tuple(
                         struct.unpack_from("<hh", frame.payload)))
        return frame

    def tick(self, pc_us):
        self._require_live()
        self._time(pc_us)
        if self.state not in ("ARM_PENDING", "ARMED"):
            return
        try:
            now = self.clock.pico_range(pc_us)
        except ClockError as error:
            self._fault(str(error))
        self._output_lease(pc_us)
        if self.state == "ARM_PENDING" and now.lower > self.timeline.epoch_pico_us + 5000:
            self._fault("no ARM acceptance after initial epoch")
        for frame, _, _ in self.tx:
            if (frame.type == Type.SENSOR and
                    now.upper > self.timeline.apply(frame.source_us) - 1000):
                self._fault("SENSOR still on PC after lookahead deadline")

    def disarm_packet(self, reason, pc_us):
        """Best effort after LOCAL inhibit; delimiter repairs any partial packet.

        Returns bytes for an external serial worker. It cannot prove delivery.
        """
        self.state = "FAULT"
        self.failure = self.failure or "operator DISARM; reset/new session required"
        self.outputs.clear()
        self.tx.clear()
        sequence = self.send_sequences.get(Type.DISARM, 0)
        self.send_sequences[Type.DISARM] = (sequence + 1) & 0xFFFFFFFF
        return b"\x00" + encode(Frame(Type.DISARM, self.session, sequence,
                                     pc_us & 0xFFFFFFFF,
                                     self.last_output.sequence if self.last_output else NO_ACK,
                                     struct.pack("<I", reason)))
