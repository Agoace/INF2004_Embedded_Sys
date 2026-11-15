#!/usr/bin/env python3
"""Check the linked Gate1 record/vector and interpret its actual Thumb stub.

This small, fail-closed instruction interpreter is NOT a hardware fault trial.
It accepts only the instructions used by this bounded stub. No target access.
"""
import struct
import sys
from pathlib import Path

SALT = 0x9e3779b9
RAM_START, RAM_END = 0x20000000, 0x20042000


class Elf:
    def __init__(self, path):
        self.raw = Path(path).read_bytes()
        h = struct.unpack_from('<16sHHIIIIIHHHHHH', self.raw)
        assert h[0][:6] == b'\x7fELF\x01\x01' and h[2] == 40
        self.sections = [struct.unpack_from('<10I', self.raw, h[6]+i*h[11])
                         for i in range(h[12])]
        names = self.contents(self.sections[h[13]])
        self.named = {self.string(names, s[0]): s for s in self.sections}
        self.symbols = {}
        self.types = {}
        for s in self.sections:
            if s[1] != 2:
                continue
            strings = self.contents(self.sections[s[6]])
            for at in range(s[4], s[4]+s[5], s[9]):
                name, value, size, info, _, _ = struct.unpack_from(
                    '<IIIBBH', self.raw, at)
                name = self.string(strings, name)
                self.symbols[name] = (value, size)
                self.types[name] = info & 15

    @staticmethod
    def string(data, at):
        return data[at:data.index(b'\0', at)].decode()

    def contents(self, section):
        return self.raw[section[4]:section[4]+section[5]]

    def address(self, name):
        return self.symbols[name][0] & ~1

    def bytes_at(self, address, size):
        for s in self.sections:
            if s[1] == 1 and s[3] <= address and address+size <= s[3]+s[5]:
                at = s[4] + address - s[3]
                return self.raw[at:at+size]
        raise AssertionError(('not initialized bytes', hex(address), size))


def run_stub(elf, frame, psp=False, active=True, partial=False):
    """Interpret the linked bytes, checking every write/frame read/delegation."""
    start = elf.address('__wrap_HardFault_Handler')
    original = elf.address('HardFault_Handler') | 1
    live = elf.address('pil_bc_live')
    fault = live + 64
    ram = bytearray(RAM_END-RAM_START)
    data = elf.named['.data']
    ram[data[3]-RAM_START:data[3]-RAM_START+data[5]] = elf.contents(data)

    def store(address, value):
        assert RAM_START <= address <= RAM_END-4
        struct.pack_into('<I', ram, address-RAM_START, value & 0xffffffff)

    def read(address):
        if address == 0x40054028:
            return 123456
        if address == 0xd0000000:
            return 0
        assert RAM_START <= address <= RAM_END-4, ('unsafe read', hex(address))
        return struct.unpack_from('<I', ram, address-RAM_START)[0]

    store(elf.address('pil_bc_active'), int(active))
    valid_frame = (frame & 3) == 0 and RAM_START <= frame <= RAM_END-32
    saved = [11, 22, 33, 44, 55, 0x10005555, 0x10006666, 0x21000000]
    if valid_frame:
        for i, value in enumerate(saved):
            store(frame+4*i, value)
    regs = [0]*16
    regs[14] = 0xfffffffd if psp else 0xfffffff9
    old_lr = regs[14]
    control = {8: 0xffffffff if psp else frame,
               9: frame if psp else 0xffffffff, 16: 1, 5: 3}
    pc, z, carry = start, False, False
    frame_reads, writes = [], []
    for _ in range(300):
        ins = struct.unpack('<H', elf.bytes_at(pc, 2))[0]
        next_pc = pc + 2
        if ins == 0xf3ef:  # MRS, without changing the actual stack.
            second = struct.unpack('<H', elf.bytes_at(pc+2, 2))[0]
            regs[(second >> 8) & 15] = control[second & 255]
            next_pc += 2
        elif ins == 0xf3bf:
            assert elf.bytes_at(pc+2, 2) == b'\x5f\x8f'  # DMB
            next_pc += 2
        elif ins & 0xff87 == 0x4700:  # BX: delegate, do not emulate reporter.
            assert regs[(ins >> 3) & 15] == original
            assert regs[14] == old_lr
            break
        elif ins & 0xfc00 == 0x4400:
            assert (ins >> 8) & 3 == 2  # high-register MOV only
            regs[(ins & 7) | ((ins >> 4) & 8)] = regs[(ins >> 3) & 15]
        elif ins & 0xf800 == 0x4800:
            regs[(ins >> 8) & 7] = read(((pc+4) & ~3) + (ins & 255)*4)
        elif ins & 0xf800 in (0x6000, 0x6800):
            base, target = (ins >> 3) & 7, ins & 7
            address = regs[base] + ((ins >> 6) & 31)*4
            if ins & 0xf800 == 0x6000:
                assert fault <= address < fault+48, ('unexpected write', address)
                store(address, regs[target]); writes.append(address)
                if partial and address == fault and regs[target] == 1:
                    break
            else:
                if base == 1:
                    frame_reads.append(address)
                regs[target] = read(address)
        elif ins & 0xfe00 == 0x5800:
            regs[ins & 7] = read(regs[(ins >> 3) & 7]+regs[(ins >> 6) & 7])
        elif ins & 0xffc0 in (0x4040, 0x4200, 0x4280, 0x43c0):
            rd, rs, kind = ins & 7, (ins >> 3) & 7, ins & 0xffc0
            if kind == 0x4040:
                regs[rd] ^= regs[rs]
                z = regs[rd] == 0
            elif kind == 0x4200:
                z = (regs[rd] & regs[rs]) == 0
            elif kind == 0x4280:
                z, carry = regs[rd] == regs[rs], regs[rd] >= regs[rs]
            else:
                regs[rd] = (~regs[rs]) & 0xffffffff
                z = regs[rd] == 0
        elif ins & 0xf800 in (0x2000, 0x2800, 0x3000):
            rd, value, kind = (ins >> 8) & 7, ins & 255, ins & 0xf800
            if kind == 0x2000:
                regs[rd] = value; z = value == 0
            elif kind == 0x2800:
                z, carry = regs[rd] == value, regs[rd] >= value
            else:
                total = regs[rd]+value
                regs[rd] = total & 0xffffffff
                z, carry = regs[rd] == 0, total > 0xffffffff
        elif ins & 0xf000 == 0xd000:
            condition = (ins >> 8) & 15
            take = {0: z, 1: not z, 3: not carry, 8: carry and not z}[condition]
            if take:
                offset = ins & 255
                next_pc = pc+4+2*(offset-256 if offset & 128 else offset)
        elif ins & 0xf800 == 0xe000:
            offset = ins & 2047
            next_pc = pc+4+2*(offset-2048 if offset & 1024 else offset)
        else:
            raise AssertionError(('unexpected instruction', hex(pc), hex(ins)))
        pc = next_pc
    else:
        raise AssertionError('stub did not delegate within bound')
    result = [read(fault+i*4) for i in range(12)]
    if not active:
        assert not writes and not frame_reads and result == [0]*12
    elif partial:
        assert result[0] == 1 and not frame_reads
    else:
        assert result[0] == (2 if valid_frame else 3)
        assert result[1] == 123456 and result[5] == frame
        assert result[6:10] == [old_lr, 1, 3, 0]
        assert result[2:5] == (saved[6:7]+saved[5:6]+saved[7:8]
                                if valid_frame else [0xffffffff]*3)
        assert frame_reads == ([frame+24, frame+20, frame+28]
                               if valid_frame else [])
        check = SALT
        for word in result[:10]:
            check ^= word
        assert result[10:] == [check, (~check) & 0xffffffff]
    return result


def main():
    elf = Elf(sys.argv[1])
    addr = elf.address
    retained = elf.named['.pil_retained']
    assert retained[1] == 8 and retained[5] == 112  # SHT_NOBITS
    assert addr('__data_end') <= retained[3]
    assert retained[3]+112 <= addr('__bss_start') < addr('__bss_end')
    assert RAM_START <= retained[3] < retained[3]+112 <= RAM_END
    assert addr('pil_bc_live') == retained[3]
    wrapper, size = elf.symbols['__wrap_HardFault_Handler']
    assert wrapper & 1 and elf.types['__wrap_HardFault_Handler'] == 2
    assert addr('__data_start') <= (wrapper & ~1)
    assert (wrapper & ~1)+size <= addr('__data_end') and size <= 256
    # Sum every added writable object, rounded to4, plus RAM code/literals.
    extra = sum((elf.symbols[n][1]+3) & ~3 for n in
                ('pil_bc_live', 'previous', 'boot_line', 'pil_bc_active'))
    extra += size + max(0, elf.symbols['g'][1]-12908)
    assert extra <= 800 and 14960+extra <= 16384
    vector = elf.bytes_at(addr('vector_tbl'), 48*4)
    hardfault = struct.unpack_from('<I', vector, 12)[0]
    assert hardfault == wrapper
    assert addr('exchdr_tbl') == RAM_START
    # Model unchanged startup: data copy, BSS zero and vector copy cannot erase
    # a sentinel in the dedicated retained span. Do NOT zero ELF p_memsz.
    ram = bytearray(b'\xa5'*(RAM_END-RAM_START))
    for lo, hi, content in (
        (addr('__data_start'), addr('__data_end'), elf.contents(elf.named['.data'])),
        (addr('__bss_start'), addr('__bss_end'), b'\0'*(addr('__bss_end')-addr('__bss_start'))),
        (RAM_START, RAM_START+len(vector), vector),
    ):
        ram[lo-RAM_START:hi-RAM_START] = content
    offset = retained[3]-RAM_START
    assert ram[offset:offset+112] == b'\xa5'*112
    frames = [0x20030000, 0x20041fe0, 0x20041fe4, 0x20042000,
              0x1ffffffc, 0xfffffffc, 0x20030001, 0]
    for frame in frames:
        for psp in (False, True):
            run_stub(elf, frame, psp)
    run_stub(elf, 0x20030000, active=False)
    run_stub(elf, 0x20030000, partial=True)
    print(f'PASS Thumb vector, {size}B RAM stub,18 interpreted executions;')
    print(f'PASS added RAM {extra}B, Gate1 total {14960+extra}B;')
    print('PASS guarded MSP/PSP reads, no stack pushes/calls/feed/other writes;')
    print('PASS NOLOAD retention outside startup ranges and below heap boundary.')


if __name__ == '__main__':
    main()
