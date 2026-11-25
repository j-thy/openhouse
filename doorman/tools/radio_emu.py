#!/usr/bin/env python3
"""Run the doorman radio firmware (radio-fw/build/radio.bin) in an emulated nRF51/nRF52 and drive its SPI slave
the way our LPC does, to check the updater BEFORE installing an image on the real radio.

Modelled: SPIS1 (semaphore, EasyDMA buffers, AMOUNTRX/TX, END->ACQUIRE short), NVMC (erase/write with
real flash semantics: a write can only clear bits), TIMER0 capture, CLOCK, WDT, FICR, AIRCR reset.
Every flash write and erase is checked to stay inside the staging bank 0x10000-0x1F7FF.

  python radio_emu.py [IMAGE]          HTC nRF51 layout (Vive Tracker 1.0)
  python radio_emu.py --idx [IMAGE]    Valve nRF52 layout (Index controller family): app at 0x1000,
                                       header +0x1C/+0x20, 4 KB pages; stages Valve's own radio image
The stock radio image (HTC 2016 / Valve Index) is staged too when SteamVR is installed (STEAMVR=dir
overrides ~/.local/share/Steam/steamapps/common/SteamVR); without it our own image is staged twice.
Run: uv run --with unicorn python radio_emu.py [--idx]
"""
import os, struct, sys
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ, UcError
from unicorn.arm_const import UC_ARM_REG_PC
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from radio_image import radio_signature   # same algorithm the PC uses

IDX = '--idx' in sys.argv
ARGS = [a for a in sys.argv[1:] if a != '--idx']
BASE, LEN_OFF, MARK_OFF, PAGE = (0x1000, 0x1C, 0x20, 4096) if IDX else (0x400, 0x10, 0x14, 1024)
def ACK(c):          # staging reply byte: Valve's main fw counts '!' (0x21) acks; the HTC LPC wants the echo
    return ord(c)                      # both radios echo the command (Valve's too)
def AB(c):
    return bytes([ACK(c)])
SPIS = 0x40004000
NVMC = 0x4001E000
STAGE, STAGE_END = 0x10000, 0x1F800

class Fail(Exception): pass

class Radio:
    def __init__(self, img, prior_flash=None):
        self.img = img
        mu = self.mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        mu.mem_map(0, 0x20000)
        mu.mem_write(0, prior_flash if prior_flash else b'\xff' * 0x20000)
        mu.mem_write(BASE, img)
        mu.mem_map(0x10000000, 0x1000)                      # FICR
        mu.mem_write(0x10000010, struct.pack('<II', PAGE, 128))
        mu.mem_write(0x10000060, bytes(range(1, 9)))
        mu.mem_map(0x10001000, 0x1000)                      # UICR
        mu.mem_map(0x20000000, 0x10000)                     # RAM
        mu.mem_map(0x40000000, 0x30000)                     # APB peripherals
        mu.mem_map(0x50000000, 0x1000)                      # GPIO
        mu.mem_map(0xE0000000, 0x100000)                    # PPB (SCB, NVIC)
        mu.mem_write(NVMC + 0x400, struct.pack('<I', 1))    # READY
        mu.mem_map(0xF0000000, 0x1000)                      # part-ID registers (read by SystemInit)
        self.t_us = 0
        self.released = False
        self.reset = False
        self.nvmc_cfg = 0
        self.flash_ops = []
        self.max_sp_used = 0
        mu.hook_add(UC_HOOK_MEM_WRITE, self.on_write)
        mu.hook_add(UC_HOOK_MEM_READ, self.on_read, begin=0x40008540, end=0x40008540)
        sp, pc = struct.unpack_from('<II', img, 0)
        mu.reg_write(UC_ARM_REG_SP, sp)
        self.pc = pc
        self.sp0 = sp

    def w32(self, a, v): self.mu.mem_write(a, struct.pack('<I', v))
    def r32(self, a): return struct.unpack('<I', self.mu.mem_read(a, 4))[0]

    def on_read(self, mu, access, addr, size, value, ud):
        pass

    def on_write(self, mu, access, addr, size, value, ud):
        if addr < 0x20000:                                  # flash
            if self.nvmc_cfg != 1: raise Fail(f'flash write at {addr:#x} with NVMC.CONFIG={self.nvmc_cfg}')
            if not (STAGE <= addr < STAGE_END): raise Fail(f'flash write OUTSIDE staging at {addr:#x}')
            old = int.from_bytes(mu.mem_read(addr, size), 'little')
            self.pending_and = (addr, size, old & value)    # flash can only clear bits
            self.flash_ops.append(('w', addr, value))
            return
        if addr == NVMC + 0x504: self.nvmc_cfg = value
        elif addr == NVMC + 0x508:                          # ERASEPAGE
            if self.nvmc_cfg != 2: raise Fail('erase with CONFIG != 2')
            if not (STAGE <= value < STAGE_END) or value % PAGE: raise Fail(f'erase OUTSIDE staging {value:#x}')
            mu.mem_write(value, b'\xff' * PAGE); self.flash_ops.append(('e', value, 0))
        elif addr in (NVMC + 0x50C, NVMC + 0x514): raise Fail('ERASEALL / ERASEUICR')
        elif addr == SPIS + 0x028: self.released = True     # TASKS_RELEASE
        elif addr == SPIS + 0x024: self.w32(SPIS + 0x128, 1) # TASKS_ACQUIRE -> ACQUIRED
        elif addr == 0x40008040: self.t_us += 20; self.w32(0x40008540, self.t_us & 0xFFFFFFFF)
        elif addr == 0x40000000: self.w32(0x40000100, 1)     # HFCLKSTART -> HFCLKSTARTED
        elif addr == 0x50000508: self.pend_gpio = ('set', value)  # OUTSET / OUTCLR -> OUT (after the write)
        elif addr == 0x5000050C: self.pend_gpio = ('clr', value)
        elif addr == 0xE000ED0C and (value >> 16) == 0x05FA and value & 4:
            self.reset = True; mu.emu_stop()
        elif addr == SPIS + 0x500 and value not in (0, 2): raise Fail('SPIS ENABLE')
        sp = mu.reg_read(UC_ARM_REG_SP)
        self.max_sp_used = max(self.max_sp_used, self.sp0 - sp)

    def run(self, steps=200000):
        """Run until the CPU releases the SPI semaphore (or resets)."""
        self.pending_and = None
        n = 0
        while not self.released and not self.reset:
            try:
                self.mu.emu_start(self.pc | 1, 0xFFFFFFFF, count=2000)
            except UcError as e:
                raise Fail(f'CPU fault {e} at pc {self.mu.reg_read(UC_ARM_REG_PC):#x}')
            self.pc = self.mu.reg_read(UC_ARM_REG_PC)
            if getattr(self, 'pend_gpio', None):
                op, v = self.pend_gpio; o = self.r32(0x50000504)
                self.w32(0x50000504, o | v if op == 'set' else o & ~v); self.pend_gpio = None
            if self.pending_and:
                a, s, v = self.pending_and
                self.mu.mem_write(a, v.to_bytes(s, 'little')); self.pending_and = None
            n += 2000
            if n > steps: raise Fail('no SPI release (CPU stuck holding the semaphore?)')

    def xfer(self, frame):
        """One LPC transaction: clock `frame`, extend with 0xFF while the reply is longer; return the reply."""
        self.run()
        if self.reset: raise Fail('reset')
        tx, mtx = self.r32(SPIS + 0x544), self.r32(SPIS + 0x548)
        rx, mrx = self.r32(SPIS + 0x534), self.r32(SPIS + 0x538)
        txb = bytes(self.mu.mem_read(tx, mtx)) if mtx else b''
        pre = 1 if getattr(self, 'lead', False) and txb[:1] == b'\x00' else 0   # Index 'l' mode prefix
        txb = txb[pre:]
        out = lambda i: txb[i] if i < len(txb) else 0x00     # ORC
        total = len(frame)
        r0 = out(0)
        if r0 not in (0x00, 0xFE, 0xFF) and r0 > total: total = r0
        clocked = bytes(out(i) for i in range(total))
        mosi = (frame + b'\xff' * (total - len(frame)))
        self.mu.mem_write(rx, mosi[:mrx])
        self.w32(SPIS + 0x53C, min(total, mrx)); self.w32(SPIS + 0x54C, min(total + pre, mtx))
        self.w32(SPIS + 0x104, 1); self.w32(SPIS + 0x128, 1)   # END, short END->ACQUIRE
        self.released = False
        return clocked[1:r0] if r0 not in (0x00, 0xFE, 0xFF) else None

    def xfer_raw(self, frame):
        """Like xfer, but return every byte the master clocked (no framing interpretation)."""
        self.run()
        tx, mtx = self.r32(SPIS + 0x544), self.r32(SPIS + 0x548)
        txb = bytes(self.mu.mem_read(tx, mtx)) if mtx else b''
        total = max(len(frame), len(txb))
        rx, mrx = self.r32(SPIS + 0x534), self.r32(SPIS + 0x538)
        self.mu.mem_write(rx, (frame + b'\xff' * total)[:min(total, mrx)])
        self.w32(SPIS + 0x53C, min(total, mrx)); self.w32(SPIS + 0x54C, len(txb))
        self.w32(SPIS + 0x104, 1); self.w32(SPIS + 0x128, 1)
        self.released = False
        return txb + b'\x00' * (total - len(txb))

    def cmd(self, frame, expect):
        """Send a frame, then idle frames until the matching reply arrives (like the LPC's 10 ms poll)."""
        rep = self.xfer(frame)
        for _ in range(20):
            if rep and rep[0] == expect: return rep
            rep = self.xfer(b'\x00')
        raise Fail(f'no {chr(expect)!r} reply to {frame[:1]!r} pc {self.mu.reg_read(UC_ARM_REG_PC):#x} (last {rep[:8].hex() if rep else rep})')


def syms(elf):
    import subprocess
    out = subprocess.run(['arm-none-eabi-nm', elf], capture_output=True, text=True).stdout
    return {l.split()[2]: int(l.split()[0], 16) for l in out.splitlines() if len(l.split()) == 3}

class Air:
    """Drives the radio's over-the-air updater by calling handle_poll() with POLL packets."""
    RET = 0x1FFF0
    def __init__(self, r, sym):
        self.r, self.s, self.dseq = r, sym, 1
    def call(self, fn, *args):
        from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_LR
        mu = self.r.mu
        from unicorn import arm_const as A
        regs = [getattr(A, f'UC_ARM_REG_R{i}') for i in range(13)] + [A.UC_ARM_REG_SP, A.UC_ARM_REG_LR, A.UC_ARM_REG_XPSR]
        saved = [mu.reg_read(x) for x in regs]
        for reg, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1), args): mu.reg_write(reg, v)
        mu.reg_write(UC_ARM_REG_LR, self.RET | 1)
        self.r.pending_and = None
        try: mu.emu_start(fn | 1, self.RET, count=5_000_000)
        except UcError as e: raise Fail(f'CPU fault {e} at pc {mu.reg_read(UC_ARM_REG_PC):#x}')
        if mu.reg_read(UC_ARM_REG_PC) != self.RET: raise Fail(f'{fn:#x} did not return')
        for x, v in zip(regs, saved): mu.reg_write(x, v)
    def frames_up(self):
        r, s = self.r, self.s
        q = bytes(r.mu.mem_read(s['uq'], 1024))
        h = struct.unpack('<H', r.mu.mem_read(s['uq_head'], 2))[0]; t = struct.unpack('<H', r.mu.mem_read(s['uq_tail'], 2))[0]
        out = []
        while h != t:
            n = q[h]; f = bytes(q[(h + 1 + i) % 1024] for i in range(n)); out.append(f); h = (h + 1 + n) % 1024
        r.mu.mem_write(s['uq_head'], struct.pack('<H', t))
        return out
    def req(self, frame):
        self.dseq ^= 1
        pkt = bytes([0, 0, 0x10 | (8 if self.dseq else 0), 1, 0xFF, 0xFF, 40, 250, len(frame)]) + frame
        buf = 0x2000F000
        self.r.mu.mem_write(buf, pkt)
        self.call(self.s['handle_poll'], buf, len(pkt))
        up = [f for f in self.frames_up() if f[:1] == b'\xf0']
        return up[-1] if up else None

def check(name, cond):
    print(('  ok   ' if cond else '  FAIL ') + name)
    if not cond: raise Fail(name)

def main():
    fw = open(ARGS[0] if ARGS else os.path.join(HERE, '..', 'radio-fw', 'build-idx' if IDX else 'build', 'radio.bin'), 'rb').read()
    steamvr = os.environ.get('STEAMVR', os.path.expanduser('~/.local/share/Steam/steamapps/common/SteamVR'))
    stock = os.path.join(steamvr, 'drivers', *(('indexcontroller', 'resources', 'firmware', 'indexcontroller',
                         'indexcontroller_radio_20190711_v1562882729.fw') if IDX else
                         ('htc', 'resources', 'firmware', 'vr_neo_tracker', 'htc_vrc_module_1466630404_2016_06_22.bin')))
    if os.path.exists(stock):                           # the stock radio (Valve's unwrapped from its .fw container)
        htc = open(stock, 'rb').read()
        if IDX: htc = htc[:struct.unpack_from('<I', htc, 0x1C)[0]]
    else:
        print(f'(stock radio image not found at {stock}: staging our own image in its place)')
        htc = fw
    r = Radio(fw)
    print('boot + version')
    rep = r.cmd(b'X', 0x58)
    check(f"'X' -> version {struct.unpack_from('<I', rep, 1)[0]:#x}", len(rep) == 5)
    rep = r.cmd(b'c', 0x63); check("'c' -> device id", rep[1:9] == bytes(range(1, 9)))
    rep = r.cmd(b'O\x00', 0x4F); check(f"'O' -> safe-mode flags {rep[1]}", len(rep) == 2)

    if IDX:
        r.run(); check('P0.20 low with nothing pending', not (r.r32(0x50000504) >> 20) & 1)
        rep = r.cmd(b'l\x01', 0x6C); check("'l' 1 -> 'l' 01 (sent without the prefix)", rep == b'l\x01')
        r.xfer(b'X'); r.run()
        check('P0.20 high while a reply waits', (r.r32(0x50000504) >> 20) & 1)
        raw = r.xfer_raw(b'\x00')
        check(f"'l' mode: reply starts with 0x00 then [len] ({raw[:7].hex()})", raw[0] == 0 and raw[1] == 6 and raw[2] == 0x58)
        r.lead = True; check("'l' 0 -> 'l' 00 (still prefixed)", r.cmd(b'l\x00', 0x6C) == b'l\x00'); r.lead = False
        check('0x81 -> 82 01', r.cmd(b'\x81', 0x82) == b'\x82\x01')
    for label, image in (('our own image', fw), ('Valve Index radio image' if IDX else 'HTC 2016 image', htc)):
        print(f'stage {label} ({len(image)} B), 56-byte blocks like the LPC')
        img = image + b'\xff' * (-len(image) % 56)
        check("'Y' -> 59 00", r.cmd(b'Y', ACK('Y')) == AB('Y') + b'\x00')
        for off in range(0, len(img), 56):
            rep = r.cmd(b'Z' + img[off:off + 56], ACK('Z'))
            if rep != AB('Z') + b'\x00': check(f'Z at {off:#x}', False)
        staged = bytes(r.mu.mem_read(STAGE, len(image)))
        check('staged bytes == image', staged == image)
        check(f'staging +{MARK_OFF:#x} still blank', r.r32(STAGE + MARK_OFF) == 0xFFFFFFFF)
        sig = radio_signature(image)
        check("'[' wrong signature -> 5b 01", r.cmd(b'[' + bytes(b ^ 1 for b in sig), ACK('[')) == AB('[') + b'\x01')
        check('  ...no marker written', r.r32(STAGE + MARK_OFF) == 0xFFFFFFFF)
        check("'Y' again", r.cmd(b'Y', ACK('Y')) == AB('Y') + b'\x00')
        for off in range(0, len(img), 56): r.cmd(b'Z' + img[off:off + 56], ACK('Z'))
        check("'[' right signature -> 5b 00", r.cmd(b'[' + sig, ACK('[')) == AB('[') + b'\x00')
        check(f'  ...marker 0x01051974 at +{MARK_OFF:#x}', r.r32(STAGE + MARK_OFF) == 0x01051974)
        check("'[' again without 'Y' is refused", r.cmd(b'[' + sig, ACK('[')) == AB('[') + b'\x01')

    print('updater with the radio link running (t > 1 s)')
    r.t_us = 3_000_000
    for _ in range(5): r.xfer(b'\x00')
    check('link started (RADIO.POWER = 1)', r.r32(0x40001FFC) == 1)
    check("'X' still answered", len(r.cmd(b'X', 0x58)) == 5)
    img = htc + b'\xff' * (-len(htc) % 56)
    check("'Y' -> 59 00", r.cmd(b'Y', ACK('Y')) == AB('Y') + b'\x00')
    for off in range(0, len(img), 56): r.cmd(b'Z' + img[off:off + 56], ACK('Z'))
    check("'[' right signature -> 5b 00", r.cmd(b'[' + radio_signature(htc), ACK('[')) == AB('[') + b'\x00')
    print('bad inputs')
    check("'Z' without 'Y' -> 5a 01", r.cmd(b'Z' + b'\0' * 8, ACK('Z')) == AB('Z') + b'\x01')
    r.cmd(b'Y', ACK('Y'))
    check("'Z' unaligned length -> 5a 01", r.cmd(b'Z' + b'\0' * 7, ACK('Z')) == AB('Z') + b'\x01')
    check('0x20 while staging open -> refused', r.cmd(b'\x20', 0x20) == b'\x20\x01')

    print('over-the-air updater (downlink 0xF0 frames through handle_poll)')
    a = Air(r, syms(os.path.join(HERE, '..', 'radio-fw', 'build-idx' if IDX else 'build', 'radio.elf')))
    rep = a.req(b'\xf0X')
    check(f"F0 'X' -> version {struct.unpack_from('<I', rep, 2)[0]:#x}", rep and rep[1:2] == b'X' and struct.unpack_from('<I', rep, 2)[0] == struct.unpack_from('<I', fw, LEN_OFF + 8)[0])
    for label, image in (('Valve image' if IDX else 'HTC image', htc), ('our own image', fw)):
        print(f'  air-stage {label} ({len(image)} B), 48-byte blocks')
        pad = image + b'\xff' * (-len(image) % 4)
        check("  F0 'Y' -> 0", a.req(b'\xf0Y') == b'\xf0Y\x00')
        check('  link not stopped by air staging', r.r32(a.s['stg_air']) & 0xFF == 1)
        for off in range(0, len(pad), 48):
            blk = pad[off:off + 48]
            rep = a.req(b'\xf0Z' + struct.pack('<I', off) + blk)
            if rep[2] != 0 or struct.unpack_from('<I', rep, 3)[0] != off + len(blk): check(f'  Z at {off:#x} -> {rep.hex()}', False)
            if off == 96:
                rep = a.req(b'\xf0Z' + struct.pack('<I', off) + blk)
                check('  resent block is acked, not rewritten', rep[2] == 0 and struct.unpack_from('<I', rep, 3)[0] == off + 48)
                rep = a.req(b'\xf0Z' + struct.pack('<I', off + 480) + blk)
                check('  out-of-order block refused (3)', rep[2] == 3)
        check('  staged bytes == image', bytes(r.mu.mem_read(STAGE, len(image))) == image)
        sig = radio_signature(image)
        check("  F0 '[' wrong signature -> 1", a.req(b'\xf0[' + bytes(x ^ 1 for x in sig)) == b'\xf0[\x01')
        check('    ...no marker', r.r32(STAGE + MARK_OFF) == 0xFFFFFFFF)
        a.req(b'\xf0Y')
        for off in range(0, len(pad), 48): a.req(b'\xf0Z' + struct.pack('<I', off) + pad[off:off + 48])
        check("  F0 '[' right signature -> 0", a.req(b'\xf0[' + sig) == b'\xf0[\x00')
        check('    ...marker 0x01051974', r.r32(STAGE + MARK_OFF) == 0x01051974)
    check("F0 'Z' without 'Y' -> 2", a.req(b'\xf0Z' + bytes(4) + bytes(8))[2] == 2)
    r.cmd(b'Y', ACK('Y'))
    check("main MCU 'Y' takes staging over (air Z refused)", a.req(b'\xf0Z' + bytes(4) + bytes(8))[2] == 2)
    r.cmd(b'[' + bytes(16), ACK('['))
    rep = a.req(b'\xf0R'); check("F0 'R' -> 0, reset scheduled", rep == b'\xf0R\x00' and r.r32(a.s['air_reset_at']) != 0)
    r.mu.mem_write(a.s['air_reset_at'], bytes(4))
    print('reset')
    r.xfer(b'\\RESET'); r.run()
    check('\\RESET -> system reset', r.reset)
    print(f'flash ops: {sum(1 for o in r.flash_ops if o[0] == "e")} erases, {sum(1 for o in r.flash_ops if o[0] == "w")} writes, all inside staging')
    print(f'stack used: {r.max_sp_used} B')
    print('ALL OK')

if __name__ == '__main__':
    try: main()
    except Fail as e: sys.exit(f'FAILED: {e}')
