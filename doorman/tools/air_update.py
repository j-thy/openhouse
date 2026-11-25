#!/usr/bin/env python3
"""Radio firmware update over the openhouse link, through our nRF52840 dongle (no main MCU, no USB
cable to the device). Works for any device running our radio firmware: Index controller (Valve's
radio images can be put back this way) and the Tracker 1.0.

  air_update.py info    SLOT
  air_update.py install SLOT IMAGE [--no-reset]     IMAGE: .fw (SteamVR or ours) or a raw .bin
      then powers the device off; turn it back on to install (it installs at boot)
      --reboot: reboot it instead and check the version automatically (headless); a Tracker 1.0
                turns off instead (its main firmware), and installs when you turn it back on

Protocol: radio-fw/src/main.c air_update() (downlink frame op 0xF0 'X'/'Y'/'Z'/'['/'R').
"""
import os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doorman_link import Dongle
from radio_image import radio_signature

OP = 0xF0
BLOCK = 48

def load(path):
    m = open(path, 'rb').read()
    for hl in (0x1C, 0x10):                               # Index family / nRF51 header offset
        n = struct.unpack_from('<I', m, hl)[0]
        if 0x400 < n <= len(m) and n <= 0xF800:
            return m[:n]
    sys.exit('no valid image length in the header')

class Air:
    def __init__(self, d, slot):
        self.d, self.slot = d, slot
    def req(self, sub, payload=b'', want=None, timeout=1.5, tries=4):
        f = bytes([OP, ord(sub)]) + payload
        for _ in range(tries):
            self.d.send(self.slot, f)
            t = time.time()
            while time.time() - t < timeout:
                for s, fr in self.d.frames(raw=True, timeout_ms=50):
                    if s == self.slot and len(fr) >= 3 and fr[0] == OP and fr[1] == ord(sub):
                        if want is None or want(fr): return fr
        raise TimeoutError(f'no reply to {sub!r}')

def info(a):
    r = a.req('X')
    ver, op, off = struct.unpack_from('<IBI', r, 2)
    print(f'radio version {ver:#010x} ({ver}), staging open {op}, offset {off}')
    if len(r) >= 36:
        fr, bad = struct.unpack_from('<II', r, 11); fl = r[19]
        print(f'  SPI frames {fr}, empty {bad}, last rx {r[20]} B tx {r[21]} B: {r[22:22 + min(12, r[20])].hex(" ")}')
        print(f'  safe {fl & 1} safe_req {fl >> 1 & 1} sleep {fl >> 2 & 1} lead0 {fl >> 3 & 1}, replies queued {r[34]} B, quick boots {r[35]}')
    if len(r) == 60:                                      # main-MCU ops log (non-diagnostic builds)
        ops = [r[36 + 4 * i:40 + 4 * i] for i in range(6)]
        print('  main MCU ops, oldest first: ' + (', '.join(f"{chr(o[0]) if 32 < o[0] < 127 else hex(o[0])} {o[1]:02x} {o[2]:02x} x{o[3]}" for o in ops if o[3]) or 'none'))
        return ver
    if len(r) >= 47:
        boots, ny, nz, nc = struct.unpack_from('<HHHH', r, 36)
        print(f'  since power-on: boots {boots} (last reset reason {r[45]:#04x}), main MCU staging: Y {ny}, Z {nz}, commit {nc} (last status {r[44]})')
    if len(r) >= 59:
        zbad, aq, ao, _, zmax = struct.unpack_from('<HHHHI', r, 47)
        print(f'  Z failed {zbad}, slowest Z {zmax} us, acks queued {aq} / read by main MCU {ao}, staging scrub {r[46]:#x} (pages, 0xee = kept pending image)')
    return ver

def main():
    args = [x for x in sys.argv[1:] if not x.startswith('--')]
    if len(args) < 2: sys.exit(__doc__)
    with Dongle() as d:
        a = Air(d, int(args[1]))
        info(a)
        if args[0] == 'info': return
        if args[0] == 'trace':                             # SPI transfer trace from the first SPI 'Y'
            i, n = 0, None
            while n is None or i < n:
                r = a.req('D', bytes([i]), want=lambda fr, i=i: len(fr) > 2 and fr[2] == i)
                n = r[3]
                for k in range(5):
                    if i + k >= n: break
                    e = r[4 + 10 * k: 14 + 10 * k]
                    dt = (e[0] | e[1] << 8) / 10
                    print(f'{i + k:3d} +{dt:7.1f} ms  rx {e[2]:3d} B [{e[4]:02x} {e[5]:02x}]  tx {e[3]:3d} B [{e[6]:02x} {e[7]:02x} {e[8]:02x}]  status {e[9]:x}')
                i += 5
            if not n: print('no trace (no SPI Y since power-on)')
            return
        img = load(args[2])
        pad = img + b'\xff' * (-len(img) % 4)
        sig = radio_signature(img)
        print(f'image {len(img)} B, signature {sig.hex()}')
        if a.req('Y')[2]: sys.exit('open refused')
        t0 = time.time()
        for off in range(0, len(pad), BLOCK):
            blk = pad[off:off + BLOCK]
            r = a.req('Z', struct.pack('<I', off) + blk, want=lambda fr, e=off + len(blk): struct.unpack_from('<I', fr, 3)[0] == e or fr[2] not in (0,))
            if r[2]: sys.exit(f'write failed at {off:#x}: status {r[2]}, radio offset {struct.unpack_from("<I", r, 3)[0]}')
            if off % 4800 == 0: print(f'  {off}/{len(pad)}', flush=True)
        print(f'sent in {time.time() - t0:.1f} s')
        if a.req('[', sig)[2]: sys.exit('commit FAILED (signature mismatch): nothing installed, the current radio stays')
        print('committed')
        if '--no-reset' in sys.argv:
            print('restart the device to install'); return
        if '--reboot' not in sys.argv:
            power_off(d, int(args[1])); return
        reboot(d, a, int(args[1]), struct.unpack_from('<I', img, 0x24 if struct.unpack_from('<I', img, 0x1C)[0] == len(img) else 0x18)[0])

def active(d, slot):
    return d.slots().get(slot, {}).get('active', False)

def power_off(d, slot):
    # Power the device off (host command 0xff 0x9f "off!", tunnelled as 'P' 9f 04 off!). The user turns it
    # back on; its main MCU resets the radio at boot and the radio's bootloader installs the image.
    d.send(slot, bytes([ord('P'), 0x9f, 4]) + b'off!')
    t = time.time()
    while active(d, slot) and time.time() - t < 5: time.sleep(0.1)
    print('device powered off: turn it back on to install' if not active(d, slot) else
          'device did not power off: restart it by hand to install')
    print('then check: air_update.py info SLOT (our firmware) or lighthouse_console version (Valve/HTC)')

def reboot(d, a, slot, want):
    # Reboot the whole device (host command 0xff 0x95, tunnelled as 'P' 95 00): its main MCU resets the
    # radio at boot and the radio's bootloader installs the committed image. A device whose main MCU
    # doesn't take it (or a radio on its own) gets the radio-only reset ('R').
    d.send(slot, bytes([ord('P'), 0x95, 0x00]))
    t = time.time()
    while active(d, slot) and time.time() - t < 3: time.sleep(0.1)
    tracker10 = want >> 24 == 0x0A
    if active(d, slot):
        print('device did not reboot; resetting the radio alone'); a.req('R')
    elif tracker10:
        # HTC's Tracker 1.0 main firmware turns the device off on 0x95 instead of rebooting it.
        print('device turned off (a Tracker 1.0 does that): turn it back on to install')
    else:
        print('device rebooting to install')
    if want >> 24 not in (0x0A, 0x6A):              # not our firmware (e.g. Valve's radio put back)
        print(f'installing {want} ({want:#x}): not our firmware, so it will not rejoin this dongle; '
              'check it through a Valve/HTC dongle (lighthouse_console: version)')
        return
    t = time.time()
    wait = 120 if tracker10 else 30
    while not active(d, slot) and time.time() - t < wait: time.sleep(0.2)
    if not active(d, slot): sys.exit(f'device did not come back within {wait} s')
    time.sleep(0.5)
    ver = info(a)
    print('INSTALLED' if ver == want else f'version {ver:#x}, expected {want:#x}: NOT installed (the old radio is still running)')

if __name__ == '__main__':
    main()
