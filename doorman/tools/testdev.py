#!/usr/bin/env python3
"""Pro Micro test devices (radio-fw TARGET=fake) over USB.

  testdev.py list                      the test devices plugged in: serial, version, link, slot, load, loss
  testdev.py flash [SERIAL|all]        UF2 bootloader -> copy the matching image (SoftDevice or not) -> back
                                       (default: all; builds with `make TARGET=fake` first)
  testdev.py gen SERIAL|all BASES [DENSITY]     synthetic LH2 load (density 4 ~ a Tundra Tracker)
  testdev.py loss SERIAL|all PCT [BURST [ANSWER_PCT]]
                                       injected loss: PCT % of exchanges, in bursts of BURST on average;
                                       ANSWER_PCT % of them lose the answer (the poll is processed), the rest
                                       lose the poll. 0 stops it
  testdev.py pair SERIAL|all [K]       pairing mode for 35 s, all its slots or its K-th (open pairing on the dongle too)
  testdev.py slots SERIAL|all N        answer for N dongle slots (1-4); each has its own device id and pairs once
  testdev.py dfu SERIAL                just the UF2 bootloader
"""
import glob, os, shutil, struct, subprocess, sys, time
import hid

VID, PID = 0x6767, 0x0728
HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, '..', 'radio-fw')


def devices():
    return sorted({d['serial_number']: d for d in hid.enumerate(VID, PID)}.values(), key=lambda d: d['serial_number'])


class Dev:
    def __init__(self, info):
        self.serial = info['serial_number']
        self.h = hid.Device(path=info['path']); self.seq = 0

    def close(self): self.h.close()

    def cmd(self, c, args=b'', timeout=1.0, want_reply=True):
        # Control endpoint only (feature reports): works on USB ports that drop interrupt transfers.
        self.seq = (self.seq + 1) & 0xFF
        self.h.send_feature_report(bytes([0, c, self.seq]) + bytes(args) + bytes(62 - len(args)))
        if not want_reply: return None
        t = time.time()
        while time.time() - t < timeout:
            r = self.h.get_feature_report(0, 65)
            r = r[1:] if len(r) == 65 else r                 # with or without the report ID byte
            if r and r[0] == c and r[1] == self.seq: return bytes(r[2:])
            time.sleep(0.01)
        raise TimeoutError(f'{self.serial}: no reply to {c:#x}')

    def slot(self, k):
        r = self.cmd(0x08, bytes([k]))[1:]
        return {'joined': r[0], 'link': r[1], 'slot': r[2], 'pairing': r[3], 'id': r[4:12][::-1].hex(), 'active': r[12]}

    def info(self):
        r = self.cmd(0x01)[1:]
        v, = struct.unpack_from('<I', r, 0)
        up, ex, lost = struct.unpack_from('<III', r, 21)
        return {'version': v, 'id': r[4:12].hex(), 'link': r[12], 'joined': r[13], 'slot': r[14], 'pairing': r[15],
                'bases': r[16], 'density': r[17], 'loss': r[18], 'burst': r[19], 'answer_pct': r[20],
                'uptime_s': up / 1000, 'exchanges': ex, 'lost': lost, 'sd_build': r[33],
                'events_dropped': struct.unpack_from('<I', r, 34)[0], 'loop_max_us': struct.unpack_from('<I', r, 38)[0]}


def pick(arg):
    ds = devices()
    if arg in (None, 'all'): return ds
    out = [d for d in ds if d['serial_number'].upper().startswith(arg.upper())]
    if not out: sys.exit(f'no test device {arg}')
    return out


def uf2_drives():
    out = {}
    for m in glob.glob('/run/media/*/*') + glob.glob('/media/*/*') + glob.glob('/mnt/*'):
        p = os.path.join(m, 'INFO_UF2.TXT')
        if os.path.isfile(p):
            try: out[m] = open(p).read()
            except OSError: pass
    return out


def flash_one(info):
    serial = info['serial_number']
    before = set(uf2_drives())
    d = Dev(info)
    try: d.cmd(0x02, want_reply=False)
    finally: d.close()
    t = time.time(); drive = None
    while time.time() - t < 20 and not drive:
        time.sleep(0.5)
        new = {m: txt for m, txt in uf2_drives().items() if m not in before}
        if new: drive, txt = next(iter(new.items()))
    if not drive: sys.exit(f'{serial}: no UF2 drive appeared (mounting may need a file manager or udisks)')
    sd = 'SoftDevice: not found' not in txt
    img = os.path.join(FW, 'build-fake', f"radio_fake_{'sd' if sd else 'nosd'}.uf2")
    print(f'{serial}: bootloader at {drive} ({"SoftDevice" if sd else "no SoftDevice"}), copying {os.path.basename(img)}')
    shutil.copyfile(img, os.path.join(drive, 'firmware.uf2'))
    os.sync()
    t = time.time()
    while time.time() - t < 30:
        time.sleep(0.5)
        back = [x for x in devices() if x['serial_number'] == serial]
        if back:
            time.sleep(0.5)
            dv = Dev(back[0])
            try: i = dv.info()
            finally: dv.close()
            print(f"{serial}: back, version {i['version']:#010x}, {'SoftDevice' if i['sd_build'] else 'no-SoftDevice'} build")
            return
    sys.exit(f'{serial}: did not come back on USB')


def main():
    a = sys.argv[1:]
    if not a: sys.exit(__doc__)
    if a[0] == 'list':
        for x in devices():
            d = Dev(x)
            try: i = d.info()
            finally: d.close()
            d = Dev(x)
            try:
                ks = [d.slot(k) for k in range(4)]
            except TimeoutError:
                ks = None
            finally: d.close()
            def st(k):
                return f"slot {k['slot']}" if k['link'] else ('pairing' if k['pairing'] else 'not connected')
            lossr = 100 * i['lost'] / i['exchanges'] if i['exchanges'] else 0
            slots = ', '.join(f"{st(k)} [{k['id']}]" for k in ks if k['active']) if ks else st(i)
            print(f"{x['serial_number']}  v{i['version']:#010x}  gen {i['bases']} bases d{i['density']}  "
                  f"loss {i['loss']}% burst {i['burst']} ({lossr:.1f}% so far)  up {i['uptime_s']:.0f} s\n    {slots}")
        if not devices(): print('no test devices on USB')
    elif a[0] == 'flash':
        subprocess.run(['make', '-s', 'TARGET=fake'], cwd=FW, check=True, stdout=subprocess.DEVNULL)
        for x in pick(a[1] if len(a) > 1 else 'all'): flash_one(x)
    elif a[0] == 'dfu' and len(a) == 2:
        for x in pick(a[1]):
            d = Dev(x)
            try: d.cmd(0x02, want_reply=False)
            finally: d.close()
    elif a[0] == 'slots' and len(a) == 3:
        for x in pick(a[1]):
            d = Dev(x)
            try: print(f"{x['serial_number']}: {d.cmd(0x07, bytes([int(a[2])]))[1]} slots")
            finally: d.close()
    elif a[0] in ('gen', 'loss', 'pair') and len(a) >= 2:
        for x in pick(a[1]):
            d = Dev(x)
            try:
                if a[0] == 'gen': r = d.cmd(0x04, bytes([int(a[2]), int(a[3]) if len(a) > 3 else 4])); print(f"{x['serial_number']}: gen {r[1]} bases d{r[2]}")
                elif a[0] == 'loss':
                    r = d.cmd(0x05, bytes([int(a[2]), int(a[3]) if len(a) > 3 else 1, int(a[4]) if len(a) > 4 else 100]))
                    print(f"{x['serial_number']}: loss {r[1]}% burst {r[2]} answer {r[3]}%")
                else:
                    d.cmd(0x06, bytes([int(a[2]) if len(a) > 2 else 0xFF]))
                    print(f"{x['serial_number']}: pairing" + (f' its slot {a[2]}' if len(a) > 2 else ''))
            finally: d.close()
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
