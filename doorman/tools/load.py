#!/usr/bin/env python3
"""Load tests: make a few devices look like a busy eight-device dongle.

  load.py dongle MASK KBPS     the dongle queues filler at KBPS kB/s on each slot in MASK (0xFF = all 8);
                               "load.py dongle 0 0" stops it. Loads the dongle's queues, USB and the host
  load.py radio SLOT KBPS      the device in SLOT sends KBPS kB/s of filler over the air (0 stops it).
                               Loads the radio cycle too. Not kept across the device's reboot
  load.py stats [SECS]         per slot over SECS (default 5): polls, loss, data, drops, test load, USB queue peak
  load.py gen SLOT BASES [DENSITY]
                               the device in SLOT (Index-family radio or a Pro Micro test device) sends a
                               synthetic LH2 stream for BASES Base Station 2.0s (0 stops it; DENSITY 2 ~ real
                               captures, higher loads the link past them). Run without the driver: it would
                               take the made-up IMU for the device's own
  load.py sweep GEN FILLER [DENSITIES] [SECS]
                               GEN, FILLER: comma-separated slots ("-" for none). Steps GEN's density through
                               DENSITIES (default 1-8), FILLER (radios without the LH2 coder, e.g. Tracker 1.0)
                               sending filler at GEN's average on-air rate, so every device loads the link alike;
                               measures each step for SECS (default 15) and stops once data is dropped

Filler frames are type 'Z': the dongle and the host carry them like data, the driver skips them.
Runs next to the driver (its commands use seq 128..255).
"""
import os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doorman_link import Dongle

def cmd(d, c, a=b''):
    for _ in range(10):
        try: return d.cmd(c, a, 200)
        except TimeoutError: pass
    sys.exit(f'no reply to {c:#x}')

def snap(d):
    out = {}
    for k in range(d.slot_count()):
        s = struct.unpack_from('<15I', cmd(d, 0x23, bytes([k]))[1])
        c = cmd(d, 0x28, bytes([k]))[1]
        out[k] = (s, struct.unpack_from('<H', c, 20)[0], struct.unpack_from('<I', c, 22)[0])
    return out

def stats(d, secs):
    used = cmd(d, 0x25, b'\x00')[1], cmd(d, 0x25, b'\x06')[1]
    a = snap(d); time.sleep(secs); b = snap(d)
    print(f'{"slot":>4} {"polls/s":>7} {"loss%":>6} {"data kB/s":>9} {"drops/s":>8} {"load kB/s":>9} {"USB queue peak":>14}')
    for k in range(d.slot_count()):
        sa, sb = a[k][0], b[k][0]
        polls, ok = sb[0] - sa[0], sb[1] - sa[1]
        data = (sb[6] - sa[6]) / secs / 1000
        drops = (sb[7] - sa[7]) / secs
        load = (b[k][2] - a[k][2]) / secs / 1000
        paired = (used[0] if k < 6 else used[1])[(k % 6) * 10]
        if not polls and not load and not drops: continue
        print(f'{k:>4} {polls / secs:>7.0f} {100 * (1 - ok / polls) if polls else 0:>6.1f} {data:>9.1f} {drops:>8.1f} {load:>9.1f} {b[k][1]:>12} B'
              + ('' if paired else '  (no device)'))

def timing(d):
    t = cmd(d, 0x2D, b'\xff')[1]
    period, slot = struct.unpack_from('<II', t, 0)
    return {'cycle_ms': period / 1000, 'answer': t[21], 'sat': t[30], 'large': t[31], 'normal': t[32], 'devices': t[33],
            'slot_sat': list(t[34:34 + d.slot_count()])}

def counters(d, k):
    s = struct.unpack_from('<15I', cmd(d, 0x23, bytes([k]))[1])
    # s[5]: frames delivered
    c = cmd(d, 0x28, bytes([k]))[1]
    air, _, _, dev_drops = struct.unpack_from('<4I', c, 0)
    return {'polls': s[0], 'ok': s[1], 'frames': s[5], 'data': s[6], 'drops': s[7], 'air': air, 'dev_drops': dev_drops}

def measure(d, slots, secs):
    a = {k: counters(d, k) for k in slots}; time.sleep(secs); b = {k: counters(d, k) for k in slots}
    out = {}
    for k in slots:
        x, y = a[k], b[k]
        frames = None
        polls = y['polls'] - x['polls']
        out[k] = {'loss': 100 * (1 - (y['ok'] - x['ok']) / polls) if polls else 0, 'data': (y['data'] - x['data']) / secs / 1000,
                  'air': (y['air'] - x['air']) / secs / 1000, 'drops': (y['drops'] - x['drops']) / secs,
                  'dev_drops': ((y['dev_drops'] - x['dev_drops']) & 0xFFFF) / secs, 'frames': (y['frames'] - x['frames']) / secs}
    return out

def slots_arg(s):
    return [] if s == '-' else [int(x) for x in s.split(',')]

def sweep(d, gen, filler, densities, secs):
    from air_update import Air
    slots = gen + filler
    print(f'gen {gen}  filler {filler}  ({secs:.0f} s a step)')
    print(f'{"dens":>4} {"data kB/s":>9} {"air kB/s":>8} {"total air":>9} {"loss%":>6} {"drops/s":>8} {"dev drops/s":>11} {"sat%":>5} {"answer":>7} {"cycle ms":>8}')
    for dens in densities:
        for k in gen: Air(d, k).req('G', bytes([4, dens]))
        time.sleep(3)
        if filler:   # filler at what a generating device puts on the air now
            probe = measure(d, gen, 3)
            rate = max(1, round(sum(v['air'] for v in probe.values()) / len(gen)))
            for k in filler: Air(d, k).req('L', bytes([min(255, rate)]))
            time.sleep(2)
        m = measure(d, slots, secs); t = timing(d)
        n = len(slots)
        data = sum(v['data'] for v in m.values()) / n; air = sum(v['air'] for v in m.values())
        loss = sum(v['loss'] for v in m.values()) / n
        drops = sum(v['drops'] for v in m.values()); dev = sum(v['dev_drops'] for v in m.values())
        ans = f"{t['answer']}{'L' if t['large'] else ''}"
        frames = sum(v['frames'] for v in m.values())
        lost = 100 * (drops + dev) / max(1, frames + drops + dev)
        who = ' '.join(f"{k}:{v['drops'] + v['dev_drops']:.0f}" for k, v in m.items() if v['drops'] + v['dev_drops'] >= 0.5)
        print(f'{dens:>4} {data:>9.1f} {air / n:>8.1f} {air:>9.1f} {loss:>6.1f} {drops:>8.1f} {dev:>11.1f} {t["sat"]:>5} {ans:>7} {t["cycle_ms"]:>8.1f}'
              f'   lost {lost:.2f} %' + (f'  ({who})' if who else ''))
        if lost > 1:
            print('over 1 % of frames dropped: past what the link carries'); break
    for k in gen: Air(d, k).req('G', bytes([0]))
    for k in filler: Air(d, k).req('L', bytes([0]))

def main():
    a = sys.argv[1:]
    if not a: sys.exit(__doc__)
    with Dongle() as d:
        if a[0] == 'dongle' and len(a) == 3:
            r = cmd(d, 0x2B, bytes([int(a[1], 0), int(a[2])]))[1]
            print(f'dongle test load: mask {r[0]:#04x}, {r[1]} kB/s per slot')
        elif a[0] == 'radio' and len(a) == 3:
            from air_update import Air
            r = Air(d, int(a[1])).req('L', bytes([int(a[2])]))
            print(f'slot {a[1]}: radio test load {r[2]} kB/s')
        elif a[0] == 'gen' and len(a) >= 3:
            from air_update import Air
            r = Air(d, int(a[1])).req('G', bytes([int(a[2]), int(a[3]) if len(a) > 3 else 2]))
            print(f'slot {a[1]}: synthetic LH2 load, {r[2]} bases, density {r[3]}')
        elif a[0] == 'sweep' and len(a) >= 3:
            dens = range(1, 9)
            if len(a) > 3:
                lo, _, hi = a[3].partition('-'); dens = range(int(lo), int(hi or lo) + 1)
            sweep(d, slots_arg(a[1]), slots_arg(a[2]), dens, float(a[4]) if len(a) > 4 else 15)
        elif a[0] == 'stats':
            stats(d, float(a[1]) if len(a) > 1 else 5)
        else:
            sys.exit(__doc__)

if __name__ == '__main__':
    main()
