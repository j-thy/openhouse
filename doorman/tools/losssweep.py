#!/usr/bin/env python3
"""Link under injected loss: every Pro Micro test device (testdev.py) loses LOSS % of its exchanges (bursts of BURST on
average); per step, for each slot: answers lost (dongle), frames dropped on the device, IMU samples and light frames
delivered per second over USB, and the device queue's peak (testdev slots).

  losssweep.py [LOSSES] [BURST] [SECS]      LOSSES: comma-separated % (default 0,2,5,10,15,20,30)
Run without the driver (it would take the test devices' made-up IMU for real)."""
import struct, sys, time, collections
import testdev, load
from doorman_link import Dongle


def frame_rates(d, secs, test_slots):
    """IMU and light frames per second per slot; for the test devices' IMU (device time in bytes 13-16, 48 MHz) also
    the delay over the fastest sample seen: p50 / p99 / max, ms."""
    imu, light = collections.Counter(), collections.Counter()
    lag = collections.defaultdict(list)
    last, wraps = {}, collections.Counter()
    t = time.time()
    while time.time() - t < secs:
        for s, f in d.frames(raw=True, timeout_ms=20):
            if f[:1] == b'p':
                imu[s] += 1
                if s in test_slots and len(f) >= 17:
                    raw = struct.unpack_from('<I', f, 13)[0]
                    prev = last.get(s)
                    if prev is not None and raw < prev[0] and prev[0] - raw > 1 << 31: wraps[s] += 1   # 48 MHz count wraps every ~89 s
                    last[s] = (raw,)
                    dev = (raw + wraps[s] * (1 << 32)) / 48000.0
                    lag[s].append(time.time() * 1000 - dev)
            elif f[:1] == b's': light[s] += 1
    delay = {}
    for k, v in lag.items():
        v = sorted(x - min(v) for x in v)
        if v: delay[k] = (v[len(v) // 2], v[int(len(v) * 0.99)], v[-1])
    return {k: x / secs for k, x in imu.items()}, {k: x / secs for k, x in light.items()}, delay


def queue_peaks():
    out = {}
    for x in testdev.devices():
        dv = testdev.Dev(x)
        try:
            for k in range(4):
                r = dv.cmd(0x08, bytes([k]))[1:]
                if r[12]: out[r[2]] = struct.unpack_from('<H', r, 17)[0]
        finally: dv.close()
    return out


def main():
    a = sys.argv[1:]
    losses = [int(x) for x in a[0].split(',')] if a else [0, 2, 5, 10, 15, 20, 30]
    burst = int(a[1]) if len(a) > 1 else 3
    secs = float(a[2]) if len(a) > 2 else 15
    with Dongle() as d:
        d.cmd(0x37, b'\x01')
        n = d.slot_count()
        for L in losses:
            for x in testdev.devices():
                dv = testdev.Dev(x)
                try: dv.cmd(0x05, bytes([L, burst, 100]))
                finally: dv.close()
            time.sleep(4)
            queue_peaks()                                    # resets the peaks
            m = load.measure(d, range(n), secs)
            qp = queue_peaks()
            imu, light, delay = frame_rates(d, 15, set(qp))
            t = load.timing(d)
            print(f'\n== injected loss {L} % (bursts of {burst})   cycle {t["cycle_ms"]} ms, answers {t["answer"]} B{" large" if t["large"] else ""}, saturation {t["sat"]} %')
            print(f'{"slot":>4} {"lost%":>6} {"dev drops/s":>11} {"IMU/s":>6} {"light/s":>8} {"queue peak":>10} {"IMU delay ms p50/p99/max":>26}')
            for k in range(n):
                v = m[k]
                dl = '/'.join(f'{x:.0f}' for x in delay[k]) if k in delay else ''
                print(f'{k:>4} {v["loss"]:>6.1f} {v["dev_drops"]:>11.1f} {imu.get(k, 0):>6.0f} {light.get(k, 0):>8.0f} {qp.get(k, ""):>10} {dl:>26}')
        for x in testdev.devices():
            dv = testdev.Dev(x)
            try: dv.cmd(0x05, bytes([0, 1, 100]))
            finally: dv.close()


if __name__ == '__main__':
    main()
