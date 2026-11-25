#!/usr/bin/env python3
"""Times the radio's batch path on an emulated Cortex-M4 (Unicorn) with real frames, and checks every batch
against the host build (batch_ref). Counts instructions per frame pushed and the longest run between two
DMC_YIELDs; prints both as us at 64 MHz for CPI 1.25 and 2.0 (Unicorn counts instructions, not cycles).
  run_emu.py BUILD_DIR FRAMES DEVICE LIMIT ANSWER_BYTES POLL_MS      (run.sh builds and calls it)
Needs unicorn (pip install unicorn)."""
import sys, struct, subprocess
from unicorn import *
from unicorn.arm_const import *
E, frames, dev, limit, answer, poll = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), float(sys.argv[6])
syms = {l.split()[2]: int(l.split()[0], 16) for l in subprocess.run(['arm-none-eabi-nm', f'{E}/radio_batch.elf'], capture_output=True, text=True).stdout.splitlines()}
code = open(f'{E}/radio_batch.bin', 'rb').read()
mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS); mu.mem_map(0, 0x80000); mu.mem_write(0, code); mu.mem_map(0x20000000, 0x10000)
STOP = 0x7FF00; st = {'n': 0, 'since': 0, 'gap': 0}
def oc(uc, a, s, u): st['n'] += 1; st['since'] += 1
def oy(uc, a, s, u): st['gap'] = max(st['gap'], st['since']); st['since'] = 0
mu.hook_add(UC_HOOK_CODE, oc, begin=0, end=len(code)); mu.hook_add(UC_HOOK_CODE, oy, begin=syms['yield_hook'], end=syms['yield_hook'])
def call(fn, *a):
    for i, v in enumerate(a): mu.reg_write(UC_ARM_REG_R0 + i, v)
    mu.reg_write(UC_ARM_REG_SP, 0x20010000 - 16); mu.reg_write(UC_ARM_REG_LR, STOP | 1); mu.emu_start(syms[fn] | 1, STOP, count=10_000_000)
    return mu.reg_read(UC_ARM_REG_R0)
call('init', answer)
# reference: the same path on the host (batch_ref)
ref = open(f'{E}/batch_ref.bin', 'rb').read(); ri = 0; bad = 0
d = open(frames, 'rb').read(); i = 0; pushes = []; t0 = None; nb = 0; tot = 0; tspan = 0
def check(m):
    global ri, bad, nb
    out = bytes(mu.mem_read(syms['out_buf'], m)); rl = ref[ri] | ref[ri+1] << 8; r = ref[ri+2:ri+2+rl]; ri += 2 + rl
    if r != out: bad += 1
    nb += 1
while i < len(d) and len(pushes) < limit:
    c = d[i]; n = d[i+1]; t = struct.unpack_from('<I', d, i+2)[0]; fr = d[i+6:i+6+n]; i += 6 + n
    if c != dev: continue
    if t0 is None: t0 = t
    if (t - t0) / 10.0 >= poll:             # a poll: the answer closes the batch
        st['n'] = 0; m = call('close_b'); tot += st['n']
        if m: check(m)
        t0 = t
    mu.mem_write(syms['in_buf'], fr); st['n'] = 0; st['since'] = 0; st['gap'] = 0
    m = call('push', n); tot += st['n']; pushes.append((st['n'], st['gap'], fr[0]))
    if m: check(m)
    tspan = t
ins = sorted(p[0] for p in pushes); gaps = sorted(p[1] for p in pushes); pc = lambda v, q: v[int(q * (len(v) - 1))]
print(f"frames {len(pushes)}, batches {nb}, mismatches vs host {bad}")
if bad: print('FAILED: the emulated radio coded differently from the host build')
print(f"instructions per frame pushed: median {pc(ins,.5)}, p99 {pc(ins,.99)}, max {ins[-1]}; longest between yields: p99 {pc(gaps,.99)}, max {gaps[-1]}")
secs = (tspan) / 1e4
for cpi in (1.25, 2.0): print(f"  CPI {cpi}: frame median {pc(ins,.5)*cpi/64:.0f} us, max {ins[-1]*cpi/64:.0f} us; longest unyielded {gaps[-1]*cpi/64:.0f} us; CPU {tot*cpi/64/1e4/secs:.1f} % of the radio")

sys.exit(1 if bad else 0)
