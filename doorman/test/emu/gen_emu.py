#!/usr/bin/env python3
"""The synthetic LH2 generator on an emulated Cortex-M4: frames vs the host build, instructions per 1 ms call,
longest run between DLG_YIELDs (as us at 64 MHz, CPI 1.25 and 2.0).
  gen_emu.py BUILD_DIR BASES DENSITY CALLS          (needs unicorn; the Makefile's gen-emu target builds it)"""
import sys, subprocess
from unicorn import *
from unicorn.arm_const import *
E, bases, density, calls = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
syms = {l.split()[2]: int(l.split()[0], 16) for l in subprocess.run(['arm-none-eabi-nm', f'{E}/gen_emu.elf'], capture_output=True, text=True).stdout.splitlines()}
code = open(f'{E}/gen_emu.bin', 'rb').read()
mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS); mu.mem_map(0, 0x80000); mu.mem_write(0, code); mu.mem_map(0x20000000, 0x10000)
STOP = 0x7FF00; st = {'n': 0, 'since': 0, 'gap': 0}
def oc(uc, a, s, u): st['n'] += 1; st['since'] += 1
def oy(uc, a, s, u): st['gap'] = max(st['gap'], st['since']); st['since'] = 0
mu.hook_add(UC_HOOK_CODE, oc, begin=0, end=len(code)); mu.hook_add(UC_HOOK_CODE, oy, begin=syms['yield_hook'], end=syms['yield_hook'])
def call(fn, *a):
    for i, v in enumerate(a): mu.reg_write(UC_ARM_REG_R0 + i, v)
    mu.reg_write(UC_ARM_REG_SP, 0x20010000 - 16); mu.reg_write(UC_ARM_REG_LR, STOP | 1); mu.emu_start(syms[fn] | 1, STOP, count=20_000_000)
    return mu.reg_read(UC_ARM_REG_R0)
subprocess.run([f'{E}/gen_ref', f'{E}/gen_ref.bin', str(bases), str(density), str(calls)], check=True)
ref = open(f'{E}/gen_ref.bin', 'rb').read(); ri = 0
call('init', bases, density)
per = []; gaps = []; bad = 0; total = 0
for ms in range(1, calls + 1):
    st['n'] = st['since'] = st['gap'] = 0
    n = call('run', ms * 48000)
    rl = ref[ri] | ref[ri + 1] << 8; r = ref[ri + 2:ri + 2 + rl]; ri += 2 + rl
    if n != rl or bytes(mu.mem_read(syms['out_buf'], n)) != r: bad += 1
    per.append(st['n']); gaps.append(max(st['gap'], st['since'])); total += st['n']
per.sort(); gaps.sort(); pc = lambda v, q: v[int(q * (len(v) - 1))]
print(f"{bases} bases, density {density}: {calls} ms, output mismatches vs host {bad}")
print(f"instructions per 1 ms call: median {pc(per,.5)}, p99 {pc(per,.99)}, max {per[-1]}; longest between yields: p99 {pc(gaps,.99)}, max {gaps[-1]}")
for cpi in (1.25, 2.0):
    print(f"  CPI {cpi}: call max {per[-1]*cpi/64:.0f} us, longest unyielded {gaps[-1]*cpi/64:.0f} us, CPU {total*cpi/64/(calls*1000)*100:.1f} %")
sys.exit(1 if bad else 0)
