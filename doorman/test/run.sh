#!/bin/sh
# Everything on one capture: dump the frames its devices sent (a replay through lhtool), round-trip them
# through the codec at the answer sizes and poll periods of 4, 6 and 8 devices, time the radio's batch path
# on the emulated Cortex-M4 for one device, and run the cycle simulation.
#   doorman/test/run.sh CAPTURE.lhraw [LHTOOL] [PYTHON]
# LHTOOL defaults to build-linux/lhtool, PYTHON to python3 (it needs unicorn for the timing; skipped without).
set -e
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
capture=${1:?usage: run.sh CAPTURE.lhraw [LHTOOL] [PYTHON]}
lhtool=${2:-$root/build-linux/lhtool}
python=${3:-python3}
make -s -C "$here" all
out="$here/build"
echo "== frames from $capture"
LH_AIR=base LH_AIR_DUMP="$out/frames.bin" "$lhtool" session "$capture" > /dev/null 2>&1
echo "== codec round trip"
"$out/codec_roundtrip" "$out/frames.bin" 4 96
"$out/codec_roundtrip" "$out/frames.bin" 6 120
"$out/codec_roundtrip" "$out/frames.bin" 10 176
if command -v arm-none-eabi-gcc > /dev/null && "$python" -c 'import unicorn' 2> /dev/null; then
  echo "== the radio's batch path on an emulated Cortex-M4 (device 0, 8-device answers)"
  make -s -C "$here" emu
  "$out/batch_ref" "$out/frames.bin" "$out/batch_ref.bin" 0 3000 176 10
  "$python" "$here/emu/run_emu.py" "$out" "$out/frames.bin" 0 3000 176 10
  echo "== the synthetic LH2 generator on the emulated Cortex-M4 (4 bases, density 2 and 8)"
  "$python" "$here/emu/gen_emu.py" "$out" 4 2 3000
  "$python" "$here/emu/gen_emu.py" "$out" 4 8 3000
else
  echo "== chip timing skipped (needs arm-none-eabi-gcc and Python's unicorn)"
fi
echo "== the synthetic LH2 stream through the codec (5 devices, 4 bases, density 2)"
"$out/lh2gen_check" "$out/gen.bin" 5 30 4 2 8
"$out/codec_roundtrip" "$out/gen.bin" 10 176
echo "== cycle simulation"
"$out/cycle_sim"
