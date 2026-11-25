#!/usr/bin/env bash
# Builds the doorman firmware from scratch (dongle: nRF52840 Dongle; radio: Index-family nRF52 and
# Vive Tracker 1.0 nRF51) and runs the radio emulator test. The packages take the images through
# scripts/stage-firmwares.py.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
dm="$repo/doorman"
command -v arm-none-eabi-gcc >/dev/null || { echo 'Install the ARM toolchain (arm-none-eabi-gcc, with newlib) first' >&2; exit 1; }
make -s -C "$dm/dongle-fw" clean
make -s -C "$dm/radio-fw" clean
make -s -C "$dm/dongle-fw"
make -s -C "$dm/radio-fw"
make -s -C "$dm/radio-fw" TARGET=idx
# The emulator drives each radio image's updater (SPI and over the air) before it can reach a device.
if command -v uv >/dev/null; then
  (cd "$dm/tools" && uv run -q --with unicorn python radio_emu.py | tail -1 && uv run -q --with unicorn python radio_emu.py --idx | tail -1)
else
  echo 'uv not found: skipping the radio emulator test (install uv to run it)' >&2
fi
