#!/usr/bin/env bash
# Builds the lhbs base station firmware from scratch and runs its host test. The packages
# take the image through scripts/stage-firmwares.py.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
fw="$repo/basestation_firmware"
command -v arm-none-eabi-gcc >/dev/null || { echo 'Install the ARM toolchain (arm-none-eabi-gcc) first' >&2; exit 1; }
command -v gcc >/dev/null || { echo 'Install gcc (for the host test) first' >&2; exit 1; }
make -s -C "$fw" clean
make -s -C "$fw" hosttest
