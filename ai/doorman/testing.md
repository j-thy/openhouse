# Testing the Doorman codec and link without hardware

`doorman/test/` has what was used to design and check the air codec, the batches and the answer sizing.
Everything runs on recordings; nothing needs a dongle or a device.

## Frames from a capture
A replay with `LH_AIR` set routes each device's IMU and LH2 data through
`src/runtime/air_experiment.hpp` before the session sees it. `LH_AIR_DUMP=FILE` writes the frames a
Doorman radio would carry (`[device][length][time, 0.1 ms][frame]`); `LH_AIR_REPORT=FILE` writes bytes
on the air per device for the experimental formats. With `LH_AIR` unset nothing changes.

    LH_AIR=base LH_AIR_DUMP=frames.bin build-linux/lhtool session CAPTURE.lhraw

The `'s'` frames are rebuilt from Watchman or wired captures (up to 59 bytes or 1 ms per frame), so
absolute byte counts are a model; comparisons between formats use the same frames.

## The tools
* `codec_roundtrip FRAMES [POLL_MS [ANSWER_BYTES]]`: every frame through today's codec (the size
  reference) and through batches as the radio builds them, decoded as the dongle does, compared byte
  for byte. Also checks the table CRC-8 against the bitwise one. ctest `doorman_codec` runs it on
  `tests/data/captures/lh2-five-devices` at 8-device settings.
* `emu/`: the radio's batch path built for the Index-family radio's Cortex-M4 with the radio's flags,
  run in Unicorn on real frames (`run_emu.py`), checked against the host build (`batch_ref`). It reports
  instructions per frame and the longest run between two `DMC_YIELD`s, as microseconds at 64 MHz for CPI
  1.25 and 2.0 (Unicorn counts instructions, not cycles). Between two yields nothing may run past the
  answer window, ~180 us. Needs `arm-none-eabi-gcc` and `pip install unicorn`.
* `cycle_sim`: a model of the polling cycle under load, fixed answer sizes against `sizing.h`.
* `run.sh CAPTURE [LHTOOL] [PYTHON]` does all of it on one capture.

## Synthetic LH2 load on real devices (no LH2 hardware)
`doorman_lh2gen.h` makes the frames an LH2 tracker's main MCU would send with up to 4 Base Station 2.0s,
on the radio itself. Density 2 matches the two captures (hits, beam words, frame sizes, and the codec's
-31..-33 %); higher densities load the link past them (8 is ~3x). `lh2gen_check` dumps it for
`codec_roundtrip`; `emu/gen_emu.py` checks the radio build against the host one and its timing (at most
125 us between yields at density 8, CPU <= 2.5 %).
* Index-family radios (0x6A000025+): air op `'G' [bases][density]` adds it to the device's own stream
  (`load.py gen SLOT BASES DENSITY`; 0 bases stops it).
* Pro Micro (nRF52840) test devices: `make TARGET=fake` in `radio-fw` -> `radio_fake_sd.uf2` /
  `radio_fake_nosd.uf2` (by the board's UF2 bootloader). Pairing mode for 35 s after power-up (LED on),
  blinking while connected, generating 4 bases at density 2 from the start.
* Radios without the LH2 coder (Tracker 1.0) stand in with filler at the same on-air rate (`L`).
* `load.py sweep GEN FILLER [DENSITIES] [SECS]` steps the density and reports per step: decoded and on-air
  kB/s per device, total on air, loss, drops (dongle, device), saturation, answer size (L = large), cycle;
  it stops once data is dropped. Without the driver: it would take the made-up IMU for the devices' own.

## What it showed (two LH2 captures, 4 bases)
* Batches with LH2 prediction: all traffic -31 to -35 % at 8 devices (176 B answers, ~10 ms cycle),
  every frame rebuilt exactly. A tracker ~8.3 -> ~5.5 kB/s.
* Radio cost: 30-57 us per frame typically, 1-2 % of the radio's CPU, at most 56 us between yields
  (after fixing a sort, the CRC and a slot clear that ran 120-200 us without yielding).
* Large answers while saturated: drops at 16 kB/s per device 10 -> 3 %, none of the cost below ~13 kB/s.
* The radio's other paths (updater, boot) are covered by `doorman/tools/radio_emu.py`. The radio
  exchange itself (POLL/RESP timing, the full main loop) isn't emulated: hardware tests cover it.
