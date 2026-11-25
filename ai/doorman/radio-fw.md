# doorman radio firmware (`doorman/radio-fw`)

One source file (`src/main.c`) for two targets:
* default: Vive Tracker 1.0, nRF51822 (`radio.ld`: app at 0x400, 0x1570 RAM). Image is a raw `.bin`.
* `TARGET=idx` (`-DDML_IDX`): Valve's nRF52832 radio module (Index controller family; `radio_idx.ld`:
  app at 0x1000). Builds `radio.bin` plus `radio_idx.fw` (the SteamVR `.fw` container, made by
  `tools/radio_image.py container`).
* `DIAG=1` (dev only, never released): noinit staging counters, a 96-entry SPI trace, air `'D'` dump and
  extended `'X'` replies (`tools/air_update.py info|trace`).

The device's main MCU keeps its stock firmware and talks to this radio over SPI with the stock frame
protocol (IMU `p`, light `q`, buttons, feature `P` tunnelling, pairing, link state). The radio carries
those frames over the doorman air link (`protocol.md`).

## Flash layout

| Region | Tracker 1.0 (nRF51) | Index family (nRF52) |
|---|---|---|
| stock bootloader | 0x0–0x3FF (HTC, 1 KB, never touched) | 0x0–0xFFF (Valve, never touched) |
| app (this image) | 0x400– | 0x1000– |
| image header | +0x10 length, +0x18 version | +0x1C length, +0x24 version |
| staging bank | 0x10000–0x1F7FF (`STAGE_MAX` 0xF800) | same; 0x1F800 is Valve's record page (never written) |

The running app is never erased by an update: images go to staging, get verified, and a commit marker is
written; the **stock bootloader** copies staging over the app at the next reset/power-on.

## The updater (stock protocol, kept exactly)

SPI frames from the main MCU (replies arrive with the next transfer):

| Frame | Effect | Reply |
|---|---|---|
| `Y` | open staging (offset 0) | `Y` 0 |
| `Z` + block | write the next block (48 B from Valve's main fw, ≤64 B from HTC's) | `Z` status |
| `[` + 16 B | check the 128-bit signature (`radio_image.radio_signature`, a 4-word LFSR over the padded image); on match write the commit marker | `[` 0 ok / 1 bad |
| `!` + 16 B | same check, other marker (stock's bootloader-reflash path) | `!` status |
| `0x20` | reboot into the bootloader when staging is closed and marked | |
| `\RESET` | reset | |

Replies **echo the command** (`Y 0`, `Z st`, `[ st`). Valve's main firmware counts 0x59/0x5a/0x5b/0x21 as
acks (all to one handler) and gives each step ~120 ms; HTC's LPC bootloader wants the echo. Valve's own
radio replies `Y 0` from a constant, so echo matches stock on both. An earlier 0x21-ack attempt broke USB
updates.

Over the air (downlink op `0xF0`, handled by the radio itself): `T` link timing (running totals: ADDRESS events,
seen >100 / >200 us late, answers skipped, codec frames and us, max lateness, max encode; the ADDRESS time is
captured by PPI channel 26 into TIMER0 CC[1]), `X` info, `Y` open, `Z [off u32][block]`,
`[ sig`, `R` radio reset, `D` trace (DIAG). Used by `tools/air_update.py`.

## Safety design (why it can't lock itself out)

* No interrupts at all (everything polled): nothing depends on how the stock bootloader forwards IRQs
  (an nRF51 RADIO interrupt reset HTC's chip into safe mode).
* The SPI slave and updater are serviced first in every loop iteration.
* The radio link starts 1 s after boot; **safe mode** (no link at all) after 3 quick watchdog/lockup
  resets in a row, or when the main MCU sends `O S`. A pin reset clears the count.
* Watchdog 8 s, fed right before every deliberate reset so the bootloader gets the full period.
* **Staging pre-clear** (Index): 10 s after boot, while idle, staging is erased one page per pass, and
  `Z` skips the erase when a page is already blank. Valve's main fw only waits ~120 ms per step; erasing
  inline was too slow in a busy loop. A committed but not yet installed image is kept (`staged_is_pending`).
* `tools/radio_emu.py` (unicorn) boots each image, runs the SPI and air updaters end to end with our image
  and the stock one, and fails on any flash write outside staging. `build-doorman.sh` runs it.

## Update routes (verified on hardware 2026-11-18/19)

| Route | Command | Verified |
|---|---|---|
| Index, USB (official) | `lighthouse_watchman_update -s <serial> --target=nrf52 radio_idx.fw` | stock→ours, ours→ours, ours→Valve 1562882729, back |
| Tracker 1.0, USB (official, undocumented) | `lighthouse_watchman_update -rnt radio.bin` (one tracker plugged in) | HTC 1466630404 ↔ ours both ways, stock LPC |
| Any device on our radio, air | `doorman/tools/air_update.py install SLOT IMAGE` | both families, ~3 s |
| Over the air through a Watchman dongle | `doorman/tools/index_radio_ota.py` | stock and ours (Valve dongles only with the owner's OK) |

USB details:
* **Index**: the tool always goes "via application": the main firmware stores the image, then at each
  boot hands it to the radio over SPI (`Y` / `Z` 48-byte blocks / `[`). `RadioArgs` in SteamVR's
  indexcontroller `manifest.vrfirmware` is the same command.
* **Tracker 1.0**: `-r` radio, `n`+`t` = device type 6 (app 28de:2022, LPC bootloader 28de:2021). The
  tool reboots the tracker into its stock LPC bootloader, which relays 0x97/0x98/0x99 to radio `Y`/`Z`/`[`.
  **Nothing is written to the LPC.** `-s` is ignored (it opens by VID/PID). `-rn` alone targets the wand
  (0x2012), `-rv` 0x2010: wrong for a tracker.
* After an update, devices are powered off (`P 9f 04 off!`); the user turns them on and the bootloader
  installs. Restore = same command with SteamVR's stock radio file.

## Lessons

* nRF52832 page erase is ~3 ms in practice, not the 90 ms worst case: the pre-clear is for the busy loop.
* Diagnostics must not change behaviour: an op-history array broke the emulator, clearing RESETREAS in the
  diag boot hook broke the quick-boot counter. Keep `DIAG` additions passive.
* A failed USB update with our radio as the receiver once looked like "the USB route doesn't work"; it was
  our ack bytes. Always test the stock radio as receiver too.

### Answer timing (no interrupts)
The answer buffer must be swapped into PACKETPTR between a POLL's ADDRESS and the auto-TX start (~180 us
later). It's polled (`addr_check`), so nothing may block longer than that: the codec calls `DMC_YIELD()`
(= `addr_check`) before every coded field. Before that, a coded frame took ~400 us (max 1.15 ms) on the
Tracker 1.0's nRF51 and its loss was 16-25 % (83 % alone on a 2.15 ms cycle); after, 1.5-9 % and the
worst lateness is ~70 us. The bit writer also writes a byte at a time (output unchanged, checked bit for bit).

### Test load ('L', 0x6A000020 / 0x0A000011)
Air op `'L' [kB/s]` (reply `[F0]['L'][kB/s]`): the radio sends filler `'Z'` frames up at that rate, as if
the main MCU sent more, so a few devices load the radio cycle like many. 0 stops it; not kept across a
reboot. `doorman/tools/load.py radio SLOT KBPS`.

The budget is kept in thousandths of a byte (`kB/s x us`), so `L` is the rate queued at any main-loop
speed. Before this, each pass rounded its share down to whole bytes and lost the rest, which on a fast
loop is nearly all of it: a Tracker 3.0 sent ~0.4 kB/s of filler for `L` 14 and ~2.4 for `L` 54.

