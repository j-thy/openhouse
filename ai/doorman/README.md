# doorman: openhouse's own wireless link

**doorman** is openhouse's replacement for Valve's Watchman wireless (lighthouse → watchman,
openhouse → doorman). It has three parts:

* the **dongle** (`doorman/dongle-fw`): an nRF52840 Dongle (Nordic PCA10059 or a compatible
  board) with our firmware. It serves up to **6 devices**, and the host sees each one as a
  wired device.
* the **device radio** (`doorman/radio-fw`): our firmware for the radio chip inside the device.
  It replaces the stock radio app and keeps the stock updater protocol, so the stock radio
  can always be put back.
  * Vive Tracker 1.0: nRF51822, replacing HTC's app.
  * Index controller family: nRF52832, replacing Valve's app. Only the Index controller is
    tested; the Vive Tracker 2018 / 3.0, Tundra and Vive Pro controllers share the layout.
* the **host side**, in this repo's C++:
  * `src/transport/usb/doorman_dongle.cpp`
  * `UsbTransport` receivers and pairing
  * the telemetry server
  * lhviz's Receivers panel
  * `apps/doorman-dongle-manager`, the dongle manager

The device's main MCU keeps its **stock** firmware. It talks to the radio over SPI exactly
as it would to the stock radio app. So users only change the radio, and only over USB with
SteamVR's own updater (`radio-fw.md`).

Code names that predate the "doorman" name still say "doorman_link" / "doorman_dongle" / "openhouse
link": `common/doorman_link.h`, `DML_*`, `doorman_dongle.cpp`, `OHDG-` receiver serials, and the UF2
file names. They mean doorman. The owner may rename user-facing strings later (USB product
string "Doorman Dongle", lhviz badge "OH"). Ask before renaming identifiers.

## Layout

| Path | What |
|---|---|
| `doorman/common/doorman_link.h` | air protocol (packets, timing constants, versions `DML_VERSION`); shared by both firmwares |
| `doorman/common/doorman_codec.h` | lossless air codec for IMU/light frames (both ends) |
| `doorman/dongle-fw/` | dongle firmware: `src/` (main, link, USB descriptors, tinyusb config), `ld/` (two linker scripts), `tools/uf2conv.py` |
| `doorman/radio-fw/` | radio firmware: `src/main.c` (whole app), `src/startup.c`, `radio.ld` (nRF51), `radio_idx.ld` (nRF52) |
| `doorman/third_party/` | **only the files the builds use** from tinyusb 0.17.0, nrfx v2.11.0 and CMSIS_5 5.9.0, plus their licenses. Commits are in `VERSIONS.txt`. To update: clone those tags, build with `-M`, copy the listed files |
| `doorman/tools/` | Python host tools (below) |
| `scripts/build-doorman.sh` | clean build of both firmwares and the radio emulator test |
| `scripts/stage-firmwares.py` | puts the dongle UF2s and both radio images (with lhbs) in a package's `firmwares/`, for openhouse-installer |

Tools (`doorman/tools`; Python 3, `hid` = hidapi bindings, `usb` = pyusb):
* `dongle.py`: dongle CLI.
  * `info`, `slots`, `pair SECS`, `forget`, `channels`, `scan`, `listen`, `signal`, `slotstats`
  * `flash sd|nosd|FILE` (puts the dongle in its UF2 bootloader and copies the image)
  * Only ever touches 6767:0727 and UF2 drives; never Valve dongles.
* `doorman_link.py`: the Python library: `Dongle().frames()`, `slots()`, `pair()`, `send()`.
* `air_update.py`: radio update over the doorman link (any image, ~3 s).
  * `install SLOT IMAGE`
  * `info SLOT`, `trace SLOT` (the last two need a `DIAG=1` radio)
  * Powers the device off afterwards; the user turns it on to install.
* `index_radio_ota.py`: radio update over the air through a Valve/HTC Watchman dongle (feature
  0xff, opcodes 97/98/99). Works on stock and our radios, both layouts. Needs the owner's OK
  before touching a Valve dongle.
* `radio_image.py`:
  * `fix [--idx]` (writes the length into the header)
  * `container IN OUT` (SteamVR `.fw` wrapper)
  * `check`
  * also provides `radio_signature()`
* `radio_emu.py`: unicorn emulation of the radio firmware. It boots the image, drives the SPI
  slave like the main MCU and the over-the-air updater like the dongle, and checks that every
  flash write stays inside staging. Run it before any radio image reaches a device:
  `uv run --with unicorn python radio_emu.py [--idx] [IMAGE]`. It also stages SteamVR's stock
  radio when SteamVR is installed.

## Build, test, package

```sh
make -C doorman/dongle-fw                 # build/doorman_dongle_sd.uf2, build/doorman_dongle_nosd.uf2
make -C doorman/radio-fw                  # build/radio.bin           (Tracker 1.0)
make -C doorman/radio-fw TARGET=idx       # build-idx/radio.bin + radio_idx.fw (Index family)
make -C doorman/radio-fw TARGET=idx DIAG=1  # dev only: staging counters + SPI trace
scripts/build-doorman.sh                  # all of it + emulator
```

Needs `arm-none-eabi-gcc` with newlib (the dongle links newlib-nano; its `_close`/`_read`
"not implemented" linker warnings are expected). The emulator needs `uv`.
`build-doorman.sh` skips the emulator test with a warning when `uv` is missing.

**Versions** (bump on every image that goes to a device; a device refuses nothing, but
SteamVR and our tools compare them):
* dongle: `DONGLE_FW_VERSION` in `dongle-fw/src/dongle.h` (0x0004)
* radio: `DML_VERSION` in `common/doorman_link.h`, one per target: Index `0x6A0000xx` (kept above
  Valve's 0x5D27B2A9 so nothing "updates" it back), Tracker 1.0 `0x0A0000xx`. Released:
  Index 0x6A000012, Tracker 1.0 0x0A000008.

## Documents

* `protocol.md`: air link, joining/pairing, codec, the dongle's USB interface and commands,
  and the device frames the host uses.
* `dongle-fw.md`: dongle firmware internals (flash layout, bootloaders, timing, crash record).
* `radio-fw.md`: radio firmware internals, the stock updater protocol, every update route
  (USB, air) with what was verified, and the hard-won lessons.
* `host.md`: the openhouse side (transport, receivers, SteamVR pairing, telemetry, tools).

## Rules that keep devices recoverable

* A radio image must keep a **working updater**. It's the only way back without opening the
  device (the owner won't open controllers, and no SWD is used). The emulator checks it. The
  firmware services the SPI updater first in every loop, uses no interrupts, enters safe mode
  after 3 quick watchdog/lockup resets, and pre-clears staging (`radio-fw.md`).
* Test restoring the stock radio before relying on a new radio build.
* Tracker 1.0 main MCU (LPC11U37): radio updates go through its stock Valve bootloader and
  write nothing to the LPC. Never write LPC flash 0x0–0x2FFF (bootloader) or 0x18000 and up,
  never word 0x2FC (CRP3), and never EEPROM at 0x400 or above.
* Never write to Valve dongles (28de:2101/2102) without the owner's OK. HTC Watchman dongles
  were OK'd for over-the-air updates and pairing.
* The link stays at 2 Mbit.

## State (2026-11-19)

Working, verified on hardware:
* 6 devices on one dongle (3 Tracker 1.0 on stock LPC firmware + our radio, 3 Index
  controllers), all tracking in openhouse at 250 Hz IMU with no drops. That needed the
  second HID interface (`protocol.md`).
* Every radio update route (`radio-fw.md` has the table). USB with SteamVR's tool works in
  both directions for both families. That's the primary install method for users.
* The dongle manager app, the lhviz Receivers panel, and per-slot telemetry.

Not tested yet:
* pairing into a chosen slot with a real device;
* pairing from the SteamVR pairing UI;
* the dongle manager with live devices and with the driver running;
* the Windows build of `doorman-dongle-manager`.

Open:
* A Linux udev rule for 6767:0727 (hidraw access).
* The openhouse installer should cover dongle flashing and device radio firmware, next to
  driver install and base station firmware (planned by the owner).
* Capacity: realistic ≈ 8 devices per dongle; 16–20 is theoretically possible with worse
  latency. USB is the bottleneck: one 64-byte report per ms per interface (`protocol.md`).
* `doorman/radio-fw` builds for Valve's other nRF52 devices are untested.
