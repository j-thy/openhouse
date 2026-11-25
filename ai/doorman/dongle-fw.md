# doorman dongle firmware (`doorman/dongle-fw`)

nRF52840 Dongle (PCA10059 or compatible), tinyusb, polled main loop (no radio interrupts). Sources:
`src/main.c` (USB commands, crash record, bootloader entry), `src/link.c` (the air link: slots,
joining/pairing, hopping, go-back-N, codec), `src/usb_descriptors.c`, `src/dongle.h` (`DONGLE_FW_VERSION`,
VID/PID 6767:0727, command numbers).

## Two builds (same code, different flash base)

| UF2 | For | App base | RAM |
|---|---|---|---|
| `doorman_dongle_sd.uf2` | dongles whose UF2 bootloader sits on Nordic's MBR/SoftDevice (the usual Adafruit-style bootloader) | 0x26000 (`ld/app_sd.ld`) | from 0x20003400 (SoftDevice RAM untouched) |
| `doorman_dongle_nosd.uf2` | UF2 bootloader without SoftDevice | 0x1000 (`ld/app_nosd.ld`) | from 0x20000000 |

Info (cmd 0x01) reports which variant is running, whether the SoftDevice magic is still in flash and
the board (byte 57: 0 UF2, 1 Holyiot 21017, 2 Nordic nRF52840 Dongle PCA10059);
doorman-dongle-manager picks the file name from that. Flash end 0xED000; the **paired table** is page 0xEC000
(magic "OHPR", never erased if the code grows into it). RAM from 0x2003E000 holds the crash record.

## Install / update

* `CMD_DFU_UF2` (0x10) sets GPREGRET and resets into the UF2 bootloader; the drive appears, the UF2 is
  copied, the bootloader starts the app. `tools/dongle.py flash sd|nosd|FILE` and doorman-dongle-manager's
  Firmware page do exactly this. By hand: double-tap reset.
* Holyiot 21017 (`doorman_dongle_holyiot.hex`, Nordic's open USB DFU bootloader): 0x10 sets GPREGRET 0xB1
  instead and the bootloader's serial port (1915:521F) appears; openhouse-installer and doorman-dongle-manager
  flash it with Nordic's serial DFU (`src/common/nordic_dfu.cpp`). Pairings live at 0xDA000 there (the
  bootloader owns 0xE0000+), the SKY66112 amplifier is switched by CTX P0.24 / CRX P0.22, TX power -4 dBm.
* PCA10059 (`doorman_dongle_pca10059.hex`): the same layout, bootloader, LED (P0.06) and update path as the
  Holyiot, no amplifier (TX +8 dBm). It runs off USB (VDDH): on first boot it sets UICR REGOUT0 to 3.0 V
  once (an unset chip gives 1.8 V GPIO, too low for the LEDs), like Nordic's board code. The bootloader
  looks the same on USB for both boards (1915:521F): the installer flashes the image of the board picked.
* 0x11 enters serial DFU instead. A brand-new PCA10059 with Nordic's stock bootloader needs a UF2 bootloader
  first (not automated).

## Robustness

* Watchdog; crash record (HardFault/MemManage/BusFault/UsageFault/WDT with pc, lr, CFSR, HFSR, count)
  survives resets and is read with cmd 0x01, cleared with 0x14; 0x13 forces one (test).
* Quick-crash counter: repeated crashes right after boot are counted (cmd 0x01 byte 52).

## Timing and capacity

See `protocol.md` for the air schedule. Per-slot overhead ~450 µs (turnaround, headers, guard), so bigger
responses win: 6 devices × 120 B = 6.3 ms cycle, ~18 kB/s each against ~8 kB/s needed on air. USB is the
other limit: one 64 B report per ms per HID interface, hence two interfaces (even/odd slots). Realistic
ceiling ≈ 8 devices per dongle (needs `DML_SLOTS` and the USB split changed); 16–20 only with worse latency.

## USB name, control path, hang detector (0x0019-0x001B)

* Product string names the board: "Doorman Dongle Pro Micro" (sd/nosd UF2), "Doorman Dongle Holyiot",
  "Doorman Dongle PCA10059". Host code (driver, dongle manager, `dongle.py`) matches the
  "Doorman Dongle" prefix.
* Commands also work over the control endpoint: a feature SET_REPORT carries the command (as the OUT
  report does), a feature GET_REPORT on interface 0 returns the last reply. Kept on purpose: a dongle
  whose interrupt endpoints don't get through (seen behind one USB hub: control worked, interrupt
  transfers were never scheduled) can still be read and sent to DFU.
* 0x7E: USB controller state (usb_rx/tx, EPSTATUS, EPDATASTATUS, EPIN/OUTEN, USBADDR, SIZE.EPOUT1-4,
  mounted, safe mode, FICR part/variant, HFCLKSTAT).
* Hang detector: SysTick (highest priority) checks a heartbeat stamped by the main loop and
  usb_service(); stale for 2 s, it records the stuck pc/lr as crash type 6 ("hang") and resets. The boot
  after a hang runs in safe mode (no radio, USB only; cmd 0x01 byte 58) until a power cycle.

## Kuroshin dongle (nRF52840, Nordic SDK open USB bootloader)

* Runs the PCA10059 image. Pads: shorting pad 1 and pad 5 (RESET P0.18 and GND, the 5-pad row on the
  chip side) enters DFU (1915:521F); then flash like a PCA10059.
* Its bootloader only enters DFU on a pin reset or an invalid app: GPREGRET entry is compiled out, and
  before starting the app it ACL-locks the MBR, the app image, itself and its settings. So DFU can't be
  requested from software: the app can't invalidate itself, and driving P0.18 as a GPIO bus-faults.
  Every update needs the pads.
* Behind one USB hub its interrupt endpoints never got through (control worked); on another port it
  works. After flashing, the host sometimes doesn't see it again until it's replugged.

## Two-bank layout and self-update (0x0020, every board)

* Flash: a boot stub at the app start (0x1000; 0x26000 for the Pro Micro "sd" build), with bank A's image
  as payload; then bank A and bank B, each a 4 KB header page + image (`src/banks.h`):
  "std" A 0x10000 / B 0x70000 (0x60000 each), "sd" A 0x30000 / B 0x8E000 (0x5E000 each, to the pairing
  page). Header: magic "OHBK", sequence, size, CRC32, origin (CRC of the stub payload the chain started from).
* The stub boots the valid bank with the highest sequence. With no valid bank, or when the newest bank's
  origin isn't its payload (a bootloader just installed a new one), it installs its payload into bank A
  first (sequence + 1). So a bootloader flash always takes effect (rescue, downgrade).
* Self-update (no bootloader): 0x40 [size][crc] erases the other bank (header page first), 0x41
  [offset][n][bytes] writes into it, 0x42 checks the CRC, writes the header (sequence + 1, origin copied)
  and restarts. A failed or interrupted update leaves the running bank as it was. `dongle.py bank-update`,
  the dongle manager's Update button and the installer do this; the bootloader is only for a first
  install or a rescue.
* Build outputs: `doorman_dongle_<board>.hex/.uf2` (stub + payload, for bootloaders) and
  `doorman_dongle_<board>_a.bin / _b.bin` (bank images); packages: `firmwares/dongle/bank/<board>-a.bin`.
  Boards: sd, nosd, holyiot, pca10059, kuroshin. cmd 0x01 byte 59: the running bank (1 A, 2 B).
* Verified 2026-11-21: Pro Micro (sd) converted through UF2, two bank updates both ways, a bootloader
  flash of a different build won over the banks and a second one brought the first back; Kuroshin bank
  updates both ways; an update interrupted after the erase and half the writes left the old bank running.
