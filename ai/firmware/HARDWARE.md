# Lighthouse 1.0 base station — hardware & firmware reverse engineering

Source image: `lighthouse_tx_htc_2_0-436-2016-09-20.bin`
(80 888 bytes, sha256 `cd396a69…78b7b8`, build "2016-09-20 19:00:20", host tag `steamservices`).

Confidence tags: **[C]** confirmed from code + data, **[I]** inferred (strong evidence, not
yet measured on hardware), **[?]** open question.

All addresses are flash offsets (image base 0). Function names are mine unless they come from
the firmware's own `assert()`/`__func__` strings, which are noted.

---

## 1. MCU

| Item | Value |
|---|---|
| Part | NXP **LPC11U3x**, Cortex-M0, ARMv6-M Thumb **[C]**. **LPC11U37** — ISP drive exposes 131 072 bytes = 128 KB flash **[C]** |
| Clock | 12 MHz crystal → SYSPLL `SYSPLLCTRL=0x23` (M=4,P=2) → **48 MHz** core; USB PLL also brought up at 48 MHz (`SystemInit` @0x1F1C) **[C]** |
| Toolchain | Keil/ARMCC (`__main` scatter-load @0xC0, RW-init table at image end) **[C]** |
| Source tree | `..\..\common\mcu\LPC11Uxx\{gpio,timer,iap,iocon,i2c}.c`, `..\..\common\peripheral\{isl58303,lis2dh}.c`, `..\..\common\utils\{cparam,ticker}.c`, `main.c laser.c timebase.c parameter.c` |
| Stack | initial SP 0x10001470; RAM 0x10001470–0x10001FDF is unused by stock **[C]** |
| CRP | word @0x2FC = 0xFFFFFFFF → **no code read protection** **[C]** |
| Vector checksum | word @0x1C = 0xEFFFCECB, sum(words 0..7)=0 **[C]** |

### Interrupt vector table (LPC11U numbering — the plan's IRQ numbers were wrong)

| Vec | IRQ | Source | Handler | Purpose |
|---|---|---|---|---|
| 0x04 | – | Reset | 0x00DC | → SystemInit, `__main` → `main` @0xA52C |
| 0x0C | – | HardFault | 0x1AF8 | register dump + "HardFault" |
| 0x3C | – | SysTick | 0x1D30 | ticker (`ticker.c`) |
| 0x40–0x5C | 0–7 | PIN_INT0..7 | 0x1AA8.. | thunks → `gpio_flex_isr(n)` @0x86FC |
| 0x7C | 15 | I2C | 0x1B5C | I2C master state machine (LTC6904, LIS2DH) |
| 0x80 | 16 | CT16B0 | 0x1FC8 | → `timer_isr(0)` @0xD170 |
| 0x84 | 17 | CT16B1 | 0x1FD8 | → `timer_isr(1)` |
| 0x88 | 18 | CT32B0 | 0x1FE8 | → `timer_isr(2)` |
| 0x8C | 19 | CT32B1 | 0x1FF8 | → `timer_isr(3)` |
| 0x94 | 21 | USART | 0x2008 | empty (UART is polled) |
| 0xA0 | 24 | ADC | 0x1A64 | per-channel callback table @0x10000318 (laser power sense) |

USB IRQ is unused: **the stock firmware has no USB stack**. All USB activity is the NXP boot
ROM's mass-storage ISP.

---

## 2. Pin map

GPIO pins are encoded in the firmware as `(port<<5)|bit`, IOCON function in bits 8–10
(e.g. `0x123D` = P1_29 function 2).

| Pin | Dir | Function | Evidence |
|---|---|---|---|
| P0_1 | in, pull-up | **Mode button**, active low, both-edge PIN_INT → `button_event` @0x3430. Also the **ROM ISP strap** | `board_init` @0xD8F8 **[C]** |
| P0_3 | in | USB_VBUS (func 1) | `main` **[C]** |
| P0_12 | in | CT32B1_CAP0 (func 3) — **3.5 mm sync cable in** | `timebase_select` @0xCBA8 **[C]**; cable confirmed on hardware (log 2026-11-08) |
| P1_4 | in | CT32B1_CAP0 (func 1) — optical sync receiver | same; selected when `P1_15` set **[C]** |
| P1_15 | out | timebase input select (set ↔ P1_4 optical, clear ↔ P0_12 cable) | **[C]** code and hardware |
| P0_14 | out | CT32B1_MAT1 — set at each timebase edge (MR1), cleared at MR3 → **sync out** (3.5 mm cable) | `timebase_init` @0xCAAC; an lhbs master's pulses received by an lhbs slave (log 2026-11-08) |
| P1_0 | out | Stock, at every MR1 (`timebase_mr1_isr` @0xCD36): set as **master**, cleared as **slave-cable**, as **slave-optical** the permits-emission flag (0x1000066D). Gates a master's flash (log 2026-10-18); doesn't gate a slave's (MULTISTATION log 2026-10-19). Raised on a slave with the sync cable in, the slave's cable input reads stuck high from an OOTX frame boundary until P1_0 goes low; held high, it stays stuck (log 2026-11-08). Not the slave's own MAT1 (same with sync out undriven) | **[C]** code; effect measured (log 2026-11-08); a latch in the jack circuit, armed by P1_0, **[I]** |
| P0_18 | out | CT32B0_MAT0 — set on MR0 = rotor-index capture + OOTX pulse width → **sync pulse end** | `rotor_init` @0xB988, `rotor_index_isr` @0xB720 **[I]** |
| P0_20 | out | carrier oscillator enable | `carrier_init` @0x36B8 **[C]** |
| P1_28 | in, hyst | CT32B0_CAP0 — **rotor 0 index** sensor | **[C]** |
| P1_29 | in, hyst | CT32B0_CAP1 — **rotor 1 index** sensor | **[C]** |
| P0_21 / P0_22 | out | CT16B1_MAT0/1 — rotor 0/1 **coarse PWM** (12-bit, period 0xFFF) | **[C]** |
| P1_13 / P1_14 | out | CT16B0_MAT0/1 — rotor 0/1 **fine/PLL PWM** (12-bit) | **[C]** code, coarse/fine split **[I]** |
| P1_12 | out | **laser 0 enable**, active high | `laser_set_enable` @0x9D28 (name from `__func__`) **[C]** |
| P0_16 | out | **laser 1 enable**, active high | same **[C]** |
| P1_22 | out | bit-bang data: display shift register + ISL58303 SDIO | **[C]** |
| P1_20 | out | bit-bang clock: display + ISL58303 | **[C]** |
| P1_1 | out | ISL58303 chip select | `isl58303_init` @0x9438 **[C]** |
| P1_24 | out | display latch | **[C]** |
| P1_23 | out | display output enable, held low | **[C]** |
| P1_16/17/18 | OD | LED0 R/G/B, active low | table @0x10430 **[C]** |
| P1_9/19/11 | OD | LED1 R/G/B, active low | **[C]** |
| P1_7 / P1_6 | out | rotor 0 / rotor 1 locked indicator (active high) | `ui_update` @0xC3C8 **[C]** |
| P1_8 | out | timebase-locked indicator | **[C]** |
| P1_10 | out | status indicator (active low) | **[C]** |
| P0_17 | out | set as output in `main`, purpose **[?]** | |
| P0_19 | in | input, no pull, purpose **[?]** | |
| P0_23 | out | cleared only when emission enabled and not stand-by → power/emitter enable (active low?) | `standby_outputs` @0x3220 **[I]** |
| P1_5 | out | follows OOTX config flag @0x100005FF **[?]** | |
| P1_26 / P1_27 | – | **UART RXD / TXD** (func 2), 115 200 8N1 console `lhtx>` | `uart_init` @0xDBF8 **[C]** |
| P1_12/P1_13… | | see above | |
| P0_4 / P0_5 | – | I2C SCL/SDA (only I2C pins on LPC11U) | I2C @0x40000000 **[I]** |
| P0_10/P0_15 | – | SWCLK/SWDIO, left in debug function | SWD header **[?]** |

---

## 3. Peripherals & external chips

| Block | Use |
|---|---|
| **CT32B1** (timer 3) | **Timebase.** Free-running 48 MHz. CAP0 = external/optical sync (`timebase_capture_isr` @0xC8B0). MR1 = next sync slot (`timebase_mr1_isr` @0xCC8C, period/2 cadence, sets MAT1 + P1_0). MR3 = sync-lost / half-slot (@0xCA9C clears MAT1) |
| **CT32B0** (timer 2) | **Rotor timing.** CAP0/CAP1 = rotor index → `rotor_index_isr(0/1)` @0xB720. MR0 → MAT0 (P0_18) = OOTX sync pulse end (@0xC5FC). MR1/MR2 = per-rotor linked list of laser on/off events (@0xB6BC / @0xB6F4) |
| **CT16B0 / CT16B1** | 12-bit PWM for the two motors (`rotor_set_pwm` @0xB94C, `rotor_control` @0xBCFC) |
| **ADC** | Laser power-sense feedback for APC (`unexpected ADC conversion source!`) |
| **USART** | Debug console, polled |
| **I2C** | LTC6904 + LIS2DH |
| **LTC6904** @ I2C 0x16/0x17 | Programmable **carrier oscillator** for the IR sync LEDs. `carrier_set(hz)` @0xA48C computes `f = 2^OCT·2078 Hz/(2−DAC/1024)` (constant 2078.0f = 0x4501E000), CNF=2 **[C]** |
| **ISL58303** | Intersil dual laser-diode driver on bit-banged 3-wire SPI (P1_22/P1_20/P1_1). Chip ID 0xE0, VSO check. Per-channel registers `ch*0x10 + {0x12,0x13,0x16,0x17}` = bias/drive currents (`isl58303_reg_set` @0x9810) **[C]** |
| **LIS2DH** | Accelerometer (I2C). Orientation → flips the 7-segment glyph when mounted upside down; gravity vector goes out in OOTX **[C]** |
| **74HC595-type** shift register | Channel display, 1 digit. MSB first, bit0..6 = a..g, bit7 = dp. Font: digits @0x10768, letters @0x10778 **[C]** |
| **EEPROM** (4 KB) | IAP 62 read (@0x8EF8), 61 write (@0x8F3C), 61/62 move (@0x8DD4). Parameter partitions with CRC (`cparam.c`), factory config partition, HW_ID string @EEPROM 0xE08 (parsed as hex; `>>8 < 3` selects an older board revision) **[C]** |

---

## 4. How the station works (control flow)

```
main @0xA52C
 ├─ clocks, gpio, i2c, uart(115200), board_init(button, LEDs, display)
 ├─ parameter_init: load EEPROM partitions → RAM config (fall back to factory/defaults)
 ├─ select mode (= channel) from mode table @0x100006F4 → period, timebase mode, carrier, ootx set
 ├─ carrier_init (LTC6904), accel init (LIS2DH), laser_init (ISL58303 + ADC APC)
 ├─ rotor_init (CT16B0/1 PWM + CT32B0 capture), timebase_init (CT32B1)
 ├─ ootx_build_frame @0x3268 (serial, fw ver, fcal.*, accel, mode) → bit buffer @0x10000624
 └─ loop: console poll, UI/LEDs, stand-by logic; exits to iap_reinvoke_isp when state==5
```

### Per revolution (60 Hz nominal, `mode.N.period` ticks)
1. Motor PLL (`rotor_control` @0xBCFC, `pll_update` @0x7E98) keeps each rotor's index edge
   phase-locked to the CT32B1 timebase (`rotor.N.pll.{p,i,d,offset}`, coarse PWM model
   `rotor.N.pwm.{m,b}`), reports "Rotor %d locked/unlocked".
2. **Rotor index capture** (`rotor_index_isr` @0xB720):
   * takes the next OOTX bit (`ootx_get_bit` @0xAA7A, MSB-first, length @+0x40),
   * looks up the sync **pulse width** `ootx_width(table, rotor, bit, parity)` @0xAAA0 =
     `table[(parity<<2)|(rotor<<1)|bit]`, table @0x10000600,
   * programs CT32B0 MR0 = capture + width → P0_18 marks the end of the sync flash,
   * computes laser on/off match times from the measured period and the per-rotor event list
     (`laser.N.start`, `laser.N.length` in degrees) and arms CT32B0 MR1/MR2.
3. **Timebase MR1** (`timebase_mr1_isr` @0xCC8C) fires every half slot and re-arms itself.
   The match raises P0_14 (sync out); MR3 is set `period/2` later to end that pulse
   (`± 100 000` ticks by the skip flag in master-tdm, opposite sign in slave-optical).
   It sets P1_0 as master, clears it as slave-cable and copies permits-emission to it as
   slave-optical, and feeds the rotor PLL references.

The sync flash therefore starts on the rotor index edge and ends at MR0. That gives
sub-tick alignment between optical sweep and sync **[I — hypothesis: hardware gating of P1_0 /
index / P0_18; verify with a scope]**.

### OOTX sync pulse-width sets (flash @0xE590, 4 × {u32 name ptr, u16[8]})
Index order is `[skip<<2 | axis<<1 | data]`, units = 48 MHz ticks:

| Set | [0] j0 | [1] j1 | [2] k0 | [3] k1 | [4] | [5] | [6] | [7] |
|---|---|---|---|---|---|---|---|---|
| 0 | 1500 | 2000 | 1750 | 2250 | 2500 | 2500 | 2500 | 2500 |
| **1 (LH 1.0 standard)** | **3000** | **4000** | **3500** | **4500** | **5000** | **6000** | **5500** | **6500** |
| 2 | 7200 | 8400 | 12000 | 13200 | 14000 | 14000 | 14000 | 14000 |
| 3 | 8858 | 1 | 8870 | 1 | 8883 | 1 | 8895 | 1 |

These exactly match the host decoder (`62.5 µs + 10.417 µs·(skip·4 + data·2 + axis)`). Each mode
selects a set through its `mode.N.ootx` parameter, so **the pulse widths and the skip bit are
configuration, not code**.

### Modes / channels
Up to 8 modes (`mode.count`, `mode.current`), each with `period`, `timebase.mode`,
`timebase.offset`, `carrier`, `label` (ASCII shown on the display) and `ootx`.
Timebase modes (enum strings @0x1229A): `master-sync`, `master-async`, `slave-cable`,
`slave-optical`, `master-tdm`. Internally, mode value 4 is the optical-slave path (correlator,
purple LED while searching) **[C]**. The mode button (short press < 1 s) cycles modes and saves
to EEPROM (`button_event` @0x3430).

### Status LED colours (LED0), `ui_update` @0xC3C8
Bit 1 = R, 2 = G, 4 = B (table @0xE520: R1 G2 B4 Y3 P5 A6 W7). Green = running/locked,
blue = rotors not locked, purple = optical slave searching for sync, white = special states.

### Console (`lhtx>` on the UART)
Command table @0xE160 (20-byte records `{name, handler, min/max args, help}`):

| Command | Handler | Help text |
|---|---|---|
| `id` | – | View device identification |
| `reboot` | 0x55E8 | Reboot board |
| `isp` | 0x4388 | Invoke bootloader for ISP |
| `stats` / `clear` | 0x577C / 0x5740 | toggle / clear statistics |
| `param` | 0x61B0 | `list|info|get|set|default|default-all|erase|raw|load|save` |
| `uptime` | 0x5850 | System uptime |
| `dump` | 0x393C | Toggle rotor data dumping |
| `step` | 0x57B4 | Test rotor step response |
| `ram` / `eeprom` | 0x5530 / 0x3988 | RAM inspect; EEPROM `r|w|e <addr> <len>` |
| `pwmscan` `pwmopt` `pwmcal` | 0x51B4 / 0x4E24 / 0x4980 | coarse motor PWM scan / optimise / calibrate |
| `crash` | 0x38F8 | Crash system |
| `isl58303` / `lis2dh` | 0x42DC / 0x484C | raw laser-driver / accelerometer registers |
| `auto` `autodebug` | 0x3878 / 0x38B4 | automatic PWM tuning (+debug) |
| `pll` `plldebug` | 0x4914 / 0x4948 | PLL control (+debug) |
| `freq` / `carrier` | 0x567C / 0x5620 | rotor spin frequency / optical carrier frequency |
| `factory` | 0x3CA4 | `save-cal|load-cal|erase-cal|laser-cal|calibrate-apc|laser-bias|laser-test|laser-sweep` |
The `param` interface exposes ~150 named parameters (rotor PLL gains, laser timing, mode table,
OOTX widths, emission masking...), so much of the multi-station work may be possible through
parameters before any code patching.

---

## 5. Flashing (no programmer needed)

The LPC11U3x boot ROM contains a USB mass-storage ISP. It runs when **PIO0_1 is low at reset and
VBUS is present**, and PIO0_1 is the mode button. So:

1. Remove power and USB. **Hold the mode button**, connect micro-USB (and power).
2. A drive labelled **`CRP DISABLD`** appears with a single `firmware.bin` (the whole flash).
3. **Back it up first** (`cp …/firmware.bin backup.bin`). This is a full flash dump of your unit.
4. Write the new image in place with the firmware package's `install-firmware.sh`,
   which uses `dd conv=notrunc,nocreat`; do not delete or `cp` over the file.
5. Power-cycle without the button.

Because ISP entry is in mask ROM and keyed on the button, **a bad custom image cannot brick the
board** as long as CRP stays 0xFFFFFFFF (our linker script pins it) and the image checksum is
valid. Running firmware can also jump into ISP: stock `isp` console command; custom firmware
`isp` command or a 3 s button hold.

---

## 6. Safety notes for custom firmware

* The rotor-spinning laser is Class 1 only *because it sweeps*. A stationary beam is not eye
  safe. Stock has a `laser.interlock` and only enables lasers once the rotors are locked
  ("transmit sync only when rotors locked").
* The ISL58303 keeps its register configuration across an MCU reset. Any custom firmware must
  drive **P1_12 and P0_16 low** immediately at reset (the `lhfw` startup does this before
  touching RAM).
* Never issue IAP command 61 (EEPROM write) unless deliberately. Factory calibration
  (`fcal.*`, serial, HW_ID) lives there and is what the tracked devices receive over OOTX.
  Dump it first (`lhfw> eeprom 0 4032`).

---

## 7. Open questions (to verify on hardware)

1. ~~Sync-LED gating~~ resolved (see log): index edge starts, P0_18 ends, P1_0 gates.
2. ~~Which of P0_12 / P1_4 is the 3.5 mm cable input~~ resolved (see log): P0_12 cable, P1_4 optical.
   Narrowed (log 2026-11-08): with P1_0 high a cable slave's input latches high from an OOTX frame
   boundary until P1_0 drops. Still open: what sets it there (not the flash data; either base's frame
   boundary fits, the two stay in step). Needs the schematic or a scope on the jack.
3. Coarse vs fine motor PWM (CT16B1 vs CT16B0) and the motor driver IC.
4. ~~P0_23, P1_5~~ resolved (emitter power, pulse stretch). P0_17, P0_19 still unknown (stock never reads P0_19).
5. UART is wired to the Bluetooth chip, which uses stock's console for power management (see log).
6. ~~Exact MCU~~ resolved: 128 KB flash → LPC11U37. Device dump (backups/device-flash-original-128k.bin) = stock image byte-for-byte + 0xFF fill.

---

## 8. Hardware verification log

| Date | Result |
|---|---|
| 2026-10-18 | Device flash dump = stock image + 0xFF fill; chip has 128 KB (LPC11U37) |
| 2026-10-18 | File-manager copy onto the ISP drive corrupts flash (first 3 KB lost, rest shifted +1 KB → blank vectors, board looks dead). Recovered with `tools/flash.sh` (in-place O_DIRECT write) + `--verify` in a fresh ISP session |
| 2026-10-18 | `lhfw` 0.1 boots: 48 MHz clock init, display shift register (P1_20/22/23/24) and LED0 (P1_16/17/18, active low) confirmed |
| 2026-10-18 | Mode button on PIO0_1 (active low) confirmed; LED0 colours G/B/Y/P match the colour-bit map; `iap_reinvoke_isp` sequence (3 s hold) re-enters USB ISP |
| 2026-10-18 | EEPROM dumped via lhfw 0.2 (copied to flash sector 31, read through the ISP drive). 3 parameter partitions, CRC32 OK. `fcal.*` matches SteamVR lighthousedb.json for serial 2701244469 = 0xA101B835 = `sys.name` |

## 9. EEPROM / parameter format (decoded)

* Partitions at EEPROM 0x000, 0x300, 0x600: `{"CPRM", u32 version, u32 crc32}` + 551-byte payload.
  0x000/0x300 = current/previous user saves (versions alternate), 0x600 = factory (version 3).
* Payload = values of every stored parameter (flags bit0 clear) concatenated in table order.
  Table: flash 0xECF0–0x10410, 185 × 32-byte `{type, ram*, name*, default*, size|flags<<8, ?, desc*, enum*}`.
  Types: 1 bool, 2/3/4 u8/u16/u32, 5 i8, 7 i32, 8 float, 10 string, 12 reserved slot, 13 enum
  (null-terminated string-pointer list). Flags bit1 = factory calibration (`fcal.*`).
* 0x80B–0x837: older-format optical calibration copy. 0xDC0–0xE0F: ASCII factory IDs; HW_ID at 0xE08.
* This unit: mode table A = slave-cable, b = master-tdm (offset 20000), c = slave-optical (current),
  L = master-sync (long pulses), 1–4 = master-async at 765053/836543/874755/914713 ticks (short pulses).
  All carriers 1.8432 MHz. Rotor coarse PWM ≈ 1615, laser currents 172/164.
| 2026-10-18 | lhbs phase 1: USB CDC console (ROM USBD; hw API table order verified from ROM dump), byte-exact parameter load/save (CRC 0xCAE6D274 = host-model prediction for c->A) |
| 2026-10-18 | lhbs phase 2: both rotors spin up and lock (60.000 Hz, +/-25 ppm, phase err ~1 us) with reimplemented stock PLL/auto-tune; auto-tuned coarse PWM 1573/1583 vs stock-saved 1615/1611 |
| 2026-10-18 | lhbs phase 3a: sync emission confirmed with an Index controller: 120 Hz flashes, widths j0/k0/j1/k1 = 64.5/75.0/85.3/95.8 us (= spec + 1.98 us controller offset); OOTX decoded from captured light, CRC OK, serial 0xA101B835, fcal = SteamVR DB. Confirms: flash starts on rotor index edge (hardware), ends on CT32B0 MAT0 (P0_18) rising; P1_0 = emission gate; P0_23 low = emitter power; P0_20 = carrier enable; serial = crc32(MCU UID) |
| 2026-10-18 | lhbs phase 4: lasers sweep (ISL58303 regs = stock, fixed saved current 172/164, no APC yet). Original SteamVR disambiguator emits frames for base A101B835 (60 Hz, no jitter); openhouse session tracks the Index controller (base located at 0.69 m, calibrated). |
| 2026-10-18 | lhbs APC: power sense AD2 (laser0, P0_13) / AD0 (laser1, P0_11), FPD gain latch P1_2/P1_20/P1_3 (8 bits MSB first = rev3(g0)<<3 or rev3(g1)). Fixed saved current read 33.6/32.7 mW; with APC settles to 29.2/30.3 mW at current 162/158 (laser0 = factory value). Partition table confirmed by emulating stock scatter-load: {0x000 user, 0x300 user, 0x600 factory}. |
| 2026-10-18 | lhbs optical slave (channel c) locks to a stock master (channel b, E236DC40): correlator locked, phase err ~5 us, rotors locked to master. Original SteamVR disambiguator pairs both bases in TDM (A101B835 + E236DC40, equal frame counts, 0 jitter). Optical receiver = P1_4 (CT32B1 CAP0, both edges) with P1_15 high; classifier table @0x11EDC; slave offset = timebase.optical.offset 20000 ticks = 1/2400 s. lhbs adds a guard against a correction scheduling MR1 in the past (seen once, cause not yet identified). |
| 2026-10-18 | Bluetooth power management: separate Broadcom BLE chip (GATT: vendor service 0000cb00/char cb01 = power control, plus a WICED OTA service 9e5d1e47-... that must not be written). A 20-byte write 12 <00 wake/02 sleep> <u16> <base ID u32 LE> 00*12 to cb01 makes the BLE chip type 'param set sys.standby 0/1' (and 'param get sys.standby', expecting 'sys.standby\t<true|false>\r\n') into the main MCU's UART console (P1_26/P1_27, 115200). lhbs implements that console subset; tools/lhpower.py does sleep/wake from Linux (verified on lhbs and stock). |
| 2026-10-18 | Open: remote ISP re-entry after lhbs USB use fails (ROM USB drive times out); manual button entry works. Boot ROM writes 0x3456ABCD near top of SRAM0 on reset |
| 2026-11-08 | Sync cable, two HTC 1.0 bases on lhbs with this cable support (slave on A, master on b, the 3.5 mm sync cable between them). v1.0.1 lhbs ignored the cable: A covered with the cable in went purple. Stock cable protocol read from the stock image (`timebase_capture_isr` @0xC8B0, `timebase_mr1_isr` @0xCC8C): one pulse per period, rising at the master's edge, falling `period/2` later (`± 100 000` ticks in master-tdm by the skip flag); a slave locks on two rises `period ± 10 000` apart and reads `width <= half + 50 000` as "sweep". With it: the slave receives the master's pulses on P0_12 at exactly 300092/500092 ticks. While emitting with P1_0 high it lost the lock for ~11 periods once per OOTX frame (every 179 periods = 358 bits); 0 losses with `sync off`, unchanged with `laser off` or uncovered; with P1_0 low on the cable, as stock: 0 losses and 0 stray edges in ~10 000 periods covered, two captures per pulse, the slave still flashing. Fallback to optical on unplugging, back to the cable from an optical lock on replugging, a bare jack idle at one level (no cable tries in 60 s while optically locked; since then a try needs a toggling pin in every state) |
| 2026-11-08 | Sync cable, stock and lhbs mixed (`lhcapture --check`, Vive tracker): lhbs master on b + stock slave on A, and stock master on b + lhbs slave on A, both `stock 2-station (417 us)` with each base sweeping 1/2 in separate revolutions. Stock master covered: the lhbs slave alone (`one station`) keeps flashing and sweeping 1/2, so it holds the stock master's cable pulse. Stock's receiver takes the lhbs half-period pulse (it has no optical fallback on A) |
| 2026-11-08 | P1_0 on a cable slave (debug build, slave on A uncovered, lhbs master on b): with P1_0 high, the slave's cable pin reads high throughout each dropout (polled every 10 ms: never low) and nothing is captured; dropouts start at OOTX bit 0-4 of the slave's frame, the same with every bit forced to 0 or to 1 (~1 per 3-5 s each), and with the slave's own sync out undriven. P1_0 forced high even after a loss: the slave never relocks on the cable. With P1_0 low (the fix): 0 dropouts |
