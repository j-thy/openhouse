# lhbs: the Lighthouse 1.0 base station firmware, end to end

This is the "how does the whole thing work" document. The details live in the code
(every function names the stock address it replaces), `ai/firmware/HARDWARE.md` (pins,
peripherals, protocols) and `ai/firmware/MULTISTATION.md` (the multi-station design and its
test log).

## 1. What a base station does

A Lighthouse 1.0 base station is a timing source, not a camera. Every 1/60 s
(one **revolution**, 800 000 ticks of the 48 MHz clock) it does this:

```
 t = 0          a sync FLASH (IR LEDs, everywhere at once)        "a revolution starts"
 t ~ 1-8 ms     rotor 0's laser line sweeps the room horizontally  (axis 0)
 t = 8.33 ms    another sync FLASH
 t ~ 9-16 ms    rotor 1's laser line sweeps vertically             (axis 1)
```

A device's photodiodes see the flash, then the laser crossing them. The time between
the two is the sensor's angle from the base, which is all the tracking needs.

Every flash also carries information in its **width** (62.5 to 135 µs, in 10.4 µs
steps). The width encodes 3 bits:
- **skip:** "I am not sweeping after this flash".
- **data:** one bit of the OOTX stream.
- **axis:** which rotor sweeps next.

Bit by bit, the **OOTX** stream sends a 33-byte frame: the base's serial, its factory
lens calibration, its accelerometer reading, its channel and its faults. A device needs
this frame before it can use the base.

Two stock bases share a room by taking turns (**TDM**). Channel A is the master and
sweeps when B skips; B is a slave, locked optically to A's flashes. Channel C runs
alone.

## 2. Hardware in one paragraph

- **MCU:** NXP LPC11U37 (Cortex-M0, 48 MHz from a 12 MHz crystal).
- **Rotors:** two motors. A coarse PWM sets each motor's speed; a fine PWM driven by a
  DPLL locks it to the timebase. An index sensor fires once per turn of each rotor.
- **Lasers:** an ISL58303 dual laser-diode driver, set over a 3-wire bus. Each laser
  is switched on and off once per turn by timer events. A photodiode (FPD) measures
  the laser power.
- **Sync flash:** IR LEDs whose carrier comes from an LTC6904 oscillator (1.8432 MHz),
  plus an optical receiver that slaves use to see the master.
- **Other parts:** a LIS2DH accelerometer, a 7-segment display, two RGB LEDs and a
  button. A separate Bluetooth chip sends standby/wake commands over a UART.
- **Storage:** EEPROM holds the parameters (stock `cparam` format, three partitions).
  The factory calibration lives in partition 0x600.

## 3. Source map (`src`)

| File | What it does |
|---|---|
| `main.c` | Init order and the main loop's periodic tasks (table below) |
| `sys.c`, `startup.c`, `lpc11u.h`, `board.h` | Clocks, vectors, registers, pin map |
| `params.c`, `params_table.h` | The parameter store: every stock parameter, default and EEPROM layout, loaded and saved in stock format |
| `modes.c` | Channels: the stock mode table (A master, b and c slaves), the button cycle `A b c 1 2 3 4`, applying a channel |
| `timebase.c` | The heart: one edge per revolution on CT32B1, skip/axis decisions, the optical slave (correlator + phase loop), the sync cable, emission control, standby |
| `rotor.c` | Rotor speed control: coarse PWM, the DPLL to the timebase, lock detection, kick and fault on stall |
| `laser.c` | The ISL58303, per-turn laser on/off windows, power regulation (APC) from the photodiode, and the safety layers |
| `ootx.c` | Builds and encodes the OOTX frame (double buffered), one bit per flash; refreshes the accelerometer in it |
| `multi.c` | lhbs multi-station: the round of bases taking turns, its announcement in OOTX, decoding the master's frame |
| `carrier.c` | LTC6904 carrier frequency |
| `accel.c` | LIS2DH: gravity direction (into OOTX) and the display flip |
| `ui.c` | Display fonts, LEDs |
| `console.c`, `usb_cdc.c` | lhbs USB serial console (stock has none): status, parameters, EEPROM dump, rotor/laser/multi diagnostics |
| `uart_sniff.c` | The stock UART: the Bluetooth chip's `param set sys.standby` commands |
| `iap.c`, `debug.c` | Flash/EEPROM programming through the boot ROM; debug snapshots readable from the ISP drive |

**Periodic work.** Interrupt priorities run from 1 (highest) to 3.

| When | What |
|---|---|
| CT32B1 interrupt, priority 1 | Timebase edge each revolution; optical captures (slave) |
| CT32B0 interrupt, priority 2 | Rotor index captures, laser on/off events, sync flash timing |
| SysTick, every 20 ms | Emission update, laser watchdog, LEDs |
| Every 10 ms | `accel_task` |
| Every 100 ms | `ui_task` (faults into OOTX, display flip), `laser_poll` (APC current to the driver chip), `multi_task` (master: who is present) |
| Every 500 ms | `carrier_task`, `timebase_supervisor` (stall kick, standby) |
| Every loop | Button, `multi_poll` (slave: check a decoded master frame), `ootx_refresh_accel`, console |

## 4. Start-up

1. Clocks, then the parameters are loaded from EEPROM (the newest valid partition).
   `multi_init` fills in multi-station defaults if the reserved block is empty.
2. The display shows the channel. The rotors spin up (the coarse PWM from
   `rotor.r.pwm`, the factory value).
3. The timebase starts. A master generates its own edges. A slave starts searching for
   a master's flash.
4. The rotors lock to the timebase (DPLL). The LED goes green once both are locked and
   emission is permitted.
5. Flashes and lasers start: lasers only on a running, locked rotor, inside the sweep
   window.

## 5. The timebase and the optical slave (`timebase.c`)

- **Master:** CT32B1 makes an edge every `timebase.period` ticks. Each edge is a new
  revolution. The flash is timed a quarter period after the edge.
- **Slave:** the optical receiver captures every flash. Its width is classified into
  (skip, data, axis). Axis-0 flashes feed the **correlator**:
  - It searches for flashes exactly one period apart, then settles for 300 periods,
    then locks.
  - A **phase loop** (PID) then steers our own edge so that our flash lands
    `offset` after the master's.
  - Losing the master (no flashes for 300 periods) sets the correlator to *lost*.
  - LED purple blinking means "waiting for a master".
- **lhbs fix:** while searching in multi mode, the correlator only looks at the first
  flash of each group, which is the master's. Stock logic took any two back-to-back
  flashes one period apart, which never happens once other slaves are flashing.
- **Sync cable (lhbs, as stock):** channels A, b and c drive the jack (P0_14, CT32B1 MAT1): a
  pulse rising at each edge, falling `period/2` later, `± 100 000` ticks by the skip
  flag in master-tdm (opposite sign in slave-optical); multi leaves P0_14 undriven, as
  v1.0.1 did (no multi station listens, and all of them raise P1_0). Channel `A` (slave-cable)
  listens on P0_12 first: two rises one period apart lock it, the rises drive the
  rotors and the width sets sweep/skip. Locked, only edges that fit the signal count
  (rises at 1–9 periods, the three widths stock sends). A loss gives the cable 2 s to
  come back, then it follows optically; it goes back to the cable only when the pin
  (sampled every 10 ms) toggles, even from an optical lock. Tries that find no lock back
  off from 5 s to 60 s. A cable slave keeps P1_0 low, as stock does for slave-cable:
  raised, it corrupts its own cable input. `corr` shows the cable state.
- **Emission permission:** a slave flashes only while locked and holding the phase
  (within 20 µs), or locked to the cable. A master always may.
- **Standby:** with no permission for 60 s, the base goes to standby. Rotors and lasers
  stop, the display goes dark and LED1 is green. Stock wakes only on the button or
  Bluetooth. **lhbs:** a slave keeps listening and wakes by itself once it locks onto a
  master again, and it doesn't time out while it hears one.

## 6. Rotors (`rotor.c`)

- Each rotor's index sensor gives its phase once per turn. A DPLL compares that phase
  with the timebase and drives the fine PWM. The coarse PWM is fixed per unit (factory
  `rotor.r.pwm`, with `pwm.m`/`pwm.b` for temperature).
- *Locked* means the error has stayed small for a while. Both rotors must be locked
  (and 180° apart) before the lasers may fire.
- A rotor that doesn't start within the kick timeout faults. The LED then blinks red.

## 7. Lasers and their safety (`laser.c`)

- **Firing:** every turn, a timer event switches the laser on at `laser.r.start`
  degrees and off again 124° later. That window covers the room in front of the base;
  the rest of the turn is dark.
- **Safety layers:** a stationary beam isn't eye safe, so several layers keep the
  lasers off unless everything is right:
  - on only from the per-turn event of a running, locked rotor;
  - off at the end of the window;
  - forced off by the rotor-control check (60 Hz) and a SysTick watchdog (50 Hz);
  - never in standby, without emission, or in a revolution this base skips.
- **Power regulation (APC):** each sweep, the photodiode reading is turned into mW
  with the unit's calibration (`laser.r.pwr.m/b/b2`). The drive current then steps
  toward the 30 mW target.
  - **Safety window:** at most 60 steps above the current the base booted with, and
    never above 230. Hitting it means the power sense doesn't follow the current: laser
    FAULT, lasers off, LED blinking red.
  - It used to be +20, which faulted healthy units as they warmed up (D7B1AD76:
    142 → 162 holding 29 mW).
- **Factory calibration matters:** `laser.r.bias/current/pwr.*` and `rotor.r.pwm*` are
  per unit. The `calibration-rescue-244` stock image (2016-03-19) resets them to
  generic defaults. Don't use it. 89AE90F2 had to be restored from a backup with the
  console's `set`/`save`.

## 8. OOTX (`ootx.c`)

Payload v6 (33 bytes; the full layout is in the header of `ootx.c`):
- firmware version and the serial (crc32 of the MCU UID);
- the factory lens calibration per axis: phase, tilt, curve, gibbous phase and
  magnitude;
- the rotor-lock count;
- the hardware model;
- the gravity direction (int8 × 3);
- the mode byte and the faults.

The frame is 17 zeros, a 1, then the length, payload, padding and crc32, with a 1
stuffed after every 16 bits. At 120 flashes/s a frame takes about 3 s.

**lhbs additions:**
- **Accelerometer refresh:** the frame is rebuilt with the current accelerometer
  reading (at a frame boundary) when it changed by 2 counts or more. Stock sends it
  once at power-up. The driver uses this to notice a knocked base.
- **Multi mode:** the payload grows to 34 bytes. Byte 31 becomes the multi byte
  `0x80 | ALL<<6 | (N-1)<<4 | slot`, and byte 33 is the round mask.

## 9. Multi-station (`multi.c`, `ai/firmware/MULTISTATION.md`)

The goal is 2–4 bases sharing a room, each sweeping in turn, with no cables and no
stock-incompatible flashes.

- **Channels:** the button cycles `A b c 1 2 3 4`. Slots 1–4 are multi channels. Slot 1
  (slot 0 inside) is the master; the others are optical slaves. Each slave's flash comes
  `slot × 250 µs` after the master's (`stagger` 12 000 ticks). Defaults after flashing:
  multi enabled, policy ALL (every flash carries an OOTX bit).
- **The round:** the bases present take turns, one whole revolution each. The k-th
  present slot sweeps when `rev % count == k`, and the master is always k = 0. A base
  not in the round, or a slave that doesn't know the round yet, only flashes.
- **Who is present:** the master watches the slaves' flashes (its receiver is on in
  multi mode). A set of slots seen for 1 s is announced in OOTX byte 33. The master
  switches at the superframe after that frame went out whole; a slave switches at its
  own superframe after decoding the frame from the master's flash widths.
- **Collision guards:**
  1. A guard of 4 superframes after any change of the round.
  2. A slave that sees the master sweep where its own round doesn't expect it drops the
     round and goes dark until the next decoded frame.
  3. Per-revolution evidence: a slave sweeps only if, this revolution, it saw the
     master's "not sweeping" flash and no earlier slot's "sweeping" flash.
- **Joining:** a slave flashes as soon as it holds the master's phase, and sweeps once
  the round is confirmed twice. Earlier, flashing also waited for the confirmation,
  which deadlocked joining for minutes.
- **A slave locked onto another slave** (the master hidden while it searched) decodes a
  frame whose slot isn't 0. It drops the lock and searches again (console `multi`:
  "from a slave (relocked)").

**Rates:** each base sweeps 60/N full scans per second. Measured on ms4-01: 15 Hz with
4 bases, 20 with 3, 30 with 2 and 60 alone, with no collisions. The practical ceiling is
about 6 bases: the flash burst has to finish before the sweep reaches the field of view,
and the worst case (a device seeing only one base) is 60/N Hz.

## 10. The console (USB, `/dev/ttyACM*` or `/dev/lhbs`)

| Command | What it shows or does |
|---|---|
| `status` | Firmware, channel, timebase, emission, standby, faults, gravity |
| `params [pattern]`, `get`, `set`, `save` | Parameters, in RAM and saved to EEPROM in stock format |
| `rotor`, `watch` | Rotor PLL and lock |
| `laser`, `laser on/off`, `apc on/off` | Laser driver, power, current, FAULT (`laser on` acknowledges a fault) |
| `corr` | Slave correlator state, cable state (input, lock, losses, stray edges) and the trace |
| `multi ...` | The round, seen slots, decoded frames and relocks, guard, superframe confirms and realigns; `enable`, `slot`, `policy`, `off` |
| `ootx`, `eeprom`, `uartlog` | OOTX frame, EEPROM hex dump, Bluetooth UART traffic |
| `isp`, `reset` | Reboot into the USB bootloader, or reboot |

**Reading a healthy multi slave:** `correlator locked`, small `realigns`, `superframe
confirms 3`, and no "[no round from the master yet]". A healthy laser shows about
29–30 mW with no FAULT.

## 11. LEDs

| LED | Meaning |
|---|---|
| Green | Locked and emitting |
| Blue | Emitting, rotors not (yet) locked |
| Blinking purple | Slave waiting for a master |
| Blinking red | Fault: laser APC window, or a rotor that didn't start |
| LED1 green, display dark | Standby |

## 12. Build, flash, test

```sh
cd lhbs && make                 # build/firmware.bin, checksummed
make hosttest                   # 4 simulated stations: the round, collisions, stress seeds
# ISP: power off, hold the mode button, power on -> "CRP DISABLD" drive
../scripts/build-firmware.sh && ../dist/openhouse-basestation-firmware/install-firmware.sh   # backs up, writes in place; never drag-and-drop
```

The installer backs up the old flash first. lhbs flash images also carry an EEPROM
snapshot (`LHEEDUMP`) with the unit's calibration, so a wiped unit's calibration can be
recovered from a backup.
