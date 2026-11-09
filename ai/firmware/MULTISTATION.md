# Multi-station Lighthouse 1.0: 3–4 base stations

Status: **firmware implemented (lhbs `multi.c` + timebase), single-unit validated**; driver not yet. Hardware for testing: 2 stations now, 4 later.
Applies to lhbs (our firmware) plus a matching change in openhouse (our driver).
Stock-compatible channels stay exactly as they are; multi-station is an opt-in mode.

## 1. Constraints (measured / decoded from stock)

| Quantity | Value | Source |
|---|---|---|
| Rotor revolution | 16.667 ms (800 000 ticks), both rotors 180° apart | `timebase.period`, rotor PLL offsets ±200 000 |
| Laser 0 window | 32°–156° after its index = 1.48–7.22 ms, **5.74 ms long** | `laser.0.start/length` |
| Laser 1 window | 25°–149° = 1.16–6.90 ms | `laser.1.start/length` |
| Sync flash | starts on the rotor index edge, 62.5–135 µs (8 width classes) | `ootx.j0..k3`, hardware |
| Stock slave offset | 20 000 ticks = 417 µs (1/2400 s) after the master's flash | `timebase.optical.offset` |
| Optical throughput | one laser in the room at a time ⇒ **120 sweeps/s total** (60 per axis) | geometry |
| Receivers | Watchman timestamps every pulse (start, width); no demodulation | LH1 receivers |

Consequences:

1. **The room's sweep capacity is fixed.** With time division, N stations share
   60 sweeps/s per axis: 2 stations → 30/s each (stock), 3 → 20/s, 4 → 15/s. A tracked
   device that sees all stations still gets 60 sweeps/s per axis in total — the same
   aggregate rate as stock with 1 or 2 stations, but from more viewpoints (coverage and
   occlusion robustness are what multi-station buys).
2. **Sweeps cannot share a revolution.** A sweep (5.74 ms) is longer than a quarter turn
   (4.17 ms), so two stations offset by 90° would have lasers in the room simultaneously.
   The original plan's 4.166 ms slots do not fit; the time-division unit must be a whole
   revolution.
3. **All sync flashes of one index event must fit in ~1.02 ms**, otherwise a flash lands
   inside the active station's sweep (latest flash must end before the earliest sweep
   start at 1.16 ms; the latest station's sweep must end before the next flash group at
   8.33 ms). Stock's 417 µs stagger fits 2 stations (0.42 ms); for 4 stations it would
   span 1.25 ms and collide. **250 µs (12 000 ticks) fits 4 stations in 0.75 ms.**
4. Faster rotors would add capacity but stress the motors/mirrors and halve angular
   timing resolution — rejected.

## 2. Chosen design: revolution round-robin, staggered flashes, stock bit semantics

### 2.1 Schedule

- N ∈ {2, 3, 4} stations, slot s ∈ {0 … N−1}. Slot 0 is the **master**, the others are
  optical slaves of it.
- Revolution counter r (shared, see 2.4). In revolution r the **active** station is
  `s = r mod N`. It sweeps both axes in that revolution (laser 0 after its rotor-0 index,
  laser 1 after its rotor-1 index), exactly like a stock station in its non-skip
  revolution. All other stations keep their lasers off.
- Station s's rotors (and hence its flashes) are phase-shifted by `s × 12 000` ticks
  (250 µs) relative to the master. Stock 2-station channels keep 20 000 ticks.

```
one revolution (16.667 ms), N = 4, revolution r with active slot 1:

index event (rotor 0)                               index event (rotor 1)
|M|  |1|                                            |M|  |1|
0   0.25 ms   [========= slot 1 laser 0 =========]  8.33 ms  [==== slot 1 laser 1 ====]
 M = master flash (skip=1), 1 = slot-1 flash (skip=0), others dark this revolution
```

### 2.2 Flash policy

Each index event, a station flashes if it is the master **or** the active station
(policy `MASTER+ACTIVE`, default). Optionally `ALL`: every station flashes every
index event.

| Policy | Flashes per index event | Flashes/s in the room | Slave OOTX rate |
|---|---|---|---|
| stock 2-station | 2 | 240 | 120 bit/s |
| **MASTER+ACTIVE** (default) | 1–2 | ≤ 240 (same as stock) | 120/N bit/s (~12 s per frame at N=4) |
| ALL | N | 120·N (480 at N=4) | 120 bit/s |

MASTER+ACTIVE keeps the optical load and the Watchman report rate identical to stock
2-station operation: every index event carries at most one skip=1 flash (master) and one
skip=0 flash (the sweeping station). The receiver/driver structure per index event is
therefore the stock one; only the identity of the skip=0 flash rotates. OOTX of the slaves
is slower but it is only needed when a station is first seen (the driver caches it).
`ALL` exists for the case where per-station sync visibility matters more than bandwidth.
Watchman throughput must be measured before ALL is considered (risk R2).

### 2.3 Bits and widths: unchanged

Widths keep their stock meaning (`skip<<2 | axis<<1 | data`):

- **skip = 0 ⇔ this station sweeps after this flash** — still exactly true, so the driver
  can attribute sweeps without knowing the schedule.
- axis and data (OOTX) unchanged; OOTX frame format unchanged (v6, 33 bytes).

The original plan's idea of reusing the skip bit as a pair id is not needed: with
round-robin revolutions, skip already tells the driver which station sweeps, and it keeps
the encoding valid for stock-style 2-station operation.

### 2.4 Synchronisation

- **Master (slot 0)**: free-running timebase as today (`master-tdm` behaviour), counter
  r increments every revolution.
- **Slaves**: optical slave as today (correlator + phase loop), with target offset
  `s × 12 000` ticks.
- **Finding the master** among up to N−1 visible stations: under MASTER+ACTIVE the master
  is the only phase cluster that flashes **every** revolution (slaves flash only in their
  active revolution); under ALL it is the earliest flash of each group (groups are ≤ 1 ms
  wide and separated by ≥ 7 ms). The correlator accepts only that cluster.
- **Superframe phase**: the master's skip=0 flash marks r ≡ 0 (mod N). A slave sets its
  counter there and re-checks on every master skip=0 flash; a mismatch drops emission
  (unlock) rather than risk two stations sweeping together.
- Loss of master ⇒ slave stops emitting (stock behaviour) and enters stand-by after 60 s.
  Chained sync to another slave is possible later (its slot is known from its offset
  relative to the group start) but is not in the first version.

### 2.5 Identification for the driver

- Phase cluster per station: offset `s × 250 µs` within the group ⇒ slot.
- OOTX payload byte 31 (`mode`): stock sends `mode.current & 7`. Multi mode sends
  `0x80 | (N−1) << 4 | s`. The high bit lets the driver auto-detect multi-station
  stations; N and s let it cross-check the cluster assignment.
- Serial (OOTX bytes 2–5) unchanged, so SteamVR/driver calibration databases keep working.
- Note: SteamVR's OOTX parser (ported in openhouse `ootx.cpp`) already treats bit 7 of this
  byte as "mode 4 + parameter" for hardware revision 0x0D; for our hw 9 it computes
  `(byte & 0x7F) + 1` and maps values > 3 to mode 0. The encoding follows Valve's bit-7
  convention; the driver change only needs to extend it to hw 9.

## 3. Firmware changes (lhbs)

1. **Configuration without breaking stock EEPROM compatibility.** The 551-byte stock
   parameter layout stays untouched; stock channels A/b/c and `mode.count = 3` are left as
   they are, so reflashing stock firmware stays safe (stock asserts on unknown timebase
   modes). Multi-station settings live in the 120-byte reserved slot of the payload
   (`reserved1`, all zero today and preserved by stock): `multi.enable`, `multi.n`,
   `multi.slot`, `multi.stagger` (default 12 000), `multi.policy`, and a channel list.
2. **Channels**: button cycles A, b, c (stock) then multi channels shown as `1`…`N`
   (= slot + 1). Selecting a multi channel never writes non-stock values into stock fields.
3. **Timebase**: replace the 1-bit TDM flag with `rev_count mod N`; slaves derive it from
   the master's skip=0 flash; correlator filters for the master cluster (2.4).
4. **Emission**: per index event decide flash on/off (policy) and skip bit
   (`skip = !(r mod N == slot)`); lasers only in the active revolution (existing laser
   events already gate on the skip flag).
5. **OOTX**: mode byte as in 2.5; rebuild on channel change (already done).
6. **Masking**: unchanged (own sweep + presync window). With 250 µs stagger, check that the
   4000-tick presync mask (83 µs) still precedes our flash without hiding the master's.
7. **Console**: `multi` status (slot, N, counter, master cluster, superframe lock).

## 4. Driver changes (openhouse)

Flag `multistation = off | on | auto` (auto: on when any base reports the OOTX multi bit).

1. Disambiguator: allow up to 4 bases (stock limit 2); sanity check accepts N bases
   within one period and distinct serials.
2. Group the flashes of an index event (≤ 1 ms window) and assign each to a phase slot of
   250 µs (multi) or 417 µs (stock 2-station).
3. Sweep attribution: samples in `[sync + P/12, sync + 5P/12]` belong to the base whose
   flash in the group had skip = 0 (unchanged rule, now with more candidates).
4. OOTX: for slaves under MASTER+ACTIVE, revolutions where a base is scheduled dark are
   "not expected", not "missed" (no 0xFF vote).
5. Base detection: a base may appear only every N-th revolution; clustering and period
   estimation must accept that cadence.
6. Pose/room solver: unchanged in principle (more bases = more constraints); room map
   bootstrapping with ≥ 3 bases should be exercised.

## 5. Validation plan

| Step | Hardware | What is proven |
|---|---|---|
| V1 | 2 stations, multi mode N=2, policy MASTER+ACTIVE | new code path ≡ stock TDM; driver multi flag with 2 bases |
| V2 | 2 stations as slots 0 and 2 of N=4 (slots 1, 3 empty) | superframe sync, 1-in-4 cadence, sparse groups, driver cadence handling |
| V3 | 2 stations, policy ALL | Watchman report-rate headroom (risk R2) |
| V4 | 4 stations, N=4 | full system, pose with 4 bases, occlusion tests |
| V5 | mixed: stock station on A/b/c alongside? (no — stock cannot join multi mode) | confirm multi and stock channels are mutually exclusive by design |

Measurements to record at each step: flash group timing per slot (controller capture),
sweep attribution errors, correlator lock time, superframe lock stability, pose noise.

## 6. Risks

| # | Risk | Mitigation |
|---|---|---|
| R1 | A slave locks to another slave instead of the master | master = every-revolution cluster; verify on every master skip=0 flash; refuse to emit on mismatch |
| R2 | Watchman/receiver bandwidth with more flashes | default MASTER+ACTIVE keeps stock load; measure before enabling ALL |
| R3 | Stagger margins (0.27 ms at sweep start, 0.36 ms at sweep end) eroded by rotor phase error | slaves already hold ~5 µs phase error; enforce "no laser unless superframe + phase locked" |
| R4 | Per-station update rate drops to 15 Hz/axis at N=4 | inherent to time division; aggregate stays 60/axis; IMU fusion covers it |
| R5 | Unmodified SteamVR sees >2 bases and fails | multi channels are opt-in; stock channels unchanged; document clearly |
| R6 | Reflashing stock firmware with multi config in EEPROM | config kept in the reserved slot only; stock fields never get non-stock values |

## 7. Alternatives considered

- **90° pairs (original plan)**: infeasible — 5.74 ms sweeps overlap within 4.17 ms slots.
- **Frequency division** (different rotor speeds per station, like the factory
  master-async channels 1–4 at 52.5–62.7 Hz): no coordination needed, but sweeps from
  different stations collide at a beat rate and LH1 receivers cannot separate overlapping
  pulses; usable only with low duty or spatially separated stations.
- **Silent non-active stations (no master flash)**: lowest optical load, but slaves would
  see the master only 1 revolution in N, weakening sync; MASTER+ACTIVE costs at most one
  extra flash per index event and keeps sync continuous.
- **Faster rotors / shorter sweep windows**: more capacity but motor stress, lower angular
  resolution or reduced field of view.

## 8. Test log

| Date | Setup | Result |
|---|---|---|
| 2026-10-18 | lhbs (A101B835) as multi master, N=4, slot 0; stock E236DC40 on channel c (stock slave of it) | SteamVR's original disambiguator: A101B835 sweeps exactly every 4th revolution (176/176 gaps), stock slave every 2nd (352/352); no collisions. ~15 sweeps/s per laser (console). OOTX mode byte 0xB0 received (driver shows "mode 0" = unknown value, as expected before the driver change). openhouse bootstrap: 703 attempts, 0 poses — consistent with the driver assuming stock cadence (driver item 5); to be fixed with the driver changes. |
| 2026-10-19 | lhbs A101B835 slot 1 + E236DC40 slot 0, N=4, policy MASTER+ACTIVE (openhouse `captures/ms-a`, Index controller) | openhouse multi disambiguator: detected, both OOTX decoded (mode bytes 0xB0/0xB1), tracking on either station alone. **The slave flashed every index event** (P1_0 gate does not suppress a slave's flash; what P1_0 does on a cable slave: `HARDWARE.md`, the P1_0 row), its non-sweeping flashes with data 0 because the OOTX bit only advanced on gated revolutions: slave OOTX at 2 bits / N revolutions (one frame ~12 s at N=4). Slave flash offset settled 235→269→250 us over ~20 s after start-up. |

### Changes after the 2026-10-19 test

* **Policy ALL implemented and default** (`multi policy all|master+active`): every
  station flashes every index event and every flash carries the next OOTX bit, so a
  slave's OOTX runs at the stock 120 bit/s (frame ~3 s instead of ~12-48 s). Same
  optical load as what the hardware did anyway. Existing EEPROM configs keep their
  policy: set it once with `multi policy all`.
* OOTX mode byte bit 6 = policy ALL: `0x80 | all << 6 | (N-1) << 4 | slot`
  (openhouse reads N from bits 4-5).
* A multi slave only emits once its phase error is < 20 us for 20 consecutive master
  flashes, and stops above 40 us (was: correlator locked + superframe confirmed).

### Runtime round (2026-10-21): N is no longer configured

Each station's slot (button 1-4, slot 0-3) is still set by hand; how many take turns
is found at runtime, so three stations share all 60 sweep revolutions per second
(20 each) instead of four turns with one empty.

* **Defaults.** A fresh EEPROM (or a stock one: the reserved bytes are zero) comes up
  with multi enabled, policy ALL, and the button cycling A b c 1 2 3 4. `multi n` is
  gone.
* **Who is present.** The master listens on its optical receiver too (before, only
  slaves did). Every flash is placed on the slot grid: relative to a station's own
  timebase edge and slot, slot s' flashes at s' x 250 us, at both index events
  (`flash_slot`, 60 us tolerance). A slot whose flash was seen in the last 0.5 s, the
  set unchanged for 1 s, is announced.
* **The round.** The mask of present slots goes in the master's OOTX frame as payload
  byte 33 (34-byte payload, multi mode only; stock modes keep 33), with the count in
  the mode byte's N bits. In revolution r the k-th present slot (by slot) sweeps when
  r mod count = k; the master is always k = 0, so its sweeping flash marks every
  superframe start.
* **Switching.** The master switches once a whole frame carrying the new mask went
  out; a slave once it decoded that frame from the master's flashes (CRC checked;
  the last bit is the same flash), both at their next superframe boundary, both
  restarting the revolution counter there.
* **Safety, three layers:**
  1. A station not in the mask, or a slave without a decoded mask, flashes (so the
     master sees it) but never sweeps.
  2. After any change a slave stays dark for 4 superframes; and the master's sweeping
     flash arriving where a slave's round doesn't expect it means a missed change:
     dark until the next decoded frame.
  3. Per revolution, a slave sweeps only with this revolution's evidence: it saw the
     master's flash saying "skip" (the master's flash is first in every revolution),
     and no earlier slot's flash saying "sweep". A missed change, even one whose
     telltale flash was the one missed, can't make two stations sweep together.
* **Host test.** `make hosttest` loads four copies of multi.c and drives them
  revolution by revolution: stations joining and leaving, 20-30 % of the master's
  flashes lost, 30 simulated minutes of random power changes per seed: no
  revolution with two stations sweeping (layer 3 was added after a seed found the
  one case layers 1-2 left).
* **Console.** `multi` shows the round, the announced mask, the guard, how long ago
  each slot's flash was seen, and on slaves the master frames decoded and failed.
* **Line of sight.** The master must see every slave's flash to count it, and every
  slave the master's. A slave the master can't see is left out (flashes, doesn't
  sweep); the console shows it as "not in the round".

### Slave acquisition with other slaves already flashing (2026-10-21)

Four stations: slaves often stayed purple (searching) for minutes, the more stations
running the worse. The stock correlator, searching, takes two *consecutive* sync
flashes one period apart; with other slaves flashing 250-750 us after the master,
consecutive flashes are never a period apart, so it locked only when a flash was
missed, and then onto whichever station. Fixed: while searching, a multi slave looks
only at the first flash of each group (the master's: slaves flash after it). If the
master was hidden and it locked onto a slave anyway, the frame decoded from the "master"
says slot != 0: the correlator restarts (console `multi`: "from a slave (relocked)").

Joining took minutes even then: a slave flashed only with the round confirmed, and
the master's round changing (to include it) un-confirmed it, so it went dark, dropped
out of the round, and the round changed back (live: realigns 1741, confirms 0, guard
stuck at 4). Flashing now needs only the master's phase held; the round confirmation
gates sweeping alone. Slaves also wake from stand-by on their own once locked onto a
master again, and don't time out to stand-by while they hear one.

## Fixed: slaves flashed the wrong sync axis (2026-10-22)

Seen from the driver in Beat Saber captures: a slave's first sweeping flash of its turn
sometimes decoded as axis 1 while its laser swept axis 0 (A101B835 ~9.5 % of sweeps,
89AE90F2 4.8 %, D7B1AD76 1.5 %). The driver drops such sweeps (`disambiguator_multi.cpp`).

Cause, measured on A101B835 (slot 2) with an instrumented build: the flash ends in the
CT32B0 handler (index capture -> MR0 = t + width), which ran at priority 2, below CT32B1.
A slave's CT32B1 handler, processing the master's axis-0 flash (`corr_callback`, soft
float), ran up to 175 us; slot 2's rotor-0 index comes 250 us after the master's flash,
so its handler started up to 168 us late (1409 late syncs in 9 min). The shortest flash
(axis 0, data 0, 62.5 us) then ended early or late (59-69 us measured), and widths past
~68 us decode as axis 1. Rotor 1 was unaffected (axis-1 master flashes skip the phase
loop), and later slots have more slack (750 us for slot 4). The light itself was right.

Fix (`rotor.c`): CT32B0 at priority 0, above CT32B1 (CT32B1 captures are latched in
hardware, so they can wait). Also the "too late" check compares wrap-safe. After: max
index latency 34 us, 0 late syncs, 0 dropped sweeps in 60 s (was 53). Flash every slave
with this build.
