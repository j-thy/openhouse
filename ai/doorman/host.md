# doorman on the host (openhouse)

## Transport: `src/transport/usb/doorman_dongle.cpp/.hpp`
* `UsbTransport` opens 6767:0727 next to Watchman dongles and pumps `DoormanDongle` from its reader thread;
  both HID interfaces are drained (`dev1_` = odd slots).
* Each slot becomes **virtual wired endpoints** (IMU, optical, buttons). Frames are wired reports with
  another first byte and are relabelled into the normal `ReportDecoder`:
  `p`→0x20 IMU, `q`→0x21 LH1 light, `s`→0x28 LH2, `Q`→buttons 0x0100, `n`→0x0700 (Index input),
  `V`→other button-interface reports, `B`→battery, `P`→feature replies.
* Identity: our Tracker 1.0 path sends `H`; otherwise read from the device (feature 0x01 IMU ranges,
  config `device_serial_number`). Config: SteamVR's lighthouse config cache first, else read over the radio
  (0x10/0x11, ~0.3 s) on a worker thread. Sensor type from feature 0x05 byte 41 when the config lacks it.
* Haptics, power off go down as `P` frames. Captures (`.lhraw`) replay like wired devices.
* Stream reassembly: a `0x80` report carries whole `[len][frame]` items and discards an unfinished
  fragment for its slot; consecutive `0x81` reports carry the raw bytes of `[len][frame]` and are
  buffered in `frag_[slot]` until the first byte (the length) says the frame is complete. A fragment
  that brings no bytes at all — its start was lost, or a short report — leaves the accumulator empty
  and is ignored: reading its first byte for the length was an access violation (a 3-byte
  `[0x81][slot][0]` crashed the driver), covered by `Doorman.AZeroLengthFragmentWithNothingBufferedIsIgnored`.

## Receivers and pairing
* The dongle is receiver `OHDG-<usb serial>`, each slot `OHDG-<serial>/<n>` (n 0–5); they show in
  `Receivers()` (lhviz Receivers panel, badge "OH"; telemetry server per-slot stats).
* `UsbTransport::SetPairing(receiver, ms)`: the dongle → first free slot, `/n` → that slot (dongle fw ≥
  0x0004). SteamVR's pairing UI goes through the driver's pairing mailbox to the same call (untested).

## `apps/doorman-dongle-manager` (dongle manager)
ImGui desktop window in openhouse-overlay's style; pages slots / radio / channels / firmware / about.
Pair into a slot, forget, live RSSI and loss, channel mask, dongle firmware update (UF2 copy), restart,
crash record. Works without the driver; with it, device names come from driver telemetry. Shipped as a
plain binary: `tools/` in the Linux package, `apps/` in the Windows zip; **not installed or registered**.
Test hooks: `OPENHOUSE_DONGLE_SNAPSHOT=FILE.ppm`, `OPENHOUSE_DONGLE_PAGE=...`.

## Python
`doorman/tools/doorman_link.py` mirrors the transport for scripts; `dongle.py` is the CLI (`README.md`).
