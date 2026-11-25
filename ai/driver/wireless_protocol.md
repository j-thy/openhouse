# Watchman wireless protocol (dongles 28de:2101 / 28de:2102)

openhouse's own dongle (6767:0727) is a different protocol: `../doorman/protocol.md`.

Port: `src/transport/usb/wireless_decoder.{hpp,cpp}`, `WirelessReportParser` in
`report_parser.cpp`, handshake in `usb_transport.cpp`.
Status: **bit-exact** vs the original decoders (`tests/oracle/test_wireless_diff.cpp`):
Wv2 (5644 packets), legacy v0 (5618 packets) from an Index controller.

## Reports (dongle HID interface 0)
- `0x23`: one 29-byte radio packet at [1]; `0x24`: two packets at [1] and [0x1e] (0x3d1920).
- Packet: `[0] time MSB, [1] length, [2] time LSB, [3..] payload` (length counts LSB+payload).
- Clock sync after each report with lastInBatch: `ClockSync(host − 0.0044 s, IMU ts)`; for 0x24
  with a bad second packet the first packet's IMU stamp is used with `host − 0.0015`.

## Device info and protocol version (0x3dd8a0, 0x3cc9a0, 0x3ccaf0)
- Feature report `ff 83` then read back `ff 83 len {u8 type, u32 value}…`.
  type 0 = tracked device serial (e.g. 0x13DA81D1 → `LHR-13DA81D1`), type 2 bit 3 = versioning
  supported. (The dongle needs a few ms; retry until len ≠ 0.)
- If supported (pid 2101/2102): send feature `ff 87 03 00 <u16 version=1>` and enter
  "pending" (version 2): drop all packets until one has `[1]=3, [3]=0xe2` → version = `[4]`.
  Requesting 0 forces the legacy protocol (the driver always requests 1 now; the
  old override for legacy captures was removed).
- Version 0 → legacy decoder, 1 → Wv2 decoder.
- Not stock: a capture started mid-session (Record Session) has no transition packet,
  and its sub-protocols (`opticalSub`, `controllerSub`) are announced too rarely to
  appear. While on legacy, `WirelessReportParser` also decodes each packet with two Wv2
  probes (optical 1 = LH2 words, optical 0 = LH1 pulses; controller 0, what live devices
  announce); 20 in a row where legacy fails and a probe decodes switches to Wv2. Before
  this, Windows Record Session captures decoded to no IMU and a few hundred pulses.
  Windows pads every report to 59 bytes (Linux: 0x23 is 30).

## Legacy (v0) payload (0x3fdec0)
1. Optional status header `0xe0–0xe7`: bit0 battery byte, bit1 → version-change byte, bit2 skip 1.
2. `0xf0–0xff` input event: low3==0 → 8 raw bytes; else bit0 buttons byte, bit2 trigger byte,
   bit1 4-byte touchpad. `0xf8–0xff` is followed by IMU.
3. `0xe8–0xef` IMU (or after 0xf8+): `u8 seq, int16 accel[3], int16 gyro[3]`;
   timestamp `msb<<24 | lsb<<16 | seq<<8`.
4. Remaining bytes = lightcap (0x3fdb00): sensor bytes `(sensor<<3 | k)` from the front,
   timestamps from the back (last 3 bytes = newest 24-bit stamp, then 7-bit varints, high bit
   terminates, each a delta to the previous). A sensor byte consumes the next unused stamp as
   pulse end and the stamp `k` further as pulse start (`width = end − start ≤ 0xffff`).
   Pulses are pushed to the front (→ chronological). 24-bit stamps are extended with
   msb−1/msb/msb+1, choosing the closest to `((msb<<24)|(lsb<<16)) − 0x744800` (0x3fd540).

## Wv2 (v1) payload (0x3fe6d0)
Flags byte `[1]`: 0x80 IMU (12 bytes after a seq byte, fixed position), 0x40 metadata,
0x20 input, 0x10 light; low nibble+1 = length of the first variable section when another
section follows. `f & 0x80 && len < 15`: control packet (`e2 <ver>` = version transition).
Metadata byte: 0x40 event(+1), 0x80 battery(+1), 0x20 optical sub-protocol(+1),
0x10 controller sub-protocol(+1). Sub-protocols start unknown (0xff → packets rejected).
- Input: controller sub-protocol 0 → legacy input event; 1 → raw bytes kept; 2 → v2 input.
- Light: optical sub-protocol 0 → legacy lightcap (LH1 pulses); 1 → LH2 tagged words like
  wired report 0x28.

## Known behaviour (identical in the original)
If a device delivers a few sync pulses, then pauses (e.g. the ~3 s protocol handshake at
connect), the TDM disambiguator creates bases from stale clusters and never matches a sync;
`tdm sync lost` cannot fire because it requires a previous match. Diff tests start after the
handshake gap. To revisit when porting the source restart logic.
