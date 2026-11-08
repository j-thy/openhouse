# openhouse: the custom Lighthouse driver, end to end

The "how does the whole thing work" document. The deeper references are:
- `ARCHITECTURE.md`: the reverse-engineered stock driver.
- `lighthouse_direct.md`, `state_vector.md`, `measurement_models.md`: the filter.
- `disambiguator_tdm.md`: the stock 2-base sync decoder.
- `lh_runtime.md` (`lh::Session`), `runtime.md` (the lower-level `lh::Tracker`): the runtime.
- `openvr_driver.md`: what the host sees.
- `tracking_improvements.md`: every non-stock change and its numbers.
- `windows_build.md`: the Windows build.

## 1. What it is

A reimplementation of SteamVR's `driver_lighthouse` for **Lighthouse 1.0 and 2.0**
tracking of headsets, Index controllers and trackers. It's built as:
- **`liblh`**, a C++ library. The tracking core is a verified port of Valve's filter,
  and the runtime improves on it.
- **`driver_lighthouse.so` / `.dll`**, an OpenVR driver on top of `liblh`. It's a
  drop-in for SteamVR and for Monado's `steamvr_lh` (WiVRn, xrizer, VRChat).
- **Tools:** `openhouse-viewer` (live status of the running driver), `lhtool`, `lhviz`,
  `lhcapture`, `openhouse-check`.

Headsets are tracked by openhouse; their display comes from SteamVR's stock driver,
loaded at runtime for the headset only (`stock_hmd.cpp`). Base stations are exposed as
tracking references. Lighthouse 2.0: `lh2_support.md`. The human-facing description of
all of this is the root `README.md`.

```
 USB HID (Watchman dongles / wired)                     src/transport/usb
   └─ IMU samples + light pulses + config + OOTX bits + inputs
        ▼  per device
 TrackingSource                                          src/tracking/tracking_source
   ├─ TDM disambiguator (stock, 2 bases)                 disambiguator_tdm
   ├─ multi-station disambiguator (lhbs, 2-4 bases)      disambiguator_multi
   └─ OOTX decoder -> base calibration, gravity, channel ootx
        ▼  optical frames (one base, one axis, many sensors) + IMU
 DeviceTracker (one per device)                          src/runtime/device_tracker
   ├─ LighthouseDirect: the stock EKF, 49 states         src/tracking/lighthouse_direct, direct_filter
   ├─ bootstrap pose solver (multi-start)                pose_solver
   ├─ room sync, health checks, reacquisition
   └─ output stage: prediction, smoothing, coasting
        ▼
 Session (shared by all devices)                         src/runtime/session
   ├─ RoomMap: every base's pose, persisted              room_map
   ├─ RoomSolver: joint solve from keyframes             room_solver
   └─ Snapshot / Updates API                             include/lh/session.hpp
        ▼
 OpenVR driver                                           src/openvr/driver_lighthouse
   ├─ HostFrame: the stable frame we report in           host_frame
   ├─ RoomPublisher: the room for alignment tools (shm)      room_publisher
   ├─ Index input + hand skeleton                        hand_skeleton, runtime/index_input
   └─ haptics routing
```

## 2. Transport (`src/transport/usb`)

hidapi reads the dongles and wired devices; reports are decoded as in stock:
- IMU samples at about 250 Hz for a controller and 1 kHz for the HMD;
- light pulses (sensor, timestamp, width);
- the device's JSON config (sensor positions and normals, IMU calibration, model);
- Index inputs and haptic output.

The device clock ticks are unwrapped onto one host timeline (`tick_unwrapper`,
`clock_sync`). `lhcapture` records this raw stream (`.lhraw`), and every tool can
replay it.

## 3. From pulses to frames (`src/tracking`)

- **Sensor type:** the pulse format per device, inferred as stock does
  (`sensor_type.md`).
- **TDM disambiguator:** the stock path. It classifies pulses into sync flashes (by
  width: skip/data/axis bits) and sweep hits, works out which base flashed (A/B), and
  emits an **optical frame**: one base, one axis, the sensors hit and their angles.
  It handles at most 2 bases.
- **Multi disambiguator:** for lhbs multi-station rooms (detected from the flash
  layout, or the OOTX mode byte).
  - It knows the 250 µs-staggered flash groups, decodes every slot's OOTX, and learns
    the round from the master's frame.
  - It attributes each revolution's sweep to the base whose turn it is.
  - While it's active, the TDM path's frames and logs are dropped.
- **OOTX:** per base, the decoded factory calibration (phase, tilt, curve, gibbous
  per axis), the gravity direction, the rotor-lock count, the faults and the channel.
  lhbs multi frames also carry the slot and round mask. Frames recalled from an earlier
  session are flagged `cached` (their gravity is old).

## 4. One device's tracking (`DeviceTracker`, `LighthouseDirect`)

**The filter** is a port of Valve's `CLighthouseDirect`, checked bit for bit against
the original binary by the oracle tests. It's an EKF with a 49-element state:
- the device pose (position, velocity, orientation, angular velocity);
- the primary base's tilt and gravity;
- the IMU calibration: gyro and accel biases, accel scales, IMU-to-body misalignment;
- up to 3 other bases' poses relative to the primary.

IMU samples predict and correct it; optical frames correct it through the LH1 angle
model (`lighthouse_model.hpp`).

**Life cycle:**
1. **Bootstrap:** the pose solver finds a first pose from one well-lit frame. The base
   must be located in the room, or located soon. Multi-start acquisition is non-stock.
2. **Calibrated:** after enough updates the IMU calibration has settled. Other bases
   join the filter from the room map.
3. **Tracking:** health checks reset it on nonsense (out of bounds, misalignment,
   spinning faster than possible). It reacquires with the calibration carried over.

**Non-stock improvements:** `LH_STOCK_FILTER` turns them all off (the per-change
switches were removed for the release). Details and numbers are in `tracking_improvements.md`.

| Improvement | What it does |
|---|---|
| Output prediction and smoothing | No visible steps when light comes back. While dark: coasting, with a slow fly-away guard |
| Waiting for late frames | Frames arrive 20–25 ms late; the filter waits up to 120 ms instead of dropping them |
| Room sync | Other bases' poses come from the room solve when the filter is clearly off. Pairs with the origin are synced only once the room is sure of them |
| Rejection-streak handling | A base whose light keeps being rejected gets the room's relationship, or the device reacquires on it |
| Knocked bases | While a base relearns, the device loosens its prior on it. An oversized correction then moves only that base, within the per-update limits, for 10 s |
| Violent motion | On saturated or near-limit IMU samples, the slow states are held (base tilt and gravity, IMU calibration, other bases), so the light corrects only the motion. A saturated axis counts as an "at least this much" bound. Frames of fewer than 3 sensor rows are rejected meanwhile |
| Primary switching (multi rooms) | A primary base dark for 0.5 s while another is lit: start over on the lit one |

## 5. The room (`RoomMap`, `RoomSolver`, `Session`)

- **What it holds:** every base's pose in one world frame. The **origin** base defines
  that frame; its tilt comes from gravity. It's saved to `~/.config/lh/roommap.json`
  (`%APPDATA%\lh` on Windows). It can be imported from SteamVR's `lighthousedb.json`,
  by default only under SteamVR with "Use SteamVR's rooms" on (`LH_STEAM_DB=auto` elsewhere).
- **Keyframes:** devices contribute moments where they saw two or more bases well. The
  room solver solves all bases jointly against the origin from the newest keyframes,
  about once a second.
- **Fusion:** each solve is weighed by its honest uncertainty (one keyframe counts as
  one measurement at the fit's noise), so the room settles instead of wandering.
  Refinements glide in at up to 2 cm/s and 0.5°/s, so devices never see a step.
- **Moves:** a solve far off (max(5 cm, 5σ) or max(0.5°, 5σ)) three times running
  means the base moved: it's re-placed at once, and its **epoch** changes, so every
  device takes the new relationship.
- **Knocks:** lhbs bases refresh their accelerometer in OOTX. A turn of more than 1.2°,
  seen twice, relearns the base. A knocked origin is re-solved against the best other
  base, and the host frame follows it (see below).
- **Locks:** a base with 100+ keyframes of evidence (this session or saved) is locked:
  no solve, observation or wake-up moves it; only its own accelerometer (a knock, or a
  gravity that differs from the saved one) unlocks it.
- **Level:** the origin's tilt is refined from the devices' gravity. An origin turned
  more than 8 deg since the room was saved makes the stored room stale: a new room is
  learned (the old one is kept, unused).

## 6. The OpenVR driver (`src/openvr`)

- **Devices:**
  - Index controllers become Controllers, with inputs, a 31-bone skeleton at up to
    200 Hz and haptics.
  - Everything else (not the HMD) becomes a GenericTracker.
  - Bases become TrackingReferences. Their mode label is the channel: `A`/`b`/`c`,
    or `1`–`4` for lhbs slots.
  - Roles: `~/.config/lh/settings.json` `controllers.left/right` pins serials,
    otherwise the device config decides.
- **Disconnects:** a disconnected device disappears. A connected device that loses
  light holds its last pose; it doesn't go to the origin.
- **Poses:** sent in the **host frame**: `qWorldFromDriverRotation` plus translation is
  rawFromWorld.
  - **HostFrame** (default): our own room, pinned so that the host frame stays the
    same across sessions and room refinements. The origin base keeps the **position and
    heading** it had when first anchored (`hostframe.json` beside the room map). The
    frame is always level: its tilt is the room's own.
    - Pinning the full pose instead kept an old level forever: a remounted origin left
      the frame 53° off level and broke alignment tools reading it.
    - A knocked origin that the room re-placed moves the anchor with it.
  - `LH_HOST_FRAME=steamvr`: align to SteamVR's universe through a shared base
    (`RawAlignment`), as the original did.
  - SteamVR's rooms (`steam_rooms.hpp`, #82; SteamVR only, `useSteamVRRooms`, on by
    default): a room imported from SteamVR keeps its universe and frame; one of ours that
    shares a base with a SteamVR universe with Room Setup (`chaperone_info.vrchap`) takes
    that universe and frame (decided in the room's first 5 s, kept for the session);
    otherwise it is its own universe in the pinned frame above, and is written into
    `lighthousedb.json` (`"openhouse": true`, never imported back) so Room Setup has a
    universe to save to. Without it SteamVR asked for Room Setup on every start.
- **RoomPublisher:** shared memory `/openhouse-room-<uid>` (Linux) or `Local\openhouse-room` (Windows; layout in
  `include/lh/room_shm.h`). It carries every base's pose in the host frame, with
  σ and an origin flag, a *settled* flag (no base moving, all σ < 5 mm) and an *epoch*
  that changes when the settled room differs. Alignment tools can use it instead of the
  runtime's base station devices.
- **Hand skeleton:** ported from the stock hand logic. Monado reads only the
  "without controller" range. The thumb rises at once when nothing is touched and
  glides between buttons; the skeleton is sent periodically, not only with input
  reports, so Monado doesn't extrapolate a stale hand.

## 7. Files, switches, logs

| File | What |
|---|---|
| `~/.config/lh/roommap.json` | Rooms and base poses |
| `~/.config/lh/hostframe.json` | Host frame anchors per room. Delete to re-anchor |
| `~/.config/lh/settings.json` | `roles`, `controllers`, `disabledDevices`, `mixedLight` (SteamVR's page wins under vrserver) |
| `~/.config/lh/viewer.json` | openhouse-viewer settings |

Every environment variable is listed in the root `README.md`. For development: `LH_STOCK_FILTER`
(stock tracking), `LH_REPLAY`, `LH_EVALUATE_ROOM`, `LH_CAPTURE`.

Logs: every event has a level (`include/lh/log_level.hpp`; `ClassifyLog` when the
emitter doesn't say). The host log gets INFO+ (`LH_LOG_LEVEL` lowers it); the viewer gets
everything over the telemetry socket (`include/lh/telemetry.hpp`, 127.0.0.1:36840).
Diagnostics are DEBUG messages, not stderr prints.

## 8. Tools

| Tool | What |
|---|---|
| `lhtool session <capture.lhraw> --room R.json [--trace T.jsonl]` | Replay a capture through the whole runtime: events, poses, a 100 Hz trace |
| `tools/eval_replay.sh <capture> OUT 50 8` | Score a change: valid %, coasting, spikes, jitter, per device |
| `openhouse-viewer` | Live status of the running driver: devices, bases, room, 3D view, log by level; Report > Save bug report |
| `openhouse-compare` | openhouse and SteamVR's own driver (loaded in-process, Monado-style host) tracking the same device side by side, with per-side stats |
| `apps/example_app` | Minimal standalone CMake project using `lh::Session` (user-facing integration example) |
| `lhviz` | Developer 3D view running its own tracking (SteamVR closed) |
| `lhcapture [dir]`, `lhcapture --check` | Record raw data; check the sync-flash layout live |

**Evaluation captures** (in `~/lh-captures`):

| Capture | What's in it |
|---|---|
| vrchat-01 | Normal play, 2 bases, 5 devices: the main regression set |
| ms4-01 | 4 bases switched on and off |
| testswing | Violent swings |
| session-* | sessions recorded with a Quest headset under WiVRn |
| fastmotionindex, fastmotion3-5 | Fast swings (the Beat Saber stand-in) |
| `tests/data/captures/*` | Short test captures in the repo (used by ctest) |

## 9. Build, install, test

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build -j16
ctest --test-dir build -j8          # unit, oracle (bit-exact vs stock), runtime, openvr
scripts/build-linux.sh              # release build -> dist/openhouse-driver-linux.tar.gz
dist/openhouse-driver/install.sh    # stops nothing: close SteamVR / Monado / WiVRn first
scripts/build-windows.sh            # MinGW cross build -> dist/openhouse-windows-x64.zip
scripts/build-firmware.sh           # lhbs + installers -> dist/openhouse-basestation-firmware.zip
```

Windows installer and build details: `windows_build.md`.
