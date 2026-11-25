# Drop-in `driver_lighthouse.so` (SteamVR / Monado steamvr_lh)

`build/openvr/lighthouse/bin/linux64/driver_lighthouse.so` is an OpenVR driver built
on liblh. It replaces SteamVR's own `drivers/lighthouse/bin/linux64/driver_lighthouse.so`
for **tracked devices**: Index controllers (with input), trackers and base stations.
Head-mounted displays are deliberately not exposed (no display / direct mode).

## Controller serial assignments

Optional `$XDG_CONFIG_HOME/lh/settings.json` (default `~/.config/lh/settings.json`):

```json
{
  "controllers": {
    "left": "LHR-YOUR-LEFT-SERIAL",
    "right": "LHR-YOUR-RIGHT-SERIAL"
  }
}
```

Restart the host after editing. When `controllers` is present, only those serials
are exposed as controllers, with the specified hand. All other non-HMD devices
are trackers (including an Index controller used on your hip). Either hand may
be omitted; an empty object makes all controllers trackers. Remove `controllers`
to restore automatic device-config roles. Other settings in this file are preserved.

```
cmake --build build -j16
scripts/build-linux.sh                # release build + package (dist/openhouse-driver/)
dist/openhouse-driver/install.sh      # backs up the stock driver once, installs ours
dist/openhouse-driver/uninstall.sh    # restores the stock driver
```

Stop SteamVR / monado-service before installing. Monado's `steamvr_lh` loads the same
file from the SteamVR folder, so one install serves both.

**Do not copy only the unbundled .so.** SteamVR runs in a container which may not
see host Ceres/SuiteSparse libraries (`libceres.so.4` load error). The installer
bundles the transitive non-platform dependencies into `bin/linux64/lh-libs`, with
origin-relative RPATHs on the driver and libraries. `driver_package` produces the
same layout in `build/driver-package`; copy its **entire contents** for manual
installation. Packaging needs Python 3, `patchelf`, `ldd`, and `strip`. License
notices available from the host distro and a dependency manifest accompany it.
The bundle targets this machine's runtime ABI; it is not a universal Linux binary.
Do not bundle glibc or the dynamic loader.

## What a host sees

| device | class | properties |
|---|---|---|
| Index controller (`device_class: controller`, Knuckles model) | Controller | `knuckles`, `{indexcontroller}/input/index_controller_profile.json`, render model from the config, role hint left/right from `tracked_controller_role`, battery |
| everything else except HMDs | GenericTracker | `vive_tracker`; HTC's (`htc` resource root, `{htc}/input/vive_tracker_profile.json`, `htc/vive_tracker<serial>`, HTC icons) for an HTC tracker, the config's own `resource_root`, `input_profile`, `registered_device_type` and that root's icons for any other maker's (Tundra Labs), as stock `0x1cf860`; render model from the config (`vr_tracker_vive_1_0` on a "VIVE Tracker 3.0…" is `{htc}vr_tracker_vive_3_0`, stock `0x1d69f0`); role OptOut, battery, `ConnectedWirelessDongle` = its receiver's serial (OyasumiVR powers off through it); inputs: HTC's get `/input/power`, `grip`, `application_menu`, `trigger` (click, value), `thumb` and `/output/haptic` from the wand decoder (stock `0x3f78a0` case 3), other makers' none |
| base stations | TrackingReference | `lh_basestation_vive`, mode label A/B/C, pose from the room map |

A Tundra Tracker with an identity `head` (all of them ship one) gets the stock driver's
head instead (`0x1dec60`): `plus_x` (−1,0,0), `plus_z` (0,0,−1), `position` (0,0,0.001577),
180° about Y; taken literally it was drawn bottom-up.

Windows exports (`src/openvr/driver_lighthouse.def`): `HmdDriverFactory` and our static
hidapi's `hid_*` functions themselves, for drivers that hook the lighthouse driver's HID
access (sboys3's Pimax config driver, #32). On Linux only `HmdDriverFactory` (`exports.map`).

All devices: `TrackingSystemName = lighthouse`, `CurrentUniverseId` = the SteamVR universe
id of the reference universe when a shared base allows alignment, else the liblh
room id. The reference transform is published as `qWorldFromDriverRotation` and
`vecWorldFromDriverTranslation`, separately from the filter's optimizable map.

Index inputs: `/input/{system,a,b}/{click,touch}`, `/input/trigger/{click,touch,value}`,
`/input/grip/{touch,value,force}`, `/input/thumbstick/{click,touch,x,y}`,
`/input/trackpad/{touch,x,y,force}`, `/input/finger/{index,middle,ring,pinky}`,
`/output/haptic` (routed to the receiver). `grip/value` is the larger of the mean
middle/ring/pinky curl and the grip force; `grip/touch` is curl > 0.7.

Poses (`DriverPose_t`): world = SteamVR raw tracking space of the imported universe
(checked: the controller on the floor lands 2 cm above the floor through SteamVR's
standing transform), `qDriverFromHead` = config head frame relative to the IMU,
angular velocity in the device (IMU) frame like the stock driver, `poseTimeOffset`
= sample time − now (steady clock), `result` = Running_OK / Running_OutOfRange /
Calibrating_InProgress / Uninitialized.

## Monado specifics (src/xrt/drivers/steamvr_lh)

* Devices are only accepted during discovery (`LH_DISCOVER_WAIT_MS`, default 3000 ms,
  extended 2 s per added device). The driver registers a device as soon as its config
  is read; live, the Index controller registered 0.52 s after `Init`. Devices turned
  on later are picked up by the transport (see below) but Monado refuses devices
  after discovery — turn controllers on before starting Monado, or raise
  `LH_DISCOVER_WAIT_MS`.
* Monado activates devices from inside `TrackedDeviceAdded`; the driver holds no locks
  across that call.
* Monado matches the input profile by file name (`index_controller`, `vive_tracker`)
  and declines base stations (they still feed tracking inside liblh).
* Monado's playspace comes from SteamVR's `lighthousedb.json` + `chaperone_info.vrchap`.
  `RawAlignment` aligns our room through a shared physical base and publishes the
  reference universe ID, including when our room was rebuilt from scratch. It
  follows origin re-levelling without changing the host's standing frame. The
  anchor must not have physically moved. No SteamVR DB or chaperone files are written.
  A completely new room with no shared fixed reference still needs room setup:
  gravity alone cannot determine floor height or the desired playspace origin.
* Index controllers supply a 31-bone skeleton in both motion ranges, reported as
  `VRSkeletalTracking_Partial` like the stock Index driver: Monado's steamvr_lh only
  turns a non-`Estimated` skeleton into (conforming) hand tracking, and only reads
  `WithoutController`. **Ported from the stock hand (2026-10-19,
  `CHandPoseLogic_IndexController`, vtable 0x470988; see hand_skeleton.cpp)**:
  * thumb, with the controller: a state from what it touches (A/B/both pressed,
    trackpad by position on a 3x3 pose grid, thumbstick by deflection on another,
    A+B touched, system), cross-faded over 0.2 s (0.9 s back to rest). Button ids as
    stock tests them: B = ApplicationMenu, A = Grip, trackpad = Axis0, stick = Axis3;
  * thumb, without: a 2D position (stick: outward, pad: by position, A/B: curled
    down) spring-smoothed, over `hand_right_poses.glb` frames 13/10/14 and the close
    animation; lifted, it stays where it was, as stock;
  * fingers: with the controller, open-to-grip by each capacitive sensor (the index
    onto frame 30 with the trigger); without, along SteamVR's
    `hand_right_closeanim.glb` fist (bias curves 0.5/0.25/0.3/0.3) blended with the
    Index squeeze clip by grip force; a pulled trigger closes the index fully.
  Assets (`vr_glove_right_poses.glb`, SteamVR `resources/anims/hand_right_*.glb`) are
  read at runtime, never redistributed; without the shared ones it falls back to
  the glove library's grip pose.
* **Left hand (2026-10-19).** OpenVR's left skeleton is not the right one with every
  bone reflected: finger bone frames are also turned half a turn about their x axis.
  From Valve's reference open hands: the wrist is a plain reflection across x
  (position x negated, quaternion (w, x, -y, -z)); bones parented to the wrist or the
  root (metacarpals, aux) take that reflection then `* (0, 1, 0, 0)`; bones inside a
  finger keep the right hand's rotation with the offset negated. Reflecting every
  bone the same way left each left finger bending backwards in Monado (whose
  conversion assumes the stock driver's frames). Tested in
  `Skeleton.ValidArticulationAndOpenVRLeftHand` (needs `LH_HAND_POSES`).
* This driver intentionally does not supply an HMD. Monado still needs a separate
  HMD/system builder capable of combining tracked devices with it. The standalone
  `steamvr` builder rejects a controller-only system with `Unable to find HMD`.

## Transport changes made for driver use

* Receivers with no wireless device connected are no longer probed for range modes
  and configs at startup (each timed out ~1.5 s); startup dropped from ~5.5 s to
  ~0.3 s with three idle dongles.
* A probe thread watches those receivers and attaches devices that connect later
  ("connected to receiver"), as the stock driver does. Its cost, measured 2026-11-21 on 3x
  `28de:2101` plus a Tundra SW7's 7x `28de:2102`: one `QueryDeviceInfo` per idle receiver per
  pass, ~150 ms each, and the stock keepalive (`ff a1 00`) to every idle receiver per pass —
  ~2.5 of them a second with those ten on the bus. `LH_LOG_LEVEL=DEBUG` logs every keepalive
  and every open's device-info query, which is how that was counted.
* **A Tundra SW7 presents seven receivers, not one:** `28de:2102`, USB serial `<serial>-<n>YX`
  for n 1-7, each answering the config read (0x10/0x11) in ~150 ms. `usb_protocol.md`'s
  "nothing" for `0x2102` describes *stream* reports only.
* **`back-facing hits` plus `no optical samples` is a room problem, not a driver one.** On the
  same machine Valve's stock `lighthouse 2.18.4` reproduced it identically (71k back-facing
  hits, a reset every ~3 s, SteamVR logging "Moving the base for tracking too"), with a base
  calibrated at 138-146 deg pitch. **Check the stock driver before attributing it to us.**
* **The SW7's heat is its own firmware, not the keepalive.** A unit stayed hot with the
  keepalive traffic at zero, so the heat is most likely the dongle holding seven radios in RX.
  Do not infer a temperature from a traffic count.
* **Receiver hotplug (2026-10-19).** The probe thread re-enumerates HID once a second
  and opens any receiver/HMD interface it doesn't have open. An unplugged interface
  (read error) is closed, detached and forgotten, so plugging it back in opens it
  again as a new endpoint; the tracked device keeps its serial, so the host keeps
  the same device and only sees it disconnect and reconnect. An idle receiver that
  disappears from enumeration is dropped. The transport also starts with nothing
  plugged in and waits (it still fails when interfaces exist but can't be opened,
  i.e. SteamVR holds them). Under Monado a device must still appear during its
  discovery window the first time; replugging a dongle later is fine.
* Radio report `0x26` and a >1 s gap in radio data now end the old device epoch.
  The report decoder resets protocol/subprotocol state, timestamp unwrapping,
  clock synchronization and IMU deduplication, emits detach/attach, and the session
  recreates the per-device tracker/input state even if the JSON is unchanged.
  The shared room and known base identities are retained. Version-capable devices
  are gated until a protocol transition arrives; negotiation retries even after a
  failed first HID write. This fixes the stale-state path that could interpret
  post-reboot bytes under the old protocol and flap battery/charging values.
  Synthetic raw and full-session power-cycle regression tests cover this path;
  physical reconnect with the new build still needs testing.
* After a power cycle the receiver may replay light pulses whose timestamps are
  behind the last optical time, so the stock 0x358300 check saw zero elapsed
  time and reset tracking. `CheckHealth` restarts acquisition after a run of 1000
  consecutive pulses timestamped >100 ms behind the filter.
  **Fixed 2026-10-19:** this used to fire when the newest pulse was older than the
  filter time, which the inertial advance (`AdvanceInertial`, 50 ms after the last
  light) makes true in *every* occlusion. Every hidden device was reset 0.5 s in,
  with its calibration discarded, instead of coasting for the stock 2 s and
  keeping it: the REACQUIRED/CALIBRATED churn. A device that drops out again before
  its new track has converged (< 200 updates) now keeps the calibration it was
  carried with, instead of starting from zero. Regression test
  `DeviceRecovery.OcclusionCoastsForTheStockTimeoutAndKeepsCalibration`.
* A device that loses tracking while switched on is held at its last valid pose
  (`TrackingResult_Running_OutOfRange`, zero velocities) until light finds it again.
  The lost record used to be an invalid identity pose, which Monado places at the
  play-space origin: hands "flew" in apps while lhviz looked fine.

## Testing without a host

`lh_openvr_host_test` (ctest `openvr_driver`) is a headless host modelled on Monado's:
dlopen + `HmdDriverFactory`, `Activate` inside `TrackedDeviceAdded`, a discovery
window, recorded properties/inputs/poses. It replays a capture through the driver
(`LH_REPLAY`):

```
build/lh_openvr_host_test build/openvr/lighthouse/bin/linux64/driver_lighthouse.so captures/<session>/input.lhraw 14
build/lh_openvr_host_test build/openvr/lighthouse/bin/linux64/driver_lighthouse.so live 20   # real devices
```

Environment: `LH_REPLAY=capture.lhraw` (driver replays instead of USB), `LH_LOG_LEVEL=DEBUG`
(the host log also gets DEBUG messages: coasting transitions, every haptic write), `LH_CAPTURE=<dir>` (record a
diagnostic session from inside the host, see `capture.md`). Disabled devices (lhviz checkbox,
`~/.config/lh/settings.json`) are not registered.

## Identify and hardware power-off

SteamVR's legacy `TriggerHapticPulse`/Identify requests and modern vibration actions
are both routed. Component handles are authoritative; a missing container handle
is allowed, and index-only legacy events are targeted by tracked-device index.
Unknown or explicitly mismatched targets are never broadcast. Feature reports are
padded to the receiver's full 64-byte size. **The previous libsurvive-style payload
was wrong for this path:** stock `0x3cb9a0`/`0x3cba30` sends `ff 8f 08 channel`, on
microseconds (u16), off microseconds (u16), repeat count (u16), then `ff`. We now
match that framing, on/off field order, and zero-count stop encoding. Previous
replay event-routing tests could not detect this hardware-packet error.
Actual vrserver replay testing delivered both legacy and action-based requests to
the driver; physical vibration with the corrected packet still needs retesting.

Controllers and trackers now advertise `Prop_DeviceCanPowerOff_Bool`. SteamVR's
per-device `EnterStandby` callback queues `Session::PowerOff` and the stock receiver
command `ff 9f 04 6f 66 66 21` (`off!`, padded to 64 bytes). This is hardware power-off,
not persistent disabling. HMDs and base stations do not advertise this capability.
Global provider standby is deliberately not wired to power off every device.
Each `EnterStandby` and power-off result is logged at INFO (idle power-off was reported
not to work; the path matches stock, cause unknown). Of the provider's standby calls
only `LeaveStandby` reaches the stock driver (`StockHmd::LeaveStandby`): it powers the
headset panels back on (#20, black display after standby); its `EnterStandby` would
power off controllers it doesn't own. The stock driver's context gets an
`IVRProperties` proxy that drops its writes of `Prop_CurrentUniverseId_Uint64` (#19:
its unused tracker switched SteamVR to its own universe mid-session). The `capture`
setting is read only under SteamVR (older Monado aborts on a missing key).
Power requests take priority over pending vibration for that receiver.
The packet is decompilation-backed (`0x3ce5a0`, `0x3ce6e0`); callback routing and
packet bytes are tested, but actual power-down/reconnect needs a live-device check.

The headset proxy's `GetPose` isn't forwarded either. SteamVR never calls it, but other
drivers do: Bigscreen's eye tracking driver reads the head's rotation from it once DFR is
on. It returns the last pose ours published for the headset with the head offset folded
in (`InHeadFrame`: `qRotation` is the head's rotation, `qDriverFromHeadRotation` the
identity), the stock driver's until there is one, and an invalid, disconnected pose once
`StockHmd::Cleanup` has closed it (before the stock provider's `Cleanup`). Bigscreen's
driver ignores the head offset, and a Beyond's IMU sits half a turn about Y from its head,
so with the IMU's rotation, which is what the stock driver hands over, it saw nods
reversed and sent SteamVR a gaze tens of degrees off on fast ones. Under MinGW the host
is given a `DeviceForHost` (`device_abi.hpp`) for each of our devices and the headset's
proxy: `GetPose` returns a `DriverPose_t` by value, and MSVC and GCC pass its hidden
result pointer in different places, so an MSVC-built caller crashed vrserver.

Base stations remain listed and the room map remains cached when tracked devices
disappear, as requested. Newly observed bases continue through the existing room
learning pipeline; this change does not expire or remove them.

Windows build feasibility and missing cross-dependencies: `windows_build.md`.

## Base station power

vrmonitor wakes and sleeps the base stations over Bluetooth, through the serial port of
a headset's built-in receiver, but first asks the headset's driver to switch that
receiver's radio, with `IVRDebug::DriverDebugRequest(0, ...)`: `set_radio_mode_ble` /
`set_radio_mode_valve` answer `1` or `0`, `get_radio_mode` answers `B`, `V` or `0`. A
failed first switch makes vrmonitor give up for the whole session, so the stations
neither wake when SteamVR starts nor sleep when it closes.

In the hybrid setup these requests reach the stock driver through the headset's proxy
(`stock_hmd.hpp`), and it can't serve them: it never sees the receivers. So
`StockHmd::SetRequestHandler` lets ours answer first (`Provider::RadioRequest`, as stock
`0x1eaa9` in the Windows DLL, 2.19.2); anything else still goes to the stock driver. The
commands run on the thread that owns the receiver, like pairing
(`UsbTransport::TakeRadioCommands`). Reports, and which receiver: `usb/radio_mode.hpp`.
Checked on a Bigscreen Beyond's receivers.

## Tracking quality

What the runtime adds over the stock filter (acquisition on few sensors and through
reflections, prediction to now, correction smoothing, damped coasting, stationary
hold, gradual room corrections), with measurements: `tracking_improvements.md`.

## Haptics

`RunFrame` queues haptic events; the provider matches the component handle (or,
without one, container + device index) and calls `Session::Haptic`.
**Monado (fixed 2026-10-19):** steamvr_lh's `ControllerDevice::set_output` sends the
device's own index as `containerHandle` (its real container, from
`TrackedDeviceToPropertyContainer`, is one more) and one less as
`trackedDeviceIndex`. Requiring the container to match rejected every in-app
vibration as "unrouted"; SteamVR's Identify worked because SteamVR gets it right. Only enabled Index controllers in live sessions are
accepted. The HID reader sends Watchman `0xff/0x8f` reports (high/low pulse durations
and repeat count), using the stock driver's framing (see above). Commands are
coalesced per device, expire after 100 ms if unsent, and last at most 2 seconds.
Nonfinite values are rejected, and frequency/amplitude are bounded. Zero frequency
selects 150 Hz. HID writes do not run inside host callbacks. Replay never vibrates.
Report encoding and host-event delivery are tested; physical vibration and stop
behaviour have **not** been confirmed on a powered controller in this session.

## Integration results (2026-10-17)

* All 9 CTest suites pass. Component tests cover pulse encoding/bounds, skeleton
  finite/unit transforms and mirrored geometry, and reference alignment after a
  translated room rebuild. Asset tests require `LH_HAND_POSES`.
* Packaged module passed the mock host **inside SteamLinuxRuntime_sniper**:
  1 skeleton, 2172 skeleton updates, no invalid bone transforms, haptic event polled.
* **Actual vrserver 2.17.10 in Sniper:** loaded this driver without the Ceres error.
  `lh_openvr_client_probe` received valid replay controller poses and an active
  skeletal action, read 31 bones, and successfully submitted a haptic action and
  legacy Identify pulse (both delivered to the driver).
  Driver log confirmed receipt (correctly ignored in replay).
* **Actual Monado v25.1.0-849:** loaded the installed module, discovered the replay
  Index controller, and initialized chaperone universe 1770781428. System creation
  then stopped because no HMD was supplied. Full OpenXR hand rendering and live
  haptics under Monado remain unverified.
* Live USB probe found no controller during discovery. No physical haptic result
  is claimed.

Manual actual-host probe (start vrserver separately, optionally with `LH_REPLAY`):

```
build/lh_openvr_client_probe "$PWD/tests/openvr/probe_actions.json"
```

Additional environment: `LH_HAND_POSES` overrides the Valve GLB path;
`LH_ROOM_MAP` chooses our map; SteamVR's rooms are not imported unless
`LH_STEAM_DB=auto` (or a lighthousedb.json path). The host frame is our room's;
`LH_HOST_FRAME=steamvr` pins it to lighthousedb.json's anchor instead
(`LH_REFERENCE_DB` overrides that file). `STEAMVR_PATH` also
helps locate assets for nonstandard Steam installs. Haptic routing is logged at DEBUG level. Only `HmdDriverFactory` is exported.
