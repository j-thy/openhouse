# Agent handoff: start here

This folder is the working knowledge for AI agents continuing this project. Human-facing
documentation is the root `README.md`, written by the owner (in their voice: casual,
short). Don't add other Markdown outside `ai/`.

## The project

* **openhouse / liblighthouse** (repository root): a reverse-engineered, improvable C++20
  reimplementation of Valve's Lighthouse 1.0 and 2.0 tracking (`driver_lighthouse`). It ships as a
  drop-in OpenVR driver (`driver_lighthouse.so` / `.dll`) for SteamVR and for Monado's
  `steamvr_lh`, plus tools (capture, replay, viewer). The owner runs it under Monado +
  WiVRn with a Quest headset (a separate tool aligns the Quest with the Lighthouse room),
  in VRChat and Beat Saber, on Linux and Windows SteamVR.
* **lhbs** (`basestation_firmware/`): a readable reimplementation of the HTC Lighthouse 1.0
  base station firmware (NXP LPC11U37), with an optical multi-station mode for more than
  two bases (TDM time slots).

Goals and constraints (from the owner):
* Tracking algorithms and constants start from the original (oracle-tested), but use
  normal libraries (Ceres, Eigen, hidapi, nlohmann-json). Readable code with documented
  provenance (stock addresses, constants), not bit-for-bit transliteration.
* Long-term aim: **better than stock SteamVR tracking**. Improvements live outside the
  stock filter core (`DeviceTracker`, `SetImprovements`, env switches for A/B), so the
  oracle tests keep checking the stock behaviour. `LH_STOCK_FILTER` turns the
  improvements off for comparisons; add per-change switches locally while testing and
  remove them before the change lands (the release has none).
* USB transport stays separate from tracking (`lh::IInputSink`); tracking must not
  depend on hidapi.
* Device roles come from each device's JSON config (`device_class`), never from the
  hardware: the owner maps some Index controllers as generic trackers. Vive wands are
  controllers (`ControllerType::ViveWand`, libsurvive button map; untested on hardware).
* Headsets: tracked by us; their display comes from SteamVR's stock driver loaded at
  runtime for the headset only (`src/openvr/stock_hmd.cpp`). Never ship Valve's files.
* Lighthouse 1.0 and 2.0 both work (`driver/lh2_support.md`). Making LH1 bases emit
  LH2-style light was researched and set aside: `driver/lh1_as_lh2_research.md`.
* Release hygiene: no Valve/HTC text or data in the repo (stock log wording, firmware
  descriptions), no stderr debug prints (use DEBUG-level log messages), the name is
  openhouse everywhere.
* **Commit only when the owner asks.** Ask for hardware captures when a real-device test
  is needed; don't guess hardware behaviour.

## Where things are

| Path | What |
|---|---|
| `src/transport/usb` | HID transport, report/wireless decoding, raw capture (`.lhraw`) |
| `src/tracking` | sync/sweep disambiguation (stock TDM, `disambiguator_multi` for lhbs multi), pose solver, the stock EKF (`direct_filter`, `lighthouse_direct`) |
| `src/runtime` | `Session`, `DeviceTracker` (improvements), room map and room solver |
| `src/openvr` | the OpenVR driver (devices, inputs, skeleton, host frame, room publisher, stock HMD, pairing, telemetry server) |
| `src/common` | small shared pieces (loopback sockets) |
| `src/platform` | OS helpers (paths, precise sleep) |
| `apps/` | `openhouse-viewer` (live status of the running driver, bug report button), `openhouse-compare` (openhouse vs the stock driver side by side; hosts the stock DLL in-process), `lhtool` (replay/session/track), `lhcapture` (recorder), `lhviz` (developer viewer), `openhouse-check` (update check), `example_app` (standalone CMake example for users of `lh::Session`) |
| `tests/` | unit, runtime, OpenVR component/mock-host, oracle tests (load the original binary) |
| `tools/` | `eval_replay.sh/.py` (tracking scorer) |
| `scripts/` | Release builds (`build-linux.sh`, `build-windows.sh`, `build-firmware.sh`) and the packagers they call (`package-linux.sh`, `package-driver.py`, `package-windows.py`, `package-firmware.py`), the Windows installer script |
| `basestation_firmware` | the firmware (`src/`, `test/`, `Makefile`); `tools/lpcsum.py` (boot checksum, run by the Makefile); `installer/` the end-user installers (Linux sh, Windows ps1/bat). `fw/`, `patches/`, `re/`, `bsfirmwares/`, `backups/` are local only |
| `re/ghidra_scripts` | headless Ghidra scripts. The rest of `re/` (project, decompiles) is local only |

Not in git, on purpose: Valve's binaries, decompiled code, HTC firmware images and flash
dumps, `libsurvive/` (reference only: LH2 polynomials and decode), recordings.

## Build, test, install

```sh
cmake -S . -B build-linux -G Ninja && cmake --build build-linux -j$(nproc)
ctest --test-dir build-linux -j4 --output-on-failure      # 9 tests; oracle tests skip without fixtures
scripts/build-linux.sh && dist/openhouse-driver/install.sh   # close SteamVR / Monado / WiVRn first
```

Release packages (each builds and packages; nothing is installed):
* `scripts/build-linux.sh`: static vcpkg Ceres (no SuiteSparse: its SPQR/CHOLMOD are GPL and
  `package-driver.py` refuses to bundle them) into `build-release`, then `package-linux.sh`.
  `package-linux.sh` alone packages whatever `build-release` holds; it doesn't build.
* `scripts/build-windows.sh`: MinGW cross-build (`driver/windows_build.md`).
* `scripts/build-firmware.sh`: lhbs + host test + installers.

Packaging deletes `dist/openhouse-driver/` (and the Windows/firmware dirs): move anything
the owner keeps there first. The version is `include/lh/version.hpp`, kept in sync with
`project(... VERSION)` and `vcpkg.json`.

## Evaluating tracking changes

Measure, don't reason. Replay captures with `tools/eval_replay.sh CAPTURE OUT 50 8 --room
COPY_OF_ROOMMAP` (parallel windows, prints valid/coast/lost %, jumps, jitter). **Always replay
against a copy of the room map** (`~/.config/lh/roommap.json`, Windows
`%APPDATA%\lh\roommap.json`); a replay can rewrite the map it is given. The script calls
`build-linux/lhtool` (the one Linux build directory).

Captures (owner's machine; ask for them if absent):
* `~/lh-captures/vrchat-01`: 10 min VRChat, 5 devices: the main regression capture.
* `~/lh-captures/testswing`: 24 s of violent swings. `~/lh-captures/ms4-01`: bases on/off.
* `~/lh-captures/fastmotionindex`, `fastmotion3`-`5`: fast swings, the stand-in for Beat
  Saber (the original Beat Saber captures are gone). Fly-aways reproduce only from some
  starting points: replay with several `--from` values and count "out of bounds" resets.
  Fast motion is the owner's top priority: a change must keep these clean.

Logs and diagnostics: every message has a level (DEBUG/INFO/WARN/ERR). Replays print
them with `lhtool session` (`"level"` field); live, the host log gets INFO+ unless
`LH_LOG_LEVEL=DEBUG`, and `openhouse-viewer` shows everything. Recording inside the host:
`LH_CAPTURE=DIR` or `capture` under `driver_lighthouse` in `steamvr.vrsettings`; the
recording has `driver.jsonl` (what the driver did and logged) next to `input.lhraw`.

## The owner's setup

Four LH1 bases on lhbs multi-station: slots 1-4 = E236DC40 (master), A101B835, D7B1AD76,
89AE90F2 (older 15-diode unit). 89AE90F2's calibration was once wiped by HTC's
"calibration-rescue-244" firmware and restored from a flash backup: **never suggest that
firmware**. Devices: Index controllers (LHR-13DA81D1 right, LHR-8F9BE6F2 left), trackers,
a Vive headset (LHR-0704E07A). Quest Pro through WiVRn for VRChat.
Alignment tools can read the room from `/openhouse-room-<uid>` on Linux or `Local\openhouse-room` on Windows (`include/lh/room_shm.h`;
copies of that header in other projects must stay identical).

## State of the work (2026-11-08)

Released as openhouse 1.0. Recent work, all in the tree:
* Rooms: one "learned" rule (`BaseStationState::learned`: origin, <= 5 mm, or locked / 40+
  keyframes <= 20 mm) drives settling, the SteamVR base status (searching until locked),
  telemetry and the viewer. `Session::ResetRoom` forgets every room and rebuilds the
  trackers; deleting `roommap.json` while running does the same.
* SteamVR settings page (`packaging/settingsschema.vrsettings`, section `driver_openhouse`):
  smoothing, acceleration prediction, Record Session (turning it off saves the bug report
  zip, `lh/bug_report.hpp`), Reset room. Only toggles/sliders/selects exist there; one-shot
  actions are toggles the driver turns back off.
* Base stations show the 2.0 or 1.0 model and icons from how they're tracked.
* Preinit devices (`~/.config/lh/preinit`) are reported connected-but-uninitialized until
  they first connect (Monado only takes devices at startup).
* The pairing mailbox logs every request and reply (`pairing:` lines in vrserver.txt).
* lhbs sync cable (A/b/c) as stock: A, b and c drive the jack, A locks to it first and
  falls back to optical without one, multi leaves the jack undriven. A cable slave keeps
  P1_0 low. `ai/firmware/FIRMWARE_OVERVIEW.md` §5, `HARDWARE.md` log 2026-11-08.

Open:
* A friend's Windows PC with no headset shows the pairing page stuck on "Initializing"
  (no reply reached it). It works on the owner's Windows install; their `pairing:` log
  lines are needed.
* A base seen well by only one device never locks and gets relearned from poor views.
* Fly-aways right after a reacquire or a base newly used from the room map.
* `openhouse-compare`: the stock side declines headsets (their Activate needs SteamVR
  internals the host lacks); both drivers share the dongles, so either side's glitches
  can be the other's commands.
* lhbs `b`/`c` channels once came up sweeping in the same phase (both drivers saw it);
  a power cycle of the `c` base fixed it. Not reproduced since.

## Hard-won lessons

* **Windows SteamVR ignores boolean inputs sent through `IVRDriverInput_004`** (accepted,
  never applied: no buttons, no dashboard). Keep component calls on `ComponentInput()`,
  which asks for `_005` like the stock DLL. When an OpenVR call behaves differently on
  Windows, check which interface version the stock DLL requests before suspecting the
  compiler (it wasn't MinGW).
* Windows `sleep_for` rounds up to the 15.6 ms timer tick; use `platform::SleepFor`.
* vcpkg's Ceres has no SuiteSparse: avoid `ceres::Covariance` (its sparse QR fallback was
  87 % of the CPU).
* Monado's `steamvr_lh` doesn't type-check input components, so Windows-only bugs hide
  on Linux. Check Monado's `steamvr_lh` source for what it implements before relying on
  an OpenVR host feature.
* Replays from different starting points follow different trajectories (and GCC vs MSVC
  differ too): a single replay proving "clean" means little.

## Index of the documents

`driver/` (the driver and library):
* Overview: `SYSTEM_OVERVIEW.md` (end to end), `ARCHITECTURE.md` (the original's design),
  `usage.md` (build/run/API), `PROGRESS.md` (port history: what came from which address).
* Transport: `usb_protocol.md`, `wireless_protocol.md`, `capture.md`, `source_layer.md`.
* Tracking: `disambiguator_tdm.md`, `multistation_plan.md`, `pose_solver.md`,
  `lighthouse_direct.md`, `state_vector.md`, `measurement_models.md`, `sensor_type.md`,
  `constants.md`, `tracking_improvements.md` (every change beyond stock, with numbers).
* Runtime and host: `lh_runtime.md`, `runtime.md`, `interfaces.md`, `openvr_driver.md`,
  `windows_build.md`.
* LH2: `lh2_support.md` (pipeline, results, open items). Research: `lh1_as_lh2_research.md`.

`firmware/` (lhbs): `usage.md` (build, flash, console; end users: `basestation_firmware/installer`), `FIRMWARE_OVERVIEW.md`,
`HARDWARE.md` (pin map, peripherals, verification log), `MULTISTATION.md` (TDM slots,
open issues).

`reverse_engineering.md`: the Ghidra workspace for `driver_lighthouse.so` (local only).
