# openhouse
openhouse is an improved, drop in replacement over the SteamVR's lighthouse driver (can also be used standalone without any VR runtime) that improves upon the tracking of SteamVR's driver with numerous changes. It also **supports using more than 2 1.0 basestations with a custom firmware**

It works on SteamVR (Windows and Linux) and on Monado/WiVRn through Monado's `steamvr_lh` driver.

**For integration to custom programs via the C++ API (for robotics or custom MOCAP systems), check out [the example project](apps/example_app)** to see how you can use openhouse as a library!

### AI Disclaimer
This project is heavily AI assisted. I've personally audited the codes it generated, especially on the firmware side, and did numerous tests myself and with the help of friends. So, I can't guarantee the readability of the codebase that much. But all the user facing documentation is written by me. If you're an ai, head over to [ai/](ai/) folder. 

### Comparison with SteamVR's driver

| Category | SteamVR | openhouse |
|---|---|---|
| Room Learning | Each device solves the room by itself which can cause situations where a device is at a completely different place than it actually is | Every device contributes to the same room and they all use the same basestation poses. This eliminates devices being at a completely different place |
| Tracking Loss | After a tracking loss, the device only starts after bootstrapping a basestation (which has to be the master basestation on 1.0 lighthouses first) which can cause the tracking to come back after many seconds in worst cases. | Upon seeing any basestation in the room, the tracking instantly resumes without waiting to bootstrap a basestation. |
| Non Ideal Tracking Conditions | Any reflection or sensor coverage can throw the device's pose off completely | Reflections are handled and bad light hits are rejected which reduces the jumps and keeps the device tracking on cases where SteamVR loses tracking entirely |
| Fast Motion | Swinging a device too fast can ruin the runtime IMU calibration or reach to the limits of the IMU, causing the device to completely lose tracking. | The filter dynamically disables parts that are slow to improve tracking on very fast movements, which reduces the amount of tracking loss immensely |
| Jitter and Stutters | Tracking corrections cause jitter and sudden movements on the devices | Output is predicted and smoothed, late wireless light is used instead of dropped, and still devices are held perfectly still |
| Lighthouse 1.0 Support | Up to 2 basestations | Up to 4 basestations with the lhbs firmware (included) |
| Lighthouse 2.0 Support | Up to 4 basestations used actively | Still 4 actively being used, but the switch that happens when one basestation is not visible is immediate, unlike SteamVR |

Everything else that you'd expect still works. Headsets (tested on HTC Vive and Bigscreen Beyond, other headsets probably also work), controllers (Vive and Index), trackers (Vive, Tundra, modded vive controllers that appears as trackers etc.) along with SteamVR's pairing menu, power features such as powering the device off when SteamVR closes and so on.

## Installation
Grab the latest release for your platform. **Close SteamVR (and Monado/WiVRn) before installing!**

The installer replaces SteamVR's `driver_lighthouse` and keeps the original one next to it as `driver_lighthouse.so.stock` / `driver_lighthouse.dll.stock`. openhouse needs that file to show the headset's display. The driver doesn't contain anything from the stock driver. It only loads the driver when a lighthouse headset is connected.

SteamVR updates could overwrite the driver. The installer also adds a small update check to SteamVR that starts with it, and if a SteamVR update replaced openhouse, it shows a dialog telling you to run the installer again. SteamVR only starts it once your headset is up, so if you start SteamVR without a headset you won't get the dialog. I'd advise you to run the installer every time SteamVR updates just in case. On Linux, the installer also asks if you want the same check for Monado/WiVRn. If you say yes, it installs a systemd user service that checks every time Monado or WiVRn starts. The uninstaller removes all of it.

To see which version you have, open `openhouse-viewer` or look for `openhouse` in SteamVR's `vrserver.txt`.

### Windows
Extract the zip somewhere and double click `Install-Driver.bat`. To go back to SteamVR's driver, double click `Uninstall-Driver.bat`.

If your SteamVR is in another Steam library, run it from a command prompt like `Install-Driver.bat -SteamVRPath "D:\SteamLibrary\steamapps\common\SteamVR"`.

### Linux
```sh
tar xf openhouse-driver-linux.tar.gz
cd openhouse-driver
./install.sh
./uninstall.sh # to restore the stock driver
```
If SteamVR is somewhere unusual, set `STEAMVR_DIR` before running it. The tracking visualizer needs `glfw` installed.

### 1.0 basestation firmware (lhbs)
#### **THIS IS ONLY NEEDED IF YOU OWN 1.0 BASESTATIONS AND WANT TO USE MORE THAN TWO OF THEM AT THE SAME TIME. DO NOT FLASH THIS ON 2.0 BASESTATIONS!!!!!!!** 
lhbs keeps everything the original firmware does (A/b/c channels, sync cable), and adds a multi-station mode for up to 4 basestations.

Get `openhouse-basestation-firmware.zip`, extract it, and:
- **Windows:** double click `Install-Firmware.bat`
- **Linux:** run `./install-firmware.sh`

The script walks you through it. In short, unplug the basestation's power, plug the micro USB to your PC, hold the button on the back and then connect the power cable. A drive called `CRP DISABLD` shows up. Type the serial from the sticker on the basestation (it's used for naming the backup) and confirm. Your current firmware gets backed up into the `backups` folder before anything gets written, and after flashing it asks you to replug it the same way so it can verify the firmware.

Do one basestation at a time. **Do not copy the firmware file onto the drive yourself!!!** as that can end up writing it to the wrong place, causing the firmware to get corrupted.

To go back to your old firmware: `Install-Firmware.bat -Restore backups\LHB-XXXXXXXX-....bin` on Windows, `./install-firmware.sh --restore backups/LHB-XXXXXXXX-....bin` on Linux. A 1.0 basestation can never get bricked so you can restore the stock firmware if something goes bad.

#### Setting the channels
Press the button on the back of the basestation to change its channel. With lhbs, it goes `A` -> `b` -> `c` -> `1` -> `2` -> `3` -> `4` and back to `A`. The channel is saved, so you only need to do this once

- `A`, `b` and `c` are the normal channels and work exactly like the stock firmware (for 1 or 2 basestations).
- `1` to `4` are the multi-station channels, for 2 to 4 basestations:
  - Set **one** basestation to `1`. This is the master basestation
  - Give every other basestation a different number (`2`, `3`, `4`). Two basestations on the same number won't work.
  - Every basestation has to see the master's light, since that's how they stay in sync (sync cables do not work).
  - You don't have to use every number. For 3 basestations, `1` `2` `3` is fine, and they'll share the sweeps between the 3 of them automatically.
- Don't mix the normal and the multi-station channels in one room. Either all of them are on `A`/`b`/`c`, or all of them are on `1`-`4`.

openhouse detects the multi-station mode by itself, nothing needs to be set in the driver. The stock SteamVR driver can't use the multi-station channels, so switch back to `A`/`b`/`c` if you go back to it. If tracking doesn't start properly, power cycle the basestations once by unplugging the power adapter and replugging it, which fixes stale mode issues sometimes

## Usage

### SteamVR
After installing, just start SteamVR. That's it. The first time, the room takes a bit to learn. Move your devices around so they see more than one basestation at once. It gets saved, so the next time it's instant.

### Monado / WiVRn
Install it into SteamVR's folder like above, and start Monado with its `steamvr_lh` driver (`STEAMVR_LH_ENABLE=1`).

### openhouse-viewer
`openhouse-viewer` is in the `tools` folder on Linux and the `apps` folder on Windows. Open it while SteamVR is running. It will show you:
- Your devices, whether they're tracking, hidden or lost, their battery and how many basestations see them
- Your basestations and how well they're placed in the room (locked, learning, not placed yet etc)
- Whether the room is ready, and if not, which basestation it's waiting for
- A 3D view of the room
- The logs. DEBUG logs are hidden by default, you can turn them on with the checkbox. You can also hide the spammy ones, or right click a message and hide the messages like it.

View > Advanced shows more details like the position of devices, how much light each device gets from each basestation, basestation faults etc.

### openhouse-compare
`openhouse-compare` (next to `openhouse-viewer`) runs openhouse and SteamVR's own lighthouse driver at the same time on the same devices, so you can see the difference yourself. Close SteamVR first, it runs both drivers by itself. Pick a device on the left, openhouse is shown in blue and SteamVR in red, with how much each one tracked, coasted and lost it.

### Settings
Settings are in `~/.config/lh/` on Linux and `%APPDATA%\lh\` on Windows.
- `roommap.json` is the learned room. Delete it if you want to learn the room from scratch. You can delete it while openhouse is running too, it notices and starts learning the room again right away (same as **Reset room** in SteamVR's settings).
- `hostframe.json` is where your play area is anchored. Delete it if you want to re-anchor.
- `settings.json` is optional. You can use it to disable/enable devices, set manual device roles etc.
```json
{
  "roles": { "LHR-180D3882": "generic_tracker" },
  "controllers": { "left": "LHR-8F9BE6F2", "right": "LHR-13DA81D1" },
  "disabledDevices": ["LHR-0AC3E7C8"]
}
```
`roles` changes a device's role (`left_hand`, `right_hand` or `generic_tracker`, for example a Vive controller on your hip as a tracker), `controllers` picks which controller is left and right, and `disabledDevices` makes openhouse ignore a device completely.
- `preinit/` is optional. Device configs in it (`<serial>.json`) are created when openhouse starts and show up as connected but not tracking until you turn them on (after that they connect and disconnect normally), even under Monado, which only takes devices at startup. Save the configs of your connected devices with `lhtool configs ~/.config/lh/preinit` (close SteamVR/Monado first, turn everything on). If a device's own config differs from its file, openhouse uses the device's and updates the file.

### SteamVR settings
In SteamVR's settings there is an **openhouse** page (Settings > openhouse, you might need to turn on "Advanced Settings"):
- **Acceleration prediction** helps SteamVR predict where your controllers will be during fast motion, like in Beat Saber. On by default.
- **Position smoothing** and **Rotation smoothing** are how long a tracking correction takes to show up, 60 ms by default. Lower means less delay, but small corrections can show up as tiny jumps. 0 turns it off.
- **Record Session (disable to save)** records your devices' raw data while it's on. When you turn it off, it saves a bug report zip with that recording (see below). The recordings themselves stay in `~/.local/share/openhouse/recordings` on Linux and `%LOCALAPPDATA%\openhouse\recordings` on Windows.
- **Reset room** throws away the learned room and learns it again from scratch. Use it if you moved your basestations around and the room doesn't fix itself. It turns itself back off.

Changes apply while you play, within a second.

### Environment variables
You don't need any of these normally. On Linux, put them in SteamVR's launch options like `LH_LOG_LEVEL=DEBUG %command%`. On Windows, set them as system environment variables and restart Steam.

| Variable | What it does |
|---|---|
| `LH_LOG_LEVEL=DEBUG` | Shows DEBUG logs in SteamVR's logs (`INFO` by default, also `WARN` and `ERR`) |
| `LH_TELEMETRY_PORT=N` | The port the viewer connects to (36840 by default, `0` turns it off) |
| `LH_CAPTURE=DIR` | Records the whole session from the start into a folder (see Reporting a bug) |
| `LH_NO_POWER_OFF_ON_EXIT=1` | Don't turn off devices when SteamVR exits, overrides SteamVR's setting |
| `LH_STOCK_HMD=1` / `LH_NO_STOCK_HMD=1` | Always/never load SteamVR's display driver for the headset |
| `LH_STOCK_DRIVER=PATH` | Where SteamVR's original driver is, if it's not next to ours |
| `LH_STOCK_ALL_DEVICES=1` | Let SteamVR's driver see every device, not only the headset. Not recommended as it causes issues with tracking |
| `LH_ROOM_MAP=FILE` | Use another room file |
| `LH_STEAM_DB=auto` | Import the basestation positions from SteamVR's own room (or `=FILE`) |
| `LH_HOST_FRAME=steamvr` | Use SteamVR's own play space instead of openhouse's (`LH_REFERENCE_DB=FILE` for the room file it uses) |
| `LH_MULTISTATION=on\|off` | Force the lhbs multi-station mode on or off for 1.0 basestations |
| `LH_PREFER_STEAM_CONFIG=1` | Use SteamVR's cached device configs instead of reading them from the devices |
| `LH_STEAM_ROOT`, `LH_STEAM_CONFIG_DIR`, `STEAMVR_PATH` | Where Steam and SteamVR are, if they can't be found |
| `LH_HAND_POSES`, `LH_HAND_ANIMS` | Where the Index hand animation files are, if they can't be found |
| `LH_REPLAY=FILE.lhraw` | Run the driver on a recording instead of your devices |
| `LH_STOCK_FILTER=1` | Turns off openhouse's tracking improvements and uses the stock driver's tracking |
| `LH_EVALUATE_ROOM=1` | Freezes the room (development) |

### Reporting a bug
1. Turn on **Record Session** in SteamVR's openhouse settings.
2. Play until the problem happens and keep using it for a minute after it.
3. Turn **Record Session** off.

That saves a zip into `Documents/openhouse reports` which contains the recording, your room map, openhouse's settings files (`settings.json`, `hostframe.json`, `preinit/`) and SteamVR's logs. Link the zip that is generated to me on either GitHub issues or through Discord/Twitter

If the problem happens right when SteamVR starts, set `LH_CAPTURE=/some/folder` (or add `"capture": "C:\\lh-capture\\run1"` under `"driver_lighthouse"` in `steamvr.vrsettings`) to record the whole session from the start. That recording isn't put in a zip, so send me that folder along with SteamVR's `vrserver.txt`. Don't use **Record Session** at the same time, as it takes over the recording.

You can also record without SteamVR with `lhcapture` (close SteamVR first). While it's recording, you can type a note and press enter to mark the time, like `left foot jumping`.

### Tools and the library
- `lhcapture` records your devices' raw data.
- `lhtool session FILE.lhraw` replays a recording through the whole tracking and prints what happened (`--trace OUT.jsonl` for every pose, `--from`/`--to` for a part of it). `lhtool stats` shows per device data rates, `lhtool haptics SERIAL` tests a controller's vibration, `lhtool configs DIR` saves every connected device's config to `DIR`.
- `lhviz` is a 3D viewer that runs its own tracking without SteamVR, mostly for development or testing

All of the tracking is in a C++ library (`liblh`), separate from the SteamVR driver, so you can use it in your own app without SteamVR: `lh::Session` (`include/lh/session.hpp`) opens the devices, tracks them and gives you their poses, inputs and the basestations. The tools above are built on it, so you can look at them to see how the library can be used.

### Building
```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```
Needs Eigen, Ceres, nlohmann-json and GTest (and glfw for the viewers). The tests use short recordings from `tests/data/captures`.

For the release packages, `scripts/build-linux.sh` builds and packages the Linux one, `scripts/build-windows.sh` cross builds and packages the Windows one with MinGW, and `scripts/build-firmware.sh` builds, tests and packages the lhbs firmware (needs `arm-none-eabi-gcc`). Both build their own Ceres through vcpkg, since the distro Ceres usually comes with SuiteSparse, which has GPL parts that can't be shipped with openhouse.

## Explanation of the Tracking
Each basestation sweeps it's lasers across the room, and devices have sensors on them that see the lasers. When a laser hits a sensor, the timing gives the angle of that sensor from the basestation. With enough sensors, you get the device's pose. The IMU (the gyroscrope and the accelerometer) fills in between the sweeps. 1.0 basestations take turns and send a flash before each sweep, 2.0 basestations all sweep at once on their own channels. Basestations also send a slow data stream (OOTX) with their calibration, their accelerometer and their mode.

openhouse's tracking is generally pretty similar in terms of how it works to SteamVR

The light from each basestation is turned into "frames". One sweep from one basestation, with the angle of every sensor that got hit. There are three decoders: regular 1.0 basestation decoder, the multi-station decoder for the custom `lhbs` firmware to use up to 4 basestations, and the 2.0 basestation decoder. Sweeps that are detected wrongly, like wrong axis, impossible angles etc. are dropped.

Each device has a Kalman filter that tracks its position, rotation, speed, IMU calibration and the other basestations it sees relative to its main one. To get its first pose, it fits the pose from one good sweep with a few different starting guesses, and drops sensors that are probably seeing a reflection. On top of SteamVR's filter, openhouse:
- Waits for late light instead of throwing it away (watchman dongles sometimes delays sweeps by a lot)
- Predicts and smooths the output, so corrections don't show up as jumps
- Holds a device that isn't moving completely still to reduce the jitter
- Freezes the slow parts of the filter while you swing a controller around, so its calibration doesn't get ruined which makes openhouse track on conditions where SteamVR fails
- Rejects frames that would suddenly throw the device away
- Handles basestations turning off or completely being hidden away instead of discarding the entire room map
- Allows devices to "coast" around instead of letting them drift to improve tracking occlusion

The room is every basestation's position, and it's shared by every device. When a device sees 2 or more basestations at the same time, it saves a "keyframe". About once a second, every basestation gets placed together from all the keyframes of all the devices. The room then slowly gets built. After a successful room learning (which generally takes around 30 or so seconds), the room map is saved, so next sessions starts with that learned room map

Once a basestation has enough keyframes, it gets locked, and devices can't move it anymore. Only a basestation that has been moved around (which is detected by the accelerometer of the basestation) gets relearned. If the main basestation was turned, the old room is out of date, so a new room gets learned instead. A basestation counts as learned once it's placed within 5 mm, or within 20 mm after it got locked or has enough keyframes. The room is ready (settled) once every basestation you can see is learned. In SteamVR, basestations blink like they're searching until they get locked.

openhouse tracks the headset itself, but the display part (lens distortion, display modes) comes from SteamVR's own driver, which gets loaded only for the headset's display.

## Credits

Huge thanks to the [libsurvive](https://github.com/collabora/libsurvive) project. Their work on the watchman protocol, device and light decoding, controller inputs and the 2.0 basestations was used as a reference a lot while making openhouse.

## License

MIT. See [LICENSE](LICENSE).

## Disclaimer

This project is not affiliated with, endorsed by, or sponsored by Valve Corporation or HTC Corporation. Valve, SteamVR and Lighthouse are trademarks of Valve Corporation. HTC and VIVE are trademarks of HTC Corporation. This repository does  not contain any Valve of HTC property. All of the work here is reverse engineering with improvements over the said reverse engineered code.
