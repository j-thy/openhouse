# Configuration

[Back to the README](../README.md)

- [Files](#files)
- [settings.json](#settingsjson)
- [Use SteamVR's rooms](#use-steamvrs-rooms)
- [Devices at startup (preinit)](#devices-at-startup-preinit)
- [Recording from the start](#recording-from-the-start)
- [Environment variables](#environment-variables)

## Files
Settings are in `~/.config/lh/` on Linux and `%APPDATA%\lh\` on Windows.

| File | What it is |
|---|---|
| `roommap.json` | Delete it to learn the room from scratch. You can delete it while openhouse is running too, it notices and starts learning again right away (same as **Reset room** in openhouse-overlay) |
| `hostframe.json` | Where your play area is anchored. Delete it to re-anchor |
| `settings.json` | openhouse's settings, see below |
| `preinit/` | Optional device configs, see [preinit](#devices-at-startup-preinit) |

If you used the openhouse page in SteamVR's settings in an older version, your settings there are moved into `settings.json` the first time the new version starts.

## settings.json
openhouse-overlay's Settings page changes it for you, but you can also edit it by hand. Smoothing, prediction and the mixed options apply within a second, the rest after a restart.

```json
{
  "roles": { "LHR-180D3882": "generic_tracker" },
  "controllers": { "left": "LHR-8F9BE6F2", "right": "LHR-13DA81D1" },
  "disabledDevices": ["LHR-0AC3E7C8"],
  "positionSmoothingMs": 60,
  "rotationSmoothingMs": 60,
  "predictWithAcceleration": true,
  "useSteamVRRooms": true,
  "mixedLight": { "enabled": true, "controllers": true, "trackers": true, "scout": true, "rotorReference": true, "preferLh2": false }
}
```

| Key | What it does |
|---|---|
| `roles` | Changes a device's role: `left_hand`, `right_hand` or `generic_tracker`. For example a Vive controller on your hip as a tracker |
| `controllers` | Which controller is left and which is right |
| `disabledDevices` | openhouse ignores these devices completely |
| `positionSmoothingMs`, `rotationSmoothingMs` | How long a tracking correction takes to show up, 60 ms by default. Lower means less delay, but small corrections can show up as tiny jumps. 0 turns it off |
| `predictWithAcceleration` | Helps SteamVR predict where your controllers will be during fast motion. On by default |
| `useSteamVRRooms` | See [below](#use-steamvrs-rooms). On by default |
| `mixedLight` | Mixed 1.0 and 2.0 base stations, off by default. `"mixedLight": true` is enough to turn it on. See [Base stations](basestations.md#mixed-10-and-20-base-stations-experimental) |

## Use SteamVR's rooms
`useSteamVRRooms` (**Use SteamVR's rooms** in openhouse-overlay, SteamVR only) keeps openhouse in step with SteamVR's own room files, so Room Setup isn't asked for on every start.

- If you already did Room Setup with SteamVR's driver for the same base stations, openhouse uses that room (its play area and standing height).
- Otherwise openhouse adds its room to SteamVR's `lighthousedb.json` (a backup of the original is kept as `lighthousedb.json.openhouse-backup`), so Room Setup is done once and remembered, and other apps that read SteamVR's room see where the base stations are.
- SteamVR's own rooms are never changed.

Turn it off (and restart SteamVR) to keep openhouse's room completely separate from SteamVR's

## Devices at startup (preinit)
Device configs in `preinit/` (`<serial>.json`) are created when openhouse starts and show up as connected but not tracking until you turn them on. After that they connect and disconnect normally. This is mainly for Monado, which only takes devices at startup.

Save the configs of your connected devices with `lhtool configs ~/.config/lh/preinit` (close SteamVR/Monado first and turn everything on). If a device's own config differs from its file, openhouse uses the device's and updates the file.

## Recording from the start
If a problem happens right when SteamVR starts, **Record session** is too late. Set `LH_CAPTURE=/some/folder` (or add `"capture": "C:\\lh-capture\\run1"` under `"driver_lighthouse"` in `steamvr.vrsettings`) to record the whole session from the start. That recording isn't put in a zip, so send that folder along with SteamVR's `vrserver.txt`. Don't use **Record session** at the same time, as it takes over the recording.

You can also record without SteamVR with `lhcapture` (close SteamVR first). While it's recording, you can type a note and press enter to mark the time, like `left foot jumping`.

## Environment variables
You don't need any of these normally. On Linux, put them in SteamVR's launch options like `LH_LOG_LEVEL=DEBUG %command%`. On Windows, set them as system environment variables and restart Steam.

| Variable | What it does |
|---|---|
| `LH_LOG_LEVEL=DEBUG` | Shows DEBUG logs in SteamVR's logs (`INFO` by default, also `WARN` and `ERR`) |
| `LH_TELEMETRY_PORT=N` | The port openhouse-overlay connects to (36840 by default, `0` turns it off, and the overlay with it) |
| `LH_CAPTURE=DIR` | Records the whole session from the start into a folder |
| `LH_NO_POWER_OFF_ON_EXIT=1` | Don't turn off devices when SteamVR exits, overrides SteamVR's setting |
| `LH_STOCK_HMD=1` / `LH_NO_STOCK_HMD=1` | Always/never load SteamVR's display driver for the headset |
| `LH_STOCK_DRIVER=PATH` | Where SteamVR's original driver is, if it's not next to ours |
| `LH_STOCK_ALL_DEVICES=1` | Let SteamVR's driver see every device, not only the headset. Not recommended as it causes issues with tracking |
| `LH_ROOM_MAP=FILE` | Use another room file |
| `LH_STEAM_DB=auto\|-` | Import the base station positions from SteamVR's own room (or `=FILE`); `-` never imports. Under SteamVR it's on with **Use SteamVR's rooms** |
| `LH_HOST_FRAME=steamvr\|openhouse` | `steamvr`: always use SteamVR's own play space (`LH_REFERENCE_DB=FILE` for the room file it uses). `openhouse`: always openhouse's, and SteamVR's room files are left alone |
| `LH_MULTISTATION=on\|off` | Force the lhbs multi-station mode on or off for 1.0 base stations |
| `LH_PREFER_STEAM_CONFIG=1` | Use SteamVR's cached device configs instead of reading them from the devices |
| `LH_STEAM_ROOT`, `LH_STEAM_CONFIG_DIR`, `STEAMVR_PATH` | Where Steam and SteamVR are, if they can't be found |
| `LH_HAND_POSES`, `LH_HAND_ANIMS` | Where the Index hand animation files are, if they can't be found |
| `LH_REPLAY=FILE.lhraw` | Run the driver on a recording instead of your devices |
| `LH_STOCK_FILTER=1` | Turns off openhouse's tracking improvements and uses the stock driver's tracking |
| `LH_EVALUATE_ROOM=1` | Freezes the room (development) |
