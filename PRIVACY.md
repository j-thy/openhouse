# Privacy

openhouse doesn't collect, store or send any personal data, and it has no analytics or telemetry that leaves your computer.

The installer adds a small update check, `openhouse-check`, that SteamVR starts with each session (on Linux it can also run when Monado or WiVRn starts, if you choose that during install). It asks GitHub (`api.github.com`) for the latest openhouse release and, if it's newer than yours, shows a dialog with a link to it

- It sends nothing about you or your devices. Only GitHub releases are requested. see [GitHub's privacy statement](https://docs.github.com/en/site-policy/privacy-policies/github-general-privacy-statement).
- `openhouse-overlay` does the same check on its About page.
- To turn it off, set the environment variable `OPENHOUSE_NO_UPDATE_CHECK=1`, or uninstall openhouse.

- Tracking data is used by the driver and saved on your computer (`roommap.json`, `settings.json`, `hostframe.json` in openhouse's config folder). It also reads and updates SteamVR's own `lighthousedb.json`, keeping a backup of it
- The driver serves its live state to `openhouse-overlay` on `127.0.0.1` (your own computer only, port 36840), and takes commands from it (changing settings, recording, resetting the room etc). Nothing outside your computer can connect to it.
- Bug reports and recordings are saved on your computer (`Documents/openhouse reports`). They are only sent if you send them yourself. A report contains your device and base station serial numbers, your room map and SteamVR's logs, which can include your Windows user name in file paths. look through it first if that matters to you.

## Third-party components

openhouse doesn't include any third-party service or SDK that sends data. Its libraries (listed in `licenses/` in each release) run locally only
