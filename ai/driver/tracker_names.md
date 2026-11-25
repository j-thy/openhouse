# Tracker names

A tracker can be given a name (_Waist_, _Left foot_) on the overlay's Devices page, and the name
is used wherever openhouse labels that device: the device list, the detail heading, the room map,
the bases page's "seen by" rows. SteamVR keeps identifying the tracker by its hardware serial;
nothing in the OpenVR device itself changes, so the name is an openhouse label and only that.

## Where it lives

`settings.json`, under `trackerNames`, keyed by hardware serial:

```json
"trackerNames": { "LHR-9EA2C624": "Left foot" }
```

Keyed by serial, not by index or connection, so a name follows the physical tracker across
reconnects, dongles and restarts. An empty name deletes the entry rather than storing `""`.

It reaches the clients through telemetry, in the `state` line's `settings` object
(`include/lh/telemetry.hpp`), not through a separate message: the overlay already parses
`settings` for its Settings page, and the viewer reads the same key. Both use
`lh::telemetry::TrackerDisplayName`, so the two copies of the client cannot disagree about it.

## Names kept on a Doorman Dongle

A Doorman Dongle (firmware 0x0025+) keeps a name per paired slot in flash with its pairings
(cmd 0x34 set, 0x35 read; at most 59 bytes; forgetting or re-pairing the slot clears it), so the
name goes wherever the dongle goes. The dongle manager writes there when renaming (and moves a name
it had only in its own settings onto the dongle once). The driver reads every paired slot's name
every 2 s and lays them over `trackerNames` in the published settings: the dongle's name wins,
except where this PC's name is longer and starts with it (cut at the dongle's 59 bytes). A rename
from the overlay of a device on such a dongle is written to both. Dongle names may belong to any
device (an Index controller too), so the clients show a name for any role; renaming from the
overlay stays trackers only.

## The command

```json
{"type":"command","id":N,"cmd":"renameTracker","serial":"LHR-...","name":"Left foot"}
```

`""` (or spaces only) clears the name. The driver answers `ok: false` with an error for an
unknown serial, a serial that isn't a tracker, a name that is too long, or a settings file it
can't write.

## What a name may be

`kTrackerNameMaxBytes` (63) is in `lh/telemetry.hpp`, with the rules in
`src/common/tracker_names.hpp` and the driver's checks in `src/openvr/tracker_names.hpp`;
`tests/unit/test_tracker_names.cpp` pins them:

- leading and trailing **spaces** are trimmed; spaces inside are kept;
- at most **63 bytes** of UTF-8 — bytes, not characters, because that is what the overlay's
  buffer and the telemetry field carry. 23 CJK characters are 69 bytes and are refused;
- no **control characters** (below 0x20, or 0x7F);
- empty after trimming means "clear the name".

`SetTrackerName` reports whether anything changed, so a rename to the same name doesn't rewrite
`settings.json`.

The SteamVR keyboard is asked for **256 characters** into a 1025-byte buffer, and the 63-byte
limit is applied afterwards. That ordering matters: asking for 63 characters and storing 63 bytes
would truncate a multi-byte name mid-sequence and store invalid UTF-8. Too long is reported to
the user instead of being silently cut.

Invalid UTF-8 cannot reach the driver from a client: the command is parsed as JSON, and
`nlohmann::json` refuses an invalid UTF-8 string before the handler runs.

## What a name can look like on screen

ImGui 1.92's font atlas is dynamic, and the OpenGL2 backend advertises
`ImGuiBackendFlags_RendererHasTextures`, so glyphs are loaded on demand instead of being baked up
front. `lh::ui::MergeTrackerNameFonts` (`src/common/ui_name_fonts.hpp`) therefore merges several
installed system fonts into the UI font without paying for a full CJK atlas: Segoe UI, YaHei,
MingLiU, MS Gothic and Malgun on Windows; DejaVu and Noto (including Noto CJK) on Linux. A font
that is missing or fails to parse is skipped (`ImFontFlags_NoLoadError`), not an error.

`IMGUI_USE_WCHAR32` is set on `lh_imgui` (publicly, so every consumer agrees) because a codepoint
above 0xFFFF — emoji, some CJK extensions — does not fit a 16-bit `ImWchar`.

On **Linux** the build links FreeType (`IMGUI_ENABLE_FREETYPE`) because Noto CJK ships CFF2
outlines, which stb_truetype cannot load; `docs/development.md` lists the package. Windows keeps
stb_truetype, which reads the `.ttc` files listed above (face 0).

**Only the default font carries the name fonts.** `MergeTrackerNameFonts` merges into
`Fonts.back()`, and the overlay merges once, after adding the default font. That is sufficient
because every device label is drawn with the default font — including the detail heading, which
uses `PushFont(nullptr, size)` to *scale* the current font rather than switching to `g_title`. If
a name ever needs `g_title` or `g_huge`, merge into those as well, at the cost of another copy of
each font file in memory.
