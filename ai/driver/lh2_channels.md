# Lighthouse 2.0 channel management

The Base stations page can change a station to an unused channel over Bluetooth LE.
Enable **Automatically resolve channel conflicts** there to resolve further conflicts
during this overlay session. The overlay keeps one base on its channel, moves the
others one at a time, and waits for a fresh optical report after each write.
A write error, lost driver connection or missing confirmation stops automatic
resolution. The toggle resets when the overlay restarts. OpenHouse drops sweeps
from every station on a shared channel. A device that sees only those stations
temporarily loses optical tracking.

Install the optional [ShayBox Lighthouse 2.1 C library](https://github.com/ShayBox/Lighthouse/releases/tag/2.1.0)
for your platform. For 64-bit Windows, use the `lighthouse-x86_64-pc-windows-msvc.dll`
release asset. Name it `lighthouse.dll` in `%LOCALAPPDATA%/openhouse` on Windows or
`liblighthouse.so` in `~/.local/share/openhouse` on Linux, or set
`OPENHOUSE_LIGHTHOUSE_BLE` to its path. Restart the overlay. A BLE adapter and a
station firmware that accepts BLE channel changes are required.

OpenHouse reserves channels known to the driver and changes only stations whose
optical serial matches a BLE advertisement. Bluetooth scans happen for channel
changes only. The desktop and SteamVR overlays share a local IPC lock independent
of their data directories. It is held from discovery through confirmation,
cancellation or timeout.
Normally the tracker has to see the station for several seconds
after the change to confirm it. If the old room was learned during a conflict,
use **Reset room** once the bases have distinct channels.

## Optical confirmation and build checks

While a channel is blocked, complete OOTX frames with a valid CRC can refresh
observations and identify additional stations sharing the channel. The driver
keeps all of those stations out of optical tracking and room learning until the
conflict clears. It disables reconstruction of OOTX frames across revolutions,
which could combine bits from different stations. If interference prevents a fresh observation,
change the channel using the station's supported controls.

The driver's telemetry and the overlay's write completion time use the host
steady clock. The overlay checks that the live driver timestamp is within two
seconds of its own clock before allowing a write. It confirms only a direct
observation on the requested channel with a timestamp newer than write
completion. The driver records this timestamp when it processes the beam word
that completes the OOTX frame. An incompatible clock outside the freshness window prevents a
write or causes confirmation to fail if the clock becomes incompatible during
the operation.

CI downloads the pinned 2.1.0 release header and checks the signatures of all
dynamically loaded functions at build time. To enable the same check locally,
set the CMake cache path `LIGHTHOUSE_INCLUDE_DIR` to the directory containing
that header.
