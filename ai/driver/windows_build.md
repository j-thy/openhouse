# Windows x64 / MinGW build

## Deliverables

`scripts/build-windows.sh` builds and packages:

* `build-win64/openvr/lighthouse/bin/win64/driver_lighthouse.dll`;
* `apps/`: `liblh.dll`, `openhouse-overlay.exe`, `openhouse-compare.exe`, `lhviz.exe`,
  `lhtool.exe`, `lhcapture.exe`, `update-checker.exe`; `openhouse-installer.exe` at the top;
* `dist/openhouse-windows-x64.zip`: stripped binaries, `LICENSE.txt`, dependency notices
  (`licenses/`), `BUILD.json` (hashes), the settings page schema, and the installer.

Every Windows binary carries a `VERSIONINFO` resource and executables an `asInvoker`
manifest (`cmake/windows/lh_windows_resources.cmake`): unsigned binaries without them
score worse with Windows Defender, which flagged releases.

The driver DLL is self-contained: Ceres, glog/gflags, hidapi, zlib and compiler
runtimes are statically linked. PE import inspection checks that it needs only
Windows system/UCRT libraries, not Linux libraries or separate MinGW DLLs.
The standalone apps share `apps/liblh.dll`; keep the apps directory together.
Windows 10/11 x64 with the Universal CRT is the intended target.

## Building on Linux

Required tools: MinGW-w64, vcpkg, Git, CMake, Ninja, Python 3, and the OpenVR driver
header (the platform-neutral SDK header at `/usr/include/openvr` works here).
No additional compiler/packages were needed on the current machine.

```
scripts/build-windows.sh
```

The system vcpkg executable initially had no ports checkout. The script uses
`$LH_VCPKG_ROOT`, or a dedicated `$XDG_CACHE_HOME/openhouse/vcpkg` checkout (default
`~/.cache/openhouse/vcpkg`). It installs the pinned `vcpkg.json` dependencies and test/
viewer features for `x64-mingw-static`, then builds with up to 16 threads.
`cmake/mingw-w64.cmake` prevents accidentally linking host Linux libraries.
`OPENVR_INCLUDE_DIR` and `VCPKG_MAX_CONCURRENCY` can override the defaults.
Use a dedicated vcpkg root: its installed dependency set is managed by the manifest.

The vcpkg manifest pins baseline `a5f736224e85b32e25a8ffc52562333180396642`:
Ceres 2.2 (Schur specialization, no SuiteSparse/LAPACK dependency), Eigen 5.0.1,
hidapi, zlib, nlohmann-json; optional GLFW and GTest. Our solvers use dense Ceres
solvers, so the omitted sparse backends are not required. vcpkg app-local DLL
copying is disabled because this is a static-dependency cross-build.

For repackaging an existing build:

```
python3 scripts/package-windows.py build-win64 "$VCPKG_ROOT/installed/x64-mingw-static"
```

`BUILD.json` records source commit/dirty status, compiler, pinned dependency
baseline, artifact hashes and PE imports. Do not mistake an artifact built from a
dirty tree for that commit alone.

## Building natively on Windows (MSVC)

Verified 2026-11-09 with Visual Studio Build Tools 2022 (17.14, MSVC 19.44, its bundled
CMake and Ninja) and a dedicated vcpkg checkout: every suite passes (unit, replay,
runtime, OpenVR components, the mock host loading the driver DLL, capture), oracle tests
excluded. From a "x64 Native Tools" prompt:

```
cmake -S . -B build-msvc -G Ninja -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
  -DVCPKG_OVERLAY_TRIPLETS=%CD%\triplets -DVCPKG_TARGET_TRIPLET=x64-windows-static-avx2 ^
  -DVCPKG_MANIFEST_FEATURES="tests;viewer" -DOPENVR_INCLUDE_DIR=<dir with openvr_driver.h> ^
  -DLH_BUILD_ORACLE_TESTS=OFF
cmake --build build-msvc
ctest --test-dir build-msvc --output-on-failure
```

MSVC needs the code to stay off GCC-only and POSIX-only spellings: `__attribute__` behind
`__GNUC__`, no `<unistd.h>`/`getpid`/`mkstemp` in tests (`platform::ProcessId`, the temp
directory), `update-checker` (a GUI app with `main`) links with
`/ENTRY:mainCRTStartup`, `-municode` is for MinGW only (MSVC starts `wmain` itself), and
the linker's own manifest is off (`/MANIFEST:NO`) because the `.rc` already embeds one.
The first configure builds Ceres and friends (~15 min). The shared library is `lh.dll`
there (`liblh.dll` with MinGW).

## Windows installer (`apps/openhouse-installer`)

A console exe (it replaced `Install-Driver.bat` running `powershell -ExecutionPolicy
Bypass`, a dropper pattern Defender flagged). Without arguments it asks install or
uninstall; `--install`, `--uninstall`, `--status`, `--steamvr PATH`, `--yes`. It offers
to close Steam / SteamVR (they hold the DLL open), overwrites with retries, keeps the stock DLL as
`driver_lighthouse.dll.stock` (needed for the headset display), verifies the installed
hash, installs the settings page schema (an existing one is kept as `.stock`), and
registers the update check (`update-checker.exe` in `%LOCALAPPDATA%\openhouse`,
autolaunched overlay). SteamVR is found through OpenVR's runtime path file, then
Steam's registry path; `--steamvr` overrides. Whenever the installed DLL lacks the
marker `liblh session started` it is SteamVR's (a newer one after an update) and becomes
the backup. Uninstall reverses all of it, but keeps a newer SteamVR DLL over an old backup.

## Platform handling

`src/platform/platform.hpp` handles:

* room/settings files in `%APPDATA%\lh`, with `%USERPROFILE%` fallback;
* Steam's HKCU `Software\Valve\Steam\SteamPath` location;
* assets relative to the installed module, covering another Steam library;
* replacing an existing saved file using `MoveFileExW` with replacement semantics
  (never delete the old file first);
* process IDs and the capture tool's `vrserver.exe` exclusion check.

Overrides: `LH_STEAM_ROOT`, `LH_STEAM_CONFIG_DIR`, `STEAMVR_PATH`, `LH_HAND_POSES`,
`LH_REFERENCE_DB`, `LH_ROOM_MAP`, `LH_STEAM_DB`. The capture tool has nonblocking
Windows console/redirected-input handling. HID access uses hidapi's Windows backend.
SteamVR host room/chaperone files are never rewritten by our alignment code.

## Validation and limits

**Verified under Wine with an isolated prefix, not native Windows:**

* all 8 available Windows CTest suites pass (Linux original-binary oracle tests
  are intentionally excluded);
* Windows `LoadLibrary`/`GetProcAddress` mock host loads the DLL, registers the
  Index controller, receives valid replay poses and skeleton/input updates;
* room persistence, replacing an existing file, and platform path overrides;
* simulated controller power-cycle replay: a fresh protocol/clock/filter epoch
  reacquires tracking without discarding the shared room;
* Windows visualizer startup/render/replay/screenshot smoke test;
* stripped package dependency/import inspection.

To run the tests via Wine, configure `CMAKE_CROSSCOMPILING_EMULATOR` as a semicolon-
separated command list, for example `/usr/bin/env;WINEPREFIX=/path/to/isolated-prefix;
WINEDEBUG=-all;/usr/bin/wine` (without a newline). Then run:

```
xvfb-run -a ctest --test-dir build-win64 --output-on-failure -j4
```

Set `LH_HAND_POSES` to the installed Valve GLB's Windows/Wine path to enable
asset-dependent skeletal checks. Wine passing does not prove native hardware behaviour; the owner and testers run the
release on native Windows.

## Size

Release build (`scripts/build-windows.sh`): no debug info, `-ffunction-sections
-fdata-sections`, `--gc-sections` and `-s`, with release-only dependencies from the
overlay triplet `cmake/triplets/x64-mingw-static-release.cmake`. The driver DLL is
7.3 MiB (was 163 MiB unstripped RelWithDebInfo). Ceres is 4.3 MiB of it, 2.5 MiB of
which are the `schur` fixed-size specializations used by the room solver's
DENSE_SCHUR; they are kept for solver speed.
