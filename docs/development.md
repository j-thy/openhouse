# Development

[Back to the README](../README.md)

- [The library](#the-library)
- [Tools](#tools)
- [Building](#building)

## The library
All of the tracking is in a C++ library (`liblh`), separate from the SteamVR driver, so you can use it in your own app without SteamVR. `lh::Session` (`include/lh/session.hpp`) opens the devices, tracks them and gives you their poses, inputs and the base stations.

[The example project](../apps/example_app) can be used as a basic reference. The tools below are built on the same library, so you can look at them to see more of how it's used.

## Tools
- `lhcapture` records your devices' raw data. While it's recording, you can type a note and press enter to mark the time.
- `lhtool session FILE.lhraw` replays a recording through the whole tracking and prints what happened (`--trace OUT.jsonl` for every pose).
- `lhtool stats` shows per device data rates, `lhtool haptics SERIAL` tests a controller's vibration, `lhtool configs DIR` saves every connected device's config to `DIR`.
- `lhviz` is a 3D viewer that runs its own tracking without SteamVR.
- `openhouse-viewer` shows the running driver's status and log in a window.

## Building
```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```
Needs Eigen, Ceres, nlohmann-json and GTest. The viewers also need GLFW and OpenGL. On Linux, install the FreeType development package (`libfreetype-dev` on Debian and Ubuntu, `freetype-devel` on Fedora). The tests use short recordings from `tests/data/captures`.

For the release packages:
- `scripts/build-linux.sh` builds and packages the Linux one
- `scripts/build-windows.sh` cross builds and packages the Windows one with MinGW
- `scripts/build-firmware.sh` builds, tests and packages the lhbs firmware (needs `arm-none-eabi-gcc`)

The package scripts build their own Ceres through vcpkg, since the distro Ceres usually comes with SuiteSparse, which has GPL parts that can't be shipped with openhouse.
