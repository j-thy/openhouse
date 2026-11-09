# example_app

A small example of using openhouse in your own C++ project. It creates a new openhouse session and handles device tracking and device updates such as
- devices connecting, disconnecting and their tracking state changing
- the room as basestations get placed and learned
- every connected device's position, once a second

## Building
```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
It builds openhouse's library from the source in this repository. To use another copy of openhouse, set `OPENHOUSE_SOURCE_DIR`:
```sh
cmake -S . -B build -DOPENHOUSE_SOURCE_DIR=/path/to/openhouse
```
You need the same dependencies as openhouse itself (Eigen, Ceres, nlohmann-json, zlib, and libudev on Linux, or hidapi via Homebrew on macOS: `brew install eigen ceres-solver nlohmann-json hidapi ninja cmake`)

## Running
```sh
./build/example_app                       # room map in room.json
./build/example_app my_room.json          # your own room map file
./build/example_app my_room.json --replay recording.lhraw   # a recording instead of your devices
```
If the room map file doesn't exist, the room gets learned from scratch while you move your devices around, and it's saved to that file when you quit with Ctrl+C. The next time it starts with that room already learned.
