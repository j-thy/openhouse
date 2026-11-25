#!/usr/bin/env bash
# Packages the Linux driver and apps from a release build:
#   scripts/package-linux.sh [BUILD_DIR]     (default build-release)
# Output: dist/openhouse-linux-x64/ and dist/openhouse-linux-x64.tar.gz
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
build="${1:-$repo/build-release}"
out="$repo/dist/openhouse-linux-x64"
rm -rf "$out"
mkdir -p "$out/apps"
python3 "$repo/scripts/package-driver.py" "$build/openvr/lighthouse/bin/linux64/driver_lighthouse.so" "$out" | tail -n1
# The installer (driver and firmware) and the firmware it flashes.
cp "$build/openhouse-installer" "$out/"
strip "$out/openhouse-installer"
patchelf --remove-rpath "$out/openhouse-installer"
python3 "$repo/scripts/stage-firmwares.py" "$out"
# Notices: openhouse's own, the vcpkg libraries built in, and what the tools link or embed.
cp "$repo/LICENSE" "$out/LICENSE.txt"
cp "$build/VERSION.txt" "$out/VERSION.txt"   # the build's version (cmake/build_id.cmake)
prefix=$(sed -n 's/^VCPKG_INSTALLED_DIR:[A-Z]*=//p' "$build/CMakeCache.txt")/$(sed -n 's/^VCPKG_TARGET_TRIPLET:[A-Z]*=//p' "$build/CMakeCache.txt")
for copyright in "$prefix"/share/*/copyright; do
  [[ -f "$copyright" ]] && cp "$copyright" "$out/lh-licenses/$(basename "$(dirname "$copyright")").txt"
done
cp "$repo/third_party/hidapi/LICENSE-bsd.txt" "$out/lh-licenses/hidapi.txt"
cp "$repo/third_party/imgui/LICENSE.txt" "$out/lh-licenses/imgui.txt"
for notice in "$repo"/third_party/lighthouse-ble/*.txt; do cp "$notice" "$out/lh-licenses/lighthouse-ble-$(basename "$notice")"; done
for package in openvr glfw systemd; do
  # -L: copies of what the notices link to (systemd's MIT-0.txt is a link into /usr/share/doc)
  [[ -d /usr/share/licenses/$package ]] && { cp -rL "/usr/share/licenses/$package" "$out/lh-licenses/" 2>/dev/null || true; }
done
find "$out/lh-licenses" -xtype l -delete  # a link whose target wasn't there
# Every app stands alone (liblh's code is built in); none keeps the build directory as its RUNPATH.
for app in lhtool lhcapture lhviz openhouse-viewer openhouse-overlay doorman-dongle-manager; do
  cp "$build/$app" "$out/apps/"
  strip "$out/apps/$app"
  patchelf --remove-rpath "$out/apps/$app"
done
# The apps' menu logos (the installer's menu entries).
cp "$repo/cmake/linux/openhouse.png" "$repo/cmake/linux/dongle-manager.png" "$out/apps/"
# ShayBox's Lighthouse library: the overlay's channel changes over Bluetooth (installed next to it).
python3 "$repo/scripts/fetch-lighthouse-ble.py" linux "$out/apps" > /dev/null
# Runnable straight out of the archive.
chmod 755 "$out/openhouse-installer" "$out"/apps/*
chmod 644 "$out"/apps/*.png
tar czf "$repo/dist/openhouse-linux-x64.tar.gz" -C "$repo/dist" openhouse-linux-x64
echo "Packaged $repo/dist/openhouse-linux-x64.tar.gz"
