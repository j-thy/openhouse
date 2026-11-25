#!/usr/bin/env bash
# Build for the Linux package, with static vcpkg dependencies (the same Ceres
# as the Windows build: no SuiteSparse), then packages it (scripts/package-linux.sh).
# Does not install or change the running driver.
set -euo pipefail
# --release: the plain version (a signed release). Without it the version gets the commit,
# and -dirty with uncommitted changes (cmake/build_id.cmake).
unset OPENHOUSE_RELEASE   # only --release makes a release build
for arg in "$@"; do
  case "$arg" in
    --release) export OPENHOUSE_RELEASE=1 ;;
    *) echo "usage: $0 [--release]" >&2; exit 2 ;;
  esac
done
repo=$(cd "$(dirname "$0")/.." && pwd)
root=${LH_VCPKG_ROOT:-${XDG_CACHE_HOME:-$HOME/.cache}/openhouse/vcpkg}
if [[ ! -f "$root/scripts/buildsystems/vcpkg.cmake" ]]; then
  git clone https://github.com/microsoft/vcpkg.git "$root"
fi
[[ -x "$root/vcpkg" ]] || "$root/bootstrap-vcpkg.sh" -disableMetrics
export VCPKG_ROOT="$root" VCPKG_MAX_CONCURRENCY="${VCPKG_MAX_CONCURRENCY:-16}"
triplet=x64-linux-static-release
# Each platform has its own install folder: a manifest install removes what it didn't ask for.
"$root/vcpkg" install --x-manifest-root="$repo" --x-install-root="$root/installed-linux" \
  --overlay-triplets="$repo/cmake/triplets" --triplet=$triplet --x-feature=tests
build="$repo/build-release"
# A build configured against the system Ceres can't be switched in place.
if [[ -f "$build/CMakeCache.txt" ]] && ! grep -q "VCPKG_TARGET_TRIPLET:STRING=$triplet" "$build/CMakeCache.txt"; then
  rm -rf "$build"
fi
cmake -S "$repo" -B "$build" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$root/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_TARGET_TRIPLET=$triplet -DVCPKG_OVERLAY_TRIPLETS="$repo/cmake/triplets" -DVCPKG_INSTALLED_DIR="$root/installed-linux" \
  -DVCPKG_MANIFEST_MODE=OFF -DCMAKE_BUILD_TYPE=Release \
  -DLH_BUILD_TESTS=ON -DLH_BUILD_VIEWER=ON -DLH_BUILD_TOOLS=ON -DLH_STATIC_GLFW=ON
cmake --build "$build" -j"${VCPKG_MAX_CONCURRENCY}"
"$repo/scripts/package-linux.sh" "$build"
