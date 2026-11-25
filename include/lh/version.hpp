// openhouse's release version (the driver log, the telemetry hello, the overlay, the installer).
// Keep in sync with project(... VERSION) in CMakeLists.txt (numbers only there) and vcpkg.json.
#pragma once

#include <array>
#include <cctype>
#include <climits>
#include <cstdlib>
#include <string>

// The release this source is. Builds add -<commit>[-dirty] unless built with --release
// (cmake/build_id.cmake): LH_VERSION_STRING is what every app shows.
#define LH_VERSION_BASE "1.0.4"
#if __has_include("lh/build_id.h")
#include "lh/build_id.h"
#endif
#ifndef LH_BUILD_SUFFIX
#define LH_BUILD_SUFFIX ""
#endif
#ifndef LH_BUILD_COMMIT
#define LH_BUILD_COMMIT ""   // the short commit of a non-release build, "" for a release
#endif
#define LH_VERSION_STRING LH_VERSION_BASE LH_BUILD_SUFFIX

namespace lh {
inline constexpr const char* kVersion = LH_VERSION_STRING;
inline constexpr const char* kBuildCommit = LH_BUILD_COMMIT;
// In the driver binary as is, so openhouse-installer can read which version is installed.
inline constexpr const char* kVersionMarker = "openhouse-version:" LH_VERSION_STRING ";";

// "v1.0.4", "1.0.4-rc1", "1.0" -> {major, minor, patch, rc}; missing parts are 0. A release
// sorts after its release candidates: its rc is INT_MAX, so 1.0.4-rc1 < 1.0.4-rc2 < 1.0.4.
// A build's -<commit> suffix doesn't count: 1.0.4-abc1234 is 1.0.4.
inline std::array<int, 4> ParseVersion(std::string v) {
  std::array<int, 4> out{0, 0, 0, INT_MAX};
  if (!v.empty() && (v[0] == 'v' || v[0] == 'V')) v.erase(0, 1);
  size_t i = 0;
  for (int part = 0; part < 3 && i < v.size() && std::isdigit(static_cast<unsigned char>(v[i])); ++part) {
    size_t end = i;
    while (end < v.size() && std::isdigit(static_cast<unsigned char>(v[end]))) ++end;
    out[size_t(part)] = std::atoi(v.substr(i, end - i).c_str());
    i = end;
    if (i < v.size() && v[i] == '.') ++i;
    else break;
  }
  if (const size_t rc = v.find("-rc", i); rc != std::string::npos) out[3] = std::atoi(v.c_str() + rc + 3);
  return out;
}

// The commit in a build's version ("1.0.4-4d771af-dirty" -> "4d771af"), "" for a release.
inline std::string VersionCommit(const std::string& v) {
  for (size_t i = v.find('-'); i != std::string::npos; i = v.find('-', i + 1)) {
    size_t end = i + 1;
    while (end < v.size() && std::isxdigit(static_cast<unsigned char>(v[end]))) ++end;
    if (end - i - 1 >= 7 && (end == v.size() || v[end] == '-')) return v.substr(i + 1, end - i - 1);
  }
  return {};
}

// How build b compares to build a. Different version numbers: by number. The same number
// with a commit on either side: GitHub's compare API (a release is its tag "v<number>").
// Unknown: GitHub didn't answer, doesn't know a commit (a local one), the two diverged, or
// anything unexpected. get(path) GETs an api.github.com path, "" on failure.
enum class BuildOrder { Older, Same, Newer, Unknown };
template <class Get>
BuildOrder CompareBuilds(const std::string& a, const std::string& b, Get get) {
  const auto x = ParseVersion(a), y = ParseVersion(b);
  if (x != y) return y > x ? BuildOrder::Newer : BuildOrder::Older;
  const std::string ca = VersionCommit(a), cb = VersionCommit(b);
  if (ca.empty() && cb.empty()) return BuildOrder::Same;
  if (ca == cb) return BuildOrder::Same;   // same commit (one may be -dirty: can't tell those apart)
  std::string tag = ca.empty() ? a : b;                  // the release side, when there is one
  if (!tag.empty() && tag[0] != 'v' && tag[0] != 'V') tag = "v" + tag;
  const std::string from = ca.empty() ? tag : ca, to = cb.empty() ? tag : cb;
  const std::string json = get("/repos/YusufOzmen01/openhouse/compare/" + from + "..." + to);
  const size_t key = json.find("\"status\"");
  if (key == std::string::npos) return BuildOrder::Unknown;
  const size_t colon = json.find(':', key);
  const size_t open = colon == std::string::npos ? colon : json.find('"', colon + 1);
  const size_t close = open == std::string::npos ? open : json.find('"', open + 1);
  if (close == std::string::npos) return BuildOrder::Unknown;
  const std::string status = json.substr(open + 1, close - open - 1);
  if (status == "ahead") return BuildOrder::Newer;      // b has commits a lacks
  if (status == "behind") return BuildOrder::Older;
  if (status == "identical") return BuildOrder::Same;
  return BuildOrder::Unknown;                           // diverged, or something new
}

// Whether GitHub's latest release (its tag, "v1.0.4") is newer than this build.
enum class UpdateState { Current, Newer, Unknown };
template <class Get>
UpdateState CheckRelease(const std::string& tag, Get get) {
  const BuildOrder o = CompareBuilds(kVersion, tag, get);
  return o == BuildOrder::Newer ? UpdateState::Newer : o == BuildOrder::Unknown ? UpdateState::Unknown : UpdateState::Current;
}
}
