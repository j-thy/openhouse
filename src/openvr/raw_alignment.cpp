#include "raw_alignment.hpp"

#include "platform/platform.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace lh::openvr {
bool RawAlignment::Load(const std::string& path, std::string* error) {
  std::ifstream f(lh::platform::FsPath(path));
  if (!f) { if (error) *error = "cannot open reference room " + path; return false; }
  std::ostringstream text; text << f.rdbuf();
  return LoadText(text.str(), error);
}
bool RawAlignment::LoadText(const std::string& lighthousedb, std::string* error) {
  reference_ = {};
  anchor_ = 0; roomId_.clear(); rawFromWorld_.setIdentity();
  levelDecided_ = false; levelFix_.setIdentity();
  return reference_.ImportSteamDatabase(lighthousedb, error);
}
bool RawAlignment::Update(const SessionSnapshot& snapshot) {
  // Select the reference universe using observed base IDs, not a made-up room ID.
  if (!anchor_) {
    for (bool originOnly : {true, false}) {
      for (const auto& b : snapshot.bases) {
        if (!b.known || (originOnly && !b.origin)) continue;
        auto known = reference_.KnownConfigs().find(b.id);
        if (known == reference_.KnownConfigs().end()) continue;
        reference_.OnBaseSeen(known->second.first, known->second.second);
        if (reference_.Base(b.id)) { anchor_ = b.id; break; }
      }
      if (anchor_) break;
    }
  }
  for (const auto& b : snapshot.bases) {
    if (b.id != anchor_ || !b.known) continue;
    const auto* ref = reference_.Base(anchor_);
    if (!ref) continue;
    const auto& p = b.worldFromBase;
    Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
    worldFromBase.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]).normalized().toRotationMatrix();
    worldFromBase.translation() = Eigen::Vector3d(p.position.data());
    rawFromWorld_ = ref->worldFromBase * worldFromBase.inverse();
    // The host frame is SteamVR's (lighthousedb.json): its yaw, origin and level, with
    // the anchor exactly where SteamVR has it, whatever our room does inside (our own
    // re-levelling cancels out here). Tools that align SteamVR's lighthouse frame to
    // something else (motoc, Space Calibrator and the like) count on the anchor
    // staying put: some start over when it turns a few degrees. Only a level that's
    // grossly off (> 5 deg: a base moved since SteamVR last saw it) is corrected, once
    // per session, about the anchor, and then held: never a frame that drifts.
    if (!levelDecided_) {
      levelDecided_ = true;
      const Eigen::Vector3d upRaw = rawFromWorld_.rotation() * Eigen::Vector3d::UnitY();
      if (std::acos(std::clamp(upRaw.y(), -1.0, 1.0)) > kGrossLevelRad)
        levelFix_ = Eigen::Quaterniond::FromTwoVectors(upRaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }
    const Eigen::Vector3d at = ref->worldFromBase.translation();
    rawFromWorld_.linear() = levelFix_ * rawFromWorld_.rotation();
    rawFromWorld_.translation() = at + levelFix_ * (rawFromWorld_.translation() - at);
    roomId_ = reference_.Active()->id;
    return true;
  }
  // Do not apply an alignment to an unrelated room, or to an invalidated anchor.
  anchor_ = 0; roomId_.clear(); rawFromWorld_.setIdentity();
  return false;
}
}
