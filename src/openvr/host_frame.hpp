// The host frame in our own room mode: fixed across sessions, whatever our room does.
//
// Our room re-levels itself from the devices' gravity every session (1-2.5 deg), and
// its solver refines the bases. Reported as is, the host frame followed: an alignment tool
// saw its anchor turn and reset its alignment ("anchor moved") every session. So, as
// RawAlignment does against SteamVR's lighthousedb.json, the room's origin base is
// pinned to the position and heading it had when this room was first anchored (saved in
// hostframe.json beside the room map): a turn about the vertical and a shift, never a
// tilt. The host frame is always the room's level (its gravity): pinning the whole pose
// kept every later re-level out of it, and a 51 deg re-level of a remounted origin left
// the frame 53 deg off level, past what an alignment tool can fit (yaw + translation). A knocked
// origin that the room re-placed (SessionSnapshot::originEpoch) moves the anchor with it
// instead: the room didn't move, so the host frame doesn't either.
#pragma once
#include <Eigen/Geometry>
#include <map>
#include <optional>
#include <string>

#include "lh/session.hpp"

namespace lh::openvr {

class HostFrame {
 public:
  void Load(const std::string& path);
  // Returns rawFromWorld (identity until the room is anchored).
  const Eigen::Isometry3d& Update(const SessionSnapshot& snapshot, double now);
  bool Anchored() const { return anchored_; }

  // Room calibration (openhouse-overlay, for runtimes without SteamVR's Room Setup): a turn
  // about the vertical and a shift applied on top of the pinned frame, per room, saved in
  // hostframe.json. Identity until set. The current room's (none: identity).
  Eigen::Isometry3d Calibration() const;
  bool SetCalibration(const Eigen::Isometry3d& calibration);  // false: no anchored room
  // The pinned frame without the calibration (what the calibration is applied to).
  const Eigen::Isometry3d& Pinned() const { return pinned_; }

 private:
  struct Anchor {
    std::string base;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d calibration = Eigen::Isometry3d::Identity();
  };
  void Save() const;
  std::string path_;
  std::map<std::string, Anchor> anchors_;  // by room id
  Eigen::Isometry3d rawFromWorld_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pinned_ = Eigen::Isometry3d::Identity();  // rawFromWorld_ without the calibration
  bool anchored_ = false;
  // A first anchor waits for the origin to hold still (the session's first level).
  uint64_t originEpoch_ = 0;
  std::string lastRoom_;
  std::optional<Eigen::Isometry3d> stillPose_;
  double stillSince_ = 0;
};

}  // namespace lh::openvr
