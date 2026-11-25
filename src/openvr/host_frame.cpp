#include "host_frame.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>

#include "platform/platform.hpp"

namespace lh::openvr {
namespace {
Eigen::Isometry3d FromPose(const Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3])
                   .normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.data());
  return t;
}

// A base's heading about the vertical (world y): its forward axis (-z) on the floor,
// or its x axis for a base looking (nearly) straight up or down.
double Heading(const Eigen::Matrix3d& r) {
  Eigen::Vector3d f = -r.col(2);
  if (std::hypot(f.x(), f.z()) < 0.2) f = r.col(0);
  return std::atan2(f.x(), f.z());
}

// The level transform that puts `now` (the origin in the room) at `anchor`'s position
// and heading: a turn about the vertical and a shift, never a tilt.
Eigen::Isometry3d LevelPin(const Eigen::Isometry3d& anchor, const Eigen::Isometry3d& now) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(Heading(anchor.rotation()) - Heading(now.rotation()), Eigen::Vector3d::UnitY()).toRotationMatrix();
  t.translation() = anchor.translation() - t.linear() * now.translation();
  return t;
}
}  // namespace

void HostFrame::Load(const std::string& path) {
  path_ = path;
  anchors_.clear();
  std::ifstream f(lh::platform::FsPath(path));
  if (!f) return;
  try {
    const auto j = nlohmann::json::parse(f);
    for (const auto& [room, a] : j.at("rooms").items()) {
      Anchor anchor;
      anchor.base = a.at("base").get<std::string>();
      const auto q = a.at("orientation").get<std::array<double, 4>>();
      const auto p = a.at("position").get<std::array<double, 3>>();
      anchor.pose.linear() = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
      anchor.pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
      if (const auto c = a.find("calibration"); c != a.end() && c->is_object()) {
        const double yaw = c->value("yaw", 0.0) * 0.017453292519943295;
        anchor.calibration.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const auto t = c->value("translation", std::array<double, 3>{0, 0, 0});
        anchor.calibration.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
      }
      anchors_[room] = anchor;
    }
  } catch (...) {
    anchors_.clear();
  }
}

void HostFrame::Save() const {
  if (path_.empty()) return;
  nlohmann::json j;
  j["note"] = "openhouse host frame: each room's origin base is reported at this pose, whatever the room does "
              "inside (tools aligning the lighthouse frame count on it). Delete to re-anchor.";
  for (const auto& [room, a] : anchors_) {
    const Eigen::Quaterniond q(a.pose.rotation());
    const Eigen::Vector3d t = a.pose.translation();
    j["rooms"][room] = {{"base", a.base}, {"orientation", {q.w(), q.x(), q.y(), q.z()}}, {"position", {t.x(), t.y(), t.z()}}};
    if (!a.calibration.isApprox(Eigen::Isometry3d::Identity())) {
      const Eigen::Matrix3d r = a.calibration.rotation();
      const double yaw = std::atan2(r(0, 2), r(0, 0)) * 57.29577951308232;
      const Eigen::Vector3d c = a.calibration.translation();
      j["rooms"][room]["calibration"] = {{"yaw", yaw}, {"translation", {c.x(), c.y(), c.z()}}};
    }
  }
  const std::string tmp = path_ + ".tmp";
  {
    std::ofstream f(lh::platform::FsPath(tmp));
    f << j.dump(1);
    f.close();
    if (!f) return;
  }
  // std::rename doesn't replace an existing file on Windows: only the first save took.
  try {
    lh::platform::ReplaceFile(tmp, path_);
  } catch (const std::exception&) {
  }
}

const Eigen::Isometry3d& HostFrame::Update(const SessionSnapshot& snapshot, double now) {
  // Tilt refinements turn the room at 0.05 deg/s after the first level: still enough.
  constexpr double kStillTime = 3.0, kStillRad = 0.3 * 0.017453292519943295;
  const BaseStationState* origin = nullptr;
  for (const auto& b : snapshot.bases) if (b.origin && b.known) origin = &b;
  if (!origin || snapshot.roomId.empty()) {
    anchored_ = false;
    stillPose_.reset();
    rawFromWorld_.setIdentity();
    pinned_.setIdentity();
    return rawFromWorld_;
  }
  const Eigen::Isometry3d worldFromOrigin = FromPose(origin->worldFromBase);
  auto found = anchors_.find(snapshot.roomId);
  if (found == anchors_.end() || found->second.base != origin->serial) {
    // Not anchored yet: report the room as it is, and anchor once the origin has held
    // still for a while (the session's first level turns it in the first seconds).
    const bool still = stillPose_ &&
        Eigen::AngleAxisd(stillPose_->rotation().transpose() * worldFromOrigin.rotation()).angle() < kStillRad &&
        (stillPose_->translation() - worldFromOrigin.translation()).norm() < 1e-4;
    if (!still) { stillPose_ = worldFromOrigin; stillSince_ = now; }
    anchored_ = false;
    rawFromWorld_.setIdentity();
    pinned_.setIdentity();
    if (now - stillSince_ < kStillTime) return rawFromWorld_;
    found = anchors_.insert_or_assign(snapshot.roomId, Anchor{origin->serial, worldFromOrigin}).first;
    Save();
  }
  if (anchored_ && snapshot.originEpoch != originEpoch_ && snapshot.roomId == lastRoom_) {
    // The origin was knocked and the room re-placed it: the room (and the devices in
    // it) didn't move, so neither does the host frame; the anchor follows the origin.
    found->second.pose = pinned_ * worldFromOrigin;
    Save();
  }
  originEpoch_ = snapshot.originEpoch;
  lastRoom_ = snapshot.roomId;
  anchored_ = true;
  pinned_ = LevelPin(found->second.pose, worldFromOrigin);
  rawFromWorld_ = found->second.calibration * pinned_;
  return rawFromWorld_;
}

Eigen::Isometry3d HostFrame::Calibration() const {
  const auto found = anchors_.find(lastRoom_);
  return anchored_ && found != anchors_.end() ? found->second.calibration : Eigen::Isometry3d::Identity();
}

bool HostFrame::SetCalibration(const Eigen::Isometry3d& calibration) {
  const auto found = anchors_.find(lastRoom_);
  if (!anchored_ || found == anchors_.end()) return false;
  // Only a turn about the vertical and a shift: a level room stays level.
  const Eigen::Matrix3d r = calibration.rotation();
  const double yaw = std::atan2(r(0, 2), r(0, 0));
  found->second.calibration.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix();
  found->second.calibration.translation() = calibration.translation();
  rawFromWorld_ = found->second.calibration * pinned_;
  Save();
  return true;
}

}  // namespace lh::openvr
