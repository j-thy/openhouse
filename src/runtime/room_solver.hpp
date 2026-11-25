// Room solver: joint least-squares estimate of all base station poses.
//
// Devices contribute keyframes: short windows (one LH1 cycle) in which a device saw
// at least two base stations with both sweep axes, together with the device's filter
// pose and velocity. The solver estimates
//     world_from_base (every base except the origin, which fixes the gauge)
//     world_from_device_k (one pose per keyframe)
// by minimizing the LH1 sweep-angle residuals of every sample (the same
// CSensorPositionAngular model as the filter, with each base's factory calibration),
// with a robust loss. This is a small bundle adjustment: the keyframe poses are
// eliminated (Schur), so cost grows linearly with the number of keyframes.
//
// Compared with following one filter's alternate-base estimate, every sample from
// every device and position constrains the room, the device poses are re-estimated
// instead of trusted, and outliers are down-weighted.
#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lh/optical.hpp"
#include "tracking/tracked_object.hpp"

namespace lh::runtime {

// One sample with the sensor position/normal expressed in the keyframe's reference
// device frame *at the sample time*: the filter's pose at each sweep plus the
// per-sample time offset are folded in, so the solver needs no motion model.
struct KeySample {
  OpticalSample sample;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

struct Keyframe {
  std::string device;
  double time = 0;                      // device time of the reference pose
  uint32_t primary = 0;                 // filter frame of primaryFromDevice
  double speed = 0, rate = 0;           // m/s, rad/s of the device when taken
  Eigen::Isometry3d primaryFromDevice = Eigen::Isometry3d::Identity();
  std::map<uint32_t, std::vector<KeySample>> samples;  // per base
  bool settled = true;  // calibrated, and tracking for kVoteSettle s: may vote a base moved (deviceSupport)
};

struct SolvedBase {
  Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
  double positionVariance = 0, rotationVariance = 0;  // traces
  int keyframes = 0;
  int fresh = 0;   // of them, new since the last solve
  double rms = 0;  // radians
  int devices = 0;    // distinct devices behind those keyframes
  double span = 0;    // s: longest time one device's keyframes of this base cover
  // Keyframes that saw this base, and how many of them the fit rejected. A moved base's
  // new keyframes are all rejected (they don't fit its old pose), the others' aren't.
  int seen = 0, rejected = 0;
  // Of the devices behind this base's keyframes, how many reject several of them, and how
  // many fit the solved pose clearly better than the room's (`known`) one. A base that
  // really moved shows in more than one device; one device's bad view doesn't.
  int rejectingDevices = 0, checkedDevices = 0, supportingDevices = 0;
  int seeingDevices = 0;  // every device with a keyframe of it, used or rejected (left-out ones aside)
};

struct RoomSolveInput {
  std::vector<std::shared_ptr<const Keyframe>> keyframes;
  std::map<uint32_t, Eigen::Isometry3d> known;        // current room map
  std::map<uint32_t, std::array<double, 14>> calibration;
  std::map<uint32_t, int> baseModels;  // BaseConfig::model; absent = 1 (LH1)
  std::map<std::string, std::shared_ptr<const TrackedObjectModel>> models;  // IMU frame
  uint32_t origin = 0;
  int fresh = -1;           // the last `fresh` keyframes are new since the last solve (< 0: all)
  bool refineBases = true;  // false: evaluate a room (only keyframe poses are solved)
};

struct RoomSolveOutput {
  bool ok = false;
  std::map<uint32_t, SolvedBase> bases;
  int keyframesUsed = 0, keyframesRejected = 0, observations = 0;
  // Devices left out of this solve: their keyframes fit far worse than every other
  // device's (a device with a wrong model or calibration must not move the room).
  std::vector<std::pair<std::string, double>> excludedDevices;  // (device, its median / the others')
  double rms = 0;
  double seconds = 0;
  std::string summary;
};

RoomSolveOutput SolveRoom(const RoomSolveInput& input);

// Diagnostics: the device pose from one base's light of a keyframe alone (base frame),
// as the bootstrap solver finds it; nullopt if it doesn't converge.
std::optional<Eigen::Isometry3d> SolveFromBase(const Keyframe& keyframe, uint32_t base, const TrackedObjectModel& model,
                                               const std::array<double, 14>& calibration, int baseModel = 1);

// Keeps a diverse, bounded set of keyframes.
class KeyframeStore {
 public:
  explicit KeyframeStore(size_t capacity = 400) : capacity_(capacity) {}
  // Returns true if the keyframe was kept. `worldFromDevice` (if known) is used to
  // skip near-duplicates of keyframes the same device already contributed.
  bool Add(std::shared_ptr<const Keyframe> keyframe, const std::optional<Eigen::Isometry3d>& worldFromDevice);
  void DropBase(uint32_t base);
  void Clear() { entries_.clear(); }
  std::vector<std::shared_ptr<const Keyframe>> All() const;
  size_t Size() const { return entries_.size(); }

 private:
  struct Entry {
    std::shared_ptr<const Keyframe> keyframe;
    std::optional<Eigen::Isometry3d> pose;
  };
  size_t capacity_;
  std::vector<Entry> entries_;
};

}  // namespace lh::runtime
