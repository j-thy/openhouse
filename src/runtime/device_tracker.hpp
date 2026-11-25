// One tracked device on top of the shared room map.
//
// The filter itself is the verified port of CLighthouseDirect (LighthouseDirect).
// What differs from the stock per-device behaviour:
//   * IMU calibration learned in a previous track (states 18..30) is carried into
//     the next one, so a re-acquired device does not recalibrate;
//   * located room bases give the bootstrap its tilt and are added to the filter
//     as known alternate bases as soon as their light is seen;
//   * converged alternate-base estimates are reported back to the room map;
//   * the world pose comes from the room map, so it is valid right after acquiring
//     on any located base.
#pragma once

#include <algorithm>
#include <array>

#include <Eigen/Geometry>
#include <deque>
#include <functional>
#include <map>
#include <set>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lh/session.hpp"
#include "room_map.hpp"
#include "room_solver.hpp"
#include "tracking/lighthouse_direct.hpp"
#include "tracking/tracking_source.hpp"

namespace lh::runtime {

struct TrackerOutput {
  TrackingState state = TrackingState::Uninitialized;
  TrackingResult result = TrackingResult::Uninitialized;
  bool valid = false;
  double deviceTime = 0;
  Eigen::Isometry3d worldFromImu = Eigen::Isometry3d::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(), angularVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  // world, gravity removed (0 if not reported)
  uint32_t primary = 0;
  std::vector<uint32_t> bases;
  std::vector<DevicePose::Coverage> coverage;  // light per base, last 0.5 s
  uint64_t updates = 0;
  Eigen::Vector3d filterPosition = Eigen::Vector3d::Zero();  // world, at filterTime (no prediction)
  double filterTime = 0;
};

class DeviceTracker {
 public:
  static constexpr double kDefaultSmoothing = 0.06;  // s
  // How long a correction from the light takes to show in the output (glide time
  // constant, s), for position and rotation; 0 shows it at once (less latency in fast
  // motion, more visible steps).
  // Whether the output carries the acceleration (hosts predict with it; settings.json
  // "acceleration").
  void SetReportAcceleration(bool on) { reportAcceleration_ = on; }
  // Whether the output holds the pose while the device is physically still (on by
  // default). Off for head-mounted displays: a head sitting still passes the stillness
  // test most of the time, so the view froze to the head and caught up 0.1-0.65 deg
  // whenever a small movement released it, every ~0.5-7 s (22.9 s of 30 s frozen).
  void SetHoldStill(bool on) {
    allowHold_ = on;
    if (!on) heldPose_.reset();
  }
  void SetSmoothing(double position, double rotation) {
    smoothPosition_ = std::max(0.0, position);
    smoothRotation_ = std::max(0.0, rotation);
  }
  // LighthouseDirect::SetMixedLightScatter, kept for every filter this tracker starts.
  void SetMixedLightScatter(double sortedSeconds, double lh1Radians, double wobbleRadians = 0, double wobbleSeconds = 0) {
    mixedScatter_ = {sortedSeconds, lh1Radians, wobbleRadians, wobbleSeconds};
    filter_->SetMixedLightScatter(sortedSeconds, lh1Radians, wobbleRadians, wobbleSeconds);
  }
  // Whether LH1 frames may go into keyframes (runtime::Lh1KeyframesUsable): off while their light may hold
  // 2.0 sweeps this device can't sort.
  void SetLh1Keyframes(bool on) { lh1Keyframes_ = on; }
  // Whether 1.0 frames are checked for 2.0 sweeps that weren't sorted out (runtime::ForeignSweeps): on
  // while 2.0 light is about. ForeignSweepSamples: how many samples that dropped.
  void SetLh1SweepCheck(bool on) { lh1SweepCheck_ = on; }
  uint64_t ForeignSweepSamples() const { return foreignSweepSamples_; }
  // Not stock (runtime/rotor_reference.hpp): a 2.0 rotor's offset at a moment on this device's clock, as
  // the devices in LH2 mode measure it (rad), taken off this device's sorted hits of it; false: none does.
  using RotorOffset = std::function<bool(uint32_t base, double deviceTime, double* radians)>;
  void SetRotorReference(RotorOffset offset) { rotorOffset_ = std::move(offset); }
  // Frames of sorted hits so far, and of those the ones the rotor reference corrected.
  uint64_t SortedFrames() const { return sortedFrames_; }
  uint64_t ReferencedFrames() const { return referencedFrames_; }
  DeviceTracker(std::string id, TrackedObjectModel model, RoomMap* room);

  void OnBaseConfig(const BaseConfig&, const BaseDynamicState&);
  void OnImu(const ImuSample&);
  void OnPulse(const LightPulse&);
  void OnLh2BeamWord(const Lh2BeamWord& w) { source_.OnLh2BeamWord(w); }
  void OnLh2SensorWord(const Lh2SensorWord& w) { source_.OnLh2SensorWord(w); }
  // Runs queued IMU and optical work after the session has processed base identities.
  void Process();

  TrackerOutput Output() const;
  // TDM/disambiguator counters for diagnostics (no effect on tracking).
  const lh::tdm::TdmStats& SourceStats() const { return source_.Stats(); }
  uint64_t FramesHandled() const { return framesHandled_; }
  int BootstrapAttempts() const { return filter_->Info().bootstrapAttempts; }
  int SensorType() const { return model_.sensorType; }
  bool Bootstrapped() const { return filter_->Bootstrapped(); }
  std::vector<std::string> TakeLogs();
  // Keyframes for the room solver collected since the last call.
  std::vector<std::shared_ptr<const Keyframe>> TakeKeyframes();
  std::shared_ptr<const TrackedObjectModel> ImuModel() const { return imuModel_; }
  TrackingSource& Source() { return source_; }

 private:
  friend struct DeviceTrackerTestAccess;  // deterministic health/recovery fault injection
  void ResetFilter();
  void HandleFrame(const OpticalFrame& frame);
  void OnAcquired();
  void SyncRoom();
  void CheckHealth();
  TrackerOutput RawOutput() const;
  void TrackStillness(const ImuSample&);
  void UpdateOutput();
  void CollectKeyframe(const OpticalFrame& frame);
  void DropForeignSweeps(OpticalFrame* frame);
  double Now() const;
  void Log(std::string text) { logs_.push_back(id_ + ": " + text); }

  std::string id_;
  TrackedObjectModel model_;
  RoomMap* room_;
  TrackingSource source_;
  std::unique_ptr<LighthouseDirect> filter_;
  std::array<double, 4> mixedScatter_{};  // SetMixedLightScatter
  std::map<uint32_t, std::pair<BaseConfig, BaseDynamicState>> bases_;
  std::optional<direct::DirectFilter> carry_;  // calibrated filter from the last track
  // The calibration as of the last calm, well-lit moment (no violent motion, light
  // flowing): what a fly-away falls back on, its own having soaked up the error.
  std::optional<direct::DirectFilter> calm_;
  uint64_t calmUpdates_ = 0;
  std::optional<double> latestImu_;
  uint64_t roomRevision_ = ~uint64_t{0};
  uint64_t altSyncRevision_ = 0;
  uint64_t updatesAtAcquire_ = 0;
  bool hadTrack_ = false;
  bool calibratedLogged_ = false;
  std::vector<std::string> logs_;
  std::shared_ptr<const TrackedObjectModel> imuModel_;
  struct BufferedFrame {
    double time;
    uint32_t base;
    int axis;
    uint32_t primary;                     // filter frame of the poses below
    Eigen::Isometry3d primaryFromDevice;  // filter pose at `time`
    std::vector<KeySample> samples;       // point/normal in the primary frame
  };
  std::deque<BufferedFrame> frameBuffer_;
  std::vector<std::shared_ptr<const Keyframe>> keyframes_;
  double lastKeyframe_ = -1e300;
  bool lh1Keyframes_ = true;  // SetLh1Keyframes
  bool lh1SweepCheck_ = false;  // SetLh1SweepCheck
  uint64_t foreignSweepSamples_ = 0;
  RotorOffset rotorOffset_;  // SetRotorReference
  uint64_t sortedFrames_ = 0, referencedFrames_ = 0;
  std::map<uint32_t, double> lastFrame_;  // last sweep time per base
  std::optional<double> preferLocatedSince_;
  uint64_t framesHandled_ = 0;
  int backwardPulses_ = 0;  // consecutive pulses timestamped behind the filter
  // Output stage (UpdateOutput)
  bool predictOutput_ = true, smoothOutput_ = true;
  // Output glide time constants in s (settings.json "smoothing"; 0 = corrections show at once).
  double smoothPosition_ = kDefaultSmoothing, smoothRotation_ = kDefaultSmoothing;
  bool reportAcceleration_ = true;
  std::map<uint32_t, double> lastUsed_;
  // consecutive well-lit frames of a base the filter rejected, per sweep axis: a view of
  // the base turned about one rotor's axis spoils only the other axis's angles
  std::map<std::pair<uint32_t, int>, int> rejectStreak_;
  int refusedLit_ = 0;  // well-lit frames refused since light was last taken
  static constexpr double kRefusedLightReset = 0.5;  // s of refused light before reacquiring
  std::map<uint32_t, int> disagreeing_;   // rejection streaks of an alt base the room and filter agree on
  std::map<uint32_t, std::pair<uint32_t, uint32_t>> altEpochs_;
  std::map<uint32_t, double> loosened_;  // alternate bases loosened for a relearn: since when
  uint32_t reacquireOn_ = 0;              // CheckHealth: the pose is wrong, start over on this base
  std::optional<std::pair<uint32_t, double>> avoidBase_;  // (base, until Now()): don't bootstrap on it
  // Reacquire throttling (not stock). A reacquire throws the filter away and starts
  // over, so one that begins on the base it just began on has gained nothing: the light
  // it rejected is the light it bootstraps on again. Live report 2026-11-09 (5 x LH 2.0,
  // 399 s): 1101 reacquires = 2.75/s, 129 of them onto the same base, one base pair
  // cycling 345 times, 4 of 6 devices affected.
  uint32_t lastReacquireOn_ = 0;           // base the last reacquire started over on
  double lastReacquireAt_ = -1e300;        // Now() of that reacquire
  double reacquireBlockedUntil_ = -1e300;  // Now() before which another is not even queued
  int reacquireBurst_ = 0;                 // reacquires since the tracker last settled
  std::map<uint32_t, std::deque<std::pair<double, int>>> recentUsed_;  // (sync time, sensors) of used frames, 0.5 s
  std::optional<double> switchedAt_;
  double acquiredAt_ = 0;
  double firstTrackedAt_ = 0;  // first acquire since this device attached (keyframe votes)
  static constexpr double kVoteSettle = 10;  // s from it before its keyframes may vote a base moved
  bool multiRoom_ = false;  // a lhbs multi-station frame was seen  // device time of the last (re)acquisition  // device time of the last deliberate primary switch  // base -> sync time of its last frame the filter used
  double sweepInterval_ = 1.0 / 60;  // of the last frame's base
  double inertialLag_ = 0.12;  // s behind the newest IMU sample the filter may advance without light
  TrackerOutput output_, lastRaw_;
  double lastSmoothAt_ = 0;  // Now() of the last smoothed output
  bool haveSmooth_ = false;
  Eigen::Vector3d offsetP_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond offsetQ_ = Eigen::Quaterniond::Identity();
  struct CoastAnchor { Eigen::Vector3d position, velocity; double time; };
  std::optional<CoastAnchor> coastAnchor_;
  // When the filter last took light, in IMU time (Now()). Not the light's own time: LH2
  // frames reach the filter ~50-100 ms after their sweep (a revolution closes on the
  // next beam word, then frames are held to merge channels), which read as darkness.
  double lastOpticalTaken_ = -1e300, lastOpticalSeen_ = -1e300;
  bool holdStill_ = true, still_ = false, haveAccelMean_ = false;
  bool allowHold_ = true;  // SetHoldStill
  Eigen::Vector3d accelMean_ = Eigen::Vector3d::Zero();
  std::optional<double> quietSince_;
  std::optional<Eigen::Isometry3d> heldPose_;
};

}  // namespace lh::runtime
