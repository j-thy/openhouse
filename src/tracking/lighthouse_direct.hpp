// Synchronous LH1 orchestration of the ported filter and bootstrap solver.
// Algorithmic baseline; whole-driver parity is not yet established.
#pragma once

#include <array>
#include <deque>
#include <map>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "lh/tracker.hpp"
#include "direct_filter.hpp"
#include "rotor_wobble.hpp"
#include "sample_history.hpp"
#include "tracked_object.hpp"

namespace lh {

struct ExpectedHit;

// Not stock: the filter has four slots, the primary base and three others, and a room of 1.0 and 2.0
// stations has more to see (two 1.0 and three 2.0 here). Whichever lit the device first took the last slot
// and kept it for as long as tracking held, lit or not. A 2018 tracker's last slot went to a 1.0 station that
// then gave it 1-3 frames a second, none used, while the other 1.0 station's 60 frames a second (2-3
// sensors) had no slot and went unread: on one plane of one rotor besides, lying still, it slid 1.8 m in 9 s
// with every frame taken (replay of 2026-11-11 13:02). Which of two 1.0 stations came first hung on a pulse.
//
// So with every slot taken, a base lighting three sensors or more takes the place of the other base whose
// light was last used longest ago (lastUsed, per slot: the last frame of three hits or more the filter
// took), if that is more than kSlotDark ago. The one put out comes back the same way when it lights the
// device again. Returns the slot to give up, 1-3; 0: none.
constexpr double kSlotDark = 2.0;  // s
inline int SlotToGiveUp(const double lastUsed[4], double now) {
  int slot = 1;
  for (int other = 2; other < 4; ++other)
    if (lastUsed[other] < lastUsed[slot]) slot = other;
  return now - lastUsed[slot] > kSlotDark ? slot : 0;
}

class LighthouseDirect {
 public:
  static constexpr double kSaturatedJerkScale = 10;  // process noise while the accelerometer saturates
  explicit LighthouseDirect(TrackedObjectModel model, double opticalTimeout = 0.5);
  // Not stock, off by default (the oracle tests compare against the original):
  // multi-start acquisition (solver::SolveMultiStart) and per-row innovation gating
  // of optical updates at `gateSigma`.
  // Swing handling: saturated IMU axes as one-sided bounds, and the slow states held
  // during violent motion (DirectFilter::holdSlowStates).
  void SetImprovements(bool multiStart, double gateSigma, bool swing = false) {
    multiStart_ = multiStart; filter_.gateSigma = gateSigma; swing_ = swing;
    filter_.gravityTolerance = multiStart ? 0.98 : 0.49033249999999996;
  }
  // Not stock, off by default (0, 0): in a room with both kinds of base station, light in LH1 mode
  // sits about twice as far from the pose as an LH2 word, and weighted as one it shakes the pose.
  // sortedSeconds: added to the timing scatter of a 2.0 hit picked out of LH1 pulses
  // (kSampleSorted); lh1Radians: to the angle scatter of a 1.0 station's hit.
  // wobbleRadians: how far a 2.0 rotor's sorted hits wander off the pose all alike, which is followed
  // per rotor and taken out of them (rotor_wobble.hpp), over wobbleSeconds; 0: not followed.
  void SetMixedLightScatter(double sortedSeconds, double lh1Radians, double wobbleRadians = 0, double wobbleSeconds = 0) {
    sortedScatter_ = sortedSeconds;
    lh1Scatter_ = lh1Radians;
    wobbleSpread_ = wobbleSeconds > 0 ? wobbleRadians : 0;
    wobbleMemory_ = wobbleSeconds;
    if (wobbleSpread_ <= 0) wobbles_.clear();
  }
  void OnBaseConfig(const BaseConfig&, const BaseDynamicState&);
  void SetRecordedTilt(uint32_t serial, std::array<double, 2> tilt, double variance);
  void OnImu(const ImuSample&);
  bool OnFrame(const OpticalFrame&);
  // Consume inertial data while optical data is unavailable. This is an explicit
  // fallback policy, not the original CImuOnlyTracker implementation.
  bool AdvanceInertial(double time);
  // Time between sweeps of the bases seen (1/60 stock; N/60 for a lhbs multi-station
  // slave): advancing on the IMU alone waits at least this long for light.
  void SetSweepInterval(double seconds) { sweepInterval_ = seconds; }
  std::optional<TrackedPose> Pose() const;
  // Not stock: the filter state carried forward through the IMU samples queued
  // since its last update (strapdown with the filter's own biases, scales, IMU
  // rotation and gravity), to the newest one. The filter only advances on optical
  // frames, so its own pose is 10-30 ms old and moves in bursts; this is the pose now.
  struct Predicted {
    double time = 0;
    double p[3] = {}, v[3] = {}, q[4] = {1, 0, 0, 0}, w[3] = {};  // base frame; w body
    double a[3] = {};  // base frame, gravity removed: the newest sample's (or the state's)
  };
  Predicted PredictNow() const;
  // Not stock, for sorting LH1 pulses (tracking/pulse_sorter.hpp): where the pose, carried
  // forward like PredictNow's, puts this sensor's hit at time t, for both planes of every base
  // in the filter. Returns how many entries (8 at most); none without a pose worth asking.
  int ExpectedHits(int sensor, double t, ExpectedHit* out) const;
  DeviceTrackingInfo Info() const;
  const direct::DirectFilter& Filter() const { return filter_; }
  // Mutable access for deterministic fault-injection tests only.
  direct::DirectFilter& FilterForTest() { return filter_; }
  // Read-only synchronous diagnostics, also used by the original-function oracle.
  const TrackedObjectModel& Model() const { return model_; }
  const std::deque<ImuSample>& PendingImu() const { return imu_; }
  uint32_t BaseSerial(int slot) const { return slot >= 0 && slot < 4 ? slots_[slot] : 0; }
  double LastOpticalTime() const { return lastOpticalTime_; }
  size_t PrimarySwitches() const { return primarySwitches_; }
  size_t SecondaryBases() const { return secondaryBases_; }
  uint32_t Counters() const { return filter_.flags; }
  // Process the buffered IMU prefix before a measurement time without adding a
  // measurement; useful to independently verify prediction/update scheduling.
  bool ProcessImuUntil(double time) { return PredictTo(time); }

  // ---- hooks for the shared room map (not part of the stock driver) ----------
  // Pose estimate of an alternate base relative to the primary, with the
  // translation / rotation variance traces the original uses for RELATIONSHIP
  // reports (0x3509e0: < 6.4e-5 and < 1.22e-5 count as converged).
  struct AltEstimate {
    uint32_t serial = 0;
    Eigen::Isometry3d altFromPrimary = Eigen::Isometry3d::Identity();
    double translationVariance = 0, rotationVariance = 0;
  };
  std::vector<AltEstimate> AltEstimates() const;
  // Start tracking against `base` from a known pose instead of a cold bootstrap.
  // `carry` (optional) supplies IMU calibration states 18..30 and their covariance
  // from a previous track of the same device; `tilt` is the base tilt (x16, x17).
  struct Seed {
    uint32_t base = 0;
    double time = 0;
    Eigen::Isometry3d baseFromImu = Eigen::Isometry3d::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // base frame
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();  // IMU frame
    double positionVariance = 1e-4, rotationVariance = 1e-3, velocityVariance = 0.04;
    std::array<double, 2> tilt{};
    double tiltVariance = 1e-4;
    const direct::DirectFilter* carry = nullptr;
    bool calibrated = false;  // start past the 200-update calibration phase
  };
  bool SeedPose(const Seed& seed);
  // Add an alternate base whose pose relative to the primary is already known. Not stock: with every slot
  // taken and `lit` (the base lights the device now), in place of a base gone dark (SlotToGiveUp), which
  // *replaced then names.
  bool AddKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                       double translationVariance, double rotationVariance, bool lit = false,
                       uint32_t* replaced = nullptr);
  // Overwrite an alternate base already in the filter (e.g. after the room solver
  // refined it). Cross-covariances with the rest of the state are cleared.
  void SetAltBaseRelearn(uint32_t serial, bool relearn);  // see DirectFilter::altBaseRelearn
  bool SetKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                       double translationVariance, double rotationVariance);
  // Stock reset checks (0x358300). Returns the reason the original would log after
  // "Resetting tracking:", or "" if tracking is healthy. `now` is device time.
  std::string HealthCheck(double now) const;
  bool Bootstrapped() const { return slots_[0] != 0; }
  double Time() const { return time_; }
  std::optional<double> ObservedTime() const { return observedTime_; }

 private:
  bool Bootstrap(const OpticalFrame&, const BaseConfig&);
  bool CanSwitchPrimary(uint32_t serial) const;
  bool ApplyOpticalResiduals(const OpticalFrame&, const BaseConfig&, int slot);
  bool OpticalUpdate(const OpticalFrame&, const BaseConfig&, int slot);
  bool ImuUpdate(const ImuSample&);
  bool PredictTo(double time);
  int Slot(uint32_t serial) const;
  double BaseDistance(const double* state, int slot) const;
  std::vector<OpticalSample> Gather(const std::vector<OpticalSample>&) const;
  void FilterOutliers(std::vector<OpticalSample>* samples, double distance) const;
  void FollowRotorWobble(uint32_t serial, double time, const std::vector<direct::Residual>& residuals,
                         const std::vector<bool>& sorted, bool referenced, std::vector<double>* z, const direct::Mat& R);

  TrackedObjectModel model_;
  direct::DirectFilter filter_;
  struct Base { BaseConfig config; BaseDynamicState dynamic; };
  std::map<uint32_t, Base> bases_;
  std::map<uint32_t, SampleHistory> history_;
  struct RecordedTilt { std::array<double, 2> angles; double variance; };
  std::map<uint32_t, RecordedTilt> recordedTilts_;
  std::deque<ImuSample> imu_;
  uint32_t slots_[4]{};
  double slotUsed_[4] = {-1e300, -1e300, -1e300, -1e300};  // SlotToGiveUp: or when the base got the slot
  double time_ = 0, lastOpticalTime_ = 0, opticalTimeout_ = 0.5;
  bool swing_ = false;
  double violentUntil_ = -1e300;  // IMU calibration held until (freezeImuCalibration)
  // Vibration (#90): the accelerometer's first and second differences between samples,
  // averaged over ~0.25 s. Jitter over kVibrationJitter (m/s2) with roughness over
  // kVibrationRoughness times it holds gravity and the IMU calibration, and scales the
  // accelerometer's variance by kVibrationAccelScale.
  double accelJitter_ = 0, accelRoughness_ = 0;
  double vibratingUntil_ = -1e300, vibrationAccelScale_ = 1.0;
  std::array<std::array<double, 3>, 2> prevAccel_{};  // last sample, the one before
  int havePrevAccel_ = 0;
  static constexpr double kVibrationJitter = 1.5, kVibrationRoughness = 1.4, kVibrationAccelScale = 100;
  double sweepInterval_ = 1.0 / 60;
  std::optional<double> observedTime_;
  uint64_t bootstrapAttempts_ = 0;
  std::map<uint32_t, double> lastMultiStart_, multiStartWait_;  // per base
  bool multiStart_ = false;  // SetImprovements
  double diameter_ = 0, radius_ = 0;
  bool calibrationSeen_ = false;  // filter_.flags & 0x100
  size_t primarySwitches_ = 0, secondaryBases_ = 0;
  uint64_t altGateUntil_ = 0;  // after a reseed: the other bases' light isn't frame-gated before this
  double lastWellLit_ = -1e300;  // the last frame of four sensors or more that was used
  double sortedScatter_ = 0, lh1Scatter_ = 0;  // SetMixedLightScatter
  double wobbleSpread_ = 0, wobbleMemory_ = 0;
  std::map<uint32_t, direct::RotorWobble> wobbles_;  // per 2.0 station (FollowRotorWobble)
  // ExpectedHits: the state carried forward, kept for a moment since it's asked per pulse.
  mutable double expectedState_[direct::kStateSize] = {};
  mutable double expectedTime_ = 0, expectedFilterTime_ = -1;
};

}  // namespace lh
