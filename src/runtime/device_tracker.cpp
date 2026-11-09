#include "device_tracker.hpp"

#include "tracking/lighthouse_model.hpp"

#include <algorithm>
#include <set>
#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace lh::runtime {
namespace {
constexpr double kRelationshipTranslationVariance = 6.4e-05;   // 0x1a98e8
constexpr double kRelationshipRotationVariance = 1.2184696791468344e-05;  // 0x1a96c8
constexpr double kOutOfRangeAge = 0.25;                         // 0x1a9bc8 (status flag 8)
// Prior variance (trace) for room-map bases entering a filter. Tuned on the static
// controller recording: 1e-6 -> 0.34 mm std, 4e-4 -> 0.13 mm, 1e-3 -> 0.10 mm.
constexpr double kAltPriorVariance = 4e-4;
constexpr int kRejectStreak = 20;  // ~0.3 s of a base's well-lit frames all rejected
// Consecutive light pulses timestamped behind the filter that mean a new epoch
// (~0.5 s of light from one base).
constexpr int kBackwardPulseReset = 1000;
// Optical rows beyond this many sigmas are left out of an update (DirectFilter::gateSigma).
constexpr double kGateSigma = 4.0;

Eigen::Isometry3d BaseFromImu(const double* x) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x[0], x[1], x[2]);
  t.linear() = Eigen::Quaterniond(x[9], x[10], x[11], x[12]).normalized().toRotationMatrix();
  return t;
}

std::array<double, 2> RoomTilt(const RoomBase& base) {
  return TiltFromUp(base.worldFromBase.rotation().transpose() * Eigen::Vector3d::UnitY());
}
}  // namespace

DeviceTracker::DeviceTracker(std::string id, TrackedObjectModel model, RoomMap* room)
    : id_(std::move(id)), model_(std::move(model)), room_(room), source_(model_.sensorType, {}, !model_.sensorTypeKnown) {
  imuModel_ = std::make_shared<const TrackedObjectModel>(model_.InImuFrame());
  ResetFilter();
}

void DeviceTracker::ResetFilter() {
  filter_ = std::make_unique<LighthouseDirect>(model_);
  refusedLit_ = 0;
  // LH_STOCK_FILTER: the verified stock behaviour, for A/B comparisons.
  static const bool stock = std::getenv("LH_STOCK_FILTER") != nullptr;
  predictOutput_ = smoothOutput_ = holdStill_ = !stock;
  inertialLag_ = stock ? 0.02 : 0.12;
  if (!stock) filter_->SetImprovements(true, kGateSigma, true);
  for (const auto& [serial, base] : bases_) filter_->OnBaseConfig(base.first, base.second);
  roomRevision_ = ~uint64_t{0};
  calibratedLogged_ = false;
  frameBuffer_.clear(); keyframes_.clear();
  lastKeyframe_ = -1e300;
  lastFrame_.clear(); preferLocatedSince_.reset();
  altSyncRevision_ = ~uint64_t{0};
  backwardPulses_ = 0;
  rejectStreak_.clear();
  disagreeing_.clear();
  altEpochs_.clear();
  loosened_.clear();
}

void DeviceTracker::OnBaseConfig(const BaseConfig& config, const BaseDynamicState& dynamic) {
  bases_[config.serial] = {config, dynamic};
  filter_->OnBaseConfig(config, dynamic);
}

void DeviceTracker::OnImu(const ImuSample& sample) {
  if (latestImu_ && sample.time < *latestImu_) return;
  latestImu_ = sample.time;
  TrackStillness(sample);
  source_.OnImu(sample);
}

void DeviceTracker::OnPulse(const LightPulse& pulse) {
  source_.OnLightPulse(pulse);
  // Light that keeps arriving with timestamps well behind the filter is a new epoch
  // (device or receiver power cycle). No light at all is an occlusion, which the
  // filter's own 2 s optical timeout handles.
  if (filter_->Bootstrapped() && pulse.time < filter_->Time() - 0.1) ++backwardPulses_;
  else backwardPulses_ = 0;
}

double DeviceTracker::Now() const { return latestImu_.value_or(filter_->Time()); }

void DeviceTracker::Process(std::vector<std::pair<BaseConfig, BaseDynamicState>>* decoded) {
  BaseConfig config;
  BaseDynamicState dynamic;
  while (source_.PopBaseConfig(&config, &dynamic)) {
    if (decoded) decoded->push_back({config, dynamic});
    OnBaseConfig(config, dynamic);
  }
  ImuSample imu;
  while (source_.PopImu(&imu)) filter_->OnImu(imu);
  OpticalFrame frame;
  while (source_.PopFrame(&frame)) { ++framesHandled_; HandleFrame(frame); }
  // Without light the filter runs on the IMU up to `inertialLag_` behind the newest
  // sample. Stock keeps it 20 ms behind because its output is the filter state; ours
  // predicts to now from the queued IMU (UpdateOutput), so the filter can wait for
  // late optical frames (20-25 ms typical, a tail past 50 ms) instead of having
  // already passed them: those were dropped, ~1% of all frames.
  if (latestImu_ && filter_->Bootstrapped()) filter_->AdvanceInertial(*latestImu_ - inertialLag_);
  std::string text;
  while (source_.PopLog(&text)) {
    while (!text.empty() && text.back() == '\n') text.pop_back();
    Log(text);
  }
  SyncRoom();
  CheckHealth();
  UpdateOutput();
}

void DeviceTracker::HandleFrame(const OpticalFrame& original) {
  const OpticalFrame& frame = original;
  const bool wasTracking = filter_->Bootstrapped();

  if (frame.baseSerial) lastFrame_[frame.baseSerial] = frame.syncTime;
  // Frames carry the time between their base's sweeps in idealPeriod (1/60 stock).
  sweepInterval_ = frame.idealPeriod > 0 ? frame.idealPeriod : sweepInterval_;
  multiRoom_ |= frame.idealPeriod > 0.02;  // multi-station frames carry N/60
  filter_->SetSweepInterval(sweepInterval_);
  if (!wasTracking && avoidBase_ && frame.baseSerial == avoidBase_->first) {
    // Reacquiring after an inconsistent pose: the other base first (up to 1 s).
    if (Now() < avoidBase_->second) return;
    avoidBase_.reset();
  }
  if (!wasTracking && frame.baseSerial && !room_->Base(frame.baseSerial)) {
    // Bootstrapping on an unlocated base gives no world pose until the room learns
    // it; if a located base is in view, wait for its sweeps instead.
    // Wait at most 1 s: a located base that is visible but too weak to bootstrap on
    // must not block tracking altogether.
    bool locatedVisible = false;
    for (const auto& [base, time] : lastFrame_)
      locatedVisible |= base != frame.baseSerial && room_->Base(base) && frame.syncTime - time < 0.3;
    if (locatedVisible) {
      if (!preferLocatedSince_) preferLocatedSince_ = frame.syncTime;
      if (frame.syncTime - *preferLocatedSince_ < 1.0) return;
    }
  } else if (wasTracking) {
    preferLocatedSince_.reset();
    avoidBase_.reset();
  }
  if (wasTracking && frame.baseSerial != 0) {
    // A located base enters the filter with its room-map pose instead of waiting
    // for the stock secondary bootstrap (which needs 260 updates and a fresh solve).
    bool inFilter = false;
    for (int slot = 0; slot < 4; ++slot) inFilter |= filter_->BaseSerial(slot) == frame.baseSerial;
    const RoomBase* primary = room_->Base(filter_->BaseSerial(0));
    const RoomBase* alt = room_->Base(frame.baseSerial);
    if (!inFilter && primary && alt && alt->positionVariance < 0.01) {
      const Eigen::Isometry3d altFromPrimary = alt->worldFromBase.inverse() * primary->worldFromBase;
      // The room pose is a starting point, not a constraint: the filter keeps refining
      // the relative pose with its own measurements. Locking it to the room (tiny
      // variance) measurably increases pose jitter (see ai/driver/lh_runtime.md).
      if (filter_->AddKnownAltBase(frame.baseSerial, altFromPrimary,
                                   std::max(alt->positionVariance + primary->positionVariance, kAltPriorVariance),
                                   std::max(alt->rotationVariance + primary->rotationVariance, kAltPriorVariance))) {
        char text[96];
        std::snprintf(text, sizeof text, "added base %08X from the room map", frame.baseSerial);
        Log(text);
      }
    }
  }
  const int rejectsBefore = filter_->Filter().rejectCount;
  const double lastOpticalBefore = filter_->LastOpticalTime();
  filter_->OnFrame(frame);
  if (frame.baseSerial && wasTracking) {
    std::set<int> lit;
    for (const auto& s : frame.samples) if (s.sensor >= 0) lit.insert(s.sensor);
    if (lit.size() >= 4) {
      int& streak = rejectStreak_[{frame.baseSerial, frame.axis & 1}];
      if (filter_->LastOpticalTime() == lastOpticalBefore) ++refusedLit_;
      else refusedLit_ = 0;
      if (filter_->Filter().rejectCount != rejectsBefore) {
        ++streak;
        // Say why the light is being refused, once per streak. Without this a log can
        // only report "keeps rejecting" (report 2026-11-09: 1092 reacquires in 399 s,
        // and nothing to distinguish a bad calibration from a reflection from a moved
        // base). The limits themselves are unchanged; only the reason is recorded.
        if (streak == kRejectStreak) {
          const auto& f = filter_->Filter();
          char text[200];
          std::snprintf(text, sizeof text, "base %08X: %d well-lit frames rejected in a row (%s, %d sensor rows)",
                        frame.baseSerial, streak, direct::RejectReasonName(f.lastRejectReason), f.lastRejectRows);
          Log(text);
        }
      } else if (filter_->LastOpticalTime() != lastOpticalBefore) {
        streak = 0;
        if (rejectStreak_[{frame.baseSerial, (frame.axis & 1) ^ 1}] == 0) disagreeing_[frame.baseSerial] = 0;
      }
    }
  }
  if (frame.baseSerial && filter_->LastOpticalTime() != lastOpticalBefore) {
    lastUsed_[frame.baseSerial] = frame.syncTime;
    std::set<int> lit;
    for (const auto& s : frame.samples) lit.insert(s.sensor);
    auto& recent = recentUsed_[frame.baseSerial];
    recent.push_back({frame.syncTime, int(lit.size())});
    while (!recent.empty() && recent.front().first < frame.syncTime - 2) recent.pop_front();
  }
  if (!wasTracking && filter_->Bootstrapped()) OnAcquired();
  CollectKeyframe(frame);
}

void DeviceTracker::CollectKeyframe(const OpticalFrame& frame) {
  if (!filter_->Bootstrapped() || frame.baseSerial == 0) { frameBuffer_.clear(); return; }
  const auto& f = filter_->Filter();
  const uint32_t primary = filter_->BaseSerial(0);
  if (f.updateCount < 110 || Now() - filter_->LastOpticalTime() > 0.05) { frameBuffer_.clear(); return; }
  if (!frameBuffer_.empty() && frameBuffer_.back().primary != primary) frameBuffer_.clear();
  // Pose of the device at time t from the filter state right now (the same
  // extrapolation the filter's residuals use, 0x376380).
  auto poseAt = [&](double t) {
    double pos[3], q[4];
    model::ExtrapolatePose(f.x, t - filter_->Time(), pos, q);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
    pose.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    return pose;
  };
  // Same sample hygiene as the filter's gather step (0x35b380).
  int counts[2][32]{};
  for (const auto& s : frame.samples)
    if (s.sensor >= 0 && s.sensor < 32 && s.axis >= 0 && s.axis < 2) ++counts[s.axis][s.sensor];
  double frameTime = 1e300;
  for (const auto& s : frame.samples) frameTime = std::min(frameTime, s.time + 0.5 * s.width);
  BufferedFrame bf{frameTime, frame.baseSerial, frame.axis, primary, poseAt(frameTime), {}};
  for (const auto& s : frame.samples) {
    if (s.sensor < 0 || s.sensor >= 32 || s.axis < 0 || s.axis > 1 || counts[s.axis][s.sensor] != 1) continue;
    const int idx = imuModel_->IndexOf(s.sensor);
    if (idx < 0 || !std::isfinite(s.angle) || !std::isfinite(s.time)) continue;
    if (!(1.0610329539459692e-6 <= s.width && s.width <= 0.00013262911924324613)) continue;
    const Eigen::Isometry3d pose = poseAt(s.time + 0.5 * s.width);
    KeySample k;
    k.sample = s;
    k.point = pose * Eigen::Vector3d(imuModel_->points[size_t(idx)].data());
    k.normal = pose.rotation() * Eigen::Vector3d(imuModel_->normals[size_t(idx)].data());
    bf.samples.push_back(k);
  }
  if (bf.samples.size() < 3) return;
  frameBuffer_.push_back(std::move(bf));
  while (!frameBuffer_.empty() && frameBuffer_.back().time - frameBuffer_.front().time > 0.05) frameBuffer_.pop_front();
  // Newest sweep per base and axis only, so a keyframe spans less than one LH1 cycle.
  std::map<std::pair<uint32_t, int>, const BufferedFrame*> newest;
  for (const auto& b : frameBuffer_) newest[{b.base, b.axis}] = &b;
  std::map<uint32_t, std::array<int, 2>> hits;
  for (const auto& [key, b] : newest) hits[key.first][size_t(key.second & 1)] += int(b->samples.size());
  int complete = 0;
  for (const auto& [base, h] : hits) complete += h[0] >= 4 && h[1] >= 4;
  if (complete < 2) return;
  const Eigen::Vector3d v(f.x[3], f.x[4], f.x[5]), w(f.x[13], f.x[14], f.x[15]);
  // Very fast motion makes the per-sample poses less certain than the samples.
  if (filter_->Time() - lastKeyframe_ < 0.1 || v.norm() > 2.0 || w.norm() > 6.0) return;
  const BufferedFrame& ref = frameBuffer_.back();
  const Eigen::Isometry3d deviceFromPrimary = ref.primaryFromDevice.inverse();
  auto k = std::make_shared<Keyframe>();
  k->device = id_;
  k->time = ref.time;
  k->primary = primary;
  k->speed = v.norm();
  k->rate = w.norm();
  k->primaryFromDevice = ref.primaryFromDevice;
  for (const auto& [key, b] : newest) {
    if (hits[key.first][0] < 4 || hits[key.first][1] < 4) continue;
    auto& out = k->samples[key.first];
    for (KeySample ks : b->samples) {
      ks.point = deviceFromPrimary * ks.point;
      ks.normal = deviceFromPrimary.rotation() * ks.normal;
      out.push_back(ks);
    }
  }
  keyframes_.push_back(std::move(k));
  lastKeyframe_ = filter_->Time();
  frameBuffer_.clear();
}

std::vector<std::shared_ptr<const Keyframe>> DeviceTracker::TakeKeyframes() {
  std::vector<std::shared_ptr<const Keyframe>> out;
  out.swap(keyframes_);
  return out;
}

void DeviceTracker::OnAcquired() {
  acquiredAt_ = Now();
  if (carry_ && !carry_->CalibrationHealthy()) {
    Log("discarding invalid cached IMU calibration before reacquisition");
    carry_.reset();
  }
  const auto& f = filter_->Filter();
  const uint32_t primary = filter_->BaseSerial(0);
  const RoomBase* base = room_->Base(primary);
  if (carry_ || base) {
    LighthouseDirect::Seed seed;
    seed.base = primary;
    seed.time = filter_->Time();
    seed.baseFromImu = BaseFromImu(f.x);
    seed.velocity = Eigen::Vector3d(f.x[3], f.x[4], f.x[5]);
    seed.angularVelocity = Eigen::Vector3d(f.x[13], f.x[14], f.x[15]);
    seed.positionVariance = f.P(0, 0);
    seed.rotationVariance = f.P(9, 9);
    seed.velocityVariance = f.P(3, 3);
    if (base) {
      seed.tilt = RoomTilt(*base);
      seed.tiltVariance = std::max(base->rotationVariance, 1e-6);
    } else {
      seed.tilt = {f.x[16], f.x[17]};
      seed.tiltVariance = f.P(16, 16);
    }
    seed.carry = carry_ ? &*carry_ : nullptr;
    seed.calibrated = carry_ && carry_->updateCount > 200;
    filter_->SeedPose(seed);
    calibratedLogged_ = seed.calibrated;
  }
  updatesAtAcquire_ = filter_->Filter().updateCount;
  char text[200];
  std::snprintf(text, sizeof text, "----- %s base %08X distance %.2fm%s%s -----", hadTrack_ ? "REACQUIRED" : "BOOTSTRAPPED",
                primary, BaseFromImu(f.x).translation().norm(), base ? " (located)" : "",
                carry_ ? " (calibration carried over)" : "");
  Log(text);
  hadTrack_ = true;
}

void DeviceTracker::SyncRoom() {
  if (!filter_->Bootstrapped()) {
    // Bootstrap tilt for located bases.
    if (room_->Revision() != roomRevision_) {
      roomRevision_ = room_->Revision();
      for (const auto& b : room_->Bases()) filter_->SetRecordedTilt(b.serial, RoomTilt(b), std::max(b.rotationVariance, 1e-6));
    }
    return;
  }
  const auto& f = filter_->Filter();
  const uint32_t primary = filter_->BaseSerial(0);
  const bool calibrated = f.updateCount > 200;
  if (calibrated && !calibratedLogged_) {
    calibratedLogged_ = true;
    char text[160];
    std::snprintf(text, sizeof text, "----- CALIBRATED base %08X at pitch %.2f deg roll %.2f deg -----", primary,
                  f.x[16] * 57.29577951308232, f.x[17] * 57.29577951308232);
    Log(text);
  }
  // The origin is set as soon as a device tracks a base (tilt from the base's own
  // accelerometer or a recorded tilt), then re-levelled when the filter's tilt
  // estimate becomes clearly better, instead of waiting for "CALIBRATED".
  if (room_->Active() && room_->Empty()) {
    // The base's accelerometer (OOTX) is within ~0.5 deg of the calibrated tilt;
    // the filter's early tilt is not (initial variance ~1 rad^2).
    const auto base = bases_.find(primary);
    const auto& g = base != bases_.end() ? base->second.second.accelDir : std::array<float, 3>{};
    const Eigen::Vector3d up(g[0], g[1], g[2]);
    if (up.norm() > 0.5) room_->EstablishOrigin(primary, TiltFromUp(up), 1.5e-4);
    else if (calibrated) room_->EstablishOrigin(primary, {f.x[16], f.x[17]}, f.P(16, 16));
  }
  if (calibrated && primary == room_->Origin()) room_->RefineOriginTilt(primary, {f.x[16], f.x[17]}, f.P(16, 16), filter_->Time());
  // Room-solver bases are the joint estimate from every device: keep the filter's
  // alternate-base states on them where the room knows better. Also when the filter
  // keeps rejecting a base's light: its own view of that base is wrong, and since a
  // rejected frame teaches it nothing it would stay wrong (the left controller of
  // session-post rejected 14 % of A101B835's frames for 50 s while the room was still
  // under the old fixed 5 mm confidence gate).
  bool rejecting = false;
  for (const auto& [key, streak] : rejectStreak_) rejecting |= streak >= kRejectStreak;
  if (room_->Revision() != altSyncRevision_ || rejecting) {
    altSyncRevision_ = room_->Revision();
    const RoomBase* p = room_->Base(primary);
    for (const auto& e : filter_->AltEstimates()) {
      const RoomBase* a = room_->Base(e.serial);
      // The origin is never "solved" (the solver measures every other base against
      // it), but a filter whose alternate base is the origin still needs the solved
      // primary's correction: without it the left controller of session-ms-01 rejected
      // a third of that base's frames for two minutes.
      // A base of the pair knocked and being relearned: the room's relationship is
      // stale and the filter's own, held confidently, makes it reject the base's light
      // (ms4-01: a third of the knocked origin's frames, for minutes, the controller
      // too still for the solver to gather keyframes). Loosen it once (5 cm, 2 deg) so
      // the filter relearns the pair from the light itself.
      const bool relearning = (p && p->relearn) || (a && a->relearn) ||
                              (room_->OriginRelearning() && ((p && p->origin) || (a && a->origin)));
      // For 10 s from the knock: then the filter has walked there, and taking oversized
      // steps any longer lets misattributed frames (a round change) drag it away.
      const auto since = loosened_.find(e.serial);
      filter_->SetAltBaseRelearn(e.serial, relearning && (since == loosened_.end() || Now() - since->second < 10));
      if (!relearning) loosened_.erase(e.serial);
      else if (loosened_.emplace(e.serial, Now()).second) {
        filter_->SetKnownAltBase(e.serial, e.altFromPrimary, 3 * 0.05 * 0.05, 3 * 0.035 * 0.035);
        char text[128];
        std::snprintf(text, sizeof text, "base %08X or %08X relearning in the room: loosened their relationship", primary, e.serial);
        Log(text);
        continue;
      }
      if (relearning) continue;  // the room's relationship is stale until re-placed
      const auto placed = [](const RoomBase* x) { return x && (x->solved || x->origin); };
      if (!placed(p) || !placed(a) || !(p->solved || a->solved)) continue;
      const double roomVariance = p->positionVariance + a->positionVariance;
      const double roomSigma = std::sqrt(roomVariance / 3);
      const Eigen::Isometry3d target = a->worldFromBase.inverse() * p->worldFromBase;
      const double dp = (target.translation() - e.altFromPrimary.translation()).norm();
      const double dr = Eigen::AngleAxisd(target.rotation().transpose() * e.altFromPrimary.rotation()).angle();
      int& streak0 = rejectStreak_[{e.serial, 0}];
      int& streak1 = rejectStreak_[{e.serial, 1}];
      const int streak = std::max(streak0, streak1);
      // A pair with the origin is synced routinely only once the room is sure of it
      // (< 5 mm): syncing every device onto a still-converging solve against the
      // origin kept vrchat-01's devices rejecting 5-10 % of the light for minutes (0 %
      // without). A filter rejecting the light still takes it.
      if ((p->origin || a->origin) && roomSigma >= 0.005 && streak < kRejectStreak) continue;
      // Either base re-placed by the room since (moved, knocked): this device's view of
      // the pair is from before; take the room's at once.
      const std::pair<uint32_t, uint32_t> epochs{p->epoch, a->epoch};
      const auto [known, inserted] = altEpochs_.try_emplace(e.serial, epochs);
      const bool replaced = !inserted && known->second != epochs;
      known->second = epochs;
      bool apply;
      if (replaced) {
        apply = dp > 0.002 || dr > 0.0005;
        if (apply) {
          char text[160];
          std::snprintf(text, sizeof text, "base %08X or %08X re-placed in the room: their relationship taken (%.1f mm, %.2f deg)",
                        primary, e.serial, dp * 1e3, dr * 57.2958);
          Log(text);
        }
      } else if (streak >= kRejectStreak) {
        // Only where the room clearly disagrees with the filter: otherwise the light is
        // rejected for some other reason (a reflection, a bad calibration).
        apply = dp > 0.01 || dr > 0.0087;
        // The filter already agrees with a confident room and still rejects the
        // light: the device's pose is what's wrong (session-post: a left controller
        // bootstrapped on E236DC40 into a pose its A101B835 light never fit, rejected
        // for 40 s while it drifted, until it ran out of bounds).
        if (!apply && roomSigma < 0.005 && ++disagreeing_[e.serial] >= 2) reacquireOn_ = e.serial;
        streak0 = streak1 = 0;
      } else {
        // The room is the joint view of every device: a filter clearly off it (> 1 cm,
        // 0.5 deg) takes it; smaller differences are the filter's own, smoother
        // refinement. (Leaving each device its own view, as stock does, left three of
        // vrchat-01's five devices rejecting a fifth of one base's light for minutes.)
        apply = dp > 0.01 || dr > 0.0087;
      }
      if (apply) disagreeing_[e.serial] = 0;
      if (apply)
        filter_->SetKnownAltBase(e.serial, target, std::max(roomVariance, kAltPriorVariance),
                                 std::max(p->rotationVariance + a->rotationVariance, kAltPriorVariance));
    }
  }
  if (!calibrated) return;
  for (const auto& e : filter_->AltEstimates()) {
    if (e.translationVariance < kRelationshipTranslationVariance && e.rotationVariance < kRelationshipRotationVariance)
      room_->Observe(id_, primary, e.serial, e.altFromPrimary, e.translationVariance, e.rotationVariance, filter_->Time());
  }
}

void DeviceTracker::CheckHealth() {
  if (!filter_->Bootstrapped()) return;
  {
    const auto& f = filter_->Filter();
    // Every 100 optical updates of calm tracking (about 2 s), the calibration is kept.
    if (f.holdSlowStates || f.updateCount < 200 || !f.CalibrationHealthy()) calmUpdates_ = f.updateCount;
    else if (f.updateCount >= calmUpdates_ + 100) { calm_ = f; calmUpdates_ = f.updateCount; }
  }
  // The stock 0x358300 check assumes device timestamps advance. After a power
  // cycle the receiver can still deliver a new epoch's light pulses whose ticks
  // are behind the last optical time, so elapsed device time never grows; once a
  // sustained run of such light has arrived, start over in the new epoch.
  // (This used to compare the newest pulse with the filter time, which the inertial
  // advance passes 50 ms into any occlusion: every hidden device was reset after
  // 0.5 s, its calibration thrown away, instead of the stock 2 s with carry-over.)
  const bool stale = backwardPulses_ >= kBackwardPulseReset / 4;
  if (backwardPulses_ >= kBackwardPulseReset) {
    Log("no forward light timestamps since reset; restarting acquisition");
    const auto& f = filter_->Filter();
    if (f.updateCount > 200 && f.CalibrationHealthy()) carry_ = f;  // the IMU's calibration outlives the epoch
    ResetFilter();
    return;
  }
  // Not stock: the primary base dark while another base in the filter is lit. Its
  // frames keep coming as alternate-base updates, but with the primary unseen the
  // device pose and the alternate base's pose relative to the primary drift together
  // unconstrained (captures/ms-n2-all: a still controller ran off at 1.4 m/s). Start
  // over on the visible base instead; the IMU calibration is kept and the output
  // stage smooths the switch (reacquired in ~70 ms there).
  // Multi-station rooms only: on vrchat-01 (stock, 2 bases) switching fired 47 times
  // and the bases' room poses disagreeing by centimetres made each a jump (spikes > 5
  // cm 0 -> 13); the stock path rides those stretches out better there.
  static const bool stock = std::getenv("LH_STOCK_FILTER") != nullptr;
  if (reacquireOn_) {
    const uint32_t on = reacquireOn_, primary = filter_->BaseSerial(0);
    reacquireOn_ = 0;
    disagreeing_.clear();
    if (!stock) {
      char text[160];
      std::snprintf(text, sizeof text, "base %08X's light keeps contradicting the pose from %08X: reacquiring on %08X",
                    on, primary, on);
      Log(text);
      const auto& f = filter_->Filter();
      if (f.updateCount > 200 && f.CalibrationHealthy()) carry_ = f;
      ResetFilter();
      avoidBase_ = std::make_pair(primary, Now() + 1.0);
      switchedAt_ = Now();
      return;
    }
  }
  if (!stock && multiRoom_) {
    const uint32_t primary = filter_->BaseSerial(0);
    // Well-lit frames (>= 4 sensors) the filter used in the last 0.5 s, and the last
    // one: a base glimpsed on 1-2 sensors anchors nothing.
    const auto lit = [&](uint32_t base, double* last) {
      int n = 0;
      *last = -1e9;
      for (const auto& [t, sensors] : recentUsed_[base])
        if (sensors >= 4) { *last = t; n += Now() - t < 0.5; }
      return n;
    };
    double primaryLast = 0;
    lit(primary, &primaryLast);
    uint32_t visible = 0;
    for (int slot = 1; slot < 4; ++slot) {
      const uint32_t b = filter_->BaseSerial(slot);
      double last = 0;
      // Half its sweeps in the last 0.5 s (a weakly lit base fails to reacquire on).
      const double sweeps = 0.5 / std::max(sweepInterval_ * 0.5, 1.0 / 120);
      if (b && lit(b, &last) >= 0.4 * sweeps && Now() - last < 0.1) visible = b;
    }
    if (primary && visible && Now() - acquiredAt_ > 1.0 && Now() - primaryLast > 0.5 && room_->Base(visible)) {
      char text[128];
      std::snprintf(text, sizeof text, "primary base %08X dark for %.0f ms, switching to base %08X", primary,
                    std::min(9999.0, (Now() - primaryLast) * 1000), visible);
      Log(text);
      const auto f = filter_->Filter();
      if (f.updateCount > 200 && f.CalibrationHealthy()) carry_ = f;
      // Hand the pose over to the new base through the room map instead of losing
      // tracking until it bootstraps again (a headset turns its sensors away from the
      // primary base all the time; each reset blanked the view for a moment).
      const RoomBase* from = room_->Base(primary);
      const RoomBase* to = room_->Base(visible);
      std::optional<LighthouseDirect::Seed> seed;
      if (from && to && carry_) {
        const Eigen::Isometry3d toFromPrimary = to->worldFromBase.inverse() * from->worldFromBase;
        LighthouseDirect::Seed s;
        s.base = visible;
        s.time = filter_->Time();
        s.baseFromImu = toFromPrimary * BaseFromImu(f.x);
        s.velocity = toFromPrimary.linear() * Eigen::Vector3d(f.x[3], f.x[4], f.x[5]);
        s.angularVelocity = Eigen::Vector3d(f.x[13], f.x[14], f.x[15]);
        // The room map's error between the two bases enters as pose uncertainty.
        s.positionVariance = f.P(0, 0) + from->positionVariance + to->positionVariance;
        s.rotationVariance = f.P(9, 9) + from->rotationVariance + to->rotationVariance;
        s.velocityVariance = f.P(3, 3);
        s.tilt = RoomTilt(*to);
        s.tiltVariance = std::max(to->rotationVariance, 1e-6);
        seed = s;
      }
      ResetFilter();
      switchedAt_ = Now();
      if (seed) {
        seed->carry = &*carry_;
        seed->calibrated = carry_->updateCount > 200;
        if (filter_->SeedPose(*seed)) {
          acquiredAt_ = Now();
          calibratedLogged_ = seed->calibrated;
          updatesAtAcquire_ = filter_->Filter().updateCount;
          std::snprintf(text, sizeof text, "handed over to base %08X through the room map", visible);
          Log(text);
        }
      }
      return;
    }
  }
  // Not stock: light keeps arriving, well lit, and none of it fits the pose for 0.5 s:
  // the filter is wrong, not the light. Reacquiring then (with the calm calibration)
  // takes 0.1-0.4 s; waiting for the 2 s no-light timeout, the device coasted that long
  // or ran out of bounds first (fastmotion3 + fastmotion5: fly-aways 2 -> 0, coasting
  // about 5 -> 0.5 %; testswing drops out more, 37 -> 29 % valid, but with fewer jumps
  // and rotation spikes: what it lost was tracking on a wrong pose).
  if (refusedLit_ >= 8 && Now() - filter_->LastOpticalTime() > kRefusedLightReset) {
    Log("light keeps arriving but none fits the pose: reacquiring");
    if (calm_) carry_ = calm_;
    ResetFilter();
    return;
  }
  const std::string reason = filter_->HealthCheck(stale ? filter_->Time() + 0.25 : Now());
  if (reason.empty()) return;
  Log(reason);
  const auto& f = filter_->Filter();
  if (f.CalibrationHealthy()) {
    // Lost light (occlusion, fast spin): the calibration is fine. One that hasn't
    // converged yet keeps the carried one it started from, so a device that drops out
    // again shortly after reacquiring doesn't recalibrate from zero.
    // Out of bounds is a fly-away: the filter put the error into its IMU biases on the
    // way, and reacquiring with them made it refuse the light and fly off again, over
    // and over, until the device was held still (fastmotion: 17 such resets). Dropping
    // the calibration helped (11), but a fresh filter can't calibrate while the device
    // keeps moving (fastmotion3: resets every 2-3 s): reacquire with the calibration
    // from the last calm stretch instead.
    if (reason.find("out of bounds") != std::string::npos) {
      if (calm_) carry_ = calm_;
      else carry_.reset();
    } else if (f.updateCount > 200) {
      carry_ = f;
    }
  } else {
    // Never reseed the very calibration rejected by HealthCheck: that creates
    // an immediate fail -> carry -> reacquire -> fail loop.
    carry_.reset();
    char details[192];
    const double alignment = std::sqrt(f.x[28]*f.x[28] + f.x[29]*f.x[29] + f.x[30]*f.x[30]);
    std::snprintf(details, sizeof details, "discarding rejected IMU calibration (gravity=%.6g alignment=%.3f deg updates=%llu)",
                  f.x[18], alignment * 57.29577951308232, static_cast<unsigned long long>(f.updateCount));
    Log(details);
  }
  ResetFilter();
}

TrackerOutput DeviceTracker::RawOutput() const {
  TrackerOutput out;
  out.deviceTime = filter_->Time();
  if (!filter_->Bootstrapped()) {
    out.state = hadTrack_ ? TrackingState::Lost : TrackingState::Uninitialized;
    return out;
  }
  const auto& f = filter_->Filter();
  out.primary = filter_->BaseSerial(0);
  for (int slot = 0; slot < 4; ++slot)
    if (filter_->BaseSerial(slot)) out.bases.push_back(filter_->BaseSerial(slot));
  for (const auto& [base, recent] : recentUsed_) {
    DevicePose::Coverage c{base, 0, 0};
    for (const auto& [t, sensors] : recent)
      if (Now() - t < 0.5) { ++c.frames; c.sensors += sensors; }
    if (c.frames) { c.sensors /= c.frames; out.coverage.push_back(c); }
  }
  out.updates = f.updateCount - std::min(f.updateCount, updatesAtAcquire_);
  const RoomBase* base = room_->Base(out.primary);
  if (!base) {
    out.state = TrackingState::Acquiring;
    out.result = TrackingResult::CalibratingInProgress;
    return out;
  }
  Eigen::Isometry3d baseFromImu = BaseFromImu(f.x);
  Eigen::Vector3d velocity(f.x[3], f.x[4], f.x[5]), omega(f.x[13], f.x[14], f.x[15]);
  Eigen::Vector3d acceleration(f.x[6], f.x[7], f.x[8]);
  out.filterPosition = base->worldFromBase * baseFromImu.translation();
  out.filterTime = filter_->Time();
  if (predictOutput_) {
    const auto now = filter_->PredictNow();
    out.deviceTime = now.time;
    baseFromImu.translation() = Eigen::Vector3d(now.p);
    baseFromImu.linear() = Eigen::Quaterniond(now.q[0], now.q[1], now.q[2], now.q[3]).toRotationMatrix();
    velocity = Eigen::Vector3d(now.v);
    omega = Eigen::Vector3d(now.w);
    acceleration = Eigen::Vector3d(now.a);
  }
  out.worldFromImu = base->worldFromBase * baseFromImu;
  out.velocity = base->worldFromBase.rotation() * velocity;
  if (reportAcceleration_) out.acceleration = base->worldFromBase.rotation() * acceleration;
  out.angularVelocity = out.worldFromImu.rotation() * omega;
  const double age = Now() - filter_->LastOpticalTime();
  out.valid = true;
  if (age <= kOutOfRangeAge) {
    out.state = TrackingState::Tracking;
    out.result = TrackingResult::RunningOk;
  } else {
    out.state = TrackingState::Coasting;
    out.result = TrackingResult::RunningOutOfRange;
  }
  return out;
}

// What the host sees, from the filter's pose (RawOutput), with two things the stock
// driver doesn't do:
//  * a discontinuity of the filter's pose (an optical correction after drift, a
//    reacquisition, a room-map change) becomes an offset that decays over kSmoothTau
//    instead of a step; motion passes through untouched, so nothing lags;
//  * without light, the position follows the last velocity, decaying over
//    kCoastTau, instead of 2 s of free inertial integration (live: 71 cm of drift,
//    snapped back when light returned). Orientation stays the IMU's.
void DeviceTracker::UpdateOutput() {
  TrackerOutput raw = RawOutput();
  // A deliberate primary switch (CheckHealth) reacquires within ~0.1 s: hold the last
  // pose as coasting meanwhile instead of telling the host the device is lost.
  if (!raw.valid && output_.valid && switchedAt_ && Now() - *switchedAt_ < 0.5) {
    output_.state = TrackingState::Coasting;
    output_.result = TrackingResult::RunningOutOfRange;
    output_.velocity.setZero();
    output_.angularVelocity.setZero();
    output_.acceleration.setZero();
    lastSmoothAt_ = Now();  // the glide afterwards starts from this held pose, not the gap
    return;
  }
  if (raw.valid) switchedAt_.reset();
  if (!smoothOutput_ || !raw.valid) {
    haveSmooth_ = false;
    coastAnchor_.reset();
    output_ = raw;
    return;
  }
  constexpr double kCoastTau = 0.12, kCoastAfter = 0.06;
  const Eigen::Vector3d p = raw.worldFromImu.translation();
  const Eigen::Quaterniond q(raw.worldFromImu.rotation());
  if (!haveSmooth_) {
    haveSmooth_ = true;
    offsetP_.setZero();
    offsetQ_.setIdentity();
  } else {
    const double dt = raw.deviceTime - lastRaw_.deviceTime;
    if (dt > 0 && dt < 0.25) {
      // Where the last raw pose was heading; what it can't explain is a jump.
      const Eigen::Vector3d expectP = lastRaw_.worldFromImu.translation() + lastRaw_.velocity * dt;
      const Eigen::Vector3d rot = lastRaw_.angularVelocity * dt;
      const Eigen::Quaterniond expectQ = (rot.norm() > 1e-12 ? Eigen::Quaterniond(Eigen::AngleAxisd(rot.norm(), rot.normalized()))
                                                             : Eigen::Quaterniond::Identity()) * Eigen::Quaterniond(lastRaw_.worldFromImu.rotation());
      const Eigen::Vector3d jumpP = p - expectP;
      const Eigen::Quaterniond jumpQ = q * expectQ.conjugate();
      if (jumpP.norm() > 0.002) offsetP_ -= jumpP;
      if (Eigen::AngleAxisd(jumpQ).angle() > 0.0087) offsetQ_ = offsetQ_ * jumpQ.conjugate();
      // Corrections over 10 cm glide 2.5x slower (they were jumps of a wrong pose).
      const double tauP = offsetP_.norm() > 0.1 ? 2.5 * smoothPosition_ : smoothPosition_;
      // The glide runs on output time: dt is filter time, and across a reacquire it
      // spans the whole dark gap, so a 41 cm correction went 37 % through in one frame
      // (vrchat-01: an 18 cm step).
      const double step = std::clamp(Now() - lastSmoothAt_, 0.0, dt);
      const double keepP = tauP > 0 ? std::exp(-step / tauP) : 0.0;
      const double keepQ = smoothRotation_ > 0 ? std::exp(-step / smoothRotation_) : 0.0;
      offsetP_ *= keepP;
      offsetQ_ = Eigen::Quaterniond::Identity().slerp(keepQ, offsetQ_);
    } else if (dt >= 0.25 || dt < -0.25) {
      offsetP_.setZero();
      offsetQ_.setIdentity();
    }
  }
  lastRaw_ = raw;
  lastSmoothAt_ = Now();
  TrackerOutput out = raw;
  Eigen::Vector3d outP = p + offsetP_;
  Eigen::Quaterniond outQ = (offsetQ_ * q).normalized();
  // Still: the pose it came to rest at, whatever few-sensor optics (one base, a side
  // on the sheet) make of it meanwhile (live: 21 cm of depth drift on a bed).
  if (still_ && holdStill_) {
    if (!heldPose_) heldPose_ = Eigen::Isometry3d(Eigen::Translation3d(outP) * outQ);
    offsetP_ = heldPose_->translation() - p;  // moving off glides from here
    offsetQ_ = (Eigen::Quaterniond(heldPose_->rotation()) * q.conjugate()).normalized();
    out.worldFromImu = *heldPose_;
    out.velocity.setZero();
    out.angularVelocity.setZero();
    out.acceleration.setZero();
    coastAnchor_.reset();
    output_ = out;
    return;
  }
  heldPose_.reset();
  if (filter_->LastOpticalTime() != lastOpticalSeen_) {
    lastOpticalSeen_ = filter_->LastOpticalTime();
    lastOpticalTaken_ = Now();
  }
  const double dark = std::min(Now() - filter_->LastOpticalTime(), Now() - lastOpticalTaken_);
  // A lone multi-station base sweeps every N revolutions (66 ms at N = 4): that is
  // not darkness yet.
  if (dark > std::max(kCoastAfter, 1.25 * sweepInterval_)) {
    if (!coastAnchor_) coastAnchor_ = CoastAnchor{outP, raw.velocity, raw.deviceTime};
    const double t = std::max(0.0, raw.deviceTime - coastAnchor_->time);
    const double decay = std::exp(-t / kCoastTau);
    outP = coastAnchor_->position + coastAnchor_->velocity * kCoastTau * (1 - decay);
    out.velocity = coastAnchor_->velocity * decay;
    out.acceleration.setZero();
    offsetP_ = outP - p;  // so light coming back glides from here
  } else {
    coastAnchor_.reset();
  }
  out.worldFromImu.translation() = outP;
  out.worldFromImu.linear() = outQ.toRotationMatrix();
  output_ = out;
}

TrackerOutput DeviceTracker::Output() const { return output_; }

// Physically still (lying on a bed or a desk, standing in a charger): the gyro, less
// the filter's bias, below ~0.9 deg/s and the accelerometer within 0.04 m/s^2 of its
// running mean, for 0.5 s. A hand held in the air doesn't get there; any motion
// leaves at once (2x the thresholds).
void DeviceTracker::TrackStillness(const ImuSample& s) {
  constexpr double kGyro = 0.015, kAccel = 0.04, kSettle = 0.5;
  const auto& x = filter_->Filter().x;
  const bool calibrated = filter_->Bootstrapped() && filter_->Filter().updateCount > 200;
  const Eigen::Vector3d bias = calibrated ? Eigen::Vector3d(x[19], x[20], x[21]) : Eigen::Vector3d::Zero();
  const double gyro = (Eigen::Vector3d(s.gyro[0], s.gyro[1], s.gyro[2]) - bias).norm();
  const Eigen::Vector3d a(s.accel[0], s.accel[1], s.accel[2]);
  if (!haveAccelMean_) { accelMean_ = a; haveAccelMean_ = true; }
  const double accel = (a - accelMean_).norm();
  accelMean_ += 0.05 * (a - accelMean_);
  const bool quiet = calibrated && !s.saturated && gyro < kGyro && accel < kAccel;
  const bool moving = !calibrated || s.saturated || gyro > 2 * kGyro || accel > 2 * kAccel;
  if (moving) { quietSince_.reset(); still_ = false; }
  else if (quiet && !quietSince_) quietSince_ = s.time;
  if (quietSince_ && s.time - *quietSince_ > kSettle) still_ = true;
}

std::vector<std::string> DeviceTracker::TakeLogs() {
  std::vector<std::string> out;
  out.swap(logs_);
  return out;
}

}  // namespace lh::runtime
