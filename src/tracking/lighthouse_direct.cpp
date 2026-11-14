#include "lighthouse_direct.hpp"
#include <map>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>

#include "pose_solver.hpp"
#include "pulse_sorter.hpp"

namespace lh {
namespace {
using namespace direct;

Residual Normalizer() {
  return {1, [](const double* x, double* out) { out[0] = QuaternionNormalizer(x); },
          [](const Jet* x, Jet* out) { out[0] = QuaternionNormalizer(x); }};
}
Eigen::Isometry3d SolverPose(const solver::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.p.data());
  t.linear() = Eigen::Quaterniond(p.q[0], p.q[1], p.q[2], p.q[3]).normalized().toRotationMatrix();
  return t;
}
}  // namespace

LighthouseDirect::LighthouseDirect(TrackedObjectModel model, double opticalTimeout)
    : model_(model.InImuFrame()), opticalTimeout_(opticalTimeout) {
  filter_.DefaultState();
  filter_.omegaCorrectionLimit2 = model_.deviceClass == "hmd" ? 304.61741978670864 : 4873.878716587338;
  for (int i = 0; i < 3; ++i) {
    filter_.x[19 + i] = double(float(model_.gyroBias[size_t(i)]));
    filter_.x[22 + i] = double(float(model_.accBias[size_t(i)]));
    filter_.x[25 + i] = double(float(model_.accScale[size_t(i)]));
  }
  for (const auto& a : model_.points) {
    radius_ = std::max(radius_, Eigen::Vector3d(a.data()).norm());
    for (const auto& b : model_.points)
      diameter_ = std::max(diameter_, (Eigen::Vector3d(a.data()) - Eigen::Vector3d(b.data())).norm());
  }
}

void LighthouseDirect::OnBaseConfig(const BaseConfig& config, const BaseDynamicState& dynamic) {
  if (!config.valid || config.serial == 0 || config.cal.size() != 14 || (config.hardwareVersion > 9 && config.model == 1)) return;
  if (!std::all_of(config.cal.begin(), config.cal.end(), [](double v) { return std::isfinite(v); })) return;
  bases_[config.serial] = {config, dynamic};
}

void LighthouseDirect::SetRecordedTilt(uint32_t serial, std::array<double, 2> tilt, double variance) {
  if (serial && std::isfinite(tilt[0]) && std::isfinite(tilt[1]) && std::isfinite(variance) && variance >= 0)
    recordedTilts_[serial] = {tilt, variance};
}

void LighthouseDirect::OnImu(const ImuSample& s) {
  if (!observedTime_ || s.time > *observedTime_) observedTime_ = s.time;
  imu_.push_back(s);
  // Bounded pre-bootstrap buffer. Original queue/reset policies are still TODO.
  while (imu_.size() > 2000) imu_.pop_front();
}

int LighthouseDirect::Slot(uint32_t serial) const {
  for (int i = 0; i < 4; ++i) if (serial != 0 && slots_[i] == serial) return i;
  return -1;
}

std::vector<OpticalSample> LighthouseDirect::Gather(const std::vector<OpticalSample>& input) const {
  int counts[2][32]{};
  for (const auto& s : input)
    if (s.sensor >= 0 && s.sensor < 32 && s.axis >= 0 && s.axis < 2) ++counts[s.axis][s.sensor];
  std::vector<OpticalSample> out;
  for (const auto& s : input) {
    if (s.sensor < 0 || s.sensor >= 32 || s.axis < 0 || s.axis >= 2) continue;
    if (counts[s.axis][s.sensor] != 1 || model_.IndexOf(s.sensor) < 0) continue;
    if (!(1.0610329539459692e-6 <= s.width && s.width <= 0.00013262911924324613)) continue;
    if (!std::isfinite(s.time) || !std::isfinite(s.angle)) continue;
    out.push_back(s);
  }
  return out;
}

double LighthouseDirect::BaseDistance(const double* state, int slot) const {
  double p[3] = {state[0], state[1], state[2]};
  if (slot > 0) {
    double alt[7], r[3];
    AltBasePose(state, kAltBaseIndex[slot - 1], alt);
    model::QuatRotate(alt + 3, p, r);
    for (int i = 0; i < 3; ++i) p[i] = r[i] + alt[i];
  }
  return std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

void LighthouseDirect::FilterOutliers(std::vector<OpticalSample>* samples, double distance) const {
  if (samples->empty()) return;
  const double bound = 2.5 * std::atan2(0.5 * diameter_, std::max(0.0, distance - radius_));
  auto& s = *samples;
  std::sort(s.begin(), s.end(), [](const auto& a, const auto& b) { return a.angle < b.angle; });
  if (s.size() == 2 && bound < s[1].angle - s[0].angle) { s.clear(); return; }
  const double median = s[s.size() / 2].angle;
  while (!s.empty() && bound < s.back().angle - median) s.pop_back();
  const auto first = std::find_if(s.begin(), s.end(), [&](const auto& p) { return median - p.angle < bound; });
  s.erase(s.begin(), first);
}

bool LighthouseDirect::PredictTo(double time) {
  if (time < time_) return false;
  while (!imu_.empty() && imu_.front().time < time) {
    ImuUpdate(imu_.front());
    imu_.pop_front();
  }
  return true;
}

bool LighthouseDirect::ImuUpdate(const ImuSample& sample) {
  if (!slots_[0]) return false;
  const double dt = sample.time - time_;
  if (!(0 <= dt && dt <= 0.25)) return false;
  std::vector<Residual> residuals;
  std::vector<double> measurements, variances;
  filter_.ComputeProcessNoise(dt);
  // Not stock: a saturated accelerometer axis means the device accelerates harder than
  // the IMU can say, by an unknown amount. The prediction is then much less certain than
  // the jerk model thinks, and the gates threw out the light that would correct it: a
  // tracker shaken hard for 7 s (fastmotion4, 70-80 % of samples saturated) flew away 5
  // times; with this, none (fastmotion3: tracking 91 -> 97 %). x10 is the strength: x100
  // was worse on both. Gyro saturation is left alone (it added rotation spikes).
  if (swing_ && (sample.saturated & 0x7)) filter_.Q.topLeftCorner(9, 9) *= kSaturatedJerkScale;
  filter_.Predict(dt, dt);
  // Not stock: a saturated axis still says the true value is at least its reading
  // (the range limit), that way. Stock drops it, and the filter, left to the light
  // alone, believed spins of 4500 deg/s (testswing: 27 % of samples saturated).
  // Predicted short of the limit: pulled to it; beyond: nothing to say.
  const bool stockSat = !swing_;
  double predicted[2][3] = {};
  if (sample.saturated && !stockSat) { ImuGyro(filter_.xLin, predicted[0]); ImuAccel(filter_.xLin, predicted[1]); }
  for (int kind = 0; kind < 2; ++kind) {
    for (int axis = 0; axis < 3; ++axis) {
      const int bit = kind == 0 ? axis + 3 : axis;
      bool bound = false;
      if (sample.saturated & (1u << bit)) {
        if (stockSat) continue;
        const double limit = kind == 0 ? sample.gyro[size_t(axis)] : sample.accel[size_t(axis)];
        if (limit == 0 || predicted[kind][axis] * (limit > 0 ? 1 : -1) >= std::fabs(limit)) continue;
        bound = true;
      }
      residuals.push_back({1,
        [=](const double* x, double* out) {
          double r[3]; if (kind == 0) ImuGyro(x, r); else ImuAccel(x, r); out[0] = r[axis];
        },
        [=](const Jet* x, Jet* out) {
          Jet r[3]; if (kind == 0) ImuGyro(x, r); else ImuAccel(x, r); out[0] = r[axis];
        }});
      measurements.push_back(kind == 0 ? sample.gyro[size_t(axis)] : sample.accel[size_t(axis)]);
      variances.push_back(bound ? (kind == 0 ? 0.25 : 4.0) : kind == 0 ? 1.6e-5 : (axis == 2 ? 0.003 : 0.0015) * vibrationAccelScale_);
    }
  }
  if (sample.saturated) filter_.flags |= 0x400;
  {
    // Violent: saturated, or near the IMU's limits (Index: 34.9 rad/s, 78 m/s2 per
    // axis). The calibration is held from then until 0.3 s after.
    double g = 0, a = 0;
    for (int i = 0; i < 3; ++i) { g = std::max(g, std::fabs(sample.gyro[size_t(i)])); a = std::max(a, std::fabs(sample.accel[size_t(i)])); }
    if (sample.saturated || g > 25 || a > 60) violentUntil_ = sample.time + 0.3;
    // Not stock: vibration (a controller's haptics held on) shakes some IMUs hard without
    // reaching their limits: the accelerometer jumps 2-4 m/s2 from one sample to the next
    // and its average reads 11-12.5 m/s2, not 9.8. Learned from that, the IMU alignment came
    // out 17-21 deg off within 1.5 s of every reacquire, tracking reset over and over and
    // the controller flew off between (#90). From then until 0.5 s after, gravity and the
    // IMU calibration are held and the accelerometer is trusted 100x less (from the next
    // sample), so the light carries the pose. Vibration is told from a hard swing, which
    // jumps as much per sample, by its roughness: the second difference over the first
    // (both averaged over ~0.25 s) is ~1.9 (the reading flips nearly every sample), a
    // swing's or a stomp's 0.4-1.0. (#90 recordings, two controllers: resets 21/8 and
    // 17 -> 0-1, steps > 5 cm 387/246 and 93 -> 1-43. Held as violent motion instead,
    // frames rejected too: a vibrating pickup in vrchat-01 flew off 4 m. On the jumps
    // alone, without roughness, hard swings jumped twice as often.)
    if (havePrevAccel_ > 1) {
      double d1 = 0, d2 = 0;
      for (int i = 0; i < 3; ++i) {
        const double a = sample.accel[size_t(i)], p = prevAccel_[0][size_t(i)], q = prevAccel_[1][size_t(i)];
        d1 += (a - p) * (a - p);
        d2 += (a - 2 * p + q) * (a - 2 * p + q);
      }
      const double k = std::min(1.0, dt / 0.25);
      accelJitter_ += k * (std::sqrt(d1) - accelJitter_);
      accelRoughness_ += k * (std::sqrt(d2) - accelRoughness_);
      if (accelJitter_ > kVibrationJitter && accelRoughness_ > kVibrationRoughness * accelJitter_)
        vibratingUntil_ = sample.time + 0.5;
    }
    prevAccel_[1] = prevAccel_[0];
    prevAccel_[0] = sample.accel;
    havePrevAccel_ = std::min(havePrevAccel_ + 1, 2);
    const bool vibrating = swing_ && sample.time < vibratingUntil_;
    filter_.holdImuCalibration = vibrating;
    vibrationAccelScale_ = vibrating ? kVibrationAccelScale : 1.0;
    filter_.holdSlowStates = swing_ && sample.time < violentUntil_;
  }
  residuals.push_back(Normalizer());
  measurements.push_back(0); variances.push_back(1e-8);
  const int m = int(measurements.size());
  Mat R = Mat::Zero(m, m);
  for (int i = 0; i < m; ++i) R(i, i) = variances[size_t(i)];
  filter_.imuUpdate = true;
  const bool updated = filter_.Update(residuals, R, Eigen::Map<Vec>(measurements.data(), m));
  filter_.imuUpdate = false;
  if (!updated) return false;
  time_ = sample.time;
  return true;
}

bool LighthouseDirect::CanSwitchPrimary(uint32_t serial) const {
  return calibrationSeen_ && filter_.flags & 0x100;
}

namespace {
// 0x35bf00: a primary bootstrap pose (base frame, the base looks down -z) must be
// in front of the base (z <= -0.1 m) and within 7 m (squared distance <= 49).
bool BootstrapPoseInRange(const solver::Pose& pose) {
  const double d2 = pose.p[0] * pose.p[0] + pose.p[1] * pose.p[1] + pose.p[2] * pose.p[2];
  return pose.p[2] <= -0.1 && d2 <= 49.0;
}

// Not in stock: one reflected or partly occluded sensor can keep an otherwise
// good bootstrap from converging forever (live: ~1700 failed attempts with one
// sensor ~2.3 mrad off). When the full set lands on a finite, in-range pose but
// misses the stock convergence test, retry without each sensor in turn and keep
// the best result that converges under the unchanged stock threshold. The
// dropped sensor is removed from `older` too, so the velocity solve matches.
solver::Result SolveBootstrap(const TrackedObjectModel& model, const BaseConfig& base,
                              std::vector<OpticalSample>* newer, std::vector<OpticalSample>* older) {
  solver::Result full = solver::Solve(model, base, *newer);
  if (full.status != 1 || !std::isfinite(full.cost) || !BootstrapPoseInRange(full.pose)) return full;
  std::vector<int> sensors;
  for (const auto& s : *newer)
    if (std::find(sensors.begin(), sensors.end(), s.sensor) == sensors.end()) sensors.push_back(s.sensor);
  if (sensors.size() < 5) return full;  // too few to leave one out safely
  solver::Result best;
  int dropped = -1;
  for (int sensor : sensors) {
    std::vector<OpticalSample> subset;
    for (const auto& s : *newer) if (s.sensor != sensor) subset.push_back(s);
    const solver::Result r = solver::Solve(model, base, subset, &full.pose);
    if (r.status == 0 && std::isfinite(r.cost) && BootstrapPoseInRange(r.pose) && (dropped < 0 || r.cost < best.cost)) {
      best = r;
      dropped = sensor;
    }
  }
  if (dropped < 0) return full;
  auto without = [dropped](std::vector<OpticalSample>* v) {
    v->erase(std::remove_if(v->begin(), v->end(), [dropped](const OpticalSample& s) { return s.sensor == dropped; }), v->end());
  };
  without(newer);
  without(older);
  return best;
}
}  // namespace

bool LighthouseDirect::Bootstrap(const OpticalFrame& frame, const BaseConfig& base) {
  OpticalFrame f = frame;
  f.samples = Gather(frame.samples);
  auto& history = history_[frame.baseSerial];
  history.Append(std::move(f));
  std::vector<OpticalSample> older, newer;
  if (!history.Select(&older, &newer)) return false;
  ++bootstrapAttempts_;
  const double time = newer.front().time + newer.front().width * 0.5;
  if (slots_[0] && CanSwitchPrimary(base.serial)) {
    if (!PredictTo(time)) return false;
    const auto solved = SolveBootstrap(model_, base, &newer, &older);
    if (solved.status != 0 || !std::isfinite(solved.cost) ||
        !std::all_of(solved.pose.p.begin(), solved.pose.p.end(), [](double v) { return std::isfinite(v); }) ||
        !std::all_of(solved.pose.q.begin(), solved.pose.q.end(), [](double v) { return std::isfinite(v); })) return false;
    if (!BootstrapPoseInRange(solved.pose)) return false;
    for (int i = 0; i < 3; ++i) filter_.x[i] = solved.pose.p[size_t(i)];
    for (int i = 0; i < 4; ++i) filter_.x[9 + i] = solved.pose.q[size_t(i)];
    filter_.P.setZero(); filter_.InitialCovariance();
    filter_.flags &= ~8u;
    filter_.rejectCount = 0; filter_.updateCount = 101;
    slots_[0] = base.serial;
    for (int slot = 1; slot < 4; ++slot) {
      slots_[slot] = 0; filter_.altBaseActive[slot - 1] = false;
    }
    filter_.n = 31; calibrationSeen_ = false; ++primarySwitches_;
    time_ = lastOpticalTime_ = time;
    while (!imu_.empty() && imu_.front().time < time) imu_.pop_front();
    return true;
  }
  auto solved = SolveBootstrap(model_, base, &newer, &older);
  // The stock start failed: try every start, at most every 0.25 s per base (not stock;
  // see solver::SolveMultiStart). An ambiguous fit is not taken.
  if (solved.status == 1 && !slots_[0] && multiStart_) {
    double& last = lastMultiStart_[base.serial];
    double& wait = multiStartWait_[base.serial];
    if (wait < 0.25) wait = 0.25;
    if (time - last > wait || time < last) {
      last = time;
      wait = std::min(2.0, wait * 2);  // backs off while it keeps failing (each try ~25 ms CPU)
      bool ambiguous = false;
      std::vector<int> dropped;
      const auto retry = solver::SolveMultiStart(model_, base, newer, &ambiguous, &dropped);
      if (retry.status == 0 && !ambiguous) {
        solved = retry;
        wait = 0.25;
        if (!dropped.empty()) {  // the velocity solve must not see them either
          auto without = [&dropped](std::vector<OpticalSample>* v) {
            v->erase(std::remove_if(v->begin(), v->end(), [&dropped](const OpticalSample& s) {
              return std::find(dropped.begin(), dropped.end(), s.sensor) != dropped.end(); }), v->end());
          };
          without(&newer);
          without(&older);
        }
      }
    }
  }
  if (solved.status != 0 || !std::isfinite(solved.cost) ||
      !std::all_of(solved.pose.p.begin(), solved.pose.p.end(), [](double v) { return std::isfinite(v); }) ||
      !std::all_of(solved.pose.q.begin(), solved.pose.q.end(), [](double v) { return std::isfinite(v); })) {
    return false;
  }
  if (!slots_[0] && !BootstrapPoseInRange(solved.pose)) return false;
  if (slots_[0]) {
    // The original only adds an alternate base via 0x35c330, which requires
    // updateCount > 0x104 (260) and a LighthouseDB sanction. Without the gate the
    // alt slice is seeded far too early and the whole filter diverges.
    if (filter_.updateCount <= 0x104) return false;
    int slot = 1;
    while (slot < 4 && slots_[slot]) ++slot;
    if (slot == 4 || !PredictTo(time)) return false;
    // 0x35c5b0 extrapolates with constant velocity and angular rate only
    // (no acceleration), composing q * exp(0.5 * omega * dt).
    const double* x = filter_.x;
    const double dt = time - time_;
    Eigen::Quaterniond q(x[9], x[10], x[11], x[12]);
    q.normalize();
    const Eigen::Vector3d half = 0.5 * dt * Eigen::Vector3d(x[13], x[14], x[15]);
    const double angle = half.norm();
    Eigen::Quaterniond dq(1, 0, 0, 0);
    if (angle != 0.0) {
      const Eigen::Vector3d v = half * (std::sin(angle) / angle);
      dq = Eigen::Quaterniond(std::cos(angle), v.x(), v.y(), v.z());
    }
    Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
    current.linear() = (q * dq).toRotationMatrix();
    current.translation() = Eigen::Vector3d(x) + dt * Eigen::Vector3d(x + 3);
    const Eigen::Isometry3d transform = SolverPose(solved.pose) * current.inverse();
    const Eigen::AngleAxisd aa(transform.rotation());
    const Eigen::Vector3d rotation = aa.axis() * aa.angle();
    const int idx = kAltBaseIndex[slot - 1];
    for (int i = 0; i < 3; ++i) {
      filter_.x[idx + i] = transform.translation()[i];
      filter_.x[idx + 3 + i] = rotation[i];
    }
    filter_.P.middleRows(idx, 6).setZero(); filter_.P.middleCols(idx, 6).setZero();
    filter_.P.diagonal().segment<3>(idx).setConstant(0.0004);
    filter_.P.diagonal().segment<3>(idx + 3).setConstant(0.0027415567780803775);
    filter_.altBaseActive[slot - 1] = true;
    filter_.altBaseRelearn[slot - 1] = false;
    filter_.n = std::max(filter_.n, idx + 6);
    slots_[slot] = base.serial; ++secondaryBases_;
    slotUsed_[slot] = time;
    return false;  // Added a base, but did not perform a measurement update.
  }
  for (int i = 0; i < 3; ++i) filter_.x[i] = solved.pose.p[size_t(i)];
  for (int i = 0; i < 4; ++i) filter_.x[9 + i] = solved.pose.q[size_t(i)];
  filter_.P.setZero(); filter_.InitialCovariance();
  // 0x350600: interpolate the queued IMU sample at the pose time and correct it
  // with the current gyro bias and the model gyro scale (float math, as original).
  for (int i = 0; i < 3; ++i) filter_.P(13 + i, 13 + i) = 1.6e-5;
  if (imu_.size() > 1 && imu_.front().time <= time && time <= imu_.back().time) {
    size_t hi = 1;
    while (hi + 1 < imu_.size() && imu_[hi].time < time) ++hi;
    const auto& a = imu_[hi - 1];
    const auto& b = imu_[hi];
    if (!(a.saturated | b.saturated)) {
      const float f = float((time - a.time) / (b.time - a.time));
      for (int i = 0; i < 3; ++i) {
        const float g = (float(b.gyro[size_t(i)]) - float(a.gyro[size_t(i)])) * f + float(a.gyro[size_t(i)]);
        filter_.x[13 + i] = double((g - float(filter_.x[19 + i])) * float(model_.gyroScale[size_t(i)]));
      }
    }
  }
  const double oldTime = older.front().time + older.front().width * 0.5;
  if (time > oldTime) {
    const auto previous = solver::Solve(model_, base, older, &solved.pose);
    if (previous.status == 0) {
      for (int i = 0; i < 3; ++i) {
        filter_.x[3 + i] = (solved.pose.p[size_t(i)] - previous.pose.p[size_t(i)]) / (time - oldTime);
        filter_.P(3 + i, 3 + i) = 0.0025000000000000005;
      }
    }
  }
  const auto& gravity = bases_.at(base.serial).dynamic.accelDir;
  const Eigen::Vector3d g(gravity[0], gravity[1], gravity[2]);
  if (g.norm() > 0.5) {
    const Eigen::AngleAxisd tilt(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), g));
    filter_.x[16] = tilt.axis().x() * tilt.angle();
    filter_.x[17] = tilt.axis().z() * tilt.angle();
  }
  const auto recorded = recordedTilts_.find(base.serial);
  if (recorded != recordedTilts_.end() && recorded->second.variance < filter_.P(16, 16)) {
    filter_.x[16] = recorded->second.angles[0]; filter_.x[17] = recorded->second.angles[1];
    filter_.P(16, 16) = filter_.P(17, 17) = recorded->second.variance;
  }
  slots_[0] = base.serial; filter_.updateCount = 101; calibrationSeen_ = false;
  time_ = lastOpticalTime_ = time;
  while (!imu_.empty() && imu_.front().time < time) imu_.pop_front();
  return true;
}

// Not stock (rotor_wobble.hpp): what a frame's sorted hits have in common against the predicted state goes
// to their rotor's wobble, as far as the state is known better than the wobble, and comes off the hits.
// z and R are the update's, R already filled in.
//
// The hits' weights stay as they were. At first what was left unsure of the wobble went onto them as scatter
// they share (its variance on every pair of the rotor's rows), which makes a frame's mean count for less:
// by 0.3 mrad while the wobble isn't followed (a fresh pose, thin light), 0.12-0.15 mrad while it is. A
// frame's mean is what holds a pose that little else lights. A 2018 tracker with one rotor's 3-4 sensors a
// frame and a 1.0 station just added had its IMU calibration run off (15 deg) and reset 1 s after its first
// LH1 mode, in 8 replays of 8 with the shared scatter (capped at 0.15 mrad too) and 0 of 9 without. The
// steadiness is all from taking the offset out: still jitter 0.25, 0.23, 0.23 mm (3.0 tracker,
// Knuckles) to 0.16, 0.11, 0.12 with both, 0.17, 0.12, 0.12 without the shared scatter, 0.24, 0.22, 0.21
// with it alone.
void LighthouseDirect::FollowRotorWobble(uint32_t serial, double time, const std::vector<Residual>& residuals,
                                         const std::vector<bool>& sorted, bool referenced, std::vector<double>* z, const Mat& R) {
  std::vector<int> rows;
  for (size_t i = 0; i < sorted.size(); ++i)
    if (sorted[i]) rows.push_back(int(i));
  if (rows.empty()) return;
  // The rotor's wobble already taken off by a device in LH2 mode (kSampleReferenced): nothing to follow. Followed
  // as well, a 2018 tracker lying still had 0.37 mm of still jitter against 0.23 not (0.27 before either), with a
  // follower of their own for such frames too. It starts over where the reference ends.
  if (referenced) {
    wobbles_.erase(serial);
    return;
  }
  RotorWobble& wobble = wobbles_[serial];
  wobble.Predict(time, wobbleSpread_, wobbleMemory_);
  const int n = filter_.n;
  Jet state[kStateSize];
  for (int i = 0; i < kStateSize; ++i) state[i] = Jet(filter_.xLin[i], i);
  std::vector<double> off(rows.size());
  std::vector<Eigen::RowVectorXd> jacobian(rows.size());
  for (size_t k = 0; k < rows.size(); ++k) {
    Jet predicted;
    residuals[size_t(rows[k])].evalJet(state, &predicted);
    off[k] = (*z)[size_t(rows[k])] - predicted.a;
    jacobian[k] = predicted.v.head(n).transpose();
  }
  const std::vector<size_t> common = direct::HitsInCommon(off);
  if (!common.empty()) {
    Eigen::RowVectorXd mean = Eigen::RowVectorXd::Zero(n);
    double sum = 0, scatter = 0;
    for (const size_t k : common) {
      sum += off[k];
      mean += jacobian[k];
      scatter += R(rows[k], rows[k]);
    }
    const double used = double(common.size());
    mean /= used;
    const double pose = (mean * filter_.Ppred.topLeftCorner(n, n) * mean.transpose())(0, 0);
    wobble.Update(sum / used, pose, scatter / (used * used));
  }
  for (const int i : rows) (*z)[size_t(i)] -= wobble.value;
}

bool LighthouseDirect::OpticalUpdate(const OpticalFrame& frame, const BaseConfig& base, int slot) {
  return ApplyOpticalResiduals(frame, base, slot);
}

bool LighthouseDirect::ApplyOpticalResiduals(const OpticalFrame& frame, const BaseConfig& base, int slot) {
  double time = std::numeric_limits<double>::infinity();
  for (const auto& s : frame.samples) time = std::min(time, s.time + 0.5 * s.width);
  auto samples = Gather(frame.samples);
  FilterOutliers(&samples, BaseDistance(filter_.x, slot));
  if (samples.empty() || !PredictTo(time)) return false;
  const double dt = std::min(0.25, time - time_);
  filter_.ComputeProcessNoise(dt); filter_.Predict(dt, dt);
  const int alt = slot == 0 ? -1 : kAltBaseIndex[slot - 1];
  std::vector<Residual> residuals;
  std::vector<double> z, facing;
  std::vector<bool> sorted;
  bool referenced = false;  // FollowRotorWobble
  for (const auto& s : samples) {
    const int idx = model_.IndexOf(s.sensor);
    const auto point = model_.points[size_t(idx)], normal = model_.normals[size_t(idx)];
    const double offset = s.time + 0.5 * s.width - time;
    const double cos = SensorNormalCosAngle(filter_.xLin, 0.0, point.data(), normal.data(), alt);
    if (!(cos >= 0.3)) continue;
    const auto cal = base.cal;
    const int baseModel = base.model;
    // LH2: a point outside the plane model's domain has no prediction; skip it.
    if (baseModel != 1 && !std::isfinite(SensorAngular(filter_.xLin, offset, point.data(), s.axis, cal.data(), alt, baseModel)))
      continue;
    residuals.push_back({1,
      [=](const double* x, double* r) { r[0] = SensorAngular(x, offset, point.data(), s.axis, cal.data(), alt, baseModel); },
      [=](const Jet* x, Jet* r) { r[0] = SensorAngular(x, offset, point.data(), s.axis, cal.data(), alt, baseModel); }});
    z.push_back(s.angle); facing.push_back(cos);
    sorted.push_back((s.flags2 & kSampleSorted) != 0);
    referenced |= (s.flags2 & kSampleReferenced) != 0;
  }
  // 0x36f740: when every sample fails the facing cutoff it skips the update but
  // still returns PredictTo's result (true); x, P and time are left untouched.
  if (z.empty()) return true;
  const int hits = int(z.size());
  residuals.push_back(Normalizer()); z.push_back(0);
  const int m = int(z.size());
  Mat R = Mat::Zero(m, m);
  R.topLeftCorner(hits, hits).setConstant(1.579136704174297e-10);
  double distance = BaseDistance(filter_.xLin, slot);
  const double inv = distance == 0 ? 1 : 1 / distance;
  const double a = std::atan(inv * 7e-5), b = std::atan(inv * 3e-5);
  const double variance = 1.579136704174297e-10 + 2.2206609902451057e-9 + a * a + b * b + 3.947841760435744e-9;
  for (int i = 0; i < hits; ++i) {
    // Original 0x36f740: for cos < 0.5 add atan()^2; above 0.2 the argument is
    // inv*0.002*3.3333*(0.5-cos), at or below 0.2 it is the constant inv*0.002.
    double grazing = 0;
    if (facing[size_t(i)] < 0.5) {
      const double arg = facing[size_t(i)] > 0.2
                             ? inv * 0.002 * 3.3333333333333335 * (0.5 - facing[size_t(i)])
                             : inv * 0.002;
      grazing = std::atan(arg);
    }
    R(i, i) = variance + grazing * grazing;
    // Not stock (SetMixedLightScatter): light in LH1 mode in a room with both kinds of station.
    if (sorted[size_t(i)]) {
      const double period = frame.averagePeriod > 0 ? frame.averagePeriod : 0.02;
      const double angle = sortedScatter_ * 6.283185307179586 / period;
      R(i, i) += angle * angle;
    } else if (base.model == 1) {
      R(i, i) += lh1Scatter_ * lh1Scatter_;
    }
  }
  R(m - 1, m - 1) = 1e-8;
  if (wobbleSpread_ > 0) FollowRotorWobble(base.serial, time, residuals, sorted, referenced, &z, R);
  // Not stock: right after a reseed (reacquired with a carried calibration, so already
  // at 201 updates) the other bases' light isn't frame-gated: it is what corrects the
  // new pose, and gated, the device tracked from one base on a pose the others never fit
  // until it ran off (fastmotion3 + fastmotion5: fly-aways 6 -> 2; spin-01's jumps > 5 cm
  // 229 -> 258; Beat Saber-like fastmotionindex unchanged). The base it reacquired on
  // stays gated: lifting that too let bad light through (spin-01: 229 -> 344).
  filter_.frameGateOff = slot != 0 && filter_.updateCount < altGateUntil_;
  const bool ok = filter_.Update(residuals, R, Eigen::Map<Vec>(z.data(), m), true);
  filter_.frameGateOff = false;
  if (!ok) return false;
  if (hits >= 3) slotUsed_[slot] = time;
  time_ = time;
  const uint64_t before = filter_.updateCount;
  if (hits >= 4) {
    ++filter_.updateCount;
    lastWellLit_ = time;
  }
  if (before < 201 && filter_.updateCount > 200) {
    // 0x350450: re-seed IMU-parameter variances (diagonal only) at CALIBRATED.
    auto& P = filter_.P;
    P(22, 22) = P(23, 23) = 1.0; P(24, 24) = 4.0;
    for (int i = 25; i < 28; ++i) P(i, i) = 0.0009;
    for (int i = 19; i < 22; ++i) P(i, i) = 0.12184696791468343;
    P(18, 18) = 2.4042596055625e-05;
    for (int i = 28; i < 31; ++i) P(i, i) = 0.0012184696791468343;
  }
  if (hits >= 2) {
    lastOpticalTime_ = time;
    // The recovered "calibrated" boundary is updateCount 200 (0xc8..0xc9 edge).
    if (filter_.updateCount >= 200) calibrationSeen_ = true;
  }
  return true;
}

bool LighthouseDirect::OnFrame(const OpticalFrame& frame) {
  filter_.flags |= 8;
  const auto base = bases_.find(frame.baseSerial);
  if (base == bases_.end() || !frame.leftSyncIsReal || !frame.rightSyncIsReal) return false;
  const int slot = Slot(frame.baseSerial);
  if (slot >= 0) return OpticalUpdate(frame, base->second.config, slot);
  if (frame.highFrameJitter) return false;
  return Bootstrap(frame, base->second.config);
}

bool LighthouseDirect::AdvanceInertial(double time) {
  if (!slots_[0] || time <= time_ || time < lastOpticalTime_ + std::max(0.05, 1.1 * sweepInterval_)) return false;
  const double previous = time_;
  PredictTo(time);
  return time_ > previous;
}

std::optional<TrackedPose> LighthouseDirect::Pose() const {
  if (!slots_[0]) return std::nullopt;
  TrackedPose pose;
  pose.time = time_; pose.primaryBase = slots_[0];
  for (int i = 0; i < 3; ++i) {
    pose.pose.position[size_t(i)] = filter_.x[i];
    pose.velocity[size_t(i)] = filter_.x[3 + i];
    pose.angularVelocity[size_t(i)] = filter_.x[13 + i];
  }
  for (int i = 0; i < 4; ++i) pose.pose.orientation[size_t(i)] = filter_.x[9 + i];
  const double age = std::max(time_, observedTime_.value_or(time_)) - lastOpticalTime_;
  pose.status = age > opticalTimeout_ ? TrackingStatus::Lost :
                (age > 0.05 ? TrackingStatus::Inertial : TrackingStatus::Optical);
  pose.opticalUpdates = filter_.updateCount; pose.flags = filter_.flags;
  return pose;
}

LighthouseDirect::Predicted LighthouseDirect::PredictNow() const {
  const double* x = filter_.x;
  Predicted out;
  out.time = time_;
  Eigen::Vector3d p(x[0], x[1], x[2]), v(x[3], x[4], x[5]), w(x[13], x[14], x[15]);
  Eigen::Quaterniond q(x[9], x[10], x[11], x[12]);
  q.normalize();
  // IMU frame -> body (inverse of the angle-axis x[28..30]), biases, scales, gravity.
  const Eigen::Vector3d imuAA(x[28], x[29], x[30]);
  const Eigen::Matrix3d imuRot = imuAA.norm() > 1e-12 ? Eigen::AngleAxisd(imuAA.norm(), imuAA.normalized()).toRotationMatrix()
                                                         : Eigen::Matrix3d::Identity();
  const Eigen::Vector3d tilt(x[16], 0, x[17]);
  const Eigen::Matrix3d tiltRot = tilt.norm() > 1e-12 ? Eigen::AngleAxisd(tilt.norm(), tilt.normalized()).toRotationMatrix()
                                                       : Eigen::Matrix3d::Identity();
  const Eigen::Vector3d gravity = tiltRot * Eigen::Vector3d(0, x[18], 0);
  double t = time_;
  Eigen::Vector3d lastA(x[6], x[7], x[8]);
  for (const auto& s : imu_) {
    const double dt = s.time - t;
    if (dt <= 0) continue;
    if (dt > 0.05) break;  // a gap: don't integrate across it
    if (!(s.saturated & 0x38)) {
      w = imuRot.transpose() * (Eigen::Vector3d(s.gyro[0], s.gyro[1], s.gyro[2]) - Eigen::Vector3d(x[19], x[20], x[21]));
    }
    // A clipped accelerometer reading is used as it is: it says the device accelerates at
    // least that hard. Replacing the sample with the filter's acceleration (up to 120 ms
    // old) ignored the hard stop at the end of a punch, the hardest acceleration of all,
    // and the output flew on past the stop until the late light pulled it back (a user's
    // Index controller: 10 cm past and back in 65 ms; error while moving p90 96 -> 52 mm).
    Eigen::Vector3d a;
    {
      const Eigen::Vector3d meas(s.accel[0], s.accel[1], s.accel[2]);
      const Eigen::Vector3d scaled((meas[0] - x[22]) * x[25], (meas[1] - x[23]) * x[26], (meas[2] - x[24]) * x[27]);
      a = q * (imuRot.transpose() * scaled) - gravity;
    }
    p += v * dt + 0.5 * a * dt * dt;
    v += a * dt;
    lastA = a;
    const Eigen::Vector3d half = 0.5 * dt * w;
    const double angle = half.norm();
    Eigen::Quaterniond dq(1, 0, 0, 0);
    if (angle > 0) {
      const Eigen::Vector3d u = half * (std::sin(angle) / angle);
      dq = Eigen::Quaterniond(std::cos(angle), u.x(), u.y(), u.z());
    }
    q = (q * dq).normalized();
    t = s.time;
  }
  out.time = t;
  for (int i = 0; i < 3; ++i) { out.p[i] = p[i]; out.v[i] = v[i]; out.w[i] = w[i]; out.a[i] = lastA[i]; }
  out.q[0] = q.w(); out.q[1] = q.x(); out.q[2] = q.y(); out.q[3] = q.z();
  return out;
}

int LighthouseDirect::ExpectedHits(int sensor, double t, ExpectedHit* out) const {
  // A pose is only worth asking if light has pinned it down lately. A frame of a sensor or two
  // doesn't: on those alone a still headset slid 3 m, refusing what would have held it.
  constexpr double kTooOld = 0.25;
  if (!slots_[0] || observedTime_.value_or(time_) - lastWellLit_ > kTooOld) return 0;
  const int idx = model_.IndexOf(sensor);
  if (idx < 0) return 0;
  if (expectedFilterTime_ != time_ || (!imu_.empty() && imu_.back().time - expectedTime_ > 0.002)) {
    const Predicted now = PredictNow();
    std::copy(filter_.x, filter_.x + direct::kStateSize, expectedState_);
    for (int i = 0; i < 3; ++i) {
      expectedState_[i] = now.p[i];
      expectedState_[3 + i] = now.v[i];
      expectedState_[6 + i] = now.a[i];
      expectedState_[13 + i] = now.w[i];
    }
    for (int i = 0; i < 4; ++i) expectedState_[9 + i] = now.q[i];
    expectedTime_ = now.time;
    expectedFilterTime_ = time_;
  }
  const double dt = t - expectedTime_;
  if (std::fabs(dt) > 0.1) return 0;
  const auto point = model_.points[size_t(idx)], normal = model_.normals[size_t(idx)];
  // How far the pose may have drifted since light last corrected it: a share of how far it has
  // gone since, as a hard swing runs past what the accelerometer reads.
  constexpr double kShare = 0.5;
  constexpr double kMostWindow = 0.15;  // any wider and a hit often fits more than one station
  const double sinceLight = std::max(0.0, t - time_);
  const double speed = Eigen::Vector3d(expectedState_ + 3).norm();
  const double accel = Eigen::Vector3d(expectedState_ + 6).norm();
  const double adrift = kShare * (speed * sinceLight + 0.5 * accel * sinceLight * sinceLight);
  int n = 0;
  for (int slot = 0; slot < 4; ++slot) {
    if (!slots_[slot]) continue;
    const auto base = bases_.find(slots_[slot]);
    if (base == bases_.end()) continue;
    const int alt = slot == 0 ? -1 : kAltBaseIndex[slot - 1];
    const auto& config = base->second.config;
    const double facing = SensorNormalCosAngle(expectedState_, dt, point.data(), normal.data(), alt);
    double inBase[3];
    SensorInBase(expectedState_, dt, point.data(), alt, inBase);
    const double distance = std::max(0.3, Eigen::Vector3d(inBase).norm());
    for (int axis = 0; axis < 2; ++axis) {
      ExpectedHit& e = out[n++];
      e.base = slots_[slot];
      e.lh2 = config.model != 1;
      e.axis = axis;
      e.angle = SensorAngular(expectedState_, dt, point.data(), axis, config.cal.data(), alt, config.model);
      e.facing = facing > 0 && std::isfinite(e.angle);
      e.window = std::min(kMostWindow, adrift / distance);
    }
  }
  return n;
}

DeviceTrackingInfo LighthouseDirect::Info() const {
  DeviceTrackingInfo info;
  info.configured = true; info.primaryBase = slots_[0];
  info.opticalUpdates = filter_.updateCount; info.bootstrapAttempts = bootstrapAttempts_;
  info.rejectedUpdates = uint64_t(filter_.rejectCount);
  if (auto pose = Pose()) info.status = pose->status;
  return info;
}

std::vector<LighthouseDirect::AltEstimate> LighthouseDirect::AltEstimates() const {
  std::vector<AltEstimate> out;
  for (int slot = 1; slot < 4; ++slot) {
    if (!slots_[slot] || !filter_.altBaseActive[slot - 1]) continue;
    const int idx = kAltBaseIndex[slot - 1];
    double pose[7];
    AltBasePose(filter_.x, idx, pose);
    AltEstimate e;
    e.serial = slots_[slot];
    e.altFromPrimary.linear() = Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).normalized().toRotationMatrix();
    e.altFromPrimary.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    e.translationVariance = filter_.P.diagonal().segment<3>(idx).sum();
    e.rotationVariance = filter_.P.diagonal().segment<3>(idx + 3).sum();
    out.push_back(e);
  }
  return out;
}

bool LighthouseDirect::SeedPose(const Seed& seed) {
  if (!bases_.count(seed.base)) return false;
  if (seed.carry && !seed.carry->CalibrationHealthy()) return false;
  const Eigen::Quaterniond q(seed.baseFromImu.rotation());
  if (!q.coeffs().allFinite() || !seed.baseFromImu.translation().allFinite()) return false;
  filter_.DefaultState();
  filter_.P.setZero();
  filter_.InitialCovariance();
  if (seed.carry) {
    for (int i = 18; i < 31; ++i) filter_.x[i] = seed.carry->x[i];
    filter_.P.block(18, 18, 13, 13) = seed.carry->P.block(18, 18, 13, 13);
  } else {
    for (int i = 0; i < 3; ++i) {
      filter_.x[19 + i] = double(float(model_.gyroBias[size_t(i)]));
      filter_.x[22 + i] = double(float(model_.accBias[size_t(i)]));
      filter_.x[25 + i] = double(float(model_.accScale[size_t(i)]));
    }
  }
  for (int i = 0; i < 3; ++i) {
    filter_.x[i] = seed.baseFromImu.translation()[i];
    filter_.x[3 + i] = seed.velocity[i];
    filter_.x[13 + i] = seed.angularVelocity[i];
    filter_.P(i, i) = seed.positionVariance;
    filter_.P(3 + i, 3 + i) = seed.velocityVariance;
    filter_.P(13 + i, 13 + i) = 1.6e-5;
  }
  filter_.x[9] = q.w(); filter_.x[10] = q.x(); filter_.x[11] = q.y(); filter_.x[12] = q.z();
  for (int i = 9; i < 13; ++i) filter_.P(i, i) = seed.rotationVariance;
  filter_.x[16] = seed.tilt[0]; filter_.x[17] = seed.tilt[1];
  filter_.P(16, 16) = filter_.P(17, 17) = seed.tiltVariance;
  filter_.n = 31; filter_.flags &= ~8u; filter_.rejectCount = 0;
  history_.clear();
  filter_.altBaseActive[0] = filter_.altBaseActive[1] = filter_.altBaseActive[2] = false;
  filter_.updateCount = seed.calibrated ? 201 : 101;
  altGateUntil_ = filter_.updateCount + 100;
  calibrationSeen_ = seed.calibrated;
  slots_[0] = seed.base;
  for (int slot = 1; slot < 4; ++slot) slots_[slot] = 0;
  time_ = lastOpticalTime_ = seed.time;
  lastWellLit_ = seed.time;  // whoever seeded it has just pinned it down
  while (!imu_.empty() && imu_.front().time < seed.time) imu_.pop_front();
  return true;
}

bool LighthouseDirect::AddKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                                       double translationVariance, double rotationVariance, bool lit,
                                       uint32_t* replaced) {
  if (!slots_[0] || Slot(serial) >= 0 || !bases_.count(serial)) return false;
  int slot = 1;
  while (slot < 4 && slots_[slot]) ++slot;
  if (slot == 4) {
    slot = lit ? SlotToGiveUp(slotUsed_, time_) : 0;
    if (slot == 0) return false;
    if (replaced) *replaced = slots_[slot];
    --secondaryBases_;
  }
  const Eigen::AngleAxisd aa(altFromPrimary.rotation());
  const Eigen::Vector3d rotation = aa.axis() * aa.angle();
  const int idx = kAltBaseIndex[slot - 1];
  for (int i = 0; i < 3; ++i) {
    filter_.x[idx + i] = altFromPrimary.translation()[i];
    filter_.x[idx + 3 + i] = rotation[i];
  }
  filter_.P.middleRows(idx, 6).setZero(); filter_.P.middleCols(idx, 6).setZero();
  filter_.P.diagonal().segment<3>(idx).setConstant(std::max(translationVariance / 3, 1e-8));
  filter_.P.diagonal().segment<3>(idx + 3).setConstant(std::max(rotationVariance / 3, 1e-9));
  filter_.altBaseActive[slot - 1] = true;
  filter_.altBaseRelearn[slot - 1] = false;
  filter_.n = std::max(filter_.n, idx + 6);
  slots_[slot] = serial; ++secondaryBases_;
  slotUsed_[slot] = time_;
  return true;
}

bool LighthouseDirect::SetKnownAltBase(uint32_t serial, const Eigen::Isometry3d& altFromPrimary,
                                       double translationVariance, double rotationVariance) {
  const int slot = Slot(serial);
  if (slot <= 0 || !filter_.altBaseActive[slot - 1]) return false;
  const Eigen::AngleAxisd aa(altFromPrimary.rotation());
  const Eigen::Vector3d rotation = aa.axis() * aa.angle();
  const int idx = kAltBaseIndex[slot - 1];
  for (int i = 0; i < 3; ++i) {
    filter_.x[idx + i] = altFromPrimary.translation()[i];
    filter_.x[idx + 3 + i] = rotation[i];
  }
  filter_.P.middleRows(idx, 6).setZero(); filter_.P.middleCols(idx, 6).setZero();
  filter_.P.diagonal().segment<3>(idx).setConstant(std::max(translationVariance / 3, 1e-8));
  filter_.P.diagonal().segment<3>(idx + 3).setConstant(std::max(rotationVariance / 3, 1e-9));
  return true;
}

void LighthouseDirect::SetAltBaseRelearn(uint32_t serial, bool relearn) {
  const int slot = Slot(serial);
  if (slot > 0) filter_.altBaseRelearn[slot - 1] = relearn;
}

std::string LighthouseDirect::HealthCheck(double now) const {
  // 0x358300 with its constants (0x1a99f8, 0x1a9af8, 0x1a9978, 0x351610, 0x1a9688).
  if (filter_.updateCount < 101) return {};
  const double* x = filter_.x;
  char text[160];
  std::string reason;
  if (std::fabs(x[16]) > 3.9269908169872414 || std::fabs(x[17]) > 3.9269908169872414) {
    std::snprintf(text, sizeof text, "bad base station orientation ( %.4g, %.4g )", x[16] * 57.29577951308232,
                  x[17] * 57.29577951308232);
    reason = text;
  } else if (std::fabs(x[18] - 9.80665) > filter_.gravityTolerance) {
    std::snprintf(text, sizeof text, "unlikely gravity %.3g m/s/s", x[18]);
    reason = text;
  } else if (x[28] * x[28] + x[29] * x[29] + x[30] * x[30] > 0.06853891945200943) {
    reason = "IMU misalignment unreasonably large";
  } else {
    bool inBounds = false;
    for (int slot = 0; slot < 4 && !inBounds; ++slot) {
      if (!slots_[slot]) continue;
      Eigen::Vector3d p(x[0], x[1], x[2]);
      if (slot > 0 && filter_.altBaseActive[slot - 1]) {
        double alt[7], r[3];
        AltBasePose(x, kAltBaseIndex[slot - 1], alt);
        model::QuatRotate(alt + 3, p.data(), r);
        p = Eigen::Vector3d(r[0] + alt[0], r[1] + alt[1], r[2] + alt[2]);
      }
      inBounds = p.z() <= -0.1 && p.squaredNorm() <= 49.0;
    }
    if (!inBounds) {
      std::snprintf(text, sizeof text, "position out of bounds (%.2f, %.2f, %.2f)", x[0], x[1], x[2]);
      reason = text;
    } else {
      const double w2 = x[13] * x[13] + x[14] * x[14] + x[15] * x[15];
      if (w2 <= filter_.omegaCorrectionLimit2) {
        if (now - lastOpticalTime_ <= 2.0) return {};
        int bases = 0;
        for (uint32_t b : slots_) bases += b != 0;
        if (bases == 1) std::snprintf(text, sizeof text, "no optical samples from base %08X for %dms", slots_[0],
                                      int((now - lastOpticalTime_) * 1000));
        else std::snprintf(text, sizeof text, "no optical samples for %dms", int((now - lastOpticalTime_) * 1000));
        reason = text;
      } else {
        std::snprintf(text, sizeof text, "rotating at %.1f deg/s", std::sqrt(w2) * 57.29577951308232);
        reason = text;
      }
    }
  }
  return (filter_.updateCount < 201 ? "Calibration failed: " : "Resetting tracking: ") + reason;
}

}  // namespace lh
