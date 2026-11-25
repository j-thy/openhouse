#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include "tracking_source.hpp"

#include <cstdlib>
#include <string>

namespace lh {

namespace {
tdm::MultiDisambiguator::Mode MultiModeFromEnvironment() {
  const char* v = std::getenv("LH_MULTISTATION");
  if (!v) return tdm::MultiDisambiguator::Mode::Auto;
  const std::string s = v;
  return s == "off" || s == "0" ? tdm::MultiDisambiguator::Mode::Off
       : s == "on" || s == "1"  ? tdm::MultiDisambiguator::Mode::On
                                : tdm::MultiDisambiguator::Mode::Auto;
}
}  // namespace

TrackingSource::TrackingSource(int sensorType, const tdm::TdmConfig& config, bool inferSensorType)
    : inferSensorType_(inferSensorType), tdm_(config, sensorType), multiMode_(MultiModeFromEnvironment()),
      multi_(sensorType, multiMode_) {}

void TrackingSource::OnImu(const ImuSample& sample) {
  std::lock_guard lock(mutex_);
  // SwSync_OnImu (0x3e2d90) snapshots these before Source_PushImu.
  const double previousImuTime = latestImuTime_;
  QueuedImu queued{sample.time, {}, {}, sample.saturated};
  for (int i = 0; i < 3; ++i) {
    queued.accel[size_t(i)] = float(sample.accel[size_t(i)]);
    queued.gyro[size_t(i)] = float(sample.gyro[size_t(i)]);
  }
  if (imu_.size() >= 1000) imu_.pop_front();
  imu_.push_back(queued);
  latestImuTime_ = sample.time;

  // LH2 words reach us well behind the IMU (batched reports); only light this late is missing.
  if (sob_.Active()) sob_.Advance(sample.time - 0.25);
  // Equality means a pulse arrived after the previous IMU sample. Do not
  // advance in that case. Keep the strict comparisons and zero sentinel.
  if (lastPulseImuTime_ != previousImuTime &&
      (nextAdvanceCheck_ == 0 || sample.time > nextAdvanceCheck_)) {
    if (lastPulseImuTime_ + 0.02 < sample.time) {
      tdm_.Advance(sample.time - 0.02);
      multi_.Advance(sample.time - 0.02);
    }
    // Once enough sync widths are seen, re-derive the classifier offset from them.
    else if (inferSensorType_ && tdm_.SensorType() == 0 && !sample.saturated) tdm_.RetrySensorType();
    nextAdvanceCheck_ = sample.time + 0.05;
  }
}

void TrackingSource::OnLightPulse(const LightPulse& pulse) {
  std::lock_guard lock(mutex_);
  if (sorting_) {
    // Reports can come grouped by sensor, so a pulse may be a little older than the last one.
    constexpr double kBeamDelay = 0.005;
    sorter_->DueBeamWords(pulse.time - kBeamDelay, [&](double t, int channel, int generation) {
      sob_.OnBeamWord(t, channel, -1, generation);
    });
    int channel = -1;
    double hitTime = 0;
    sorter_->SetFlashSchedule(!UseMulti());  // lhbs multi-station flashes keep their own schedule
    const auto result = sorter_->Sort(pulse.time, pulse.width, pulse.sensor, &channel, &hitTime);
    if (result == PulseSorter::Result::Hit) sob_.OnSensorWord(hitTime, channel, pulse.sensor, true);
    if (result != PulseSorter::Result::NotOurs) return;
  }
  lastPulseImuTime_ = latestImuTime_;
  const int type = tdm_.SensorType();
  tdm_.OnPulse(pulse.sensor, pulse.time, pulse.width);
  if (tdm_.SensorType() != type) multi_.SetSensorType(tdm_.SensorType());
  if (!sorting_ || multiMode_ == tdm::MultiDisambiguator::Mode::On) multi_.OnPulse(pulse.sensor, pulse.time, pulse.width);
  if (UseMulti()) {  // the stock path's frames would be mis-attributed; drop them
    while (tdm_.PopFrame(nullptr)) {}
  }
}

void TrackingSource::OnLh2BeamWord(const Lh2BeamWord& word) {
  std::lock_guard lock(mutex_);
  if (sorter_) sorter_->OnBeamWord(word.time, word.channel, word.bit27);
  sob_.OnBeamWord(word.time, word.channel, word.bit26, word.bit27);
}

void TrackingSource::OnLh2SensorWord(const Lh2SensorWord& word) {
  std::lock_guard lock(mutex_);
  if (sorter_) sorter_->OnSensorWord(word.time, word.channel, word.value);
  sob_.OnSensorWord(word.time, word.channel, word.value);
}

bool TrackingSource::PopImu(ImuSample* out) {
  std::lock_guard lock(mutex_);
  if (imu_.empty()) return false;
  const auto& sample = imu_.front();
  out->time = sample.time;
  out->saturated = sample.saturated;
  for (int i = 0; i < 3; ++i) {
    out->accel[size_t(i)] = sample.accel[size_t(i)];
    out->gyro[size_t(i)] = sample.gyro[size_t(i)];
  }
  imu_.pop_front();
  return true;
}

bool TrackingSource::PeekImuTime(DeviceTime* out) const {
  std::lock_guard lock(mutex_);
  if (imu_.empty()) return false;
  *out = imu_.front().time;
  return true;
}

bool TrackingSource::PopFrame(OpticalFrame* out) {
  std::lock_guard lock(mutex_);
  if (sorting_) {
    // Both paths feed the filter here, and it drops a frame older than the last it used. LH2
    // frames are held back, so an LH1 frame waits for the LH2 frames older than it.
    OpticalFrame frame;
    while (UseMulti() ? multi_.PopFrame(&frame) : tdm_.PopFrame(&frame)) pendingLh1_.push_back(std::move(frame));
    if (pendingLh1_.empty()) return sob_.PopFrame(out);
    double first = 1e300;
    for (const auto& s : pendingLh1_.front().samples) first = std::min(first, s.time + 0.5 * s.width);
    if (pendingLh1_.front().samples.empty()) first = pendingLh1_.front().syncTime;  // a sweep with no hits
    double oldest = 0;
    if (sob_.OldestFrameTime(&oldest) && oldest <= first) return sob_.PopFrame(out);
    if (sob_.Active() && first > sob_.SettledUpTo()) return false;
    if (out) *out = std::move(pendingLh1_.front());
    pendingLh1_.pop_front();
    return true;
  }
  if (!pendingLh1_.empty()) {  // held back when sorting was turned off
    if (out) *out = std::move(pendingLh1_.front());
    pendingLh1_.pop_front();
    return true;
  }
  if (sob_.PopFrame(out)) return true;
  return UseMulti() ? multi_.PopFrame(out) : tdm_.PopFrame(out);
}

bool TrackingSource::PopSobBaseConfig(BaseConfig* config, BaseDynamicState* dynamic) {
  std::lock_guard lock(mutex_);
  return sob_.PopBaseConfig(config, dynamic);
}

bool TrackingSource::PopBaseConfig(BaseConfig* config, BaseDynamicState* dynamic) {
  std::lock_guard lock(mutex_);
  if (sob_.PopBaseConfig(config, dynamic)) return true;
  if (UseMulti()) {
    while (tdm_.PopBaseConfig(nullptr, nullptr)) {}
    return multi_.PopBaseConfig(config, dynamic);
  }
  return tdm_.PopBaseConfig(config, dynamic);
}

bool TrackingSource::PopEvent(tdm::TdmEvent* out) {
  std::lock_guard lock(mutex_);
  return tdm_.PopEvent(out);
}

bool TrackingSource::PopLog(std::string* out) {
  std::lock_guard lock(mutex_);
  if (sob_.PopLog(out)) { *out = "[lh2] " + *out; return true; }
  // The multi-station detector's messages matter once it is active; before that it may be
  // trying to make sense of light that isn't lhbs at all (LH2 bases seen in LH1 mode).
  while (multi_.PopLog(out)) {
    if (!UseMulti() || sob_.Active()) continue;
    *out = "[multi] " + *out;
    return true;
  }
  if (UseMulti()) {  // the two-base sync path re-acquiring in a multi room: noise
    while (tdm_.PopLog(nullptr)) {}
    return false;
  }
  return tdm_.PopLog(out);
}

void TrackingSource::SetKnownIdentities(std::vector<tdm::TdmDisambiguator::Identity> identities) {
  std::lock_guard lock(mutex_);
  multi_.SetKnownIdentities(identities);
  tdm_.SetKnownIdentities(std::move(identities));
}

void TrackingSource::SetKnownSobChannels(const std::map<int, std::pair<BaseConfig, BaseDynamicState>>& channels,
                                         std::bitset<sob::kChannels> blocked) {
  std::lock_guard lock(mutex_);
  sob_.SetKnownChannels(channels, blocked);
}

bool TrackingSource::PopIdentity(tdm::TdmDisambiguator::Identity* out) {
  std::lock_guard lock(mutex_);
  return multi_.PopIdentity(out) || tdm_.PopIdentity(out);
}

void TrackingSource::SetSorting(bool learn, bool sort) {
  std::lock_guard lock(mutex_);
  learn = learn || sort;
  sorting_ = sort;
  if (learn == (sorter_ != nullptr)) return;
  if (!learn) {
    sorter_.reset();
    return;
  }
  sorter_ = std::make_unique<PulseSorter>();
  if (poseView_) sorter_->SetPose(poseView_, [this](int channel) { return sob_.ChannelSerial(channel); });
}

bool TrackingSource::Sorting() const {
  std::lock_guard lock(mutex_);
  return sorting_;
}

void TrackingSource::SetPoseView(PulseSorter::PoseView view) {
  std::lock_guard lock(mutex_);
  poseView_ = std::move(view);
  if (sorter_) sorter_->SetPose(poseView_, [this](int channel) { return sob_.ChannelSerial(channel); });
}

std::vector<PulseSorter::RotorState> TrackingSource::Rotors(double now) const {
  std::lock_guard lock(mutex_);
  return sorter_ ? sorter_->Rotors(now) : std::vector<PulseSorter::RotorState>{};
}

bool TrackingSource::RecentPeriod(int channel, double now, double* period) const {
  std::lock_guard lock(mutex_);
  return sorter_ && sorter_->RecentPeriod(channel, now, period);
}

void TrackingSource::SetRotorDrift(int channel, double drift) {
  std::lock_guard lock(mutex_);
  if (sorter_) sorter_->SetDrift(channel, drift);
}

uint32_t TrackingSource::ChannelSerial(int channel) const {
  std::lock_guard lock(mutex_);
  return sob_.ChannelSerial(channel);
}

}  // namespace lh
