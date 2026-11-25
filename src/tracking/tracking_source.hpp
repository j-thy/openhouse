// Per-device LH1 source: normalized transport events -> tracker input queues.
// Port of the TDM path of CSwSyncDetectSource; no USB or host clock conversion.
// LH2 words go to the sync-on-beam disambiguator (disambiguator_sob.hpp).
#pragma once

#include <deque>
#include <memory>
#include <mutex>

#include "lh/types.hpp"
#include "tracking/disambiguator_multi.hpp"
#include "tracking/disambiguator_sob.hpp"
#include "tracking/disambiguator_tdm.hpp"
#include "tracking/pulse_sorter.hpp"

namespace lh {

class TrackingSource {
 public:
  // inferSensorType: re-derive the type from measured sync widths while it is 0.
  // Off when the type came from the device's hardware id (report 0x05).
  explicit TrackingSource(int sensorType = 0, const tdm::TdmConfig& config = {}, bool inferSensorType = true);

  void OnImu(const ImuSample& sample);
  void OnLightPulse(const LightPulse& pulse);
  // LH2 (2.0 base stations): device-decoded beam and sensor words.
  void OnLh2BeamWord(const Lh2BeamWord& word);
  void OnLh2SensorWord(const Lh2SensorWord& word);

  bool PopImu(ImuSample* out);
  bool PeekImuTime(DeviceTime* out) const;
  bool PopFrame(OpticalFrame* out);
  const tdm::TdmStats& Stats() const { return tdm_.Stats(); }
  bool PopSobBaseConfig(BaseConfig* config, BaseDynamicState* dynamic);
  bool PopBaseConfig(BaseConfig* config, BaseDynamicState* dynamic);
  bool PopEvent(tdm::TdmEvent* out);
  bool PopLog(std::string* out);
  // Fast base identification (see TdmDisambiguator::Identity).
  void SetKnownIdentities(std::vector<tdm::TdmDisambiguator::Identity> identities);
  // Whether the stations run a multi-station round (lhbs), as this device found it; and the
  // session's answer, shared with every device.
  bool MultiLayout() const { std::lock_guard lock(mutex_); return multi_.LayoutKnown(); }
  void SetMultiLayout(bool known) { std::lock_guard lock(mutex_); if (multi_.LayoutKnown() != known) multi_.SetLayoutKnown(known); }
  bool PopIdentity(tdm::TdmDisambiguator::Identity* out);
  void SetKnownSobChannels(const std::map<int, std::pair<BaseConfig, BaseDynamicState>>& channels,
                           std::bitset<sob::kChannels> blocked = {});

  // Not stock, for rooms with both kinds of base station (ai/driver/mixed_stations.md): LH1
  // pulses from 2.0 stations go down the LH2 path (PulseSorter), the rest down the LH1 path as
  // usual. Off by default. learn: the sorter learns rotors from LH2 words, sorting nothing yet, so
  // it's ready when sorting starts. sort: LH1 pulses are sorted too (and it learns), and lhbs
  // multi-station detection waits (LH_MULTISTATION=on still forces it).
  void SetSorting(bool learn, bool sort);
  bool Sorting() const;  // sort
  // Lets the tracker's pose help decide whose each pulse is, while sorting.
  void SetPoseView(PulseSorter::PoseView view);
  // The sorter's rotors; none without a sorter.
  std::vector<PulseSorter::RotorState> Rotors(double now) const;
  // The scout (runtime/mixed_light.hpp), under the source's lock: PulseSorter::RecentPeriod and
  // SetDrift. Without a sorter, no period and nothing to set.
  bool RecentPeriod(int channel, double now, double* period) const;
  void SetRotorDrift(int channel, double drift);
  // The base on an LH2 channel, 0 while it isn't known.
  uint32_t ChannelSerial(int channel) const;

 private:
  // The original stores float components, even though transport callbacks use doubles.
  struct QueuedImu {
    double time;
    std::array<float, 3> accel, gyro;
    uint32_t saturated;
  };
  mutable std::mutex mutex_;
  bool inferSensorType_ = true;
  tdm::TdmDisambiguator tdm_;
  const tdm::MultiDisambiguator::Mode multiMode_;  // LH_MULTISTATION
  // Runs beside the stock disambiguator; takes over once it detects lhbs multi mode.
  tdm::MultiDisambiguator multi_;
  // LH2 light; a device reports either LH1 pulses or LH2 words (its light mode).
  sob::SobDisambiguator sob_;
  // SetSorting. While it sorts, multi_ waits, as it was off for every test of this: a lone stock
  // 1.0 station passed for an lhbs multi-station layout there (ai/driver/mixed_stations.md).
  std::unique_ptr<PulseSorter> sorter_;  // learning, and sorting if sorting_
  bool sorting_ = false;
  PulseSorter::PoseView poseView_;
  std::deque<OpticalFrame> pendingLh1_;  // LH1 frames waiting for older LH2 frames
  bool UseMulti() const { return multi_.Active() && (!sorting_ || multiMode_ == tdm::MultiDisambiguator::Mode::On); }
  std::deque<QueuedImu> imu_;
  double latestImuTime_ = 0;
  double lastPulseImuTime_ = 0;
  double nextAdvanceCheck_ = 0;
};

}  // namespace lh
