// Lighthouse 2.0 ("sync on beam", SOB) disambiguator: LH2 light words -> optical frames.
//
// After vortex::CDisambiguatorSyncOnBeam (vtable 0x4710f8; beam word 0x3207e0, sensor
// word 0x31e610, frame builder 0x31f340). An LH2 base has no sync flash: each of its
// channels (0-15) spins at its own rate (period table 0x1a1ce0, 48 MHz ticks) and the
// device's FPGA, decoding the beam's LFSR, reports
//  * a beam word once per revolution, at a fixed rotor phase: the revolution's sync,
//    carrying one OOTX bit (bit 26);
//  * a sensor word per hit: sensor id and the time the beam crossed it.
// A hit's angle is 2*pi*(t - sync)/period. Each revolution sweeps the device twice, with
// two planes tilted -/+30 deg: hits in the first half turn are plane 0 (frame axis 0),
// in the second plane 1 (axis 1). The gen2 measurement model (lighthouse_model.hpp,
// Lh2Angle) maps them to a position.
//
// Frames are built per revolution when the next beam word arrives (the right sync is
// then measured), or once the revolution is overdue. All bases sweep at once (unlike LH1's
// turns), so frames of different channels overlap in time while the filter only takes
// them in time order (it drops a frame older than the last one applied): frames are held
// and released in order of their first hit, kHold behind the newest word seen.
#pragma once

#include <array>
#include <bitset>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "lh/optical.hpp"
#include "tracking/ootx.hpp"

namespace lh::sob {

constexpr int kChannels = 16;
// Ticks (48 MHz) per revolution of each channel (0x1a1ce0): 50.05 .. 54.11 Hz.
constexpr std::array<uint32_t, kChannels> kPeriodTicks = {959000, 957000, 953000, 949000, 947000, 943000,
                                                          941000, 939000, 937000, 929000, 919000, 911000,
                                                          907000, 901000, 893000, 887000};
constexpr double kTickSeconds = 1.0 / 48e6;  // 0x1a96f0

class SobDisambiguator {
 public:
  void OnBeamWord(double t, int channel, int ootxBit, int generation);
  // sorted: a hit PulseSorter picked out of LH1 pulses; its samples get kSampleSorted.
  void OnSensorWord(double t, int channel, int sensor, bool sorted = false);
  // Emits revolutions whose next beam word is overdue (call with the latest device time).
  void Advance(double t);
  void Reset();
  // Bases whose channel is known (SteamVR database, or another device's OOTX): their
  // frames flow at once instead of after a whole OOTX frame (~8 s of unbroken light).
  // The base's stored configuration is announced (PopBaseConfig) when its channel shows up,
  // as if its OOTX had been decoded: the session then knows the base is in view.
  void SetKnownChannels(const std::map<int, std::pair<BaseConfig, BaseDynamicState>>& channels,
                        std::bitset<kChannels> blocked = {});

  // True once any LH2 word was seen.
  bool Active() const { return active_; }
  // The base on this channel, 0 while it isn't known.
  uint32_t ChannelSerial(int channel) const {
    return channel >= 0 && channel < kChannels && !blocked_[size_t(channel)] ? channels_[size_t(channel)].serial : 0;
  }
  bool PopFrame(OpticalFrame* out);
  // For merging with LH1 frames, since the filter drops a frame older than the last it used:
  // the oldest queued frame's first hit, and the time no older frame can still be built before.
  bool OldestFrameTime(double* t) const;
  double SettledUpTo() const;
  bool PopBaseConfig(BaseConfig* cfg, BaseDynamicState* dyn);
  bool PopLog(std::string* out);

 private:
  struct Hit {
    double time;
    int sensor;
    bool sorted = false;
  };
  struct Channel {
    bool seen = false;
    int generation = -1;
    double nominal = 0;      // nominal period (s)
    double period = 0;       // smoothed measured period (s)
    double sync = 0;         // current revolution's beam word time
    bool syncReal = false;   // sync was measured (not predicted)
    int ootxBit = -1;        // bit carried by the current revolution's beam word
    std::vector<Hit> hits;   // current revolution
    ootx::OotxFramer framer;  // fast path: one unbroken frame
    // Robust path, as stock's revolution-indexed bit buffer (0x3b7430/0x3b8140): one slot
    // per revolution, missed revolutions keep their place, repeated frames vote per bit.
    // 443 bits = an OOTX version 8 frame (0x2b-byte payload), what 2.0 bases send.
    ootx::OotxMajorityRing ring{443};
    long ringPushes = 0;
    uint32_t serial = 0;
    int uniqueId = 0;
    int frames = 0;
  };
  void Emit(Channel& c, int channel, double next, bool nextReal);
  void PushOotx(Channel& c, int channel, int bit);  // bit < 0: revolution without a beam word
  void Skip(Channel& c, long revolutions);
  void Log(const std::string& s);
  void DiscardFrames(Channel& c);

  std::map<int, std::pair<BaseConfig, BaseDynamicState>> known_;
  std::bitset<kChannels> blocked_;
  void Announce(Channel& c, int channel, const BaseConfig& cfg, BaseDynamicState dyn);
  void QueueConfig(int channel, const BaseConfig& cfg, BaseDynamicState dyn);
  bool active_ = false;
  int nextUniqueId_ = 0;
  std::array<Channel, kChannels> channels_{};
  std::deque<OpticalFrame> frames_;  // sorted by first hit
  double latest_ = 0;                // newest word time seen
  std::deque<std::pair<BaseConfig, BaseDynamicState>> configs_;
  std::deque<std::string> logs_;
};

}  // namespace lh::sob
