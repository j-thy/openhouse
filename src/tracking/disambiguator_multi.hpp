// Lighthouse 1.0 multi-station disambiguator for lhbs multi mode (N = 2..4 stations
// taking whole revolutions in turn, flashes staggered by 250 us; see
// ai/firmware/MULTISTATION.md and ai/driver/multistation_plan.md).
//
// Not a port: the stock CDisambiguatorTDM (disambiguator_tdm.cpp) stays untouched and
// bit-exact. This class watches the same pulse stream all the time and only becomes
// Active() on positive evidence of the multi layout (flashes a multiple of 250 us apart
// but not 417 us, or a lone station sweeping after far fewer than half its flashes).
// TrackingSource then takes frames and base configs from it instead.
//
// Algorithm:
//  1. Sync-width pulses starting within 3 us on several sensors form one flash; flashes
//     within 1 ms form an index-event group (one every half revolution).
//  2. A reference (the master's flash time) is predicted every half period; each flash
//     gets slot round((t - ref) / 250 us). The reference re-anchors to an earlier
//     station once one keeps showing up before it, so slot 0 is the earliest (the
//     master); a station's OOTX slot, once decoded, corrects the numbering.
//  3. Each station (slot) decodes its own OOTX from its own flashes. A station that
//     flashes every index event (master, or policy ALL) pushes "no data" when it is
//     missed, as stock does.
//  4. A skip = 0 flash means that station sweeps now: every pulse in
//     [sync + P/12, sync + 5P/12] is a sample with angle 2*pi*(mid - sync)/P, P the
//     master-locked rotor period.
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "lh/optical.hpp"
#include "tracking/disambiguator_tdm.hpp"
#include "tracking/ootx.hpp"

namespace lh::tdm {

class MultiDisambiguator {
 public:
  enum class Mode { Auto, Off, On };  // LH_MULTISTATION=auto|off|on
  explicit MultiDisambiguator(int sensorType = 0, Mode mode = Mode::Auto);

  void SetSensorType(int sensorType) { widthOffset_ = SensorTypeWidthOffset(sensorType); }
  void OnPulse(int sensor, double t, double width);
  void Advance(double t);
  bool Active() const { return active_; }
  // The room's layout, shared between the devices of a session: whether the stations run a
  // multi-station round. A device that knows it doesn't have to find it from its own light.
  bool LayoutKnown() const { return layoutKnown_; }
  void SetLayoutKnown(bool known) {
    if (mode_ != Mode::Auto) return;
    layoutKnown_ = known;
    active_ = known;
  }

  bool PopFrame(OpticalFrame* out);
  bool PopBaseConfig(BaseConfig* cfg, BaseDynamicState* dyn);
  bool PopLog(std::string* out);
  bool PopIdentity(TdmDisambiguator::Identity* out);
  void SetKnownIdentities(std::vector<TdmDisambiguator::Identity> identities) { known_ = std::move(identities); }

  static constexpr double kStagger = 250e-6;
  static constexpr int kMaxSlots = 4;

 private:
  struct Flash { double start, width; int sensors; SyncInfo sync; };
  struct Station {
    bool seen = false;
    ootx::OotxMajorityRing ring[2]{ootx::OotxMajorityRing(0), ootx::OotxMajorityRing(0)};
    uint32_t serial = 0;
    int uniqueId = 0;
    int mode = 0, multiN = 0, multiSlot = -1;
    bool ootxStock = false;  // its OOTX (decoded or recalled) says a stock station: not the multi layout
    // Of the last 128 groups: present / swept after.
    std::deque<uint8_t> presence;
    int flashes = 0, sweeps = 0;
    double lastSweep[2] = {0, 0};
    std::deque<uint8_t> recentBits;
    int bits[3] = {};
    int frames = 0;
    bool everyFlashBits = false;  // OOTX on every flash (policy ALL)
    int skipFlashes = 0, skipOnes = 0;
    int64_t activeStart = -1;  // half-revolution index of its last sweeping revolution
    int64_t lastSweepHalf = -10;  // half-revolution index and axis of its last sweep
    int lastSweepAxis = -1;
    int axisMismatches = 0;        // sweeps dropped: decoded axis out of sequence
  };
  struct PendingSweep { int slot; int axis; double sync; Flash flash; };

  void CloseFlash();
  void CloseGroup();
  void Detect(const std::vector<Flash>& flashes);
  void EmitDue(double t);
  void PushOotx(Station& s, int slot, int bit);
  bool Identify(Station& s, int bit);
  void Log(const std::string& s);
  void Reset();

  Mode mode_;
  double widthOffset_ = 0;
  bool active_ = false;
  // The room runs a multi-station round: once detected, kept through a reset (sync lost).
  bool layoutKnown_ = false;
  int multiVotes_ = 0, stockVotes_ = 0, lowShareVotes_ = 0;

  // Pulse store (all pulses of the last ~50 ms; sweeps are read back from it).
  struct Pulse { double start, end; int sensor; };
  std::deque<Pulse> pulses_;

  // Flash being assembled, group being assembled.
  std::vector<Pulse> flashPulses_;
  std::vector<Flash> group_;
  double groupStart_ = 0;

  // Timing reference: master flash time and rotor period.
  double ref_ = 0;
  double period_ = 1.0 / 60;
  int periodSamples_ = 0;
  double lastFlash_ = 0;
  int64_t halfIndex_ = 0;
  int stationCount_ = 0;
  int unmatched_ = 0;  // consecutive groups with no flash on the slot grid
  // Re-anchoring to an earlier station: only once it showed up in most recent groups (a
  // stray early pulse in fast motion moved every slot for good, and pushed real stations
  // past the last slot). earlyGroups_: the last 64 groups, 1 = had an earlier flash.
  std::deque<uint8_t> earlyGroups_;
  // A decoded station whose OOTX slot differs from its relative one: shift by this at the
  // next group (0 = none).
  int pendingShift_ = 0;
  void ShiftSlots(int delta);
  bool policyAll_ = false;  // from an OOTX mode byte  // N; 0 until known  // half revolutions since the first reference

  Station stations_[kMaxSlots];
  int nextUniqueId_ = 0;
  std::deque<PendingSweep> sweeps_;
  std::deque<OpticalFrame> frames_;
  std::deque<std::pair<BaseConfig, BaseDynamicState>> configs_;
  std::deque<std::string> logs_;
  std::vector<TdmDisambiguator::Identity> known_;
  std::deque<TdmDisambiguator::Identity> learned_;
};

}  // namespace lh::tdm
