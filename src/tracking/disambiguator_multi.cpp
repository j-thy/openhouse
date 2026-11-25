#include "disambiguator_multi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace lh::tdm {
namespace {
constexpr double kIdealPeriod = 1.0 / 60;
constexpr double kFlashMerge = 3e-6;        // one flash seen by several sensors
constexpr double kGroupSpan = 1.0e-3;       // all flashes of an index event (<= 0.75 ms)
constexpr double kSlotTol = 60e-6;          // stations sit on the 250 us grid within this
constexpr double kStockStagger = 1.0 / 2400;
constexpr double kSampleStart = 1.0 / 12, kSampleEnd = 5.0 / 12;  // as stock
constexpr double kPulseKeep = 0.06;
constexpr double kSyncLostAfter = 5.0;
constexpr double kTwoPi = 6.283185307179586;
constexpr size_t kPresenceWindow = 128;
}  // namespace

MultiDisambiguator::MultiDisambiguator(int sensorType, Mode mode) : mode_(mode) {
  SetSensorType(sensorType);
  active_ = mode_ == Mode::On;
}

void MultiDisambiguator::Log(const std::string& s) {
  if (logs_.size() >= 16) logs_.pop_front();
  logs_.push_back(s);
}

void MultiDisambiguator::Reset() {
  pulses_.clear();
  flashPulses_.clear();
  group_.clear();
  sweeps_.clear();
  ref_ = 0;
  period_ = kIdealPeriod;
  periodSamples_ = 0;
  halfIndex_ = 0;
  for (auto& s : stations_) s = Station{};
  stationCount_ = 0;
  policyAll_ = false;
  multiVotes_ = stockVotes_ = lowShareVotes_ = 0;
  // The stations didn't change when this device lost their sync (covered, at the edge of the
  // room). Forgetting the layout too left it to be found again from scratch: 50 flash pairs or a
  // whole OOTX frame, which took 90 s where the light was poor, while the two-base path reset
  // on seeing more than two stations hundreds of times and the controller showed as lost
  // (2026-11-24, four lhbs stations). A station's OOTX saying stock forgets it.
  active_ = mode_ == Mode::On || (mode_ == Mode::Auto && layoutKnown_);
}

void MultiDisambiguator::OnPulse(int sensor, double t, double width) {
  if (unsigned(sensor) >= 32 || !(width > 0)) return;
  if (lastFlash_ != 0 && t - lastFlash_ > kSyncLostAfter && ref_ != 0) {
    Log("multi: sync lost\n");
    Reset();
  }
  const bool sync = width >= 56.25e-6 + widthOffset_ && width <= 137.5e-6 + widthOffset_ + 3e-6;
  if (!flashPulses_.empty() && t - flashPulses_.front().start > kFlashMerge) CloseFlash();
  if (!group_.empty() && t - groupStart_ > kGroupSpan) CloseGroup();
  if (sync) {
    flashPulses_.push_back({t, t + width, sensor});
    lastFlash_ = t;
  } else {
    pulses_.push_back({t, t + width, sensor});
  }
  while (!pulses_.empty() && pulses_.front().start < t - kPulseKeep) pulses_.pop_front();
  EmitDue(t);
}

void MultiDisambiguator::Advance(double t) {
  if (!flashPulses_.empty() && t - flashPulses_.front().start > kFlashMerge) CloseFlash();
  if (!group_.empty() && t - groupStart_ > kGroupSpan) CloseGroup();
  EmitDue(t);
}

void MultiDisambiguator::CloseFlash() {
  std::vector<double> widths;
  for (const auto& p : flashPulses_) widths.push_back(p.end - p.start);
  std::sort(widths.begin(), widths.end());
  Flash f{flashPulses_.front().start, widths[widths.size() / 2], int(flashPulses_.size()), {}};
  for (const auto& p : flashPulses_) f.start = std::min(f.start, p.start);
  ClassifySync(f.width, widthOffset_, false, &f.sync);
  flashPulses_.clear();
  if (group_.empty()) groupStart_ = f.start;
  group_.push_back(f);
}

void MultiDisambiguator::Detect(const std::vector<Flash>& flashes) {
  for (size_t i = 1; i < flashes.size(); ++i) {
    const double d = flashes[i].start - flashes[0].start;
    if (std::abs(d - kStockStagger) < 40e-6) ++stockVotes_;
    else {
      const double k = std::round(d / kStagger);
      if (k >= 1 && k <= 3 && std::abs(d - k * kStagger) < 40e-6) ++multiVotes_;
    }
  }
  if (active_ || mode_ != Mode::Auto) return;
  // A lone station seen sweeping after < 35 % of its flashes: a stock base sweeps after
  // every flash (alone) or every other (paired); multi master at N >= 3 after 1/N.
  const auto& m = stations_[0];
  // Not one whose OOTX says stock: a device facing away from a lone stock base still sees
  // its flashes (the sync LEDs light the whole room) but none of its sweeps, and was taken
  // for a multi master, then dropped every sweep as out of sequence and lost tracking with
  // the base in plain view (2026-11-16, Beyond + Index, one 1.0 base in mode A).
  if (m.flashes >= 400 && stockVotes_ == 0 && multiVotes_ == 0 && m.sweeps < 0.35 * m.flashes && !m.ootxStock)
    ++lowShareVotes_;
  if ((multiVotes_ >= 50 && multiVotes_ > 5 * stockVotes_) || lowShareVotes_ >= 50) {
    active_ = true;
    layoutKnown_ = true;
    char buf[160];
    snprintf(buf, sizeof buf, "multi-station layout detected (%d multi / %d stock flash pairs)\n", multiVotes_,
             stockVotes_);
    Log(buf);
  }
}

// Moves every station by `delta` slots (slot s becomes s + delta); stations pushed out of
// range are dropped. Slot 0's time moves with it.
void MultiDisambiguator::ShiftSlots(int delta) {
  if (delta == 0) return;
  std::array<Station, kMaxSlots> moved{};
  for (int s = 0; s < kMaxSlots; ++s)
    if (s + delta >= 0 && s + delta < kMaxSlots) moved[s + delta] = std::move(stations_[s]);
  for (int s = 0; s < kMaxSlots; ++s) stations_[s] = std::move(moved[s]);
  if (ref_ != 0) ref_ -= delta * kStagger;
}

void MultiDisambiguator::CloseGroup() {
  std::vector<Flash> flashes;
  flashes.swap(group_);
  Detect(flashes);
  if (pendingShift_) {
    char buf[112];
    snprintf(buf, sizeof buf, "multi: slots renumbered by %d to match the stations' own OOTX slots\n", pendingShift_);
    Log(buf);
    ShiftSlots(pendingShift_);
    pendingShift_ = 0;
  }
  const double half = 0.5 * period_;
  double pred;
  int64_t steps = 0;
  if (ref_ == 0) {
    pred = flashes.front().start;
  } else {
    steps = std::llround((flashes.front().start - ref_) / half);
    pred = ref_ + double(steps) * half;
  }
  // Slot of each flash; re-anchor if a station precedes the reference.
  std::vector<int> slots(flashes.size(), -100);
  int minSlot = 0;
  for (size_t i = 0; i < flashes.size(); ++i) {
    const double d = flashes[i].start - pred;
    const double k = std::round(d / kStagger);
    if (std::abs(d - k * kStagger) > kSlotTol || k < -(kMaxSlots - 1) || k > kMaxSlots - 1) continue;
    slots[i] = int(k);
    minSlot = std::min(minSlot, int(k));
  }
  // Nothing fits the reference (anchored while the rotors were still spinning up,
  // or the period drifted away while nothing was seen): re-anchor on this group.
  bool any = false;
  for (int s : slots) any |= s != -100;
  if (!any && ref_ != 0) {
    if (++unmatched_ >= 8) {
      Log("multi: timing reference lost, re-anchoring\n");
      unmatched_ = 0;
      periodSamples_ = 0;
      period_ = kIdealPeriod;
      pred = flashes.front().start;
      steps = 0;
      for (size_t i = 0; i < flashes.size(); ++i) {
        const double d = flashes[i].start - pred;
        const double k = std::round(d / kStagger);
        if (std::abs(d - k * kStagger) <= kSlotTol && k >= 0 && k < kMaxSlots) slots[i] = int(k);
      }
    }
  } else {
    unmatched_ = 0;
  }
  earlyGroups_.push_back(minSlot < 0);
  if (earlyGroups_.size() > 64) earlyGroups_.pop_front();
  int early = 0;
  for (uint8_t e : earlyGroups_) early += e;
  if (minSlot < 0 && early < 30) {
    // Not (yet) a station: a stray pulse. Its flashes are ignored, nothing moves.
    for (auto& s : slots) if (s < 0) s = -100;
    minSlot = 0;
  }
  if (minSlot < 0) {
    const int shift = -minSlot;
    earlyGroups_.clear();
    for (int s = kMaxSlots - 1; s >= 0; --s)
      stations_[s] = s >= shift ? std::move(stations_[s - shift]) : Station{};
    for (auto& s : slots) if (s != -100) s += shift;
    pred -= shift * kStagger;
    char buf[96];
    snprintf(buf, sizeof buf, "multi: earlier station found, slots shifted by %d\n", shift);
    Log(buf);
  }
  // New reference from the earliest assigned flash.
  double newRef = pred;
  for (size_t i = 0; i < flashes.size(); ++i)
    if (slots[i] >= 0 && slots[i] < kMaxSlots) { newRef = flashes[i].start - slots[i] * kStagger; break; }
  if (ref_ != 0 && steps >= 1 && steps <= 16 && unmatched_ == 0) {
    const double measured = 2 * (newRef - ref_) / double(steps);
    if (std::abs(measured - period_) < 2e-5 * double(steps)) {
      const double gain = periodSamples_ < 100 ? 1.0 / (periodSamples_ + 2) : 0.01;
      period_ += gain * (measured - period_);
      ++periodSamples_;
    }
  }
  if (ref_ != 0) halfIndex_ += steps;
  ref_ = newRef;

  // Stations in the schedule: from the OOTX mode byte, else the master's sweep share.
  if (stationCount_ == 0) {
    for (const auto& st : stations_) if (st.multiN >= 2) stationCount_ = st.multiN;
    const auto& m = stations_[0];
    if (stationCount_ == 0 && m.flashes >= 240 && m.sweeps > 0)
      stationCount_ = std::clamp(int(std::lround(double(m.flashes) / m.sweeps)), 2, kMaxSlots);
  }
  bool present[kMaxSlots] = {};
  for (size_t i = 0; i < flashes.size(); ++i) {
    const int s = slots[i];
    if (s < 0 || s >= kMaxSlots || present[s]) continue;
    present[s] = true;
    Station& st = stations_[s];
    const Flash& f = flashes[i];
    if (!st.seen) {
      st.seen = true;
      st.ring[0].Init(0x166);
      st.ring[1].Init(0x155);
      st.uniqueId = ++nextUniqueId_;
    }
    ++st.flashes;
    const bool sweeping = f.sync.status == 0 && f.sync.notSkip && f.sync.axis >= 0;
    // The master's OOTX advances on every flash. A slave's: with lhbs policy ALL on
    // every flash too; with MASTER+ACTIVE only on its own sweeping flashes, its
    // other flashes carry data 0 (measured, ms-a). ALL shows in the OOTX mode byte,
    // or as 1-bits on non-sweeping flashes.
    if (s > 0 && !st.everyFlashBits && f.sync.status == 0 && !f.sync.notSkip) {
      ++st.skipFlashes;
      st.skipOnes += f.sync.data == 1;
      if (policyAll_ || (st.skipFlashes >= 64 && st.skipOnes * 10 >= st.skipFlashes)) {
        st.everyFlashBits = true;
        for (auto& ring : st.ring) ring.Reset();  // bits so far were from sweeping flashes only
        st.recentBits.clear();
        if (active_) Log("multi: slave sends OOTX on every flash (policy ALL)\n");
      }
    }
    if (s == 0 || sweeping || st.everyFlashBits) PushOotx(st, s, f.sync.status == 0 ? f.sync.data : 0xff);
    if (sweeping) {
      // A station sweeps axis 0, then axis 1 in the next half-revolution. A sweep whose
      // decoded axis breaks that is dropped: in Beat Saber play A101B835's axis-0 sweeps
      // sometimes decoded as axis 1, and the filter, reading horizontal angles as vertical
      // (0.33 rad off), threw the controller 0.7-1.8 m. Dropping one sweep costs nothing.
      const int expected = st.lastSweepHalf == halfIndex_ - 1 && st.lastSweepAxis == 0 ? 1 : 0;
      st.lastSweepHalf = halfIndex_;
      st.lastSweepAxis = expected;  // what this sweep was, whatever it decoded as
      if (f.sync.axis != expected) {
        if (++st.axisMismatches <= 3 || st.axisMismatches % 100 == 0) {
          char buf[128];
          snprintf(buf, sizeof buf, "multi: slot %d swept axis %d out of sequence (expected %d), dropped (%d so far)\n", s,
                   f.sync.axis, expected, st.axisMismatches);
          Log(buf);
        }
        continue;
      }
      ++st.sweeps;
      st.activeStart = halfIndex_ - f.sync.axis;
      sweeps_.push_back({s, f.sync.axis, f.start, f});
    }
  }
  for (int s = 0; s < kMaxSlots; ++s) {
    Station& st = stations_[s];
    if (!st.seen) continue;
    // A station that flashes every index event (master, policy ALL) missed one: a
    // missing OOTX bit, as stock. Scheduled-dark stations push nothing.
    if (s == 0 || st.everyFlashBits) {
      int recent = 0;
      for (uint8_t p : st.presence) recent += p;
      if (!present[s] && st.presence.size() >= 32 && recent > 0.8 * st.presence.size()) PushOotx(st, s, 0xff);
    } else if (stationCount_ > 0 && st.activeStart >= 0 && st.activeStart != halfIndex_ &&
               st.activeStart + 1 != halfIndex_) {
      // A scheduled sweeping flash (both index events of its revolution) that did not
      // arrive is a missed bit.
      const int64_t phase = ((halfIndex_ - st.activeStart) % (2 * stationCount_) + 2 * stationCount_) % (2 * stationCount_);
      if (phase <= 1) PushOotx(st, s, 0xff);
    }
    st.presence.push_back(present[s]);
    if (st.presence.size() > kPresenceWindow) st.presence.pop_front();
  }
}

void MultiDisambiguator::EmitDue(double t) {
  while (!sweeps_.empty() && t > sweeps_.front().sync + kSampleEnd * period_ + 1e-4) {
    const PendingSweep sw = sweeps_.front();
    sweeps_.pop_front();
    Station& st = stations_[sw.slot];
    const double previous = st.lastSweep[sw.axis];
    st.lastSweep[sw.axis] = sw.sync;
    if (!active_ || st.serial == 0) continue;
    OpticalFrame f;
    f.baseSerial = st.serial;
    f.axis = sw.axis;
    f.syncTime = sw.sync;
    f.nextSyncTime = sw.sync + period_;
    f.syncWidth = sw.flash.width;
    f.nominalSyncWidth = sw.flash.sync.nominal;
    f.averagePeriod = period_;
    f.previousPeriod = period_;
    f.ootxBit = sw.flash.sync.data;
    f.frameCounter = int(halfIndex_ + std::llround((sw.sync - ref_) / (0.5 * period_)));
    f.leftSyncIsReal = true;
    f.rightSyncIsReal = periodSamples_ >= 20;  // the predicted next sync is as good as a measured one
    f.highFrameJitter = false;
    f.uniqueBaseId = st.uniqueId;
    const double gap = sw.sync - previous;
    const int n = st.multiN > 0 ? st.multiN : 4;
    // The sample history sizes its windows (2.95 x, +-2 x) by idealPeriod, the time
    // between sweeps of a stock base. A multi station sweeps every N revolutions, so
    // the frame carries that interval instead (the frame layout is the original's).
    const double interval = previous > 0 && gap > 0.5 * period_ && gap < 4.5 * period_ ? gap : n * period_;
    f.idealPeriod = std::max(kIdealPeriod, interval);
    const double lo = sw.sync + kSampleStart * kIdealPeriod, hi = sw.sync + kSampleEnd * kIdealPeriod;
    for (const auto& p : pulses_) {
      if (p.start < lo || p.start > hi) continue;
      OpticalSample s;
      s.sensor = p.sensor;
      s.axis = sw.axis;
      s.time = p.start;
      s.width = p.end - p.start;
      s.angle = ((p.start + p.end) * 0.5 - sw.sync) / period_ * kTwoPi;
      f.samples.push_back(s);
    }
    std::sort(f.samples.begin(), f.samples.end(),
              [](const OpticalSample& a, const OpticalSample& b) { return a.sensor != b.sensor ? a.sensor < b.sensor : a.time < b.time; });
    ++st.frames;
    if (frames_.size() >= 16) frames_.pop_front();
    frames_.push_back(std::move(f));
  }
}

bool MultiDisambiguator::Identify(Station& b, int bit) {
  constexpr size_t kWindow = 40;
  constexpr int kMinDataBits = 32, kMaxMismatch = 1, kMargin = 6;
  b.recentBits.push_back(uint8_t(bit < 0 || bit > 1 ? 0xff : bit));
  while (b.recentBits.size() > kWindow) b.recentBits.pop_front();
  if (b.recentBits.size() < kWindow) return false;
  int dataBits = 0;
  for (uint8_t v : b.recentBits) dataBits += v <= 1;
  if (dataBits < kMinDataBits) return false;
  int best = -1, bestErr = 1 << 30, secondErr = 1 << 30;
  for (size_t k = 0; k < known_.size(); ++k) {
    const auto& frame = known_[k].frame;
    const size_t L = frame.size();
    if (L < kWindow) continue;
    bool taken = false;
    for (const auto& other : stations_) taken |= &other != &b && other.serial == known_[k].config.serial;
    if (taken) continue;
    int err = 1 << 30;
    for (size_t shift = 0; shift < L && err > 0; ++shift) {
      int e = 0;
      for (size_t i = 0; i < kWindow && e <= kMargin + kMaxMismatch; ++i) {
        const uint8_t v = b.recentBits[i], f = frame[(shift + i) % L];
        if (v <= 1 && f <= 1 && v != f) ++e;
      }
      err = std::min(err, e);
    }
    if (err < bestErr) { secondErr = bestErr; bestErr = err; best = int(k); }
    else secondErr = std::min(secondErr, err);
  }
  if (best < 0 || bestErr > kMaxMismatch || secondErr < bestErr + kMargin) return false;
  const auto& id = known_[size_t(best)];
  b.serial = id.config.serial;
  b.mode = id.dynamic.mode;
  b.multiN = id.dynamic.multiN;
  b.multiSlot = id.dynamic.multiSlot;
  b.ootxStock = !id.dynamic.multi;
  b.recentBits.clear();
  BaseDynamicState recalled = id.dynamic;
  recalled.cached = true;
  configs_.push_back({id.config, recalled});
  char buf[128];
  snprintf(buf, sizeof buf, "multi: identified base %08X from %zu OOTX bits\n", id.config.serial, kWindow);
  Log(buf);
  return true;
}

void MultiDisambiguator::PushOotx(Station& st, int slot, int bit) {
  ++st.bits[bit == 0 ? 0 : bit == 1 ? 1 : 2];
  if (st.serial == 0 && !known_.empty()) Identify(st, bit);
  for (auto& ring : st.ring) {
    if (!ring.Push(bit) || !ring.Complete()) continue;
    BaseConfig cfg;
    BaseDynamicState dyn;
    if (!ootx::DecodeRing(ring, &cfg, &dyn)) continue;
    st.serial = cfg.serial;
    st.mode = dyn.mode;
    st.multiN = dyn.multiN;
    st.multiSlot = dyn.multiSlot;
    st.ootxStock = !dyn.multi;
    if (dyn.multiN >= 2) stationCount_ = dyn.multiN;
    if (dyn.multi && dyn.multiSlot != slot && dyn.multiSlot >= 0 && dyn.multiSlot < kMaxSlots)
      pendingShift_ = dyn.multiSlot - slot;
    if (dyn.multiAll) policyAll_ = true;
    if (learned_.size() < 8) learned_.push_back({cfg, dyn, ring.Bits()});
    ring.Reset();
    if (configs_.size() >= 16) configs_.pop_front();
    configs_.push_back({cfg, dyn});
    char buf[160];
    snprintf(buf, sizeof buf, "multi: base %08X at relative slot %d, OOTX %s N=%d slot=%d%s round 0x%x\n", cfg.serial,
             slot, dyn.multi ? "multi" : "stock", dyn.multiN, dyn.multiSlot, dyn.multiAll ? " policy ALL" : "", dyn.multiMask);
    if (active_ || dyn.multi) Log(buf);
    if (!dyn.multi && layoutKnown_ && mode_ == Mode::Auto) {   // the stations were set back to stock
      layoutKnown_ = false;
      active_ = false;
      Log("stations report stock mode: multi-station layout forgotten\n");
    }
    if (dyn.multi && !active_ && mode_ == Mode::Auto) {
      active_ = true;
      layoutKnown_ = true;
      Log("multi-station layout detected (OOTX mode byte)\n");
    }
  }
}

bool MultiDisambiguator::PopFrame(OpticalFrame* out) {
  if (frames_.empty()) return false;
  *out = std::move(frames_.front());
  frames_.pop_front();
  return true;
}

bool MultiDisambiguator::PopBaseConfig(BaseConfig* cfg, BaseDynamicState* dyn) {
  if (configs_.empty()) return false;
  if (cfg) *cfg = configs_.front().first;
  if (dyn) *dyn = configs_.front().second;
  configs_.pop_front();
  return true;
}

bool MultiDisambiguator::PopLog(std::string* out) {
  if (logs_.empty()) return false;
  *out = std::move(logs_.front());
  logs_.pop_front();
  return true;
}

bool MultiDisambiguator::PopIdentity(TdmDisambiguator::Identity* out) {
  if (learned_.empty()) return false;
  *out = std::move(learned_.front());
  learned_.pop_front();
  return true;
}

}  // namespace lh::tdm
