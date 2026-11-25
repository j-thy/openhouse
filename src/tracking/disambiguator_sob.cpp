#include "disambiguator_sob.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace lh::sob {
namespace {
constexpr double kTwoPi = 6.283185307179586;      // 0x1a9600
constexpr double kHitWidth = 250 * kTickSeconds;  // 0x1a9ad0: nominal hit width (not reported)
constexpr double kPeriodGain = 0.03;              // 0x1a98a0
constexpr double kPeriodLow = 0.995, kPeriodHigh = 1.005;  // 0x1a9838, 0x1a9800
constexpr double kOverdue = 1.99;                 // 0x1a9a68: emit once the next sync is this late
// Release delay: a revolution's frame is built when the next beam word arrives, up to one
// period (~20 ms) after its first hit, plus USB batching.
constexpr double kHold = 0.05;

double FirstHit(const OpticalFrame& f) {
  double t = 1e300;
  for (const auto& s : f.samples) t = std::min(t, s.time + 0.5 * s.width);
  return t;
}
}  // namespace

void SobDisambiguator::Reset() {
  for (auto& c : channels_) c = Channel{};
  frames_.clear();
  configs_.clear();
  active_ = false;
}

void SobDisambiguator::SetKnownChannels(const std::map<int, std::pair<BaseConfig, BaseDynamicState>>& channels,
                                        std::bitset<kChannels> blocked) {
  const auto changed = blocked_ ^ blocked;
  blocked_ = blocked;
  if (changed.any()) {
    std::erase_if(configs_, [&](const auto& config) { return changed[config.second.sobChannel]; });
  }
  for (int ch = 0; ch < kChannels; ++ch) {
    Channel& c = channels_[size_t(ch)];
    if (!c.seen) continue;
    if (changed[ch]) {
      DiscardFrames(c);
      c = Channel{};
      continue;
    }
    const auto next = channels.find(ch);
    const auto previous = known_.find(ch);
    const bool withdrawn = previous != known_.end() && previous->second.first.serial == c.serial && next == channels.end();
    const bool replaced = next != channels.end() && c.serial && c.serial != next->second.first.serial;
    if (withdrawn || replaced) {
      DiscardFrames(c);
      c.serial = 0;
    }
    if (!c.serial && next != channels.end()) {
      auto dynamic = next->second.second;
      dynamic.cached = true;
      Announce(c, ch, next->second.first, dynamic);
    }
  }
  known_ = channels;
}

void SobDisambiguator::DiscardFrames(Channel& c) {
  c.hits.clear();
  std::erase_if(frames_, [&](const OpticalFrame& f) {
    return f.uniqueBaseId == c.uniqueId || (c.serial && f.baseSerial == c.serial);
  });
}

void SobDisambiguator::Announce(Channel& c, int channel, const BaseConfig& cfg, BaseDynamicState dyn) {
  if (blocked_[size_t(channel)]) return;
  if (cfg.serial != c.serial) {
    if (c.serial) DiscardFrames(c);
    char buf[96];
    snprintf(buf, sizeof buf, "SOB: channel %d is base %08X\n", channel + 1, cfg.serial);
    Log(buf);
  }
  c.serial = cfg.serial;
  QueueConfig(channel, cfg, dyn);
}

void SobDisambiguator::QueueConfig(int channel, const BaseConfig& cfg, BaseDynamicState dyn) {
  dyn.sobChannel = channel;
  if (configs_.size() >= 16) configs_.pop_front();
  configs_.push_back({cfg, dyn});
}

void SobDisambiguator::Log(const std::string& s) {
  if (logs_.size() >= 16) logs_.pop_front();
  logs_.push_back(s);
}

void SobDisambiguator::OnBeamWord(double t, int channel, int ootxBit, int generation) {
  if (channel < 0 || channel >= kChannels) return;
  latest_ = std::max(latest_, t);
  Channel& c = channels_[size_t(channel)];
  active_ = true;
  if (c.seen && c.generation != generation) {  // stock: "(new sync generation)"
    c = Channel{};
  }
  if (!c.seen) {
    c.seen = true;
    c.generation = generation;
    c.nominal = c.period = double(kPeriodTicks[size_t(channel)]) * kTickSeconds;
    c.sync = t;
    c.syncReal = true;
    c.ootxBit = ootxBit;
    c.uniqueId = ++nextUniqueId_;
    c.ring.Init(443);
    if (const auto k = known_.find(channel); k != known_.end()) {
      auto dynamic = k->second.second;
      dynamic.cached = true;
      Announce(c, channel, k->second.first, dynamic);
    }
    char buf[64];
    snprintf(buf, sizeof buf, "SOB: add channel %d\n", channel + 1);
    Log(buf);
    PushOotx(c, channel, ootxBit);
    return;
  }
  if (t <= c.sync) return;
  const double dt = t - c.sync;
  const long revs = std::lround(dt / c.period);
  if (revs <= 0) return;
  // Close the current revolution: its right sync is this word, or predicted if any
  // revolution in between went unseen.
  Emit(c, channel, revs == 1 ? t : c.sync + c.period, revs == 1);
  if (revs == 1 && dt > kPeriodLow * c.period && dt < kPeriodHigh * c.period)
    c.period += (dt - c.period) * kPeriodGain;
  Skip(c, revs - 1);
  PushOotx(c, channel, ootxBit);
  c.sync = t;
  c.syncReal = true;
  c.ootxBit = ootxBit;
  // The unseen revolutions' hits, as OnSensorWord and Advance drop them: left here, they
  // would join this revolution a period early, with negative angles that read as plane 0.
  if (revs > 1) std::erase_if(c.hits, [&](const Hit& h) { return h.time < c.sync; });
}

void SobDisambiguator::OnSensorWord(double t, int channel, int sensor, bool sorted) {
  if (channel < 0 || channel >= kChannels || sensor < 0 || sensor >= 32) return;
  latest_ = std::max(latest_, t);
  Channel& c = channels_[size_t(channel)];
  if (!c.seen || t < c.sync || blocked_[size_t(channel)]) return;
  // The next revolution's first hits can arrive before its beam word (the device reports
  // the word once decoded): keep them; only two periods without a word is a missed one.
  if (t - c.sync >= 2 * c.period) {
    const long k = long((t - c.sync) / c.period);
    Emit(c, channel, c.sync + c.period, false);
    c.sync += double(k) * c.period;
    c.syncReal = false;
    c.ootxBit = -1;
    Skip(c, k);
    std::erase_if(c.hits, [&](const Hit& h) { return h.time < c.sync; });  // unseen revolutions
  }
  c.hits.push_back({t, sensor, sorted});
}

void SobDisambiguator::Advance(double t) {
  latest_ = std::max(latest_, t);
  for (int ch = 0; ch < kChannels; ++ch) {
    Channel& c = channels_[size_t(ch)];
    if (!c.seen || t < c.sync + kOverdue * c.period) continue;
    Emit(c, ch, c.sync + c.period, false);
    const long k = long(std::floor((t - c.sync) / c.period));
    c.sync += double(k) * c.period;
    c.syncReal = false;
    c.ootxBit = -1;
    Skip(c, k);
    std::erase_if(c.hits, [&](const Hit& h) { return h.time < c.sync; });  // unseen revolutions
  }
}

void SobDisambiguator::Skip(Channel& c, long revolutions) {
  if (revolutions <= 0) return;
  c.framer.Reset();  // the fast path needs an unbroken frame
  // The ring is one 443-slot lap: only the remainder moves it. Capping at 443 left it
  // n % 443 slots out of step after a longer gap.
  for (long i = 0; i < revolutions % 443; ++i) { c.ring.Push(0xff); ++c.ringPushes; }
}

void SobDisambiguator::PushOotx(Channel& c, int channel, int bit) {
  BaseConfig cfg;
  BaseDynamicState dyn;
  bool ok = false;
  if (bit >= 0 && c.framer.PushBit(bit != 0)) ok = ootx::DecodeFramer(c.framer, &cfg, &dyn);
  if (blocked_[size_t(channel)]) {
    // Keep intact optical reports for Session's freshness check on known stations.
    // Combining bits across revolutions can mix stations sharing this channel.
    if (bit < 0) c.framer.Reset();
    else if (ok) QueueConfig(channel, cfg, dyn);
    return;
  }
  const bool decided = c.ring.Push(bit >= 0 ? bit : 0xff);
  // Retry when a slot gets decided, and once a lap (votes may have flipped a wrong bit).
  if (!ok && c.ring.Complete() && (decided || ++c.ringPushes % 443 == 0)) {
    ok = ootx::DecodeRing(c.ring, &cfg, &dyn);
    if (ok) c.ring.Reset();
  }
  if (ok) Announce(c, channel, cfg, dyn);
}

void SobDisambiguator::Emit(Channel& c, int channel, double next, bool nextReal) {
  if (blocked_[size_t(channel)]) return;
  // This revolution's hits; later ones (the next revolution's, reported before its beam
  // word) stay for it.
  std::vector<Hit> hits, later;
  for (const Hit& h : c.hits) (h.time < next ? hits : later).push_back(h);
  c.hits.swap(later);
  if (hits.empty() || c.serial == 0) return;
  // Stock divides by the measured revolution when both syncs are real, else the period.
  const double span = c.syncReal && nextReal ? next - c.sync : c.period;
  OpticalFrame f[2];
  for (int axis = 0; axis < 2; ++axis) {
    f[axis].baseSerial = c.serial;
    f[axis].axis = axis;
    f[axis].syncTime = c.sync;
    f[axis].nextSyncTime = next;
    f[axis].idealPeriod = c.nominal;
    f[axis].averagePeriod = c.period;
    f[axis].previousPeriod = c.period;
    f[axis].ootxBit = c.ootxBit;
    f[axis].frameCounter = c.frames;
    f[axis].leftSyncIsReal = c.syncReal;
    f[axis].rightSyncIsReal = nextReal;
    f[axis].uniqueBaseId = c.uniqueId;
  }
  ++c.frames;
  for (const Hit& h : hits) {
    OpticalSample s;
    s.sensor = h.sensor;
    s.time = h.time - 0.5 * kHitWidth;  // stock keeps the start; the angle uses the centre
    s.width = kHitWidth;
    s.angle = (h.time - c.sync) * kTwoPi / span;
    s.axis = s.angle >= kTwoPi / 2 ? 1 : 0;  // first half turn: plane 0
    if (h.sorted) s.flags2 |= kSampleSorted;
    f[s.axis].samples.push_back(s);
  }
  for (auto& frame : f) {
    if (frame.samples.empty()) continue;
    std::sort(frame.samples.begin(), frame.samples.end(), [](const OpticalSample& a, const OpticalSample& b) {
      return a.sensor != b.sensor ? a.sensor < b.sensor : a.time < b.time;
    });
    if (frames_.size() >= 256) frames_.pop_front();
    const double key = FirstHit(frame);
    auto at = frames_.end();
    while (at != frames_.begin() && FirstHit(*std::prev(at)) > key) --at;
    frames_.insert(at, std::move(frame));
  }
}

bool SobDisambiguator::PopFrame(OpticalFrame* out) {
  if (frames_.empty() || FirstHit(frames_.front()) > latest_ - kHold) return false;
  if (out) *out = std::move(frames_.front());
  frames_.pop_front();
  return true;
}

bool SobDisambiguator::OldestFrameTime(double* t) const {
  if (frames_.empty()) return false;
  *t = FirstHit(frames_.front());
  return true;
}

double SobDisambiguator::SettledUpTo() const { return latest_ - kHold; }

bool SobDisambiguator::PopBaseConfig(BaseConfig* cfg, BaseDynamicState* dyn) {
  if (configs_.empty()) return false;
  if (cfg) *cfg = configs_.front().first;
  if (dyn) *dyn = configs_.front().second;
  configs_.pop_front();
  return true;
}

bool SobDisambiguator::PopLog(std::string* out) {
  if (logs_.empty()) return false;
  if (out) *out = std::move(logs_.front());
  logs_.pop_front();
  return true;
}

}  // namespace lh::sob
