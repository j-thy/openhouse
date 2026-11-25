#include "disambiguator_tdm.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <set>

namespace lh::tdm {
namespace {

// Sync width classes (0x1a4140): 62.5µs + 10.4µs·(skip·4 + data·2 + axis), ordered
// so that index i encodes axis = i>3, data = i&1, skip = (i>>1)&1.
constexpr double kSyncWidths[8] = {62.5e-6,        83.333333333333333e-6, 104.16666666666667e-6, 125e-6,
                                   72.916666666666667e-6, 93.75e-6,     114.58333333333333e-6, 135.41666666666666e-6};
constexpr double kSyncMinWidth = 56.25e-6;         // 0x1a99e8
constexpr double kSyncMaxWidth = 137.5e-6;         // 0x1a9b10 (0.00013749999999999998)
constexpr double kSyncTolLow = 6.25e-6;            // 0x1a9bc0
constexpr double kSyncTolHigh = 2.0833333333333334e-6;  // 0x1a9788
constexpr double kMinPulseWidth = 6.25e-7;         // 0x1a95b8
constexpr double kClusterTol = 0.011111111111111112;    // 0x1a9968 (1/90)
constexpr double kPairTol = 0.002777777777777778;       // 0x1a9a80 (1/360)
constexpr double kPhaseTol = 0.008333333333333333;      // 0x1a9658 (1/120)
constexpr double kWindow30 = 0.03333333333333333;       // 0x1a99e0 (1/30)
constexpr double kMergeTol = 8.333333333333334e-08;     // 0x1a9a10 (4 ticks)
constexpr double kScanLag = 0.00020833333333333335;     // 0x1a9a38 (10 ticks)
constexpr double kSyncLostAfter = 5.0;                  // 0x1a95f8
constexpr double kClusterCheckInterval = 0.125;         // 0x1a97d8
constexpr double kJitterThreshold = 2.0833333333333333e-05;  // 0x1a9698 (1000 ticks)
constexpr double kMaxSyncGap = 1.1;                     // 0x1a9c38
constexpr double kAvgPeriodGain = 0.03;                 // 0x1a98a0
constexpr double kPeriodGain = 0.01;                    // 0x1a9738
constexpr float kNotSkipGain = 0.03f;                   // 0x1a94c0
constexpr double kSlotRate = 2400.0;                    // 0x1a96d8
constexpr double kSlot = 0.0004166666666666667;         // 0x1a9b50
constexpr double kSampleStart = 0.08333333333333333;    // 0x1a9760 (1/12)
constexpr double kSampleEnd = 0.4166666666666667;       // 0x1a97a0 (5/12)
constexpr double kTwoPi = 6.283185307179586;            // 0x1a9600

// float round-half-away-from-zero as compiled: (int)(f + copysign(0.49999997f, f))
int RoundF(float f) { return int(f + std::copysign(0.49999997f, f)); }

// std::lower_bound(lo) / std::upper_bound(hi) over sorted starts (0x3225b0).
bool Range(double lo, double hi, const std::vector<double>& v, int* i0, int* i1) {
  auto a = std::lower_bound(v.begin(), v.end(), lo);
  if (a == v.end()) {
    *i0 = *i1 = -1;
    return false;
  }
  *i0 = int(a - v.begin());
  auto b = std::upper_bound(a, v.end(), hi);
  *i1 = int(b - v.begin());
  return *i0 != *i1;
}

int Popcount(uint32_t x) { return std::popcount(x); }

}  // namespace

double SensorTypeWidthOffset(int t) {
  // 0x3e2e20
  if (unsigned(t - 4) < 4) return 2.6041666666666666e-06;
  if (t == 9) return 1.9791666666666666e-06;
  if (t == 8) return 1.3541666666666667e-06;
  if (t == 10) return 1.4583333333333333e-06;
  if (t == 11) return 1.0416666666666667e-06;
  return 0.0;
}

int InferSensorTypeFromSyncWidths(const std::vector<double>& widths) {
  // Each accepted window is 8.33 us wide and the classes are 10.4 us apart, so the
  // offsets land in overlapping windows: counting hits cannot tell 4/8/9/10/11 apart.
  // Instead take the median measured offset from the nearest nominal class, which is
  // what actually matters for classification.
  if (widths.size() < 8) return -1;
  std::vector<double> deltas;
  deltas.reserve(widths.size());
  for (double w : widths) {
    double best = 1e30;
    for (double c : kSyncWidths) {
      const double d = w - c;
      // Nearest class in width; ties are impossible (classes are 10.4 us apart).
      if (std::fabs(d) < std::fabs(best)) best = d;
    }
    deltas.push_back(best);
  }
  std::sort(deltas.begin(), deltas.end());
  const double median = deltas[deltas.size() / 2];
  int best = 0;
  double bestErr = 1e30;
  for (int t : {4, 8, 9, 10, 11}) {
    const double err = std::fabs(SensorTypeWidthOffset(t) - median);
    if (err < bestErr) { bestErr = err; best = t; }
  }
  return (bestErr <= 0.6e-6) ? best : -1;
}

bool ClassifySync(double width, double off, bool absolute, SyncInfo* out) {
  // 0x3e2fc0
  double base, lo;
  if (!absolute) {
    base = off;
    lo = kSyncMinWidth + off;
  } else {
    base = 0.0;
    lo = kSyncMinWidth;
  }
  int status;
  if (lo <= width) {
    if (width <= kSyncMaxWidth + base) {
      for (int i = 0; i < 8; i++) {
        const double w = kSyncWidths[i] + base;
        if (w - kSyncTolLow <= width && width <= w + kSyncTolHigh) {
          if (out) {
            out->status = 0;
            out->notSkip = ((i >> 1) & 1) == 0;
            out->axis = i > 3 ? 1 : 0;
            out->data = i & 1;
            out->base = 0xff;
            out->nominal = w;
          }
          return true;
        }
      }
      status = 3;
    } else {
      status = 1;
    }
  } else {
    status = 2;
  }
  if (out) {
    out->status = status;
    out->notSkip = false;
    out->axis = -1;
    out->data = 0xff;
    out->base = 0xff;
    out->nominal = 0;
  }
  return false;
}

double TdmDisambiguator::Cluster::Avg() {
  if (count == cachedCount) return cachedAvg;
  cachedAvg = count > 0 ? sum / double(count) : 0.0;
  cachedCount = count;
  return cachedAvg;
}

int TdmDisambiguator::Base::Counter(double t) const {
  if (counterBase < 0) return 0;
  return RoundF(float((t - counterT0) * counterRate)) + counterBase;
}

TdmDisambiguator::TdmDisambiguator(const TdmConfig& cfg, int sensorType) : cfg_(cfg) { SetSensorType(sensorType); }

void TdmDisambiguator::SetSensorType(int t) {
  sensorType_ = t;
  widthOffset_ = SensorTypeWidthOffset(t);
}

void TdmDisambiguator::Log(const std::string& s) { PushBounded(logs_, s); }

void TdmDisambiguator::PushEvent(double t, int type) { PushBounded(events_, TdmEvent{t, type}); }

void TdmDisambiguator::Reset() {
  // 0x321a60
  pending_.clear();
  for (auto& s : stores_) {
    s.starts.clear();
    s.ends.clear();
  }
  lastClusterCheck_ = 0;
  lastSyncTime_ = 0;
  period_ = idealPeriod_;
  nextExpectedSync_ = 0;
  scanTime_ = 0;
  avgNotSkip_ = 0;
  syncCount_ = 0;
  bases_.clear();
  clusters_.clear();
}

int TdmDisambiguator::RetrySensorType() {
  if (widthSample_.size() < 12) return sensorType_;
  const int better = InferSensorTypeFromSyncWidths(widthSample_);
  if (better < 0 || better == sensorType_) return sensorType_;
  char buf[128];
  snprintf(buf, sizeof buf, "SensorType %d (selected %d) from %zu measured sync widths\n", sensorType_, better,
           widthSample_.size());
  Log(buf);
  SetSensorType(better);
  widthSample_.clear();
  // Rebuild discovery from the stored pulses under the corrected classification.
  clusters_.clear();
  bases_.clear();
  pending_.clear();
  nextExpectedSync_ = 0;
  scanTime_ = 0;
  for (auto& s : stores_) {
    for (size_t i = 0; i < s.starts.size(); ++i) {
      const double t = s.starts[i];
      const double w = s.ends[i] - t;
      if (!ClassifySync(w, widthOffset_, false, nullptr)) continue;
      const double ph = std::fmod(t, idealPeriod_) / idealPeriod_;
      bool matched = false;
      for (auto& c : clusters_) {
        const double avg = c.Avg();
        double d = std::fabs(ph - avg);
        d = std::min(d, 1.0 - d);
        if (d < kClusterTol) {
          // Its phase into the sum too (it was only counted, so the average halved and no
          // cluster ever reached a lock), unwrapped next to the average.
          double x = ph;
          if (x - avg > 0.5) x -= 1.0;
          else if (avg - x > 0.5) x += 1.0;
          c.sum += x;
          c.count++;
          c.lastTime = std::max(c.lastTime, t);
          matched = true;
          break;
        }
      }
      if (!matched) {
        Cluster c;
        c.sum = ph;
        c.count = 1;
        c.cachedAvg = ph;
        c.cachedCount = 1;
        c.lastTime = t;
        clusters_.push_back(c);
      }
    }
  }
  return sensorType_;
}

void TdmDisambiguator::CheckSyncLost(double t) {
  // 0x324ba0
  if (lastSyncTime_ != 0.0 && !bases_.empty() && kSyncLostAfter < t - lastSyncTime_) {
    Log("lost the base stations' sync timing\n");
    Reset();
  }
}

unsigned TdmDisambiguator::OnPulse(int sensor, double t, double w) {
  // 0x328610
  CheckSyncLost(t);
  if (unsigned(sensor) < 32) {
    if (kSyncMaxWidth + widthOffset_ < w) {
      stats_.invalidLong++;
    } else if (kMinPulseWidth <= w) {
      stats_.total++;
      // Learn the width offset from measurement: the configured sensor type is a
      // classifier input, the widths are the evidence (ai/driver/sensor_type.md).
      if (widthOffset_ == 0.0 && widthSample_.size() < 400 && w > kSyncMinWidth && w < kSyncMaxWidth + 3e-6)
        widthSample_.push_back(w);
      PulseStore& st = stores_[sensor];
      st.starts.push_back(t);
      st.ends.push_back(w + t);
      if (st.cap <= st.starts.size()) {
        st.starts.erase(st.starts.begin(), st.starts.begin() + long(st.cap >> 1));
        st.ends.erase(st.ends.begin(), st.ends.begin() + long(st.cap >> 1));
      }
      if (ClassifySync(w, widthOffset_, false, nullptr)) {
        const double ph = std::fmod(t, idealPeriod_) / idealPeriod_;
        const double phm1 = ph - 1.0;
        bool matched = false;
        for (size_t i = 0; i < clusters_.size(); i++) {
          Cluster& c = clusters_[i];
          if (c.count == 0) continue;
          double avg, lo;
          if (c.count == c.cachedCount) {
            avg = c.cachedAvg;
            lo = avg - 0.5;
          } else if (c.count < 1) {
            avg = 0.0;
            lo = -0.5;
            c.cachedAvg = avg;
            c.cachedCount = c.count;
          } else {
            avg = c.sum / double(c.count);
            lo = avg - 0.5;
            c.cachedAvg = avg;
            c.cachedCount = c.count;
          }
          double x = phm1;
          if (ph <= avg) x = ph;
          x = double(int(avg - ph)) + x;
          if (x < lo) x += 1.0;
          if (std::fabs(x - avg) < kClusterTol) {
            c.count++;
            c.sum = x + c.sum;
            const double q = 0.25 * idealPeriod_;
            double nt = t;
            if (t <= c.lastTime + q) {
              if (q <= std::fabs(t - c.lastTime)) {
                matched = true;
                break;
              }
              if (c.lastTime <= t) nt = c.lastTime;
            }
            c.lastTime = nt;
            matched = true;
            break;
          }
        }
        if (!matched) {
          Cluster c;
          c.sum = ph;
          c.count = 1;
          c.cachedAvg = ph;
          c.cachedCount = 1;
          c.lastTime = t;
          clusters_.push_back(c);
        }
      }
    } else {
      stats_.runt++;
    }
  } else {
    stats_.badSensor++;
  }
  UpdateBases(t);
  ScanSyncCandidates(t);
  const unsigned flags = ProcessExpectedSync(t);
  if (!SanityCheck(t)) Reset();
  return flags;
}

unsigned TdmDisambiguator::Advance(double t) {
  // 0x3285a0: like the tail of OnPulse but without base discovery.
  CheckSyncLost(t);
  ScanSyncCandidates(t);
  const unsigned flags = ProcessExpectedSync(t);
  if (!SanityCheck(t)) Reset();
  return flags;
}

void TdmDisambiguator::UpdateBases(double t) {
  // 0x325260
  if (t - lastClusterCheck_ < kClusterCheckInterval && lastClusterCheck_ != 0.0) return;
  if (bases_.size() >= 2) {
    clusters_.clear();
    lastClusterCheck_ = t;
    return;
  }
  if (clusters_.empty()) {
    lastClusterCheck_ = t;
    return;
  }
  // Pairs of clusters half a period apart: the two axis syncs of one base.
  std::vector<Cluster> pairs;
  int maxMin = 0;
  for (size_t a = 0; a < clusters_.size(); a++) {
    for (size_t b = 0; b < clusters_.size(); b++) {
      if (a == b) continue;
      Cluster& A = clusters_[a];
      Cluster& B = clusters_[b];
      const double avgA = A.Avg();
      const double diff = avgA - B.Avg();
      if (std::fabs(std::fabs(diff) - 0.5) < kPairTol && B.lastTime < A.lastTime) {
        const int n = std::min(A.count, B.count);
        maxMin = std::max(maxMin, n);
        const double avgA2 = A.Avg();
        Cluster p;
        p.sum = double(n) * avgA2;
        p.count = n;
        p.cachedAvg = avgA2;
        p.cachedCount = n;
        p.lastTime = A.lastTime;
        pairs.push_back(p);
        break;
      }
    }
  }
  if (!pairs.empty()) {
    const double thr = double(maxMin) * 0.1;
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
                               [&](const Cluster& p) { return !(4 < p.count && thr <= double(p.count)); }),
                pairs.end());
  }
  clusters_.clear();
  lastClusterCheck_ = t;

  for (const Cluster& p : pairs) {
    const double P = period_;
    const double pt = p.lastTime;
    // Skip pairs that line up with an existing base.
    bool known = false;
    const double T = P * 0.5;
    for (const Base& b : bases_) {
      const double d = b.lastSync[0];
      if (d == 0.0) continue;
      double x = pt;
      if (d < pt) x = pt - T;
      x = double(int((d - pt) * (2.0 / P))) * T + x;
      if (x < d - 0.5 * T) x += T;
      if (std::fabs((x - d) * (1.0 / P)) < kPhaseTol) {
        known = true;
        break;
      }
    }
    if (known) continue;

    // Classify the axis of this cluster from the 10 most recent syncs at its phase.
    int sumAxis = 0, hits = 0;
    double lo = pt - (kPhaseTol * P + P * 0.0);
    double hi = pt + (kPhaseTol * P - P * 0.0);
    std::vector<Candidate> cands;
    for (int k = 0;;) {
      cands.clear();
      Gather(lo, hi, &cands);
      if (cands.size() == 1 && cands[0].sync.axis >= 0) {
        sumAxis += cands[0].sync.axis;
        hits++;
      }
      k++;
      if (k == 10) break;
      const double w = kPhaseTol * period_;
      const double off = double(k) * period_;
      lo = pt - (w + off);
      hi = (w - off) + pt;
    }
    if (hits < 5) continue;
    float avgAxis = float(sumAxis) / float(hits);
    const int axis = int(std::round(avgAxis));

    // Unique small id for the base.
    uint8_t uid = ++nextUniqueId_;
    std::set<uint8_t> used;
    for (const Base& b : bases_) used.insert(b.uniqueId);
    if (used.count(uid)) {
      for (uint8_t c = 1; c != 0xff; c++) {
        if (!used.count(c)) {
          nextUniqueId_ = c;
          uid = c;
          break;
        }
      }
    }

    Base nb;
    nb.ring[0].Init(0x166);
    nb.ring[1].Init(0x155);
    nb.uniqueId = uid;
    nb.period = P;
    nb.avgPeriod[0] = nb.avgPeriod[1] = P;
    nb.prevDt[0] = nb.prevDt[1] = P;
    nb.lastSync[axis] = pt;
    nb.syncSeen[axis] = 0;
    nb.lastSync[1 - axis] = pt - 0.5 * period_;
    nb.syncSeen[1 - axis] = 0;
    nb.counterBase = -1;
    nb.counterT0 = 0;
    nb.counterRate = (0.5 * period_ <= 0.0) ? 0.0 : 2.0 / period_;
    // Counter starts at 0 at lastSync[0].
    nb.counterBase = 0;
    nb.counterT0 = nb.lastSync[0];
    bases_.push_back(std::move(nb));
    if (nextExpectedSync_ == 0.0) {
      Log("locked onto the base stations' sync timing\n");
      nextExpectedSync_ = 0.5 * idealPeriod_ + pt;
      scanTime_ = pt;
    }
  }
}

void TdmDisambiguator::Gather(double lo, double hi, std::vector<Candidate>* out) const {
  // 0x322e50
  for (int s = 0; s < 32; s++) {
    const PulseStore& st = stores_[s];
    if (st.starts.empty()) continue;
    int i0, i1;
    if (!Range(lo, hi, st.starts, &i0, &i1) || !(i0 < i1)) continue;
    const uint32_t bit = 1u << s;
    for (int k = i0; k < i1; k++) {
      double a = st.starts[size_t(k)];
      const double e = st.ends[size_t(k)];
      bool merged = false;
      for (Candidate& c : *out) {
        if (std::fabs(c.start - a) < kMergeTol && std::fabs(c.end - e) < kMergeTol) {
          if (c.start <= a) a = c.start;
          c.start = a;
          c.end = e <= c.end ? c.end : e;
          c.mask |= bit;
          merged = true;
          break;
        }
      }
      if (!merged) {
        Candidate c;
        c.start = a;
        c.end = e;
        c.mask = bit;
        c.sync.status = 4;
        out->push_back(c);
      }
    }
  }
  if (out->empty()) return;
  std::sort(out->begin(), out->end(), [](const Candidate& x, const Candidate& y) { return x.start < y.start; });
  for (size_t i = 0; i < out->size(); i++) {
    Candidate& ci = (*out)[i];
    if (ci.mask2 == 0 && ci.mask == 0) continue;
    for (size_t j = i + 1; j < out->size(); j++) {
      Candidate& cj = (*out)[j];
      if (cj.mask2 == 0 && cj.mask == 0) continue;
      if (ci.start <= cj.start && cj.start <= ci.end) {
        ci.mask2 |= cj.mask2 | cj.mask;
        cj.mask = 0;
        cj.mask2 = 0;
      }
    }
  }
  out->erase(std::remove_if(out->begin(), out->end(), [](const Candidate& c) { return c.mask == 0 && c.mask2 == 0; }),
             out->end());
  for (Candidate& c : *out) ClassifySync(c.end - c.start, widthOffset_, false, &c.sync);
}

void TdmDisambiguator::ScanSyncCandidates(double t) {
  // 0x323430
  if (t < scanTime_) return;
  if (nextExpectedSync_ == 0.0) return;
  if (bases_.empty()) return;
  const double w = kWindow30 * idealPeriod_;
  double hi = nextExpectedSync_ + w;
  double lo = nextExpectedSync_ - w;
  double end = hi + kScanLag;
  if (end <= t) {
    do {
      if (pending_.empty() || pending_.back().end < lo) {
        std::vector<Candidate> c;
        Gather(lo, hi, &c);
        pending_.insert(pending_.end(), c.begin(), c.end());
      }
      const double step = 0.5 * period_;
      lo += step;
      hi += step;
      end = step + end;
      scanTime_ = end <= scanTime_ ? scanTime_ : end;
    } while (end <= t);
  }
  if (pending_.size() > 15) pending_.erase(pending_.begin(), pending_.begin() + long(pending_.size() >> 1));
}

void TdmDisambiguator::PendingInWindow(double lo, double hi, std::vector<Candidate>* out) const {
  // 0x3224d0
  out->clear();
  for (const Candidate& c : pending_)
    if (!(c.start < lo || hi < c.start)) out->push_back(c);
}

bool TdmDisambiguator::PopIdentity(Identity* out) {
  if (learned_.empty()) return false;
  *out = std::move(learned_.front());
  learned_.pop_front();
  return true;
}

unsigned TdmDisambiguator::Identify(Base& b, int bit) {
  constexpr size_t kWindow = 40;      // data values compared
  constexpr int kMinDataBits = 32;    // real 0/1 bits inside the window
  constexpr int kMaxMismatch = 1;
  constexpr int kMargin = 6;          // runner-up must be clearly worse
  b.recentBits.push_back(uint8_t(bit < 0 || bit > 1 ? 0xff : bit));
  while (b.recentBits.size() > kWindow) b.recentBits.pop_front();
  if (b.recentBits.size() < kWindow) return 0;
  int dataBits = 0;
  for (uint8_t v : b.recentBits) dataBits += v <= 1;
  if (dataBits < kMinDataBits) return 0;
  int best = -1, bestErr = 1 << 30, secondErr = 1 << 30;
  for (size_t k = 0; k < known_.size(); ++k) {
    const auto& frame = known_[k].frame;
    const size_t L = frame.size();
    if (L < kWindow) continue;
    bool taken = false;
    for (const auto& other : bases_) taken |= &other != &b && other.serial == known_[k].config.serial;
    if (taken) continue;
    int err = 1 << 30;
    for (size_t shift = 0; shift < L && err > 0; ++shift) {
      int e = 0;
      for (size_t i = 0; i < kWindow && e <= kMargin + kMaxMismatch; ++i) {
        const uint8_t v = b.recentBits[i];
        const uint8_t f = frame[(shift + i) % L];
        if (v <= 1 && f <= 1 && v != f) ++e;
      }
      err = std::min(err, e);
    }
    if (err < bestErr) { secondErr = bestErr; bestErr = err; best = int(k); }
    else secondErr = std::min(secondErr, err);
  }
  if (best < 0 || bestErr > kMaxMismatch || secondErr < bestErr + kMargin) return 0;
  const auto& id = known_[size_t(best)];
  b.serial = id.config.serial;
  b.mode = id.dynamic.mode;
  b.recentBits.clear();
  BaseDynamicState recalled = id.dynamic;
  recalled.cached = true;
  PushBounded(configs_, std::make_pair(id.config, recalled));
  char buf[128];
  snprintf(buf, sizeof buf, "identified base %08X from %zu OOTX bits (%d mismatches)\n", id.config.serial, kWindow, bestErr);
  Log(buf);
  return 1;
}

unsigned TdmDisambiguator::PushOotx(Base& b, int bit) {
  // 0x326c60
  unsigned r = 0;
  if (b.serial == 0 && !known_.empty()) r |= Identify(b, bit);
  for (auto& ring : b.ring) {
    if (!ring.Push(bit) || !ring.Complete()) continue;
    BaseConfig cfg;
    BaseDynamicState dyn;
    if (!ootx::DecodeRing(ring, &cfg, &dyn)) continue;
    if (dyn.mode == 4) {
      static int warned = 0;
      if (warned < 1) {
        Log("a base's OOTX data reports single-base mode in a two-base setup; ignoring its mode\n");
        warned++;
      }
      dyn.mode = 0;
      dyn.modeParam = 0;
    }
    b.mode = dyn.mode;
    if (learned_.size() < 8) learned_.push_back({cfg, dyn, ring.Bits()});
    PushBounded(configs_, std::make_pair(cfg, dyn));
    b.serial = cfg.serial;
    ring.Reset();
    r = 1;
  }
  return r;
}

unsigned TdmDisambiguator::ProcessExpectedSync(double t) {
  // 0x3273e0
  const double factor = cfg_.rightSync ? 1.05 : 0.45;  // 0x1a9c58 / 0x1a9be0
  const double next = nextExpectedSync_;
  if (next == 0.0 || t < idealPeriod_ * factor + next || bases_.empty()) return 0;
  const double w = idealPeriod_ * kWindow30;
  std::vector<Candidate> cands;
  PendingInWindow(next - w, w + next, &cands);

  unsigned flags = 0;
  Base* best = nullptr;
  const Candidate* bestCand = nullptr;
  int axisSel = -1;
  bool conflict = false;
  std::vector<Base*> unmatched;

  for (Base& b : bases_) {
    bool matched = false;
    for (const Candidate& c : cands) {
      bool stopThisBase = false;
      for (int ax = 0; ax < 2; ax++) {
        const double s = c.start;
        double d = b.lastSync[ax];
        const double P = b.avgPeriod[ax];
        const double diff = s - d;
        if (s < d) d = d - P;
        double x = double(int(diff / P)) * P + d;
        if (x < s - P * 0.5) x += P;
        if (!(std::fabs((x - s) / P) < kPairTol)) continue;
        if (c.sync.axis != ax) {
          if (c.sync.status == 3) stats_.guardBand++;
          stopThisBase = true;
          break;
        }
        // Matched: base b, axis ax, candidate c.
        flags |= PushOotx(b, c.sync.data);
        if (b.lastSync[ax] == 0.0) {
          b.syncSeen[ax] = 0;
          b.jitter[ax] = 0;
        } else if (!b.syncSeen[ax]) {
          b.jitter[ax] = 0;
        } else {
          const double dt = c.start - b.lastSync[ax];
          if (kMaxSyncGap * period_ < dt) {
            b.syncSeen[ax] = 0;
            b.jitter[ax] = 0;
          } else {
            b.prevDt[ax] = dt;
            if (kJitterThreshold <= std::fabs(dt - period_)) {
              b.jitter[ax] = 1;
            } else {
              b.avgPeriod[ax] = (dt - b.avgPeriod[ax]) * kAvgPeriodGain + b.avgPeriod[ax];
              period_ = (dt - period_) * kPeriodGain + period_;
              b.periodStats[ax].Add(dt);
              b.jitter[ax] = 0;
            }
          }
        }
        lastSyncTime_ = c.start;
        b.lastSync[ax] = c.start;
        b.syncSeen[ax] = 1;
        if (b.counterBase < 0) {
          b.counterBase = 0;
          b.counterT0 = c.start;
        } else {
          const int n = RoundF(float((c.start - b.counterT0) * b.counterRate)) + b.counterBase;
          if (b.counterBase < n) {
            b.counterBase = n;
            b.counterT0 = c.start;
          }
        }
        const uint8_t ns = c.sync.notSkip ? 1 : 0;
        syncCount_++;
        avgNotSkip_ = (float(ns) - avgNotSkip_) * kNotSkipGain + avgNotSkip_;
        axisSel = ax;
        if (ns) {
          const bool hadBest = best != nullptr;
          best = &b;
          bestCand = &c;
          if (hadBest) conflict = true;
        }
        matched = true;
        break;
      }
      if (matched || stopThisBase) break;
    }
    if (!matched) {
      flags |= PushOotx(b, 0xff);
      stats_.syncMiss++;
      unmatched.push_back(&b);
    }
  }

  if (conflict || best == nullptr) {
    nextExpectedSync_ = 0.5 * period_ + nextExpectedSync_;
    return flags;
  }

  bool panic = false;
  for (Base* ub : unmatched) {
    const double d0 = ub->lastSync[axisSel];
    if (d0 == 0.0) continue;
    const double P = period_;
    const double s = bestCand->start;
    double d = d0;
    const double diff = s - d;
    if (s < d) d = d - P;
    d = double(int(diff / P)) * P + d;
    if (d < s - 0.5 * P) d += P;
    double off = kSlotRate * (s - d);
    double r;
    if (off <= 0.0) {
      r = off - 0.5;
      if (std::fabs(r) < 4503599627370496.0) r = std::ceil(r);
    } else {
      r = off + 0.5;
      if (std::fabs(r) < 4503599627370496.0) r = std::floor(r);
    }
    if (1.0 < std::fabs(r)) {
      char buf[256];
      snprintf(buf, sizeof buf, "base sync offset out of range (%f): resetting sync\n", off);
      Log(buf);
      panic = true;
    } else {
      ub->lastSync[axisSel] = s - r * kSlot;
      ub->syncSeen[axisSel] = 0;
    }
  }

  const int axis = bestCand->sync.axis;
  double nextSync = axis < 0 ? period_ : bestCand->start + best->avgPeriod[axis];
  bool rightReal = false;
  if (cfg_.rightSync) {
    const double rw = kPairTol * idealPeriod_;
    std::vector<Candidate> rc;
    PendingInWindow(nextSync - rw, rw + nextSync, &rc);
    if (rc.size() == 1 && rc[0].sync.axis == axis) {
      nextSync = rc[0].start;
      best->jitter[axis] = kJitterThreshold <= std::fabs((nextSync - period_) - bestCand->start);
      rightReal = true;
    }
  }

  if ((best->serial != 0 || cfg_.emitBeforeOotx) && axis >= 0) {
    OpticalFrame f;
    f.baseSerial = best->serial;
    f.axis = axis;
    f.syncTime = bestCand->start;
    f.nextSyncTime = nextSync;
    f.syncWidth = bestCand->end - bestCand->start;
    f.nominalSyncWidth = bestCand->sync.nominal;
    f.idealPeriod = idealPeriod_;
    f.averagePeriod = best->avgPeriod[axis];
    f.previousPeriod = best->prevDt[axis];
    f.ootxBit = bestCand->sync.data;
    f.frameCounter = best->Counter(bestCand->start);
    f.leftSyncIsReal = true;
    f.rightSyncIsReal = rightReal;
    f.highFrameJitter = best->jitter[axis] != 0;
    f.uniqueBaseId = best->uniqueId;
    const double s = f.syncTime;
    const double lo = kSampleStart * idealPeriod_ + s;
    const double hi = idealPeriod_ * kSampleEnd + s;
    int sensorsHit = 0, multi = 0;
    for (int sensor = 0; sensor < 32; sensor++) {
      const PulseStore& st = stores_[sensor];
      if (st.starts.empty()) continue;
      int i0 = 0, i1 = 0;
      if (!Range(lo, hi, st.starts, &i0, &i1)) continue;
      sensorsHit++;
      multi += 1 < i1 - i0;
      for (int k = i0; k < i1; k++) {
        OpticalSample smp;
        smp.sensor = sensor;
        smp.axis = axis;
        smp.time = st.starts[size_t(k)];
        smp.width = st.ends[size_t(k)] - smp.time;
        smp.angle = (((smp.time + st.ends[size_t(k)]) * 0.5 - s) / (nextSync - s)) * kTwoPi;
        f.samples.push_back(smp);
      }
    }
    best->sampleSensors[axis].Add(double(sensorsHit));
    best->syncSensors[axis].Add(double(Popcount(bestCand->mask2)) + double(Popcount(bestCand->mask)));
    best->multihit[axis].Add(double(multi));
    best->widthDelta[axis].Add(f.syncWidth - f.nominalSyncWidth);
    PushBounded(frames_, std::move(f));
    flags |= 2;
  }
  nextExpectedSync_ = 0.5 * period_ + bestCand->start;
  if (panic) Reset();
  return flags;
}

bool TdmDisambiguator::SanityCheck(double t) {
  // 0x324c00
  if (bases_.size() > 2) {
    Log("cable/optical sync mode: tracking at most two base stations\n");
    PushEvent(t, 8);
    return false;
  }
  if (bases_.empty()) return true;
  bool any1 = false, any3 = false;
  for (const Base& b : bases_) {
    if (b.mode == 1) any1 = true;
    if (b.mode == 3) any3 = true;
  }
  if (any1 && any3) {
    Log("two bases both act as B (one on the sync cable, one syncing optically): check their modes\n");
    PushEvent(t, 10);
    return false;
  }
  for (size_t i = 0; i < bases_.size(); i++) {
    const Base& bi = bases_[i];
    double d0 = bi.lastSync[0], latest = bi.lastSync[1];
    if (d0 != 0.0) {
      if (latest != 0.0) {
        const double T = bi.period * 0.5;
        double x = latest;
        if (d0 < latest) x = latest - T;
        x = double(int((d0 - latest) / T)) * T + x;
        if (x < d0 - T * 0.5) x += T;
        if (kPhaseTol < std::fabs((x - d0) / bi.period)) {
          Log("a base's two rotors are not half a turn apart: it may still be spinning up\n");
          return false;
        }
      }
      if (latest <= d0) latest = d0;
    }
    for (size_t j = i + 1; j < bases_.size(); j++) {
      const Base& bj = bases_[j];
      if (bi.serial != 0 && bi.serial == bj.serial) {
        Log("two base stations report the same serial\n");
        return false;
      }
      double e = bj.lastSync[1];
      if (bj.lastSync[0] != 0.0 && e <= bj.lastSync[0]) e = bj.lastSync[0];
      if (e != 0.0 && latest != 0.0) {
        const double T = bi.period * 0.5;
        const double diff = latest - e;
        double x = e;
        if (latest < e) x = e - T;
        x = double(int(diff / T)) * T + x;
        if (x < latest - T * 0.5) x += T;
        if (kWindow30 < std::fabs((x - latest) / bi.period)) {
          Log("two bases sweep at the same time: check the sync cable and that they are set to A and B\n");
          PushEvent(t, 9);
          return false;
        }
      }
    }
  }
  return true;
}

bool TdmDisambiguator::PopFrame(OpticalFrame* out) {
  if (frames_.empty()) return false;
  if (out) *out = std::move(frames_.front());
  frames_.pop_front();
  return true;
}

bool TdmDisambiguator::PopBaseConfig(BaseConfig* cfg, BaseDynamicState* dyn) {
  if (configs_.empty()) return false;
  if (cfg) *cfg = configs_.front().first;
  if (dyn) *dyn = configs_.front().second;
  configs_.pop_front();
  return true;
}

bool TdmDisambiguator::PopEvent(TdmEvent* out) {
  if (events_.empty()) return false;
  if (out) *out = events_.front();
  events_.pop_front();
  return true;
}

bool TdmDisambiguator::PopLog(std::string* out) {
  if (logs_.empty()) return false;
  if (out) *out = logs_.front();
  logs_.pop_front();
  return true;
}

bool TdmDisambiguator::HasOutput() const {
  return !frames_.empty() || !configs_.empty() || !logs_.empty() || !events_.empty();
}

double TdmDisambiguator::EarliestPendingTime() const {
  // 0x321540
  double r = 1.7976931348623157e+308;  // stock initial value (DBL_MAX)
  if (bases_.empty()) return r;
  if (frames_.empty()) return nextExpectedSync_;
  const OpticalFrame& f = frames_.front();
  if (f.samples.empty()) return 0.5 * f.idealPeriod + f.syncTime;
  r = f.samples[0].time;
  for (size_t i = 1; i < f.samples.size(); i++) r = std::min(r, f.samples[i].time);
  return r;
}

std::string TdmDisambiguator::StatsString(bool reset) {
  char buf[1024];
  int n = snprintf(buf, sizeof buf, "sync stats: pulses %d, too long %d, too short %d, missed syncs %d, in guard band %d\n",
                   stats_.total, stats_.invalidLong, stats_.runt, stats_.syncMiss, stats_.guardBand);
  std::string s(buf, size_t(n));
  if (reset) {
    stats_.syncMiss = 0;
    stats_.invalidLong = stats_.badSensor = stats_.runt = stats_.total = 0;
    stats_.guardBand = 0;
  }
  return s;
}

}  // namespace lh::tdm
