#pragma once
// Experiment only (LH_AIR): what moving parts of the LH2 decoding onto a Doorman device's radio
// would do to the air throughput, the tracking and the delay. Sits between the report parser and
// the session in a replay:
//   - changes what reaches the session the way a radio-side variant would (accuracy: compare the
//     replay's poses with an unchanged run);
//   - counts the bytes the radio would send for what it forwards, in three formats: today's air codec
//     (doorman_codec.h), that codec without the bits the host never reads, and a sweep-decoded format;
//   - measures how long each light hit waits on the radio before it could be sent.
// LH_AIR = '+'-joined variants: base, nohalf, cap<N> (N hits per sweep pass), imu2 (pairs averaged).
// LH_AIR_REPORT = file for the totals (JSON), written at the end.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "lh/types.hpp"
#include "tracking/disambiguator_sob.hpp"

extern "C" {
#include "../../doorman/common/doorman_codec.h"
}

namespace lh::runtime {

class AirExperiment {
 public:
  struct Sinks {
    std::function<void(const DeviceId&, const ImuSample&)> imu;
    std::function<void(const DeviceId&, const Lh2BeamWord&)> beam;
    std::function<void(const DeviceId&, const Lh2SensorWord&)> sensor;
  };

  static std::unique_ptr<AirExperiment> FromEnv(Sinks sinks) {
    const char* v = std::getenv("LH_AIR");
    if (!v || !*v) return nullptr;
    auto e = std::make_unique<AirExperiment>();
    e->sinks_ = std::move(sinks);
    std::string s = v;
    for (size_t p = 0; p <= s.size();) {
      size_t q = s.find('+', p);
      if (q == std::string::npos) q = s.size();
      const std::string t = s.substr(p, q - p);
      if (t == "nohalf") e->noHalf_ = true;
      else if (t == "imu2") e->imu2_ = true;
      else if (t.rfind("imuq", 0) == 0) e->imuQ_ = std::atoi(t.c_str() + 4);   // IMU LSBs dropped (rounded)
      else if (t.rfind("cap", 0) == 0) e->cap_ = std::atoi(t.c_str() + 3);
      p = q + 1;
    }
    if (const char* r = std::getenv("LH_AIR_REPORT")) e->reportPath_ = r;
    if (const char* r = std::getenv("LH_AIR_DUMP")) e->dump_ = std::fopen(r, "wb");
    return e;
  }
  ~AirExperiment() { Report(); if (dump_) std::fclose(dump_); }

  void OnImu(const DeviceId& id, const ImuSample& in) {
    Dev& d = At(id);
    ImuSample s = in;
    if (imuQ_ > 0) {   // keep the raw integers' top bits only (at the accounting range)
      const double qa = (8 * 9.80665 / 32768) * (1 << imuQ_), qg = (2000 * M_PI / 180 / 32768) * (1 << imuQ_);
      for (int k = 0; k < 3; ++k) { s.accel[k] = std::round(s.accel[k] / qa) * qa; s.gyro[k] = std::round(s.gyro[k] / qg) * qg; }
    }
    Seen(d, s.time);
    if (imu2_) {
      if (!d.havePair) { d.pair = s; d.havePair = true; return; }
      ImuSample m = s;
      for (int k = 0; k < 3; ++k) { m.accel[k] = 0.5 * (s.accel[k] + d.pair.accel[k]); m.gyro[k] = 0.5 * (s.gyro[k] + d.pair.gyro[k]); }
      m.time = 0.5 * (s.time + d.pair.time);
      m.saturated = s.saturated | d.pair.saturated;
      d.imuWait.push_back(s.time - d.pair.time);   // the first of the pair waited this long
      d.imuWait.push_back(0);
      d.havePair = false;
      ImuFrame(d, m);
      sinks_.imu(id, m);
      return;
    }
    ImuFrame(d, s);
    sinks_.imu(id, s);
  }

  void OnBeam(const DeviceId& id, const Lh2BeamWord& w) {
    Dev& d = At(id);
    Seen(d, w.time);
    FlushPassIfOver(id, d, w.time);
    d.beams++;
    // format D keeping beam words (no LH2 decoding on the radio): [b26 b27][EG time from the last hit, half ticks]
    Init(d);
    d.bitsDBeam += 2 + 1 + EgBits(d.predCtx, DMC_AX, dmc_zz(int32_t(std::lround((w.time - d.lastHit) / (kTick / 2)) % (1 << 24))));
    // today's codec: the word as the main MCU sends it (bits 28-31 unknown here: sent as zeros, coded raw either way)
    uint32_t word = (Ticks(w.time) & 0xFFFFFF) << 2 | uint32_t(w.bit26) << 26 | uint32_t(w.bit27) << 27;
    StreamWord(d, w.channel, word);
    sinks_.beam(id, w);
  }

  void OnSensor(const DeviceId& id, const Lh2SensorWord& w) {
    Dev& d = At(id);
    Seen(d, w.time);
    FlushPassIfOver(id, d, w.time);
    Lh2SensorWord f = w;
    if (noHalf_) f.time = std::floor(w.time / kTick + 1e-6) * kTick;   // the half tick dropped
    if (d.pass.empty() || f.channel != d.passChannel) { FlushPass(id, d); d.passChannel = f.channel; }
    d.pass.push_back(f);
    d.passLast = w.time;
    d.lastHit = w.time;
  }

  void Finish() {
    for (auto& [id, d] : dev_) FlushPass(id, d);
  }

 private:
  static constexpr double kTick = 1.0 / 48e6;
  static constexpr double kPassGap = 0.0005;   // hits of one sweep pass come closer than this

  struct Dev {
    int index = 0;
    bool seen = false;
    double first = 0, last = 0;
    // today's codec ('s' stream framing as the main MCU does it: up to 59 bytes or 1 ms)
    dmc_state_t codec{};
    bool codecInit = false;
    std::vector<uint8_t> stream;
    int streamChannel = -1;
    double streamStart = 0;
    uint64_t bytesA = 0, bytesImu = 0, framesS = 0, framesP = 0;
    uint64_t beams = 0, sensors = 0, sensorsSent = 0, passes = 0;
    // sweep format
    dmc_state_t sweep{};
    bool sweepInit = false;
    uint64_t bitsC = 0, bitsD = 0, hitsPred = 0, hitsNew = 0;
    struct Pass { int channel; double first; std::map<int, double> t; };   // sensor -> hit time
    std::deque<Pass> hist;
    dmc_state_t predCtx{}, imuCtx1{}, imuCtx2{};
    int x1[6] = {}, x2[6] = {};
    uint64_t bitsImuP1 = 0, bitsImuP2 = 0, bitsDBeam = 0;
    double lastHit = 0;
    std::map<int, double> lastRef;
    // the current pass
    std::vector<Lh2SensorWord> pass;
    int passChannel = -1;
    double passLast = 0;
    std::vector<double> lightWait, imuWait;
    // imu2
    bool havePair = false;
    ImuSample pair;
    uint8_t seq = 0;
  };

  Dev& At(const DeviceId& id) {
    auto [it, fresh] = dev_.try_emplace(id);
    if (fresh) it->second.index = int(dev_.size()) - 1;
    return it->second;
  }
  static uint32_t Ticks(double t) { return uint32_t(int64_t(std::llround(t / kTick))); }
  // Device times can be negative: "not seen yet" is a flag, not a sentinel time.
  void Seen(Dev& d, double t) {
    if (!d.seen) { d.seen = true; d.first = d.last = t; }
    d.last = std::max(d.last, t);
  }

  void Init(Dev& d) {
    if (d.codecInit) return;
    dmc_reset(&d.codec); dmc_reset(&d.sweep); dmc_reset(&d.predCtx); dmc_reset(&d.imuCtx1); dmc_reset(&d.imuCtx2);
    d.codecInit = d.sweepInit = true;
  }

  // One frame through today's codec: coded length (or raw) plus its length byte.
  uint32_t Code(Dev& d, const uint8_t* f, int n) {
    Init(d);
    if (dump_) {   // for codec tests
      // [device][length][time, 0.1 ms since the device's first data, u32][frame]
      const uint32_t t = uint32_t(std::max(0.0, d.last - d.first) * 1e4);
      std::fputc(d.index, dump_); std::fputc(n, dump_); std::fwrite(&t, 4, 1, dump_); std::fwrite(f, 1, size_t(n), dump_);
    }
    uint8_t o[80];
    const int m = dmc_encode(&d.codec, f, n, o);
    return uint32_t((m ? m : n) + 1);
  }

  void ImuFrame(Dev& d, const ImuSample& s) {
    // raw integers at a typical range (+-8 g, +-2000 dps); the same for every variant
    uint8_t f[18];
    f[0] = 'p';
    for (int k = 0; k < 3; ++k) {
      const int a = std::clamp(int(std::lround(s.accel[k] / (8 * 9.80665 / 32768))), -32768, 32767);
      const int g = std::clamp(int(std::lround(s.gyro[k] / (2000 * M_PI / 180 / 32768))), -32768, 32767);
      dmc_w16(f + 1 + 2 * k, uint16_t(int16_t(a)));
      dmc_w16(f + 7 + 2 * k, uint16_t(int16_t(g)));
    }
    dmc_w32(f + 13, Ticks(s.time));
    f[17] = d.seq++;
    d.bytesImu += Code(d, f, 18);
    // other predictors, values in units of the kept LSB: P1 previous value, P2 linear extrapolation
    Init(d);
    for (int k = 0; k < 6; ++k) {
      const int v = int(int16_t(dmc_u16(f + 1 + 2 * k))) >> imuQ_;
      const int p1 = d.x1[k], p2 = 2 * d.x1[k] - d.x2[k];
      d.bitsImuP1 += EgBits(d.imuCtx1, k, dmc_zz(v - p1));
      d.bitsImuP2 += EgBits(d.imuCtx2, k, dmc_zz(v - p2));
      d.x2[k] = d.x1[k]; d.x1[k] = v;
    }
    d.bitsImuP1 += 2; d.bitsImuP2 += 2;   // the timestamp step (constant) and seq
    d.framesP++;
  }

  void StreamFlush(Dev& d) {
    if (d.stream.size() <= 1) { d.stream.clear(); d.streamChannel = -1; return; }
    d.bytesA += Code(d, d.stream.data(), int(d.stream.size()));
    d.framesS++;
    d.stream.clear();
    d.streamChannel = -1;   // the next frame starts with its channel byte again
  }
  void StreamWord(Dev& d, int channel, uint32_t word) {
    const double now = d.last;
    if (!d.stream.empty() && (d.stream.size() + 5 > 60 || now - d.streamStart > 0.001)) StreamFlush(d);
    if (d.stream.empty()) { d.stream.push_back('s'); d.streamStart = now; }
    if (channel != d.streamChannel) { d.stream.push_back(uint8_t((channel & 15) << 4 | 1)); d.streamChannel = channel; }
    uint8_t b[4]; dmc_w32(b, word);
    d.stream.insert(d.stream.end(), b, b + 4);
  }

  void FlushPassIfOver(const DeviceId& id, Dev& d, double t) {
    if (!d.pass.empty() && t - d.passLast > kPassGap) FlushPass(id, d);
  }

  // A sweep pass is over: choose what the radio sends, count it in both formats, forward it.
  void FlushPass(const DeviceId& id, Dev& d) {
    if (d.pass.empty()) return;
    std::vector<Lh2SensorWord> keep = d.pass;
    const double ready = d.passLast + (cap_ ? kPassGap : 0.0);   // a cap must see the whole pass first
    if (cap_ > 0 && int(keep.size()) > cap_) {
      std::vector<Lh2SensorWord> sel;
      const int n = int(keep.size());
      for (int i = 0; i < cap_; ++i) sel.push_back(keep[size_t((i * (n - 1)) / std::max(1, cap_ - 1))]);
      keep = sel;
    }
    d.sensors += d.pass.size();
    d.sensorsSent += keep.size();
    d.passes++;
    for (const auto& w : keep) d.lightWait.push_back(cap_ ? ready - w.time : 0.0);
    // today's codec
    for (const auto& w : keep) {
      const double ticks = w.time / kTick;
      const uint32_t t24 = uint32_t(int64_t(std::floor(ticks + 1e-6))) & 0xFFFFFF;
      const uint32_t half = (ticks - std::floor(ticks + 1e-6)) > 0.25 ? 1u : 0u;
      StreamWord(d, w.channel, 2u | half << 2 | t24 << 3 | uint32_t(w.value & 31) << 27);
    }
    // sweep format: [channel 4][axis 1][EG ref vs the channel's last ref + its period], then per hit
    // [sensor 5][EG offset from the ref, ticks][half 1]
    Init(d);
    const double ref = keep.front().time;
    uint64_t bits = 4 + 1;
    {
      const auto lr = d.lastRef.find(d.passChannel);
      uint32_t u = 0xFFFFFF;
      if (lr != d.lastRef.end()) {
        const double since = (ref - lr->second) / kTick;
        u = dmc_zz(int32_t(std::fmod(since, 1 << 20)));
      }
      bits += EgBits(d.sweep, DMC_LT, u);
      d.lastRef[d.passChannel] = ref;
    }
    for (const auto& w : keep) bits += 5 + 1 + EgBits(d.sweep, DMC_W, uint32_t(std::lround((w.time - ref) / kTick)));
    d.bitsC += bits;
    d.bitsD += PredictedBits(d, keep);
    for (const auto& w : keep) sinks_.sensor(id, w);
    d.pass.clear();
  }

  // Format D: each hit predicted from the same sensor in the same beam's passes one and two rotations
  // earlier (linear: 2 t1 - t2; one pass only: t1 + period). Per pass: [channel 4][predicted 1], then
  // with a prediction: a present bit per sensor of the last pass, EG(new sensors) + 5 bits each, and
  // per hit EG(zz(residual in half ticks)); without: format C's layout.
  uint64_t PredictedBits(Dev& d, const std::vector<Lh2SensorWord>& keep) {
    const int ch = d.passChannel;
    const double T = (ch >= 0 && ch < sob::kChannels) ? sob::kPeriodTicks[size_t(ch)] * kTick : 0.02;
    const double first = keep.front().time;
    const Dev::Pass* p1 = nullptr; const Dev::Pass* p2 = nullptr;
    for (const auto& h : d.hist) {
      if (h.channel != ch) continue;
      if (std::fabs(first - (h.first + T)) < 0.0015) p1 = &h;
      if (std::fabs(first - (h.first + 2 * T)) < 0.003) p2 = &h;
    }
    uint64_t bits = 4 + 1;
    const double half = kTick / 2;
    if (!p1) {
      bits += 24 + EgBits(d.predCtx, DMC_LT, 0);   // a reference time, then offsets as format C
      for (const auto& w : keep) bits += 5 + 1 + EgBits(d.predCtx, DMC_W, uint32_t(std::lround((w.time - first) / half)));
      d.hitsNew += keep.size();
    } else {
      std::map<int, double> now;
      for (const auto& w : keep) now[w.value] = w.time;
      bits += p1->t.size();   // present bits
      int added = 0;
      for (const auto& [sensor, t] : now) if (!p1->t.count(sensor)) ++added;
      bits += EgBits(d.predCtx, DMC_L2, uint32_t(added)) + 5u * uint32_t(added);
      for (const auto& [sensor, t] : now) {
        const auto a = p1->t.find(sensor);
        if (a == p1->t.end()) {
          bits += EgBits(d.predCtx, DMC_W, uint32_t(std::lround(std::fabs(t - first) / half)));
          d.hitsNew++;
          continue;
        }
        double pred = a->second + T;
        if (p2) if (const auto b = p2->t.find(sensor); b != p2->t.end()) pred = 2 * a->second - b->second;
        bits += EgBits(d.predCtx, DMC_IMU_T, dmc_zz(int32_t(std::lround((t - pred) / half))));
        d.hitsPred++;
      }
    }
    Dev::Pass np{ch, first, {}};
    for (const auto& w : keep) np.t[w.value] = w.time;
    d.hist.push_back(std::move(np));
    while (d.hist.size() > 24) d.hist.pop_front();
    return bits;
  }

  static uint64_t EgBits(dmc_state_t& s, int c, uint32_t u) {
    const int k = dmc_k(&s, c);
    const uint64_t x = (uint64_t(u) >> k) + 1;
    int L = 0; for (uint64_t v = x; v; v >>= 1) ++L;
    dmc_learn(&s, c, u);
    return uint64_t(2 * L - 1 + k);
  }

  static double Pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[size_t(std::min<double>(v.size() - 1, p * (v.size() - 1)))];
  }

  void Report() {
    sinks_.sensor = [](const DeviceId&, const Lh2SensorWord&) {};   // counted only: the session may be gone
    Finish();
    for (auto& [id, d] : dev_) StreamFlush(d);
    FILE* f = reportPath_.empty() ? stderr : std::fopen(reportPath_.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "{\"variant\":\"%s\",\"devices\":{", std::getenv("LH_AIR"));
    bool firstDev = true;
    for (auto& [id, d] : dev_) {
      const double span = std::max(1e-3, d.last - d.first);
      // without unused bits: the 4 beam bits nobody reads
      const double bytesB = double(d.bytesA) - d.beams * 4 / 8.0;
      // sweep format: no beam words (decoded on the radio), frames of up to 59 bytes, a tag and length each
      const double cBytes = d.bitsC / 8.0;
      const double bytesC = cBytes + 2 * std::ceil(cBytes / 59);
      const double dBytes = d.bitsD / 8.0, bytesD = dBytes + 2 * std::ceil(dBytes / 59);
      const double dbBytes = (d.bitsD + d.bitsDBeam) / 8.0, bytesDB = dbBytes + 2 * std::ceil(dbBytes / 59);
      // IMU with other predictors: one sample a frame, a tag and length each, as today
      const double imuP1 = d.bitsImuP1 / 8.0 + 2.0 * d.framesP, imuP2 = d.bitsImuP2 / 8.0 + 2.0 * d.framesP;
      std::fprintf(f, "%s\"%s\":{\"seconds\":%.1f,\"imu_Bps\":%.0f,\"light_codec_Bps\":%.0f,\"light_nounused_Bps\":%.0f,"
                   "\"light_sweep_Bps\":%.0f,\"light_pred_Bps\":%.0f,\"light_pred_beams_Bps\":%.0f,\"pred_share\":%.3f,\"imu_p1_Bps\":%.0f,\"imu_p2_Bps\":%.0f,\"imu_ps\":%.1f,\"beams_ps\":%.1f,\"hits_ps\":%.1f,\"hits_sent_ps\":%.1f,\"passes_ps\":%.1f,"
                   "\"light_wait_ms_mean\":%.3f,\"light_wait_ms_p99\":%.3f,\"imu_wait_ms_mean\":%.3f,\"imu_wait_ms_max\":%.3f}",
                   firstDev ? "" : ",", id.c_str(), span, d.bytesImu / span, d.bytesA / span, bytesB / span, bytesC / span, bytesD / span, bytesDB / span,
                   double(d.hitsPred) / std::max<uint64_t>(1, d.hitsPred + d.hitsNew), imuP1 / span, imuP2 / span, d.framesP / span,
                   d.beams / span, d.sensors / span, d.sensorsSent / span, d.passes / span,
                   1e3 * (d.lightWait.empty() ? 0 : [&] { double s = 0; for (double x : d.lightWait) s += x; return s / d.lightWait.size(); }()),
                   1e3 * Pct(d.lightWait, 0.99),
                   1e3 * (d.imuWait.empty() ? 0 : [&] { double s = 0; for (double x : d.imuWait) s += x; return s / d.imuWait.size(); }()),
                   1e3 * Pct(d.imuWait, 1.0));
      firstDev = false;
    }
    std::fprintf(f, "}}\n");
    if (f != stderr) std::fclose(f);
  }

  Sinks sinks_;
  bool noHalf_ = false, imu2_ = false;
  int imuQ_ = 0;
  FILE* dump_ = nullptr;
  int cap_ = 0;
  std::string reportPath_;
  std::map<DeviceId, Dev> dev_;
};

}  // namespace lh::runtime
