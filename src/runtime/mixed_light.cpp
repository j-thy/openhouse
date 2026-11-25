#include "mixed_light.hpp"

#include <algorithm>
#include <cmath>

namespace lh::runtime {

namespace {
constexpr double kResend = 5.0;       // s dark before sending the mode again: it may have been lost
constexpr double kMinVisit = 2.5;     // s in LH2 mode: 100 beam words take about 2 s
constexpr double kMaxVisit = 8.0;     // s, when a rotor is still learning or tracking hasn't come back
constexpr double kMaxNaming = 25.0;   // s: identifying a station takes about 9 s of steady light
constexpr double kMaxNamingLost = 60.0;  // s with no pose: leaving would start the 9 s over, for little
constexpr double kWakeDark = 30.0;    // s dark in LH2 mode knowing no rotors: stations may be waking
constexpr double kLook = 3.0;         // s, the same with no 2.0 station seen yet: a lit one's light comes in 1 s
constexpr double kVisitDark = 4.0;    // s dark in LH2 mode knowing rotors: nothing more to learn
constexpr double kLh1Dark = 5.0;      // s with no pulses at all in LH1 mode
constexpr double kNoRotorsRetry = 15.0;  // s in LH1 mode knowing no rotors, before looking again
constexpr double kLostBeforeHelp = 1.0;  // s lost in LH1 mode: leaves time to recover where it is
constexpr double kMinLh1 = 3.0;       // s in LH1 mode before going back because lost
constexpr double kOverdueFactor = 2;  // a visit goes ahead after this many intervals, calm or not
constexpr double kLh1OnlyAfter = 8.0; // s in LH2 mode still sending LH1 pulses: it can't do LH2 mode
constexpr double kScoutHold = 20.0;   // s the last rate is kept without the scout's period (offline: holes
                                      // over 20 s were where it went wrong)
constexpr double kScoutMaxChange = 3e-6;  // a period this far off is a bad fit, not the rotor
// The rate follows the scout no faster than this, per second: a rotor's speed changed by up to about
// 0.008 ppm/s in the recordings (warming up), so what moves faster is a bad stretch of the scout's.
constexpr double kScoutMaxRateChange = 0.05e-6;
}  // namespace

RotorScout StartRotorScout(double now, double period) {
  RotorScout s;
  s.reference = period;
  s.lastHeard = s.lastStep = now;
  return s;
}

double StepRotorScout(RotorScout& s, double now, const double* period) {
  const double dt = std::max(0.0, now - s.lastStep);
  s.lastStep = now;
  if (s.lost) return s.drift;
  const double before = s.rate;
  if (period && s.reference > 0 && std::fabs(*period / s.reference - 1) < kScoutMaxChange) {
    const double step = kScoutMaxRateChange * dt;
    s.rate = std::clamp(*period / s.reference - 1, before - step, before + step);
    s.lastHeard = now;
  } else if (now - s.lastHeard > kScoutHold) {
    s.lost = true;
    return s.drift;
  }
  s.drift += 0.5 * (before + s.rate) * dt;
  return s.drift;
}

bool MixesLight(DeviceRole role, const MixedLightOptions& options) {
  switch (role) {
    case DeviceRole::Controller: return options.controllers;
    case DeviceRole::Tracker: return options.trackers;
    case DeviceRole::Hmd:          // the stock driver puts it back in LH2 mode (mixed_stations.md)
    case DeviceRole::BaseStation: return false;
  }
  return false;
}

bool Lh2LightEnough(const std::vector<Lh2Light>& stations) {
  double hits = 0;
  for (const auto& s : stations)
    if (s.frames >= kPreferFrames && s.sensors >= kPreferSensors) hits += s.frames * s.sensors;
  return hits >= kPreferHits;
}

MixedLightStep ChooseMixedLight(const MixedLightView& v, double visitEvery) {
  if (v.mode < 0) return {1, "start in LH2"};
  const bool resend = v.dark >= kResend && v.sinceSent >= kResend;
  const bool noLh2Pose = v.lh2Unseen || v.lh2Unplaced;  // LH2 mode can't give a pose in this room yet
  if (v.stayLh2 && !noLh2Pose) {
    if (v.mode != 1) return {1, "stays in LH2"};
    return resend ? MixedLightStep{1, "re-send"} : MixedLightStep{};
  }
  if (v.mode == 1) {
    if (v.readyRotors == 0) {
      if (v.dark >= (noLh2Pose ? kLook : kWakeDark)) return {0, "no 2.0 light"};
    } else {
      // Unplaced, a pose can't come in LH2 mode: waiting for one would only keep the device blind.
      const bool posed = v.tracking || v.lh2Unplaced;
      const bool done = v.learningRotors == 0 && v.unnamedRotors == 0 && posed;
      // preferLh2: its rotors known, it stays while the 2.0 stations light it well, and leaves for the 1.0
      // light when none does, or when it has lost tracking on the 2.0 light alone.
      if (v.preferLh2 && !noLh2Pose && v.inMode >= kMinVisit) {
        if (v.learningRotors == 0 && v.unnamedRotors == 0) {
          if (v.tracking && v.lh2Short >= kPreferShort) return {0, "too little 2.0 light"};
          if (!v.tracking && v.lost >= kLostBeforeHelp) return {0, "lost on 2.0 light"};
          if (v.dark >= kVisitDark) return {0, "no 2.0 light"};
          return resend ? MixedLightStep{1, "re-send"} : MixedLightStep{};
        }
        // A rotor learning, or a station being named, while the 2.0 light is enough: that's done here, where
        // a visit's time limit (below) sent the device off at once, long past it (live 2026-11-11: 3 times in
        // 3 min, "visit over" 27-87 s in). Short of light, the visit's rules decide as before.
        if (v.tracking && v.lh2Short < kPreferShort) return resend ? MixedLightStep{1, "re-send"} : MixedLightStep{};
      }
      if (done && v.inMode >= kMinVisit) return {0, "rotors ready"};
      if (v.inMode >= kMinVisit && v.dark >= kVisitDark) return {0, "no 2.0 light"};
      const double naming = posed ? kMaxNaming : kMaxNamingLost;
      if (v.inMode >= (v.unnamedRotors > 0 ? naming : kMaxVisit)) return {0, "visit over"};
    }
    return resend ? MixedLightStep{1, "re-send"} : MixedLightStep{};
  }
  if (v.dark >= kLh1Dark) return {1, "no light"};
  if (v.readyRotors == 0 && v.inMode >= kNoRotorsRetry) return {1, "no rotors yet"};
  if (v.lost >= kLostBeforeHelp && v.inMode >= kMinLh1 && !noLh2Pose) return {1, "lost"};
  if (visitEvery > 0 && v.inMode >= visitEvery && (v.calm || v.inMode >= visitEvery * kOverdueFactor)) return {1, "visit"};
  if (v.preferLh2 && !noLh2Pose && v.tracking && v.calm && v.inMode >= kPreferMinLh1 && v.lh2Covered >= kPreferCovered)
    return {1, "2.0 light is enough"};
  return {};
}

bool LooksLh1Only(const MixedLightView& v) { return v.mode == 1 && v.inMode >= kLh1OnlyAfter && v.lh1Pulses && !v.everLh2; }

MixedRoomStep ChooseMixedRoom(const MixedRoomView& v) {
  if (!v.enabled || !(v.lh2Seen || v.lh2InRoom)) return MixedRoomStep::NotInUse;
  if (!v.lh2Seen && v.inUse >= 0 && std::min(v.inUse, v.sinceLh2Light) >= kLh2RoomDark) return MixedRoomStep::RoomDark;
  return MixedRoomStep::InUse;
}

MixedLightStep ChooseLightMode(const LightModeView& v) {
  constexpr double kDark = 2.5;  // stock's
  if (v.mode < 0) {
    const bool lh2 = v.knownLh2 && !v.knownLh1;
    return {lh2 ? 1 : 0, lh2 ? "room has 2.0 bases" : "default"};
  }
  if (v.inMode < kDark) return {};
  if (v.everLh2 && v.sinceLh2Light < kLh2Lit) {
    if (v.mode != 1) return {1, "2.0 stations lit"};
    return v.dark >= kDark && v.inMode >= kResend ? MixedLightStep{1, "re-send"} : MixedLightStep{};
  }
  if (v.dark < kDark) return {};
  // Once stations of one kind only have been seen, every device stays in that mode (one that sees nothing is
  // occluded); nothing seen yet in a room known to be one kind, the same. The command is sent again now and
  // then in case it was lost.
  int want = -1;
  const char* why = nullptr;
  if (v.seenLh1 != v.seenLh2) want = v.seenLh2 ? 1 : 0, why = "room's bases";
  else if (v.knownLh2 != v.knownLh1 && !v.seenLh1 && !v.seenLh2) {
    // SteamVR's database is a guess, not the room: it keeps the bases of an earlier setup (2.0 bases swapped
    // for 1.0 ones kept every tracker in LH2 mode, blind). Dark that long in its mode, the other is tried for
    // a while, long enough for a station to be named; light there keeps it.
    want = v.knownLh2 ? 1 : 0, why = "room's bases (known)";
    if (v.mode == want && v.inMode >= kKnownProbe) return {1 - want, "nothing in the known room's mode"};
    if (v.mode != want && v.inMode < kKnownOther) return {};
  }
  if (want >= 0) {
    if (want != v.mode) return {want, why};
    return v.inMode >= kResend ? MixedLightStep{want, "re-send"} : MixedLightStep{};
  }
  return {1 - v.mode, "no usable light"};
}

bool Lh1KeyframesUsable(const Lh1KeyframeView& v) {
  return v.sinceLh2Light >= kLh2Lit || (v.sorting && v.knownRotors >= std::max(1, v.lh2Stations));
}

std::vector<bool> ForeignSweeps(const std::vector<SweepSample>& samples) {
  std::vector<bool> drop(samples.size(), false);
  auto median = [](std::vector<double> v) {
    std::nth_element(v.begin(), v.begin() + std::ptrdiff_t(v.size() / 2), v.end());
    return v[v.size() / 2];
  };
  size_t first = 0;
  while (first < samples.size()) {
    size_t end = first + 1;
    while (end < samples.size() && samples[end].time - samples[end - 1].time < kSweepGap) ++end;
    std::vector<double> residuals;
    for (size_t i = first; i < end; ++i)
      if (samples[i].judged) residuals.push_back(samples[i].residual);
    if (int(residuals.size()) >= kSweepMinSamples) {
      const double centre = median(residuals);
      std::vector<double> off;
      for (const double r : residuals) off.push_back(std::fabs(r - centre));
      if (median(off) > kSweepSpread)
        for (size_t i = first; i < end; ++i) drop[i] = true;
    }
    first = end;
  }
  return drop;
}

}  // namespace lh::runtime
