#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace lh::usb {
// Stock Watchman 0x3cb9a0: ff 8f 08 channel, on:u16, off:u16,
// repeats:u16, ff. The older libsurvive 7-byte payload is not this command.
inline std::array<uint8_t, 64> HapticReport(double frequency, double amplitude, double duration) {
  // Port of stock 0x3f4e50 (SteamVR haptic event -> 0x8f on/off/repeats), in
  // float like the original. Verified against a stock capture: SteamVR Identify
  // (duration 0, 200 Hz, amplitude 0.125) -> on 500 us, off 500 us, 1 repeat,
  // which the controller firmware also answers with an LED flash.
  std::array<uint8_t, 64> out{0xff, 0x8f, 8, 0, 0, 0, 0, 0, 0, 0, 0xff};
  const float f0 = float(frequency), a = float(amplitude);
  float d = float(duration);
  if (!std::isfinite(f0) || !std::isfinite(a) || !std::isfinite(d)) return out;
  int on = 0, off = 0, repeats = 0;
  if (a >= 0.f) {
    const float amp = std::min(a, 1.f);
    if (f0 > 0.f && (a > 1.f || a > 0.f)) {
      if (d <= 0.f) {
        repeats = 1;
        on = off = int(amp * 4000.f) & 0xffff;
      } else {
        constexpr float kMinOn = 7.9999998e-05f, kMaxDuration = 10.f;
        auto count = [](float v) { return std::clamp(int(v), 1, 0xffff); };
        float f = f0, period, span = 0.003919f, half = 0.f;
        bool shortPeriod = false;
        if (f < 15.259022f) {  // longest period the u16 microsecond fields hold
          period = 0.065535f;
          if (d < kMinOn) { repeats = 1; d = kMinOn; }
          else if (d <= kMaxDuration) repeats = count(15.259022f * d);
          else { repeats = 0x98; d = kMaxDuration; }
        } else if (f > 3333.3333f) {  // shortest supported period (300 us)
          period = 0.0003f;
          if (d < kMinOn) { repeats = 1; d = kMinOn; span = 7.0000009e-05f; }
          else if (d <= kMaxDuration) { f = 3333.3333f; half = 0.00015f; repeats = count(f * d); shortPeriod = true; }
          else { repeats = 0x8235; d = kMaxDuration; span = 7.0000009e-05f; }
        } else {
          period = 1.f / f;
          half = 0.5f * period;
          if (d < kMinOn) { repeats = count(f * kMinOn); d = kMinOn; }
          else { d = std::min(d, kMaxDuration); repeats = count(f * d); }
          shortPeriod = period <= 0.007998f;
        }
        if (shortPeriod) span = half - kMinOn;
        const float onSeconds = std::min(amp * span + kMinOn, d);
        on = std::clamp(int(onSeconds * 1e6f), 0, 0xffff);
        off = std::clamp(int((period - (amp * span + kMinOn)) * 1e6f), 0, 0xffff);
      }
    }
  }
  out[4] = uint8_t(on); out[5] = uint8_t(on >> 8);
  out[6] = uint8_t(off); out[7] = uint8_t(off >> 8);
  out[8] = uint8_t(repeats); out[9] = uint8_t(repeats >> 8);
  return out;
}

// Stock receiver power command: report ff, opcode 9f, 4-byte magic "off!".
// driver_lighthouse 0x3ce5a0/0x3ce6e0; full 64-byte HID feature report.
inline std::array<uint8_t, 64> PowerOffReport() {
  return {0xff, 0x9f, 4, 'o', 'f', 'f', '!'};
}
// A receiver's keepalive: report ff, opcode a1, no data. Stock driver_lighthouse (0x18004b5c0
// in the Windows DLL, "Triggered keepalive") and lighthouse_console send it to a dongle
// (28de:2101/2102) that has gone quiet.
inline std::array<uint8_t, 64> ReceiverKeepaliveReport() { return {0xff, 0xa1, 0x00}; }
// Optical sub-protocol (light report format) of a wireless device: 0 = LH1 pulses,
// 1 = LH2 words. Same 0x87 command as the protocol-version request, entry type 1
// instead of 0 ({u8 type, u16 value}; libsurvive vive_switch_mode).
inline std::array<uint8_t, 64> OpticalSubProtocolReport(int sub) {
  return {0xff, 0x87, 3, 1, uint8_t(sub), 0};
}

// The same for a wired device (USB cable: Index HMD 28de:2000, controllers and trackers
// 28de:2300): feature 0x04 {mode} (0 = LH1, 1 = LH2), then 0x07 {0x03} (lightcap
// verbosity; LH1 pulses then come as report 0x25). From libsurvive (vive_switch_mode);
// verified on a wired Index controller: LH2 mode stops its LH1 reports, LH1 mode restores
// them. Any of its interfaces takes it.
inline std::array<uint8_t, 64> WiredModeReport(int sub) { return {0x04, uint8_t(sub)}; }
inline std::array<uint8_t, 64> WiredVerbosityReport() { return {0x07, 0x03}; }
}
