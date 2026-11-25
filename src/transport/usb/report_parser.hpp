// HID report decoding for wired Lighthouse devices (Index HMD, wired trackers).
//
// Pure functions of (bytes, per-device state) -> IInputSink calls. No hidapi here,
// so the same code decodes live reports and raw captures.
// Reference: ai/driver/usb_protocol.md.
#pragma once

#include <cstddef>
#include <climits>
#include <cstdint>
#include <memory>

#include "clock_sync.hpp"
#include "lh/input_sink.hpp"
#include "tick_unwrapper.hpp"
#include "wireless_decoder.hpp"

namespace lh::usb {

// State shared by all endpoints of one physical device.
struct DeviceState {
  DeviceId id;
  TickUnwrapper ticks;
  ClockSync clock;
  int gyroRange = -1;   // feature report 0x01 byte 1; -1 = unknown (IMU dropped)
  int accelRange = -1;  // feature report 0x01 byte 2
};

// Per-endpoint IMU sample emission (0x3d07c0): scaling, saturation flags and
// "only newer than the last emitted sample".
class ImuEmitter {
 public:
  ImuEmitter(std::shared_ptr<DeviceState> dev, IInputSink* sink) : dev_(std::move(dev)), sink_(sink) {}
  bool IsNew(int64_t ticks) const { return ticks > lastTicks_; }
  void Emit(const uint8_t* raw12, int64_t ticks);  // raw12: int16 accel[3], gyro[3]
  int64_t LastTicks() const { return lastTicks_; }

 private:
  std::shared_ptr<DeviceState> dev_;
  IInputSink* sink_;
  int64_t lastTicks_ = INT64_MIN;  // endpoint+0x50 (times may be negative right after start)
};

// CLighthouseIMUThread::OnReport (0x3d1a60), wired report 0x20.
class ImuReportParser {
 public:
  ImuReportParser(std::shared_ptr<DeviceState> dev, IInputSink* sink) : dev_(dev), sink_(sink), imu_(dev, sink) {}
  // Returns false if the report id is not handled.
  bool Parse(double hostTime, const uint8_t* data, size_t len, bool lastInBatch);

 private:
  std::shared_ptr<DeviceState> dev_;
  IInputSink* sink_;
  ImuEmitter imu_;
};

// A wired device's buttons interface (iface 2), report 0x01, as libsurvive reads it
// (driver_vive.c, USB_IF_TRACKER*_BUTTONS; offsets here count the report id): u16 type at
// [1]; type 0x0100 = user input: u32 buttons at [8], int16 trackpad x/y at [0x14]/[0x16],
// u16 trigger at [0x1a]; Index controllers: type 0x0700 with the 0xa1 record (see Parse);
// type 0x0400: battery byte at [14]. The wired button bits (trigger 0,
// grip 2, menu 12, system 13, trackpad click 18, touch 20; libsurvive WiredWandMapping) are
// re-packed into the radio's button byte, so the same decoders read both.
class ButtonsReportParser {
 public:
  ButtonsReportParser(std::shared_ptr<DeviceState> dev, IInputSink* sink) : dev_(std::move(dev)), sink_(sink) {}
  bool Parse(double hostTime, const uint8_t* data, size_t len);

 private:
  std::shared_ptr<DeviceState> dev_;
  IInputSink* sink_;
};

// Watchman dongle reports 0x23 (one 29-byte packet) / 0x24 (two) — 0x3d1920 / 0x3d18d0.
class WirelessReportParser {
 public:
  WirelessReportParser(std::shared_ptr<DeviceState> dev, IInputSink* sink) : dev_(dev), sink_(sink), imu_(dev, sink) {}
  bool Parse(double hostTime, const uint8_t* data, size_t len, bool lastInBatch);
  int ProtocolVersion() const { return version_; }
  // After sending the "request protocol version" feature report (0x3cc9a0) the
  // endpoint waits (version 2) for the device's transition packet.
  void SetPendingVersionRequest() { version_ = 2; }

 private:
  bool DecodePacket(const uint8_t* pkt, int64_t* imuTicks);
  std::shared_ptr<DeviceState> dev_;
  IInputSink* sink_;
  ImuEmitter imu_;
  int version_ = 0;  // +0x128: 0 legacy, 1 Wv2
  Wv2Decoder wv2_;
  // Not stock: a recording started mid-session (Record Session) has no transition
  // packet, so every Wv2 packet went to the legacy decoder (no IMU, a few hundred
  // pulses in 100 s). While on legacy, packets are also tried as Wv2 on two probes (LH2
  // and LH1 light); legacy failing where one decodes, again and again, is a Wv2 device.
  Wv2Decoder probe_[2] = {MakeProbe(1), MakeProbe(0)};
  int probeDecoded_[2] = {0, 0};
  static Wv2Decoder MakeProbe(int opticalSub) {
    Wv2Decoder d;
    d.opticalSub = opticalSub;
    d.controllerSub = 0;
    return d;
  }
  WirelessPacket pkt_;
};

// CLighthouseOpticalThread::OnReport (0x3cf250).
class OpticalReportParser {
 public:
  OpticalReportParser(std::shared_ptr<DeviceState> dev, IInputSink* sink) : dev_(std::move(dev)), sink_(sink) {}
  // The stock wired index-HMD optical reader uses absolute (unshifted) widths
  // (0x3cec20 compares 23/48k..25/48k directly). Wireless devices go through the
  // TDM path, so their sensor-type width offset must not be applied here either.
  bool Parse(double hostTime, const uint8_t* data, size_t len);

 private:
  void OnRawPulse(double t, double width, int sensor);  // 0x3cec20
  void ParseLh2Stream(const uint8_t* data, size_t len); // 0x3cf040
  std::shared_ptr<DeviceState> dev_;
  IInputSink* sink_;
  double syncEventHoldoff_ = 0;  // endpoint+0x58
};

}  // namespace lh::usb
