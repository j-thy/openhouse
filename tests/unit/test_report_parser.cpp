#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "report_parser.hpp"
#include "usb_transport.hpp"

namespace {

struct Recorder : lh::InputSinkBase {
  std::vector<lh::ImuSample> imu;
  std::vector<lh::LightPulse> pulses;
  std::vector<lh::BatteryReport> batteries;
  int attached = 0, detached = 0;
  void OnDeviceAttached(const lh::DeviceId&, const std::string&) override { ++attached; }
  void OnDeviceDetached(const lh::DeviceId&) override { ++detached; }
  void OnBattery(const lh::DeviceId&, const lh::BatteryReport& b) override { batteries.push_back(b); }
  void OnImu(const lh::DeviceId&, const lh::ImuSample& s) override { imu.push_back(s); }
  void OnLightPulse(const lh::DeviceId&, const lh::LightPulse& p) override { pulses.push_back(p); }
};

template <typename T>
void Put(uint8_t* p, T v) { std::memcpy(p, &v, sizeof v); }

TEST(TickUnwrapper, Wraps24Bit) {
  lh::usb::TickUnwrapper u;
  EXPECT_EQ(u.Unwrap(0xfffff0, 32), 0xfffff0);
  EXPECT_EQ(u.Unwrap(0x000010, 24), 0x1000010);  // wrapped forward
  EXPECT_EQ(u.Unwrap(0xffffe0, 24), 0xffffe0);   // slightly older, no wrap
}

TEST(ImuParser, ScalesAndOrders) {
  auto dev = std::make_shared<lh::usb::DeviceState>();
  dev->id = "X";
  dev->gyroRange = 0;
  dev->accelRange = 0;
  Recorder rec;
  lh::usb::ImuReportParser p(dev, &rec);
  uint8_t r[52] = {0x20};
  const uint32_t ts[3] = {300, 100, 200};  // ring order: oldest is index 1
  for (int i = 0; i < 3; i++) {
    uint8_t* s = r + 1 + 17 * i;
    Put<int16_t>(s + 0, 16384);   // ax = 1 g
    Put<int16_t>(s + 6, 32767);   // gx saturated
    Put<uint32_t>(s + 12, ts[i]);
  }
  ASSERT_TRUE(p.Parse(1.0, r, sizeof r, false));
  ASSERT_EQ(rec.imu.size(), 3u);
  EXPECT_NEAR(rec.imu[0].time, 100 / 48e6, 1e-15);
  EXPECT_NEAR(rec.imu[2].time, 300 / 48e6, 1e-15);
  EXPECT_NEAR(rec.imu[0].accel[0], 9.80665, 1e-9);
  EXPECT_EQ(rec.imu[0].saturated, 1u << 3);
  // Same report again: nothing new.
  ASSERT_TRUE(p.Parse(1.0, r, sizeof r, false));
  EXPECT_EQ(rec.imu.size(), 3u);
}

TEST(OpticalParser, Report25) {
  auto dev = std::make_shared<lh::usb::DeviceState>();
  Recorder rec;
  lh::usb::OpticalReportParser p(dev, &rec);
  uint8_t r[64] = {0x25};
  for (int i = 0; i < 9; i++) Put<uint16_t>(r + 1 + 7 * i + 1, 0xffff);
  r[1] = 5;
  Put<uint16_t>(r + 2, 480);       // 10 us
  Put<uint32_t>(r + 4, 48000);     // 1 ms
  ASSERT_TRUE(p.Parse(0, r, sizeof r));
  ASSERT_EQ(rec.pulses.size(), 1u);
  EXPECT_EQ(rec.pulses[0].sensor, 5);
  EXPECT_NEAR(rec.pulses[0].width, 1e-5, 1e-15);
  EXPECT_NEAR(rec.pulses[0].time, 1e-3, 1e-15);
}

TEST(WirelessReconnect, ResetsProtocolClockAndImuEpoch) {
  Recorder rec;
  lh::usb::ReportDecoder decoder(&rec);
  lh::usb::EndpointInfo endpoint;
  endpoint.ep = 1; endpoint.kind = lh::usb::EndpointKind::Wireless;
  endpoint.serial = "LHR-TEST"; endpoint.gyroRange = endpoint.accelRange = 0;
  decoder.AddEndpoint(endpoint);
  auto imu = [&](uint8_t msb, double host) {
    uint8_t r[30] = {0x23, msb, 15, 0, 0xe8, 1};
    Put<int16_t>(r + 6, 16384);
    decoder.OnReport(host, 1, true, r, sizeof r);
  };
  uint8_t malformed[30] = {0x23, 100, 255};
  decoder.OnReport(0.9, 1, true, malformed, sizeof malformed);
  EXPECT_TRUE(rec.imu.empty()); EXPECT_TRUE(rec.batteries.empty());
  imu(100, 1.0);
  ASSERT_EQ(rec.imu.size(), 1u);
  const double oldTime = rec.imu.back().time;
  decoder.SetVersionNegotiation(1);
  uint8_t transition[30] = {0x23, 0, 3, 0, 0xe2, 1};
  decoder.OnReport(1.1, 1, true, transition, sizeof transition);
  ASSERT_EQ(decoder.Wireless(1)->ProtocolVersion(), 1);
  const uint8_t off[] = {0x26};
  decoder.OnReport(1.2, 1, true, off, sizeof off);
  EXPECT_EQ(rec.detached, 1);
  EXPECT_EQ(decoder.WirelessGeneration(1), 1u);
  EXPECT_EQ(decoder.Wireless(1)->ProtocolVersion(), 2);  // wait, don't misdecode v0 as Wv2
  uint8_t battery[30] = {0x23, 1, 3, 0, 0xe1, 77};
  decoder.OnReport(1.3, 1, true, battery, sizeof battery);
  EXPECT_TRUE(rec.batteries.empty());
  transition[5] = 0;
  decoder.OnReport(1.4, 1, true, transition, sizeof transition);
  battery[5] = 42;
  decoder.OnReport(1.5, 1, true, battery, sizeof battery);
  ASSERT_EQ(rec.batteries.size(), 1u);
  EXPECT_EQ(rec.batteries[0].raw, 42);
  imu(1, 1.6);
  ASSERT_EQ(rec.imu.size(), 2u);
  EXPECT_LT(rec.imu.back().time, oldTime);  // rebooted timestamp must not be dropped as old
  EXPECT_EQ(rec.attached, 2);
  // Missing 0x26: a long radio-data gap must also create a fresh epoch.
  imu(2, 4.0);
  EXPECT_EQ(rec.detached, 2);
  EXPECT_EQ(decoder.WirelessGeneration(1), 2u);
  EXPECT_EQ(decoder.Wireless(1)->ProtocolVersion(), 2);
}

}  // namespace

// A section announced longer than the packet made Decode memcpy a negative length.
TEST(Wv2Decoder, RejectsSectionsLongerThanThePacket) {
  // meta + light, the meta section announced as 16 bytes in a 6-byte packet.
  const uint8_t metaLight[6] = {0x00, 0x40 | 0x10 | 0x0f, 0x00, 0x00, 0x00, 0x00};
  lh::usb::Wv2Decoder lh1Probe;
  lh1Probe.opticalSub = 0;
  lh1Probe.controllerSub = 0;
  lh::usb::WirelessPacket out;
  EXPECT_FALSE(lh1Probe.Decode(0, metaLight, 6, &out));
  // input + light, the input announced as 16 bytes.
  const uint8_t inputLight[5] = {0x00, 0x20 | 0x10 | 0x0f, 0x00, 0x00, 0x00};
  lh::usb::Wv2Decoder input;
  input.opticalSub = 0;
  input.controllerSub = 1;
  EXPECT_FALSE(input.Decode(0, inputLight, 5, &out));
}

// Every flag byte at every length a radio report can carry, from both probe states: nothing
// may read or write outside the packet (run under AddressSanitizer to see it).
TEST(Wv2Decoder, AnyShortPacketDecodesWithinItsBytes) {
  for (int optical : {0, 1}) {
    for (int controller : {0, 1, 2}) {
      for (int flags = 0; flags < 256; ++flags) {
        for (int n = 1; n <= 27; ++n) {
          std::vector<uint8_t> packet(size_t(n), 0x5a);
          if (n > 1) packet[1] = uint8_t(flags);
          lh::usb::Wv2Decoder d;
          d.opticalSub = optical;
          d.controllerSub = controller;
          lh::usb::WirelessPacket out;
          d.Decode(0, packet.data(), n, &out);
        }
      }
    }
  }
  SUCCEED();
}

namespace {
struct InputRecorder : Recorder {
  std::vector<lh::InputReport> inputs;
  void OnInput(const lh::DeviceId&, const lh::InputReport& r) override { inputs.push_back(r); }
};
}  // namespace

TEST(ButtonsParser, RepacksWiredBitsIntoTheRadioLayout) {
  auto dev = std::make_shared<lh::usb::DeviceState>();
  dev->id = "LHR-T";
  InputRecorder rec;
  lh::usb::ButtonsReportParser p(dev, &rec);
  uint8_t r[64] = {0x01, 0x00, 0x01};  // report 1, type 0x0100: user input
  // power/system (13), grip (2), menu (12), thumb = trackpad click (18), trigger (0)
  Put<uint32_t>(r + 8, (1u << 13) | (1u << 2) | (1u << 12) | (1u << 18) | (1u << 0));
  Put<uint16_t>(r + 0x1a, 0x4000);
  ASSERT_TRUE(p.Parse(0, r, sizeof r));
  ASSERT_EQ(rec.inputs.size(), 1u);
  const auto& in = rec.inputs[0];
  EXPECT_EQ(in.buttons, 0x01 | 0x04 | 0x08 | 0x10 | 0x20);  // trigger, pad click, system, grip, menu
  EXPECT_EQ(in.trigger, 255);  // clicked: full
  Put<uint32_t>(r + 8, 0);
  ASSERT_TRUE(p.Parse(0, r, sizeof r));
  EXPECT_EQ(rec.inputs[1].buttons, 0);
  EXPECT_EQ(rec.inputs[1].trigger, 0x4000 >> 7);
}

TEST(ButtonsParser, ReadsBatteryAndIgnoresOtherReports) {
  auto dev = std::make_shared<lh::usb::DeviceState>();
  dev->id = "LHR-T";
  InputRecorder rec;
  lh::usb::ButtonsReportParser p(dev, &rec);
  uint8_t battery[64] = {0x01, 0x00, 0x04};
  battery[14] = 0x80 | 57;
  EXPECT_TRUE(p.Parse(0, battery, sizeof battery));
  ASSERT_EQ(rec.batteries.size(), 1u);
  EXPECT_EQ(rec.batteries[0].raw, 0x80 | 57);
  uint8_t other[64] = {0x02, 0x00, 0x01};
  EXPECT_FALSE(p.Parse(0, other, sizeof other));
  uint8_t shortInput[8] = {0x01, 0x00, 0x01};
  p.Parse(0, shortInput, sizeof shortInput);
  EXPECT_TRUE(rec.inputs.empty());
}

// A report read from an Index controller on USB (LHR-13DA81D1), thumbstick touched and pushed.
TEST(ButtonsParser, ReadsTheIndexControllersStateReport) {
  auto dev = std::make_shared<lh::usb::DeviceState>();
  dev->id = "LHR-I";
  InputRecorder rec;
  lh::usb::ButtonsReportParser p(dev, &rec);
  uint8_t r[64] = {0x01, 0x00, 0x07, 0x3c, 0x24};
  Put<uint32_t>(r + 8, (1u << 2) | (1u << 18));  // A, thumbstick click
  r[12] = 200;                                    // trigger
  Put<int16_t>(r + 16, 12000);
  Put<int16_t>(r + 18, -9000);
  const uint8_t a1[8] = {0xa1, 0x40 | 0x10, 10, 20, 30, 40, 50, 60};
  std::memcpy(r + 20, a1, 8);
  ASSERT_TRUE(p.Parse(0, r, sizeof r));
  ASSERT_EQ(rec.inputs.size(), 1u);
  const auto& in = rec.inputs[0];
  EXPECT_EQ(in.buttons, 0x04 | 0x10);  // stick click, A in the radio's layout
  EXPECT_EQ(in.trigger, 200);
  EXPECT_EQ(in.axisX, 12000);
  EXPECT_EQ(in.axisY, -9000);
  ASSERT_TRUE(in.hasExtended);
  EXPECT_EQ(in.extended[0], 0xa1);
  EXPECT_EQ(in.extended[1], 0x50);
  EXPECT_EQ(in.extended[7], 60);
  // A wand's or tracker's 0x0700 (a counter, no record) is not input.
  uint8_t counter[64] = {0x01, 0x00, 0x07, 0x3c, 0x25};
  EXPECT_TRUE(p.Parse(0, counter, sizeof counter));
  EXPECT_EQ(rec.inputs.size(), 1u);
}
