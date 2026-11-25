#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tracking/disambiguator_sob.hpp"

namespace {

constexpr double kPi = 3.141592653589793;

// One base on channel 1, known up front so its frames flow without OOTX. Revolution k
// starts at t0 + k * period; each one lights sensor 2k early in the turn (plane 0) and
// sensor 2k + 1 late (plane 1). The beam words of the revolutions in `missed` never arrive.
std::vector<lh::OpticalFrame> Decode(const std::vector<int>& missed) {
  lh::sob::SobDisambiguator d;
  lh::BaseConfig cfg;
  cfg.valid = true;
  cfg.serial = 0x12345678;
  d.SetKnownChannels({{0, {cfg, lh::BaseDynamicState{}}}});
  const double period = double(lh::sob::kPeriodTicks[0]) * lh::sob::kTickSeconds;
  const double t0 = 1.0;
  for (int k = 0; k < 5; ++k) {
    const double sync = t0 + k * period;
    if (std::find(missed.begin(), missed.end(), k) == missed.end()) d.OnBeamWord(sync, 0, 0, 0);
    d.OnSensorWord(sync + 0.2 * period, 0, 2 * k);
    d.OnSensorWord(sync + 0.7 * period, 0, 2 * k + 1);
  }
  d.Advance(t0 + 10 * period);
  std::vector<lh::OpticalFrame> frames;
  lh::OpticalFrame f;
  while (d.PopFrame(&f)) frames.push_back(f);
  return frames;
}

std::vector<int> Sensors(const std::vector<lh::OpticalFrame>& frames, double sync, int axis) {
  std::vector<int> out;
  for (const auto& f : frames)
    if (f.axis == axis && std::abs(f.syncTime - sync) < 1e-9)
      for (const auto& s : f.samples) out.push_back(s.sensor);
  return out;
}

}  // namespace

TEST(SobDisambiguator, ConflictedChannelDiscardsQueuedHitsAndCanResume) {
  lh::sob::SobDisambiguator decoder;
  lh::BaseConfig first, second;
  first.serial = 1;
  second.serial = 2;
  first.valid = second.valid = true;
  const std::map<int, std::pair<lh::BaseConfig, lh::BaseDynamicState>> channels{
      {0, {first, {}}}, {1, {second, {}}}};
  const auto sweep = [&](double start, int channel) {
    const double period = lh::sob::kPeriodTicks[size_t(channel)] * lh::sob::kTickSeconds;
    decoder.OnBeamWord(start, channel, 0, 0);
    decoder.OnSensorWord(start + 0.2 * period, channel, 0);
    decoder.OnBeamWord(start + period, channel, 0, 0);
  };

  decoder.SetKnownChannels(channels);
  sweep(1.0, 0);
  sweep(1.0, 1);
  lh::BaseConfig announced;
  lh::BaseDynamicState dynamic;
  ASSERT_TRUE(decoder.PopBaseConfig(&announced, &dynamic));
  EXPECT_TRUE(dynamic.cached);
  decoder.SetKnownChannels({{1, channels.at(1)}}, std::bitset<lh::sob::kChannels>{1});
  EXPECT_EQ(decoder.ChannelSerial(0), 0u);
  EXPECT_EQ(decoder.ChannelSerial(1), second.serial);
  sweep(1.1, 0);
  decoder.Advance(1.2);
  lh::OpticalFrame frame;
  ASSERT_TRUE(decoder.PopFrame(&frame));
  EXPECT_EQ(frame.baseSerial, second.serial);
  EXPECT_FALSE(decoder.PopFrame(&frame));

  decoder.SetKnownChannels(channels);
  sweep(1.3, 0);
  decoder.Advance(1.4);
  ASSERT_TRUE(decoder.PopFrame(&frame));
  EXPECT_EQ(frame.baseSerial, first.serial);
  EXPECT_FALSE(decoder.PopFrame(&frame));
}

TEST(SobDisambiguator, EveryRevolutionWithItsBeamWordKeepsItsOwnHits) {
  const auto frames = Decode({});
  const double period = double(lh::sob::kPeriodTicks[0]) * lh::sob::kTickSeconds;
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(Sensors(frames, 1.0 + k * period, 0), std::vector<int>{2 * k}) << "revolution " << k;
    EXPECT_EQ(Sensors(frames, 1.0 + k * period, 1), std::vector<int>{2 * k + 1}) << "revolution " << k;
  }
}

TEST(SobDisambiguator, MissedBeamWordDropsTheUnseenRevolutionsHits) {
  // Revolution 2's word goes missing: revolution 1 closes at its predicted end when
  // revolution 3's word arrives, and revolution 2's hits must not join revolution 3.
  const auto frames = Decode({2});
  const double period = double(lh::sob::kPeriodTicks[0]) * lh::sob::kTickSeconds;
  const double sync3 = 1.0 + 3 * period;
  EXPECT_EQ(Sensors(frames, 1.0 + period, 0), std::vector<int>{2});
  EXPECT_EQ(Sensors(frames, 1.0 + period, 1), std::vector<int>{3});
  EXPECT_EQ(Sensors(frames, sync3, 0), std::vector<int>{6});
  EXPECT_EQ(Sensors(frames, sync3, 1), std::vector<int>{7});
  for (const auto& f : frames)
    for (const auto& s : f.samples) {
      EXPECT_GE(s.angle, 0.0) << "sensor " << s.sensor << " in the frame at " << f.syncTime;
      EXPECT_LT(s.angle, 2 * kPi) << "sensor " << s.sensor << " in the frame at " << f.syncTime;
    }
}
