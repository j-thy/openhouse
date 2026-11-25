#include <gtest/gtest.h>

#include <cstdlib>

#include "base_channels.hpp"
#include "platform/platform.hpp"

extern "C" void lh_fake_ble_scenario(int value);
extern "C" int lh_fake_ble_writes();

namespace openhouse::overlay {
namespace {
viewer::Base Station(const char* serial, const char* channel, double observed = 98.0) {
  viewer::Base base;
  base.serial = serial;
  base.channel = channel;
  base.lh2 = base.seen = true;
  base.channelObservedAt = observed;
  return base;
}

#ifdef _WIN32
constexpr auto kDataHome = "LOCALAPPDATA";
#else
constexpr auto kDataHome = "XDG_DATA_HOME";
#endif

void SetDataHome(const std::filesystem::path& path) {
#ifdef _WIN32
  _wputenv_s(L"LOCALAPPDATA", path.c_str());
#else
  if (path.empty()) unsetenv(kDataHome);
  else setenv(kDataHome, path.c_str(), 1);
#endif
}

struct BleTestData {
  const std::filesystem::path previous = lh::platform::FsPath(lh::platform::Env(kDataHome));
  const std::filesystem::path directory = std::filesystem::temp_directory_path() /
      ("openhouse-ble-test-" + std::to_string(lh::platform::ProcessId()));

  BleTestData() { SetDataHome(directory); }
  ~BleTestData() {
    SetDataHome(previous);
    std::error_code error;
    std::filesystem::remove(directory / "alternate/openhouse/base-channels.lock", error);
    std::filesystem::remove(directory / "alternate/openhouse", error);
    std::filesystem::remove(directory / "alternate", error);
    std::filesystem::remove(directory / "openhouse/base-channels.lock", error);
    std::filesystem::remove(directory / "openhouse", error);
    std::filesystem::remove(directory, error);
  }
};
}  // namespace

TEST(BaseChannels, PlansDeterministicMovesAndReservesCachedChannels) {
  viewer::State state;
  state.time = 100;
  state.bases = {Station("B", "1"), Station("A", "1"), Station("D", "3"), Station("C", "3"),
                 Station("cached", "2", -1), Station("old", "3", 60)};
  const std::vector<BaseChannelChange> expected{{"B", 1, 4}, {"D", 3, 5}};
  EXPECT_EQ(ResolveBaseChannels(state), expected);
  EXPECT_TRUE(CanChangeBaseChannel(state, expected.front(), true));
  EXPECT_FALSE(CanChangeBaseChannel(state, expected.back(), true));
  EXPECT_FALSE(CanChangeBaseChannel(state, {"old", 3, 6}, false));
  for (int channel = 4; channel <= 16; ++channel)
    state.bases.push_back(Station("known", std::to_string(channel).c_str(), -1));
  EXPECT_TRUE(FreeBaseChannels(state.bases).empty());
  EXPECT_TRUE(ResolveBaseChannels(state).empty());
  EXPECT_FALSE(CanChangeBaseChannel(state, expected.front(), false));
}

TEST(BaseChannels, ConfirmsOnlyANewerDirectObservationOfTheExpectedTarget) {
  viewer::State state;
  state.time = 100;
  state.bases = {Station("B", "4", 97)};
  const BaseChannelChange change{"B", 1, 4};
  EXPECT_FALSE(BaseChannelObserved(state, change, 98));
  state.bases[0].channelObservedAt = 98;
  EXPECT_FALSE(BaseChannelObserved(state, change, 98));
  state.bases[0].channelObservedAt = 99;
  EXPECT_TRUE(BaseChannelObserved(state, change, 98));
  EXPECT_FALSE(BaseChannelObserved(state, {"C", 1, 4}, 98));
  EXPECT_FALSE(BaseChannelObserved(state, {"B", 1, 5}, 98));
}

TEST(BaseChannels, DiscoveryChecksAllAdaptersBeforeOneExplicitWrite) {
  const BleTestData testData;
#ifdef _WIN32
  _putenv_s("OPENHOUSE_LIGHTHOUSE_BLE", LH_TEST_BLE);
#else
  setenv("OPENHOUSE_LIGHTHOUSE_BLE", LH_TEST_BLE, 1);
#endif
  const BaseChannelChange change{"LHB-ABCDEF01", 1, 4};
  lh_fake_ble_scenario(0);
  auto prepared = DiscoverBaseChannel(change);
  ASSERT_TRUE(prepared.write) << prepared.error;
  EXPECT_EQ(lh_fake_ble_writes(), 0);
  EXPECT_TRUE(prepared.write().empty());
  EXPECT_EQ(lh_fake_ble_writes(), 1);

  auto confirmationLock = std::move(prepared.lock);
  prepared = {};
  SetDataHome(testData.directory / "alternate");
  auto competing = DiscoverBaseChannel(change);
  EXPECT_FALSE(competing.write);
  EXPECT_EQ(competing.error, "Another overlay is changing a base station channel");
  EXPECT_EQ(lh_fake_ble_writes(), 1);
  competing = {};
  confirmationLock.reset();

  lh_fake_ble_scenario(1);
  const auto duplicate = DiscoverBaseChannel(change);
  EXPECT_FALSE(duplicate.write);
  EXPECT_NE(duplicate.error.find("Multiple"), std::string::npos);
  EXPECT_EQ(lh_fake_ble_writes(), 0);

  lh_fake_ble_scenario(2);
  const auto failed = DiscoverBaseChannel(change);
  EXPECT_FALSE(failed.write);
  EXPECT_EQ(failed.error, "simulated scan failure");
  EXPECT_EQ(lh_fake_ble_writes(), 0);

  lh_fake_ble_scenario(3);
  const auto missing = DiscoverBaseChannel(change);
  EXPECT_FALSE(missing.write);
  EXPECT_NE(missing.error.find("not found"), std::string::npos);
  EXPECT_EQ(lh_fake_ble_writes(), 0);

  lh_fake_ble_scenario(4);
  auto rejected = DiscoverBaseChannel(change);
  ASSERT_TRUE(rejected.write) << rejected.error;
  EXPECT_EQ(rejected.write(), "simulated BLE write failure");
  EXPECT_EQ(lh_fake_ble_writes(), 1);
}

}  // namespace openhouse::overlay
