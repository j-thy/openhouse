#include <gtest/gtest.h>

#include "openvr/tracker_names.hpp"

namespace {
using lh::openvr::SetTrackerName;
using lh::openvr::ValidateTrackerRename;
using nlohmann::json;

std::vector<lh::DeviceState> Devices() {
  std::vector<lh::DeviceState> devices(2);
  devices[0].info.serial = "LHR-tracker";
  devices[0].info.role = lh::DeviceRole::Tracker;
  devices[1].info.serial = "LHR-controller";
  devices[1].info.role = lh::DeviceRole::Controller;
  return devices;
}

TEST(TrackerNames, TrimsSpacesAndAcceptsBlankNames) {
  std::string error;
  EXPECT_EQ(ValidateTrackerRename(Devices(), "LHR-tracker", " Waist ", &error), "Waist");
  EXPECT_EQ(ValidateTrackerRename(Devices(), "LHR-tracker", "   ", &error), "");
}

TEST(TrackerNames, ChecksTheByteLimitAndControlCharacters) {
  std::string error;
  const std::string name = std::string(61, 'a') + "\xD0\xAF";
  EXPECT_EQ(ValidateTrackerRename(Devices(), "LHR-tracker", name, &error), name);
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-tracker", name + "a", &error));
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-tracker", "Line\nBreak", &error));
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-tracker", "\x7f", &error));
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-tracker", nullptr, &error));
}

TEST(TrackerNames, RequiresAKnownTracker) {
  std::string error;
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-unknown", "Waist", &error));
  EXPECT_FALSE(ValidateTrackerRename(Devices(), "LHR-controller", "Waist", &error));
}

TEST(TrackerNames, ReadsAliasesAndAcceptsOlderTelemetry) {
  const json state = {{"settings", {{"trackerNames", {{"LHR-tracker", "腰 ##1"}, {"LHR-other", 42}}}}}};
  EXPECT_EQ(lh::telemetry::TrackerDisplayName(state, "LHR-tracker"), "腰 ##1");
  EXPECT_TRUE(lh::telemetry::TrackerDisplayName(state, "LHR-unknown").empty());
  EXPECT_TRUE(lh::telemetry::TrackerDisplayName(state, "LHR-other").empty());
  EXPECT_TRUE(lh::telemetry::TrackerDisplayName(json::object(), "LHR-tracker").empty());
  EXPECT_TRUE(lh::telemetry::TrackerDisplayName({{"settings", false}}, "LHR-tracker").empty());
}

TEST(TrackerNames, ClearingAnAbsentNameLeavesSettingsUntouched) {
  json settings = {{"hideLostDevices", true}};
  const json original = settings;
  EXPECT_FALSE(SetTrackerName(settings, "LHR-tracker", ""));
  EXPECT_EQ(settings, original);

  ASSERT_TRUE(SetTrackerName(settings, "LHR-tracker", "Waist"));
  EXPECT_EQ(settings["trackerNames"]["LHR-tracker"], "Waist");
  EXPECT_FALSE(SetTrackerName(settings, "LHR-tracker", "Waist"));
  EXPECT_TRUE(SetTrackerName(settings, "LHR-tracker", ""));
  EXPECT_FALSE(SetTrackerName(settings, "LHR-tracker", ""));
  EXPECT_EQ(settings["hideLostDevices"], true);
  EXPECT_FALSE(settings["trackerNames"].contains("LHR-tracker"));
}
}  // namespace
