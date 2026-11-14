#pragma once
// SteamVR's watchdog: starts SteamVR when a device is turned on (watchdog.cpp).
#include <openvr_driver.h>

#include <nlohmann/json.hpp>

#include <string>

namespace lh::openvr {
vr::IVRWatchdogProvider* WatchdogProvider();

// A wired device just plugged in: whether that's it being turned on, from SteamVR's cached config for its
// serial ("" when SteamVR hasn't seen it). A controller or tracker on a USB cable shows up whether or not
// it's on, since that's how it charges (#91), so it doesn't start SteamVR: stock's watchdog starts it on
// a new wireless connection, a system button or the headset's button, never on a plug-in. A headset has
// no other way (the Bigscreen Beyond has no power button), and a device SteamVR doesn't know might be one.
inline bool WiredPlugInWakes(const std::string& cachedConfig) {
  const auto j = nlohmann::json::parse(cachedConfig, nullptr, false);
  if (!j.is_object()) return true;
  const auto c = j.find("device_class");
  if (c == j.end() || !c->is_string()) return true;
  const std::string deviceClass = c->get<std::string>();
  return deviceClass != "controller" && deviceClass != "generic_tracker";
}
}  // namespace lh::openvr
