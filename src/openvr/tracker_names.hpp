#pragma once

#include <optional>
#include <vector>

#include "common/tracker_names.hpp"
#include "lh/session.hpp"

namespace lh::openvr {

inline std::optional<std::string> ValidateTrackerRename(const std::vector<DeviceState>& devices,
                                                        const std::string& serial, const nlohmann::json& value,
                                                        std::string* error) {
  const auto tracker = std::find_if(devices.begin(), devices.end(), [&](const DeviceState& device) {
    return device.info.serial == serial && device.info.role == DeviceRole::Tracker;
  });
  if (tracker == devices.end()) {
    *error = "unknown tracker " + serial;
    return std::nullopt;
  }
  if (!value.is_string()) {
    *error = "name must be a string";
    return std::nullopt;
  }
  const auto name = telemetry::TrimTrackerName(value.get_ref<const std::string&>());
  if (!telemetry::IsTrackerName(name)) {
    *error = "enter up to " + std::to_string(telemetry::kTrackerNameMaxBytes) + " UTF-8 bytes without control characters";
    return std::nullopt;
  }
  return std::string(name);
}

// Returns whether any settings changed, so callers can skip a redundant write.
inline bool SetTrackerName(nlohmann::json& settings, const std::string& serial, const std::string& name) {
  const auto names = settings.find("trackerNames");
  if (name.empty())
    return names != settings.end() && names->is_object() && names->erase(serial) != 0;

  auto& entries = settings["trackerNames"];
  if (!entries.is_object()) entries = nlohmann::json::object();
  auto& entry = entries[serial];
  if (entry == name) return false;
  entry = name;
  return true;
}

}  // namespace lh::openvr
